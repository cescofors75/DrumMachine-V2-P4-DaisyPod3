#include "i2c_rotaries.h"

#include "../app_state.h"
#include "../control_api.h"
#include "../ui/ui_screens.h"
#include "../daisy_usb_transport.h"
#include "../master/protocol.h"
#include "display_init.h"
#include "../../include/config.h"
#include <Arduino.h>
#include <Wire.h>
#include <atomic>
#include <esp32-hal-rmt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace
{
constexpr uint8_t kRotaryCount = 4;
constexpr uint8_t kI2cDeviceCount = kRotaryCount;
constexpr uint8_t kPassiveAddresses[kRotaryCount] = {0x54, 0x55, 0x56, 0x57};
constexpr uint8_t kDefaultRotaryAddress = 0x54;
constexpr uint8_t kMuxFirstAddress = 0x70;
constexpr uint8_t kMuxLastAddress = 0x77;
constexpr uint8_t kPidRegister = 0x00;
constexpr uint8_t kCountRegister = 0x08;
constexpr uint8_t kKeyStatusRegister = 0x0A;
constexpr uint8_t kGainRegister = 0x0B;
constexpr uint16_t kExpectedPid = 0x01F6;
constexpr uint8_t kRotaryGain = 51;
constexpr uint16_t kRotaryInitialValue = 512;
constexpr uint32_t kRotaryI2cHz = 100000;
constexpr uint32_t kRotaryTaskPeriodMs = 1;
constexpr uint32_t kRotaryI2cTimeoutMs = 3;
constexpr uint32_t kAbsentRetryMs = 1000;
constexpr uint32_t kMuxRetryMs = 2000;
constexpr uint8_t kFailuresBeforeOffline = 3;
constexpr uint32_t kButtonReleaseQuietMs = 80;
constexpr uint32_t kButtonDebounceMs = 180;
constexpr uint8_t kFaderAdcGpio = 20;
constexpr uint16_t kFaderAdcMax = 4095;
constexpr uint32_t kFaderPollMs = 10;
constexpr uint8_t kFaderLedGpio = 45;
constexpr uint8_t kFaderLedCount = 14;
constexpr size_t kFaderLedSymbols = kFaderLedCount * 24u;
constexpr uint32_t kFaderLedRefreshMs = 1000;
constexpr uint32_t kFaderLedRetryMs = 2000;

std::atomic<uint16_t> s_values[kRotaryCount];
std::atomic<uint8_t> s_gains[kRotaryCount];
std::atomic<uint32_t> s_buttonPressCount{0};
std::atomic<uint32_t> s_buttonPressCounts[kRotaryCount];
std::atomic<uint8_t> s_presentMask{0};
std::atomic<uint8_t> s_addressAckMask{0};
std::atomic<uint8_t> s_changedMask{0};
std::atomic<uint8_t> s_functions[kRotaryCount];
std::atomic<uint64_t> s_ownedFunctionMask{0};
std::atomic<TaskHandle_t> s_applyingTask{nullptr};
std::atomic<uint8_t> s_faderFunction{POD_FUNC_SCREEN_BRIGHTNESS};
std::atomic<bool> s_faderReady{false};
std::atomic<uint16_t> s_faderRaw{0};
std::atomic<uint16_t> s_faderValue{0};
std::atomic<bool> s_faderChanged{false};

uint32_t s_lastProbeMs[kRotaryCount] = {};
uint8_t s_failures[kRotaryCount] = {};
bool s_haveValue[kRotaryCount] = {};
bool s_buttonLatched[kRotaryCount] = {};
uint32_t s_buttonLastHighMs[kRotaryCount] = {};
uint32_t s_buttonLastAcceptedMs[kRotaryCount] = {};
uint32_t s_processedButtonPressCounts[kRotaryCount] = {};
uint8_t s_nextI2cDevice = 0;
std::atomic<uint8_t> s_muxAddress{0};
uint8_t s_rotaryAddresses[kRotaryCount] = {};
uint32_t s_lastMuxProbeMs = 0;
uint32_t s_lastFaderPollMs = 0;
uint32_t s_faderFilteredQ3 = 0;
bool s_haveFaderValue = false;
std::atomic<bool> s_faderLedReady{false};
uint32_t s_lastFaderLedRefreshMs = 0;
uint32_t s_lastFaderLedRetryMs = 0;
rmt_data_t s_faderLedData[kFaderLedSymbols] = {};
TaskHandle_t s_rotaryTask = nullptr;

bool addressResponds(uint8_t address)
{
    Wire1.beginTransmission(address);
    return Wire1.endTransmission() == 0;
}

bool writeMuxControl(uint8_t address, uint8_t channels)
{
    Wire1.beginTransmission(address);
    Wire1.write(channels);
    return Wire1.endTransmission() == 0;
}

bool readMuxControl(uint8_t address, uint8_t& channels)
{
    const uint8_t received = Wire1.requestFrom(address, static_cast<uint8_t>(1));
    if(received != 1 || Wire1.available() < 1) return false;
    channels = static_cast<uint8_t>(Wire1.read());
    while(Wire1.available()) Wire1.read();
    return true;
}

bool hasMuxSignature(uint8_t address)
{
    // PCA9548A/TCA9548A has no chip-ID register. Prove its identity by writing
    // two distinct one-channel masks and reading the control byte back. A mere
    // endTransmission()==0 is not reliable on this ESP32-P4 shared bus.
    uint8_t readback = 0;
    bool valid = writeMuxControl(address, 0x01);
    delayMicroseconds(300);
    valid = valid && readMuxControl(address, readback) && readback == 0x01;
    if(valid)
    {
        valid = writeMuxControl(address, 0x02);
        delayMicroseconds(300);
        valid = valid && readMuxControl(address, readback) && readback == 0x02;
    }
    // Always leave every downstream channel disabled. The normal poll selects
    // the required channel immediately after a successful signature check.
    writeMuxControl(address, 0x00);
    return valid;
}

uint8_t findMux()
{
    // Scan the reserved mux range, but accept only a verified control-register
    // signature. This rejects false ACKs caused by a floating or crossed bus.
    for(uint8_t address = kMuxFirstAddress; address <= kMuxLastAddress; ++address)
        if(hasMuxSignature(address)) return address;
    return 0;
}

bool selectMuxChannel(uint8_t channel)
{
    if(s_muxAddress == 0) return true;
    const uint8_t address = s_muxAddress.load(std::memory_order_acquire);
    const uint8_t channels = static_cast<uint8_t>(1u << channel);
    if(writeMuxControl(address, channels))
    {
        delayMicroseconds(200);
        return true;
    }
    delayMicroseconds(100);
    const bool selected = writeMuxControl(address, channels);
    if(selected) delayMicroseconds(200);
    return selected;
}

uint8_t rotaryAddress(uint8_t index)
{
    return s_muxAddress != 0
        ? (s_rotaryAddresses[index] ? s_rotaryAddresses[index]
                                    : kDefaultRotaryAddress)
                             : kPassiveAddresses[index];
}

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value)
{
    Wire1.beginTransmission(address);
    Wire1.write(reg);
    Wire1.write(value);
    if(Wire1.endTransmission() == 0) return true;
    delayMicroseconds(100);
    Wire1.beginTransmission(address);
    Wire1.write(reg);
    Wire1.write(value);
    return Wire1.endTransmission() == 0;
}

bool writeBigEndian16(uint8_t address, uint8_t reg, uint16_t value)
{
    for(uint8_t attempt = 0; attempt < 2; ++attempt)
    {
        Wire1.beginTransmission(address);
        Wire1.write(reg);
        Wire1.write(static_cast<uint8_t>(value >> 8));
        Wire1.write(static_cast<uint8_t>(value & 0xFF));
        if(Wire1.endTransmission() == 0) return true;
        delayMicroseconds(100);
    }
    return false;
}

bool readRegistersOnce(uint8_t address, uint8_t reg, uint8_t* data,
                       uint8_t length)
{
    Wire1.beginTransmission(address);
    Wire1.write(reg);
    // SEN0502's official driver commits the register pointer with a STOP
    // before requestFrom(); repeated-start can leave the live count at zero.
    if(Wire1.endTransmission() != 0) return false;
    if(Wire1.requestFrom(address, length) != length) return false;
    for(uint8_t i = 0; i < length; ++i) data[i] = Wire1.read();
    return true;
}

bool readRegisters(uint8_t address, uint8_t reg, uint8_t* data, uint8_t length)
{
    if(readRegistersOnce(address, reg, data, length)) return true;
    delayMicroseconds(100);
    return readRegistersOnce(address, reg, data, length);
}

bool readBigEndian16(uint8_t address, uint8_t reg, uint16_t& value)
{
    uint8_t bytes[2] = {};
    if(!readRegisters(address, reg, bytes, sizeof(bytes))) return false;
    value = (static_cast<uint16_t>(bytes[0]) << 8) | bytes[1];
    return true;
}

bool probeRotary(uint8_t index, uint16_t& value)
{
    if(!selectMuxChannel(index)) return false;
    const uint8_t firstAddress = s_muxAddress != 0 ? 0x54 : rotaryAddress(index);
    const uint8_t lastAddress = s_muxAddress != 0 ? 0x57 : rotaryAddress(index);
    bool anyAddressAck = false;
    for(uint8_t address = firstAddress; address <= lastAddress; ++address)
    {
        if(!addressResponds(address)) continue;
        anyAddressAck = true;
        s_addressAckMask.fetch_or(static_cast<uint8_t>(1u << index),
                                  std::memory_order_release);
        uint16_t pid = 0;
        if(!readBigEndian16(address, kPidRegister, pid)
           || pid != kExpectedPid)
            continue;
        s_rotaryAddresses[index] = address;
        uint8_t gain = 0;
        uint8_t gainData[1] = {};
        if(readRegisters(address, kGainRegister, gainData, 1)) gain = gainData[0];
        if(gain != kRotaryGain)
        {
            if(!writeRegister(address, kGainRegister, kRotaryGain)) continue;
            delay(2);
            if(!readRegisters(address, kGainRegister, gainData, 1)) continue;
            gain = gainData[0];
        }
        s_gains[index].store(gain, std::memory_order_release);
        if(!readBigEndian16(address, kCountRegister, value) || value > 1023)
            continue;
        // SEN0502 is an endless encoder. Starting at zero makes every
        // anti-clockwise click appear dead because the counter is clamped.
        // Centre a fresh/reset module so both directions work immediately.
        if(value == 0)
        {
            if(!writeBigEndian16(address, kCountRegister, kRotaryInitialValue))
                continue;
            delay(2);
            if(!readBigEndian16(address, kCountRegister, value)
               || value > 1023)
                continue;
        }
        return true;
    }
    if(!anyAddressAck)
        s_addressAckMask.fetch_and(
            static_cast<uint8_t>(~(1u << index)), std::memory_order_release);
    return false;
}

bool readRotary(uint8_t index, uint16_t& value)
{
    const uint8_t address = rotaryAddress(index);
    if(!selectMuxChannel(index)) return false;
    // COUNT_MSB, COUNT_LSB and KEY_STATUS are consecutive registers. Reading
    // them together matches DFRobot's layout and adds no extra request cycle.
    uint8_t state[3] = {};
    if(!readRegisters(address, kCountRegister, state, sizeof(state))) return false;
    value = (static_cast<uint16_t>(state[0]) << 8) | state[1];
    if(value > 1023) return false;
    const uint32_t now = millis();
    if((state[2] & 0x01u) != 0)
    {
        // The SEN0502 status can be asserted more than once during one
        // mechanical press (or on both edges). Always acknowledge it, but
        // publish exactly one event until the input has stayed quiet.
        const bool acknowledged = writeRegister(address, kKeyStatusRegister, 0x00);
        if(acknowledged && !s_buttonLatched[index]
           && (s_buttonLastAcceptedMs[index] == 0
               || static_cast<uint32_t>(now - s_buttonLastAcceptedMs[index])
                    >= kButtonDebounceMs))
        {
            s_buttonLastAcceptedMs[index] = now;
            s_buttonPressCount.fetch_add(1, std::memory_order_release);
            s_buttonPressCounts[index].fetch_add(1, std::memory_order_release);
        }
        if(acknowledged)
        {
            s_buttonLatched[index] = true;
            s_buttonLastHighMs[index] = now;
        }
    }
    else if(s_buttonLatched[index]
            && static_cast<uint32_t>(now - s_buttonLastHighMs[index])
                   >= kButtonReleaseQuietMs)
        s_buttonLatched[index] = false;
    return true;
}

void faderLedByte(size_t& symbol, uint8_t value)
{
    for(uint8_t bit = 0; bit < 8; ++bit)
    {
        rmt_data_t& pulse = s_faderLedData[symbol++];
        const bool one = (value & (1u << (7u - bit))) != 0;
        pulse.level0 = 1;
        pulse.duration0 = one ? 8 : 4; // 0.8 / 0.4 us at 10 MHz.
        pulse.level1 = 0;
        pulse.duration1 = one ? 4 : 8;
    }
}

bool ensureFaderLedDriver(uint32_t now)
{
    if(s_faderLedReady.load(std::memory_order_acquire)) return true;
    if(s_lastFaderLedRetryMs != 0
       && static_cast<uint32_t>(now - s_lastFaderLedRetryMs)
            < kFaderLedRetryMs)
        return false;
    s_lastFaderLedRetryMs = now;
    pinMode(kFaderLedGpio, OUTPUT);
    digitalWrite(kFaderLedGpio, LOW);
    const bool ready = rmtInit(kFaderLedGpio, RMT_TX_MODE,
                               RMT_MEM_NUM_BLOCKS_1, 10000000);
    s_faderLedReady.store(ready, std::memory_order_release);
    P4_LOG_PRINTF("[Fader LED] RMT %s on GPIO%u\n",
                  ready ? "ready" : "retry pending", kFaderLedGpio);
    return ready;
}

void updateFaderLeds(uint16_t value, uint32_t now)
{
    if(!ensureFaderLedDriver(now)) return;
    // Keep the first position visible even at zero so power/data wiring can be
    // distinguished from an unpowered fader.
    const uint8_t lit = static_cast<uint8_t>(1u
        + (static_cast<uint32_t>(value) * (kFaderLedCount - 1u) + 511u)
          / 1023u);
    size_t symbol = 0;
    for(uint8_t pixel = 0; pixel < kFaderLedCount; ++pixel)
    {
        uint8_t red = 0, green = 0, blue = 0;
        if(pixel < lit)
        {
            const bool tip = pixel + 1u == lit;
            red = tip ? 64 : 32;
            green = tip ? 24 : 5;
            blue = tip ? 2 : 0;
        }
        faderLedByte(symbol, green);
        faderLedByte(symbol, red);
        faderLedByte(symbol, blue);
    }
    if(!rmtWrite(kFaderLedGpio, s_faderLedData, kFaderLedSymbols, 20))
        P4_LOG_PRINTLN("[Fader LED] RMT write timed out");
    s_lastFaderLedRefreshMs = now;
}

void pollFader(uint32_t now)
{
    if(s_lastFaderPollMs != 0
       && static_cast<uint32_t>(now - s_lastFaderPollMs) < kFaderPollMs)
        return;
    s_lastFaderPollMs = now;

    uint32_t sum = 0;
    for(uint8_t sample = 0; sample < 4; ++sample)
        sum += static_cast<uint16_t>(analogRead(kFaderAdcGpio));
    const uint16_t raw = static_cast<uint16_t>((sum + 2u) / 4u);
    s_faderRaw.store(raw, std::memory_order_release);
    const bool firstValue = !s_haveFaderValue;
    if(firstValue)
    {
        s_faderFilteredQ3 = static_cast<uint32_t>(raw) << 3;
        s_haveFaderValue = true;
    }
    else
    {
        // One-pole smoothing removes ADC noise while retaining quick fader feel.
        s_faderFilteredQ3 = (s_faderFilteredQ3 * 3u
                             + (static_cast<uint32_t>(raw) << 3)) / 4u;
    }
    s_faderReady.store(true, std::memory_order_release);
    const uint16_t filtered = static_cast<uint16_t>(s_faderFilteredQ3 >> 3);
    const uint16_t clamped = filtered > kFaderAdcMax ? kFaderAdcMax : filtered;
    const uint16_t normalized = static_cast<uint16_t>(
        (static_cast<uint32_t>(clamped) * 1023u + kFaderAdcMax / 2u)
        / kFaderAdcMax);
    const uint16_t previous = s_faderValue.load(std::memory_order_relaxed);
    const uint16_t delta = previous > normalized ? previous - normalized
                                                  : normalized - previous;
    const bool acceptedChange = previous != normalized
        && (delta >= 3u || firstValue || normalized == 0u
            || normalized == 1023u);
    if(acceptedChange)
    {
        s_faderValue.store(normalized, std::memory_order_release);
        s_faderChanged.store(true, std::memory_order_release);
    }
    const bool refreshDue = s_lastFaderLedRefreshMs == 0
        || static_cast<uint32_t>(now - s_lastFaderLedRefreshMs)
            >= kFaderLedRefreshMs;
    if(firstValue || acceptedChange || refreshDue)
        updateFaderLeds(acceptedChange ? normalized : previous, now);
}

uint8_t mapAbsolute(uint16_t raw, uint16_t low, uint16_t high)
{
    const uint32_t span = high - low;
    return static_cast<uint8_t>(low + (raw * span + 511u) / 1023u);
}

uint64_t functionOwnershipBits(uint8_t function)
{
    if(function == POD_FUNC_NONE || function >= 64u) return 0;
    uint64_t bits = 1ull << function;
    if(function == POD_FUNC_CRUSH_MACRO)
        bits |= (1ull << POD_FUNC_BIT_DEPTH) | (1ull << POD_FUNC_SAMPLE_RATE);
    return bits;
}

bool daisyKnobOwns(const PodConfigPayload& config, uint8_t function)
{
    const uint64_t requested = functionOwnershipBits(function);
    return (functionOwnershipBits(config.knob1Function) & requested) != 0
        || (functionOwnershipBits(config.knob2Function) & requested) != 0;
}

void applyPhysicalValue(uint8_t function, uint16_t raw)
{
    const float normalized = static_cast<float>(raw) / 1023.0f;
    s_applyingTask.store(xTaskGetCurrentTaskHandle(), std::memory_order_release);
    switch(function)
    {
        case POD_FUNC_MASTER_VOLUME:
            control_send_set_volume(mapAbsolute(raw, 0, 150));
            break;
        case POD_FUNC_SEQ_VOLUME:
            control_send_set_seq_volume(mapAbsolute(raw, 0, 150));
            break;
        case POD_FUNC_LIVE_VOLUME:
            control_send_set_live_volume(mapAbsolute(raw, 0, 150));
            break;
        case POD_FUNC_TEMPO:
            control_send_tempo(static_cast<float>(mapAbsolute(raw, 40, 240)));
            break;
        case POD_FUNC_DELAY_MIX: {
            const uint8_t value = mapAbsolute(raw, 0, 127);
            control_send_fx_enc(1, value, value == 0);
            break;
        }
        case POD_FUNC_REVERB_MIX: {
            const uint8_t value = mapAbsolute(raw, 0, 127);
            control_send_fx_enc(2, value, value == 0);
            break;
        }
        case POD_FUNC_FLANGER_DEPTH: {
            const uint8_t value = mapAbsolute(raw, 0, 127);
            control_send_fx_enc(0, value, value == 0);
            break;
        }
        case POD_FUNC_WAVEFOLDER_GAIN: {
            const uint8_t value = mapAbsolute(raw, 0, 127);
            control_send_fx_pot(0, value, value == 0);
            break;
        }
        case POD_FUNC_CRUSH_MACRO:
            control_send_set_crush_macro(mapAbsolute(raw, 0, 127));
            break;
        case POD_FUNC_PHASER_DEPTH: {
            const uint8_t value = mapAbsolute(raw, 0, 127);
            control_send_fx_pot(2, value, value == 0);
            break;
        }
        case POD_FUNC_FILTER_CUTOFF: {
            const int hz = static_cast<int>(20.0f
                * powf(1000.0f, normalized) + 0.5f);
            control_send_set_filter_cutoff(hz);
            break;
        }
        case POD_FUNC_FILTER_RESONANCE:
            control_send_set_filter_resonance(0.7f + normalized * 19.3f);
            break;
        case POD_FUNC_DISTORTION:
            control_send_set_distortion(normalized);
            break;
        case POD_FUNC_BIT_DEPTH:
            control_send_set_bitcrush(static_cast<int>(16.0f
                - normalized * 12.0f + 0.5f));
            break;
        case POD_FUNC_SAMPLE_RATE: {
            const int rate = raw < 4u ? 0 : static_cast<int>(42000.0f
                * powf(4000.0f / 42000.0f, normalized) + 0.5f);
            control_send_set_sample_rate(rate);
            break;
        }
        case POD_FUNC_FILTER_TYPE:
            control_send_set_filter(static_cast<int>(normalized * 14.0f + 0.5f));
            break;
        case POD_FUNC_SCREEN_BRIGHTNESS:
            display_set_brightness(mapAbsolute(raw, 0, 100));
            break;
        default:
            break;
    }
    s_applyingTask.store(nullptr, std::memory_order_release);
}

void togglePhysicalFx(uint8_t function, uint16_t raw)
{
    const float normalized = static_cast<float>(raw) / 1023.0f;
    const uint8_t value = mapAbsolute(raw, 0, 127);
    s_applyingTask.store(xTaskGetCurrentTaskHandle(), std::memory_order_release);
    switch(function)
    {
        case POD_FUNC_FLANGER_DEPTH:
            control_send_fx_enc(0, value, !p4.enc_muted[0]);
            break;
        case POD_FUNC_DELAY_MIX:
            control_send_fx_enc(1, value, !p4.enc_muted[1]);
            break;
        case POD_FUNC_REVERB_MIX:
            control_send_fx_enc(2, value, !p4.enc_muted[2]);
            break;
        case POD_FUNC_WAVEFOLDER_GAIN:
            control_send_fx_pot(0, value, !p4.pot_muted[0]);
            break;
        case POD_FUNC_CRUSH_MACRO:
            if(p4.bitcrush_bits < 16 || p4.sample_rate_hz > 0)
                control_send_set_crush_macro(0);
            else
                control_send_set_crush_macro(value);
            break;
        case POD_FUNC_PHASER_DEPTH:
            control_send_fx_pot(2, value, !p4.pot_muted[2]);
            break;
        case POD_FUNC_FILTER_CUTOFF:
        case POD_FUNC_FILTER_RESONANCE:
        case POD_FUNC_FILTER_TYPE: {
            static uint8_t savedFilterType = 1;
            if(p4.filter_type != 0)
            {
                savedFilterType = static_cast<uint8_t>(p4.filter_type);
                control_send_set_filter(0);
            }
            else
                control_send_set_filter(savedFilterType == 0 ? 1
                                                               : savedFilterType);
            break;
        }
        case POD_FUNC_DISTORTION:
            control_send_set_distortion(p4.distortion_pct > 0 ? 0.0f
                                                               : normalized);
            break;
        case POD_FUNC_BIT_DEPTH:
            control_send_set_bitcrush(p4.bitcrush_bits < 16 ? 16
                : static_cast<int>(16.0f - normalized * 12.0f + 0.5f));
            break;
        case POD_FUNC_SAMPLE_RATE:
            control_send_set_sample_rate(p4.sample_rate_hz > 0 ? 0
                : (raw < 4u ? 0 : static_cast<int>(42000.0f
                    * powf(4000.0f / 42000.0f, normalized) + 0.5f)));
            break;
        default:
            break;
    }
    s_applyingTask.store(nullptr, std::memory_order_release);
}

}

void i2c_rotaries_init()
{
    Wire1.begin(ROTARY_I2C_SDA, ROTARY_I2C_SCL, kRotaryI2cHz);
    Wire1.setTimeOut(kRotaryI2cTimeoutMs);
    for(uint8_t i = 0; i < kRotaryCount; ++i)
    {
        s_values[i].store(0, std::memory_order_relaxed);
        s_gains[i].store(0, std::memory_order_relaxed);
        s_functions[i].store(POD_FUNC_NONE, std::memory_order_relaxed);
        s_lastProbeMs[i] = 0;
        s_failures[i] = 0;
        s_haveValue[i] = false;
        s_buttonLatched[i] = false;
        s_buttonLastHighMs[i] = 0;
        s_buttonLastAcceptedMs[i] = 0;
        s_processedButtonPressCounts[i] = 0;
        s_buttonPressCounts[i].store(0, std::memory_order_relaxed);
        s_rotaryAddresses[i] = 0;
    }
    s_presentMask.store(0, std::memory_order_relaxed);
    s_addressAckMask.store(0, std::memory_order_relaxed);
    s_buttonPressCount.store(0, std::memory_order_relaxed);
    s_changedMask.store(0, std::memory_order_relaxed);
    s_ownedFunctionMask.store(0, std::memory_order_relaxed);
    s_nextI2cDevice = 0;
    s_muxAddress = 0;
    s_lastMuxProbeMs = 0;
    pinMode(kFaderAdcGpio, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(kFaderAdcGpio, ADC_11db);
    s_faderFunction.store(POD_FUNC_SCREEN_BRIGHTNESS, std::memory_order_relaxed);
    s_faderReady.store(false, std::memory_order_relaxed);
    s_faderRaw.store(0, std::memory_order_relaxed);
    s_faderValue.store(0, std::memory_order_relaxed);
    s_faderChanged.store(false, std::memory_order_relaxed);
    s_lastFaderPollMs = 0;
    s_faderFilteredQ3 = 0;
    s_haveFaderValue = false;
    s_faderLedReady.store(false, std::memory_order_relaxed);
    s_lastFaderLedRefreshMs = 0;
    s_lastFaderLedRetryMs = 0;
    updateFaderLeds(0, millis());
}

void i2c_rotaries_poll()
{
    const uint32_t now = millis();
    pollFader(now);
    // Once a PCA9548A/TCA9548A responds, keep the topology latched until
    // reboot. A single missed ACK must never switch live hardware to the
    // incompatible passive-address mode.
    if(s_muxAddress == 0 && (s_lastMuxProbeMs == 0
       || static_cast<uint32_t>(now - s_lastMuxProbeMs) >= kMuxRetryMs)
      )
    {
        s_lastMuxProbeMs = now;
        const uint8_t detectedMux = findMux();
        if(detectedMux != 0)
        {
            s_muxAddress = detectedMux;
            s_presentMask.store(0, std::memory_order_release);
            s_addressAckMask.store(0, std::memory_order_release);
            s_changedMask.store(0x0Fu, std::memory_order_release);
            for(uint8_t i = 0; i < kRotaryCount; ++i)
            {
                s_haveValue[i] = false;
                s_buttonLatched[i] = false;
                s_buttonLastHighMs[i] = 0;
                s_buttonLastAcceptedMs[i] = 0;
                s_gains[i].store(0, std::memory_order_release);
                s_rotaryAddresses[i] = 0;
                s_failures[i] = 0;
                s_lastProbeMs[i] = 0;
            }
            P4_LOG_PRINTF("[I2C Rotary] active 8-channel mux locked at 0x%02X\n",
                          detectedMux);
        }
    }

    // One SEN0502 downstream channel per pass, so every rotary is sampled
    // approximately every 20 ms.
    const uint8_t device = s_nextI2cDevice;
    s_nextI2cDevice = static_cast<uint8_t>((s_nextI2cDevice + 1u)
                                            % kI2cDeviceCount);
    const uint8_t index = device;
    const uint8_t bit = static_cast<uint8_t>(1u << index);

    uint8_t present = s_presentMask.load(std::memory_order_acquire);
    const bool wasPresent = (present & bit) != 0;
    if(!wasPresent && s_lastProbeMs[index] != 0
       && static_cast<uint32_t>(now - s_lastProbeMs[index]) < kAbsentRetryMs)
        return;

    uint16_t value = 0;
    const bool ok = wasPresent ? readRotary(index, value)
                               : probeRotary(index, value);
    if(!ok)
    {
        s_lastProbeMs[index] = now;
        if(wasPresent && ++s_failures[index] >= kFailuresBeforeOffline)
        {
            s_failures[index] = 0;
            s_haveValue[index] = false;
            s_buttonLatched[index] = false;
            s_buttonLastHighMs[index] = 0;
            s_gains[index].store(0, std::memory_order_release);
            s_rotaryAddresses[index] = 0;
            s_addressAckMask.fetch_and(static_cast<uint8_t>(~bit),
                                       std::memory_order_release);
            s_presentMask.fetch_and(static_cast<uint8_t>(~bit),
                                    std::memory_order_release);
            s_changedMask.fetch_or(bit, std::memory_order_release);
            P4_LOG_PRINTF("[I2C Rotary] #%u 0x%02X disconnected\n",
                          index + 1u, rotaryAddress(index));
        }
        return;
    }

    s_failures[index] = 0;
    if(!wasPresent)
    {
        s_presentMask.fetch_or(bit, std::memory_order_release);
        if(s_muxAddress != 0)
            P4_LOG_PRINTF("[I2C Rotary] #%u SEN0502 channel %u at 0x%02X\n",
                          index + 1u, index, rotaryAddress(index));
        else
            P4_LOG_PRINTF("[I2C Rotary] #%u SEN0502 detected at 0x%02X\n",
                          index + 1u, rotaryAddress(index));
    }
    if(!s_haveValue[index]
       || s_values[index].load(std::memory_order_relaxed) != value)
    {
        s_values[index].store(value, std::memory_order_release);
        s_haveValue[index] = true;
        s_changedMask.fetch_or(bit, std::memory_order_release);
    }
}

void i2c_rotaries_task_start()
{
    if(s_rotaryTask != nullptr) return;
    const BaseType_t ok = xTaskCreatePinnedToCore(
        [](void*) {
            while(true)
            {
                i2c_rotaries_poll();
                vTaskDelay(pdMS_TO_TICKS(kRotaryTaskPeriodMs));
            }
        },
        "rotary_i2c", 4096, nullptr, 2, &s_rotaryTask, 1);
    if(ok != pdPASS)
    {
        s_rotaryTask = nullptr;
        P4_LOG_PRINTLN("[I2C Rotary] failed to create Wire1 task");
    }
}

void i2c_rotaries_process()
{
    const auto& transport = daisyUsb.state();
    uint8_t functions[kRotaryCount] = {};
    const bool configValid = transport.pod.config.version == POD_CONFIG_VERSION;
    if(configValid)
    {
        functions[0] = transport.pod.config.rotary1Function;
        functions[1] = transport.pod.config.rotary2Function;
        functions[2] = transport.pod.config.rotary3Function;
        functions[3] = transport.pod.config.rotary4Function;
    }
    const uint8_t faderFunction = configValid
        ? transport.pod.config.faderFunction : POD_FUNC_SCREEN_BRIGHTNESS;

    uint8_t changed = s_changedMask.exchange(0, std::memory_order_acq_rel);
    if(changed) ui_note_control_activity();
    for(uint8_t i = 0; i < kRotaryCount; ++i)
    {
        const uint8_t previous = s_functions[i].exchange(functions[i],
                                                          std::memory_order_acq_rel);
        if(previous != functions[i]) changed |= static_cast<uint8_t>(1u << i);
    }

    bool faderChanged = s_faderChanged.exchange(false,
                                                 std::memory_order_acq_rel);
    if(faderChanged) ui_note_control_activity();
    if(s_faderFunction.exchange(faderFunction, std::memory_order_acq_rel)
       != faderFunction)
        faderChanged = true;

    const uint8_t present = s_presentMask.load(std::memory_order_acquire);
    uint64_t owned = 0;
    for(uint8_t i = 0; i < kRotaryCount; ++i)
    {
        const uint8_t bit = static_cast<uint8_t>(1u << i);
        const uint8_t function = functions[i];
        const uint32_t buttonCount =
            s_buttonPressCounts[i].load(std::memory_order_acquire);
        const uint32_t presses = buttonCount
                               - s_processedButtonPressCounts[i];
        if(presses) ui_note_control_activity();
        s_processedButtonPressCounts[i] = buttonCount;
        if((present & bit) == 0 || function == POD_FUNC_NONE
           || function >= POD_FUNC_COUNT)
            continue;
        // DaisyPod knobs are the highest-priority mechanical controls. Among
        // P4 rotaries, the lowest address wins if a duplicated assignment is
        // ever received from an older configuration.
        if(configValid && daisyKnobOwns(transport.pod.config, function)) continue;
        const uint64_t functionBits = functionOwnershipBits(function);
        if((owned & functionBits) != 0) continue;
        owned |= functionBits;
        if(changed & bit)
            applyPhysicalValue(function,
                s_values[i].load(std::memory_order_acquire));
        if(presses != 0)
        {
            // A press is consumed per rotary. The hardware task already
            // debounces/acknowledges the SEN0502 latch, so unrelated rotaries
            // can never toggle the wrong effect.
            for(uint32_t press = 0; press < presses && press < 4u; ++press)
                togglePhysicalFx(function,
                    s_values[i].load(std::memory_order_acquire));
        }
    }
    if(s_faderReady.load(std::memory_order_acquire)
       && faderFunction != POD_FUNC_NONE && faderFunction < POD_FUNC_COUNT
       && (!configValid
           || !daisyKnobOwns(transport.pod.config, faderFunction)))
    {
        const uint64_t functionBits = functionOwnershipBits(faderFunction);
        if((owned & functionBits) == 0)
        {
            owned |= functionBits;
            if(faderChanged)
                applyPhysicalValue(faderFunction,
                    s_faderValue.load(std::memory_order_acquire));
        }
    }

    s_ownedFunctionMask.store(owned, std::memory_order_release);
}

bool i2c_rotaries_owns_function(uint8_t function)
{
    if(function >= 64u) return false;
    return (s_ownedFunctionMask.load(std::memory_order_acquire)
            & functionOwnershipBits(function)) != 0;
}

bool i2c_rotaries_is_applying()
{
    return s_applyingTask.load(std::memory_order_acquire)
        == xTaskGetCurrentTaskHandle();
}

uint8_t i2c_rotaries_detected_mask()
{
    return s_presentMask.load(std::memory_order_acquire);
}

uint8_t i2c_rotaries_address_ack_mask()
{
    return s_addressAckMask.load(std::memory_order_acquire);
}

uint16_t i2c_rotaries_value(uint8_t index)
{
    if(index >= kRotaryCount) return 0;
    return s_values[index].load(std::memory_order_acquire);
}

uint8_t i2c_rotaries_gain(uint8_t index)
{
    if(index >= kRotaryCount) return 0;
    return s_gains[index].load(std::memory_order_acquire);
}

uint32_t i2c_rotaries_button_press_count()
{
    return s_buttonPressCount.load(std::memory_order_acquire);
}

uint32_t i2c_rotaries_button_press_count(uint8_t index)
{
    if(index >= kRotaryCount) return 0;
    return s_buttonPressCounts[index].load(std::memory_order_acquire);
}

uint8_t i2c_rotaries_mux_address()
{
    return s_muxAddress.load(std::memory_order_acquire);
}

bool p4_fader_detected()
{
    return s_faderReady.load(std::memory_order_acquire);
}

bool p4_fader_led_driver_ready()
{
    return s_faderLedReady.load(std::memory_order_acquire);
}

uint16_t p4_fader_raw()
{
    return s_faderRaw.load(std::memory_order_acquire);
}

uint16_t p4_fader_value()
{
    return s_faderValue.load(std::memory_order_acquire);
}
