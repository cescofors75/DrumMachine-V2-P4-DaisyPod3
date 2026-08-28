#include "daisy_usb_transport.h"

#include "usb_cdc_handler.h"
#include "ui/ui_screens.h"
#include <string.h>

DaisyUsbTransport daisyUsb;

namespace
{
constexpr uint32_t kEngineTimeoutMs = 3000;
constexpr uint32_t kPingIntervalMs = 1000;
constexpr uint32_t kPositionIntervalMs = 40;
constexpr uint32_t kPodStateIntervalMs = 25;
constexpr uint32_t kMidiEventsIntervalMs = 50;
constexpr uint32_t kQueryTimeoutMs = 400;
constexpr size_t kTelemetryBacklogLimit = 8;
}

uint16_t DaisyUsbTransport::crc16(const uint8_t* data, uint16_t length)
{
    // RED808/DaisyPod3 uses CRC16-Modbus (poly 0xA001, reflected). Keep this
    // identical to DaisyPod3::crc16(): using CRC16-CCITT here makes P4 accept
    // the CDC link but discard every valid response, leaving the UI at
    // "USB WAIT" indefinitely.
    uint16_t crc = 0xFFFFu;
    for(uint16_t i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for(uint8_t bit = 0; bit < 8; ++bit)
            crc = (crc & 1u) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001u)
                             : static_cast<uint16_t>(crc >> 1);
    }
    return crc;
}

void DaisyUsbTransport::begin()
{
    state_ = {};
    sequence_.store(0, std::memory_order_relaxed);
    sample_end_ack_revision_.store(0, std::memory_order_relaxed);
    sample_end_ack_pad_.store(0xFFu, std::memory_order_relaxed);
    sample_end_ack_accepted_.store(false, std::memory_order_relaxed);
    rx_length_ = 0;
    rx_target_ = 0;
    pending_query_ = false;
    pending_query_command_ = 0;
    pending_query_sequence_ = 0;
    pending_query_since_ms_ = 0;
}

bool DaisyUsbTransport::send(uint8_t command, const void* payload,
                             uint16_t payload_length)
{
    return sendPacket(command, payload, payload_length, nullptr);
}

bool DaisyUsbTransport::sendPacket(uint8_t command, const void* payload,
                                   uint16_t payload_length,
                                   uint16_t* assigned_sequence)
{
    if(payload_length > SPI_MAX_PAYLOAD || !usb_cdc_connected())
    {
        state_.tx_drops++;
        return false;
    }

    uint8_t packet[sizeof(SPIPacketHeader) + SPI_MAX_PAYLOAD];
    SPIPacketHeader header = {};
    header.magic = SPI_MAGIC_CMD;
    header.cmd = command;
    header.length = payload_length;
    header.sequence = sequence_.fetch_add(1, std::memory_order_relaxed);
    header.checksum = payload_length > 0
        ? crc16(static_cast<const uint8_t*>(payload), payload_length)
        : 0;
    memcpy(packet, &header, sizeof(header));
    if(payload != nullptr && payload_length > 0)
        memcpy(packet + sizeof(header), payload, payload_length);

    const size_t packet_length = sizeof(header) + payload_length;
    if(usb_cdc_write(packet, packet_length) != packet_length)
    {
        state_.tx_drops++;
        return false;
    }
    state_.tx_packets++;
    if(assigned_sequence != nullptr) *assigned_sequence = header.sequence;
    return true;
}

bool DaisyUsbTransport::sendQuery(uint8_t command, const void* payload,
                                  uint16_t payload_length)
{
    uint16_t sequence = 0;
    if(!sendPacket(command, payload, payload_length, &sequence)) return false;
    pending_query_ = true;
    pending_query_command_ = command;
    pending_query_sequence_ = sequence;
    pending_query_since_ms_ = millis();
    return true;
}

bool DaisyUsbTransport::sendU8(uint8_t command, uint8_t value)
{
    return send(command, &value, sizeof(value));
}

bool DaisyUsbTransport::sendFloat(uint8_t command, float value)
{
    return send(command, &value, sizeof(value));
}

void DaisyUsbTransport::trigger(uint8_t pad, uint8_t velocity)
{
    const uint8_t payload[2] = {pad, velocity};
    send(CMD_TRIGGER_LIVE, payload, sizeof(payload));
}

void DaisyUsbTransport::start() { sendU8(CMD_DSQ_CONTROL, 1); }
void DaisyUsbTransport::stop() { sendU8(CMD_DSQ_CONTROL, 0); }
void DaisyUsbTransport::reset() { sendU8(CMD_DSQ_CONTROL, 2); }
void DaisyUsbTransport::stopAll() { send(CMD_TRIGGER_STOP_ALL); }
void DaisyUsbTransport::setTempo(float bpm) { sendFloat(CMD_TEMPO, bpm); }
void DaisyUsbTransport::selectPattern(uint8_t pattern)
{
    sendU8(CMD_DSQ_SELECT_PATTERN, pattern);
}

void DaisyUsbTransport::queuePattern(uint8_t pattern, uint8_t bars)
{
    const uint8_t payload[2] = {pattern, bars};
    send(CMD_DSQ_QUEUE_PATTERN, payload, sizeof(payload));
}

void DaisyUsbTransport::cancelPatternQueue()
{
    queuePattern(0xFFu, 0);
}

void DaisyUsbTransport::setStep(uint8_t pattern, uint8_t track, uint8_t step,
                                bool active, uint8_t velocity,
                                uint8_t note_division, uint8_t probability)
{
    const uint8_t payload[8] = {
        pattern, track, step, static_cast<uint8_t>(active ? 1 : 0),
        velocity, note_division, probability, 0
    };
    send(CMD_DSQ_SET_STEP, payload, sizeof(payload));
}

void DaisyUsbTransport::uploadTrack(uint8_t pattern, uint8_t track,
                                    const bool steps[16], uint8_t velocity)
{
    uint8_t payload[4 + 16 * 4] = {pattern, track, 16, 0};
    for(uint8_t step = 0; step < 16; ++step)
    {
        const uint16_t offset = 4 + step * 4;
        payload[offset + 0] = steps[step] ? 1u : 0u;
        payload[offset + 1] = velocity;
        payload[offset + 2] = 1u;
        payload[offset + 3] = 100u;
    }
    send(CMD_DSQ_UPLOAD_TRACK, payload, sizeof(payload));
}

void DaisyUsbTransport::setTrackMute(uint8_t track, bool muted)
{
    const uint8_t payload[2] = {track, static_cast<uint8_t>(muted ? 1 : 0)};
    send(CMD_TRACK_MUTE, payload, sizeof(payload));
    send(CMD_DSQ_SET_MUTE, payload, sizeof(payload));
}

void DaisyUsbTransport::setTrackSolo(uint8_t track, bool soloed)
{
    const uint8_t payload[2] = {track, static_cast<uint8_t>(soloed ? 1 : 0)};
    send(CMD_TRACK_SOLO, payload, sizeof(payload));
}

void DaisyUsbTransport::setTrackMuteMask(uint16_t mask)
{
    const uint8_t payload[2] = {
        static_cast<uint8_t>(mask & 0xFFu),
        static_cast<uint8_t>(mask >> 8)
    };
    send(CMD_TRACK_MUTE_MASK, payload, sizeof(payload));
}

void DaisyUsbTransport::setTrackSoloMask(uint16_t mask)
{
    const uint8_t payload[2] = {
        static_cast<uint8_t>(mask & 0xFFu),
        static_cast<uint8_t>(mask >> 8)
    };
    send(CMD_TRACK_SOLO_MASK, payload, sizeof(payload));
}

void DaisyUsbTransport::setTrackVolume(uint8_t track, uint8_t volume)
{
    const uint8_t payload[2] = {track, volume};
    send(CMD_TRACK_VOLUME, payload, sizeof(payload));
}

void DaisyUsbTransport::setTrackEngine(uint8_t track, int8_t engine)
{
    const uint8_t payload[2] = {track, static_cast<uint8_t>(engine)};
    send(CMD_DSQ_SET_TRACK_ENGINE, payload, sizeof(payload));
}

// CMD_TRACK_FILTER payload (16B): [track, filterType, rsvd, rsvd,
// float cutoffHz(4B), float resonance(4B), float gain(4B, unused here=0)]
void DaisyUsbTransport::setTrackFilter(uint8_t track, uint8_t filterType,
                                       float cutoffHz, float resonance)
{
    uint8_t payload[16] = {track, filterType, 0, 0};
    memcpy(payload + 4, &cutoffHz, 4);
    memcpy(payload + 8, &resonance, 4);
    const float gain = 0.0f;
    memcpy(payload + 12, &gain, 4);
    send(CMD_TRACK_FILTER, payload, sizeof(payload));
}

void DaisyUsbTransport::clearTrackFilter(uint8_t track)
{
    sendU8(CMD_TRACK_CLEAR_FILTER, track);
}

// CMD_TRACK_DISTORTION payload (8B): [track, mode, rsvd, rsvd, float amount(4B) 0..1]
void DaisyUsbTransport::setTrackDistortion(uint8_t track, float amount01)
{
    uint8_t payload[8] = {track, 0, 0, 0};
    memcpy(payload + 4, &amount01, 4);
    send(CMD_TRACK_DISTORTION, payload, sizeof(payload));
}

void DaisyUsbTransport::setTrackBitcrush(uint8_t track, uint8_t bits)
{
    const uint8_t payload[2] = {track, bits};
    send(CMD_TRACK_BITCRUSH, payload, sizeof(payload));
}

void DaisyUsbTransport::setTrackReverbSend(uint8_t track, uint8_t percent)
{
    const uint8_t payload[2] = {track, percent};
    send(CMD_TRACK_REVERB_SEND, payload, sizeof(payload));
}

void DaisyUsbTransport::setTrackDelaySend(uint8_t track, uint8_t percent)
{
    const uint8_t payload[2] = {track, percent};
    send(CMD_TRACK_DELAY_SEND, payload, sizeof(payload));
}

void DaisyUsbTransport::clearTrackFx(uint8_t track)
{
    sendU8(CMD_TRACK_CLEAR_FX, track);
}

void DaisyUsbTransport::synthTrigger(uint8_t engine, uint8_t instrument,
                                     uint8_t velocity)
{
    const uint8_t payload[3] = {engine, instrument, velocity};
    send(CMD_SYNTH_TRIGGER, payload, sizeof(payload));
}

void DaisyUsbTransport::synthNoteOn(uint8_t engine, uint8_t note,
                                    uint8_t velocity, bool accent, bool slide)
{
    const uint8_t payload[5] = {
        engine, note, velocity, static_cast<uint8_t>(accent ? 1 : 0),
        static_cast<uint8_t>(slide ? 1 : 0)
    };
    send(CMD_SYNTH_NOTE_ON_EX, payload, sizeof(payload));
}

void DaisyUsbTransport::synthNoteOff(uint8_t engine, uint8_t track, uint8_t note)
{
    const uint8_t payload[3] = {engine, track, note};
    send(CMD_SYNTH_NOTE_OFF, payload, sizeof(payload));
}

void DaisyUsbTransport::synthParam(uint8_t engine, uint8_t instrument,
                                   uint8_t parameter, float value)
{
    uint8_t payload[7] = {engine, instrument, parameter, 0, 0, 0, 0};
    memcpy(payload + 3, &value, sizeof(value));
    send(CMD_SYNTH_PARAM, payload, sizeof(payload));
}

void DaisyUsbTransport::synthPreset(uint8_t engine, uint8_t preset)
{
    const uint8_t payload[2] = {engine, preset};
    send(CMD_SYNTH_PRESET, payload, sizeof(payload));
}

bool DaisyUsbTransport::uploadSong(const SongEntry* entries, uint8_t count)
{
    if(entries == nullptr || count == 0) return false;
    if(count > SONG_MAX_ENTRIES) count = SONG_MAX_ENTRIES;
    uint8_t payload[1 + SONG_MAX_ENTRIES * 2] = {};
    payload[0] = count;
    for(uint8_t i = 0; i < count; ++i)
    {
        payload[1 + i * 2] = entries[i].pattern;
        payload[2 + i * 2] = entries[i].repeats == 0
            ? 1u : entries[i].repeats;
    }
    return send(CMD_SONG_UPLOAD, payload, static_cast<uint16_t>(1 + count * 2));
}

bool DaisyUsbTransport::controlSong(uint8_t action)
{
    if(action > 2u) return false;
    return sendU8(CMD_SONG_CONTROL, action);
}

bool DaisyUsbTransport::sendMidiMap(const MidiMapEntry* entries, uint8_t count)
{
    if(count > MIDI_MAP_MAX_ENTRIES) count = MIDI_MAP_MAX_ENTRIES;
    if(count > 0 && entries == nullptr) return false;
    uint8_t payload[1 + MIDI_MAP_MAX_ENTRIES * sizeof(MidiMapEntry)];
    payload[0] = count;
    if(count > 0)
        memcpy(payload + 1, entries, count * sizeof(MidiMapEntry));
    return send(CMD_MIDI_MAP_SET, payload,
                static_cast<uint16_t>(1 + count * sizeof(MidiMapEntry)));
}

bool DaisyUsbTransport::popMidiEvent(MidiMonitorEvent& event)
{
    if(midi_events_tail_ == midi_events_head_) return false;
    event = midi_events_[midi_events_tail_];
    midi_events_tail_ = (midi_events_tail_ + 1u) & 63u;
    return true;
}

void DaisyUsbTransport::parseByte(uint8_t byte)
{
    if(rx_length_ == 0)
    {
        if(byte != SPI_MAGIC_RESP)
            return;
        rx_packet_[rx_length_++] = byte;
        rx_target_ = 0;
        return;
    }

    if(rx_length_ >= sizeof(rx_packet_))
    {
        state_.framing_errors++;
        rx_length_ = 0;
        rx_target_ = 0;
        return;
    }
    rx_packet_[rx_length_++] = byte;

    if(rx_length_ == sizeof(SPIPacketHeader))
    {
        const SPIPacketHeader* header
            = reinterpret_cast<const SPIPacketHeader*>(rx_packet_);
        rx_target_ = sizeof(SPIPacketHeader) + header->length;
        if(header->length > SPI_MAX_PAYLOAD || rx_target_ > sizeof(rx_packet_))
        {
            state_.framing_errors++;
            rx_length_ = 0;
            rx_target_ = 0;
            return;
        }
    }

    if(rx_target_ != 0 && rx_length_ == rx_target_)
    {
        handleResponse(rx_packet_, rx_length_);
        rx_length_ = 0;
        rx_target_ = 0;
    }
}

void DaisyUsbTransport::handleResponse(const uint8_t* packet,
                                       uint16_t packet_length)
{
    if(packet_length < sizeof(SPIPacketHeader))
        return;
    const SPIPacketHeader* header
        = reinterpret_cast<const SPIPacketHeader*>(packet);
    const uint8_t* payload = packet + sizeof(SPIPacketHeader);
    if(header->length > 0 && crc16(payload, header->length) != header->checksum)
    {
        state_.crc_errors++;
        return;
    }

    state_.rx_packets++;
    state_.last_response_ms = millis();
    // Do not declare the audio engine ready until PING has negotiated the
    // exact wire layout. A 2.2 Daisy can answer position/status queries, but
    // its shorter PodStatePayload is not safe for a 2.3 P4.
    if(state_.protocol_version == RED808_PROTOCOL_VERSION)
        state_.engine_responding = true;

    const bool is_query_response = header->cmd == CMD_PING
        || header->cmd == CMD_DSQ_GET_POS || header->cmd == CMD_GET_STATUS
        || header->cmd == CMD_POD_GET_STATE
        || header->cmd == CMD_MIDI_GET_EVENTS;
    if(pending_query_ && is_query_response)
    {
        if(header->cmd == pending_query_command_
           && header->sequence == pending_query_sequence_)
        {
            pending_query_ = false;
        }
        else if(header->cmd == pending_query_command_)
        {
            state_.stale_responses++;
        }
    }

    if(header->cmd == CMD_PING && header->length >= 8)
    {
        uint32_t echo_ms = 0;
        memcpy(&echo_ms, payload, sizeof(echo_ms));
        state_.round_trip_ms = millis() - echo_ms;
        if(header->length >= sizeof(LinkHealthResponse))
        {
            LinkHealthResponse health = {};
            memcpy(&health, payload, sizeof(health));
            state_.protocol_version = health.protocolVersion;
            state_.engine_responding =
                health.protocolVersion == RED808_PROTOCOL_VERSION;
            state_.capability_flags = health.capabilityFlags;
            state_.daisy_rx_drops = health.rxDrops;
            state_.daisy_protocol_errors = health.protocolErrors;
        }
    }
    else if(header->cmd == CMD_DSQ_GET_POS && header->length >= 3)
    {
        state_.step = payload[0];
        state_.pattern = payload[1];
        state_.playing = payload[2] != 0;
    }
    else if(header->cmd == CMD_GET_STATUS && header->length >= 10)
    {
        state_.sample_mask = static_cast<uint16_t>(payload[2])
                           | (static_cast<uint16_t>(payload[3]) << 8);
        state_.sd_present = payload[8] != 0;
        state_.xtra_sample_mask = payload[9];
        if(header->length >= 46)
        {
            memcpy(state_.kit_name, payload + 14, 31);
            state_.kit_name[31] = '\0';
        }
        if(header->length >= 76)
            ui_pad_sound_sync_track_engines(
                reinterpret_cast<const int8_t*>(payload + 60));
        if(header->length >= 80)
        {
            state_.sd_mount_result = payload[76];
            state_.sd_root_result = payload[77];
            state_.sd_boot_loaded = payload[78];
            state_.sd_load_failures = payload[79];
        }
        if(header->length >= 87)
        {
            state_.sd_diag_stage = payload[80];
            state_.sd_card_type = payload[81];
            state_.sd_last_command = payload[82];
            state_.sd_last_response = payload[83];
            state_.sd_last_data_token = payload[84];
            state_.sd_spi_errors = static_cast<uint16_t>(payload[85])
                                 | (static_cast<uint16_t>(payload[86]) << 8);
        }
    }
    else if(header->cmd == CMD_SAMPLE_END && header->length >= 2)
    {
        sample_end_ack_pad_.store(payload[0], std::memory_order_relaxed);
        sample_end_ack_accepted_.store(payload[1] != 0, std::memory_order_relaxed);
        sample_end_ack_revision_.fetch_add(1, std::memory_order_release);
    }
    else if(header->cmd == CMD_SD_STATUS
            && header->length >= sizeof(SdStatusResponse))
    {
        SdStatusResponse response = {};
        memcpy(&response, payload, sizeof(response));
        state_.daisy_sd_status_seen = true;
        state_.sd_present = response.present != 0;
        state_.daisy_sd_sample_mask = response.samplesLoaded;
        memcpy(state_.daisy_sd_kit, response.currentKit,
               sizeof(state_.daisy_sd_kit));
        state_.daisy_sd_kit[sizeof(state_.daisy_sd_kit) - 1] = '\0';
        state_.daisy_sd_revision++;
    }
    else if(header->cmd == CMD_SD_LIST_FOLDERS && header->length >= 1)
    {
        uint8_t count = payload[0];
        const uint16_t available = (header->length - 1u) / 32u;
        if(count > 16u) count = 16u;
        if(count > available) count = static_cast<uint8_t>(available);
        state_.daisy_sd_folder_count = count;
        memset(state_.daisy_sd_folders, 0, sizeof(state_.daisy_sd_folders));
        if(count > 0)
            memcpy(state_.daisy_sd_folders, payload + 1, count * 32u);
        for(uint8_t i = 0; i < count; i++)
            state_.daisy_sd_folders[i][31] = '\0';
        state_.daisy_sd_revision++;
    }
    else if(header->cmd == CMD_SD_LIST_FILES && header->length >= 1)
    {
        uint8_t count = payload[0];
        const uint16_t available = (header->length - 1u) / 32u;
        if(count > 20u) count = 20u;
        if(count > available) count = static_cast<uint8_t>(available);
        state_.daisy_sd_file_count = count;
        memset(state_.daisy_sd_files, 0, sizeof(state_.daisy_sd_files));
        if(count > 0)
            memcpy(state_.daisy_sd_files, payload + 1, count * 32u);
        for(uint8_t i = 0; i < count; i++)
            state_.daisy_sd_files[i][31] = '\0';
        state_.daisy_sd_revision++;
    }
    else if((header->cmd == CMD_POD_GET_STATE
             || header->cmd == CMD_POD_SET_CONFIG)
            && header->length >= sizeof(PodStatePayload))
    {
        memcpy(&state_.pod, payload, sizeof(PodStatePayload));
        state_.pod_revision++;
    }
    else if(header->cmd == CMD_MIDI_GET_EVENTS && header->length >= 1)
    {
        uint8_t count = payload[0];
        const uint16_t available = (header->length - 1u) / 3u;
        if(count > MIDI_MON_EVENTS_PER_POLL) count = MIDI_MON_EVENTS_PER_POLL;
        if(count > available) count = static_cast<uint8_t>(available);
        for(uint8_t i = 0; i < count; ++i)
        {
            const uint8_t next = (midi_events_head_ + 1u) & 63u;
            if(next == midi_events_tail_) // full → drop oldest
                midi_events_tail_ = (midi_events_tail_ + 1u) & 63u;
            midi_events_[midi_events_head_].status = payload[1 + i * 3 + 0];
            midi_events_[midi_events_head_].data0 = payload[1 + i * 3 + 1];
            midi_events_[midi_events_head_].data1 = payload[1 + i * 3 + 2];
            midi_events_head_ = next;
        }
    }
}

void DaisyUsbTransport::poll()
{
    const uint32_t now = millis();
    if(pending_query_)
    {
        if(now - pending_query_since_ms_ < kQueryTimeoutMs) return;
        pending_query_ = false;
        state_.query_timeouts++;
    }

    // Telemetry is expendable; live pad hits, transport and FX control are not.
    // Do not add polling packets while the USB worker is draining a burst.
    if(usb_cdc_tx_pending() > kTelemetryBacklogLimit) return;

    if(now - last_ping_ms_ >= kPingIntervalMs)
    {
        if(sendQuery(CMD_PING, &now, sizeof(now))) last_ping_ms_ = now;
        return;
    }
    if(now - last_position_ms_ >= kPositionIntervalMs)
    {
        if(sendQuery(CMD_DSQ_GET_POS)) last_position_ms_ = now;
        return;
    }
    if(now - last_status_ms_ >= 1000u)
    {
        if(sendQuery(CMD_GET_STATUS)) last_status_ms_ = now;
        return;
    }
    if(now - last_pod_state_ms_ >= kPodStateIntervalMs)
    {
        if(sendQuery(CMD_POD_GET_STATE)) last_pod_state_ms_ = now;
        return;
    }
    if((state_.capability_flags & RED808_CAP_MIDI_MONITOR)
       && now - last_midi_events_ms_ >= kMidiEventsIntervalMs)
    {
        if(sendQuery(CMD_MIDI_GET_EVENTS)) last_midi_events_ms_ = now;
        return;
    }
}

void DaisyUsbTransport::process()
{
    state_.link_ready = usb_cdc_connected();
    while(usb_cdc_available() > 0)
    {
        const int byte = usb_cdc_read();
        if(byte >= 0)
            parseByte(static_cast<uint8_t>(byte));
    }

    if(!state_.link_ready)
    {
        state_.engine_responding = false;
        state_.protocol_version = 0;
        state_.capability_flags = 0;
        rx_length_ = 0;
        rx_target_ = 0;
        pending_query_ = false;
        return;
    }
    if(state_.last_response_ms != 0
       && millis() - state_.last_response_ms > kEngineTimeoutMs)
        state_.engine_responding = false;
    poll();
}
