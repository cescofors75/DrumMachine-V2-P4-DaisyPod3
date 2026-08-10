#include "pod_config_store.h"

#include <Preferences.h>
#include <cstring>

namespace {
constexpr uint32_t POD_STORE_MAGIC = 0x32444F50u; // "POD2"
// v3 installs the V7 factory map once. Later user assignments keep persisting
// normally until the wire/config schema is intentionally migrated again.
constexpr uint16_t POD_STORE_VERSION = 3;

struct PodStoreBlob {
    uint32_t magic;
    uint16_t version;
    uint16_t payloadSize;
    PodConfigPayload config;
    uint32_t checksum;
};

uint32_t checksum(const PodConfigPayload& config)
{
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&config);
    uint32_t hash = 2166136261u;
    for(size_t i = 0; i < sizeof(config); ++i)
    {
        hash ^= bytes[i];
        hash *= 16777619u;
    }
    return hash;
}

void fields(PodConfigPayload& config, uint8_t* out[11])
{
    out[0] = &config.button1Function;
    out[1] = &config.button2Function;
    out[2] = &config.knob1Function;
    out[3] = &config.knob2Function;
    out[4] = &config.encoderFunction;
    out[5] = &config.encoderButtonFunction;
    out[6] = &config.rotary1Function;
    out[7] = &config.rotary2Function;
    out[8] = &config.rotary3Function;
    out[9] = &config.rotary4Function;
    out[10] = &config.faderFunction;
}

bool functionsConflict(uint8_t left, uint8_t right)
{
    if(left == POD_FUNC_NONE || right == POD_FUNC_NONE) return false;
    if(left == right) return true;
    const bool leftCrush = left == POD_FUNC_CRUSH_MACRO;
    const bool rightCrush = right == POD_FUNC_CRUSH_MACRO;
    return (leftCrush && (right == POD_FUNC_BIT_DEPTH
                          || right == POD_FUNC_SAMPLE_RATE))
        || (rightCrush && (left == POD_FUNC_BIT_DEPTH
                           || left == POD_FUNC_SAMPLE_RATE));
}
}

void pod_config_store_factory_defaults(PodConfigPayload& config)
{
    config = PodConfigPayload{
        POD_CONFIG_VERSION,
        POD_FUNC_BACK, POD_FUNC_CONTROL_CONFIG,
        POD_FUNC_MASTER_VOLUME, POD_FUNC_TEMPO,
        POD_FUNC_PATTERN_NEXT, POD_FUNC_PLAY_TOGGLE,
        POD_FUNC_DELAY_MIX, POD_FUNC_REVERB_MIX,
        POD_FUNC_FLANGER_DEPTH, POD_FUNC_PHASER_DEPTH,
        POD_FUNC_NONE,
        POD_LED_USB_LINK, 255, 24, 12,
        POD_LED_SAMPLES_READY, 0, 255, 90,
        POD_FUNC_SCREEN_BRIGHTNESS
    };
}

void pod_config_store_sanitize(PodConfigPayload& config)
{
    config.version = POD_CONFIG_VERSION;
    config.selectorFunction = POD_FUNC_NONE;
    uint8_t* functionFields[11] = {};
    fields(config, functionFields);
    for(uint8_t i = 0; i < 11; ++i)
    {
        if(*functionFields[i] >= POD_FUNC_COUNT)
            *functionFields[i] = POD_FUNC_NONE;
        if(*functionFields[i] == POD_FUNC_NONE) continue;
        for(uint8_t previous = 0; previous < i; ++previous)
        {
            if(functionsConflict(*functionFields[previous], *functionFields[i]))
            {
                *functionFields[i] = POD_FUNC_NONE;
                break;
            }
        }
    }
    if(config.led1Function >= POD_LED_COUNT) config.led1Function = POD_LED_FIXED;
    if(config.led2Function >= POD_LED_COUNT) config.led2Function = POD_LED_FIXED;
}

bool pod_config_store_load(PodConfigPayload& config)
{
    Preferences prefs;
    if(!prefs.begin("drumv2pod", true)) return false;
    PodStoreBlob blob{};
    const bool ok = prefs.getBytesLength("config") == sizeof(blob)
                 && prefs.getBytes("config", &blob, sizeof(blob)) == sizeof(blob)
                 && blob.magic == POD_STORE_MAGIC
                 && blob.version == POD_STORE_VERSION
                 && blob.payloadSize == sizeof(PodConfigPayload)
                 && blob.checksum == checksum(blob.config);
    prefs.end();
    if(!ok) return false;
    config = blob.config;
    pod_config_store_sanitize(config);
    return true;
}

bool pod_config_store_save(const PodConfigPayload& requested)
{
    PodStoreBlob blob{};
    blob.magic = POD_STORE_MAGIC;
    blob.version = POD_STORE_VERSION;
    blob.payloadSize = sizeof(PodConfigPayload);
    blob.config = requested;
    pod_config_store_sanitize(blob.config);
    blob.checksum = checksum(blob.config);

    Preferences prefs;
    if(!prefs.begin("drumv2pod", false)) return false;
    const bool ok = prefs.putBytes("config", &blob, sizeof(blob)) == sizeof(blob);
    prefs.end();
    return ok;
}
