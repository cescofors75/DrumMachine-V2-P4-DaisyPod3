#include "pattern_store.h"

#include "master/PatternBank.h"
#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <cstring>

namespace {
constexpr uint32_t PATTERN_MAGIC = 0x32504D44u; // "DMP2"
constexpr uint16_t PATTERN_FILE_VERSION = 1;
uint32_t savedMask = 0;

struct __attribute__((packed)) PatternFilePayload {
    uint8_t profileValid;
    uint8_t reserved[3];
    BuiltinPatternSoundProfile profile;
    PatternStorageData pattern;
};

struct __attribute__((packed)) PatternFileHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t logicalPattern;
    uint32_t payloadSize;
    uint32_t checksum;
};

uint32_t checksum(const void* data, size_t length)
{
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    uint32_t hash = 2166136261u;
    for(size_t i = 0; i < length; ++i)
    {
        hash ^= bytes[i];
        hash *= 16777619u;
    }
    return hash;
}

void patternPath(int pattern, char* out, size_t size)
{
    snprintf(out, size, "/user_p%03d.dmv2", pattern + 1);
}

bool loadOne(Sequencer& sequencer, int pattern)
{
    char path[32];
    patternPath(pattern, path, sizeof(path));
    File file = SPIFFS.open(path, FILE_READ);
    if(!file) return false;

    PatternFileHeader header{};
    if(file.read(reinterpret_cast<uint8_t*>(&header), sizeof(header)) != sizeof(header)
       || header.magic != PATTERN_MAGIC
       || header.version != PATTERN_FILE_VERSION
       || header.logicalPattern != pattern
       || header.payloadSize != sizeof(PatternFilePayload))
    {
        file.close();
        return false;
    }

    auto* payload = static_cast<PatternFilePayload*>(ps_malloc(sizeof(PatternFilePayload)));
    if(!payload)
    {
        file.close();
        return false;
    }
    const bool readOk = file.read(reinterpret_cast<uint8_t*>(payload), sizeof(*payload))
                     == sizeof(*payload);
    file.close();
    const bool valid = readOk && header.checksum == checksum(payload, sizeof(*payload));
    bool restored = false;
    if(valid)
    {
        restored = sequencer.restorePatternFromStorage(pattern, &payload->pattern);
        if(restored && payload->profileValid)
            setPatternSoundProfile(pattern, payload->profile);
    }
    free(payload);
    return restored;
}
}

bool pattern_store_is_user_slot(int pattern)
{
    return pattern >= USER_PATTERN_FIRST && pattern < MAX_PATTERNS;
}

bool pattern_store_is_saved(int pattern)
{
    if(!pattern_store_is_user_slot(pattern)) return false;
    return (savedMask & (1u << (pattern - USER_PATTERN_FIRST))) != 0;
}

void pattern_store_load_user_bank(Sequencer& sequencer)
{
    savedMask = 0;
    for(int pattern = USER_PATTERN_FIRST; pattern < MAX_PATTERNS; ++pattern)
    {
        if(loadOne(sequencer, pattern))
            savedMask |= 1u << (pattern - USER_PATTERN_FIRST);
    }
}

bool pattern_store_save_user(Sequencer& sequencer, int source, int destination)
{
    if(source < 0 || source >= MAX_PATTERNS
       || !pattern_store_is_user_slot(destination))
        return false;

    if(source != destination)
    {
        sequencer.copyPattern(source, destination);
        BuiltinPatternSoundProfile sourceProfile{};
        if(getBuiltinPatternSoundProfile(source, sourceProfile))
            setPatternSoundProfile(destination, sourceProfile);
    }

    auto* payload = static_cast<PatternFilePayload*>(ps_calloc(1, sizeof(PatternFilePayload)));
    if(!payload) return false;
    payload->profileValid = getBuiltinPatternSoundProfile(destination, payload->profile) ? 1u : 0u;
    if(!sequencer.snapshotPatternForStorage(destination, &payload->pattern))
    {
        free(payload);
        return false;
    }

    PatternFileHeader header{};
    header.magic = PATTERN_MAGIC;
    header.version = PATTERN_FILE_VERSION;
    header.logicalPattern = static_cast<uint16_t>(destination);
    header.payloadSize = sizeof(PatternFilePayload);
    header.checksum = checksum(payload, sizeof(*payload));

    char path[32];
    patternPath(destination, path, sizeof(path));
    if(SPIFFS.exists(path)) SPIFFS.remove(path);
    File file = SPIFFS.open(path, FILE_WRITE);
    bool ok = file
           && file.write(reinterpret_cast<const uint8_t*>(&header), sizeof(header)) == sizeof(header)
           && file.write(reinterpret_cast<const uint8_t*>(payload), sizeof(*payload)) == sizeof(*payload);
    if(file)
    {
        file.flush();
        file.close();
    }
    free(payload);
    if(ok) savedMask |= 1u << (destination - USER_PATTERN_FIRST);
    return ok;
}
