#pragma once
#include <stdint.h>
#include <stddef.h>

// Full replacement protocol: BEGIN, 16 TRACK packets, COMMIT(hash).
#define CMD_PATTERN_BEGIN  0xF5
#define CMD_PATTERN_TRACK  0xF6
#define CMD_PATTERN_COMMIT 0xF7
struct __attribute__((packed)) PatternWireStep {
    uint8_t active, velocity, division, probability, flags;
    uint8_t notes[4];
    uint8_t cutoffEn;
    uint16_t cutoffHz;
    uint8_t reverbEn, reverbSend, volumeEn, volume;
};
static_assert(sizeof(PatternWireStep) == 16, "pattern wire layout");
inline uint8_t PackStepDivision(uint8_t division, uint8_t ratchet) {
    if(ratchet < 1) ratchet = 1;
    if(ratchet > 4) ratchet = 4;
    return (division & 15u) | ((ratchet - 1u) << 4);
}
inline uint32_t PatternHash(const void* data, size_t size, uint32_t hash = 2166136261u) {
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    while(size--) { hash ^= *bytes++; hash *= 16777619u; }
    return hash;
}

struct PatternTransferCheck {
    uint8_t slot = 0xFF;
    uint16_t token = 0, mask = 0;
    uint32_t hash = 2166136261u;
    void begin(uint8_t destination, uint16_t id) {
        slot = destination; token = id; mask = 0; hash = 2166136261u;
    }
    bool track(uint8_t destination, uint16_t id, uint8_t index, const void* data, size_t bytes) {
        if(destination != slot || id != token || index >= 16
           || bytes != 16 * sizeof(PatternWireStep) || mask != ((1u << index) - 1u)) return false;
        hash = PatternHash(data, bytes, hash);
        mask |= 1u << index;
        return true;
    }
    bool ready(uint8_t destination, uint16_t id, uint32_t expectedHash) const {
        return destination == slot && id == token && mask == 0xFFFFu && hash == expectedHash;
    }
};
