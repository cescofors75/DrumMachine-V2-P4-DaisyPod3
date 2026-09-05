#include "pattern_store.h"
#include "master/PatternBank.h"
#include "../../shared/pattern_transfer.h"
#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <atomic>
#include <cstring>

namespace {
constexpr uint32_t PATTERN_MAGIC = 0x32504D44u;
std::atomic<uint32_t> savedMask{0};
std::atomic_flag writer = ATOMIC_FLAG_INIT;
struct __attribute__((packed)) Payload {
    uint8_t profileValid;
    uint8_t reserved[3];
    BuiltinPatternSoundProfile profile;
    PatternStorageData pattern;
};
struct __attribute__((packed)) Header {
    uint32_t magic;
    uint16_t version, logicalPattern;
    uint32_t payloadSize, checksum;
};
// V2 appends generation to the V1 header and includes it in the checksum.
void pathFor(int pattern, int copy, char* path) {
    if(copy < 0) snprintf(path, 40, "/user_p%03d.dmv2", pattern + 1);
    else snprintf(path, 40, "/user_p%03d_%c.dmv2", pattern + 1, 'a' + copy);
}
bool readCopy(int pattern, int copy, Payload* payload, uint32_t& generation) {
    char path[40]; pathFor(pattern, copy, path);
    File file = SPIFFS.open(path, FILE_READ);
    Header h{};
    generation = 0;
    if(!file || file.read(reinterpret_cast<uint8_t*>(&h), sizeof(h)) != sizeof(h)
       || h.magic != PATTERN_MAGIC || h.logicalPattern != pattern
       || h.payloadSize != sizeof(Payload) || (h.version != 1 && h.version != 2)) return false;
    if(h.version == 2 && file.read(reinterpret_cast<uint8_t*>(&generation), 4) != 4) return false;
    if(file.read(reinterpret_cast<uint8_t*>(payload), sizeof(*payload)) != sizeof(*payload)) return false;
    const uint32_t hash = h.version == 2 ? PatternHash(&generation, 4) : 2166136261u;
    return h.checksum == PatternHash(payload, sizeof(*payload), hash);
}
bool newer(uint32_t a, uint32_t b) { return int32_t(a - b) > 0; }
bool restore(Sequencer& seq, int pattern, const Payload* payload) {
    if(!seq.restorePatternFromStorage(pattern, &payload->pattern)) return false;
    if(payload->profileValid) setPatternSoundProfile(pattern, payload->profile);
    else clearPatternSoundProfile(pattern);
    return true;
}
}

bool pattern_store_is_user_slot(int pattern) { return pattern >= USER_PATTERN_FIRST && pattern < MAX_PATTERNS; }
bool pattern_store_is_saved(int pattern) {
    return pattern_store_is_user_slot(pattern) && (savedMask.load() & (1u << (pattern - USER_PATTERN_FIRST)));
}
void pattern_store_load_user_bank(Sequencer& seq) {
    savedMask.store(0);
    auto* payload = static_cast<Payload*>(ps_malloc(sizeof(Payload)));
    if(!payload) return;
    for(int pattern = USER_PATTERN_FIRST; pattern < MAX_PATTERNS; ++pattern) {
        bool found = false;
        uint32_t best = 0;
        for(int copy = -1; copy < 2; ++copy) {
            uint32_t generation;
            if(readCopy(pattern, copy, payload, generation) && (!found || newer(generation, best))) {
                if(restore(seq, pattern, payload)) { found = true; best = generation; }
            }
        }
        if(found) savedMask.fetch_or(1u << (pattern - USER_PATTERN_FIRST));
    }
    free(payload);
}
bool pattern_store_save_user(Sequencer& seq, int source, int destination) {
    if(source < 0 || source >= MAX_PATTERNS || !pattern_store_is_user_slot(destination)) return false;
    if(writer.test_and_set(std::memory_order_acquire)) return false;
    struct Unlock { ~Unlock() { writer.clear(std::memory_order_release); } } unlock;
    auto* payload = static_cast<Payload*>(ps_calloc(1, sizeof(Payload)));
    auto* verify = static_cast<Payload*>(ps_malloc(sizeof(Payload)));
    if(!payload || !verify) { free(payload); free(verify); return false; }
    payload->profileValid = getBuiltinPatternSoundProfile(source, payload->profile);
    if(!seq.snapshotPatternForStorage(source, &payload->pattern)) { free(payload); free(verify); return false; }
    uint32_t generation[2]{};
    const bool a = readCopy(destination, 0, verify, generation[0]);
    const bool b = readCopy(destination, 1, verify, generation[1]);
    const int newest = !a ? 1 : !b ? 0 : newer(generation[1], generation[0]) ? 1 : 0;
    const int target = !a ? 0 : !b ? 1 : 1 - newest;
    const uint32_t next = (a || b) ? generation[newest] + 1u : 1u;
    Header h{PATTERN_MAGIC, 2, uint16_t(destination), sizeof(Payload),
             PatternHash(payload, sizeof(*payload), PatternHash(&next, 4))};
    char path[40]; pathFor(destination, target, path);
    // Only the older/incomplete copy is truncated. The last verified version
    // (or legacy file during migration) survives a failed write.
    File file = SPIFFS.open(path, FILE_WRITE);
    bool ok = file && file.write(reinterpret_cast<const uint8_t*>(&h), sizeof(h)) == sizeof(h)
        && file.write(reinterpret_cast<const uint8_t*>(&next), 4) == 4
        && file.write(reinterpret_cast<const uint8_t*>(payload), sizeof(*payload)) == sizeof(*payload);
    if(file) { file.flush(); file.close(); }
    uint32_t verifiedGeneration = 0;
    ok = ok && readCopy(destination, target, verify, verifiedGeneration)
        && verifiedGeneration == next && memcmp(payload, verify, sizeof(*payload)) == 0;
    if(ok) {
        // Saving in place must not revert edits made while flash was busy.
        if(source != destination) ok = restore(seq, destination, payload);
        if(ok) savedMask.fetch_or(1u << (destination - USER_PATTERN_FIRST));
    }
    free(payload); free(verify);
    return ok;
}
