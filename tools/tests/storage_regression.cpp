#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cstdio>
#include <cstdint>
#define SEQUENCER_H
#define MAX_PATTERNS 128
#define MAX_TRACKS 16
// The store treats pattern data opaquely; a small stand-in lets us inject
// failures at every byte boundary without FreeRTOS or a physical flash chip.
struct PatternStorageData { uint8_t data[128]; };
class Sequencer {
public:
    PatternStorageData patterns[MAX_PATTERNS]{};
    bool snapshotPatternForStorage(int p, PatternStorageData* out) { *out=patterns[p]; return true; }
    bool restorePatternFromStorage(int p, const PatternStorageData* in) { patterns[p]=*in; return true; }
};
#include "../../P4/src/pattern_store.cpp"
static BuiltinPatternSoundProfile profiles[MAX_PATTERNS]{};
static bool valid[MAX_PATTERNS]{};
bool getBuiltinPatternSoundProfile(int p, BuiltinPatternSoundProfile& out) { out=profiles[p]; return valid[p]; }
void setPatternSoundProfile(int p, const BuiltinPatternSoundProfile& in) { profiles[p]=in; valid[p]=true; }
void clearPatternSoundProfile(int p) { valid[p]=false; }
int main() {
    Sequencer seq;
    const int bytes = sizeof(Header) + 4 + sizeof(Payload);
    for(int cutoff = 0; cutoff < bytes; ++cutoff) {
        SPIFFS.files.clear(); writeBudget=-1;
        seq.patterns[0].data[0]=17;
        assert(pattern_store_save_user(seq,0,100));
        seq.patterns[0].data[0]=29;
        assert(pattern_store_save_user(seq,0,100));
        seq.patterns[0].data[0]=91;
        writeBudget=cutoff;
        assert(!pattern_store_save_user(seq,0,100));
        assert(seq.patterns[100].data[0]==29); // failed save must not mutate RAM
        seq.patterns[100].data[0]=0;
        pattern_store_load_user_bank(seq);
        assert(seq.patterns[100].data[0]==29); // reboot recovers last complete copy
        assert(pattern_store_is_saved(100));
    }
    writeBudget=-1;
    failOpenWrite=true;
    assert(!pattern_store_save_user(seq,0,100));
    failOpenWrite=false;
    assert(pattern_store_save_user(seq,0,100));
    pattern_store_load_user_bank(seq);
    assert(seq.patterns[100].data[0]==91);
    // A corrupted latest generation falls back to the other verified copy.
    SPIFFS.files["/user_p101_a.dmv2"]->back() ^= 1;
    pattern_store_load_user_bank(seq);
    assert(seq.patterns[100].data[0]==29);
    // Existing V1 user files migrate without deleting the only good copy.
    SPIFFS.files.clear();
    Payload legacy{};
    legacy.pattern.data[0]=63;
    Header h{PATTERN_MAGIC,1,100,sizeof(Payload),PatternHash(&legacy,sizeof(legacy))};
    File file=SPIFFS.open("/user_p101.dmv2",FILE_WRITE);
    file.write(reinterpret_cast<const uint8_t*>(&h),sizeof(h));
    file.write(reinterpret_cast<const uint8_t*>(&legacy),sizeof(legacy));
    pattern_store_load_user_bank(seq);
    assert(seq.patterns[100].data[0]==63);
    assert(pattern_store_save_user(seq,0,100));
    assert(SPIFFS.files.count("/user_p101.dmv2"));
    pattern_store_load_user_bank(seq);
    assert(seq.patterns[100].data[0]==91);
    std::printf("Storage: %d interrupted writes, failed open, reboot recovery PASS\n", bytes);
    std::puts("Storage: corrupt latest copy, legacy V1 migration PASS");
}
