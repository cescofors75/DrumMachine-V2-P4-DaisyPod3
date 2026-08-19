#include "midi_map_store.h"

#include <Preferences.h>
#include <cstring>

namespace {
constexpr uint32_t MIDI_STORE_MAGIC = 0x3149444Du; // "MDI1"
constexpr uint16_t MIDI_STORE_VERSION = 1;

struct MidiStoreBlob {
    uint32_t magic;
    uint16_t version;
    uint8_t count;
    uint8_t reserved;
    MidiMapEntry entries[MIDI_MAP_MAX_ENTRIES];
    uint32_t checksum;
};

uint32_t checksum(const MidiMapEntry* entries, uint8_t count)
{
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(entries);
    uint32_t hash = 2166136261u;
    hash ^= count;
    hash *= 16777619u;
    for(size_t i = 0; i < count * sizeof(MidiMapEntry); ++i)
    {
        hash ^= bytes[i];
        hash *= 16777619u;
    }
    return hash;
}

bool entryValid(const MidiMapEntry& entry)
{
    return entry.channel <= 15u && entry.number <= 127u
        && (entry.kind == MIDI_MAP_KIND_NOTE || entry.kind == MIDI_MAP_KIND_CC);
}
}

bool midi_map_store_load(MidiMapEntry* entries, uint8_t& count)
{
    count = 0;
    Preferences prefs;
    if(!prefs.begin("drumv2midi", true)) return false;
    MidiStoreBlob blob{};
    const bool ok = prefs.getBytesLength("map") == sizeof(blob)
                 && prefs.getBytes("map", &blob, sizeof(blob)) == sizeof(blob)
                 && blob.magic == MIDI_STORE_MAGIC
                 && blob.version == MIDI_STORE_VERSION
                 && blob.count <= MIDI_MAP_MAX_ENTRIES
                 && blob.checksum == checksum(blob.entries, blob.count);
    prefs.end();
    if(!ok) return false;
    for(uint8_t i = 0; i < blob.count; ++i)
        if(entryValid(blob.entries[i]))
            entries[count++] = blob.entries[i];
    return true;
}

bool midi_map_store_save(const MidiMapEntry* entries, uint8_t count)
{
    if(count > MIDI_MAP_MAX_ENTRIES) count = MIDI_MAP_MAX_ENTRIES;
    MidiStoreBlob blob{};
    blob.magic = MIDI_STORE_MAGIC;
    blob.version = MIDI_STORE_VERSION;
    blob.count = count;
    if(count > 0)
        memcpy(blob.entries, entries, count * sizeof(MidiMapEntry));
    blob.checksum = checksum(blob.entries, blob.count);

    Preferences prefs;
    if(!prefs.begin("drumv2midi", false)) return false;
    const bool ok = prefs.putBytes("map", &blob, sizeof(blob)) == sizeof(blob);
    prefs.end();
    return ok;
}
