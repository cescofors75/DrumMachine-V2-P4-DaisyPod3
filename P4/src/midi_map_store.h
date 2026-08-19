#pragma once

#include "master/protocol.h"

// Persisted user MIDI map (LEARN assignments for the AKAI MPD218).
// Stored in NVS; P4 is the single owner and re-uploads the map to
// DaisyPod3 (CMD_MIDI_MAP_SET) on every reconnection.
bool midi_map_store_load(MidiMapEntry* entries, uint8_t& count);
bool midi_map_store_save(const MidiMapEntry* entries, uint8_t count);
