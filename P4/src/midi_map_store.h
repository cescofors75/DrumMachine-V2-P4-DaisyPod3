#pragma once

#include "master/protocol.h"

// Persisted user MIDI map (LEARN assignments for the AKAI MPD218).
// Stored in NVS; P4 is the single owner and re-uploads the map to
// DaisyPod3 (CMD_MIDI_MAP_SET) on every reconnection.
bool midi_map_store_load(MidiMapEntry* entries, uint8_t& count);
bool midi_map_store_save(const MidiMapEntry* entries, uint8_t count);

// Backup/restore to the P4 SD card (/midi_map_backup.mmap), independent of
// NVS — survives a factory reset or moving the map to another P4 unit.
// Caller is responsible for confirming the SD card is mounted first.
bool midi_map_store_export_sd(const MidiMapEntry* entries, uint8_t count);
bool midi_map_store_import_sd(MidiMapEntry* entries, uint8_t& count);
