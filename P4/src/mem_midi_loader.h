// =============================================================================
// mem_midi_loader.h — Parse a .mid file from P4 internal flash (SPIFFS)
// =============================================================================
// Reads a Standard MIDI File from the P4's SPIFFS partition (mounted at "/" by
// SPIFFS.begin()), extracts drum hits (GM channel 10 with fallback to all
// channels) and folds multi-bar patterns onto a single 16-step bar so it
// matches the Master RED808 16-step protocol.
//
// Track mapping is identical to the S3-side parser and to the live P4 UI:
//   0:BD  1:SD  2:CH  3:OH  4:CP  5:CB  6:RS  7:CL
//   8:MA  9:CY  10:HT 11:LT 12:MC 13:MT 14:HC 15:LC
// =============================================================================
#pragma once
#include <cstdint>
#include <FS.h>

namespace mem_midi {

// Daisy has 20 resident sequencer patterns and a 128-entry song chain. The
// importer deduplicates equal bars, so a long song with repeated sections can
// usually fit without throwing away its arrangement.
static constexpr int MIDI_SONG_MAX_PATTERNS = 20;
static constexpr int MIDI_SONG_MAX_CHAIN = 128;

struct MidiSongPattern {
    bool steps[16][16];
    uint8_t velocity[16][16];
};

struct MidiSongChainEntry {
    uint8_t pattern;  // index in patterns[] / Daisy resident slot
    uint8_t repeats;  // consecutive repetitions, 1..255
};

struct MidiSongData {
    char name[32];
    MidiSongPattern patterns[MIDI_SONG_MAX_PATTERNS];
    MidiSongChainEntry chain[MIDI_SONG_MAX_CHAIN];
    uint16_t total_bars;
    uint16_t imported_bars;
    uint32_t hits;
    float bpm;
    uint8_t pattern_count;
    uint8_t chain_count;
    uint8_t tracks_used;
    uint8_t tempo_events;
    bool truncated;
};

// Returns true if at least one drum hit was found.
// steps[16][16] is written as a folded 1-bar grid.
// name_out receives the stem of the filename (max name_max chars).
// bpm_out  receives the first Set Tempo in BPM (0.0 if none).
// raw_len_out receives the original length before folding (16/32/48/64).
bool load_pattern(const char* path,
                  bool steps[16][16],
                  char* name_out,
                  int name_max,
                  int* steps_found_out = nullptr,
                  float* bpm_out = nullptr,
                  int* raw_len_out = nullptr);

// Same as load_pattern but preserves the full multi-bar grid (no folding).
// raw_steps[track][step] covers up to 64 steps (4 bars × 16). raw_len_out
// returns the actually used length (16/32/48/64). Caller can page through
// bars in the UI without re-parsing.
//
// mode: 0 = PRO (merge all channels, map each channel to a kit slot —
//              dense, uses every instrument as percussion).
//       1 = STD (parse ONLY GM channel 9 = drum kit — closest to what a
//              standard MIDI player like Windows plays as drums).
bool load_pattern_raw(const char* path,
                      bool raw_steps[16][64],
                      char* name_out,
                      int name_max,
                      int* steps_found_out = nullptr,
                      float* bpm_out = nullptr,
                      int* raw_len_out = nullptr,
                      int mode = 0);

        // Same parser, but reading from an arbitrary mounted filesystem such as
        // SD_MMC. This lets the P4 browse/import MIDI from its own SD card.
bool load_pattern_raw_from_fs(fs::FS& storage,
                            const char* path,
                            bool raw_steps[16][64],
                            char* name_out,
                            int name_max,
                            int* steps_found_out = nullptr,
                            float* bpm_out = nullptr,
                            int* raw_len_out = nullptr,
                            int mode = 0);

// Import a complete SMF arrangement. Notes are quantized to the nearest 16th,
// velocities are preserved, equal bars share one resident pattern and adjacent
// repetitions are run-length encoded into the Daisy song chain. The result is
// bounded by MIDI_SONG_MAX_PATTERNS/MIDI_SONG_MAX_CHAIN; `truncated` reports
// that the source exceeded those hardware limits.
bool load_song(const char* path, MidiSongData* song, int mode = 0);
bool load_song_from_fs(fs::FS& storage, const char* path,
                       MidiSongData* song, int mode = 0);

        int list_midi_files_from_fs(fs::FS& storage,
                            const char* dir,
                            char names[][48],
                            int cap);

// List .mid files in a SPIFFS directory. Writes up to `cap` names (stems only,
// no extension) truncated to 47 chars. Returns the count written (<= cap).
int list_midi_files(const char* dir, char names[][48], int cap);

}  // namespace mem_midi
