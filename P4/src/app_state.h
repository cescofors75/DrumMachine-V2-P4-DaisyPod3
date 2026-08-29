#pragma once

#include <atomic>
#include <stdint.h>

void p4_publish_pending_melody(uint8_t engine, uint8_t octave,
                               bool recording, uint8_t pad);
bool p4_consume_pending_melody(uint8_t* engine, uint8_t* octave,
                               bool* recording, uint8_t* pad);

struct P4State
{
    int bpm_int;
    int bpm_frac;
    int original_bpm_x10;     // source pattern/song tempo, 0 when undefined
    int current_pattern;
    int current_step;
    bool is_playing;
    bool master_connected;    // true after Daisy responds over USB-C
    int theme;
    int master_volume;
    int seq_volume;
    int live_volume;
    bool screensaver_enabled;   // preference, persisted — see settings_store.cpp

    uint8_t enc_value[3];
    bool enc_muted[3];
    uint8_t pot_value[4];
    bool pot_muted[3];

    int filter_type;
    int cutoff_hz;
    int resonance_x10;
    int distortion_pct;
    int bitcrush_bits;
    int sample_rate_hz;
    int fx_resp_mode;

    bool track_muted[16];
    bool track_solo[16];
    int track_volume[16];

    char kit_name[32];
    bool sample_loaded[24];
    char sample_name[24][32];
    bool steps[16][16];
    uint8_t pad_velocity[16];
    unsigned long pad_flash_until[16];
    int current_screen;
    unsigned long last_heartbeat_ms;
};

extern P4State p4;

// Results captured from the real setup() calls and shown by the boot POST.
// These are deliberately separate from P4State: they describe whether each
// local subsystem actually initialized, not the current musical state.
struct P4BootState
{
    bool display_ready;
    bool lvgl_ready;
    bool spiffs_mounted;
    bool ui_ready;
    bool usb_host_ready;
    bool patterns_ready;
    bool dsp_ready;
    bool setup_complete;
    uint8_t factory_patterns_found;
    uint8_t factory_patterns_expected;
    uint32_t psram_total_bytes;
    uint32_t psram_free_bytes;
    uint32_t spiffs_used_bytes;
    uint32_t spiffs_total_bytes;
};

extern P4BootState p4boot;

#define P4_SD_MAX_ENTRIES 64

struct P4SdEntry
{
    char name[48];
    bool is_dir;
    bool is_midi;
};

struct P4SdState
{
    bool mounted;
    char path[128];
    char selected_file[64];
    int selected_pad;
    bool selected_is_midi;
    int8_t midi_load_result;
    P4SdEntry entries[P4_SD_MAX_ENTRIES];
    int entry_count;
    bool list_complete;
    std::atomic<bool> needs_refresh;
};

extern P4SdState p4sd;
