// =============================================================================
// ui_screens.cpp — P4 UI screens (local master state + Daisy USB telemetry)
// All screen rendering reads from P4State (p4.*) — no direct hardware access.
// =============================================================================

#include "ui_screens.h"
#include "ui_theme.h"
#include "../drivers/lvgl_port.h"
#include "../drivers/i2c_rotaries.h"
#include "../control_api.h"
#include "../daisy_usb_transport.h"
#include "../app_state.h"
#include "../../include/ui_events.h"
#include "../dsp_task.h"
#include "../mem_midi_loader.h"
#include "config.h"
#include "../../../shared/synth_params.h"
#include "../../../DaisyPod3/mpd218_mapping.h"
#include <Arduino.h>
#include <SD_MMC.h>
#include <SPIFFS.h>
#include "../pod_config_store.h"
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <atomic>
#include <math.h>

// ── IntelliSense fallbacks ───────────────────────────────────────────────────
// The real values are provided via -D flags in platformio.ini and via
// config.h / lv_conf.h. These fallbacks only kick in for editor analysis
// when IntelliSense can't resolve the build system's include/define graph.
#ifndef LCD_H_RES
#define LCD_H_RES 1024
#endif
#ifndef LCD_V_RES
#define LCD_V_RES 600
#endif
#ifndef UI_H
#define UI_H LCD_V_RES
#endif
#if defined(__INTELLISENSE__)
LV_FONT_DECLARE(lv_font_montserrat_10)
#endif

// ── Pad event queue: touch_task (Core 0) → loop (Core 1) ──
// Each entry packs (velocity << 8) | pad to carry MPC-style velocity all the
// way to the local controller without adding a parallel array. Decouples send from
// the LVGL mutex.
static volatile uint16_t s_pad_q[32];
static std::atomic<uint8_t> s_pad_qh{0};
static std::atomic<uint8_t> s_pad_qt{0};
static std::atomic<uint32_t> s_pad_q_drops{0};

static std::atomic<uint16_t> s_ctrl_mute_dirty{0};
static std::atomic<uint16_t> s_ctrl_mute_values{0};
static std::atomic<bool>     s_ctrl_mute_mask_pending{false};
static std::atomic<uint16_t> s_ctrl_mute_mask{0};
static std::atomic<bool>     s_ctrl_solo_mask_pending{false};
static std::atomic<uint16_t> s_ctrl_solo_mask{0};
static std::atomic<bool>     s_ctrl_pattern_sync_pending{false};
static std::atomic<int8_t>   s_ctrl_pattern_step_pending{0};

// ── UI work requested FROM the control task (Core 1) ──
// LVGL is not thread-safe and only the LVGL task (Core 0) may touch
// widgets. control_api.cpp used to call ui_sequencer_sync_from_current_
// pattern() / fx_random_apply() / mix_random_apply() / ui_sequencer_
// refresh_all_step_dots() directly from control_process() — racing
// lv_timer_handler() on the other core, which corrupts LVGL's internal
// state (escalating visual glitches until the UI task dies while audio
// keeps playing). Control code now only flips these flags; the actual
// widget work runs in ui_update_current_screen() on the LVGL task.
static std::atomic<bool> s_ui_seq_resync_pending{false};
static std::atomic<bool> s_ui_step_dots_pending{false};
static std::atomic<bool> s_ui_fx_random_tick_pending{false};
static std::atomic<bool> s_ui_mix_random_tick_pending{false};
static std::atomic<bool> s_ui_matrix_tick_pending{false};
static std::atomic<uint8_t> s_ui_matrix_tick_idx{0};

void ui_request_sequencer_resync(void) {
    s_ui_seq_resync_pending.store(true, std::memory_order_release);
}
void ui_request_step_dots_refresh(void) {
    s_ui_step_dots_pending.store(true, std::memory_order_release);
}
void ui_request_fx_random_tick(void) {
    s_ui_fx_random_tick_pending.store(true, std::memory_order_release);
}
void ui_request_mix_random_tick(void) {
    s_ui_mix_random_tick_pending.store(true, std::memory_order_release);
}
void ui_request_matrix_tick(uint8_t idx) {
    s_ui_matrix_tick_idx.store(idx, std::memory_order_relaxed);
    s_ui_matrix_tick_pending.store(true, std::memory_order_release);
}

// Defined near the end of this file (after the filter/mixer/melody preset
// systems it recalls into); forward-declared here so both the SONG modal's
// MATRIX launcher button (early in the file) and the bar-clock tick
// consumer above can reference them.
static void matrix_apply_column(uint8_t idx);
static void matrix_modal_show(lv_event_t* e);

// RANDOM SONG's musical-jump reason (see triggerRandomSongJump in
// control_api.cpp) — single producer (the control task), so a plain
// buffer guarded by release/acquire on the pending flag is enough: the
// buffer write happens-before the flag's release store, and the LVGL
// task's acquire load on exchange() happens-before its read of the buffer.
static char s_ui_random_song_toast_msg[64];
static std::atomic<bool> s_ui_random_song_toast_pending{false};

void ui_request_random_song_toast(const char* msg) {
    strncpy(s_ui_random_song_toast_msg, msg, sizeof(s_ui_random_song_toast_msg) - 1);
    s_ui_random_song_toast_msg[sizeof(s_ui_random_song_toast_msg) - 1] = '\0';
    s_ui_random_song_toast_pending.store(true, std::memory_order_release);
}

// Same single-producer pattern as the RANDOM SONG toast above, for AUTO
// VARIATIONS' own toast.
static char s_ui_variation_toast_msg[64];
static std::atomic<bool> s_ui_variation_toast_pending{false};

void ui_request_variation_toast(const char* msg) {
    strncpy(s_ui_variation_toast_msg, msg, sizeof(s_ui_variation_toast_msg) - 1);
    s_ui_variation_toast_msg[sizeof(s_ui_variation_toast_msg) - 1] = '\0';
    s_ui_variation_toast_pending.store(true, std::memory_order_release);
}

// Touch debounce tuned for GT911 + multi-indev setup.
static const uint32_t MUTE_DEBOUNCE_TRACK_MS = 180;
static const uint32_t MUTE_DEBOUNCE_GLOBAL_MS = 60;
static const uint32_t SOLO_DEBOUNCE_TRACK_MS = 220;
static const uint32_t SOLO_DEBOUNCE_GLOBAL_MS = 70;

// Direct touch bypass: flag used by touch_task to early-out when not on LIVE
static std::atomic<bool> g_live_screen_active{false};

// Bumped by ui_reload_themed_screens(). Update functions keep function-local
// dirty caches (prev_* statics) that survive widget recreation; they compare
// against this generation and force a full repaint after a theme reload.
static uint32_t s_ui_refresh_gen = 1;

// LIVE pad hit-rects mirrored from the LVGL layout. touch_task (Core 0,
// prio 6) hit-tests against this cache instead of calling lv_obj_get_x/
// get_width: those APIs can mutate layout state concurrently with the
// render task, and the pad objects themselves die during theme reload.
// Written only from the LVGL task (create_live_screen / apply_pad_layout).
struct PadHitRect { int16_t x, y, w, h; bool visible; };
static PadHitRect s_pad_hit[16];
static portMUX_TYPE s_pad_hit_mux = portMUX_INITIALIZER_UNLOCKED;

static void pad_hit_store(int i, int x, int y, int w, int h, bool visible) {
    if (i < 0 || i >= 16) return;
    portENTER_CRITICAL(&s_pad_hit_mux);
    s_pad_hit[i].x = (int16_t)x;
    s_pad_hit[i].y = (int16_t)y;
    s_pad_hit[i].w = (int16_t)w;
    s_pad_hit[i].h = (int16_t)h;
    s_pad_hit[i].visible = visible;
    portEXIT_CRITICAL(&s_pad_hit_mux);
}

static inline void enqueue_pad_event(uint8_t pad, uint8_t velocity) {
    uint8_t h = s_pad_qh.load(std::memory_order_relaxed);
    uint8_t t = s_pad_qt.load(std::memory_order_acquire);
    if ((uint8_t)(h - t) >= 32) {
        s_pad_q_drops.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    s_pad_q[h & 0x1F] = (uint16_t)((velocity << 8) | pad);
    s_pad_qh.store(h + 1, std::memory_order_release);
}

static inline void enqueue_mute_control(uint8_t track, bool muted) {
    if (track >= 16) return;
    uint16_t bit = (uint16_t)(1U << track);
    uint16_t values = s_ctrl_mute_values.load(std::memory_order_relaxed);
    values = muted ? (uint16_t)(values | bit) : (uint16_t)(values & ~bit);
    s_ctrl_mute_values.store(values, std::memory_order_release);
    s_ctrl_mute_dirty.fetch_or(bit, std::memory_order_release);
}

static inline void enqueue_mute_mask_control(uint16_t mask) {
    s_ctrl_mute_dirty.store(0, std::memory_order_release);
    s_ctrl_mute_mask.store(mask, std::memory_order_release);
    s_ctrl_mute_mask_pending.store(true, std::memory_order_release);
}

static inline void enqueue_solo_mask_control(uint16_t mask) {
    s_ctrl_solo_mask.store(mask, std::memory_order_release);
    s_ctrl_solo_mask_pending.store(true, std::memory_order_release);
}

// =============================================================================
// MPC-STYLE PLAYBACK STATE — note repeat, 16 levels, velocity fade
// =============================================================================
// Note repeat: subdivisions per beat = {1, 2, 4, 8, 3, 6} for
// 1/4, 1/8, 1/16, 1/32, 1/8T, 1/16T respectively. Index 0 = OFF.
static const uint8_t     NR_SUBDIV_PER_BEAT[7] = {0, 1, 2, 4, 8, 3, 6};
static const char* const NR_LABEL[7] = {"NR\nOFF", "NR\n1/4", "NR\n1/8",
                                         "NR\n1/16", "NR\n1/32",
                                         "NR\n1/8T", "NR\n1/16T"};
static volatile uint8_t s_nr_idx = 0;                  // 0 = OFF

// 16 Levels: all 16 pads become 16 velocities of a single source pad
static volatile bool    s_16l_active   = false;
static volatile uint8_t s_16l_src_pad  = 0;            // last non-16L tap

// Per-pad state, written by touch_task (Core 0) and read by update_live_screen
// (Core 0 LVGL task). Single writer / single reader per field → no locks.
static volatile bool          s_pad_held[16] = {};
static volatile unsigned long s_pad_repeat_next_ms[16] = {};
static volatile uint8_t       s_pad_held_velocity[16] = {};
static volatile uint8_t       s_pad_hold_x[16] = {};
static volatile uint8_t       s_pad_hold_y[16] = {};
static volatile unsigned long s_pad_hold_start_ms[16] = {};
static volatile uint8_t       s_pad_roll_phase[16] = {};

static const unsigned long PAD_TREMOLO_HOLD_MS = 165;
static const unsigned long PAD_TREMOLO_FAST_MS = 42;
static const unsigned long PAD_TREMOLO_SLOW_MS = 235;

// Velocity fade visualisation: each press stores (velocity, start_ms) and
// update_live_screen interpolates an exponential decay over FADE_MS to drive
// the pad background opacity. Quantised into 8 brightness bands so LVGL only
// re-invalidates a pad rect when the band actually changes (keeps partial
// refresh cheap even with 16 pads decaying at once).
static const int FADE_MS = 320;
static volatile uint8_t       s_pad_flash_vel[16] = {};
static volatile unsigned long s_pad_flash_start_ms[16] = {};

static inline void ui_pad_flash_start(uint8_t pad, uint8_t velocity) {
    if (pad >= 16) return;
    s_pad_flash_vel[pad]      = velocity ? velocity : 1;
    s_pad_flash_start_ms[pad] = millis();
}

// Called from control_process() (loop task) when a MIDI event resolved to a
// pad: mirrors the LIVE pad flash used by touch/sequencer hits. The volatile
// latch arrays make this safe across tasks; the LVGL task paints the fade.
void ui_external_pad_flash(uint8_t pad, uint8_t velocity) {
    if (pad >= 16) return;
    ui_pad_flash_start(pad, velocity);
    p4.pad_flash_until[pad] = millis() + 120;
}

static inline uint8_t ui_live_pad_velocity(void);

static unsigned long ui_nr_interval_ms(void) {
    // Use current tempo from P4State. BPM can be 0 briefly at
    // boot; clamp to 40..300 for safety.
    extern struct P4State p4;
    int bpm_x10 = p4.bpm_int * 10 + p4.bpm_frac;
    if (bpm_x10 < 400)  bpm_x10 = 1200;
    if (bpm_x10 > 3000) bpm_x10 = 3000;
    uint8_t idx = s_nr_idx;
    if (idx == 0 || idx >= (sizeof(NR_SUBDIV_PER_BEAT) / sizeof(NR_SUBDIV_PER_BEAT[0]))) return 0;
    uint32_t div = NR_SUBDIV_PER_BEAT[idx];
    // interval = 60000 ms / (bpm * div). bpm_x10 is BPM*10, so:
    //   ms = 600000 / (bpm_x10 * div)
    unsigned long ms = 600000UL / ((unsigned long)bpm_x10 * div);
    if (ms < 15) ms = 15;   // safety floor (~66 Hz max retrigger)
    return ms;
}

static unsigned long ui_pad_tremolo_interval_ms(uint8_t pad, unsigned long nr_interval) {
    if (nr_interval) return nr_interval;
    if (pad >= 16) return 0;
    uint8_t x = s_pad_hold_x[pad];
    if (x > 127) x = 127;
    return PAD_TREMOLO_SLOW_MS - (((PAD_TREMOLO_SLOW_MS - PAD_TREMOLO_FAST_MS) * (unsigned long)x) / 127UL);
}

static uint8_t ui_pad_tremolo_velocity(uint8_t pad, unsigned long now_ms) {
    if (pad >= 16) return 100;
    uint8_t y = s_pad_hold_y[pad];
    if (y > 127) y = 127;
    uint16_t base = ui_live_pad_velocity();
    uint16_t amp = 34 + (((uint16_t)(127 - y) * 93U) / 127U);
    uint16_t vel = (base * amp) / 127U;

    unsigned long held_ms = now_ms - s_pad_hold_start_ms[pad];
    if (held_ms < 420UL) {
        vel = (vel * (64U + (uint16_t)((held_ms * 63UL) / 420UL))) / 127U;
    }

    static const int8_t wobble[8] = {0, 5, 9, 5, 0, -4, -7, -4};
    uint8_t phase = s_pad_roll_phase[pad];
    uint8_t depth = 2 + (uint8_t)(((uint16_t)(127 - y) * 10U) / 127U);
    int16_t shaped = (int16_t)vel + (int16_t)((wobble[phase & 0x07] * (int8_t)depth) / 4);
    if (shaped < 8) shaped = 8;
    if (shaped > 127) shaped = 127;
    return (uint8_t)shaped;
}


// Screen objects
lv_obj_t* scr_boot = NULL;
lv_obj_t* scr_live = NULL;
lv_obj_t* scr_sequencer = NULL;
lv_obj_t* scr_fx = NULL;
lv_obj_t* scr_volumes = NULL;
lv_obj_t* scr_sdcard = NULL;
lv_obj_t* scr_performance = NULL;
lv_obj_t* scr_piano = NULL;       /* v2.6 — PIANO live keyboard */
lv_obj_t* scr_piano_params = NULL; /* v2.7 — synth engine parameter editor */
lv_obj_t* scr_fx_xy = NULL;       /* v3.2 — FX XY performance pad */
static lv_obj_t* scr_screensaver = NULL; /* local status screensaver */

// Header widgets
static lv_obj_t* header_bar = NULL;
static lv_obj_t* s_live_midi_badge = NULL;
static lv_obj_t* s_seq_midi_badge = NULL;
static lv_obj_t* hdr_bpm_label = NULL;
static lv_obj_t* hdr_pattern_label = NULL;
static lv_obj_t* hdr_play_btn = NULL;
static lv_obj_t* hdr_play_label = NULL;
static lv_obj_t* hdr_pattern_minus_btn = NULL;
static lv_obj_t* hdr_pattern_plus_btn = NULL;
static lv_obj_t* hdr_step_dots[16] = {};

// Current active screen index + history for back navigation
static int active_screen = 0;
static int prev_active_screen = 0;

// Canonical engine/sample order shared by P4 and DaisyPod3.
static const char* trackNames[] = {
    "BD", "SD", "CH", "OH", "CY", "CP", "RS", "CB",
    "LT", "MT", "HT", "MA", "CL", "HC", "MC", "LC"
};

static int ui_layout_w(void) {
    return LCD_H_RES;
}

static int ui_layout_h(void) {
    // The current panel path renders 1024x600 without framebuffer rotation.
    // Keep the live layout inside the visible 600px height.
    return (UI_H > LCD_V_RES) ? LCD_V_RES : UI_H;
}

static inline lv_color_t ui_track_color(int track) {
    return lv_color_hex(theme_presets[ui_theme_index()].track_colors[track & 0x0F]);
}

static inline bool ui_track_color_is_light(int track) {
    uint32_t c = theme_presets[ui_theme_index()].track_colors[track & 0x0F];
    uint8_t r = (uint8_t)((c >> 16) & 0xFF);
    uint8_t g = (uint8_t)((c >> 8) & 0xFF);
    uint8_t b = (uint8_t)(c & 0xFF);
    return ((uint16_t)r + (uint16_t)g + (uint16_t)b) > 560;
}

static inline lv_color_t ui_track_label_color(int track, bool lit) {
    return (lit && ui_track_color_is_light(track)) ? RED808_BG : ui_track_color(track);
}

static inline uint32_t ui_tremolo_neon_hex(uint8_t x, uint8_t y) {
    uint8_t amp = (uint8_t)(127U - (y > 127 ? 127 : y));
    if (x < 32)  return (amp > 84) ? 0xFF3324 : 0xC9271B;
    if (x < 64)  return (amp > 72) ? 0xFF6A2A : 0xE86820;
    if (x < 96)  return (amp > 60) ? 0xFFD052 : 0xF5BC31;
    return (amp > 48) ? 0xFFF7E8 : 0xF7EAD7;
}

// =============================================================================
// HELPER: Section shell (styled container)
// =============================================================================
lv_obj_t* create_section_shell(lv_obj_t* parent, int x, int y, int w, int h) {
    lv_obj_t* obj = lv_obj_create(parent);
    lv_obj_set_pos(obj, x, y);
    lv_obj_set_size(obj, w, h);
    lv_obj_set_style_bg_color(obj, RED808_PANEL, 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_80, 0);
    lv_obj_set_style_border_width(obj, 1, 0);
    lv_obj_set_style_border_color(obj, RED808_BORDER, 0);
    lv_obj_set_style_radius(obj, 8, 0);
    lv_obj_set_style_pad_all(obj, 14, 0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    return obj;
}

static bool ui_control_available(void) {
    return control_available();
}

static bool ui_master_link_display_on(void) {
    static bool shown = false;
    static bool pending = false;
    static uint32_t pending_since = 0;

    bool raw = p4.master_connected;
    uint32_t now = millis();
    uint32_t settle_ms = raw ? 350UL : 2200UL;

    if (raw == shown) {
        pending = raw;
        pending_since = now;
        return shown;
    }

    if (raw != pending) {
        pending = raw;
        pending_since = now;
        return shown;
    }

    if ((uint32_t)(now - pending_since) >= settle_ms) shown = raw;
    return shown;
}

static void apply_control_button_style(lv_obj_t* button, lv_color_t accent,
                                       bool filled, int radius) {
    if (!button) return;
    lv_obj_set_style_radius(button, radius > 8 ? 8 : radius, 0);
    lv_obj_set_style_bg_color(button, filled ? accent : RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_color(button, filled ? RED808_SURFACE : RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_dir(button, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(button, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(button, 2, 0);
    lv_obj_set_style_border_color(button, accent, 0);
    lv_obj_set_style_border_opa(button, filled ? LV_OPA_COVER : LV_OPA_70, 0);
    lv_obj_set_style_shadow_width(button, 0, 0);
    lv_obj_set_style_bg_color(button, accent, LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(button, (lv_opa_t)216, LV_STATE_PRESSED);
    lv_obj_clear_flag(button, LV_OBJ_FLAG_SCROLLABLE);
}

static lv_obj_t* create_header_button(lv_obj_t* parent, int x, int y, int w, int h,
                                      const char* text, lv_color_t bg, lv_color_t border) {
    lv_obj_t* btn = lv_btn_create(parent);
    lv_obj_set_size(btn, w, h);
    lv_obj_set_pos(btn, x, y);
    apply_control_button_style(btn, border, true, 12);
    lv_obj_set_style_bg_color(btn, bg, 0);

    lv_obj_t* label = lv_label_create(btn);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(label, RED808_TEXT, 0);
    lv_obj_center(label);
    return btn;
}

static lv_obj_t* s_ui_toast = NULL;
static lv_obj_t* s_ui_toast_label = NULL;
static uint32_t s_ui_toast_until_ms = 0;

static void ui_toast_hide(void) {
    if (s_ui_toast) lv_obj_add_flag(s_ui_toast, LV_OBJ_FLAG_HIDDEN);
    s_ui_toast_until_ms = 0;
}

static void ui_toast_update(void) {
    if (s_ui_toast && s_ui_toast_until_ms &&
        (int32_t)(millis() - s_ui_toast_until_ms) >= 0) {
        ui_toast_hide();
    }
}

static void ui_show_toast(const char* text, lv_color_t accent) {
    lv_obj_t* parent = lv_scr_act();
    if (!parent) return;

    // Re-parent by recreation: delete the old toast first or it stays
    // visible forever on the previous screen (and leaks).
    if (s_ui_toast && lv_obj_get_parent(s_ui_toast) != parent) {
        lv_obj_del(s_ui_toast);
        s_ui_toast = NULL;
        s_ui_toast_label = NULL;
    }
    if (!s_ui_toast) {
        s_ui_toast = lv_obj_create(parent);
        lv_obj_set_size(s_ui_toast, 420, 62);
        lv_obj_align(s_ui_toast, LV_ALIGN_TOP_MID, 0, 54);
        lv_obj_set_style_radius(s_ui_toast, 8, 0);
        lv_obj_set_style_bg_color(s_ui_toast, RED808_PANEL, 0);
        lv_obj_set_style_bg_opa(s_ui_toast, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(s_ui_toast, 2, 0);
        lv_obj_set_style_shadow_width(s_ui_toast, 18, 0);
        lv_obj_set_style_shadow_color(s_ui_toast, lv_color_hex(0x000000), 0);
        lv_obj_set_style_pad_hor(s_ui_toast, 18, 0);
        lv_obj_set_style_pad_ver(s_ui_toast, 12, 0);
        lv_obj_clear_flag(s_ui_toast, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(s_ui_toast, LV_OBJ_FLAG_CLICKABLE);

        s_ui_toast_label = lv_label_create(s_ui_toast);
        lv_obj_set_width(s_ui_toast_label, 384);
        lv_label_set_long_mode(s_ui_toast_label, LV_LABEL_LONG_DOT);
        lv_obj_set_style_text_font(s_ui_toast_label, &lv_font_montserrat_18, 0);
        lv_obj_center(s_ui_toast_label);
    }

    lv_label_set_text(s_ui_toast_label, text);
    lv_obj_set_style_border_color(s_ui_toast, accent, 0);
    lv_obj_set_style_text_color(s_ui_toast_label, RED808_TEXT, 0);
    lv_obj_move_foreground(s_ui_toast);
    lv_obj_clear_flag(s_ui_toast, LV_OBJ_FLAG_HIDDEN);
    s_ui_toast_until_ms = millis() + 1800U;
}

static void seq_pattern_modal_show(int pattern);
static void seq_pattern_modal_hide(void);
static void seq_pattern_modal_mark_loaded(void);
static void seq_launch_absolute_pattern(int pattern);
static int  seq_queued_pattern = -1;
static bool seq_quantize_enabled = true;

static void step_pattern_relative(int delta) {
    if (delta == 0) return;
    s_ctrl_pattern_step_pending.fetch_add(delta > 0 ? 1 : -1,
                                          std::memory_order_release);
}

static void header_play_cb(lv_event_t* e) {
    LV_UNUSED(e);
    // Debounce — LVGL can double-fire on a sloppy tap, causing visible
    // start/stop/start flicker and duplicate USB commands.
    static uint32_t last_ms = 0;
    uint32_t now = millis();
    if (now - last_ms < 250) return;
    last_ms = now;

    bool next_play = !p4.is_playing;
    // P4 owns its play/pause toggle — send once to DaisyPod3.
    if (next_play) control_send_start();
    else           control_send_stop();
    // P4 owns step clock: reset phase explicitly on every transport toggle.
    local_apply_message(MSG_SYSTEM, SYS_STEP, 0);
    p4.current_step = 0;
    p4.is_playing = next_play;
}

static void header_pattern_cb(lv_event_t* e) {
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    step_pattern_relative(delta);
}

// =============================================================================
// BACK BUTTON — replaces the old header bar (floating top-left corner)
// =============================================================================
lv_obj_t* ui_create_header(lv_obj_t* parent) {
    // Nullify all header widget pointers — not used anymore
    header_bar = NULL;
    hdr_bpm_label = NULL; hdr_pattern_label = NULL;
    hdr_play_btn = NULL; hdr_play_label = NULL;
    hdr_pattern_minus_btn = NULL; hdr_pattern_plus_btn = NULL;
    for (int i = 0; i < 16; i++) hdr_step_dots[i] = NULL;

    // Small floating back button (top-left) — ≥40px tall for reliable live taps
    lv_obj_t* back_btn = lv_btn_create(parent);
    lv_obj_set_size(back_btn, 48, 42);
    lv_obj_set_pos(back_btn, 8, 8);
    apply_control_button_style(back_btn, RED808_BORDER, false, 8);
    lv_obj_add_event_cb(back_btn, [](lv_event_t* e) {
        LV_UNUSED(e);
        if (active_screen != 2) ui_navigate_to(2);
    }, LV_EVENT_CLICKED, NULL);
    lv_obj_t* back_lbl = lv_label_create(back_btn);
    lv_label_set_text(back_lbl, LV_SYMBOL_LEFT);
    lv_obj_set_style_text_font(back_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(back_lbl, RED808_TEXT, 0);
    lv_obj_center(back_lbl);
    return back_btn;
}

// MIDI (MPD218) activity badge — a small notification-style dot. MIDI DIN
// has no plug-detect line, so "connected" isn't knowable; this reflects
// recent traffic instead: bright green while a note/CC arrived in the last
// few seconds, dim amber if one has arrived this session but gone quiet,
// hidden if none has ever been seen. Placed individually on LIVE and the
// sequencer (not inside the shared ui_create_header(), which is re-run per
// screen and would leave every screen but the last-created one tracking a
// stale pointer).
static lv_obj_t* ui_create_midi_badge(lv_obj_t* parent, int x, int y) {
    lv_obj_t* badge = lv_obj_create(parent);
    lv_obj_set_size(badge, 14, 14);
    lv_obj_set_pos(badge, x, y);
    lv_obj_set_style_radius(badge, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(badge, RED808_TEXT_DIM, 0);
    lv_obj_set_style_bg_opa(badge, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(badge, 2, 0);
    lv_obj_set_style_border_color(badge, RED808_PANEL, 0);
    lv_obj_clear_flag(badge, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(badge, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(badge, LV_OBJ_FLAG_HIDDEN);
    return badge;
}

// Small "kill this AUTO mode" corner badge for a header/random button —
// lets the user turn off RANDOM SONG / AUTO FX / AUTO MIX with one tap
// instead of opening its modal just to reach the AUTO toggle. Overlays the
// button's own top-right corner so it costs no extra header width. Caller
// owns showing/hiding it (via LV_OBJ_FLAG_HIDDEN) to reflect active state.
static lv_obj_t* ui_create_auto_stop_badge(lv_obj_t* parent, lv_event_cb_t cb) {
    lv_obj_t* badge = lv_btn_create(parent);
    lv_obj_set_size(badge, 15, 15);
    lv_obj_align(badge, LV_ALIGN_TOP_RIGHT, 2, -4);
    lv_obj_set_style_radius(badge, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(badge, RED808_ERROR, 0);
    lv_obj_set_style_bg_opa(badge, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(badge, 1, 0);
    lv_obj_set_style_border_color(badge, lv_color_white(), 0);
    lv_obj_set_style_pad_all(badge, 0, 0);
    lv_obj_set_style_shadow_width(badge, 5, 0);
    lv_obj_set_style_shadow_color(badge, RED808_ERROR, 0);
    lv_obj_set_style_shadow_opa(badge, LV_OPA_60, 0);
    lv_obj_add_flag(badge, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(badge, cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* lbl = lv_label_create(badge);
    lv_label_set_text(lbl, LV_SYMBOL_CLOSE);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
    lv_obj_center(lbl);
    return badge;
}

static void ui_midi_badge_refresh(lv_obj_t* badge) {
    if (!badge) return;
    static uint32_t prevMidiRev = 0;
    static uint32_t midiLastSeenMs = 0;
    const uint32_t midiRev = control_midi_activity_revision();
    if (midiRev != prevMidiRev) {
        prevMidiRev = midiRev;
        midiLastSeenMs = millis();
    }
    if (midiLastSeenMs == 0) {
        lv_obj_add_flag(badge, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    lv_obj_clear_flag(badge, LV_OBJ_FLAG_HIDDEN);
    const bool recent = (millis() - midiLastSeenMs) < 3000;
    lv_obj_set_style_bg_color(badge, recent ? RED808_SUCCESS : RED808_WARNING, 0);
}

void ui_update_header(void) {
    static int prev_bpm = -1, prev_frac = -1, prev_pat = -1;
    static bool prev_play = false;

    if (p4.bpm_int != prev_bpm || p4.bpm_frac != prev_frac) {
        prev_bpm = p4.bpm_int;
        prev_frac = p4.bpm_frac;
        if (hdr_bpm_label) lv_label_set_text_fmt(hdr_bpm_label, "%d.%d", p4.bpm_int, p4.bpm_frac);
    }

    if (p4.current_pattern != prev_pat) {
        prev_pat = p4.current_pattern;
        if (hdr_pattern_label) lv_label_set_text_fmt(hdr_pattern_label, "P%02d", p4.current_pattern + 1);
    }

    if (p4.is_playing != prev_play) {
        prev_play = p4.is_playing;
        if (hdr_play_btn && hdr_play_label) {
            lv_label_set_text(hdr_play_label, p4.is_playing ? "PAUSE" : "PLAY");
            lv_obj_set_style_bg_color(hdr_play_btn, p4.is_playing ? RED808_SUCCESS : RED808_ACCENT, 0);
            lv_obj_set_style_border_color(hdr_play_btn, p4.is_playing ? RED808_CYAN : RED808_ACCENT2, 0);
        }
    }

    ui_midi_badge_refresh(s_live_midi_badge);
    ui_midi_badge_refresh(s_seq_midi_badge);
}

// =============================================================================
// BOOT SCREEN
// =============================================================================
// ── Boot estilo terminal 90s (BIOS/POST) pero moderno ────────────────────────
// Líneas que van apareciendo tipo consola con estados dinámicos ([....]→ OK)
// y cursor de bloque parpadeante. Fuente pixel UNSCII_16 (mono retro).
#define BOOT_TERM_LINES 8
static lv_obj_t* s_boot_term[BOOT_TERM_LINES] = {};
static lv_obj_t* s_boot_cursor = NULL;
static lv_obj_t* s_boot_progress = NULL;
static lv_obj_t* s_boot_status_lbl = NULL;
static lv_obj_t* s_boot_continue_btn = NULL;

// Colores del boot ligados al tema activo (currentTheme) — ui_theme_apply()
// ya se llama con el tema persistido del usuario antes de crear esta pantalla
// (ver main.cpp: settings_load() + ui_theme_apply() preceden a create_boot_screen()).
static inline lv_color_t boot_phosphor(void)     { return theme_accent(); }
static inline lv_color_t boot_phosphor_dim(void) { return theme_text_dim(); }

static void boot_continue_cb(lv_event_t* e) {
    LV_UNUSED(e);
    ui_navigate_to(2);  // SCREEN_LIVE
}

static void create_boot_screen(void) {
    scr_boot = lv_obj_create(NULL);
    // Fondo del tema activo del usuario (Ocean por defecto, ver settings_store.cpp)
    lv_obj_set_style_bg_color(scr_boot, theme_bg(), 0);
    lv_obj_clear_flag(scr_boot, LV_OBJ_FLAG_SCROLLABLE);

    // "Scanlines" sutiles: banda superior degradada que insinúa fósforo CRT
    lv_obj_t* glow = lv_obj_create(scr_boot);
    lv_obj_set_size(glow, LCD_H_RES, 110);
    lv_obj_align(glow, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(glow, boot_phosphor(), 0);
    lv_obj_set_style_bg_opa(glow, LV_OPA_10, 0);
    lv_obj_set_style_bg_grad_color(glow, theme_bg(), 0);
    lv_obj_set_style_bg_grad_dir(glow, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_width(glow, 0, 0);
    lv_obj_clear_flag(glow, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(glow, LV_OBJ_FLAG_CLICKABLE);

    // Marco perimetral fino, rollo consola de rack
    lv_obj_t* frame = lv_obj_create(scr_boot);
    lv_obj_set_size(frame, LCD_H_RES - 20, LCD_V_RES - 20);
    lv_obj_center(frame);
    lv_obj_set_style_bg_opa(frame, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(frame, boot_phosphor_dim(), 0);
    lv_obj_set_style_border_opa(frame, LV_OPA_60, 0);
    lv_obj_set_style_border_width(frame, 1, 0);
    lv_obj_set_style_radius(frame, 2, 0);
    lv_obj_clear_flag(frame, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(frame, LV_OBJ_FLAG_CLICKABLE);

    // Cabecera: identidad arriba a la izquierda, build a la derecha
    lv_obj_t* hdr = lv_label_create(scr_boot);
    lv_label_set_text(hdr, "BLUESLAVEP4 PERFORMANCE SYSTEM");
    lv_obj_set_style_text_font(hdr, &lv_font_unscii_16, 0);
    lv_obj_set_style_text_letter_space(hdr, 2, 0);
    lv_obj_set_style_text_color(hdr, boot_phosphor(), 0);
    lv_obj_align(hdr, LV_ALIGN_TOP_LEFT, 44, 32);

    lv_obj_t* hdr2 = lv_label_create(scr_boot);
    lv_label_set_text(hdr2, "BIOS v4.0 - ESP32-P4 CONTROL SURFACE");
    lv_obj_set_style_text_font(hdr2, &lv_font_unscii_8, 0);
    lv_obj_set_style_text_color(hdr2, boot_phosphor_dim(), 0);
    lv_obj_align(hdr2, LV_ALIGN_TOP_LEFT, 44, 56);

    // Firma de build (confirma qué firmware corre — útil con varios flasheos)
    lv_obj_t* ver = lv_label_create(scr_boot);
    lv_label_set_text(ver, "BUILD " __DATE__);
    lv_obj_set_style_text_font(ver, &lv_font_unscii_8, 0);
    lv_obj_set_style_text_color(ver, boot_phosphor_dim(), 0);
    lv_obj_align(ver, LV_ALIGN_TOP_RIGHT, -44, 56);

    lv_obj_t* hr = lv_obj_create(scr_boot);
    lv_obj_set_size(hr, LCD_H_RES - 88, 2);
    lv_obj_align(hr, LV_ALIGN_TOP_LEFT, 44, 74);
    lv_obj_set_style_bg_color(hr, boot_phosphor_dim(), 0);
    lv_obj_set_style_border_width(hr, 0, 0);
    lv_obj_clear_flag(hr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(hr, LV_OBJ_FLAG_CLICKABLE);

    // Líneas del terminal (ocultas; el loop de boot las revela en secuencia)
    for (int i = 0; i < BOOT_TERM_LINES; i++) {
        s_boot_term[i] = lv_label_create(scr_boot);
        lv_label_set_text(s_boot_term[i], "");
        lv_obj_set_style_text_font(s_boot_term[i], &lv_font_unscii_16, 0);
        lv_obj_set_style_text_color(s_boot_term[i], boot_phosphor(), 0);
        lv_obj_align(s_boot_term[i], LV_ALIGN_TOP_LEFT, 44, 100 + i * 34);
        lv_obj_add_flag(s_boot_term[i], LV_OBJ_FLAG_HIDDEN);
    }

    // Cursor de bloque parpadeante (siempre bajo la última línea visible)
    s_boot_cursor = lv_label_create(scr_boot);
    lv_label_set_text(s_boot_cursor, "\xE2\x96\x88");  /* █ */
    lv_obj_set_style_text_font(s_boot_cursor, &lv_font_unscii_16, 0);
    lv_obj_set_style_text_color(s_boot_cursor, boot_phosphor(), 0);
    lv_obj_align(s_boot_cursor, LV_ALIGN_TOP_LEFT, 44, 100);

    // Botón CONTINUE: oculto hasta que el self-test termina. El boot ya no
    // salta solo al LIVE — así el POST se puede leer con calma.
    s_boot_continue_btn = lv_btn_create(scr_boot);
    lv_obj_set_size(s_boot_continue_btn, 280, 56);
    lv_obj_align(s_boot_continue_btn, LV_ALIGN_BOTTOM_MID, 0, -118);
    lv_obj_set_style_radius(s_boot_continue_btn, 0, 0);
    lv_obj_set_style_bg_color(s_boot_continue_btn, boot_phosphor(), 0);
    lv_obj_set_style_bg_opa(s_boot_continue_btn, LV_OPA_10, 0);
    lv_obj_set_style_bg_opa(s_boot_continue_btn, LV_OPA_30, LV_STATE_PRESSED);
    lv_obj_set_style_border_color(s_boot_continue_btn, boot_phosphor(), 0);
    lv_obj_set_style_border_width(s_boot_continue_btn, 2, 0);
    lv_obj_set_style_shadow_width(s_boot_continue_btn, 0, 0);
    lv_obj_add_event_cb(s_boot_continue_btn, boot_continue_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(s_boot_continue_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t* cont_lbl = lv_label_create(s_boot_continue_btn);
    lv_label_set_text(cont_lbl, "[ CONTINUE ]");
    lv_obj_set_style_text_font(cont_lbl, &lv_font_unscii_16, 0);
    lv_obj_set_style_text_color(cont_lbl, boot_phosphor(), 0);
    lv_obj_center(cont_lbl);

    // Pie: separador + estado + barra de progreso fina estilo carga retro
    lv_obj_t* hr2 = lv_obj_create(scr_boot);
    lv_obj_set_size(hr2, LCD_H_RES - 88, 1);
    lv_obj_align(hr2, LV_ALIGN_BOTTOM_LEFT, 44, -92);
    lv_obj_set_style_bg_color(hr2, boot_phosphor_dim(), 0);
    lv_obj_set_style_border_width(hr2, 0, 0);
    lv_obj_clear_flag(hr2, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(hr2, LV_OBJ_FLAG_CLICKABLE);

    s_boot_status_lbl = lv_label_create(scr_boot);
    lv_label_set_text(s_boot_status_lbl, "SELF-TEST IN PROGRESS ...");
    lv_obj_set_style_text_font(s_boot_status_lbl, &lv_font_unscii_8, 0);
    lv_obj_set_style_text_color(s_boot_status_lbl, boot_phosphor_dim(), 0);
    lv_obj_align(s_boot_status_lbl, LV_ALIGN_BOTTOM_LEFT, 44, -66);

    s_boot_progress = lv_bar_create(scr_boot);
    lv_obj_set_size(s_boot_progress, LCD_H_RES - 88, 6);
    lv_obj_align(s_boot_progress, LV_ALIGN_BOTTOM_LEFT, 44, -42);
    lv_bar_set_range(s_boot_progress, 0, 100);
    lv_bar_set_value(s_boot_progress, 4, LV_ANIM_OFF);
    lv_obj_set_style_radius(s_boot_progress, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(s_boot_progress, 0, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s_boot_progress, theme_panel(), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_boot_progress, boot_phosphor(), LV_PART_INDICATOR);
}

// =============================================================================
// LIVE GRID SCREEN — 8×4 full-screen grid (4×4 pads + 4×4 controls)
// =============================================================================
static lv_obj_t* live_pad_btns[16] = {};
static lv_obj_t* live_pad_labels[16] = {};
static lv_obj_t* live_pad_num_labels[16] = {};
static lv_obj_t* live_pad_state_labels[16] = {};
static lv_obj_t* live_pad_inst_labels[16] = {};
static lv_obj_t* live_pad_accent_strips[16] = {};
static lv_obj_t* live_pad_midi_badges[16] = {};
static lv_obj_t* live_spectrum_bars[16] = {};  // spectrum bar per pad (bottom of pad)
static lv_obj_t* live_home_panels[24] = {};
static int       live_home_panel_count = 0;
static lv_obj_t* grid_fx_btn = NULL;
static lv_obj_t* grid_fx_active_badge = NULL;

// Implemented with the FX state helpers below; HOME only needs the semantic
// answer, not knowledge of individual DSP cards.
static bool fx_any_active(void);

// Defined after the sequencer raw grid (multi-bar patterns): true when the
// track has a hit on the step that is actually SOUNDING right now.
static bool live_step_hit(int track);

// Pad layout mode: 0=16 normal, 1=16 FS, 2=8 FS, 3=4 FS, 4=2 FS, 5=1 FS
static int        s_pad_mode      = 0;
static lv_obj_t*  s_pad_back_btn  = NULL;
static lv_obj_t*  s_pad_mode_modal = NULL;

// Right-side control widgets (dynamic updates)
static lv_obj_t* grid_play_btn = NULL;
static lv_obj_t* grid_play_lbl = NULL;
static lv_obj_t* grid_bpm_lbl = NULL;
static lv_obj_t* grid_tempo_ref_lbl = NULL;
static lv_obj_t* grid_home_vol_lbl = NULL;
static lv_obj_t* grid_pat_lbl = NULL;
static lv_obj_t* grid_step_lbl = NULL;
static lv_obj_t* grid_step_dots[16] = {};
static lv_obj_t* grid_nr_btn  = NULL;   // Note Repeat toggle + subdivision cycler
static lv_obj_t* grid_nr_lbl  = NULL;
static lv_obj_t* grid_16l_btn = NULL;   // 16 Levels toggle
static lv_obj_t* grid_16l_lbl = NULL;
// Link status indicators (replaces the old "LIVE" badge)
static lv_obj_t* grid_mstr_dot = NULL;  // DaisyPod3 command link
static lv_obj_t* grid_mstr_lbl = NULL;
static lv_obj_t* grid_vol_lbl = NULL;
static lv_obj_t* grid_pad_prev_btn = NULL;
static lv_obj_t* grid_pad_next_btn = NULL;
static lv_obj_t* grid_pad_lbl = NULL;
static lv_obj_t* grid_inst_prev_btn = NULL;
static lv_obj_t* grid_inst_next_btn = NULL;
static lv_obj_t* grid_inst_lbl = NULL;
static lv_obj_t* grid_inst_edit_btn = NULL;
static lv_obj_t* grid_xtra_btns[4] = {};
static lv_obj_t* grid_xtra_lbls[4] = {};
static lv_obj_t* grid_xtra_change_btns[4] = {};
static lv_obj_t* grid_xtra_delete_btns[4] = {};
static lv_obj_t* grid_xtra_meta_lbls[4] = {};
static lv_obj_t* grid_xtra_slot_lbls[4] = {};
static int s_xtra_pending_slot = -1;
static lv_obj_t* s_pad_inst_modal = NULL;
static lv_obj_t* s_pad_inst_modal_pad_lbl = NULL;
static lv_obj_t* s_pad_inst_modal_inst_lbl = NULL;
static lv_obj_t* s_pad_inst_modal_pad_btns[16] = {};
static lv_obj_t* s_pad_inst_modal_inst_btns[8] = {};
static lv_obj_t* s_pad_inst_modal_kit_btns[3][5] = {};   // [engine 0=808/1=909/2=505][preset 0..4]
static lv_obj_t* s_pad_inst_modal_kit_lbl_eng[3] = {};   // labels "808"/"909"/"505"

// ── Per-instrument FX (idea 2): filter + distortion/bitcrush + sends, one
// track (pad) at a time. P4-local only — DaisyPod3 does not report these
// back, so the panel keeps the last value it sent per pad as its state. ──
struct PadFxState {
    uint8_t filterType;   // 0 OFF, else matches fx_filter_model_name() codes
    uint8_t cutoffU7;     // 0..127, default 127 (fully open)
    uint8_t resoU7;       // 0..127, default 0 (gentle)
    uint8_t driveU7;      // 0..127, default 0 (bypass)
    uint8_t bitsU7;       // 0..127, default 0 (bypass, 16-bit)
    uint8_t rvbU7;        // 0..127, default 0 (no send)
    uint8_t dlyU7;        // 0..127, default 0 (no send)
};
static PadFxState s_pad_fx_state[16] = {};
static bool       s_pad_fx_state_init[16] = {};
static lv_obj_t*  s_pad_fx_modal = NULL;
static lv_obj_t*  s_pad_fx_modal_title = NULL;
static lv_obj_t*  s_pad_fx_filter_btns[7] = {};
static lv_obj_t*  s_pad_fx_cutoff_slider = NULL;
static lv_obj_t*  s_pad_fx_reso_slider = NULL;
static lv_obj_t*  s_pad_fx_drive_slider = NULL;
static lv_obj_t*  s_pad_fx_bits_slider = NULL;
static lv_obj_t*  s_pad_fx_rvb_slider = NULL;
static lv_obj_t*  s_pad_fx_dly_slider = NULL;
static lv_obj_t*  s_pad_fx_cutoff_lbl = NULL;
static lv_obj_t*  s_pad_fx_reso_lbl = NULL;
static lv_obj_t*  s_pad_fx_drive_lbl = NULL;
static lv_obj_t*  s_pad_fx_bits_lbl = NULL;
static lv_obj_t*  s_pad_fx_rvb_lbl = NULL;
static lv_obj_t*  s_pad_fx_dly_lbl = NULL;
static lv_obj_t*  s_pad_fx_subtitle_lbl = NULL;
static uint8_t    s_pad_fx_focus_pad = 0;

static lv_obj_t* s_pod_status_modal = NULL;
static lv_obj_t* s_pod_status_label = NULL;
static lv_obj_t* s_pod_screensaver_btn = NULL;   // STATUS > preferencia SALVAPANTALLAS
// ── AKAI MPD218 MIDI MAP + LEARN ─────────────────────────────────────
static lv_obj_t* s_mpd_map_modal = NULL;
static lv_obj_t* s_mpd_map_summary_label = NULL;
static lv_obj_t* s_mpd_activity_label = NULL;
static lv_obj_t* s_mpd_pad_cells[16] = {};
static lv_obj_t* s_mpd_pad_labels[16] = {};
static lv_obj_t* s_mpd_knob_cells[6] = {};
static lv_obj_t* s_mpd_knob_arcs[6] = {};
static lv_obj_t* s_mpd_knob_labels[6] = {};
static lv_obj_t* s_mpd_learn_btn = NULL;
static lv_obj_t* s_mpd_learn_label = NULL;
static lv_obj_t* s_mpd_dev_btn = NULL;
static lv_obj_t* s_mpd_prog_btn = NULL;
static lv_obj_t* s_mpd_padbank_btn = NULL;
static lv_obj_t* s_mpd_ctrlbank_btn = NULL;
static lv_obj_t* s_mpd_batch_btn = NULL;
static bool s_mpd_batch_learn = false;
static uint8_t s_mpd_device = 0;
static uint8_t s_mpd_bank = 0;
static uint8_t s_mpd_pad_layer = 0;
static uint8_t s_mpd_knob_layer = 0;
static uint32_t s_mpd_seen_capture_rev = 0;
static uint32_t s_mpd_seen_activity_rev = 0;
static uint32_t s_mpd_seen_timeout_rev = 0;
static unsigned long s_mpd_pad_glow_until[16] = {};
static unsigned long s_mpd_knob_glow_until[6] = {};
// Assignment picker (opened by LEARN capture or by tapping a pad/knob cell)
static lv_obj_t* s_mpd_assign_modal = NULL;
static lv_obj_t* s_mpd_assign_grid = NULL;
static lv_obj_t* s_mpd_assign_cat_btns[7] = {};
static MidiLearnCapture s_mpd_assign_capture = {};
static uint8_t s_mpd_assign_category = 0;
static constexpr uint8_t POD_CONTROL_ROW_COUNT = 11;
static lv_obj_t* s_pod_control_value_labels[POD_CONTROL_ROW_COUNT] = {};
static lv_obj_t* s_pod_function_modal = NULL;
static uint8_t s_pod_function_modal_row = 0xFF;
static lv_obj_t* s_pod_led_function_labels[2] = {};
static lv_obj_t* s_pod_led_color_labels[2] = {};
static PodConfigPayload s_pod_config = {};
static uint32_t s_pod_seen_revision = 0;
static void pod_status_modal_close_cb(lv_event_t* e);
static void pod_function_modal_close_cb(lv_event_t* e);
static void mpd_map_modal_close_cb(lv_event_t* e);

static const char* POD_CONTROL_TITLES[POD_CONTROL_ROW_COUNT] = {
    "BUTTON 1", "BUTTON 2", "KNOB 1", "KNOB 2", "ENCODER",
    "ENC PUSH", "ROTARY 1 / CH0", "ROTARY 2 / CH1",
    "ROTARY 3 / CH2", "ROTARY 4 / CH3", "FADER / GPIO20 ADC"
};

static const char* PAD_INST_NAMES[8] = {
    "Sampler", "808", "909", "505", "303", "WT", "FM2", "SH101"
};
static const char* PAD_INST_SHORT[8] = {
    "SMP", "808", "909", "505", "303", "WT", "FM2", "SH1"
};
static uint8_t s_pad_inst_sel[16] = {0};
// Selección pendiente del modal PAD SOUND — no se aplica al master hasta
// pulsar PREVIEW (suena) o ASIGNAR (suena y cierra).
static uint8_t s_pad_inst_pending[16] = {0};
// Kit (preset 0..4) por pad. Solo aplica cuando el instrumento del pad es un
// drum engine (808/909/505). El "pending" es lo que el usuario marca en el
// modal antes de PREVIEW/ASIGNAR; el "assigned" es lo confirmado por pad.
static uint8_t s_pad_kit_pending[16] = {0};
static uint8_t s_pad_kit_assigned[16] = {0};
// Último kit enviado a la Daisy por engine drum (0=808, 1=909, 2=505) para
// deduplicar envíos: solo se manda CMD_SYNTH_PRESET si difiere.
static int8_t s_engine_kit_last_applied[3] = {-1,-1,-1};
static volatile uint8_t s_pad_inst_focus_pad = 0;
static unsigned long s_pad_inst_local_ms[16] = {};
static const unsigned long PAD_INST_OWNERSHIP_MS = 1800;
static void pad_inst_modal_refresh(void);
static void seq_refresh_track_label(uint8_t track);
static bool pad_inst_unload_daisy_sample(uint8_t pad);
static int pp_engine_idx_from_code(uint8_t engine);
static uint8_t xtra_slot_engine_code(int slot);

static constexpr int XTRA_PARAM_MAX = 21;

struct XtraPadSlot {
    bool used;
    uint8_t pad;
    char name[24];
    uint8_t synth_engine_idx;
    uint8_t preset_idx;
    bool synth_mode;
    uint8_t trim_start_pct;
    uint8_t trim_end_pct;
    uint8_t fade_in_ms;      // 0-255ms, 0=off — ramps from trim_start
    uint8_t fade_out_ms;     // 0-255ms, 0=off — ramps into trim_end
    uint16_t gate_ms;
    uint8_t play_mode;       // 0=one shot, 1=tempo-synced repeat
    uint32_t duration_ms;
    uint32_t sample_rate;
    uint8_t channels;
    uint8_t bits;
};

static XtraPadSlot s_xtra_slots[4] = {};
static const char* XTRA_PADS_STATE_FILE = "/xtra_pads.txt";
static const char* XTRA_PADS_PARAMS_FILE = "/xtra_params.txt";
static bool s_sd_for_xtra = false;
static uint8_t xtra_backing_pad_for_slot(int slot);
static bool s_xtra_touch_active[4] = {};
static bool s_xtra_hold_latched[4] = {};
static int s_xtra_last_note[4] = {-1, -1, -1, -1};
static lv_coord_t s_xtra_last_lx[4] = {};
static lv_coord_t s_xtra_last_ly[4] = {};
static uint32_t s_xtra_touch_start_ms[4] = {};
static uint32_t s_xtra_sampler_next_ms[4] = {};
static uint32_t s_xtra_noteoff_at[4] = {};
static uint32_t s_xtra_xy_last_send_ms[4] = {};
static float s_xtra_param_values[4][XTRA_PARAM_MAX] = {};
static bool s_xtra_param_valid[4] = {};
static lv_obj_t* s_xtra_editor_modal = NULL;
static lv_obj_t* s_xtra_editor_start = NULL;
static lv_obj_t* s_xtra_editor_end = NULL;
static lv_obj_t* s_xtra_editor_gate = NULL;
static lv_obj_t* s_xtra_editor_fade_in = NULL;
static lv_obj_t* s_xtra_editor_fade_out = NULL;
static lv_obj_t* s_xtra_editor_start_lbl = NULL;
static lv_obj_t* s_xtra_editor_end_lbl = NULL;
static lv_obj_t* s_xtra_editor_gate_lbl = NULL;
static lv_obj_t* s_xtra_editor_fade_in_lbl = NULL;
static lv_obj_t* s_xtra_editor_fade_out_lbl = NULL;
static lv_obj_t* s_xtra_editor_mode_lbl = NULL;
static lv_obj_t* s_xtra_editor_wave = NULL;
static lv_point_t s_xtra_editor_wave_points[192];
static int s_xtra_editor_slot = -1;
static int8_t s_xtra_wave_max[4][96] = {};
static int8_t s_xtra_wave_min[4][96] = {};
static uint8_t s_xtra_wave_count[4] = {};

// ── XTRA PAGES: one page per folder (at any depth) under /data/xtra on
// Daisy's own SD that directly contains at least one WAV ───────────────────
// Page 0 ("DEFAULT") is whatever loose WAVs sit directly in /data/xtra (the
// folder's original flat layout, kept working exactly as before this was
// added). RESCAN walks the tree under /data/xtra breadth-first, up to
// XTRA_SCAN_MAX_DEPTH levels deep, and any folder it finds containing a WAV
// becomes one more page (e.g. a folder "HOUSE/KICKS" with WAVs directly in
// it shows as its own page even though it's two levels down) — a folder
// with only subfolders and no WAVs of its own is walked through but not
// added as a page. Switching pages (PREV/NEXT) swaps what's loaded on the
// 4 backing pads (16-19) for the files found in that folder, in directory
// order. Building a new page is manual — Daisy can only read its SD card
// over this link, not write to it — so "add more" means copying a folder
// with up to 4 WAVs somewhere under /data/xtra and pressing RESCAN, not
// creating one from the UI.
#define XTRA_PAGE_MAX        9   // DEFAULT + up to 8 discovered pages
#define XTRA_SCAN_MAX_DEPTH  3   // xtra/A, xtra/A/B, xtra/A/B/C
#define XTRA_SCAN_MAX_VISITS 24  // folders inspected per RESCAN, across all depths
static char     s_xtra_page_names[XTRA_PAGE_MAX][40] = {};
static int      s_xtra_page_count = 1;   // DEFAULT always exists
static int      s_xtra_page_index = 0;
static uint32_t s_xtra_seen_rev = 0;
enum XtraPageReq {
    XTRA_PAGE_REQ_NONE = 0,
    XTRA_PAGE_REQ_ROOT_FOLDERS,  // listing /data/xtra's direct subfolders (seeds the scan queue)
    XTRA_PAGE_REQ_ITEM_FILES,    // does the scan queue's current folder hold any WAV directly?
    XTRA_PAGE_REQ_ITEM_FOLDERS,  // listing that folder's own subfolders, to keep walking down
    XTRA_PAGE_REQ_FILES,         // loading the user-selected active page's files onto pads 16-19
};
static XtraPageReq s_xtra_page_req = XTRA_PAGE_REQ_NONE;
static lv_obj_t* s_xtra_page_lbl = NULL;
static lv_obj_t* s_xtra_page_prev_btn = NULL;
static lv_obj_t* s_xtra_page_next_btn = NULL;

// Breadth-first scan queue: each item is a folder path relative to
// /data/xtra (e.g. "HOUSE" or "HOUSE/KICKS"), not yet checked for WAVs.
struct XtraScanItem { char path[40]; uint8_t depth; };
static XtraScanItem s_xtra_scan_queue[XTRA_SCAN_MAX_VISITS];
static int s_xtra_scan_count = 0;  // items pushed so far
static int s_xtra_scan_idx   = 0;  // next item index to process

static const uint8_t XTRA_SYNTH_ENGINE_CODES[7] = {0, 1, 2, 3, 4, 5, 6};
static const char* XTRA_SYNTH_ENGINE_NAMES[7] = {"808", "909", "505", "303", "WT", "SH101", "FM2"};
static const char* XTRA_PRESET_LABELS[3] = {"A", "B", "C"};
static const uint8_t XTRA_DRUM_INSTRUMENTS[3][3][4] = {
    { {0, 1, 2, 5}, {0, 3, 6, 7}, {0, 4, 8, 9} },
    { {0, 1, 2, 5}, {0, 3, 4, 6}, {1, 4, 7, 8} },
    { {0, 1, 2, 3}, {1, 3, 4, 6}, {0, 2, 5, 7} }
};
static const uint8_t XTRA_MELODIC_BASE_NOTES[3][4] = {
    {48, 52, 55, 60},
    {36, 43, 48, 55},
    {60, 64, 67, 72}
};

static inline lv_color_t xtra_slot_color(int slot) {
    return lv_color_hex(theme_presets[ui_theme_index()].track_colors[slot & 0x0F]);
}

static void xtra_apply_visual_state(int slot, bool active, lv_coord_t lx, lv_coord_t ly) {
    if (slot < 0 || slot >= 4 || !grid_xtra_btns[slot]) return;
    lv_obj_t* obj = grid_xtra_btns[slot];
    lv_color_t accent = xtra_slot_color(slot);
    if (!active) {
        // Idle: dark slot-tinted gradient with the slot color on the border —
        // reads as "armed" without competing with the touch glow below.
        bool used = s_xtra_slots[slot].used;
        lv_obj_set_style_shadow_width(obj, 0, 0);
        lv_obj_set_style_shadow_opa(obj, LV_OPA_0, 0);
        lv_obj_set_style_outline_width(obj, 0, 0);
        lv_obj_set_style_outline_opa(obj, LV_OPA_0, 0);
        lv_obj_set_style_bg_color(obj,
            used ? lv_color_mix(accent, RED808_PANEL, 90) : RED808_SURFACE, 0);
        lv_obj_set_style_bg_grad_color(obj, RED808_PANEL, 0);
        lv_obj_set_style_bg_grad_dir(obj, LV_GRAD_DIR_VER, 0);
        lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(obj, 2, 0);
        lv_obj_set_style_border_color(obj, used ? accent : theme_border(), 0);
        lv_obj_set_style_border_opa(obj, used ? LV_OPA_80 : LV_OPA_50, 0);
        if (grid_xtra_meta_lbls[slot]) lv_obj_set_style_text_color(grid_xtra_meta_lbls[slot], theme_text(), 0);
        if (grid_xtra_slot_lbls[slot]) lv_obj_set_style_text_color(grid_xtra_slot_lbls[slot], theme_text_dim(), 0);
        return;
    }

    lv_coord_t w = lv_obj_get_width(obj);
    lv_coord_t h = lv_obj_get_height(obj);
    lv_coord_t dx = abs((int)lx - (int)(w / 2));
    lv_coord_t dy = abs((int)ly - (int)(h / 2));
    uint16_t travel = (uint16_t)constrain(dx + dy, 0, 240);
    lv_opa_t fill_opa = (lv_opa_t)(190 + travel / 4);
    if (fill_opa > LV_OPA_COVER) fill_opa = LV_OPA_COVER;
    lv_coord_t outline_w = (lv_coord_t)(2 + travel / 30);
    lv_coord_t shadow_w = (lv_coord_t)(18 + travel / 8);
    lv_opa_t glow_opa = (lv_opa_t)(160 + travel / 3);
    if (glow_opa > LV_OPA_COVER) glow_opa = LV_OPA_COVER;

    lv_obj_set_style_bg_color(obj, accent, 0);
    lv_obj_set_style_bg_grad_color(obj, RED808_SURFACE, 0);
    lv_obj_set_style_bg_opa(obj, fill_opa, 0);
    lv_obj_set_style_border_width(obj, 3, 0);
    lv_obj_set_style_border_color(obj, accent, 0);
    lv_obj_set_style_border_opa(obj, LV_OPA_COVER, 0);
    lv_obj_set_style_outline_width(obj, outline_w, 0);
    lv_obj_set_style_outline_pad(obj, 1, 0);
    lv_obj_set_style_outline_color(obj, accent, 0);
    lv_obj_set_style_outline_opa(obj, glow_opa, 0);
    lv_obj_set_style_shadow_width(obj, shadow_w, 0);
    lv_obj_set_style_shadow_color(obj, accent, 0);
    lv_obj_set_style_shadow_opa(obj, glow_opa, 0);
    if (grid_xtra_meta_lbls[slot]) lv_obj_set_style_text_color(grid_xtra_meta_lbls[slot], lv_color_white(), 0);
    if (grid_xtra_slot_lbls[slot]) lv_obj_set_style_text_color(grid_xtra_slot_lbls[slot], lv_color_white(), 0);
}

static void xtra_save_param_state(void);

static bool fs_read_line(File& file, char* out, size_t cap) {
    if (!out || cap == 0 || !file.available()) return false;
    size_t len = 0;
    bool read_any = false;
    while (file.available()) {
        int c = file.read();
        if (c < 0) break;
        read_any = true;
        if (c == '\n') break;
        if (c == '\r') continue;
        if (len + 1 < cap) out[len++] = (char)c;
    }
    out[len] = '\0';
    return read_any;
}

static int xtra_slot_pp_engine_idx(int slot) {
    if (slot < 0 || slot >= 4 || !s_xtra_slots[slot].synth_mode) return -1;
    return pp_engine_idx_from_code(xtra_slot_engine_code(slot));
}

static void xtra_reset_slot_params(int slot) {
    if (slot < 0 || slot >= 4) return;
    memset(s_xtra_param_values[slot], 0, sizeof(s_xtra_param_values[slot]));
    s_xtra_param_valid[slot] = false;
    int eng_idx = xtra_slot_pp_engine_idx(slot);
    if (eng_idx < 0 || eng_idx >= SP_ENGINE_COUNT) return;
    const SynthEngineDef* eng = &SP_ENGINES[eng_idx];
    for (uint8_t i = 0; i < eng->param_count && i < XTRA_PARAM_MAX; i++) {
        s_xtra_param_values[slot][i] = eng->params[i].vdef;
    }
    int preset_idx = constrain((int)s_xtra_slots[slot].preset_idx, 0, (int)eng->preset_count - 1);
    if (preset_idx >= 0 && preset_idx < eng->preset_count) {
        const SynthPreset* pr = &eng->presets[preset_idx];
        for (uint8_t pv = 0; pv < pr->count; pv++) {
            for (uint8_t i = 0; i < eng->param_count && i < XTRA_PARAM_MAX; i++) {
                if (eng->params[i].param_id == pr->values[pv].param_id) {
                    s_xtra_param_values[slot][i] = pr->values[pv].value;
                    break;
                }
            }
        }
    }
    s_xtra_param_valid[slot] = true;
}

static void xtra_capture_editor_state(int slot);
static void xtra_load_editor_state(int slot);

static void xtra_send_slot_param_snapshot(int slot) {
    if (slot < 0 || slot >= 4 || !s_xtra_slots[slot].synth_mode || !ui_control_available()) return;
    int eng_idx = xtra_slot_pp_engine_idx(slot);
    if (eng_idx < 0 || eng_idx >= SP_ENGINE_COUNT) return;
    if (!s_xtra_param_valid[slot]) xtra_reset_slot_params(slot);
    const SynthEngineDef* eng = &SP_ENGINES[eng_idx];
    control_send_synth_preset(eng->engine, s_xtra_slots[slot].preset_idx % 3);
    for (uint8_t i = 0; i < eng->param_count && i < XTRA_PARAM_MAX; i++) {
        control_send_synth_param(eng->engine, 0, eng->params[i].param_id, s_xtra_param_values[slot][i]);
    }
}

static void xtra_load_param_state(void) {
    for (int i = 0; i < 4; i++) xtra_reset_slot_params(i);
    File f = SPIFFS.open(XTRA_PADS_PARAMS_FILE, FILE_READ);
    if (!f) return;
    char buf[512];
    while (fs_read_line(f, buf, sizeof(buf))) {
        if (buf[0] == '\0') continue;
        char* ctx = nullptr;
        char* tok = strtok_r(buf, ",", &ctx);
        if (!tok) continue;
        int slot = atoi(tok);
        tok = strtok_r(nullptr, ",", &ctx);
        if (!tok) continue;
        int engine = atoi(tok);
        tok = strtok_r(nullptr, ",", &ctx);
        if (!tok) continue;
        int count = atoi(tok);
        if (slot < 0 || slot >= 4) continue;
        if (engine != (int)xtra_slot_engine_code(slot)) continue;
        int eng_idx = xtra_slot_pp_engine_idx(slot);
        if (eng_idx < 0 || eng_idx >= SP_ENGINE_COUNT) continue;
        const SynthEngineDef* eng = &SP_ENGINES[eng_idx];
        int limit = constrain(count, 0, (int)eng->param_count);
        for (int i = 0; i < limit && i < XTRA_PARAM_MAX; i++) {
            tok = strtok_r(nullptr, ",", &ctx);
            if (!tok) break;
            s_xtra_param_values[slot][i] = (float)atof(tok);
            s_xtra_param_valid[slot] = true;
        }
    }
    f.close();
}

static void xtra_save_param_state_now(void) {
    File f = SPIFFS.open(XTRA_PADS_PARAMS_FILE, FILE_WRITE);
    if (!f) return;
    for (int slot = 0; slot < 4; slot++) {
        int eng_idx = xtra_slot_pp_engine_idx(slot);
        if (!s_xtra_param_valid[slot] || eng_idx < 0 || eng_idx >= SP_ENGINE_COUNT) continue;
        const SynthEngineDef* eng = &SP_ENGINES[eng_idx];
        f.printf("%d,%u,%u", slot, (unsigned)eng->engine, (unsigned)eng->param_count);
        for (uint8_t i = 0; i < eng->param_count && i < XTRA_PARAM_MAX; i++) {
            f.printf(",%.5f", s_xtra_param_values[slot][i]);
        }
        f.print('\n');
    }
    f.close();
}

// Debounced SPIFFS write: param cells repeat at ~10 Hz while held (slider
// drags likewise), and each save rewrites the whole file from the LVGL task
// (SPIFFS GC can block 100+ ms — visible hitches, plus flash wear). Mark
// dirty here; xtra_param_save_tick() persists 2 s after the last change and
// ui_navigate_to() flushes on screen exit.
static bool     s_xtra_param_save_dirty = false;
static uint32_t s_xtra_param_save_last_change_ms = 0;

static void xtra_save_param_state(void) {
    s_xtra_param_save_dirty = true;
    s_xtra_param_save_last_change_ms = millis();
}

static void xtra_param_save_flush(void) {
    if (!s_xtra_param_save_dirty) return;
    s_xtra_param_save_dirty = false;
    xtra_save_param_state_now();
}

static void xtra_param_save_tick(void) {
    if (!s_xtra_param_save_dirty) return;
    if (millis() - s_xtra_param_save_last_change_ms >= 2000) {
        xtra_param_save_flush();
    }
}

static void xtra_apply_default_slots(void) {
    // XTRA pads: solo sampler — la selección de kit/instrumento/engine se
    // quitó de la UI (ver xtra_change_cb), así que los slots ya no arrancan
    // en modo synth por defecto.
    for (int i = 0; i < 4; i++) {
        s_xtra_slots[i].used = true;
        s_xtra_slots[i].pad = xtra_backing_pad_for_slot(i);
        s_xtra_slots[i].synth_mode = false;
        s_xtra_slots[i].synth_engine_idx = (uint8_t)i;
        s_xtra_slots[i].preset_idx = 0;
        s_xtra_slots[i].trim_start_pct = 0;
        s_xtra_slots[i].trim_end_pct = 100;
        s_xtra_slots[i].gate_ms = 180;
        s_xtra_slots[i].play_mode = 0;
        s_xtra_slots[i].duration_ms = 0;
        s_xtra_slots[i].sample_rate = 0;
        s_xtra_slots[i].channels = 0;
        s_xtra_slots[i].bits = 0;
        s_xtra_slots[i].name[0] = '\0';
        xtra_reset_slot_params(i);
    }
}

static uint8_t xtra_slot_engine_code(int slot) {
    return XTRA_SYNTH_ENGINE_CODES[constrain((int)s_xtra_slots[slot].synth_engine_idx, 0, 6)];
}

static bool xtra_slot_is_drum(int slot) {
    return s_xtra_slots[slot].synth_mode && s_xtra_slots[slot].synth_engine_idx < 3;
}

static void xtra_slot_refresh_name(int slot) {
    if (slot < 0 || slot >= 4) return;
    if (!s_xtra_slots[slot].synth_mode) {
        if (s_xtra_slots[slot].name[0] == '\0') {
            strncpy(s_xtra_slots[slot].name, "SAMPLER", sizeof(s_xtra_slots[slot].name) - 1);
            s_xtra_slots[slot].name[sizeof(s_xtra_slots[slot].name) - 1] = '\0';
        }
        return;
    }
    snprintf(s_xtra_slots[slot].name, sizeof(s_xtra_slots[slot].name), "%s %s",
             XTRA_SYNTH_ENGINE_NAMES[constrain((int)s_xtra_slots[slot].synth_engine_idx, 0, 6)],
             XTRA_PRESET_LABELS[s_xtra_slots[slot].preset_idx % 3]);
}

static void xtra_apply_preset(int slot) {
    if (slot < 0 || slot >= 4 || !ui_control_available() || !s_xtra_slots[slot].synth_mode) return;
    uint8_t engine = xtra_slot_engine_code(slot);
    control_send_synth_preset(engine, s_xtra_slots[slot].preset_idx % 3);
}

static void xtra_send_note_on(int slot, int note, uint8_t velocity) {
    uint8_t engine = xtra_slot_engine_code(slot);
    control_send_synth_note_on_ex(engine, (uint8_t)constrain(note, 24, 96), velocity, false, false);
    s_xtra_last_note[slot] = constrain(note, 24, 96);
    if (s_xtra_slots[slot].play_mode == 0) {
        s_xtra_noteoff_at[slot] = millis() +
            (uint32_t)constrain((int)s_xtra_slots[slot].gate_ms, 40, 2000);
    } else {
        s_xtra_noteoff_at[slot] = 0;
    }
}

static void xtra_send_note_off(int slot) {
    if (slot < 0 || slot >= 4 || s_xtra_last_note[slot] < 0) return;
    uint8_t engine = xtra_slot_engine_code(slot);
    control_send_synth_note_off_ex(engine, 0, (uint8_t)s_xtra_last_note[slot]);
    s_xtra_last_note[slot] = -1;
    s_xtra_noteoff_at[slot] = 0;
}

static void xtra_audio_tick(void) {
    uint32_t now = millis();
    for (int slot = 0; slot < 4; slot++) {
        if (!s_xtra_noteoff_at[slot]) continue;
        if ((int32_t)(now - s_xtra_noteoff_at[slot]) < 0) continue;
        xtra_send_note_off(slot);
        xtra_apply_visual_state(slot, false, 0, 0);
    }
}

static uint32_t xtra_repeat_interval_ms(float xNorm) {
    float bpm = (float)p4.bpm_int + (float)p4.bpm_frac * 0.1f;
    if (bpm < 40.0f || bpm > 300.0f) bpm = 120.0f;
    int division = xNorm < 0.34f ? 2 : (xNorm < 0.67f ? 4 : 8);
    uint32_t interval = (uint32_t)(60000.0f / bpm / (float)division + 0.5f);
    return (uint32_t)constrain((int)interval, 30, 750);
}

static void xtra_apply_xy_modulation(int slot, uint8_t engine, float xNorm, float yNorm) {
    uint32_t now = millis();
    if ((now - s_xtra_xy_last_send_ms[slot]) < 14) return;
    s_xtra_xy_last_send_ms[slot] = now;

    switch (engine) {
        case 3: {
            float bend = (xNorm - 0.5f) * 12.0f;
            float cutoff = 180.0f + yNorm * 4200.0f;
            float envMod = 0.18f + yNorm * 0.82f;
            control_send_synth_param(engine, 0, 14, bend);
            control_send_synth_param(engine, 0, 0, cutoff);
            control_send_synth_param(engine, 0, 2, envMod);
            break;
        }
        case 4: {
            float wavePos = xNorm * 7.0f;
            float cutoff = 1500.0f + yNorm * 12000.0f;
            float volume = 0.40f + yNorm * 0.45f;
            control_send_synth_param(engine, 0, 0, wavePos);
            control_send_synth_param(engine, 0, 4, cutoff);
            control_send_synth_param(engine, 0, 3, volume);
            break;
        }
        case 5: {
            float pwm = 0.1f + xNorm * 0.8f;
            float cutoff = 120.0f + yNorm * 12000.0f;
            float res = 0.12f + yNorm * 0.70f;
            control_send_synth_param(engine, 0, 1, pwm);
            control_send_synth_param(engine, 0, 4, cutoff);
            control_send_synth_param(engine, 0, 5, res);
            break;
        }
        case 6: {
            float ratio = 0.5f + xNorm * 7.5f;
            float detune = (xNorm - 0.5f) * 50.0f;
            float fmIndex = yNorm * 12.0f;
            float feedback = yNorm;
            control_send_synth_param(engine, 0, 8, ratio);
            control_send_synth_param(engine, 0, 12, detune);
            control_send_synth_param(engine, 0, 9, fmIndex);
            control_send_synth_param(engine, 0, 10, feedback);
            break;
        }
        case 7: {
            float mStruct = xNorm;
            float sStruct = 1.0f - xNorm * 0.85f;
            float mDamp = 0.08f + yNorm * 0.86f;
            float sBright = 0.10f + yNorm * 0.90f;
            control_send_synth_param(engine, 0, 1, mStruct);
            control_send_synth_param(engine, 0, 6, sStruct);
            control_send_synth_param(engine, 0, 3, mDamp);
            control_send_synth_param(engine, 0, 7, sBright);
            break;
        }
        default:
            break;
    }
}

static void xtra_trigger_slot(int slot, int lx, int ly, bool initialPress) {
    if (slot < 0 || slot >= 4 || !control_available()) return;
    int w = grid_xtra_btns[slot] ? lv_obj_get_width(grid_xtra_btns[slot]) : 1;
    int h = grid_xtra_btns[slot] ? lv_obj_get_height(grid_xtra_btns[slot]) : 1;
    float xNorm = (float)constrain(lx, 0, w) / (float)(w > 0 ? w : 1);
    float yNorm = 1.0f - (float)constrain(ly, 0, h) / (float)(h > 0 ? h : 1);
    uint8_t velocity = (uint8_t)constrain((int)(40.0f + yNorm * 87.0f + 0.5f), 20, 127);
    if (!s_xtra_slots[slot].synth_mode) {
        if (initialPress) {
            control_send_trigger(s_xtra_slots[slot].pad, velocity);
            s_xtra_sampler_next_ms[slot] = millis() + xtra_repeat_interval_ms(xNorm);
        } else if (s_xtra_slots[slot].play_mode == 1) {
            uint32_t now = millis();
            if ((int32_t)(now - s_xtra_sampler_next_ms[slot]) >= 0) {
                control_send_trigger(s_xtra_slots[slot].pad, velocity);
                uint32_t interval = xtra_repeat_interval_ms(xNorm);
                do { s_xtra_sampler_next_ms[slot] += interval; }
                while ((int32_t)(now - s_xtra_sampler_next_ms[slot]) >= 0);
            }
        }
        return;
    }
    if (initialPress && !xtra_slot_is_drum(slot)) xtra_send_slot_param_snapshot(slot);
    else xtra_apply_preset(slot);
    if (xtra_slot_is_drum(slot)) {
        if (!initialPress) return;
        uint8_t engineIdx = s_xtra_slots[slot].synth_engine_idx;
        uint8_t instrument = XTRA_DRUM_INSTRUMENTS[engineIdx][s_xtra_slots[slot].preset_idx % 3][slot];
        control_send_synth_trigger(xtra_slot_engine_code(slot), instrument, velocity);
        return;
    }

    int note = XTRA_MELODIC_BASE_NOTES[s_xtra_slots[slot].preset_idx % 3][slot] + (int)((xNorm - 0.5f) * 12.0f + (xNorm >= 0.5f ? 0.5f : -0.5f));
    if (initialPress) {
        xtra_send_note_on(slot, note, velocity);
    } else if (note != s_xtra_last_note[slot]) {
        xtra_send_note_off(slot);
        xtra_send_note_on(slot, note, velocity);
    }

    uint8_t engine = xtra_slot_engine_code(slot);
    xtra_apply_xy_modulation(slot, engine, xNorm, yNorm);
}

static bool xtra_local_touch(lv_event_t* e, lv_coord_t* lx, lv_coord_t* ly) {
    if (!e || !lx || !ly) return false;
    lv_obj_t* obj = (lv_obj_t*)lv_event_get_target(e);
    lv_indev_t* indev = lv_indev_get_act();
    if (!obj || !indev) return false;
    lv_point_t p;
    lv_indev_get_point(indev, &p);
    lv_area_t a;
    lv_obj_get_coords(obj, &a);
    *lx = (lv_coord_t)(p.x - a.x1);
    *ly = (lv_coord_t)(p.y - a.y1);
    return true;
}

static void xtra_pad_touch_cb(lv_event_t* e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    if (slot < 0 || slot >= 4 || !grid_xtra_btns[slot]) return;
    lv_event_code_t code = lv_event_get_code(e);
    lv_coord_t lx = 0, ly = 0;
    xtra_local_touch(e, &lx, &ly);
    if (code == LV_EVENT_PRESSED) {
        s_xtra_touch_active[slot] = true;
        s_xtra_hold_latched[slot] = false;
        s_xtra_touch_start_ms[slot] = millis();
        s_xtra_last_lx[slot] = lx;
        s_xtra_last_ly[slot] = ly;
        xtra_trigger_slot(slot, lx, ly, true);
        xtra_apply_visual_state(slot, true, lx, ly);
    } else if (code == LV_EVENT_PRESSING) {
        if (!s_xtra_touch_active[slot]) return;
        uint32_t now = millis();
        if (!s_xtra_hold_latched[slot] && (now - s_xtra_touch_start_ms[slot]) >= 28) {
            s_xtra_hold_latched[slot] = true;
        }
        lv_coord_t dx = abs((int)lx - (int)s_xtra_last_lx[slot]);
        lv_coord_t dy = abs((int)ly - (int)s_xtra_last_ly[slot]);
        if (dx >= 3 || dy >= 3 || s_xtra_hold_latched[slot]) {
            s_xtra_last_lx[slot] = lx;
            s_xtra_last_ly[slot] = ly;
            xtra_trigger_slot(slot, lx, ly, false);
        }
        xtra_apply_visual_state(slot, true, lx, ly);
    } else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        s_xtra_touch_active[slot] = false;
        s_xtra_hold_latched[slot] = false;
        s_xtra_touch_start_ms[slot] = 0;
        s_xtra_sampler_next_ms[slot] = 0;
        if (!xtra_slot_is_drum(slot)) xtra_send_note_off(slot);
        xtra_apply_visual_state(slot, false, 0, 0);
    }
}

static void xtra_refresh_panel(void);
static void xtra_edit_cb(lv_event_t* e);
static lv_obj_t* piano_make_chip(lv_obj_t* parent, int x, int y, int w, int h,
                                 const char* text);

static uint8_t xtra_backing_pad_for_slot(int slot) {
    if (slot < 0) slot = 0;
    if (slot > 3) slot = 3;
    return (uint8_t)(16 + slot);
}

static void xtra_begin_load_for_slot(int slot) {
    if (slot < 0 || slot >= 4) return;
    s_xtra_pending_slot = slot;
    s_sd_for_xtra = true;
    p4sd.selected_is_midi = false;
    p4sd.selected_pad = xtra_backing_pad_for_slot(slot);
    ui_show_toast("XTRA: elige WAV y LOAD", RED808_CYAN);
    ui_navigate_to(9);
}

static void trim_wav_extension(char* name) {
    if (!name) return;
    size_t n = strlen(name);
    if (n > 4) {
        const char* ext = name + n - 4;
        if (strcasecmp(ext, ".wav") == 0) {
            name[n - 4] = '\0';
        }
    }
}

static void xtra_save_state(void) {
    File f = SPIFFS.open(XTRA_PADS_STATE_FILE, FILE_WRITE);
    if (!f) return;
    for (int i = 0; i < 4; i++) {
        const XtraPadSlot& s = s_xtra_slots[i];
        f.printf("%d,%u,%s,%u,%u,%u,%u,%u,%u,%u,%lu,%lu,%u,%u,%u,%u\n",
                 s.used ? 1 : 0, (unsigned)s.pad, s.name,
                 s.synth_mode ? 1U : 0U, (unsigned)s.synth_engine_idx,
                 (unsigned)s.preset_idx, (unsigned)s.trim_start_pct,
                 (unsigned)s.trim_end_pct, (unsigned)s.gate_ms,
                 (unsigned)s.play_mode, (unsigned long)s.duration_ms,
                 (unsigned long)s.sample_rate, (unsigned)s.channels,
                 (unsigned)s.bits, (unsigned)s.fade_in_ms, (unsigned)s.fade_out_ms);
    }
    f.close();
    xtra_save_param_state();
}

static void xtra_load_state(void) {
    memset(s_xtra_slots, 0, sizeof(s_xtra_slots));
    xtra_apply_default_slots();
    File f = SPIFFS.open(XTRA_PADS_STATE_FILE, FILE_READ);
    if (!f) return;
    int idx = 0;
    char line[160];
    while (idx < 4 && fs_read_line(f, line, sizeof(line))) {
        if (line[0] == '\0') {
            idx++;
            continue;
        }
        int used = 0;
        unsigned pad = 0;
        char name[24] = {0};
        unsigned synth_mode = 1, synth_engine_idx = (unsigned)idx, preset_idx = 0;
        unsigned trim_start = 0, trim_end = 100, gate_ms = 180, play_mode = 0;
        unsigned long duration_ms = 0, sample_rate = 0;
        unsigned channels = 0, bits = 0;
        unsigned fade_in = 0, fade_out = 0;
        int parsed = sscanf(line,
            "%d,%u,%23[^,],%u,%u,%u,%u,%u,%u,%u,%lu,%lu,%u,%u,%u,%u",
            &used, &pad, name, &synth_mode, &synth_engine_idx, &preset_idx,
            &trim_start, &trim_end, &gate_ms, &play_mode, &duration_ms,
            &sample_rate, &channels, &bits, &fade_in, &fade_out);
        if (parsed >= 2) {
            s_xtra_slots[idx].used = (used != 0);
            // Enforce fixed XTRA backing slots (16..19) regardless of legacy file values.
            s_xtra_slots[idx].pad = xtra_backing_pad_for_slot(idx);
            // XTRA pads son solo sampler ahora (sin selección de kit/engine en
            // la UI) — ignoramos el synth_mode guardado por configuraciones
            // antiguas para que no reaparezca un slot en modo synth.
            (void)synth_mode;
            s_xtra_slots[idx].synth_mode = false;
            s_xtra_slots[idx].synth_engine_idx = (uint8_t)constrain((int)synth_engine_idx, 0, 6);
            s_xtra_slots[idx].preset_idx = (uint8_t)constrain((int)preset_idx, 0, 2);
            s_xtra_slots[idx].trim_start_pct = (uint8_t)constrain((int)trim_start, 0, 95);
            s_xtra_slots[idx].trim_end_pct = (uint8_t)constrain((int)trim_end, 5, 100);
            if (s_xtra_slots[idx].trim_end_pct <= s_xtra_slots[idx].trim_start_pct)
                s_xtra_slots[idx].trim_end_pct = (uint8_t)min(100, (int)s_xtra_slots[idx].trim_start_pct + 5);
            s_xtra_slots[idx].gate_ms = (uint16_t)constrain((int)gate_ms, 40, 2000);
            s_xtra_slots[idx].play_mode = (uint8_t)constrain((int)play_mode, 0, 1);
            if (parsed >= 11) s_xtra_slots[idx].duration_ms = (uint32_t)duration_ms;
            if (parsed >= 12) s_xtra_slots[idx].sample_rate = (uint32_t)sample_rate;
            if (parsed >= 13) s_xtra_slots[idx].channels = (uint8_t)constrain((int)channels, 0, 8);
            if (parsed >= 14) s_xtra_slots[idx].bits = (uint8_t)constrain((int)bits, 0, 32);
            s_xtra_slots[idx].fade_in_ms = (parsed >= 15) ? (uint8_t)constrain((int)fade_in, 0, 255) : 0;
            s_xtra_slots[idx].fade_out_ms = (parsed >= 16) ? (uint8_t)constrain((int)fade_out, 0, 255) : 0;
            if (parsed >= 3) {
                strncpy(s_xtra_slots[idx].name, name, sizeof(s_xtra_slots[idx].name) - 1);
                s_xtra_slots[idx].name[sizeof(s_xtra_slots[idx].name) - 1] = '\0';
                trim_wav_extension(s_xtra_slots[idx].name);
            }
            xtra_slot_refresh_name(idx);
        }
        idx++;
    }
    f.close();
    xtra_load_param_state();
}

static void xtra_page_refresh_label(void) {
    if (s_xtra_page_lbl) {
        lv_label_set_text_fmt(s_xtra_page_lbl, "PAGINA %d/%d  ·  %s",
                              s_xtra_page_index + 1, s_xtra_page_count,
                              s_xtra_page_names[s_xtra_page_index]);
    }
    bool multi = s_xtra_page_count > 1;
    if (s_xtra_page_prev_btn) {
        if (multi) lv_obj_clear_state(s_xtra_page_prev_btn, LV_STATE_DISABLED);
        else lv_obj_add_state(s_xtra_page_prev_btn, LV_STATE_DISABLED);
    }
    if (s_xtra_page_next_btn) {
        if (multi) lv_obj_clear_state(s_xtra_page_next_btn, LV_STATE_DISABLED);
        else lv_obj_add_state(s_xtra_page_next_btn, LV_STATE_DISABLED);
    }
}

static void xtra_page_folder_path(int pageIdx, char* out, size_t outSize) {
    if (pageIdx <= 0 || pageIdx >= s_xtra_page_count) {
        strncpy(out, "xtra", outSize - 1);
    } else {
        snprintf(out, outSize, "xtra/%s", s_xtra_page_names[pageIdx]);
    }
    out[outSize - 1] = '\0';
}

static void xtra_page_request_files(int pageIdx) {
    if (pageIdx < 0) pageIdx = 0;
    if (pageIdx >= s_xtra_page_count) pageIdx = s_xtra_page_count - 1;
    s_xtra_page_index = pageIdx;
    xtra_page_refresh_label();
    if (!ui_control_available()) return;
    char folder[48];
    xtra_page_folder_path(pageIdx, folder, sizeof(folder));
    SdListFilesPayload payload = {};
    strncpy(payload.folderName, folder, sizeof(payload.folderName) - 1);
    if (daisyUsb.send(CMD_SD_LIST_FILES, &payload, sizeof(payload))) {
        s_xtra_page_req = XTRA_PAGE_REQ_FILES;
    } else {
        ui_show_toast("Daisy USB no disponible", RED808_WARNING);
    }
}

// Queues one more folder (relative to /data/xtra) to inspect during the
// current RESCAN walk. Silently drops it once XTRA_SCAN_MAX_VISITS is hit —
// a safety cap, not something a normal-sized sample library should reach.
static void xtra_scan_push(const char* parentPath, const char* name, uint8_t depth) {
    if (s_xtra_scan_count >= XTRA_SCAN_MAX_VISITS) return;
    XtraScanItem& it = s_xtra_scan_queue[s_xtra_scan_count];
    if (parentPath && parentPath[0])
        snprintf(it.path, sizeof(it.path), "%s/%s", parentPath, name);
    else {
        strncpy(it.path, name, sizeof(it.path) - 1);
        it.path[sizeof(it.path) - 1] = '\0';
    }
    it.depth = depth;
    s_xtra_scan_count++;
}

// Asks Daisy whether the scan queue's current folder holds any WAV directly
// — the answer decides if it becomes a page (see the ITEM_FILES branch in
// xtra_pages_poll below).
static void xtra_scan_request_item_files(void) {
    const XtraScanItem& it = s_xtra_scan_queue[s_xtra_scan_idx];
    char folder[48];
    snprintf(folder, sizeof(folder), "xtra/%s", it.path);
    SdListFilesPayload payload = {};
    strncpy(payload.folderName, folder, sizeof(payload.folderName) - 1);
    if (daisyUsb.send(CMD_SD_LIST_FILES, &payload, sizeof(payload))) {
        s_xtra_page_req = XTRA_PAGE_REQ_ITEM_FILES;
    } else {
        s_xtra_page_req = XTRA_PAGE_REQ_NONE;
        xtra_page_refresh_label();
    }
}

// Moves on to the next unprocessed item in the scan queue, or ends the walk
// once the queue is exhausted or the page list is full.
static void xtra_scan_advance(void) {
    s_xtra_scan_idx++;
    if (s_xtra_scan_idx >= s_xtra_scan_count || s_xtra_page_count >= XTRA_PAGE_MAX) {
        s_xtra_page_req = XTRA_PAGE_REQ_NONE;
        if (s_xtra_page_index >= s_xtra_page_count) s_xtra_page_index = 0;
        xtra_page_refresh_label();
        return;
    }
    xtra_scan_request_item_files();
}

// Requests the list of /data/xtra's direct subfolders from Daisy's own SD
// card, then breadth-first walks down from there (see the header comment on
// the XTRA PAGES state block above) — one page per folder actually holding
// a WAV, at any depth up to XTRA_SCAN_MAX_DEPTH. Called on entry to the
// XTRAPADS screen and by the RESCAN button — read-only, never creates a
// folder and never touches what's currently loaded on pads 16-19; only
// PREV/NEXT (xtra_page_go) does that.
static void xtra_pages_request_folders(void) {
    strncpy(s_xtra_page_names[0], "DEFAULT", sizeof(s_xtra_page_names[0]) - 1);
    s_xtra_page_names[0][sizeof(s_xtra_page_names[0]) - 1] = '\0';
    s_xtra_page_count = 1;
    s_xtra_scan_count = 0;
    s_xtra_scan_idx = 0;
    if (!ui_control_available()) {
        xtra_page_refresh_label();
        return;
    }
    SdListFilesPayload payload = {};
    strncpy(payload.folderName, "xtra", sizeof(payload.folderName) - 1);
    if (daisyUsb.send(CMD_SD_LIST_FOLDERS, &payload, sizeof(payload))) {
        s_xtra_page_req = XTRA_PAGE_REQ_ROOT_FOLDERS;
    } else {
        ui_show_toast("Daisy USB no disponible", RED808_WARNING);
    }
}

static void xtra_page_go(int delta) {
    if (s_xtra_page_count <= 1) return;
    int next = s_xtra_page_index + delta;
    if (next < 0) next = s_xtra_page_count - 1;
    if (next >= s_xtra_page_count) next = 0;
    xtra_page_request_files(next);
}

// Drains the CMD_SD_LIST_FOLDERS / CMD_SD_LIST_FILES responses this screen
// asked for. Poll-driven (called every frame the XTRAPADS screen is active,
// see update_performance_screen) rather than event-driven: the daisy_sd_*
// buffers it reads are a single shared slot also written by the SD CARD
// screen's own Daisy-SD browser, so this only runs while that other screen
// isn't the active one — same revision-counter pattern that screen uses.
// Only one request is ever in flight at a time (each step waits for its
// response before firing the next), so one "have I seen this yet" revision
// counter is enough to cover all four request kinds below.
static void xtra_pages_poll(void) {
    if (s_xtra_page_req == XTRA_PAGE_REQ_NONE || !ui_control_available()) return;
    const auto& state = daisyUsb.state();
    if (state.daisy_sd_revision == s_xtra_seen_rev) return;
    s_xtra_seen_rev = state.daisy_sd_revision;

    if (s_xtra_page_req == XTRA_PAGE_REQ_ROOT_FOLDERS) {
        int count = state.daisy_sd_folder_count;
        for (int i = 0; i < count; i++)
            xtra_scan_push("", state.daisy_sd_folders[i], 1);
        if (s_xtra_scan_count == 0) {
            s_xtra_page_req = XTRA_PAGE_REQ_NONE;
            if (s_xtra_page_index >= s_xtra_page_count) s_xtra_page_index = 0;
            xtra_page_refresh_label();
            return;
        }
        s_xtra_scan_idx = 0;
        xtra_scan_request_item_files();
        return;
    }

    if (s_xtra_page_req == XTRA_PAGE_REQ_ITEM_FILES) {
        const XtraScanItem it = s_xtra_scan_queue[s_xtra_scan_idx];  // copy: push() below may not touch it, but keep it stable regardless
        if (state.daisy_sd_file_count > 0 && s_xtra_page_count < XTRA_PAGE_MAX) {
            strncpy(s_xtra_page_names[s_xtra_page_count], it.path,
                    sizeof(s_xtra_page_names[0]) - 1);
            s_xtra_page_names[s_xtra_page_count][sizeof(s_xtra_page_names[0]) - 1] = '\0';
            s_xtra_page_count++;
        }
        if (it.depth < XTRA_SCAN_MAX_DEPTH && s_xtra_page_count < XTRA_PAGE_MAX) {
            char folder[48];
            snprintf(folder, sizeof(folder), "xtra/%s", it.path);
            SdListFilesPayload payload = {};
            strncpy(payload.folderName, folder, sizeof(payload.folderName) - 1);
            if (daisyUsb.send(CMD_SD_LIST_FOLDERS, &payload, sizeof(payload))) {
                s_xtra_page_req = XTRA_PAGE_REQ_ITEM_FOLDERS;
                return;
            }
        }
        xtra_scan_advance();
        return;
    }

    if (s_xtra_page_req == XTRA_PAGE_REQ_ITEM_FOLDERS) {
        const XtraScanItem it = s_xtra_scan_queue[s_xtra_scan_idx];
        int count = state.daisy_sd_folder_count;
        for (int i = 0; i < count; i++)
            xtra_scan_push(it.path, state.daisy_sd_folders[i], it.depth + 1);
        xtra_scan_advance();
        return;
    }

    // XTRA_PAGE_REQ_FILES — a user-triggered PREV/NEXT page load onto pads 16-19
    s_xtra_page_req = XTRA_PAGE_REQ_NONE;
    char folder[48];
    xtra_page_folder_path(s_xtra_page_index, folder, sizeof(folder));
    int count = state.daisy_sd_file_count;
    if (count > 4) count = 4;
    for (int slot = 0; slot < 4; slot++) {
        uint8_t pad = xtra_backing_pad_for_slot(slot);
        if (slot < count) {
            SdLoadSamplePayload payload = {};
            strncpy(payload.folderName, folder, sizeof(payload.folderName) - 1);
            strncpy(payload.fileName, state.daisy_sd_files[slot], sizeof(payload.fileName) - 1);
            payload.padIndex = pad;
            daisyUsb.send(CMD_SD_LOAD_SAMPLE, &payload, sizeof(payload));

            s_xtra_slots[slot].used = true;
            s_xtra_slots[slot].synth_mode = false;
            s_xtra_slots[slot].pad = pad;
            strncpy(s_xtra_slots[slot].name, state.daisy_sd_files[slot],
                    sizeof(s_xtra_slots[slot].name) - 1);
            s_xtra_slots[slot].name[sizeof(s_xtra_slots[slot].name) - 1] = '\0';
            trim_wav_extension(s_xtra_slots[slot].name);
            s_xtra_slots[slot].trim_start_pct = 0;
            s_xtra_slots[slot].trim_end_pct = 100;
            s_xtra_slots[slot].duration_ms = 0;
            // Loaded straight off Daisy's own SD, never streamed through P4 —
            // no peak data available, so the waveform card falls back to its
            // usual deterministic placeholder envelope (wave_count == 0).
            s_xtra_wave_count[slot] = 0;
        } else if (s_xtra_slots[slot].used) {
            control_send_unload_daisy(pad);
            s_xtra_slots[slot].used = false;
            s_xtra_slots[slot].name[0] = '\0';
            s_xtra_wave_count[slot] = 0;
        }
    }
    daisyUsb.send(CMD_SD_STATUS);
    xtra_save_state();
    xtra_refresh_panel();
    xtra_page_refresh_label();

    char toast[56];
    snprintf(toast, sizeof(toast), "XTRA pagina %d/%d: %s (%d WAV)",
             s_xtra_page_index + 1, s_xtra_page_count,
             s_xtra_page_names[s_xtra_page_index], count);
    ui_show_toast(toast, RED808_SUCCESS);
}

static void xtra_page_prev_cb(lv_event_t* e) { LV_UNUSED(e); xtra_page_go(-1); }
static void xtra_page_next_cb(lv_event_t* e) { LV_UNUSED(e); xtra_page_go(1); }
static void xtra_page_rescan_cb(lv_event_t* e) {
    LV_UNUSED(e);
    ui_show_toast("Buscando carpetas en /data/xtra...", RED808_CYAN);
    xtra_pages_request_folders();
}

static void xtra_refresh_panel(void) {
    for (int i = 0; i < 4; i++) {
        if (!grid_xtra_btns[i] || !grid_xtra_lbls[i]) continue;
        lv_color_t accent = xtra_slot_color(i);
        if (s_xtra_slots[i].used) {
            xtra_slot_refresh_name(i);
            lv_label_set_text(grid_xtra_lbls[i],
                              s_xtra_slots[i].name[0] ? s_xtra_slots[i].name : "XTRA");
            lv_obj_set_style_text_color(grid_xtra_lbls[i], lv_color_white(), 0);
            if (grid_xtra_meta_lbls[i]) {
                if (s_xtra_slots[i].synth_mode) {
                    lv_label_set_text_fmt(grid_xtra_meta_lbls[i], "PRESET %s  ·  XY %s",
                                          XTRA_PRESET_LABELS[s_xtra_slots[i].preset_idx % 3],
                                          xtra_slot_is_drum(i) ? "TRIG" : "NOTE");
                } else {
                    const char* mode = s_xtra_slots[i].play_mode == 1 ? "SYNC REPEAT" : "ONE SHOT";
                    if (s_xtra_slots[i].duration_ms > 0) {
                        lv_label_set_text_fmt(grid_xtra_meta_lbls[i], "%s  ·  %.2fs",
                                              mode, s_xtra_slots[i].duration_ms / 1000.0f);
                    } else {
                        lv_label_set_text_fmt(grid_xtra_meta_lbls[i], "SAMPLER  ·  %s", mode);
                    }
                }
            }
        } else {
            lv_label_set_text(grid_xtra_lbls[i], "+ ADD");
            lv_obj_set_style_text_color(grid_xtra_lbls[i], theme_text_dim(), 0);
            if (grid_xtra_meta_lbls[i]) lv_label_set_text(grid_xtra_meta_lbls[i], "ENGINE SLOT");
        }
        // Idle look (gradient, slot-colored border) lives in
        // xtra_apply_visual_state(false) so a released pad lands on the
        // exact same style as a freshly refreshed one.
        xtra_apply_visual_state(i, false, 0, 0);
        if (grid_xtra_slot_lbls[i]) lv_label_set_text_fmt(grid_xtra_slot_lbls[i], "S%02d", i + 1);
        if (grid_xtra_change_btns[i]) {
            lv_obj_set_style_border_color(grid_xtra_change_btns[i], accent, 0);
            lv_obj_t* lbl = lv_obj_get_child(grid_xtra_change_btns[i], 0);
            if (lbl) {
                lv_label_set_text(lbl, "CARGAR\nSAMPLER");
                lv_obj_set_style_text_color(lbl, accent, 0);
            }
        }
        if (grid_xtra_delete_btns[i]) {
            lv_obj_add_flag(grid_xtra_delete_btns[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void xtra_change_cb(lv_event_t* e) {
    // XTRAPADS: una sola acción, cargar/reemplazar un WAV desde SD.
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    if (slot < 0 || slot >= 4) return;
    s_xtra_slots[slot].used = true;
    s_xtra_slots[slot].synth_mode = false;
    xtra_begin_load_for_slot(slot);
}

static void xtra_delete_cb(lv_event_t* e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    if (slot < 0 || slot >= 4) return;
    s_xtra_slots[slot].used = true;
    xtra_begin_load_for_slot(slot);
}

static void xtra_slot_cb(lv_event_t* e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    if (slot < 0 || slot >= 4) return;
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (s_xtra_slots[slot].synth_mode) ui_show_toast("Hold para XY", theme_success());
    else ui_show_toast("Sampler XTRA listo", theme_success());
}

// Devuelve idx 0..2 si el instrumento es engine drum (808/909/505), -1 si no.
static int8_t pad_inst_drum_engine_idx(uint8_t inst_idx) {
    if (inst_idx == 1) return 0; // 808
    if (inst_idx == 2) return 1; // 909
    if (inst_idx == 3) return 2; // 505
    return -1;
}

static int8_t pad_inst_engine_code(uint8_t inst_idx) {
    switch (inst_idx) {
        case 0: return -1; // Sampler
        case 1: return 0;  // 808
        case 2: return 1;  // 909
        case 3: return 2;  // 505
        case 4: return 3;  // 303
        case 5: return 4; // WT
        case 6: return 6; // FM2
        case 7: return 5; // SH101
        default: return -1;
    }
}

static uint8_t pad_inst_idx_from_engine_code(int8_t engine) {
    switch (engine) {
        case -1: return 0; // Sampler
        case 0: return 1;  // 808
        case 1: return 2;  // 909
        case 2: return 3;  // 505
        case 3: return 4;  // 303
        case 4: return 5;  // WT
        case 6: return 6;  // FM2
        case 5: return 7;  // SH101
        default: return 0;
    }
}

// Legacy engine 7 is deliberately rendered as Sampler: GTR is no longer a
// selectable P4 instrument and must not reappear from old master state.
static void pad_inst_refresh_pad_badge(uint8_t pad) {
    if (pad > 15 || !live_pad_inst_labels[pad]) return;
    uint8_t inst = s_pad_inst_sel[pad];
    if (inst > 7) inst = 0;
    lv_label_set_text(live_pad_inst_labels[pad], PAD_INST_SHORT[inst]);
}

static void pad_inst_refresh_controls(void) {
    uint8_t pad = s_pad_inst_focus_pad;
    if (pad > 15) pad = 15;
    uint8_t inst = s_pad_inst_sel[pad];
    if (inst > 7) inst = 0;
    if (grid_pad_lbl) lv_label_set_text_fmt(grid_pad_lbl, "PAD %02d", (int)pad + 1);
    if (grid_inst_lbl) lv_label_set_text(grid_inst_lbl, PAD_INST_NAMES[inst]);
}

static void pad_inst_apply_to_master(uint8_t pad) {
    if (pad > 15) return;
    uint8_t inst = s_pad_inst_sel[pad];
    if (inst > 7) inst = 0;
    int8_t engine = pad_inst_engine_code(inst);
    s_pad_inst_local_ms[pad] = millis();
    if (control_available() || control_engine_connected()) {
        control_send_set_track_engine(pad, engine);
        // Audible feedback to confirm assignment. Hand the trigger over to the
        // pad queue so the melodic note-off scheduler also runs (avoids 303
        // hanging after the assignment-confirmation tap).
        enqueue_pad_event(pad, 110);
    }
}

// Engine sync arrives from the local controller (Core 1). Latch the payload and
// let the LVGL task apply it from ui_update_current_screen() — the badge and
// modal refreshes below touch LVGL objects, which are not thread-safe.
static int8_t s_pad_engine_sync[16];
static std::atomic<bool> s_pad_engine_sync_pending{false};

void ui_pad_sound_sync_track_engines(const int8_t engines[16]) {
    if (!engines) return;
    memcpy(s_pad_engine_sync, engines, sizeof(s_pad_engine_sync));
    s_pad_engine_sync_pending.store(true, std::memory_order_release);
}

// LVGL task only.
static void pad_inst_consume_engine_sync(void) {
    if (!s_pad_engine_sync_pending.exchange(false, std::memory_order_acquire)) return;
    int8_t engines[16];
    memcpy(engines, s_pad_engine_sync, sizeof(engines));
    unsigned long nowMs = millis();
    bool anyChanged = false;
    for (int pad = 0; pad < 16; pad++) {
        if (nowMs - s_pad_inst_local_ms[pad] < PAD_INST_OWNERSHIP_MS) {
            continue;
        }
        uint8_t incomingInst = pad_inst_idx_from_engine_code(engines[pad]);
        uint8_t oldAssigned = s_pad_inst_sel[pad];
        if (oldAssigned == incomingInst) {
            continue;
        }
        s_pad_inst_sel[pad] = incomingInst;
        if (s_pad_inst_pending[pad] == oldAssigned) {
            // Keep pending in lockstep only when user isn't editing that pad.
            s_pad_inst_pending[pad] = incomingInst;
        }
        pad_inst_refresh_pad_badge((uint8_t)pad);
        seq_refresh_track_label((uint8_t)pad);
        anyChanged = true;
    }
    if (anyChanged) {
        pad_inst_refresh_controls();
        if (s_pad_inst_modal) {
            pad_inst_modal_refresh();
        }
    }
}

// Sync Pads LEDs — pads illuminate automatically with sequencer
static lv_obj_t* grid_sync_btn = NULL;
static bool sync_pads_active = false;  // OFF by default (synced with S3)

// Ripple effect removed — the overlay forced LVGL to invalidate a large
// expanding area every frame for 200ms per tap, drowning the render task
// when tapping fast. The pad border flash on press is enough feedback.

static void pad_touch_cb(lv_event_t* e) {
    // Safety fallback for the LVGL button event. In practice the GT911 direct
    // path (ui_pad_frame_update) already serviced the press at 200Hz; this
    // only fires if LVGL somehow received a touch the cache did not classify
    // as a pad (shouldn't happen with matching geometry). No-op is safe — the
    // real velocity-aware tap handling lives in ui_pad_frame_update().
    LV_UNUSED(e);
}

static void grid_nav_cb(lv_event_t* e) {
    int screen_id = (int)(intptr_t)lv_event_get_user_data(e);
    ui_navigate_to(screen_id);
}

static void live_step_nav_cb(lv_event_t* e) {
    LV_UNUSED(e);
    ui_navigate_to(3);
}

static void grid_sync_cb(lv_event_t* e) {
    LV_UNUSED(e);
    sync_pads_active = !sync_pads_active;
    if (grid_sync_btn) {
        lv_obj_set_style_bg_color(grid_sync_btn,
            sync_pads_active ? RED808_SUCCESS : RED808_SURFACE, 0);
        lv_obj_set_style_border_color(grid_sync_btn,
            sync_pads_active ? RED808_CYAN : RED808_BORDER, 0);
        lv_obj_t* lbl = lv_obj_get_child(grid_sync_btn, 0);
        if (lbl) lv_label_set_text(lbl, sync_pads_active ? "SYNC\nON" : "SYNC\nOFF");
    }
    // Sync state to S3
    local_apply_message(MSG_TOUCH_CMD, TCMD_SYNC_PADS, sync_pads_active ? 1 : 0);
}

// Cycle Note Repeat: OFF → 1/4 → 1/8 → 1/16 → 1/32 → 1/8T → 1/16T → OFF
static void grid_nr_cb(lv_event_t* e) {
    LV_UNUSED(e);
    uint8_t idx = (uint8_t)((s_nr_idx + 1) % 7);
    s_nr_idx = idx;
    if (grid_nr_btn) {
        bool on = (idx != 0);
        lv_obj_set_style_bg_color(grid_nr_btn,
            on ? RED808_ACCENT : RED808_SURFACE, 0);
        lv_obj_set_style_border_color(grid_nr_btn,
            on ? RED808_ACCENT2 : RED808_BORDER, 0);
        if (grid_nr_lbl) lv_label_set_text(grid_nr_lbl, NR_LABEL[idx]);
    }
}

// Toggle 16 Levels — all pads play the last-tapped sample at 16 velocities
static void grid_16l_cb(lv_event_t* e) {
    LV_UNUSED(e);
    bool on = !s_16l_active;
    s_16l_active = on;
    if (grid_16l_btn) {
        lv_obj_set_style_bg_color(grid_16l_btn,
            on ? RED808_CYAN : RED808_SURFACE, 0);
        lv_obj_set_style_border_color(grid_16l_btn,
            on ? RED808_ACCENT : RED808_BORDER, 0);
        if (grid_16l_lbl) {
            if (on) {
                lv_label_set_text_fmt(grid_16l_lbl, "16 LVL\nSRC %d", s_16l_src_pad + 1);
            } else {
                lv_label_set_text(grid_16l_lbl, "16 LVL\nOFF");
            }
        }
    }
}

// Called when the local sync toggle changes on Core 1;
// so only latch the request here. The LVGL task applies it from
// ui_update_current_screen(); LVGL APIs are not thread-safe. -1 = idle.
static std::atomic<int> s_sync_p4_pending{-1};

void ui_live_set_sync_p4(bool on) {
    s_sync_p4_pending.store(on ? 1 : 0, std::memory_order_release);
}

// LVGL task only.
static void ui_live_consume_sync_p4(void) {
    int v = s_sync_p4_pending.exchange(-1, std::memory_order_acquire);
    if (v < 0) return;
    bool on = (v != 0);
    sync_pads_active = on;
    if (grid_sync_btn) {
        lv_obj_set_style_bg_color(grid_sync_btn,
            on ? RED808_SUCCESS : RED808_SURFACE, 0);
        lv_obj_set_style_border_color(grid_sync_btn,
            on ? RED808_CYAN : RED808_BORDER, 0);
        lv_obj_t* lbl = lv_obj_get_child(grid_sync_btn, 0);
        if (lbl) lv_label_set_text(lbl, on ? "SYNC\nON" : "SYNC\nOFF");
    }
}

static void grid_theme_cb(lv_event_t* e) {
    LV_UNUSED(e);
    const uint8_t previous = ui_theme_index();
    int next = (previous + 1) % THEME_COUNT;
    P4_THEME_LOG_PRINTF("[THEME] click current=%u p4=%d next=%d heap=%u psram=%u\n",
                        static_cast<unsigned>(previous), p4.theme, next,
                        static_cast<unsigned>(ESP.getFreeHeap()),
                        static_cast<unsigned>(ESP.getFreePsram()));
    p4.theme = next;
    ui_theme_apply((VisualTheme)next);
    // Sync theme to S3
    local_apply_message(MSG_TOUCH_CMD, TCMD_THEME_NEXT, (uint8_t)next);
}

static void grid_master_vol_step_cb(lv_event_t* e) {
    if (i2c_rotaries_owns_function(POD_FUNC_MASTER_VOLUME)) return;
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    int next = constrain((int)p4.master_volume + delta, 0, Config::MAX_VOLUME);
    if (next == p4.master_volume) return;
    p4.master_volume = (uint8_t)next;
    if (control_available() || control_engine_connected()) control_send_set_volume(next);
}

static void grid_bpm_step_cb(lv_event_t* e) {
    if (i2c_rotaries_owns_function(POD_FUNC_TEMPO)) return;
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    int next = constrain((int)p4.bpm_int + delta, 40, 240);
    if (next == p4.bpm_int) return;
    p4.bpm_int = (uint16_t)next;
    p4.bpm_frac = 0;
    if (control_available() || control_engine_connected()) control_send_tempo((float)next);
}

static void grid_pad_prev_cb(lv_event_t* e) {
    LV_UNUSED(e);
    uint8_t pad = s_pad_inst_focus_pad;
    pad = (pad == 0) ? 15 : (uint8_t)(pad - 1);
    s_pad_inst_focus_pad = pad;
    pad_inst_refresh_controls();
}

static void grid_pad_next_cb(lv_event_t* e) {
    LV_UNUSED(e);
    uint8_t pad = s_pad_inst_focus_pad;
    pad = (pad >= 15) ? 0 : (uint8_t)(pad + 1);
    s_pad_inst_focus_pad = pad;
    pad_inst_refresh_controls();
}

static void grid_inst_prev_cb(lv_event_t* e) {
    LV_UNUSED(e);
    uint8_t pad = s_pad_inst_focus_pad;
    if (pad > 15) pad = 15;
    uint8_t inst = s_pad_inst_sel[pad];
    inst = (inst == 0) ? 7 : (uint8_t)(inst - 1);
    s_pad_inst_sel[pad] = inst;
    pad_inst_refresh_pad_badge(pad);
    pad_inst_refresh_controls();
    pad_inst_apply_to_master(pad);
}

static void grid_inst_next_cb(lv_event_t* e) {
    LV_UNUSED(e);
    uint8_t pad = s_pad_inst_focus_pad;
    if (pad > 15) pad = 15;
    uint8_t inst = s_pad_inst_sel[pad];
    inst = (inst >= 7) ? 0 : (uint8_t)(inst + 1);
    s_pad_inst_sel[pad] = inst;
    pad_inst_refresh_pad_badge(pad);
    pad_inst_refresh_controls();
    pad_inst_apply_to_master(pad);
}

static void pad_inst_modal_refresh(void) {
    uint8_t pad = s_pad_inst_focus_pad;
    if (pad > 15) pad = 15;
    uint8_t inst_assigned = s_pad_inst_sel[pad];
    if (inst_assigned > 7) inst_assigned = 0;
    uint8_t inst_pending = s_pad_inst_pending[pad];
    if (inst_pending > 7) inst_pending = 0;
    bool dirty = (inst_pending != inst_assigned);
    if (s_pad_inst_modal_pad_lbl)
        lv_label_set_text_fmt(s_pad_inst_modal_pad_lbl, "PAD %02d", (int)pad + 1);
    if (s_pad_inst_modal_inst_lbl) {
        if (dirty)
            lv_label_set_text_fmt(s_pad_inst_modal_inst_lbl, "%s  >  %s",
                                  PAD_INST_NAMES[inst_assigned], PAD_INST_NAMES[inst_pending]);
        else
            lv_label_set_text(s_pad_inst_modal_inst_lbl, PAD_INST_NAMES[inst_assigned]);
        lv_obj_set_style_text_color(s_pad_inst_modal_inst_lbl,
            dirty ? RED808_WARNING : RED808_TEXT, 0);
    }
    for (int i = 0; i < 16; i++) {
        lv_obj_t* btn = s_pad_inst_modal_pad_btns[i];
        if (!btn) continue;
        bool active = (i == (int)pad);
        lv_obj_set_style_bg_color(btn, active ? RED808_ACCENT2 : RED808_SURFACE, 0);
        lv_obj_set_style_border_color(btn, active ? RED808_CYAN : RED808_BORDER, 0);
        lv_obj_set_style_bg_opa(btn, active ? LV_OPA_COVER : LV_OPA_80, 0);
        lv_obj_set_style_border_width(btn, active ? 2 : 1, 0);
    }
    for (int i = 0; i < 8; i++) {
        lv_obj_t* btn = s_pad_inst_modal_inst_btns[i];
        if (!btn) continue;
        bool is_pending = (i == (int)inst_pending);
        bool is_assigned = (i == (int)inst_assigned);
        lv_color_t bg = RED808_SURFACE;
        lv_color_t bd = RED808_BORDER;
        lv_opa_t op = LV_OPA_80;
        int bw = 1;
        if (is_pending && dirty) {
            bg = RED808_WARNING;  // amarillo: seleccionado, pendiente de asignar
            bd = RED808_CYAN;
            op = LV_OPA_COVER;
            bw = 3;
        } else if (is_assigned) {
            bg = RED808_ACCENT;
            bd = RED808_CYAN;
            op = LV_OPA_COVER;
            bw = 2;
        }
        lv_obj_set_style_bg_color(btn, bg, 0);
        lv_obj_set_style_border_color(btn, bd, 0);
        lv_obj_set_style_bg_opa(btn, op, 0);
        lv_obj_set_style_border_width(btn, bw, 0);
    }

    // Kit chips: la fila del engine drum pendiente queda activa y resalta el
    // kit pendiente del pad activo. Las otras filas quedan atenuadas.
    int8_t drum_eng = pad_inst_drum_engine_idx(inst_pending);
    uint8_t pad_kit = s_pad_kit_pending[pad];
    if (pad_kit > 4) pad_kit = 0;
    for (int eng = 0; eng < 3; eng++) {
        bool row_active = (eng == drum_eng);
        if (s_pad_inst_modal_kit_lbl_eng[eng]) {
            lv_obj_set_style_text_color(s_pad_inst_modal_kit_lbl_eng[eng],
                row_active ? RED808_ACCENT : RED808_TEXT_DIM, 0);
        }
        for (int p = 0; p < 5; p++) {
            lv_obj_t* kb = s_pad_inst_modal_kit_btns[eng][p];
            if (!kb) continue;
            bool is_pending_kit = row_active && (p == (int)pad_kit);
            lv_color_t bg = RED808_SURFACE;
            lv_color_t bd = RED808_BORDER;
            lv_opa_t op = row_active ? LV_OPA_80 : LV_OPA_30;
            int bw = 1;
            if (is_pending_kit) {
                bg = (p == 4) ? RED808_SUCCESS : RED808_ACCENT;
                bd = RED808_CYAN;
                op = LV_OPA_COVER;
                bw = 3;
            } else if (p == 4 && row_active) {
                // Pure: mantén sello visual cuando la fila está activa
                bd = RED808_CYAN;
                bw = 2;
            }
            lv_obj_set_style_bg_color(kb, bg, 0);
            lv_obj_set_style_border_color(kb, bd, 0);
            lv_obj_set_style_bg_opa(kb, op, 0);
            lv_obj_set_style_border_width(kb, bw, 0);
            if (row_active) {
                lv_obj_add_flag(kb, LV_OBJ_FLAG_CLICKABLE);
            } else {
                lv_obj_clear_flag(kb, LV_OBJ_FLAG_CLICKABLE);
            }
        }
    }
}

static void pad_inst_modal_close_cb(lv_event_t* e) {
    if (e && lv_event_get_target(e) != lv_event_get_current_target(e)) return;
    if (s_pad_inst_modal) {
        lv_obj_del(s_pad_inst_modal);
        s_pad_inst_modal = NULL;
        s_pad_inst_modal_pad_lbl = NULL;
        s_pad_inst_modal_inst_lbl = NULL;
        for (int i = 0; i < 16; i++) s_pad_inst_modal_pad_btns[i] = NULL;
        for (int i = 0; i < 8; i++) s_pad_inst_modal_inst_btns[i] = NULL;
        for (int e2 = 0; e2 < 3; e2++) {
            s_pad_inst_modal_kit_lbl_eng[e2] = NULL;
            for (int p = 0; p < 5; p++) s_pad_inst_modal_kit_btns[e2][p] = NULL;
        }
    }
}

// fwd decl: defined later; used inside the PAD SOUND modal builder
static void grid_pad_kit_select_cb(lv_event_t* e);
// fwd decl: instrument-FX modal (idea 2), opened from a button in this popup
static void pad_fx_modal_show(lv_event_t* e);

static void pad_inst_modal_pick_pad_cb(lv_event_t* e) {
    int pad = (int)(intptr_t)lv_event_get_user_data(e);
    if (pad < 0 || pad > 15) return;
    s_pad_inst_focus_pad = (uint8_t)pad;
    // Al cambiar de pad, descarta pendiente del anterior y empieza con
    // la asignación actual del nuevo.
    s_pad_inst_pending[pad] = s_pad_inst_sel[pad];
    pad_inst_refresh_controls();
    pad_inst_modal_refresh();
}

static void pad_inst_modal_pad_cb(lv_event_t* e) {
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    uint8_t pad = s_pad_inst_focus_pad;
    pad = (uint8_t)((pad + delta + 16) % 16);
    s_pad_inst_focus_pad = pad;
    pad_inst_refresh_controls();
    pad_inst_modal_refresh();
}

static void pad_inst_modal_inst_cb(lv_event_t* e) {
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    uint8_t pad = s_pad_inst_focus_pad;
    if (pad > 15) pad = 15;
    uint8_t inst = s_pad_inst_sel[pad];
    inst = (uint8_t)((inst + delta + 8) % 8);
    s_pad_inst_sel[pad] = inst;
    pad_inst_refresh_pad_badge(pad);
    pad_inst_refresh_controls();
    pad_inst_apply_to_master(pad);
    pad_inst_modal_refresh();
}

// Async unload keeps the LVGL callback independent from the USB TX queue.
// 0 = idle, 1 = running, 2 = done
static std::atomic<uint8_t> s_pad_unload_state{0};
static uint8_t s_pad_unload_pad = 0;
static bool    s_pad_unload_ok = false;

static void pad_inst_unload_task(void* arg) {
    (void)arg;
    uint8_t pad = s_pad_unload_pad;
    const bool ok = control_available();
    if(ok) control_send_unload_daisy(pad);
    s_pad_unload_ok = ok;
    s_pad_unload_state.store(2, std::memory_order_release);
    vTaskDelete(NULL);
}

static bool pad_inst_unload_daisy_sample(uint8_t pad) {
    if (s_pad_unload_state.load(std::memory_order_acquire) != 0) return false;
    s_pad_unload_pad = pad;
    s_pad_unload_state.store(1, std::memory_order_release);
    if (xTaskCreatePinnedToCore(pad_inst_unload_task, "padunload",
                                4096, NULL, 1, NULL, 1) != pdPASS) {
        s_pad_unload_state.store(0, std::memory_order_release);
        return false;
    }
    return true;
}

// LVGL task (ui_update_current_screen): apply a finished unload's result.
static void pad_inst_unload_consume_result(void) {
    if (s_pad_unload_state.load(std::memory_order_acquire) != 2) return;
    uint8_t pad = s_pad_unload_pad;
    bool ok = s_pad_unload_ok;
    s_pad_unload_state.store(0, std::memory_order_release);
    if (!ok) {
        ui_show_toast("No se pudo restaurar sample", RED808_WARNING);
        return;
    }
    if (pad > 15) pad = 15;
    s_pad_inst_sel[pad] = 0;
    pad_inst_refresh_pad_badge(pad);
    pad_inst_refresh_controls();
    pad_inst_apply_to_master(pad);
    pad_inst_modal_refresh();
    ui_show_toast("Sampler original restaurado", RED808_SUCCESS);
}

static void pad_inst_sampler_original_cb(lv_event_t* e) {
    LV_UNUSED(e);
    uint8_t pad = s_pad_inst_focus_pad;
    if (pad > 15) pad = 15;
    if (!control_available() && !control_engine_connected()) {
        ui_show_toast("Master no conectado", RED808_WARNING);
        return;
    }
    if (!pad_inst_unload_daisy_sample(pad)) {
        ui_show_toast("Restauracion en curso...", RED808_WARNING);
        return;
    }
    ui_show_toast("Restaurando sample...", RED808_CYAN);
}

static void pad_inst_modal_pick_inst_cb(lv_event_t* e) {
    int inst = (int)(intptr_t)lv_event_get_user_data(e);
    if (inst < 0 || inst > 7) return;
    uint8_t pad = s_pad_inst_focus_pad;
    if (pad > 15) pad = 15;
    // Solo marca la selección como pendiente. La asignación al master se hace
    // al pulsar PREVIEW (escuchar) o ASIGNAR (confirmar).
    s_pad_inst_pending[pad] = (uint8_t)inst;
    pad_inst_modal_refresh();
}

// Aplica al master la selección pendiente del pad (instrumento + kit si drum).
// Devuelve true si se cambió algo.
static bool pad_inst_commit_pending(uint8_t pad) {
    if (pad > 15) return false;
    bool changed = false;
    uint8_t inst = s_pad_inst_pending[pad];
    if (inst > 7) inst = 0;
    int8_t drum = pad_inst_drum_engine_idx(inst);
    // Cambio de kit por pad. El envío a la Daisy se deduplica con el último
    // kit aplicado al engine: solo se manda CMD_SYNTH_PRESET si difiere del
    // último. (En tiempo real, ui_process_pad_queue también hace switch).
    if (drum >= 0) {
        uint8_t kit = s_pad_kit_pending[pad];
        if (kit > 4) kit = 0;
        if (s_pad_kit_assigned[pad] != kit) {
            s_pad_kit_assigned[pad] = kit;
            changed = true;
        }
        if (control_available() && s_engine_kit_last_applied[drum] != (int8_t)kit) {
            control_send_synth_preset((uint8_t)drum, kit);
            s_engine_kit_last_applied[drum] = (int8_t)kit;
            changed = true;
        }
    }
    // Cambio de instrumento (engine asignado al pad)
    if (s_pad_inst_sel[pad] != inst) {
        s_pad_inst_sel[pad] = inst;
        pad_inst_refresh_pad_badge(pad);
        pad_inst_refresh_controls();
        pad_inst_apply_to_master(pad);  // control_send_set_track_engine + trigger
        changed = true;
    }
    return changed;
}

// PREVIEW: aplica la selección pendiente y dispara el pad para escuchar.
static void pad_inst_modal_preview_cb(lv_event_t* e) {
    LV_UNUSED(e);
    uint8_t pad = s_pad_inst_focus_pad;
    if (pad > 15) pad = 15;
    if (!control_available() && !control_engine_connected()) {
        ui_show_toast("Master no conectado", RED808_WARNING);
        return;
    }
    bool changed = pad_inst_commit_pending(pad);
    if (!changed) {
        // Sin cambios: dispara igualmente para volver a oír
        control_send_trigger(pad, 110);
    }
    pad_inst_modal_refresh();
}

// ASIGNAR: confirma la selección pendiente y cierra el modal.
static void pad_inst_modal_assign_cb(lv_event_t* e) {
    LV_UNUSED(e);
    uint8_t pad = s_pad_inst_focus_pad;
    if (pad > 15) pad = 15;
    if (!control_available() && !control_engine_connected()) {
        ui_show_toast("Master no conectado", RED808_WARNING);
        return;
    }
    pad_inst_commit_pending(pad);
    ui_show_toast("Asignado", RED808_SUCCESS);
    pad_inst_modal_close_cb(NULL);
}

// RANDOM: re-rolls each pad's drum engine (Sampler/808/909/505), the same
// pool a kick can already cycle through by hand. Tasteful on purpose: only
// a majority of pads reroll each tap, and a pad never rerolls to the engine
// it already had, so results stay varied instead of a uniform reset.
static void pad_random_all_cb(lv_event_t* e) {
    LV_UNUSED(e);
    if (!control_available() && !control_engine_connected()) {
        ui_show_toast("Master no conectado", RED808_WARNING);
        return;
    }
    static uint32_t s = 0;
    if (s == 0) s = (uint32_t)millis() ^ 0x2545F491u | 1u;
    static const uint8_t drumPool[4] = {0, 1, 2, 3}; // Sampler, 808, 909, 505
    int rerolled = 0;
    for (int pad = 0; pad < 16; pad++) {
        s ^= s << 13; s ^= s >> 17; s ^= s << 5;
        if ((s % 100u) >= 65u) continue;   // ~65% of pads reroll per tap

        uint8_t current = s_pad_inst_sel[pad];
        if (current > 3) current = 0;      // melodic engine: treat as sampler slot
        s ^= s << 13; s ^= s >> 17; s ^= s << 5;
        const uint8_t choice = (uint8_t)(s % 3u);   // pick among the OTHER 3
        const uint8_t next = drumPool[choice >= current ? choice + 1 : choice];

        s_pad_inst_pending[pad] = next;
        if (pad_inst_commit_pending(pad)) rerolled++;
    }
    pad_inst_modal_refresh();
    if (rerolled == 0) {
        ui_show_toast("RANDOM: sin cambios esta vez", RED808_TEXT_DIM);
    } else {
        char msg[48];
        snprintf(msg, sizeof(msg), "RANDOM: %d pad%s reasignados",
                 rerolled, rerolled == 1 ? "" : "s");
        ui_show_toast(msg, RED808_INFO);
    }
}

static void grid_pad_inst_popup_cb(lv_event_t* e) {
    if (e && lv_event_get_target(e) != lv_event_get_current_target(e)) return;
    if (s_pad_inst_modal) {
        pad_inst_modal_close_cb(NULL);
        return;
    }

    // Inicializa la selección pendiente con la asignación actual de cada pad
    for (int i = 0; i < 16; i++) {
        s_pad_inst_pending[i] = s_pad_inst_sel[i];
        s_pad_kit_pending[i]  = s_pad_kit_assigned[i];
    }

    s_pad_inst_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_pad_inst_modal, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(s_pad_inst_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_pad_inst_modal, LV_OPA_70, 0);
    lv_obj_set_style_border_width(s_pad_inst_modal, 0, 0);
    lv_obj_set_style_pad_all(s_pad_inst_modal, 0, 0);
    lv_obj_clear_flag(s_pad_inst_modal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_pad_inst_modal, pad_inst_modal_close_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* card = lv_obj_create(s_pad_inst_modal);
    lv_obj_set_size(card, 984, 560);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_color(card, RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_dir(card, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_border_color(card, RED808_CYAN, 0);
    lv_obj_set_style_radius(card, 18, 0);
    lv_obj_set_style_pad_all(card, 14, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t* title = lv_label_create(card);
    lv_label_set_text(title, "PAD INSTRUMENT SELECT");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(title, RED808_CYAN, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 2);

    lv_obj_t* sub = lv_label_create(card);
    lv_label_set_text(sub, "Seleccion directa por instrumento");
    lv_obj_set_style_text_font(sub, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(sub, RED808_TEXT_DIM, 0);
    lv_obj_align(sub, LV_ALIGN_TOP_MID, 0, 32);

    s_pad_inst_modal_pad_lbl = lv_label_create(card);
    lv_obj_set_style_text_font(s_pad_inst_modal_pad_lbl, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(s_pad_inst_modal_pad_lbl, RED808_ACCENT, 0);
    lv_obj_align(s_pad_inst_modal_pad_lbl, LV_ALIGN_TOP_MID, 0, 64);

    s_pad_inst_modal_inst_lbl = lv_label_create(card);
    lv_obj_set_style_text_font(s_pad_inst_modal_inst_lbl, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(s_pad_inst_modal_inst_lbl, RED808_TEXT, 0);
    lv_obj_align(s_pad_inst_modal_inst_lbl, LV_ALIGN_TOP_MID, 0, 98);

    const int pad_grid_x0 = 32;
    const int pad_grid_y0 = 138;
    const int pad_btn_w = 64;
    const int pad_btn_h = 34;
    const int pad_gap_x = 10;
    const int pad_gap_y = 10;
    for (int i = 0; i < 16; i++) {
        lv_obj_t* pb = lv_btn_create(card);
        s_pad_inst_modal_pad_btns[i] = pb;
        int col = i % 4;
        int row = i / 4;
        lv_obj_set_size(pb, pad_btn_w, pad_btn_h);
        lv_obj_set_pos(pb, pad_grid_x0 + col * (pad_btn_w + pad_gap_x), pad_grid_y0 + row * (pad_btn_h + pad_gap_y));
        apply_control_button_style(pb, RED808_BORDER, false, 8);
        // Franja de color a la izquierda con el mismo track-color que usa el
        // pad en la pantalla LIVE, para reconocer el pad de un vistazo.
        lv_obj_t* strip = lv_obj_create(pb);
        lv_obj_set_size(strip, 4, pad_btn_h);
        lv_obj_set_pos(strip, 0, 0);
        lv_obj_set_style_bg_color(strip, ui_track_color(i), 0);
        lv_obj_set_style_bg_opa(strip, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(strip, 0, 0);
        lv_obj_set_style_radius(strip, 0, 0);
        lv_obj_clear_flag(strip, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(strip, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_t* pl = lv_label_create(pb);
        lv_label_set_text_fmt(pl, "%02d", i + 1);
        lv_obj_set_style_text_font(pl, &lv_font_montserrat_12, 0);
        lv_obj_center(pl);
        lv_obj_add_event_cb(pb, pad_inst_modal_pick_pad_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
    }

    lv_obj_t* pad_hdr = lv_label_create(card);
    lv_label_set_text(pad_hdr, "PAD");
    lv_obj_set_style_text_font(pad_hdr, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(pad_hdr, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(pad_hdr, pad_grid_x0, pad_grid_y0 - 20);

    const int grid_x0 = 350;
    const int grid_y0 = 138;
    const int grid_cols = 4;
    const int btn_w = 116;
    const int btn_h = 46;
    const int gap_x = 10;
    const int gap_y = 10;

    lv_obj_t* inst_hdr = lv_label_create(card);
    lv_label_set_text(inst_hdr, "INSTRUMENT");
    lv_obj_set_style_text_font(inst_hdr, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(inst_hdr, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(inst_hdr, grid_x0, grid_y0 - 20);

    // Fila 0 (Sampler/808/909/505) = "SAMPLE + DRUM", fila 1 (303/WT/FM2/SH101)
    // = "MELODIC". La franja de color a la izquierda de cada botón hace visible
    // esa agrupación de un vistazo, en vez de que las 8 casillas se vean iguales.
    // No 'static' on the colors: RED808_ACCENT2/CYAN resolve the active theme
    // at call time, and a static local would freeze on whatever theme was
    // active the first time this modal opened.
    const lv_color_t inst_row_colors[2] = { RED808_ACCENT2, RED808_CYAN };
    static const char* inst_row_names[2] = { "SAMPLE + DRUM", "MELODIC" };
    for (int row = 0; row < 2; row++) {
        lv_obj_t* rl = lv_label_create(card);
        lv_label_set_text(rl, inst_row_names[row]);
        lv_obj_set_style_text_font(rl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(rl, inst_row_colors[row], 0);
        lv_obj_set_pos(rl, grid_x0 + grid_cols * (btn_w + gap_x) + 6,
                        grid_y0 + row * (btn_h + gap_y) + btn_h / 2 - 7);
    }

    for (int i = 0; i < 8; i++) {
        lv_obj_t* ib = lv_btn_create(card);
        s_pad_inst_modal_inst_btns[i] = ib;
        int col = i % grid_cols;
        int row = i / grid_cols;
        lv_obj_set_size(ib, btn_w, btn_h);
        lv_obj_set_pos(ib, grid_x0 + col * (btn_w + gap_x), grid_y0 + row * (btn_h + gap_y));
        apply_control_button_style(ib, RED808_BORDER, false, 10);
        lv_obj_t* strip = lv_obj_create(ib);
        lv_obj_set_size(strip, 4, btn_h);
        lv_obj_set_pos(strip, 0, 0);
        lv_obj_set_style_bg_color(strip, inst_row_colors[row], 0);
        lv_obj_set_style_bg_opa(strip, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(strip, 0, 0);
        lv_obj_set_style_radius(strip, 0, 0);
        lv_obj_clear_flag(strip, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(strip, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_t* il = lv_label_create(ib);
        lv_label_set_text(il, PAD_INST_NAMES[i]);
        lv_obj_set_style_text_font(il, &lv_font_montserrat_14, 0);
        lv_obj_center(il);
        lv_obj_add_event_cb(ib, pad_inst_modal_pick_inst_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
    }

    /* === DRUM KITS — preset selector (engines × presets, Pure incluido) === */
    const int kits_y0 = grid_y0 + 2 * (btn_h + gap_y) + 18;  // tras el grid 2x4 de instruments

    lv_obj_t* kit_hdr = lv_label_create(card);
    lv_label_set_text(kit_hdr, "DRUM KITS");
    lv_obj_set_style_text_font(kit_hdr, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(kit_hdr, RED808_CYAN, 0);
    lv_obj_set_pos(kit_hdr, grid_x0, kits_y0 - 18);

    static const char* engine_names[3]      = { "808", "909", "505" };
    static const char* preset_names_808[5] = { "Classic", "HipHop", "Techno",  "Latin",      "Pure 808" };
    static const char* preset_names_909[5] = { "Classic", "Techno", "House",   "Industrial", "Pure 909" };
    static const char* preset_names_505[5] = { "Classic", "NewWav", "Electro", "LoFi HH",    "Pure 505" };
    const char* const* preset_names[3] = { preset_names_808, preset_names_909, preset_names_505 };

    const int eng_lbl_w = 56;
    const int kit_btn_h = 36;
    const int kit_gap_x = 6;
    const int kit_gap_y = 6;
    const int kit_btn_w = (4 * btn_w + 3 * gap_x - eng_lbl_w - kit_gap_x - 4 * kit_gap_x) / 5;

    for (int eng = 0; eng < 3; eng++) {
        int row_y = kits_y0 + eng * (kit_btn_h + kit_gap_y);

        lv_obj_t* eng_lbl = lv_label_create(card);
        lv_label_set_text(eng_lbl, engine_names[eng]);
        lv_obj_set_style_text_font(eng_lbl, &lv_font_montserrat_18, 0);
        lv_obj_set_style_text_color(eng_lbl, RED808_ACCENT, 0);
        lv_obj_set_pos(eng_lbl, grid_x0, row_y + 6);
        s_pad_inst_modal_kit_lbl_eng[eng] = eng_lbl;

        for (int p = 0; p < 5; p++) {
            bool is_pure = (p == 4);
            lv_obj_t* kb = lv_btn_create(card);
            s_pad_inst_modal_kit_btns[eng][p] = kb;
            lv_obj_set_size(kb, kit_btn_w, kit_btn_h);
            lv_obj_set_pos(kb,
                grid_x0 + eng_lbl_w + kit_gap_x + p * (kit_btn_w + kit_gap_x),
                row_y);
            apply_control_button_style(kb, RED808_BORDER, false, 8);
            if (is_pure) {
                lv_obj_set_style_border_color(kb, RED808_CYAN, 0);
                lv_obj_set_style_border_width(kb, 2, 0);
            }
            lv_obj_t* kl = lv_label_create(kb);
            lv_label_set_text(kl, preset_names[eng][p]);
            lv_obj_set_style_text_font(kl, &lv_font_montserrat_12, 0);
            lv_obj_set_style_text_color(kl, RED808_TEXT, 0);
            lv_obj_center(kl);
            uint32_t key = ((uint32_t)eng << 8) | (uint32_t)p;
            lv_obj_add_event_cb(kb, grid_pad_kit_select_cb, LV_EVENT_CLICKED,
                                (void*)(intptr_t)key);
        }
    }

    // Bottom action row — a single running cursor (all anchored BOTTOM_LEFT)
    // instead of the previous mix of LEFT/MID/RIGHT anchors, which left
    // some gaps as tight as 8px and made the six buttons look like they
    // were touching on real hardware. 20px gaps here guarantee separation
    // regardless of card width, and there's still ~30px of margin on
    // both sides at the card's 984px width.
    int bottomBtnX = 48;
    auto makeBottomBtn = [&](int width, lv_color_t color) -> lv_obj_t* {
        lv_obj_t* btn = lv_btn_create(card);
        lv_obj_set_size(btn, width, 48);
        lv_obj_align(btn, LV_ALIGN_BOTTOM_LEFT, bottomBtnX, -14);
        apply_control_button_style(btn, color, false, 10);
        bottomBtnX += width + 20;
        return btn;
    };

    lv_obj_t* original_btn = makeBottomBtn(170, RED808_ACCENT2);
    lv_obj_t* original_lbl = lv_label_create(original_btn);
    lv_label_set_text(original_lbl, "SAMPLER ORIG.");
    lv_obj_set_style_text_font(original_lbl, &lv_font_montserrat_14, 0);
    lv_obj_center(original_lbl);
    lv_obj_add_event_cb(original_btn, pad_inst_sampler_original_cb, LV_EVENT_CLICKED, NULL);

    // Per-instrument FX page (idea 2): filter + drive/crush + sends, scoped
    // to just the focused pad, with its own RANDOM.
    lv_obj_t* inst_fx_btn = makeBottomBtn(64, RED808_CYAN);
    lv_obj_t* inst_fx_lbl = lv_label_create(inst_fx_btn);
    lv_label_set_text(inst_fx_lbl, "FX");
    lv_obj_set_style_text_font(inst_fx_lbl, &lv_font_montserrat_16, 0);
    lv_obj_center(inst_fx_lbl);
    lv_obj_add_event_cb(inst_fx_btn, pad_fx_modal_show, LV_EVENT_CLICKED, NULL);

    lv_obj_t* preview_btn = makeBottomBtn(160, RED808_SURFACE);
    lv_obj_set_style_border_color(preview_btn, RED808_CYAN, 0);
    lv_obj_set_style_border_width(preview_btn, 2, 0);
    lv_obj_t* preview_lbl = lv_label_create(preview_btn);
    lv_label_set_text(preview_lbl, LV_SYMBOL_PLAY "  PREVIEW");
    lv_obj_set_style_text_font(preview_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(preview_lbl, RED808_TEXT, 0);
    lv_obj_center(preview_lbl);
    lv_obj_add_event_cb(preview_btn, pad_inst_modal_preview_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* assign_btn = makeBottomBtn(170, RED808_SUCCESS);
    lv_obj_set_style_border_color(assign_btn, RED808_CYAN, 0);
    lv_obj_set_style_border_width(assign_btn, 2, 0);
    lv_obj_t* assign_lbl = lv_label_create(assign_btn);
    lv_label_set_text(assign_lbl, LV_SYMBOL_OK "  ASIGNAR");
    lv_obj_set_style_text_font(assign_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(assign_lbl, RED808_TEXT, 0);
    lv_obj_center(assign_lbl);
    lv_obj_add_event_cb(assign_btn, pad_inst_modal_assign_cb, LV_EVENT_CLICKED, NULL);

    // RANDOM: re-roll pad engines across the 16-pad kit (Sampler/808/909/505).
    lv_obj_t* random_btn = makeBottomBtn(130, RED808_INFO);
    lv_obj_t* random_lbl = lv_label_create(random_btn);
    lv_label_set_text(random_lbl, LV_SYMBOL_SHUFFLE "  RANDOM");
    lv_obj_set_style_text_font(random_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(random_lbl, RED808_TEXT, 0);
    lv_obj_center(random_lbl);
    lv_obj_add_event_cb(random_btn, pad_random_all_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* close_btn = makeBottomBtn(110, RED808_BORDER);
    lv_obj_t* close_lbl = lv_label_create(close_btn);
    lv_label_set_text(close_lbl, "CERRAR");
    lv_obj_center(close_lbl);
    lv_obj_add_event_cb(close_btn, pad_inst_modal_close_cb, LV_EVENT_CLICKED, NULL);

    pad_inst_modal_refresh();
}

// =============================================================================
// PER-INSTRUMENT FX modal (idea 2) — filter + drive/crush + sends for a
// single pad, independent from the FX LAB's global chain. Reachable from
// the "FX" button inside the PAD INSTRUMENT SELECT popup above.
// =============================================================================
enum PadFxSliderId : uint8_t { PFX_CUTOFF = 0, PFX_RESO, PFX_DRIVE, PFX_BITS, PFX_RVB, PFX_DLY };

// LADDER (10) used to be listed here but never actually filtered anything:
// this modal drives the per-TRACK filter (control_send_track_filter ->
// trkFilter[track], a plain BiquadEQ), and BiquadEQ::SetType() has no
// FTYPE_LADDER case — it silently fell through to the identity/bypass
// coefficients, so picking "LADDER" just played the track dry with no
// warning. NOTCH and ALLPASS are real BiquadEQ types (same object, same
// CPU cost) that were simply never exposed here.
static const uint8_t PAD_FX_FILTER_TYPES[7] = {0, 1, 2, 3, 4, 5, 9};
static const char* const PAD_FX_FILTER_NAMES[7] = {
    "OFF", "LOWPASS", "HIGHPASS", "BANDPASS", "NOTCH", "ALLPASS", "RESONANT"
};

static PadFxState& pad_fx_state_for(uint8_t pad) {
    if (pad > 15) pad = 15;
    if (!s_pad_fx_state_init[pad]) {
        s_pad_fx_state[pad] = PadFxState{0, 127, 0, 0, 0, 0, 0};
        s_pad_fx_state_init[pad] = true;
    }
    return s_pad_fx_state[pad];
}

static float pad_fx_cutoff_hz(uint8_t u7) {
    return 20.0f * powf(1000.0f, (float)u7 / 127.0f);
}
static float pad_fx_resonance(uint8_t u7) {
    return 0.7f + ((float)u7 / 127.0f) * 19.3f;
}
static uint8_t pad_fx_bits(uint8_t u7) {
    return (uint8_t)constrain((int)(16.0f - ((float)u7 / 127.0f) * 12.0f + 0.5f), 4, 16);
}
static uint8_t pad_fx_percent(uint8_t u7) {
    return (uint8_t)constrain((int)(((float)u7 / 127.0f) * 100.0f + 0.5f), 0, 100);
}

// Sends the pad's current filter (or clears it when OFF). Cutoff/resonance
// only matter while a filter type is selected.
static void pad_fx_send_filter(uint8_t pad) {
    const PadFxState& st = pad_fx_state_for(pad);
    if (st.filterType == 0) {
        control_send_track_clear_filter(pad);
    } else {
        control_send_track_filter(pad, st.filterType,
                                  pad_fx_cutoff_hz(st.cutoffU7),
                                  pad_fx_resonance(st.resoU7));
    }
}

static void pad_fx_modal_refresh(void) {
    if (!s_pad_fx_modal) return;
    uint8_t pad = s_pad_fx_focus_pad;
    if (pad > 15) pad = 15;
    const PadFxState& st = pad_fx_state_for(pad);
    uint8_t inst = s_pad_inst_sel[pad];
    if (inst > 7) inst = 0;

    if (s_pad_fx_subtitle_lbl)
        lv_label_set_text_fmt(s_pad_fx_subtitle_lbl, "PAD %02d  -  %s",
                              (int)pad + 1, PAD_INST_NAMES[inst]);

    for (int i = 0; i < 7; i++) {
        lv_obj_t* btn = s_pad_fx_filter_btns[i];
        if (!btn) continue;
        bool active = (PAD_FX_FILTER_TYPES[i] == st.filterType);
        lv_obj_set_style_bg_color(btn, active ? RED808_ACCENT : RED808_SURFACE, 0);
        lv_obj_set_style_border_color(btn, active ? RED808_CYAN : RED808_BORDER, 0);
        lv_obj_set_style_bg_opa(btn, active ? LV_OPA_COVER : LV_OPA_80, 0);
        lv_obj_set_style_border_width(btn, active ? 2 : 1, 0);
    }

    if (s_pad_fx_cutoff_slider) lv_slider_set_value(s_pad_fx_cutoff_slider, st.cutoffU7, LV_ANIM_OFF);
    if (s_pad_fx_reso_slider)   lv_slider_set_value(s_pad_fx_reso_slider, st.resoU7, LV_ANIM_OFF);
    if (s_pad_fx_drive_slider)  lv_slider_set_value(s_pad_fx_drive_slider, st.driveU7, LV_ANIM_OFF);
    if (s_pad_fx_bits_slider)   lv_slider_set_value(s_pad_fx_bits_slider, st.bitsU7, LV_ANIM_OFF);
    if (s_pad_fx_rvb_slider)    lv_slider_set_value(s_pad_fx_rvb_slider, st.rvbU7, LV_ANIM_OFF);
    if (s_pad_fx_dly_slider)    lv_slider_set_value(s_pad_fx_dly_slider, st.dlyU7, LV_ANIM_OFF);

    if (s_pad_fx_cutoff_lbl) {
        float hz = pad_fx_cutoff_hz(st.cutoffU7);
        if (hz >= 1000.0f) lv_label_set_text_fmt(s_pad_fx_cutoff_lbl, "%d.%01dkHz",
                                                 (int)(hz / 1000.0f),
                                                 ((int)(hz / 100.0f)) % 10);
        else lv_label_set_text_fmt(s_pad_fx_cutoff_lbl, "%dHz", (int)hz);
    }
    if (s_pad_fx_reso_lbl) {
        float q = pad_fx_resonance(st.resoU7);
        lv_label_set_text_fmt(s_pad_fx_reso_lbl, "Q %d.%01d", (int)q, ((int)(q * 10.0f)) % 10);
    }
    if (s_pad_fx_drive_lbl)
        lv_label_set_text_fmt(s_pad_fx_drive_lbl, "%d%%", pad_fx_percent(st.driveU7));
    if (s_pad_fx_bits_lbl)
        lv_label_set_text_fmt(s_pad_fx_bits_lbl, "%d bit", pad_fx_bits(st.bitsU7));
    if (s_pad_fx_rvb_lbl)
        lv_label_set_text_fmt(s_pad_fx_rvb_lbl, "%d%%", pad_fx_percent(st.rvbU7));
    if (s_pad_fx_dly_lbl)
        lv_label_set_text_fmt(s_pad_fx_dly_lbl, "%d%%", pad_fx_percent(st.dlyU7));
}

static void pad_fx_modal_close_cb(lv_event_t* e) {
    if (e && lv_event_get_target(e) != lv_event_get_current_target(e)) return;
    if (s_pad_fx_modal) {
        lv_obj_del(s_pad_fx_modal);
        s_pad_fx_modal = NULL;
        for (int i = 0; i < 7; i++) s_pad_fx_filter_btns[i] = NULL;
        s_pad_fx_cutoff_slider = s_pad_fx_reso_slider = s_pad_fx_drive_slider = NULL;
        s_pad_fx_bits_slider = s_pad_fx_rvb_slider = s_pad_fx_dly_slider = NULL;
        s_pad_fx_cutoff_lbl = s_pad_fx_reso_lbl = s_pad_fx_drive_lbl = NULL;
        s_pad_fx_bits_lbl = s_pad_fx_rvb_lbl = s_pad_fx_dly_lbl = NULL;
        s_pad_fx_subtitle_lbl = NULL;
        s_pad_fx_modal_title = NULL;
    }
}

static void pad_fx_filter_select_cb(lv_event_t* e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (idx < 0 || idx >= 7) return;
    uint8_t pad = s_pad_fx_focus_pad;
    if (!control_available() && !control_engine_connected()) {
        ui_show_toast("Master no conectado", RED808_WARNING);
        return;
    }
    PadFxState& st = pad_fx_state_for(pad);
    st.filterType = PAD_FX_FILTER_TYPES[idx];
    pad_fx_send_filter(pad);
    pad_fx_modal_refresh();
}

static void pad_fx_slider_cb(lv_event_t* e) {
    int id = (int)(intptr_t)lv_event_get_user_data(e);
    lv_obj_t* slider = (lv_obj_t*)lv_event_get_target(e);
    uint8_t u7 = (uint8_t)constrain(lv_slider_get_value(slider), 0, 127);
    uint8_t pad = s_pad_fx_focus_pad;

    static uint32_t lastTxMs = 0;
    uint32_t now = millis();
    bool finalValue = (lv_event_get_code(e) == LV_EVENT_RELEASED ||
                       lv_event_get_code(e) == LV_EVENT_PRESS_LOST);
    bool transmit = finalValue || lastTxMs == 0 || (uint32_t)(now - lastTxMs) >= 30;

    PadFxState& st = pad_fx_state_for(pad);
    switch (id) {
        case PFX_CUTOFF: st.cutoffU7 = u7; break;
        case PFX_RESO:   st.resoU7 = u7;   break;
        case PFX_DRIVE:  st.driveU7 = u7;  break;
        case PFX_BITS:   st.bitsU7 = u7;   break;
        case PFX_RVB:    st.rvbU7 = u7;    break;
        case PFX_DLY:    st.dlyU7 = u7;    break;
        default: return;
    }

    if (!transmit || (!control_available() && !control_engine_connected())) {
        pad_fx_modal_refresh();
        return;
    }
    lastTxMs = now;

    switch (id) {
        case PFX_CUTOFF:
        case PFX_RESO:
            if (st.filterType == 0) st.filterType = 1;  // moving cutoff/reso engages LOWPASS
            pad_fx_send_filter(pad);
            break;
        case PFX_DRIVE:
            control_send_track_distortion(pad, (float)st.driveU7 / 127.0f);
            break;
        case PFX_BITS:
            control_send_track_bitcrush(pad, pad_fx_bits(st.bitsU7));
            break;
        case PFX_RVB:
            control_send_track_reverb_send(pad, pad_fx_percent(st.rvbU7));
            break;
        case PFX_DLY:
            control_send_track_delay_send(pad, pad_fx_percent(st.dlyU7));
            break;
    }
    pad_fx_modal_refresh();
}

// RANDOM FX: tasteful, single-instrument version of the FX LAB randomizer —
// mostly bypassed, occasionally engages a musical filter and a light touch
// of drive/crush/sends. "no siempre", varied, kept subtle.
static void pad_fx_random_cb(lv_event_t* e) {
    LV_UNUSED(e);
    uint8_t pad = s_pad_fx_focus_pad;
    if (!control_available() && !control_engine_connected()) {
        ui_show_toast("Master no conectado", RED808_WARNING);
        return;
    }
    static uint32_t s = 0;
    if (s == 0) s = (uint32_t)millis() ^ 0x1F123BB5u | 1u;
    auto nextRand = [&]() -> uint32_t {
        s ^= s << 13; s ^= s >> 17; s ^= s << 5;
        return s;
    };
    auto randRange = [&](int mn, int mx) -> int {
        if (mx <= mn) return mn;
        return mn + (int)(nextRand() % (uint32_t)(mx - mn + 1));
    };

    PadFxState& st = pad_fx_state_for(pad);

    static const uint8_t filterPool[6] = {0, 0, 0, 1, 2, 9};  // OFF is common
    st.filterType = filterPool[randRange(0, 5)];
    st.cutoffU7 = (st.filterType == 0) ? 127
        : (uint8_t)((randRange(0, 1) == 0) ? 127 : randRange(35, 100));
    st.resoU7 = (st.filterType == 0) ? 0 : (uint8_t)randRange(0, 55);
    pad_fx_send_filter(pad);

    st.driveU7 = (randRange(0, 4) == 0) ? (uint8_t)randRange(15, 55) : 0;
    control_send_track_distortion(pad, (float)st.driveU7 / 127.0f);

    st.bitsU7 = (randRange(0, 5) == 0) ? (uint8_t)randRange(20, 55) : 0;
    control_send_track_bitcrush(pad, pad_fx_bits(st.bitsU7));

    st.rvbU7 = (randRange(0, 2) == 0) ? (uint8_t)randRange(15, 60) : 0;
    control_send_track_reverb_send(pad, pad_fx_percent(st.rvbU7));

    st.dlyU7 = (randRange(0, 3) == 0) ? (uint8_t)randRange(15, 50) : 0;
    control_send_track_delay_send(pad, pad_fx_percent(st.dlyU7));

    pad_fx_modal_refresh();
    ui_show_toast(st.filterType == 0 ? "RANDOM FX: sutil" : "RANDOM FX aplicado",
                  RED808_CYAN);
}

static void pad_fx_clear_cb(lv_event_t* e) {
    LV_UNUSED(e);
    uint8_t pad = s_pad_fx_focus_pad;
    if (!control_available() && !control_engine_connected()) {
        ui_show_toast("Master no conectado", RED808_WARNING);
        return;
    }
    s_pad_fx_state[pad] = PadFxState{0, 127, 0, 0, 0, 0, 0};
    s_pad_fx_state_init[pad] = true;
    control_send_track_clear_fx(pad);
    pad_fx_modal_refresh();
    ui_show_toast("FX del instrumento reseteados", RED808_SUCCESS);
}

// Shared by the "FX" button inside PAD INSTRUMENT SELECT (uses whichever
// pad that modal has focused) and by the per-row FX button on the
// sequencer step grid (uses that row's track directly).
static void pad_fx_modal_show_for_pad(uint8_t pad) {
    if (s_pad_fx_modal) return;
    if (!control_available() && !control_engine_connected()) {
        ui_show_toast("Master no conectado", RED808_WARNING);
        return;
    }
    s_pad_fx_focus_pad = pad > 15 ? 15 : pad;

    s_pad_fx_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_pad_fx_modal, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(s_pad_fx_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_pad_fx_modal, LV_OPA_80, 0);
    lv_obj_set_style_border_width(s_pad_fx_modal, 0, 0);
    lv_obj_set_style_pad_all(s_pad_fx_modal, 0, 0);
    lv_obj_clear_flag(s_pad_fx_modal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_pad_fx_modal, pad_fx_modal_close_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* card = lv_obj_create(s_pad_fx_modal);
    lv_obj_set_size(card, 720, 540);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_color(card, RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_dir(card, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_border_color(card, RED808_CYAN, 0);
    lv_obj_set_style_radius(card, 18, 0);
    lv_obj_set_style_pad_all(card, 14, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);

    s_pad_fx_modal_title = lv_label_create(card);
    lv_label_set_text(s_pad_fx_modal_title, "INSTRUMENT FX");
    lv_obj_set_style_text_font(s_pad_fx_modal_title, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(s_pad_fx_modal_title, RED808_CYAN, 0);
    lv_obj_align(s_pad_fx_modal_title, LV_ALIGN_TOP_MID, 0, 2);

    s_pad_fx_subtitle_lbl = lv_label_create(card);
    lv_obj_set_style_text_font(s_pad_fx_subtitle_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_pad_fx_subtitle_lbl, RED808_TEXT_DIM, 0);
    lv_obj_align(s_pad_fx_subtitle_lbl, LV_ALIGN_TOP_MID, 0, 32);

    lv_obj_t* filter_hdr = lv_label_create(card);
    lv_label_set_text(filter_hdr, "FILTER");
    lv_obj_set_style_text_font(filter_hdr, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(filter_hdr, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(filter_hdr, 4, 58);

    {
        constexpr int btnW = 90, btnH = 40, gapX = 6, y0 = 76;
        for (int i = 0; i < 7; i++) {
            lv_obj_t* fb = lv_btn_create(card);
            s_pad_fx_filter_btns[i] = fb;
            lv_obj_set_size(fb, btnW, btnH);
            lv_obj_set_pos(fb, 4 + i * (btnW + gapX), y0);
            apply_control_button_style(fb, RED808_BORDER, false, 8);
            lv_obj_t* fl = lv_label_create(fb);
            lv_label_set_text(fl, PAD_FX_FILTER_NAMES[i]);
            lv_obj_set_style_text_font(fl, &lv_font_montserrat_12, 0);
            lv_obj_center(fl);
            lv_obj_add_event_cb(fb, pad_fx_filter_select_cb, LV_EVENT_CLICKED,
                                (void*)(intptr_t)i);
        }
    }

    // ── Sliders: label | slider | value readout ──
    auto makeRow = [&](int y, int id, const char* name, lv_obj_t** slider,
                       lv_obj_t** valueLbl) {
        lv_obj_t* nameLbl = lv_label_create(card);
        lv_label_set_text(nameLbl, name);
        lv_obj_set_style_text_font(nameLbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(nameLbl, RED808_TEXT, 0);
        lv_obj_set_pos(nameLbl, 4, y + 4);
        lv_obj_set_width(nameLbl, 112);

        *slider = lv_slider_create(card);
        lv_obj_set_pos(*slider, 122, y + 6);
        lv_obj_set_size(*slider, 470, 16);
        lv_slider_set_range(*slider, 0, 127);
        lv_obj_set_style_bg_color(*slider, RED808_SURFACE, LV_PART_MAIN);
        lv_obj_set_style_bg_color(*slider, RED808_ACCENT, LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(*slider, lv_color_white(), LV_PART_KNOB);
        lv_obj_add_event_cb(*slider, pad_fx_slider_cb, LV_EVENT_VALUE_CHANGED, (void*)(intptr_t)id);
        lv_obj_add_event_cb(*slider, pad_fx_slider_cb, LV_EVENT_RELEASED, (void*)(intptr_t)id);
        lv_obj_add_event_cb(*slider, pad_fx_slider_cb, LV_EVENT_PRESS_LOST, (void*)(intptr_t)id);

        *valueLbl = lv_label_create(card);
        lv_obj_set_style_text_font(*valueLbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(*valueLbl, RED808_ACCENT, 0);
        lv_obj_set_pos(*valueLbl, 606, y + 4);
        lv_obj_set_width(*valueLbl, 96);
    };

    const int rowY0 = 134, rowH = 58;
    makeRow(rowY0 + 0 * rowH, PFX_CUTOFF, "CUTOFF",      &s_pad_fx_cutoff_slider, &s_pad_fx_cutoff_lbl);
    makeRow(rowY0 + 1 * rowH, PFX_RESO,   "RESONANCE",   &s_pad_fx_reso_slider,   &s_pad_fx_reso_lbl);
    makeRow(rowY0 + 2 * rowH, PFX_DRIVE,  "DRIVE",       &s_pad_fx_drive_slider,  &s_pad_fx_drive_lbl);
    makeRow(rowY0 + 3 * rowH, PFX_BITS,   "BITCRUSH",    &s_pad_fx_bits_slider,   &s_pad_fx_bits_lbl);
    makeRow(rowY0 + 4 * rowH, PFX_RVB,    "REVERB SEND", &s_pad_fx_rvb_slider,    &s_pad_fx_rvb_lbl);
    makeRow(rowY0 + 5 * rowH, PFX_DLY,    "DELAY SEND",  &s_pad_fx_dly_slider,    &s_pad_fx_dly_lbl);

    lv_obj_t* random_btn = lv_btn_create(card);
    lv_obj_set_size(random_btn, 160, 48);
    lv_obj_align(random_btn, LV_ALIGN_BOTTOM_LEFT, 4, -14);
    apply_control_button_style(random_btn, RED808_INFO, false, 10);
    lv_obj_t* random_lbl = lv_label_create(random_btn);
    lv_label_set_text(random_lbl, LV_SYMBOL_SHUFFLE "  RANDOM FX");
    lv_obj_set_style_text_font(random_lbl, &lv_font_montserrat_14, 0);
    lv_obj_center(random_lbl);
    lv_obj_add_event_cb(random_btn, pad_fx_random_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* clear_btn = lv_btn_create(card);
    lv_obj_set_size(clear_btn, 140, 48);
    lv_obj_align(clear_btn, LV_ALIGN_BOTTOM_MID, 0, -14);
    apply_control_button_style(clear_btn, RED808_ERROR, false, 10);
    lv_obj_t* clear_lbl = lv_label_create(clear_btn);
    lv_label_set_text(clear_lbl, "CLEAR FX");
    lv_obj_set_style_text_font(clear_lbl, &lv_font_montserrat_14, 0);
    lv_obj_center(clear_lbl);
    lv_obj_add_event_cb(clear_btn, pad_fx_clear_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* close_btn2 = lv_btn_create(card);
    lv_obj_set_size(close_btn2, 110, 48);
    lv_obj_align(close_btn2, LV_ALIGN_BOTTOM_RIGHT, -4, -14);
    apply_control_button_style(close_btn2, RED808_BORDER, false, 10);
    lv_obj_t* close_lbl2 = lv_label_create(close_btn2);
    lv_label_set_text(close_lbl2, "CERRAR");
    lv_obj_center(close_lbl2);
    lv_obj_add_event_cb(close_btn2, pad_fx_modal_close_cb, LV_EVENT_CLICKED, NULL);

    pad_fx_modal_refresh();
}

// Entry point from the "FX" button inside PAD INSTRUMENT SELECT: uses
// whichever pad that popup currently has focused.
static void pad_fx_modal_show(lv_event_t* e) {
    LV_UNUSED(e);
    pad_fx_modal_show_for_pad(s_pad_inst_focus_pad);
}

// Entry point from the per-row FX button on the sequencer step grid:
// user_data carries the row's track index directly.
static void seq_row_fx_cb(lv_event_t* e) {
    int track = (int)(intptr_t)lv_event_get_user_data(e);
    if (track < 0 || track >= 16) return;
    pad_fx_modal_show_for_pad((uint8_t)track);
}

// Quick "X" button per row: clears that track's instrument FX (filter,
// drive, bitcrush, sends) without opening the panel first.
static void seq_row_fx_clear_cb(lv_event_t* e) {
    int track = (int)(intptr_t)lv_event_get_user_data(e);
    if (track < 0 || track >= 16) return;
    if (!control_available() && !control_engine_connected()) {
        ui_show_toast("Master no conectado", RED808_WARNING);
        return;
    }
    s_pad_fx_state[track] = PadFxState{0, 127, 0, 0, 0, 0, 0};
    s_pad_fx_state_init[track] = true;
    control_send_track_clear_fx((uint8_t)track);
    if (s_pad_fx_modal && s_pad_fx_focus_pad == track) pad_fx_modal_refresh();
    ui_show_toast("FX de la pista borrados", RED808_SUCCESS);
}

// Helper: styled control button
static lv_obj_t* create_ctrl_btn(lv_obj_t* parent, int x, int y, int w, int h,
                                  const char* text, lv_color_t border_color,
                                  const lv_font_t* font) {
    lv_obj_t* btn = lv_btn_create(parent);
    lv_obj_set_size(btn, w, h);
    lv_obj_set_pos(btn, x, y);
    lv_obj_set_style_radius(btn, 14, 0);
    lv_obj_set_style_bg_color(btn, RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_color(btn, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_dir(btn, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(btn, 2, 0);
    lv_obj_set_style_border_color(btn, border_color, 0);
    lv_obj_set_style_shadow_width(btn, 0, 0);
    lv_obj_set_style_bg_color(btn, border_color, LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(btn, LV_OPA_80, LV_STATE_PRESSED);

    lv_obj_t* label = lv_label_create(btn);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, RED808_TEXT, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(label);
    return btn;
}

// Helper: info display cell (non-clickable)
static lv_obj_t* create_info_cell(lv_obj_t* parent, int x, int y, int w, int h,
                                   const char* title, const char* value,
                                   lv_color_t value_color, lv_obj_t** value_out) {
    lv_obj_t* panel = lv_obj_create(parent);
    lv_obj_set_size(panel, w, h);
    lv_obj_set_pos(panel, x, y);
    lv_obj_set_style_radius(panel, 14, 0);
    lv_obj_set_style_bg_color(panel, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_color(panel, RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_dir(panel, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(panel, 1, 0);
    lv_obj_set_style_border_color(panel, RED808_BORDER, 0);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* t = lv_label_create(panel);
    lv_label_set_text(t, title);
    lv_obj_set_style_text_font(t, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(t, RED808_TEXT_DIM, 0);
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 4);

    lv_obj_t* v = lv_label_create(panel);
    lv_label_set_text(v, value);
    lv_obj_set_style_text_font(v, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(v, value_color, 0);
    lv_obj_align(v, LV_ALIGN_CENTER, 0, 10);

    if (value_out) *value_out = v;
    return panel;
}

static const char* pod_control_function_name(uint8_t function) {
    static const char* names[POD_FUNC_COUNT] = {
        "NONE", "PLAY / PAUSE", "STOP + RESET", "TRIGGER PAD",
        "PATTERN -", "PATTERN +", "MASTER VOL", "SEQ VOL",
        "LIVE VOL", "TEMPO", "SELECT PAD", "BACK", "MIXER", "FX",
        "SEQUENCER", "PAD GRID", "PAD SOUNDS", "XTRA PADS",
        "DELAY MIX", "REVERB MIX", "BUTTON CONFIG", "DISPLAY BRIGHTNESS",
        "FLANGER DEPTH", "WAVEFOLDER", "CRUSH MACRO", "PHASER DEPTH",
        "FILTER CUTOFF", "FILTER RESONANCE", "DISTORTION", "BIT DEPTH",
        "SAMPLE RATE", "FILTER TYPE"
    };
    return function < POD_FUNC_COUNT ? names[function] : "NONE";
}

static const char* pod_led_function_name(uint8_t function) {
    static const char* names[POD_LED_COUNT] = {
        "FIXED", "USB LINK", "PLAY STATE", "PAD ACTIVITY",
        "DAISY SD", "SAMPLES + PATTERN"
    };
    return function < POD_LED_COUNT ? names[function] : "FIXED";
}

// Small "HW" badges make mechanical ownership visible without changing the
// established color language of each screen cell.
struct PodOwnerBadge {
    lv_obj_t* badge;
    uint8_t function;
    bool visible;
};
static constexpr uint8_t POD_OWNER_BADGE_MAX = 40;
static PodOwnerBadge s_pod_owner_badges[POD_OWNER_BADGE_MAX] = {};
static uint8_t s_pod_owner_badge_count = 0;

static bool pod_control_functions_conflict(uint8_t left, uint8_t right) {
    if (left == POD_FUNC_NONE || right == POD_FUNC_NONE) return false;
    if (left == right) return true;
    const bool leftPattern = left == POD_FUNC_PATTERN_PREV
                          || left == POD_FUNC_PATTERN_NEXT;
    const bool rightPattern = right == POD_FUNC_PATTERN_PREV
                           || right == POD_FUNC_PATTERN_NEXT;
    if (leftPattern && rightPattern) return true;
    const bool leftCrush = left == POD_FUNC_CRUSH_MACRO;
    const bool rightCrush = right == POD_FUNC_CRUSH_MACRO;
    return (leftCrush && (right == POD_FUNC_BIT_DEPTH
                          || right == POD_FUNC_SAMPLE_RATE))
        || (rightCrush && (left == POD_FUNC_BIT_DEPTH
                           || left == POD_FUNC_SAMPLE_RATE));
}

static bool pod_function_has_physical_owner(uint8_t function) {
    if (function == POD_FUNC_NONE) return false;
    const auto& podState = daisyUsb.state().pod;
    const PodConfigPayload* config = podState.config.version == POD_CONFIG_VERSION
        ? &podState.config
        : (s_pod_config.version == POD_CONFIG_VERSION ? &s_pod_config : NULL);
    if (!config) return false;
    const uint8_t podAssigned[] = {
        config->button1Function, config->button2Function,
        config->knob1Function, config->knob2Function,
        config->encoderFunction, config->encoderButtonFunction
    };
    for (uint8_t value : podAssigned)
        if (pod_control_functions_conflict(value, function)) return true;

    const uint8_t rotaryAssigned[] = {
        config->rotary1Function, config->rotary2Function,
        config->rotary3Function, config->rotary4Function
    };
    const uint8_t rotaryMask = i2c_rotaries_detected_mask();
    for (uint8_t index = 0; index < 4; ++index)
        if ((rotaryMask & (1u << index))
            && pod_control_functions_conflict(rotaryAssigned[index], function))
            return true;
    if (p4_fader_detected()
        && pod_control_functions_conflict(config->faderFunction, function))
        return true;
    return false;
}

static void pod_register_owner_badge(lv_obj_t* parent, uint8_t function) {
    if (!parent || function == POD_FUNC_NONE
        || s_pod_owner_badge_count >= POD_OWNER_BADGE_MAX) return;
    PodOwnerBadge& entry = s_pod_owner_badges[s_pod_owner_badge_count++];
    entry.function = function;
    entry.visible = false;
    entry.badge = lv_label_create(parent);
    lv_label_set_text(entry.badge, "HW");
    lv_obj_set_style_text_font(entry.badge, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(entry.badge, lv_color_black(), 0);
    lv_obj_set_style_bg_color(entry.badge, RED808_CYAN, 0);
    lv_obj_set_style_bg_opa(entry.badge, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(entry.badge, 4, 0);
    lv_obj_set_style_pad_hor(entry.badge, 4, 0);
    lv_obj_set_style_pad_ver(entry.badge, 2, 0);
    lv_obj_align(entry.badge, LV_ALIGN_TOP_RIGHT, -5, 5);
    lv_obj_clear_flag(entry.badge, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(entry.badge, LV_OBJ_FLAG_HIDDEN);
}

static void pod_owner_badges_update(void) {
    for (uint8_t i = 0; i < s_pod_owner_badge_count; ++i) {
        PodOwnerBadge& entry = s_pod_owner_badges[i];
        const bool visible = pod_function_has_physical_owner(entry.function);
        if (visible == entry.visible || !entry.badge) continue;
        entry.visible = visible;
        if (visible) {
            lv_obj_clear_flag(entry.badge, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(entry.badge);
        } else {
            lv_obj_add_flag(entry.badge, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static const char* sd_diag_stage_name(uint8_t stage) {
    static const char* names[] = {
        "OK", "SPI INIT", "CLOCKS", "CMD0", "CMD8", "ACMD41",
        "CMD58", "CMD16", "CMD17", "DATA TOKEN", "READY", "SPI IO"
    };
    return stage < (sizeof(names) / sizeof(names[0])) ? names[stage] : "UNKNOWN";
}

struct PodPaletteColor { const char* name; uint8_t r, g, b; };
static const PodPaletteColor POD_LED_PALETTE[] = {
    {"CYAN", 0, 180, 255}, {"GREEN", 0, 255, 90},
    {"AMBER", 255, 145, 0}, {"RED", 255, 24, 12},
    {"MAGENTA", 255, 0, 180}, {"BLUE", 40, 70, 255},
    {"WHITE", 255, 255, 255}
};

static uint8_t* pod_control_config_field(uint8_t row) {
    switch(row) {
        case 0: return &s_pod_config.button1Function;
        case 1: return &s_pod_config.button2Function;
        case 2: return &s_pod_config.knob1Function;
        case 3: return &s_pod_config.knob2Function;
        case 4: return &s_pod_config.encoderFunction;
        case 5: return &s_pod_config.encoderButtonFunction;
        case 6: return &s_pod_config.rotary1Function;
        case 7: return &s_pod_config.rotary2Function;
        case 8: return &s_pod_config.rotary3Function;
        case 9: return &s_pod_config.rotary4Function;
        case 10: return &s_pod_config.faderFunction;
        default: return NULL;
    }
}

static bool pod_control_function_used_by_other(uint8_t row, uint8_t function) {
    if (function == POD_FUNC_NONE) return false;
    for (uint8_t other = 0; other < POD_CONTROL_ROW_COUNT; ++other) {
        if (other == row) continue;
        uint8_t* otherField = pod_control_config_field(other);
        if (otherField
            && pod_control_functions_conflict(*otherField, function)) return true;
    }
    return false;
}

static void pod_control_function_list(uint8_t row, const uint8_t*& list,
                                      size_t& count) {
    static const uint8_t buttonFunctions[] = {
        POD_FUNC_NONE, POD_FUNC_PLAY_TOGGLE, POD_FUNC_STOP,
        POD_FUNC_TRIGGER_SELECTED, POD_FUNC_PATTERN_PREV, POD_FUNC_PATTERN_NEXT,
        POD_FUNC_BACK, POD_FUNC_MIXER, POD_FUNC_FX, POD_FUNC_SEQUENCER,
        POD_FUNC_PAD_GRID, POD_FUNC_PAD_SOUNDS, POD_FUNC_XTRA_PADS,
        POD_FUNC_CONTROL_CONFIG
    };
    static const uint8_t absoluteFunctions[] = {
        POD_FUNC_NONE, POD_FUNC_MASTER_VOLUME, POD_FUNC_SEQ_VOLUME,
        POD_FUNC_LIVE_VOLUME, POD_FUNC_TEMPO, POD_FUNC_SELECT_PAD,
        POD_FUNC_FLANGER_DEPTH, POD_FUNC_DELAY_MIX, POD_FUNC_REVERB_MIX,
        POD_FUNC_WAVEFOLDER_GAIN, POD_FUNC_CRUSH_MACRO,
        POD_FUNC_PHASER_DEPTH, POD_FUNC_FILTER_CUTOFF,
        POD_FUNC_FILTER_RESONANCE, POD_FUNC_DISTORTION,
        POD_FUNC_BIT_DEPTH, POD_FUNC_SAMPLE_RATE, POD_FUNC_FILTER_TYPE,
        POD_FUNC_SCREEN_BRIGHTNESS
    };
    static const uint8_t encoderFunctions[] = {
        POD_FUNC_NONE, POD_FUNC_PATTERN_NEXT, POD_FUNC_SELECT_PAD,
        POD_FUNC_TEMPO, POD_FUNC_MASTER_VOLUME, POD_FUNC_SEQ_VOLUME,
        POD_FUNC_LIVE_VOLUME, POD_FUNC_FLANGER_DEPTH, POD_FUNC_DELAY_MIX,
        POD_FUNC_REVERB_MIX, POD_FUNC_WAVEFOLDER_GAIN,
        POD_FUNC_CRUSH_MACRO, POD_FUNC_PHASER_DEPTH,
        POD_FUNC_FILTER_CUTOFF, POD_FUNC_FILTER_RESONANCE,
        POD_FUNC_DISTORTION, POD_FUNC_BIT_DEPTH, POD_FUNC_SAMPLE_RATE,
        POD_FUNC_FILTER_TYPE
    };
    if (row == 4) {
        list = encoderFunctions;
        count = sizeof(encoderFunctions);
    } else if (row == 0 || row == 1 || row == 5) {
        list = buttonFunctions;
        count = sizeof(buttonFunctions);
    } else {
        list = absoluteFunctions;
        count = sizeof(absoluteFunctions);
    }
}

static void pod_control_value_refresh(uint8_t row) {
    if (row >= POD_CONTROL_ROW_COUNT || !s_pod_control_value_labels[row]) return;
    uint8_t* field = pod_control_config_field(row);
    if (!field) return;
    const bool patternEncoder = row == 4
        && (*field == POD_FUNC_PATTERN_PREV || *field == POD_FUNC_PATTERN_NEXT);
    lv_label_set_text(s_pod_control_value_labels[row], patternEncoder
        ? "PATTERN +/-" : pod_control_function_name(*field));
}

static void pod_status_modal_refresh(void) {
    if (!s_pod_status_modal) return;
    for (uint8_t row = 0; row < POD_CONTROL_ROW_COUNT; row++)
        pod_control_value_refresh(row);
    const uint8_t ledFunctions[2] = {
        s_pod_config.led1Function, s_pod_config.led2Function
    };
    const uint8_t colors[2][3] = {
        {s_pod_config.led1R, s_pod_config.led1G, s_pod_config.led1B},
        {s_pod_config.led2R, s_pod_config.led2G, s_pod_config.led2B}
    };
    for (uint8_t led = 0; led < 2; led++) {
        if (s_pod_led_function_labels[led])
            lv_label_set_text(s_pod_led_function_labels[led],
                              pod_led_function_name(ledFunctions[led]));
        if (s_pod_led_color_labels[led]) {
            const char* colorName = "CUSTOM";
            for (const auto& color : POD_LED_PALETTE)
                if (color.r == colors[led][0] && color.g == colors[led][1]
                    && color.b == colors[led][2]) colorName = color.name;
            lv_label_set_text(s_pod_led_color_labels[led], colorName);
            lv_obj_set_style_text_color(s_pod_led_color_labels[led],
                lv_color_make(colors[led][0], colors[led][1], colors[led][2]), 0);
        }
    }
}

static void pod_send_config(void) {
    pod_config_store_sanitize(s_pod_config);
    if (!pod_config_store_save(s_pod_config))
        ui_show_toast("No se pudo guardar controles", RED808_WARNING);
    if (!daisyUsb.send(CMD_POD_SET_CONFIG, &s_pod_config, sizeof(s_pod_config)))
        ui_show_toast("Daisy USB no disponible", RED808_WARNING);
}

static void pod_function_modal_close_cb(lv_event_t* e) {
    if (e && lv_event_get_target(e) != lv_event_get_current_target(e)) return;
    if (s_pod_function_modal) lv_obj_del(s_pod_function_modal);
    s_pod_function_modal = NULL;
    s_pod_function_modal_row = 0xFF;
}

static void pod_function_select_cb(lv_event_t* e) {
    const uint16_t packed = static_cast<uint16_t>(
        reinterpret_cast<uintptr_t>(lv_event_get_user_data(e)));
    const uint8_t row = static_cast<uint8_t>(packed >> 8);
    const uint8_t function = static_cast<uint8_t>(packed & 0xFFu);
    uint8_t* field = pod_control_config_field(row);
    if (!field || function >= POD_FUNC_COUNT) return;
    if (pod_control_function_used_by_other(row, function)) {
        ui_show_toast("Funcion ya asignada: quitala primero", RED808_WARNING);
        return;
    }
    if (*field != function) {
        *field = function;
        pod_send_config();
        pod_status_modal_refresh();
    }
    pod_function_modal_close_cb(NULL);
}

static void pod_control_modal_open_cb(lv_event_t* e) {
    const uint8_t row = static_cast<uint8_t>(
        reinterpret_cast<uintptr_t>(lv_event_get_user_data(e)));
    if (row >= POD_CONTROL_ROW_COUNT || !s_pod_status_modal) return;
    if (s_pod_function_modal) pod_function_modal_close_cb(NULL);
    uint8_t* field = pod_control_config_field(row);
    if (!field) return;

    s_pod_function_modal_row = row;
    s_pod_function_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_pod_function_modal, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(s_pod_function_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_pod_function_modal, LV_OPA_80, 0);
    lv_obj_set_style_border_width(s_pod_function_modal, 0, 0);
    lv_obj_set_style_pad_all(s_pod_function_modal, 0, 0);
    lv_obj_clear_flag(s_pod_function_modal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_pod_function_modal, pod_function_modal_close_cb,
                        LV_EVENT_CLICKED, NULL);

    lv_obj_t* card = lv_obj_create(s_pod_function_modal);
    lv_obj_set_size(card, 930, 520);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_color(card, RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_dir(card, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_border_color(card, RED808_CYAN, 0);
    lv_obj_set_style_radius(card, 16, 0);
    lv_obj_set_style_pad_all(card, 0, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t* title = lv_label_create(card);
    lv_label_set_text_fmt(title, "ASIGNAR  %s", POD_CONTROL_TITLES[row]);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title, RED808_CYAN, 0);
    lv_obj_set_pos(title, 22, 18);
    lv_obj_t* hint = lv_label_create(card);
    lv_label_set_text(hint, "Todas las opciones compatibles. IN USE requiere liberar el otro control.");
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(hint, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(hint, 22, 50);

    lv_obj_t* close = lv_btn_create(card);
    lv_obj_set_size(close, 92, 38);
    lv_obj_set_pos(close, 816, 16);
    apply_control_button_style(close, RED808_WARNING, false, 9);
    lv_obj_t* closeLabel = lv_label_create(close);
    lv_label_set_text(closeLabel, "CANCEL");
    lv_obj_set_style_text_font(closeLabel, &lv_font_montserrat_12, 0);
    lv_obj_center(closeLabel);
    lv_obj_add_event_cb(close, [](lv_event_t*) {
        pod_function_modal_close_cb(NULL);
    }, LV_EVENT_CLICKED, NULL);

    const uint8_t* functions = NULL;
    size_t functionCount = 0;
    pod_control_function_list(row, functions, functionCount);
    constexpr int columns = 5;
    constexpr int buttonWidth = 170;
    constexpr int buttonHeight = 62;
    constexpr int gapX = 10;
    constexpr int gapY = 14;
    for (size_t option = 0; option < functionCount; ++option) {
        const uint8_t function = functions[option];
        const bool selected = function == *field;
        const bool used = !selected
            && pod_control_function_used_by_other(row, function);
        const int column = static_cast<int>(option % columns);
        const int line = static_cast<int>(option / columns);
        lv_obj_t* button = lv_btn_create(card);
        lv_obj_set_size(button, buttonWidth, buttonHeight);
        lv_obj_set_pos(button, 20 + column * (buttonWidth + gapX),
                       86 + line * (buttonHeight + gapY));
        apply_control_button_style(button,
            selected ? RED808_CYAN : RED808_ACCENT2, false, 11);
        lv_obj_set_style_bg_color(button,
            selected ? RED808_ACCENT : RED808_SURFACE, 0);
        lv_obj_set_style_bg_opa(button, selected ? LV_OPA_COVER : LV_OPA_80, 0);
        if (selected) lv_obj_set_style_border_width(button, 3, 0);

        lv_obj_t* label = lv_label_create(button);
        lv_label_set_text(label, pod_control_function_name(function));
        lv_obj_set_width(label, buttonWidth - 14);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(label, selected ? lv_color_white()
                                                       : RED808_TEXT, 0);
        lv_obj_align(label, LV_ALIGN_CENTER, 0, (selected || used) ? -7 : 0);

        if (selected || used) {
            lv_obj_t* state = lv_label_create(button);
            lv_label_set_text(state, selected ? "SELECTED" : "IN USE");
            lv_obj_set_style_text_font(state, &lv_font_montserrat_10, 0);
            lv_obj_set_style_text_color(state,
                selected ? lv_color_white() : RED808_TEXT_DIM, 0);
            lv_obj_align(state, LV_ALIGN_BOTTOM_MID, 0, -5);
        }
        if (used) {
            lv_obj_add_state(button, LV_STATE_DISABLED);
            lv_obj_set_style_bg_opa(button, LV_OPA_30, LV_STATE_DISABLED);
            lv_obj_set_style_border_color(button, RED808_BORDER,
                                          LV_STATE_DISABLED);
        } else {
            const uint16_t packed = static_cast<uint16_t>(
                (static_cast<uint16_t>(row) << 8) | function);
            lv_obj_add_event_cb(button, pod_function_select_cb,
                                LV_EVENT_CLICKED,
                                reinterpret_cast<void*>(
                                    static_cast<uintptr_t>(packed)));
        }
    }
}

static void pod_led_function_cycle_cb(lv_event_t* e) {
    uint8_t led = static_cast<uint8_t>((intptr_t)lv_event_get_user_data(e));
    uint8_t* field = led == 0 ? &s_pod_config.led1Function : &s_pod_config.led2Function;
    *field = static_cast<uint8_t>((*field + 1u) % POD_LED_COUNT);
    pod_status_modal_refresh();
    pod_send_config();
}

static void pod_led_color_cycle_cb(lv_event_t* e) {
    uint8_t led = static_cast<uint8_t>((intptr_t)lv_event_get_user_data(e));
    uint8_t* r = led == 0 ? &s_pod_config.led1R : &s_pod_config.led2R;
    uint8_t* g = led == 0 ? &s_pod_config.led1G : &s_pod_config.led2G;
    uint8_t* b = led == 0 ? &s_pod_config.led1B : &s_pod_config.led2B;
    size_t next = 0;
    for (size_t i = 0; i < sizeof(POD_LED_PALETTE) / sizeof(POD_LED_PALETTE[0]); i++)
        if (*r == POD_LED_PALETTE[i].r && *g == POD_LED_PALETTE[i].g
            && *b == POD_LED_PALETTE[i].b)
            next = (i + 1) % (sizeof(POD_LED_PALETTE) / sizeof(POD_LED_PALETTE[0]));
    *r = POD_LED_PALETTE[next].r;
    *g = POD_LED_PALETTE[next].g;
    *b = POD_LED_PALETTE[next].b;
    pod_status_modal_refresh();
    pod_send_config();
}

// =============================================================================
// AKAI MPD218 MIDI MAP — device-style view + MIDI LEARN
// The layout mirrors the physical MPD218: 6 knobs (rows 5·6 / 3·4 / 1·2) on
// the left, the 4x4 pad matrix on the right (PAD 13..16 on top like the
// hardware), PROG / PAD BANK / CTRL BANK cycle buttons and a red LEARN key.
// LEARN captures the next note/CC polled from DaisyPod3's MIDI monitor and
// opens the assignment picker; assignments persist in P4 NVS and are pushed
// to Daisy so the trigger path stays on the audio engine.
// =============================================================================

// Fixed device-replica palette: the modal deliberately looks like the black
// MPD218 regardless of the active UI theme. Selection/learned accents come
// from the theme so they stay legible on every preset.
#define MPD_BODY_BG      lv_color_hex(0x0D0D0D)
#define MPD_PANEL_BG     lv_color_hex(0x161616)
#define MPD_PAD_BG       lv_color_hex(0x050505)
#define MPD_PAD_RIM      lv_color_hex(0xB61A1A)
#define MPD_PAD_GLOW     lv_color_hex(0xFF3B2F)
#define MPD_TEXT_MAIN    lv_color_hex(0xEDEDED)
#define MPD_TEXT_FAINT   lv_color_hex(0x8F8F8F)
#define MPD_LEARN_RED    lv_color_hex(0xD42B1E)
#define MPD_LEARNED_MARK lv_color_hex(0x35C8FF)

static const char* const MPD_BANK_NAMES[red808_mpd218::kBankCount] = {
    "LIVE / MASTER", "SEQUENCER", "SYNTHS"
};

static const char* const MPD_DRUM_PAD_NAMES[3][16] = {
    {"KICK", "SNARE", "CLOSED HH", "OPEN HH", "CYMBAL", "CLAP",
     "RIMSHOT", "COWBELL", "LOW TOM", "MID TOM", "HIGH TOM", "MARACAS",
     "CLAVES", "HIGH CONGA", "MID CONGA", "LOW CONGA"},
    {"KICK", "SNARE", "CLOSED HH", "OPEN HH", "CRASH", "CLAP",
     "RIMSHOT", "RIDE", "LOW TOM", "MID TOM", "HIGH TOM", "SHAKER",
     "CLAVE", "HIGH PERC", "MID PERC", "LOW PERC"},
    {"KICK", "SNARE", "CLOSED HH", "OPEN HH", "CYMBAL", "CLAP",
     "RIMSHOT", "COWBELL", "LOW TOM", "MID TOM", "HIGH TOM", "SHAKER",
     "CLAVE", "HIGH PERC", "MID PERC", "LOW PERC"}
};

static const char* const MPD_KNOB_ACTION_NAMES[] = {
    "NONE",
    "MASTER VOLUME", "LIVE VOLUME", "SEQ VOLUME", "TEMPO",
    "DELAY MIX", "REVERB MIX", "FILTER CUTOFF", "FILTER RESONANCE",
    "DISTORTION", "BIT DEPTH", "SAMPLE RATE", "FILTER TYPE",
    "FLANGER DEPTH", "PHASER DEPTH", "WAVEFOLDER", "CRUSH MACRO",
    "LIVE PITCH", "SELECT PAD",
    "TRACK VOLUME", "TRACK PAN", "TRACK REVERB", "TRACK DELAY",
    "TRACK CHORUS", "TRACK PITCH", "TRACK FILTER", "TRACK RESONANCE",
    "TRACK DISTORT", "TRACK BIT DEPTH", "TRACK EQ LOW", "TRACK EQ HIGH",
    "SEQ TEMPO", "SEQ SWING", "HUMANIZE TIME", "HUMANIZE VEL",
    "SELECT PATTERN", "SELECT TRACK",
    "DRUM DECAY", "DRUM PITCH", "DRUM TONE", "DRUM VOLUME",
    "DRUM SNAPPY", "DRUM PUNCH",
    "MELODIC PARAM 1", "MELODIC PARAM 2", "MELODIC PARAM 3",
    "MELODIC PARAM 4", "MELODIC PARAM 5", "MELODIC PARAM 6",
    "SYNTH ENGINE", "SYNTH PRESET", "CHORUS MIX", "TREMOLO DEPTH",
    "AUTO-WAH MIX", "STEREO WIDTH"
};

static_assert(sizeof(MPD_KNOB_ACTION_NAMES) / sizeof(MPD_KNOB_ACTION_NAMES[0])
              == red808_mpd218::KNOB_STEREO_WIDTH + 1,
              "MPD knob labels must follow KnobActionType");

static const uint8_t MPD_TRANSPORT_ACTIONS[8] = {
    red808_mpd218::PAD_PLAY_TOGGLE, red808_mpd218::PAD_STOP_ALL,
    red808_mpd218::PAD_PATTERN_PREV, red808_mpd218::PAD_PATTERN_NEXT,
    red808_mpd218::PAD_TOGGLE_LOOP, red808_mpd218::PAD_TOGGLE_REVERSE,
    red808_mpd218::PAD_TOGGLE_STUTTER, red808_mpd218::PAD_CLEAR_SELECTED_FX
};
static const char* const MPD_TRANSPORT_NAMES[8] = {
    "PLAY / PAUSE", "STOP + PANIC", "PATTERN -", "PATTERN +",
    "PAD LOOP", "PAD REVERSE", "PAD STUTTER", "CLEAR PAD FX"
};

static const char* const MPD_ASSIGN_CATEGORIES[7] = {
    "SAMPLES", "TR-808", "TR-909", "TR-505", "PATRON", "MUTE", "TRANSP."
};

static void mpd_note_name(uint8_t note, char* out, size_t outSize) {
    static const char* const names[12] = {
        "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
    };
    snprintf(out, outSize, "%s%d", names[note % 12], (int)(note / 12) - 1);
}

static void mpd_pad_action_text(uint8_t type, uint8_t arg0, uint8_t arg1,
                                char* out, size_t outSize) {
    using namespace red808_mpd218;
    switch (type) {
        case PAD_TRIGGER_SAMPLE: {
            // This action triggers whatever sound the P4 pad currently
            // holds — show that live, not a static "SAMPLE NN": if PAD
            // SOUND on HOME reassigns the pad, the MIDI MAP view (and any
            // AKAI key mapped to it) should read that change immediately.
            // arg0 also covers the 8 XTRA pads (16..23), which only ever
            // hold a sample — s_pad_inst_sel[]/MPD_DRUM_PAD_NAMES[] cover
            // just the 16 main pads, so only consult them below that.
            const uint8_t pad = arg0 < 24 ? arg0 : 0;
            const uint8_t inst = pad < 16 ? s_pad_inst_sel[pad] : 0;
            if (inst == 0) { // Sampler (or an XTRA pad)
                if (p4.sample_name[pad][0] != '\0')
                    snprintf(out, outSize, "S%02u %s", pad + 1u,
                             p4.sample_name[pad]);
                else
                    snprintf(out, outSize, "SAMPLE %02u", pad + 1u);
            } else if (inst >= 1 && inst <= 3) { // 808/909/505
                static const char* const engines[3] = {"808", "909", "505"};
                snprintf(out, outSize, "%s %s", engines[inst - 1],
                         MPD_DRUM_PAD_NAMES[inst - 1][pad]);
            } else if (inst < 8) { // melodic: 303/WT/FM2/SH101
                snprintf(out, outSize, "%s MELODIC", PAD_INST_SHORT[inst]);
            } else {
                snprintf(out, outSize, "SAMPLE %02u", pad + 1u);
            }
            break;
        }
        case PAD_TRIGGER_MELODIC: {
            char note[6] = {};
            mpd_note_name(arg0, note, sizeof(note));
            snprintf(out, outSize, "MELODIC %s", note);
            break;
        }
        case PAD_SELECT_PATTERN:
            snprintf(out, outSize, "PATTERN %02u", (unsigned)arg0 + 1u);
            break;
        case PAD_TOGGLE_TRACK_MUTE:
            snprintf(out, outSize, "MUTE TRK %02u", (unsigned)arg0 + 1u);
            break;
        case PAD_SELECT_TRACK:
            snprintf(out, outSize, "SEL TRK %02u", (unsigned)arg0 + 1u);
            break;
        case PAD_TRIGGER_SYNTH: {
            const uint8_t engine = arg0 < 3 ? arg0 : 0;
            const uint8_t pad = arg1 < 16 ? arg1 : 0;
            static const char* const engines[3] = {"808", "909", "505"};
            snprintf(out, outSize, "%s %s", engines[engine],
                     MPD_DRUM_PAD_NAMES[engine][pad]);
            break;
        }
        case PAD_PLAY_TOGGLE:       snprintf(out, outSize, "PLAY / PAUSE"); break;
        case PAD_STOP_ALL:          snprintf(out, outSize, "STOP + PANIC"); break;
        case PAD_PATTERN_PREV:      snprintf(out, outSize, "PATTERN -"); break;
        case PAD_PATTERN_NEXT:      snprintf(out, outSize, "PATTERN +"); break;
        case PAD_TOGGLE_LOOP:       snprintf(out, outSize, "PAD LOOP"); break;
        case PAD_TOGGLE_REVERSE:    snprintf(out, outSize, "PAD REVERSE"); break;
        case PAD_TOGGLE_STUTTER:    snprintf(out, outSize, "PAD STUTTER"); break;
        case PAD_CLEAR_SELECTED_FX: snprintf(out, outSize, "CLEAR PAD FX"); break;
        default:                    snprintf(out, outSize, "NONE"); break;
    }
}

// True if any learned MIDI note maps to triggering this P4 pad (0..15).
static bool pad_has_midi_mapping(uint8_t pad) {
    using namespace red808_mpd218;
    const uint8_t count = control_midi_map_count();
    for (uint8_t i = 0; i < count; i++) {
        MidiMapEntry entry;
        if (!control_midi_map_get(i, entry)) continue;
        if (entry.kind != MIDI_MAP_KIND_NOTE) continue;
        if ((entry.action == PAD_TRIGGER_SAMPLE || entry.action == PAD_TRIGGER_MELODIC)
            && entry.arg0 == pad)
            return true;
    }
    return false;
}

// Selected MIDI channel, zero-based (DEV1 → CH 1/2/3, DEV2 → CH 4/5/6).
static uint8_t mpd_map_channel(void) {
    return (uint8_t)(red808_mpd218::kFirstMidiChannel
                     + s_mpd_device * red808_mpd218::kChannelsPerDevice
                     + s_mpd_bank);
}

static void mpd_map_refresh(void);
static void mpd_assign_modal_open(const MidiLearnCapture& capture);
static void mpd_assign_modal_close(void);
static void mpd_assign_rearm_if_batch(void);

// ── Assignment picker ────────────────────────────────────────────────────────

static void mpd_assign_feedback(const char* text, lv_color_t color) {
    if (!s_mpd_activity_label) return;
    lv_label_set_text(s_mpd_activity_label, text);
    lv_obj_set_style_text_color(s_mpd_activity_label, color, 0);
}

static void mpd_assign_modal_close(void) {
    if (s_mpd_assign_modal) lv_obj_del(s_mpd_assign_modal);
    s_mpd_assign_modal = NULL;
    s_mpd_assign_grid = NULL;
    memset(s_mpd_assign_cat_btns, 0, sizeof(s_mpd_assign_cat_btns));
}

static bool mpd_assign_current_action(uint8_t& type, uint8_t& arg0,
                                      uint8_t& arg1) {
    MidiMapEntry learned{};
    if (control_midi_map_find(s_mpd_assign_capture.channel,
                              s_mpd_assign_capture.kind,
                              s_mpd_assign_capture.number, learned)) {
        type = learned.action; arg0 = learned.arg0; arg1 = learned.arg1;
        return true;
    }
    uint8_t device = 0, bank = 0, layer = 0, index = 0;
    if (s_mpd_assign_capture.kind == MIDI_MAP_KIND_NOTE) {
        if (red808_mpd218::DecodePad(s_mpd_assign_capture.channel,
                                     s_mpd_assign_capture.number,
                                     device, bank, layer, index)) {
            const red808_mpd218::PadAction& pad =
                red808_mpd218::kPadMap[bank][layer][index];
            type = pad.type; arg0 = pad.arg0; arg1 = pad.arg1;
            return true;
        }
    } else if (red808_mpd218::DecodeKnob(s_mpd_assign_capture.channel,
                                         s_mpd_assign_capture.number,
                                         device, bank, layer, index)) {
        type = (uint8_t)red808_mpd218::kKnobMap[bank][layer][index].type;
        arg0 = 0; arg1 = 0;
        return true;
    }
    return false;
}

static void mpd_assign_apply(uint8_t action, uint8_t arg0, uint8_t arg1) {
    // Capture what this slot pointed at BEFORE overwriting it, so the
    // confirmation toast can show "OLD -> NEW" instead of just the new
    // sound — makes it obvious a reassignment actually landed on the
    // intended pad/knob and not somewhere else.
    uint8_t prevType = 0, prevA0 = 0, prevA1 = 0;
    const bool hadPrevious =
        mpd_assign_current_action(prevType, prevA0, prevA1);
    char before[40] = {};
    if (hadPrevious) {
        if (s_mpd_assign_capture.kind == MIDI_MAP_KIND_NOTE)
            mpd_pad_action_text(prevType, prevA0, prevA1, before,
                                sizeof(before));
        else
            snprintf(before, sizeof(before), "%s",
                     prevType <= red808_mpd218::KNOB_STEREO_WIDTH
                         ? MPD_KNOB_ACTION_NAMES[prevType] : "NONE");
    } else {
        snprintf(before, sizeof(before), "NONE");
    }

    MidiMapEntry entry{};
    entry.channel = s_mpd_assign_capture.channel;
    entry.kind = s_mpd_assign_capture.kind;
    entry.number = s_mpd_assign_capture.number;
    entry.action = action;
    entry.arg0 = arg0;
    entry.arg1 = arg1;
    char what[40] = {};
    if (entry.kind == MIDI_MAP_KIND_NOTE)
        mpd_pad_action_text(action, arg0, arg1, what, sizeof(what));
    else
        snprintf(what, sizeof(what), "%s",
                 action <= red808_mpd218::KNOB_STEREO_WIDTH
                     ? MPD_KNOB_ACTION_NAMES[action] : "NONE");
    char message[160] = {};
    if (control_midi_map_assign(entry)) {
        int n = snprintf(message, sizeof(message),
                         "MAPEADO  CH%u %s%u   %s  >  %s",
                         (unsigned)entry.channel + 1u,
                         entry.kind == MIDI_MAP_KIND_NOTE ? "N" : "CC",
                         (unsigned)entry.number, before, what);
        // Heads-up if another learned pad/knob already fires the exact same
        // thing — not necessarily wrong (some setups do want two physical
        // controls sharing one action), but worth flagging so it isn't an
        // accidental double-assignment.
        MidiMapEntry dup{};
        if (n > 0 && n < (int)sizeof(message)
            && control_midi_map_find_duplicate(
                   entry.kind, entry.action, entry.arg0, entry.arg1,
                   entry.channel, entry.number, dup))
            snprintf(message + n, sizeof(message) - n,
                     "  (tambien CH%u %s%u)", (unsigned)dup.channel + 1u,
                     dup.kind == MIDI_MAP_KIND_NOTE ? "N" : "CC",
                     (unsigned)dup.number);
    } else
        snprintf(message, sizeof(message), "MAPA LLENO (%u ENTRADAS)",
                 (unsigned)MIDI_MAP_MAX_ENTRIES);
    mpd_assign_feedback(message, MPD_LEARNED_MARK);
    mpd_assign_modal_close();
    mpd_map_refresh();
    mpd_assign_rearm_if_batch();
}

static void mpd_assign_option_cb(lv_event_t* e) {
    const uint32_t packed = (uint32_t)(uintptr_t)lv_event_get_user_data(e);
    const uint8_t category = (uint8_t)(packed >> 8);
    const uint8_t index = (uint8_t)(packed & 0xFFu);
    using namespace red808_mpd218;
    if (s_mpd_assign_capture.kind == MIDI_MAP_KIND_CC) {
        mpd_assign_apply(index, 0, 0); // index == KnobActionType
        return;
    }
    switch (category) {
        case 0: mpd_assign_apply(PAD_TRIGGER_SAMPLE, index, 0); break;
        case 1: mpd_assign_apply(PAD_TRIGGER_SYNTH, 0, index); break;
        case 2: mpd_assign_apply(PAD_TRIGGER_SYNTH, 1, index); break;
        case 3: mpd_assign_apply(PAD_TRIGGER_SYNTH, 2, index); break;
        case 4: mpd_assign_apply(PAD_SELECT_PATTERN, index, 0); break;
        case 5: mpd_assign_apply(PAD_TOGGLE_TRACK_MUTE, index, 0); break;
        case 6:
            if (index < 8) mpd_assign_apply(MPD_TRANSPORT_ACTIONS[index], 0, 0);
            break;
        default: break;
    }
}

static void mpd_assign_build_options(void) {
    if (!s_mpd_assign_grid) return;
    lv_obj_clean(s_mpd_assign_grid);

    uint8_t currentType = 0xFF, currentA0 = 0, currentA1 = 0;
    mpd_assign_current_action(currentType, currentA0, currentA1);

    auto makeOption = [&](const char* text, uint32_t packed, bool active) {
        lv_obj_t* button = lv_btn_create(s_mpd_assign_grid);
        lv_obj_set_size(button, 208, 52);
        apply_control_button_style(button,
            active ? RED808_CYAN : RED808_ACCENT2, active, 8);
        lv_obj_t* label = lv_label_create(button);
        lv_label_set_text(label, text);
        lv_obj_set_width(label, 196);
        lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(label,
            active ? lv_color_white() : RED808_TEXT, 0);
        lv_obj_center(label);
        lv_obj_add_event_cb(button, mpd_assign_option_cb, LV_EVENT_CLICKED,
                            (void*)(uintptr_t)packed);
    };

    using namespace red808_mpd218;
    if (s_mpd_assign_capture.kind == MIDI_MAP_KIND_CC) {
        for (uint8_t action = KNOB_MASTER_VOLUME;
             action <= KNOB_STEREO_WIDTH; ++action) {
            makeOption(MPD_KNOB_ACTION_NAMES[action], action,
                       currentType == action);
        }
        return;
    }

    char text[48] = {};
    switch (s_mpd_assign_category) {
        case 0:
            for (uint8_t i = 0; i < 16; ++i) {
                if (p4.sample_name[i][0] != '\0')
                    snprintf(text, sizeof(text), "S%02u  %s", i + 1u,
                             p4.sample_name[i]);
                else
                    snprintf(text, sizeof(text), "SAMPLE %02u", i + 1u);
                makeOption(text, (0u << 8) | i,
                           currentType == PAD_TRIGGER_SAMPLE && currentA0 == i);
            }
            break;
        case 1:
        case 2:
        case 3: {
            const uint8_t engine = s_mpd_assign_category - 1;
            static const char* const engines[3] = {"808", "909", "505"};
            for (uint8_t i = 0; i < 16; ++i) {
                snprintf(text, sizeof(text), "%s %s", engines[engine],
                         MPD_DRUM_PAD_NAMES[engine][i]);
                makeOption(text, ((uint32_t)s_mpd_assign_category << 8) | i,
                           currentType == PAD_TRIGGER_SYNTH
                               && currentA0 == engine && currentA1 == i);
            }
            break;
        }
        case 4:
            for (uint8_t i = 0; i < 16; ++i) {
                snprintf(text, sizeof(text), "PATTERN %02u", i + 1u);
                makeOption(text, (4u << 8) | i,
                           currentType == PAD_SELECT_PATTERN && currentA0 == i);
            }
            break;
        case 5:
            for (uint8_t i = 0; i < 16; ++i) {
                snprintf(text, sizeof(text), "MUTE TRACK %02u", i + 1u);
                makeOption(text, (5u << 8) | i,
                           currentType == PAD_TOGGLE_TRACK_MUTE
                               && currentA0 == i);
            }
            break;
        case 6:
            for (uint8_t i = 0; i < 8; ++i)
                makeOption(MPD_TRANSPORT_NAMES[i], (6u << 8) | i,
                           currentType == MPD_TRANSPORT_ACTIONS[i]);
            break;
        default: break;
    }
}

static void mpd_assign_category_cb(lv_event_t* e) {
    const uint8_t category = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    if (category >= 7) return;
    s_mpd_assign_category = category;
    for (uint8_t i = 0; i < 7; ++i) {
        lv_obj_t* button = s_mpd_assign_cat_btns[i];
        if (!button) continue;
        const bool selected = i == s_mpd_assign_category;
        lv_obj_set_style_bg_color(button,
            selected ? RED808_ACCENT : MPD_PANEL_BG, 0);
        lv_obj_set_style_bg_grad_color(button,
            selected ? RED808_ACCENT : MPD_PANEL_BG, 0);
        lv_obj_t* label = lv_obj_get_child(button, 0);
        if (label) lv_obj_set_style_text_color(label,
            selected ? lv_color_white() : MPD_TEXT_FAINT, 0);
    }
    mpd_assign_build_options();
}

// LEARN CONTINUO: re-arms LEARN right after the picker closes, so the user
// can tap several AKAI pads/knobs in a row without reopening LEARN each
// time. Called from the apply/remove/cancel paths alike, so backing out of
// a wrong pick doesn't break the streak.
static void mpd_assign_rearm_if_batch(void) {
    if (s_mpd_batch_learn) control_midi_learn_arm(true);
}

static void mpd_assign_remove_cb(lv_event_t* e) {
    LV_UNUSED(e);
    if (control_midi_map_clear(s_mpd_assign_capture.channel,
                               s_mpd_assign_capture.kind,
                               s_mpd_assign_capture.number))
        mpd_assign_feedback("ASIGNACION ELIMINADA — vuelve el mapa de fabrica",
                            RED808_WARNING);
    else
        mpd_assign_feedback("SIN ASIGNACION PROPIA — ya usa el mapa de fabrica",
                            MPD_TEXT_FAINT);
    mpd_assign_modal_close();
    mpd_map_refresh();
    mpd_assign_rearm_if_batch();
}

static void mpd_assign_cancel_cb(lv_event_t* e) {
    LV_UNUSED(e);
    mpd_assign_modal_close();
    mpd_assign_rearm_if_batch();
}

static void mpd_assign_modal_open(const MidiLearnCapture& capture) {
    mpd_assign_modal_close();
    s_mpd_assign_capture = capture;
    if (s_mpd_assign_capture.kind == MIDI_MAP_KIND_NOTE
        && s_mpd_assign_category > 6)
        s_mpd_assign_category = 0;

    s_mpd_assign_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_mpd_assign_modal, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(s_mpd_assign_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_mpd_assign_modal, LV_OPA_70, 0);
    lv_obj_set_style_border_width(s_mpd_assign_modal, 0, 0);
    lv_obj_set_style_pad_all(s_mpd_assign_modal, 0, 0);
    lv_obj_clear_flag(s_mpd_assign_modal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_mpd_assign_modal, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t* card = lv_obj_create(s_mpd_assign_modal);
    lv_obj_set_size(card, 952, 552);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, MPD_BODY_BG, 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_border_color(card, RED808_CYAN, 0);
    lv_obj_set_style_radius(card, 16, 0);
    lv_obj_set_style_pad_all(card, 0, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    char captured[64] = {};
    if (capture.kind == MIDI_MAP_KIND_NOTE) {
        char note[6] = {};
        mpd_note_name(capture.number, note, sizeof(note));
        snprintf(captured, sizeof(captured),
                 "ASIGNAR  ·  CH %u  ·  NOTA %u (%s)  ·  VEL %u",
                 (unsigned)capture.channel + 1u, (unsigned)capture.number,
                 note, (unsigned)capture.value);
    } else {
        snprintf(captured, sizeof(captured),
                 "ASIGNAR  ·  CH %u  ·  CC %u  ·  VALOR %u",
                 (unsigned)capture.channel + 1u, (unsigned)capture.number,
                 (unsigned)capture.value);
    }
    lv_obj_t* title = lv_label_create(card);
    lv_label_set_text(title, captured);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title, RED808_CYAN, 0);
    lv_obj_set_pos(title, 18, 12);

    int gridTop = 56;
    if (capture.kind == MIDI_MAP_KIND_NOTE) {
        for (uint8_t i = 0; i < 7; ++i) {
            lv_obj_t* button = lv_btn_create(card);
            lv_obj_set_size(button, 126, 38);
            lv_obj_set_pos(button, 18 + i * 132, 50);
            apply_control_button_style(button, RED808_ACCENT2, false, 8);
            lv_obj_t* label = lv_label_create(button);
            lv_label_set_text(label, MPD_ASSIGN_CATEGORIES[i]);
            lv_obj_set_style_text_font(label, &lv_font_montserrat_12, 0);
            lv_obj_center(label);
            lv_obj_add_event_cb(button, mpd_assign_category_cb,
                                LV_EVENT_CLICKED, (void*)(uintptr_t)i);
            s_mpd_assign_cat_btns[i] = button;
        }
        gridTop = 98;
    }

    s_mpd_assign_grid = lv_obj_create(card);
    lv_obj_set_size(s_mpd_assign_grid, 916, 484 - gridTop);
    lv_obj_set_pos(s_mpd_assign_grid, 18, gridTop);
    lv_obj_set_style_bg_opa(s_mpd_assign_grid, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_mpd_assign_grid, 0, 0);
    lv_obj_set_style_pad_all(s_mpd_assign_grid, 4, 0);
    lv_obj_set_style_pad_row(s_mpd_assign_grid, 8, 0);
    lv_obj_set_style_pad_column(s_mpd_assign_grid, 8, 0);
    lv_obj_set_flex_flow(s_mpd_assign_grid, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_scroll_dir(s_mpd_assign_grid, LV_DIR_VER);

    lv_obj_t* remove = lv_btn_create(card);
    lv_obj_set_size(remove, 260, 44);
    lv_obj_set_pos(remove, 18, 494);
    apply_control_button_style(remove, RED808_WARNING, false, 10);
    lv_obj_t* removeLabel = lv_label_create(remove);
    lv_label_set_text(removeLabel, "QUITAR ASIGNACION");
    lv_obj_set_style_text_font(removeLabel, &lv_font_montserrat_12, 0);
    lv_obj_center(removeLabel);
    lv_obj_add_event_cb(remove, mpd_assign_remove_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* cancel = lv_btn_create(card);
    lv_obj_set_size(cancel, 160, 44);
    lv_obj_set_pos(cancel, 774, 494);
    apply_control_button_style(cancel, RED808_BORDER, false, 10);
    lv_obj_t* cancelLabel = lv_label_create(cancel);
    lv_label_set_text(cancelLabel, "CANCELAR");
    lv_obj_set_style_text_font(cancelLabel, &lv_font_montserrat_12, 0);
    lv_obj_center(cancelLabel);
    lv_obj_add_event_cb(cancel, mpd_assign_cancel_cb, LV_EVENT_CLICKED, NULL);

    if (capture.kind == MIDI_MAP_KIND_NOTE) {
        // Re-select the active category so its button paints highlighted.
        for (uint8_t i = 0; i < 7; ++i) {
            lv_obj_t* button = s_mpd_assign_cat_btns[i];
            if (!button) continue;
            const bool selected = i == s_mpd_assign_category;
            lv_obj_set_style_bg_color(button,
                selected ? RED808_ACCENT : MPD_PANEL_BG, 0);
            lv_obj_set_style_bg_grad_color(button,
                selected ? RED808_ACCENT : MPD_PANEL_BG, 0);
            lv_obj_t* label = lv_obj_get_child(button, 0);
            if (label) lv_obj_set_style_text_color(label,
                selected ? lv_color_white() : MPD_TEXT_FAINT, 0);
        }
    }
    mpd_assign_build_options();
}

// ── Device view ──────────────────────────────────────────────────────────────

static void mpd_map_refresh(void) {
    using namespace red808_mpd218;
    if (!s_mpd_map_modal) return;

    const uint8_t channel = mpd_map_channel();
    const uint8_t noteBase = kPadNoteBase[s_mpd_pad_layer];
    const uint8_t ccBase = kKnobCcBase[s_mpd_knob_layer];

    if (s_mpd_dev_btn) {
        lv_obj_t* label = lv_obj_get_child(s_mpd_dev_btn, 0);
        if (label) lv_label_set_text_fmt(label, "DEVICE %u",
                                         (unsigned)s_mpd_device + 1u);
    }
    if (s_mpd_prog_btn) {
        lv_obj_t* label = lv_obj_get_child(s_mpd_prog_btn, 0);
        if (label) lv_label_set_text_fmt(label, "PROG %u",
                                         (unsigned)s_mpd_bank + 1u);
    }
    if (s_mpd_padbank_btn) {
        lv_obj_t* label = lv_obj_get_child(s_mpd_padbank_btn, 0);
        if (label) lv_label_set_text_fmt(label, "PAD BANK %c",
                                         'A' + s_mpd_pad_layer);
    }
    if (s_mpd_ctrlbank_btn) {
        lv_obj_t* label = lv_obj_get_child(s_mpd_ctrlbank_btn, 0);
        if (label) lv_label_set_text_fmt(label, "CTRL BANK %c",
                                         'A' + s_mpd_knob_layer);
    }
    if (s_mpd_map_summary_label) {
        lv_label_set_text_fmt(s_mpd_map_summary_label,
            "DEV %u  ·  PROG %u %s  ·  MIDI CH %u  ·  PADS %c N%u-%u  ·  "
            "KNOBS %c CC%u-%u  ·  %u APRENDIDOS",
            (unsigned)s_mpd_device + 1u, (unsigned)s_mpd_bank + 1u,
            MPD_BANK_NAMES[s_mpd_bank], (unsigned)channel + 1u,
            'A' + s_mpd_pad_layer, (unsigned)noteBase,
            (unsigned)(noteBase + kPadsPerLayer - 1u),
            'A' + s_mpd_knob_layer, (unsigned)ccBase,
            (unsigned)(ccBase + kKnobsPerLayer - 1u),
            (unsigned)control_midi_map_count());
    }

    for (uint8_t pad = 0; pad < kPadsPerLayer; ++pad) {
        if (!s_mpd_pad_labels[pad]) continue;
        const uint8_t note = (uint8_t)(noteBase + pad);
        char action[40] = {};
        MidiMapEntry learned{};
        const bool hasLearned = control_midi_map_find(
            channel, MIDI_MAP_KIND_NOTE, note, learned);
        if (hasLearned)
            mpd_pad_action_text(learned.action, learned.arg0, learned.arg1,
                                action, sizeof(action));
        else {
            const PadAction& factory = kPadMap[s_mpd_bank][s_mpd_pad_layer][pad];
            mpd_pad_action_text(factory.type, factory.arg0, factory.arg1,
                                action, sizeof(action));
        }
        lv_label_set_text_fmt(s_mpd_pad_labels[pad], "PAD %u  N%u%s\n%s",
                              (unsigned)pad + 1u, (unsigned)note,
                              hasLearned ? "  *" : "", action);
        lv_obj_set_style_text_color(s_mpd_pad_labels[pad],
            hasLearned ? MPD_LEARNED_MARK : MPD_TEXT_MAIN, 0);
        if (s_mpd_pad_cells[pad])
            lv_obj_set_style_border_color(s_mpd_pad_cells[pad],
                hasLearned ? MPD_LEARNED_MARK : MPD_PAD_RIM, 0);
    }

    for (uint8_t knob = 0; knob < kKnobsPerLayer; ++knob) {
        if (!s_mpd_knob_labels[knob]) continue;
        const uint8_t cc = (uint8_t)(ccBase + knob);
        MidiMapEntry learned{};
        const bool hasLearned = control_midi_map_find(
            channel, MIDI_MAP_KIND_CC, cc, learned);
        uint8_t action = hasLearned
            ? learned.action
            : (uint8_t)kKnobMap[s_mpd_bank][s_mpd_knob_layer][knob].type;
        if (action > KNOB_STEREO_WIDTH) action = KNOB_NONE;
        lv_label_set_text_fmt(s_mpd_knob_labels[knob], "K%u · CC%u%s\n%s",
                              (unsigned)knob + 1u, (unsigned)cc,
                              hasLearned ? " *" : "",
                              MPD_KNOB_ACTION_NAMES[action]);
        lv_obj_set_style_text_color(s_mpd_knob_labels[knob],
            hasLearned ? MPD_LEARNED_MARK : MPD_TEXT_FAINT, 0);
    }
}

static void mpd_map_cycle_cb(lv_event_t* e) {
    const uint8_t group = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    switch (group) {
        case 0: s_mpd_device = (uint8_t)((s_mpd_device + 1u)
                    % red808_mpd218::kDeviceCount); break;
        case 1: s_mpd_bank = (uint8_t)((s_mpd_bank + 1u)
                    % red808_mpd218::kBankCount); break;
        case 2: s_mpd_pad_layer = (uint8_t)((s_mpd_pad_layer + 1u)
                    % red808_mpd218::kLayerCount); break;
        case 3: s_mpd_knob_layer = (uint8_t)((s_mpd_knob_layer + 1u)
                    % red808_mpd218::kLayerCount); break;
        default: return;
    }
    mpd_map_refresh();
}

static void mpd_map_pad_cb(lv_event_t* e) {
    const uint8_t pad = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    if (pad >= 16) return;
    MidiLearnCapture capture{};
    capture.channel = mpd_map_channel();
    capture.kind = MIDI_MAP_KIND_NOTE;
    capture.number = (uint8_t)(red808_mpd218::kPadNoteBase[s_mpd_pad_layer]
                               + pad);
    capture.value = 100;
    mpd_assign_modal_open(capture);
}

static void mpd_map_knob_cb(lv_event_t* e) {
    const uint8_t knob = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    if (knob >= 6) return;
    MidiLearnCapture capture{};
    capture.channel = mpd_map_channel();
    capture.kind = MIDI_MAP_KIND_CC;
    capture.number = (uint8_t)(red808_mpd218::kKnobCcBase[s_mpd_knob_layer]
                               + knob);
    capture.value = 0;
    mpd_assign_modal_open(capture);
}

static void mpd_map_batch_button_refresh(void) {
    if (!s_mpd_batch_btn) return;
    lv_obj_set_style_bg_color(s_mpd_batch_btn,
        s_mpd_batch_learn ? MPD_LEARN_RED : MPD_PANEL_BG, 0);
    lv_obj_set_style_bg_grad_color(s_mpd_batch_btn,
        s_mpd_batch_learn ? MPD_LEARN_RED : MPD_BODY_BG, 0);
    lv_obj_t* label = lv_obj_get_child(s_mpd_batch_btn, 0);
    if (label) lv_obj_set_style_text_color(label,
        s_mpd_batch_learn ? lv_color_white() : MPD_TEXT_MAIN, 0);
}

static void mpd_map_learn_cb(lv_event_t* e) {
    LV_UNUSED(e);
    const bool arm = !control_midi_learn_armed();
    control_midi_learn_arm(arm);
    if (arm)
        mpd_assign_feedback("LEARN ACTIVO — toca un pad o mueve un knob del AKAI",
                            MPD_LEARN_RED);
    else {
        // Manually cancelling LEARN also stops any batch streak in
        // progress — pressing the red key again is the universal "stop".
        s_mpd_batch_learn = false;
        mpd_map_batch_button_refresh();
        mpd_assign_feedback("LEARN cancelado", MPD_TEXT_FAINT);
    }
}

static void mpd_map_batch_cb(lv_event_t* e) {
    LV_UNUSED(e);
    s_mpd_batch_learn = !s_mpd_batch_learn;
    mpd_map_batch_button_refresh();
    if (s_mpd_batch_learn) {
        control_midi_learn_arm(true);
        mpd_assign_feedback(
            "LEARN CONTINUO — cada asignacion rearma LEARN sola",
            MPD_LEARN_RED);
    } else {
        mpd_assign_feedback("Learn continuo desactivado", MPD_TEXT_FAINT);
    }
}

static void mpd_map_export_cb(lv_event_t* e) {
    LV_UNUSED(e);
    if (!p4sd.mounted) {
        mpd_assign_feedback("EXPORT SD: tarjeta no montada", RED808_WARNING);
        return;
    }
    if (control_midi_map_export_sd())
        mpd_assign_feedback("Mapa exportado a /midi_map_backup.mmap",
                            MPD_LEARNED_MARK);
    else
        mpd_assign_feedback("EXPORT SD: fallo de escritura", RED808_WARNING);
}

static void mpd_map_import_cb(lv_event_t* e) {
    LV_UNUSED(e);
    if (!p4sd.mounted) {
        mpd_assign_feedback("IMPORT SD: tarjeta no montada", RED808_WARNING);
        return;
    }
    if (control_midi_map_import_sd()) {
        mpd_assign_feedback("Mapa importado desde /midi_map_backup.mmap",
                            MPD_LEARNED_MARK);
        mpd_map_refresh();
    } else
        mpd_assign_feedback("IMPORT SD: sin backup valido en la tarjeta",
                            RED808_WARNING);
}

static void mpd_map_clear_cb(lv_event_t* e) {
    LV_UNUSED(e);
    control_midi_map_clear_all();
    mpd_assign_feedback("MAPA APRENDIDO BORRADO — mapa de fabrica activo",
                        RED808_WARNING);
    mpd_map_refresh();
}

static void mpd_map_modal_close_cb(lv_event_t* e) {
    if (e && lv_event_get_target(e) != lv_event_get_current_target(e)) return;
    control_midi_learn_arm(false);
    s_mpd_batch_learn = false;
    mpd_assign_modal_close();
    if (s_mpd_map_modal) lv_obj_del(s_mpd_map_modal);
    s_mpd_map_modal = NULL;
    s_mpd_map_summary_label = NULL;
    s_mpd_activity_label = NULL;
    s_mpd_learn_btn = NULL;
    s_mpd_learn_label = NULL;
    s_mpd_dev_btn = NULL;
    s_mpd_prog_btn = NULL;
    s_mpd_padbank_btn = NULL;
    s_mpd_ctrlbank_btn = NULL;
    s_mpd_batch_btn = NULL;
    memset(s_mpd_pad_cells, 0, sizeof(s_mpd_pad_cells));
    memset(s_mpd_pad_labels, 0, sizeof(s_mpd_pad_labels));
    memset(s_mpd_knob_cells, 0, sizeof(s_mpd_knob_cells));
    memset(s_mpd_knob_arcs, 0, sizeof(s_mpd_knob_arcs));
    memset(s_mpd_knob_labels, 0, sizeof(s_mpd_knob_labels));
}

static lv_obj_t* mpd_map_make_button(lv_obj_t* parent, int x, int y, int w,
                                     int h, const char* text,
                                     lv_event_cb_t cb, uint8_t userValue) {
    lv_obj_t* button = lv_btn_create(parent);
    lv_obj_set_size(button, w, h);
    lv_obj_set_pos(button, x, y);
    lv_obj_set_style_radius(button, 6, 0);
    lv_obj_set_style_bg_color(button, MPD_PANEL_BG, 0);
    lv_obj_set_style_bg_grad_color(button, MPD_BODY_BG, 0);
    lv_obj_set_style_bg_grad_dir(button, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(button, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(button, 1, 0);
    lv_obj_set_style_border_color(button, lv_color_hex(0x3A3A3A), 0);
    lv_obj_set_style_shadow_width(button, 0, 0);
    lv_obj_t* label = lv_label_create(button);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(label, MPD_TEXT_MAIN, 0);
    lv_obj_center(label);
    lv_obj_add_event_cb(button, cb, LV_EVENT_CLICKED,
                        (void*)(uintptr_t)userValue);
    return button;
}

static void mpd_map_modal_open_cb(lv_event_t* e) {
    LV_UNUSED(e);
    if (s_mpd_map_modal) { mpd_map_modal_close_cb(NULL); return; }

    // Ignore captures/activity produced before the screen opened.
    s_mpd_seen_capture_rev = control_midi_capture_revision();
    s_mpd_seen_activity_rev = control_midi_activity_revision();
    s_mpd_seen_timeout_rev = control_midi_learn_timeout_revision();
    memset((void*)s_mpd_pad_glow_until, 0, sizeof(s_mpd_pad_glow_until));
    memset((void*)s_mpd_knob_glow_until, 0, sizeof(s_mpd_knob_glow_until));

    s_mpd_map_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_mpd_map_modal, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(s_mpd_map_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_mpd_map_modal, LV_OPA_80, 0);
    lv_obj_set_style_border_width(s_mpd_map_modal, 0, 0);
    lv_obj_set_style_pad_all(s_mpd_map_modal, 0, 0);
    lv_obj_clear_flag(s_mpd_map_modal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_mpd_map_modal, mpd_map_modal_close_cb,
                        LV_EVENT_CLICKED, NULL);

    lv_obj_t* card = lv_obj_create(s_mpd_map_modal);
    lv_obj_set_size(card, 984, 576);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, MPD_BODY_BG, 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_border_color(card, lv_color_hex(0x2C2C2C), 0);
    lv_obj_set_style_radius(card, 18, 0);
    lv_obj_set_style_pad_all(card, 0, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t* brand = lv_label_create(card);
    lv_label_set_text(brand, "AKAI  MPD218");
    lv_obj_set_style_text_font(brand, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(brand, MPD_TEXT_MAIN, 0);
    lv_obj_set_pos(brand, 18, 10);

    lv_obj_t* title = lv_label_create(card);
    lv_label_set_text(title, "MIDI MAP  +  LEARN");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(title, MPD_PAD_RIM, 0);
    lv_obj_set_pos(title, 200, 18);

    lv_obj_t* close = lv_btn_create(card);
    lv_obj_set_size(close, 78, 34);
    lv_obj_set_pos(close, 888, 10);
    apply_control_button_style(close, RED808_WARNING, false, 8);
    lv_obj_t* closeLabel = lv_label_create(close);
    lv_label_set_text(closeLabel, "BACK");
    lv_obj_set_style_text_font(closeLabel, &lv_font_montserrat_12, 0);
    lv_obj_center(closeLabel);
    lv_obj_add_event_cb(close, [](lv_event_t*) { mpd_map_modal_close_cb(NULL); },
                        LV_EVENT_CLICKED, NULL);

    s_mpd_batch_btn = mpd_map_make_button(card, 440, 8, 108, 34,
                                          "BATCH", mpd_map_batch_cb, 0);
    mpd_map_make_button(card, 556, 8, 108, 34, "EXPORT SD",
                        mpd_map_export_cb, 0);
    mpd_map_make_button(card, 672, 8, 108, 34, "IMPORT SD",
                        mpd_map_import_cb, 0);
    mpd_map_batch_button_refresh();

    s_mpd_map_summary_label = lv_label_create(card);
    lv_obj_set_width(s_mpd_map_summary_label, 948);
    lv_obj_set_pos(s_mpd_map_summary_label, 18, 40);
    lv_obj_set_style_text_font(s_mpd_map_summary_label,
                               &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(s_mpd_map_summary_label, MPD_TEXT_FAINT, 0);

    // ── Left panel: knobs + banks + LEARN, like the device front plate ──
    lv_obj_t* panel = lv_obj_create(card);
    lv_obj_set_size(panel, 330, 460);
    lv_obj_set_pos(panel, 18, 58);
    lv_obj_set_style_bg_color(panel, MPD_PANEL_BG, 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(panel, 1, 0);
    lv_obj_set_style_border_color(panel, lv_color_hex(0x2A2A2A), 0);
    lv_obj_set_style_radius(panel, 12, 0);
    lv_obj_set_style_pad_all(panel, 0, 0);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(panel, LV_OBJ_FLAG_CLICKABLE);

    // Knob cells. Physical numbering per row (top→bottom): 5·6, 3·4, 1·2.
    for (uint8_t row = 0; row < 3; ++row) {
        for (uint8_t col = 0; col < 2; ++col) {
            const uint8_t number = (uint8_t)((2 - row) * 2 + col + 1);
            const uint8_t knob = (uint8_t)(number - 1);
            lv_obj_t* cell = lv_obj_create(panel);
            lv_obj_set_size(cell, 150, 100);
            lv_obj_set_pos(cell, 12 + col * 158, 6 + row * 106);
            lv_obj_set_style_bg_color(cell, MPD_BODY_BG, 0);
            lv_obj_set_style_bg_opa(cell, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(cell, 1, 0);
            lv_obj_set_style_border_color(cell, lv_color_hex(0x2A2A2A), 0);
            lv_obj_set_style_radius(cell, 10, 0);
            lv_obj_set_style_pad_all(cell, 2, 0);
            lv_obj_clear_flag(cell, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_flag(cell, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(cell, mpd_map_knob_cb, LV_EVENT_CLICKED,
                                (void*)(uintptr_t)knob);
            s_mpd_knob_cells[knob] = cell;

            lv_obj_t* arc = lv_arc_create(cell);
            lv_obj_set_size(arc, 52, 52);
            lv_obj_align(arc, LV_ALIGN_TOP_MID, 0, 0);
            lv_arc_set_bg_angles(arc, 135, 45);
            lv_arc_set_range(arc, 0, 127);
            lv_arc_set_value(arc, 0);
            lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
            lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_set_style_arc_width(arc, 4, LV_PART_MAIN);
            lv_obj_set_style_arc_color(arc, lv_color_hex(0x333333),
                                       LV_PART_MAIN);
            lv_obj_set_style_arc_width(arc, 4, LV_PART_INDICATOR);
            lv_obj_set_style_arc_color(arc, MPD_PAD_GLOW, LV_PART_INDICATOR);
            s_mpd_knob_arcs[knob] = arc;

            lv_obj_t* num = lv_label_create(arc);
            lv_label_set_text_fmt(num, "%u", (unsigned)number);
            lv_obj_set_style_text_font(num, &lv_font_montserrat_12, 0);
            lv_obj_set_style_text_color(num, MPD_TEXT_MAIN, 0);
            lv_obj_center(num);

            lv_obj_t* label = lv_label_create(cell);
            lv_obj_set_width(label, 142);
            lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
            lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_set_style_text_font(label, &lv_font_montserrat_10, 0);
            lv_obj_set_style_text_color(label, MPD_TEXT_FAINT, 0);
            lv_obj_align(label, LV_ALIGN_BOTTOM_MID, 0, -2);
            s_mpd_knob_labels[knob] = label;
        }
    }

    s_mpd_dev_btn = mpd_map_make_button(panel, 12, 326, 150, 34, "DEVICE 1",
                                        mpd_map_cycle_cb, 0);
    s_mpd_prog_btn = mpd_map_make_button(panel, 170, 326, 150, 34, "PROG 1",
                                         mpd_map_cycle_cb, 1);
    s_mpd_padbank_btn = mpd_map_make_button(panel, 12, 366, 150, 34,
                                            "PAD BANK A", mpd_map_cycle_cb, 2);
    s_mpd_ctrlbank_btn = mpd_map_make_button(panel, 170, 366, 150, 34,
                                             "CTRL BANK A", mpd_map_cycle_cb,
                                             3);

    s_mpd_learn_btn = lv_btn_create(panel);
    lv_obj_set_size(s_mpd_learn_btn, 150, 46);
    lv_obj_set_pos(s_mpd_learn_btn, 12, 408);
    lv_obj_set_style_radius(s_mpd_learn_btn, 8, 0);
    lv_obj_set_style_bg_color(s_mpd_learn_btn, MPD_LEARN_RED, 0);
    lv_obj_set_style_bg_grad_color(s_mpd_learn_btn, lv_color_hex(0x7A130C), 0);
    lv_obj_set_style_bg_grad_dir(s_mpd_learn_btn, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_width(s_mpd_learn_btn, 2, 0);
    lv_obj_set_style_border_color(s_mpd_learn_btn, lv_color_hex(0x581008), 0);
    lv_obj_set_style_shadow_width(s_mpd_learn_btn, 0, 0);
    s_mpd_learn_label = lv_label_create(s_mpd_learn_btn);
    lv_label_set_text(s_mpd_learn_label, "LEARN");
    lv_obj_set_style_text_font(s_mpd_learn_label, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(s_mpd_learn_label, lv_color_white(), 0);
    lv_obj_center(s_mpd_learn_label);
    lv_obj_add_event_cb(s_mpd_learn_btn, mpd_map_learn_cb, LV_EVENT_CLICKED,
                        NULL);

    lv_obj_t* clearBtn = mpd_map_make_button(panel, 170, 408, 150, 46,
                                             "CLEAR MAP", mpd_map_clear_cb, 0);
    LV_UNUSED(clearBtn);

    // ── Right side: 4x4 pad matrix, PAD 13..16 on the top row ──
    for (uint8_t pad = 0; pad < 16; ++pad) {
        const int column = pad % 4;
        const int row = 3 - pad / 4; // hardware order: 13..16 up, 1..4 down
        lv_obj_t* cell = lv_obj_create(card);
        lv_obj_set_size(cell, 146, 118);
        lv_obj_set_pos(cell, 360 + column * 154, 58 + row * 126);
        lv_obj_set_style_bg_color(cell, MPD_PAD_BG, 0);
        lv_obj_set_style_bg_opa(cell, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(cell, 2, 0);
        lv_obj_set_style_border_color(cell, MPD_PAD_RIM, 0);
        lv_obj_set_style_radius(cell, 8, 0);
        lv_obj_set_style_pad_all(cell, 4, 0);
        lv_obj_clear_flag(cell, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(cell, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(cell, mpd_map_pad_cb, LV_EVENT_CLICKED,
                            (void*)(uintptr_t)pad);
        s_mpd_pad_cells[pad] = cell;

        lv_obj_t* label = lv_label_create(cell);
        lv_obj_set_width(label, 134);
        lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(label, MPD_TEXT_MAIN, 0);
        lv_obj_center(label);
        s_mpd_pad_labels[pad] = label;
    }

    s_mpd_activity_label = lv_label_create(card);
    lv_obj_set_width(s_mpd_activity_label, 330);
    lv_label_set_long_mode(s_mpd_activity_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(s_mpd_activity_label, 18, 524);
    lv_obj_set_style_text_font(s_mpd_activity_label, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(s_mpd_activity_label, MPD_TEXT_FAINT, 0);
    lv_label_set_text(s_mpd_activity_label,
                      "Toca un pad de esta pantalla para asignarlo, o pulsa "
                      "LEARN y toca el AKAI");

    mpd_map_refresh();
}

// Periodic tick from the LVGL task: LEARN capture handoff, pad/knob glow on
// incoming MIDI, and LEARN button blink.
static void mpd_map_modal_update(void) {
    if (!s_mpd_map_modal) return;
    const unsigned long now = millis();

    const uint32_t captureRev = control_midi_capture_revision();
    if (captureRev != s_mpd_seen_capture_rev) {
        s_mpd_seen_capture_rev = captureRev;
        mpd_assign_modal_open(control_midi_capture());
    }

    const uint32_t timeoutRev = control_midi_learn_timeout_revision();
    if (timeoutRev != s_mpd_seen_timeout_rev) {
        s_mpd_seen_timeout_rev = timeoutRev;
        // A timeout mid-batch means the user stopped touching the AKAI —
        // end the streak instead of leaving BATCH lit with nothing armed.
        s_mpd_batch_learn = false;
        mpd_map_batch_button_refresh();
        mpd_assign_feedback(
            "LEARN CANCELADO — sin pad/knob del AKAI en 8 s",
            RED808_WARNING);
    }

    const uint32_t activityRev = control_midi_activity_revision();
    if (activityRev != s_mpd_seen_activity_rev) {
        s_mpd_seen_activity_rev = activityRev;
        uint8_t status = 0, data0 = 0, data1 = 0;
        control_midi_last_activity(status, data0, data1);
        const uint8_t type = status & 0xF0u;
        const uint8_t channel = status & 0x0Fu;
        uint8_t device = 0, bank = 0, layer = 0, index = 0;
        if (type == 0x90u && data1 > 0) {
            if (s_mpd_activity_label) {
                char note[6] = {};
                mpd_note_name(data0, note, sizeof(note));
                lv_label_set_text_fmt(s_mpd_activity_label,
                    "IN  CH%u  NOTA %u (%s)  VEL %u",
                    (unsigned)channel + 1u, (unsigned)data0, note,
                    (unsigned)data1);
                lv_obj_set_style_text_color(s_mpd_activity_label,
                                            MPD_PAD_GLOW, 0);
            }
            int glowPad = -1;
            if (red808_mpd218::DecodePad(channel, data0, device, bank, layer,
                                         index)) {
                // DecodePad resolves index relative to the event's OWN
                // device/bank/layer, not necessarily the one on screen. If
                // the AKAI is sitting on a different program than what is
                // shown, glowing `index` directly lit the wrong on-screen
                // cell (whatever pad happens to share that position in the
                // CURRENT layer). Snap the view to match instead, so the
                // grid the user sees is always the one actually playing.
                if (device != s_mpd_device || bank != s_mpd_bank
                    || layer != s_mpd_pad_layer) {
                    s_mpd_device = device;
                    s_mpd_bank = bank;
                    s_mpd_pad_layer = layer;
                    mpd_map_refresh();
                }
                glowPad = index;
            } else if (data0 >= red808_mpd218::kPadNoteBase[s_mpd_pad_layer]
                     && data0 < red808_mpd218::kPadNoteBase[s_mpd_pad_layer]
                                + 16)
                glowPad = data0
                        - red808_mpd218::kPadNoteBase[s_mpd_pad_layer];
            if (glowPad >= 0 && glowPad < 16)
                s_mpd_pad_glow_until[glowPad] = now + 160;
        } else if (type == 0xB0u) {
            if (s_mpd_activity_label) {
                lv_label_set_text_fmt(s_mpd_activity_label,
                    "IN  CH%u  CC %u  VALOR %u",
                    (unsigned)channel + 1u, (unsigned)data0, (unsigned)data1);
                lv_obj_set_style_text_color(s_mpd_activity_label,
                                            MPD_LEARNED_MARK, 0);
            }
            int glowKnob = -1;
            if (red808_mpd218::DecodeKnob(channel, data0, device, bank, layer,
                                          index)) {
                // Same fix as the pad case: follow the incoming CC's real
                // program/layer instead of assuming it matches the screen.
                if (device != s_mpd_device || bank != s_mpd_bank
                    || layer != s_mpd_knob_layer) {
                    s_mpd_device = device;
                    s_mpd_bank = bank;
                    s_mpd_knob_layer = layer;
                    mpd_map_refresh();
                }
                glowKnob = index;
            }
            else if (data0 >= red808_mpd218::kKnobCcBase[s_mpd_knob_layer]
                     && data0 < red808_mpd218::kKnobCcBase[s_mpd_knob_layer]
                                + 6)
                glowKnob = data0
                         - red808_mpd218::kKnobCcBase[s_mpd_knob_layer];
            if (glowKnob >= 0 && glowKnob < 6) {
                s_mpd_knob_glow_until[glowKnob] = now + 300;
                if (s_mpd_knob_arcs[glowKnob])
                    lv_arc_set_value(s_mpd_knob_arcs[glowKnob], data1);
            }
        }
    }

    // Pad/knob glow transitions (only touch styles on edges).
    static bool padGlowing[16] = {};
    for (uint8_t pad = 0; pad < 16; ++pad) {
        if (!s_mpd_pad_cells[pad]) { padGlowing[pad] = false; continue; }
        const bool glow = now < s_mpd_pad_glow_until[pad];
        if (glow == padGlowing[pad]) continue;
        padGlowing[pad] = glow;
        lv_obj_set_style_bg_color(s_mpd_pad_cells[pad],
            glow ? MPD_PAD_GLOW : MPD_PAD_BG, 0);
        lv_obj_set_style_border_width(s_mpd_pad_cells[pad], glow ? 4 : 2, 0);
    }
    static bool knobGlowing[6] = {};
    for (uint8_t knob = 0; knob < 6; ++knob) {
        if (!s_mpd_knob_cells[knob]) { knobGlowing[knob] = false; continue; }
        const bool glow = now < s_mpd_knob_glow_until[knob];
        if (glow == knobGlowing[knob]) continue;
        knobGlowing[knob] = glow;
        lv_obj_set_style_border_color(s_mpd_knob_cells[knob],
            glow ? MPD_PAD_GLOW : lv_color_hex(0x2A2A2A), 0);
    }

    // LEARN key: solid red normally, blinking bright while armed.
    if (s_mpd_learn_btn) {
        static bool lastArmed = false;
        static bool lastBlink = false;
        const bool armed = control_midi_learn_armed();
        const bool blink = armed && ((now / 250u) & 1u);
        if (armed != lastArmed || blink != lastBlink) {
            lastArmed = armed;
            lastBlink = blink;
            lv_obj_set_style_bg_color(s_mpd_learn_btn,
                blink ? lv_color_hex(0xFF5040)
                      : (armed ? lv_color_hex(0xE8281A) : MPD_LEARN_RED), 0);
            if (s_mpd_learn_label)
                lv_label_set_text(s_mpd_learn_label,
                                  armed ? "LEARN..." : "LEARN");
        }
    }
}

static void pod_status_modal_update(void) {
    if (!s_pod_status_modal) return;
    static uint32_t lastUpdateMs = 0;
    const uint32_t now = millis();
    if (lastUpdateMs != 0 && now - lastUpdateMs < 50) return;
    lastUpdateMs = now;
    const auto& state = daisyUsb.state();
    if (state.pod.revision != s_pod_seen_revision
        && state.pod.config.version == POD_CONFIG_VERSION) {
        s_pod_seen_revision = state.pod.revision;
        s_pod_config = state.pod.config;
        s_pod_config.selectorFunction = POD_FUNC_NONE;
        pod_status_modal_refresh();
    }
    if (s_pod_status_label) {
        uint8_t loaded = 0;
        for (uint8_t i = 0; i < 16; i++) if (state.pod.sampleMask & (1u << i)) loaded++;
        const uint8_t rotaryMask = i2c_rotaries_detected_mask();
        const uint8_t addressAckMask = i2c_rotaries_address_ack_mask();
        const uint8_t muxAddress = i2c_rotaries_mux_address();
        uint8_t rotaryCount = 0;
        for (uint8_t i = 0; i < 4; i++)
            if (rotaryMask & (1u << i)) rotaryCount++;
        const char* kit = p4.kit_name[0] ? p4.kit_name : "RED 808 KARZ";
        const char* resetReason = "OTHER";
        switch (esp_reset_reason()) {
            case ESP_RST_POWERON:  resetReason = "POWERON"; break;
            case ESP_RST_EXT:      resetReason = "EXTERNAL"; break;
            case ESP_RST_SW:       resetReason = "SOFTWARE"; break;
            case ESP_RST_PANIC:    resetReason = "PANIC"; break;
            case ESP_RST_INT_WDT:  resetReason = "INT_WDT"; break;
            case ESP_RST_TASK_WDT: resetReason = "TASK_WDT"; break;
            case ESP_RST_WDT:      resetReason = "WDT"; break;
            case ESP_RST_DEEPSLEEP: resetReason = "DEEPSLEEP"; break;
            case ESP_RST_BROWNOUT: resetReason = "BROWNOUT"; break;
            case ESP_RST_SDIO:     resetReason = "SDIO"; break;
            default: break;
        }
        lv_label_set_text_fmt(s_pod_status_label,
            "RESET %s | DAISY AUDIO | WAV %u/16 | KIT %s\n"
            "I2C2 GPIO3/4 | PCA9548A %s | ADDR 0x%02X | ACK CH 0x%02X | 100k\n"
            "SEN0502 PID OK %u/4 | R1 %u  R2 %u  R3 %u  R4 %u\n"
            "STEP/CLICK G1 %u G2 %u G3 %u G4 %u | BTN R1 %lu R2 %lu R3 %lu R4 %lu\n"
            "FADER DIRECT GPIO20 | %s | ADC %u | VALUE %u/1023\n"
            "FADER LED DATA GPIO45 | RMT %s",
            resetReason, loaded, kit,
            muxAddress ? "SIGNATURE OK" : "NO DETECTADO", muxAddress,
            addressAckMask, rotaryCount,
            i2c_rotaries_value(0), i2c_rotaries_value(1),
            i2c_rotaries_value(2), i2c_rotaries_value(3),
            i2c_rotaries_gain(0), i2c_rotaries_gain(1),
            i2c_rotaries_gain(2), i2c_rotaries_gain(3),
            static_cast<unsigned long>(i2c_rotaries_button_press_count(0)),
            static_cast<unsigned long>(i2c_rotaries_button_press_count(1)),
            static_cast<unsigned long>(i2c_rotaries_button_press_count(2)),
            static_cast<unsigned long>(i2c_rotaries_button_press_count(3)),
            p4_fader_detected() ? "READY" : "WAIT",
            p4_fader_raw(), p4_fader_value(),
            p4_fader_led_driver_ready() ? "READY" : "ERROR");
        lv_obj_set_style_text_color(s_pod_status_label,
            loaded > 0 ? RED808_SUCCESS : RED808_WARNING, 0);
    }
}

static void pod_status_modal_close_cb(lv_event_t* e) {
    if (e && lv_event_get_target(e) != lv_event_get_current_target(e)) return;
    if (s_mpd_map_modal) mpd_map_modal_close_cb(NULL);
    if (s_pod_function_modal) pod_function_modal_close_cb(NULL);
    if (s_pod_status_modal) lv_obj_del(s_pod_status_modal);
    s_pod_status_modal = NULL;
    s_pod_status_label = NULL;
    s_pod_screensaver_btn = NULL;
    memset(s_pod_control_value_labels, 0, sizeof(s_pod_control_value_labels));
    memset(s_pod_led_function_labels, 0, sizeof(s_pod_led_function_labels));
    memset(s_pod_led_color_labels, 0, sizeof(s_pod_led_color_labels));
}

// Preferencia persistida (settings_store.cpp) — no toca nada del hardware,
// solo si la propia P4 muestra su salvapantallas local tras 1 min sin touch.
static void pod_screensaver_refresh(void) {
    if (!s_pod_screensaver_btn) return;
    const bool on = p4.screensaver_enabled;
    apply_control_button_style(s_pod_screensaver_btn, on ? RED808_SUCCESS : RED808_BORDER, false, 8);
    lv_obj_t* lbl = lv_obj_get_child(s_pod_screensaver_btn, 0);
    if (lbl) lv_label_set_text_fmt(lbl, "SALVAPANTALLAS\n%s", on ? "ON" : "OFF");
}

static void pod_screensaver_toggle_cb(lv_event_t* /*e*/) {
    p4.screensaver_enabled = !p4.screensaver_enabled;
    pod_screensaver_refresh();
}

static void pod_status_popup_cb(lv_event_t* e) {
    LV_UNUSED(e);
    if (s_pod_status_modal) { pod_status_modal_close_cb(NULL); return; }
    if (!daisyUsb.connected()) {
        ui_show_toast("DaisyPod3 no responde", RED808_WARNING);
        return;
    }
    const auto& state = daisyUsb.state();
    if (state.pod.config.version == POD_CONFIG_VERSION) {
        s_pod_config = state.pod.config;
        s_pod_config.selectorFunction = POD_FUNC_NONE;
        s_pod_seen_revision = state.pod.revision;
    }
    daisyUsb.send(CMD_POD_GET_STATE);

    s_pod_status_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_pod_status_modal, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(s_pod_status_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_pod_status_modal, LV_OPA_70, 0);
    lv_obj_set_style_border_width(s_pod_status_modal, 0, 0);
    lv_obj_set_style_pad_all(s_pod_status_modal, 0, 0);
    lv_obj_clear_flag(s_pod_status_modal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_pod_status_modal, pod_status_modal_close_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* card = lv_obj_create(s_pod_status_modal);
    lv_obj_set_size(card, 960, 550);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_color(card, RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_dir(card, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_border_color(card, RED808_CYAN, 0);
    lv_obj_set_style_radius(card, 18, 0);
    lv_obj_set_style_pad_all(card, 14, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t* title = lv_label_create(card);
    lv_label_set_text(title, "CONTROL MAP  /  DAISYPOD3 + P4 + MIDI");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title, RED808_CYAN, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_t* subtitle = lv_label_create(card);
    lv_label_set_text(subtitle,
        "Hardware asignable + mapa completo de 2 controladores MIDI");
    lv_obj_set_style_text_font(subtitle, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(subtitle, RED808_TEXT_DIM, 0);
    lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 32);

    for (uint8_t row = 0; row < POD_CONTROL_ROW_COUNT; row++) {
        const int col = row % 4, line = row / 4;
        const int x = 10 + col * 230, y = 60 + line * 62;
        lv_obj_t* label = lv_label_create(card);
        lv_label_set_text(label, POD_CONTROL_TITLES[row]);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(label, RED808_TEXT_DIM, 0);
        lv_obj_set_pos(label, x, y);
        lv_obj_t* selectButton = lv_btn_create(card);
        lv_obj_set_size(selectButton, 216, 40);
        lv_obj_set_pos(selectButton, x, y + 15);
        apply_control_button_style(selectButton, RED808_ACCENT2, false, 10);
        lv_obj_set_style_bg_color(selectButton, RED808_SURFACE, 0);
        s_pod_control_value_labels[row] = lv_label_create(selectButton);
        lv_obj_set_width(s_pod_control_value_labels[row], 194);
        lv_obj_set_style_text_align(s_pod_control_value_labels[row],
                                    LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(s_pod_control_value_labels[row],
                                   &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(s_pod_control_value_labels[row],
                                    RED808_TEXT, 0);
        lv_obj_center(s_pod_control_value_labels[row]);
        lv_obj_add_event_cb(selectButton, pod_control_modal_open_cb,
                            LV_EVENT_CLICKED,
                            reinterpret_cast<void*>(
                                static_cast<uintptr_t>(row)));
    }

    for (uint8_t led = 0; led < 2; led++) {
        const int y = 252 + led * 64;
        lv_obj_t* label = lv_label_create(card);
        lv_label_set_text_fmt(label, "LED %u", led + 1);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(label, led == 0 ? RED808_CYAN : RED808_ACCENT, 0);
        lv_obj_set_pos(label, 24, y + 14);
        lv_obj_t* functionButton = lv_btn_create(card);
        lv_obj_set_size(functionButton, 490, 44);
        lv_obj_set_pos(functionButton, 110, y);
        apply_control_button_style(functionButton, RED808_INFO, false, 10);
        s_pod_led_function_labels[led] = lv_label_create(functionButton);
        lv_obj_set_style_text_font(s_pod_led_function_labels[led], &lv_font_montserrat_16, 0);
        lv_obj_center(s_pod_led_function_labels[led]);
        lv_obj_add_event_cb(functionButton, pod_led_function_cycle_cb, LV_EVENT_CLICKED,
                            (void*)(intptr_t)led);
        lv_obj_t* colorButton = lv_btn_create(card);
        lv_obj_set_size(colorButton, 270, 44);
        lv_obj_set_pos(colorButton, 620, y);
        apply_control_button_style(colorButton, RED808_ACCENT, false, 10);
        s_pod_led_color_labels[led] = lv_label_create(colorButton);
        lv_obj_set_style_text_font(s_pod_led_color_labels[led], &lv_font_montserrat_16, 0);
        lv_obj_center(s_pod_led_color_labels[led]);
        lv_obj_add_event_cb(colorButton, pod_led_color_cycle_cb, LV_EVENT_CLICKED,
                            (void*)(intptr_t)led);
    }

    s_pod_status_label = lv_label_create(card);
    lv_obj_set_width(s_pod_status_label, 780);
    lv_obj_set_style_text_align(s_pod_status_label, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_set_style_text_font(s_pod_status_label, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(s_pod_status_label, 18, 382);

    lv_obj_t* midiMap = lv_btn_create(card);
    lv_obj_set_size(midiMap, 100, 42);
    lv_obj_set_pos(midiMap, 816, 392);
    apply_control_button_style(midiMap, RED808_CYAN, false, 10);
    lv_obj_t* midiMapLabel = lv_label_create(midiMap);
    lv_label_set_text(midiMapLabel, "MIDI MAP\n2 DEVICES");
    lv_obj_set_width(midiMapLabel, 88);
    lv_obj_set_style_text_align(midiMapLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(midiMapLabel, &lv_font_montserrat_12, 0);
    lv_obj_center(midiMapLabel);
    lv_obj_add_event_cb(midiMap, mpd_map_modal_open_cb,
                        LV_EVENT_CLICKED, NULL);

    lv_obj_t* close = lv_btn_create(card);
    lv_obj_set_size(close, 100, 42);
    lv_obj_set_pos(close, 816, 444);
    apply_control_button_style(close, RED808_WARNING, false, 10);
    lv_obj_t* closeLabel = lv_label_create(close);
    lv_label_set_text(closeLabel, "CLOSE");
    lv_obj_center(closeLabel);
    lv_obj_add_event_cb(close, [](lv_event_t*) { pod_status_modal_close_cb(NULL); },
                        LV_EVENT_CLICKED, NULL);

    // Preferencias — hueco libre bajo el status y a la izquierda de MIDI MAP/CLOSE.
    s_pod_screensaver_btn = lv_btn_create(card);
    lv_obj_set_size(s_pod_screensaver_btn, 220, 34);
    lv_obj_set_pos(s_pod_screensaver_btn, 18, 494);
    lv_obj_add_event_cb(s_pod_screensaver_btn, pod_screensaver_toggle_cb,
                        LV_EVENT_CLICKED, NULL);
    lv_obj_t* screensaverLbl = lv_label_create(s_pod_screensaver_btn);
    lv_obj_set_style_text_font(screensaverLbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_align(screensaverLbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(screensaverLbl);

    pod_status_modal_refresh();
    pod_status_modal_update();
    pod_screensaver_refresh();
}

// =============================================================================
// PAD LAYOUT — resize/reposition live_pad_btns for 6 display modes
// =============================================================================
static void apply_pad_layout(int mode) {
    s_pad_mode = mode;
    const int M = 8, G = 4, SCR_W = 1024, SCR_H = 600;
    int cols, count, pw, ph;
    switch (mode) {
        default:
        case 0: cols=4; count=16; pw=122;                ph=143;                break;
        case 1: cols=4; count=16; pw=(SCR_W-2*M-3*G)/4;  ph=(SCR_H-2*M-3*G)/4; break;
        case 2: cols=4; count=8;  pw=(SCR_W-2*M-3*G)/4;  ph=(SCR_H-2*M-1*G)/2; break;
        case 3: cols=2; count=4;  pw=(SCR_W-2*M-1*G)/2;  ph=(SCR_H-2*M-1*G)/2; break;
        case 4: cols=2; count=2;  pw=(SCR_W-2*M-1*G)/2;  ph=(SCR_H-2*M);        break;
        case 5: cols=1; count=1;  pw=(SCR_W-2*M);         ph=(SCR_H-2*M);        break;
    }

    bool fullscreen = (mode != 0);
    for (int i = 0; i < live_home_panel_count && i < (int)(sizeof(live_home_panels) / sizeof(live_home_panels[0])); i++) {
        if (!live_home_panels[i]) continue;
        if (fullscreen) lv_obj_add_flag(live_home_panels[i], LV_OBJ_FLAG_HIDDEN);
        else            lv_obj_clear_flag(live_home_panels[i], LV_OBJ_FLAG_HIDDEN);
    }

    for (int i = 0; i < 16; i++) {
        if (!live_pad_btns[i]) continue;
        if (i < count) {
            int c = i % cols, r = i / cols;
            lv_obj_set_size(live_pad_btns[i], pw, ph);
            lv_obj_set_pos(live_pad_btns[i], M + c*(pw+G), M + r*(ph+G));
            lv_obj_clear_flag(live_pad_btns[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(live_pad_btns[i]);
            pad_hit_store(i, M + c*(pw+G), M + r*(ph+G), pw, ph, true);
        } else {
            lv_obj_add_flag(live_pad_btns[i], LV_OBJ_FLAG_HIDDEN);
            pad_hit_store(i, 0, 0, 0, 0, false);
        }
    }
    if (s_pad_back_btn) {
        if (mode == 0) lv_obj_add_flag(s_pad_back_btn, LV_OBJ_FLAG_HIDDEN);
        else { lv_obj_clear_flag(s_pad_back_btn, LV_OBJ_FLAG_HIDDEN); lv_obj_move_foreground(s_pad_back_btn); }
    }
}

static void pad_mode_select_cb(lv_event_t* e) {
    int mode = (int)(intptr_t)lv_event_get_user_data(e);
    if (s_pad_mode_modal) { lv_obj_del(s_pad_mode_modal); s_pad_mode_modal = NULL; }
    apply_pad_layout(mode);
    if (lv_scr_act() != scr_live) ui_navigate_to(2);
}

// Kit chip click — marca el preset como pendiente para el PAD ACTIVO.
static void grid_pad_kit_select_cb(lv_event_t* e) {
    uint32_t key = (uint32_t)(intptr_t)lv_event_get_user_data(e);
    uint8_t engine = (uint8_t)(key >> 8);   // 0=808, 1=909, 2=505
    uint8_t preset = (uint8_t)(key & 0xFF); // 0..4
    if (engine > 2 || preset > 4) return;
    uint8_t pad = s_pad_inst_focus_pad;
    if (pad > 15) pad = 15;
    // Solo permite cambiar el kit si el inst pendiente del pad coincide con
    // el engine de la fila pulsada. (El refresh ya las desactiva visualmente,
    // pero defendemos por si acaso.)
    int8_t drum = pad_inst_drum_engine_idx(s_pad_inst_pending[pad]);
    if (drum != (int8_t)engine) return;
    s_pad_kit_pending[pad] = preset;
    pad_inst_modal_refresh();
}

// Mini preview icon: draws the actual cols x rows arrangement each pad-grid
// mode produces (mirrors apply_pad_layout's cols/count table) so the picker
// shows what the layout looks like instead of just naming it. Mode 0 (the
// compact non-fullscreen layout) additionally renders a dimmed panel on the
// right, representing the control deck that stays visible in that mode.
static void build_pad_grid_icon(lv_obj_t* parent, int x, int y, int mode) {
    const int box_w = 64, box_h = 36;
    lv_obj_t* box = lv_obj_create(parent);
    lv_obj_set_size(box, box_w, box_h);
    lv_obj_set_pos(box, x, y);
    lv_obj_set_style_bg_color(box, RED808_BG, 0);
    lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(box, RED808_BORDER, 0);
    lv_obj_set_style_border_width(box, 1, 0);
    lv_obj_set_style_radius(box, 4, 0);
    lv_obj_set_style_pad_all(box, 3, 0);
    lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(box, LV_OBJ_FLAG_CLICKABLE);

    int cols, rows, count;
    bool full;
    switch (mode) {
        default:
        case 0: cols = 4; rows = 4; count = 16; full = false; break;
        case 1: cols = 4; rows = 4; count = 16; full = true;  break;
        case 2: cols = 4; rows = 2; count = 8;  full = true;  break;
        case 3: cols = 2; rows = 2; count = 4;  full = true;  break;
        case 4: cols = 2; rows = 1; count = 2;  full = true;  break;
        case 5: cols = 1; rows = 1; count = 1;  full = true;  break;
    }

    const int drawable_w = box_w - 6, drawable_h = box_h - 6, gap = 1;
    int grid_w = full ? drawable_w : (int)(drawable_w * 0.58f);
    int cw = (grid_w - (cols - 1) * gap) / cols;
    int ch = (drawable_h - (rows - 1) * gap) / rows;
    if (cw < 2) cw = 2;
    if (ch < 2) ch = 2;

    for (int i = 0; i < count; i++) {
        int c = i % cols, r = i / cols;
        lv_obj_t* cell = lv_obj_create(box);
        lv_obj_set_size(cell, cw, ch);
        lv_obj_set_pos(cell, c * (cw + gap), r * (ch + gap));
        lv_obj_set_style_bg_color(cell, RED808_ACCENT2, 0);
        lv_obj_set_style_bg_opa(cell, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(cell, 0, 0);
        lv_obj_set_style_radius(cell, 1, 0);
        lv_obj_clear_flag(cell, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(cell, LV_OBJ_FLAG_CLICKABLE);
    }
    if (!full) {
        int panel_x = grid_w + 4;
        int panel_w = drawable_w - grid_w - 4;
        if (panel_w < 4) panel_w = 4;
        lv_obj_t* panel = lv_obj_create(box);
        lv_obj_set_size(panel, panel_w, drawable_h);
        lv_obj_set_pos(panel, panel_x, 0);
        lv_obj_set_style_bg_color(panel, RED808_ACCENT2, 0);
        lv_obj_set_style_bg_opa(panel, LV_OPA_30, 0);
        lv_obj_set_style_border_width(panel, 0, 0);
        lv_obj_set_style_radius(panel, 1, 0);
        lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(panel, LV_OBJ_FLAG_CLICKABLE);
    }
}

// PAD MODE modal — selector de visualización (1/2/4/8/16 pads), con icono de
// previsualización real de cada layout junto al nombre.
static void grid_pad_mode_cb(lv_event_t* e) {
    if (s_pad_mode_modal) { lv_obj_del(s_pad_mode_modal); s_pad_mode_modal = NULL; return; }

    s_pad_mode_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_pad_mode_modal, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(s_pad_mode_modal, 0, 0);
    lv_obj_set_style_bg_color(s_pad_mode_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_pad_mode_modal, LV_OPA_70, 0);
    lv_obj_set_style_border_width(s_pad_mode_modal, 0, 0);
    lv_obj_set_style_radius(s_pad_mode_modal, 0, 0);
    lv_obj_set_style_pad_all(s_pad_mode_modal, 0, 0);
    lv_obj_clear_flag(s_pad_mode_modal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_pad_mode_modal, [](lv_event_t*){
        if (s_pad_mode_modal) { lv_obj_del(s_pad_mode_modal); s_pad_mode_modal = NULL; }
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_t* card = lv_obj_create(s_pad_mode_modal);
    lv_obj_set_size(card, 420, 400);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(card, RED808_ACCENT2, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_radius(card, 16, 0);
    lv_obj_set_style_pad_all(card, 14, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* title = lv_label_create(card);
    lv_label_set_text(title, "PAD GRID");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(title, RED808_ACCENT2, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 2);

    static const char* mode_labels[] = {
        "16 PADS", "16 PADS  FULLSCREEN",
        "8 PADS  FULLSCREEN", "4 PADS  FULLSCREEN",
        "2 PADS  FULLSCREEN", "1 PAD  FULLSCREEN"
    };
    for (int i = 0; i < 6; i++) {
        bool sel = (i == s_pad_mode);
        lv_obj_t* btn = lv_btn_create(card);
        lv_obj_set_size(btn, 392, 44);
        lv_obj_set_pos(btn, 6, 44 + i*52);
        apply_control_button_style(btn, sel ? RED808_ACCENT2 : RED808_SURFACE, sel, 10);
        build_pad_grid_icon(btn, 8, 4, i);
        lv_obj_t* lbl = lv_label_create(btn);
        lv_label_set_text(lbl, mode_labels[i]);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(lbl, RED808_TEXT, 0);
        lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 84, 0);
        lv_obj_add_event_cb(btn, pad_mode_select_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
    }
}

// Daisy latches physical button edges in PodStatePayload so a short click is
// never lost between the 100 ms USB polls. Audio actions are executed locally
// by Daisy; navigation actions are consumed here because P4 owns the display.
static uint32_t s_pod_action_seen_revision = 0;
static int16_t s_pod_encoder_position_seen = 0;
static bool s_pod_encoder_position_ready = false;

static void pod_apply_navigation_action(uint8_t function, uint8_t selected_pad) {
    switch (function) {
        case POD_FUNC_BACK: {
            if (s_pod_status_modal) { pod_status_modal_close_cb(NULL); return; }
            if (s_pad_inst_modal) { pad_inst_modal_close_cb(NULL); return; }
            if (s_pad_mode_modal) {
                lv_obj_del(s_pad_mode_modal);
                s_pad_mode_modal = NULL;
                return;
            }
            int target = prev_active_screen;
            if (target == active_screen || target == 0 || target == 1
                || target == 4 || target == 5 || target == 12)
                target = 2;
            ui_navigate_to(target);
            break;
        }
        case POD_FUNC_MIXER:     ui_navigate_to(7); break;
        case POD_FUNC_FX:        ui_navigate_to(8); break;
        case POD_FUNC_SEQUENCER: ui_navigate_to(3); break;
        case POD_FUNC_PAD_GRID:
            ui_navigate_to(2);
            grid_pad_mode_cb(NULL);
            break;
        case POD_FUNC_PAD_SOUNDS:
            ui_navigate_to(2);
            s_pad_inst_focus_pad = selected_pad < 16 ? selected_pad : 15;
            grid_pad_inst_popup_cb(NULL);
            break;
        case POD_FUNC_XTRA_PADS: ui_navigate_to(6); break;
        case POD_FUNC_CONTROL_CONFIG: pod_status_popup_cb(NULL); break;
        default: break;
    }
}

static void pod_process_physical_actions(void) {
    const auto& transport = daisyUsb.state();
    if (!transport.engine_responding) {
        s_pod_encoder_position_ready = false;
        return;
    }
    if (transport.pod_revision == s_pod_action_seen_revision) return;
    s_pod_action_seen_revision = transport.pod_revision;
    if (transport.pod.config.version != POD_CONFIG_VERSION) {
        s_pod_encoder_position_ready = false;
        return;
    }

    const int16_t encoderPosition = transport.pod.encoderPosition;
    if (!s_pod_encoder_position_ready) {
        s_pod_encoder_position_seen = encoderPosition;
        s_pod_encoder_position_ready = true;
    } else {
        const int16_t encoderDelta = static_cast<int16_t>(
            static_cast<uint16_t>(encoderPosition)
            - static_cast<uint16_t>(s_pod_encoder_position_seen));
        s_pod_encoder_position_seen = encoderPosition;
        const uint8_t encoderFunction = transport.pod.config.encoderFunction;
        if (encoderDelta != 0
            && (encoderFunction == POD_FUNC_PATTERN_PREV
                || encoderFunction == POD_FUNC_PATTERN_NEXT))
            step_pattern_relative(encoderDelta);
    }

    const uint8_t events = transport.pod.buttonPressEvents;
    const uint8_t functions[3] = {
        transport.pod.config.button1Function,
        transport.pod.config.button2Function,
        transport.pod.config.encoderButtonFunction
    };
    for (uint8_t i = 0; i < 3; i++)
        if (events & (1u << i))
            pod_apply_navigation_action(functions[i], transport.pod.selectedPad);
}

// Fondo con identidad de tema: degradado horizontal bg → acento secundario.
// Es el tratamiento que estrenó el LIVE deck; ahora lo comparten todas las
// pantallas para que el tema se sienta uno solo al navegar.
static void apply_screen_theme_bg(lv_obj_t* scr) {
    lv_obj_set_style_bg_color(scr, RED808_BG, 0);
    lv_obj_set_style_bg_grad_color(scr, lv_color_mix(theme_accent2(), RED808_BG, 238), 0);
    lv_obj_set_style_bg_grad_dir(scr, LV_GRAD_DIR_HOR, 0);
}

static void create_live_screen(void) {
    scr_live = lv_obj_create(NULL);
    apply_screen_theme_bg(scr_live);
    lv_obj_clear_flag(scr_live, LV_OBJ_FLAG_SCROLLABLE);

    // 8×4 full-screen grid: 1024×600
    // Left 4 cols = pads, Right 4 cols = controls
    const int M = 8, G = 4, CG = 8, CW = 122, CH = 143;
    #define COL_X(c) ((c) < 4 ? (M + (c)*(CW+G)) : (M + 4*(CW+G) + CG + ((c)-4)*(CW+G)))
    #define ROW_Y(r) (M + (r)*(CH+G))

    lv_obj_t* pads_deck = lv_obj_create(scr_live);
    lv_obj_set_pos(pads_deck, 3, 3);
    lv_obj_set_size(pads_deck, 506, LCD_V_RES - 6);
    lv_obj_set_style_radius(pads_deck, 18, 0);
    lv_obj_set_style_bg_color(pads_deck, RED808_PANEL, 0);
    lv_obj_set_style_bg_opa(pads_deck, LV_OPA_30, 0);
    lv_obj_set_style_border_color(pads_deck, theme_border(), 0);
    lv_obj_set_style_border_opa(pads_deck, LV_OPA_30, 0);
    lv_obj_set_style_border_width(pads_deck, 1, 0);
    lv_obj_clear_flag(pads_deck, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(pads_deck, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t* control_deck = lv_obj_create(scr_live);
    lv_obj_set_pos(control_deck, 515, 3);
    lv_obj_set_size(control_deck, LCD_H_RES - 518, LCD_V_RES - 6);
    lv_obj_set_style_radius(control_deck, 18, 0);
    lv_obj_set_style_bg_color(control_deck, lv_color_mix(theme_accent2(), RED808_PANEL, 235), 0);
    lv_obj_set_style_bg_opa(control_deck, LV_OPA_40, 0);
    lv_obj_set_style_border_color(control_deck, theme_accent2(), 0);
    lv_obj_set_style_border_opa(control_deck, LV_OPA_20, 0);
    lv_obj_set_style_border_width(control_deck, 1, 0);
    lv_obj_clear_flag(control_deck, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(control_deck, LV_OBJ_FLAG_CLICKABLE);

    // Vertical separator
    lv_obj_t* sep = lv_obj_create(scr_live);
    lv_obj_set_size(sep, 2, LCD_V_RES - 2*M);
    lv_obj_set_pos(sep, M + 4*(CW+G) + CG/2 - 1, M);
    lv_obj_set_style_bg_color(sep, RED808_BORDER, 0);
    lv_obj_set_style_bg_opa(sep, LV_OPA_40, 0);
    lv_obj_set_style_border_width(sep, 0, 0);
    lv_obj_set_style_radius(sep, 1, 0);
    lv_obj_clear_flag(sep, LV_OBJ_FLAG_SCROLLABLE);
    live_home_panel_count = 0;
    live_home_panels[live_home_panel_count++] = sep;

    // === LEFT 4×4: Drum Pads (Neon Ring Style) ===
    for (int i = 0; i < 16; i++) {
        int c = i % 4, r = i / 4;
        lv_color_t tc = ui_track_color(i);

        live_pad_btns[i] = lv_btn_create(scr_live);
        lv_obj_set_size(live_pad_btns[i], CW, CH);
        lv_obj_set_pos(live_pad_btns[i], COL_X(c), ROW_Y(r));
        pad_hit_store(i, COL_X(c), ROW_Y(r), CW, CH, true);
        lv_obj_set_style_radius(live_pad_btns[i], 12, 0);
        lv_obj_set_style_bg_color(live_pad_btns[i], RED808_SURFACE, 0);
        lv_obj_set_style_bg_grad_color(live_pad_btns[i], RED808_PANEL, 0);
        lv_obj_set_style_bg_grad_dir(live_pad_btns[i], LV_GRAD_DIR_VER, 0);
        lv_obj_set_style_bg_opa(live_pad_btns[i], LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(live_pad_btns[i], 3, 0);
        lv_obj_set_style_border_color(live_pad_btns[i], tc, 0);
        lv_obj_set_style_outline_width(live_pad_btns[i], 0, 0);
        lv_obj_set_style_shadow_width(live_pad_btns[i], 0, 0);
        lv_obj_set_style_bg_color(live_pad_btns[i], tc, LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(live_pad_btns[i], RED808_SURFACE, LV_STATE_PRESSED);
        lv_obj_set_style_bg_opa(live_pad_btns[i], LV_OPA_COVER, LV_STATE_PRESSED);
        lv_obj_set_style_border_width(live_pad_btns[i], 4, LV_STATE_PRESSED);
        lv_obj_set_style_shadow_width(live_pad_btns[i], 14, LV_STATE_PRESSED);
        lv_obj_set_style_shadow_color(live_pad_btns[i], tc, LV_STATE_PRESSED);
        lv_obj_set_style_shadow_opa(live_pad_btns[i], LV_OPA_60, LV_STATE_PRESSED);
        lv_obj_set_style_pad_all(live_pad_btns[i], 0, 0);
        lv_obj_clear_flag(live_pad_btns[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(live_pad_btns[i], pad_touch_cb, LV_EVENT_PRESSED, (void*)(intptr_t)i);

        live_pad_accent_strips[i] = lv_obj_create(live_pad_btns[i]);
        lv_obj_set_size(live_pad_accent_strips[i], 6, CH - 24);
        lv_obj_align(live_pad_accent_strips[i], LV_ALIGN_LEFT_MID, 8, 0);
        lv_obj_set_style_radius(live_pad_accent_strips[i], 3, 0);
        lv_obj_set_style_bg_color(live_pad_accent_strips[i], tc, 0);
        lv_obj_set_style_bg_opa(live_pad_accent_strips[i], LV_OPA_70, 0);
        lv_obj_set_style_border_width(live_pad_accent_strips[i], 0, 0);
        lv_obj_clear_flag(live_pad_accent_strips[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(live_pad_accent_strips[i], LV_OBJ_FLAG_CLICKABLE);

        // "MIDI" badge — bottom-left corner, the one free spot on this pad.
        // Shown only while a learned MPD218 note triggers this pad.
        // Visibility refreshed alongside the rest of the pad in update_live_screen().
        live_pad_midi_badges[i] = lv_label_create(live_pad_btns[i]);
        lv_label_set_text(live_pad_midi_badges[i], "MIDI");
        lv_obj_set_style_text_font(live_pad_midi_badges[i], &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(live_pad_midi_badges[i], RED808_SUCCESS, 0);
        lv_obj_align(live_pad_midi_badges[i], LV_ALIGN_BOTTOM_LEFT, 8, -7);
        lv_obj_add_flag(live_pad_midi_badges[i], LV_OBJ_FLAG_HIDDEN);

        live_pad_num_labels[i] = lv_label_create(live_pad_btns[i]);
        lv_label_set_text_fmt(live_pad_num_labels[i], "%02d", i + 1);
        lv_obj_set_style_text_font(live_pad_num_labels[i], &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(live_pad_num_labels[i], RED808_TEXT_DIM, 0);
        lv_obj_align(live_pad_num_labels[i], LV_ALIGN_TOP_LEFT, 10, 8);

        live_pad_state_labels[i] = lv_label_create(live_pad_btns[i]);
        lv_label_set_text(live_pad_state_labels[i], "");
        lv_obj_set_style_text_font(live_pad_state_labels[i], &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(live_pad_state_labels[i], RED808_TEXT_DIM, 0);
        lv_obj_align(live_pad_state_labels[i], LV_ALIGN_TOP_RIGHT, -8, 9);

        live_pad_inst_labels[i] = lv_label_create(live_pad_btns[i]);
        lv_label_set_text(live_pad_inst_labels[i],
                          PAD_INST_SHORT[s_pad_inst_sel[i] <= 7 ? s_pad_inst_sel[i] : 0]);
        lv_obj_set_style_text_font(live_pad_inst_labels[i], &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(live_pad_inst_labels[i], tc, 0);
        lv_obj_align(live_pad_inst_labels[i], LV_ALIGN_BOTTOM_RIGHT, -8, -7);

        live_pad_labels[i] = lv_label_create(live_pad_btns[i]);
        lv_label_set_text(live_pad_labels[i], trackNames[i]);
        lv_obj_set_style_text_font(live_pad_labels[i], &lv_font_montserrat_28, 0);
        lv_obj_set_style_text_color(live_pad_labels[i], tc, 0);
        lv_obj_center(live_pad_labels[i]);

        // Spectrum bar — thin horizontal bar at bottom of pad, grows upward
        live_spectrum_bars[i] = lv_obj_create(live_pad_btns[i]);
        lv_obj_set_size(live_spectrum_bars[i], CW - 20, 0);  // starts at 0 height
        lv_obj_align(live_spectrum_bars[i], LV_ALIGN_BOTTOM_MID, 0, -4);
        lv_obj_set_style_bg_color(live_spectrum_bars[i], tc, 0);
        lv_obj_set_style_bg_opa(live_spectrum_bars[i], LV_OPA_60, 0);
        lv_obj_set_style_border_width(live_spectrum_bars[i], 0, 0);
        lv_obj_set_style_radius(live_spectrum_bars[i], 3, 0);
        lv_obj_clear_flag(live_spectrum_bars[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(live_spectrum_bars[i], LV_OBJ_FLAG_CLICKABLE);
    }

    // === RIGHT 4×4: Controls ===

    // --- Row 0: Transport ---
    // [4,0] PLAY / PAUSE
    grid_play_btn = lv_btn_create(scr_live);
    lv_obj_set_size(grid_play_btn, CW, CH);
    lv_obj_set_pos(grid_play_btn, COL_X(4), ROW_Y(0));
    apply_control_button_style(grid_play_btn, RED808_ACCENT2, true, 12);
    lv_obj_set_style_bg_color(grid_play_btn, RED808_ACCENT, 0);
    lv_obj_add_event_cb(grid_play_btn, header_play_cb, LV_EVENT_CLICKED, NULL);
    pod_register_owner_badge(grid_play_btn, POD_FUNC_PLAY_TOGGLE);
    live_home_panels[live_home_panel_count++] = grid_play_btn;
    grid_play_lbl = lv_label_create(grid_play_btn);
    lv_label_set_text(grid_play_lbl, LV_SYMBOL_PLAY "\nPLAY");
    lv_obj_set_style_text_font(grid_play_lbl, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(grid_play_lbl, RED808_TEXT, 0);
    lv_obj_set_style_text_align(grid_play_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(grid_play_lbl);

    // [5,0] PAT -
    lv_obj_t* b;
    b = create_ctrl_btn(scr_live, COL_X(5), ROW_Y(0), CW, CH,
                         LV_SYMBOL_LEFT "\nPAT", RED808_WARNING, &lv_font_montserrat_20);
    lv_obj_add_event_cb(b, header_pattern_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
    pod_register_owner_badge(b, POD_FUNC_PATTERN_PREV);
    live_home_panels[live_home_panel_count++] = b;

    // [6,0] PAT +
    b = create_ctrl_btn(scr_live, COL_X(6), ROW_Y(0), CW, CH,
                         "PAT\n" LV_SYMBOL_RIGHT, RED808_WARNING, &lv_font_montserrat_20);
    lv_obj_add_event_cb(b, header_pattern_cb, LV_EVENT_CLICKED, (void*)(intptr_t)1);
    pod_register_owner_badge(b, POD_FUNC_PATTERN_NEXT);
    live_home_panels[live_home_panel_count++] = b;

    // [7,0] Home status cell — pattern + link health
    lv_obj_t* home_cell = create_info_cell(scr_live, COL_X(7), ROW_Y(0), CW, CH,
                                           "STATUS", "P01", RED808_WARNING, &grid_bpm_lbl);
    lv_obj_add_flag(home_cell, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(home_cell, pod_status_popup_cb, LV_EVENT_CLICKED, NULL);
    pod_register_owner_badge(home_cell, POD_FUNC_CONTROL_CONFIG);
    grid_home_vol_lbl = lv_label_create(home_cell);
    lv_label_set_text(grid_home_vol_lbl, "MASTER --");
    lv_obj_set_style_text_font(grid_home_vol_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(grid_home_vol_lbl, RED808_CYAN, 0);
    lv_obj_align(grid_home_vol_lbl, LV_ALIGN_BOTTOM_MID, 0, -10);
    live_home_panels[live_home_panel_count++] = home_cell;

    // --- Row 1: STEP + Navigation ---
    // [5..7,1] Screen navigation
    static const char* nav_texts[] = {"FX", "MIXER", "PIANO"};
    static const int   nav_screens[] = {8, 7, 10};
    lv_color_t         nav_colors[] = {RED808_INFO, RED808_SUCCESS, RED808_ACCENT};
    for (int i = 0; i < 3; i++) {
        b = create_ctrl_btn(scr_live, COL_X(5 + i), ROW_Y(1), CW, CH,
                             nav_texts[i], nav_colors[i], &lv_font_montserrat_20);
        lv_obj_add_event_cb(b, grid_nav_cb, LV_EVENT_CLICKED, (void*)(intptr_t)nav_screens[i]);
        if (i == 0) {
            grid_fx_btn = b;
            pod_register_owner_badge(b, POD_FUNC_FX);
            grid_fx_active_badge = lv_label_create(b);
            lv_label_set_text(grid_fx_active_badge, LV_SYMBOL_AUDIO " ON");
            lv_obj_set_size(grid_fx_active_badge, 54, 24);
            lv_obj_align(grid_fx_active_badge, LV_ALIGN_TOP_RIGHT, -6, 6);
            lv_obj_set_style_radius(grid_fx_active_badge, 12, 0);
            lv_obj_set_style_bg_color(grid_fx_active_badge, RED808_INFO, 0);
            lv_obj_set_style_bg_opa(grid_fx_active_badge, LV_OPA_COVER, 0);
            lv_obj_set_style_text_color(grid_fx_active_badge, RED808_BG, 0);
            lv_obj_set_style_text_font(grid_fx_active_badge, &lv_font_montserrat_12, 0);
            lv_obj_set_style_text_align(grid_fx_active_badge, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_add_flag(grid_fx_active_badge, LV_OBJ_FLAG_HIDDEN);
        }
        else if (i == 1) pod_register_owner_badge(b, POD_FUNC_MIXER);
        live_home_panels[live_home_panel_count++] = b;
    }

    // --- Row 2: XTRA / PAD SOUND / PAD GRID / SYNC ---
    // [4,2] XTRA PADS
    b = create_ctrl_btn(scr_live, COL_X(4), ROW_Y(2), CW, CH,
                         "XTRA\nPADS", RED808_INFO, &lv_font_montserrat_20);
    lv_obj_add_event_cb(b, grid_nav_cb, LV_EVENT_CLICKED, (void*)(intptr_t)6);
    pod_register_owner_badge(b, POD_FUNC_XTRA_PADS);
    live_home_panels[live_home_panel_count++] = b;

    // [6,2] PAD GRID — selector de visualización (1/2/4/8/16 pads)
    grid_nr_btn = create_ctrl_btn(scr_live, COL_X(6), ROW_Y(2), CW, CH,
                                  "PAD\nGRID", RED808_ACCENT2,
                                  &lv_font_montserrat_18);
    lv_obj_set_style_border_color(grid_nr_btn, RED808_ACCENT2, 0);
    grid_nr_lbl = lv_obj_get_child(grid_nr_btn, 0);
    lv_obj_add_event_cb(grid_nr_btn, grid_pad_mode_cb, LV_EVENT_CLICKED, NULL);
    pod_register_owner_badge(grid_nr_btn, POD_FUNC_PAD_GRID);
    live_home_panels[live_home_panel_count++] = grid_nr_btn;

    // [7,2] SYNC PADS ON/OFF
    grid_sync_btn = create_ctrl_btn(scr_live, COL_X(7), ROW_Y(2), CW, CH,
                                    sync_pads_active ? "SYNC\nON" : "SYNC\nOFF",
                                    sync_pads_active ? RED808_SUCCESS : RED808_SURFACE,
                                    &lv_font_montserrat_20);
    lv_obj_set_style_border_color(grid_sync_btn,
        sync_pads_active ? RED808_CYAN : RED808_BORDER, 0);
    lv_obj_add_event_cb(grid_sync_btn, grid_sync_cb, LV_EVENT_CLICKED, NULL);
    live_home_panels[live_home_panel_count++] = grid_sync_btn;
    grid_16l_btn = NULL;
    grid_16l_lbl = NULL;

    // --- Row 3: Info Displays ---
    // [4,3] MASTER volume control
    lv_obj_t* vol_panel = lv_obj_create(scr_live);
    lv_obj_set_size(vol_panel, CW, CH);
    lv_obj_set_pos(vol_panel, COL_X(4), ROW_Y(3));
    lv_obj_set_style_radius(vol_panel, 14, 0);
    lv_obj_set_style_bg_color(vol_panel, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_color(vol_panel, RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_dir(vol_panel, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(vol_panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(vol_panel, 1, 0);
    lv_obj_set_style_border_color(vol_panel, RED808_BORDER, 0);
    lv_obj_set_style_pad_all(vol_panel, 6, 0);
    lv_obj_clear_flag(vol_panel, LV_OBJ_FLAG_SCROLLABLE);
    pod_register_owner_badge(vol_panel, POD_FUNC_MASTER_VOLUME);
    live_home_panels[live_home_panel_count++] = vol_panel;

    lv_obj_t* vol_title = lv_label_create(vol_panel);
    lv_label_set_text(vol_title, "MASTER VOL");
    lv_obj_set_style_text_font(vol_title, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(vol_title, RED808_TEXT_DIM, 0);
    lv_obj_align(vol_title, LV_ALIGN_TOP_MID, 0, 2);

    lv_obj_t* vol_minus = lv_btn_create(vol_panel);
    lv_obj_set_size(vol_minus, 50, 40);
    lv_obj_set_pos(vol_minus, 4, CH - 54);
    apply_control_button_style(vol_minus, RED808_WARNING, false, 10);
    lv_obj_set_style_transform_zoom(vol_minus, 230, LV_STATE_PRESSED);
    lv_obj_t* vol_minus_lbl = lv_label_create(vol_minus);
    lv_label_set_text(vol_minus_lbl, "-");
    lv_obj_set_style_text_font(vol_minus_lbl, &lv_font_montserrat_24, 0);
    lv_obj_center(vol_minus_lbl);
    lv_obj_add_event_cb(vol_minus, grid_master_vol_step_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
    lv_obj_add_event_cb(vol_minus, grid_master_vol_step_cb, LV_EVENT_LONG_PRESSED_REPEAT, (void*)(intptr_t)-1);

    lv_obj_t* vol_plus = lv_btn_create(vol_panel);
    lv_obj_set_size(vol_plus, 50, 40);
    lv_obj_set_pos(vol_plus, CW - 62, CH - 54);
    apply_control_button_style(vol_plus, RED808_SUCCESS, false, 10);
    lv_obj_set_style_transform_zoom(vol_plus, 230, LV_STATE_PRESSED);
    lv_obj_t* vol_plus_lbl = lv_label_create(vol_plus);
    lv_label_set_text(vol_plus_lbl, "+");
    lv_obj_set_style_text_font(vol_plus_lbl, &lv_font_montserrat_24, 0);
    lv_obj_center(vol_plus_lbl);
    lv_obj_add_event_cb(vol_plus, grid_master_vol_step_cb, LV_EVENT_CLICKED, (void*)(intptr_t)1);
    lv_obj_add_event_cb(vol_plus, grid_master_vol_step_cb, LV_EVENT_LONG_PRESSED_REPEAT, (void*)(intptr_t)1);

    grid_vol_lbl = lv_label_create(vol_panel);
    lv_label_set_text(grid_vol_lbl, "75");
    lv_obj_set_style_text_font(grid_vol_lbl, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(grid_vol_lbl, RED808_ACCENT, 0);
    lv_obj_align(grid_vol_lbl, LV_ALIGN_CENTER, 0, -12);

    // [4,1] STEP ACTIVO + mini secuencer
    lv_obj_t* step_panel = lv_obj_create(scr_live);
    lv_obj_set_size(step_panel, CW, CH);
    lv_obj_set_pos(step_panel, COL_X(4), ROW_Y(1));
    lv_obj_set_style_radius(step_panel, 14, 0);
    lv_obj_set_style_bg_color(step_panel, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_color(step_panel, RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_dir(step_panel, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(step_panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(step_panel, 1, 0);
    lv_obj_set_style_border_color(step_panel, RED808_CYAN, 0);
    lv_obj_set_style_pad_all(step_panel, 6, 0);
    lv_obj_clear_flag(step_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(step_panel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(step_panel, live_step_nav_cb, LV_EVENT_CLICKED, NULL);
    pod_register_owner_badge(step_panel, POD_FUNC_SEQUENCER);
    live_home_panels[live_home_panel_count++] = step_panel;

    // Title doubles as the SEQUENCER nav cue (the whole panel is clickable →
    // live_step_nav_cb). The chevron + accent color signal it is tappable, so
    // the step sequencer is discoverable from the LIVE hub.
    lv_obj_t* step_title = lv_label_create(step_panel);
    lv_label_set_text(step_title, "SEQUENCER " LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_font(step_title, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(step_title, RED808_CYAN, 0);
    lv_obj_align(step_title, LV_ALIGN_TOP_MID, 0, 2);

    grid_step_lbl = lv_label_create(step_panel);
    lv_label_set_text(grid_step_lbl, "--");
    lv_obj_set_style_text_font(grid_step_lbl, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(grid_step_lbl, RED808_CYAN, 0);
    lv_obj_align(grid_step_lbl, LV_ALIGN_TOP_MID, 0, 20);

    const int step_dot_cols = 8;
    const int step_dot_size = 9;
    const int step_dot_gap_x = 13;
    const int step_dot_gap_y = 14;
    const int step_grid_w = (step_dot_cols - 1) * step_dot_gap_x + step_dot_size;
    const int step_start_x = (CW - step_grid_w) / 2;
    const int step_start_y = 74;
    for (int i = 0; i < 16; i++) {
        grid_step_dots[i] = lv_obj_create(step_panel);
        lv_obj_set_size(grid_step_dots[i], step_dot_size, step_dot_size);
        lv_obj_set_pos(grid_step_dots[i],
                       step_start_x + (i % step_dot_cols) * step_dot_gap_x,
                       step_start_y + (i / step_dot_cols) * step_dot_gap_y);
        lv_obj_set_style_radius(grid_step_dots[i], 4, 0);
        lv_obj_set_style_bg_color(grid_step_dots[i], RED808_BORDER, 0);
        lv_obj_set_style_bg_opa(grid_step_dots[i], LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(grid_step_dots[i], 0, 0);
        lv_obj_clear_flag(grid_step_dots[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(grid_step_dots[i], LV_OBJ_FLAG_CLICKABLE);
    }

    // [5,2] PAD SOUND
    lv_obj_t* inst_panel = lv_obj_create(scr_live);
    lv_obj_set_size(inst_panel, CW, CH);
    lv_obj_set_pos(inst_panel, COL_X(5), ROW_Y(2));
    lv_obj_set_style_radius(inst_panel, 14, 0);
    lv_obj_set_style_bg_color(inst_panel, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_color(inst_panel, RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_dir(inst_panel, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(inst_panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(inst_panel, 2, 0);
    lv_obj_set_style_border_color(inst_panel, RED808_ACCENT2, 0);
    lv_obj_set_style_pad_all(inst_panel, 8, 0);
    lv_obj_clear_flag(inst_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(inst_panel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(inst_panel, grid_pad_inst_popup_cb, LV_EVENT_CLICKED, NULL);
    pod_register_owner_badge(inst_panel, POD_FUNC_PAD_SOUNDS);
    live_home_panels[live_home_panel_count++] = inst_panel;

    grid_pad_prev_btn = NULL;
    grid_pad_next_btn = NULL;
    grid_pad_lbl = NULL;
    grid_inst_prev_btn = NULL;
    grid_inst_next_btn = NULL;
    grid_inst_lbl = NULL;

    grid_inst_edit_btn = lv_label_create(inst_panel);
    lv_label_set_text(grid_inst_edit_btn, "PAD\nSOUND");
    lv_obj_set_width(grid_inst_edit_btn, CW - 18);
    lv_obj_set_style_text_font(grid_inst_edit_btn, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(grid_inst_edit_btn, RED808_TEXT, 0);
    lv_obj_set_style_text_align(grid_inst_edit_btn, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(grid_inst_edit_btn);
    lv_obj_add_flag(grid_inst_edit_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(grid_inst_edit_btn, grid_pad_inst_popup_cb, LV_EVENT_CLICKED, NULL);

    pad_inst_refresh_controls();

    // [5,3] SD CARD
    b = create_ctrl_btn(scr_live, COL_X(5), ROW_Y(3), CW, CH,
                         LV_SYMBOL_DRIVE "\nSD", RED808_CYAN, &lv_font_montserrat_18);
    lv_obj_add_event_cb(b, grid_nav_cb, LV_EVENT_CLICKED, (void*)(intptr_t)9);
    live_home_panels[live_home_panel_count++] = b;

    // [6,3] THEME
    b = create_ctrl_btn(scr_live, COL_X(6), ROW_Y(3), CW, CH,
                         "THEME\n" LV_SYMBOL_RIGHT, RED808_ACCENT2, &lv_font_montserrat_18);
    lv_obj_add_event_cb(b, grid_theme_cb, LV_EVENT_CLICKED, NULL);
    live_home_panels[live_home_panel_count++] = b;

    // [7,3] BPM control
    lv_obj_t* bpm_panel = lv_obj_create(scr_live);
    lv_obj_set_size(bpm_panel, CW, CH);
    lv_obj_set_pos(bpm_panel, COL_X(7), ROW_Y(3));
    lv_obj_set_style_radius(bpm_panel, 14, 0);
    lv_obj_set_style_bg_color(bpm_panel, RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_color(bpm_panel, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_dir(bpm_panel, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(bpm_panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(bpm_panel, 2, 0);
    lv_obj_set_style_border_color(bpm_panel, RED808_BORDER, 0);
    lv_obj_set_style_pad_all(bpm_panel, 6, 0);
    lv_obj_clear_flag(bpm_panel, LV_OBJ_FLAG_SCROLLABLE);
    pod_register_owner_badge(bpm_panel, POD_FUNC_TEMPO);
    live_home_panels[live_home_panel_count++] = bpm_panel;

    lv_obj_t* bpm_title = lv_label_create(bpm_panel);
    lv_label_set_text(bpm_title, "BPM");
    lv_obj_set_style_text_font(bpm_title, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(bpm_title, RED808_TEXT_DIM, 0);
    lv_obj_align(bpm_title, LV_ALIGN_TOP_MID, 0, 2);

    lv_obj_t* bpm_minus = lv_btn_create(bpm_panel);
    lv_obj_set_size(bpm_minus, 50, 40);
    lv_obj_set_pos(bpm_minus, 4, CH - 54);
    apply_control_button_style(bpm_minus, RED808_WARNING, false, 10);
    lv_obj_set_style_transform_zoom(bpm_minus, 230, LV_STATE_PRESSED);
    lv_obj_t* bpm_minus_lbl = lv_label_create(bpm_minus);
    lv_label_set_text(bpm_minus_lbl, "-");
    lv_obj_set_style_text_font(bpm_minus_lbl, &lv_font_montserrat_24, 0);
    lv_obj_center(bpm_minus_lbl);
    lv_obj_add_event_cb(bpm_minus, grid_bpm_step_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
    lv_obj_add_event_cb(bpm_minus, grid_bpm_step_cb, LV_EVENT_LONG_PRESSED_REPEAT, (void*)(intptr_t)-1);

    lv_obj_t* bpm_plus = lv_btn_create(bpm_panel);
    lv_obj_set_size(bpm_plus, 50, 40);
    lv_obj_set_pos(bpm_plus, CW - 62, CH - 54);
    apply_control_button_style(bpm_plus, RED808_SUCCESS, false, 10);
    lv_obj_set_style_transform_zoom(bpm_plus, 230, LV_STATE_PRESSED);
    lv_obj_t* bpm_plus_lbl = lv_label_create(bpm_plus);
    lv_label_set_text(bpm_plus_lbl, "+");
    lv_obj_set_style_text_font(bpm_plus_lbl, &lv_font_montserrat_24, 0);
    lv_obj_center(bpm_plus_lbl);
    lv_obj_add_event_cb(bpm_plus, grid_bpm_step_cb, LV_EVENT_CLICKED, (void*)(intptr_t)1);
    lv_obj_add_event_cb(bpm_plus, grid_bpm_step_cb, LV_EVENT_LONG_PRESSED_REPEAT, (void*)(intptr_t)1);

    grid_pat_lbl = lv_label_create(bpm_panel);
    lv_label_set_text(grid_pat_lbl, "120");
    lv_obj_set_style_text_font(grid_pat_lbl, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(grid_pat_lbl, RED808_CYAN, 0);
    lv_obj_align(grid_pat_lbl, LV_ALIGN_CENTER, 0, -12);

    grid_tempo_ref_lbl = lv_label_create(bpm_panel);
    lv_label_set_text(grid_tempo_ref_lbl, "");
    lv_obj_set_width(grid_tempo_ref_lbl, CW - 16);
    lv_obj_set_style_text_font(grid_tempo_ref_lbl, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_align(grid_tempo_ref_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(grid_tempo_ref_lbl, LV_ALIGN_CENTER, 0, 18);
    lv_obj_add_flag(grid_tempo_ref_lbl, LV_OBJ_FLAG_HIDDEN);

    // Floating back button — shown only in FS pad modes (mode 1-5)
    s_pad_back_btn = lv_btn_create(scr_live);
    lv_obj_set_size(s_pad_back_btn, 72, 36);
    lv_obj_set_pos(s_pad_back_btn, 1024 - 8 - 72, 8);
    apply_control_button_style(s_pad_back_btn, RED808_BORDER, true, 8);
    lv_obj_set_style_bg_color(s_pad_back_btn, RED808_SURFACE, 0);
    lv_obj_set_style_bg_opa(s_pad_back_btn, LV_OPA_90, 0);
    lv_obj_t* back_lbl2 = lv_label_create(s_pad_back_btn);
    lv_label_set_text(back_lbl2, LV_SYMBOL_HOME);
    lv_obj_set_style_text_font(back_lbl2, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(back_lbl2, RED808_TEXT, 0);
    lv_obj_center(back_lbl2);
    lv_obj_add_flag(s_pad_back_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(s_pad_back_btn, [](lv_event_t*){ apply_pad_layout(0); }, LV_EVENT_CLICKED, NULL);
    pod_register_owner_badge(s_pad_back_btn, POD_FUNC_BACK);

    // MIDI (MPD218) activity badge, top-right corner. Created last (and thus
    // on top) so it survives whatever else fills the control deck's corner.
    s_live_midi_badge = ui_create_midi_badge(scr_live, LCD_H_RES - 24, 10);

    #undef COL_X
    #undef ROW_Y
}

static void update_live_screen(void) {
    unsigned long now = millis();

    // ── MPC-style pad fade: velocity-weighted exponential decay ──
    // Each pad maps its current "brightness" to one of 8 bands (0 = idle,
    // 7 = peak hit). LVGL only re-invalidates a pad when its band actually
    // changes, keeping partial refresh cheap even when all 16 pads fire.
    static int prev_sync_step = -1;
    bool step_changed = (p4.current_step != prev_sync_step);
    if (step_changed) prev_sync_step = p4.current_step;
    static uint8_t pad_prev_band[16] = {};
    static bool pad_prev_audible_for_flash[16] = {};
    bool anySolo = false;
    for (int i = 0; i < 16; ++i) anySolo |= p4.track_solo[i];
    // Theme reload recreates every widget; force-invalidate the dirty caches
    // or recreated pads/labels keep skipping their first style repaint.
    static uint32_t live_gen = 0;
    bool force_refresh = (live_gen != s_ui_refresh_gen);
    if (force_refresh) {
        live_gen = s_ui_refresh_gen;
        for (int i = 0; i < 16; i++) {
            pad_prev_band[i] = 0xFF;
            pad_prev_audible_for_flash[i] = !p4.track_muted[i]
                && (!anySolo || p4.track_solo[i]);
        }
        prev_sync_step = -1;
        step_changed = true;
    }
    for (int i = 0; i < 16; i++) {
        if (!live_pad_btns[i]) continue;

        bool muted = p4.track_muted[i];
        bool audible = !muted && (!anySolo || p4.track_solo[i]);
        bool audible_changed = (audible != pad_prev_audible_for_flash[i]);
        if (audible_changed) {
            pad_prev_audible_for_flash[i] = audible;
            pad_prev_band[i] = 0xFF;
        }

        uint8_t band = 0;
        uint8_t vel  = audible ? s_pad_flash_vel[i] : 0;
        if (!audible) s_pad_flash_vel[i] = 0;
        if (vel) {
            unsigned long el = now - s_pad_flash_start_ms[i];
            if (el >= (unsigned long)FADE_MS) {
                s_pad_flash_vel[i] = 0;   // fade complete → back to idle
            } else {
                float t   = (float)el / (float)FADE_MS;           // 0..1
                float env = expf(-3.2f * t);                       // 1→~0.04
                float b   = (vel / 127.0f) * env * 7.999f;
                band = (uint8_t)b;
                if (band > 7) band = 7;
            }
        }
        // Sequencer sync floor: if this pad is active on the current step,
        // render at mid brightness so the groove is always visible.
        if (audible && sync_pads_active && p4.is_playing && live_step_hit(i)) {
            if (band < 4) band = 4;
        }
        if (band == pad_prev_band[i]) continue;
        pad_prev_band[i] = band;

        lv_color_t tc = ui_track_color(i);

        if (!audible) {
            lv_obj_set_style_bg_color(live_pad_btns[i], RED808_SURFACE, 0);
            lv_obj_set_style_bg_grad_color(live_pad_btns[i], RED808_PANEL, 0);
            lv_obj_set_style_bg_opa(live_pad_btns[i], LV_OPA_70, 0);
            lv_obj_set_style_border_width(live_pad_btns[i], 3, 0);
            lv_obj_set_style_border_color(live_pad_btns[i], RED808_TEXT_DIM, 0);
            lv_obj_set_style_border_opa(live_pad_btns[i], LV_OPA_50, 0);
            if (live_pad_accent_strips[i]) {
                lv_obj_set_style_bg_color(live_pad_accent_strips[i], RED808_TEXT_DIM, 0);
                lv_obj_set_style_bg_opa(live_pad_accent_strips[i], LV_OPA_40, 0);
            }
            if (live_pad_labels[i])
                lv_obj_set_style_text_color(live_pad_labels[i], RED808_TEXT_DIM, 0);
            if (live_pad_num_labels[i])
                lv_obj_set_style_text_color(live_pad_num_labels[i], RED808_TEXT_DIM, 0);
            continue;
        }

        if (band == 0) {
            // Idle: dark surface + colored ring
            lv_obj_set_style_bg_color(live_pad_btns[i], RED808_SURFACE, 0);
            lv_obj_set_style_bg_grad_color(live_pad_btns[i], RED808_PANEL, 0);
            lv_obj_set_style_bg_opa(live_pad_btns[i], LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(live_pad_btns[i], 3, 0);
            lv_obj_set_style_border_color(live_pad_btns[i], tc, 0);
            lv_obj_set_style_border_opa(live_pad_btns[i], LV_OPA_COVER, 0);
            if (live_pad_accent_strips[i])
                lv_obj_set_style_bg_opa(live_pad_accent_strips[i], LV_OPA_70, 0);
            if (live_pad_labels[i])
                lv_obj_set_style_text_color(live_pad_labels[i], ui_track_label_color(i, false), 0);
            if (live_pad_num_labels[i])
                lv_obj_set_style_text_color(live_pad_num_labels[i], RED808_TEXT_DIM, 0);
        } else {
            // Lit: keep a solid pad body throughout the decay. The previous
            // 32..255 opacity ramp made band 1 almost transparent, so the pad
            // appeared to disappear for a frame before returning to idle.
            lv_opa_t opa_fill = (lv_opa_t)(185 + (band * 70) / 7);
            lv_obj_set_style_bg_color(live_pad_btns[i], tc, 0);
            lv_obj_set_style_bg_grad_color(live_pad_btns[i], RED808_SURFACE, 0);
            lv_obj_set_style_bg_opa(live_pad_btns[i], opa_fill, 0);
            // Border widens on hard hits (band 6-7) and stays colored for soft
            lv_coord_t bw = (band >= 6) ? 4 : 3;
            lv_obj_set_style_border_width(live_pad_btns[i], bw, 0);
            lv_color_t bc = (band >= 6) ? lv_color_white() : tc;
            lv_obj_set_style_border_color(live_pad_btns[i], bc, 0);
            lv_obj_set_style_border_opa(live_pad_btns[i], LV_OPA_COVER, 0);
            if (live_pad_labels[i])
                lv_obj_set_style_text_color(live_pad_labels[i], ui_track_label_color(i, band >= 5), 0);
            if (live_pad_num_labels[i])
                lv_obj_set_style_text_color(live_pad_num_labels[i],
                    (band >= 5 && ui_track_color_is_light(i)) ? RED808_BG : (band >= 5 ? lv_color_white() : RED808_TEXT), 0);
            if (live_pad_accent_strips[i])
                lv_obj_set_style_bg_opa(live_pad_accent_strips[i], band >= 5 ? LV_OPA_COVER : LV_OPA_80, 0);
        }
    }

    // Per-pad performance state: keep mute/solo/current-step readable on the
    // pad itself so the live surface works as an instrument, not just a grid.
    static bool prev_muted[16] = {};
    static bool prev_solo[16] = {};
    static bool prev_step_lit[16] = {};
    static bool prev_live_playing = false;
    for (int i = 0; i < 16; i++) {
        if (!live_pad_btns[i]) continue;
        bool muted = p4.track_muted[i];
        bool solo = p4.track_solo[i];
        bool isolated = anySolo && !solo;
        bool step_lit = !muted && !isolated && p4.is_playing && live_step_hit(i);
        if (!force_refresh &&
            muted == prev_muted[i] && solo == prev_solo[i] &&
            step_lit == prev_step_lit[i] && p4.is_playing == prev_live_playing) {
            continue;
        }
        prev_muted[i] = muted;
        prev_solo[i] = solo;
        prev_step_lit[i] = step_lit;

        lv_color_t tc = ui_track_color(i);
        if (live_pad_state_labels[i]) {
            if (solo) {
                lv_label_set_text(live_pad_state_labels[i], "SOLO");
                lv_obj_set_style_text_color(live_pad_state_labels[i], RED808_WARNING, 0);
            } else if (muted) {
                lv_label_set_text(live_pad_state_labels[i], "MUTE");
                lv_obj_set_style_text_color(live_pad_state_labels[i], RED808_ERROR, 0);
            } else {
                lv_label_set_text(live_pad_state_labels[i], "");
            }
        }
        lv_obj_set_style_border_color(live_pad_btns[i],
            (muted || isolated) ? RED808_TEXT_DIM : (solo ? RED808_WARNING : tc), 0);
        lv_obj_set_style_border_opa(live_pad_btns[i],
            (muted || isolated) ? LV_OPA_50 : LV_OPA_COVER, 0);
        if (live_pad_labels[i] && (muted || isolated))
            lv_obj_set_style_text_color(live_pad_labels[i], RED808_TEXT_DIM, 0);
        if (live_pad_accent_strips[i]) {
            lv_obj_set_style_bg_color(live_pad_accent_strips[i], solo ? RED808_WARNING : ((muted || isolated) ? RED808_TEXT_DIM : tc), 0);
            lv_obj_set_style_bg_opa(live_pad_accent_strips[i],
                (muted || isolated) ? LV_OPA_40 : LV_OPA_80, 0);
        }
    }
    prev_live_playing = p4.is_playing;

    // HOME reveals active processing before FX LAB is opened. The icon plus
    // border change is intentionally redundant so state is not color-only.
    static int8_t prev_home_fx_active = -1;
    static lv_obj_t* prev_home_fx_badge = NULL;
    const bool homeFxActive = fx_any_active();
    if(grid_fx_btn && grid_fx_active_badge
       && (prev_home_fx_active != static_cast<int8_t>(homeFxActive)
           || prev_home_fx_badge != grid_fx_active_badge))
    {
        prev_home_fx_active = homeFxActive ? 1 : 0;
        prev_home_fx_badge = grid_fx_active_badge;
        if(homeFxActive)
        {
            lv_obj_clear_flag(grid_fx_active_badge, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_border_width(grid_fx_btn, 3, 0);
            lv_obj_set_style_border_color(grid_fx_btn, RED808_CYAN, 0);
            lv_obj_set_style_shadow_width(grid_fx_btn, 12, 0);
            lv_obj_set_style_shadow_color(grid_fx_btn, RED808_INFO, 0);
            lv_obj_set_style_shadow_opa(grid_fx_btn, LV_OPA_30, 0);
        }
        else
        {
            lv_obj_add_flag(grid_fx_active_badge, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_border_width(grid_fx_btn, 2, 0);
            lv_obj_set_style_border_color(grid_fx_btn, RED808_INFO, 0);
            lv_obj_set_style_shadow_width(grid_fx_btn, 0, 0);
            lv_obj_set_style_shadow_opa(grid_fx_btn, LV_OPA_0, 0);
        }
    }

    // Play button state
    static bool gp_prev_play = false;
    if (force_refresh) {
        gp_prev_play = !p4.is_playing;   // force label/style repaint below
    }
    if (grid_play_btn && grid_play_lbl && p4.is_playing != gp_prev_play) {
        gp_prev_play = p4.is_playing;
        lv_label_set_text(grid_play_lbl, p4.is_playing
            ? LV_SYMBOL_PAUSE "\nPAUSE" : LV_SYMBOL_PLAY "\nPLAY");
        lv_obj_set_style_bg_color(grid_play_btn,
            p4.is_playing ? RED808_SUCCESS : RED808_ACCENT, 0);
        lv_obj_set_style_border_color(grid_play_btn,
            p4.is_playing ? RED808_CYAN : RED808_ACCENT2, 0);
    }

    // Top status cell: pattern + active Master transport.
    static int gp_prev_pat_top = -1;
    if (grid_bpm_lbl && p4.current_pattern != gp_prev_pat_top) {
        gp_prev_pat_top = p4.current_pattern;
        lv_label_set_text_fmt(grid_bpm_lbl, "P%02d", p4.current_pattern + 1);
    }

    // 0=offline, 2=USB application ready, 4=CDC enumerated/handshake pending.
    // Track the label pointer too: theme reloads recreate widgets while these
    // function-local caches survive.
    static int8_t gp_prev_mstr_transport = -1;
    static lv_obj_t* gp_prev_mstr_transport_lbl = NULL;
    static uint16_t gp_prev_protocol_version = 0xFFFFu;
    static uint32_t gp_prev_link_rtt = UINT32_MAX;
    const bool mstr_on = ui_master_link_display_on();
    const uint16_t protocolVersion = daisyUsb.state().protocol_version;
    const bool protocolMismatch = daisyUsb.state().link_ready
        && protocolVersion != 0 && protocolVersion != RED808_PROTOCOL_VERSION;
    const int8_t mstr_transport = daisyUsb.connected() ? 2
                                  : protocolMismatch ? 5
                                  : daisyUsb.state().link_ready ? 4 : 0;
    const uint32_t linkRtt = daisyUsb.state().round_trip_ms;
    if (grid_home_vol_lbl &&
        (mstr_transport != gp_prev_mstr_transport ||
         grid_home_vol_lbl != gp_prev_mstr_transport_lbl ||
         protocolVersion != gp_prev_protocol_version ||
         linkRtt != gp_prev_link_rtt)) {
        gp_prev_mstr_transport = mstr_transport;
        gp_prev_mstr_transport_lbl = grid_home_vol_lbl;
        gp_prev_protocol_version = protocolVersion;
        gp_prev_link_rtt = linkRtt;
        if (mstr_transport == 2 && protocolVersion != 0) {
            if (linkRtt == 0) {
                lv_label_set_text_fmt(grid_home_vol_lbl, "USB %u.%u <1ms",
                    (protocolVersion >> 8) & 0xFFu,
                    protocolVersion & 0xFFu);
            } else {
                lv_label_set_text_fmt(grid_home_vol_lbl, "USB %u.%u %lums",
                    (protocolVersion >> 8) & 0xFFu,
                    protocolVersion & 0xFFu,
                    static_cast<unsigned long>(linkRtt));
            }
        } else if (mstr_transport == 5) {
            lv_label_set_text_fmt(grid_home_vol_lbl, "UPDATE %u.%u",
                (protocolVersion >> 8) & 0xFFu, protocolVersion & 0xFFu);
        } else {
            const char* status = mstr_transport == 2 ? "DAISY USB"
                               : mstr_transport == 4 ? "USB WAIT"
                                                     : "DAISY --";
            lv_label_set_text(grid_home_vol_lbl, status);
        }
        lv_obj_set_style_text_color(grid_home_vol_lbl,
            mstr_on ? RED808_SUCCESS : RED808_TEXT_DIM, 0);
    }

    // Dedicated HOME controls labels
    static int gp_prev_home_master = -1;
    static lv_obj_t* gp_prev_home_master_lbl = NULL;
    if (grid_vol_lbl && (p4.master_volume != gp_prev_home_master || grid_vol_lbl != gp_prev_home_master_lbl)) {
        gp_prev_home_master = p4.master_volume;
        gp_prev_home_master_lbl = grid_vol_lbl;
        lv_label_set_text_fmt(grid_vol_lbl, "%d", p4.master_volume);
        lv_obj_clear_flag(grid_vol_lbl, LV_OBJ_FLAG_HIDDEN);
    }

    static int gp_prev_home_bpm = -1;
    static int gp_prev_home_original_bpm = -1;
    static lv_obj_t* gp_prev_home_bpm_lbl = NULL;
    if (grid_pat_lbl && (p4.bpm_int * 10 + p4.bpm_frac != gp_prev_home_bpm
            || p4.original_bpm_x10 != gp_prev_home_original_bpm
            || grid_pat_lbl != gp_prev_home_bpm_lbl)) {
        const int current_x10 = p4.bpm_int * 10 + p4.bpm_frac;
        gp_prev_home_bpm = current_x10;
        gp_prev_home_original_bpm = p4.original_bpm_x10;
        gp_prev_home_bpm_lbl = grid_pat_lbl;
        lv_label_set_text_fmt(grid_pat_lbl, "%d.%d", p4.bpm_int, p4.bpm_frac);
        lv_obj_clear_flag(grid_pat_lbl, LV_OBJ_FLAG_HIDDEN);
        if (grid_tempo_ref_lbl) {
            const int delta_x10 = current_x10 - p4.original_bpm_x10;
            if (p4.original_bpm_x10 > 0 && delta_x10 != 0) {
                const char* direction = delta_x10 > 0 ? "FAST" : "SLOW";
                lv_label_set_text_fmt(grid_tempo_ref_lbl, "%s %+.1f | ORIG %.1f",
                    direction, delta_x10 / 10.0f,
                    p4.original_bpm_x10 / 10.0f);
                lv_obj_set_style_text_color(grid_tempo_ref_lbl,
                    delta_x10 > 0 ? RED808_WARNING : RED808_INFO, 0);
                lv_obj_clear_flag(grid_tempo_ref_lbl, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(grid_tempo_ref_lbl, LV_OBJ_FLAG_HIDDEN);
            }
        }
    }

    // Step — only show the running step while playing; show "--" when paused
    // or disconnected so the counter doesn't appear to run on the home screen.
    static int gp_prev_step = -2;   // -2 = never set, -1 = currently showing "--"
    if (grid_step_lbl) {
        if (!p4.is_playing) {
            if (gp_prev_step != -1) {
                gp_prev_step = -1;
                lv_label_set_text(grid_step_lbl, "--");
            }
        } else if (p4.current_step != gp_prev_step) {
            gp_prev_step = p4.current_step;
            lv_label_set_text_fmt(grid_step_lbl, "%02d", p4.current_step + 1);
        }
    }

    static int live_prev_step_dot = -2;
    if (live_prev_step_dot != gp_prev_step) {
        live_prev_step_dot = gp_prev_step;
        for (int i = 0; i < 16; i++) {
            if (!grid_step_dots[i]) continue;
            bool active_dot = p4.is_playing && i == p4.current_step;
            bool has_hits = p4.is_playing;
            if (has_hits) {
                has_hits = false;
                for (int track = 0; track < 16; track++) {
                    if (p4.steps[track][i]) { has_hits = true; break; }
                }
            }
            lv_obj_set_style_bg_color(grid_step_dots[i], active_dot ? RED808_WARNING : (has_hits ? RED808_CYAN : RED808_BORDER), 0);
            lv_obj_set_style_bg_opa(grid_step_dots[i], active_dot ? LV_OPA_COVER : (has_hits ? LV_OPA_70 : LV_OPA_40), 0);
        }
    }

    // 16 Levels source pad label tracking — keeps the right-side button in
    // sync with whichever pad the player last tapped outside of 16L mode.
    static uint8_t prev_16l_src = 255;
    if (grid_16l_lbl && s_16l_active) {
        uint8_t cur = s_16l_src_pad;
        if (cur != prev_16l_src) {
            prev_16l_src = cur;
            lv_label_set_text_fmt(grid_16l_lbl, "16 LVL\nSRC %d", cur + 1);
        }
    } else {
        prev_16l_src = 255;
    }

    // Tremolo hold: neon ring responds to X(rate) and Y(amplitude), no marker dot.
    static bool prev_trem_visible[16] = {};
    static uint8_t prev_trem_x[16] = {};
    static uint8_t prev_trem_y[16] = {};
    static uint8_t prev_trem_phase[16] = {};
    for (int i = 0; i < 16; i++) {
        if (!live_pad_btns[i]) continue;
        bool visible = s_pad_held[i] && !p4.track_muted[i] &&
                       !lv_obj_has_flag(live_pad_btns[i], LV_OBJ_FLAG_HIDDEN);
        uint8_t x = s_pad_hold_x[i];
        uint8_t y = s_pad_hold_y[i];
        uint8_t phase = s_pad_roll_phase[i] & 0x07;
        if (!visible) {
            if (prev_trem_visible[i]) {
                lv_obj_set_style_outline_width(live_pad_btns[i], 0, 0);
                lv_obj_set_style_shadow_width(live_pad_btns[i], 0, 0);
                lv_obj_set_style_shadow_opa(live_pad_btns[i], LV_OPA_0, 0);
                pad_prev_band[i] = 0xFF;
            }
            prev_trem_visible[i] = false;
            continue;
        }

        if (visible == prev_trem_visible[i] && x == prev_trem_x[i] &&
            y == prev_trem_y[i] && phase == prev_trem_phase[i]) {
            continue;
        }
        prev_trem_visible[i] = true;
        prev_trem_x[i] = x;
        prev_trem_y[i] = y;
        prev_trem_phase[i] = phase;

        uint8_t amp = (uint8_t)(127U - (y > 127 ? 127 : y));
        uint8_t speed = x > 127 ? 127 : x;
        uint8_t pulse = (phase <= 4) ? (uint8_t)(phase * 16U) : (uint8_t)((8U - phase) * 16U);
        lv_color_t neon = lv_color_hex(ui_tremolo_neon_hex(speed, y));
        lv_opa_t fill_opa = (lv_opa_t)(78 + ((uint16_t)amp * 112U) / 127U + pulse / 3U);
        if (fill_opa > LV_OPA_COVER) fill_opa = LV_OPA_COVER;
        lv_coord_t border_w = (lv_coord_t)(4 + ((uint16_t)speed * 3U) / 127U);
        lv_coord_t outline_w = (lv_coord_t)(2 + ((uint16_t)amp * 4U) / 127U);
        lv_coord_t shadow_w = (lv_coord_t)(10 + ((uint16_t)amp * 18U) / 127U + ((uint16_t)speed * 10U) / 127U);
        lv_opa_t glow_opa = (lv_opa_t)(70 + ((uint16_t)amp * 145U) / 127U + pulse / 2U);
        if (glow_opa > LV_OPA_COVER) glow_opa = LV_OPA_COVER;

        lv_obj_set_style_bg_color(live_pad_btns[i], neon, 0);
        lv_obj_set_style_bg_grad_color(live_pad_btns[i], RED808_SURFACE, 0);
        lv_obj_set_style_bg_opa(live_pad_btns[i], fill_opa, 0);
        lv_obj_set_style_border_width(live_pad_btns[i], border_w, 0);
        lv_obj_set_style_border_color(live_pad_btns[i], neon, 0);
        lv_obj_set_style_border_opa(live_pad_btns[i], LV_OPA_COVER, 0);
        lv_obj_set_style_outline_width(live_pad_btns[i], outline_w, 0);
        lv_obj_set_style_outline_pad(live_pad_btns[i], 1, 0);
        lv_obj_set_style_outline_color(live_pad_btns[i], neon, 0);
        lv_obj_set_style_outline_opa(live_pad_btns[i], glow_opa, 0);
        lv_obj_set_style_shadow_width(live_pad_btns[i], shadow_w, 0);
        lv_obj_set_style_shadow_color(live_pad_btns[i], neon, 0);
        lv_obj_set_style_shadow_opa(live_pad_btns[i], glow_opa, 0);
        if (live_pad_labels[i])
            lv_obj_set_style_text_color(live_pad_labels[i], ui_track_color_is_light(i) ? RED808_BG : lv_color_white(), 0);
        if (live_pad_num_labels[i])
            lv_obj_set_style_text_color(live_pad_num_labels[i], ui_track_color_is_light(i) ? RED808_BG : RED808_TEXT, 0);
        if (live_pad_accent_strips[i]) {
            lv_obj_set_style_bg_color(live_pad_accent_strips[i], neon, 0);
            lv_obj_set_style_bg_opa(live_pad_accent_strips[i], LV_OPA_COVER, 0);
        }
    }

    // Spectrum bars — read from DSP task
    {
        SpectrumData sp;
        dsp_get_spectrum(&sp);
        static uint8_t prev_bar_h[16] = {};
        const int MAX_BAR_H = 60;  // max bar height in pixels (pad is 143px)
        for (int i = 0; i < 16; i++) {
            if (!live_spectrum_bars[i]) continue;
            uint8_t h_px = (uint8_t)((sp.bars[i] * MAX_BAR_H) / 255);
            if (h_px == prev_bar_h[i]) continue;
            prev_bar_h[i] = h_px;
            lv_obj_set_height(live_spectrum_bars[i], h_px);
            lv_obj_align(live_spectrum_bars[i], LV_ALIGN_BOTTOM_MID, 0, -4);
            lv_obj_set_style_bg_opa(live_spectrum_bars[i],
                h_px > 0 ? LV_OPA_60 : LV_OPA_0, 0);
        }
    }

    // "MIDI" pad badges: mapping only changes via MIDI LEARN, so this is
    // throttled well below frame rate rather than re-scanned every tick.
    static uint32_t lastMidiBadgeCheckMs = 0;
    if (now - lastMidiBadgeCheckMs >= 500) {
        lastMidiBadgeCheckMs = now;
        for (int i = 0; i < 16; i++) {
            if (!live_pad_midi_badges[i]) continue;
            if (pad_has_midi_mapping((uint8_t)i))
                lv_obj_clear_flag(live_pad_midi_badges[i], LV_OBJ_FLAG_HIDDEN);
            else
                lv_obj_add_flag(live_pad_midi_badges[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

// =============================================================================
// Generic "AUTO" popup — shared by FX LAB's RANDOM and MIXER's RANDOM MIX.
// Lets the user pick a bar cadence, flip AUTO on/off (re-applies every N
// bars while the sequencer plays, via control_random_auto_tick() in
// control_api.cpp), or just APPLY NOW once without turning AUTO on.
// =============================================================================
struct AutoModalConfig {
    const char* title;
    bool (*isActive)();
    void (*setActive)(bool);
    uint8_t (*getBars)();
    void (*setBars)(uint8_t);
    void (*applyNow)();
    bool (*isSmooth)();     // transition style: false = brusca, true = suave
    void (*setSmooth)(bool);
};

static lv_obj_t*      s_auto_modal            = NULL;
static lv_obj_t*      s_auto_modal_bars_btns[4] = {};
static lv_obj_t*      s_auto_modal_toggle_btn  = NULL;
static lv_obj_t*      s_auto_modal_smooth_btn  = NULL;
static AutoModalConfig s_auto_modal_cfg = {};
static const uint8_t  AUTO_MODAL_BAR_OPTIONS[4] = {1, 2, 4, 8};

static void auto_modal_refresh(void) {
    if (!s_auto_modal) return;
    const uint8_t bars = s_auto_modal_cfg.getBars();
    for (int i = 0; i < 4; i++) {
        lv_obj_t* btn = s_auto_modal_bars_btns[i];
        if (!btn) continue;
        const bool sel = AUTO_MODAL_BAR_OPTIONS[i] == bars;
        apply_control_button_style(btn, sel ? RED808_ACCENT : RED808_BORDER, false, 8);
    }
    if (s_auto_modal_smooth_btn && s_auto_modal_cfg.isSmooth) {
        const bool smooth = s_auto_modal_cfg.isSmooth();
        apply_control_button_style(s_auto_modal_smooth_btn,
            smooth ? RED808_CYAN : RED808_BORDER, false, 10);
        lv_obj_t* lbl = lv_obj_get_child(s_auto_modal_smooth_btn, 0);
        if (lbl) lv_label_set_text(lbl,
            smooth ? LV_SYMBOL_LOOP "  TRANSICION: SUAVE" : LV_SYMBOL_SHUFFLE "  TRANSICION: BRUSCA");
    }
    if (s_auto_modal_toggle_btn) {
        const bool active = s_auto_modal_cfg.isActive();
        apply_control_button_style(s_auto_modal_toggle_btn,
            active ? RED808_SUCCESS : RED808_BORDER, false, 10);
        lv_obj_t* lbl = lv_obj_get_child(s_auto_modal_toggle_btn, 0);
        if (lbl) lv_label_set_text(lbl,
            active ? LV_SYMBOL_OK "  AUTO: ON" : LV_SYMBOL_CLOSE "  AUTO: OFF");
    }
}

static void auto_modal_close_cb(lv_event_t* e = NULL) {
    if (e && lv_event_get_target(e) != lv_event_get_current_target(e)) return;
    if (s_auto_modal) {
        lv_obj_del(s_auto_modal);
        s_auto_modal = NULL;
        for (int i = 0; i < 4; i++) s_auto_modal_bars_btns[i] = NULL;
        s_auto_modal_toggle_btn = NULL;
        s_auto_modal_smooth_btn = NULL;
    }
}

static void auto_modal_bars_cb(lv_event_t* e) {
    const int bars = (int)(intptr_t)lv_event_get_user_data(e);
    s_auto_modal_cfg.setBars((uint8_t)bars);
    auto_modal_refresh();
}

static void auto_modal_toggle_cb(lv_event_t* e) {
    LV_UNUSED(e);
    s_auto_modal_cfg.setActive(!s_auto_modal_cfg.isActive());
    auto_modal_refresh();
}

static void auto_modal_smooth_cb(lv_event_t* e) {
    LV_UNUSED(e);
    if (s_auto_modal_cfg.isSmooth && s_auto_modal_cfg.setSmooth)
        s_auto_modal_cfg.setSmooth(!s_auto_modal_cfg.isSmooth());
    auto_modal_refresh();
}

static void auto_modal_apply_now_cb(lv_event_t* e) {
    LV_UNUSED(e);
    if (s_auto_modal_cfg.applyNow) s_auto_modal_cfg.applyNow();
}

static void auto_modal_show(const AutoModalConfig& cfg) {
    if (s_auto_modal) return;
    s_auto_modal_cfg = cfg;

    s_auto_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_auto_modal, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(s_auto_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_auto_modal, LV_OPA_80, 0);
    lv_obj_set_style_border_width(s_auto_modal, 0, 0);
    lv_obj_set_style_pad_all(s_auto_modal, 0, 0);
    lv_obj_clear_flag(s_auto_modal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_auto_modal, auto_modal_close_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* card = lv_obj_create(s_auto_modal);
    lv_obj_set_size(card, 420, 340);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_border_color(card, RED808_CYAN, 0);
    lv_obj_set_style_radius(card, 16, 0);
    lv_obj_set_style_pad_all(card, 16, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t* title = lv_label_create(card);
    lv_label_set_text(title, cfg.title);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title, RED808_CYAN, 0);
    lv_obj_set_pos(title, 0, 0);

    lv_obj_t* barsLbl = lv_label_create(card);
    lv_label_set_text(barsLbl, "CADA CUANTOS COMPASES:");
    lv_obj_set_style_text_font(barsLbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(barsLbl, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(barsLbl, 0, 42);

    for (int i = 0; i < 4; i++) {
        lv_obj_t* btn = lv_btn_create(card);
        s_auto_modal_bars_btns[i] = btn;
        lv_obj_set_size(btn, 86, 40);
        lv_obj_set_pos(btn, i * (86 + 8), 64);
        lv_obj_add_event_cb(btn, auto_modal_bars_cb, LV_EVENT_CLICKED,
                            (void*)(intptr_t)AUTO_MODAL_BAR_OPTIONS[i]);
        lv_obj_t* lbl = lv_label_create(btn);
        lv_label_set_text_fmt(lbl, "%d", AUTO_MODAL_BAR_OPTIONS[i]);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
        lv_obj_center(lbl);
    }

    s_auto_modal_smooth_btn = lv_btn_create(card);
    lv_obj_set_size(s_auto_modal_smooth_btn, 388, 44);
    lv_obj_set_pos(s_auto_modal_smooth_btn, 0, 112);
    lv_obj_add_event_cb(s_auto_modal_smooth_btn, auto_modal_smooth_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* smoothLbl = lv_label_create(s_auto_modal_smooth_btn);
    lv_obj_set_style_text_font(smoothLbl, &lv_font_montserrat_14, 0);
    lv_obj_center(smoothLbl);
    if (!cfg.isSmooth) lv_obj_add_flag(s_auto_modal_smooth_btn, LV_OBJ_FLAG_HIDDEN);

    s_auto_modal_toggle_btn = lv_btn_create(card);
    lv_obj_set_size(s_auto_modal_toggle_btn, 388, 48);
    lv_obj_set_pos(s_auto_modal_toggle_btn, 0, 164);
    lv_obj_add_event_cb(s_auto_modal_toggle_btn, auto_modal_toggle_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* toggleLbl = lv_label_create(s_auto_modal_toggle_btn);
    lv_obj_set_style_text_font(toggleLbl, &lv_font_montserrat_16, 0);
    lv_obj_center(toggleLbl);

    lv_obj_t* applyBtn = lv_btn_create(card);
    lv_obj_set_size(applyBtn, 388, 40);
    lv_obj_set_pos(applyBtn, 0, 224);
    apply_control_button_style(applyBtn, RED808_INFO, false, 10);
    lv_obj_add_event_cb(applyBtn, auto_modal_apply_now_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* applyLbl = lv_label_create(applyBtn);
    lv_label_set_text(applyLbl, LV_SYMBOL_SHUFFLE "  APLICAR AHORA");
    lv_obj_set_style_text_font(applyLbl, &lv_font_montserrat_14, 0);
    lv_obj_center(applyLbl);

    lv_obj_t* closeBtn = lv_btn_create(card);
    lv_obj_set_size(closeBtn, 388, 36);
    lv_obj_set_pos(closeBtn, 0, 272);
    apply_control_button_style(closeBtn, RED808_BORDER, false, 10);
    lv_obj_add_event_cb(closeBtn, auto_modal_close_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* closeLbl = lv_label_create(closeBtn);
    lv_label_set_text(closeLbl, "CERRAR");
    lv_obj_set_style_text_font(closeLbl, &lv_font_montserrat_14, 0);
    lv_obj_center(closeLbl);

    auto_modal_refresh();
}

// =============================================================================
// FX LAB SCREEN — dynamic grid with 12 available cards and view modes 3/6/9/12
// =============================================================================
enum FxCardKind : uint8_t {
    FX_CARD_FLANGE = 0,
    FX_CARD_DELAY,
    FX_CARD_REVERB,
    FX_CARD_FOLD,
    FX_CARD_CRUSH,
    FX_CARD_PHASER,
    FX_CARD_CUTOFF,
    FX_CARD_RESO,
    FX_CARD_DRIVE,
    FX_CARD_BITS,
    FX_CARD_SRATE,
    FX_CARD_FILTER,
    // Already fully implemented in DaisyPod3's DSP and wired into the
    // protocol (tremolo/chorus/compressor/autowah), just never had a card
    // here before — see control_send_set_*_macro in control_api.cpp.
    FX_CARD_TREMOLO,
    FX_CARD_CHORUS,
    FX_CARD_COMP,
    FX_CARD_AUTOWAH,
    // Drives gFilterMorph (CMD_FILTER_MORPH) — only audible once the FILTER
    // model is SVF MORPH (15); touching this card auto-selects that model,
    // same idea as CUTOFF/RESO auto-engaging LOWPASS.
    FX_CARD_MORPH,
    // Beat Repeat / stutter — CMD_BEAT_REPEAT already existed in DaisyPod3
    // (only ever sent a hardcoded 0 by a panic/reset path) but had no card
    // driving an actual division. The 18th FX card.
    FX_CARD_STUTTER,
};

static constexpr int FX_CARD_COUNT = 18;
static constexpr int FX_VIEW_MODE_COUNT = 4;
// 18 cards at 3-per-page is 6 pages — bumped so the page-dot row still has
// one dot per page in the densest-page (fewest-per-page) view mode.
static constexpr int FX_PAGE_DOT_COUNT = 6;
static const int fx_view_modes[FX_VIEW_MODE_COUNT] = {3, 6, 12, 18};

static int fx_page = 0;
static int fx_view_mode = 0;

static lv_obj_t* fx_cards[FX_CARD_COUNT]       = {};
static lv_obj_t* fx_arcs[FX_CARD_COUNT]        = {};
static lv_obj_t* fx_value_labels[FX_CARD_COUNT]= {};
static lv_obj_t* fx_name_labels[FX_CARD_COUNT] = {};
static lv_obj_t* fx_src_labels[FX_CARD_COUNT]  = {};
static lv_obj_t* fx_toggle_btns[FX_CARD_COUNT] = {};
static lv_obj_t* fx_pct_labels[FX_CARD_COUNT]  = {};
static lv_obj_t* fx_page_dot[FX_PAGE_DOT_COUNT]= {};
static lv_obj_t* fx_page_lbl                   = NULL;
static lv_obj_t* fx_view_btn                   = NULL;
static lv_obj_t* fx_view_lbl                   = NULL;
static lv_obj_t* fx_pattern_lbl                = NULL;
static lv_obj_t* fx_active_lbl                 = NULL;
static lv_obj_t* fx_all_off_btn                = NULL;
static lv_obj_t* s_fx_random_btn               = NULL;
static lv_obj_t* fx_midi_badges[FX_CARD_COUNT] = {};
static bool s_fx_ui_syncing = false;

// ── VIZ: alternate per-card value indicators alongside the original arc ──
// ARC is the neon circle already built for every card; LED is a horizontal
// ladder of small segments (plain lv_obj rectangles — the same bg_color/
// bg_opa toggling already proven by fx_page_dot above, rather than the
// unverified lv_led widget API); BAR is a horizontal VU-style lv_bar, the
// same widget already used for the boot progress bar. All three exist for
// every card at all times; fx_apply_viz_style() just shows/hides them, so
// switching style never needs to wait for the next value change.
enum FxVizStyle { FX_VIZ_ARC = 0, FX_VIZ_LED, FX_VIZ_BAR, FX_VIZ_STYLE_COUNT };
static constexpr int FX_LED_COUNT = 10;
static int        s_fx_viz_style = FX_VIZ_ARC;
static lv_obj_t*  s_fx_viz_btn   = NULL;
static lv_obj_t*  s_fx_viz_lbl   = NULL;
static lv_obj_t*  fx_bars[FX_CARD_COUNT] = {};
static lv_obj_t*  fx_leds[FX_CARD_COUNT][FX_LED_COUNT] = {};

// True if any learned MIDI CC maps to this FX LAB knob action. Only the
// filter-related cards have a matching KnobActionType today.
// Returns the learned CC number (0-127) for this control, or -1 if it has
// no mapping OR no MIDI activity has been seen this session yet. DIN MIDI
// has no plug-detect line, so "connected" is approximated the same way the
// LIVE/SEQ MIDI activity dots already do (ui_midi_badge_refresh): a learned
// mapping is a saved setting that outlives any one session, so without this
// gate the badge would keep showing "MIDI" from a previous session even
// with nothing plugged in — this only lights up once real traffic arrives.
static int fx_card_midi_cc_number(int cell) {
    if (control_midi_activity_revision() == 0) return -1;
    using namespace red808_mpd218;
    uint8_t knobAction;
    switch (cell) {
        case FX_CARD_CUTOFF: knobAction = KNOB_FILTER_CUTOFF;     break;
        case FX_CARD_RESO:   knobAction = KNOB_FILTER_RESONANCE;  break;
        case FX_CARD_DRIVE:  knobAction = KNOB_DISTORTION;        break;
        case FX_CARD_BITS:   knobAction = KNOB_BIT_DEPTH;         break;
        case FX_CARD_SRATE:  knobAction = KNOB_SAMPLE_RATE;       break;
        case FX_CARD_FILTER: knobAction = KNOB_FILTER_TYPE;       break;
        default: return -1;
    }
    const uint8_t count = control_midi_map_count();
    for (uint8_t i = 0; i < count; i++) {
        MidiMapEntry entry;
        if (!control_midi_map_get(i, entry)) continue;
        if (entry.kind == MIDI_MAP_KIND_CC && entry.action == knobAction)
            return entry.number;
    }
    return -1;
}
static uint32_t s_fx_toggle_last_ms[FX_CARD_COUNT] = {};
static uint32_t s_fx_any_toggle_last_ms = 0;          // global across all FX buttons
static float    s_fx_arc_anim[FX_CARD_COUNT] = {};    // file-scope for snap access
static uint32_t s_fx_arc_user_ms[FX_CARD_COUNT] = {}; // last user-touch timestamp
static uint8_t  s_fx_last_active_u7[FX_CARD_COUNT] = {64, 64, 64, 64, 64, 64, 96, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64};
// True current value of every card, updated on every send (unlike
// s_fx_last_active_u7 above, which only tracks the last non-neutral value).
// Used as the ramp start point for the smooth-transition RANDOM mode.
// TREMOLO/CHORUS/COMP/AUTOWAH/STUTTER start at 0, not 64 like the rest — 0
// is their true "off" value (matching Daisy's actual boot-time
// tremoloActive/chorusActive/compActive/autowahActive/beatRepActive ==
// false), and this array directly drives each card's mute/arc display,
// unlike s_fx_last_active_u7 above which only seeds the value a later
// re-enable restores.
static uint8_t  s_fx_current_u7[FX_CARD_COUNT]     = {64, 64, 64, 64, 64, 64, 96, 64, 64, 64, 64, 64, 0, 0, 0, 0, 0, 0};
static bool     s_fx_random_smooth = false;   // false = brusca (snap), true = suave (ramp)
static bool     fx_random_smooth_get(void) { return s_fx_random_smooth; }
static void     fx_random_smooth_set(bool v) { s_fx_random_smooth = v; }

static const char* fx_names[FX_CARD_COUNT] = {
    "FLANGE", "DELAY", "REVERB", "FOLD", "CRUSH", "PHASER",
    "CUTOFF", "RESO", "DRIVE", "BITS", "SRATE", "FILTER",
    "TREMOLO", "CHORUS", "COMP", "A.WAH", "MORPH", "STUTTER"
};

static const char* fx_filter_model_name(int type) {
    static const char* names[] = {
        "OFF", "LOWPASS", "HIGHPASS", "BANDPASS", "NOTCH",
        "ALLPASS", "PEAK", "LOW SHELF", "HIGH SHELF", "RESONANT",
        "LADDER", "SVF LP", "SVF HP", "SVF BP", "COMB", "SVF MORPH"
    };
    return names[constrain(type, 0, 15)];
}

static const uint32_t fx_colors[FX_CARD_COUNT] = {
    0xC9271B, 0xE86820, 0xF5BC31, 0xF2552F, 0xFF8C2A, 0xF7EAD7,
    0x27B0D0, 0x31D2A1, 0xF2466B, 0xD18A2B, 0x4CA8FF, 0xA17BFF,
    0x5FD9A0, 0xB07CF0, 0xFF5C8A, 0x8FE0FF, 0xE0C24C, 0xFF4FD8
};

static const char* fx_src[FX_CARD_COUNT] = {
    "DEPTH", "DRY / WET", "DRY / WET", "INPUT GAIN", "DUAL MACRO", "DEPTH",
    "FREQUENCY", "Q / RESONANCE", "DRIVE", "RESOLUTION", "HOLD RATE", "MODEL",
    "DEPTH", "DRY / WET", "SQUASH", "SENSITIVITY", "LP > BP > HP > NOTCH",
    "1/2 > 1/32 DIV"
};

static uint8_t fx_card_owner_function(int cell) {
    // TREMOLO/CHORUS/COMP/AUTOWAH/MORPH/STUTTER have no physical Daisy Pod
    // knob assigned — POD_FUNC_NONE means no ownership arbitration ever
    // blocks P4 from driving them, unlike the first 12 cards.
    static const uint8_t functions[FX_CARD_COUNT] = {
        POD_FUNC_FLANGER_DEPTH, POD_FUNC_DELAY_MIX, POD_FUNC_REVERB_MIX,
        POD_FUNC_WAVEFOLDER_GAIN, POD_FUNC_CRUSH_MACRO,
        POD_FUNC_PHASER_DEPTH, POD_FUNC_FILTER_CUTOFF,
        POD_FUNC_FILTER_RESONANCE, POD_FUNC_DISTORTION,
        POD_FUNC_BIT_DEPTH, POD_FUNC_SAMPLE_RATE, POD_FUNC_FILTER_TYPE,
        POD_FUNC_NONE, POD_FUNC_NONE, POD_FUNC_NONE, POD_FUNC_NONE, POD_FUNC_NONE,
        POD_FUNC_NONE
    };
    return (cell >= 0 && cell < FX_CARD_COUNT) ? functions[cell] : POD_FUNC_NONE;
}

static lv_color_t fx_safe_text_color(uint32_t hexColor) {
    uint8_t r = (uint8_t)((hexColor >> 16) & 0xFF);
    uint8_t g = (uint8_t)((hexColor >> 8) & 0xFF);
    uint8_t b = (uint8_t)(hexColor & 0xFF);
    int luminance = (int)(0.299f * (float)r + 0.587f * (float)g + 0.114f * (float)b);
    return (luminance > 190) ? RED808_TEXT : lv_color_hex(hexColor);
}

static bool fx_card_has_onoff(int cell) {
    return cell >= 0 && cell < FX_CARD_COUNT;
}

static int fx_card_current_value_u7(int cell) {
    switch (cell) {
        case FX_CARD_FLANGE: return p4.enc_value[0];
        case FX_CARD_DELAY:  return p4.enc_value[1];
        case FX_CARD_REVERB: return p4.enc_value[2];
        case FX_CARD_FOLD:   return p4.pot_value[3];
        case FX_CARD_CRUSH: return p4.pot_value[1];
        case FX_CARD_PHASER: return p4.pot_value[2];
        case FX_CARD_CUTOFF: {
            float norm = logf((float)constrain(p4.cutoff_hz, 20, 20000) / 20.0f)
                       / logf(1000.0f);
            return constrain((int)(norm * 127.0f + 0.5f), 0, 127);
        }
        case FX_CARD_RESO: {
            float norm = (float)(constrain(p4.resonance_x10, 7, 200) - 7) / 193.0f;
            return constrain((int)(norm * 127.0f + 0.5f), 0, 127);
        }
        case FX_CARD_DRIVE: {
            float norm = (float)constrain(p4.distortion_pct, 0, 100) / 100.0f;
            return constrain((int)(norm * 127.0f + 0.5f), 0, 127);
        }
        case FX_CARD_BITS: {
            float norm = (float)(16 - constrain(p4.bitcrush_bits, 4, 16)) / 12.0f;
            return constrain((int)(norm * 127.0f + 0.5f), 0, 127);
        }
        case FX_CARD_SRATE: {
            if (p4.sample_rate_hz <= 0) return 0;
            float norm = logf((float)constrain(p4.sample_rate_hz, 4000, 42000)
                              / 42000.0f) / logf(4000.0f / 42000.0f);
            return constrain((int)(norm * 127.0f + 0.5f), 0, 127);
        }
        case FX_CARD_FILTER: {
            float norm = (float)constrain(p4.filter_type, 0, 15) / 15.0f;
            return constrain((int)(norm * 127.0f + 0.5f), 0, 127);
        }
        // No dedicated p4.* field for these — they have no physical knob or
        // MIDI mapping to receive state from, so the value P4 itself last
        // sent (already tracked generically for every cell) is authoritative.
        case FX_CARD_TREMOLO:
        case FX_CARD_CHORUS:
        case FX_CARD_COMP:
        case FX_CARD_AUTOWAH:
        case FX_CARD_MORPH:
        case FX_CARD_STUTTER:
            return s_fx_current_u7[cell];
    }
    return 0;
}

static bool fx_card_is_muted(int cell) {
    switch (cell) {
        case FX_CARD_FLANGE: return p4.enc_muted[0];
        case FX_CARD_DELAY:  return p4.enc_muted[1];
        case FX_CARD_REVERB: return p4.enc_muted[2];
        case FX_CARD_FOLD:   return p4.pot_muted[0];
        case FX_CARD_CRUSH:  return p4.pot_value[1] == 0;
        case FX_CARD_PHASER: return p4.pot_muted[2];
        case FX_CARD_CUTOFF: return p4.cutoff_hz >= 19950;
        case FX_CARD_RESO:   return p4.resonance_x10 <= 8;
        case FX_CARD_DRIVE:  return p4.distortion_pct <= 0;
        case FX_CARD_BITS:   return p4.bitcrush_bits >= 16;
        case FX_CARD_SRATE:  return p4.sample_rate_hz <= 0;
        case FX_CARD_FILTER: return p4.filter_type == 0;
        case FX_CARD_TREMOLO:
        case FX_CARD_CHORUS:
        case FX_CARD_COMP:
        case FX_CARD_AUTOWAH:
        case FX_CARD_MORPH:
        case FX_CARD_STUTTER:
            return s_fx_current_u7[cell] == 0;
    }
    return false;
}

static const char* fx_card_button_text(int cell, bool muted) {
    if (cell == FX_CARD_FILTER) return "OFF ALL FX";
    if (fx_card_has_onoff(cell)) return muted ? "OFF" : "ON";
    return muted ? "OFF" : "ON";
}

static int fx_card_neutral_u7(int cell) {
    switch (cell) {
        case FX_CARD_CRUSH:  return 0;
        case FX_CARD_CUTOFF: return 127;
        case FX_CARD_RESO:   return 0;
        case FX_CARD_DRIVE:  return 0;
        case FX_CARD_BITS:   return 0;
        case FX_CARD_SRATE:  return 0;
        case FX_CARD_FILTER: return 0;
        default:             return 0;
    }
}

static void fx_card_send_value(int cell, int u7, bool transmit = true) {
    const uint8_t ownerFunction = fx_card_owner_function(cell);
    if (ownerFunction != POD_FUNC_NONE
        && pod_function_has_physical_owner(ownerFunction)) return;
    int neutral_u7 = fx_card_neutral_u7(cell);
    if (u7 != neutral_u7) {
        s_fx_last_active_u7[cell] = (uint8_t)u7;
    }
    if (cell >= 0 && cell < FX_CARD_COUNT) s_fx_current_u7[cell] = (uint8_t)u7;

    switch (cell) {
        case FX_CARD_FLANGE:
        case FX_CARD_DELAY:
        case FX_CARD_REVERB:
            p4.enc_value[cell] = (uint8_t)u7;
            if (transmit && control_available()) control_send_fx_enc(cell, p4.enc_value[cell], p4.enc_muted[cell]);
            break;
        case FX_CARD_FOLD:
            p4.pot_value[3] = (uint8_t)u7;
            if (transmit && control_available()) control_send_fx_pot(0, p4.pot_value[3], p4.pot_muted[0]);
            break;
        case FX_CARD_CRUSH:
            p4.pot_value[1] = (uint8_t)u7;
            if (transmit && control_available())
                control_send_set_crush_macro((uint8_t)u7);
            break;
        case FX_CARD_PHASER:
            p4.pot_value[2] = (uint8_t)u7;
            if (transmit && control_available()) control_send_fx_pot(2, p4.pot_value[2], p4.pot_muted[2]);
            break;
        case FX_CARD_CUTOFF: {
            int hz = constrain((int)(20.0f
                * powf(1000.0f, (float)u7 / 127.0f) + 0.5f), 20, 20000);
            p4.cutoff_hz = hz;
            if (transmit && control_available()) {
                // CUTOFF only has an audible effect once a filter MODEL is
                // picked on the FILTER card — every other card in this row
                // is its own self-contained on/off, so leaving CUTOFF silent
                // until a separate card is touched read as "does nothing".
                // Auto-engage a sensible default (LOWPASS = model 1) the
                // first time this leaves its neutral (fully open) position;
                // the FILTER card can still pick a different model any time.
                if (u7 != fx_card_neutral_u7(FX_CARD_CUTOFF) && p4.filter_type == 0) {
                    p4.filter_type = 1;
                    control_send_set_filter(1);
                }
                control_send_set_filter_cutoff(hz);
            }
            break;
        }
        case FX_CARD_RESO: {
            int resonanceX10 = constrain((int)(7.0f
                + ((float)u7 / 127.0f) * 193.0f + 0.5f), 7, 200);
            p4.resonance_x10 = resonanceX10;
            if (transmit && control_available()) {
                // Same reasoning as FX_CARD_CUTOFF above.
                if (u7 != fx_card_neutral_u7(FX_CARD_RESO) && p4.filter_type == 0) {
                    p4.filter_type = 1;
                    control_send_set_filter(1);
                }
                control_send_set_filter_resonance((float)resonanceX10 / 10.0f);
            }
            break;
        }
        case FX_CARD_DRIVE: {
            int drive = constrain((int)(((float)u7 / 127.0f) * 100.0f + 0.5f), 0, 100);
            p4.distortion_pct = drive;
            if (transmit && control_available()) control_send_set_distortion((float)drive / 100.0f);
            break;
        }
        case FX_CARD_BITS: {
            int bits = constrain((int)(16.0f
                - ((float)u7 / 127.0f) * 12.0f + 0.5f), 4, 16);
            p4.bitcrush_bits = bits;
            p4.pot_value[1] = (uint8_t)u7;
            if (transmit && control_available()) control_send_set_bitcrush(bits);
            break;
        }
        case FX_CARD_SRATE: {
            int sr = u7 == 0 ? 0 : constrain((int)(42000.0f
                * powf(4000.0f / 42000.0f, (float)u7 / 127.0f) + 0.5f),
                4000, 42000);
            p4.sample_rate_hz = sr;
            if (transmit && control_available()) control_send_set_sample_rate(sr);
            break;
        }
        case FX_CARD_FILTER: {
            int type = constrain((int)((float)u7 / 127.0f * 15.0f + 0.5f), 0, 15);
            p4.filter_type = type;
            if (transmit && control_available()) control_send_set_filter(type);
            break;
        }
        case FX_CARD_TREMOLO:
            if (transmit && control_available()) control_send_set_tremolo_macro((uint8_t)u7);
            break;
        case FX_CARD_CHORUS:
            if (transmit && control_available()) control_send_set_chorus_macro((uint8_t)u7);
            break;
        case FX_CARD_COMP:
            if (transmit && control_available()) control_send_set_comp_macro((uint8_t)u7);
            break;
        case FX_CARD_AUTOWAH:
            if (transmit && control_available()) control_send_set_autowah_macro((uint8_t)u7);
            break;
        case FX_CARD_MORPH:
            if (transmit && control_available()) {
                // MORPH is only audible under the SVF MORPH model — moving
                // this knob always selects it, same idea as CUTOFF/RESO
                // auto-engaging LOWPASS when the filter was off.
                if (p4.filter_type != 15) {
                    p4.filter_type = 15;
                    control_send_set_filter(15);
                }
                control_send_set_filter_morph((float)u7 / 127.0f);
            }
            break;
        case FX_CARD_STUTTER:
            if (transmit && control_available()) control_send_set_beatrepeat_macro((uint8_t)u7);
            break;
    }
    control_mark_fx_screen_dirty();
}

static void fx_card_reset(int cell) {
    switch (cell) {
        case FX_CARD_CRUSH: fx_card_send_value(cell, 0); break;
        case FX_CARD_CUTOFF: fx_card_send_value(cell, 127); break;
        case FX_CARD_RESO: fx_card_send_value(cell, 0); break;
        case FX_CARD_DRIVE: fx_card_send_value(cell, 0); break;
        case FX_CARD_BITS: fx_card_send_value(cell, 0); break;
        case FX_CARD_SRATE: fx_card_send_value(cell, 0); break;
        case FX_CARD_FILTER: fx_card_send_value(cell, 0); break;
        default: break;
    }
}

static void fx_active_name_append(char* text, size_t textSize, size_t& used,
                                  int& count, const char* name) {
    if (!text || textSize == 0 || !name || used >= textSize - 1) return;
    const size_t remaining = textSize - used;
    const int written = snprintf(text + used, remaining, "%s%s",
                                 count == 0 ? "" : " / ", name);
    if (written > 0) {
        const size_t appended = (size_t)written < remaining
            ? (size_t)written : remaining - 1;
        used += appended;
    }
    count++;
}

static bool fx_any_processing_active(void) {
    const uint8_t extraFx = daisyUsb.state().pod.reservedFx;
    return !p4.enc_muted[0] || !p4.enc_muted[1] || !p4.enc_muted[2]
        || !p4.pot_muted[0] || p4.pot_value[1] > 0 || !p4.pot_muted[2]
        || p4.distortion_pct > 0 || p4.bitcrush_bits < 16
        || p4.sample_rate_hz > 0 || p4.filter_type != 0 || extraFx != 0;
}

static void fx_active_header_refresh(void) {
    char text[192] = {};
    size_t used = 0;
    int count = 0;

    for (int cell = FX_CARD_FLANGE; cell <= FX_CARD_REVERB; ++cell)
        if (!p4.enc_muted[cell])
            fx_active_name_append(text, sizeof(text), used, count, fx_names[cell]);
    if (!p4.pot_muted[0])
        fx_active_name_append(text, sizeof(text), used, count, fx_names[FX_CARD_FOLD]);
    if (p4.pot_value[1] > 0)
        fx_active_name_append(text, sizeof(text), used, count, fx_names[FX_CARD_CRUSH]);
    if (!p4.pot_muted[2])
        fx_active_name_append(text, sizeof(text), used, count, fx_names[FX_CARD_PHASER]);
    if (p4.distortion_pct > 0)
        fx_active_name_append(text, sizeof(text), used, count, fx_names[FX_CARD_DRIVE]);
    if (p4.pot_value[1] == 0 && p4.bitcrush_bits < 16)
        fx_active_name_append(text, sizeof(text), used, count, fx_names[FX_CARD_BITS]);
    if (p4.pot_value[1] == 0 && p4.sample_rate_hz > 0)
        fx_active_name_append(text, sizeof(text), used, count, fx_names[FX_CARD_SRATE]);
    if (p4.filter_type != 0)
        fx_active_name_append(text, sizeof(text), used, count,
                              fx_filter_model_name(p4.filter_type));
    const uint8_t extraFx = daisyUsb.state().pod.reservedFx;
    if ((extraFx & POD_FX_EXTRA_AUTOWAH) != 0)
        fx_active_name_append(text, sizeof(text), used, count, "AUTOWAH");
    if ((extraFx & POD_FX_EXTRA_BEAT_REPEAT) != 0)
        fx_active_name_append(text, sizeof(text), used, count, "BEAT REPEAT");
    if ((extraFx & POD_FX_EXTRA_TAPE_STOP) != 0)
        fx_active_name_append(text, sizeof(text), used, count, "TAPE STOP");
    if ((extraFx & POD_FX_EXTRA_STEREO_WIDTH) != 0)
        fx_active_name_append(text, sizeof(text), used, count, "WIDTH");

    if (count == 0) snprintf(text, sizeof(text), "ALL FX OFF");
    else {
        char prefixed[sizeof(text)] = {};
        snprintf(prefixed, sizeof(prefixed), "ON: %s", text);
        snprintf(text, sizeof(text), "%s", prefixed);
    }

    static char previous[192] = {};
    static lv_obj_t* previousLabel = NULL;
    if (fx_active_lbl
        && (previousLabel != fx_active_lbl || strcmp(previous, text) != 0)) {
        previousLabel = fx_active_lbl;
        snprintf(previous, sizeof(previous), "%s", text);
        lv_label_set_text(fx_active_lbl, text);
        lv_obj_set_style_text_color(fx_active_lbl,
            count == 0 ? RED808_TEXT_DIM : RED808_CYAN, 0);
    }

    static bool previousAnyActive = false;
    static lv_obj_t* previousFilterButton = NULL;
    static lv_obj_t* previousHeaderButton = NULL;
    lv_obj_t* filterButton = fx_toggle_btns[FX_CARD_FILTER];
    const bool anyActive = fx_any_processing_active();
    if (previousFilterButton != filterButton
        || previousHeaderButton != fx_all_off_btn
        || previousAnyActive != anyActive) {
        previousFilterButton = filterButton;
        previousHeaderButton = fx_all_off_btn;
        previousAnyActive = anyActive;
        if (filterButton) {
            lv_obj_t* label = lv_obj_get_child(filterButton, 0);
            if (label) {
                lv_label_set_text(label, "OFF ALL FX");
                lv_obj_set_style_text_color(label,
                    anyActive ? fx_safe_text_color(fx_colors[FX_CARD_FILTER])
                              : RED808_TEXT_DIM, 0);
            }
            lv_obj_set_style_bg_opa(filterButton,
                anyActive ? LV_OPA_30 : LV_OPA_10, 0);
            lv_obj_set_style_shadow_opa(filterButton,
                anyActive ? LV_OPA_50 : LV_OPA_0, 0);
        }
        if (fx_all_off_btn) {
            lv_obj_t* label = lv_obj_get_child(fx_all_off_btn, 0);
            if (label)
                lv_obj_set_style_text_color(label,
                    anyActive ? RED808_TEXT : RED808_TEXT_DIM, 0);
            lv_obj_set_style_bg_opa(fx_all_off_btn,
                anyActive ? LV_OPA_70 : LV_OPA_10, 0);
            lv_obj_set_style_shadow_opa(fx_all_off_btn,
                anyActive ? LV_OPA_40 : LV_OPA_0, 0);
        }
    }

    for (int cell = 0; cell < FX_CARD_COUNT; ++cell) {
        if (!fx_midi_badges[cell]) continue;
        int cc = fx_card_midi_cc_number(cell);
        if (cc >= 0) {
            lv_obj_clear_flag(fx_midi_badges[cell], LV_OBJ_FLAG_HIDDEN);
            // Shows which CC it's bound to (not just "MIDI") so re-learning
            // a control onto a different CC is visible on the badge itself.
            lv_label_set_text_fmt(fx_midi_badges[cell], "CC%d", cc);
        } else {
            lv_obj_add_flag(fx_midi_badges[cell], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void fx_card_turn_off(int cell) {
    if (cell < 0 || cell >= FX_CARD_COUNT) return;
    const uint8_t ownerFunction = fx_card_owner_function(cell);
    if (ownerFunction != POD_FUNC_NONE
        && pod_function_has_physical_owner(ownerFunction)) return;

    if (cell <= FX_CARD_REVERB) {
        p4.enc_muted[cell] = true;
        if (control_available())
            control_send_fx_enc(cell, p4.enc_value[cell], true);
        s_fx_arc_anim[cell] = 0.0f;
    } else if (cell == FX_CARD_FOLD || cell == FX_CARD_PHASER) {
        const int potIndex = cell == FX_CARD_FOLD ? 0 : 2;
        const int valueIndex = cell == FX_CARD_FOLD ? 3 : 2;
        p4.pot_muted[potIndex] = true;
        if (control_available())
            control_send_fx_pot(potIndex, p4.pot_value[valueIndex], true);
        s_fx_arc_anim[cell] = 0.0f;
    } else {
        const int neutral = fx_card_neutral_u7(cell);
        fx_card_send_value(cell, neutral);
        s_fx_arc_anim[cell] = (float)neutral;
    }
}

static lv_obj_t* s_fx_random_stop_badge = NULL;

// Reflects AUTO FX (control_random_fx_active()) on the RANDOM button:
// highlighted while AUTO keeps re-randomizing every few bars, plain
// shuffle icon otherwise. Manual one-shot taps no longer change this —
// see fx_random_modal_show / the AUTO popup. The corner badge lets AUTO be
// killed with one tap, without opening the modal.
static void fx_random_btn_refresh(void) {
    if (!s_fx_random_btn) return;
    const bool active = control_random_fx_active();
    apply_control_button_style(s_fx_random_btn,
        active ? RED808_SUCCESS : RED808_ACCENT2, false, 8);
    if (s_fx_random_stop_badge) {
        if (active) lv_obj_clear_flag(s_fx_random_stop_badge, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(s_fx_random_stop_badge, LV_OBJ_FLAG_HIDDEN);
    }
}

static void fx_random_stop_badge_cb(lv_event_t* e) {
    LV_UNUSED(e);
    control_random_fx_set_active(false);
    fx_random_btn_refresh();
}

static void fx_all_turn_off(void) {
    control_send_all_fx_off();
    for (int cell = 0; cell < FX_CARD_COUNT; ++cell)
        s_fx_arc_anim[cell] = 0.0f;
    control_mark_fx_screen_dirty();
    fx_active_header_refresh();
    control_random_fx_set_active(false);
    fx_random_btn_refresh();
}

static void fx_all_off_cb(lv_event_t* e) {
    LV_UNUSED(e);
    fx_all_turn_off();
}

// ── RANDOM: tasteful global filter randomizer (not always-on) ──────────────
static uint32_t fx_random_rand_u32(void) {
    static uint32_t s = 0;
    if (s == 0) s = (uint32_t)millis() ^ 0xB5297A4Du | 1u;
    s ^= s << 13; s ^= s >> 17; s ^= s << 5;
    return s;
}
static int fx_random_range(int mn, int mx) {
    if (mx <= mn) return mn;
    return mn + (int)(fx_random_rand_u32() % (uint32_t)(mx - mn + 1));
}

// Smooth-transition ramp for RANDOM FX ("suave" mode): interpolates the
// touched cards from their current value to the new random target over
// ~700ms instead of snapping instantly ("brusca" mode, the default).
struct FxRampStep { uint8_t cell; uint8_t fromU7; uint8_t toU7; };
static FxRampStep s_fx_ramp_steps[5];
static int        s_fx_ramp_count = 0;

static void fx_ramp_anim_cb(void* /*var*/, int32_t v) {
    for (int i = 0; i < s_fx_ramp_count; i++) {
        const FxRampStep& st = s_fx_ramp_steps[i];
        int u7 = st.fromU7 + (int)(((int32_t)(st.toU7 - st.fromU7) * v) / 1000);
        fx_card_send_value(st.cell, u7);
    }
    fx_active_header_refresh();
}

static void fx_ramp_start(int count) {
    s_fx_ramp_count = count;
    if (count <= 0) return;
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, &s_fx_ramp_steps);
    lv_anim_set_exec_cb(&a, fx_ramp_anim_cb);
    lv_anim_set_values(&a, 0, 1000);
    lv_anim_set_time(&a, 700);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);
}

// Applies one random filter/cutoff/reso/drive/bits/srate pass. Called both
// by a manual tap on the RANDOM button and by AUTO FX's bar clock
// (control_random_auto_tick(), control_api.cpp) — one implementation of
// what "random" means for the FX LAB, regardless of who triggers it.
// showToast is false for AUTO's periodic re-randomization so it does not
// nag every few bars during a live set; manual taps still confirm.
void fx_random_apply(bool showToast) {
    // FILTER model: weighted toward musical, commonly-useful types, with a
    // real chance of landing back on OFF so RANDOM does not always engage
    // a filter. The filter type itself is discrete, so it always snaps
    // immediately even in "suave" mode — only the continuous knobs ramp.
    static const uint8_t filterPool[] = {
        0, 0, 1, 1, 1, 2, 2, 3, 3, 7, 8, 9, 10, 11, 11, 13
    };
    const uint8_t filterType =
        filterPool[fx_random_range(0, (int)(sizeof(filterPool) / sizeof(filterPool[0])) - 1)];
    const int filterU7 = (int)((float)filterType / 14.0f * 127.0f + 0.5f);
    fx_card_send_value(FX_CARD_FILTER, filterU7);

    int cutoffU7, resoU7;
    if (filterType == 0) {
        // No filter this round: keep cutoff/resonance neutral so the mix
        // stays clean rather than leaving a stray sweep engaged.
        cutoffU7 = 127;
        resoU7 = 0;
    } else {
        // Half the time keep the sweep fully open (filter colors the tone
        // without an audible cutoff move); otherwise land somewhere musical.
        cutoffU7 = (fx_random_range(0, 1) == 0) ? 127 : fx_random_range(40, 100);
        // Resonance is usually gentle; only occasionally spicier.
        const int resoMax = (fx_random_range(0, 3) == 0) ? 110 : 45;
        resoU7 = fx_random_range(0, resoMax);
    }

    // DRIVE / BITS / SRATE: mostly left bypassed, occasionally a subtle
    // touch — "no siempre", varied, kept professional rather than extreme.
    const int driveU7 = (fx_random_range(0, 4) == 0) ? fx_random_range(15, 55) : 0;
    const int bitsU7  = (fx_random_range(0, 5) == 0) ? fx_random_range(20, 60) : 0;
    const int srateU7 = (fx_random_range(0, 5) == 0) ? fx_random_range(15, 50) : 0;

    if (s_fx_random_smooth) {
        s_fx_ramp_steps[0] = {FX_CARD_CUTOFF, s_fx_current_u7[FX_CARD_CUTOFF], (uint8_t)cutoffU7};
        s_fx_ramp_steps[1] = {FX_CARD_RESO,   s_fx_current_u7[FX_CARD_RESO],   (uint8_t)resoU7};
        s_fx_ramp_steps[2] = {FX_CARD_DRIVE,  s_fx_current_u7[FX_CARD_DRIVE],  (uint8_t)driveU7};
        s_fx_ramp_steps[3] = {FX_CARD_BITS,   s_fx_current_u7[FX_CARD_BITS],   (uint8_t)bitsU7};
        s_fx_ramp_steps[4] = {FX_CARD_SRATE,  s_fx_current_u7[FX_CARD_SRATE],  (uint8_t)srateU7};
        fx_ramp_start(5);
    } else {
        fx_card_send_value(FX_CARD_CUTOFF, cutoffU7);
        fx_card_send_value(FX_CARD_RESO, resoU7);
        fx_card_send_value(FX_CARD_DRIVE, driveU7);
        fx_card_send_value(FX_CARD_BITS, bitsU7);
        fx_card_send_value(FX_CARD_SRATE, srateU7);
    }

    fx_active_header_refresh();
    if (showToast)
        ui_show_toast(filterType == 0 ? "RANDOM: FILTRO OFF" : "RANDOM: FILTRO APLICADO",
                      RED808_CYAN);
}

static void fx_random_apply_now(void) {
    fx_random_apply(true);
}

// Tapping the RANDOM button opens the AUTO FX popup (cadence + on/off +
// "apply now") instead of applying directly, so AUTO mode is reachable
// without hunting for a separate control.
static void fx_random_modal_show(lv_event_t* e) {
    LV_UNUSED(e);
    static const AutoModalConfig cfg = {
        "AUTO FX", control_random_fx_active, control_random_fx_set_active,
        control_random_fx_bars, control_random_fx_set_bars, fx_random_apply_now,
        fx_random_smooth_get, fx_random_smooth_set
    };
    auto_modal_show(cfg);
}

static int fx_page_count(void) {
    int perPage = fx_view_modes[constrain(fx_view_mode, 0, FX_VIEW_MODE_COUNT - 1)];
    return (FX_CARD_COUNT + perPage - 1) / perPage;
}

static void fx_apply_layout(void) {
    int perPage = fx_view_modes[constrain(fx_view_mode, 0, FX_VIEW_MODE_COUNT - 1)];
    int pageCount = fx_page_count();
    if (fx_page >= pageCount) fx_page = pageCount - 1;
    if (fx_page < 0) fx_page = 0;

    if (fx_page_lbl) lv_label_set_text_fmt(fx_page_lbl, "%d / %d", fx_page + 1, pageCount);
    if (fx_view_lbl) lv_label_set_text_fmt(fx_view_lbl, "VIEW %d", perPage);

    for (int p = 0; p < FX_PAGE_DOT_COUNT; p++) {
        if (!fx_page_dot[p]) continue;
        bool visible = p < pageCount;
        if (visible) lv_obj_clear_flag(fx_page_dot[p], LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(fx_page_dot[p], LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_bg_opa(fx_page_dot[p], p == fx_page ? LV_OPA_COVER : LV_OPA_30, 0);
    }

    int start = fx_page * perPage;
    int visibleCount = constrain(FX_CARD_COUNT - start, 0, perPage);

    // Grid geometry (cols/rows) and "how compact" the styling is are
    // properties of the VIEW MODE (perPage) itself, never of how many cards
    // happen to be left on the current page — otherwise a partially-filled
    // last page (e.g. 18 cards in VIEW 12 leaves only 6 on page 2) would
    // silently fall back to VIEW 3's oversized styling on an undersized
    // slot, which is exactly what made the new FX cards "look weird" on
    // VIEW 6/12's last page before this fix.
    int cols, rows;
    bool compact12, compact6;
    switch (perPage) {
        case 6:  cols = 3; rows = 2; compact12 = false; compact6 = true;  break;
        case 12: cols = 4; rows = 3; compact12 = true;  compact6 = false; break;
        case 18: cols = 6; rows = 3; compact12 = true;  compact6 = false; break;
        default: cols = 3; rows = 1; compact12 = false; compact6 = false; break; // VIEW 3
    }

    const int topY = 96;
    const int bottomPad = 8;
    const int sidePad = 12;
    const int gap = 10;
    int gridH = LCD_V_RES - topY - bottomPad;
    int cardW = (LCD_H_RES - sidePad * 2 - gap * (cols - 1)) / cols;
    int cardH = (gridH - gap * (rows - 1)) / rows;
    int arcSize = compact12
        ? constrain((cardW < cardH ? cardW : cardH) - 96, 60, 130)
        : (compact6
            ? constrain((cardW < cardH ? cardW : cardH) - 84, 96, 190)
            : constrain((cardW < cardH ? cardW : cardH) - 72, 120, 290));

    const lv_font_t* titleFont = compact12 ? &lv_font_montserrat_12 : (compact6 ? &lv_font_montserrat_16 : &lv_font_montserrat_22);
    const lv_font_t* valueFont = compact12 ? &lv_font_montserrat_20 : (compact6 ? &lv_font_montserrat_28 : &lv_font_montserrat_40);
    const lv_font_t* srcFont = compact12 ? &lv_font_montserrat_10 : (compact6 ? &lv_font_montserrat_10 : &lv_font_montserrat_12);
    const lv_font_t* toggleFont = compact12 ? &lv_font_montserrat_12 : (compact6 ? &lv_font_montserrat_12 : &lv_font_montserrat_16);

    int nameY = compact12 ? 4 : (compact6 ? 8 : 14);
    int srcY = compact12 ? 20 : (compact6 ? 28 : 42);
    int centerY = compact12 ? -4 : (compact6 ? -8 : -18);
    int pctY = compact12 ? 10 : (compact6 ? 4 : -2);

    for (int cell = 0; cell < FX_CARD_COUNT; cell++) {
        if (!fx_cards[cell]) continue;
        bool visible = (cell >= start && cell < start + visibleCount);
        if (!visible) {
            lv_obj_add_flag(fx_cards[cell], LV_OBJ_FLAG_HIDDEN);
            continue;
        }
        lv_obj_clear_flag(fx_cards[cell], LV_OBJ_FLAG_HIDDEN);
        int local = cell - start;
        int col = local % cols;
        int row = local / cols;
        int x = sidePad + col * (cardW + gap);
        int y = topY + row * (cardH + gap);
        lv_obj_set_pos(fx_cards[cell], x, y);
        lv_obj_set_size(fx_cards[cell], cardW, cardH);

        if (fx_name_labels[cell]) {
            lv_obj_set_width(fx_name_labels[cell], cardW);
            lv_obj_set_style_text_font(fx_name_labels[cell], titleFont, 0);
            lv_obj_align(fx_name_labels[cell], LV_ALIGN_TOP_MID, 0, nameY);
        }
        if (fx_arcs[cell]) {
            lv_obj_set_size(fx_arcs[cell], arcSize, arcSize);
            lv_obj_align(fx_arcs[cell], LV_ALIGN_CENTER, 0, centerY);
        }
        if (fx_value_labels[cell]) {
            lv_obj_set_width(fx_value_labels[cell], cardW);
            lv_obj_set_style_text_font(fx_value_labels[cell], valueFont, 0);
            lv_obj_align(fx_value_labels[cell], LV_ALIGN_CENTER, 0, centerY);
        }
        if (fx_src_labels[cell]) {
            lv_obj_set_width(fx_src_labels[cell], cardW);
            lv_obj_set_style_text_font(fx_src_labels[cell], srcFont, 0);
            lv_obj_align(fx_src_labels[cell], LV_ALIGN_TOP_MID, 0, srcY);
            if (compact12) lv_obj_add_flag(fx_src_labels[cell], LV_OBJ_FLAG_HIDDEN);
            else lv_obj_clear_flag(fx_src_labels[cell], LV_OBJ_FLAG_HIDDEN);
        }
        if (fx_toggle_btns[cell]) {
            int toggleWidth = compact12 ? 76 : (compact6 ? 90 : 104);
            if (cell == FX_CARD_FILTER)
                toggleWidth = compact12 ? 100 : (compact6 ? 112 : 126);
            lv_obj_set_size(fx_toggle_btns[cell], toggleWidth, compact12 ? 32 : (compact6 ? 38 : 42));
            lv_obj_align(fx_toggle_btns[cell], LV_ALIGN_BOTTOM_MID, 0, compact12 ? -6 : (compact6 ? -10 : -14));
            lv_obj_t* lbl = lv_obj_get_child(fx_toggle_btns[cell], 0);
            if (lbl) lv_obj_set_style_text_font(lbl, toggleFont, 0);
        }
        if (fx_pct_labels[cell]) {
            lv_obj_align(fx_pct_labels[cell], LV_ALIGN_CENTER, arcSize / 4, pctY);
            if (compact12) lv_obj_add_flag(fx_pct_labels[cell], LV_OBJ_FLAG_HIDDEN);
            else lv_obj_clear_flag(fx_pct_labels[cell], LV_OBJ_FLAG_HIDDEN);
        }

        // LED/BAR alt visualizations — sit just under the source tag,
        // independent of arcSize so they stay legible even in VIEW 18's
        // tiny cards. Visibility (which of ARC/LED/BAR shows) is handled
        // separately by fx_apply_viz_style(); this only sizes/positions.
        {
            int meterY = srcY + 16;
            int meterH = compact12 ? 10 : (compact6 ? 14 : 18);
            int meterX = compact12 ? 6 : 14;
            int meterW = cardW - meterX * 2;
            if (meterW < 20) meterW = 20;
            if (fx_bars[cell]) {
                lv_obj_set_size(fx_bars[cell], meterW, meterH);
                lv_obj_set_pos(fx_bars[cell], meterX, meterY);
            }
            int ledGap = compact12 ? 2 : 4;
            int ledW = (meterW - (FX_LED_COUNT - 1) * ledGap) / FX_LED_COUNT;
            if (ledW < 4) ledW = 4;
            for (int i = 0; i < FX_LED_COUNT; i++) {
                if (!fx_leds[cell][i]) continue;
                lv_obj_set_size(fx_leds[cell][i], ledW, meterH);
                lv_obj_set_pos(fx_leds[cell][i], meterX + i * (ledW + ledGap), meterY);
            }
        }
    }
}

// Shows the widget matching s_fx_viz_style for every card, hides the other
// two. All three (arc/bar/led row) already exist and are kept in sync with
// the live value regardless of which is visible (see update_fx_screen()),
// so switching style is instant with no stale display.
static void fx_apply_viz_style(void) {
    static const char* names[FX_VIZ_STYLE_COUNT] = {"VIZ: ARC", "VIZ: LED", "VIZ: BAR"};
    if (s_fx_viz_lbl)
        lv_label_set_text(s_fx_viz_lbl, names[constrain(s_fx_viz_style, 0, FX_VIZ_STYLE_COUNT - 1)]);
    for (int cell = 0; cell < FX_CARD_COUNT; cell++) {
        if (fx_arcs[cell]) {
            if (s_fx_viz_style == FX_VIZ_ARC) lv_obj_clear_flag(fx_arcs[cell], LV_OBJ_FLAG_HIDDEN);
            else lv_obj_add_flag(fx_arcs[cell], LV_OBJ_FLAG_HIDDEN);
        }
        if (fx_bars[cell]) {
            if (s_fx_viz_style == FX_VIZ_BAR) lv_obj_clear_flag(fx_bars[cell], LV_OBJ_FLAG_HIDDEN);
            else lv_obj_add_flag(fx_bars[cell], LV_OBJ_FLAG_HIDDEN);
        }
        for (int i = 0; i < FX_LED_COUNT; i++) {
            if (!fx_leds[cell][i]) continue;
            if (s_fx_viz_style == FX_VIZ_LED) lv_obj_clear_flag(fx_leds[cell][i], LV_OBJ_FLAG_HIDDEN);
            else lv_obj_add_flag(fx_leds[cell][i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void fx_viz_style_cb(lv_event_t* e) {
    LV_UNUSED(e);
    s_fx_viz_style = (s_fx_viz_style + 1) % FX_VIZ_STYLE_COUNT;
    fx_apply_viz_style();
}

// Callback: toggle FX mute on card click
static void fx_toggle_cb(lv_event_t* e) {
    int cell = (int)(intptr_t)lv_event_get_user_data(e);
    if (cell < 0 || cell >= FX_CARD_COUNT) return;
    uint32_t now = millis();
    // Per-button debounce (700ms) + global cross-button cooldown (200ms).
    // GT911 on P4 with LVGL can fire duplicate CLICKED events within <300ms;
    // the global guard prevents two adjacent buttons from both triggering on
    // a sloppy wide tap.
    if (now - s_fx_toggle_last_ms[cell] < 700) return;
    if (now - s_fx_any_toggle_last_ms  < 200) return;
    s_fx_toggle_last_ms[cell] = now;
    s_fx_any_toggle_last_ms   = now;
    if (cell == FX_CARD_FILTER) {
        fx_all_turn_off();
        return;
    }
    const uint8_t ownerFunction = fx_card_owner_function(cell);
    if (ownerFunction != POD_FUNC_NONE
        && pod_function_has_physical_owner(ownerFunction)) return;
    if (fx_card_has_onoff(cell)) {
        if (cell < 3) {
            bool unmuting = p4.enc_muted[cell];
            p4.enc_muted[cell] = !p4.enc_muted[cell];
            // Delay/Reverb/Flange need a non-zero value when enabling, otherwise
            // active=false in the command path and it looks ON but sounds OFF.
            if (unmuting && p4.enc_value[cell] == 0) {
                p4.enc_value[cell] = 48;
                s_fx_arc_anim[cell] = 48.0f;
            }
            if (control_available()) control_send_fx_enc(cell, p4.enc_value[cell], p4.enc_muted[cell]);
        } else if (cell == FX_CARD_FOLD || cell == FX_CARD_PHASER) {
            const int pot_idx = cell == FX_CARD_FOLD ? 0 : 2;
            const int value_idx = cell == FX_CARD_FOLD ? 3 : 2;
            const bool unmuting = p4.pot_muted[pot_idx];
            p4.pot_muted[pot_idx] = !p4.pot_muted[pot_idx];
            if (unmuting && p4.pot_value[value_idx] == 0) {
                p4.pot_value[value_idx] = 48;
                s_fx_arc_anim[cell] = 48.0f;
            }
            if (control_available()) {
                if (pot_idx == 0)
                    control_send_fx_pot(0, p4.pot_value[3], p4.pot_muted[0]);
                else
                    control_send_fx_pot(2, p4.pot_value[2], p4.pot_muted[2]);
            }
        } else {
            const bool muted = fx_card_is_muted(cell);
            const int neutral_u7 = fx_card_neutral_u7(cell);
            if (muted) {
                int value = (int)s_fx_last_active_u7[cell];
                if (value == neutral_u7)
                    value = (cell == FX_CARD_CUTOFF) ? 96 : 64;
                fx_card_send_value(cell, value);
                s_fx_arc_anim[cell] = (float)value;
            } else {
                fx_card_send_value(cell, neutral_u7);
                s_fx_arc_anim[cell] = (float)neutral_u7;
            }
        }
    } else {
        fx_card_reset(cell);
    }
    control_mark_fx_screen_dirty();
}

static void fx_arc_cb(lv_event_t* e) {
    if (s_fx_ui_syncing) return;
    int cell = (int)(intptr_t)lv_event_get_user_data(e);
    if (cell < 0 || cell >= FX_CARD_COUNT) return;
    lv_obj_t* arc = (lv_obj_t*)lv_event_get_target(e);
    int val = lv_arc_get_value(arc);
    // Snap the lerp animation immediately to avoid the animation overwriting the
    // value the user just set (e.g. set 50, sees 22 because lerp was still at 0).
    s_fx_arc_anim[cell] = (float)val;
    s_fx_arc_user_ms[cell] = millis();   // own this arc for 800ms
    static uint32_t last_tx_ms[FX_CARD_COUNT] = {};
    uint32_t now = millis();
    bool final_value = (lv_event_get_code(e) == LV_EVENT_RELEASED ||
                        lv_event_get_code(e) == LV_EVENT_PRESS_LOST);
    bool transmit = final_value || last_tx_ms[cell] == 0 ||
                    (uint32_t)(now - last_tx_ms[cell]) >= 25;
    fx_card_send_value(cell, val, transmit);
    if (transmit) last_tx_ms[cell] = now;
}

static void fx_page_cb(lv_event_t* e) {
    int dir = (int)(intptr_t)lv_event_get_user_data(e);
    int pages = fx_page_count();
    fx_page = (fx_page + dir + pages) % pages;
    fx_apply_layout();
}

static void fx_view_cb(lv_event_t* e) {
    LV_UNUSED(e);
    fx_view_mode = (fx_view_mode + 1) % FX_VIEW_MODE_COUNT;
    fx_page = 0;
    fx_apply_layout();
}

// =============================================================================
// FILTER PRESETS — 8 savable snapshots of the global filter's full state
// (model, cutoff, resonance, distortion, bitcrush, sample-rate reduction,
// SVF MORPH position). First building block for the SONG "MATRIX" the user
// wants: each column there will pick one of these 8 slots instead of the
// live FX LAB knobs. Recall just replays the same public setters a user
// touching each knob by hand would call — no new protocol needed, and it
// stays correct automatically if those setters' ranges ever change.
// =============================================================================
#define FILTER_PRESET_COUNT 8
struct FilterPresetSlot {
    bool used;
    char name[16];
    int filterType;
    int cutoffHz;
    int resonanceX10;
    int distortionPct;
    int bitcrushBits;
    int sampleRateHz;
    uint8_t morphU7;
};
static FilterPresetSlot s_filter_presets[FILTER_PRESET_COUNT] = {};
static const char* FILTER_PRESETS_FILE = "/filter_presets.txt";

static lv_obj_t* s_filter_preset_modal = NULL;
static lv_obj_t* s_filter_preset_slot_btns[FILTER_PRESET_COUNT] = {};
static lv_obj_t* s_filter_preset_slot_lbls[FILTER_PRESET_COUNT] = {};

static void filter_presets_save_to_disk(void) {
    File f = SPIFFS.open(FILTER_PRESETS_FILE, FILE_WRITE);
    if (!f) return;
    for (int i = 0; i < FILTER_PRESET_COUNT; i++) {
        const FilterPresetSlot& s = s_filter_presets[i];
        f.printf("%d,%s,%d,%d,%d,%d,%d,%d,%u\n",
                 s.used ? 1 : 0, s.name[0] ? s.name : "-",
                 s.filterType, s.cutoffHz, s.resonanceX10,
                 s.distortionPct, s.bitcrushBits, s.sampleRateHz,
                 (unsigned)s.morphU7);
    }
    f.close();
}

static void filter_presets_load_from_disk(void) {
    memset(s_filter_presets, 0, sizeof(s_filter_presets));
    for (int i = 0; i < FILTER_PRESET_COUNT; i++) {
        s_filter_presets[i].cutoffHz = 10000;
        s_filter_presets[i].bitcrushBits = 16;
    }
    File f = SPIFFS.open(FILTER_PRESETS_FILE, FILE_READ);
    if (!f) return;
    int idx = 0;
    char line[128];
    while (idx < FILTER_PRESET_COUNT && fs_read_line(f, line, sizeof(line))) {
        if (line[0] == '\0') { idx++; continue; }
        int used = 0, filterType = 0, cutoffHz = 10000, resonanceX10 = 7,
            distortionPct = 0, bitcrushBits = 16, sampleRateHz = 0;
        unsigned morphU7 = 0;
        char name[16] = {};
        int parsed = sscanf(line, "%d,%15[^,],%d,%d,%d,%d,%d,%d,%u",
            &used, name, &filterType, &cutoffHz, &resonanceX10,
            &distortionPct, &bitcrushBits, &sampleRateHz, &morphU7);
        if (parsed >= 2) {
            FilterPresetSlot& s = s_filter_presets[idx];
            s.used = (used != 0);
            strncpy(s.name, name, sizeof(s.name) - 1);
            if (parsed >= 3) s.filterType = constrain(filterType, 0, 15);
            if (parsed >= 4) s.cutoffHz = constrain(cutoffHz, 20, 20000);
            if (parsed >= 5) s.resonanceX10 = constrain(resonanceX10, 3, 300);
            if (parsed >= 6) s.distortionPct = constrain(distortionPct, 0, 100);
            if (parsed >= 7) s.bitcrushBits = constrain(bitcrushBits, 4, 16);
            if (parsed >= 8) s.sampleRateHz = sampleRateHz <= 0 ? 0 : constrain(sampleRateHz, 1000, 48000);
            if (parsed >= 9) s.morphU7 = (uint8_t)constrain((int)morphU7, 0, 127);
        }
        idx++;
    }
    f.close();
}

static void filter_preset_modal_refresh(void) {
    for (int i = 0; i < FILTER_PRESET_COUNT; i++) {
        if (!s_filter_preset_slot_lbls[i]) continue;
        const FilterPresetSlot& s = s_filter_presets[i];
        if (s.used) {
            lv_label_set_text_fmt(s_filter_preset_slot_lbls[i], "S%d\n%s", i + 1, s.name[0] ? s.name : "PRESET");
            lv_obj_set_style_text_color(s_filter_preset_slot_lbls[i], lv_color_white(), 0);
        } else {
            lv_label_set_text_fmt(s_filter_preset_slot_lbls[i], "S%d\nVACIO", i + 1);
            lv_obj_set_style_text_color(s_filter_preset_slot_lbls[i], theme_text_dim(), 0);
        }
    }
}

static void filter_preset_save_current(int slot) {
    if (slot < 0 || slot >= FILTER_PRESET_COUNT) return;
    FilterPresetSlot& s = s_filter_presets[slot];
    s.used = true;
    snprintf(s.name, sizeof(s.name), "%s", fx_filter_model_name(constrain(p4.filter_type, 0, 15)));
    s.filterType = p4.filter_type;
    s.cutoffHz = p4.cutoff_hz;
    s.resonanceX10 = p4.resonance_x10;
    s.distortionPct = p4.distortion_pct;
    s.bitcrushBits = p4.bitcrush_bits;
    s.sampleRateHz = p4.sample_rate_hz;
    s.morphU7 = s_fx_current_u7[FX_CARD_MORPH];
    filter_presets_save_to_disk();
    filter_preset_modal_refresh();
    ui_show_toast("Preset de filtro guardado", RED808_SUCCESS);
}

static void filter_preset_recall(int slot) {
    if (slot < 0 || slot >= FILTER_PRESET_COUNT) return;
    const FilterPresetSlot& s = s_filter_presets[slot];
    if (!s.used) {
        ui_show_toast("Slot vacio — manten pulsado para guardar", RED808_WARNING);
        return;
    }
    if (!control_available()) {
        ui_show_toast("Master no conectado", RED808_WARNING);
        return;
    }
    control_send_set_filter(s.filterType);
    control_send_set_filter_cutoff(s.cutoffHz);
    control_send_set_filter_resonance(s.resonanceX10 / 10.0f);
    control_send_set_distortion(s.distortionPct / 100.0f);
    control_send_set_bitcrush(s.bitcrushBits);
    control_send_set_sample_rate(s.sampleRateHz);
    control_send_set_filter_morph(s.morphU7 / 127.0f);
    s_fx_current_u7[FX_CARD_MORPH] = s.morphU7;
    control_mark_fx_screen_dirty();
    ui_show_toast("Preset de filtro cargado", RED808_SUCCESS);
}

static void filter_preset_slot_clicked_cb(lv_event_t* e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    if (lv_event_get_code(e) == LV_EVENT_LONG_PRESSED) filter_preset_save_current(slot);
    else filter_preset_recall(slot);
}

static void filter_preset_modal_close_cb(lv_event_t* e) {
    if (e && lv_event_get_target(e) != lv_event_get_current_target(e)) return;
    if (s_filter_preset_modal) {
        lv_obj_del(s_filter_preset_modal);
        s_filter_preset_modal = NULL;
        for (int i = 0; i < FILTER_PRESET_COUNT; i++) {
            s_filter_preset_slot_btns[i] = NULL;
            s_filter_preset_slot_lbls[i] = NULL;
        }
    }
}

static void filter_preset_modal_show(lv_event_t* e) {
    LV_UNUSED(e);
    if (s_filter_preset_modal) return;

    s_filter_preset_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_filter_preset_modal, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(s_filter_preset_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_filter_preset_modal, LV_OPA_80, 0);
    lv_obj_set_style_border_width(s_filter_preset_modal, 0, 0);
    lv_obj_set_style_pad_all(s_filter_preset_modal, 0, 0);
    lv_obj_clear_flag(s_filter_preset_modal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_filter_preset_modal, filter_preset_modal_close_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* card = lv_obj_create(s_filter_preset_modal);
    lv_obj_set_size(card, 720, 260);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_color(card, RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_dir(card, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_border_color(card, RED808_CYAN, 0);
    lv_obj_set_style_radius(card, 18, 0);
    lv_obj_set_style_pad_all(card, 14, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t* title = lv_label_create(card);
    lv_label_set_text(title, "FILTER PRESETS");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(title, RED808_CYAN, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 2);

    lv_obj_t* hint = lv_label_create(card);
    lv_label_set_text(hint, "TOCA = cargar   ·   MANTEN PULSADO = guardar el filtro actual aqui");
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(hint, RED808_TEXT_DIM, 0);
    lv_obj_align(hint, LV_ALIGN_TOP_MID, 0, 32);

    constexpr int btnW = 80, btnH = 84, gapX = 6, y0 = 68;
    for (int i = 0; i < FILTER_PRESET_COUNT; i++) {
        lv_obj_t* btn = lv_btn_create(card);
        s_filter_preset_slot_btns[i] = btn;
        lv_obj_set_size(btn, btnW, btnH);
        lv_obj_set_pos(btn, 4 + i * (btnW + gapX), y0);
        apply_control_button_style(btn, RED808_ACCENT2, false, 8);
        lv_obj_add_event_cb(btn, filter_preset_slot_clicked_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
        lv_obj_add_event_cb(btn, filter_preset_slot_clicked_cb, LV_EVENT_LONG_PRESSED, (void*)(intptr_t)i);

        lv_obj_t* lbl = lv_label_create(btn);
        s_filter_preset_slot_lbls[i] = lbl;
        lv_obj_set_width(lbl, btnW - 8);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_center(lbl);
    }

    lv_obj_t* close_btn = lv_btn_create(card);
    lv_obj_set_size(close_btn, 160, 40);
    lv_obj_set_pos(close_btn, 280, 172);
    apply_control_button_style(close_btn, RED808_ERROR, false, 10);
    lv_obj_add_event_cb(close_btn, filter_preset_modal_close_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* close_lbl = lv_label_create(close_btn);
    lv_label_set_text(close_lbl, "CERRAR");
    lv_obj_center(close_lbl);

    filter_preset_modal_refresh();
}

static void fx_xy_open_cb(lv_event_t* e) {
    LV_UNUSED(e);
    ui_navigate_to(13);   // FX XY performance pad
}

static void create_fx_screen(void) {
    scr_fx = lv_obj_create(NULL);
    apply_screen_theme_bg(scr_fx);
    lv_obj_clear_flag(scr_fx, LV_OBJ_FLAG_SCROLLABLE);
    ui_create_header(scr_fx);

    // ── Title row ──
    lv_obj_t* title = lv_label_create(scr_fx);
    lv_label_set_text(title, LV_SYMBOL_AUDIO "  FX LAB");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(title, RED808_ACCENT, 0);
    lv_obj_set_pos(title, 60, 10);

    fx_pattern_lbl = lv_label_create(scr_fx);
    lv_label_set_text_fmt(fx_pattern_lbl, "PAT P%03d", p4.current_pattern + 1);
    lv_obj_set_size(fx_pattern_lbl, 112, 30);
    lv_obj_set_pos(fx_pattern_lbl, 220, 8);
    lv_obj_set_style_radius(fx_pattern_lbl, 8, 0);
    lv_obj_set_style_bg_color(fx_pattern_lbl, RED808_SURFACE, 0);
    lv_obj_set_style_bg_opa(fx_pattern_lbl, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(fx_pattern_lbl, 1, 0);
    lv_obj_set_style_border_color(fx_pattern_lbl, RED808_WARNING, 0);
    lv_obj_set_style_text_color(fx_pattern_lbl, RED808_WARNING, 0);
    lv_obj_set_style_text_font(fx_pattern_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_align(fx_pattern_lbl, LV_TEXT_ALIGN_CENTER, 0);

    fx_active_lbl = lv_label_create(scr_fx);
    lv_label_set_text(fx_active_lbl, "ALL FX OFF");
    lv_obj_set_size(fx_active_lbl, 140, 28);
    lv_obj_set_pos(fx_active_lbl, 342, 15);
    lv_label_set_long_mode(fx_active_lbl, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_style_text_font(fx_active_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(fx_active_lbl, RED808_TEXT_DIM, 0);
    lv_obj_set_style_text_align(fx_active_lbl, LV_TEXT_ALIGN_LEFT, 0);

    // ── Action row (row 2) — kept off row 1 entirely, which was already
    // fully occupied by TITLE/PAT/ACTIVE FX. PRESETS used to share row 1's
    // x=708 with the page-nav cluster below (same anchor point, computed
    // independently) and sat invisibly underneath it; a full second row
    // both fixes that and leaves room for VIZ without another squeeze.
    const int actionY = 50;
    fx_all_off_btn = lv_btn_create(scr_fx);
    lv_obj_set_size(fx_all_off_btn, 110, 36);
    lv_obj_set_pos(fx_all_off_btn, 8, actionY);
    apply_control_button_style(fx_all_off_btn, RED808_ERROR, false, 8);
    lv_obj_add_event_cb(fx_all_off_btn, fx_all_off_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* allOffLabel = lv_label_create(fx_all_off_btn);
    lv_label_set_text(allOffLabel, "OFF ALL FX");
    lv_obj_set_style_text_font(allOffLabel, &lv_font_montserrat_12, 0);
    lv_obj_center(allOffLabel);

    // ── RANDOM: tasteful global filter randomizer ──
    s_fx_random_btn = lv_btn_create(scr_fx);
    lv_obj_set_size(s_fx_random_btn, 60, 36);
    lv_obj_set_pos(s_fx_random_btn, 126, actionY);
    lv_obj_add_event_cb(s_fx_random_btn, fx_random_modal_show, LV_EVENT_CLICKED, NULL);
    lv_obj_t* randomLabel = lv_label_create(s_fx_random_btn);
    lv_label_set_text(randomLabel, LV_SYMBOL_SHUFFLE);
    lv_obj_set_style_text_font(randomLabel, &lv_font_montserrat_16, 0);
    lv_obj_center(randomLabel);
    s_fx_random_stop_badge = ui_create_auto_stop_badge(s_fx_random_btn, fx_random_stop_badge_cb);
    fx_random_btn_refresh();

    // ── FILTER PRESETS: 8 savable snapshots of the whole filter section ──
    lv_obj_t* presets_btn = lv_btn_create(scr_fx);
    lv_obj_set_size(presets_btn, 90, 36);
    lv_obj_set_pos(presets_btn, 192, actionY);
    apply_control_button_style(presets_btn, RED808_CYAN, false, 8);
    lv_obj_add_event_cb(presets_btn, filter_preset_modal_show, LV_EVENT_CLICKED, NULL);
    lv_obj_t* presetsLabel = lv_label_create(presets_btn);
    lv_label_set_text(presetsLabel, "PRESETS");
    lv_obj_set_style_text_font(presetsLabel, &lv_font_montserrat_12, 0);
    lv_obj_center(presetsLabel);

    // ── VIZ: cycles each card's value indicator between ARC / LED / BAR ──
    s_fx_viz_btn = lv_btn_create(scr_fx);
    lv_obj_set_size(s_fx_viz_btn, 90, 36);
    lv_obj_set_pos(s_fx_viz_btn, 288, actionY);
    apply_control_button_style(s_fx_viz_btn, RED808_ACCENT2, false, 8);
    lv_obj_add_event_cb(s_fx_viz_btn, fx_viz_style_cb, LV_EVENT_CLICKED, NULL);
    s_fx_viz_lbl = lv_label_create(s_fx_viz_btn);
    lv_label_set_text(s_fx_viz_lbl, "VIZ: ARC");
    lv_obj_set_style_text_font(s_fx_viz_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(s_fx_viz_lbl);

    for (int cell = 0; cell < FX_CARD_COUNT; cell++) {
        // Card container
        lv_obj_t* card = lv_obj_create(scr_fx);
        fx_cards[cell] = card;
        lv_obj_set_size(card, 320, 260);
        lv_obj_set_pos(card, 0, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
        // Themed card background
        lv_obj_set_style_bg_color(card, RED808_SURFACE, 0);
        lv_obj_set_style_bg_grad_color(card, RED808_PANEL, 0);
        lv_obj_set_style_bg_grad_dir(card, LV_GRAD_DIR_VER, 0);
        lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(card, 8, 0);
        lv_obj_set_style_border_width(card, 2, 0);
        lv_obj_set_style_border_color(card, lv_color_hex(fx_colors[cell]), 0);
        lv_obj_set_style_border_opa(card, LV_OPA_40, 0);
        // Outer neon glow
        lv_obj_set_style_outline_width(card, 4, 0);
        lv_obj_set_style_outline_color(card, lv_color_hex(fx_colors[cell]), 0);
        lv_obj_set_style_outline_opa(card, LV_OPA_20, 0);
        lv_obj_set_style_outline_pad(card, 2, 0);
        lv_obj_set_style_shadow_width(card, 0, 0);
        lv_obj_set_style_pad_all(card, 0, 0);
        lv_obj_set_style_bg_color(card, lv_color_hex(fx_colors[cell]), LV_STATE_PRESSED);
        lv_obj_set_style_bg_opa(card, (lv_opa_t)216, LV_STATE_PRESSED);
        // Card is just a visual container; toggle is on its dedicated button
        // (see ON/OFF below). Otherwise the arc drag bubbled up into a card
        // click and silently muted the FX.
        lv_obj_clear_flag(card, LV_OBJ_FLAG_CLICKABLE);
        pod_register_owner_badge(card, fx_card_owner_function(cell));

        // "MIDI" badge — top-left corner, shown only while a learned CC maps
        // to this control. Visibility refreshed in fx_active_header_refresh().
        fx_midi_badges[cell] = lv_label_create(card);
        lv_label_set_text(fx_midi_badges[cell], "MIDI");
        lv_obj_set_style_text_font(fx_midi_badges[cell], &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(fx_midi_badges[cell], RED808_SUCCESS, 0);
        lv_obj_align(fx_midi_badges[cell], LV_ALIGN_TOP_LEFT, 8, 8);
        lv_obj_add_flag(fx_midi_badges[cell], LV_OBJ_FLAG_HIDDEN);

        // FX Name — top center, neon style
        fx_name_labels[cell] = lv_label_create(card);
        lv_label_set_text(fx_name_labels[cell], fx_names[cell]);
        lv_obj_set_style_text_font(fx_name_labels[cell], &lv_font_montserrat_22, 0);
        lv_obj_set_style_text_color(fx_name_labels[cell], lv_color_hex(fx_colors[cell]), 0);
        lv_obj_set_width(fx_name_labels[cell], 320);
        lv_obj_set_style_text_align(fx_name_labels[cell], LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(fx_name_labels[cell], LV_ALIGN_TOP_MID, 0, 14);

        // Source tag — subtle under name
        fx_src_labels[cell] = lv_label_create(card);
        lv_label_set_text(fx_src_labels[cell], fx_src[cell]);
        lv_obj_set_style_text_font(fx_src_labels[cell], &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(fx_src_labels[cell], RED808_TEXT_DIM, 0);
        lv_obj_set_width(fx_src_labels[cell], 320);
        lv_obj_set_style_text_align(fx_src_labels[cell], LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(fx_src_labels[cell], LV_ALIGN_TOP_MID, 0, 42);

        // ── BIG ARC (neon circle indicator) ──
        fx_arcs[cell] = lv_arc_create(card);
        lv_obj_set_size(fx_arcs[cell], 220, 220);
        lv_obj_align(fx_arcs[cell], LV_ALIGN_CENTER, 0, -18);
        lv_arc_set_rotation(fx_arcs[cell], 135);
        lv_arc_set_bg_angles(fx_arcs[cell], 0, 270);
        lv_arc_set_range(fx_arcs[cell], 0, 127);
        lv_arc_set_value(fx_arcs[cell], 0);
        lv_obj_add_event_cb(fx_arcs[cell], fx_arc_cb, LV_EVENT_VALUE_CHANGED, (void*)(intptr_t)cell);
        lv_obj_add_event_cb(fx_arcs[cell], fx_arc_cb, LV_EVENT_RELEASED, (void*)(intptr_t)cell);
        lv_obj_add_event_cb(fx_arcs[cell], fx_arc_cb, LV_EVENT_PRESS_LOST, (void*)(intptr_t)cell);
        // Track (background ring) — dim, theme-aware
        lv_obj_set_style_arc_width(fx_arcs[cell], 14, LV_PART_MAIN);
        lv_obj_set_style_arc_color(fx_arcs[cell], RED808_BORDER, LV_PART_MAIN);
        lv_obj_set_style_arc_opa(fx_arcs[cell], LV_OPA_COVER, LV_PART_MAIN);
        // Indicator (filled arc) — neon glow
        lv_obj_set_style_arc_width(fx_arcs[cell], 20, LV_PART_INDICATOR);
        lv_obj_set_style_arc_color(fx_arcs[cell], lv_color_hex(fx_colors[cell]), LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(fx_arcs[cell], lv_color_white(), LV_PART_KNOB);
        lv_obj_set_style_bg_opa(fx_arcs[cell], LV_OPA_COVER, LV_PART_KNOB);
        lv_obj_set_style_pad_all(fx_arcs[cell], 5, LV_PART_KNOB);
        lv_obj_set_style_radius(fx_arcs[cell], LV_RADIUS_CIRCLE, LV_PART_KNOB);
        lv_obj_set_style_shadow_color(fx_arcs[cell], lv_color_hex(fx_colors[cell]), LV_PART_KNOB);
        lv_obj_set_style_shadow_width(fx_arcs[cell], 10, LV_PART_KNOB);
        lv_obj_set_style_shadow_opa(fx_arcs[cell], LV_OPA_50, LV_PART_KNOB);

        // ── Alt visualization: VU bar (hidden unless VIZ: BAR is selected) ──
        fx_bars[cell] = lv_bar_create(card);
        lv_obj_set_size(fx_bars[cell], 240, 18);
        lv_obj_set_pos(fx_bars[cell], 40, 58);
        lv_bar_set_range(fx_bars[cell], 0, 127);
        lv_bar_set_value(fx_bars[cell], 0, LV_ANIM_OFF);
        lv_obj_set_style_radius(fx_bars[cell], 4, LV_PART_MAIN);
        lv_obj_set_style_radius(fx_bars[cell], 4, LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(fx_bars[cell], RED808_BORDER, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(fx_bars[cell], LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_width(fx_bars[cell], 0, LV_PART_MAIN);
        lv_obj_set_style_bg_color(fx_bars[cell], lv_color_hex(fx_colors[cell]), LV_PART_INDICATOR);
        lv_obj_set_style_bg_opa(fx_bars[cell], LV_OPA_COVER, LV_PART_INDICATOR);
        lv_obj_clear_flag(fx_bars[cell], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(fx_bars[cell], LV_OBJ_FLAG_HIDDEN);

        // ── Alt visualization: LED ladder (hidden unless VIZ: LED) — plain
        // lv_obj rectangles with bg_color/bg_opa toggling, the same proven
        // technique as the fx_page_dot page indicators, rather than the
        // unverified lv_led widget API. ──
        for (int i = 0; i < FX_LED_COUNT; i++) {
            lv_obj_t* led = lv_obj_create(card);
            fx_leds[cell][i] = led;
            lv_obj_set_size(led, 20, 18);
            lv_obj_set_pos(led, 40 + i * 24, 58);
            lv_obj_set_style_radius(led, 3, 0);
            lv_obj_set_style_bg_color(led, lv_color_hex(fx_colors[cell]), 0);
            lv_obj_set_style_bg_opa(led, LV_OPA_20, 0);
            lv_obj_set_style_border_width(led, 0, 0);
            lv_obj_clear_flag(led, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_clear_flag(led, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_flag(led, LV_OBJ_FLAG_HIDDEN);
        }

        // Value label — center of arc (big neon number)
        fx_value_labels[cell] = lv_label_create(card);
        lv_label_set_text(fx_value_labels[cell], "000");
        lv_obj_set_style_text_font(fx_value_labels[cell], &lv_font_montserrat_40, 0);
        lv_obj_set_style_text_color(fx_value_labels[cell], RED808_TEXT, 0);
        lv_obj_set_width(fx_value_labels[cell], 320);
        lv_obj_set_style_text_align(fx_value_labels[cell], LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(fx_value_labels[cell], LV_ALIGN_CENTER, 0, -18);

        // Percentage sub-label
        fx_pct_labels[cell] = lv_label_create(card);
        lv_label_set_text(fx_pct_labels[cell], "%");
        lv_obj_set_style_text_font(fx_pct_labels[cell], &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(fx_pct_labels[cell], RED808_TEXT_DIM, 0);
        lv_obj_align(fx_pct_labels[cell], LV_ALIGN_CENTER, 55, -2);

        // ── ON/OFF Toggle Button ──
        fx_toggle_btns[cell] = lv_btn_create(card);
        lv_obj_set_size(fx_toggle_btns[cell], 100, 38);
        lv_obj_align(fx_toggle_btns[cell], LV_ALIGN_BOTTOM_MID, 0, -14);
        apply_control_button_style(fx_toggle_btns[cell], lv_color_hex(fx_colors[cell]), false, 8);
        lv_obj_set_style_bg_color(fx_toggle_btns[cell], lv_color_hex(fx_colors[cell]), 0);
        lv_obj_set_style_bg_opa(fx_toggle_btns[cell], LV_OPA_20, 0);
        lv_obj_set_style_border_opa(fx_toggle_btns[cell], LV_OPA_80, 0);
        lv_obj_set_style_shadow_width(fx_toggle_btns[cell], 12, 0);
        lv_obj_set_style_shadow_color(fx_toggle_btns[cell], lv_color_hex(fx_colors[cell]), 0);
        lv_obj_set_style_shadow_opa(fx_toggle_btns[cell], LV_OPA_40, 0);
        lv_obj_add_flag(fx_toggle_btns[cell], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(fx_toggle_btns[cell], fx_toggle_cb, LV_EVENT_CLICKED, (void*)(intptr_t)cell);
        lv_obj_t* tog_lbl = lv_label_create(fx_toggle_btns[cell]);
        lv_label_set_text(tog_lbl, fx_card_button_text(cell, fx_card_is_muted(cell)));
        lv_obj_set_style_text_font(tog_lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(tog_lbl, fx_safe_text_color(fx_colors[cell]), 0);
        lv_obj_center(tog_lbl);
    }

    // Page controls — same row 2 as the action buttons above (actionY), far
    // enough to the right (anchored from LCD_H_RES) that the two clusters
    // never meet even at VIEW 18's widest page label.
    const int page_ctrl_y = actionY;
    const int page_ctrl_w = 46;
    const int page_ctrl_gap = 6;
    const int page_lbl_w = 46;
    const int page_dots_w = 48;
    const int page_group_w = page_ctrl_w * 2 + page_ctrl_gap + page_lbl_w + 12 + page_dots_w + 88;
    const int page_group_x = LCD_H_RES - 24 - page_group_w;

    lv_obj_t* prev_btn = lv_btn_create(scr_fx);
    lv_obj_set_size(prev_btn, 46, 42);
    lv_obj_set_pos(prev_btn, page_group_x, page_ctrl_y);
    apply_control_button_style(prev_btn, RED808_CYAN, false, 8);
    lv_obj_t* prev_lbl = lv_label_create(prev_btn);
    lv_label_set_text(prev_lbl, LV_SYMBOL_LEFT);
    lv_obj_center(prev_lbl);
    lv_obj_add_event_cb(prev_btn, fx_page_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-1);

    lv_obj_t* next_btn = lv_btn_create(scr_fx);
    lv_obj_set_size(next_btn, 46, 42);
    lv_obj_set_pos(next_btn, page_group_x + page_ctrl_w + page_ctrl_gap, page_ctrl_y);
    apply_control_button_style(next_btn, RED808_CYAN, false, 8);
    lv_obj_t* next_lbl = lv_label_create(next_btn);
    lv_label_set_text(next_lbl, LV_SYMBOL_RIGHT);
    lv_obj_center(next_lbl);
    lv_obj_add_event_cb(next_btn, fx_page_cb, LV_EVENT_CLICKED, (void*)(intptr_t)1);

    fx_page_lbl = lv_label_create(scr_fx);
    lv_label_set_text(fx_page_lbl, "1 / 4");
    lv_obj_set_style_text_font(fx_page_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(fx_page_lbl, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(fx_page_lbl, page_group_x + page_ctrl_w * 2 + page_ctrl_gap * 2, actionY + 8);

    for (int p = 0; p < FX_PAGE_DOT_COUNT; p++) {
        fx_page_dot[p] = lv_obj_create(scr_fx);
        lv_obj_set_size(fx_page_dot[p], 8, 8);
        lv_obj_set_pos(fx_page_dot[p], page_group_x + page_ctrl_w * 2 + page_ctrl_gap * 2 + page_lbl_w + 6 + p * 14, actionY + 14);
        lv_obj_set_style_radius(fx_page_dot[p], LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(fx_page_dot[p], RED808_CYAN, 0);
        lv_obj_set_style_bg_opa(fx_page_dot[p], p == 0 ? LV_OPA_COVER : LV_OPA_30, 0);
        lv_obj_set_style_border_width(fx_page_dot[p], 0, 0);
        lv_obj_clear_flag(fx_page_dot[p], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(fx_page_dot[p], LV_OBJ_FLAG_CLICKABLE);
    }

    fx_view_btn = lv_btn_create(scr_fx);
    lv_obj_set_size(fx_view_btn, 80, 42);
    lv_obj_set_pos(fx_view_btn, page_group_x + page_group_w - 80, page_ctrl_y);
    apply_control_button_style(fx_view_btn, RED808_WARNING, false, 8);
    lv_obj_add_event_cb(fx_view_btn, fx_view_cb, LV_EVENT_CLICKED, NULL);
    fx_view_lbl = lv_label_create(fx_view_btn);
    lv_label_set_text(fx_view_lbl, "VIEW 3");
    lv_obj_set_style_text_font(fx_view_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(fx_view_lbl);

    // XY performance pad entry (screen 13)
    lv_obj_t* xy_btn = lv_btn_create(scr_fx);
    lv_obj_set_size(xy_btn, 52, 42);
    lv_obj_set_pos(xy_btn, page_group_x + page_group_w - 80 - 52 - 6, page_ctrl_y);
    apply_control_button_style(xy_btn, RED808_ACCENT, false, 8);
    lv_obj_add_event_cb(xy_btn, fx_xy_open_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* xy_lbl = lv_label_create(xy_btn);
    lv_label_set_text(xy_lbl, "XY");
    lv_obj_set_style_text_font(xy_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(xy_lbl);

    filter_presets_load_from_disk();
    fx_apply_layout();
    // Re-applies s_fx_viz_style's visibility to the freshly (re)created
    // arc/bar/led widgets above — needed because that style choice
    // persists across a theme-reload screen rebuild, but every rebuilt
    // widget starts back at its creation-time default (arc visible).
    fx_apply_viz_style();
}

static void fx_format_display_value(int cell, int u7, bool muted,
                                    char* value, size_t valueSize,
                                    char* unit, size_t unitSize) {
    if (!value || valueSize == 0 || !unit || unitSize == 0) return;
    value[0] = '\0';
    unit[0] = '\0';
    if (muted) {
        snprintf(value, valueSize, "OFF");
        return;
    }

    const float normalized = constrain(u7, 0, 127) / 127.0f;
    switch (cell) {
        case FX_CARD_FLANGE:
        case FX_CARD_DELAY:
        case FX_CARD_REVERB:
        case FX_CARD_CRUSH:
        case FX_CARD_PHASER:
        case FX_CARD_DRIVE:
            snprintf(value, valueSize, "%d", (int)(normalized * 100.0f + 0.5f));
            snprintf(unit, unitSize, "%%");
            break;
        case FX_CARD_FOLD:
            snprintf(value, valueSize, "%.1f", 1.0f + normalized * 9.0f);
            snprintf(unit, unitSize, "GAIN");
            break;
        case FX_CARD_CUTOFF: {
            const float hz = 20.0f * powf(1000.0f, normalized);
            if (hz >= 1000.0f) {
                snprintf(value, valueSize, "%.1f", hz / 1000.0f);
                snprintf(unit, unitSize, "kHz");
            } else {
                snprintf(value, valueSize, "%d", (int)(hz + 0.5f));
                snprintf(unit, unitSize, "Hz");
            }
            break;
        }
        case FX_CARD_RESO:
            snprintf(value, valueSize, "%.1f", 0.7f + normalized * 19.3f);
            snprintf(unit, unitSize, "Q");
            break;
        case FX_CARD_BITS:
            snprintf(value, valueSize, "%d",
                     constrain((int)(16.0f - normalized * 12.0f + 0.5f), 4, 16));
            snprintf(unit, unitSize, "BIT");
            break;
        case FX_CARD_SRATE: {
            const float hz = 42000.0f * powf(4000.0f / 42000.0f, normalized);
            snprintf(value, valueSize, "%.1f", hz / 1000.0f);
            snprintf(unit, unitSize, "kHz");
            break;
        }
        case FX_CARD_FILTER: {
            const int type = constrain((int)(normalized * 15.0f + 0.5f), 0, 15);
            snprintf(value, valueSize, "%s", fx_filter_model_name(type));
            break;
        }
        case FX_CARD_STUTTER: {
            // Mirrors control_send_set_beatrepeat_macro's binning exactly so
            // the displayed division always matches what was actually sent.
            static const uint8_t kDivs[5] = {2, 4, 8, 16, 32};
            int idx = u7 > 0 ? constrain((int)((u7 - 1) * 5 / 127), 0, 4) : -1;
            if (idx < 0) snprintf(value, valueSize, "OFF");
            else snprintf(value, valueSize, "1/%d", kDivs[idx]);
            break;
        }
        default:
            snprintf(value, valueSize, "%d", (int)(normalized * 100.0f + 0.5f));
            snprintf(unit, unitSize, "%%");
            break;
    }
}

static void update_fx_screen(void) {
    static uint16_t prev_key[FX_CARD_COUNT];
    static bool prev_init = false;
    static uint32_t fx_gen = 0;
    if (!prev_init || fx_gen != s_ui_refresh_gen) {
        // First run OR theme reload (arcs recreated at 0 while prev_key and
        // s_fx_arc_anim still hold the live values — without this reset the
        // recreated arcs stay blank until the FX value next changes).
        prev_init = true;
        fx_gen = s_ui_refresh_gen;
        for (int i = 0; i < FX_CARD_COUNT; i++) {
            prev_key[i] = 0xFFFF;
            s_fx_arc_anim[i] = 0.0f;
        }
    }

    uint32_t now = millis();

    static int prev_fx_pattern = -1;
    static lv_obj_t* prev_fx_pattern_lbl = NULL;
    if(fx_pattern_lbl && (prev_fx_pattern != p4.current_pattern
       || prev_fx_pattern_lbl != fx_pattern_lbl))
    {
        prev_fx_pattern = p4.current_pattern;
        prev_fx_pattern_lbl = fx_pattern_lbl;
        lv_label_set_text_fmt(fx_pattern_lbl, "PAT P%03d", p4.current_pattern + 1);
    }

    fx_active_header_refresh();

    for (int cell = 0; cell < FX_CARD_COUNT; cell++) {
        int val = fx_card_current_value_u7(cell);
        bool muted = fx_card_is_muted(cell);
        int display_val = muted ? 0 : val;

        // If the user just touched this arc, hold the lerp for 800ms so the
        // animation does NOT overwrite what they set (root cause of "set 50 → shows 22").
        bool user_owns = (now - s_fx_arc_user_ms[cell]) < 800;
        if (!user_owns) {
            s_fx_arc_anim[cell] += ((float)display_val - s_fx_arc_anim[cell]) * 0.40f;
        }
        int anim_val = (int)(s_fx_arc_anim[cell] + 0.5f);

        // Key: tracks mute + target (for expensive style ops)
        uint16_t key = (uint16_t)((muted ? 0x100 : 0) | (display_val & 0xFF));
        bool still_animating = (fabsf(s_fx_arc_anim[cell] - (float)display_val) > 0.4f);
        bool key_changed = (key != prev_key[cell]);

        if (!still_animating && !key_changed) continue;

        s_fx_ui_syncing = true;
        if (fx_arcs[cell])
            lv_arc_set_value(fx_arcs[cell], anim_val);
        s_fx_ui_syncing = false;

        // Keep BAR/LED in sync too, even while hidden — switching VIZ style
        // then shows the right value immediately instead of a stale one.
        if (fx_bars[cell]) lv_bar_set_value(fx_bars[cell], anim_val, LV_ANIM_OFF);
        {
            int lit = (anim_val * FX_LED_COUNT + 63) / 127;   // round to nearest
            if (lit > FX_LED_COUNT) lit = FX_LED_COUNT;
            for (int i = 0; i < FX_LED_COUNT; i++) {
                if (!fx_leds[cell][i]) continue;
                bool on = i < lit;
                lv_obj_set_style_bg_opa(fx_leds[cell][i], on ? LV_OPA_COVER : LV_OPA_20, 0);
            }
        }

        char valueText[16] = {};
        char unitText[8] = {};
        fx_format_display_value(cell, anim_val, muted, valueText, sizeof(valueText),
                                unitText, sizeof(unitText));
        if (fx_value_labels[cell]) lv_label_set_text(fx_value_labels[cell], valueText);
        if (fx_pct_labels[cell]) lv_label_set_text(fx_pct_labels[cell], unitText);

        // Expensive style ops only when the logical key changes (not every lerp tick)
        if (key_changed) {
            prev_key[cell] = key;

            // Update card border glow intensity based on value
            lv_obj_t* card = fx_arcs[cell] ? lv_obj_get_parent(fx_arcs[cell]) : NULL;
            if (card && !muted && val > 0) {
                lv_obj_set_style_border_opa(card, LV_OPA_90, 0);
                lv_obj_set_style_outline_opa(card, LV_OPA_50, 0);
            } else if (card) {
                lv_obj_set_style_border_opa(card, muted ? LV_OPA_20 : LV_OPA_40, 0);
                lv_obj_set_style_outline_opa(card, LV_OPA_10, 0);
            }

            // Update toggle button
            if (fx_toggle_btns[cell] && cell != FX_CARD_FILTER) {
                lv_obj_t* lbl = lv_obj_get_child(fx_toggle_btns[cell], 0);
                if (lbl) lv_label_set_text(lbl, fx_card_button_text(cell, muted));
                lv_color_t tc = fx_safe_text_color(fx_colors[cell]);
                lv_obj_set_style_bg_opa(fx_toggle_btns[cell], muted ? LV_OPA_10 : LV_OPA_20, 0);
                lv_obj_set_style_shadow_opa(fx_toggle_btns[cell], muted ? LV_OPA_0 : LV_OPA_40, 0);
                if (lbl) lv_obj_set_style_text_color(lbl, muted ? RED808_TEXT_DIM : tc, 0);
            }

            // Update arc indicator color (dim if muted)
            if (fx_arcs[cell]) {
                lv_obj_set_style_arc_opa(fx_arcs[cell], muted ? LV_OPA_20 : LV_OPA_COVER, LV_PART_INDICATOR);
            }
        }
    }
}

// =============================================================================
// FX XY PAD — full-screen Kaoss-style performance surface (screen id 13)
// One finger drives two FX parameters at once through fx_card_send_value()
// (same path as the FX cards: P4 state + USB command ownership), so the
// master echo can't fight the gesture and the FX screen arcs stay in sync.
// =============================================================================
static lv_obj_t* s_fxxy_pad      = NULL;
static lv_obj_t* s_fxxy_dot      = NULL;
static lv_obj_t* s_fxxy_x_lbl    = NULL;
static lv_obj_t* s_fxxy_y_lbl    = NULL;
static lv_obj_t* s_fxxy_mode_lbl = NULL;
static int       s_fxxy_mode     = 0;   // index into FXXY_MODES (survives theme reload)

// Fixed layout (1024×600) — coords are constants so the dot can be placed
// from p4 state before the first touch.
static constexpr int FXXY_PAD_X = 8;
static constexpr int FXXY_PAD_Y = 56;
static constexpr int FXXY_PAD_W = 1008;
static constexpr int FXXY_PAD_H = 532;
static constexpr int FXXY_DOT   = 30;

struct FxXyMode {
    const char* name;
    int         cell_x;     // FX_CARD_* driven by the X axis
    const char* x_name;
    int         cell_y;     // FX_CARD_* driven by the Y axis
    const char* y_name;
};
static const FxXyMode FXXY_MODES[] = {
    {"FILTER", FX_CARD_CUTOFF, "CUTOFF", FX_CARD_RESO, "RESO"},
    {"CRUSH",  FX_CARD_SRATE,  "SRATE",  FX_CARD_BITS, "BITS"},
    {"DRIVE",  FX_CARD_DRIVE,  "DRIVE",  FX_CARD_FOLD, "FOLD"},
};
static constexpr int FXXY_MODE_COUNT = sizeof(FXXY_MODES) / sizeof(FXXY_MODES[0]);

static void fxxy_update_value_labels(int ux, int uy) {
    const FxXyMode& m = FXXY_MODES[s_fxxy_mode];
    if (s_fxxy_x_lbl)
        lv_label_set_text_fmt(s_fxxy_x_lbl, "%s %d%%", m.x_name, (ux * 100) / 127);
    if (s_fxxy_y_lbl)
        lv_label_set_text_fmt(s_fxxy_y_lbl, "%s %d%%", m.y_name, (uy * 100) / 127);
}

// Place the dot + labels from the CURRENT p4 FX state (screen entry / mode change).
static void fxxy_sync_from_state(void) {
    const FxXyMode& m = FXXY_MODES[s_fxxy_mode];
    int ux = fx_card_current_value_u7(m.cell_x);
    int uy = fx_card_current_value_u7(m.cell_y);
    if (s_fxxy_dot) {
        int lx = (ux * FXXY_PAD_W) / 127;
        int ly = FXXY_PAD_H - (uy * FXXY_PAD_H) / 127;
        lv_obj_set_pos(s_fxxy_dot, lx - FXXY_DOT / 2, ly - FXXY_DOT / 2);
    }
    fxxy_update_value_labels(ux, uy);
    if (s_fxxy_mode_lbl) lv_label_set_text_fmt(s_fxxy_mode_lbl, "%s " LV_SYMBOL_LOOP, m.name);
}

static void fxxy_pad_event_cb(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_PRESSED && code != LV_EVENT_PRESSING) return;
    lv_indev_t* indev = lv_indev_get_act();
    if (!indev || !s_fxxy_pad) return;
    lv_point_t p;
    lv_indev_get_point(indev, &p);

    lv_area_t a;
    lv_obj_get_coords(s_fxxy_pad, &a);
    int w = a.x2 - a.x1;
    int h = a.y2 - a.y1;
    if (w <= 0 || h <= 0) return;
    int lx = constrain((int)p.x - a.x1, 0, w);
    int ly = constrain((int)p.y - a.y1, 0, h);
    int ux = (lx * 127) / w;
    int uy = ((h - ly) * 127) / h;   // up = more

    if (s_fxxy_dot) lv_obj_set_pos(s_fxxy_dot, lx - FXXY_DOT / 2, ly - FXXY_DOT / 2);

    // Throttle to ~60 Hz and only send axes whose value actually moved —
    // each send is one direct binary command to DaisyPod3.
    static uint32_t last_tx_ms = 0;
    static int last_ux = -1, last_uy = -1;
    uint32_t now = millis();
    if (code == LV_EVENT_PRESSED) { last_ux = -1; last_uy = -1; }
    if (now - last_tx_ms < 33) return;
    const FxXyMode& m = FXXY_MODES[s_fxxy_mode];
    bool sent = false;
    if (ux != last_ux) {
        last_ux = ux;
        s_fx_arc_user_ms[m.cell_x] = now;
        fx_card_send_value(m.cell_x, ux);
        sent = true;
    }
    if (uy != last_uy) {
        last_uy = uy;
        s_fx_arc_user_ms[m.cell_y] = now;
        fx_card_send_value(m.cell_y, uy);
        sent = true;
    }
    if (sent) {
        last_tx_ms = now;
        fxxy_update_value_labels(ux, uy);
    }
}

static void fxxy_mode_cb(lv_event_t* e) {
    LV_UNUSED(e);
    s_fxxy_mode = (s_fxxy_mode + 1) % FXXY_MODE_COUNT;
    fxxy_sync_from_state();
}

static void fxxy_back_cb(lv_event_t* e) {
    LV_UNUSED(e);
    ui_navigate_to(8);   // back to FX cards
}

static void create_fx_xy_screen(void) {
    scr_fx_xy = lv_obj_create(NULL);
    apply_screen_theme_bg(scr_fx_xy);
    lv_obj_clear_flag(scr_fx_xy, LV_OBJ_FLAG_SCROLLABLE);

    // Header row: BACK · title · mode toggle
    lv_obj_t* back_btn = lv_btn_create(scr_fx_xy);
    lv_obj_set_size(back_btn, 96, 40);
    lv_obj_set_pos(back_btn, FXXY_PAD_X, 8);
    apply_control_button_style(back_btn, RED808_CYAN, false, 8);
    lv_obj_add_event_cb(back_btn, fxxy_back_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* back_lbl = lv_label_create(back_btn);
    lv_label_set_text(back_lbl, LV_SYMBOL_LEFT "  FX");
    lv_obj_center(back_lbl);

    lv_obj_t* title = lv_label_create(scr_fx_xy);
    lv_label_set_text(title, "XY PAD");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title, RED808_ACCENT, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 16);

    lv_obj_t* mode_btn = lv_btn_create(scr_fx_xy);
    lv_obj_set_size(mode_btn, 140, 40);
    lv_obj_set_pos(mode_btn, FXXY_PAD_X + FXXY_PAD_W - 140, 8);
    apply_control_button_style(mode_btn, RED808_WARNING, false, 8);
    lv_obj_add_event_cb(mode_btn, fxxy_mode_cb, LV_EVENT_CLICKED, NULL);
    s_fxxy_mode_lbl = lv_label_create(mode_btn);
    lv_label_set_text(s_fxxy_mode_lbl, "FILTER " LV_SYMBOL_LOOP);
    lv_obj_set_style_text_font(s_fxxy_mode_lbl, &lv_font_montserrat_14, 0);
    lv_obj_center(s_fxxy_mode_lbl);

    // Touch surface
    s_fxxy_pad = lv_obj_create(scr_fx_xy);
    lv_obj_set_size(s_fxxy_pad, FXXY_PAD_W, FXXY_PAD_H);
    lv_obj_set_pos(s_fxxy_pad, FXXY_PAD_X, FXXY_PAD_Y);
    lv_obj_set_style_bg_color(s_fxxy_pad, RED808_SURFACE, 0);
    lv_obj_set_style_bg_opa(s_fxxy_pad, LV_OPA_40, 0);
    lv_obj_set_style_border_width(s_fxxy_pad, 2, 0);
    lv_obj_set_style_border_color(s_fxxy_pad, RED808_ACCENT, 0);
    lv_obj_set_style_radius(s_fxxy_pad, 12, 0);
    lv_obj_set_style_pad_all(s_fxxy_pad, 0, 0);
    lv_obj_clear_flag(s_fxxy_pad, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_fxxy_pad, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_fxxy_pad, fxxy_pad_event_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(s_fxxy_pad, fxxy_pad_event_cb, LV_EVENT_PRESSING, NULL);

    // Faint quadrant guides
    for (int i = 1; i < 4; i++) {
        lv_obj_t* vline = lv_obj_create(s_fxxy_pad);
        lv_obj_set_size(vline, 1, FXXY_PAD_H);
        lv_obj_set_pos(vline, (FXXY_PAD_W * i) / 4, 0);
        lv_obj_set_style_bg_color(vline, RED808_BORDER, 0);
        lv_obj_set_style_bg_opa(vline, LV_OPA_30, 0);
        lv_obj_set_style_border_width(vline, 0, 0);
        lv_obj_clear_flag(vline, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_t* hline = lv_obj_create(s_fxxy_pad);
        lv_obj_set_size(hline, FXXY_PAD_W, 1);
        lv_obj_set_pos(hline, 0, (FXXY_PAD_H * i) / 4);
        lv_obj_set_style_bg_color(hline, RED808_BORDER, 0);
        lv_obj_set_style_bg_opa(hline, LV_OPA_30, 0);
        lv_obj_set_style_border_width(hline, 0, 0);
        lv_obj_clear_flag(hline, LV_OBJ_FLAG_CLICKABLE);
    }

    // Axis value readouts (inside the pad corners)
    s_fxxy_x_lbl = lv_label_create(s_fxxy_pad);
    lv_obj_set_style_text_font(s_fxxy_x_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_fxxy_x_lbl, RED808_CYAN, 0);
    lv_obj_align(s_fxxy_x_lbl, LV_ALIGN_BOTTOM_RIGHT, -14, -10);
    lv_obj_clear_flag(s_fxxy_x_lbl, LV_OBJ_FLAG_CLICKABLE);

    s_fxxy_y_lbl = lv_label_create(s_fxxy_pad);
    lv_obj_set_style_text_font(s_fxxy_y_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_fxxy_y_lbl, RED808_WARNING, 0);
    lv_obj_align(s_fxxy_y_lbl, LV_ALIGN_TOP_LEFT, 14, 10);
    lv_obj_clear_flag(s_fxxy_y_lbl, LV_OBJ_FLAG_CLICKABLE);

    // Crosshair dot
    s_fxxy_dot = lv_obj_create(s_fxxy_pad);
    lv_obj_set_size(s_fxxy_dot, FXXY_DOT, FXXY_DOT);
    lv_obj_set_style_radius(s_fxxy_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(s_fxxy_dot, RED808_ACCENT, 0);
    lv_obj_set_style_bg_opa(s_fxxy_dot, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_fxxy_dot, 2, 0);
    lv_obj_set_style_border_color(s_fxxy_dot, RED808_TEXT, 0);
    lv_obj_set_style_shadow_width(s_fxxy_dot, 18, 0);
    lv_obj_set_style_shadow_color(s_fxxy_dot, RED808_ACCENT, 0);
    lv_obj_set_style_shadow_opa(s_fxxy_dot, LV_OPA_70, 0);
    lv_obj_clear_flag(s_fxxy_dot, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_fxxy_dot, LV_OBJ_FLAG_SCROLLABLE);

    fxxy_sync_from_state();
}

// =============================================================================
// =============================================================================
// SEQUENCER SCREEN — Studio-grade 16-track × 16-step grid (1024×600)
// =============================================================================
static lv_obj_t* seq_step_btns[16][16]  = {};
static lv_obj_t* seq_step_accents[16][16] = {};  // bottom velocity strip per cell
static lv_obj_t* seq_step_prob_dot[16][16] = {}; // top-right dot: step probability < 100%
static lv_obj_t* seq_track_labels[16]   = {};
static lv_obj_t* seq_mute_btns[16]      = {};
static lv_obj_t* seq_solo_btns[16]      = {};
static lv_obj_t* seq_solo_labels[16]    = {};
static lv_obj_t* seq_fx_btns[16]        = {};
static lv_obj_t* seq_fx_clear_btns[16]  = {};
static lv_obj_t* seq_ruler_labels[16]   = {};  // beat/step number ruler
static lv_obj_t* seq_beat_bg[4]         = {};  // beat group shading panels
static lv_obj_t* seq_playhead_line      = NULL; // glowing vertical playhead
static lv_obj_t* seq_status_step_lbl    = NULL; // bottom "STEP 05 / 16"
static lv_obj_t* seq_status_pat_lbl     = NULL; // bottom "PATTERN 01"
static lv_obj_t* seq_status_name_lbl    = NULL; // factory-bank name for active slot
static lv_obj_t* seq_status_bpm_lbl     = NULL; // bottom "BPM 120.0"
static lv_obj_t* seq_status_mix_lbl     = NULL; // bottom "M 02 · S 01"
static int        seq_step_x[16]        = {};  // precomputed step column X

static void seq_refresh_track_label(uint8_t track) {
    if (track >= 16 || !seq_track_labels[track]) return;
    uint8_t inst = s_pad_inst_sel[track];
    if (inst > 7) inst = 0;
    const bool showEngine = (inst >= 4) || (inst == 0 && track >= 11);
    lv_label_set_text_fmt(seq_track_labels[track], "%02d\n%s", track + 1,
                          showEngine ? PAD_INST_SHORT[inst] : trackNames[track]);
}

// ── Multi-bar pagination (populated by MEM-MIDI load with raw grid) ────────
static bool       seq_raw_grid[16][64]  = {};   // up to 4 bars of raw steps
static int        seq_raw_len           = 16;   // 16/32/48/64

// Paso sonando de verdad para el sync de pads. p4.steps solo contiene la
// página visible del sequencer y p4.current_step va plegado a 0..15, así que
// con patrones de 2+ compases los golpes exclusivos del compás 2 (típicamente
// las voces generadas: acordes, respuestas, vueltas de toms) sonaban sin
// marcarse en los pads. Con la rejilla RAW y el paso absoluto del master, el
// pad marca exactamente lo que suena.
static bool live_step_hit(int track) {
    if (track < 0 || track > 15) return false;
    if (seq_raw_len > 16) {
        int rs = control_current_step_raw() % seq_raw_len;
        return seq_raw_grid[track][rs];
    }
    return p4.steps[track][p4.current_step];
}
static int        seq_page              = 0;    // 0..3
static bool       seq_force_refresh_cells = false; // force full cell repaint on next update_sequencer_screen
static lv_obj_t*  seq_page_btns[4]      = {};
static lv_obj_t*  seq_page_lbls[4]      = {};
static bool       seq_page_styles_dirty = false;
// Sequencer-local header buttons
static lv_obj_t*  seq_hdr_play_btn      = NULL;
static lv_obj_t*  seq_hdr_play_lbl      = NULL;
static lv_obj_t*  seq_hdr_pat_lbl       = NULL;
static lv_obj_t*  seq_hdr_name_lbl      = NULL;
static lv_obj_t*  seq_hdr_queue_btn     = NULL;
static lv_obj_t*  seq_hdr_queue_lbl     = NULL;
static lv_obj_t*  seq_hdr_var_btn       = NULL;
static lv_obj_t*  seq_variation_modal   = NULL;
static lv_obj_t*  seq_hdr_song_btn      = NULL;
static lv_obj_t*  seq_song_modal        = NULL;
static lv_obj_t*  seq_song_style_btns[6] = {};
static lv_obj_t*  seq_song_bars_btns[4]  = {};
static lv_obj_t*  seq_song_toggle_btn    = NULL;
static lv_obj_t*  seq_hdr_save_btn      = NULL;
static lv_obj_t*  seq_hdr_save_lbl      = NULL;
static lv_obj_t*  seq_hdr_kanban_btn    = NULL;
static lv_obj_t*  seq_kanban_modal      = NULL;
static lv_obj_t*  seq_hdr_group_btns[4] = {};
static uint8_t    seq_hdr_group_state[4] = {0xFF, 0xFF, 0xFF, 0xFF};
static lv_obj_t*  seq_pattern_list_modal = NULL;
static bool       seq_pattern_list_save_mode = false;
static lv_obj_t*  seq_save_confirm_modal = NULL;
static int        seq_save_confirm_slot = -1;
static bool       seq_pattern_dirty = false;
static lv_obj_t*  seq_pattern_modal     = NULL;
static lv_obj_t*  seq_pattern_modal_lbl = NULL;
static lv_obj_t*  seq_pattern_modal_spin = NULL;
static int        seq_pattern_wait_pat  = -1;
static uint32_t   seq_pattern_wait_ms   = 0;
static bool       seq_pattern_waiting   = false;

static void seq_open_midi_library(void);

static void seq_set_pattern_dirty(bool dirty) {
    seq_pattern_dirty = dirty;
    if (!seq_hdr_save_btn || !seq_hdr_save_lbl) return;
    lv_label_set_text(seq_hdr_save_lbl,
        dirty ? LV_SYMBOL_SAVE " SAVE*" : LV_SYMBOL_SAVE " SAVE");
    lv_obj_set_style_bg_color(seq_hdr_save_btn,
        dirty ? RED808_WARNING : RED808_SURFACE, 0);
    lv_obj_set_style_border_color(seq_hdr_save_btn,
        dirty ? RED808_ACCENT : RED808_SUCCESS, 0);
}

/* Slot remains authoritative; unknown slots are deliberately labeled generic
 * so an imported/user bank is never presented as the factory bank. */
static const char* seq_pattern_name(int pattern) {
    static const char* const factory[] = {
        "TECHNO FULL", "TECHNO BUILD", "TECHNO DRUMS", "TECHNO ACID", "TECHNO BREAK",
        "ELECTRO 505 VECTOR", "ELECTRO BUILD", "ELECTRO DRUMS", "ELECTRO ACID", "ELECTRO BREAK",
        "AMBIENT 505 PULSE", "AMBIENT SPARSE", "505 DUB ANCHOR", "AMBIENT MACHINE",
        "ACID RUN UP", "ACID DORIAN FALL", "ACID OCTAVE", "TOM FILL", "SNARE LIFT",
        "FINAL TRANSCENDENCE"
    };
    if (pattern >= 0 && pattern < (int)(sizeof(factory) / sizeof(factory[0])))
        return factory[pattern];
    if (pattern >= 100)
        return control_user_pattern_is_saved(pattern) ? "USER SAVED" : "USER EMPTY";
    return "MEMORY / IMPORT";
}

void ui_pattern_queue_committed(int pattern) {
    if (seq_queued_pattern == pattern) seq_queued_pattern = -1;
}

static void seq_pattern_list_hide(void) {
    if (!seq_pattern_list_modal) return;
    lv_obj_del(seq_pattern_list_modal);
    seq_pattern_list_modal = NULL;
}

static void seq_quantize_toggle_cb(lv_event_t* /*e*/) {
    seq_quantize_enabled = !seq_quantize_enabled;
    if (!seq_quantize_enabled) {
        seq_queued_pattern = -1;
        control_send_cancel_pattern_queue();
    }
    if (seq_hdr_queue_lbl)
        lv_label_set_text(seq_hdr_queue_lbl,
            seq_quantize_enabled ? "Q 1 BAR" : "Q OFF");
    if (seq_hdr_queue_btn) {
        lv_obj_set_style_bg_color(seq_hdr_queue_btn,
            seq_quantize_enabled ? RED808_SURFACE : RED808_ERROR, 0);
        lv_obj_set_style_border_color(seq_hdr_queue_btn,
            seq_quantize_enabled ? RED808_CYAN : RED808_WARNING, 0);
    }
    ui_show_toast(seq_quantize_enabled ? "Q 1 BAR activado" : "Q desactivado: cambio inmediato",
                  seq_quantize_enabled ? RED808_CYAN : RED808_WARNING);
}

static void seq_launch_absolute_pattern(int pattern) {
    pattern = constrain(pattern, 0, Config::MAX_PATTERNS - 1);
    if (p4.is_playing && seq_quantize_enabled && ui_control_available()) {
        seq_queued_pattern = pattern;
        control_send_queue_pattern(pattern);
        char msg[48];
        snprintf(msg, sizeof(msg), "Q 1 BAR -> P%02d", pattern + 1);
        ui_show_toast(msg, RED808_CYAN);
        return;
    }
    seq_queued_pattern = -1;
    if (control_available() || control_engine_connected())
        seq_pattern_modal_show(pattern);
    // Pattern selection is authoritative and synchronous in the P4 bank.
    // Daisy receives the resident-slot upload inside this call, but there is
    // no pattern_sync response packet to wait for.
    control_send_select_pattern(pattern);
    ui_sequencer_sync_from_current_pattern();
    if (seq_pattern_modal) seq_pattern_modal_mark_loaded();
}

static void seq_save_confirm_hide(void) {
    if (!seq_save_confirm_modal) return;
    lv_obj_del(seq_save_confirm_modal);
    seq_save_confirm_modal = NULL;
    seq_save_confirm_slot = -1;
}

static void seq_save_user_pattern(int destination) {
    const int source = p4.current_pattern;
    const bool saved = control_save_user_pattern(source, destination);
    seq_save_confirm_hide();
    seq_pattern_list_hide();
    if (!saved) {
        ui_show_toast("No se pudo guardar el patron", RED808_ERROR);
        return;
    }
    char message[64];
    snprintf(message, sizeof(message), "P%03d guardado desde P%03d",
             destination + 1, source + 1);
    seq_set_pattern_dirty(false);
    seq_launch_absolute_pattern(destination);
    ui_show_toast(message, RED808_SUCCESS);
}

static void seq_save_confirm_cb(lv_event_t* e) {
    const bool accept = (intptr_t)lv_event_get_user_data(e) != 0;
    const int destination = seq_save_confirm_slot;
    if (!accept) {
        seq_save_confirm_hide();
        return;
    }
    if (destination >= 100 && destination < Config::MAX_PATTERNS)
        seq_save_user_pattern(destination);
}

static void seq_show_save_confirm(int destination) {
    seq_save_confirm_hide();
    seq_save_confirm_slot = destination;
    seq_save_confirm_modal = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(seq_save_confirm_modal);
    lv_obj_set_size(seq_save_confirm_modal, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(seq_save_confirm_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(seq_save_confirm_modal, LV_OPA_60, 0);
    lv_obj_add_flag(seq_save_confirm_modal, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(seq_save_confirm_modal, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* card = lv_obj_create(seq_save_confirm_modal);
    lv_obj_set_size(card, 540, 220);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_border_color(card, RED808_WARNING, 0);
    lv_obj_set_style_radius(card, 14, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* title = lv_label_create(card);
    lv_label_set_text_fmt(title, "REEMPLAZAR P%03d?", destination + 1);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title, RED808_WARNING, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    lv_obj_t* copy = lv_label_create(card);
    lv_label_set_text(copy,
        "Ese slot de usuario ya contiene un patron.\nLa copia anterior sera sustituida.");
    lv_obj_set_style_text_font(copy, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(copy, RED808_TEXT, 0);
    lv_obj_set_style_text_align(copy, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(copy, LV_ALIGN_CENTER, 0, -10);

    lv_obj_t* cancel = lv_btn_create(card);
    lv_obj_set_size(cancel, 190, 48);
    lv_obj_align(cancel, LV_ALIGN_BOTTOM_LEFT, 18, -12);
    apply_control_button_style(cancel, RED808_BORDER, false, 8);
    lv_obj_add_event_cb(cancel, seq_save_confirm_cb, LV_EVENT_CLICKED,
                        (void*)(intptr_t)0);
    lv_obj_t* cancelLabel = lv_label_create(cancel);
    lv_label_set_text(cancelLabel, LV_SYMBOL_CLOSE "  CANCELAR");
    lv_obj_center(cancelLabel);

    lv_obj_t* replace = lv_btn_create(card);
    lv_obj_set_size(replace, 260, 48);
    lv_obj_align(replace, LV_ALIGN_BOTTOM_RIGHT, -18, -12);
    apply_control_button_style(replace, RED808_WARNING, true, 8);
    lv_obj_add_event_cb(replace, seq_save_confirm_cb, LV_EVENT_CLICKED,
                        (void*)(intptr_t)1);
    lv_obj_t* replaceLabel = lv_label_create(replace);
    lv_label_set_text(replaceLabel, LV_SYMBOL_SAVE "  REEMPLAZAR");
    lv_obj_set_style_text_color(replaceLabel, RED808_BG, 0);
    lv_obj_center(replaceLabel);
}

static void seq_pattern_list_pick_cb(lv_event_t* e) {
    const int pattern = (int)(intptr_t)lv_event_get_user_data(e);
    if (seq_pattern_list_save_mode) {
        if (control_user_pattern_is_saved(pattern)
            && pattern != p4.current_pattern) {
            seq_show_save_confirm(pattern);
            return;
        }
        seq_save_user_pattern(pattern);
        return;
    }
    seq_pattern_list_hide();
    seq_launch_absolute_pattern(pattern);
}

static void seq_pattern_list_show_mode(bool saveMode) {
    seq_pattern_list_hide();
    if (!scr_sequencer) return;
    seq_pattern_list_save_mode = saveMode;
    seq_pattern_list_modal = lv_obj_create(scr_sequencer);
    lv_obj_set_size(seq_pattern_list_modal, 850, 520);
    lv_obj_align(seq_pattern_list_modal, LV_ALIGN_CENTER, 0, 10);
    lv_obj_set_style_radius(seq_pattern_list_modal, 16, 0);
    lv_obj_set_style_bg_color(seq_pattern_list_modal, RED808_PANEL, 0);
    lv_obj_set_style_bg_opa(seq_pattern_list_modal, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(seq_pattern_list_modal, 2, 0);
    lv_obj_set_style_border_color(seq_pattern_list_modal, RED808_CYAN, 0);
    lv_obj_clear_flag(seq_pattern_list_modal, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* title = lv_label_create(seq_pattern_list_modal);
    lv_label_set_text(title, saveMode
        ? "GUARDAR COPIA · P101-P128"
        : "P1-P100 FACTORY · P101-P128 USER");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title, RED808_ACCENT, 0);
    lv_obj_set_pos(title, 16, 10);

    lv_obj_t* mode = lv_btn_create(seq_pattern_list_modal);
    lv_obj_set_size(mode, 164, 38);
    lv_obj_set_pos(mode, 600, 5);
    apply_control_button_style(mode, saveMode ? RED808_BORDER : RED808_SUCCESS, true, 8);
    lv_obj_add_event_cb(mode, [](lv_event_t*) {
        const bool nextMode = !seq_pattern_list_save_mode;
        seq_pattern_list_show_mode(nextMode);
    }, LV_EVENT_CLICKED, NULL);
    lv_obj_t* modeLabel = lv_label_create(mode);
    lv_label_set_text(modeLabel, saveMode ? LV_SYMBOL_LEFT "  LISTA" : LV_SYMBOL_SAVE "  SAVE USER");
    lv_obj_set_style_text_font(modeLabel, &lv_font_montserrat_14, 0);
    lv_obj_center(modeLabel);

    lv_obj_t* close = lv_btn_create(seq_pattern_list_modal);
    lv_obj_set_size(close, 42, 38);
    lv_obj_set_pos(close, 784, 5);
    apply_control_button_style(close, RED808_ERROR, false, 8);
    lv_obj_add_event_cb(close, [](lv_event_t*) { seq_pattern_list_hide(); }, LV_EVENT_CLICKED, NULL);
    lv_obj_t* cl = lv_label_create(close);
    lv_label_set_text(cl, LV_SYMBOL_CLOSE);
    lv_obj_center(cl);

    lv_obj_t* list = lv_obj_create(seq_pattern_list_modal);
    lv_obj_set_size(list, 818, 452);
    lv_obj_set_pos(list, 5, 50);
    lv_obj_set_style_bg_color(list, RED808_SURFACE, 0);
    lv_obj_set_style_bg_opa(list, LV_OPA_60, 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_style_pad_all(list, 6, 0);
    lv_obj_add_flag(list, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_AUTO);

    const int firstPattern = saveMode ? 100 : 0;
    for (int p = firstPattern; p < Config::MAX_PATTERNS; ++p) {
        const int visibleIndex = p - firstPattern;
        const int col = visibleIndex & 1;
        const int row = visibleIndex >> 1;
        lv_obj_t* btn = lv_btn_create(list);
        lv_obj_set_size(btn, 394, 42);
        lv_obj_set_pos(btn, col * 402, row * 46);
        const bool current = (p == p4.current_pattern);
        const bool occupied = control_user_pattern_is_saved(p);
        const lv_color_t slotColor = current ? RED808_ACCENT
            : (saveMode ? (occupied ? RED808_WARNING : RED808_SUCCESS)
                        : RED808_BORDER);
        apply_control_button_style(btn, slotColor, current, 7);
        lv_obj_add_event_cb(btn, seq_pattern_list_pick_cb, LV_EVENT_CLICKED,
                            (void*)(intptr_t)p);
        lv_obj_t* label = lv_label_create(btn);
        lv_label_set_text_fmt(label, "P%03d  %s", p + 1, seq_pattern_name(p));
        lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(label, current ? lv_color_white() : RED808_TEXT, 0);
        lv_obj_align(label, LV_ALIGN_LEFT_MID, 4, 0);
    }
    lv_obj_move_foreground(seq_pattern_list_modal);
}

static void seq_pattern_list_show(void) {
    seq_pattern_list_show_mode(false);
}

static uint16_t seq_group_mask(int group) {
    // Complete, non-overlapping performance groups for the 16 DSQ tracks.
    // DRUMS excludes track 7 (bass); XTRA are the three auxiliary sample rows.
    static const uint16_t masks[4] = {
        (uint16_t)(0x07FFu & ~(1u << 7)), // DRUMS: 0-6, 8-10
        (uint16_t)(1u << 7),              // BASS
        (uint16_t)((1u << 14) | (1u << 15)), // SYNTH
        (uint16_t)((1u << 11) | (1u << 12) | (1u << 13)) // XTRA
    };
    return (group >= 0 && group < 4) ? masks[group] : 0;
}

static void seq_group_mute_cb(lv_event_t* e) {
    const int group = (int)(intptr_t)lv_event_get_user_data(e);
    const uint16_t groupMask = seq_group_mask(group);
    bool allMuted = true;
    for (int t = 0; t < 16; ++t) {
        if ((groupMask & (1u << t)) && !p4.track_muted[t]) allMuted = false;
    }
    const bool mute = !allMuted;
    uint16_t fullMask = 0;
    for (int t = 0; t < 16; ++t) {
        if (groupMask & (1u << t)) {
            p4.track_muted[t] = mute;
        }
        if (p4.track_muted[t]) fullMask |= (uint16_t)(1u << t);
    }
    if (ui_control_available()) enqueue_mute_mask_control(fullMask);
    ui_show_toast(mute ? "Grupo en MUTE" : "Grupo activo",
                  mute ? RED808_ERROR : RED808_SUCCESS);
}

// GROUPS popup — replaces 4 separate always-visible header buttons
// (DRUMS/BASS/SYNTH/XTRA) with a single "GROUPS" entry point, freeing header
// width that was overlapping with the buttons next to it.
static lv_obj_t* seq_groups_modal = NULL;
static lv_obj_t* seq_groups_modal_btns[4] = {};

static void seq_groups_modal_hide(lv_event_t* e = NULL) {
    if (e && lv_event_get_target(e) != lv_event_get_current_target(e)) return;
    if (seq_groups_modal) lv_obj_del(seq_groups_modal);
    seq_groups_modal = NULL;
    for (int i = 0; i < 4; i++) seq_groups_modal_btns[i] = NULL;
}

static void seq_groups_modal_refresh(void) {
    if (!seq_groups_modal) return;
    for (int g = 0; g < 4; ++g) {
        lv_obj_t* btn = seq_groups_modal_btns[g];
        if (!btn) continue;
        const uint16_t groupMask = seq_group_mask(g);
        bool allMuted = true;
        for (int t = 0; t < 16; ++t)
            if ((groupMask & (1u << t)) && !p4.track_muted[t]) allMuted = false;
        apply_control_button_style(btn, allMuted ? RED808_ERROR : RED808_BORDER, false, 10);
    }
}

static void seq_groups_modal_btn_cb(lv_event_t* e) {
    seq_group_mute_cb(e);   // reuse the existing mask/toggle logic
    seq_groups_modal_refresh();
}

static void seq_groups_modal_show(lv_event_t* /*e*/) {
    if (seq_groups_modal) return;

    seq_groups_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(seq_groups_modal, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(seq_groups_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(seq_groups_modal, LV_OPA_80, 0);
    lv_obj_set_style_border_width(seq_groups_modal, 0, 0);
    lv_obj_set_style_pad_all(seq_groups_modal, 0, 0);
    lv_obj_clear_flag(seq_groups_modal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(seq_groups_modal, seq_groups_modal_hide, LV_EVENT_CLICKED, NULL);

    lv_obj_t* card = lv_obj_create(seq_groups_modal);
    lv_obj_set_size(card, 300, 296);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_border_color(card, RED808_CYAN, 0);
    lv_obj_set_style_radius(card, 16, 0);
    lv_obj_set_style_pad_all(card, 16, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t* title = lv_label_create(card);
    lv_label_set_text(title, "MUTE POR GRUPO");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(title, RED808_CYAN, 0);
    lv_obj_set_pos(title, 0, 0);

    static const char* const groupNames[4] = {"DRUMS", "BASS", "SYNTH", "XTRA"};
    for (int g = 0; g < 4; ++g) {
        lv_obj_t* btn = lv_btn_create(card);
        seq_groups_modal_btns[g] = btn;
        lv_obj_set_size(btn, 268, 40);
        lv_obj_set_pos(btn, 0, 36 + g * (40 + 8));
        lv_obj_add_event_cb(btn, seq_groups_modal_btn_cb, LV_EVENT_CLICKED,
                            (void*)(intptr_t)g);
        lv_obj_t* lbl = lv_label_create(btn);
        lv_label_set_text(lbl, groupNames[g]);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
        lv_obj_center(lbl);
    }

    lv_obj_t* close = lv_btn_create(card);
    lv_obj_set_size(close, 268, 40);
    lv_obj_set_pos(close, 0, 36 + 4 * (40 + 8));
    apply_control_button_style(close, RED808_BORDER, false, 10);
    lv_obj_add_event_cb(close, seq_groups_modal_hide, LV_EVENT_CLICKED, NULL);
    lv_obj_t* closeLbl = lv_label_create(close);
    lv_label_set_text(closeLbl, "CERRAR");
    lv_obj_set_style_text_font(closeLbl, &lv_font_montserrat_14, 0);
    lv_obj_center(closeLbl);

    seq_groups_modal_refresh();
}

static void seq_fill_cb(lv_event_t* /*e*/) {
    if (!p4.is_playing) {
        ui_show_toast("FILL necesita PLAY", RED808_WARNING);
        return;
    }
    control_send_fill();
    ui_show_toast("FILL: 1 compas + retorno", RED808_ACCENT);
}

struct SequencerVariationOption {
    uint8_t id;
    const char* name;
    const char* detail;
};

static const SequencerVariationOption SEQ_VARIATION_OPTIONS[] = {
    {SEQ_VAR_NEON_BREAK,    "NEON BREAK",    "Corte sincopado y remate final"},
    {SEQ_VAR_RATCHET_STORM, "RATCHET STORM", "Hi-hats en rafagas 2x / 4x"},
    {SEQ_VAR_GHOST_GROOVE,  "GHOST GROOVE",  "Golpes fantasma con probabilidad"},
    {SEQ_VAR_POLYRHYTHM,    "POLYRHYTHM 3x5", "Capas cruzadas 3, 5 y 7"},
    {SEQ_VAR_HALF_TIME,     "HALF-TIME DROP", "Peso en mitad de tiempo"},
    {SEQ_VAR_MIRROR,        "MIRROR BEAT",    "Invierte el pulso del compas"},
    {SEQ_VAR_TOM_CASCADE,   "TOM CASCADE",    "Descenso de toms al cierre"},
    {SEQ_VAR_ACID_SWITCH,   "ACID SWITCH",    "Secuencia acida en pistas synth"},
    {SEQ_VAR_HAT_LIFT,      "HAT LIFT",       "Subida de hats con ratchets"},
    {SEQ_VAR_SPARSE_SPACE,  "SPARSE SPACE",   "Abre huecos sin perder el kick"},
    {SEQ_VAR_UNDO,          "UNDO LAST VAR",  "Restaura el estado anterior"},
};

const char* ui_variation_name(uint8_t id) {
    for (const auto& option : SEQ_VARIATION_OPTIONS)
        if (option.id == id) return option.name;
    return "VARIACION";
}

struct SeqRandomStyleOption {
    uint8_t id;
    const char* name;
};

// RANDOM SONG's style names/short labels. The style only steers which
// existing pattern gets picked (see control_random_song_*); it never
// generates new step data.
static const SeqRandomStyleOption SEQ_RANDOM_STYLE_OPTIONS[] = {
    {RND_STYLE_TECHNO,    "TECHNO"},
    {RND_STYLE_HOUSE,     "HOUSE"},
    {RND_STYLE_BREAKBEAT, "BREAK"},
    {RND_STYLE_HIPHOP,    "HIP-HOP"},
    {RND_STYLE_TRAP,      "TRAP"},
    {RND_STYLE_MINIMAL,   "MINIMAL"},
};

static const char* seq_random_style_name(int8_t style) {
    for (const auto& option : SEQ_RANDOM_STYLE_OPTIONS)
        if (option.id == style) return option.name;
    return nullptr;
}

static lv_obj_t* seq_song_stop_badge = NULL;

// Reflects RANDOM SONG (control_random_song_active()) on its own SONG
// button: a shuffle icon and accent color while the mode keeps jumping
// between saved patterns every few bars, plain "SONG" otherwise. The
// corner badge lets RANDOM SONG be killed with one tap, without opening
// its modal. Pulled out of the VAR button/modal — RANDOM SONG is an AUTO
// mode like EVOLVE/AUTO FX/AUTO MIX, not a one-shot pattern transform.
static void seq_song_btn_refresh(void) {
    if (!seq_hdr_song_btn) return;
    const bool active = control_random_song_active();
    apply_control_button_style(seq_hdr_song_btn, active ? RED808_ACCENT2 : RED808_CYAN, false, 7);
    lv_obj_t* lbl = lv_obj_get_child(seq_hdr_song_btn, 0);
    if (lbl) lv_label_set_text(lbl, active ? LV_SYMBOL_SHUFFLE " SONG" : "SONG");
    if (seq_song_stop_badge) {
        if (active) lv_obj_clear_flag(seq_song_stop_badge, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(seq_song_stop_badge, LV_OBJ_FLAG_HIDDEN);
    }
}

static void seq_song_stop_badge_cb(lv_event_t* e) {
    LV_UNUSED(e);
    control_random_song_set_active(false);
    seq_song_btn_refresh();
}

// ── EVOLVE — third piece of PATTERN -> MUTATE -> EVOLVE ─────────────────
// Same bar-clock AUTO family as RANDOM SONG/AUTO FX/MIX, but it never
// swaps the pattern or its hit layout: control_random_evolve_apply_now()
// (control_api.cpp) only nudges the probability of already-active steps
// and the pattern's humanize amount, both scaled by the AMOUNT dial here
// and a fixed per-track freedom weight (kick close to fixed, hats/percs
// free) — the pattern keeps its identity, only its feel drifts.
static lv_obj_t* seq_hdr_evolve_btn = NULL;
static lv_obj_t* seq_evolve_stop_badge = NULL;
static lv_obj_t* seq_evolve_modal = NULL;
static lv_obj_t* seq_evolve_amount_slider = NULL;
static lv_obj_t* seq_evolve_amount_lbl = NULL;
static lv_obj_t* seq_evolve_bars_btns[4] = {};
static lv_obj_t* seq_evolve_toggle_btn = NULL;
static const uint8_t SEQ_EVOLVE_BAR_OPTIONS[4] = {1, 2, 4, 8};

static void seq_evolve_btn_refresh(void) {
    if (!seq_hdr_evolve_btn) return;
    const bool active = control_random_evolve_active();
    apply_control_button_style(seq_hdr_evolve_btn,
        active ? RED808_SUCCESS : RED808_ACCENT2, false, 7);
    if (seq_evolve_stop_badge) {
        if (active) lv_obj_clear_flag(seq_evolve_stop_badge, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(seq_evolve_stop_badge, LV_OBJ_FLAG_HIDDEN);
    }
}

static void seq_evolve_stop_badge_cb(lv_event_t* e) {
    LV_UNUSED(e);
    control_random_evolve_set_active(false);
    seq_evolve_btn_refresh();
}

static void seq_evolve_modal_refresh(void) {
    if (!seq_evolve_modal) return;
    const uint8_t amount = control_random_evolve_amount();
    if (seq_evolve_amount_slider)
        lv_slider_set_value(seq_evolve_amount_slider, amount, LV_ANIM_OFF);
    if (seq_evolve_amount_lbl) lv_label_set_text_fmt(seq_evolve_amount_lbl, "%d%%", amount);

    const uint8_t bars = control_random_evolve_bars();
    for (int i = 0; i < 4; i++) {
        lv_obj_t* btn = seq_evolve_bars_btns[i];
        if (!btn) continue;
        const bool sel = SEQ_EVOLVE_BAR_OPTIONS[i] == bars;
        apply_control_button_style(btn, sel ? RED808_ACCENT : RED808_BORDER, false, 8);
    }

    if (seq_evolve_toggle_btn) {
        const bool active = control_random_evolve_active();
        apply_control_button_style(seq_evolve_toggle_btn,
            active ? RED808_SUCCESS : RED808_BORDER, false, 10);
        lv_obj_t* lbl = lv_obj_get_child(seq_evolve_toggle_btn, 0);
        if (lbl) lv_label_set_text(lbl,
            active ? LV_SYMBOL_OK "  EVOLVE: ON" : LV_SYMBOL_CLOSE "  EVOLVE: OFF");
    }
}

static void seq_evolve_modal_hide(lv_event_t* e = NULL) {
    if (e && lv_event_get_target(e) != lv_event_get_current_target(e)) return;
    if (seq_evolve_modal) lv_obj_del(seq_evolve_modal);
    seq_evolve_modal = NULL;
    seq_evolve_amount_slider = NULL;
    seq_evolve_amount_lbl = NULL;
    for (int i = 0; i < 4; i++) seq_evolve_bars_btns[i] = NULL;
    seq_evolve_toggle_btn = NULL;
}

static void seq_evolve_amount_cb(lv_event_t* e) {
    lv_obj_t* slider = (lv_obj_t*)lv_event_get_target(e);
    control_random_evolve_set_amount((uint8_t)lv_slider_get_value(slider));
    seq_evolve_modal_refresh();
}

static void seq_evolve_bars_cb(lv_event_t* e) {
    const int bars = (int)(intptr_t)lv_event_get_user_data(e);
    control_random_evolve_set_bars((uint8_t)bars);
    seq_evolve_modal_refresh();
}

static void seq_evolve_toggle_cb(lv_event_t* e) {
    LV_UNUSED(e);
    control_random_evolve_set_active(!control_random_evolve_active());
    seq_evolve_modal_refresh();
    seq_evolve_btn_refresh();
}

static void seq_evolve_apply_now_cb(lv_event_t* e) {
    LV_UNUSED(e);
    control_random_evolve_apply_now();
}

static void seq_evolve_modal_show(lv_event_t* e) {
    LV_UNUSED(e);
    if (seq_evolve_modal) return;

    seq_evolve_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(seq_evolve_modal, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(seq_evolve_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(seq_evolve_modal, LV_OPA_80, 0);
    lv_obj_set_style_border_width(seq_evolve_modal, 0, 0);
    lv_obj_set_style_pad_all(seq_evolve_modal, 0, 0);
    lv_obj_clear_flag(seq_evolve_modal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(seq_evolve_modal, seq_evolve_modal_hide, LV_EVENT_CLICKED, NULL);

    lv_obj_t* card = lv_obj_create(seq_evolve_modal);
    lv_obj_set_size(card, 420, 344);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_border_color(card, RED808_SUCCESS, 0);
    lv_obj_set_style_radius(card, 16, 0);
    lv_obj_set_style_pad_all(card, 16, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t* title = lv_label_create(card);
    lv_label_set_text(title, "EVOLVE");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title, RED808_SUCCESS, 0);
    lv_obj_set_pos(title, 0, 0);

    lv_obj_t* hint = lv_label_create(card);
    lv_label_set_text(hint, "Prob + ghost hits en hats/percs; kick/snare fijos");
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(hint, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(hint, 0, 26);

    lv_obj_t* amountLbl = lv_label_create(card);
    lv_label_set_text(amountLbl, "CANTIDAD:");
    lv_obj_set_style_text_font(amountLbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(amountLbl, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(amountLbl, 0, 48);

    seq_evolve_amount_lbl = lv_label_create(card);
    lv_obj_set_style_text_font(seq_evolve_amount_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(seq_evolve_amount_lbl, RED808_SUCCESS, 0);
    lv_obj_set_pos(seq_evolve_amount_lbl, 340, 44);

    seq_evolve_amount_slider = lv_slider_create(card);
    lv_obj_set_size(seq_evolve_amount_slider, 388, 16);
    lv_obj_set_pos(seq_evolve_amount_slider, 0, 68);
    lv_slider_set_range(seq_evolve_amount_slider, 0, 100);
    lv_obj_set_style_bg_color(seq_evolve_amount_slider, RED808_SURFACE, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(seq_evolve_amount_slider, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(seq_evolve_amount_slider, RED808_SUCCESS, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(seq_evolve_amount_slider, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(seq_evolve_amount_slider, lv_color_white(), LV_PART_KNOB);
    lv_obj_set_style_bg_opa(seq_evolve_amount_slider, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_pad_all(seq_evolve_amount_slider, 8, LV_PART_KNOB);
    lv_obj_set_style_radius(seq_evolve_amount_slider, LV_RADIUS_CIRCLE, LV_PART_KNOB);
    lv_obj_add_event_cb(seq_evolve_amount_slider, seq_evolve_amount_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(seq_evolve_amount_slider, seq_evolve_amount_cb, LV_EVENT_RELEASED, NULL);

    lv_obj_t* barsLbl = lv_label_create(card);
    lv_label_set_text(barsLbl, "CADA CUANTOS COMPASES:");
    lv_obj_set_style_text_font(barsLbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(barsLbl, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(barsLbl, 0, 96);

    for (int i = 0; i < 4; i++) {
        lv_obj_t* btn = lv_btn_create(card);
        seq_evolve_bars_btns[i] = btn;
        lv_obj_set_size(btn, 86, 36);
        lv_obj_set_pos(btn, i * (86 + 8), 116);
        lv_obj_add_event_cb(btn, seq_evolve_bars_cb, LV_EVENT_CLICKED,
                            (void*)(intptr_t)SEQ_EVOLVE_BAR_OPTIONS[i]);
        lv_obj_t* lbl = lv_label_create(btn);
        lv_label_set_text_fmt(lbl, "%d", SEQ_EVOLVE_BAR_OPTIONS[i]);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
        lv_obj_center(lbl);
    }

    seq_evolve_toggle_btn = lv_btn_create(card);
    lv_obj_set_size(seq_evolve_toggle_btn, 388, 48);
    lv_obj_set_pos(seq_evolve_toggle_btn, 0, 160);
    lv_obj_add_event_cb(seq_evolve_toggle_btn, seq_evolve_toggle_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* toggleLbl = lv_label_create(seq_evolve_toggle_btn);
    lv_obj_set_style_text_font(toggleLbl, &lv_font_montserrat_16, 0);
    lv_obj_center(toggleLbl);

    lv_obj_t* applyBtn = lv_btn_create(card);
    lv_obj_set_size(applyBtn, 388, 40);
    lv_obj_set_pos(applyBtn, 0, 220);
    apply_control_button_style(applyBtn, RED808_INFO, false, 10);
    lv_obj_add_event_cb(applyBtn, seq_evolve_apply_now_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* applyLbl = lv_label_create(applyBtn);
    lv_label_set_text(applyLbl, LV_SYMBOL_SHUFFLE "  APLICAR AHORA");
    lv_obj_set_style_text_font(applyLbl, &lv_font_montserrat_14, 0);
    lv_obj_center(applyLbl);

    lv_obj_t* closeBtn = lv_btn_create(card);
    lv_obj_set_size(closeBtn, 388, 36);
    lv_obj_set_pos(closeBtn, 0, 268);
    apply_control_button_style(closeBtn, RED808_BORDER, false, 10);
    lv_obj_add_event_cb(closeBtn, seq_evolve_modal_hide, LV_EVENT_CLICKED, NULL);
    lv_obj_t* closeLbl = lv_label_create(closeBtn);
    lv_label_set_text(closeLbl, "CERRAR");
    lv_obj_set_style_text_font(closeLbl, &lv_font_montserrat_14, 0);
    lv_obj_center(closeLbl);

    seq_evolve_modal_refresh();
}

static void seq_variation_modal_hide(lv_event_t* e = NULL) {
    if (e && lv_event_get_target(e) != lv_event_get_current_target(e)) return;
    if (seq_variation_modal) lv_obj_del(seq_variation_modal);
    seq_variation_modal = NULL;
}

static void seq_variation_select_cb(lv_event_t* e) {
    static uint32_t lastApplyMs = 0;
    const uint32_t now = millis();
    if (lastApplyMs != 0 && now - lastApplyMs < 180u) return;
    lastApplyMs = now;

    const uint8_t selected = static_cast<uint8_t>(
        reinterpret_cast<uintptr_t>(lv_event_get_user_data(e)));
    if (selected < SEQ_VAR_NEON_BREAK || selected > SEQ_VAR_UNDO) return;
    if (selected == SEQ_VAR_UNDO && !control_variation_can_undo()) {
        ui_show_toast("No hay una variacion anterior para restaurar",
                      RED808_WARNING);
        return;
    }

    const bool changed = control_apply_sequencer_variation(selected);
    if (!changed) {
        ui_show_toast("La variacion no cambia este patron", RED808_WARNING);
        return;
    }

    const int base = seq_page * 16;
    for (int track = 0; track < 16; ++track)
        for (int step = 0; step < 16; ++step)
            if (base + step < 64)
                seq_raw_grid[track][base + step] = p4.steps[track][step];
    seq_force_refresh_cells = true;
    seq_set_pattern_dirty(true);

    // Never perform a multi-packet upload from the LVGL callback. The loop
    // drains this flag and sends one coherent pattern snapshot to Daisy.
    s_ctrl_pattern_sync_pending.store(true, std::memory_order_release);
    const char* feedback = "VAR APLICADA";
    for (const auto& option : SEQ_VARIATION_OPTIONS)
        if (option.id == selected) { feedback = option.name; break; }
    seq_variation_modal_hide();
    ui_show_toast(feedback,
                  selected == SEQ_VAR_UNDO ? RED808_SUCCESS : RED808_CYAN);
}

// ── RANDOM SONG controls: style select + bar cadence + on/off toggle ──────
static void seq_song_controls_refresh(void) {
    const uint8_t style = control_random_song_style();
    const uint8_t bars = control_random_song_bars();
    const bool active = control_random_song_active();
    for (size_t i = 0; i < sizeof(SEQ_RANDOM_STYLE_OPTIONS) / sizeof(SEQ_RANDOM_STYLE_OPTIONS[0]); ++i) {
        lv_obj_t* chip = seq_song_style_btns[i];
        if (!chip) continue;
        const bool sel = SEQ_RANDOM_STYLE_OPTIONS[i].id == style;
        apply_control_button_style(chip, sel ? RED808_ACCENT2 : RED808_BORDER, false, 10);
    }
    static const uint8_t barOptions[4] = {1, 2, 4, 8};
    for (int i = 0; i < 4; ++i) {
        lv_obj_t* chip = seq_song_bars_btns[i];
        if (!chip) continue;
        apply_control_button_style(chip, barOptions[i] == bars ? RED808_ACCENT2 : RED808_BORDER, false, 8);
    }
    if (seq_song_toggle_btn) {
        apply_control_button_style(seq_song_toggle_btn,
            active ? RED808_SUCCESS : RED808_BORDER, false, 10);
        lv_obj_t* lbl = lv_obj_get_child(seq_song_toggle_btn, 0);
        if (lbl) lv_label_set_text(lbl,
            active ? LV_SYMBOL_OK "  RANDOM SONG: ON" : LV_SYMBOL_CLOSE "  RANDOM SONG: OFF");
    }
    seq_song_btn_refresh();
}

static void seq_song_style_cb(lv_event_t* e) {
    const uint8_t style = static_cast<uint8_t>(
        reinterpret_cast<uintptr_t>(lv_event_get_user_data(e)));
    control_random_song_set_style(style);
    seq_song_controls_refresh();
}

static void seq_song_bars_cb(lv_event_t* e) {
    const uint8_t bars = static_cast<uint8_t>(
        reinterpret_cast<uintptr_t>(lv_event_get_user_data(e)));
    control_random_song_set_bars(bars);
    seq_song_controls_refresh();
}

static void seq_song_toggle_cb(lv_event_t* /*e*/) {
    const bool nowActive = !control_random_song_active();
    if (nowActive && !p4.is_playing)
        ui_show_toast("RANDOM SONG activado: empieza a saltar en cuanto le des a PLAY",
                      RED808_CYAN);
    control_random_song_set_active(nowActive);
    seq_song_controls_refresh();
}

// ── RANDOM SONG modal — pulled out of VAR's popup. It's an AUTO mode tied
// to playback like EVOLVE/AUTO FX/AUTO MIX (jump to a different saved
// pattern every N bars), not a one-shot pattern transform, so it gets the
// same standalone treatment: its own header button + popup + stop badge.
static void seq_song_modal_hide(lv_event_t* e = NULL) {
    if (e && lv_event_get_target(e) != lv_event_get_current_target(e)) return;
    if (seq_song_modal) lv_obj_del(seq_song_modal);
    seq_song_modal = NULL;
    for (int i = 0; i < 6; i++) seq_song_style_btns[i] = NULL;
    for (int i = 0; i < 4; i++) seq_song_bars_btns[i] = NULL;
    seq_song_toggle_btn = NULL;
}

static void seq_song_modal_show(lv_event_t* /*e*/) {
    if (seq_song_modal) return;

    seq_song_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(seq_song_modal, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(seq_song_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(seq_song_modal, LV_OPA_80, 0);
    lv_obj_set_style_border_width(seq_song_modal, 0, 0);
    lv_obj_set_style_pad_all(seq_song_modal, 0, 0);
    lv_obj_clear_flag(seq_song_modal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(seq_song_modal, seq_song_modal_hide, LV_EVENT_CLICKED, NULL);

    lv_obj_t* card = lv_obj_create(seq_song_modal);
    lv_obj_set_size(card, 640, 312);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_border_color(card, RED808_CYAN, 0);
    lv_obj_set_style_radius(card, 16, 0);
    lv_obj_set_style_pad_all(card, 16, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t* title = lv_label_create(card);
    lv_label_set_text(title, "RANDOM SONG");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title, RED808_CYAN, 0);
    lv_obj_set_pos(title, 0, 0);

    lv_obj_t* hint = lv_label_create(card);
    lv_label_set_text(hint, "Salta a otro patron guardado cada N compases");
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(hint, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(hint, 0, 26);

    lv_obj_t* songLabel = lv_label_create(card);
    lv_label_set_text(songLabel, "ESTILO:");
    lv_obj_set_style_text_font(songLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(songLabel, RED808_ACCENT2, 0);
    lv_obj_set_pos(songLabel, 0, 48);

    {
        constexpr int chipW = 92;
        constexpr int chipH = 34;
        constexpr int chipGap = 6;
        constexpr int chipY = 64;
        for (size_t index = 0;
             index < sizeof(SEQ_RANDOM_STYLE_OPTIONS) / sizeof(SEQ_RANDOM_STYLE_OPTIONS[0]);
             ++index) {
            const auto& style = SEQ_RANDOM_STYLE_OPTIONS[index];
            lv_obj_t* chip = lv_btn_create(card);
            seq_song_style_btns[index] = chip;
            lv_obj_set_size(chip, chipW, chipH);
            lv_obj_set_pos(chip, static_cast<int>(index) * (chipW + chipGap), chipY);
            lv_obj_add_event_cb(chip, seq_song_style_cb, LV_EVENT_CLICKED,
                                reinterpret_cast<void*>(
                                    static_cast<uintptr_t>(style.id)));
            lv_obj_t* chipLabel = lv_label_create(chip);
            lv_label_set_text(chipLabel, style.name);
            lv_obj_set_style_text_font(chipLabel, &lv_font_montserrat_12, 0);
            lv_obj_set_style_text_color(chipLabel, lv_color_white(), 0);
            lv_obj_center(chipLabel);
        }
    }

    {
        constexpr int barsY = 106;
        lv_obj_t* barsLabel = lv_label_create(card);
        lv_label_set_text(barsLabel, "COMPASES:");
        lv_obj_set_style_text_font(barsLabel, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(barsLabel, RED808_TEXT_DIM, 0);
        lv_obj_set_pos(barsLabel, 0, barsY + 10);

        static const uint8_t barOptions[4] = {1, 2, 4, 8};
        constexpr int barsChipX0 = 86;
        constexpr int barsChipW = 48;
        constexpr int barsChipGap = 6;
        for (int i = 0; i < 4; ++i) {
            lv_obj_t* chip = lv_btn_create(card);
            seq_song_bars_btns[i] = chip;
            lv_obj_set_size(chip, barsChipW, 32);
            lv_obj_set_pos(chip, barsChipX0 + i * (barsChipW + barsChipGap), barsY);
            lv_obj_add_event_cb(chip, seq_song_bars_cb, LV_EVENT_CLICKED,
                                (void*)(intptr_t)barOptions[i]);
            lv_obj_t* lbl = lv_label_create(chip);
            lv_label_set_text_fmt(lbl, "%d", barOptions[i]);
            lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
            lv_obj_center(lbl);
        }
    }

    seq_song_toggle_btn = lv_btn_create(card);
    lv_obj_set_size(seq_song_toggle_btn, 608, 40);
    lv_obj_set_pos(seq_song_toggle_btn, 0, 146);
    lv_obj_add_event_cb(seq_song_toggle_btn, seq_song_toggle_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* toggleLbl = lv_label_create(seq_song_toggle_btn);
    lv_obj_set_style_text_font(toggleLbl, &lv_font_montserrat_14, 0);
    lv_obj_center(toggleLbl);

    // MATRIX — compose an authored song from existing patterns + filter/
    // mixer/melody presets per column, instead of RANDOM SONG's automatic
    // jumps above. Its own full-screen grid, opened from here.
    lv_obj_t* matrixBtn = lv_btn_create(card);
    lv_obj_set_size(matrixBtn, 608, 40);
    lv_obj_set_pos(matrixBtn, 0, 194);
    apply_control_button_style(matrixBtn, RED808_ACCENT2, false, 10);
    lv_obj_add_event_cb(matrixBtn, matrix_modal_show, LV_EVENT_CLICKED, NULL);
    lv_obj_t* matrixLbl = lv_label_create(matrixBtn);
    lv_label_set_text(matrixLbl, "MATRIX — componer cancion");
    lv_obj_set_style_text_font(matrixLbl, &lv_font_montserrat_14, 0);
    lv_obj_center(matrixLbl);

    lv_obj_t* closeBtn = lv_btn_create(card);
    lv_obj_set_size(closeBtn, 608, 32);
    lv_obj_set_pos(closeBtn, 0, 242);
    apply_control_button_style(closeBtn, RED808_BORDER, false, 10);
    lv_obj_add_event_cb(closeBtn, seq_song_modal_hide, LV_EVENT_CLICKED, NULL);
    lv_obj_t* closeLbl = lv_label_create(closeBtn);
    lv_label_set_text(closeLbl, "CERRAR");
    lv_obj_set_style_text_font(closeLbl, &lv_font_montserrat_14, 0);
    lv_obj_center(closeLbl);

    seq_song_controls_refresh();
}

// ── AUTO panel — Kanban-style command center for the four bar-clock AUTO
// modes (SONG/AUTO FX/AUTO MIX/EVOLVE). Each already has its own header
// button + popup; this just puts all four ON/OFF + cadence controls
// side by side so you don't have to hop between four popups to see or
// change what's currently running. All state lives in control_api.cpp
// already (control_random_song/fx/mix/evolve_*) — this panel is pure UI.
enum { KANBAN_SONG = 0, KANBAN_FX, KANBAN_MIX, KANBAN_EVOLVE, KANBAN_VARIATION, KANBAN_COUNT };
static const char* const KANBAN_NAMES[KANBAN_COUNT] = {"SONG", "AUTO FX", "AUTO MIX", "EVOLVE", "VAR"};
static const uint8_t KANBAN_BAR_OPTIONS[4] = {1, 2, 4, 8};

// fwd decl: defined later, in the MIXER section — used here to keep the
// Mixer's own RANDOM MIX button in sync the instant this panel toggles it.
static void mix_random_btn_refresh(void);

static lv_obj_t* seq_kanban_toggle_btns[KANBAN_COUNT]  = {};
static lv_obj_t* seq_kanban_bars_btns[KANBAN_COUNT][4] = {};
static lv_obj_t* seq_kanban_style_btn    = NULL;   // SONG only
static lv_obj_t* seq_kanban_evolve_slider = NULL;  // EVOLVE only
static lv_obj_t* seq_kanban_evolve_lbl    = NULL;  // EVOLVE only
static lv_obj_t* seq_kanban_variation_btn = NULL;  // VARIATIONS only — manual "fire one now"
static lv_obj_t* seq_kanban_toast_btn = NULL;        // global — mute/unmute AUTO toasts
static lv_obj_t* seq_kanban_song_curated_btn = NULL; // SONG only — curated vs weighted flow

// Each column's ON/OFF toggle doubles as its bypass switch: turning a mode
// OFF never clears its bars/style/amount — those stay exactly as set, so
// flipping it back ON resumes with the same configuration. That's true for
// all five columns, VARIATIONS included, with no extra state needed.
static bool kanban_active(int mode) {
    switch (mode) {
        case KANBAN_SONG:      return control_random_song_active();
        case KANBAN_FX:        return control_random_fx_active();
        case KANBAN_MIX:       return control_random_mix_active();
        case KANBAN_EVOLVE:    return control_random_evolve_active();
        case KANBAN_VARIATION: return control_random_variation_active();
        default: return false;
    }
}
static void kanban_set_active(int mode, bool v) {
    switch (mode) {
        case KANBAN_SONG:      control_random_song_set_active(v); break;
        case KANBAN_FX:        control_random_fx_set_active(v); break;
        case KANBAN_MIX:       control_random_mix_set_active(v); break;
        case KANBAN_EVOLVE:    control_random_evolve_set_active(v); break;
        case KANBAN_VARIATION: control_random_variation_set_active(v); break;
    }
}
static uint8_t kanban_bars(int mode) {
    switch (mode) {
        case KANBAN_SONG:      return control_random_song_bars();
        case KANBAN_FX:        return control_random_fx_bars();
        case KANBAN_MIX:       return control_random_mix_bars();
        case KANBAN_EVOLVE:    return control_random_evolve_bars();
        case KANBAN_VARIATION: return control_random_variation_bars();
        default: return 1;
    }
}
static void kanban_set_bars(int mode, uint8_t bars) {
    switch (mode) {
        case KANBAN_SONG:      control_random_song_set_bars(bars); break;
        case KANBAN_FX:        control_random_fx_set_bars(bars); break;
        case KANBAN_MIX:       control_random_mix_set_bars(bars); break;
        case KANBAN_EVOLVE:    control_random_evolve_set_bars(bars); break;
        case KANBAN_VARIATION: control_random_variation_set_bars(bars); break;
    }
}
// Keeps each mode's own dedicated header button (SONG/EVOLVE) or screen
// button (AUTO FX/AUTO MIX) visually in sync the instant this panel
// changes it, so nothing here can go stale next time you see it elsewhere.
// VARIATIONS has no such dedicated button anywhere else — this panel is
// its only home — so there is nothing to sync for it.
static void kanban_refresh_owner_btn(int mode) {
    switch (mode) {
        case KANBAN_SONG:   seq_song_btn_refresh(); break;
        case KANBAN_FX:     fx_random_btn_refresh(); break;
        case KANBAN_MIX:    mix_random_btn_refresh(); break;
        case KANBAN_EVOLVE: seq_evolve_btn_refresh(); break;
    }
}

static void seq_kanban_refresh(void) {
    if (!seq_kanban_modal) return;
    for (int m = 0; m < KANBAN_COUNT; ++m) {
        const bool active = kanban_active(m);
        lv_obj_t* toggle = seq_kanban_toggle_btns[m];
        if (toggle) {
            apply_control_button_style(toggle, active ? RED808_SUCCESS : RED808_BORDER, false, 8);
            lv_obj_t* lbl = lv_obj_get_child(toggle, 0);
            if (lbl) lv_label_set_text_fmt(lbl, "%s\n%s", KANBAN_NAMES[m], active ? "ON" : "OFF");
        }
        const uint8_t bars = kanban_bars(m);
        for (int i = 0; i < 4; ++i) {
            lv_obj_t* chip = seq_kanban_bars_btns[m][i];
            if (!chip) continue;
            apply_control_button_style(chip,
                KANBAN_BAR_OPTIONS[i] == bars ? RED808_ACCENT2 : RED808_BORDER, false, 6);
        }
    }
    if (seq_kanban_style_btn) {
        lv_obj_t* lbl = lv_obj_get_child(seq_kanban_style_btn, 0);
        const char* name = seq_random_style_name(control_random_song_style());
        if (lbl) lv_label_set_text_fmt(lbl, "ESTILO\n%s", name ? name : "?");
    }
    if (seq_kanban_evolve_slider)
        lv_slider_set_value(seq_kanban_evolve_slider, control_random_evolve_amount(), LV_ANIM_OFF);
    if (seq_kanban_evolve_lbl)
        lv_label_set_text_fmt(seq_kanban_evolve_lbl, "%d%%", control_random_evolve_amount());
    if (seq_kanban_toast_btn) {
        const bool on = control_auto_toast_enabled();
        apply_control_button_style(seq_kanban_toast_btn, on ? RED808_SUCCESS : RED808_BORDER, false, 8);
        lv_obj_t* lbl = lv_obj_get_child(seq_kanban_toast_btn, 0);
        if (lbl) lv_label_set_text_fmt(lbl, "TOASTS %s", on ? "ON" : "OFF");
    }
    if (seq_kanban_song_curated_btn) {
        const bool on = control_random_song_curated();
        apply_control_button_style(seq_kanban_song_curated_btn, on ? RED808_SUCCESS : RED808_BORDER, false, 8);
        lv_obj_t* lbl = lv_obj_get_child(seq_kanban_song_curated_btn, 0);
        if (lbl) lv_label_set_text_fmt(lbl, "CURADO\n%s", on ? "ON" : "OFF");
    }
}

static void seq_kanban_toggle_cb(lv_event_t* e) {
    const int mode = (int)(intptr_t)lv_event_get_user_data(e);
    kanban_set_active(mode, !kanban_active(mode));
    seq_kanban_refresh();
    kanban_refresh_owner_btn(mode);
}

static void seq_kanban_bars_cb(lv_event_t* e) {
    const int packed = (int)(intptr_t)lv_event_get_user_data(e);
    const int mode = (packed >> 8) & 0xFF;
    const uint8_t bars = (uint8_t)(packed & 0xFF);
    kanban_set_bars(mode, bars);
    seq_kanban_refresh();
}

static void seq_kanban_style_cb(lv_event_t* /*e*/) {
    const uint8_t next = (control_random_song_style() + 1) % 6;
    control_random_song_set_style(next);
    seq_kanban_refresh();
    seq_song_btn_refresh();
}

static void seq_kanban_evolve_amount_cb(lv_event_t* e) {
    lv_obj_t* slider = (lv_obj_t*)lv_event_get_target(e);
    control_random_evolve_set_amount((uint8_t)lv_slider_get_value(slider));
    seq_kanban_refresh();
}

static void seq_kanban_toast_toggle_cb(lv_event_t* /*e*/) {
    control_auto_toast_set_enabled(!control_auto_toast_enabled());
    seq_kanban_refresh();
}

static void seq_kanban_song_curated_cb(lv_event_t* /*e*/) {
    control_random_song_set_curated(!control_random_song_curated());
    seq_kanban_refresh();
}

static void seq_kanban_variation_random_cb(lv_event_t* /*e*/) {
    if (!control_random_variation_apply_now()) {
        ui_show_toast("La variacion no cambia este patron", RED808_WARNING);
        return;
    }
    const int base = seq_page * 16;
    for (int track = 0; track < 16; ++track)
        for (int step = 0; step < 16; ++step)
            if (base + step < 64)
                seq_raw_grid[track][base + step] = p4.steps[track][step];
    seq_force_refresh_cells = true;
    seq_set_pattern_dirty(true);
    // Never perform a multi-packet upload from the LVGL callback — same
    // rule the manual VAR popup follows. The loop drains this flag and
    // sends one coherent pattern snapshot to Daisy.
    s_ctrl_pattern_sync_pending.store(true, std::memory_order_release);
    ui_show_toast("Variacion aleatoria aplicada", RED808_CYAN);
}

static void seq_kanban_modal_hide(lv_event_t* e = NULL) {
    if (e && lv_event_get_target(e) != lv_event_get_current_target(e)) return;
    if (seq_kanban_modal) lv_obj_del(seq_kanban_modal);
    seq_kanban_modal = NULL;
    for (int m = 0; m < KANBAN_COUNT; ++m) {
        seq_kanban_toggle_btns[m] = NULL;
        for (int i = 0; i < 4; ++i) seq_kanban_bars_btns[m][i] = NULL;
    }
    seq_kanban_style_btn = NULL;
    seq_kanban_evolve_slider = NULL;
    seq_kanban_evolve_lbl = NULL;
    seq_kanban_variation_btn = NULL;
    seq_kanban_toast_btn = NULL;
    seq_kanban_song_curated_btn = NULL;
}

static void seq_kanban_modal_show(lv_event_t* /*e*/) {
    if (seq_kanban_modal) return;

    seq_kanban_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(seq_kanban_modal, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(seq_kanban_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(seq_kanban_modal, LV_OPA_80, 0);
    lv_obj_set_style_border_width(seq_kanban_modal, 0, 0);
    lv_obj_set_style_pad_all(seq_kanban_modal, 0, 0);
    lv_obj_clear_flag(seq_kanban_modal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(seq_kanban_modal, seq_kanban_modal_hide, LV_EVENT_CLICKED, NULL);

    lv_obj_t* card = lv_obj_create(seq_kanban_modal);
    lv_obj_set_size(card, 1000, 320);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_border_color(card, RED808_CYAN, 0);
    lv_obj_set_style_radius(card, 16, 0);
    lv_obj_set_style_pad_all(card, 16, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t* title = lv_label_create(card);
    lv_label_set_text(title, "AUTO");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title, RED808_CYAN, 0);
    lv_obj_set_pos(title, 0, 0);

    lv_obj_t* hint = lv_label_create(card);
    lv_label_set_text(hint, "Los 5 modos ligados a reproduccion, de un vistazo. ON/OFF = bypass: no borra el ajuste");
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(hint, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(hint, 0, 26);

    // Global — mutes the informational toasts SONG/VARIATIONS/FX/MIX show
    // while running unattended (never affects what the modes actually do).
    seq_kanban_toast_btn = lv_btn_create(card);
    lv_obj_set_size(seq_kanban_toast_btn, 140, 26);
    lv_obj_set_pos(seq_kanban_toast_btn, 1000 - 32 - 140 - 8, 2);
    lv_obj_add_event_cb(seq_kanban_toast_btn, seq_kanban_toast_toggle_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* toastLbl = lv_label_create(seq_kanban_toast_btn);
    lv_obj_set_style_text_font(toastLbl, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_align(toastLbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(toastLbl);

    constexpr int colW = 185;
    constexpr int colGap = 8;

    for (int m = 0; m < KANBAN_COUNT; ++m) {
        const int colX = m * (colW + colGap);

        lv_obj_t* toggle = lv_btn_create(card);
        seq_kanban_toggle_btns[m] = toggle;
        lv_obj_set_size(toggle, colW, 36);
        lv_obj_set_pos(toggle, colX, 48);
        lv_obj_add_event_cb(toggle, seq_kanban_toggle_cb, LV_EVENT_CLICKED, (void*)(intptr_t)m);
        lv_obj_t* toggleLbl = lv_label_create(toggle);
        lv_obj_set_style_text_font(toggleLbl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_align(toggleLbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_center(toggleLbl);

        lv_obj_t* barsLbl = lv_label_create(card);
        lv_label_set_text(barsLbl, "COMPASES:");
        lv_obj_set_style_text_font(barsLbl, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(barsLbl, RED808_TEXT_DIM, 0);
        lv_obj_set_pos(barsLbl, colX, 92);

        constexpr int chipW = 41;
        constexpr int chipGap = 5;
        for (int i = 0; i < 4; ++i) {
            lv_obj_t* chip = lv_btn_create(card);
            seq_kanban_bars_btns[m][i] = chip;
            lv_obj_set_size(chip, chipW, 28);
            lv_obj_set_pos(chip, colX + i * (chipW + chipGap), 106);
            lv_obj_add_event_cb(chip, seq_kanban_bars_cb, LV_EVENT_CLICKED,
                                (void*)(intptr_t)((m << 8) | KANBAN_BAR_OPTIONS[i]));
            lv_obj_t* lbl = lv_label_create(chip);
            lv_label_set_text_fmt(lbl, "%d", KANBAN_BAR_OPTIONS[i]);
            lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, 0);
            lv_obj_center(lbl);
        }

        if (m == KANBAN_SONG) {
            seq_kanban_style_btn = lv_btn_create(card);
            lv_obj_set_size(seq_kanban_style_btn, colW, 34);
            lv_obj_set_pos(seq_kanban_style_btn, colX, 140);
            apply_control_button_style(seq_kanban_style_btn, RED808_ACCENT2, false, 8);
            lv_obj_add_event_cb(seq_kanban_style_btn, seq_kanban_style_cb, LV_EVENT_CLICKED, NULL);
            lv_obj_t* lbl = lv_label_create(seq_kanban_style_btn);
            lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);
            lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_center(lbl);

            // Curated: walks the expansion bank's written scene order
            // (known-good sequence) instead of the weighted-random jump.
            seq_kanban_song_curated_btn = lv_btn_create(card);
            lv_obj_set_size(seq_kanban_song_curated_btn, colW, 32);
            lv_obj_set_pos(seq_kanban_song_curated_btn, colX, 180);
            lv_obj_add_event_cb(seq_kanban_song_curated_btn, seq_kanban_song_curated_cb,
                                LV_EVENT_CLICKED, NULL);
            lv_obj_t* curLbl = lv_label_create(seq_kanban_song_curated_btn);
            lv_obj_set_style_text_font(curLbl, &lv_font_montserrat_10, 0);
            lv_obj_set_style_text_align(curLbl, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_center(curLbl);
        } else if (m == KANBAN_EVOLVE) {
            lv_obj_t* amountLbl = lv_label_create(card);
            lv_label_set_text(amountLbl, "CANTIDAD:");
            lv_obj_set_style_text_font(amountLbl, &lv_font_montserrat_10, 0);
            lv_obj_set_style_text_color(amountLbl, RED808_TEXT_DIM, 0);
            lv_obj_set_pos(amountLbl, colX, 140);

            seq_kanban_evolve_lbl = lv_label_create(card);
            lv_obj_set_style_text_font(seq_kanban_evolve_lbl, &lv_font_montserrat_10, 0);
            lv_obj_set_style_text_color(seq_kanban_evolve_lbl, RED808_SUCCESS, 0);
            lv_obj_set_pos(seq_kanban_evolve_lbl, colX + colW - 30, 140);

            seq_kanban_evolve_slider = lv_slider_create(card);
            lv_obj_set_size(seq_kanban_evolve_slider, colW, 14);
            lv_obj_set_pos(seq_kanban_evolve_slider, colX, 158);
            lv_slider_set_range(seq_kanban_evolve_slider, 0, 100);
            lv_obj_set_style_bg_color(seq_kanban_evolve_slider, RED808_SURFACE, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(seq_kanban_evolve_slider, LV_OPA_COVER, LV_PART_MAIN);
            lv_obj_set_style_bg_color(seq_kanban_evolve_slider, RED808_SUCCESS, LV_PART_INDICATOR);
            lv_obj_set_style_bg_opa(seq_kanban_evolve_slider, LV_OPA_COVER, LV_PART_INDICATOR);
            lv_obj_set_style_bg_color(seq_kanban_evolve_slider, lv_color_white(), LV_PART_KNOB);
            lv_obj_set_style_bg_opa(seq_kanban_evolve_slider, LV_OPA_COVER, LV_PART_KNOB);
            lv_obj_set_style_pad_all(seq_kanban_evolve_slider, 6, LV_PART_KNOB);
            lv_obj_set_style_radius(seq_kanban_evolve_slider, LV_RADIUS_CIRCLE, LV_PART_KNOB);
            lv_obj_add_event_cb(seq_kanban_evolve_slider, seq_kanban_evolve_amount_cb,
                                LV_EVENT_VALUE_CHANGED, NULL);
            lv_obj_add_event_cb(seq_kanban_evolve_slider, seq_kanban_evolve_amount_cb,
                                LV_EVENT_RELEASED, NULL);
        } else if (m == KANBAN_VARIATION) {
            seq_kanban_variation_btn = lv_btn_create(card);
            lv_obj_set_size(seq_kanban_variation_btn, colW, 34);
            lv_obj_set_pos(seq_kanban_variation_btn, colX, 140);
            apply_control_button_style(seq_kanban_variation_btn, RED808_ACCENT2, false, 8);
            lv_obj_add_event_cb(seq_kanban_variation_btn, seq_kanban_variation_random_cb,
                                LV_EVENT_CLICKED, NULL);
            lv_obj_t* lbl = lv_label_create(seq_kanban_variation_btn);
            lv_label_set_text(lbl, "ALEATORIA\nAHORA");
            lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);
            lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_center(lbl);
        }
    }

    lv_obj_t* closeBtn = lv_btn_create(card);
    lv_obj_set_size(closeBtn, 1000 - 32, 32);
    lv_obj_set_pos(closeBtn, 0, 236);
    apply_control_button_style(closeBtn, RED808_BORDER, false, 10);
    lv_obj_add_event_cb(closeBtn, seq_kanban_modal_hide, LV_EVENT_CLICKED, NULL);
    lv_obj_t* closeLbl = lv_label_create(closeBtn);
    lv_label_set_text(closeLbl, "CERRAR");
    lv_obj_set_style_text_font(closeLbl, &lv_font_montserrat_14, 0);
    lv_obj_center(closeLbl);

    seq_kanban_refresh();
}

static void seq_variation_modal_show(lv_event_t* /*e*/) {
    if (seq_variation_modal) return;

    seq_variation_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(seq_variation_modal, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(seq_variation_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(seq_variation_modal, LV_OPA_80, 0);
    lv_obj_set_style_border_width(seq_variation_modal, 0, 0);
    lv_obj_set_style_pad_all(seq_variation_modal, 0, 0);
    lv_obj_clear_flag(seq_variation_modal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(seq_variation_modal, seq_variation_modal_hide,
                        LV_EVENT_CLICKED, NULL);

    lv_obj_t* card = lv_obj_create(seq_variation_modal);
    lv_obj_set_size(card, 950, 456);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_border_color(card, RED808_CYAN, 0);
    lv_obj_set_style_radius(card, 16, 0);
    lv_obj_set_style_pad_all(card, 0, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t* title = lv_label_create(card);
    lv_label_set_text(title, "VARIATIONS  /  PATTERN TRANSFORM");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title, RED808_CYAN, 0);
    lv_obj_set_pos(title, 24, 18);

    lv_obj_t* hint = lv_label_create(card);
    lv_label_set_text(hint,
        "Los botones de abajo transforman el patron actual. UNDO recupera la ultima version.");
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(hint, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(hint, 24, 50);

    lv_obj_t* close = lv_btn_create(card);
    lv_obj_set_size(close, 92, 38);
    lv_obj_set_pos(close, 834, 16);
    apply_control_button_style(close, RED808_WARNING, false, 9);
    lv_obj_add_event_cb(close, [](lv_event_t*) { seq_variation_modal_hide(); },
                        LV_EVENT_CLICKED, NULL);
    lv_obj_t* closeLabel = lv_label_create(close);
    lv_label_set_text(closeLabel, "CANCEL");
    lv_obj_set_style_text_font(closeLabel, &lv_font_montserrat_12, 0);
    lv_obj_center(closeLabel);

    constexpr int columns = 4;
    constexpr int buttonWidth = 216;
    constexpr int buttonHeight = 94;
    constexpr int gapX = 10;
    constexpr int gapY = 8;
    constexpr int gridY0 = 80;
    const bool canUndo = control_variation_can_undo();
    for (size_t index = 0;
         index < sizeof(SEQ_VARIATION_OPTIONS) / sizeof(SEQ_VARIATION_OPTIONS[0]);
         ++index) {
        const auto& option = SEQ_VARIATION_OPTIONS[index];
        const bool undoDisabled = option.id == SEQ_VAR_UNDO && !canUndo;
        const int column = static_cast<int>(index % columns);
        const int row = static_cast<int>(index / columns);
        lv_obj_t* button = lv_btn_create(card);
        lv_obj_set_size(button, buttonWidth, buttonHeight);
        lv_obj_set_pos(button, 24 + column * (buttonWidth + gapX),
                       gridY0 + row * (buttonHeight + gapY));
        apply_control_button_style(button,
            option.id == SEQ_VAR_UNDO ? RED808_SUCCESS : RED808_ACCENT2,
            false, 12);
        lv_obj_set_style_bg_color(button, RED808_SURFACE, 0);

        lv_obj_t* name = lv_label_create(button);
        lv_label_set_text(name, option.name);
        lv_obj_set_width(name, buttonWidth - 20);
        lv_obj_set_style_text_align(name, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(name, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(name, RED808_TEXT, 0);
        lv_obj_align(name, LV_ALIGN_CENTER, 0, -12);

        lv_obj_t* detail = lv_label_create(button);
        lv_label_set_text(detail, undoDisabled ? "Nada que restaurar" : option.detail);
        lv_obj_set_width(detail, buttonWidth - 18);
        lv_obj_set_style_text_align(detail, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(detail, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(detail, RED808_TEXT_DIM, 0);
        lv_obj_align(detail, LV_ALIGN_CENTER, 0, 13);

        if (undoDisabled) {
            lv_obj_add_state(button, LV_STATE_DISABLED);
            lv_obj_set_style_bg_opa(button, LV_OPA_30, LV_STATE_DISABLED);
            lv_obj_set_style_border_color(button, RED808_BORDER,
                                          LV_STATE_DISABLED);
        } else {
            lv_obj_add_event_cb(button, seq_variation_select_cb,
                                LV_EVENT_CLICKED,
                                reinterpret_cast<void*>(
                                    static_cast<uintptr_t>(option.id)));
        }
    }

    seq_song_controls_refresh();
}

static void seq_build4_cb(lv_event_t* /*e*/) {
    if (!p4.is_playing) {
        ui_show_toast("BUILD 4 necesita PLAY", RED808_WARNING);
        return;
    }
    control_send_build4();
    ui_show_toast("BUILD: 4 compases", RED808_WARNING);
}

static void seq_drop_cb(lv_event_t* /*e*/) {
    if (!p4.is_playing) {
        ui_show_toast("DROP necesita PLAY", RED808_WARNING);
        return;
    }
    control_send_drop();
    ui_show_toast("DROP armado para el proximo compas", RED808_ACCENT);
}

// Last-loaded MIDI info — kept so the info button in the sequencer header
// can re-open the summary modal on demand.
static char   seq_last_midi_name[64]   = "";
static int    seq_last_midi_slot       = 0;   // 1-based pattern slot
static int    seq_last_midi_steps      = 0;   // total active hits
static int    seq_last_midi_raw_len    = 0;   // 16/32/48/64
static float  seq_last_midi_bpm        = 0.0f;
static int    seq_last_midi_tracks     = 0;
static bool   seq_last_midi_valid      = false;

static void seq_apply_page_styles(void);
static void seq_copy_page_to_p4(int page);
static void show_midi_load_summary(const char* title, int slot,
                                   int steps, int raw_len, float bpm,
                                   int tracks_used);

static void seq_pattern_modal_hide(void) {
    if (!seq_pattern_modal) return;
    lv_obj_del(seq_pattern_modal);
    seq_pattern_modal = NULL;
    seq_pattern_modal_lbl = NULL;
    seq_pattern_modal_spin = NULL;
    seq_pattern_wait_pat = -1;
    seq_pattern_wait_ms = 0;
    seq_pattern_waiting = false;
}

static void seq_pattern_modal_show(int pattern) {
    seq_pattern_modal_hide();
    if (!scr_sequencer) return;

    seq_pattern_modal = lv_obj_create(scr_sequencer);
    lv_obj_set_size(seq_pattern_modal, 330, 120);
    lv_obj_align(seq_pattern_modal, LV_ALIGN_TOP_RIGHT, -16, 48);
    lv_obj_set_style_radius(seq_pattern_modal, 14, 0);
    lv_obj_set_style_bg_color(seq_pattern_modal, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_color(seq_pattern_modal, RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_dir(seq_pattern_modal, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(seq_pattern_modal, LV_OPA_90, 0);
    lv_obj_set_style_border_width(seq_pattern_modal, 2, 0);
    lv_obj_set_style_border_color(seq_pattern_modal, RED808_CYAN, 0);
    lv_obj_set_style_pad_all(seq_pattern_modal, 10, 0);
    lv_obj_clear_flag(seq_pattern_modal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(seq_pattern_modal, LV_OBJ_FLAG_CLICKABLE);

    seq_pattern_modal_spin = lv_spinner_create(seq_pattern_modal, 1000, 60);
    lv_obj_set_size(seq_pattern_modal_spin, 32, 32);
    lv_obj_align(seq_pattern_modal_spin, LV_ALIGN_LEFT_MID, 6, 0);
    lv_obj_set_style_arc_color(seq_pattern_modal_spin, RED808_CYAN, LV_PART_INDICATOR);

    lv_obj_t* t = lv_label_create(seq_pattern_modal);
    lv_label_set_text_fmt(t, "CARGANDO P%02d", pattern + 1);
    lv_obj_set_style_text_font(t, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(t, RED808_ACCENT, 0);
    lv_obj_align(t, LV_ALIGN_TOP_LEFT, 50, 8);

    seq_pattern_modal_lbl = lv_label_create(seq_pattern_modal);
    lv_label_set_text(seq_pattern_modal_lbl, "Leyendo banco local P4...");
    lv_obj_set_style_text_font(seq_pattern_modal_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(seq_pattern_modal_lbl, RED808_TEXT_DIM, 0);
    lv_obj_align(seq_pattern_modal_lbl, LV_ALIGN_TOP_LEFT, 50, 40);

    seq_pattern_wait_pat = pattern;
    seq_pattern_wait_ms = millis();
    seq_pattern_waiting = true;
}

static void seq_pattern_modal_mark_loaded(void) {
    if (!seq_pattern_modal) return;
    if (seq_pattern_modal_spin) {
        lv_obj_add_flag(seq_pattern_modal_spin, LV_OBJ_FLAG_HIDDEN);
    }
    if (seq_pattern_modal_lbl) {
        lv_label_set_text(seq_pattern_modal_lbl,
            ui_control_available() ? "P4 cargado / sync USB enviada"
                                   : "Pattern cargado en P4");
        lv_obj_set_style_text_color(seq_pattern_modal_lbl, RED808_SUCCESS, 0);
    }
    lv_obj_set_style_border_color(seq_pattern_modal, RED808_SUCCESS, 0);
    seq_pattern_waiting = false;
    seq_pattern_wait_ms = millis();
}

// Layout — landscape 1024×600 (LCD native, LVGL canvas)
static const int SEQ_RULER_Y    = 44;   // ruler starts below header
static const int SEQ_RULER_H    = 14;   // ruler height
static const int SEQ_GRID_Y     = 58;   // first track row Y
static const int SEQ_TRACK_H    = 32;   // track row height
static const int SEQ_TRACK_GAP  = 1;    // gap between rows
static const int SEQ_STRIPE_W   = 4;    // left color accent stripe
static const int SEQ_NAME_X     = 4;    // track name button X
static const int SEQ_NAME_W     = 54;   // track name button width
static const int SEQ_GRID_X     = 62;   // step grid start X
static const int SEQ_CELL_W     = 48;   // step cell width
static const int SEQ_BEAT_GAP   = 4;    // gap between beat groups (every 4 steps)
static const int SEQ_CELL_GAP   = 1;    // gap between cells within a beat
// Row order (left to right): FX, XFX (clear FX), SOLO.
static const int SEQ_FX_X       = 858;  // per-row instrument FX button X
static const int SEQ_FX_W       = 52;   // per-row instrument FX button width
static const int SEQ_FXCLR_X    = 914;  // per-row "clear FX" (XFX) button X
static const int SEQ_FXCLR_W    = 60;   // per-row "clear FX" button width
static const int SEQ_SOLO_X     = 978;  // solo button X
static const int SEQ_SOLO_W     = 32;   // solo button width
static const int SEQ_STATUS_Y   = 586;  // bottom status bar Y
static const int SEQ_STATUS_H   = 14;   // bottom status bar height

static void seq_step_cb(lv_event_t* e) {
    int data = (int)(intptr_t)lv_event_get_user_data(e);
    int track = (data >> 8) & 0xFF;
    int step  = data & 0xFF;
    if (track < 16 && step < 16) {
        bool next = !p4.steps[track][step];
        p4.steps[track][step] = next;
        // Always update the resident P4 pattern so SAVE works offline too;
        // the transport safely drops the packet when Daisy is unavailable.
        control_send_set_step(track, step, next);
        // Mirror into raw multi-bar grid so manual edits persist across pages.
        int idx = seq_page * 16 + step;
        if (idx < 64) seq_raw_grid[track][idx] = next;
        seq_set_pattern_dirty(true);
    }
}

// ── STEP PROBABILITY + PARAMETER LOCK popup (EVOLVE groundwork) ─────────
// Long-press a step cell to tune how often it fires and whether it carries
// a one-hit flourish. Both already exist end-to-end in the engine — the
// probability dice roll and the three param locks (cutoff/reverb/volume)
// are applied sample-accurately at trigger time on DaisyPod3
// (DsqFireStep/DsqTriggerTrackNow), the same mechanism used for years to
// author the factory patterns. This popup is the first UI that lets the
// user reach any of it on their own patterns.
//
// Locks use one fixed "flourish" value each rather than a free value
// picker (cutoff opens bright, reverb tails long, volume accents hard) —
// enough expressive range for now without a second layer of sliders
// crammed into an already-small popup; a finer editor can follow later
// if it turns out to be wanted.
static lv_obj_t* seq_step_prob_modal = NULL;
static lv_obj_t* seq_step_prob_title = NULL;
static lv_obj_t* seq_step_prob_btns[5] = {};
static lv_obj_t* seq_step_lock_btns[3] = {};  // 0=cutoff, 1=reverb, 2=volume
static int8_t    seq_step_prob_track = -1;
static int8_t    seq_step_prob_step = -1;
static const uint8_t  SEQ_STEP_PROB_OPTIONS[5] = {100, 75, 50, 25, 10};
static const char* const SEQ_STEP_LOCK_NAMES[3] = {"CUTOFF", "REVERB", "VOLUMEN"};
static const int SEQ_STEP_LOCK_CUTOFF_HZ = 6000;
static const int SEQ_STEP_LOCK_REVERB_PCT = 80;
static const int SEQ_STEP_LOCK_VOLUME = 127;

// True if the step carries any customization — probability<100 or a lock —
// so the grid's single corner dot can flag "long-press to see what's set
// here" without needing one indicator per customization type.
static bool seq_step_is_customized(int track, int step) {
    if (control_get_step_probability(track, step) < 100) return true;
    StepParamLock lock;
    control_get_step_param_lock(track, step, lock);
    return lock.cutoffEnabled || lock.reverbEnabled || lock.volumeEnabled;
}

static void seq_step_prob_dot_refresh(int track, int step) {
    if (track < 0 || track >= 16 || step < 0 || step >= 16) return;
    lv_obj_t* dot = seq_step_prob_dot[track][step];
    if (!dot) return;
    if (seq_step_is_customized(track, step))
        lv_obj_clear_flag(dot, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_add_flag(dot, LV_OBJ_FLAG_HIDDEN);
}

static void seq_step_prob_modal_refresh(void) {
    if (!seq_step_prob_modal || seq_step_prob_track < 0) return;
    if (seq_step_prob_title)
        lv_label_set_text_fmt(seq_step_prob_title, "%s - STEP %02d",
                              trackNames[seq_step_prob_track], seq_step_prob_step + 1);
    const uint8_t current = control_get_step_probability(seq_step_prob_track, seq_step_prob_step);
    for (int i = 0; i < 5; i++) {
        lv_obj_t* btn = seq_step_prob_btns[i];
        if (!btn) continue;
        const bool sel = SEQ_STEP_PROB_OPTIONS[i] == current;
        apply_control_button_style(btn, sel ? RED808_WARNING : RED808_BORDER, false, 8);
    }
    StepParamLock lock;
    control_get_step_param_lock(seq_step_prob_track, seq_step_prob_step, lock);
    const bool enabled[3] = {lock.cutoffEnabled, lock.reverbEnabled, lock.volumeEnabled};
    for (int i = 0; i < 3; i++) {
        lv_obj_t* btn = seq_step_lock_btns[i];
        if (!btn) continue;
        apply_control_button_style(btn, enabled[i] ? RED808_CYAN : RED808_BORDER, false, 8);
        lv_obj_t* lbl = lv_obj_get_child(btn, 0);
        if (lbl) lv_label_set_text_fmt(lbl, "%s: %s", SEQ_STEP_LOCK_NAMES[i],
                                       enabled[i] ? "ON" : "OFF");
    }
}

static void seq_step_prob_modal_hide(lv_event_t* e = NULL) {
    if (e && lv_event_get_target(e) != lv_event_get_current_target(e)) return;
    if (seq_step_prob_modal) lv_obj_del(seq_step_prob_modal);
    seq_step_prob_modal = NULL;
    seq_step_prob_title = NULL;
    for (int i = 0; i < 5; i++) seq_step_prob_btns[i] = NULL;
    for (int i = 0; i < 3; i++) seq_step_lock_btns[i] = NULL;
    seq_step_prob_track = -1;
    seq_step_prob_step = -1;
}

static void seq_step_prob_btn_cb(lv_event_t* e) {
    if (seq_step_prob_track < 0 || seq_step_prob_step < 0) return;
    const int probability = (int)(intptr_t)lv_event_get_user_data(e);
    control_send_set_step_probability(seq_step_prob_track, seq_step_prob_step, probability);
    seq_step_prob_dot_refresh(seq_step_prob_track, seq_step_prob_step);
    seq_step_prob_modal_refresh();
}

static void seq_step_lock_btn_cb(lv_event_t* e) {
    if (seq_step_prob_track < 0 || seq_step_prob_step < 0) return;
    const int which = (int)(intptr_t)lv_event_get_user_data(e);
    StepParamLock lock;
    control_get_step_param_lock(seq_step_prob_track, seq_step_prob_step, lock);
    switch (which) {
        case 0:
            control_send_set_step_cutoff_lock(seq_step_prob_track, seq_step_prob_step,
                !lock.cutoffEnabled, SEQ_STEP_LOCK_CUTOFF_HZ);
            break;
        case 1:
            control_send_set_step_reverb_lock(seq_step_prob_track, seq_step_prob_step,
                !lock.reverbEnabled, SEQ_STEP_LOCK_REVERB_PCT);
            break;
        case 2:
            control_send_set_step_volume_lock(seq_step_prob_track, seq_step_prob_step,
                !lock.volumeEnabled, SEQ_STEP_LOCK_VOLUME);
            break;
        default: return;
    }
    seq_step_prob_dot_refresh(seq_step_prob_track, seq_step_prob_step);
    seq_step_prob_modal_refresh();
}

static void seq_step_long_press_cb(lv_event_t* e) {
    if (seq_step_prob_modal) return;
    const int data = (int)(intptr_t)lv_event_get_user_data(e);
    const int track = (data >> 8) & 0xFF;
    const int step  = data & 0xFF;
    if (track >= 16 || step >= 16) return;
    seq_step_prob_track = (int8_t)track;
    seq_step_prob_step  = (int8_t)step;

    seq_step_prob_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(seq_step_prob_modal, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(seq_step_prob_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(seq_step_prob_modal, LV_OPA_80, 0);
    lv_obj_set_style_border_width(seq_step_prob_modal, 0, 0);
    lv_obj_set_style_pad_all(seq_step_prob_modal, 0, 0);
    lv_obj_clear_flag(seq_step_prob_modal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(seq_step_prob_modal, seq_step_prob_modal_hide, LV_EVENT_CLICKED, NULL);

    lv_obj_t* card = lv_obj_create(seq_step_prob_modal);
    lv_obj_set_size(card, 420, 352);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_border_color(card, RED808_WARNING, 0);
    lv_obj_set_style_radius(card, 16, 0);
    lv_obj_set_style_pad_all(card, 16, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);

    seq_step_prob_title = lv_label_create(card);
    lv_obj_set_style_text_font(seq_step_prob_title, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(seq_step_prob_title, RED808_WARNING, 0);
    lv_obj_set_pos(seq_step_prob_title, 0, 0);

    lv_obj_t* probLbl = lv_label_create(card);
    lv_label_set_text(probLbl, "PROBABILIDAD:");
    lv_obj_set_style_text_font(probLbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(probLbl, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(probLbl, 0, 26);

    for (int i = 0; i < 5; i++) {
        lv_obj_t* btn = lv_btn_create(card);
        seq_step_prob_btns[i] = btn;
        lv_obj_set_size(btn, 70, 48);
        lv_obj_set_pos(btn, i * (70 + 8), 44);
        lv_obj_add_event_cb(btn, seq_step_prob_btn_cb, LV_EVENT_CLICKED,
                            (void*)(intptr_t)SEQ_STEP_PROB_OPTIONS[i]);
        lv_obj_t* lbl = lv_label_create(btn);
        lv_label_set_text_fmt(lbl, "%d%%", SEQ_STEP_PROB_OPTIONS[i]);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
        lv_obj_center(lbl);
    }

    lv_obj_t* lockLbl = lv_label_create(card);
    lv_label_set_text(lockLbl, "PARAMETER LOCK (flourish al disparar este step):");
    lv_obj_set_style_text_font(lockLbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lockLbl, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(lockLbl, 0, 104);

    for (int i = 0; i < 3; i++) {
        lv_obj_t* btn = lv_btn_create(card);
        seq_step_lock_btns[i] = btn;
        lv_obj_set_size(btn, 388, 40);
        lv_obj_set_pos(btn, 0, 122 + i * (40 + 8));
        lv_obj_add_event_cb(btn, seq_step_lock_btn_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
        lv_obj_t* lbl = lv_label_create(btn);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
        lv_obj_center(lbl);
    }

    lv_obj_t* closeBtn = lv_btn_create(card);
    lv_obj_set_size(closeBtn, 388, 40);
    lv_obj_set_pos(closeBtn, 0, 122 + 3 * (40 + 8) + 4);
    apply_control_button_style(closeBtn, RED808_BORDER, false, 10);
    lv_obj_add_event_cb(closeBtn, seq_step_prob_modal_hide, LV_EVENT_CLICKED, NULL);
    lv_obj_t* closeLbl = lv_label_create(closeBtn);
    lv_label_set_text(closeLbl, "CERRAR");
    lv_obj_set_style_text_font(closeLbl, &lv_font_montserrat_14, 0);
    lv_obj_center(closeLbl);

    seq_step_prob_modal_refresh();
}

static void seq_mute_cb(lv_event_t* e) {
    int track = (int)(intptr_t)lv_event_get_user_data(e);
    if (track < 16) {
        // Debounce per-track and global window: 5 LVGL indevs are registered
        // for multi-touch and a single tap can fire CLICKED twice from
        // neighbour slots.
        static uint32_t last_ms[16] = {};
        static uint32_t last_any_ms = 0;
        uint32_t now = millis();
        if (now - last_ms[track] < MUTE_DEBOUNCE_TRACK_MS) {
            P4_LOG_PRINTF("[MUTE] DROPPED dup t=%d dt=%lu\n", track, (unsigned long)(now - last_ms[track]));
            return;
        }
        if (now - last_any_ms < MUTE_DEBOUNCE_GLOBAL_MS) {
            P4_LOG_PRINTF("[MUTE] DROPPED global t=%d dt=%lu\n", track, (unsigned long)(now - last_any_ms));
            return;
        }
        last_ms[track] = now;
        last_any_ms = now;

        bool next = !p4.track_muted[track];
        p4.track_muted[track] = next;
        bool soloCleared = false;
        if (next && p4.track_solo[track]) {
            p4.track_solo[track] = false;
            soloCleared = true;
        }
        char tb[48];
        snprintf(tb, sizeof(tb), "MUTE T%d %s",
                 track + 1, next ? "ON" : "OFF");
        ui_show_toast(tb, next ? RED808_ERROR : RED808_SUCCESS);
        if (ui_control_available()) {
            enqueue_mute_control((uint8_t)track, next);
            if (soloCleared) {
                uint16_t soloMask = 0;
                for (int other = 0; other < 16; ++other)
                    if (p4.track_solo[other]) soloMask |= (uint16_t)(1u << other);
                enqueue_solo_mask_control(soloMask);
            }
        }
    }
}

static void seq_solo_cb(lv_event_t* e) {
    int track = (int)(intptr_t)lv_event_get_user_data(e);
    if (track >= 16) return;
    static uint32_t last_ms[16] = {};
    static uint32_t last_any_ms = 0;
    uint32_t nowDbg = millis();
    if (nowDbg - last_ms[track] < SOLO_DEBOUNCE_TRACK_MS) {
        P4_LOG_PRINTF("[SOLO] DROPPED dup t=%d dt=%lu\n", track, (unsigned long)(nowDbg - last_ms[track]));
        return;
    }
    if (nowDbg - last_any_ms < SOLO_DEBOUNCE_GLOBAL_MS) {
        P4_LOG_PRINTF("[SOLO] DROPPED global t=%d dt=%lu\n", track, (unsigned long)(nowDbg - last_any_ms));
        return;
    }
    last_ms[track] = nowDbg;
    last_any_ms = nowDbg;
    bool wasSolo = p4.track_solo[track];

    // Visible feedback so we can confirm the callback fires.
    char toastBuf[64];
    snprintf(toastBuf, sizeof(toastBuf), "SOLO T%d %s",
             track + 1, wasSolo ? "OFF" : "ON");
    ui_show_toast(toastBuf, wasSolo ? RED808_BORDER : RED808_ACCENT);

    const uint16_t soloMask = wasSolo ? 0u : (uint16_t)(1u << track);
    for (int t = 0; t < 16; ++t)
        p4.track_solo[t] = (soloMask & (1u << t)) != 0;
    const bool muteCleared = !wasSolo && p4.track_muted[track];
    if (muteCleared) p4.track_muted[track] = false;
    if (ui_control_available()) {
        enqueue_solo_mask_control(soloMask);
        if (muteCleared) enqueue_mute_control((uint8_t)track, false);
    }
}

// ── Pagination helpers ─────────────────────────────────────────────────────
// Copy the 16 steps of the given raw-grid page into p4.steps, stage the
// resulting bar to the Master, and sync to S3. Used when the user taps P1..P4.
static void seq_copy_page_to_p4(int page) {
    if (page < 0 || page > 3) return;
    int base = page * 16;
    int num_pages = (seq_raw_len + 15) / 16;
    if (num_pages < 1) num_pages = 1;
    if (page >= num_pages) return;
    for (int t = 0; t < 16; t++)
        for (int s = 0; s < 16; s++)
            p4.steps[t][s] = seq_raw_grid[t][base + s];
    seq_page = page;
    // Page changes replace a complete 16-step snapshot. Pause while uploading
    // it, then restore the prior transport state.
    const bool resume = p4.is_playing && ui_control_available();
    if (resume) control_send_stop();
    local_stage_pattern((uint8_t)p4.current_pattern, p4.steps);
    if (resume) control_send_start();
    seq_set_pattern_dirty(true);
}

// Refresh page button highlighting + enable/disable based on seq_raw_len.
static void seq_apply_page_styles(void) {
    int num_pages = (seq_raw_len + 15) / 16;
    if (num_pages < 1) num_pages = 1;
    for (int p = 0; p < 4; p++) {
        if (!seq_page_btns[p]) continue;
        bool enabled = (p < num_pages);
        bool active  = enabled && (p == seq_page);
        lv_obj_set_style_bg_color(seq_page_btns[p],
            active ? RED808_ACCENT : (enabled ? RED808_SURFACE : lv_color_hex(0x080808)), 0);
        lv_obj_set_style_bg_opa(seq_page_btns[p],
            enabled ? LV_OPA_COVER : LV_OPA_30, 0);
        lv_obj_set_style_border_color(seq_page_btns[p],
            active ? RED808_CYAN : RED808_BORDER, 0);
        if (seq_page_lbls[p]) {
            lv_obj_set_style_text_color(seq_page_lbls[p],
                active ? lv_color_white() : (enabled ? RED808_TEXT : RED808_TEXT_DIM), 0);
        }
        if (enabled) lv_obj_clear_state(seq_page_btns[p], LV_STATE_DISABLED);
        else         lv_obj_add_state  (seq_page_btns[p], LV_STATE_DISABLED);
    }
}

static void seq_page_cb(lv_event_t* e) {
    int page = (int)(intptr_t)lv_event_get_user_data(e);
    int num_pages = (seq_raw_len + 15) / 16;
    if (page < 0 || page >= num_pages) return;
    if (page == seq_page) return;
    seq_copy_page_to_p4(page);
    seq_apply_page_styles();
}

// Called by MEM-MIDI loader after filling seq_raw_grid.
static void seq_install_raw_and_show_page0(int raw_len) {
    seq_raw_len = (raw_len < 16) ? 16 : (raw_len > 64 ? 64 : raw_len);
    seq_page = 0;
    seq_copy_page_to_p4(0);
    seq_apply_page_styles();
}

// Probability/locks aren't part of p4.steps/prev_cell_key, so they need
// their own explicit refresh here — deliberately NOT polled per-frame (see
// the comment at seq_step_prob_dot's creation). Public so control_api.cpp
// can call it too, after an EVOLVE pass touches probabilities directly.
void ui_sequencer_refresh_all_step_dots(void) {
    for (int t = 0; t < 16; t++)
        for (int s = 0; s < 16; s++)
            seq_step_prob_dot_refresh(t, s);
}

void ui_sequencer_sync_from_current_pattern(void) {
    seq_raw_len = 16;
    seq_page = 0;
    for (int t = 0; t < 16; ++t) {
        for (int s = 0; s < 16; ++s) seq_raw_grid[t][s] = p4.steps[t][s];
        for (int s = 16; s < 64; ++s) seq_raw_grid[t][s] = false;
    }
    seq_force_refresh_cells = true;
    seq_page_styles_dirty = true;
    seq_pattern_dirty = false;
    // A different pattern slot became authoritative — including the jumps
    // RANDOM SONG itself makes. Keep the SONG button's indicator in sync.
    seq_song_btn_refresh();
    ui_sequencer_refresh_all_step_dots();
}

void ui_sequencer_load_external_pattern(const bool steps[16][64], int raw_len) {
    seq_raw_len = (raw_len < 16) ? 16 : (raw_len > 64 ? 64 : raw_len);
    seq_page = 0;
    for (int t = 0; t < 16; t++) {
        for (int s = 0; s < 64; s++) {
            seq_raw_grid[t][s] = (s < seq_raw_len) ? steps[t][s] : false;
        }
        for (int s = 0; s < 16; s++) {
            p4.steps[t][s] = seq_raw_grid[t][s];
        }
    }
    seq_force_refresh_cells = true;  // force full cell repaint — prev_cell_key may be stale
    seq_page_styles_dirty = true;
    seq_pattern_dirty = true;
    ui_sequencer_refresh_all_step_dots();
}

static void create_sequencer_screen(void) {
    scr_sequencer = lv_obj_create(NULL);
    apply_screen_theme_bg(scr_sequencer);
    lv_obj_clear_flag(scr_sequencer, LV_OBJ_FLAG_SCROLLABLE);
    {
        lv_obj_t* backBtn = ui_create_header(scr_sequencer);
        // MIDI (MPD218) activity badge, poking out of the back button's
        // corner — no dedicated space free on this screen's packed header row.
        s_seq_midi_badge = ui_create_midi_badge(backBtn, 38, -4);
    }

    // ── Live header: transport, bar queue, performance actions and mix ──
    {
        const int HY = 6;   // top padding
        const int HH = 36;  // button height
        int hx = 60;

        // PLAY / PAUSE
        seq_hdr_play_btn = lv_btn_create(scr_sequencer);
        lv_obj_set_size(seq_hdr_play_btn, 70, HH);
        lv_obj_set_pos(seq_hdr_play_btn, hx, HY);
        apply_control_button_style(seq_hdr_play_btn,
            p4.is_playing ? RED808_CYAN : RED808_ACCENT2, true, 8);
        lv_obj_set_style_bg_color(seq_hdr_play_btn,
            p4.is_playing ? RED808_SUCCESS : RED808_ACCENT, 0);
        lv_obj_add_event_cb(seq_hdr_play_btn, header_play_cb, LV_EVENT_CLICKED, NULL);
        pod_register_owner_badge(seq_hdr_play_btn, POD_FUNC_PLAY_TOGGLE);
        seq_hdr_play_lbl = lv_label_create(seq_hdr_play_btn);
        lv_label_set_text(seq_hdr_play_lbl, p4.is_playing ? "PAUSE" : "PLAY");
        lv_obj_set_style_text_font(seq_hdr_play_lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(seq_hdr_play_lbl, lv_color_white(), 0);
        lv_obj_center(seq_hdr_play_lbl);
        hx += 70 + 4;

        // PATTERN -
        lv_obj_t* pm = lv_btn_create(scr_sequencer);
        lv_obj_set_size(pm, 30, HH);
        lv_obj_set_pos(pm, hx, HY);
        apply_control_button_style(pm, RED808_WARNING, false, 8);
        lv_obj_add_event_cb(pm, header_pattern_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
        pod_register_owner_badge(pm, POD_FUNC_PATTERN_PREV);
        lv_obj_t* pml = lv_label_create(pm);
        lv_label_set_text(pml, LV_SYMBOL_MINUS);
        lv_obj_set_style_text_font(pml, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(pml, RED808_TEXT, 0);
        lv_obj_center(pml);
        hx += 30 + 3;

        // PATTERN label
        seq_hdr_pat_lbl = lv_label_create(scr_sequencer);
        lv_obj_set_size(seq_hdr_pat_lbl, 118, 20);
        lv_obj_set_pos(seq_hdr_pat_lbl, hx, HY);
        lv_label_set_text_fmt(seq_hdr_pat_lbl, "P%02d", p4.current_pattern + 1);
        lv_obj_set_style_text_font(seq_hdr_pat_lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(seq_hdr_pat_lbl, RED808_ACCENT, 0);
        lv_obj_set_style_text_align(seq_hdr_pat_lbl, LV_TEXT_ALIGN_CENTER, 0);
        seq_hdr_name_lbl = lv_label_create(scr_sequencer);
        lv_obj_set_size(seq_hdr_name_lbl, 118, 15);
        lv_obj_set_pos(seq_hdr_name_lbl, hx, HY + 19);
        lv_label_set_text(seq_hdr_name_lbl, seq_pattern_name(p4.current_pattern));
        lv_label_set_long_mode(seq_hdr_name_lbl, LV_LABEL_LONG_DOT);
        lv_obj_set_style_text_font(seq_hdr_name_lbl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(seq_hdr_name_lbl, RED808_TEXT_DIM, 0);
        lv_obj_set_style_text_align(seq_hdr_name_lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_add_flag(seq_hdr_name_lbl, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(seq_hdr_name_lbl,
            [](lv_event_t*) { seq_pattern_list_show(); }, LV_EVENT_CLICKED, NULL);
        hx += 118 + 3;

        // PATTERN +
        lv_obj_t* pp = lv_btn_create(scr_sequencer);
        lv_obj_set_size(pp, 30, HH);
        lv_obj_set_pos(pp, hx, HY);
        apply_control_button_style(pp, RED808_WARNING, false, 8);
        lv_obj_add_event_cb(pp, header_pattern_cb, LV_EVENT_CLICKED, (void*)(intptr_t)1);
        pod_register_owner_badge(pp, POD_FUNC_PATTERN_NEXT);
        lv_obj_t* ppl = lv_label_create(pp);
        lv_label_set_text(ppl, LV_SYMBOL_PLUS);
        lv_obj_set_style_text_font(ppl, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(ppl, RED808_TEXT, 0);
        lv_obj_center(ppl);
        hx += 30 + 4;

        auto makeHeaderButton = [&](int width, const char* text, lv_color_t color,
                                    lv_event_cb_t cb) -> lv_obj_t* {
            lv_obj_t* btn = lv_btn_create(scr_sequencer);
            lv_obj_set_size(btn, width, HH);
            lv_obj_set_pos(btn, hx, HY);
            apply_control_button_style(btn, color, false, 7);
            lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);
            lv_obj_t* label = lv_label_create(btn);
            lv_label_set_text(label, text);
            lv_obj_set_style_text_font(label, &lv_font_montserrat_12, 0);
            lv_obj_set_style_text_color(label, lv_color_white(), 0);
            lv_obj_center(label);
            hx += width + 4;
            return btn;
        };

        seq_hdr_queue_btn = makeHeaderButton(68,
            seq_quantize_enabled ? "Q 1 BAR" : "Q OFF", RED808_CYAN,
            seq_quantize_toggle_cb);
        seq_hdr_queue_lbl = lv_obj_get_child(seq_hdr_queue_btn, 0);
        if (seq_quantize_enabled && seq_queued_pattern >= 0)
            lv_label_set_text_fmt(seq_hdr_queue_lbl, "Q>P%02d", seq_queued_pattern + 1);
        if (!seq_quantize_enabled) {
            lv_obj_set_style_bg_color(seq_hdr_queue_btn, RED808_ERROR, 0);
            lv_obj_set_style_border_color(seq_hdr_queue_btn, RED808_WARNING, 0);
        }
        seq_hdr_var_btn = makeHeaderButton(64, "VAR", RED808_CYAN,
                                           seq_variation_modal_show);

        seq_hdr_song_btn = makeHeaderButton(64, "SONG", RED808_CYAN,
                                            seq_song_modal_show);
        seq_song_stop_badge = ui_create_auto_stop_badge(seq_hdr_song_btn, seq_song_stop_badge_cb);
        seq_song_btn_refresh();

        seq_hdr_evolve_btn = makeHeaderButton(72, "EVOLVE", RED808_ACCENT2,
                                              seq_evolve_modal_show);
        seq_evolve_stop_badge = ui_create_auto_stop_badge(seq_hdr_evolve_btn, seq_evolve_stop_badge_cb);
        seq_evolve_btn_refresh();

        makeHeaderButton(68, "GROUPS", RED808_BORDER, seq_groups_modal_show);

        makeHeaderButton(52, "LIST", RED808_ACCENT,
            [](lv_event_t*) { seq_pattern_list_show(); });
        makeHeaderButton(56, LV_SYMBOL_DOWNLOAD " MIDI", RED808_CYAN,
            [](lv_event_t*) { seq_open_midi_library(); });
        seq_hdr_save_btn = makeHeaderButton(68, LV_SYMBOL_SAVE " SAVE",
            RED808_SUCCESS,
            [](lv_event_t*) { seq_pattern_list_show_mode(true); });
        seq_hdr_save_lbl = lv_obj_get_child(seq_hdr_save_btn, 0);
        seq_set_pattern_dirty(seq_pattern_dirty);

        seq_hdr_kanban_btn = makeHeaderButton(64, "AUTO", RED808_CYAN, seq_kanban_modal_show);
    }

    // ── Precompute step X positions ──
    {
        int xOff = SEQ_GRID_X;
        for (int s = 0; s < 16; s++) {
            if (s > 0 && (s % 4) == 0) xOff += SEQ_BEAT_GAP;
            seq_step_x[s] = xOff;
            xOff += SEQ_CELL_W + SEQ_CELL_GAP;
        }
    }
    const int grid_bottom = SEQ_GRID_Y + 16 * (SEQ_TRACK_H + SEQ_TRACK_GAP) - SEQ_TRACK_GAP;

    // ── Beat group shading panels (drawn first = behind everything) ──
    // 4 groups × 4 steps each, alternating subtle bg
    const uint32_t beat_shade[2] = { 0x0B0B0B, 0x0F0F0F };
    for (int b = 0; b < 4; b++) {
        int bx = seq_step_x[b * 4];
        int bw = seq_step_x[b * 4 + 3] + SEQ_CELL_W - bx + 2;
        seq_beat_bg[b] = lv_obj_create(scr_sequencer);
        lv_obj_set_pos(seq_beat_bg[b], bx - 1, SEQ_RULER_Y);
        lv_obj_set_size(seq_beat_bg[b], bw, grid_bottom - SEQ_RULER_Y + SEQ_STATUS_H);
        lv_obj_set_style_bg_color(seq_beat_bg[b], lv_color_hex(beat_shade[b & 1]), 0);
        lv_obj_set_style_bg_opa(seq_beat_bg[b], LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(seq_beat_bg[b], 0, 0);
        lv_obj_set_style_radius(seq_beat_bg[b], 0, 0);
        lv_obj_clear_flag(seq_beat_bg[b], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(seq_beat_bg[b], LV_OBJ_FLAG_CLICKABLE);
    }

    // ── Beat separator lines between groups ──
    for (int b = 1; b < 4; b++) {
        lv_obj_t* sep = lv_obj_create(scr_sequencer);
        lv_obj_set_pos(sep, seq_step_x[b * 4] - 3, SEQ_RULER_Y);
        lv_obj_set_size(sep, 1, grid_bottom - SEQ_RULER_Y);
        lv_obj_set_style_bg_color(sep, lv_color_hex(0x2A2A2A), 0);
        lv_obj_set_style_bg_opa(sep, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(sep, 0, 0);
        lv_obj_set_style_radius(sep, 0, 0);
        lv_obj_clear_flag(sep, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(sep, LV_OBJ_FLAG_CLICKABLE);
    }

    // ── Step ruler: beat numbers 1–16 ──
    for (int s = 0; s < 16; s++) {
        bool is_beat_start = (s % 4 == 0);
        seq_ruler_labels[s] = lv_label_create(scr_sequencer);
        lv_label_set_text_fmt(seq_ruler_labels[s], "%d", s + 1);
        lv_obj_set_style_text_font(seq_ruler_labels[s],
            is_beat_start ? &lv_font_montserrat_14 : &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(seq_ruler_labels[s],
            is_beat_start ? RED808_ACCENT : RED808_TEXT_DIM, 0);
        lv_obj_set_width(seq_ruler_labels[s], SEQ_CELL_W);
        lv_obj_set_pos(seq_ruler_labels[s], seq_step_x[s], SEQ_RULER_Y);
        lv_obj_set_style_text_align(seq_ruler_labels[s], LV_TEXT_ALIGN_CENTER, 0);
    }

    // ── Ruler bottom separator line ──
    {
        lv_obj_t* rl = lv_obj_create(scr_sequencer);
        lv_obj_set_pos(rl, SEQ_GRID_X - 2, SEQ_GRID_Y - 2);
        lv_obj_set_size(rl, seq_step_x[15] + SEQ_CELL_W - SEQ_GRID_X + 4, 1);
        lv_obj_set_style_bg_color(rl, lv_color_hex(0x2A2A2A), 0);
        lv_obj_set_style_bg_opa(rl, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(rl, 0, 0);
        lv_obj_set_style_radius(rl, 0, 0);
        lv_obj_clear_flag(rl, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(rl, LV_OBJ_FLAG_CLICKABLE);
    }

    // ── Track rows ──
    for (int t = 0; t < 16; t++) {
        int rowY = SEQ_GRID_Y + t * (SEQ_TRACK_H + SEQ_TRACK_GAP);
        lv_color_t tc = lv_color_hex(theme_presets[ui_theme_index()].track_colors[t]);

        // Alternating row background for legibility
        lv_obj_t* row_bg = lv_obj_create(scr_sequencer);
        lv_obj_set_pos(row_bg, 0, rowY);
        lv_obj_set_size(row_bg, LCD_H_RES, SEQ_TRACK_H);
        lv_obj_set_style_bg_color(row_bg, lv_color_hex(t & 1 ? 0x0E0E0E : 0x0A0A0A), 0);
        lv_obj_set_style_bg_opa(row_bg, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(row_bg, 0, 0);
        lv_obj_set_style_radius(row_bg, 0, 0);
        lv_obj_clear_flag(row_bg, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(row_bg, LV_OBJ_FLAG_CLICKABLE);

        // Left color accent stripe
        lv_obj_t* stripe = lv_obj_create(scr_sequencer);
        lv_obj_set_pos(stripe, 0, rowY);
        lv_obj_set_size(stripe, SEQ_STRIPE_W, SEQ_TRACK_H);
        lv_obj_set_style_bg_color(stripe, tc, 0);
        lv_obj_set_style_bg_opa(stripe, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(stripe, 0, 0);
        lv_obj_set_style_radius(stripe, 0, 0);
        lv_obj_clear_flag(stripe, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(stripe, LV_OBJ_FLAG_CLICKABLE);

        // Track name + number button (tap = mute toggle)
        seq_mute_btns[t] = lv_btn_create(scr_sequencer);
        lv_obj_set_size(seq_mute_btns[t], SEQ_NAME_W, SEQ_TRACK_H);
        lv_obj_set_pos(seq_mute_btns[t], SEQ_NAME_X, rowY);
        apply_control_button_style(seq_mute_btns[t], tc, false, 4);
        lv_obj_set_style_bg_opa(seq_mute_btns[t], (lv_opa_t)140, 0);
        lv_obj_set_style_border_opa(seq_mute_btns[t], LV_OPA_60, 0);
        lv_obj_set_style_pad_all(seq_mute_btns[t], 0, 0);
        lv_obj_add_event_cb(seq_mute_btns[t], seq_mute_cb, LV_EVENT_CLICKED, (void*)(intptr_t)t);

        // Track label: "01\nBD" — number above, name below
        seq_track_labels[t] = lv_label_create(seq_mute_btns[t]);
        seq_refresh_track_label((uint8_t)t);
        lv_obj_set_style_text_font(seq_track_labels[t], &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(seq_track_labels[t], tc, 0);
        lv_obj_set_style_text_align(seq_track_labels[t], LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_line_space(seq_track_labels[t], 1, 0);
        lv_obj_center(seq_track_labels[t]);

        // ── 16 step cells ──
        for (int s = 0; s < 16; s++) {
            bool act = p4.steps[t][s];
            seq_step_btns[t][s] = lv_obj_create(scr_sequencer);
            lv_obj_set_size(seq_step_btns[t][s], SEQ_CELL_W, SEQ_TRACK_H);
            lv_obj_set_pos(seq_step_btns[t][s], seq_step_x[s], rowY);
            lv_obj_set_style_radius(seq_step_btns[t][s], 4, 0);
            lv_obj_set_style_bg_color(seq_step_btns[t][s], act ? tc : RED808_SURFACE, 0);
            lv_obj_set_style_bg_opa(seq_step_btns[t][s], act ? LV_OPA_80 : LV_OPA_40, 0);
            lv_obj_set_style_border_width(seq_step_btns[t][s], 1, 0);
            lv_obj_set_style_border_color(seq_step_btns[t][s], act ? tc : lv_color_hex(0x1E1E1E), 0);
            lv_obj_set_style_shadow_width(seq_step_btns[t][s], act ? 8 : 0, 0);
            lv_obj_set_style_shadow_color(seq_step_btns[t][s], tc, 0);
            lv_obj_set_style_shadow_opa(seq_step_btns[t][s], LV_OPA_60, 0);
            lv_obj_clear_flag(seq_step_btns[t][s], LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_flag(seq_step_btns[t][s], LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(seq_step_btns[t][s], seq_step_cb, LV_EVENT_CLICKED,
                                (void*)(intptr_t)((t << 8) | s));

            // Bottom accent line inside active cells (velocity-like feel).
            // Created on every cell and shown/hidden in the update path —
            // creating it only for cells active at construction left the
            // accents stale after edits or pattern loads.
            {
                lv_obj_t* accent = lv_obj_create(seq_step_btns[t][s]);
                seq_step_accents[t][s] = accent;
                lv_obj_set_size(accent, SEQ_CELL_W - 8, 3);
                lv_obj_align(accent, LV_ALIGN_BOTTOM_MID, 0, -3);
                lv_obj_set_style_bg_color(accent, lv_color_white(), 0);
                lv_obj_set_style_bg_opa(accent, LV_OPA_40, 0);
                lv_obj_set_style_radius(accent, 2, 0);
                lv_obj_set_style_border_width(accent, 0, 0);
                lv_obj_clear_flag(accent, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_clear_flag(accent, LV_OBJ_FLAG_CLICKABLE);
                if (!act) lv_obj_add_flag(accent, LV_OBJ_FLAG_HIDDEN);
            }

            // EVOLVE groundwork: small corner dot flags a step with a
            // custom probability and/or an active parameter lock
            // (long-press to edit/inspect). Refreshed here at construction
            // and again whenever a pattern becomes authoritative
            // (ui_sequencer_sync_from_current_pattern) — never polled
            // per-frame, since it needs Sequencer getter calls.
            {
                lv_obj_t* dot = lv_obj_create(seq_step_btns[t][s]);
                seq_step_prob_dot[t][s] = dot;
                lv_obj_set_size(dot, 8, 8);
                lv_obj_align(dot, LV_ALIGN_TOP_RIGHT, 1, 1);
                lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
                lv_obj_set_style_bg_color(dot, RED808_WARNING, 0);
                lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
                lv_obj_set_style_border_width(dot, 0, 0);
                lv_obj_clear_flag(dot, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_clear_flag(dot, LV_OBJ_FLAG_CLICKABLE);
                if (!seq_step_is_customized(t, s))
                    lv_obj_add_flag(dot, LV_OBJ_FLAG_HIDDEN);
            }

            lv_obj_add_event_cb(seq_step_btns[t][s], seq_step_long_press_cb,
                                LV_EVENT_LONG_PRESSED, (void*)(intptr_t)((t << 8) | s));
        }

        // ── Instrument FX button (idea 2) — opens the per-pad filter/FX
        // panel (with its own RANDOM FX) directly for this track. ──
        seq_fx_btns[t] = lv_btn_create(scr_sequencer);
        lv_obj_set_size(seq_fx_btns[t], SEQ_FX_W, SEQ_TRACK_H);
        lv_obj_set_pos(seq_fx_btns[t], SEQ_FX_X, rowY);
        apply_control_button_style(seq_fx_btns[t], tc, false, 4);
        lv_obj_set_style_bg_opa(seq_fx_btns[t], LV_OPA_20, 0);
        lv_obj_set_style_pad_all(seq_fx_btns[t], 0, 0);
        lv_obj_add_event_cb(seq_fx_btns[t], seq_row_fx_cb, LV_EVENT_CLICKED, (void*)(intptr_t)t);
        lv_obj_t* fx_lbl = lv_label_create(seq_fx_btns[t]);
        lv_label_set_text(fx_lbl, "FX");
        lv_obj_set_style_text_font(fx_lbl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(fx_lbl, tc, 0);
        lv_obj_center(fx_lbl);

        // ── Solo button ──
        seq_solo_btns[t] = lv_btn_create(scr_sequencer);
        lv_obj_set_size(seq_solo_btns[t], SEQ_SOLO_W, SEQ_TRACK_H);
        lv_obj_set_pos(seq_solo_btns[t], SEQ_SOLO_X, rowY);
        apply_control_button_style(seq_solo_btns[t], tc, false, 4);
        lv_obj_set_style_pad_all(seq_solo_btns[t], 0, 0);
        lv_obj_add_event_cb(seq_solo_btns[t], seq_solo_cb, LV_EVENT_CLICKED, (void*)(intptr_t)t);
        seq_solo_labels[t] = lv_label_create(seq_solo_btns[t]);
        lv_label_set_text(seq_solo_labels[t], "S");
        lv_obj_set_style_text_font(seq_solo_labels[t], &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(seq_solo_labels[t], RED808_TEXT_DIM, 0);
        lv_obj_center(seq_solo_labels[t]);

        // ── Clear FX button — resets this track's instrument FX in one tap ──
        seq_fx_clear_btns[t] = lv_btn_create(scr_sequencer);
        lv_obj_set_size(seq_fx_clear_btns[t], SEQ_FXCLR_W, SEQ_TRACK_H);
        lv_obj_set_pos(seq_fx_clear_btns[t], SEQ_FXCLR_X, rowY);
        apply_control_button_style(seq_fx_clear_btns[t], RED808_ERROR, false, 4);
        lv_obj_set_style_bg_opa(seq_fx_clear_btns[t], LV_OPA_20, 0);
        lv_obj_set_style_pad_all(seq_fx_clear_btns[t], 0, 0);
        lv_obj_add_event_cb(seq_fx_clear_btns[t], seq_row_fx_clear_cb, LV_EVENT_CLICKED, (void*)(intptr_t)t);
        lv_obj_t* fx_clear_lbl = lv_label_create(seq_fx_clear_btns[t]);
        lv_label_set_text(fx_clear_lbl, LV_SYMBOL_CLOSE " FX");
        lv_obj_set_style_text_font(fx_clear_lbl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(fx_clear_lbl, RED808_ERROR, 0);
        lv_obj_center(fx_clear_lbl);
    }   // end for(t)

    // ── Glowing vertical playhead (spans all rows, created last = on top) ──
    seq_playhead_line = lv_obj_create(scr_sequencer);
    lv_obj_set_pos(seq_playhead_line, seq_step_x[0], SEQ_RULER_Y);
    lv_obj_set_size(seq_playhead_line, SEQ_CELL_W, grid_bottom - SEQ_RULER_Y);
    lv_obj_set_style_radius(seq_playhead_line, 0, 0);
    lv_obj_set_style_bg_color(seq_playhead_line, RED808_WARNING, 0);
    lv_obj_set_style_bg_opa(seq_playhead_line, LV_OPA_20, 0);
    lv_obj_set_style_border_width(seq_playhead_line, 0, 0);
    lv_obj_set_style_border_color(seq_playhead_line, RED808_WARNING, 0);
    lv_obj_set_style_shadow_width(seq_playhead_line, 28, 0);
    lv_obj_set_style_shadow_color(seq_playhead_line, RED808_WARNING, 0);
    lv_obj_set_style_shadow_opa(seq_playhead_line, LV_OPA_50, 0);
    lv_obj_clear_flag(seq_playhead_line, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(seq_playhead_line, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(seq_playhead_line, LV_OBJ_FLAG_HIDDEN);

    // ── Status bar (bottom strip) ──
    {
        lv_obj_t* bar = lv_obj_create(scr_sequencer);
        lv_obj_set_pos(bar, 0, SEQ_STATUS_Y);
        lv_obj_set_size(bar, LCD_H_RES, SEQ_STATUS_H);
        lv_obj_set_style_bg_color(bar, lv_color_hex(0x060606), 0);
        lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(bar, 0, 0);
        lv_obj_set_style_radius(bar, 0, 0);
        lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(bar, LV_OBJ_FLAG_CLICKABLE);

        seq_status_pat_lbl = lv_label_create(scr_sequencer);
        lv_label_set_text_fmt(seq_status_pat_lbl, "PATTERN %02d", p4.current_pattern + 1);
        lv_obj_set_style_text_font(seq_status_pat_lbl, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(seq_status_pat_lbl, RED808_TEXT_DIM, 0);
        lv_obj_set_pos(seq_status_pat_lbl, 70, SEQ_STATUS_Y + 2);

        seq_status_name_lbl = lv_label_create(scr_sequencer);
        lv_label_set_text(seq_status_name_lbl, seq_pattern_name(p4.current_pattern));
        lv_obj_set_style_text_font(seq_status_name_lbl, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(seq_status_name_lbl, RED808_SUCCESS, 0);
        lv_obj_set_width(seq_status_name_lbl, 300);
        lv_obj_set_style_text_align(seq_status_name_lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_pos(seq_status_name_lbl, 160, SEQ_STATUS_Y + 2);

        seq_status_step_lbl = lv_label_create(scr_sequencer);
        lv_label_set_text(seq_status_step_lbl, "STEP -- / 16");
        lv_obj_set_style_text_font(seq_status_step_lbl, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(seq_status_step_lbl, RED808_ACCENT, 0);
        lv_obj_set_width(seq_status_step_lbl, 120);
        lv_obj_set_style_text_align(seq_status_step_lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_pos(seq_status_step_lbl, 500, SEQ_STATUS_Y + 2);

        seq_status_mix_lbl = lv_label_create(scr_sequencer);
        lv_label_set_text(seq_status_mix_lbl, "M 00  S 00");
        lv_obj_set_style_text_font(seq_status_mix_lbl, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(seq_status_mix_lbl, RED808_TEXT_DIM, 0);
        lv_obj_set_width(seq_status_mix_lbl, 160);
        lv_obj_set_style_text_align(seq_status_mix_lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_pos(seq_status_mix_lbl, 635, SEQ_STATUS_Y + 2);

        lv_obj_t* bpm_lbl = lv_label_create(scr_sequencer);
        lv_label_set_text_fmt(bpm_lbl, "BPM %d.%d", p4.bpm_int, p4.bpm_frac);
        lv_obj_set_style_text_font(bpm_lbl, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(bpm_lbl, RED808_INFO, 0);
        lv_obj_set_width(bpm_lbl, 205);
        lv_obj_set_style_text_align(bpm_lbl, LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_set_pos(bpm_lbl, LCD_H_RES - 215, SEQ_STATUS_Y + 2);
        seq_status_bpm_lbl = bpm_lbl;
    }

}   // end create_sequencer_screen

static void update_sequencer_screen(void) {
    int step = p4.current_step;
    bool playing = p4.is_playing;
    unsigned long now = millis();

    // Pattern loading modal lifecycle. P4 owns the bank, so completion is the
    // actual selected local slot. Daisy sync is an outbound upload and has no
    // pattern_sync response packet.
    if (seq_pattern_modal) {
        if (seq_pattern_waiting) {
            if (p4.current_pattern == seq_pattern_wait_pat) {
                seq_pattern_modal_mark_loaded();
            } else if ((now - seq_pattern_wait_ms) > 1000) {
                if (seq_pattern_modal_spin) {
                    lv_obj_add_flag(seq_pattern_modal_spin, LV_OBJ_FLAG_HIDDEN);
                }
                if (seq_pattern_modal_lbl) {
                    lv_label_set_text(seq_pattern_modal_lbl,
                        "No se pudo seleccionar el slot P4");
                    lv_obj_set_style_text_color(seq_pattern_modal_lbl, RED808_ERROR, 0);
                }
                lv_obj_set_style_border_color(seq_pattern_modal, RED808_ERROR, 0);
                seq_pattern_waiting = false;
                seq_pattern_wait_ms = now;
            }
        } else if ((now - seq_pattern_wait_ms) > 700) {
            seq_pattern_modal_hide();
        }
    }

    // ── Dirty tracking state (persistent across calls) ──
    // Cell visual key: bit2=active, bit1=is_cur(col), bit0=muted
    // Track key:       bit1=soloed, bit0=muted
    // 0xFF = uninitialized/force-refresh
    static uint8_t prev_cell_key[16][16];
    static uint8_t prev_trk_key[16];
    static int     prev_seq_theme = -1;
    static bool    seq_dt_init = false;
    if (!seq_dt_init) {
        seq_dt_init = true;
        memset(prev_cell_key, 0xFF, sizeof(prev_cell_key));
        memset(prev_trk_key,  0xFF, sizeof(prev_trk_key));
    }
    // Theme change invalidates all color-dependent styles
    if (currentTheme != prev_seq_theme) {
        prev_seq_theme = currentTheme;
        memset(prev_cell_key, 0xFF, sizeof(prev_cell_key));
        memset(prev_trk_key,  0xFF, sizeof(prev_trk_key));
    }
    // External pattern loaded: force full cell repaint regardless of dirty state
    if (seq_force_refresh_cells) {
        seq_force_refresh_cells = false;
        memset(prev_cell_key, 0xFF, sizeof(prev_cell_key));
        memset(prev_trk_key,  0xFF, sizeof(prev_trk_key));
    }
    if (seq_page_styles_dirty) {
        seq_page_styles_dirty = false;
        seq_apply_page_styles();
    }

    // Multi-bar imports remain available through P1..P4. Automatic page
    // switching is intentionally disabled: the Master only accepts individual
    // setStep packets, so changing a bar at step 0 cannot be atomic and used to
    // play a partially rewritten first beat. Manual page changes stop, load,
    // then resume through the deferred push path above.

    // ── Move / show glowing playhead line — disabled: the full-height overlay
    // sweeps over muted rows and reads as mute flicker. Per-cell current-step
    // highlighting below remains active for unmuted tracks only.
    static int  prev_ph_step    = -2;
    static bool prev_ph_playing = false;
    if (seq_playhead_line && (step != prev_ph_step || playing != prev_ph_playing)) {
        prev_ph_step    = step;
        prev_ph_playing = playing;
        lv_obj_add_flag(seq_playhead_line, LV_OBJ_FLAG_HIDDEN);
    }

    // ── Status bar step counter — only when changed ──
    static int  prev_stp_step    = -2;
    static bool prev_stp_playing = false;
    if (seq_status_step_lbl && (step != prev_stp_step || playing != prev_stp_playing)) {
        prev_stp_step    = step;
        prev_stp_playing = playing;
        if (playing && step >= 0 && step < 16) {
            lv_label_set_text_fmt(seq_status_step_lbl, "STEP %02d / 16", step + 1);
        } else {
            lv_label_set_text(seq_status_step_lbl, "STEP -- / 16");
        }
    }
    static int prev_stp_pat = -1;
    if (seq_status_pat_lbl && p4.current_pattern != prev_stp_pat) {
        prev_stp_pat = p4.current_pattern;
        lv_label_set_text_fmt(seq_status_pat_lbl, "PATTERN %02d", p4.current_pattern + 1);
        if (seq_status_name_lbl) lv_label_set_text(seq_status_name_lbl, seq_pattern_name(p4.current_pattern));
    }
    static int prev_stp_bpm_int = -1, prev_stp_bpm_frac = -1;
    static int prev_stp_original_bpm = -1;
    static lv_obj_t* prev_stp_bpm_lbl = NULL;
    if (seq_status_bpm_lbl &&
            (p4.bpm_int != prev_stp_bpm_int || p4.bpm_frac != prev_stp_bpm_frac
             || p4.original_bpm_x10 != prev_stp_original_bpm
             || seq_status_bpm_lbl != prev_stp_bpm_lbl)) {
        prev_stp_bpm_int  = p4.bpm_int;
        prev_stp_bpm_frac = p4.bpm_frac;
        prev_stp_original_bpm = p4.original_bpm_x10;
        prev_stp_bpm_lbl = seq_status_bpm_lbl;
        const int current_x10 = p4.bpm_int * 10 + p4.bpm_frac;
        const int delta_x10 = current_x10 - p4.original_bpm_x10;
        if (p4.original_bpm_x10 > 0 && delta_x10 != 0) {
            const char* direction = delta_x10 > 0 ? "FAST" : "SLOW";
            lv_label_set_text_fmt(seq_status_bpm_lbl,
                "BPM %d.%d  %s %+.1f / ORIG %.1f",
                p4.bpm_int, p4.bpm_frac, direction,
                delta_x10 / 10.0f, p4.original_bpm_x10 / 10.0f);
            lv_obj_set_style_text_color(seq_status_bpm_lbl,
                delta_x10 > 0 ? RED808_WARNING : RED808_INFO, 0);
        } else {
            lv_label_set_text_fmt(seq_status_bpm_lbl,
                "BPM %d.%d", p4.bpm_int, p4.bpm_frac);
            lv_obj_set_style_text_color(seq_status_bpm_lbl, RED808_INFO, 0);
        }
    }

    static uint16_t prev_status_mute_mask = 0xFFFFu;
    static uint16_t prev_status_solo_mask = 0xFFFFu;
    uint16_t statusMuteMask = 0;
    uint16_t statusSoloMask = 0;
    for (int t = 0; t < 16; ++t) {
        if (p4.track_muted[t]) statusMuteMask |= (uint16_t)(1u << t);
        if (p4.track_solo[t]) statusSoloMask |= (uint16_t)(1u << t);
    }
    if (seq_status_mix_lbl &&
        (statusMuteMask != prev_status_mute_mask
         || statusSoloMask != prev_status_solo_mask)) {
        prev_status_mute_mask = statusMuteMask;
        prev_status_solo_mask = statusSoloMask;
        lv_label_set_text_fmt(seq_status_mix_lbl, "M %02u  S %02u",
            (unsigned)__builtin_popcount((unsigned)statusMuteMask),
            (unsigned)__builtin_popcount((unsigned)statusSoloMask));
        lv_obj_set_style_text_color(seq_status_mix_lbl,
            statusSoloMask ? RED808_WARNING
                           : (statusMuteMask ? RED808_ERROR : RED808_TEXT_DIM), 0);
    }

    // ── Sequencer header play/pause + pattern — only when changed ──
    static bool prev_hdr_playing = false;
    if (seq_hdr_play_btn && seq_hdr_play_lbl && playing != prev_hdr_playing) {
        prev_hdr_playing = playing;
        lv_label_set_text(seq_hdr_play_lbl, playing ? "PAUSE" : "PLAY");
        lv_obj_set_style_bg_color(seq_hdr_play_btn,
            playing ? RED808_SUCCESS : RED808_ACCENT, 0);
        lv_obj_set_style_border_color(seq_hdr_play_btn,
            playing ? RED808_CYAN : RED808_ACCENT2, 0);
    }
    static int prev_hdr_pat = -1;
    if (seq_hdr_pat_lbl && p4.current_pattern != prev_hdr_pat) {
        prev_hdr_pat = p4.current_pattern;
        lv_label_set_text_fmt(seq_hdr_pat_lbl, "P%02d", p4.current_pattern + 1);
        if (seq_hdr_name_lbl) lv_label_set_text(seq_hdr_name_lbl, seq_pattern_name(p4.current_pattern));
        if (seq_queued_pattern == p4.current_pattern) seq_queued_pattern = -1;
    }
    static int prev_hdr_queue = -2;
    static bool prev_hdr_quantize = !seq_quantize_enabled;
    if (seq_hdr_queue_lbl && (seq_queued_pattern != prev_hdr_queue ||
                             seq_quantize_enabled != prev_hdr_quantize)) {
        prev_hdr_queue = seq_queued_pattern;
        prev_hdr_quantize = seq_quantize_enabled;
        if (!seq_quantize_enabled)
            lv_label_set_text(seq_hdr_queue_lbl, "Q OFF");
        else if (seq_queued_pattern >= 0)
            lv_label_set_text_fmt(seq_hdr_queue_lbl, "Q>P%02d", seq_queued_pattern + 1);
        else
            lv_label_set_text(seq_hdr_queue_lbl, "Q 1 BAR");
    }
    static int8_t prev_hdr_dirty = -1;
    if (seq_hdr_save_btn && seq_hdr_save_lbl
        && prev_hdr_dirty != (seq_pattern_dirty ? 1 : 0)) {
        prev_hdr_dirty = seq_pattern_dirty ? 1 : 0;
        seq_set_pattern_dirty(seq_pattern_dirty);
    }

    for (int g = 0; g < 4; ++g) {
        const uint16_t mask = seq_group_mask(g);
        bool allMuted = true;
        for (int t = 0; t < 16; ++t)
            if ((mask & (1u << t)) && !p4.track_muted[t]) allMuted = false;
        const uint8_t state = allMuted ? 1 : 0;
        if (seq_hdr_group_btns[g] && seq_hdr_group_state[g] != state) {
            seq_hdr_group_state[g] = state;
            lv_obj_set_style_bg_color(seq_hdr_group_btns[g],
                allMuted ? RED808_ERROR : RED808_SURFACE, 0);
            lv_obj_set_style_border_color(seq_hdr_group_btns[g],
                allMuted ? RED808_WARNING : RED808_BORDER, 0);
        }
    }

    // ── Per-track updates — dirty tracking ──
    // Only call lv_obj_set_style_* when visual state actually changes.
    // During playback the cursor column changes every step (16 cells update).
    // Static patterns: only toggled cells update (1 cell per tap).
    // This reduces 1280 style-calls/frame → ~2-32 calls/frame typical.
    bool anySolo = false;
    for (int t = 0; t < 16; ++t) anySolo |= p4.track_solo[t];
    for (int t = 0; t < 16; t++) {
        bool muted  = p4.track_muted[t];
        bool soloed = p4.track_solo[t];
        bool isolated = anySolo && !soloed;
        uint8_t trk_key = (uint8_t)((isolated ? 4 : 0) | (soloed ? 2 : 0)
                                    | (muted ? 1 : 0));
        lv_color_t tc = lv_color_hex(theme_presets[ui_theme_index()].track_colors[t]);

        if (trk_key != prev_trk_key[t]) {
            prev_trk_key[t] = trk_key;

            if (seq_mute_btns[t]) {
                if (muted) {
                    lv_obj_set_style_bg_color(seq_mute_btns[t], RED808_ERROR, 0);
                    lv_obj_set_style_bg_opa(seq_mute_btns[t], LV_OPA_90, 0);
                    lv_obj_set_style_border_color(seq_mute_btns[t], RED808_ERROR, 0);
                } else if (isolated) {
                    lv_obj_set_style_bg_color(seq_mute_btns[t], RED808_SURFACE, 0);
                    lv_obj_set_style_bg_opa(seq_mute_btns[t], LV_OPA_30, 0);
                    lv_obj_set_style_border_color(seq_mute_btns[t], RED808_TEXT_DIM, 0);
                } else {
                    lv_obj_set_style_bg_color(seq_mute_btns[t], RED808_SURFACE, 0);
                    lv_obj_set_style_bg_opa(seq_mute_btns[t], LV_OPA_50, 0);
                    lv_obj_set_style_border_color(seq_mute_btns[t], tc, 0);
                }
            }
            if (seq_track_labels[t]) {
                lv_obj_set_style_text_color(seq_track_labels[t],
                    muted ? lv_color_white() : (isolated ? RED808_TEXT_DIM : tc), 0);
            }
            if (seq_solo_btns[t]) {
                lv_obj_set_style_bg_color(seq_solo_btns[t], soloed ? tc : RED808_SURFACE, 0);
                lv_obj_set_style_border_color(seq_solo_btns[t], soloed ? tc : RED808_BORDER, 0);
                lv_obj_set_style_shadow_width(seq_solo_btns[t], soloed ? 14 : 0, 0);
                lv_obj_set_style_shadow_color(seq_solo_btns[t], tc, 0);
            }
            if (seq_solo_labels[t]) {
                lv_obj_set_style_text_color(seq_solo_labels[t],
                    soloed ? lv_color_black() : RED808_TEXT_DIM, 0);
            }
            // Mute change affects all cells in this track — invalidate their keys
            for (int s = 0; s < 16; s++) prev_cell_key[t][s] = 0xFF;
        }

        // Step cells — skip if visual key unchanged
        for (int s = 0; s < 16; s++) {
            if (!seq_step_btns[t][s]) continue;
            bool active = p4.steps[t][s];
            bool silenced = muted || isolated;
            bool is_cur = !silenced && playing && (step == s);
            uint8_t cell_key = (uint8_t)((isolated ? 8 : 0) | (active ? 4 : 0)
                                         | (is_cur ? 2 : 0) | (muted ? 1 : 0));
            if (cell_key == prev_cell_key[t][s]) continue;
            prev_cell_key[t][s] = cell_key;

            lv_color_t bg;
            lv_opa_t opa;
            lv_color_t border;
            int shadow_w;

            if (is_cur && active) {
                bg = lv_color_white();
                opa = LV_OPA_COVER;
                border = tc;
                shadow_w = 20;
            } else if (is_cur) {
                bg = lv_color_hex(0x262626);
                opa = LV_OPA_COVER;
                border = RED808_WARNING;
                shadow_w = 0;
            } else if (active) {
                bg = tc;
                opa = silenced ? LV_OPA_20 : LV_OPA_80;
                border = tc;
                shadow_w = silenced ? 0 : 8;
            } else {
                bg = RED808_SURFACE;
                opa = LV_OPA_40;
                border = lv_color_hex(0x1E1E1E);
                shadow_w = 0;
            }

            lv_obj_set_style_bg_color(seq_step_btns[t][s], bg, 0);
            lv_obj_set_style_bg_opa(seq_step_btns[t][s], opa, 0);
            lv_obj_set_style_border_color(seq_step_btns[t][s], border, 0);
            lv_obj_set_style_shadow_width(seq_step_btns[t][s], shadow_w, 0);
            lv_obj_set_style_shadow_color(seq_step_btns[t][s],
                is_cur ? RED808_WARNING : tc, 0);
            if (seq_step_accents[t][s]) {
                if (active) lv_obj_clear_flag(seq_step_accents[t][s], LV_OBJ_FLAG_HIDDEN);
                else        lv_obj_add_flag(seq_step_accents[t][s], LV_OBJ_FLAG_HIDDEN);
            }
        }
    }
}

// =============================================================================
// MIXER SCREEN — 16 track faders in a single row
// =============================================================================
static lv_obj_t* vol_sliders[16] = {};
static lv_obj_t* vol_labels[16] = {};
static lv_obj_t* vol_name_labels[16] = {};
static lv_obj_t* vol_mute_dots[16] = {};
static lv_obj_t* vol_strip_panels[16] = {};
static lv_obj_t* mix_master_slider = NULL;
static lv_obj_t* mix_seq_slider = NULL;
static lv_obj_t* mix_live_slider = NULL;
static lv_obj_t* mix_bpm_slider = NULL;
static lv_obj_t* mix_master_lbl = NULL;
static lv_obj_t* mix_seq_lbl = NULL;
static lv_obj_t* mix_live_lbl = NULL;
static lv_obj_t* mix_bpm_lbl = NULL;
static lv_obj_t* mix_pattern_lbl = NULL;   // active pattern number, shown in the header

static void mix_global_slider_cb(lv_event_t* e) {
    int which = (int)(intptr_t)lv_event_get_user_data(e);
    lv_obj_t* slider = (lv_obj_t*)lv_event_get_target(e);
    int val = lv_slider_get_value(slider);
    static uint32_t last_tx_ms[4] = {};
    uint32_t now = millis();
    bool final_value = (lv_event_get_code(e) == LV_EVENT_RELEASED ||
                        lv_event_get_code(e) == LV_EVENT_PRESS_LOST);
    bool transmit = final_value || last_tx_ms[which & 3] == 0 ||
                    (uint32_t)(now - last_tx_ms[which & 3]) >= 30;
    const uint8_t functions[4] = {
        POD_FUNC_MASTER_VOLUME, POD_FUNC_SEQ_VOLUME,
        POD_FUNC_LIVE_VOLUME, POD_FUNC_TEMPO
    };
    if (which >= 0 && which < 4
        && i2c_rotaries_owns_function(functions[which])) {
        const int canonical[4] = {
            p4.master_volume, p4.seq_volume, p4.live_volume, p4.bpm_int
        };
        lv_slider_set_value(slider, canonical[which], LV_ANIM_OFF);
        return;
    }
    switch (which) {
        case 0:
            p4.master_volume = val;
            if (transmit) control_send_set_volume(val);
            break;
        case 1:
            p4.seq_volume = val;
            if (transmit) control_send_set_seq_volume(val);
            break;
        case 2:
            p4.live_volume = val;
            if (transmit) control_send_set_live_volume(val);
            break;
        case 3:
            p4.bpm_int = val;
            p4.bpm_frac = 0;
            if (transmit) control_send_tempo((float)val);
            break;
        default:
            break;
    }
    if (transmit) last_tx_ms[which & 3] = now;
}

static void xtra_editor_refresh_values(void) {
    if (s_xtra_editor_slot < 0 || s_xtra_editor_slot >= 4) return;
    XtraPadSlot& slot = s_xtra_slots[s_xtra_editor_slot];
    int start = s_xtra_editor_start ? lv_slider_get_value(s_xtra_editor_start) : slot.trim_start_pct;
    int end = s_xtra_editor_end ? lv_slider_get_value(s_xtra_editor_end) : slot.trim_end_pct;
    int gate = s_xtra_editor_gate ? lv_slider_get_value(s_xtra_editor_gate) : slot.gate_ms;
    int fadeIn = s_xtra_editor_fade_in ? lv_slider_get_value(s_xtra_editor_fade_in) : slot.fade_in_ms;
    int fadeOut = s_xtra_editor_fade_out ? lv_slider_get_value(s_xtra_editor_fade_out) : slot.fade_out_ms;
    if (s_xtra_editor_start_lbl) lv_label_set_text_fmt(s_xtra_editor_start_lbl, "START  %d%%", start);
    if (s_xtra_editor_end_lbl) lv_label_set_text_fmt(s_xtra_editor_end_lbl, "END  %d%%", end);
    if (s_xtra_editor_gate_lbl) lv_label_set_text_fmt(s_xtra_editor_gate_lbl, "GATE  %d ms", gate);
    if (s_xtra_editor_fade_in_lbl) {
        if (fadeIn == 0) lv_label_set_text(s_xtra_editor_fade_in_lbl, "FADE IN  OFF");
        else lv_label_set_text_fmt(s_xtra_editor_fade_in_lbl, "FADE IN  %d ms", fadeIn);
    }
    if (s_xtra_editor_fade_out_lbl) {
        if (fadeOut == 0) lv_label_set_text(s_xtra_editor_fade_out_lbl, "FADE OUT  OFF");
        else lv_label_set_text_fmt(s_xtra_editor_fade_out_lbl, "FADE OUT  %d ms", fadeOut);
    }
    if (s_xtra_editor_mode_lbl) {
        lv_label_set_text(s_xtra_editor_mode_lbl,
            slot.play_mode == 1 ? "SYNC REPEAT" : "ONE SHOT");
    }
}

static void xtra_editor_slider_cb(lv_event_t* e) {
    LV_UNUSED(e);
    if (!s_xtra_editor_start || !s_xtra_editor_end) return;
    int start = lv_slider_get_value(s_xtra_editor_start);
    int end = lv_slider_get_value(s_xtra_editor_end);
    if (start >= end - 2) {
        lv_obj_t* target = (lv_obj_t*)lv_event_get_target(e);
        if (target == s_xtra_editor_start) lv_slider_set_value(s_xtra_editor_start, end - 2, LV_ANIM_OFF);
        else lv_slider_set_value(s_xtra_editor_end, start + 2, LV_ANIM_OFF);
    }
    xtra_editor_refresh_values();
}

static void xtra_editor_close_cb(lv_event_t* e) {
    LV_UNUSED(e);
    if (s_xtra_editor_modal) lv_obj_del(s_xtra_editor_modal);
    s_xtra_editor_modal = NULL;
    s_xtra_editor_start = NULL;
    s_xtra_editor_end = NULL;
    s_xtra_editor_gate = NULL;
    s_xtra_editor_fade_in = NULL;
    s_xtra_editor_fade_out = NULL;
    s_xtra_editor_start_lbl = NULL;
    s_xtra_editor_end_lbl = NULL;
    s_xtra_editor_gate_lbl = NULL;
    s_xtra_editor_fade_in_lbl = NULL;
    s_xtra_editor_fade_out_lbl = NULL;
    s_xtra_editor_mode_lbl = NULL;
    s_xtra_editor_wave = NULL;
    s_xtra_editor_slot = -1;
}

static void xtra_editor_mode_cb(lv_event_t* e) {
    LV_UNUSED(e);
    if (s_xtra_editor_slot < 0 || s_xtra_editor_slot >= 4) return;
    XtraPadSlot& slot = s_xtra_slots[s_xtra_editor_slot];
    slot.play_mode = slot.play_mode == 0 ? 1 : 0;
    xtra_editor_refresh_values();
}

static void xtra_editor_preview_cb(lv_event_t* e) {
    LV_UNUSED(e);
    if (s_xtra_editor_slot < 0 || s_xtra_editor_slot >= 4 || !control_available()) return;
    XtraPadSlot& slot = s_xtra_slots[s_xtra_editor_slot];
    if (slot.synth_mode) {
        xtra_trigger_slot(s_xtra_editor_slot, 200, 40, true);
    } else {
        control_send_trigger(slot.pad, 112);
    }
}

static void xtra_editor_apply_cb(lv_event_t* e) {
    LV_UNUSED(e);
    if (s_xtra_editor_slot < 0 || s_xtra_editor_slot >= 4) return;
    XtraPadSlot& slot = s_xtra_slots[s_xtra_editor_slot];
    int start = s_xtra_editor_start ? lv_slider_get_value(s_xtra_editor_start) : 0;
    int end = s_xtra_editor_end ? lv_slider_get_value(s_xtra_editor_end) : 100;
    int gate = s_xtra_editor_gate ? lv_slider_get_value(s_xtra_editor_gate) : 180;
    int fadeIn = s_xtra_editor_fade_in ? lv_slider_get_value(s_xtra_editor_fade_in) : 0;
    int fadeOut = s_xtra_editor_fade_out ? lv_slider_get_value(s_xtra_editor_fade_out) : 0;
    slot.gate_ms = (uint16_t)constrain(gate, 40, 2000);
    slot.trim_start_pct = (uint8_t)constrain(start, 0, 95);
    slot.trim_end_pct = (uint8_t)constrain(end, 5, 100);
    slot.fade_in_ms = (uint8_t)constrain(fadeIn, 0, 255);
    slot.fade_out_ms = (uint8_t)constrain(fadeOut, 0, 255);
    // Non-destructive on Daisy (CMD_PAD_TRIM applies start/end at trigger
    // time — never rewrites the uploaded PCM), so the sliders keep showing
    // exactly what's active instead of resetting to 0/100 as if consumed.
    if (!slot.synth_mode) {
        control_send_trim_sample(slot.pad, start / 100.0f, end / 100.0f);
        control_send_set_pad_fade_in(slot.pad, slot.fade_in_ms);
        control_send_set_pad_fade_out(slot.pad, slot.fade_out_ms);
        ui_show_toast("Trim aplicado al sample", theme_success());
    } else {
        ui_show_toast("Ajustes XTRA guardados", theme_success());
    }
    xtra_save_state();
    xtra_refresh_panel();
    xtra_editor_refresh_values();
}

static void xtra_editor_load_cb(lv_event_t* e) {
    LV_UNUSED(e);
    int slot = s_xtra_editor_slot;
    xtra_editor_close_cb(NULL);
    xtra_begin_load_for_slot(slot);
}

static void xtra_editor_open(int slot_idx) {
    if (slot_idx < 0 || slot_idx >= 4 || !scr_performance) return;
    if (s_xtra_editor_modal) lv_obj_del(s_xtra_editor_modal);
    s_xtra_editor_slot = slot_idx;
    XtraPadSlot& slot = s_xtra_slots[slot_idx];

    s_xtra_editor_modal = lv_obj_create(scr_performance);
    lv_obj_set_size(s_xtra_editor_modal, LCD_H_RES, LCD_V_RES);
    lv_obj_set_pos(s_xtra_editor_modal, 0, 0);
    lv_obj_set_style_bg_color(s_xtra_editor_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_xtra_editor_modal, LV_OPA_70, 0);
    lv_obj_set_style_border_width(s_xtra_editor_modal, 0, 0);
    lv_obj_set_style_pad_all(s_xtra_editor_modal, 0, 0);
    lv_obj_clear_flag(s_xtra_editor_modal, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* card = lv_obj_create(s_xtra_editor_modal);
    lv_obj_set_size(card, 900, 566);
    lv_obj_center(card);
    lv_obj_set_style_radius(card, 18, 0);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_color(card, RED808_BG, 0);
    lv_obj_set_style_bg_grad_dir(card, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_color(card, xtra_slot_color(slot_idx), 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_pad_all(card, 0, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* title = lv_label_create(card);
    lv_label_set_text_fmt(title, "S%02d  ·  %s", slot_idx + 1,
                          slot.name[0] ? slot.name : "XTRA SAMPLE");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(title, xtra_slot_color(slot_idx), 0);
    lv_obj_set_pos(title, 24, 18);

    lv_obj_t* meta = lv_label_create(card);
    if (slot.sample_rate) {
        lv_label_set_text_fmt(meta, "%.2fs  ·  %lu Hz  ·  %u-bit  ·  %s",
            slot.duration_ms / 1000.0f, (unsigned long)slot.sample_rate,
            (unsigned)slot.bits, slot.channels == 2 ? "STEREO" : "MONO");
    } else {
        lv_label_set_text(meta, slot.synth_mode ? "SYNTH VOICE" : "WAV SAMPLE");
    }
    lv_obj_set_style_text_font(meta, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(meta, theme_text_dim(), 0);
    lv_obj_set_pos(meta, 26, 52);

    lv_obj_t* wave_card = lv_obj_create(card);
    lv_obj_set_pos(wave_card, 24, 82);
    lv_obj_set_size(wave_card, 852, 150);
    lv_obj_set_style_radius(wave_card, 12, 0);
    lv_obj_set_style_bg_color(wave_card, RED808_SURFACE, 0);
    lv_obj_set_style_border_color(wave_card, theme_border(), 0);
    lv_obj_set_style_border_width(wave_card, 1, 0);
    lv_obj_set_style_pad_all(wave_card, 0, 0);
    lv_obj_clear_flag(wave_card, LV_OBJ_FLAG_SCROLLABLE);

    uint32_t seed = 2166136261u;
    for (const char* p = slot.name; *p; ++p) seed = (seed ^ (uint8_t)*p) * 16777619u;
    bool real_wave = s_xtra_wave_count[slot_idx] == 96;
    for (int i = 0; i < 96; i++) {
        int vmax, vmin;
        if (real_wave) {
            vmax = s_xtra_wave_max[slot_idx][i];
            vmin = s_xtra_wave_min[slot_idx][i];
        } else {
            seed = seed * 1664525u + 1013904223u;
            int envelope = 20 + (int)((seed >> 25) & 31U);
            vmax = envelope;
            vmin = -envelope;
        }
        int x = 8 + i * 8;
        s_xtra_editor_wave_points[i * 2].x = x;
        s_xtra_editor_wave_points[i * 2].y = 75 - vmax * 64 / 127;
        s_xtra_editor_wave_points[i * 2 + 1].x = x;
        s_xtra_editor_wave_points[i * 2 + 1].y = 75 - vmin * 64 / 127;
    }
    s_xtra_editor_wave = lv_line_create(wave_card);
    lv_line_set_points(s_xtra_editor_wave, s_xtra_editor_wave_points, 192);
    lv_obj_set_style_line_color(s_xtra_editor_wave, xtra_slot_color(slot_idx), 0);
    lv_obj_set_style_line_width(s_xtra_editor_wave, 2, 0);

    auto make_slider = [&](int y, int minv, int maxv, int value, lv_obj_t** out,
                           lv_obj_t** out_lbl) {
        *out_lbl = lv_label_create(card);
        lv_obj_set_style_text_font(*out_lbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(*out_lbl, theme_text(), 0);
        lv_obj_set_pos(*out_lbl, 28, y - 5);
        *out = lv_slider_create(card);
        lv_obj_set_pos(*out, 170, y);
        lv_obj_set_size(*out, 520, 18);
        lv_slider_set_range(*out, minv, maxv);
        lv_slider_set_value(*out, value, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(*out, RED808_SURFACE, LV_PART_MAIN);
        lv_obj_set_style_bg_color(*out, xtra_slot_color(slot_idx), LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(*out, lv_color_white(), LV_PART_KNOB);
        lv_obj_add_event_cb(*out, xtra_editor_slider_cb, LV_EVENT_VALUE_CHANGED, NULL);
    };
    make_slider(256, 0, 95, slot.trim_start_pct, &s_xtra_editor_start, &s_xtra_editor_start_lbl);
    make_slider(300, 5, 100, slot.trim_end_pct, &s_xtra_editor_end, &s_xtra_editor_end_lbl);
    make_slider(344, 40, 2000, slot.gate_ms, &s_xtra_editor_gate, &s_xtra_editor_gate_lbl);
    // Fade in/out — click-free ramps around the trim window's own edges
    // (see control_send_set_pad_fade_in/out); 0ms = off, same convention
    // as every other macro-style FX in this app.
    make_slider(388, 0, 255, slot.fade_in_ms, &s_xtra_editor_fade_in, &s_xtra_editor_fade_in_lbl);
    make_slider(432, 0, 255, slot.fade_out_ms, &s_xtra_editor_fade_out, &s_xtra_editor_fade_out_lbl);

    lv_obj_t* mode_btn = piano_make_chip(card, 712, 252, 164, 54,
                                         slot.play_mode == 1 ? "SYNC REPEAT" : "ONE SHOT");
    s_xtra_editor_mode_lbl = lv_obj_get_child(mode_btn, 0);
    lv_obj_set_style_border_color(mode_btn, xtra_slot_color(slot_idx), 0);
    lv_obj_add_event_cb(mode_btn, xtra_editor_mode_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* preview = piano_make_chip(card, 712, 318, 164, 54, "PREVIEW");
    lv_obj_set_style_border_color(preview, theme_success(), 0);
    lv_obj_add_event_cb(preview, xtra_editor_preview_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* apply = piano_make_chip(card, 24, 486, 180, 52, "APPLY / TRIM");
    lv_obj_set_style_border_color(apply, xtra_slot_color(slot_idx), 0);
    lv_obj_add_event_cb(apply, xtra_editor_apply_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* load = piano_make_chip(card, 220, 486, 180, 52, "LOAD NEW WAV");
    lv_obj_add_event_cb(load, xtra_editor_load_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* close = piano_make_chip(card, 708, 486, 168, 52, "CLOSE");
    lv_obj_add_event_cb(close, xtra_editor_close_cb, LV_EVENT_CLICKED, NULL);
    xtra_editor_refresh_values();
}

static void vol_slider_cb(lv_event_t* e) {
    int trk = (int)(intptr_t)lv_event_get_user_data(e);
    if (trk < 0 || trk >= 16) return;
    lv_obj_t* slider = (lv_obj_t*)lv_event_get_target(e);
    int val = lv_slider_get_value(slider);
    p4.track_volume[trk] = val;
    static uint32_t last_tx_ms[16] = {};
    uint32_t now = millis();
    bool final_value = (lv_event_get_code(e) == LV_EVENT_RELEASED ||
                        lv_event_get_code(e) == LV_EVENT_PRESS_LOST);
    bool transmit = final_value || last_tx_ms[trk] == 0 ||
                    (uint32_t)(now - last_tx_ms[trk]) >= 30;
    if (transmit) {
        control_send_set_track_volume(trk, val);
        local_apply_message(MSG_TRACK, TRK_VOLUME | (trk & 0x0F), (uint8_t)val);
        last_tx_ms[trk] = now;
    }
}

// ── MIXER: RANDOM MIX — one tap rebalances all 16 channel faders to a
// randomly chosen style, staying within a musically sane volume range per
// track role instead of a flat/uniform level. Same style family as the
// sequencer's PLAY RANDOM and the FX LAB / PAD SOUND random buttons.
struct MixStyleProfile {
    const char* name;
    uint8_t volMin[16];   // track order: BD SD CH OH CY CP RS CB LT MT HT MA CL HC MC LC
    uint8_t volMax[16];
};
static const MixStyleProfile MIX_STYLE_PROFILES[6] = {
    {"TECHNO",  {120, 40, 90, 70, 50, 60, 30, 20, 40, 40, 40, 30, 30, 25, 25, 25},
                {150, 70,120,100, 80, 90, 55, 45, 70, 70, 70, 55, 55, 50, 50, 50}},
    {"HOUSE",   {115, 45, 80, 90, 55, 85, 35, 40, 40, 40, 40, 45, 45, 40, 40, 40},
                {145, 75,110,120, 85,115, 60, 70, 65, 65, 65, 75, 75, 65, 65, 65}},
    {"BREAK",   {110,100, 70, 60, 60, 70, 45, 30, 60, 60, 60, 35, 35, 30, 30, 30},
                {140,130,100, 90, 90,100, 75, 55, 90, 90, 90, 60, 60, 55, 55, 55}},
    {"HIPHOP",  {115, 95, 55, 40, 35, 60, 30, 20, 35, 35, 35, 25, 25, 20, 20, 20},
                {145,125, 85, 65, 60, 90, 55, 40, 60, 60, 60, 50, 50, 45, 45, 45}},
    {"TRAP",    {120, 90,100, 55, 30, 55, 25, 20, 30, 30, 30, 20, 20, 20, 20, 20},
                {150,120,135, 85, 55, 85, 50, 40, 55, 55, 55, 40, 40, 40, 40, 40}},
    {"MINIMAL", { 90, 45, 55, 45, 35, 40, 40, 30, 30, 30, 30, 40, 40, 35, 35, 35},
                {120, 75, 85, 75, 60, 65, 65, 55, 55, 55, 55, 65, 65, 60, 60, 60}},
};

static lv_obj_t* s_mix_random_btn = NULL;
static uint8_t   s_mix_last_style = 0xFF;   // 0xFF = none applied yet this session
static bool      s_mix_random_smooth = false; // false = brusca (snap), true = suave (ramp)
static bool      mix_random_smooth_get(void) { return s_mix_random_smooth; }
static void      mix_random_smooth_set(bool v) { s_mix_random_smooth = v; }

// Smooth-transition ramp for RANDOM MIX ("suave" mode): interpolates every
// fader from its current volume to the new random target over ~700ms
// instead of snapping instantly ("brusca" mode, the default).
struct MixRampStep { uint8_t track; uint8_t fromVol; uint8_t toVol; };
static MixRampStep s_mix_ramp_steps[16];
static int         s_mix_ramp_count = 0;

static void mix_ramp_anim_cb(void* /*var*/, int32_t v) {
    for (int i = 0; i < s_mix_ramp_count; i++) {
        const MixRampStep& st = s_mix_ramp_steps[i];
        int vol = st.fromVol + (int)(((int32_t)(st.toVol - st.fromVol) * v) / 1000);
        control_send_set_track_volume(st.track, vol);
        local_apply_message(MSG_TRACK, TRK_VOLUME | (st.track & 0x0F), (uint8_t)vol);
        if (vol_sliders[st.track]) lv_slider_set_value(vol_sliders[st.track], vol, LV_ANIM_OFF);
    }
}

static void mix_ramp_start(int count) {
    s_mix_ramp_count = count;
    if (count <= 0) return;
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, &s_mix_ramp_steps);
    lv_anim_set_exec_cb(&a, mix_ramp_anim_cb);
    lv_anim_set_values(&a, 0, 1000);
    lv_anim_set_time(&a, 700);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);
}

static lv_obj_t* s_mix_random_stop_badge = NULL;

static void mix_random_btn_refresh(void) {
    if (!s_mix_random_btn) return;
    const bool active = control_random_mix_active();
    apply_control_button_style(s_mix_random_btn,
        active ? RED808_SUCCESS : RED808_INFO, false, 10);
    if (s_mix_random_stop_badge) {
        if (active) lv_obj_clear_flag(s_mix_random_stop_badge, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(s_mix_random_stop_badge, LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_t* lbl = lv_obj_get_child(s_mix_random_btn, 0);
    if (!lbl) return;
    if (s_mix_last_style < 6)
        lv_label_set_text_fmt(lbl, LV_SYMBOL_SHUFFLE "  %s", MIX_STYLE_PROFILES[s_mix_last_style].name);
    else
        lv_label_set_text(lbl, LV_SYMBOL_SHUFFLE "  RANDOM MIX");
}

static void mix_random_stop_badge_cb(lv_event_t* e) {
    LV_UNUSED(e);
    control_random_mix_set_active(false);
    mix_random_btn_refresh();
}

// Applies one random per-style volume rebalance across all 16 faders.
// Called both by a manual tap and by AUTO MIX's bar clock
// (control_random_auto_tick(), control_api.cpp).
void mix_random_apply(bool showToast) {
    if (!control_available() && !control_engine_connected()) {
        ui_show_toast("Master no conectado", RED808_WARNING);
        return;
    }
    static uint32_t s = 0;
    if (s == 0) s = (uint32_t)millis() ^ 0x7F4A7C15u | 1u;
    auto nextRand = [&]() -> uint32_t {
        s ^= s << 13; s ^= s >> 17; s ^= s << 5;
        return s;
    };
    auto randRange = [&](int mn, int mx) -> int {
        if (mx <= mn) return mn;
        return mn + (int)(nextRand() % (uint32_t)(mx - mn + 1));
    };

    const int styleIdx = randRange(0, 5);
    const MixStyleProfile& style = MIX_STYLE_PROFILES[styleIdx];

    if (s_mix_random_smooth) {
        for (int t = 0; t < 16; t++) {
            int vol = randRange(style.volMin[t], style.volMax[t]);
            s_mix_ramp_steps[t] = {(uint8_t)t, (uint8_t)p4.track_volume[t], (uint8_t)vol};
        }
        mix_ramp_start(16);
    } else {
        for (int t = 0; t < 16; t++) {
            int vol = randRange(style.volMin[t], style.volMax[t]);
            control_send_set_track_volume(t, vol);
            local_apply_message(MSG_TRACK, TRK_VOLUME | (t & 0x0F), (uint8_t)p4.track_volume[t]);
            if (vol_sliders[t]) lv_slider_set_value(vol_sliders[t], p4.track_volume[t], LV_ANIM_OFF);
        }
    }

    s_mix_last_style = (uint8_t)styleIdx;
    mix_random_btn_refresh();
    if (showToast) {
        char msg[48];
        snprintf(msg, sizeof(msg), "RANDOM MIX: %s", style.name);
        ui_show_toast(msg, RED808_INFO);
    }
}

static void mix_random_apply_now(void) {
    mix_random_apply(true);
}

// Tapping RANDOM MIX opens the AUTO MIX popup (cadence + on/off + "apply
// now") instead of applying directly, matching AUTO FX's entry point.
static void mix_random_modal_show(lv_event_t* e) {
    LV_UNUSED(e);
    static const AutoModalConfig cfg = {
        "AUTO MIX", control_random_mix_active, control_random_mix_set_active,
        control_random_mix_bars, control_random_mix_set_bars, mix_random_apply_now,
        mix_random_smooth_get, mix_random_smooth_set
    };
    auto_modal_show(cfg);
}

// =============================================================================
// MIXER PRESETS — 8 savable snapshots of the 16-track volume/mute/solo state.
// Phase 2 of the SONG "MATRIX" plan (see FILTER PRESETS above for Phase 1
// and the reasoning). Pan isn't captured: P4 doesn't cache a live pan value
// anywhere today (only volume/mute/solo round-trip into the p4 struct), and
// adding that would mean new state plumbing outside this feature's scope.
// =============================================================================
#define MIXER_PRESET_COUNT 8
struct MixerPresetSlot {
    bool used;
    char name[16];
    int volume[16];
    bool muted[16];
    bool solo[16];
};
static MixerPresetSlot s_mixer_presets[MIXER_PRESET_COUNT] = {};
static const char* MIXER_PRESETS_FILE = "/mixer_presets.txt";

static lv_obj_t* s_mixer_preset_modal = NULL;
static lv_obj_t* s_mixer_preset_slot_btns[MIXER_PRESET_COUNT] = {};
static lv_obj_t* s_mixer_preset_slot_lbls[MIXER_PRESET_COUNT] = {};

// One header line "used,name" per slot, followed by 16 lines "volume,muted,
// solo" (one per track) — avoids a 50-field sscanf format string.
static void mixer_presets_save_to_disk(void) {
    File f = SPIFFS.open(MIXER_PRESETS_FILE, FILE_WRITE);
    if (!f) return;
    for (int i = 0; i < MIXER_PRESET_COUNT; i++) {
        const MixerPresetSlot& s = s_mixer_presets[i];
        f.printf("%d,%s\n", s.used ? 1 : 0, s.name[0] ? s.name : "-");
        for (int t = 0; t < 16; t++)
            f.printf("%d,%d,%d\n", s.volume[t], s.muted[t] ? 1 : 0, s.solo[t] ? 1 : 0);
    }
    f.close();
}

static void mixer_presets_load_from_disk(void) {
    memset(s_mixer_presets, 0, sizeof(s_mixer_presets));
    for (int i = 0; i < MIXER_PRESET_COUNT; i++)
        for (int t = 0; t < 16; t++) s_mixer_presets[i].volume[t] = 100;
    File f = SPIFFS.open(MIXER_PRESETS_FILE, FILE_READ);
    if (!f) return;
    char line[64];
    for (int i = 0; i < MIXER_PRESET_COUNT; i++) {
        if (!fs_read_line(f, line, sizeof(line))) break;
        int used = 0;
        char name[16] = {};
        int parsed = sscanf(line, "%d,%15[^\n]", &used, name);
        MixerPresetSlot& s = s_mixer_presets[i];
        s.used = (used != 0);
        if (parsed >= 2) strncpy(s.name, name, sizeof(s.name) - 1);
        for (int t = 0; t < 16; t++) {
            if (!fs_read_line(f, line, sizeof(line))) break;
            int vol = 100, muted = 0, solo = 0;
            sscanf(line, "%d,%d,%d", &vol, &muted, &solo);
            s.volume[t] = constrain(vol, 0, 150);
            s.muted[t] = (muted != 0);
            s.solo[t] = (solo != 0);
        }
    }
    f.close();
}

static void mixer_preset_modal_refresh(void) {
    for (int i = 0; i < MIXER_PRESET_COUNT; i++) {
        if (!s_mixer_preset_slot_lbls[i]) continue;
        const MixerPresetSlot& s = s_mixer_presets[i];
        if (s.used) {
            lv_label_set_text_fmt(s_mixer_preset_slot_lbls[i], "S%d\n%s", i + 1, s.name[0] ? s.name : "PRESET");
            lv_obj_set_style_text_color(s_mixer_preset_slot_lbls[i], lv_color_white(), 0);
        } else {
            lv_label_set_text_fmt(s_mixer_preset_slot_lbls[i], "S%d\nVACIO", i + 1);
            lv_obj_set_style_text_color(s_mixer_preset_slot_lbls[i], theme_text_dim(), 0);
        }
    }
}

static void mixer_preset_save_current(int slot) {
    if (slot < 0 || slot >= MIXER_PRESET_COUNT) return;
    MixerPresetSlot& s = s_mixer_presets[slot];
    s.used = true;
    snprintf(s.name, sizeof(s.name), "MIX %d", slot + 1);
    for (int t = 0; t < 16; t++) {
        s.volume[t] = p4.track_volume[t];
        s.muted[t] = p4.track_muted[t];
        s.solo[t] = p4.track_solo[t];
    }
    mixer_presets_save_to_disk();
    mixer_preset_modal_refresh();
    ui_show_toast("Preset de mixer guardado", RED808_SUCCESS);
}

static void mixer_preset_recall(int slot) {
    if (slot < 0 || slot >= MIXER_PRESET_COUNT) return;
    const MixerPresetSlot& s = s_mixer_presets[slot];
    if (!s.used) {
        ui_show_toast("Slot vacio — manten pulsado para guardar", RED808_WARNING);
        return;
    }
    if (!control_available()) {
        ui_show_toast("Master no conectado", RED808_WARNING);
        return;
    }
    // update_volumes_screen() polls p4.track_volume/muted every frame and
    // repaints sliders/mute buttons on change — no manual slider sync needed
    // here, same as how RANDOM MIX's own ramped path already relies on it.
    for (int t = 0; t < 16; t++) {
        control_send_set_track_volume(t, s.volume[t]);
        control_send_mute(t, s.muted[t]);
        control_send_solo(t, s.solo[t]);
    }
    ui_show_toast("Preset de mixer cargado", RED808_SUCCESS);
}

static void mixer_preset_slot_clicked_cb(lv_event_t* e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    if (lv_event_get_code(e) == LV_EVENT_LONG_PRESSED) mixer_preset_save_current(slot);
    else mixer_preset_recall(slot);
}

static void mixer_preset_modal_close_cb(lv_event_t* e) {
    if (e && lv_event_get_target(e) != lv_event_get_current_target(e)) return;
    if (s_mixer_preset_modal) {
        lv_obj_del(s_mixer_preset_modal);
        s_mixer_preset_modal = NULL;
        for (int i = 0; i < MIXER_PRESET_COUNT; i++) {
            s_mixer_preset_slot_btns[i] = NULL;
            s_mixer_preset_slot_lbls[i] = NULL;
        }
    }
}

static void mixer_preset_modal_show(lv_event_t* e) {
    LV_UNUSED(e);
    if (s_mixer_preset_modal) return;

    s_mixer_preset_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_mixer_preset_modal, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(s_mixer_preset_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_mixer_preset_modal, LV_OPA_80, 0);
    lv_obj_set_style_border_width(s_mixer_preset_modal, 0, 0);
    lv_obj_set_style_pad_all(s_mixer_preset_modal, 0, 0);
    lv_obj_clear_flag(s_mixer_preset_modal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_mixer_preset_modal, mixer_preset_modal_close_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* card = lv_obj_create(s_mixer_preset_modal);
    lv_obj_set_size(card, 720, 260);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_color(card, RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_dir(card, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_border_color(card, RED808_CYAN, 0);
    lv_obj_set_style_radius(card, 18, 0);
    lv_obj_set_style_pad_all(card, 14, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t* title = lv_label_create(card);
    lv_label_set_text(title, "MIXER PRESETS");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(title, RED808_CYAN, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 2);

    lv_obj_t* hint = lv_label_create(card);
    lv_label_set_text(hint, "TOCA = cargar   ·   MANTEN PULSADO = guardar el mixer actual aqui");
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(hint, RED808_TEXT_DIM, 0);
    lv_obj_align(hint, LV_ALIGN_TOP_MID, 0, 32);

    constexpr int btnW = 80, btnH = 84, gapX = 6, y0 = 68;
    for (int i = 0; i < MIXER_PRESET_COUNT; i++) {
        lv_obj_t* btn = lv_btn_create(card);
        s_mixer_preset_slot_btns[i] = btn;
        lv_obj_set_size(btn, btnW, btnH);
        lv_obj_set_pos(btn, 4 + i * (btnW + gapX), y0);
        apply_control_button_style(btn, RED808_ACCENT2, false, 8);
        lv_obj_add_event_cb(btn, mixer_preset_slot_clicked_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
        lv_obj_add_event_cb(btn, mixer_preset_slot_clicked_cb, LV_EVENT_LONG_PRESSED, (void*)(intptr_t)i);

        lv_obj_t* lbl = lv_label_create(btn);
        s_mixer_preset_slot_lbls[i] = lbl;
        lv_obj_set_width(lbl, btnW - 8);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_center(lbl);
    }

    lv_obj_t* close_btn = lv_btn_create(card);
    lv_obj_set_size(close_btn, 160, 40);
    lv_obj_set_pos(close_btn, 280, 172);
    apply_control_button_style(close_btn, RED808_ERROR, false, 10);
    lv_obj_add_event_cb(close_btn, mixer_preset_modal_close_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* close_lbl = lv_label_create(close_btn);
    lv_label_set_text(close_lbl, "CERRAR");
    lv_obj_center(close_lbl);

    mixer_preset_modal_refresh();
}

static void create_volumes_screen(void) {
    scr_volumes = lv_obj_create(NULL);
    apply_screen_theme_bg(scr_volumes);
    lv_obj_clear_flag(scr_volumes, LV_OBJ_FLAG_SCROLLABLE);
    ui_create_header(scr_volumes);

    // Layout in actual LVGL canvas coordinates (landscape 1024×600)
    const int LW = LCD_H_RES;   // 1024 — full display width
    const int LH = LCD_V_RES;   // 600  — full display height

    lv_obj_t* title = lv_label_create(scr_volumes);
    lv_label_set_text(title, LV_SYMBOL_VOLUME_MAX "  MIXER");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(title, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(title, 60, 10);

    // Active pattern number — so it's clear which pattern the mixer is
    // shaping without having to switch to the sequencer to check.
    mix_pattern_lbl = lv_label_create(scr_volumes);
    lv_label_set_text_fmt(mix_pattern_lbl, "P%02d", p4.current_pattern + 1);
    lv_obj_set_style_text_font(mix_pattern_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(mix_pattern_lbl, RED808_ACCENT, 0);
    lv_obj_set_pos(mix_pattern_lbl, LCD_H_RES - 120, 10);

    // Global controls row: MAIN gain + BPM, restructured (narrower) to make
    // room for RANDOM MIX alongside them instead of hiding it in a submenu.
    const int global_y = 30;
    const int slider_w = 190;
    const int slider_gap = 40;
    const int label_w = 54;
    const int value_w = 44;
    const int block_w = label_w + slider_w + value_w;   // 288
    const int random_w = 190;
    const int random_h = 44;
    const int row_content_w = 2 * block_w + 2 * slider_gap + random_w;
    const int global_x = (LCD_H_RES - row_content_w) / 2;
    struct GlobalCtl { const char* name; int value; int max; lv_obj_t** slider; lv_obj_t** label; };
    GlobalCtl globals[] = {
        {"MAIN", p4.master_volume, Config::MAX_VOLUME, &mix_master_slider, &mix_master_lbl},
        {"BPM",  p4.bpm_int,       240,                &mix_bpm_slider,    &mix_bpm_lbl},
    };
    mix_seq_slider = NULL;
    mix_seq_lbl = NULL;
    mix_live_slider = NULL;
    mix_live_lbl = NULL;
    for (int i = 0; i < 2; i++) {
        int x = global_x + i * (block_w + slider_gap);
        lv_obj_t* name = lv_label_create(scr_volumes);
        lv_label_set_text(name, globals[i].name);
        lv_obj_set_style_text_font(name, &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_color(name, i == 1 ? RED808_ACCENT : RED808_CYAN, 0);
        lv_obj_set_pos(name, x, global_y);

        *globals[i].label = lv_label_create(scr_volumes);
        lv_label_set_text_fmt(*globals[i].label, "%d", globals[i].value);
        lv_obj_set_style_text_font(*globals[i].label, &lv_font_montserrat_28, 0);
        lv_obj_set_style_text_color(*globals[i].label, RED808_TEXT, 0);
        lv_obj_set_pos(*globals[i].label, x + label_w + slider_w + 10, global_y + 10);

        *globals[i].slider = lv_slider_create(scr_volumes);
        lv_obj_set_size(*globals[i].slider, slider_w, 18);
        lv_obj_set_pos(*globals[i].slider, x + label_w, global_y + 16);
        lv_slider_set_range(*globals[i].slider, i == 1 ? 40 : 0, globals[i].max);
        lv_slider_set_value(*globals[i].slider, globals[i].value, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(*globals[i].slider, lv_color_hex(0x2A2A2A), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(*globals[i].slider, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_bg_color(*globals[i].slider, i == 1 ? RED808_ACCENT : RED808_CYAN, LV_PART_INDICATOR);
        lv_obj_set_style_bg_opa(*globals[i].slider, LV_OPA_COVER, LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(*globals[i].slider, lv_color_white(), LV_PART_KNOB);
        lv_obj_set_style_bg_opa(*globals[i].slider, LV_OPA_COVER, LV_PART_KNOB);
        lv_obj_set_style_pad_all(*globals[i].slider, 10, LV_PART_KNOB);
        lv_obj_set_style_radius(*globals[i].slider, LV_RADIUS_CIRCLE, LV_PART_KNOB);
        int which = (i == 0 ? 0 : 3);
        lv_obj_add_event_cb(*globals[i].slider, mix_global_slider_cb, LV_EVENT_VALUE_CHANGED, (void*)(intptr_t)which);
        lv_obj_add_event_cb(*globals[i].slider, mix_global_slider_cb, LV_EVENT_RELEASED, (void*)(intptr_t)which);
        lv_obj_add_event_cb(*globals[i].slider, mix_global_slider_cb, LV_EVENT_PRESS_LOST, (void*)(intptr_t)which);
    }

    // RANDOM MIX: opens the AUTO MIX popup (cadence + on/off + apply now).
    // The button label keeps showing the last applied style.
    {
        int rx = global_x + 2 * block_w + 2 * slider_gap;
        s_mix_random_btn = lv_btn_create(scr_volumes);
        lv_obj_set_size(s_mix_random_btn, random_w, random_h);
        lv_obj_set_pos(s_mix_random_btn, rx, global_y + 8);
        lv_obj_add_event_cb(s_mix_random_btn, mix_random_modal_show, LV_EVENT_CLICKED, NULL);
        lv_obj_t* random_lbl = lv_label_create(s_mix_random_btn);
        lv_obj_set_style_text_font(random_lbl, &lv_font_montserrat_14, 0);
        lv_obj_center(random_lbl);
        s_mix_random_stop_badge = ui_create_auto_stop_badge(s_mix_random_btn, mix_random_stop_badge_cb);
        mix_random_btn_refresh();
    }

    // MIXER PRESETS — sits in the band freed up below by pushing y_top down
    // (was 100) rather than fighting for room in the centered MAIN/BPM/
    // RANDOM MIX row above, which already spans most of the screen width.
    {
        lv_obj_t* presets_btn = lv_btn_create(scr_volumes);
        lv_obj_set_size(presets_btn, 120, 26);
        lv_obj_set_pos(presets_btn, LCD_H_RES - 130, 74);
        apply_control_button_style(presets_btn, RED808_CYAN, false, 6);
        lv_obj_add_event_cb(presets_btn, mixer_preset_modal_show, LV_EVENT_CLICKED, NULL);
        lv_obj_t* presetsLabel = lv_label_create(presets_btn);
        lv_label_set_text(presetsLabel, "PRESETS");
        lv_obj_set_style_text_font(presetsLabel, &lv_font_montserrat_12, 0);
        lv_obj_center(presetsLabel);
    }
    mixer_presets_load_from_disk();

    // Single row of 16 strips filling the full display width
    int margin = 10;
    int gap    = 4;
    int total_w = LW - 2 * margin;
    int strip_w = (total_w - 15 * gap) / 16;   // ~56px each
    int y_top   = 134;
    int y_bottom = LH - 8;
    int strip_h  = y_bottom - y_top;            // ~508px
    int name_h   = 14;
    int value_h  = 14;
    int mute_h   = 42;
    int slider_h = strip_h - name_h - value_h - mute_h - 18;

    for (int i = 0; i < 16; i++) {
        int x = margin + i * (strip_w + gap);
        int cx = x + strip_w / 2;
        lv_color_t tc = lv_color_hex(theme_presets[ui_theme_index()].track_colors[i]);
        lv_color_t tc_hi = lv_color_mix(lv_color_white(), tc, 110);   // lit top of the fader

        // Strip panel background
        vol_strip_panels[i] = lv_obj_create(scr_volumes);
        lv_obj_set_size(vol_strip_panels[i], strip_w, strip_h);
        lv_obj_set_pos(vol_strip_panels[i], x, y_top);
        lv_obj_set_style_bg_color(vol_strip_panels[i], RED808_SURFACE, 0);
        lv_obj_set_style_bg_grad_color(vol_strip_panels[i], RED808_PANEL, 0);
        lv_obj_set_style_bg_grad_dir(vol_strip_panels[i], LV_GRAD_DIR_VER, 0);
        lv_obj_set_style_bg_opa(vol_strip_panels[i], LV_OPA_40, 0);
        lv_obj_set_style_radius(vol_strip_panels[i], 8, 0);
        lv_obj_set_style_border_width(vol_strip_panels[i], 1, 0);
        lv_obj_set_style_border_color(vol_strip_panels[i], tc, 0);
        lv_obj_set_style_border_opa(vol_strip_panels[i], LV_OPA_40, 0);
        lv_obj_set_style_shadow_width(vol_strip_panels[i], 0, 0);
        lv_obj_set_style_pad_all(vol_strip_panels[i], 0, 0);
        lv_obj_clear_flag(vol_strip_panels[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(vol_strip_panels[i], LV_OBJ_FLAG_CLICKABLE);

        // Track number + name ("01·BD")
        vol_name_labels[i] = lv_label_create(scr_volumes);
        lv_label_set_text_fmt(vol_name_labels[i], "%02d·%s", i + 1, trackNames[i]);
        lv_obj_set_style_text_font(vol_name_labels[i], &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(vol_name_labels[i], tc, 0);
        lv_obj_set_style_text_align(vol_name_labels[i], LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_pos(vol_name_labels[i], x, y_top + 2);
        lv_obj_set_width(vol_name_labels[i], strip_w);

        // Fader scale ticks at 25/50/75% — console-style orientation marks
        int y_sl = y_top + name_h + 4;
        for (int tk = 1; tk <= 3; tk++) {
            lv_obj_t* tick = lv_obj_create(scr_volumes);
            lv_obj_set_size(tick, strip_w - 18, 1);
            lv_obj_set_pos(tick, x + 9, y_sl + (slider_h * tk) / 4);
            lv_obj_set_style_bg_color(tick, RED808_TEXT_DIM, 0);
            lv_obj_set_style_bg_opa(tick, tk == 2 ? LV_OPA_40 : LV_OPA_20, 0);
            lv_obj_set_style_border_width(tick, 0, 0);
            lv_obj_clear_flag(tick, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_clear_flag(tick, LV_OBJ_FLAG_SCROLLABLE);
        }

        // Vertical fader — wide groove, gradient indicator that "lights up"
        vol_sliders[i] = lv_slider_create(scr_volumes);
        lv_obj_set_size(vol_sliders[i], 14, slider_h);
        lv_obj_set_pos(vol_sliders[i], cx - 7, y_sl);
        lv_slider_set_range(vol_sliders[i], 0, Config::MAX_VOLUME);
        lv_slider_set_value(vol_sliders[i], p4.track_volume[i], LV_ANIM_OFF);
        lv_obj_set_style_bg_color(vol_sliders[i], lv_color_hex(0x1C1C1C), LV_PART_MAIN);
        lv_obj_set_style_bg_grad_color(vol_sliders[i], lv_color_hex(0x303030), LV_PART_MAIN);
        lv_obj_set_style_bg_grad_dir(vol_sliders[i], LV_GRAD_DIR_HOR, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(vol_sliders[i], LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_width(vol_sliders[i], 1, LV_PART_MAIN);
        lv_obj_set_style_border_color(vol_sliders[i], lv_color_hex(0x000000), LV_PART_MAIN);
        lv_obj_set_style_radius(vol_sliders[i], 5, LV_PART_MAIN);
        lv_obj_set_style_bg_color(vol_sliders[i], tc_hi, LV_PART_INDICATOR);
        lv_obj_set_style_bg_grad_color(vol_sliders[i], tc, LV_PART_INDICATOR);
        lv_obj_set_style_bg_grad_dir(vol_sliders[i], LV_GRAD_DIR_VER, LV_PART_INDICATOR);
        lv_obj_set_style_bg_opa(vol_sliders[i], LV_OPA_COVER, LV_PART_INDICATOR);
        lv_obj_set_style_radius(vol_sliders[i], 5, LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(vol_sliders[i], lv_color_hex(0xF2F2F2), LV_PART_KNOB);
        lv_obj_set_style_bg_grad_color(vol_sliders[i], lv_color_hex(0xB8B8B8), LV_PART_KNOB);
        lv_obj_set_style_bg_grad_dir(vol_sliders[i], LV_GRAD_DIR_VER, LV_PART_KNOB);
        lv_obj_set_style_bg_opa(vol_sliders[i], LV_OPA_COVER, LV_PART_KNOB);
        lv_obj_set_style_pad_hor(vol_sliders[i], 7, LV_PART_KNOB);
        lv_obj_set_style_pad_ver(vol_sliders[i], 4, LV_PART_KNOB);
        lv_obj_set_style_radius(vol_sliders[i], 4, LV_PART_KNOB);
        lv_obj_set_style_shadow_color(vol_sliders[i], tc, LV_PART_KNOB);
        lv_obj_set_style_shadow_width(vol_sliders[i], 10, LV_PART_KNOB);
        lv_obj_set_style_shadow_opa(vol_sliders[i], LV_OPA_60, LV_PART_KNOB);
        lv_obj_set_style_border_color(vol_sliders[i], tc, LV_PART_KNOB);
        lv_obj_set_style_border_width(vol_sliders[i], 2, LV_PART_KNOB);
        lv_obj_add_event_cb(vol_sliders[i], vol_slider_cb, LV_EVENT_VALUE_CHANGED, (void*)(intptr_t)i);
        lv_obj_add_event_cb(vol_sliders[i], vol_slider_cb, LV_EVENT_RELEASED, (void*)(intptr_t)i);
        lv_obj_add_event_cb(vol_sliders[i], vol_slider_cb, LV_EVENT_PRESS_LOST, (void*)(intptr_t)i);

        // Color bar at bottom of slider
        lv_obj_t* color_bar = lv_obj_create(scr_volumes);
        lv_obj_set_size(color_bar, strip_w - 6, 3);
        lv_obj_set_pos(color_bar, x + 3, y_sl + slider_h + 2);
        lv_obj_set_style_bg_color(color_bar, tc, 0);
        lv_obj_set_style_bg_opa(color_bar, LV_OPA_80, 0);
        lv_obj_set_style_radius(color_bar, 1, 0);
        lv_obj_set_style_border_width(color_bar, 0, 0);
        lv_obj_clear_flag(color_bar, LV_OBJ_FLAG_SCROLLABLE);

        // Value label
        int y_val = y_sl + slider_h + 8;
        vol_labels[i] = lv_label_create(scr_volumes);
        lv_label_set_text_fmt(vol_labels[i], "%d", p4.track_volume[i]);
        lv_obj_set_style_text_font(vol_labels[i], &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(vol_labels[i], RED808_TEXT, 0);
        lv_obj_set_style_text_align(vol_labels[i], LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_pos(vol_labels[i], x, y_val);
        lv_obj_set_width(vol_labels[i], strip_w);

        // Mute button — real console "M": tap toggles the track (same
        // debounced callback as the sequencer column). update_volumes_screen
        // keeps recoloring it red/green through the vol_mute_dots pointer.
        vol_mute_dots[i] = lv_btn_create(scr_volumes);
        lv_obj_set_size(vol_mute_dots[i], strip_w - 6, 36);
        lv_obj_set_pos(vol_mute_dots[i], x + 4, y_val + value_h + 2);
        lv_obj_set_style_radius(vol_mute_dots[i], 5, 0);
        lv_obj_set_style_bg_color(vol_mute_dots[i], RED808_SUCCESS, 0);
        lv_obj_set_style_bg_opa(vol_mute_dots[i], LV_OPA_60, 0);
        lv_obj_set_style_shadow_color(vol_mute_dots[i], RED808_SUCCESS, 0);
        lv_obj_set_style_shadow_width(vol_mute_dots[i], 8, 0);
        lv_obj_set_style_shadow_opa(vol_mute_dots[i], LV_OPA_40, 0);
        lv_obj_set_style_border_width(vol_mute_dots[i], 0, 0);
        lv_obj_clear_flag(vol_mute_dots[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(vol_mute_dots[i], seq_mute_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
        lv_obj_t* mlbl = lv_label_create(vol_mute_dots[i]);
        lv_label_set_text(mlbl, "M");
        lv_obj_set_style_text_font(mlbl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(mlbl, lv_color_white(), 0);
        lv_obj_center(mlbl);

    }
}

static void update_volumes_screen(void) {
    static int  prev_volume[16] = {-1, -1, -1, -1, -1, -1, -1, -1,
                                   -1, -1, -1, -1, -1, -1, -1, -1};
    static bool prev_muted[16] = {};
    static bool prev_init = false;
    static int prev_master = -1;
    static int prev_bpm = -1;
    static int prev_pattern = -1;
    static uint32_t vol_gen = 0;
    if (vol_gen != s_ui_refresh_gen) {
        // Theme reload recreated the strips; redo the first full repaint.
        vol_gen = s_ui_refresh_gen;
        prev_init = false;
        prev_master = -1;
        prev_bpm = -1;
        prev_pattern = -1;
        for (int i = 0; i < 16; i++) {
            prev_volume[i] = -1;
        }
    }

    if (mix_master_slider && p4.master_volume != prev_master) {
        prev_master = p4.master_volume;
        lv_slider_set_value(mix_master_slider, p4.master_volume, LV_ANIM_OFF);
        if (mix_master_lbl) lv_label_set_text_fmt(mix_master_lbl, "%d", p4.master_volume);
    }
    if (mix_bpm_slider && p4.bpm_int != prev_bpm) {
        prev_bpm = p4.bpm_int;
        lv_slider_set_value(mix_bpm_slider, p4.bpm_int, LV_ANIM_OFF);
        if (mix_bpm_lbl) lv_label_set_text_fmt(mix_bpm_lbl, "%d", p4.bpm_int);
    }
    if (mix_pattern_lbl && p4.current_pattern != prev_pattern) {
        prev_pattern = p4.current_pattern;
        lv_label_set_text_fmt(mix_pattern_lbl, "P%02d", p4.current_pattern + 1);
    }

    for (int i = 0; i < 16; i++) {
        bool volume_changed = p4.track_volume[i] != prev_volume[i];
        bool mute_changed = !prev_init || p4.track_muted[i] != prev_muted[i];
        if (!volume_changed && !mute_changed) continue;

        prev_volume[i] = p4.track_volume[i];
        prev_muted[i] = p4.track_muted[i];

        if (volume_changed) {
            if (vol_sliders[i]) lv_slider_set_value(vol_sliders[i], p4.track_volume[i], LV_ANIM_OFF);
            if (vol_labels[i]) lv_label_set_text_fmt(vol_labels[i], "%d", p4.track_volume[i]);
        }
        if (vol_mute_dots[i]) {
            lv_obj_set_style_bg_color(vol_mute_dots[i],
                p4.track_muted[i] ? RED808_ERROR : RED808_SUCCESS, 0);
            lv_obj_set_style_shadow_color(vol_mute_dots[i],
                p4.track_muted[i] ? RED808_ERROR : RED808_SUCCESS, 0);
        }
        if (vol_strip_panels[i]) {
            lv_obj_set_style_border_color(vol_strip_panels[i],
                p4.track_muted[i] ? RED808_ERROR :
                lv_color_hex(theme_presets[ui_theme_index()].track_colors[i]), 0);
            lv_obj_set_style_border_opa(vol_strip_panels[i],
                p4.track_muted[i] ? LV_OPA_80 : LV_OPA_40, 0);
            lv_obj_set_style_bg_opa(vol_strip_panels[i],
                p4.track_muted[i] ? LV_OPA_20 : LV_OPA_40, 0);
        }
        if (vol_name_labels[i]) {
            lv_obj_set_style_text_color(vol_name_labels[i],
                p4.track_muted[i] ? RED808_TEXT_DIM :
                lv_color_hex(theme_presets[ui_theme_index()].track_colors[i]), 0);
        }
    }
    prev_init = true;

    // Heartbeat: cada strip late al ritmo del patrón. Cuando su track dispara
    // en el step actual, el borde sube a tope en su color, el fondo se
    // ilumina y ahora además se añade un glow (shadow) en el color de la
    // pista para que el golpe se note mucho más — antes solo subía opacidad
    // de borde/fondo y era poco visible; luego decae en unos frames. Al
    // llegar a cero se restaura el estilo base y se deja de tocar el strip.
    // Usa live_step_hit() (no p4.steps[][] directo) para que patrones de 2+
    // compases laten igual que el resto de pantallas sincronizadas por pads.
    static uint8_t beat_glow[16] = {};
    static int prev_beat_step = -1;
    bool anySoloMix = false;
    for (int i = 0; i < 16; ++i) anySoloMix |= p4.track_solo[i];
    int raw_step_now = control_current_step_raw();
    if (p4.is_playing && raw_step_now != prev_beat_step) {
        prev_beat_step = raw_step_now;
        for (int i = 0; i < 16; i++) {
            const bool audible = !p4.track_muted[i]
                && (!anySoloMix || p4.track_solo[i]);
            if (audible && live_step_hit(i)) beat_glow[i] = 255;
        }
    }
    if (!p4.is_playing) prev_beat_step = -1;
    for (int i = 0; i < 16; i++) {
        if (!vol_strip_panels[i] || beat_glow[i] == 0) continue;
        if (p4.track_muted[i] || (anySoloMix && !p4.track_solo[i])) {
            beat_glow[i] = 0;
            lv_obj_set_style_shadow_width(vol_strip_panels[i], 0, 0);
            continue;  // mute pinta su propio estado
        }
        int next = (int)beat_glow[i] - 40;
        if (next < 0) next = 0;
        beat_glow[i] = (uint8_t)next;
        lv_color_t tc = lv_color_hex(theme_presets[ui_theme_index()].track_colors[i]);
        lv_obj_set_style_border_opa(vol_strip_panels[i],
            (lv_opa_t)max((int)LV_OPA_40, next), 0);
        lv_obj_set_style_bg_opa(vol_strip_panels[i],
            (lv_opa_t)(LV_OPA_40 + ((next * 60) >> 8)), 0);
        lv_coord_t shadow_w = (lv_coord_t)(4 + (next * 22) / 255);
        lv_opa_t shadow_opa = (lv_opa_t)((next * 220) / 255);
        lv_obj_set_style_shadow_color(vol_strip_panels[i], tc, 0);
        lv_obj_set_style_shadow_width(vol_strip_panels[i], next > 0 ? shadow_w : 0, 0);
        lv_obj_set_style_shadow_opa(vol_strip_panels[i], shadow_opa, 0);
        lv_obj_set_style_shadow_spread(vol_strip_panels[i], next > 0 ? 2 : 0, 0);
    }
}

// =============================================================================
// SD CARD SCREEN — browse P4 local SD card or P4 internal MEM MIDI storage
// =============================================================================

// SD screen widgets
static lv_obj_t* sd_left_panel  = NULL;
static lv_obj_t* sd_right_panel = NULL;
static lv_obj_t* sd_status_lbl  = NULL;
static lv_obj_t* sd_path_lbl    = NULL;
static lv_obj_t* sd_file_list   = NULL;
static lv_obj_t* sd_selected_lbl = NULL;
static lv_obj_t* sd_assign_lbl = NULL;
static lv_obj_t* sd_pad_btns[16] = {};
// Non-destructive per-pad trim window + captured waveform for the 16 main
// pads (see the PAD SAMPLE TRIM EDITOR section below, near
// create_sdcard_screen, for the editor UI that reads/writes these).
static uint8_t s_pad_trim_start_pct[16] = {};
static uint8_t s_pad_trim_end_pct[16] = {
    100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100
};
static uint8_t s_pad_wave_count[16] = {};
static int8_t  s_pad_wave_max[16][96] = {};
static int8_t  s_pad_wave_min[16][96] = {};
static lv_obj_t* sd_load_btn    = NULL;
static lv_obj_t* sd_load_lbl    = NULL;
static lv_obj_t* sd_preview_btn = NULL;
static lv_obj_t* sd_preview_lbl = NULL;
// MIDI section
static lv_obj_t* sd_wav_section       = NULL;
static lv_obj_t* sd_midi_section      = NULL;
static lv_obj_t* sd_midi_pat_btns[10] = {};
static lv_obj_t* sd_midi_load_btn     = NULL;
static lv_obj_t* sd_midi_song_btn     = NULL;
static lv_obj_t* sd_midi_info_lbl     = NULL;
static lv_obj_t* sd_midi_status_lbl   = NULL;
static int        sd_midi_target_slot  = 0;   // default: P01
static bool       sd_is_midi_mode      = false;
// 0 = PRO (merge all channels, dense drum sequencer feel)
// 1 = STD (GM drum channel 9 only, closer to a standard MIDI player)
static int        sd_midi_import_mode  = 0;
static lv_obj_t*  sd_midi_mode_pro_btn = NULL;
static lv_obj_t*  sd_midi_mode_std_btn = NULL;

// Forward declarations
static void sd_refresh_ui(void);
static void sd_switch_panel_mode(bool midi_mode);
static void sd_midi_pat_btn_cb(lv_event_t* e);
static void sd_midi_load_btn_cb(lv_event_t* e);
static void sd_midi_song_btn_cb(lv_event_t* e);
static void sd_midi_begin_load(bool song_mode);
static void sd_refresh_source(void);
static void show_midi_load_summary(const char* title, int slot,
                                   int steps, int raw_len, float bpm,
                                   int tracks_used);

// ── Sources: 0 = P4 SD, 1 = MEM MIDI, 2 = Daisy's authoritative SD ──────────
static int        sd_source            = 0;
static lv_obj_t*  sd_src_sd_btn        = NULL;
static lv_obj_t*  sd_src_mem_btn       = NULL;
static lv_obj_t*  sd_src_daisy_btn     = NULL;
static char       sd_mem_files[64][48] = {};
static int        sd_mem_count         = 0;
static int        sd_mem_selected      = -1;
static bool       sd_local_mounted     = false;
static char       sd_daisy_folder[32]  = {};
static char       sd_daisy_selected[32] = {};
static uint32_t   sd_daisy_seen_revision = 0;
static bool       sd_daisy_waiting = false;

static void sd_daisy_request_root(void) {
    sd_daisy_folder[0] = '\0';
    sd_daisy_selected[0] = '\0';
    sd_daisy_waiting = true;
    if (!daisyUsb.send(CMD_SD_STATUS) || !daisyUsb.send(CMD_SD_LIST_FOLDERS)) {
        sd_daisy_waiting = false;
        ui_show_toast("Daisy USB no disponible", RED808_WARNING);
    }
    p4sd.needs_refresh.store(true, std::memory_order_release);
}

static void sd_daisy_request_folder(const char* folder) {
    SdListFilesPayload payload = {};
    strncpy(payload.folderName, folder ? folder : "", sizeof(payload.folderName) - 1);
    strncpy(sd_daisy_folder, payload.folderName, sizeof(sd_daisy_folder) - 1);
    sd_daisy_folder[sizeof(sd_daisy_folder) - 1] = '\0';
    sd_daisy_selected[0] = '\0';
    sd_daisy_waiting = true;
    if (!daisyUsb.send(CMD_SD_LIST_FILES, &payload, sizeof(payload))) {
        sd_daisy_waiting = false;
        ui_show_toast("No se pudo consultar Daisy SD", RED808_WARNING);
    }
    p4sd.needs_refresh.store(true, std::memory_order_release);
}

static const char* sd_basename(const char* path) {
    const char* base = strrchr(path, '/');
    return base ? base + 1 : path;
}

static bool sd_local_try_mount(void) {
    if (sd_local_mounted) return true;
    if (SD_MMC.begin("/sdcard", false, false)) {
        sd_local_mounted = true;
        return true;
    }
    SD_MMC.end();
    if (SD_MMC.begin("/sdcard", true, false)) {
        sd_local_mounted = true;
        return true;
    }
    sd_local_mounted = false;
    return false;
}

static void sd_local_reset_selection(void) {
    p4sd.selected_file[0] = '\0';
    p4sd.selected_is_midi = false;
    p4sd.midi_load_result = -2;
}

static bool sd_upload_in_flight(void);   // defined with the upload worker below
static bool sd_midi_load_in_flight(void); // defined with the MIDI worker below

// Worker context (or LVGL-context fallback). Mount + directory scan:
// SD_MMC.begin() with no card inserted blocks ~1 s across its two attempts,
// and this used to run inside ui_navigate_to(), freezing the whole UI on
// every entry to the SD screen. entry_count is published once at the end so
// a concurrent sd_refresh_ui() never renders half-written entries.
static void sd_local_scan_blocking(void) {
    if (!sd_local_try_mount()) {
        p4sd.mounted = false;
        if (p4sd.path[0] == '\0') strcpy(p4sd.path, "/");
        p4sd.list_complete = true;
        p4sd.needs_refresh = true;
        return;
    }

    p4sd.mounted = true;
    if (p4sd.path[0] == '\0') strcpy(p4sd.path, "/");

    File dir = SD_MMC.open(p4sd.path);
    if (!dir || !dir.isDirectory()) {
        strcpy(p4sd.path, "/");
        dir = SD_MMC.open("/");
    }
    if (!dir || !dir.isDirectory()) {
        p4sd.mounted = false;
        p4sd.list_complete = true;
        p4sd.needs_refresh = true;
        return;
    }

    int count = 0;
    File entry = dir.openNextFile();
    while (entry && count < P4_SD_MAX_ENTRIES) {
        const char* base = sd_basename(entry.name());
        bool is_dir = entry.isDirectory();
        if (base[0] == '\0' || base[0] == '.') {
            entry.close();
            entry = dir.openNextFile();
            continue;
        }
        bool is_midi = false;
        if (!is_dir) {
            size_t nlen = strlen(base);
            bool is_wav = (nlen > 4 && strcasecmp(base + nlen - 4, ".wav") == 0);
            is_midi = (nlen > 4 && strcasecmp(base + nlen - 4, ".mid") == 0);
            if (!is_wav && !is_midi) {
                entry.close();
                entry = dir.openNextFile();
                continue;
            }
        }
        P4SdEntry& out = p4sd.entries[count++];
        strncpy(out.name, base, sizeof(out.name) - 1);
        out.name[sizeof(out.name) - 1] = '\0';
        out.is_dir = is_dir;
        out.is_midi = is_midi;
        entry.close();
        entry = dir.openNextFile();
    }
    dir.close();
    // Stable, predictable browser: directories first, then alphabetical.
    for (int i = 1; i < count; i++) {
        P4SdEntry key = p4sd.entries[i];
        int j = i - 1;
        while (j >= 0) {
            bool key_before = (key.is_dir != p4sd.entries[j].is_dir)
                ? key.is_dir
                : (strcasecmp(key.name, p4sd.entries[j].name) < 0);
            if (!key_before) break;
            p4sd.entries[j + 1] = p4sd.entries[j];
            j--;
        }
        p4sd.entries[j + 1] = key;
    }
    p4sd.entry_count = count;
    p4sd.list_complete = true;
    p4sd.needs_refresh = true;
}

// 0 = idle, 1 = scanning
static std::atomic<uint8_t> s_sd_scan_state{0};

static void sd_scan_task(void* arg) {
    (void)arg;
    sd_local_scan_blocking();
    s_sd_scan_state.store(0, std::memory_order_release);
    vTaskDelete(NULL);
}

// LVGL task: kick the scan to a one-shot worker; sd_refresh_ui() shows a
// scanning placeholder until list_complete flips and needs_refresh repaints.
static void sd_local_refresh_listing(bool reset_selection) {
    if (reset_selection) sd_local_reset_selection();
    if (s_sd_scan_state.load(std::memory_order_acquire) != 0) return;  // already scanning
    if (sd_upload_in_flight() || sd_midi_load_in_flight()) {
        // The upload worker owns SD_MMC right now — don't scan concurrently.
        ui_show_toast("Almacenamiento ocupado...", RED808_WARNING);
        return;
    }
    p4sd.entry_count = 0;
    p4sd.list_complete = false;
    p4sd.needs_refresh = true;   // paint the scanning placeholder immediately

    s_sd_scan_state.store(1, std::memory_order_release);
    if (xTaskCreatePinnedToCore(sd_scan_task, "sdscan", 6144, NULL, 1, NULL, 1) != pdPASS) {
        // Never fall back to a blocking mount/scan on the LVGL task.
        s_sd_scan_state.store(0, std::memory_order_release);
        p4sd.list_complete = true;
        p4sd.needs_refresh.store(true, std::memory_order_release);
        ui_show_toast("No se pudo iniciar el escaneo SD", RED808_WARNING);
    }
}

static void sd_local_select(int idx) {
    if (idx < 0 || idx >= p4sd.entry_count) return;
    const P4SdEntry& entry = p4sd.entries[idx];
    if (entry.is_dir) {
        char next_path[128];
        if (strcmp(p4sd.path, "/") == 0) {
            snprintf(next_path, sizeof(next_path), "/%s", entry.name);
        } else {
            snprintf(next_path, sizeof(next_path), "%s/%s", p4sd.path, entry.name);
        }
        strncpy(p4sd.path, next_path, sizeof(p4sd.path) - 1);
        p4sd.path[sizeof(p4sd.path) - 1] = '\0';
        sd_local_refresh_listing(true);
    } else {
        strncpy(p4sd.selected_file, entry.name, sizeof(p4sd.selected_file) - 1);
        p4sd.selected_file[sizeof(p4sd.selected_file) - 1] = '\0';
        p4sd.selected_is_midi = entry.is_midi;
        p4sd.midi_load_result = -2;
        p4sd.needs_refresh = true;
    }
}

static void sd_local_back(void) {
    if (strcmp(p4sd.path, "/") == 0 || p4sd.path[0] == '\0') {
        sd_local_refresh_listing(true);
        return;
    }
    char* last = strrchr(p4sd.path, '/');
    if (last && last != p4sd.path) *last = '\0';
    else strcpy(p4sd.path, "/");
    sd_local_refresh_listing(true);
}

static void sd_mem_refresh_list(void) {
    sd_mem_count = mem_midi::list_midi_files("/mid", sd_mem_files, 64);
    if (sd_mem_selected >= sd_mem_count) sd_mem_selected = -1;
}

static void sd_refresh_source(void) {
    if (s_sd_for_xtra) {
        sd_source = 0;
    }
    if (sd_source == 1) {
        sd_mem_refresh_list();
        sd_switch_panel_mode(true);
    } else if (sd_source == 2) {
        sd_switch_panel_mode(false);
        sd_daisy_request_root();
    } else {
        if (p4sd.path[0] == '\0') strcpy(p4sd.path, "/");
        sd_local_refresh_listing(false);
    }
    sd_refresh_ui();
}

static void sd_mem_file_btn_cb(lv_event_t* e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (idx < 0 || idx >= sd_mem_count) return;
    sd_mem_selected = idx;
    if (sd_midi_info_lbl) lv_label_set_text(sd_midi_info_lbl, sd_mem_files[idx]);
    if (sd_midi_load_btn) lv_obj_clear_state(sd_midi_load_btn, LV_STATE_DISABLED);
    if (sd_midi_song_btn) lv_obj_clear_state(sd_midi_song_btn, LV_STATE_DISABLED);
    if (sd_midi_song_btn) lv_obj_clear_state(sd_midi_song_btn, LV_STATE_DISABLED);
    if (sd_midi_status_lbl) lv_label_set_text(sd_midi_status_lbl, "");
    sd_refresh_ui();
}

static void sd_source_btn_cb(lv_event_t* e) {
    if (sd_midi_load_in_flight()) {
        ui_show_toast("Carga MIDI en curso...", RED808_WARNING);
        return;
    }
    int src = (int)(intptr_t)lv_event_get_user_data(e);
    if (s_sd_for_xtra && src != 0) {
        ui_show_toast("XTRA usa solo SD WAV", RED808_WARNING);
        return;
    }
    if (src == sd_source) {
        if (src == 2) sd_daisy_request_root();
        return;
    }
    sd_source = src;
    sd_mem_selected = -1;
    if (sd_midi_info_lbl) lv_label_set_text(sd_midi_info_lbl, "");
    if (sd_midi_status_lbl) lv_label_set_text(sd_midi_status_lbl, "");
    if (sd_midi_load_btn) lv_obj_add_state(sd_midi_load_btn, LV_STATE_DISABLED);
    if (sd_midi_song_btn) lv_obj_add_state(sd_midi_song_btn, LV_STATE_DISABLED);
    if (src == 1) {
        sd_mem_refresh_list();
        sd_switch_panel_mode(true);   // MEM is MIDI-only
    } else if (src == 2) {
        sd_switch_panel_mode(false);
        sd_daisy_request_root();
    } else {
        if (p4sd.path[0] == '\0') strcpy(p4sd.path, "/");
        sd_local_refresh_listing(true);
        sd_switch_panel_mode(false);  // default local SD view
    }
    if (sd_src_sd_btn) {
        lv_obj_set_style_bg_color(sd_src_sd_btn,
            src == 0 ? RED808_CYAN : lv_color_hex(0x1A2A3A), 0);
    }
    if (sd_src_mem_btn) {
        lv_obj_set_style_bg_color(sd_src_mem_btn,
            src == 1 ? RED808_WARNING : lv_color_hex(0x1A2A3A), 0);
    }
    if (sd_src_daisy_btn) {
        lv_obj_set_style_bg_color(sd_src_daisy_btn,
            src == 2 ? RED808_SUCCESS : lv_color_hex(0x1A2A3A), 0);
    }
    sd_refresh_ui();
}

static void sd_switch_panel_mode(bool midi_mode) {
    sd_is_midi_mode = midi_mode;
    if (sd_wav_section) {
        if (midi_mode) lv_obj_add_flag(sd_wav_section, LV_OBJ_FLAG_HIDDEN);
        else           lv_obj_clear_flag(sd_wav_section, LV_OBJ_FLAG_HIDDEN);
    }
    if (sd_midi_section) {
        if (midi_mode) lv_obj_clear_flag(sd_midi_section, LV_OBJ_FLAG_HIDDEN);
        else           lv_obj_add_flag(sd_midi_section, LV_OBJ_FLAG_HIDDEN);
    }
}

static void seq_open_midi_library(void) {
    s_sd_for_xtra = false;
    sd_source = 0;  // P4 SD is the authoritative MIDI/song source.
    sd_mem_selected = -1;
    sd_local_reset_selection();
    if (p4sd.path[0] == '\0') strcpy(p4sd.path, "/");
    ui_navigate_to(9);
    sd_switch_panel_mode(true);
    if (sd_src_sd_btn)
        lv_obj_set_style_bg_color(sd_src_sd_btn, RED808_CYAN, 0);
    if (sd_src_mem_btn)
        lv_obj_set_style_bg_color(sd_src_mem_btn, lv_color_hex(0x1A2A3A), 0);
    sd_local_refresh_listing(false);
    sd_refresh_ui();
}

static void sd_file_btn_cb(lv_event_t* e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (idx < 0 || idx >= p4sd.entry_count) return;
    const P4SdEntry& entry = p4sd.entries[idx];
    if (!entry.is_dir) {
        // Track selection type immediately (before S3 response arrives)
        p4sd.selected_is_midi = entry.is_midi;
        sd_switch_panel_mode(entry.is_midi);
        if (entry.is_midi && sd_midi_info_lbl) {
            lv_label_set_text(sd_midi_info_lbl, entry.name);
        }
        if (entry.is_midi && sd_midi_load_btn) {
            lv_obj_clear_state(sd_midi_load_btn, LV_STATE_DISABLED);
        }
        if (entry.is_midi && sd_midi_song_btn) {
            lv_obj_clear_state(sd_midi_song_btn, LV_STATE_DISABLED);
        }
        if (entry.is_midi && sd_midi_status_lbl) {
            lv_label_set_text(sd_midi_status_lbl, "");
        }
        if (entry.is_midi) {
            // Reset any stale load result when picking a new file
            p4sd.midi_load_result = -2;
        }
    }
    sd_local_select(idx);
}

static void sd_daisy_file_btn_cb(lv_event_t* e) {
    const int idx = (int)(intptr_t)lv_event_get_user_data(e);
    const auto& state = daisyUsb.state();
    if (sd_daisy_folder[0] == '\0') {
        if (idx < 0 || idx >= state.daisy_sd_folder_count) return;
        sd_daisy_request_folder(state.daisy_sd_folders[idx]);
        return;
    }
    if (idx < 0 || idx >= state.daisy_sd_file_count) return;
    strncpy(sd_daisy_selected, state.daisy_sd_files[idx], sizeof(sd_daisy_selected) - 1);
    sd_daisy_selected[sizeof(sd_daisy_selected) - 1] = '\0';
    ui_show_toast("WAV visible en la SD de Daisy", RED808_SUCCESS);
    p4sd.needs_refresh.store(true, std::memory_order_release);
}

static void sd_back_btn_cb(lv_event_t* e) {
    (void)e;
    if (sd_source == 2) {
        sd_daisy_request_root();
        return;
    }
    p4sd.selected_is_midi = false;
    sd_switch_panel_mode(false);
    sd_local_back();
}

static void sd_pad_btn_cb(lv_event_t* e) {
    if (s_sd_for_xtra) return;
    int pad = (int)(intptr_t)lv_event_get_user_data(e);
    if (pad < 0 || pad >= 16) return;
    p4sd.selected_pad = pad;
    // Update pad button highlights
    for (int i = 0; i < 16; i++) {
        if (sd_pad_btns[i]) {
            lv_obj_set_style_bg_color(sd_pad_btns[i],
                i == pad ? RED808_ACCENT : lv_color_hex(0x222233), 0);
        }
    }
}

static void sd_midi_pat_btn_cb(lv_event_t* e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    if (slot < 0 || slot > 5) return;
    sd_midi_target_slot = slot;
    for (int i = 0; i < 6; i++) {
        if (!sd_midi_pat_btns[i]) continue;
        bool sel = (i == slot);
        lv_obj_set_style_bg_color(sd_midi_pat_btns[i],
            sel ? RED808_ACCENT : lv_color_hex(0x1A2A3A), 0);
        lv_obj_set_style_border_color(sd_midi_pat_btns[i],
            sel ? RED808_CYAN : lv_color_hex(0x334455), 0);
    }
}

// ── MIDI load summary modal ─────────────────────────────────────────────────
// Shown after a successful MEM-MIDI load. Displays filename, BPM, step count
// and unique tracks. OK button dismisses and navigates to the sequencer.
static lv_obj_t* midi_summary_modal = NULL;
static lv_obj_t* midi_song_confirm_modal = NULL;

static void midi_song_confirm_close(void) {
    if (!midi_song_confirm_modal) return;
    lv_obj_del(midi_song_confirm_modal);
    midi_song_confirm_modal = NULL;
}

static void midi_song_confirm_cb(lv_event_t* e) {
    const bool accept = (bool)(intptr_t)lv_event_get_user_data(e);
    midi_song_confirm_close();
    if (accept) sd_midi_begin_load(true);
}

static void sd_midi_song_btn_cb(lv_event_t* e) {
    (void)e;
    if (sd_midi_load_in_flight() || midi_song_confirm_modal) return;

    midi_song_confirm_modal = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(midi_song_confirm_modal);
    lv_obj_set_size(midi_song_confirm_modal, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(midi_song_confirm_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(midi_song_confirm_modal, LV_OPA_60, 0);
    lv_obj_add_flag(midi_song_confirm_modal, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(midi_song_confirm_modal, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* card = lv_obj_create(midi_song_confirm_modal);
    lv_obj_set_size(card, 540, 300);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_border_color(card, RED808_WARNING, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_radius(card, 12, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* heading = lv_label_create(card);
    lv_label_set_text(heading, LV_SYMBOL_WARNING "  IMPORT FULL MIDI SONG");
    lv_obj_set_style_text_font(heading, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(heading, RED808_WARNING, 0);
    lv_obj_align(heading, LV_ALIGN_TOP_MID, 0, 10);

    lv_obj_t* copy = lv_label_create(card);
    lv_label_set_text(copy,
        "La cancion se convertira en escenas P101-P120.\n"
        "Esos patrones de usuario se sustituiran y guardaran.\n\n"
        "Se conservan velocity, silencios y compases repetidos.");
    lv_obj_set_width(copy, 480);
    lv_obj_set_style_text_align(copy, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(copy, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(copy, RED808_TEXT, 0);
    lv_obj_align(copy, LV_ALIGN_TOP_MID, 0, 62);

    lv_obj_t* cancel = lv_btn_create(card);
    lv_obj_set_size(cancel, 190, 52);
    lv_obj_align(cancel, LV_ALIGN_BOTTOM_LEFT, 22, -18);
    apply_control_button_style(cancel, RED808_BORDER, false, 8);
    lv_obj_add_event_cb(cancel, midi_song_confirm_cb, LV_EVENT_CLICKED,
                        (void*)(intptr_t)false);
    lv_obj_t* cancel_label = lv_label_create(cancel);
    lv_label_set_text(cancel_label, LV_SYMBOL_CLOSE "  CANCEL");
    lv_obj_set_style_text_color(cancel_label, RED808_TEXT, 0);
    lv_obj_center(cancel_label);

    lv_obj_t* import_btn = lv_btn_create(card);
    lv_obj_set_size(import_btn, 250, 52);
    lv_obj_align(import_btn, LV_ALIGN_BOTTOM_RIGHT, -22, -18);
    apply_control_button_style(import_btn, RED808_WARNING, true, 8);
    lv_obj_add_event_cb(import_btn, midi_song_confirm_cb, LV_EVENT_CLICKED,
                        (void*)(intptr_t)true);
    lv_obj_t* import_label = lv_label_create(import_btn);
    lv_label_set_text(import_label, LV_SYMBOL_DOWNLOAD "  IMPORT & SAVE");
    lv_obj_set_style_text_color(import_label, RED808_BG, 0);
    lv_obj_center(import_label);
}

static void midi_summary_ok_cb(lv_event_t* e) {
    (void)e;
    if (midi_summary_modal) {
        lv_obj_del(midi_summary_modal);
        midi_summary_modal = NULL;
    }
    ui_navigate_to(3);   // screen 3 = SEQUENCER
}

static void show_midi_load_summary(const char* title, int slot,
                                   int steps, int raw_len, float bpm,
                                   int tracks_used) {
    // Remember last summary so the sequencer info button can re-open it.
    snprintf(seq_last_midi_name, sizeof(seq_last_midi_name), "%s",
             title ? title : "(unknown)");
    seq_last_midi_slot    = slot;
    seq_last_midi_steps   = steps;
    seq_last_midi_raw_len = raw_len;
    seq_last_midi_bpm     = bpm;
    seq_last_midi_tracks  = tracks_used;
    seq_last_midi_valid   = true;

    if (midi_summary_modal) {
        lv_obj_del(midi_summary_modal);
        midi_summary_modal = NULL;
    }
    lv_obj_t* scr = lv_layer_top();
    midi_summary_modal = lv_obj_create(scr);
    lv_obj_remove_style_all(midi_summary_modal);
    lv_obj_set_size(midi_summary_modal, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(midi_summary_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(midi_summary_modal, LV_OPA_60, 0);
    lv_obj_add_flag(midi_summary_modal, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(midi_summary_modal, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* card = lv_obj_create(midi_summary_modal);
    lv_obj_set_size(card, 520, 320);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_border_color(card, RED808_ACCENT, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_radius(card, 10, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* t = lv_label_create(card);
    lv_label_set_text(t, LV_SYMBOL_OK "  MIDI Loaded");
    lv_obj_set_style_text_color(t, RED808_SUCCESS, 0);
    lv_obj_set_style_text_font(t, &lv_font_montserrat_24, 0);
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 10);

    lv_obj_t* fn = lv_label_create(card);
    char fnbuf[96];
    snprintf(fnbuf, sizeof(fnbuf), "File: %s", title ? title : "(unknown)");
    lv_label_set_text(fn, fnbuf);
    lv_obj_set_style_text_color(fn, RED808_TEXT, 0);
    lv_obj_set_style_text_font(fn, &lv_font_montserrat_18, 0);
    lv_obj_align(fn, LV_ALIGN_TOP_LEFT, 20, 58);

    lv_obj_t* l1 = lv_label_create(card);
    char b1[192];
    snprintf(b1, sizeof(b1),
        "Pattern slot:  P%02d\n"
        "Tempo:         %.1f BPM\n"
        "Steps (bar1):  %d\n"
        "Raw length:    %d steps (%d bars)\n"
        "Pages:         manual P1-P4\n"
        "Tracks used:   %d / 16",
        slot, bpm, steps, raw_len, raw_len / 16, tracks_used);
    lv_label_set_text(l1, b1);
    lv_obj_set_style_text_color(l1, RED808_TEXT, 0);
    lv_obj_set_style_text_font(l1, &lv_font_montserrat_18, 0);
    lv_obj_align(l1, LV_ALIGN_TOP_LEFT, 20, 96);

    lv_obj_t* ok = lv_btn_create(card);
    lv_obj_set_size(ok, 240, 56);
    lv_obj_align(ok, LV_ALIGN_BOTTOM_MID, 0, -18);
    lv_obj_set_style_bg_color(ok, RED808_ACCENT, 0);
    lv_obj_set_style_radius(ok, 8, 0);
    lv_obj_add_event_cb(ok, midi_summary_ok_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* okl = lv_label_create(ok);
    lv_label_set_text(okl, LV_SYMBOL_OK "  Go to Sequencer");
    lv_obj_set_style_text_color(okl, lv_color_white(), 0);
    lv_obj_set_style_text_font(okl, &lv_font_montserrat_18, 0);
    lv_obj_center(okl);
}

static void show_midi_song_summary(const char* title,
                                   const mem_midi::MidiSongData& song) {
    snprintf(seq_last_midi_name, sizeof(seq_last_midi_name), "%s",
             title ? title : "(unknown)");
    seq_last_midi_slot = 101;
    seq_last_midi_steps = (int)song.hits;
    seq_last_midi_raw_len = (int)song.imported_bars * 16;
    seq_last_midi_bpm = song.bpm;
    seq_last_midi_tracks = song.tracks_used;
    seq_last_midi_valid = true;

    if (midi_summary_modal) {
        lv_obj_del(midi_summary_modal);
        midi_summary_modal = NULL;
    }
    midi_summary_modal = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(midi_summary_modal);
    lv_obj_set_size(midi_summary_modal, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(midi_summary_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(midi_summary_modal, LV_OPA_60, 0);
    lv_obj_add_flag(midi_summary_modal, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(midi_summary_modal, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* card = lv_obj_create(midi_summary_modal);
    lv_obj_set_size(card, 590, 392);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_border_color(card,
        song.truncated ? RED808_WARNING : RED808_SUCCESS, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_radius(card, 12, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* heading = lv_label_create(card);
    lv_label_set_text(heading, song.truncated
        ? LV_SYMBOL_WARNING "  MIDI SONG · PARTIAL"
        : LV_SYMBOL_OK "  MIDI SONG READY");
    lv_obj_set_style_text_color(heading,
        song.truncated ? RED808_WARNING : RED808_SUCCESS, 0);
    lv_obj_set_style_text_font(heading, &lv_font_montserrat_24, 0);
    lv_obj_align(heading, LV_ALIGN_TOP_MID, 0, 8);

    lv_obj_t* filename = lv_label_create(card);
    lv_label_set_text_fmt(filename, "File: %s", title ? title : "(unknown)");
    lv_obj_set_width(filename, 540);
    lv_label_set_long_mode(filename, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_color(filename, RED808_TEXT, 0);
    lv_obj_set_style_text_font(filename, &lv_font_montserrat_16, 0);
    lv_obj_align(filename, LV_ALIGN_TOP_LEFT, 10, 52);

    lv_obj_t* details = lv_label_create(card);
    char info[420];
    char tempo_text[28];
    if (song.bpm > 0.0f)
        snprintf(tempo_text, sizeof(tempo_text), "%.1f BPM", song.bpm);
    else
        snprintf(tempo_text, sizeof(tempo_text), "not defined");
    const int last_slot = 100 + song.pattern_count;
    snprintf(info, sizeof(info),
        "Tempo original:   %s\n"
        "Arrangement:      %u / %u bars\n"
        "Resident scenes:  %u / %d  -> P101-P%03d\n"
        "Song sections:    %u / %d\n"
        "Hits / tracks:    %lu / %u\n"
        "Dynamics:         MIDI velocity preserved\n"
        "Tempo events:     %u%s\n"
        "Storage:          %s",
        tempo_text,
        (unsigned)song.imported_bars, (unsigned)song.total_bars,
        (unsigned)song.pattern_count, mem_midi::MIDI_SONG_MAX_PATTERNS,
        last_slot, (unsigned)song.chain_count, mem_midi::MIDI_SONG_MAX_CHAIN,
        (unsigned long)song.hits, (unsigned)song.tracks_used,
        (unsigned)song.tempo_events,
        song.tempo_events > 1 ? " (initial tempo is reference)" : "",
        control_midi_song_persisted() ? "saved in user patterns" : "RAM only · check SPIFFS");
    lv_label_set_text(details, info);
    lv_obj_set_style_text_color(details, RED808_TEXT, 0);
    lv_obj_set_style_text_font(details, &lv_font_montserrat_16, 0);
    lv_obj_align(details, LV_ALIGN_TOP_LEFT, 10, 82);

    lv_obj_t* ok = lv_btn_create(card);
    lv_obj_set_size(ok, 330, 54);
    lv_obj_align(ok, LV_ALIGN_BOTTOM_MID, 0, -14);
    apply_control_button_style(ok, RED808_CYAN, true, 8);
    lv_obj_set_style_bg_color(ok, RED808_ACCENT, 0);
    lv_obj_add_event_cb(ok, midi_summary_ok_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* label = lv_label_create(ok);
    lv_label_set_text(label, LV_SYMBOL_PLAY "  SEQUENCER · PLAY SONG");
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_18, 0);
    lv_obj_center(label);
}

// ── Async MIDI parser ───────────────────────────────────────────────────────
// Parsing can legitimately take seconds on a dense or malformed SMF. Keep it
// off the LVGL task and publish the complete grid only after the worker exits.
struct SdMidiLoadJob {
    uint8_t source;              // 0 = SD_MMC, 1 = SPIFFS
    int mode;
    int target_slot;
    bool song_mode;
    char path[192];
    char display_name[64];
    char parsed_name[16];
    bool grid[16][64];
    int steps_found;
    int raw_len;
    float bpm;
    bool ok;
    bool installed;
    mem_midi::MidiSongData song;
};
static SdMidiLoadJob s_sd_midi_job;
// 0 = idle, 1 = parsing, 2 = completed and waiting for LVGL consumption.
static std::atomic<uint8_t> s_sd_midi_state{0};

static bool sd_midi_load_in_flight(void) {
    return s_sd_midi_state.load(std::memory_order_acquire) != 0;
}

static void sd_midi_load_task(void* arg) {
    (void)arg;
    SdMidiLoadJob& job = s_sd_midi_job;
    job.steps_found = 0;
    job.raw_len = 0;
    job.bpm = 0.0f;
    job.installed = false;
    if (job.song_mode) {
        job.ok = job.source == 1
            ? mem_midi::load_song(job.path, &job.song, job.mode)
            : mem_midi::load_song_from_fs(SD_MMC, job.path, &job.song, job.mode);
        if (job.ok) job.installed = control_install_midi_song(job.song);
    } else if (job.source == 1) {
        job.ok = mem_midi::load_pattern_raw(job.path, job.grid,
                                             job.parsed_name, sizeof(job.parsed_name),
                                             &job.steps_found, &job.bpm, &job.raw_len,
                                             job.mode);
    } else {
        job.ok = mem_midi::load_pattern_raw_from_fs(SD_MMC, job.path, job.grid,
                                                     job.parsed_name, sizeof(job.parsed_name),
                                                     &job.steps_found, &job.bpm, &job.raw_len,
                                                     job.mode);
    }
    s_sd_midi_state.store(2, std::memory_order_release);
    vTaskDelete(NULL);
}

static void sd_midi_load_consume_result(void) {
    if (s_sd_midi_state.load(std::memory_order_acquire) != 2) return;
    SdMidiLoadJob& job = s_sd_midi_job;

    if (sd_midi_load_btn) lv_obj_clear_state(sd_midi_load_btn, LV_STATE_DISABLED);
    if (!job.ok) {
        if (sd_midi_status_lbl) {
            lv_label_set_text(sd_midi_status_lbl,
                              job.source == 1 ? "Error al leer MIDI de MEM" : "Error al leer MIDI de SD");
            lv_obj_set_style_text_color(sd_midi_status_lbl, RED808_ACCENT, 0);
        }
        s_sd_midi_state.store(0, std::memory_order_release);
        return;
    }

    if (job.song_mode) {
        if (!job.installed) {
            if (sd_midi_status_lbl) {
                lv_label_set_text(sd_midi_status_lbl,
                    "MIDI valido, pero no se pudo instalar la cancion");
                lv_obj_set_style_text_color(sd_midi_status_lbl, RED808_ERROR, 0);
            }
            s_sd_midi_state.store(0, std::memory_order_release);
            return;
        }
        ui_sequencer_sync_from_current_pattern();
        if (sd_midi_status_lbl) {
            lv_label_set_text_fmt(sd_midi_status_lbl,
                "%s · %u/%u compases · %u escenas",
                job.song.truncated ? "PARCIAL" : "SONG READY",
                (unsigned)job.song.imported_bars,
                (unsigned)job.song.total_bars,
                (unsigned)job.song.pattern_count);
            lv_obj_set_style_text_color(sd_midi_status_lbl,
                job.song.truncated ? RED808_WARNING : RED808_SUCCESS, 0);
        }
        show_midi_song_summary(job.display_name, job.song);
        s_sd_midi_state.store(0, std::memory_order_release);
        return;
    }

    for (int t = 0; t < 16; t++)
        for (int s = 0; s < 64; s++)
            seq_raw_grid[t][s] = job.grid[t][s];

    p4.current_pattern = job.target_slot;
    seq_install_raw_and_show_page0(job.raw_len);
    control_set_pattern_source_tempo(job.target_slot, job.bpm,
                                     job.parsed_name[0] ? job.parsed_name
                                                        : job.display_name);

    if (job.bpm >= 40.0f && job.bpm <= 240.0f
        && !i2c_rotaries_owns_function(POD_FUNC_TEMPO)) {
        p4.bpm_int  = (int)job.bpm;
        p4.bpm_frac = (int)((job.bpm - p4.bpm_int) * 10.0f);
        local_lock_tempo(3000);
        if (ui_control_available()) control_send_tempo(job.bpm);
    }

    int tracks_used = 0;
    for (int t = 0; t < 16; t++) {
        for (int s = 0; s < job.raw_len; s++) {
            if (job.grid[t][s]) { tracks_used++; break; }
        }
    }
    if (sd_midi_status_lbl) {
        lv_label_set_text_fmt(sd_midi_status_lbl, "Pat %02d · %d golpes · %d compases",
                              job.target_slot + 1, job.steps_found, job.raw_len / 16);
        lv_obj_set_style_text_color(sd_midi_status_lbl, RED808_SUCCESS, 0);
    }
    show_midi_load_summary(job.display_name, job.target_slot + 1,
                           job.steps_found, job.raw_len, job.bpm, tracks_used);
    s_sd_midi_state.store(0, std::memory_order_release);
}

static void sd_midi_begin_load(bool song_mode) {
    if (sd_midi_load_in_flight()) return;

    SdMidiLoadJob& job = s_sd_midi_job;
    memset(&job, 0, sizeof(job));
    job.source = (uint8_t)sd_source;
    job.mode = sd_midi_import_mode;
    job.target_slot = sd_midi_target_slot;
    job.song_mode = song_mode;

    if (sd_source == 1) {
        if (sd_mem_selected < 0 || sd_mem_selected >= sd_mem_count) return;
        snprintf(job.path, sizeof(job.path), "/mid/%s", sd_mem_files[sd_mem_selected]);
        snprintf(job.display_name, sizeof(job.display_name), "%s", sd_mem_files[sd_mem_selected]);
    } else {
        if (p4sd.selected_file[0] == '\0') return;
        if (s_sd_scan_state.load(std::memory_order_acquire) != 0 || sd_upload_in_flight()) {
            ui_show_toast("SD ocupada, reintenta...", RED808_WARNING);
            return;
        }
        if (strcmp(p4sd.path, "/") == 0)
            snprintf(job.path, sizeof(job.path), "/%s", p4sd.selected_file);
        else
            snprintf(job.path, sizeof(job.path), "%s/%s", p4sd.path, p4sd.selected_file);
        snprintf(job.display_name, sizeof(job.display_name), "%s", p4sd.selected_file);
    }

    if (sd_midi_status_lbl) {
        lv_label_set_text(sd_midi_status_lbl,
            song_mode ? "Analizando arrangement completo..." : "Analizando patron MIDI...");
        lv_obj_set_style_text_color(sd_midi_status_lbl, RED808_WARNING, 0);
    }
    if (sd_midi_load_btn) lv_obj_add_state(sd_midi_load_btn, LV_STATE_DISABLED);
    if (sd_midi_song_btn) lv_obj_add_state(sd_midi_song_btn, LV_STATE_DISABLED);

    s_sd_midi_state.store(1, std::memory_order_release);
    if (xTaskCreatePinnedToCore(sd_midi_load_task, "midiparse", 12288,
                                NULL, 1, NULL, 1) != pdPASS) {
        s_sd_midi_state.store(0, std::memory_order_release);
        if (sd_midi_load_btn) lv_obj_clear_state(sd_midi_load_btn, LV_STATE_DISABLED);
        if (sd_midi_song_btn) lv_obj_clear_state(sd_midi_song_btn, LV_STATE_DISABLED);
        ui_show_toast("No se pudo iniciar el parser MIDI", RED808_WARNING);
    }
}

static void sd_midi_load_btn_cb(lv_event_t* e) {
    (void)e;
    sd_midi_begin_load(false);
}

// ── Async WAV upload ────────────────────────────────────────────────────
// The transfer (SD read + PCM conversion + USB-C streaming) runs
// synchronously inside the LVGL button callback, freezing rendering and
// touch for seconds — the "UPLOAD PAD xx..." label never even painted.
// It now runs in a one-shot worker task (Core 1, prio 1); results are
// consumed by sd_upload_consume_result() on the LVGL task. The worker must
// never touch LVGL objects.
enum SdUploadResult : uint8_t {
    SD_UP_OK = 0,
    SD_UP_OPEN_FAILED,
    SD_UP_INVALID,
    SD_UP_OFFLINE,
    SD_UP_WRITE_CUT,
    SD_UP_TRANSPORT_ERROR,
    SD_UP_DAISY_REJECTED,
    SD_UP_ACK_TIMEOUT,
};
struct SdUploadJob {
    // request (LVGL task → worker)
    char path[192];
    char filename[64];
    int  pad;
    int  xtra_slot;
    bool close_after;
    bool trigger_after;
    // result (worker → LVGL task)
    uint8_t result;
    int     transport_error;
    uint32_t duration_ms;
    uint32_t sample_rate;
    uint32_t data_offset;
    uint32_t data_size;
    uint32_t source_frames;
    uint16_t format;
    uint8_t channels;
    uint8_t bits;
    uint8_t peak_count;
    int8_t peak_max[96];
    int8_t peak_min[96];
};
static SdUploadJob s_sd_upload_job;
// 0 = idle, 1 = running, 2 = done (result pending consumption)
static std::atomic<uint8_t> s_sd_upload_state{0};
static std::atomic<uint8_t> s_sd_upload_progress{0};

// Default-kit authority is the P4 SD. Daisy is only the audio destination.
enum FactoryKitState : uint8_t {
    FACTORY_KIT_WAIT_LINK = 0,
    FACTORY_KIT_SCANNING,
    FACTORY_KIT_READY,
    FACTORY_KIT_UPLOADING,
    FACTORY_KIT_COMPLETE,
    FACTORY_KIT_ERROR,
};
struct FactoryKitFile {
    char path[192];
    char name[64];
};
static FactoryKitFile s_factory_kit_files[16] = {};
static std::atomic<uint8_t> s_factory_kit_state{FACTORY_KIT_WAIT_LINK};
static uint8_t s_factory_kit_cursor = 0;
static uint8_t s_factory_kit_loaded = 0;
static uint8_t s_factory_kit_failures = 0;
static uint32_t s_factory_link_since_ms = 0;
static bool s_factory_link_seen = false;
static bool s_factory_result_announced = false;

static uint16_t wav_le16(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t wav_le32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static bool sd_inspect_wav(File& file, SdUploadJob& job) {
    uint8_t riff[12];
    if (!file.seek(0) || file.read(riff, sizeof(riff)) != sizeof(riff) ||
        memcmp(riff, "RIFF", 4) != 0 || memcmp(riff + 8, "WAVE", 4) != 0) return false;

    uint16_t format = 0, channels = 0, bits = 0;
    uint32_t sample_rate = 0, data_offset = 0, data_size = 0;
    bool got_fmt = false;
    while (file.position() + 8 <= file.size()) {
        uint8_t chdr[8];
        if (file.read(chdr, sizeof(chdr)) != sizeof(chdr)) break;
        uint32_t chunk_size = wav_le32(chdr + 4);
        uint32_t payload = file.position();
        if (memcmp(chdr, "fmt ", 4) == 0 && chunk_size >= 16) {
            uint8_t fmt[16];
            if (file.read(fmt, sizeof(fmt)) != sizeof(fmt)) return false;
            format = wav_le16(fmt);
            channels = wav_le16(fmt + 2);
            sample_rate = wav_le32(fmt + 4);
            bits = wav_le16(fmt + 14);
            got_fmt = true;
        } else if (memcmp(chdr, "data", 4) == 0) {
            data_offset = payload;
            uint32_t remain = (uint32_t)file.size() - payload;
            data_size = chunk_size < remain ? chunk_size : remain;
        }
        uint64_t next64 = (uint64_t)payload + chunk_size + (chunk_size & 1U);
        if (next64 > file.size() || !file.seek((uint32_t)next64)) break;
        if (got_fmt && data_size) break;
    }

    if (!got_fmt || !data_size || (format != 1 && format != 3) ||
        channels < 1 || channels > 8 || sample_rate < 8000 || sample_rate > 192000 ||
        (bits != 8 && bits != 16 && bits != 24 && bits != 32)) return false;
    uint32_t bytes_per_sample = bits / 8;
    uint32_t frame_bytes = bytes_per_sample * channels;
    uint32_t frames = frame_bytes ? data_size / frame_bytes : 0;
    if (!frames) return false;

    job.sample_rate = sample_rate;
    job.data_offset = data_offset;
    job.data_size = data_size;
    job.source_frames = frames;
    job.format = format;
    job.channels = (uint8_t)channels;
    job.bits = (uint8_t)bits;
    job.duration_ms = (uint32_t)(((uint64_t)frames * 1000ULL) / sample_rate);
    job.peak_count = 96;
    for (int i = 0; i < 96; i++) { job.peak_max[i] = -127; job.peak_min[i] = 127; }

    if (!file.seek(data_offset)) return false;
    uint8_t buf[2048];
    uint32_t frame_index = 0;
    size_t block_bytes = (sizeof(buf) / frame_bytes) * frame_bytes;
    if (!block_bytes) return false;
    while (frame_index < frames) {
        uint32_t frames_left = frames - frame_index;
        size_t want = frames_left < block_bytes / frame_bytes
            ? (size_t)frames_left * frame_bytes : block_bytes;
        size_t got = file.read(buf, want);
        if (got < frame_bytes) break;
        uint32_t got_frames = (uint32_t)(got / frame_bytes);
        for (uint32_t f = 0; f < got_frames; f++) {
            const uint8_t* p = buf + f * frame_bytes;
            int sample8 = 0;
            if (bits == 8) sample8 = (int)p[0] - 128;
            else if (bits == 16) sample8 = (int16_t)wav_le16(p) / 256;
            else if (bits == 24) sample8 = (int8_t)p[2];
            else if (format == 3) {
                float fv = 0.0f;
                memcpy(&fv, p, sizeof(float));
                sample8 = (int)lroundf(fv * 127.0f);
            } else sample8 = (int8_t)p[3];
            sample8 = constrain(sample8, -127, 127);
            uint32_t bin = (uint32_t)(((uint64_t)(frame_index + f) * 96ULL) / frames);
            if (bin > 95) bin = 95;
            if (sample8 > job.peak_max[bin]) job.peak_max[bin] = (int8_t)sample8;
            if (sample8 < job.peak_min[bin]) job.peak_min[bin] = (int8_t)sample8;
        }
        frame_index += got_frames;
        if ((frame_index & 0x3FFFU) == 0) vTaskDelay(1);
    }
    for (int i = 0; i < 96; i++) {
        if (job.peak_max[i] < job.peak_min[i]) job.peak_max[i] = job.peak_min[i] = 0;
    }
    return file.seek(data_offset);
}

static bool sd_upload_in_flight(void) {
    return s_sd_upload_state.load(std::memory_order_acquire) != 0;
}

static int16_t sd_decode_wav_frame(const uint8_t* data, uint32_t frame,
                                        const SdUploadJob& job) {
    const uint32_t bytes_per_sample = job.bits / 8u;
    const uint32_t frame_bytes = bytes_per_sample * job.channels;
    const uint8_t* base = data + frame * frame_bytes;
    int64_t sum = 0;
    for (uint8_t channel = 0; channel < job.channels; ++channel) {
        const uint8_t* sample = base + channel * bytes_per_sample;
        int32_t value = 0;
        if (job.format == 3 && job.bits == 32) {
            float fv = 0.0f;
            memcpy(&fv, sample, sizeof(fv));
            if (!isfinite(fv)) fv = 0.0f;
            fv = constrain(fv, -1.0f, 1.0f);
            value = (int32_t)lroundf(fv * 32767.0f);
        } else if (job.bits == 8) {
            value = ((int32_t)sample[0] - 128) << 8;
        } else if (job.bits == 16) {
            value = (int16_t)wav_le16(sample);
        } else if (job.bits == 24) {
            value = (int32_t)sample[0] | ((int32_t)sample[1] << 8)
                  | ((int32_t)sample[2] << 16);
            if (value & 0x00800000) value |= (int32_t)0xFF000000;
            value >>= 8;
        } else {
            value = (int32_t)wav_le32(sample) >> 16;
        }
        sum += value;
    }
    const int32_t mono = (int32_t)(sum / job.channels);
    return (int16_t)constrain(mono, -32768, 32767);
}

static bool sd_usb_send_with_retry(uint8_t command, const void* payload,
                                   uint16_t length) {
    for (int attempt = 0; attempt < 100; ++attempt) {
        if (!control_available()) return false;
        if (daisyUsb.send(command, payload, length)) return true;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return false;
}

static bool fx_any_active(void) {
    if((!p4.enc_muted[0] && p4.enc_value[0] > 0)
       || (!p4.enc_muted[1] && p4.enc_value[1] > 0)
       || (!p4.enc_muted[2] && p4.enc_value[2] > 0)
       || (!p4.pot_muted[0] && p4.pot_value[3] > 0)
       || (!p4.pot_muted[2] && p4.pot_value[2] > 0))
        return true;
    return p4.filter_type != 0 || p4.distortion_pct > 0
        || p4.bitcrush_bits < 16 || p4.sample_rate_hz > 0;
}

static void sd_upload_task(void* arg) {
    (void)arg;
    SdUploadJob& job = s_sd_upload_job;
    job.result = SD_UP_OK;
    job.transport_error = 0;
    job.duration_ms = 0;
    job.sample_rate = 0;
    job.data_offset = 0;
    job.data_size = 0;
    job.source_frames = 0;
    job.format = 0;
    job.channels = 0;
    job.bits = 0;
    job.peak_count = 0;
    s_sd_upload_progress.store(1, std::memory_order_release);

    File sample = SD_MMC.open(job.path, FILE_READ);
    if (!sample) {
        job.result = SD_UP_OPEN_FAILED;
        s_sd_upload_state.store(2, std::memory_order_release);
        vTaskDelete(NULL);
        return;
    }
    if (sample.size() == 0 || sample.size() > 4U * 1024U * 1024U
        || !sd_inspect_wav(sample, job)) {
        sample.close();
        job.result = SD_UP_INVALID;
        s_sd_upload_state.store(2, std::memory_order_release);
        vTaskDelete(NULL);
        return;
    }
    if (!control_available()) {
        sample.close();
        job.result = SD_UP_OFFLINE;
        s_sd_upload_state.store(2, std::memory_order_release);
        vTaskDelete(NULL);
        return;
    }

    uint8_t* source = static_cast<uint8_t*>(heap_caps_malloc(
        job.data_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!source)
        source = static_cast<uint8_t*>(heap_caps_malloc(job.data_size,
                                                        MALLOC_CAP_8BIT));
    if (!source || !sample.seek(job.data_offset)
        || sample.read(source, job.data_size) != job.data_size) {
        if (source) heap_caps_free(source);
        sample.close();
        job.result = SD_UP_OPEN_FAILED;
        s_sd_upload_state.store(2, std::memory_order_release);
        vTaskDelete(NULL);
        return;
    }
    sample.close();

    constexpr uint32_t kTargetRate = 48000u;
    constexpr uint32_t kMaxOutputSamples = 4u * 1024u * 1024u;
    uint32_t output_samples = (uint32_t)(
        ((uint64_t)job.source_frames * kTargetRate) / job.sample_rate);
    output_samples = constrain(output_samples, 1u, kMaxOutputSamples);

    SampleBeginPayload begin = {};
    begin.padIndex = (uint8_t)job.pad;
    begin.bitsPerSample = 16;
    begin.sampleRate = (uint16_t)kTargetRate;
    begin.totalBytes = output_samples * sizeof(int16_t);
    begin.totalSamples = output_samples;
    bool ok = sd_usb_send_with_retry(CMD_SAMPLE_BEGIN, &begin, sizeof(begin));
    if (ok) vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t packet[sizeof(SampleDataHeader) + 512];
    auto* header = reinterpret_cast<SampleDataHeader*>(packet);
    auto* pcm = reinterpret_cast<int16_t*>(packet + sizeof(SampleDataHeader));
    uint32_t output_index = 0;
    while (ok && output_index < output_samples) {
        const uint32_t remaining = output_samples - output_index;
        const uint16_t count = static_cast<uint16_t>(remaining < 256u
                                                     ? remaining : 256u);
        for (uint16_t i = 0; i < count; ++i) {
            const uint32_t out = output_index + i;
            const uint64_t position = ((uint64_t)out * job.sample_rate << 16)
                                    / kTargetRate;
            uint32_t source_index = (uint32_t)(position >> 16);
            const uint16_t fraction = (uint16_t)(position & 0xFFFFu);
            if (source_index >= job.source_frames) source_index = job.source_frames - 1;
            const uint32_t next_index = source_index + 1 < job.source_frames
                                      ? source_index + 1 : source_index;
            const int32_t first = sd_decode_wav_frame(source, source_index, job);
            const int32_t second = sd_decode_wav_frame(source, next_index, job);
            pcm[i] = (int16_t)(first + (((second - first) * (int32_t)fraction) >> 16));
        }
        header->padIndex = (uint8_t)job.pad;
        header->reserved = 0;
        header->chunkSize = count * sizeof(int16_t);
        header->offset = output_index * sizeof(int16_t);
        const uint16_t payload_length = sizeof(SampleDataHeader) + header->chunkSize;
        ok = sd_usb_send_with_retry(CMD_SAMPLE_DATA, packet, payload_length);
        output_index += count;
        const uint8_t progress = (uint8_t)(8u
            + (uint32_t)((uint64_t)output_index * 88u / output_samples));
        s_sd_upload_progress.store(progress, std::memory_order_release);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    SampleEndPayload finish = {};
    finish.padIndex = (uint8_t)job.pad;
    finish.status = ok && output_index == output_samples ? 0u : 1u;
    const uint32_t ack_before = daisyUsb.sampleEndAckRevision();
    if (!sd_usb_send_with_retry(CMD_SAMPLE_END, &finish, sizeof(finish))) {
        ok = false;
    } else if (finish.status == 0u) {
        bool ack_received = false;
        const uint32_t ack_deadline = millis() + 1500u;
        while ((int32_t)(millis() - ack_deadline) < 0) {
            if (daisyUsb.sampleEndAckRevision() != ack_before
                && daisyUsb.sampleEndAckPad() == (uint8_t)job.pad) {
                ack_received = true;
                if (!daisyUsb.sampleEndAckAccepted())
                    job.result = SD_UP_DAISY_REJECTED;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        if (!ack_received)
            job.result = SD_UP_ACK_TIMEOUT;
        if (!ack_received || !daisyUsb.sampleEndAckAccepted())
            ok = false;
    }
    heap_caps_free(source);

    if (job.result == SD_UP_OK)
        job.result = ok && output_index == output_samples
                   ? SD_UP_OK : SD_UP_TRANSPORT_ERROR;
    job.transport_error = ok ? 0 : 1;
    s_sd_upload_progress.store(job.result == SD_UP_OK ? 100u : 0u,
                               std::memory_order_release);
    s_sd_upload_state.store(2, std::memory_order_release);
    vTaskDelete(NULL);
}

static bool factory_wav_name(const char* name) {
    if (!name) return false;
    const size_t len = strlen(name);
    return len > 4 && strcasecmp(name + len - 4, ".wav") == 0;
}

static int factory_pad_from_filename(const char* name) {
    if (!name) return -1;
    // Exact RED 808 KARZ identities. HH is the kit's closed-hat name and maps
    // to the canonical CH pad used by the sequencer and the S3 project.
    struct Mapping { const char* prefix; uint8_t pad; };
    static const Mapping map[] = {
        {"808 BD", 0}, {"808 SD", 1}, {"808 HH", 2}, {"808 CH", 2},
        {"808 OH", 3}, {"808 CY", 4}, {"808 CP", 5}, {"808 RS", 6},
        {"808 COW", 7}, {"808 CB", 7}, {"808 LT", 8}, {"808 MT", 9},
        {"808 HT", 10}, {"808 MA", 11}, {"808 CL", 12}, {"808 HC", 13},
        {"808 MC", 14}, {"808 LC", 15},
    };
    for (const Mapping& item : map) {
        const size_t prefixLen = strlen(item.prefix);
        if (strncasecmp(name, item.prefix, prefixLen) == 0)
            return item.pad;
    }
    return -1;
}

static void factory_kit_scan_task(void*) {
    memset(s_factory_kit_files, 0, sizeof(s_factory_kit_files));
    s_factory_kit_loaded = 0;
    s_factory_kit_failures = 0;
    s_factory_kit_cursor = 0;

    bool ok = sd_local_try_mount();
    p4sd.mounted = ok;
    File dir;
    if (ok) dir = SD_MMC.open("/data/RED 808 KARZ");
    if (!dir || !dir.isDirectory()) ok = false;

    if (ok) {
        File entry = dir.openNextFile();
        while (entry) {
            if (!entry.isDirectory()) {
                const char* base = sd_basename(entry.name());
                const int pad = factory_pad_from_filename(base);
                if (pad >= 0 && pad < 16 && factory_wav_name(base)) {
                    FactoryKitFile& selected = s_factory_kit_files[pad];
                    // The S3 asset generator sorted filenames and took the
                    // first variant for every instrument. Do the same here so
                    // the kit sounds identical across FAT directory layouts.
                    if (!selected.name[0] || strcasecmp(base, selected.name) < 0) {
                        snprintf(selected.path, sizeof(selected.path),
                                 "/data/RED 808 KARZ/%s", base);
                        snprintf(selected.name, sizeof(selected.name), "%s", base);
                    }
                }
            }
            entry.close();
            entry = dir.openNextFile();
        }
        dir.close();
        uint8_t found = 0;
        for (const FactoryKitFile& file : s_factory_kit_files)
            if (file.path[0]) ++found;
        ok = found > 0;
    }

    s_sd_scan_state.store(0, std::memory_order_release);
    s_factory_kit_state.store(ok ? FACTORY_KIT_READY : FACTORY_KIT_ERROR,
                              std::memory_order_release);
    p4sd.needs_refresh.store(true, std::memory_order_release);
    vTaskDelete(NULL);
}

// Called only from the LVGL task. Returns true while the factory loader owns
// the shared upload result slot, so the normal one-file UI consumer leaves it
// alone. Manual SD uploads can still run between factory samples.
static bool sd_factory_autoload_tick(void) {
    const bool connected = daisyUsb.connected();
    if (!connected) {
        s_factory_link_seen = false;
        if (s_sd_upload_state.load(std::memory_order_acquire) == 0)
            s_factory_kit_state.store(FACTORY_KIT_WAIT_LINK,
                                      std::memory_order_release);
        return s_factory_kit_state.load(std::memory_order_acquire)
            == FACTORY_KIT_UPLOADING;
    }

    if (!s_factory_link_seen) {
        s_factory_link_seen = true;
        s_factory_link_since_ms = millis();
        s_factory_result_announced = false;
        if (s_factory_kit_state.load(std::memory_order_acquire)
            >= FACTORY_KIT_COMPLETE)
            s_factory_kit_state.store(FACTORY_KIT_WAIT_LINK,
                                      std::memory_order_release);
    }

    uint8_t state = s_factory_kit_state.load(std::memory_order_acquire);
    if (state == FACTORY_KIT_WAIT_LINK) {
        // Initial pattern/performance sync gets the USB queue first.
        if (millis() - s_factory_link_since_ms < 1200u) return false;
        if (s_sd_scan_state.load(std::memory_order_acquire) != 0
            || sd_midi_load_in_flight() || sd_upload_in_flight())
            return false;
        s_sd_scan_state.store(1, std::memory_order_release);
        s_factory_kit_state.store(FACTORY_KIT_SCANNING,
                                  std::memory_order_release);
        if (xTaskCreatePinnedToCore(factory_kit_scan_task, "factorykit", 6144,
                                    NULL, 1, NULL, 1) != pdPASS) {
            s_sd_scan_state.store(0, std::memory_order_release);
            s_factory_kit_state.store(FACTORY_KIT_ERROR,
                                      std::memory_order_release);
        }
        return true;
    }
    if (state == FACTORY_KIT_SCANNING) return true;

    if (state == FACTORY_KIT_READY) {
        // A manual preview/load already owns the worker: let its normal result
        // consumer finish, then continue the default kit on the next tick.
        if (s_sd_upload_state.load(std::memory_order_acquire) != 0) return false;
        while (s_factory_kit_cursor < 16
               && !s_factory_kit_files[s_factory_kit_cursor].path[0]) {
            ++s_factory_kit_failures;
            ++s_factory_kit_cursor;
        }
        if (s_factory_kit_cursor >= 16) {
            s_factory_kit_state.store(FACTORY_KIT_COMPLETE,
                                      std::memory_order_release);
            state = FACTORY_KIT_COMPLETE;
        } else {
            const uint8_t pad = s_factory_kit_cursor;
            const FactoryKitFile& source = s_factory_kit_files[pad];
            SdUploadJob& job = s_sd_upload_job;
            memset(&job, 0, sizeof(job));
            snprintf(job.path, sizeof(job.path), "%s", source.path);
            snprintf(job.filename, sizeof(job.filename), "%s", source.name);
            job.pad = pad;
            job.xtra_slot = -1;
            job.close_after = false;
            job.trigger_after = false;
            s_sd_upload_progress.store(0, std::memory_order_release);
            s_sd_upload_state.store(1, std::memory_order_release);
            s_factory_kit_state.store(FACTORY_KIT_UPLOADING,
                                      std::memory_order_release);
            if (xTaskCreatePinnedToCore(sd_upload_task, "kitupload", 8192,
                                        NULL, 1, NULL, 1) != pdPASS) {
                s_sd_upload_state.store(0, std::memory_order_release);
                ++s_factory_kit_failures;
                ++s_factory_kit_cursor;
                s_factory_kit_state.store(FACTORY_KIT_READY,
                                          std::memory_order_release);
            }
            p4sd.needs_refresh.store(true, std::memory_order_release);
            return true;
        }
    }

    if (state == FACTORY_KIT_UPLOADING) {
        const uint8_t uploadState = s_sd_upload_state.load(std::memory_order_acquire);
        if (uploadState != 2) return true;
        const SdUploadJob result = s_sd_upload_job;
        s_sd_upload_state.store(0, std::memory_order_release);
        if (result.result == SD_UP_OK) {
            ++s_factory_kit_loaded;
            control_restore_track_engine(s_factory_kit_cursor);
        } else {
            ++s_factory_kit_failures;
        }
        ++s_factory_kit_cursor;
        s_factory_kit_state.store(FACTORY_KIT_READY,
                                  std::memory_order_release);
        p4sd.needs_refresh.store(true, std::memory_order_release);
        return true;
    }

    if (state == FACTORY_KIT_COMPLETE && !s_factory_result_announced) {
        s_factory_result_announced = true;
        snprintf(p4.kit_name, sizeof(p4.kit_name), "RED 808 KARZ");
        daisyUsb.send(CMD_GET_STATUS);
        char message[80];
        snprintf(message, sizeof(message), "P4 SD: RED 808 KARZ %u/16 cargados",
                 (unsigned)s_factory_kit_loaded);
        ui_show_toast(message, s_factory_kit_loaded > 0
                               ? RED808_SUCCESS : RED808_WARNING);
    } else if (state == FACTORY_KIT_ERROR && !s_factory_result_announced) {
        s_factory_result_announced = true;
        ui_show_toast("P4 SD: falta /data/RED 808 KARZ", RED808_WARNING);
    }
    return false;
}

// LVGL task: validate, snapshot the request and launch the worker.
static bool sd_upload_selected_wav(bool closeAfterSuccess, bool triggerAfterUpload) {
    if (p4sd.selected_file[0] == '\0' || p4sd.selected_is_midi) return false;
    if (sd_upload_in_flight() || sd_midi_load_in_flight()) {
        ui_show_toast("Upload en curso...", RED808_WARNING);
        return false;
    }
    if (s_sd_scan_state.load(std::memory_order_acquire) != 0) {
        ui_show_toast("SD ocupada, espera al escaneo", RED808_WARNING);
        return false;
    }

    int xtraSlot = -1;
    if (s_xtra_pending_slot >= 0 && s_xtra_pending_slot < 4) {
        xtraSlot = s_xtra_pending_slot;
        p4sd.selected_pad = xtra_backing_pad_for_slot(xtraSlot);
    }

    SdUploadJob& job = s_sd_upload_job;
    if (strcmp(p4sd.path, "/") == 0)
        snprintf(job.path, sizeof(job.path), "/%s", p4sd.selected_file);
    else
        snprintf(job.path, sizeof(job.path), "%s/%s", p4sd.path, p4sd.selected_file);
    strncpy(job.filename, p4sd.selected_file, sizeof(job.filename) - 1);
    job.filename[sizeof(job.filename) - 1] = '\0';
    for (char* p = job.filename; *p; ++p) {
        unsigned char c = (unsigned char)*p;
        if (c < 0x20 || *p == '"' || *p == '\\') *p = '_';
    }
    job.pad           = p4sd.selected_pad;
    job.xtra_slot     = xtraSlot;
    job.close_after   = closeAfterSuccess;
    job.trigger_after = triggerAfterUpload;

    s_sd_upload_state.store(1, std::memory_order_release);
    s_sd_upload_progress.store(0, std::memory_order_release);
    BaseType_t ok = xTaskCreatePinnedToCore(sd_upload_task, "sdupload",
                                            8192, NULL, 1, NULL, 1);
    if (ok != pdPASS) {
        s_sd_upload_state.store(0, std::memory_order_release);
        ui_show_toast("No se pudo iniciar el upload", RED808_WARNING);
        return false;
    }

    if (sd_status_lbl) {
        lv_label_set_text_fmt(sd_status_lbl, "%s PAD %02d...",
                              triggerAfterUpload ? "PREVIEW" : "UPLOAD",
                              p4sd.selected_pad + 1);
        lv_obj_set_style_text_color(sd_status_lbl, RED808_CYAN, 0);
    }
    if (sd_load_btn)    lv_obj_add_state(sd_load_btn, LV_STATE_DISABLED);
    if (sd_preview_btn) lv_obj_add_state(sd_preview_btn, LV_STATE_DISABLED);
    return true;
}

// LVGL task (ui_update_current_screen): apply a finished upload's result.
static void sd_upload_consume_result(void) {
    uint8_t state = s_sd_upload_state.load(std::memory_order_acquire);
    if (state == 1) {
        static uint8_t prev_progress = 255;
        uint8_t progress = s_sd_upload_progress.load(std::memory_order_acquire);
        if (progress != prev_progress && sd_status_lbl) {
            prev_progress = progress;
            lv_label_set_text_fmt(sd_status_lbl, "UPLOAD PAD %02d  ·  %u%%",
                                  s_sd_upload_job.pad + 1, (unsigned)progress);
            lv_obj_set_style_text_color(sd_status_lbl, RED808_CYAN, 0);
        }
        return;
    }
    if (state != 2) return;
    SdUploadJob job = s_sd_upload_job;   // copy before releasing the slot
    s_sd_upload_state.store(0, std::memory_order_release);

    if (sd_load_btn)    lv_obj_clear_state(sd_load_btn, LV_STATE_DISABLED);
    if (sd_preview_btn) lv_obj_clear_state(sd_preview_btn, LV_STATE_DISABLED);

    if (job.result != SD_UP_OK) {
        char msg[64];
        switch (job.result) {
            case SD_UP_OPEN_FAILED: snprintf(msg, sizeof(msg), "No se puede abrir el WAV"); break;
            case SD_UP_INVALID:     snprintf(msg, sizeof(msg), "WAV no valido o demasiado grande"); break;
            case SD_UP_OFFLINE:     snprintf(msg, sizeof(msg), "Master no conectado"); break;
            case SD_UP_WRITE_CUT:   snprintf(msg, sizeof(msg), "Upload cortado"); break;
            case SD_UP_DAISY_REJECTED: snprintf(msg, sizeof(msg), "Daisy rechazo el sample"); break;
            case SD_UP_ACK_TIMEOUT: snprintf(msg, sizeof(msg), "Daisy no confirmo el sample"); break;
            default:                snprintf(msg, sizeof(msg), "Fallo enviando por USB-C"); break;
        }
        ui_show_toast(msg, RED808_WARNING);
        if (sd_status_lbl) {
            lv_label_set_text(sd_status_lbl, msg);
            lv_obj_set_style_text_color(sd_status_lbl, RED808_WARNING, 0);
        }
        return;
    }

    /* A normal pad upload selects Sampler on both authorities before preview.
     * XTRA slots do not use the 16-track engine assignment. */
    if (job.xtra_slot < 0 && job.pad >= 0 && job.pad < 16) {
        const uint8_t pad = (uint8_t)job.pad;
        s_pad_inst_sel[pad] = 0;
        s_pad_inst_pending[pad] = 0;
        s_pad_inst_local_ms[pad] = millis();
        control_send_set_track_engine(pad, -1);
        pad_inst_refresh_pad_badge(pad);
        seq_refresh_track_label(pad);
        pad_inst_refresh_controls();
        // A trim window sized for the previous file makes no sense against
        // a new one of a different length — reset, mirroring what Daisy
        // itself does for this pad in CMD_SAMPLE_END/LoadWavToPad.
        s_pad_trim_start_pct[pad] = 0;
        s_pad_trim_end_pct[pad] = 100;
        s_pad_wave_count[pad] = job.peak_count;
        memcpy(s_pad_wave_max[pad], job.peak_max, sizeof(job.peak_max));
        memcpy(s_pad_wave_min[pad], job.peak_min, sizeof(job.peak_min));
    }

    if (job.trigger_after && control_available()) {
        control_send_trigger(job.pad, 110);
    }

    char msg[72];
    snprintf(msg, sizeof(msg), "%s PAD %02d",
             job.trigger_after ? "Preview listo en" : "Sample cargado en Daisy",
             job.pad + 1);
    ui_show_toast(msg, RED808_SUCCESS);
    if (sd_status_lbl) {
        lv_label_set_text(sd_status_lbl, msg);
        lv_obj_set_style_text_color(sd_status_lbl, RED808_SUCCESS, 0);
    }

    if (job.xtra_slot >= 0 && job.xtra_slot < 4) {
        XtraPadSlot& slot = s_xtra_slots[job.xtra_slot];
        slot.used = true;
        slot.pad = xtra_backing_pad_for_slot(job.xtra_slot);
        slot.synth_mode = false;
        strncpy(slot.name, job.filename, sizeof(slot.name) - 1);
        slot.name[sizeof(slot.name) - 1] = '\0';
        trim_wav_extension(slot.name);
        slot.trim_start_pct = 0;
        slot.trim_end_pct = 100;
        slot.duration_ms = job.duration_ms;
        slot.sample_rate = job.sample_rate;
        slot.channels = job.channels;
        slot.bits = job.bits;
        s_xtra_wave_count[job.xtra_slot] = job.peak_count;
        memcpy(s_xtra_wave_max[job.xtra_slot], job.peak_max, sizeof(job.peak_max));
        memcpy(s_xtra_wave_min[job.xtra_slot], job.peak_min, sizeof(job.peak_min));
        xtra_save_state();
        xtra_refresh_panel();
        if (job.close_after) {
            s_xtra_pending_slot = -1;
            s_sd_for_xtra = false;
            ui_show_toast("XTRA cargado", RED808_SUCCESS);
            ui_navigate_to(6);
        }
    }
}

static void sd_preview_btn_cb(lv_event_t* e) {
    LV_UNUSED(e);
    (void)sd_upload_selected_wav(false, true);
}

static void sd_load_btn_cb(lv_event_t* e) {
    LV_UNUSED(e);
    if (sd_source == 2) {
        if (!daisyUsb.connected() || !daisyUsb.state().sd_present) {
            ui_show_toast("Daisy no ve su SD", RED808_WARNING);
            return;
        }
        if (!sd_daisy_folder[0] || !sd_daisy_selected[0]) {
            ui_show_toast("Elige una carpeta y un WAV de Daisy", RED808_WARNING);
            return;
        }
        SdLoadSamplePayload payload = {};
        strncpy(payload.folderName, sd_daisy_folder, sizeof(payload.folderName) - 1);
        strncpy(payload.fileName, sd_daisy_selected, sizeof(payload.fileName) - 1);
        payload.padIndex = static_cast<uint8_t>(constrain(p4sd.selected_pad, 0, 15));
        if (!daisyUsb.send(CMD_SD_LOAD_SAMPLE, &payload, sizeof(payload))) {
            ui_show_toast("No se pudo ordenar la carga", RED808_WARNING);
            return;
        }
        // This status request sits behind the blocking FatFS load and arrives
        // with the real post-load mask, so the UI verifies the result.
        daisyUsb.send(CMD_SD_STATUS);
        ui_show_toast("Daisy cargando WAV al pad...", RED808_CYAN);
        return;
    }
    (void)sd_upload_selected_wav(true, false);
}

static void sd_refresh_ui(void) {
    if (!sd_file_list) return;

    if (sd_assign_lbl) {
        lv_label_set_text(sd_assign_lbl, s_sd_for_xtra ? "XTRA SLOT TARGET" : "ASSIGN TO PAD");
    }
    for (int i = 0; i < 16; i++) {
        if (!sd_pad_btns[i]) continue;
        if (s_sd_for_xtra) lv_obj_add_flag(sd_pad_btns[i], LV_OBJ_FLAG_HIDDEN);
        else lv_obj_clear_flag(sd_pad_btns[i], LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_clean(sd_file_list);

    if (sd_load_lbl) {
        lv_label_set_text(sd_load_lbl,
            s_sd_for_xtra ? LV_SYMBOL_UPLOAD "  LOAD TO XTRA" : LV_SYMBOL_UPLOAD "  LOAD TO PAD");
    }
    if (sd_preview_lbl) {
        lv_label_set_text(sd_preview_lbl,
            s_sd_for_xtra ? LV_SYMBOL_PLAY "  PREVIEW XTRA" : LV_SYMBOL_PLAY "  PREVIEW PAD");
    }
    if (sd_preview_btn) {
        if (p4sd.mounted && p4sd.selected_file[0] && !p4sd.selected_is_midi)
            lv_obj_clear_state(sd_preview_btn, LV_STATE_DISABLED);
        else
            lv_obj_add_state(sd_preview_btn, LV_STATE_DISABLED);
    }

    // ── MEM branch: list P4's own /mid/*.mid from SPIFFS ────────────────
    if (sd_source == 1) {
        if (sd_status_lbl) {
            char sbuf[24];
            snprintf(sbuf, sizeof(sbuf), "MEM %d MIDI", sd_mem_count);
            lv_label_set_text(sd_status_lbl, sbuf);
            lv_obj_set_style_text_color(sd_status_lbl, RED808_WARNING, 0);
        }
        if (sd_path_lbl) lv_label_set_text(sd_path_lbl, "/mid (flash)");

        // Enable/disable LOAD button based on MEM selection
        if (sd_midi_load_btn) {
            if (sd_mem_selected >= 0)
                lv_obj_clear_state(sd_midi_load_btn, LV_STATE_DISABLED);
            else
                lv_obj_add_state(sd_midi_load_btn, LV_STATE_DISABLED);
        }
        if (sd_midi_song_btn) {
            if (sd_mem_selected >= 0)
                lv_obj_clear_state(sd_midi_song_btn, LV_STATE_DISABLED);
            else
                lv_obj_add_state(sd_midi_song_btn, LV_STATE_DISABLED);
        }

        if (sd_mem_count <= 0) {
            lv_obj_t* lbl = lv_label_create(sd_file_list);
            lv_label_set_text(lbl, "No MEM MIDI files");
            lv_obj_set_style_text_color(lbl, RED808_TEXT_DIM, 0);
            lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
            return;
        }

        for (int i = 0; i < sd_mem_count; i++) {
            lv_obj_t* btn = lv_btn_create(sd_file_list);
            lv_obj_set_size(btn, 580, 44);
            lv_obj_set_style_radius(btn, 6, 0);
            bool sel = (i == sd_mem_selected);
            lv_obj_set_style_bg_color(btn,
                sel ? RED808_ACCENT : lv_color_hex(0x3A2A00), 0);
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x886622), LV_STATE_PRESSED);

            lv_obj_t* lbl = lv_label_create(btn);
            char display[64];
            snprintf(display, sizeof(display),
                LV_SYMBOL_FILE "  %s [MEM]", sd_mem_files[i]);
            lv_label_set_text(lbl, display);
            lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
            lv_obj_set_style_text_color(lbl,
                sel ? lv_color_white() : RED808_WARNING, 0);
            lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 12, 0);

            lv_obj_add_event_cb(btn, sd_mem_file_btn_cb, LV_EVENT_CLICKED,
                                (void*)(intptr_t)i);
        }
        return;
    }

    // ── DAISY branch: inspect the card mounted by Daisy itself over USB ──
    if (sd_source == 2) {
        const auto& state = daisyUsb.state();
        if (state.daisy_sd_revision != sd_daisy_seen_revision) {
            sd_daisy_seen_revision = state.daisy_sd_revision;
            sd_daisy_waiting = false;
        }
        if (sd_assign_lbl) lv_label_set_text(sd_assign_lbl, "DAISY SD -> PAD");
        if (sd_preview_btn) lv_obj_add_state(sd_preview_btn, LV_STATE_DISABLED);
        if (sd_load_lbl) lv_label_set_text(sd_load_lbl, LV_SYMBOL_UPLOAD "  LOAD DAISY -> PAD");
        if (sd_load_btn) {
            if (state.sd_present && sd_daisy_folder[0] && sd_daisy_selected[0])
                lv_obj_clear_state(sd_load_btn, LV_STATE_DISABLED);
            else
                lv_obj_add_state(sd_load_btn, LV_STATE_DISABLED);
        }
        if (sd_selected_lbl)
            lv_label_set_text(sd_selected_lbl,
                sd_daisy_selected[0] ? sd_daisy_selected : "Elige WAV de la SD de Daisy");

        uint8_t loaded = 0;
        for (uint8_t i = 0; i < 16; i++)
            if (state.daisy_sd_sample_mask & (1u << i)) loaded++;
        if (sd_status_lbl) {
            if (sd_daisy_waiting) {
                lv_label_set_text(sd_status_lbl, "D: WAIT");
                lv_obj_set_style_text_color(sd_status_lbl, RED808_CYAN, 0);
            } else {
                lv_label_set_text_fmt(sd_status_lbl, "D:%s W%u",
                    state.sd_present ? "OK" : "NO", loaded);
                lv_obj_set_style_text_color(sd_status_lbl,
                    state.sd_present ? RED808_SUCCESS : RED808_WARNING, 0);
            }
        }
        if (sd_path_lbl) {
            if (sd_daisy_folder[0])
                lv_label_set_text_fmt(sd_path_lbl, "/data/%s", sd_daisy_folder);
            else
                lv_label_set_text(sd_path_lbl, "/data (SD conectada a Daisy)");
        }

        if (!state.sd_present && !sd_daisy_waiting) {
            lv_obj_t* lbl = lv_label_create(sd_file_list);
            lv_label_set_text_fmt(lbl,
                "DAISY NO VE LA SD\n"
                "FatFS M%u /data R%u | SPI %s\n"
                "TYPE %u CMD %u R1 %02X TOKEN %02X ERR %u",
                (unsigned)state.sd_mount_result, (unsigned)state.sd_root_result,
                sd_diag_stage_name(state.sd_diag_stage),
                (unsigned)state.sd_card_type,
                (unsigned)state.sd_last_command,
                (unsigned)state.sd_last_response,
                (unsigned)state.sd_last_data_token,
                (unsigned)state.sd_spi_errors);
            lv_obj_set_style_text_color(lbl, RED808_ACCENT, 0);
            lv_obj_set_style_text_font(lbl, &lv_font_montserrat_18, 0);
            return;
        }

        if (sd_daisy_folder[0]) {
            lv_obj_t* back_btn = lv_btn_create(sd_file_list);
            lv_obj_set_size(back_btn, 580, 44);
            lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x333344), 0);
            lv_obj_set_style_radius(back_btn, 6, 0);
            lv_obj_t* back_lbl = lv_label_create(back_btn);
            lv_label_set_text(back_lbl, LV_SYMBOL_LEFT "  /data");
            lv_obj_set_style_text_font(back_lbl, &lv_font_montserrat_16, 0);
            lv_obj_set_style_text_color(back_lbl, RED808_TEXT_DIM, 0);
            lv_obj_center(back_lbl);
            lv_obj_add_event_cb(back_btn, sd_back_btn_cb, LV_EVENT_CLICKED, NULL);
        }

        const int count = sd_daisy_folder[0]
            ? state.daisy_sd_file_count : state.daisy_sd_folder_count;
        for (int i = 0; i < count; i++) {
            const char* name = sd_daisy_folder[0]
                ? state.daisy_sd_files[i] : state.daisy_sd_folders[i];
            lv_obj_t* btn = lv_btn_create(sd_file_list);
            lv_obj_set_size(btn, 580, 44);
            lv_obj_set_style_radius(btn, 6, 0);
            lv_obj_set_style_bg_color(btn,
                sd_daisy_folder[0] ? lv_color_hex(0x173A25) : lv_color_hex(0x173A4A), 0);
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x446688), LV_STATE_PRESSED);
            lv_obj_t* lbl = lv_label_create(btn);
            lv_label_set_text_fmt(lbl,
                sd_daisy_folder[0] ? LV_SYMBOL_AUDIO "  %s" : LV_SYMBOL_DIRECTORY "  %s",
                name);
            lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
            lv_obj_set_style_text_color(lbl,
                sd_daisy_folder[0] ? RED808_SUCCESS : RED808_CYAN, 0);
            lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 12, 0);
            lv_obj_add_event_cb(btn, sd_daisy_file_btn_cb, LV_EVENT_CLICKED,
                                (void*)(intptr_t)i);
        }
        if (!sd_daisy_waiting && count == 0) {
            lv_obj_t* lbl = lv_label_create(sd_file_list);
            lv_label_set_text(lbl, sd_daisy_folder[0]
                ? "Daisy no encuentra WAV en esta carpeta"
                : "Daisy ve /data, pero no encuentra carpetas");
            lv_obj_set_style_text_color(lbl, RED808_WARNING, 0);
            lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
        }
        return;
    }

    // ── SD branch (local P4 SD_MMC) ─────────────────────────────────────
    // Update status
    if (sd_status_lbl) {
        const uint8_t kitState = s_factory_kit_state.load(std::memory_order_acquire);
        if (kitState == FACTORY_KIT_SCANNING) {
            lv_label_set_text(sd_status_lbl, "P4 KIT SCAN...");
            lv_obj_set_style_text_color(sd_status_lbl, RED808_CYAN, 0);
        } else if (kitState == FACTORY_KIT_UPLOADING) {
            lv_label_set_text_fmt(sd_status_lbl, "KIT %u/16 %u%%",
                (unsigned)(s_factory_kit_cursor + 1u),
                (unsigned)s_sd_upload_progress.load(std::memory_order_acquire));
            lv_obj_set_style_text_color(sd_status_lbl, RED808_CYAN, 0);
        } else if (kitState == FACTORY_KIT_COMPLETE) {
            lv_label_set_text_fmt(sd_status_lbl, "P4 KIT READY %u/16",
                                  (unsigned)s_factory_kit_loaded);
            lv_obj_set_style_text_color(sd_status_lbl,
                s_factory_kit_loaded > 0 ? RED808_SUCCESS : RED808_WARNING, 0);
        } else if (kitState == FACTORY_KIT_ERROR) {
            lv_label_set_text(sd_status_lbl, "P4 KIT PATH ERROR");
            lv_obj_set_style_text_color(sd_status_lbl, RED808_WARNING, 0);
        } else {
            lv_label_set_text(sd_status_lbl, p4sd.mounted ? "P4 SD READY" : "NO P4 SD");
            lv_obj_set_style_text_color(sd_status_lbl,
                p4sd.mounted ? RED808_SUCCESS : RED808_WARNING, 0);
        }
    }
    // Update path
    if (sd_path_lbl) lv_label_set_text(sd_path_lbl, p4sd.path);

    // Update selected file
    if (sd_selected_lbl) {
        if (p4sd.selected_file[0])
            lv_label_set_text(sd_selected_lbl, p4sd.selected_file);
        else
            lv_label_set_text(sd_selected_lbl, "");
    }
    // Enable/disable LOAD button
    if (sd_load_btn) {
        if (p4sd.mounted && p4sd.selected_file[0] && !p4sd.selected_is_midi)
            lv_obj_clear_state(sd_load_btn, LV_STATE_DISABLED);
        else
            lv_obj_add_state(sd_load_btn, LV_STATE_DISABLED);
    }
    if (sd_midi_load_btn) {
        if (p4sd.selected_file[0] && p4sd.selected_is_midi)
            lv_obj_clear_state(sd_midi_load_btn, LV_STATE_DISABLED);
        else
            lv_obj_add_state(sd_midi_load_btn, LV_STATE_DISABLED);
    }
    if (sd_midi_song_btn) {
        if (p4sd.selected_file[0] && p4sd.selected_is_midi)
            lv_obj_clear_state(sd_midi_song_btn, LV_STATE_DISABLED);
        else
            lv_obj_add_state(sd_midi_song_btn, LV_STATE_DISABLED);
    }
    if (!p4sd.mounted) {
        lv_obj_t* lbl = lv_label_create(sd_file_list);
        lv_label_set_text(lbl, "SD NOT MOUNTED");
        lv_obj_set_style_text_color(lbl, RED808_ACCENT, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, 0);
        return;
    }

    // "Back" button if not root
    if (strcmp(p4sd.path, "/") != 0 && p4sd.path[0] != '\0') {
        lv_obj_t* back_btn = lv_btn_create(sd_file_list);
        lv_obj_set_size(back_btn, 580, 44);
        lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x333344), 0);
        lv_obj_set_style_radius(back_btn, 6, 0);
        lv_obj_t* back_lbl = lv_label_create(back_btn);
        lv_label_set_text(back_lbl, LV_SYMBOL_LEFT "  .. (back)");
        lv_obj_set_style_text_font(back_lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(back_lbl, lv_color_hex(0xCCCCCC), 0);
        lv_obj_center(back_lbl);
        lv_obj_add_event_cb(back_btn, sd_back_btn_cb, LV_EVENT_CLICKED, NULL);
    }

    // File/directory entries
    for (int i = 0; i < p4sd.entry_count; i++) {
        lv_obj_t* btn = lv_btn_create(sd_file_list);
        lv_obj_set_size(btn, 580, 44);
        lv_obj_set_style_radius(btn, 6, 0);

        bool is_dir  = p4sd.entries[i].is_dir;
        bool is_midi = p4sd.entries[i].is_midi;
        lv_obj_set_style_bg_color(btn,
            is_dir  ? lv_color_hex(0x1A3A5C) :
            is_midi ? lv_color_hex(0x3A2A00) : lv_color_hex(0x1A2A1A), 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x446688), LV_STATE_PRESSED);

        lv_obj_t* lbl = lv_label_create(btn);
        char display[64];
        snprintf(display, sizeof(display),
            is_dir  ? LV_SYMBOL_DIRECTORY "  %s" :
            is_midi ? LV_SYMBOL_FILE "  %s [MIDI]" : LV_SYMBOL_AUDIO "  %s",
            p4sd.entries[i].name);
        lv_label_set_text(lbl, display);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(lbl,
            is_dir  ? RED808_CYAN :
            is_midi ? RED808_WARNING : RED808_SUCCESS, 0);
        lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 12, 0);

        lv_obj_add_event_cb(btn, sd_file_btn_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
    }

    if (!p4sd.list_complete && sd_source == 0) {
        // Async scan in flight (sd_scan_task) — placeholder until it lands.
        lv_obj_t* lbl = lv_label_create(sd_file_list);
        lv_label_set_text(lbl, LV_SYMBOL_REFRESH "  Escaneando SD...");
        lv_obj_set_style_text_color(lbl, RED808_CYAN, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
    } else if (p4sd.list_complete && p4sd.entry_count == 0) {
        lv_obj_t* lbl = lv_label_create(sd_file_list);
        lv_label_set_text(lbl, "No files found (.wav / .mid)");
        lv_obj_set_style_text_color(lbl, RED808_TEXT_DIM, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
    }
}

// =============================================================================
// PAD SAMPLE TRIM EDITOR — non-destructive start/end window for the 16 main
// sequencer pads. Sends CMD_PAD_TRIM (DaisyPod3 applies it in TriggerPad()
// at trigger time, never touching the uploaded PCM). Mirrors XTRAPADS'
// editor UI (waveform + start/end sliders + preview) minus gate/play-mode,
// which are live-performance concepts (hold-to-repeat) that don't map onto
// a pattern-triggered pad — note length there is already the per-step
// noteLenDiv the sequencer editor sets. Reachable from the SD CARD screen's
// pad panel, where "this pad's sample" is already the selected context.
// Data arrays (s_pad_trim_start_pct etc.) are declared earlier, near
// sd_pad_btns[], since sd_upload_consume_result() writes them too.
static lv_obj_t*  s_pad_trim_modal = NULL;
static lv_obj_t*  s_pad_trim_wave  = NULL;
static lv_point_t s_pad_trim_wave_points[192];
static lv_obj_t*  s_pad_trim_start_slider = NULL;
static lv_obj_t*  s_pad_trim_end_slider   = NULL;
static lv_obj_t*  s_pad_trim_start_lbl    = NULL;
static lv_obj_t*  s_pad_trim_end_lbl      = NULL;
static lv_obj_t*  s_pad_trim_fade_in_slider  = NULL;
static lv_obj_t*  s_pad_trim_fade_out_slider = NULL;
static lv_obj_t*  s_pad_trim_fade_in_lbl     = NULL;
static lv_obj_t*  s_pad_trim_fade_out_lbl    = NULL;
static int        s_pad_trim_editor_pad   = -1;
// Fade in/out per main pad (0-255ms, 0=off) — same non-persisted, in-memory
// treatment as s_pad_trim_start_pct/end_pct above (no SPIFFS file for main-
// pad trim exists today), sent via CMD_PAD_FADE_IN/OUT alongside CMD_PAD_TRIM.
static uint8_t s_pad_fade_in_ms[16] = {};
static uint8_t s_pad_fade_out_ms[16] = {};

static void pad_trim_editor_refresh_values(void) {
    if (s_pad_trim_editor_pad < 0) return;
    int start = s_pad_trim_start_slider ? lv_slider_get_value(s_pad_trim_start_slider)
                                        : s_pad_trim_start_pct[s_pad_trim_editor_pad];
    int end = s_pad_trim_end_slider ? lv_slider_get_value(s_pad_trim_end_slider)
                                    : s_pad_trim_end_pct[s_pad_trim_editor_pad];
    int fadeIn = s_pad_trim_fade_in_slider ? lv_slider_get_value(s_pad_trim_fade_in_slider)
                                           : s_pad_fade_in_ms[s_pad_trim_editor_pad];
    int fadeOut = s_pad_trim_fade_out_slider ? lv_slider_get_value(s_pad_trim_fade_out_slider)
                                             : s_pad_fade_out_ms[s_pad_trim_editor_pad];
    if (s_pad_trim_start_lbl) lv_label_set_text_fmt(s_pad_trim_start_lbl, "START  %d%%", start);
    if (s_pad_trim_end_lbl) lv_label_set_text_fmt(s_pad_trim_end_lbl, "END  %d%%", end);
    if (s_pad_trim_fade_in_lbl) {
        if (fadeIn == 0) lv_label_set_text(s_pad_trim_fade_in_lbl, "FADE IN  OFF");
        else lv_label_set_text_fmt(s_pad_trim_fade_in_lbl, "FADE IN  %d ms", fadeIn);
    }
    if (s_pad_trim_fade_out_lbl) {
        if (fadeOut == 0) lv_label_set_text(s_pad_trim_fade_out_lbl, "FADE OUT  OFF");
        else lv_label_set_text_fmt(s_pad_trim_fade_out_lbl, "FADE OUT  %d ms", fadeOut);
    }
}

static void pad_trim_slider_cb(lv_event_t* e) {
    if (!s_pad_trim_start_slider || !s_pad_trim_end_slider) return;
    int start = lv_slider_get_value(s_pad_trim_start_slider);
    int end = lv_slider_get_value(s_pad_trim_end_slider);
    if (start >= end - 2) {
        lv_obj_t* target = (lv_obj_t*)lv_event_get_target(e);
        if (target == s_pad_trim_start_slider) lv_slider_set_value(s_pad_trim_start_slider, end - 2, LV_ANIM_OFF);
        else lv_slider_set_value(s_pad_trim_end_slider, start + 2, LV_ANIM_OFF);
    }
    pad_trim_editor_refresh_values();
}

static void pad_trim_editor_close_cb(lv_event_t* e) {
    LV_UNUSED(e);
    if (s_pad_trim_modal) lv_obj_del(s_pad_trim_modal);
    s_pad_trim_modal = NULL;
    s_pad_trim_wave = NULL;
    s_pad_trim_start_slider = NULL;
    s_pad_trim_end_slider = NULL;
    s_pad_trim_start_lbl = NULL;
    s_pad_trim_end_lbl = NULL;
    s_pad_trim_fade_in_slider = NULL;
    s_pad_trim_fade_out_slider = NULL;
    s_pad_trim_fade_in_lbl = NULL;
    s_pad_trim_fade_out_lbl = NULL;
    s_pad_trim_editor_pad = -1;
}

static void pad_trim_preview_cb(lv_event_t* e) {
    LV_UNUSED(e);
    if (s_pad_trim_editor_pad < 0 || !control_available()) return;
    control_send_trigger((uint8_t)s_pad_trim_editor_pad, 112);
}

static void pad_trim_apply_cb(lv_event_t* e) {
    LV_UNUSED(e);
    if (s_pad_trim_editor_pad < 0) return;
    int pad = s_pad_trim_editor_pad;
    int start = s_pad_trim_start_slider ? lv_slider_get_value(s_pad_trim_start_slider) : 0;
    int end = s_pad_trim_end_slider ? lv_slider_get_value(s_pad_trim_end_slider) : 100;
    int fadeIn = s_pad_trim_fade_in_slider ? lv_slider_get_value(s_pad_trim_fade_in_slider) : 0;
    int fadeOut = s_pad_trim_fade_out_slider ? lv_slider_get_value(s_pad_trim_fade_out_slider) : 0;
    s_pad_trim_start_pct[pad] = (uint8_t)constrain(start, 0, 95);
    s_pad_trim_end_pct[pad] = (uint8_t)constrain(end, 5, 100);
    s_pad_fade_in_ms[pad] = (uint8_t)constrain(fadeIn, 0, 255);
    s_pad_fade_out_ms[pad] = (uint8_t)constrain(fadeOut, 0, 255);
    control_send_trim_sample((uint8_t)pad, s_pad_trim_start_pct[pad] / 100.0f,
                             s_pad_trim_end_pct[pad] / 100.0f);
    control_send_set_pad_fade_in((uint8_t)pad, s_pad_fade_in_ms[pad]);
    control_send_set_pad_fade_out((uint8_t)pad, s_pad_fade_out_ms[pad]);
    ui_show_toast("Trim aplicado al pad", theme_success());
}

static void pad_trim_editor_open(int pad) {
    if (pad < 0 || pad >= 16) return;
    pad_trim_editor_close_cb(NULL);
    s_pad_trim_editor_pad = pad;

    s_pad_trim_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_pad_trim_modal, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(s_pad_trim_modal, 0, 0);
    lv_obj_set_style_bg_color(s_pad_trim_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_pad_trim_modal, LV_OPA_70, 0);
    lv_obj_set_style_border_width(s_pad_trim_modal, 0, 0);
    lv_obj_set_style_pad_all(s_pad_trim_modal, 0, 0);
    lv_obj_clear_flag(s_pad_trim_modal, LV_OBJ_FLAG_SCROLLABLE);

    lv_color_t accent = lv_color_hex(theme_presets[ui_theme_index()].track_colors[pad]);

    lv_obj_t* card = lv_obj_create(s_pad_trim_modal);
    lv_obj_set_size(card, 900, 500);
    lv_obj_center(card);
    lv_obj_set_style_radius(card, 18, 0);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_color(card, RED808_BG, 0);
    lv_obj_set_style_bg_grad_dir(card, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_color(card, accent, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_pad_all(card, 0, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* title = lv_label_create(card);
    lv_label_set_text_fmt(title, "%s  ·  P%02d  ·  TRIM", trackNames[pad], pad + 1);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(title, accent, 0);
    lv_obj_set_pos(title, 24, 18);

    lv_obj_t* wave_card = lv_obj_create(card);
    lv_obj_set_pos(wave_card, 24, 62);
    lv_obj_set_size(wave_card, 852, 150);
    lv_obj_set_style_radius(wave_card, 12, 0);
    lv_obj_set_style_bg_color(wave_card, RED808_SURFACE, 0);
    lv_obj_set_style_border_color(wave_card, theme_border(), 0);
    lv_obj_set_style_border_width(wave_card, 1, 0);
    lv_obj_set_style_pad_all(wave_card, 0, 0);
    lv_obj_clear_flag(wave_card, LV_OBJ_FLAG_SCROLLABLE);

    bool real_wave = s_pad_wave_count[pad] == 96;
    uint32_t seed = 2166136261u;
    for (const char* c = trackNames[pad]; *c; ++c) seed = (seed ^ (uint8_t)*c) * 16777619u;
    for (int i = 0; i < 96; i++) {
        int vmax, vmin;
        if (real_wave) {
            vmax = s_pad_wave_max[pad][i];
            vmin = s_pad_wave_min[pad][i];
        } else {
            seed = seed * 1664525u + 1013904223u;
            int envelope = 20 + (int)((seed >> 25) & 31U);
            vmax = envelope;
            vmin = -envelope;
        }
        int x = 8 + i * 8;
        s_pad_trim_wave_points[i * 2].x = x;
        s_pad_trim_wave_points[i * 2].y = 75 - vmax * 64 / 127;
        s_pad_trim_wave_points[i * 2 + 1].x = x;
        s_pad_trim_wave_points[i * 2 + 1].y = 75 - vmin * 64 / 127;
    }
    s_pad_trim_wave = lv_line_create(wave_card);
    lv_line_set_points(s_pad_trim_wave, s_pad_trim_wave_points, 192);
    lv_obj_set_style_line_color(s_pad_trim_wave, accent, 0);
    lv_obj_set_style_line_width(s_pad_trim_wave, 2, 0);

    auto make_slider = [&](int y, int minv, int maxv, int value, lv_obj_t** out,
                           lv_obj_t** out_lbl) {
        *out_lbl = lv_label_create(card);
        lv_obj_set_style_text_font(*out_lbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(*out_lbl, theme_text(), 0);
        lv_obj_set_pos(*out_lbl, 28, y - 5);
        *out = lv_slider_create(card);
        lv_obj_set_pos(*out, 170, y);
        lv_obj_set_size(*out, 520, 18);
        lv_slider_set_range(*out, minv, maxv);
        lv_slider_set_value(*out, value, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(*out, RED808_SURFACE, LV_PART_MAIN);
        lv_obj_set_style_bg_color(*out, accent, LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(*out, lv_color_white(), LV_PART_KNOB);
        lv_obj_add_event_cb(*out, pad_trim_slider_cb, LV_EVENT_VALUE_CHANGED, NULL);
    };
    make_slider(236, 0, 95, s_pad_trim_start_pct[pad], &s_pad_trim_start_slider, &s_pad_trim_start_lbl);
    make_slider(280, 5, 100, s_pad_trim_end_pct[pad], &s_pad_trim_end_slider, &s_pad_trim_end_lbl);
    // Fade in/out — click-free ramps around the trim window's own edges.
    make_slider(324, 0, 255, s_pad_fade_in_ms[pad], &s_pad_trim_fade_in_slider, &s_pad_trim_fade_in_lbl);
    make_slider(368, 0, 255, s_pad_fade_out_ms[pad], &s_pad_trim_fade_out_slider, &s_pad_trim_fade_out_lbl);

    lv_obj_t* preview = piano_make_chip(card, 712, 232, 164, 54, "PREVIEW");
    lv_obj_set_style_border_color(preview, theme_success(), 0);
    lv_obj_add_event_cb(preview, pad_trim_preview_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* apply = piano_make_chip(card, 24, 422, 200, 52, "APLICAR TRIM");
    lv_obj_set_style_border_color(apply, accent, 0);
    lv_obj_add_event_cb(apply, pad_trim_apply_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* reset = piano_make_chip(card, 240, 422, 160, 52, "RESET");
    lv_obj_add_event_cb(reset, [](lv_event_t*) {
        if (s_pad_trim_editor_pad < 0) return;
        if (s_pad_trim_start_slider) lv_slider_set_value(s_pad_trim_start_slider, 0, LV_ANIM_OFF);
        if (s_pad_trim_end_slider) lv_slider_set_value(s_pad_trim_end_slider, 100, LV_ANIM_OFF);
        if (s_pad_trim_fade_in_slider) lv_slider_set_value(s_pad_trim_fade_in_slider, 0, LV_ANIM_OFF);
        if (s_pad_trim_fade_out_slider) lv_slider_set_value(s_pad_trim_fade_out_slider, 0, LV_ANIM_OFF);
        pad_trim_editor_refresh_values();
    }, LV_EVENT_CLICKED, NULL);
    lv_obj_t* close = piano_make_chip(card, 708, 422, 168, 52, "CERRAR");
    lv_obj_add_event_cb(close, pad_trim_editor_close_cb, LV_EVENT_CLICKED, NULL);

    pad_trim_editor_refresh_values();
}

static void create_sdcard_screen(void) {
    scr_sdcard = lv_obj_create(NULL);
    apply_screen_theme_bg(scr_sdcard);
    lv_obj_clear_flag(scr_sdcard, LV_OBJ_FLAG_SCROLLABLE);

    // Landscape layout: 1024×600
    // Left panel (file browser): 620px wide
    // Right panel (pad assign):  380px wide
    const int TOP    = 8;
    const int PANEL_H = LCD_V_RES - TOP - 8;  // ~584px
    const int LEFT_W  = 620;
    const int RIGHT_W = LCD_H_RES - LEFT_W - 16;  // ~392px
    const int GAP     = 8;

    // ── Left Panel: file browser ──
    sd_left_panel = lv_obj_create(scr_sdcard);
    lv_obj_set_size(sd_left_panel, LEFT_W, PANEL_H);
    lv_obj_set_pos(sd_left_panel, 4, TOP);
    lv_obj_set_style_bg_color(sd_left_panel, lv_color_hex(0x0D1520), 0);
    lv_obj_set_style_bg_opa(sd_left_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(sd_left_panel, RED808_INFO, 0);
    lv_obj_set_style_border_width(sd_left_panel, 1, 0);
    lv_obj_set_style_radius(sd_left_panel, 8, 0);
    lv_obj_set_style_pad_all(sd_left_panel, 8, 0);
    lv_obj_clear_flag(sd_left_panel, LV_OBJ_FLAG_SCROLLABLE);

    // Title
    lv_obj_t* title_lbl = lv_label_create(sd_left_panel);
    lv_label_set_text(title_lbl, LV_SYMBOL_DRIVE "  SD CARD BROWSER");
    lv_obj_set_style_text_font(title_lbl, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title_lbl, RED808_CYAN, 0);
    lv_obj_set_pos(title_lbl, 8, 4);

    // Source toggle: [P4 SD] [MEM]. Daisy no longer owns storage.
    {
        int btn_w = 70, btn_h = 28;
        int bx = 240, by = 4;
        sd_src_sd_btn = lv_btn_create(sd_left_panel);
        lv_obj_set_size(sd_src_sd_btn, btn_w, btn_h);
        lv_obj_set_pos(sd_src_sd_btn, bx, by);
        lv_obj_set_style_bg_color(sd_src_sd_btn,
            sd_source == 0 ? RED808_CYAN : lv_color_hex(0x1A2A3A), 0);
        lv_obj_set_style_radius(sd_src_sd_btn, 6, 0);
        lv_obj_set_style_border_width(sd_src_sd_btn, 1, 0);
        lv_obj_set_style_border_color(sd_src_sd_btn, lv_color_hex(0x334455), 0);
        lv_obj_t* l1 = lv_label_create(sd_src_sd_btn);
        lv_label_set_text(l1, "P4 SD");
        lv_obj_set_style_text_font(l1, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(l1, lv_color_black(), 0);
        lv_obj_center(l1);
        lv_obj_add_event_cb(sd_src_sd_btn, sd_source_btn_cb,
                            LV_EVENT_CLICKED, (void*)(intptr_t)0);

        sd_src_mem_btn = lv_btn_create(sd_left_panel);
        lv_obj_set_size(sd_src_mem_btn, btn_w, btn_h);
        lv_obj_set_pos(sd_src_mem_btn, bx + btn_w + 6, by);
        lv_obj_set_style_bg_color(sd_src_mem_btn,
            sd_source == 1 ? RED808_WARNING : lv_color_hex(0x1A2A3A), 0);
        lv_obj_set_style_radius(sd_src_mem_btn, 6, 0);
        lv_obj_set_style_border_width(sd_src_mem_btn, 1, 0);
        lv_obj_set_style_border_color(sd_src_mem_btn, lv_color_hex(0x334455), 0);
        lv_obj_t* l2 = lv_label_create(sd_src_mem_btn);
        lv_label_set_text(l2, "MEM");
        lv_obj_set_style_text_font(l2, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(l2, RED808_WARNING, 0);
        lv_obj_center(l2);
        lv_obj_add_event_cb(sd_src_mem_btn, sd_source_btn_cb,
                            LV_EVENT_CLICKED, (void*)(intptr_t)1);

        sd_src_daisy_btn = NULL;
    }

    // Status label
    sd_status_lbl = lv_label_create(sd_left_panel);
    lv_label_set_text(sd_status_lbl, "CONNECTING...");
    lv_obj_set_style_text_font(sd_status_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(sd_status_lbl, RED808_WARNING, 0);
    lv_obj_set_pos(sd_status_lbl, 490, 8);

    // Path label
    sd_path_lbl = lv_label_create(sd_left_panel);
    lv_label_set_text(sd_path_lbl, "/");
    lv_obj_set_style_text_font(sd_path_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(sd_path_lbl, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(sd_path_lbl, 8, 30);

    // Scrollable file list
    sd_file_list = lv_obj_create(sd_left_panel);
    lv_obj_set_size(sd_file_list, LEFT_W - 24, PANEL_H - 72);
    lv_obj_set_pos(sd_file_list, 4, 54);
    lv_obj_set_style_bg_opa(sd_file_list, LV_OPA_0, 0);
    lv_obj_set_style_border_width(sd_file_list, 0, 0);
    lv_obj_set_style_pad_row(sd_file_list, 4, 0);
    lv_obj_set_style_pad_all(sd_file_list, 2, 0);
    lv_obj_set_flex_flow(sd_file_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(sd_file_list, LV_DIR_VER);
    lv_obj_add_flag(sd_file_list, LV_OBJ_FLAG_SCROLLABLE);

    // ── Right Panel: WAV pad assign + MIDI pattern slot ──
    sd_right_panel = lv_obj_create(scr_sdcard);
    lv_obj_set_size(sd_right_panel, RIGHT_W, PANEL_H);
    lv_obj_set_pos(sd_right_panel, LEFT_W + GAP, TOP);
    lv_obj_set_style_bg_color(sd_right_panel, lv_color_hex(0x0D1520), 0);
    lv_obj_set_style_bg_opa(sd_right_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(sd_right_panel, RED808_ACCENT, 0);
    lv_obj_set_style_border_width(sd_right_panel, 1, 0);
    lv_obj_set_style_radius(sd_right_panel, 8, 0);
    lv_obj_set_style_pad_all(sd_right_panel, 8, 0);
    lv_obj_clear_flag(sd_right_panel, LV_OBJ_FLAG_SCROLLABLE);

    // ── WAV section (default visible) ─────────────────────────────────────
    sd_wav_section = lv_obj_create(sd_right_panel);
    lv_obj_set_size(sd_wav_section, LV_PCT(100), LV_PCT(100));
    lv_obj_set_pos(sd_wav_section, 0, 0);
    lv_obj_set_style_bg_opa(sd_wav_section, LV_OPA_0, 0);
    lv_obj_set_style_border_width(sd_wav_section, 0, 0);
    lv_obj_set_style_pad_all(sd_wav_section, 0, 0);
    lv_obj_clear_flag(sd_wav_section, LV_OBJ_FLAG_SCROLLABLE);

    // "ASSIGN TO PAD" title
    sd_assign_lbl = lv_label_create(sd_wav_section);
    lv_label_set_text(sd_assign_lbl, "ASSIGN TO PAD");
    lv_obj_set_style_text_font(sd_assign_lbl, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(sd_assign_lbl, RED808_ACCENT, 0);
    lv_obj_set_pos(sd_assign_lbl, 8, 4);

    // 4x4 pad grid — fit within RIGHT_W
    int pad_gap = 6;
    int pad_w = (RIGHT_W - 32 - 3 * pad_gap) / 4;
    int pad_h = 56;
    int px_start = 8, py_start = 36;
    for (int i = 0; i < 16; i++) {
        int col = i % 4;
        int row = i / 4;
        int px = px_start + col * (pad_w + pad_gap);
        int py = py_start + row * (pad_h + pad_gap);

        lv_obj_t* btn = lv_btn_create(sd_wav_section);
        lv_obj_set_size(btn, pad_w, pad_h);
        lv_obj_set_pos(btn, px, py);
        lv_obj_set_style_bg_color(btn, i == 0 ? RED808_ACCENT : lv_color_hex(0x222233), 0);
        lv_obj_set_style_radius(btn, 6, 0);
        lv_obj_set_style_border_width(btn, 1, 0);
        lv_obj_set_style_border_color(btn, lv_color_hex(0x444466), 0);

        lv_obj_t* num_lbl = lv_label_create(btn);
        char num_str[12];
        snprintf(num_str, sizeof(num_str), "%s\n%d", trackNames[i], i);
        lv_label_set_text(num_lbl, num_str);
        lv_obj_set_style_text_font(num_lbl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(num_lbl, lv_color_hex(theme_presets[ui_theme_index()].track_colors[i]), 0);
        lv_obj_set_style_text_align(num_lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_center(num_lbl);

        sd_pad_btns[i] = btn;
        lv_obj_add_event_cb(btn, sd_pad_btn_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
    }

    // Selected file label
    sd_selected_lbl = lv_label_create(sd_wav_section);
    lv_label_set_text(sd_selected_lbl, "");
    lv_obj_set_width(sd_selected_lbl, RIGHT_W - 24);
    lv_obj_set_style_text_font(sd_selected_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(sd_selected_lbl, RED808_SUCCESS, 0);
    lv_obj_set_style_text_align(sd_selected_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(sd_selected_lbl, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(sd_selected_lbl, 8, 300);

    sd_preview_btn = lv_btn_create(sd_wav_section);
    lv_obj_set_size(sd_preview_btn, RIGHT_W - 24, 48);
    lv_obj_set_pos(sd_preview_btn, 8, 308);
    lv_obj_set_style_bg_color(sd_preview_btn, RED808_SURFACE, 0);
    lv_obj_set_style_bg_color(sd_preview_btn, lv_color_hex(0x223344), LV_STATE_DISABLED);
    lv_obj_set_style_border_color(sd_preview_btn, RED808_CYAN, 0);
    lv_obj_set_style_border_width(sd_preview_btn, 2, 0);
    lv_obj_set_style_radius(sd_preview_btn, 10, 0);
    lv_obj_add_state(sd_preview_btn, LV_STATE_DISABLED);
    sd_preview_lbl = lv_label_create(sd_preview_btn);
    lv_label_set_text(sd_preview_lbl, LV_SYMBOL_PLAY "  PREVIEW PAD");
    lv_obj_set_style_text_font(sd_preview_lbl, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(sd_preview_lbl, lv_color_white(), 0);
    lv_obj_center(sd_preview_lbl);
    lv_obj_add_event_cb(sd_preview_btn, sd_preview_btn_cb, LV_EVENT_CLICKED, NULL);

    // LOAD WAV button
    sd_load_btn = lv_btn_create(sd_wav_section);
    lv_obj_set_size(sd_load_btn, RIGHT_W - 24, 60);
    lv_obj_set_pos(sd_load_btn, 8, 362);
    lv_obj_set_style_bg_color(sd_load_btn, RED808_ACCENT, 0);
    lv_obj_set_style_bg_color(sd_load_btn, lv_color_hex(0x882200), LV_STATE_DISABLED);
    lv_obj_set_style_radius(sd_load_btn, 10, 0);
    lv_obj_add_state(sd_load_btn, LV_STATE_DISABLED);

    sd_load_lbl = lv_label_create(sd_load_btn);
    lv_label_set_text(sd_load_lbl, LV_SYMBOL_UPLOAD "  LOAD TO PAD");
    lv_obj_set_style_text_font(sd_load_lbl, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(sd_load_lbl, lv_color_white(), 0);
    lv_obj_center(sd_load_lbl);
    lv_obj_add_event_cb(sd_load_btn, sd_load_btn_cb, LV_EVENT_CLICKED, NULL);

    // Trim editor for whichever pad is selected above — works whether it
    // just got a new WAV or already had one from a previous session.
    lv_obj_t* sd_edit_sample_btn = lv_btn_create(sd_wav_section);
    lv_obj_set_size(sd_edit_sample_btn, RIGHT_W - 24, 48);
    lv_obj_set_pos(sd_edit_sample_btn, 8, 432);
    apply_control_button_style(sd_edit_sample_btn, RED808_INFO, false, 10);
    lv_obj_t* sd_edit_sample_lbl = lv_label_create(sd_edit_sample_btn);
    lv_label_set_text(sd_edit_sample_lbl, LV_SYMBOL_EDIT "  EDITAR SAMPLE (TRIM)");
    lv_obj_set_style_text_font(sd_edit_sample_lbl, &lv_font_montserrat_16, 0);
    lv_obj_center(sd_edit_sample_lbl);
    lv_obj_add_event_cb(sd_edit_sample_btn, [](lv_event_t*) {
        pad_trim_editor_open(constrain(p4sd.selected_pad, 0, 15));
    }, LV_EVENT_CLICKED, NULL);

    // ── MIDI section (hidden by default) ──────────────────────────────────
    sd_midi_section = lv_obj_create(sd_right_panel);
    lv_obj_set_size(sd_midi_section, LV_PCT(100), LV_PCT(100));
    lv_obj_set_pos(sd_midi_section, 0, 0);
    lv_obj_set_style_bg_opa(sd_midi_section, LV_OPA_0, 0);
    lv_obj_set_style_border_width(sd_midi_section, 0, 0);
    lv_obj_set_style_pad_all(sd_midi_section, 0, 0);
    lv_obj_clear_flag(sd_midi_section, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(sd_midi_section, LV_OBJ_FLAG_HIDDEN);

    // MIDI title
    lv_obj_t* midi_title = lv_label_create(sd_midi_section);
    lv_label_set_text(midi_title, LV_SYMBOL_AUDIO "  MIDI IMPORT · PATTERN / SONG");
    lv_obj_set_style_text_font(midi_title, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(midi_title, RED808_WARNING, 0);
    lv_obj_set_pos(midi_title, 8, 4);

    // File info label
    sd_midi_info_lbl = lv_label_create(sd_midi_section);
    lv_label_set_text(sd_midi_info_lbl, "");
    lv_obj_set_style_text_font(sd_midi_info_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(sd_midi_info_lbl, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(sd_midi_info_lbl, 8, 30);
    lv_obj_set_width(sd_midi_info_lbl, RIGHT_W - 24);
    lv_label_set_long_mode(sd_midi_info_lbl, LV_LABEL_LONG_DOT);

    // "SELECT TARGET SLOT:"
    lv_obj_t* slot_lbl = lv_label_create(sd_midi_section);
    lv_label_set_text(slot_lbl, "SELECT TARGET PATTERN SLOT:");
    lv_obj_set_style_text_font(slot_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(slot_lbl, RED808_CYAN, 0);
    lv_obj_set_pos(slot_lbl, 8, 54);

    // Pattern slot grid 2×3 (P01-P06 = slots 0-5, valid master patterns)
    {
        int mp_btn_w = (RIGHT_W - 16 - 10) / 2;  // 2 cols, gap=10
        int mp_btn_h = 52;
        int mp_gap   = 6;
        int mp_x0 = 0, mp_y0 = 76;
        for (int i = 0; i < 6; i++) {
            int col = i % 2, row = i / 2;
            int slot_id = i;
            int bx = mp_x0 + col * (mp_btn_w + 10);
            int by = mp_y0 + row * (mp_btn_h + mp_gap);

            lv_obj_t* btn = lv_btn_create(sd_midi_section);
            lv_obj_set_size(btn, mp_btn_w, mp_btn_h);
            lv_obj_set_pos(btn, bx, by);
            lv_obj_set_style_bg_color(btn,
                slot_id == sd_midi_target_slot ? RED808_ACCENT : lv_color_hex(0x1A2A3A), 0);
            lv_obj_set_style_border_width(btn, 2, 0);
            lv_obj_set_style_border_color(btn,
                slot_id == sd_midi_target_slot ? RED808_CYAN : lv_color_hex(0x334455), 0);
            lv_obj_set_style_radius(btn, 8, 0);

            lv_obj_t* bl = lv_label_create(btn);
            char bname[8];
            snprintf(bname, sizeof(bname), "P%02d", slot_id + 1);
            lv_label_set_text(bl, bname);
            lv_obj_set_style_text_font(bl, &lv_font_montserrat_20, 0);
            lv_obj_set_style_text_color(bl,
                slot_id == sd_midi_target_slot ? lv_color_white() : RED808_TEXT_DIM, 0);
            lv_obj_center(bl);

            sd_midi_pat_btns[i] = btn;
            lv_obj_add_event_cb(btn, sd_midi_pat_btn_cb, LV_EVENT_CLICKED, (void*)(intptr_t)slot_id);
        }
    }

    // Import-mode selector: PRO / STD
    {
        lv_obj_t* ml = lv_label_create(sd_midi_section);
        lv_label_set_text(ml, "IMPORT MODE:");
        lv_obj_set_style_text_font(ml, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(ml, RED808_CYAN, 0);
        lv_obj_set_pos(ml, 8, 258);

        auto make_mode_btn = [](lv_obj_t* parent, int x, int y, int w, int h,
                                const char* title, const char* subtitle,
                                bool active) -> lv_obj_t* {
            lv_obj_t* btn = lv_btn_create(parent);
            lv_obj_set_size(btn, w, h);
            lv_obj_set_pos(btn, x, y);
            lv_obj_set_style_radius(btn, 8, 0);
            lv_obj_set_style_border_width(btn, 2, 0);
            lv_obj_set_style_bg_color(btn,
                active ? RED808_ACCENT : lv_color_hex(0x1A2A3A), 0);
            lv_obj_set_style_border_color(btn,
                active ? RED808_CYAN : lv_color_hex(0x334455), 0);
            lv_obj_t* t = lv_label_create(btn);
            lv_label_set_text(t, title);
            lv_obj_set_style_text_font(t, &lv_font_montserrat_18, 0);
            lv_obj_set_style_text_color(t,
                active ? lv_color_white() : RED808_TEXT, 0);
            lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 6);
            lv_obj_t* s = lv_label_create(btn);
            lv_label_set_text(s, subtitle);
            lv_obj_set_style_text_font(s, &lv_font_montserrat_10, 0);
            lv_obj_set_style_text_color(s,
                active ? lv_color_white() : RED808_TEXT_DIM, 0);
            lv_obj_align(s, LV_ALIGN_BOTTOM_MID, 0, -4);
            return btn;
        };

        int mb_w = (RIGHT_W - 16 - 10) / 2;
        int mb_h = 50;
        sd_midi_mode_pro_btn = make_mode_btn(sd_midi_section,
            0, 282, mb_w, mb_h, "PRO", "all channels",
            sd_midi_import_mode == 0);
        sd_midi_mode_std_btn = make_mode_btn(sd_midi_section,
            mb_w + 10, 282, mb_w, mb_h, "STD", "GM drums only",
            sd_midi_import_mode == 1);

        lv_obj_add_event_cb(sd_midi_mode_pro_btn, [](lv_event_t*){
            sd_midi_import_mode = 0;
            if (sd_midi_mode_pro_btn) {
                lv_obj_set_style_bg_color(sd_midi_mode_pro_btn, RED808_ACCENT, 0);
                lv_obj_set_style_border_color(sd_midi_mode_pro_btn, RED808_CYAN, 0);
            }
            if (sd_midi_mode_std_btn) {
                lv_obj_set_style_bg_color(sd_midi_mode_std_btn, lv_color_hex(0x1A2A3A), 0);
                lv_obj_set_style_border_color(sd_midi_mode_std_btn, lv_color_hex(0x334455), 0);
            }
        }, LV_EVENT_CLICKED, NULL);
        lv_obj_add_event_cb(sd_midi_mode_std_btn, [](lv_event_t*){
            sd_midi_import_mode = 1;
            if (sd_midi_mode_std_btn) {
                lv_obj_set_style_bg_color(sd_midi_mode_std_btn, RED808_ACCENT, 0);
                lv_obj_set_style_border_color(sd_midi_mode_std_btn, RED808_CYAN, 0);
            }
            if (sd_midi_mode_pro_btn) {
                lv_obj_set_style_bg_color(sd_midi_mode_pro_btn, lv_color_hex(0x1A2A3A), 0);
                lv_obj_set_style_border_color(sd_midi_mode_pro_btn, lv_color_hex(0x334455), 0);
            }
        }, LV_EVENT_CLICKED, NULL);
    }

    // Pattern and full-arrangement actions. SONG is deliberately separated:
    // it replaces persisted user scenes and therefore opens a confirmation.
    const int midi_action_w = (RIGHT_W - 24 - 8) / 2;
    sd_midi_load_btn = lv_btn_create(sd_midi_section);
    lv_obj_set_size(sd_midi_load_btn, midi_action_w, 56);
    lv_obj_set_pos(sd_midi_load_btn, 8, 362);
    lv_obj_set_style_bg_color(sd_midi_load_btn, RED808_WARNING, 0);
    lv_obj_set_style_bg_color(sd_midi_load_btn, lv_color_hex(0x554400), LV_STATE_DISABLED);
    lv_obj_set_style_radius(sd_midi_load_btn, 10, 0);
    lv_obj_add_state(sd_midi_load_btn, LV_STATE_DISABLED);
    lv_obj_add_event_cb(sd_midi_load_btn, sd_midi_load_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* midi_load_lbl = lv_label_create(sd_midi_load_btn);
    lv_label_set_text(midi_load_lbl, LV_SYMBOL_DOWNLOAD "  PATTERN");
    lv_obj_set_style_text_font(midi_load_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(midi_load_lbl, lv_color_black(), 0);
    lv_obj_center(midi_load_lbl);

    sd_midi_song_btn = lv_btn_create(sd_midi_section);
    lv_obj_set_size(sd_midi_song_btn, midi_action_w, 56);
    lv_obj_set_pos(sd_midi_song_btn, 8 + midi_action_w + 8, 362);
    lv_obj_set_style_bg_color(sd_midi_song_btn, RED808_CYAN, 0);
    lv_obj_set_style_bg_color(sd_midi_song_btn, lv_color_hex(0x17343A), LV_STATE_DISABLED);
    lv_obj_set_style_radius(sd_midi_song_btn, 10, 0);
    lv_obj_add_state(sd_midi_song_btn, LV_STATE_DISABLED);
    lv_obj_add_event_cb(sd_midi_song_btn, sd_midi_song_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* midi_song_lbl = lv_label_create(sd_midi_song_btn);
    lv_label_set_text(midi_song_lbl, LV_SYMBOL_LIST "  FULL SONG");
    lv_obj_set_style_text_font(midi_song_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(midi_song_lbl, RED808_BG, 0);
    lv_obj_center(midi_song_lbl);

    // Status label
    sd_midi_status_lbl = lv_label_create(sd_midi_section);
    lv_label_set_text(sd_midi_status_lbl, "");
    lv_obj_set_style_text_font(sd_midi_status_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(sd_midi_status_lbl, RED808_SUCCESS, 0);
    lv_obj_set_pos(sd_midi_status_lbl, 8, 426);
    lv_obj_set_width(sd_midi_status_lbl, RIGHT_W - 24);
    lv_label_set_long_mode(sd_midi_status_lbl, LV_LABEL_LONG_WRAP);

    // BACK button (return to live)
    lv_obj_t* back_btn = lv_btn_create(sd_right_panel);
    lv_obj_set_size(back_btn, RIGHT_W - 24, 50);
    lv_obj_set_pos(back_btn, 8, PANEL_H - 74);
    lv_obj_set_style_bg_color(back_btn, RED808_SURFACE, 0);
    lv_obj_set_style_radius(back_btn, 8, 0);
    lv_obj_set_style_border_width(back_btn, 1, 0);
    lv_obj_set_style_border_color(back_btn, RED808_BORDER, 0);
    lv_obj_t* back_lbl = lv_label_create(back_btn);
    lv_label_set_text(back_lbl, LV_SYMBOL_LEFT "  BACK TO LIVE");
    lv_obj_set_style_text_font(back_lbl, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(back_lbl, RED808_TEXT, 0);
    lv_obj_center(back_lbl);
    lv_obj_add_event_cb(back_btn, [](lv_event_t* e) {
        (void)e;
        ui_navigate_to(2);  // SCREEN_LIVE
    }, LV_EVENT_CLICKED, NULL);
}

// =============================================================================
// PIANO SCREEN — live keyboard (12 / 24 keys) → DaisyPod3 synth via USB
// Engines: 3=303, 4=WTosc, 5=SH101, 6=FM2Op.
// =============================================================================
static int  s_piano_engine_idx = 0;          // index into PIANO_ENGINES
static int  s_piano_octave     = 4;          // C4..C5
static bool s_piano_two_oct    = false;      // false=12 keys, true=24 keys
static uint8_t s_piano_velocity = 100;
static int  s_piano_held_note  = -1;         // last note-on still ringing
static bool s_piano_note_active[128] = {false};
static bool s_piano_rec_active = false;      // v2.7 — record to S3 melody screen
// v2.8 — local mirror of recorded notes so P4 can ASSIGN to a pad without
// round-tripping through S3. Same shape as S3: 16 steps × 12 pitch-classes.
static bool s_piano_rec_grid[16][12] = {{false}};
static uint8_t s_piano_rec_notes[16][12] = {{0}};
static int  s_piano_rec_step  = 0;
static int  s_piano_assign_pad = 0;          // 0..15
static uint8_t s_piano_rec_engine = 3;
static uint8_t s_piano_rec_octave = 4;
static bool s_piano_rec_has_notes = false;
static std::atomic<uint8_t> s_piano_gate_percent{55};
static uint32_t s_piano_rec_last_ms = 0;
static int s_piano_rec_last_col = -1;

// The piano deliberately exposes the four musical engines only.
static constexpr int PIANO_ENGINE_COUNT = 4;
static const uint8_t PIANO_ENGINES[PIANO_ENGINE_COUNT]      = {3, 4, 5, 6};
static const char*   PIANO_ENGINE_LABELS[PIANO_ENGINE_COUNT] = {"303", "WT", "SH101", "FM2"};
// Signature color per engine — used by the selector chips and the pressed-key
// glow, so the active synth is readable at a glance from across the room.
static const uint32_t PIANO_ENGINE_COLORS[PIANO_ENGINE_COUNT] = {
    0xFFE066,   // 303 — acid yellow
    0x00E5FF,   // WT — cyan
    0xB07CFF,   // SH101 — violet
    0xFF8C42,   // FM2 — orange
};

static lv_obj_t* s_piano_engine_btns[PIANO_ENGINE_COUNT]   = {NULL, NULL, NULL, NULL};
static lv_obj_t* s_piano_octave_lbl       = NULL;
static lv_obj_t* s_piano_keys24_btn       = NULL;
static lv_obj_t* s_piano_keys24_lbl       = NULL;
static lv_obj_t* s_piano_status_lbl       = NULL;
static lv_obj_t* s_piano_keys_container   = NULL;
static lv_obj_t* s_piano_expr_bar         = NULL;
static lv_obj_t* s_piano_rec_btn          = NULL;  // v2.7
static lv_obj_t* s_piano_rec_lbl          = NULL;  // v2.7
static lv_obj_t* s_piano_pad_lbl          = NULL;  // v2.8
static lv_obj_t* s_piano_glide_btn        = NULL;
static lv_obj_t* s_piano_glide_lbl        = NULL;
static lv_obj_t* s_piano_bend_btn         = NULL;
static lv_obj_t* s_piano_bend_lbl         = NULL;
static lv_obj_t* s_piano_gate_btn         = NULL;
static lv_obj_t* s_piano_gate_lbl         = NULL;
// Per-engine sound-preset chips, shown on the piano page and relabeled when
// the engine is selected. Selection tracked per piano engine.
static lv_obj_t* s_piano_eng_preset_btns[4] = {NULL, NULL, NULL, NULL};
static int       s_piano_preset_sel[PIANO_ENGINE_COUNT] = {0, 0, 0, 0};
static void piano_refresh_engine_presets(void);

// Piano gesture state (v3.0): hold+drag for glide and pitch bend.
static bool      s_piano_glide_enabled     = false;
static bool      s_piano_bend_enabled      = false;
static int       s_piano_bend_range_st     = 2;     // default +/-2 semitones
static bool      s_piano_gesture_active    = false;
static int16_t   s_piano_touch_start_x     = 0;
static int16_t   s_piano_touch_start_y     = 0;
static uint8_t   s_piano_touch_base_note   = 60;
static int       s_piano_last_slide_note   = -1;
static float     s_piano_last_bend_st      = 0.0f;
static uint32_t  s_piano_last_bend_send_ms = 0;
static float     s_piano_expr_base_cutoff  = 8000.0f;
static float     s_piano_expr_last_cutoff  = 8000.0f;
static float     s_piano_expr_base_volume  = 0.75f;
static float     s_piano_expr_last_volume  = 0.75f;
static float     s_piano_expr_last_amount  = 0.0f;
static uint32_t  s_piano_expr_last_send_ms = 0;
static bool      s_piano_release_pending   = false;
static uint32_t  s_piano_release_due_ms    = 0;
static lv_obj_t* s_piano_key_obj_by_note[128] = {NULL};
static constexpr uint32_t PIANO_RELEASE_DEBOUNCE_MS = 24;

// Forward declarations for melody grid (defined after key handlers)
static void piano_grid_refresh_cell(int col, int row);
static void piano_grid_refresh_all(void);
static bool piano_vertical_expression_active(void);
static void piano_apply_vertical_expression(int16_t ly);
static void piano_reset_vertical_expression(void);
static void piano_update_status_note(uint8_t midi_note);
static void piano_update_expression_status(void);
static void piano_update_expression_bar(void);
static void piano_sync_active_engine_state(void);

static inline bool piano_pc_is_black(uint8_t pc) {
    return (pc == 1) || (pc == 3) || (pc == 6) || (pc == 8) || (pc == 10);
}

static const char* piano_note_name(uint8_t midi) {
    static const char* NAMES[12] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
    static char buf[8];
    int oct = (midi / 12) - 1;
    snprintf(buf, sizeof(buf), "%s%d", NAMES[midi % 12], oct);
    return buf;
}

static inline uint8_t piano_engine_code(void) {
    return PIANO_ENGINES[s_piano_engine_idx];
}

static void piano_set_key_visual(lv_obj_t* btn, uint8_t note, bool pressed) {
    if (!btn) return;
    bool is_black = piano_pc_is_black(note % 12);
    lv_color_t eng = lv_color_hex(PIANO_ENGINE_COLORS[s_piano_engine_idx]);
    lv_obj_set_style_bg_color(btn,
        pressed ? eng : (is_black ? lv_color_hex(0x141414) : lv_color_hex(0xFAFAF2)), 0);
    // Pressed keys glow in the active engine's color; black keys keep a
    // subtle drop shadow for depth when idle.
    lv_obj_set_style_shadow_color(btn, pressed ? eng : lv_color_black(), 0);
    lv_obj_set_style_shadow_opa(btn,
        pressed ? LV_OPA_80 : (is_black ? LV_OPA_40 : LV_OPA_0), 0);
}

static void piano_set_note_active(uint8_t note, bool pressed) {
    if (note > 127) return;
    s_piano_note_active[note] = pressed;
    if (s_piano_key_obj_by_note[note]) {
        piano_set_key_visual(s_piano_key_obj_by_note[note], note, pressed);
    }
}

static bool piano_poly_mode_active(void) {
    return piano_engine_code() == 4 && !s_piano_glide_enabled && !s_piano_bend_enabled;
}

static void piano_send_note_off_specific(uint8_t midi_note) {
    if (!ui_control_available()) return;
    if (piano_engine_code() == 4) {
        control_send_synth_note_off_ex(piano_engine_code(), 0, midi_note);
        return;
    }
    control_send_synth_note_off(piano_engine_code(), 0);
}

static void piano_send_engine_all_notes_off(uint8_t engine) {
    if (!ui_control_available()) return;
    control_send_synth_note_off(engine, 0xFF);
}

static void piano_send_panic_melodic(void) {
    if (!ui_control_available()) return;
    for (int i = 0; i < PIANO_ENGINE_COUNT; i++) {
        piano_send_engine_all_notes_off(PIANO_ENGINES[i]);
    }
#if P4_ENABLE_DEBUG_LOG
    Serial.printf("[P4 piano] panic melodic engines=%d\n", PIANO_ENGINE_COUNT);
#endif
}

static void piano_reset_bend(void) {
    if (!ui_control_available()) return;
    uint8_t engine = piano_engine_code();
    if (engine == 3) {
        control_send_synth_param(engine, 0, 14, 0.0f);   // PitchBend
    } else if (engine == 6) {
        control_send_synth_param(engine, 0, 12, 0.0f);   // Detune cents
    }
    s_piano_last_bend_st = 0.0f;
}

static void piano_apply_glide_setting(void) {
    if (!ui_control_available()) return;
    uint8_t engine = piano_engine_code();
    if (engine == 3) {
        control_send_synth_param(engine, 0, 5, s_piano_glide_enabled ? 0.12f : 0.01f);
    } else if (engine == 5) {
        control_send_synth_param(engine, 0, 17, s_piano_glide_enabled ? 0.35f : 0.0f);
    }
}

static void piano_send_off(void);
static void piano_send_on(uint8_t midi_note, bool legato);

static void piano_refresh_gesture_controls(void) {
    if (s_piano_glide_lbl) {
        lv_label_set_text(s_piano_glide_lbl, s_piano_glide_enabled ? "GLIDE ON" : "GLIDE OFF");
    }
    if (s_piano_glide_btn) {
        lv_obj_set_style_border_color(s_piano_glide_btn,
            s_piano_glide_enabled ? RED808_CYAN : RED808_BORDER, 0);
        lv_obj_set_style_border_width(s_piano_glide_btn, s_piano_glide_enabled ? 2 : 1, 0);
    }

    if (s_piano_bend_lbl) {
        lv_label_set_text(s_piano_bend_lbl, s_piano_bend_enabled ? "BEND ON" : "BEND OFF");
    }
    if (s_piano_bend_btn) {
        lv_obj_set_style_border_color(s_piano_bend_btn,
            s_piano_bend_enabled ? RED808_SUCCESS : RED808_BORDER, 0);
        lv_obj_set_style_border_width(s_piano_bend_btn, s_piano_bend_enabled ? 2 : 1, 0);
    }
}

static bool piano_get_local_touch(int16_t* lx, int16_t* ly) {
    if (!s_piano_keys_container || !lx || !ly) return false;
    lv_indev_t* indev = lv_indev_get_act();
    if (!indev) return false;
    lv_point_t p;
    lv_indev_get_point(indev, &p);
    lv_area_t a;
    lv_obj_get_coords(s_piano_keys_container, &a);
    *lx = (int16_t)(p.x - a.x1);
    *ly = (int16_t)(p.y - a.y1);
    return true;
}

static bool piano_note_from_local_xy(int16_t lx, int16_t ly, uint8_t* note_out) {
    if (!s_piano_keys_container || !note_out) return false;
    int container_w = lv_obj_get_width(s_piano_keys_container);
    int container_h = lv_obj_get_height(s_piano_keys_container);
    if (container_w < 80 || container_h < 80) return false;
    if (lx < 0 || lx >= container_w || ly < 0 || ly >= container_h) return false;

    int num_octaves = s_piano_two_oct ? 2 : 1;
    int num_white = num_octaves * 7;
    if (num_white < 1) num_white = 1;
    int white_w = container_w / num_white;
    if (white_w < 4) white_w = 4;
    int white_h = container_h;
    int black_w = (white_w * 6) / 10;
    int black_h = (white_h * 6) / 10;

    static const uint8_t WHITE_PCS[7] = {0, 2, 4, 5, 7, 9, 11};
    static const uint8_t BLACK_PCS[5] = {1, 3, 6, 8, 10};
    static const uint8_t BLACK_AFTER_WHITE[5] = {0, 1, 3, 4, 5};

    int base_midi = (s_piano_octave + 1) * 12;

    if (ly < black_h) {
        for (int oct = 0; oct < num_octaves; oct++) {
            for (int b = 0; b < 5; b++) {
                int wpos = BLACK_AFTER_WHITE[b];
                int x0 = (oct * 7 + wpos) * white_w + white_w - black_w / 2;
                int x1 = x0 + black_w;
                if (lx >= x0 && lx < x1) {
                    int midi = base_midi + oct * 12 + BLACK_PCS[b];
                    if (midi >= 0 && midi <= 127) {
                        *note_out = (uint8_t)midi;
                        return true;
                    }
                }
            }
        }
    }

    int white_idx = lx / white_w;
    if (white_idx < 0) white_idx = 0;
    if (white_idx >= num_white) white_idx = num_white - 1;
    int oct = white_idx / 7;
    int w = white_idx % 7;
    int midi = base_midi + oct * 12 + WHITE_PCS[w];
    if (midi < 0 || midi > 127) return false;
    *note_out = (uint8_t)midi;
    return true;
}

static bool piano_touch_inside_keys(void) {
    int16_t lx = 0, ly = 0;
    if (!piano_get_local_touch(&lx, &ly)) return false;
    uint8_t dummy = 0;
    return piano_note_from_local_xy(lx, ly, &dummy);
}

static void piano_send_bend(float bend_st) {
    if (!ui_control_available()) return;
    uint8_t engine = piano_engine_code();
    uint32_t now = millis();
    if ((now - s_piano_last_bend_send_ms) < 20 && fabsf(bend_st - s_piano_last_bend_st) < 0.08f) return;

    if (engine == 3) {
        if (bend_st < -12.0f) bend_st = -12.0f;
        if (bend_st > 12.0f) bend_st = 12.0f;
        control_send_synth_param(engine, 0, 14, bend_st);
        s_piano_last_bend_st = bend_st;
        s_piano_last_bend_send_ms = now;
    } else if (engine == 6) {
        float det = bend_st * 25.0f;  // map +/-2st gesture to +/-50ct detune
        if (det < -50.0f) det = -50.0f;
        if (det > 50.0f) det = 50.0f;
        control_send_synth_param(engine, 0, 12, det);
        s_piano_last_bend_st = bend_st;
        s_piano_last_bend_send_ms = now;
    }
}

static void piano_send_off(void) {
    int held_note = s_piano_held_note;
    if (held_note >= 0 && ui_control_available()) {
        if (piano_engine_code() == 4) {
            control_send_synth_note_off_ex(piano_engine_code(), 0, (uint8_t)held_note);
        } else {
            control_send_synth_note_off(piano_engine_code(), 0);
        }
    }
    s_piano_release_pending = false;
    s_piano_release_due_ms = 0;
    s_piano_held_note = -1;
    memset(s_piano_note_active, 0, sizeof(s_piano_note_active));
    piano_reset_vertical_expression();
    piano_reset_bend();
    for (int note = 0; note < 128; note++) {
        if (s_piano_key_obj_by_note[note]) {
            piano_set_key_visual(s_piano_key_obj_by_note[note], (uint8_t)note, false);
        }
    }
    if (s_piano_status_lbl) lv_label_set_text(s_piano_status_lbl, "—");
}

static void piano_schedule_release(void) {
    if (s_piano_held_note < 0) return;
    s_piano_release_pending = true;
    s_piano_release_due_ms = millis() + PIANO_RELEASE_DEBOUNCE_MS;
}

static void piano_cancel_release(void) {
    s_piano_release_pending = false;
    s_piano_release_due_ms = 0;
}

static void piano_send_on(uint8_t midi_note, bool legato) {
    piano_cancel_release();
    bool poly_mode = piano_poly_mode_active();
    if (!legato) {
        if (!poly_mode) piano_send_off();
    } else if (s_piano_held_note < 0) {
        legato = false;
    }
    uint8_t attack_velocity = lvgl_port_get_touch_velocity();
    if (attack_velocity == 0) attack_velocity = s_piano_velocity;
#if P4_ENABLE_DEBUG_LOG
    Serial.printf("[P4 piano] note=%u vel=%u rec=%d transport=%d legato=%d\n", midi_note, (unsigned)attack_velocity, (int)s_piano_rec_active, (int)ui_control_available(), (int)legato);
#endif
    piano_apply_glide_setting();
    if (ui_control_available()) {
        control_send_synth_note_on_ex(piano_engine_code(),
                                   midi_note, attack_velocity, false, legato && s_piano_glide_enabled);
        if (s_piano_rec_active) {
#if P4_ENABLE_DEBUG_LOG
            Serial.printf("[P4 piano] -> melodyRecNote eng=%u note=%u\n", piano_engine_code(), midi_note);
#endif
            control_send_melody_rec_note(piano_engine_code(), midi_note);
        }
    }
    if (s_piano_rec_active) {
        local_apply_message(MSG_TOUCH_CMD, TCMD_MELODY_NOTE, midi_note);
        int pc = midi_note % 12;
        int row = -1;
        for (int r = 0; r < 12; r++) {
            if ((11 - r) == pc) { row = r; break; }
        }
        if (row >= 0) {
            uint32_t rec_now = millis();
            bool chord_note = s_piano_rec_last_col >= 0 &&
                              (uint32_t)(rec_now - s_piano_rec_last_ms) <= 90;
            int col = chord_note ? s_piano_rec_last_col : s_piano_rec_step;
            if (col < 0 || col >= 16) col = 0;
            s_piano_rec_grid[col][row] = true;
            s_piano_rec_notes[col][row] = midi_note;
            s_piano_rec_has_notes = true;
            piano_grid_refresh_cell(col, row);
            if (!chord_note) s_piano_rec_step = (col + 1) % 16;
            s_piano_rec_last_col = col;
            s_piano_rec_last_ms = rec_now;
        }
    }
    s_piano_held_note = (int)midi_note;
    s_piano_last_slide_note = (int)midi_note;
    piano_set_note_active(midi_note, true);
    piano_update_status_note(midi_note);
}

static void piano_handle_pressing(void) {
    if (!s_piano_gesture_active || s_piano_held_note < 0) return;
    int16_t lx = 0, ly = 0;
    if (!piano_get_local_touch(&lx, &ly)) return;

    if (piano_vertical_expression_active()) {
        piano_apply_vertical_expression(ly);
    }

    uint8_t touch_note = 0;
    if (s_piano_glide_enabled && piano_note_from_local_xy(lx, ly, &touch_note)) {
        if ((int)touch_note != s_piano_last_slide_note) {
            piano_send_on(touch_note, true);
            s_piano_touch_start_x = lx;
            s_piano_touch_base_note = touch_note;
            s_piano_last_bend_st = 0.0f;
        }
    }

    if (!s_piano_bend_enabled) return;

    int container_w = lv_obj_get_width(s_piano_keys_container);
    int num_white = (s_piano_two_oct ? 2 : 1) * 7;
    if (num_white < 1) num_white = 1;
    int white_w = container_w / num_white;
    if (white_w < 10) white_w = 10;
    float sens_px = (float)(white_w * 2);  // full bend across ~2 white keys
    float bend_st = ((float)(lx - s_piano_touch_start_x) / sens_px) * (float)s_piano_bend_range_st;
    if (bend_st < -(float)s_piano_bend_range_st) bend_st = -(float)s_piano_bend_range_st;
    if (bend_st > (float)s_piano_bend_range_st) bend_st = (float)s_piano_bend_range_st;

    uint8_t engine = piano_engine_code();
    if (engine == 3 || engine == 6) {
        piano_send_bend(bend_st);
    } else {
        int bend_steps = (int)lroundf(bend_st);
        int note = (int)s_piano_touch_base_note + bend_steps;
        if (note < 0) note = 0;
        if (note > 127) note = 127;
        if (note != s_piano_held_note) {
            piano_send_on((uint8_t)note, true);
        }
    }
}

static void piano_key_event_cb(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    uint8_t note = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    bool poly_mode = piano_poly_mode_active();
    if (code == LV_EVENT_PRESSED) {
        bool from_glide = !poly_mode && s_piano_gesture_active && s_piano_held_note >= 0;
        piano_cancel_release();
        int16_t lx = 0, ly = 0;
        piano_get_local_touch(&lx, &ly);
        if (!poly_mode) {
            s_piano_gesture_active = true;
            s_piano_touch_start_x = lx;
            s_piano_touch_start_y = ly;
            s_piano_touch_base_note = note;
            s_piano_last_slide_note = note;
            s_piano_last_bend_st = 0.0f;
            s_piano_last_bend_send_ms = 0;
        }
        if (piano_vertical_expression_active()) {
            s_piano_touch_start_y = ly;
            s_piano_expr_base_cutoff = s_piano_expr_last_cutoff;
            s_piano_expr_base_volume = s_piano_expr_last_volume;
        }
        piano_send_on(note, from_glide);
    } else if (code == LV_EVENT_PRESSING) {
        piano_cancel_release();
        if (!poly_mode) piano_handle_pressing();
    } else if (code == LV_EVENT_PRESS_LOST) {
        if (poly_mode) {
            piano_set_note_active(note, false);
            piano_send_note_off_specific(note);
            if (!piano_touch_inside_keys()) {
                piano_reset_vertical_expression();
            }
            return;
        }
        if (piano_touch_inside_keys()) {
            // Keep gesture alive while sliding across adjacent keys.
            piano_cancel_release();
            piano_handle_pressing();
            return;
        }
        s_piano_gesture_active = false;
        piano_schedule_release();
    } else if (code == LV_EVENT_RELEASED) {
        if (poly_mode) {
            piano_set_note_active(note, false);
            piano_send_note_off_specific(note);
            if (!piano_touch_inside_keys()) {
                piano_reset_vertical_expression();
            }
            return;
        }
        s_piano_gesture_active = false;
        piano_schedule_release();
    }
}

// Publishes the complete piano state through the in-process controller.
// Garantiza que el receptor aplique un estado coherente sin mezclar defaults.
static void piano_publish_local_state(void) {
    local_apply_message(MSG_TOUCH_CMD, TCMD_MELODY_ENGINE, PIANO_ENGINES[s_piano_engine_idx]);
    local_apply_message(MSG_TOUCH_CMD, TCMD_MELODY_OCTAVE, (uint8_t)s_piano_octave);
    local_apply_message(MSG_TOUCH_CMD, TCMD_MELODY_REC,    s_piano_rec_active ? 1 : 0);
    local_apply_message(MSG_TOUCH_CMD, TCMD_MELODY_PAD,    (uint8_t)s_piano_assign_pad);
}

static void piano_refresh_engine_chips(void) {
    for (int i = 0; i < PIANO_ENGINE_COUNT; i++) {
        if (!s_piano_engine_btns[i]) continue;
        bool sel = (i == s_piano_engine_idx);
        lv_color_t ec = lv_color_hex(PIANO_ENGINE_COLORS[i]);
        lv_obj_set_style_bg_color(s_piano_engine_btns[i], sel ? ec : RED808_SURFACE, 0);
        lv_obj_set_style_border_color(s_piano_engine_btns[i], ec, 0);
        lv_obj_set_style_border_opa(s_piano_engine_btns[i], sel ? LV_OPA_COVER : LV_OPA_50, 0);
        lv_obj_set_style_shadow_width(s_piano_engine_btns[i], sel ? 14 : 0, 0);
        lv_obj_set_style_shadow_color(s_piano_engine_btns[i], ec, 0);
        lv_obj_set_style_shadow_opa(s_piano_engine_btns[i], sel ? LV_OPA_50 : LV_OPA_0, 0);
        lv_obj_t* lbl = lv_obj_get_child(s_piano_engine_btns[i], 0);
        if (lbl) lv_obj_set_style_text_color(lbl, sel ? RED808_BG : ec, 0);
    }
    if (s_piano_status_lbl) {
        lv_label_set_text_fmt(s_piano_status_lbl, "ENG %s  OCT %d  PAD %d",
                              PIANO_ENGINE_LABELS[s_piano_engine_idx],
                              s_piano_octave, s_piano_assign_pad + 1);
    }
    // Engine changed → relabel the per-engine sound presets.
    piano_refresh_engine_presets();
}

static void piano_engine_btn_cb(lv_event_t* e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (idx < 0 || idx >= PIANO_ENGINE_COUNT) return;
#if P4_ENABLE_DEBUG_LOG
    uint8_t old_engine = piano_engine_code();
#endif
    piano_send_off();
    piano_send_panic_melodic();
    s_piano_engine_idx = idx;
#if P4_ENABLE_DEBUG_LOG
    Serial.printf("[P4 piano] engine %u -> %u\n", old_engine, piano_engine_code());
#endif
    piano_refresh_engine_chips();
    // v2.9 — broadcast engine selection through master
    if (ui_control_available()) {
        control_send_melody_set_engine(PIANO_ENGINES[s_piano_engine_idx]);
    }
    piano_sync_active_engine_state();
    // Synchronize all piano fields in the local controller.
    piano_publish_local_state();
}

static void piano_rebuild_keys(void);

static void piano_octave_btn_cb(lv_event_t* e) {
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    int next = s_piano_octave + delta;
    if (next < 1) next = 1;
    if (next > 7) next = 7;
    if (next == s_piano_octave) return;
    piano_send_off();
    s_piano_octave = next;
    if (s_piano_octave_lbl)
        lv_label_set_text_fmt(s_piano_octave_lbl, "OCT %d", s_piano_octave);
    piano_rebuild_keys();
    // v2.9 — broadcast octave through master
    if (ui_control_available()) control_send_melody_set_octave((uint8_t)s_piano_octave);
    // Synchronize all piano fields in the local controller.
    piano_publish_local_state();
}

static void piano_keys24_btn_cb(lv_event_t* e) {
    LV_UNUSED(e);
    piano_send_off();
    s_piano_two_oct = !s_piano_two_oct;
    if (s_piano_keys24_lbl)
        lv_label_set_text(s_piano_keys24_lbl, s_piano_two_oct ? "24 K" : "12 K");
    piano_rebuild_keys();
}

static void piano_glide_btn_cb(lv_event_t* e) {
    LV_UNUSED(e);
    s_piano_glide_enabled = !s_piano_glide_enabled;
    // Avoid zipper/clicks: if a note is currently held, apply on next note-on.
    if (s_piano_held_note < 0) {
        piano_apply_glide_setting();
    }
    piano_refresh_gesture_controls();
}

static void piano_bend_btn_cb(lv_event_t* e) {
    LV_UNUSED(e);
    s_piano_bend_enabled = !s_piano_bend_enabled;
    // Avoid abrupt pitch jump while holding a note.
    if (!s_piano_bend_enabled && s_piano_held_note < 0) {
        piano_reset_bend();
    }
    piano_refresh_gesture_controls();
}

static void piano_rec_btn_cb(lv_event_t* e) {
    LV_UNUSED(e);
    static uint32_t s_last_p4_rec_toggle_ms = 0;
    uint32_t now = millis();
    if ((now - s_last_p4_rec_toggle_ms) < 280U) return;
    s_last_p4_rec_toggle_ms = now;
    s_piano_rec_active = !s_piano_rec_active;
    if (s_piano_rec_active) {
        // v2.8 — fresh take: clear local mirror and rewind step cursor
        memset(s_piano_rec_grid, 0, sizeof(s_piano_rec_grid));
        memset(s_piano_rec_notes, 0, sizeof(s_piano_rec_notes));
        s_piano_rec_step = 0;
        s_piano_rec_engine = PIANO_ENGINES[s_piano_engine_idx];
        s_piano_rec_octave = (uint8_t)s_piano_octave;
        s_piano_rec_has_notes = false;
        piano_grid_refresh_all();
    }
    // v2.9 — tell master so all slaves mirror REC state and grid clear
    if (ui_control_available()) {
        control_send_melody_rec_toggle(s_piano_rec_active,
                                   PIANO_ENGINES[s_piano_engine_idx],
                                   (uint8_t)s_piano_octave);
    }
    // Synchronize all piano fields in the local controller.
    piano_publish_local_state();
    if (s_piano_rec_btn) {
        lv_obj_set_style_border_color(s_piano_rec_btn,
            s_piano_rec_active ? lv_color_hex(0xFF3030) : RED808_BORDER, 0);
        lv_obj_set_style_border_width(s_piano_rec_btn, s_piano_rec_active ? 3 : 1, 0);
    }
    if (s_piano_rec_lbl) {
        lv_obj_set_style_text_color(s_piano_rec_lbl,
            s_piano_rec_active ? lv_color_hex(0xFF3030) : RED808_TEXT, 0);
        lv_label_set_text(s_piano_rec_lbl, s_piano_rec_active ? "● REC" : "○ REC");
    }
}

// v2.8 — pad +/- chips: cycle assign target across pads 1..16
static void piano_pad_btn_cb(lv_event_t* e) {
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    s_piano_assign_pad = (s_piano_assign_pad + delta + 16) % 16;
    if (s_piano_pad_lbl) {
        lv_label_set_text_fmt(s_piano_pad_lbl, "PAD %d", s_piano_assign_pad + 1);
    }
    // v2.9 — broadcast pad selection through master
    if (ui_control_available()) control_send_melody_set_pad((uint8_t)s_piano_assign_pad);
    // Synchronize all piano fields in the local controller.
    piano_publish_local_state();
}

// =============================================================================
// PIANO MELODY GRID — 16 steps × 12 pitch-classes editor with preview
// (P3/P4 Prioridad 4: visual partitura, edición, presets, play)
// =============================================================================
static lv_obj_t* s_piano_grid_container = NULL;
static lv_obj_t* s_piano_grid_btns[16][12] = {{NULL}};
static lv_obj_t* s_piano_play_btn = NULL;
static lv_obj_t* s_piano_play_lbl = NULL;
static bool      s_piano_play_active   = false;
static int       s_piano_play_step     = 0;
static uint32_t  s_piano_play_next_ms  = 0;
static uint32_t  s_piano_play_off_due_ms = 0;
static int       s_piano_play_held_note = -1;
static uint8_t   s_piano_play_notes[12] = {};
static uint8_t   s_piano_play_note_count = 0;

// Row 0 = B (top), Row 11 = C (bottom). Matches existing recording mapping.
static const uint8_t PIANO_ROW_TO_PC[12] = {11,10,9,8,7,6,5,4,3,2,1,0};

static uint8_t piano_midi_for_grid_cell(int col, int row, uint8_t fallback_octave) {
    if (col >= 0 && col < 16 && row >= 0 && row < 12 && s_piano_rec_notes[col][row] > 0) {
        return s_piano_rec_notes[col][row];
    }
    int pc = (row >= 0 && row < 12) ? PIANO_ROW_TO_PC[row] : 0;
    int midi = ((int)fallback_octave + 1) * 12 + pc;
    if (midi < 0) midi = 0;
    if (midi > 127) midi = 127;
    return (uint8_t)midi;
}

static void piano_grid_refresh_cell(int col, int row) {
    if (col < 0 || col >= 16 || row < 0 || row >= 12) return;
    lv_obj_t* b = s_piano_grid_btns[col][row];
    if (!b) return;
    bool on      = s_piano_rec_grid[col][row];
    bool playing = (s_piano_play_active && col == s_piano_play_step);
    bool black   = piano_pc_is_black(PIANO_ROW_TO_PC[row]);
    bool beat    = ((col & 3) == 0);   // lighten beats 1/5/9/13 for orientation
    uint32_t color;
    if (on && playing)       color = 0xFFD000;       // yellow — note + playhead
    else if (on)             color = 0x00E5FF;       // cyan — armed note
    else if (playing)        color = 0x303030;       // dim grey — playhead
    else if (black)          color = beat ? 0x16222C : 0x101820;
    else                     color = beat ? 0x2A343C : 0x222A30;
    lv_obj_set_style_bg_color(b, lv_color_hex(color), 0);
    lv_obj_set_style_bg_opa(b, on ? LV_OPA_COVER : (playing ? LV_OPA_70 : LV_OPA_60), 0);
}

static void piano_grid_refresh_all(void) {
    for (int c = 0; c < 16; c++)
        for (int r = 0; r < 12; r++)
            piano_grid_refresh_cell(c, r);
}

void piano_grid_visual_refresh_external(void) {
    piano_grid_refresh_all();
}

static void piano_grid_cell_cb(lv_event_t* e) {
    int packed = (int)(intptr_t)lv_event_get_user_data(e);
    int col = (packed >> 8) & 0xFF;
    int row = packed & 0xFF;
    if (col < 0 || col >= 16 || row < 0 || row >= 12) return;
    s_piano_rec_grid[col][row] = !s_piano_rec_grid[col][row];
    s_piano_rec_notes[col][row] = s_piano_rec_grid[col][row]
        ? piano_midi_for_grid_cell(col, row, (uint8_t)s_piano_octave)
        : 0;
    if (s_piano_rec_grid[col][row]) {
        s_piano_rec_engine = PIANO_ENGINES[s_piano_engine_idx];
        s_piano_rec_octave = (uint8_t)s_piano_octave;
        s_piano_rec_has_notes = true;
    }
    piano_grid_refresh_cell(col, row);
}

static void piano_grid_clear(void) {
    memset(s_piano_rec_grid, 0, sizeof(s_piano_rec_grid));
    memset(s_piano_rec_notes, 0, sizeof(s_piano_rec_notes));
    s_piano_rec_step = 0;
    s_piano_rec_has_notes = false;
    s_piano_rec_last_ms = 0;
    s_piano_rec_last_col = -1;
    piano_grid_refresh_all();
}

static void piano_apply_preset(int idx) {
    memset(s_piano_rec_grid, 0, sizeof(s_piano_rec_grid));
    memset(s_piano_rec_notes, 0, sizeof(s_piano_rec_notes));
    s_piano_rec_engine = PIANO_ENGINES[s_piano_engine_idx];
    s_piano_rec_octave = (uint8_t)s_piano_octave;
    s_piano_rec_has_notes = true;
    // (col, pc) pairs. pc 0=C ... 11=B.
    static const uint8_t PRESET_BASS[][2] = {
        {0,9},{2,9},{4,9},{6,12 % 12},{8,9},{10,7},{12,9},{14,7}
    };
    static const uint8_t PRESET_ARP[][2] = {
        {0,0},{2,4},{4,7},{6,0},{8,4},{10,7},{12,0},{14,7}
    };
    static const uint8_t PRESET_SCALE[][2] = {
        {0,0},{1,2},{2,4},{3,5},{4,7},{5,9},{6,11},{7,0},
        {8,11},{9,9},{10,7},{11,5},{12,4},{13,2},{14,0},{15,7}
    };
    const uint8_t (*p)[2] = NULL;
    int len = 0;
    switch (idx) {
        case 0: p = PRESET_BASS;  len = sizeof(PRESET_BASS)/2;  break;
        case 1: p = PRESET_ARP;   len = sizeof(PRESET_ARP)/2;   break;
        case 2: p = PRESET_SCALE; len = sizeof(PRESET_SCALE)/2; break;
        default: return;
    }
    for (int i = 0; i < len; i++) {
        int col = p[i][0];
        int pc  = p[i][1] % 12;
        for (int r = 0; r < 12; r++) {
            if (PIANO_ROW_TO_PC[r] == (uint8_t)pc) {
                if (col >= 0 && col < 16) {
                    s_piano_rec_grid[col][r] = true;
                    s_piano_rec_notes[col][r] = piano_midi_for_grid_cell(col, r, s_piano_rec_octave);
                }
                break;
            }
        }
    }
    piano_grid_refresh_all();
}

static void piano_preset_btn_cb(lv_event_t* e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    piano_apply_preset(idx);
}

static void piano_clear_btn_cb(lv_event_t* e) {
    LV_UNUSED(e);
    piano_grid_clear();
    if (ui_control_available()) control_send_melody_clear();
}

static void piano_play_step_off(void) {
    if (s_piano_play_held_note < 0) return;
    if (ui_control_available()) {
        uint8_t engine = PIANO_ENGINES[s_piano_engine_idx];
        if (engine == 4 && s_piano_play_note_count > 0) {
            for (uint8_t i = 0; i < s_piano_play_note_count; i++) {
                control_send_synth_note_off_ex(engine, 0, s_piano_play_notes[i]);
            }
        } else if (engine == 3) {
            control_send_synth303_note_off();
        } else {
            control_send_synth_note_off(engine, 0);
        }
    }
    s_piano_play_held_note = -1;
    s_piano_play_note_count = 0;
}

static void piano_gate_btn_cb(lv_event_t* e) {
    LV_UNUSED(e);
    static const uint8_t gates[] = {35, 55, 75, 95};
    int idx = 0;
    uint8_t current = s_piano_gate_percent.load(std::memory_order_relaxed);
    for (int i = 0; i < 4; i++) if (gates[i] == current) idx = i;
    uint8_t next = gates[(idx + 1) & 3];
    s_piano_gate_percent.store(next, std::memory_order_relaxed);
    if (s_piano_gate_lbl) lv_label_set_text_fmt(s_piano_gate_lbl, "GATE %u%%",
                                                (unsigned)next);
}

static void piano_play_btn_cb(lv_event_t* e) {
    LV_UNUSED(e);
    s_piano_play_active     = !s_piano_play_active;
    s_piano_play_step       = -1;   // first tick advances to 0
    s_piano_play_next_ms    = millis();
    s_piano_play_off_due_ms = 0;
    piano_play_step_off();
    if (s_piano_play_lbl)
        lv_label_set_text(s_piano_play_lbl, s_piano_play_active ? "■ STOP" : "▶ PLAY");
    if (s_piano_play_btn) {
        lv_obj_set_style_border_color(s_piano_play_btn,
            s_piano_play_active ? RED808_SUCCESS : RED808_BORDER, 0);
        lv_obj_set_style_border_width(s_piano_play_btn, s_piano_play_active ? 3 : 1, 0);
    }
    if (!s_piano_play_active) piano_grid_refresh_all();
}

// Per-engine sound-preset chips on the piano page. Relabels from the
// selected engine's SP_ENGINES presets and applies via control_send_synth_preset.
static void piano_refresh_engine_presets(void) {
    int pp_idx = pp_engine_idx_from_code(PIANO_ENGINES[s_piano_engine_idx]);
    if (pp_idx < 0 || pp_idx >= SP_ENGINE_COUNT) return;
    const SynthEngineDef* eng = &SP_ENGINES[pp_idx];
    lv_color_t ec = lv_color_hex(PIANO_ENGINE_COLORS[s_piano_engine_idx]);
    int sel = s_piano_preset_sel[s_piano_engine_idx];
    for (int i = 0; i < 4; i++) {
        lv_obj_t* b = s_piano_eng_preset_btns[i];
        if (!b) continue;
        if (i < eng->preset_count) {
            lv_obj_clear_flag(b, LV_OBJ_FLAG_HIDDEN);
            bool on = (i == sel);
            lv_obj_set_style_bg_color(b, on ? ec : RED808_SURFACE, 0);
            lv_obj_set_style_border_color(b, ec, 0);
            lv_obj_set_style_border_opa(b, on ? LV_OPA_COVER : LV_OPA_50, 0);
            lv_obj_t* l = lv_obj_get_child(b, 0);
            if (l) {
                lv_label_set_text(l, eng->presets[i].name);
                lv_obj_set_style_text_color(l, on ? RED808_BG : ec, 0);
            }
        } else {
            lv_obj_add_flag(b, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void piano_eng_preset_cb(lv_event_t* e) {
    int preset_idx = (int)(intptr_t)lv_event_get_user_data(e);
    int pp_idx = pp_engine_idx_from_code(PIANO_ENGINES[s_piano_engine_idx]);
    if (pp_idx < 0 || pp_idx >= SP_ENGINE_COUNT) return;
    const SynthEngineDef* eng = &SP_ENGINES[pp_idx];
    if (preset_idx < 0 || preset_idx >= eng->preset_count) return;
    s_piano_preset_sel[s_piano_engine_idx] = preset_idx;
    if (ui_control_available()) control_send_synth_preset(eng->engine, (uint8_t)preset_idx);
    piano_refresh_engine_presets();
    ui_show_toast(eng->presets[preset_idx].name, theme_success());
}

void update_piano_screen(void) {
    uint32_t now = millis();
    if (s_piano_release_pending && s_piano_held_note >= 0) {
        if (piano_touch_inside_keys()) {
            piano_cancel_release();
        } else if ((int32_t)(now - s_piano_release_due_ms) >= 0) {
            piano_send_off();
        }
    }
    if (!s_piano_play_active) return;
    // Follow the global transport tempo (the BPM control was removed from the
    // piano UI — there is one tempo for the whole instrument).
    float bpm = (float)p4.bpm_int + (float)p4.bpm_frac * 0.1f;
    if (bpm < 40.0f) bpm = 120.0f;
    uint32_t step_ms = (uint32_t)(60000.0f / bpm / 4.0f); // 16th note
    if (step_ms < 30) step_ms = 30;

    // Note-off scheduling for the currently held note
    if (s_piano_play_held_note >= 0 &&
        (int32_t)(now - s_piano_play_off_due_ms) >= 0) {
        piano_play_step_off();
    }

    if ((int32_t)(now - s_piano_play_next_ms) < 0) return;
    int prev = s_piano_play_step;
    uint32_t late = now - s_piano_play_next_ms;
    uint32_t advance = 1U + late / step_ms;
    int next = (prev + (int)(advance % 16U)) % 16;
    if (next < 0) next += 16;
    s_piano_play_step = next;

    if (prev >= 0 && prev < 16)
        for (int r = 0; r < 12; r++) piano_grid_refresh_cell(prev, r);
    for (int r = 0; r < 12; r++) piano_grid_refresh_cell(next, r);

    // WT is polyphonic: preserve chords recorded in one column. Mono engines
    // use the lowest note in the column, matching their voice architecture.
    uint8_t engine = PIANO_ENGINES[s_piano_engine_idx];
    piano_play_step_off();
    s_piano_play_note_count = 0;
    for (int r = 11; r >= 0; r--) {
        if (s_piano_rec_grid[next][r]) {
            int midi = piano_midi_for_grid_cell(next, r, (uint8_t)s_piano_octave);
            if (ui_control_available()) {
                control_send_synth_note_on_ex(engine, (uint8_t)midi,
                                          s_piano_velocity, false, false);
            }
            if (s_piano_play_note_count < 12)
                s_piano_play_notes[s_piano_play_note_count++] = (uint8_t)midi;
            s_piano_play_held_note = midi;
            if (engine != 4) break;
        }
    }
    if (s_piano_play_note_count > 0) {
        uint8_t gate = s_piano_gate_percent.load(std::memory_order_relaxed);
        s_piano_play_off_due_ms = now +
            (uint32_t)((uint64_t)step_ms * gate / 100U);
    }
    // Advance from the prior deadline, not from `now`, so a slow UI frame
    // never accumulates tempo drift. Missed steps are skipped, not burst-fired.
    s_piano_play_next_ms += advance * step_ms;
}

// v2.8 — push the locally recorded grid to master as a melodyAssign packet
static void piano_assign_btn_cb(lv_event_t* e) {
    LV_UNUSED(e);
    if (!ui_control_available()) return;
    uint8_t assign_engine = s_piano_rec_has_notes ? s_piano_rec_engine : PIANO_ENGINES[s_piano_engine_idx];
    uint8_t assign_octave = s_piano_rec_has_notes ? s_piano_rec_octave : (uint8_t)s_piano_octave;
    control_send_melody_assign((uint8_t)s_piano_assign_pad,
                           assign_engine,
                           assign_octave,
                           s_piano_rec_grid,
                           s_piano_rec_notes,
                           s_piano_gate_percent.load(std::memory_order_relaxed));
    if (s_piano_status_lbl) {
        lv_label_set_text_fmt(s_piano_status_lbl, "→ PAD %d", s_piano_assign_pad + 1);
    }
    char msg[64];
    snprintf(msg, sizeof(msg), "Melodia asignada a PAD %02d", s_piano_assign_pad + 1);
    ui_show_toast(msg, RED808_SUCCESS);
}

// v2.9 — Apply melody_sync payload from master (engine/octave/rec/pad).
// Must be called from within lvgl_port_lock.
void piano_apply_melody_sync(uint8_t engine, uint8_t octave, bool rec, uint8_t pad) {
    // Map engine code 3..6 → index 0..3; legacy engine 7 keeps the current one.
    int new_idx = s_piano_engine_idx;
    for (int i = 0; i < PIANO_ENGINE_COUNT; i++) {
        if (PIANO_ENGINES[i] == engine) { new_idx = i; break; }
    }
    bool octave_changed = ((int)octave != s_piano_octave && octave >= 1 && octave <= 7);
    s_piano_engine_idx = new_idx;
    if (octave_changed) s_piano_octave = (int)octave;
    if (pad < 16) s_piano_assign_pad = (int)pad;
    s_piano_rec_active = rec;

    piano_refresh_engine_chips();
    // Refresh octave label
    if (s_piano_octave_lbl)
        lv_label_set_text_fmt(s_piano_octave_lbl, "OCT %d", s_piano_octave);
    // Rebuild key layout only when octave changes (expensive but necessary)
    if (octave_changed) piano_rebuild_keys();
    // Refresh REC button visual
    if (s_piano_rec_btn) {
        lv_obj_set_style_border_color(s_piano_rec_btn,
            rec ? lv_color_hex(0xFF3030) : RED808_BORDER, 0);
        lv_obj_set_style_border_width(s_piano_rec_btn, rec ? 3 : 1, 0);
    }
    if (s_piano_rec_lbl) {
        lv_obj_set_style_text_color(s_piano_rec_lbl,
            rec ? lv_color_hex(0xFF3030) : RED808_TEXT, 0);
        lv_label_set_text(s_piano_rec_lbl, rec ? "● REC" : "○ REC");
    }
    // Refresh pad label
    if (s_piano_pad_lbl)
        lv_label_set_text_fmt(s_piano_pad_lbl, "PAD %d", s_piano_assign_pad + 1);
    // Refresh status label
    if (s_piano_status_lbl) {
        lv_label_set_text_fmt(s_piano_status_lbl, "ENG %s  OCT %d  PAD %d",
                              PIANO_ENGINE_LABELS[new_idx],
                              s_piano_octave, s_piano_assign_pad + 1);
    }
}

static lv_obj_t* piano_make_chip(lv_obj_t* parent, int x, int y, int w, int h,
                                 const char* text) {
    lv_obj_t* btn = lv_btn_create(parent);
    lv_obj_set_size(btn, w, h);
    lv_obj_set_pos(btn, x, y);
    apply_control_button_style(btn, RED808_BORDER, false, 8);
    lv_obj_t* lbl = lv_label_create(btn);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(lbl, RED808_TEXT, 0);
    lv_obj_center(lbl);
    return btn;
}

static void piano_rebuild_keys(void) {
    if (!s_piano_keys_container) return;
    lv_obj_update_layout(s_piano_keys_container);
    lv_obj_clean(s_piano_keys_container);
    memset(s_piano_key_obj_by_note, 0, sizeof(s_piano_key_obj_by_note));
    memset(s_piano_note_active, 0, sizeof(s_piano_note_active));

    int container_w = lv_obj_get_width(s_piano_keys_container);
    int container_h = lv_obj_get_height(s_piano_keys_container);
    if (container_w < 100) container_w = ui_layout_w();
    if (container_h < 100) container_h = ui_layout_h() - 152;
    if (container_h < 160) container_h = 160;
    int num_octaves = s_piano_two_oct ? 2 : 1;
    int num_white   = num_octaves * 7;
    if (num_white < 1) num_white = 1;
    int white_w     = container_w / num_white;
    int white_h     = container_h;
    int black_w     = (white_w * 6) / 10;
    int black_h     = (white_h * 6) / 10;

    static const uint8_t WHITE_PCS[7]         = {0, 2, 4, 5, 7, 9, 11};
    static const uint8_t BLACK_PCS[5]         = {1, 3, 6, 8, 10};
    /* Black key sits between white index N and N+1 (within an octave) */
    static const uint8_t BLACK_AFTER_WHITE[5] = {0, 1, 3, 4, 5};

    int base_midi = (s_piano_octave + 1) * 12;   // C(s_piano_octave) MIDI

    /* White keys — ivory gradient, note name on every key (C carries the
     * octave number in the engine color, the rest stay dim). */
    static const char* WHITE_NAMES[7] = {"C", "D", "E", "F", "G", "A", "B"};
    lv_color_t eng_color = lv_color_hex(PIANO_ENGINE_COLORS[s_piano_engine_idx]);
    for (int oct = 0; oct < num_octaves; oct++) {
        for (int w = 0; w < 7; w++) {
            int x = (oct * 7 + w) * white_w;
            uint8_t midi = base_midi + oct * 12 + WHITE_PCS[w];
            lv_obj_t* k = lv_btn_create(s_piano_keys_container);
            lv_obj_set_pos(k, x, 0);
            lv_obj_set_size(k, white_w - 2, white_h - 2);
            lv_obj_set_style_radius(k, 6, 0);
            lv_obj_set_style_bg_color(k, lv_color_hex(0xFAFAF2), 0);
            lv_obj_set_style_bg_grad_color(k, lv_color_hex(0xC4C4B8), 0);
            lv_obj_set_style_bg_grad_dir(k, LV_GRAD_DIR_VER, 0);
            lv_obj_set_style_bg_opa(k, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(k, 1, 0);
            lv_obj_set_style_border_color(k, lv_color_hex(0x404040), 0);
            // Shadow reserved for the pressed engine-glow (opa toggled in
            // piano_set_key_visual); width must be pre-set or LVGL skips it.
            lv_obj_set_style_shadow_width(k, 16, 0);
            lv_obj_set_style_shadow_opa(k, LV_OPA_0, 0);
            lv_obj_set_style_bg_color(k, eng_color, LV_STATE_PRESSED);
            lv_obj_set_style_translate_y(k, 3, LV_STATE_PRESSED);
            lv_obj_set_style_outline_width(k, 2, LV_STATE_PRESSED);
            lv_obj_set_style_outline_color(k, eng_color, LV_STATE_PRESSED);
            lv_obj_clear_flag(k, LV_OBJ_FLAG_SCROLLABLE);
            s_piano_key_obj_by_note[midi] = k;
            lv_obj_add_event_cb(k, piano_key_event_cb, LV_EVENT_PRESSED, (void*)(uintptr_t)midi);
            lv_obj_add_event_cb(k, piano_key_event_cb, LV_EVENT_PRESSING, (void*)(uintptr_t)midi);
            lv_obj_add_event_cb(k, piano_key_event_cb, LV_EVENT_RELEASED, (void*)(uintptr_t)midi);
            lv_obj_add_event_cb(k, piano_key_event_cb, LV_EVENT_PRESS_LOST, (void*)(uintptr_t)midi);
            lv_obj_t* lbl = lv_label_create(k);
            if (w == 0) {
                lv_label_set_text_fmt(lbl, "C%d", s_piano_octave + oct);
                lv_obj_set_style_text_color(lbl, lv_color_hex(0x1A1A1A), 0);
                lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
            } else {
                lv_label_set_text(lbl, WHITE_NAMES[w]);
                lv_obj_set_style_text_color(lbl, lv_color_hex(0x9A9A90), 0);
                lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, 0);
            }
            lv_obj_align(lbl, LV_ALIGN_BOTTOM_MID, 0, -6);
        }
    }
    /* Black keys (drawn last so they appear on top) */
    for (int oct = 0; oct < num_octaves; oct++) {
        for (int b = 0; b < 5; b++) {
            int wpos = BLACK_AFTER_WHITE[b];
            int x = (oct * 7 + wpos) * white_w + white_w - black_w / 2;
            uint8_t midi = base_midi + oct * 12 + BLACK_PCS[b];
            lv_obj_t* k = lv_btn_create(s_piano_keys_container);
            lv_obj_set_pos(k, x, 0);
            lv_obj_set_size(k, black_w, black_h);
            lv_obj_set_style_radius(k, 5, 0);
            lv_obj_set_style_bg_color(k, lv_color_hex(0x262626), 0);
            lv_obj_set_style_bg_grad_color(k, lv_color_hex(0x0A0A0A), 0);
            lv_obj_set_style_bg_grad_dir(k, LV_GRAD_DIR_VER, 0);
            lv_obj_set_style_bg_opa(k, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(k, 1, 0);
            lv_obj_set_style_border_color(k, lv_color_hex(0x000000), 0);
            // Drop shadow gives the black keys depth over the whites; the
            // same shadow flips to the engine glow while pressed.
            lv_obj_set_style_shadow_width(k, 12, 0);
            lv_obj_set_style_shadow_ofs_y(k, 4, 0);
            lv_obj_set_style_shadow_color(k, lv_color_black(), 0);
            lv_obj_set_style_shadow_opa(k, LV_OPA_40, 0);
            lv_obj_set_style_bg_color(k, eng_color, LV_STATE_PRESSED);
            lv_obj_set_style_translate_y(k, 3, LV_STATE_PRESSED);
            lv_obj_set_style_outline_width(k, 2, LV_STATE_PRESSED);
            lv_obj_set_style_outline_color(k, eng_color, LV_STATE_PRESSED);
            lv_obj_clear_flag(k, LV_OBJ_FLAG_SCROLLABLE);
            s_piano_key_obj_by_note[midi] = k;
            lv_obj_add_event_cb(k, piano_key_event_cb, LV_EVENT_PRESSED, (void*)(uintptr_t)midi);
            lv_obj_add_event_cb(k, piano_key_event_cb, LV_EVENT_PRESSING, (void*)(uintptr_t)midi);
            lv_obj_add_event_cb(k, piano_key_event_cb, LV_EVENT_RELEASED, (void*)(uintptr_t)midi);
            lv_obj_add_event_cb(k, piano_key_event_cb, LV_EVENT_PRESS_LOST, (void*)(uintptr_t)midi);
        }
    }
}

// =============================================================================
// MELODY PRESETS — 8 savable snapshots of the piano melody grid (notes +
// engine + octave + gate). Phase 3 of the SONG "MATRIX" plan (see FILTER
// PRESETS / MIXER PRESETS above for Phases 1-2). A melody preset is a note
// sequence independent of any pattern — recalling one only loads it into the
// piano editor; MATRIX (Phase 4/5) is what will later push a recalled preset
// onto a track's steps for a given song column, reusing the existing ASSIGN
// path (control_send_melody_assign) rather than duplicating it here.
// =============================================================================
#define MELODY_PRESET_COUNT 8
struct MelodyPresetSlot {
    bool used;
    char name[16];
    bool grid[16][12];
    uint8_t notes[16][12];
    uint8_t engine;
    uint8_t octave;
    uint8_t gatePercent;
};
static MelodyPresetSlot s_melody_presets[MELODY_PRESET_COUNT] = {};
static const char* MELODY_PRESETS_FILE = "/melody_presets.txt";

static lv_obj_t* s_melody_preset_modal = NULL;
static lv_obj_t* s_melody_preset_slot_btns[MELODY_PRESET_COUNT] = {};
static lv_obj_t* s_melody_preset_slot_lbls[MELODY_PRESET_COUNT] = {};

// One header line "used,name,engine,octave,gate" per slot, followed by 16
// lines of 12 comma-separated MIDI notes (one per grid column) — grid[on] is
// derived from notes>0 on load, so it doesn't need its own storage.
static void melody_presets_save_to_disk(void) {
    File f = SPIFFS.open(MELODY_PRESETS_FILE, FILE_WRITE);
    if (!f) return;
    for (int i = 0; i < MELODY_PRESET_COUNT; i++) {
        const MelodyPresetSlot& s = s_melody_presets[i];
        f.printf("%d,%s,%d,%d,%d\n", s.used ? 1 : 0, s.name[0] ? s.name : "-",
                 s.engine, s.octave, s.gatePercent);
        for (int c = 0; c < 16; c++) {
            for (int r = 0; r < 12; r++) f.printf(r ? ",%d" : "%d", s.notes[c][r]);
            f.printf("\n");
        }
    }
    f.close();
}

static void melody_presets_load_from_disk(void) {
    memset(s_melody_presets, 0, sizeof(s_melody_presets));
    for (int i = 0; i < MELODY_PRESET_COUNT; i++) {
        s_melody_presets[i].engine = 3;
        s_melody_presets[i].octave = 4;
        s_melody_presets[i].gatePercent = 55;
    }
    File f = SPIFFS.open(MELODY_PRESETS_FILE, FILE_READ);
    if (!f) return;
    char line[160];
    for (int i = 0; i < MELODY_PRESET_COUNT; i++) {
        if (!fs_read_line(f, line, sizeof(line))) break;
        MelodyPresetSlot& s = s_melody_presets[i];
        int used = 0, engine = 3, octave = 4, gate = 55;
        char name[16] = {};
        int parsed = sscanf(line, "%d,%15[^,],%d,%d,%d", &used, name, &engine, &octave, &gate);
        s.used = (used != 0);
        if (parsed >= 2) strncpy(s.name, name, sizeof(s.name) - 1);
        s.engine = (uint8_t)constrain(engine, 0, 255);
        s.octave = (uint8_t)constrain(octave, 1, 7);
        s.gatePercent = (uint8_t)constrain(gate, 1, 100);
        for (int c = 0; c < 16; c++) {
            if (!fs_read_line(f, line, sizeof(line))) break;
            int vals[12] = {};
            int n = sscanf(line, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                &vals[0], &vals[1], &vals[2], &vals[3], &vals[4], &vals[5],
                &vals[6], &vals[7], &vals[8], &vals[9], &vals[10], &vals[11]);
            for (int r = 0; r < (n < 12 ? n : 12); r++) {
                s.notes[c][r] = (uint8_t)constrain(vals[r], 0, 127);
                s.grid[c][r] = s.notes[c][r] > 0;
            }
        }
    }
    f.close();
}

static void melody_preset_modal_refresh(void) {
    for (int i = 0; i < MELODY_PRESET_COUNT; i++) {
        if (!s_melody_preset_slot_lbls[i]) continue;
        const MelodyPresetSlot& s = s_melody_presets[i];
        if (s.used) {
            lv_label_set_text_fmt(s_melody_preset_slot_lbls[i], "S%d\n%s", i + 1, s.name[0] ? s.name : "PRESET");
            lv_obj_set_style_text_color(s_melody_preset_slot_lbls[i], lv_color_white(), 0);
        } else {
            lv_label_set_text_fmt(s_melody_preset_slot_lbls[i], "S%d\nVACIO", i + 1);
            lv_obj_set_style_text_color(s_melody_preset_slot_lbls[i], theme_text_dim(), 0);
        }
    }
}

static void melody_preset_save_current(int slot) {
    if (slot < 0 || slot >= MELODY_PRESET_COUNT) return;
    MelodyPresetSlot& s = s_melody_presets[slot];
    s.used = true;
    snprintf(s.name, sizeof(s.name), "MEL %d", slot + 1);
    memcpy(s.grid, s_piano_rec_grid, sizeof(s.grid));
    memcpy(s.notes, s_piano_rec_notes, sizeof(s.notes));
    s.engine = s_piano_rec_has_notes ? s_piano_rec_engine : PIANO_ENGINES[s_piano_engine_idx];
    s.octave = s_piano_rec_has_notes ? s_piano_rec_octave : (uint8_t)s_piano_octave;
    s.gatePercent = s_piano_gate_percent.load(std::memory_order_relaxed);
    melody_presets_save_to_disk();
    melody_preset_modal_refresh();
    ui_show_toast("Preset de melodia guardado", RED808_SUCCESS);
}

static void melody_preset_recall(int slot) {
    if (slot < 0 || slot >= MELODY_PRESET_COUNT) return;
    const MelodyPresetSlot& s = s_melody_presets[slot];
    if (!s.used) {
        ui_show_toast("Slot vacio — manten pulsado para guardar", RED808_WARNING);
        return;
    }
    memcpy(s_piano_rec_grid, s.grid, sizeof(s_piano_rec_grid));
    memcpy(s_piano_rec_notes, s.notes, sizeof(s_piano_rec_notes));
    s_piano_rec_engine = s.engine;
    s_piano_rec_octave = s.octave;
    s_piano_rec_has_notes = true;
    s_piano_gate_percent.store(s.gatePercent, std::memory_order_relaxed);
    if (s_piano_gate_lbl) lv_label_set_text_fmt(s_piano_gate_lbl, "GATE %u%%", (unsigned)s.gatePercent);
    for (int i = 0; i < PIANO_ENGINE_COUNT; i++) {
        if (PIANO_ENGINES[i] == s.engine) { s_piano_engine_idx = i; break; }
    }
    piano_refresh_engine_chips();
    piano_refresh_engine_presets();
    piano_grid_refresh_all();
    ui_show_toast("Preset de melodia cargado", RED808_SUCCESS);
}

static void melody_preset_slot_clicked_cb(lv_event_t* e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    if (lv_event_get_code(e) == LV_EVENT_LONG_PRESSED) melody_preset_save_current(slot);
    else melody_preset_recall(slot);
}

static void melody_preset_modal_close_cb(lv_event_t* e) {
    if (e && lv_event_get_target(e) != lv_event_get_current_target(e)) return;
    if (s_melody_preset_modal) {
        lv_obj_del(s_melody_preset_modal);
        s_melody_preset_modal = NULL;
        for (int i = 0; i < MELODY_PRESET_COUNT; i++) {
            s_melody_preset_slot_btns[i] = NULL;
            s_melody_preset_slot_lbls[i] = NULL;
        }
    }
}

static void melody_preset_modal_show(lv_event_t* e) {
    LV_UNUSED(e);
    if (s_melody_preset_modal) return;

    s_melody_preset_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_melody_preset_modal, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(s_melody_preset_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_melody_preset_modal, LV_OPA_80, 0);
    lv_obj_set_style_border_width(s_melody_preset_modal, 0, 0);
    lv_obj_set_style_pad_all(s_melody_preset_modal, 0, 0);
    lv_obj_clear_flag(s_melody_preset_modal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_melody_preset_modal, melody_preset_modal_close_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* card = lv_obj_create(s_melody_preset_modal);
    lv_obj_set_size(card, 720, 260);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_color(card, RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_dir(card, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_border_color(card, RED808_CYAN, 0);
    lv_obj_set_style_radius(card, 18, 0);
    lv_obj_set_style_pad_all(card, 14, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t* title = lv_label_create(card);
    lv_label_set_text(title, "MELODY PRESETS");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(title, RED808_CYAN, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 2);

    lv_obj_t* hint = lv_label_create(card);
    lv_label_set_text(hint, "TOCA = cargar en el editor   ·   MANTEN PULSADO = guardar la melodia actual aqui");
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(hint, RED808_TEXT_DIM, 0);
    lv_obj_align(hint, LV_ALIGN_TOP_MID, 0, 32);

    constexpr int btnW = 80, btnH = 84, gapX = 6, y0 = 68;
    for (int i = 0; i < MELODY_PRESET_COUNT; i++) {
        lv_obj_t* btn = lv_btn_create(card);
        s_melody_preset_slot_btns[i] = btn;
        lv_obj_set_size(btn, btnW, btnH);
        lv_obj_set_pos(btn, 4 + i * (btnW + gapX), y0);
        apply_control_button_style(btn, RED808_ACCENT2, false, 8);
        lv_obj_add_event_cb(btn, melody_preset_slot_clicked_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
        lv_obj_add_event_cb(btn, melody_preset_slot_clicked_cb, LV_EVENT_LONG_PRESSED, (void*)(intptr_t)i);

        lv_obj_t* lbl = lv_label_create(btn);
        s_melody_preset_slot_lbls[i] = lbl;
        lv_obj_set_width(lbl, btnW - 8);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_center(lbl);
    }

    lv_obj_t* close_btn = lv_btn_create(card);
    lv_obj_set_size(close_btn, 160, 40);
    lv_obj_set_pos(close_btn, 280, 172);
    apply_control_button_style(close_btn, RED808_ERROR, false, 10);
    lv_obj_add_event_cb(close_btn, melody_preset_modal_close_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* close_lbl = lv_label_create(close_btn);
    lv_label_set_text(close_lbl, "CERRAR");
    lv_obj_center(close_lbl);

    melody_preset_modal_refresh();
}

// =============================================================================
// MATRIX — compose an authored song from existing patterns plus filter/
// mixer/melody presets, one column per song step. Phase 4 of the SONG
// "MATRIX" plan (see FILTER/MIXER/MELODY PRESETS above for Phases 1-3).
// This grid only holds the authoring data (persisted to disk); actual
// playback state (which pattern is queued, current column, bar cadence)
// lives in control_api.cpp's MatrixStepEntry chain — PLAY compacts this
// grid's non-empty columns into that chain via control_matrix_upload(),
// and matrix_apply_column() (called from the bar-clock tick bridge near
// ui_update_current_screen()) recalls each column's presets as it becomes
// active, reusing the Phase 1-3 recall functions above unchanged.
// =============================================================================
#define MATRIX_UI_STEPS MATRIX_MAX_STEPS
struct MatrixUiColumn {
    int8_t pattern;       // -1 = empty (not part of the song)
    int8_t filterPreset;  // -1 = none, else 0..7
    int8_t mixerPreset;   // -1 = none, else 0..7
    int8_t melodyPreset;  // -1 = none, else 0..7
};
static MatrixUiColumn s_matrix_cols[MATRIX_UI_STEPS];
static bool s_matrix_loaded_from_disk = false;
static const char* MATRIX_SONG_FILE = "/matrix_song.txt";

static lv_obj_t* s_matrix_modal = NULL;
static lv_obj_t* s_matrix_pattern_lbls[MATRIX_UI_STEPS] = {};
static lv_obj_t* s_matrix_filter_lbls[MATRIX_UI_STEPS] = {};
static lv_obj_t* s_matrix_mixer_lbls[MATRIX_UI_STEPS] = {};
static lv_obj_t* s_matrix_melody_lbls[MATRIX_UI_STEPS] = {};
static lv_obj_t* s_matrix_status_lbl = NULL;
static lv_obj_t* s_matrix_play_btn = NULL;
static lv_obj_t* s_matrix_play_lbl = NULL;
static lv_obj_t* s_matrix_bars_btns[4] = {};

static void matrix_song_save_to_disk(void) {
    File f = SPIFFS.open(MATRIX_SONG_FILE, FILE_WRITE);
    if (!f) return;
    for (int c = 0; c < MATRIX_UI_STEPS; c++) {
        const MatrixUiColumn& s = s_matrix_cols[c];
        f.printf("%d,%d,%d,%d\n", s.pattern, s.filterPreset, s.mixerPreset, s.melodyPreset);
    }
    f.close();
}

static void matrix_song_load_from_disk(void) {
    for (int c = 0; c < MATRIX_UI_STEPS; c++)
        s_matrix_cols[c] = MatrixUiColumn{-1, -1, -1, -1};
    File f = SPIFFS.open(MATRIX_SONG_FILE, FILE_READ);
    if (!f) return;
    char line[32];
    for (int c = 0; c < MATRIX_UI_STEPS; c++) {
        if (!fs_read_line(f, line, sizeof(line))) break;
        int pat = -1, filt = -1, mix = -1, mel = -1;
        sscanf(line, "%d,%d,%d,%d", &pat, &filt, &mix, &mel);
        s_matrix_cols[c].pattern      = (int8_t)constrain(pat,  -1, Config::MAX_PATTERNS - 1);
        s_matrix_cols[c].filterPreset = (int8_t)constrain(filt, -1, FILTER_PRESET_COUNT - 1);
        s_matrix_cols[c].mixerPreset  = (int8_t)constrain(mix,  -1, MIXER_PRESET_COUNT - 1);
        s_matrix_cols[c].melodyPreset = (int8_t)constrain(mel,  -1, MELODY_PRESET_COUNT - 1);
    }
    f.close();
}

static void matrix_col_refresh(int c) {
    if (c < 0 || c >= MATRIX_UI_STEPS) return;
    const MatrixUiColumn& s = s_matrix_cols[c];
    char buf[72];
    if (s_matrix_pattern_lbls[c]) {
        if (s.pattern >= 0)
            snprintf(buf, sizeof(buf), "P%02d\n%s", s.pattern + 1, seq_pattern_name(s.pattern));
        else
            snprintf(buf, sizeof(buf), "--\nVACIA");
        lv_label_set_text(s_matrix_pattern_lbls[c], buf);
    }
    if (s_matrix_filter_lbls[c]) {
        if (s.filterPreset >= 0 && s.filterPreset < FILTER_PRESET_COUNT) {
            const FilterPresetSlot& p = s_filter_presets[s.filterPreset];
            snprintf(buf, sizeof(buf), "F%d\n%s", s.filterPreset + 1, p.used ? p.name : "VACIO");
        } else snprintf(buf, sizeof(buf), "--\nSIN FILTRO");
        lv_label_set_text(s_matrix_filter_lbls[c], buf);
    }
    if (s_matrix_mixer_lbls[c]) {
        if (s.mixerPreset >= 0 && s.mixerPreset < MIXER_PRESET_COUNT) {
            const MixerPresetSlot& p = s_mixer_presets[s.mixerPreset];
            snprintf(buf, sizeof(buf), "M%d\n%s", s.mixerPreset + 1, p.used ? p.name : "VACIO");
        } else snprintf(buf, sizeof(buf), "--\nSIN MIXER");
        lv_label_set_text(s_matrix_mixer_lbls[c], buf);
    }
    if (s_matrix_melody_lbls[c]) {
        if (s.melodyPreset >= 0 && s.melodyPreset < MELODY_PRESET_COUNT) {
            const MelodyPresetSlot& p = s_melody_presets[s.melodyPreset];
            snprintf(buf, sizeof(buf), "N%d\n%s", s.melodyPreset + 1, p.used ? p.name : "VACIO");
        } else snprintf(buf, sizeof(buf), "--\nSIN MELODIA");
        lv_label_set_text(s_matrix_melody_lbls[c], buf);
    }
}

// Forward declarations — the picker modal (defined below, after the preset
// systems it lists) opens on tap instead of the old blind tap-to-cycle,
// which is what caused both problems reported: a flicker from writing
// /matrix_song.txt to flash on every single tap while cycling through up
// to 128 patterns, and no visibility into what a preset slot actually is
// while picking. One picker, one save, one clean list of names.
enum MatrixPickKind { MATRIX_PICK_PATTERN, MATRIX_PICK_FILTER, MATRIX_PICK_MIXER, MATRIX_PICK_MELODY };
static void matrix_pattern_picker_show(int col);
static void matrix_preset_picker_show(int col, MatrixPickKind kind);

static void matrix_pattern_cell_cb(lv_event_t* e) {
    int c = (int)(intptr_t)lv_event_get_user_data(e);
    matrix_pattern_picker_show(c);
}

static void matrix_filter_cell_cb(lv_event_t* e) {
    int c = (int)(intptr_t)lv_event_get_user_data(e);
    matrix_preset_picker_show(c, MATRIX_PICK_FILTER);
}

static void matrix_mixer_cell_cb(lv_event_t* e) {
    int c = (int)(intptr_t)lv_event_get_user_data(e);
    matrix_preset_picker_show(c, MATRIX_PICK_MIXER);
}

static void matrix_melody_cell_cb(lv_event_t* e) {
    int c = (int)(intptr_t)lv_event_get_user_data(e);
    matrix_preset_picker_show(c, MATRIX_PICK_MELODY);
}

static void matrix_status_refresh(void) {
    bool active = control_matrix_active();
    if (s_matrix_status_lbl) {
        if (active)
            lv_label_set_text_fmt(s_matrix_status_lbl, "MATRIX: ON  col %d/%d",
                control_matrix_idx() + 1, control_matrix_count());
        else
            lv_label_set_text(s_matrix_status_lbl, "MATRIX: OFF");
    }
    if (s_matrix_play_lbl) lv_label_set_text(s_matrix_play_lbl, active ? "STOP" : "PLAY");
    if (s_matrix_play_btn)
        lv_obj_set_style_bg_color(s_matrix_play_btn, active ? RED808_ERROR : RED808_SUCCESS, 0);
}

static void matrix_bars_refresh(void) {
    static const uint8_t opts[4] = {1, 2, 4, 8};
    uint8_t bars = control_matrix_bars();
    for (int i = 0; i < 4; i++) {
        if (!s_matrix_bars_btns[i]) continue;
        bool sel = (opts[i] == bars);
        apply_control_button_style(s_matrix_bars_btns[i], sel ? RED808_ACCENT2 : RED808_BORDER, false, 8);
    }
}

static void matrix_bars_cb(lv_event_t* e) {
    uint8_t bars = (uint8_t)(intptr_t)lv_event_get_user_data(e);
    control_matrix_set_bars(bars);
    matrix_bars_refresh();
}

// =============================================================================
// MATRIX pickers — a scrollable list of named options instead of the old
// tap-to-cycle. Each pick writes /matrix_song.txt exactly once (on
// selection), not once per tap, which was the real cause of the flicker
// reported when cycling through patterns.
// =============================================================================
static lv_obj_t* s_matrix_picker_modal = NULL;
static int        s_matrix_picker_col  = -1;
static MatrixPickKind s_matrix_picker_kind = MATRIX_PICK_PATTERN;

static void matrix_picker_close_cb(lv_event_t* e) {
    if (e && lv_event_get_target(e) != lv_event_get_current_target(e)) return;
    if (s_matrix_picker_modal) {
        lv_obj_del(s_matrix_picker_modal);
        s_matrix_picker_modal = NULL;
    }
    s_matrix_picker_col = -1;
}

static void matrix_picker_pattern_pick_cb(lv_event_t* e) {
    int p = (int)(intptr_t)lv_event_get_user_data(e);
    if (s_matrix_picker_col >= 0 && s_matrix_picker_col < MATRIX_UI_STEPS) {
        s_matrix_cols[s_matrix_picker_col].pattern = (int8_t)p;
        matrix_col_refresh(s_matrix_picker_col);
        matrix_song_save_to_disk();
    }
    matrix_picker_close_cb(NULL);
}

static void matrix_pattern_picker_show(int col) {
    if (col < 0 || col >= MATRIX_UI_STEPS) return;
    matrix_picker_close_cb(NULL);
    s_matrix_picker_col = col;

    s_matrix_picker_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_matrix_picker_modal, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(s_matrix_picker_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_matrix_picker_modal, LV_OPA_70, 0);
    lv_obj_set_style_border_width(s_matrix_picker_modal, 0, 0);
    lv_obj_set_style_pad_all(s_matrix_picker_modal, 0, 0);
    lv_obj_clear_flag(s_matrix_picker_modal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_matrix_picker_modal, matrix_picker_close_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* card = lv_obj_create(s_matrix_picker_modal);
    lv_obj_set_size(card, 850, 530);
    lv_obj_center(card);
    lv_obj_set_style_radius(card, 16, 0);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_border_color(card, RED808_CYAN, 0);
    lv_obj_set_style_pad_all(card, 0, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t* title = lv_label_create(card);
    lv_label_set_text_fmt(title, "COLUMNA %d \xC2\xB7 ELIGE PATRON", col + 1);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title, RED808_CYAN, 0);
    lv_obj_set_pos(title, 16, 10);

    lv_obj_t* close = lv_btn_create(card);
    lv_obj_set_size(close, 42, 38);
    lv_obj_set_pos(close, 800, 6);
    apply_control_button_style(close, RED808_ERROR, false, 8);
    lv_obj_add_event_cb(close, matrix_picker_close_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* cl = lv_label_create(close);
    lv_label_set_text(cl, LV_SYMBOL_CLOSE);
    lv_obj_center(cl);

    lv_obj_t* list = lv_obj_create(card);
    lv_obj_set_size(list, 818, 466);
    lv_obj_set_pos(list, 16, 52);
    lv_obj_set_style_bg_color(list, RED808_SURFACE, 0);
    lv_obj_set_style_bg_opa(list, LV_OPA_60, 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_style_pad_all(list, 6, 0);
    lv_obj_add_flag(list, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_AUTO);

    // Row 0: clears the column (marks it empty / end of song).
    {
        lv_obj_t* btn = lv_btn_create(list);
        lv_obj_set_size(btn, 806, 42);
        lv_obj_set_pos(btn, 0, 0);
        bool sel = (s_matrix_cols[col].pattern < 0);
        apply_control_button_style(btn, sel ? RED808_WARNING : RED808_BORDER, sel, 7);
        lv_obj_add_event_cb(btn, matrix_picker_pattern_pick_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
        lv_obj_t* lbl = lv_label_create(btn);
        lv_label_set_text(lbl, "-- VACIA / FIN DE CANCION --");
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
        lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 6, 0);
    }
    for (int p = 0; p < Config::MAX_PATTERNS; ++p) {
        const int visibleIndex = p + 1;   // offset by the clear row above
        const int pcol = visibleIndex & 1;
        const int prow = visibleIndex >> 1;
        lv_obj_t* btn = lv_btn_create(list);
        lv_obj_set_size(btn, 394, 42);
        lv_obj_set_pos(btn, pcol * 402, prow * 46);
        const bool current = (p == s_matrix_cols[col].pattern);
        apply_control_button_style(btn, current ? RED808_ACCENT : RED808_BORDER, current, 7);
        lv_obj_add_event_cb(btn, matrix_picker_pattern_pick_cb, LV_EVENT_CLICKED, (void*)(intptr_t)p);
        lv_obj_t* label = lv_label_create(btn);
        lv_label_set_text_fmt(label, "P%03d  %s", p + 1, seq_pattern_name(p));
        lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(label, current ? lv_color_white() : RED808_TEXT, 0);
        lv_obj_align(label, LV_ALIGN_LEFT_MID, 6, 0);
    }
}

static void matrix_picker_preset_pick_cb(lv_event_t* e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    if (s_matrix_picker_col >= 0 && s_matrix_picker_col < MATRIX_UI_STEPS) {
        MatrixUiColumn& c = s_matrix_cols[s_matrix_picker_col];
        switch (s_matrix_picker_kind) {
            case MATRIX_PICK_FILTER: c.filterPreset = (int8_t)slot; break;
            case MATRIX_PICK_MIXER:  c.mixerPreset  = (int8_t)slot; break;
            case MATRIX_PICK_MELODY: c.melodyPreset = (int8_t)slot; break;
            default: break;
        }
        matrix_col_refresh(s_matrix_picker_col);
        matrix_song_save_to_disk();
    }
    matrix_picker_close_cb(NULL);
}

static void matrix_preset_picker_show(int col, MatrixPickKind kind) {
    if (col < 0 || col >= MATRIX_UI_STEPS) return;
    matrix_picker_close_cb(NULL);
    s_matrix_picker_col = col;
    s_matrix_picker_kind = kind;

    const char* titleTxt = "PRESET";
    lv_color_t accent = RED808_CYAN;
    int8_t current = -1;
    switch (kind) {
        case MATRIX_PICK_FILTER:
            titleTxt = "FILTRO"; accent = lv_color_hex(0xFFE066);
            current = s_matrix_cols[col].filterPreset; break;
        case MATRIX_PICK_MIXER:
            titleTxt = "MIXER"; accent = lv_color_hex(0x7CFF6B);
            current = s_matrix_cols[col].mixerPreset; break;
        case MATRIX_PICK_MELODY:
            titleTxt = "MELODIA"; accent = lv_color_hex(0xFF1493);
            current = s_matrix_cols[col].melodyPreset; break;
        default: break;
    }

    s_matrix_picker_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_matrix_picker_modal, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(s_matrix_picker_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_matrix_picker_modal, LV_OPA_70, 0);
    lv_obj_set_style_border_width(s_matrix_picker_modal, 0, 0);
    lv_obj_set_style_pad_all(s_matrix_picker_modal, 0, 0);
    lv_obj_clear_flag(s_matrix_picker_modal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_matrix_picker_modal, matrix_picker_close_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* card = lv_obj_create(s_matrix_picker_modal);
    lv_obj_set_size(card, 480, 500);
    lv_obj_center(card);
    lv_obj_set_style_radius(card, 16, 0);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_border_color(card, accent, 0);
    lv_obj_set_style_pad_all(card, 0, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t* title = lv_label_create(card);
    lv_label_set_text_fmt(title, "COLUMNA %d \xC2\xB7 %s", col + 1, titleTxt);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title, accent, 0);
    lv_obj_set_pos(title, 16, 10);

    lv_obj_t* close = lv_btn_create(card);
    lv_obj_set_size(close, 42, 38);
    lv_obj_set_pos(close, 422, 6);
    apply_control_button_style(close, RED808_ERROR, false, 8);
    lv_obj_add_event_cb(close, matrix_picker_close_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* cl = lv_label_create(close);
    lv_label_set_text(cl, LV_SYMBOL_CLOSE);
    lv_obj_center(cl);

    int y = 56;
    {
        lv_obj_t* btn = lv_btn_create(card);
        lv_obj_set_size(btn, 448, 40);
        lv_obj_set_pos(btn, 16, y);
        bool sel = (current < 0);
        apply_control_button_style(btn, sel ? RED808_WARNING : RED808_BORDER, sel, 8);
        lv_obj_add_event_cb(btn, matrix_picker_preset_pick_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
        lv_obj_t* lbl = lv_label_create(btn);
        lv_label_set_text(lbl, "-- NINGUNO --");
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
        lv_obj_center(lbl);
    }
    y += 46;
    for (int i = 0; i < 8; i++) {
        bool used = false;
        const char* name = "VACIO";
        switch (kind) {
            case MATRIX_PICK_FILTER:
                used = s_filter_presets[i].used;
                name = used ? s_filter_presets[i].name : "VACIO"; break;
            case MATRIX_PICK_MIXER:
                used = s_mixer_presets[i].used;
                name = used ? s_mixer_presets[i].name : "VACIO"; break;
            case MATRIX_PICK_MELODY:
                used = s_melody_presets[i].used;
                name = used ? s_melody_presets[i].name : "VACIO"; break;
            default: break;
        }
        lv_obj_t* btn = lv_btn_create(card);
        lv_obj_set_size(btn, 448, 40);
        lv_obj_set_pos(btn, 16, y);
        bool sel = (current == i);
        apply_control_button_style(btn, sel ? accent : RED808_BORDER, sel, 8);
        lv_obj_add_event_cb(btn, matrix_picker_preset_pick_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
        lv_obj_t* lbl = lv_label_create(btn);
        lv_label_set_text_fmt(lbl, "%d \xC2\xB7 %s", i + 1, name);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(lbl, sel ? RED808_BG : (used ? RED808_TEXT : RED808_TEXT_DIM), 0);
        lv_obj_center(lbl);
        y += 46;
    }
}

// Called from the SONG modal's own PLAY, and from the bar-clock tick bridge
// (see ui_request_matrix_tick / ui_update_current_screen) whenever MATRIX
// advances to a new column while running unattended — recalls that
// column's presets through the exact same functions their own modals use.
static void matrix_apply_column(uint8_t idx) {
    MatrixStepEntry e{};
    if (!control_matrix_get_entry(idx, &e)) return;
    if (e.filterPreset >= 0) filter_preset_recall(e.filterPreset);
    if (e.mixerPreset  >= 0) mixer_preset_recall(e.mixerPreset);
    if (e.melodyPreset >= 0) melody_preset_recall(e.melodyPreset);
    matrix_status_refresh();
}

static void matrix_play_btn_cb(lv_event_t* e) {
    LV_UNUSED(e);
    if (control_matrix_active()) {
        control_matrix_set_active(false);
        matrix_status_refresh();
        ui_show_toast("MATRIX detenido", RED808_WARNING);
        return;
    }
    MatrixStepEntry entries[MATRIX_MAX_STEPS];
    uint8_t count = 0;
    for (int c = 0; c < MATRIX_UI_STEPS && count < MATRIX_MAX_STEPS; c++) {
        if (s_matrix_cols[c].pattern < 0) continue;
        entries[count].pattern      = (uint8_t)s_matrix_cols[c].pattern;
        entries[count].filterPreset = s_matrix_cols[c].filterPreset;
        entries[count].mixerPreset  = s_matrix_cols[c].mixerPreset;
        entries[count].melodyPreset = s_matrix_cols[c].melodyPreset;
        count++;
    }
    if (count == 0) {
        ui_show_toast("Elige al menos un patron en la fila 1", RED808_WARNING);
        return;
    }
    if (!control_available()) {
        ui_show_toast("Master no conectado", RED808_WARNING);
        return;
    }
    control_matrix_upload(entries, count);
    // The bar-clock only handles columns AFTER the first — apply column 0
    // immediately so hitting PLAY has an instant effect.
    if (entries[0].filterPreset >= 0) filter_preset_recall(entries[0].filterPreset);
    if (entries[0].mixerPreset  >= 0) mixer_preset_recall(entries[0].mixerPreset);
    if (entries[0].melodyPreset >= 0) melody_preset_recall(entries[0].melodyPreset);
    control_send_select_pattern(entries[0].pattern);
    if (!p4.is_playing) control_send_start();
    control_matrix_set_active(true);
    matrix_status_refresh();
    ui_show_toast("MATRIX: reproduciendo", RED808_SUCCESS);
}

static lv_obj_t* s_matrix_grid_area = NULL;
static lv_obj_t* s_matrix_page_lbl  = NULL;
static int        s_matrix_view_page = 0;
static constexpr int MATRIX_VIS_COLS = 8;   // per page — bigger cells than showing all 16 at once

static void matrix_modal_close_cb(lv_event_t* e) {
    if (e && lv_event_get_target(e) != lv_event_get_current_target(e)) return;
    if (s_matrix_modal) {
        lv_obj_del(s_matrix_modal);
        s_matrix_modal = NULL;
        for (int i = 0; i < MATRIX_UI_STEPS; i++) {
            s_matrix_pattern_lbls[i] = NULL;
            s_matrix_filter_lbls[i] = NULL;
            s_matrix_mixer_lbls[i] = NULL;
            s_matrix_melody_lbls[i] = NULL;
        }
        s_matrix_status_lbl = NULL;
        s_matrix_play_btn = NULL;
        s_matrix_play_lbl = NULL;
        s_matrix_grid_area = NULL;
        s_matrix_page_lbl = NULL;
        s_matrix_view_page = 0;
        for (int i = 0; i < 4; i++) s_matrix_bars_btns[i] = NULL;
    }
}

// (Re)builds only the grid area's cells for the current page — called on
// open and on PREV/NEXT — instead of tearing down the whole screen, so
// paging feels instant and doesn't touch the header/footer controls.
static void matrix_build_grid_page(void) {
    if (!s_matrix_grid_area) return;
    lv_obj_clean(s_matrix_grid_area);
    for (int i = 0; i < MATRIX_UI_STEPS; i++) {
        s_matrix_pattern_lbls[i] = NULL;
        s_matrix_filter_lbls[i] = NULL;
        s_matrix_mixer_lbls[i] = NULL;
        s_matrix_melody_lbls[i] = NULL;
    }

    constexpr int colW = 118, colGap = 8;
    constexpr int rowH = 108, rowGap = 8;
    static const uint32_t rowColors[4] = {0x00E5FF, 0xFFE066, 0x7CFF6B, 0xFF1493};
    static const char* rowIcons[4]     = {LV_SYMBOL_AUDIO, LV_SYMBOL_TINT,
                                          LV_SYMBOL_VOLUME_MAX, LV_SYMBOL_KEYBOARD};

    const int pageCount = (MATRIX_UI_STEPS + MATRIX_VIS_COLS - 1) / MATRIX_VIS_COLS;
    if (s_matrix_view_page >= pageCount) s_matrix_view_page = pageCount - 1;
    if (s_matrix_view_page < 0) s_matrix_view_page = 0;
    const int start = s_matrix_view_page * MATRIX_VIS_COLS;
    const int visible = constrain(MATRIX_UI_STEPS - start, 0, MATRIX_VIS_COLS);

    for (int vc = 0; vc < visible; vc++) {
        int c = start + vc;
        int x = vc * (colW + colGap);

        lv_obj_t* idxLbl = lv_label_create(s_matrix_grid_area);
        lv_label_set_text_fmt(idxLbl, "COL %d", c + 1);
        lv_obj_set_style_text_font(idxLbl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(idxLbl, RED808_TEXT_DIM, 0);
        lv_obj_set_pos(idxLbl, x + 4, 0);

        for (int r = 0; r < 4; r++) {
            lv_obj_t* btn = lv_btn_create(s_matrix_grid_area);
            lv_obj_set_size(btn, colW, rowH);
            lv_obj_set_pos(btn, x, 18 + r * (rowH + rowGap));
            apply_control_button_style(btn, lv_color_hex(rowColors[r]), false, 8);
            lv_event_cb_t cb = r == 0 ? matrix_pattern_cell_cb
                              : r == 1 ? matrix_filter_cell_cb
                              : r == 2 ? matrix_mixer_cell_cb
                                       : matrix_melody_cell_cb;
            lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, (void*)(intptr_t)c);

            lv_obj_t* icon = lv_label_create(btn);
            lv_label_set_text(icon, rowIcons[r]);
            lv_obj_set_style_text_font(icon, &lv_font_montserrat_16, 0);
            lv_obj_set_style_text_color(icon, lv_color_hex(rowColors[r]), 0);
            lv_obj_align(icon, LV_ALIGN_TOP_LEFT, 6, 6);

            lv_obj_t* lbl = lv_label_create(btn);
            lv_obj_set_width(lbl, colW - 14);
            lv_label_set_long_mode(lbl, LV_LABEL_LONG_WRAP);
            lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
            lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_align(lbl, LV_ALIGN_CENTER, 0, 8);
            switch (r) {
                case 0: s_matrix_pattern_lbls[c] = lbl; break;
                case 1: s_matrix_filter_lbls[c]  = lbl; break;
                case 2: s_matrix_mixer_lbls[c]   = lbl; break;
                default: s_matrix_melody_lbls[c] = lbl; break;
            }
        }
        matrix_col_refresh(c);
    }

    if (s_matrix_page_lbl) {
        int lastVisible = start + (visible > 0 ? visible - 1 : 0) + 1;
        lv_label_set_text_fmt(s_matrix_page_lbl, "COL %d-%d / %d",
            start + 1, lastVisible, MATRIX_UI_STEPS);
    }
}

static void matrix_page_cb(lv_event_t* e) {
    int dir = (int)(intptr_t)lv_event_get_user_data(e);
    int pageCount = (MATRIX_UI_STEPS + MATRIX_VIS_COLS - 1) / MATRIX_VIS_COLS;
    s_matrix_view_page = (s_matrix_view_page + dir + pageCount) % pageCount;
    matrix_build_grid_page();
}

static void matrix_modal_show(lv_event_t* e) {
    LV_UNUSED(e);
    if (s_matrix_modal) return;
    if (!s_matrix_loaded_from_disk) {
        matrix_song_load_from_disk();
        s_matrix_loaded_from_disk = true;
    }
    s_matrix_view_page = 0;

    s_matrix_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_matrix_modal, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(s_matrix_modal, RED808_BG, 0);
    lv_obj_set_style_bg_opa(s_matrix_modal, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_matrix_modal, 0, 0);
    lv_obj_set_style_pad_all(s_matrix_modal, 0, 0);
    lv_obj_clear_flag(s_matrix_modal, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* title = lv_label_create(s_matrix_modal);
    lv_label_set_text(title, "MATRIX");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(title, RED808_CYAN, 0);
    lv_obj_set_pos(title, 12, 6);

    // Page nav — top right, since only MATRIX_VIS_COLS of MATRIX_UI_STEPS
    // columns fit on screen at this cell size.
    lv_obj_t* prev = lv_btn_create(s_matrix_modal);
    lv_obj_set_size(prev, 42, 36);
    lv_obj_set_pos(prev, 700, 6);
    apply_control_button_style(prev, RED808_CYAN, false, 8);
    lv_obj_add_event_cb(prev, matrix_page_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
    lv_obj_t* prevLbl = lv_label_create(prev);
    lv_label_set_text(prevLbl, LV_SYMBOL_LEFT);
    lv_obj_center(prevLbl);

    s_matrix_page_lbl = lv_label_create(s_matrix_modal);
    lv_label_set_text(s_matrix_page_lbl, "COL 1-8 / 16");
    lv_obj_set_style_text_font(s_matrix_page_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_matrix_page_lbl, RED808_TEXT, 0);
    lv_obj_set_pos(s_matrix_page_lbl, 748, 16);

    lv_obj_t* next = lv_btn_create(s_matrix_modal);
    lv_obj_set_size(next, 42, 36);
    lv_obj_set_pos(next, 858, 6);
    apply_control_button_style(next, RED808_CYAN, false, 8);
    lv_obj_add_event_cb(next, matrix_page_cb, LV_EVENT_CLICKED, (void*)(intptr_t)1);
    lv_obj_t* nextLbl = lv_label_create(next);
    lv_label_set_text(nextLbl, LV_SYMBOL_RIGHT);
    lv_obj_center(nextLbl);

    lv_obj_t* closeBtn = lv_btn_create(s_matrix_modal);
    lv_obj_set_size(closeBtn, 42, 36);
    lv_obj_set_pos(closeBtn, 970, 6);
    apply_control_button_style(closeBtn, RED808_ERROR, false, 8);
    lv_obj_add_event_cb(closeBtn, matrix_modal_close_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* closeLbl = lv_label_create(closeBtn);
    lv_label_set_text(closeLbl, LV_SYMBOL_CLOSE);
    lv_obj_center(closeLbl);

    lv_obj_t* hint = lv_label_create(s_matrix_modal);
    lv_label_set_text(hint, "Toca una celda para elegir de una lista: patron, filtro, mixer o melodia.");
    lv_obj_set_width(hint, 680);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(hint, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(hint, 12, 34);

    s_matrix_grid_area = lv_obj_create(s_matrix_modal);
    lv_obj_set_size(s_matrix_grid_area, 1000, 480);
    lv_obj_set_pos(s_matrix_grid_area, 12, 52);
    lv_obj_set_style_bg_opa(s_matrix_grid_area, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_matrix_grid_area, 0, 0);
    lv_obj_set_style_pad_all(s_matrix_grid_area, 0, 0);
    lv_obj_clear_flag(s_matrix_grid_area, LV_OBJ_FLAG_SCROLLABLE);
    matrix_build_grid_page();

    lv_obj_t* barsLbl = lv_label_create(s_matrix_modal);
    lv_label_set_text(barsLbl, "COMPASES:");
    lv_obj_set_style_text_font(barsLbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(barsLbl, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(barsLbl, 12, 546);
    {
        static const uint8_t barOptions[4] = {1, 2, 4, 8};
        for (int i = 0; i < 4; i++) {
            lv_obj_t* chip = lv_btn_create(s_matrix_modal);
            s_matrix_bars_btns[i] = chip;
            lv_obj_set_size(chip, 48, 32);
            lv_obj_set_pos(chip, 100 + i * 54, 540);
            lv_obj_add_event_cb(chip, matrix_bars_cb, LV_EVENT_CLICKED, (void*)(intptr_t)barOptions[i]);
            lv_obj_t* lbl = lv_label_create(chip);
            lv_label_set_text_fmt(lbl, "%d", barOptions[i]);
            lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
            lv_obj_center(lbl);
        }
    }
    matrix_bars_refresh();

    s_matrix_status_lbl = lv_label_create(s_matrix_modal);
    lv_label_set_text(s_matrix_status_lbl, "MATRIX: OFF");
    lv_obj_set_style_text_font(s_matrix_status_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_matrix_status_lbl, RED808_TEXT, 0);
    lv_obj_set_pos(s_matrix_status_lbl, 340, 548);

    s_matrix_play_btn = lv_btn_create(s_matrix_modal);
    lv_obj_set_size(s_matrix_play_btn, 120, 40);
    lv_obj_set_pos(s_matrix_play_btn, LV_HOR_RES - 260, 540);
    apply_control_button_style(s_matrix_play_btn, RED808_SUCCESS, false, 8);
    lv_obj_add_event_cb(s_matrix_play_btn, matrix_play_btn_cb, LV_EVENT_CLICKED, NULL);
    s_matrix_play_lbl = lv_label_create(s_matrix_play_btn);
    lv_label_set_text(s_matrix_play_lbl, "PLAY");
    lv_obj_set_style_text_font(s_matrix_play_lbl, &lv_font_montserrat_16, 0);
    lv_obj_center(s_matrix_play_lbl);

    lv_obj_t* closeBtn2 = lv_btn_create(s_matrix_modal);
    lv_obj_set_size(closeBtn2, 120, 40);
    lv_obj_set_pos(closeBtn2, LV_HOR_RES - 132, 540);
    apply_control_button_style(closeBtn2, RED808_BORDER, false, 8);
    lv_obj_add_event_cb(closeBtn2, matrix_modal_close_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* closeLbl2 = lv_label_create(closeBtn2);
    lv_label_set_text(closeLbl2, "CERRAR");
    lv_obj_set_style_text_font(closeLbl2, &lv_font_montserrat_14, 0);
    lv_obj_center(closeLbl2);

    matrix_status_refresh();
}

static void create_piano_screen(void) {
    int W = ui_layout_w();
    int H = ui_layout_h();
    scr_piano = lv_obj_create(NULL);
    apply_screen_theme_bg(scr_piano);
    lv_obj_clear_flag(scr_piano, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* top_deck = lv_obj_create(scr_piano);
    lv_obj_set_pos(top_deck, 4, 4);
    lv_obj_set_size(top_deck, W - 8, 140);
    lv_obj_set_style_radius(top_deck, 14, 0);
    lv_obj_set_style_bg_color(top_deck, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_color(top_deck, RED808_BG, 0);
    lv_obj_set_style_bg_grad_dir(top_deck, LV_GRAD_DIR_HOR, 0);
    lv_obj_set_style_bg_opa(top_deck, LV_OPA_80, 0);
    lv_obj_set_style_border_color(top_deck, RED808_BORDER, 0);
    lv_obj_set_style_border_width(top_deck, 1, 0);
    lv_obj_clear_flag(top_deck, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(top_deck, LV_OBJ_FLAG_CLICKABLE);

    /* Floating back button (top-left, lands back on LIVE) */
    ui_create_header(scr_piano);

    /* Title */
    lv_obj_t* title = lv_label_create(scr_piano);
    lv_label_set_text(title, "PIANO");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(title, RED808_TEXT, 0);
    lv_obj_set_pos(title, 64, 12);

    /* Engine selector buttons (top-right) — colored per engine */
    int eng_w = 80, eng_h = 36, eng_gap = 6;
    int eng_x_start = W - (eng_w + eng_gap) * PIANO_ENGINE_COUNT - 12;
    for (int i = 0; i < PIANO_ENGINE_COUNT; i++) {
        lv_obj_t* btn = piano_make_chip(scr_piano,
            eng_x_start + i * (eng_w + eng_gap), 8,
            eng_w, eng_h, PIANO_ENGINE_LABELS[i]);
        lv_obj_add_event_cb(btn, piano_engine_btn_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
        s_piano_engine_btns[i] = btn;
    }
    piano_refresh_engine_chips();

    /* Controls row: octave -/+ + 12/24 toggle + status */
    int row_y = 56;
    lv_obj_t* oct_minus = piano_make_chip(scr_piano, 12, row_y, 70, 36, "-");
    lv_obj_add_event_cb(oct_minus, piano_octave_btn_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
    s_piano_octave_lbl = lv_label_create(scr_piano);
    lv_label_set_text_fmt(s_piano_octave_lbl, "OCT %d", s_piano_octave);
    lv_obj_set_style_text_font(s_piano_octave_lbl, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(s_piano_octave_lbl, RED808_TEXT, 0);
    lv_obj_set_pos(s_piano_octave_lbl, 92, row_y + 8);
    lv_obj_t* oct_plus = piano_make_chip(scr_piano, 168, row_y, 70, 36, "+");
    lv_obj_add_event_cb(oct_plus, piano_octave_btn_cb, LV_EVENT_CLICKED, (void*)(intptr_t)+1);

    s_piano_keys24_btn = piano_make_chip(scr_piano, 248, row_y, 80, 36,
                                          s_piano_two_oct ? "24 K" : "12 K");
    s_piano_keys24_lbl = lv_obj_get_child(s_piano_keys24_btn, 0);
    lv_obj_add_event_cb(s_piano_keys24_btn, piano_keys24_btn_cb, LV_EVENT_CLICKED, NULL);

    /* PARAMS button → synth parameter editor screen (id 11) */
    {
        lv_obj_t* pbtn = piano_make_chip(scr_piano, 348, row_y, 100, 36, "PARAMS");
        lv_obj_set_style_border_color(pbtn, RED808_CYAN, 0);
        lv_obj_t* plbl = lv_obj_get_child(pbtn, 0);
        if (plbl) lv_obj_set_style_text_color(plbl, RED808_CYAN, 0);
        lv_obj_add_event_cb(pbtn, [](lv_event_t* e){ (void)e; ui_navigate_to(11); },
                             LV_EVENT_CLICKED, NULL);
    }

    /* v2.7 — REC toggle: sends melodyRecNote to master while active */
    s_piano_rec_btn = piano_make_chip(scr_piano, 580, row_y, 116, 44, "○ REC");
    s_piano_rec_lbl = lv_obj_get_child(s_piano_rec_btn, 0);
    lv_obj_add_event_cb(s_piano_rec_btn, piano_rec_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(s_piano_rec_btn, LV_OBJ_FLAG_PRESS_LOCK);
    if (s_piano_rec_active) {
        lv_obj_set_style_border_color(s_piano_rec_btn, lv_color_hex(0xFF3030), 0);
        lv_obj_set_style_border_width(s_piano_rec_btn, 3, 0);
        if (s_piano_rec_lbl) {
            lv_obj_set_style_text_color(s_piano_rec_lbl, lv_color_hex(0xFF3030), 0);
            lv_label_set_text(s_piano_rec_lbl, "● REC");
        }
    }

    s_piano_status_lbl = lv_label_create(scr_piano);
    lv_label_set_text_fmt(s_piano_status_lbl, "%s · OCT %d · PAD %d",
                          PIANO_ENGINE_LABELS[s_piano_engine_idx],
                          s_piano_octave, s_piano_assign_pad + 1);
    lv_obj_set_style_text_font(s_piano_status_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_piano_status_lbl, RED808_ACCENT, 0);
    lv_obj_set_pos(s_piano_status_lbl, 66, 39);

    lv_obj_t* expr_track = lv_obj_create(scr_piano);
    lv_obj_set_size(expr_track, 14, H - 168);
    lv_obj_set_pos(expr_track, W - 24, 150);
    lv_obj_clear_flag(expr_track, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(expr_track, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_radius(expr_track, 7, 0);
    lv_obj_set_style_bg_color(expr_track, RED808_SURFACE, 0);
    lv_obj_set_style_border_color(expr_track, RED808_BORDER, 0);
    lv_obj_set_style_border_width(expr_track, 1, 0);
    lv_obj_set_style_pad_all(expr_track, 0, 0);

    s_piano_expr_bar = lv_obj_create(expr_track);
    lv_obj_set_size(s_piano_expr_bar, 10, 8);
    lv_obj_clear_flag(s_piano_expr_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(s_piano_expr_bar, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_radius(s_piano_expr_bar, 5, 0);
    lv_obj_set_style_bg_color(s_piano_expr_bar, RED808_CYAN, 0);
    lv_obj_set_style_bg_grad_color(s_piano_expr_bar, RED808_SUCCESS, 0);
    lv_obj_set_style_bg_grad_dir(s_piano_expr_bar, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_width(s_piano_expr_bar, 0, 0);
    piano_update_expression_bar();

    s_piano_gate_btn = piano_make_chip(scr_piano, 458, row_y, 110, 36, "GATE 55%");
    s_piano_gate_lbl = lv_obj_get_child(s_piano_gate_btn, 0);
    if (s_piano_gate_lbl) lv_label_set_text_fmt(s_piano_gate_lbl, "GATE %u%%",
        (unsigned)s_piano_gate_percent.load(std::memory_order_relaxed));
    lv_obj_set_style_border_color(s_piano_gate_btn, RED808_SUCCESS, 0);
    lv_obj_add_event_cb(s_piano_gate_btn, piano_gate_btn_cb, LV_EVENT_CLICKED, NULL);

    // MELODY PRESETS — sits in the gap between GATE (ends 568) and GLIDE
    // (starts 730) in this same compact row; no free room elsewhere on screen.
    {
        lv_obj_t* mpb = piano_make_chip(scr_piano, 580, row_y, 140, 36, "M.PRESETS");
        lv_obj_set_style_border_color(mpb, RED808_CYAN, 0);
        lv_obj_t* l = lv_obj_get_child(mpb, 0);
        if (l) lv_obj_set_style_text_color(l, RED808_CYAN, 0);
        lv_obj_add_event_cb(mpb, melody_preset_modal_show, LV_EVENT_CLICKED, NULL);
    }
    melody_presets_load_from_disk();

    // v3.0 — visual controls for gesture piano (glide/bend/range)
    s_piano_glide_btn = piano_make_chip(scr_piano, 730, row_y, 98, 36,
                                        s_piano_glide_enabled ? "GLIDE ON" : "GLIDE OFF");
    s_piano_glide_lbl = lv_obj_get_child(s_piano_glide_btn, 0);
    lv_obj_add_event_cb(s_piano_glide_btn, piano_glide_btn_cb, LV_EVENT_CLICKED, NULL);

    s_piano_bend_btn = piano_make_chip(scr_piano, 836, row_y, 92, 36,
                                       s_piano_bend_enabled ? "BEND ON" : "BEND OFF");
    s_piano_bend_lbl = lv_obj_get_child(s_piano_bend_btn, 0);
    lv_obj_add_event_cb(s_piano_bend_btn, piano_bend_btn_cb, LV_EVENT_CLICKED, NULL);
    piano_refresh_gesture_controls();

    /* v2.8/v2.9 — compact melody row: pad assign + presets + transport */
    int row_y2 = 104;
    int x_cursor = 12;
    lv_obj_t* pad_minus = piano_make_chip(scr_piano, x_cursor, row_y2, 54, 42, "PAD-");
    lv_obj_set_style_border_color(pad_minus, RED808_ACCENT2, 0);
    lv_obj_add_event_cb(pad_minus, piano_pad_btn_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
    x_cursor += 54 + 4;
    s_piano_pad_lbl = lv_label_create(scr_piano);
    lv_label_set_text_fmt(s_piano_pad_lbl, "PAD %d", s_piano_assign_pad + 1);
    lv_obj_set_style_text_font(s_piano_pad_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_piano_pad_lbl, RED808_ACCENT2, 0);
    lv_obj_set_pos(s_piano_pad_lbl, x_cursor, row_y2 + 11);
    x_cursor += 70 + 4;
    lv_obj_t* pad_plus = piano_make_chip(scr_piano, x_cursor, row_y2, 54, 42, "PAD+");
    lv_obj_set_style_border_color(pad_plus, RED808_ACCENT2, 0);
    lv_obj_add_event_cb(pad_plus, piano_pad_btn_cb, LV_EVENT_CLICKED, (void*)(intptr_t)+1);
    x_cursor += 54 + 6;
    {
        lv_obj_t* assign = piano_make_chip(scr_piano, x_cursor, row_y2, 90, 42, "ASSIGN");
        lv_obj_set_style_border_color(assign, RED808_ACCENT2, 0);
        lv_obj_t* l = lv_obj_get_child(assign, 0);
        if (l) lv_obj_set_style_text_color(l, RED808_ACCENT2, 0);
        lv_obj_add_event_cb(assign, piano_assign_btn_cb, LV_EVENT_CLICKED, NULL);
    }
    x_cursor += 90 + 10;

    /* Melody-pattern presets, clear and play. */
    {
        struct PresetCfg { const char* lbl; int idx; uint32_t color; };
        const PresetCfg presets[3] = {
            { "BASS",  0, 0xFFE066 },
            { "ARP",   1, 0x7CFF6B },
            { "SCALE", 2, 0x00E5FF },
        };
        for (int i = 0; i < 3; i++) {
            lv_obj_t* b = piano_make_chip(scr_piano, x_cursor, row_y2, 64, 42, presets[i].lbl);
            lv_obj_set_style_border_color(b, lv_color_hex(presets[i].color), 0);
            lv_obj_t* l = lv_obj_get_child(b, 0);
            if (l) lv_obj_set_style_text_color(l, lv_color_hex(presets[i].color), 0);
            lv_obj_add_event_cb(b, piano_preset_btn_cb, LV_EVENT_CLICKED, (void*)(intptr_t)presets[i].idx);
            x_cursor += 64 + 4;
        }
        lv_obj_t* clr = piano_make_chip(scr_piano, x_cursor, row_y2, 64, 42, "CLEAR");
        lv_obj_set_style_border_color(clr, RED808_ERROR, 0);
        lv_obj_add_event_cb(clr, piano_clear_btn_cb, LV_EVENT_CLICKED, NULL);
        x_cursor += 64 + 8;

        s_piano_play_btn = piano_make_chip(scr_piano, x_cursor, row_y2, 84, 42, "PLAY");
        s_piano_play_lbl = lv_obj_get_child(s_piano_play_btn, 0);
        lv_obj_set_style_border_color(s_piano_play_btn, RED808_SUCCESS, 0);
        if (s_piano_play_lbl) lv_obj_set_style_text_color(s_piano_play_lbl, RED808_SUCCESS, 0);
        lv_obj_add_event_cb(s_piano_play_btn, piano_play_btn_cb, LV_EVENT_CLICKED, NULL);
        x_cursor += 84 + 12;
    }

    /* Per-engine sound presets (TONE) — relabel with the selected engine.
     * Up to 4 chips; piano_refresh_engine_presets() fills/hides them. */
    {
        lv_obj_t* tone_lbl = lv_label_create(scr_piano);
        lv_label_set_text(tone_lbl, "TONE");
        lv_obj_set_style_text_font(tone_lbl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(tone_lbl, RED808_TEXT_DIM, 0);
        lv_obj_set_pos(tone_lbl, x_cursor, row_y2 + 15);
        x_cursor += 42;
        for (int i = 0; i < 4; i++) {
            lv_obj_t* b = piano_make_chip(scr_piano, x_cursor, row_y2, 70, 42, "--");
            s_piano_eng_preset_btns[i] = b;
            lv_obj_add_event_cb(b, piano_eng_preset_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
            x_cursor += 70 + 4;
        }
    }

    /* Melody grid container: 16 cols × 12 rows pitch grid */
    int grid_y = 148;
    int grid_h = 184;
    s_piano_grid_container = lv_obj_create(scr_piano);
    lv_obj_set_pos(s_piano_grid_container, 0, grid_y);
    lv_obj_set_size(s_piano_grid_container, W, grid_h);
    lv_obj_set_style_bg_color(s_piano_grid_container, RED808_PANEL, 0);
    lv_obj_set_style_bg_opa(s_piano_grid_container, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_piano_grid_container, 0, 0);
    lv_obj_set_style_pad_all(s_piano_grid_container, 0, 0);
    lv_obj_clear_flag(s_piano_grid_container, LV_OBJ_FLAG_SCROLLABLE);
    {
        int gx0 = 8;
        int gy0 = 4;
        int gw  = W - 16;
        int gh  = grid_h - 8;
        int cw  = gw / 16;
        int ch  = gh / 12;
        for (int c = 0; c < 16; c++) {
            for (int r = 0; r < 12; r++) {
                lv_obj_t* cell = lv_btn_create(s_piano_grid_container);
                lv_obj_set_pos(cell, gx0 + c * cw + 1, gy0 + r * ch + 1);
                lv_obj_set_size(cell, cw - 2, ch - 2);
                lv_obj_set_style_radius(cell, 3, 0);
                lv_obj_set_style_border_width(cell, 1, 0);
                lv_obj_set_style_border_color(cell, RED808_BORDER, 0);
                lv_obj_set_style_shadow_width(cell, 0, 0);
                int packed = (c << 8) | r;
                lv_obj_add_event_cb(cell, piano_grid_cell_cb, LV_EVENT_CLICKED, (void*)(intptr_t)packed);
                s_piano_grid_btns[c][r] = cell;
            }
        }
        piano_grid_refresh_all();
    }

    /* Keys area (bottom half) */
    int keys_y = grid_y + grid_h + 8;
    int keys_h = H - keys_y - 8;
    s_piano_keys_container = lv_obj_create(scr_piano);
    lv_obj_set_pos(s_piano_keys_container, 0, keys_y);
    lv_obj_set_size(s_piano_keys_container, W, keys_h);
    lv_obj_set_style_bg_color(s_piano_keys_container, RED808_PANEL, 0);
    lv_obj_set_style_bg_opa(s_piano_keys_container, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_piano_keys_container, 0, 0);
    lv_obj_set_style_pad_all(s_piano_keys_container, 0, 0);
    lv_obj_clear_flag(s_piano_keys_container, LV_OBJ_FLAG_SCROLLABLE);

    piano_rebuild_keys();
    // Fill the per-engine TONE preset chips now that they exist.
    piano_refresh_engine_presets();
}

// =============================================================================
// PIANO PARAMS SCREEN — synth engine parameter editor (303/WT/SH101/FM2)
// v2.7 — mirrors web synth-editor with engine tabs + presets + sliders.
// =============================================================================
#include "../../../shared/synth_params.h"

#define PP_GRID_COLS_P4   3
#define PP_GRID_ROWS_P4   7
#define PP_MAX_PARAMS_P4  21

static int       s_pp_engine_idx = 0;
static int       s_pp_preset_idx[SP_ENGINE_COUNT] = { -1 };
static float     s_pp_values[SP_ENGINE_COUNT][PP_MAX_PARAMS_P4] = {};
static lv_obj_t* s_pp_engine_btns[SP_ENGINE_COUNT] = {};
static lv_obj_t* s_pp_preset_btns[4] = {};
static lv_obj_t* s_pp_param_panel    = NULL;
static lv_obj_t* s_pp_sliders[PP_MAX_PARAMS_P4] = {};
static lv_obj_t* s_pp_val_lbls[PP_MAX_PARAMS_P4] = {};
static lv_obj_t* s_pp_title_lbl = NULL;
static lv_obj_t* s_pp_wave_card = NULL;
static lv_obj_t* s_pp_wave_line = NULL;
static lv_obj_t* s_pp_wave_lbl = NULL;
static lv_point_t s_pp_wave_points[96] = {};
static bool      s_pp_from_xtra = false;
static int       s_pp_xtra_slot = -1;

static void xtra_capture_editor_state(int slot) {
    if (slot < 0 || slot >= 4) return;
    int eng_idx = xtra_slot_pp_engine_idx(slot);
    if (eng_idx < 0 || eng_idx >= SP_ENGINE_COUNT) return;
    const SynthEngineDef* eng = &SP_ENGINES[eng_idx];
    for (uint8_t i = 0; i < eng->param_count && i < XTRA_PARAM_MAX; i++) {
        s_xtra_param_values[slot][i] = s_pp_values[eng_idx][i];
    }
    s_xtra_param_valid[slot] = true;
    xtra_save_param_state();
}

static void xtra_load_editor_state(int slot) {
    if (slot < 0 || slot >= 4) return;
    int eng_idx = xtra_slot_pp_engine_idx(slot);
    if (eng_idx < 0 || eng_idx >= SP_ENGINE_COUNT) return;
    if (!s_xtra_param_valid[slot]) xtra_reset_slot_params(slot);
    const SynthEngineDef* eng = &SP_ENGINES[eng_idx];
    for (uint8_t i = 0; i < eng->param_count && i < XTRA_PARAM_MAX; i++) {
        s_pp_values[eng_idx][i] = s_xtra_param_values[slot][i];
    }
}

static float pp_param_value_or_default(int eng_idx, uint8_t param_id, float fallback) {
    if (eng_idx < 0 || eng_idx >= SP_ENGINE_COUNT) return fallback;
    const SynthEngineDef* eng = &SP_ENGINES[eng_idx];
    for (uint8_t i = 0; i < eng->param_count && i < PP_MAX_PARAMS_P4; i++) {
        if (eng->params[i].param_id == param_id) return s_pp_values[eng_idx][i];
    }
    return fallback;
}

static void pp_refresh_wave_preview(void) {
    if (!s_pp_wave_line || !s_pp_wave_lbl) return;
    const int count = (int)(sizeof(s_pp_wave_points) / sizeof(s_pp_wave_points[0]));
    const int width = 300;
    const int height = 68;
    const int mid = height / 2;
    const uint8_t engine = SP_ENGINES[s_pp_engine_idx].engine;
    const float cutoffNorm = constrain(pp_param_value_or_default(s_pp_engine_idx, 4, 8000.0f) / 18000.0f, 0.05f, 1.0f);
    const float modA = constrain(pp_param_value_or_default(s_pp_engine_idx, 1, 0.5f), 0.0f, 1.0f);
    const float modB = constrain(pp_param_value_or_default(s_pp_engine_idx, 9, 0.25f), 0.0f, 12.0f);
    const float shape = constrain(pp_param_value_or_default(s_pp_engine_idx, 0, 0.0f), 0.0f, 8.0f);

    for (int i = 0; i < count; i++) {
        float t = (float)i / (float)(count - 1);
        float y = 0.0f;
        switch (engine) {
            case SP_ENGINE_303: {
                float phase = t + shape * 0.03f;
                float saw = 2.0f * (phase - floorf(phase + 0.5f));
                float sq = (sinf(2.0f * PI * phase) >= 0.0f) ? 1.0f : -1.0f;
                float mix = constrain(pp_param_value_or_default(s_pp_engine_idx, 6, 0.0f), 0.0f, 1.0f);
                y = saw * (1.0f - mix) + sq * mix * (0.65f + modA * 0.35f);
                break;
            }
            case SP_ENGINE_WT:
                y = 0.58f * sinf(2.0f * PI * t) + 0.24f * sinf(4.0f * PI * t + shape * 0.4f) + 0.18f * sinf(6.0f * PI * t + shape * 0.85f);
                break;
            case SP_ENGINE_SH101: {
                float pwm = 0.1f + modA * 0.8f;
                float phase = t + shape * 0.02f;
                y = (fmodf(phase, 1.0f) < pwm) ? 1.0f : -1.0f;
                y *= 0.75f + cutoffNorm * 0.25f;
                break;
            }
            case SP_ENGINE_FM2OP: {
                float ratio = pp_param_value_or_default(s_pp_engine_idx, 8, 2.0f);
                float idx = constrain(modB / 12.0f, 0.0f, 1.0f) * 8.0f;
                y = sinf(2.0f * PI * t + idx * sinf(2.0f * PI * t * ratio));
                break;
            }
            case SP_ENGINE_PHYS: {
                float bright = constrain(pp_param_value_or_default(s_pp_engine_idx, 7, 0.64f), 0.0f, 1.0f);
                float damp = constrain(pp_param_value_or_default(s_pp_engine_idx, 3, 0.78f), 0.0f, 1.0f);
                float env = expf(-t * (1.0f + damp * 5.0f));
                y = env * (sinf(2.0f * PI * t * (1.2f + bright * 2.8f)) + 0.35f * sinf(2.0f * PI * t * (4.0f + bright * 6.0f)));
                break;
            }
            default:
                y = sinf(2.0f * PI * t);
                break;
        }
        y *= 0.85f;
        s_pp_wave_points[i].x = (lv_coord_t)((i * width) / (count - 1));
        s_pp_wave_points[i].y = (lv_coord_t)(mid - y * (mid - 8));
    }

    lv_line_set_points(s_pp_wave_line, s_pp_wave_points, count);
    if (s_pp_from_xtra && s_pp_xtra_slot >= 0 && s_pp_xtra_slot < 4) {
        lv_label_set_text_fmt(s_pp_wave_lbl, "Preview XTRA · slot %d · engine %s", s_pp_xtra_slot + 1, SP_ENGINES[s_pp_engine_idx].label);
    } else {
        lv_label_set_text_fmt(s_pp_wave_lbl, "Preview synth · engine %s", SP_ENGINES[s_pp_engine_idx].label);
    }
}

static int pp_engine_idx_from_code(uint8_t engine) {
    for (int i = 0; i < SP_ENGINE_COUNT; i++) {
        if (SP_ENGINES[i].engine == engine) return i;
    }
    return -1;
}

static uint8_t xtra_engine_idx_from_pp_engine(int pp_idx) {
    if (pp_idx < 0 || pp_idx >= SP_ENGINE_COUNT) return 3;
    switch (SP_ENGINES[pp_idx].engine) {
        case 3: return 3;
        case 4: return 4;
        case 5: return 5;
        case 6: return 6;
        case 7: return 7;
        default: return 3;
    }
}

static lv_color_t pp_engine_color(int idx) {
    static const uint32_t colors[SP_ENGINE_COUNT] = {
        0x00E5FF, 0xFF1493, 0xFFE066, 0x7CFF6B
    };
    if (idx < 0 || idx >= SP_ENGINE_COUNT) return RED808_CYAN;
    return lv_color_hex(colors[idx]);
}

static inline int pp_f2i(float v, float vmin, float vmax) {
    if (vmax <= vmin) return 0;
    float t = (v - vmin) / (vmax - vmin);
    if (t < 0) t = 0; if (t > 1) t = 1;
    return (int)(t * 1000.0f + 0.5f);
}
static inline float pp_i2f(int i, float vmin, float vmax) {
    return vmin + (vmax - vmin) * (float)i / 1000.0f;
}

static void pp_format_value(char* buf, size_t bufsz, const SynthParamDef* p, float v) {
    if (p->step_int) {
        snprintf(buf, bufsz, "%d%s%s", (int)(v + 0.5f), p->unit[0] ? " " : "", p->unit);
    } else if (p->vmax >= 100.f) {
        snprintf(buf, bufsz, "%.0f%s%s", v, p->unit[0] ? " " : "", p->unit);
    } else if (p->vmax >= 10.f) {
        snprintf(buf, bufsz, "%.2f%s%s", v, p->unit[0] ? " " : "", p->unit);
    } else {
        snprintf(buf, bufsz, "%.3f%s%s", v, p->unit[0] ? " " : "", p->unit);
    }
}

static void pp_init_engine_defaults(int eng_idx) {
    const SynthEngineDef* eng = &SP_ENGINES[eng_idx];
    for (uint8_t i = 0; i < eng->param_count && i < PP_MAX_PARAMS_P4; i++) {
        s_pp_values[eng_idx][i] = eng->params[i].vdef;
    }
}

static void piano_sync_active_engine_state(void) {
    if (!ui_control_available()) return;
    int eng_idx = s_piano_engine_idx;
    if (eng_idx < 0 || eng_idx >= SP_ENGINE_COUNT) return;
    const SynthEngineDef* eng = &SP_ENGINES[eng_idx];
    int preset_idx = s_pp_preset_idx[eng_idx];
    uint8_t packets = 0;
    if (preset_idx >= 0 && preset_idx < eng->preset_count) {
        control_send_synth_preset(eng->engine, (uint8_t)preset_idx);
        packets++;
    }
    piano_apply_glide_setting();
    if (eng->engine == SP_ENGINE_WT) {
        piano_reset_vertical_expression();
        packets += 2;
    }
    (void)packets;
#if P4_ENABLE_DEBUG_LOG
    Serial.printf("[P4 piano] sync eng=%u preset=%d packets~%u\n",
                  eng->engine, preset_idx, packets);
#endif
}

static bool piano_vertical_expression_active(void) {
    return piano_engine_code() == SP_ENGINE_WT && !s_piano_glide_enabled && !s_piano_bend_enabled;
}

static float piano_get_wt_cutoff_base(void) {
    const SynthEngineDef* eng = &SP_ENGINES[1];
    for (uint8_t i = 0; i < eng->param_count && i < PP_MAX_PARAMS_P4; i++) {
        if (eng->params[i].param_id == 4) {
            float v = s_pp_values[1][i];
            if (v >= eng->params[i].vmin && v <= eng->params[i].vmax) return v;
            return eng->params[i].vdef;
        }
    }
    return 8000.0f;
}

static float piano_get_wt_volume_base(void) {
    const SynthEngineDef* eng = &SP_ENGINES[1];
    for (uint8_t i = 0; i < eng->param_count && i < PP_MAX_PARAMS_P4; i++) {
        if (eng->params[i].param_id == 3) {
            float v = s_pp_values[1][i];
            if (v >= eng->params[i].vmin && v <= eng->params[i].vmax) return v;
            return eng->params[i].vdef;
        }
    }
    return 0.75f;
}

static void piano_update_status_note(uint8_t midi_note) {
    if (!s_piano_status_lbl) return;
    lv_label_set_text_fmt(s_piano_status_lbl, "%s · %s",
                          PIANO_ENGINE_LABELS[s_piano_engine_idx],
                          piano_note_name(midi_note));
}

static void piano_update_expression_status(void) {
    if (!s_piano_status_lbl) return;
    if (s_piano_held_note < 0) {
        lv_label_set_text(s_piano_status_lbl, "—");
        return;
    }
    int amount = (int)lroundf(s_piano_expr_last_amount * 100.0f);
    lv_label_set_text_fmt(s_piano_status_lbl, "%s · %s · EXP %d%%",
                          PIANO_ENGINE_LABELS[s_piano_engine_idx],
                          piano_note_name((uint8_t)s_piano_held_note),
                          amount);
}

static void piano_update_expression_bar(void) {
    if (!s_piano_expr_bar) return;
    int container_h = s_piano_keys_container ? lv_obj_get_height(s_piano_keys_container) : 0;
    if (container_h < 120) container_h = ui_layout_h() - 152;
    if (container_h < 120) container_h = 120;
    int fill_h = (int)(s_piano_expr_last_amount * (float)(container_h - 8) + 0.5f);
    if (fill_h < 8) fill_h = 8;
    lv_obj_set_height(s_piano_expr_bar, fill_h);
    lv_obj_align(s_piano_expr_bar, LV_ALIGN_BOTTOM_MID, 0, -2);
    lv_obj_set_style_bg_opa(s_piano_expr_bar,
                            s_piano_expr_last_amount > 0.01f ? LV_OPA_COVER : LV_OPA_30,
                            0);
}

static void piano_apply_vertical_expression(int16_t ly) {
    if (!ui_control_available() || !piano_vertical_expression_active()) return;
    const float ceil_cutoff = 18000.0f;
    const float max_volume_boost = 0.18f;
    const float dead_zone_px = 5.0f;
    const float full_scale_px = 80.0f;
    float dy = (float)(s_piano_touch_start_y - ly);
    float target = s_piano_expr_base_cutoff;
    float target_volume = s_piano_expr_base_volume;
    if (dy > dead_zone_px) {
        float t = (dy - dead_zone_px) / (full_scale_px - dead_zone_px);
        if (t > 1.0f) t = 1.0f;
        s_piano_expr_last_amount = t;
        target = s_piano_expr_base_cutoff + t * (ceil_cutoff - s_piano_expr_base_cutoff);
        target_volume = s_piano_expr_base_volume + t * max_volume_boost;
        if (target_volume > 1.0f) target_volume = 1.0f;
    } else {
        s_piano_expr_last_amount = 0.0f;
    }
    uint32_t now = millis();
    if ((now - s_piano_expr_last_send_ms) < 10 &&
        fabsf(target - s_piano_expr_last_cutoff) < 80.0f &&
        fabsf(target_volume - s_piano_expr_last_volume) < 0.015f) {
        return;
    }
    s_piano_expr_last_cutoff = target;
    s_piano_expr_last_volume = target_volume;
    s_piano_expr_last_send_ms = now;
    control_send_synth_param(SP_ENGINE_WT, 0, 4, target);
    control_send_synth_param(SP_ENGINE_WT, 0, 3, target_volume);
    piano_update_expression_status();
    piano_update_expression_bar();
}

static void piano_reset_vertical_expression(void) {
    if (!ui_control_available()) return;
    float base = piano_get_wt_cutoff_base();
    float base_volume = piano_get_wt_volume_base();
    s_piano_expr_base_cutoff = base;
    s_piano_expr_last_cutoff = base;
    s_piano_expr_base_volume = base_volume;
    s_piano_expr_last_volume = base_volume;
    s_piano_expr_last_amount = 0.0f;
    s_piano_expr_last_send_ms = 0;
    if (piano_engine_code() == SP_ENGINE_WT) {
        control_send_synth_param(SP_ENGINE_WT, 0, 4, base);
        control_send_synth_param(SP_ENGINE_WT, 0, 3, base_volume);
    }
    if (s_piano_held_note >= 0) {
        piano_update_status_note((uint8_t)s_piano_held_note);
    }
    piano_update_expression_bar();
}

static void pp_apply_preset_local(int eng_idx, int preset_idx) {
    const SynthEngineDef* eng = &SP_ENGINES[eng_idx];
    if (preset_idx < 0 || preset_idx >= eng->preset_count) return;
    pp_init_engine_defaults(eng_idx);
    const SynthPreset* pr = &eng->presets[preset_idx];
    for (uint8_t pv = 0; pv < pr->count; pv++) {
        for (uint8_t i = 0; i < eng->param_count; i++) {
            if (eng->params[i].param_id == pr->values[pv].param_id) {
                s_pp_values[eng_idx][i] = pr->values[pv].value;
                break;
            }
        }
    }
}

static void pp_slider_event_cb(lv_event_t* e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    const SynthEngineDef* eng = &SP_ENGINES[s_pp_engine_idx];
    if (slot < 0 || slot >= (int)eng->param_count) return;
    const SynthParamDef* p = &eng->params[slot];
    lv_obj_t* sl = (lv_obj_t*)lv_event_get_target(e);
    int iv = lv_slider_get_value(sl);
    float fv = pp_i2f(iv, p->vmin, p->vmax);
    if (p->step_int) fv = (float)((int)(fv + 0.5f));
    s_pp_values[s_pp_engine_idx][slot] = fv;
    if (s_pp_val_lbls[slot]) {
        char buf[24];
        pp_format_value(buf, sizeof(buf), p, fv);
        lv_label_set_text(s_pp_val_lbls[slot], buf);
    }
    if (ui_control_available()) {
        control_send_synth_param(eng->engine, 0, p->param_id, fv);
    }
    if (s_pp_from_xtra && s_pp_xtra_slot >= 0 && s_pp_xtra_slot < 4) {
        xtra_capture_editor_state(s_pp_xtra_slot);
    }
    pp_refresh_wave_preview();
}

// v2.9 — Hold-to-increment with cell background tracking the value.
// Press = step once. Hold = keep stepping (~10/s). RST badge resets.
static lv_obj_t* s_pp_cell_bars[PP_MAX_PARAMS_P4] = {};

static inline lv_color_t pp_value_color(float t) {
    if (t < 0) t = 0; if (t > 1) t = 1;
    uint8_t r = (uint8_t)(0x0A + t * (0xFF - 0x0A));
    uint8_t g = (uint8_t)(0x18 + t * (0x14 - 0x18));
    uint8_t b = (uint8_t)(0x40 + t * (0x93 - 0x40));
    return lv_color_make(r, g, b);
}

static void pp_cell_redraw_value(int slot) {
    const SynthEngineDef* eng = &SP_ENGINES[s_pp_engine_idx];
    if (slot < 0 || slot >= (int)eng->param_count) return;
    const SynthParamDef* p = &eng->params[slot];
    float fv = s_pp_values[s_pp_engine_idx][slot];
    float t = (fv - p->vmin) / (p->vmax - p->vmin);
    if (t < 0) t = 0; if (t > 1) t = 1;
    if (s_pp_val_lbls[slot]) {
        char buf[24];
        pp_format_value(buf, sizeof(buf), p, fv);
        lv_label_set_text(s_pp_val_lbls[slot], buf);
    }
    if (s_pp_cell_bars[slot]) {
        int full_w = (int)(intptr_t)lv_obj_get_user_data(s_pp_cell_bars[slot]);
        int w = (int)(full_w * t + 0.5f);
        if (w < 4) w = 4;
        lv_obj_set_width(s_pp_cell_bars[slot], w);
        uint8_t r = (uint8_t)(0x00 + t * (0xFF - 0x00));
        uint8_t g = (uint8_t)(0xE5 - t * (0xE5 - 0x14));
        uint8_t b = (uint8_t)(0xFF - t * (0xFF - 0x93));
        lv_obj_set_style_bg_color(s_pp_cell_bars[slot], lv_color_make(r, g, b), 0);
    }
    if (s_pp_sliders[slot]) {
        lv_color_t bg = pp_value_color(t);
        lv_obj_set_style_bg_color(s_pp_sliders[slot], bg, 0);
        uint8_t r = (uint8_t)(0x00 + t * (0xFF - 0x00));
        uint8_t g = (uint8_t)(0xE5 - t * (0xE5 - 0x14));
        uint8_t b = (uint8_t)(0xFF - t * (0xFF - 0x93));
        lv_obj_set_style_border_color(s_pp_sliders[slot], lv_color_make(r, g, b), 0);
    }
}

static void pp_cell_step(int slot) {
    const SynthEngineDef* eng = &SP_ENGINES[s_pp_engine_idx];
    if (slot < 0 || slot >= (int)eng->param_count) return;
    const SynthParamDef* p = &eng->params[slot];
    float fv = s_pp_values[s_pp_engine_idx][slot];
    float range = p->vmax - p->vmin;
    if (range <= 0.f) return;
    float step;
    if (p->step_int && range <= 20.f) {
        step = 1.f;
    } else if (p->step_int) {
        step = range * 0.04f;
    } else {
        step = range * 0.03f;
    }
    fv += step;
    if (fv > p->vmax + 1e-3f) fv = p->vmin;
    if (p->step_int) fv = (float)((int)(fv + 0.5f));
    s_pp_values[s_pp_engine_idx][slot] = fv;
    pp_cell_redraw_value(slot);
    if (ui_control_available()) {
        control_send_synth_param(eng->engine, 0, p->param_id, fv);
    }
    if (s_pp_from_xtra && s_pp_xtra_slot >= 0 && s_pp_xtra_slot < 4) {
        xtra_capture_editor_state(s_pp_xtra_slot);
    }
    pp_refresh_wave_preview();
}

static void pp_cell_press_cb(lv_event_t* e) {
    pp_cell_step((int)(intptr_t)lv_event_get_user_data(e));
}

static void pp_cell_long_repeat_cb(lv_event_t* e) {
    pp_cell_step((int)(intptr_t)lv_event_get_user_data(e));
}

static void pp_cell_reset_cb(lv_event_t* e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    const SynthEngineDef* eng = &SP_ENGINES[s_pp_engine_idx];
    if (slot < 0 || slot >= (int)eng->param_count) return;
    const SynthParamDef* p = &eng->params[slot];
    s_pp_values[s_pp_engine_idx][slot] = p->vdef;
    pp_cell_redraw_value(slot);
    if (ui_control_available()) {
        control_send_synth_param(eng->engine, 0, p->param_id, p->vdef);
    }
    if (s_pp_from_xtra && s_pp_xtra_slot >= 0 && s_pp_xtra_slot < 4) {
        xtra_capture_editor_state(s_pp_xtra_slot);
    }
    pp_refresh_wave_preview();
}

static void pp_update_engine_chips(void);
static void pp_update_preset_chips(void);

static void pp_rebuild_param_grid(void) {
    if (!s_pp_param_panel) return;
    lv_obj_clean(s_pp_param_panel);
    for (int i = 0; i < PP_MAX_PARAMS_P4; i++) {
        s_pp_sliders[i] = NULL;
        s_pp_val_lbls[i] = NULL;
        s_pp_cell_bars[i] = NULL;
    }
    const SynthEngineDef* eng = &SP_ENGINES[s_pp_engine_idx];
    int count = eng->param_count;

    // Use the screen geometry directly — lv_obj_get_width() can return 0 when
    // called before LVGL has laid out the panel after creation.
    int W = ui_layout_w();
    int H = ui_layout_h();
    int panel_w = W - 24;
    int panel_h = H - 152 - 12;

    int cols = PP_GRID_COLS_P4;
    int rows = (count + cols - 1) / cols;
    if (rows < 1) rows = 1;
    int gap = 6;
    int pad = 6;
    int cell_w = (panel_w - 2 * pad - (cols - 1) * gap) / cols;
    int cell_h = (panel_h - 2 * pad - (rows - 1) * gap) / rows;
    if (cell_h < 52) cell_h = 52;

    // Hint label on top of panel telling user the interaction model.
    // We place this OUTSIDE the cells so it doesn't fight for vertical space.

    for (int i = 0; i < count && i < PP_MAX_PARAMS_P4; i++) {
        int col = i % cols;
        int row = i / cols;
        int x = pad + col * (cell_w + gap);
        int y = pad + row * (cell_h + gap);
        const SynthParamDef* p = &eng->params[i];

        // Whole-cell button — gives big touch target, works for portrait LCD.
        lv_obj_t* cell = lv_btn_create(s_pp_param_panel);
        lv_obj_set_size(cell, cell_w, cell_h);
        lv_obj_set_pos(cell, x, y);
        lv_obj_set_style_radius(cell, 8, 0);
        lv_obj_set_style_bg_color(cell, RED808_SURFACE, 0);
        lv_obj_set_style_bg_grad_color(cell, RED808_PANEL, 0);
        lv_obj_set_style_bg_grad_dir(cell, LV_GRAD_DIR_VER, 0);
        lv_obj_set_style_bg_opa(cell, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(cell, pp_engine_color(s_pp_engine_idx), 0);
        lv_obj_set_style_border_opa(cell, (lv_opa_t)115, 0);
        lv_obj_set_style_border_width(cell, 2, 0);
        lv_obj_set_style_shadow_width(cell, 0, 0);
        lv_obj_set_style_pad_all(cell, 0, 0);
        lv_obj_set_style_bg_color(cell, pp_engine_color(s_pp_engine_idx), LV_STATE_PRESSED);
        lv_obj_set_style_bg_opa(cell, (lv_opa_t)216, LV_STATE_PRESSED);
        lv_obj_clear_flag(cell, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t* top_line = lv_obj_create(cell);
        lv_obj_set_size(top_line, cell_w - 12, 3);
        lv_obj_align(top_line, LV_ALIGN_TOP_MID, 0, 5);
        lv_obj_set_style_radius(top_line, 2, 0);
        lv_obj_set_style_bg_color(top_line, pp_engine_color(s_pp_engine_idx), 0);
        lv_obj_set_style_bg_opa(top_line, LV_OPA_70, 0);
        lv_obj_set_style_border_width(top_line, 0, 0);
        lv_obj_clear_flag(top_line, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(top_line, LV_OBJ_FLAG_CLICKABLE);

        // Background fill bar (full cell width = max). Positioned at bottom,
        // grows upward as a horizontal strip showing fill.
        int bar_full_w = cell_w - 14;
        int bar_h = 8;
        lv_obj_t* bar = lv_obj_create(cell);
        lv_obj_set_size(bar, bar_full_w, bar_h);
        lv_obj_set_pos(bar, 7, cell_h - bar_h - 6);
        lv_obj_set_style_radius(bar, bar_h / 2, 0);
        lv_obj_set_style_bg_color(bar, RED808_CYAN, 0);
        lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(bar, 0, 0);
        lv_obj_set_style_shadow_width(bar, 0, 0);
        lv_obj_set_style_pad_all(bar, 0, 0);
        lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(bar, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_user_data(bar, (void*)(intptr_t)bar_full_w);
        s_pp_cell_bars[i] = bar;

        // Param name (top-left, big)
        lv_obj_t* nlbl = lv_label_create(cell);
        lv_label_set_text(nlbl, p->name);
        lv_obj_set_style_text_font(nlbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(nlbl, RED808_TEXT, 0);
        lv_obj_align(nlbl, LV_ALIGN_TOP_LEFT, 10, 11);

        // Value text (centered, very big)
        lv_obj_t* vlbl = lv_label_create(cell);
        lv_obj_set_style_text_font(vlbl, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(vlbl, RED808_WARNING, 0);
        lv_obj_align(vlbl, LV_ALIGN_CENTER, 0, 4);
        s_pp_val_lbls[i] = vlbl;

        // RST badge top-right — child button consumes its own click so the
        // surrounding cell only sees press/hold events.
        lv_obj_t* rst = lv_btn_create(cell);
        lv_obj_set_size(rst, 42, 22);
        lv_obj_align(rst, LV_ALIGN_TOP_RIGHT, -6, 9);
        lv_obj_set_style_radius(rst, 6, 0);
        lv_obj_set_style_bg_color(rst, RED808_BG, 0);
        lv_obj_set_style_bg_grad_color(rst, RED808_SURFACE, 0);
        lv_obj_set_style_bg_grad_dir(rst, LV_GRAD_DIR_VER, 0);
        lv_obj_set_style_border_color(rst, pp_engine_color(s_pp_engine_idx), 0);
        lv_obj_set_style_border_opa(rst, LV_OPA_60, 0);
        lv_obj_set_style_border_width(rst, 1, 0);
        lv_obj_set_style_shadow_width(rst, 0, 0);
        lv_obj_t* rl = lv_label_create(rst);
        lv_label_set_text(rl, "RST");
        lv_obj_set_style_text_font(rl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(rl, RED808_TEXT, 0);
        lv_obj_center(rl);
        lv_obj_add_event_cb(rst, pp_cell_reset_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);

        lv_obj_add_event_cb(cell, pp_cell_press_cb, LV_EVENT_PRESSED, (void*)(intptr_t)i);
        lv_obj_add_event_cb(cell, pp_cell_long_repeat_cb, LV_EVENT_LONG_PRESSED_REPEAT, (void*)(intptr_t)i);
        s_pp_sliders[i] = cell;  // reuse the array slot for cleanup

        pp_cell_redraw_value(i);
    }
}

static void pp_refresh_view(void) {
    if (s_pp_title_lbl) {
        char tbuf[64];
        if (s_pp_from_xtra && s_pp_xtra_slot >= 0 && s_pp_xtra_slot < 4) {
            snprintf(tbuf, sizeof(tbuf), "XTRA EDIT · S%d · %s", s_pp_xtra_slot + 1,
                     SP_ENGINES[s_pp_engine_idx].long_name);
        } else {
            snprintf(tbuf, sizeof(tbuf), "SYNTH LAB · %s", SP_ENGINES[s_pp_engine_idx].long_name);
        }
        lv_label_set_text(s_pp_title_lbl, tbuf);
        lv_obj_set_style_text_color(s_pp_title_lbl, pp_engine_color(s_pp_engine_idx), 0);
    }
    pp_update_engine_chips();
    pp_update_preset_chips();
    pp_refresh_wave_preview();
    pp_rebuild_param_grid();
}

static void pp_update_engine_chips(void) {
    for (int i = 0; i < SP_ENGINE_COUNT; i++) {
        if (!s_pp_engine_btns[i]) continue;
        bool active = (i == s_pp_engine_idx);
        lv_color_t color = pp_engine_color(i);
        lv_obj_set_style_bg_color(s_pp_engine_btns[i], active ? color : RED808_SURFACE, 0);
        lv_obj_set_style_bg_grad_color(s_pp_engine_btns[i], active ? RED808_SURFACE : RED808_PANEL, 0);
        lv_obj_set_style_border_color(s_pp_engine_btns[i], active ? color : RED808_BORDER, 0);
        lv_obj_set_style_border_opa(s_pp_engine_btns[i], active ? LV_OPA_COVER : LV_OPA_70, 0);
        lv_obj_t* lbl = lv_obj_get_child(s_pp_engine_btns[i], 0);
        if (lbl) lv_obj_set_style_text_color(lbl, active ? RED808_BG : color, 0);
    }
}

static void pp_update_preset_chips(void) {
    const SynthEngineDef* eng = &SP_ENGINES[s_pp_engine_idx];
    for (int i = 0; i < 4; i++) {
        if (!s_pp_preset_btns[i]) continue;
        bool active = (i == s_pp_preset_idx[s_pp_engine_idx]);
        lv_color_t color = pp_engine_color(s_pp_engine_idx);
        lv_obj_set_style_bg_color(s_pp_preset_btns[i], active ? color : RED808_SURFACE, 0);
        lv_obj_set_style_bg_grad_color(s_pp_preset_btns[i], active ? RED808_SURFACE : RED808_PANEL, 0);
        lv_obj_set_style_border_color(s_pp_preset_btns[i],
                                      active ? color : RED808_BORDER, 0);
        lv_obj_set_style_border_opa(s_pp_preset_btns[i], active ? LV_OPA_COVER : LV_OPA_70, 0);
        lv_obj_t* lbl = lv_obj_get_child(s_pp_preset_btns[i], 0);
        if (lbl) {
            const char* nm = (i < eng->preset_count) ? eng->presets[i].name : "—";
            lv_label_set_text(lbl, nm);
            lv_obj_set_style_text_color(lbl, active ? RED808_BG : RED808_TEXT, 0);
        }
    }
}

static void pp_engine_cb(lv_event_t* e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (idx < 0 || idx >= SP_ENGINE_COUNT) return;
    s_pp_engine_idx = idx;
    if (s_pp_from_xtra && s_pp_xtra_slot >= 0 && s_pp_xtra_slot < 4) {
        s_xtra_slots[s_pp_xtra_slot].synth_mode = true;
        s_xtra_slots[s_pp_xtra_slot].synth_engine_idx = xtra_engine_idx_from_pp_engine(idx);
        xtra_reset_slot_params(s_pp_xtra_slot);
        xtra_slot_refresh_name(s_pp_xtra_slot);
        xtra_save_state();
        xtra_refresh_panel();
    }
    pp_refresh_view();
}

static void pp_preset_cb(lv_event_t* e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    const SynthEngineDef* eng = &SP_ENGINES[s_pp_engine_idx];
    if (idx < 0 || idx >= eng->preset_count) return;
    pp_apply_preset_local(s_pp_engine_idx, idx);
    s_pp_preset_idx[s_pp_engine_idx] = idx;
    pp_update_preset_chips();
    pp_rebuild_param_grid();
    if (ui_control_available()) {
        piano_send_panic_melodic();
        control_send_synth_preset(eng->engine, (uint8_t)idx);
#if P4_ENABLE_DEBUG_LOG
        Serial.printf("[P4 params] preset eng=%u preset=%d packets=6\n", eng->engine, idx);
#endif
    }
    if (s_pp_from_xtra && s_pp_xtra_slot >= 0 && s_pp_xtra_slot < 4) {
        s_xtra_slots[s_pp_xtra_slot].preset_idx = (uint8_t)idx;
        s_xtra_slots[s_pp_xtra_slot].synth_mode = true;
        s_xtra_slots[s_pp_xtra_slot].synth_engine_idx = xtra_engine_idx_from_pp_engine(s_pp_engine_idx);
        xtra_capture_editor_state(s_pp_xtra_slot);
        xtra_slot_refresh_name(s_pp_xtra_slot);
        xtra_save_state();
        xtra_refresh_panel();
    }
}

static void pp_init_cb(lv_event_t* e) {
    (void)e;
    pp_init_engine_defaults(s_pp_engine_idx);
    s_pp_preset_idx[s_pp_engine_idx] = -1;
    pp_update_preset_chips();
    pp_rebuild_param_grid();
    if (ui_control_available()) {
        const SynthEngineDef* eng = &SP_ENGINES[s_pp_engine_idx];
        for (uint8_t i = 0; i < eng->param_count; i++) {
            control_send_synth_param(eng->engine, 0, eng->params[i].param_id,
                                 s_pp_values[s_pp_engine_idx][i]);
        }
    }
    if (s_pp_from_xtra && s_pp_xtra_slot >= 0 && s_pp_xtra_slot < 4) {
        xtra_capture_editor_state(s_pp_xtra_slot);
    }
}

static void pp_back_cb(lv_event_t* e) {
    (void)e;
    if (s_pp_from_xtra) {
        s_pp_from_xtra = false;
        s_pp_xtra_slot = -1;
        ui_navigate_to(6);
        return;
    }
    ui_navigate_to(10);  // back to PIANO
}

static void xtra_edit_cb(lv_event_t* e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    if (slot < 0 || slot >= 4) return;
    if (!s_xtra_slots[slot].synth_mode) {
        xtra_editor_open(slot);
        return;
    }
    if (xtra_slot_is_drum(slot)) {
        ui_show_toast("Editor XTRA: melodic only", theme_warning());
        return;
    }
    int pp_idx = pp_engine_idx_from_code(xtra_slot_engine_code(slot));
    if (pp_idx < 0 || pp_idx >= SP_ENGINE_COUNT) {
        ui_show_toast("Engine sin editor", theme_warning());
        return;
    }
    s_pp_from_xtra = true;
    s_pp_xtra_slot = slot;
    s_pp_engine_idx = pp_idx;
    int preset_idx = constrain((int)s_xtra_slots[slot].preset_idx, 0, 2);
    xtra_load_editor_state(slot);
    s_pp_preset_idx[pp_idx] = preset_idx;
    ui_navigate_to(11);
}

static void xtra_timing_edit_cb(lv_event_t* e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    if (slot < 0 || slot >= 4) return;
    xtra_editor_open(slot);
}

static void create_piano_params_screen(void) {
    int W = ui_layout_w();
    int H = ui_layout_h();

    for (int e = 0; e < SP_ENGINE_COUNT; e++) {
        pp_init_engine_defaults(e);
    }

    scr_piano_params = lv_obj_create(NULL);
    apply_screen_theme_bg(scr_piano_params);
    lv_obj_clear_flag(scr_piano_params, LV_OBJ_FLAG_SCROLLABLE);
    ui_create_header(scr_piano_params);

    // Title
    s_pp_title_lbl = lv_label_create(scr_piano_params);
    char tbuf[64];
    snprintf(tbuf, sizeof(tbuf), "SYNTH LAB · %s", SP_ENGINES[s_pp_engine_idx].long_name);
    lv_label_set_text(s_pp_title_lbl, tbuf);
    lv_obj_set_style_text_font(s_pp_title_lbl, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(s_pp_title_lbl, pp_engine_color(s_pp_engine_idx), 0);
    lv_obj_set_pos(s_pp_title_lbl, 64, 12);

    // Return-to-PIANO button (top right). Distinct from the header back
    // button (top-left → LIVE): label it "PIANO" so the two aren't ambiguous.
    {
        lv_obj_t* b = lv_btn_create(scr_piano_params);
        lv_obj_set_size(b, 96, 42);
        lv_obj_set_pos(b, W - 108, 8);
        apply_control_button_style(b, RED808_ACCENT, false, 8);
        lv_obj_t* l = lv_label_create(b);
        lv_label_set_text(l, LV_SYMBOL_LEFT " PIANO");
        lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(l, RED808_ACCENT, 0);
        lv_obj_center(l);
        lv_obj_add_event_cb(b, pp_back_cb, LV_EVENT_CLICKED, NULL);
    }

    // Engine tabs row (y=56)
    const int tab_y = 56, tab_h = 38, tab_gap = 6;
    int tab_w = (W - 24 - (SP_ENGINE_COUNT - 1) * tab_gap) / SP_ENGINE_COUNT;
    for (int i = 0; i < SP_ENGINE_COUNT; i++) {
        lv_obj_t* b = lv_btn_create(scr_piano_params);
        lv_obj_set_size(b, tab_w, tab_h);
        lv_obj_set_pos(b, 12 + i * (tab_w + tab_gap), tab_y);
        apply_control_button_style(b, pp_engine_color(i), i == s_pp_engine_idx, 8);
        lv_obj_t* lbl = lv_label_create(b);
        lv_label_set_text(lbl, SP_ENGINES[i].label);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_18, 0);
        lv_obj_center(lbl);
        lv_obj_add_event_cb(b, pp_engine_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
        s_pp_engine_btns[i] = b;
    }

    // Preset chips row (y=104) — 4 presets + INIT
    const int p_y = 104, p_h = 38, p_gap = 6;
    int p_w = (W - 24 - 4 * p_gap - 80) / 4;  // reserve 80 for INIT
    for (int i = 0; i < 4; i++) {
        lv_obj_t* b = lv_btn_create(scr_piano_params);
        lv_obj_set_size(b, p_w, p_h);
        lv_obj_set_pos(b, 12 + i * (p_w + p_gap), p_y);
        apply_control_button_style(b, RED808_BORDER, false, 8);
        lv_obj_t* lbl = lv_label_create(b);
        lv_label_set_text(lbl, SP_ENGINES[s_pp_engine_idx].presets[i].name);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
        lv_obj_center(lbl);
        lv_obj_add_event_cb(b, pp_preset_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
        s_pp_preset_btns[i] = b;
    }
    {
        lv_obj_t* b = lv_btn_create(scr_piano_params);
        lv_obj_set_size(b, 76, p_h);
        lv_obj_set_pos(b, W - 88, p_y);
        apply_control_button_style(b, RED808_INFO, false, 8);
        lv_obj_t* l = lv_label_create(b);
        lv_label_set_text(l, "INIT");
        lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(l, RED808_INFO, 0);
        lv_obj_center(l);
        lv_obj_add_event_cb(b, pp_init_cb, LV_EVENT_CLICKED, NULL);
    }

    s_pp_wave_card = lv_obj_create(scr_piano_params);
    lv_obj_set_size(s_pp_wave_card, W - 24, 82);
    lv_obj_set_pos(s_pp_wave_card, 12, 152);
    lv_obj_set_style_radius(s_pp_wave_card, 10, 0);
    lv_obj_set_style_bg_color(s_pp_wave_card, RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_color(s_pp_wave_card, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_dir(s_pp_wave_card, LV_GRAD_DIR_HOR, 0);
    lv_obj_set_style_border_color(s_pp_wave_card, RED808_BORDER, 0);
    lv_obj_set_style_border_width(s_pp_wave_card, 1, 0);
    lv_obj_set_style_pad_all(s_pp_wave_card, 0, 0);
    lv_obj_clear_flag(s_pp_wave_card, LV_OBJ_FLAG_SCROLLABLE);

    s_pp_wave_lbl = lv_label_create(s_pp_wave_card);
    lv_label_set_text(s_pp_wave_lbl, "Preview synth");
    lv_obj_set_style_text_font(s_pp_wave_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_pp_wave_lbl, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(s_pp_wave_lbl, 12, 8);

    s_pp_wave_line = lv_line_create(s_pp_wave_card);
    lv_obj_set_size(s_pp_wave_line, 300, 68);
    lv_obj_align(s_pp_wave_line, LV_ALIGN_BOTTOM_MID, 0, -4);
    lv_obj_set_style_line_color(s_pp_wave_line, RED808_CYAN, 0);
    lv_obj_set_style_line_width(s_pp_wave_line, 3, 0);
    lv_obj_set_style_line_rounded(s_pp_wave_line, true, 0);

    // Param panel
    int panel_y = 242;
    int panel_h = H - panel_y - 12;
    s_pp_param_panel = lv_obj_create(scr_piano_params);
    lv_obj_set_size(s_pp_param_panel, W - 24, panel_h);
    lv_obj_set_pos(s_pp_param_panel, 12, panel_y);
    lv_obj_set_style_radius(s_pp_param_panel, 8, 0);
    lv_obj_set_style_bg_color(s_pp_param_panel, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_color(s_pp_param_panel, RED808_BG, 0);
    lv_obj_set_style_bg_grad_dir(s_pp_param_panel, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_color(s_pp_param_panel, RED808_BORDER, 0);
    lv_obj_set_style_border_width(s_pp_param_panel, 1, 0);
    lv_obj_set_style_pad_all(s_pp_param_panel, 0, 0);
    lv_obj_clear_flag(s_pp_param_panel, LV_OBJ_FLAG_SCROLLABLE);

    pp_refresh_view();
}

// =============================================================================
// PERFORMANCE SCREEN (placeholder)
// =============================================================================
static void create_performance_screen(void) {
    scr_performance = lv_obj_create(NULL);
    apply_screen_theme_bg(scr_performance);
    lv_obj_clear_flag(scr_performance, LV_OBJ_FLAG_SCROLLABLE);
    ui_create_header(scr_performance);

    int W = ui_layout_w();

    lv_obj_t* title = lv_label_create(scr_performance);
    lv_label_set_text(title, "XTRA PADS");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(title, theme_accent2(), 0);
    lv_obj_set_pos(title, 20, 72);

    lv_obj_t* sub = lv_label_create(scr_performance);
    lv_label_set_text(sub, "Banco local por P4 (sin asignacion a los 16 pads)");
    lv_obj_set_style_text_font(sub, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(sub, theme_text_dim(), 0);
    lv_obj_set_pos(sub, 20, 114);

    lv_obj_t* hint = lv_label_create(scr_performance);
    lv_label_set_text(hint, "MODE tap · hold MODE = sound editor · hold PRESET/LOAD = timing + trim");
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(hint, theme_warning(), 0);
    lv_obj_set_pos(hint, 20, 142);

    const int start_x = 20;
    const int start_y = 180;
    const int pad_w = (W - 20 * 2 - 16) / 2;
    const int pad_h = 150;
    for (int i = 0; i < 4; i++) {
        int col = i % 2;
        int row = i / 2;
        grid_xtra_btns[i] = lv_btn_create(scr_performance);
        int main_w = pad_w - 96;
        if (main_w < 120) main_w = 120;
        lv_obj_set_size(grid_xtra_btns[i], main_w, pad_h);
        lv_obj_set_pos(grid_xtra_btns[i], start_x + col * (pad_w + 16), start_y + row * (pad_h + 14));
        lv_obj_set_style_radius(grid_xtra_btns[i], 12, 0);
        lv_obj_set_style_border_width(grid_xtra_btns[i], 2, 0);
        lv_obj_set_style_bg_opa(grid_xtra_btns[i], LV_OPA_80, 0);
        lv_obj_add_event_cb(grid_xtra_btns[i], xtra_slot_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
        lv_obj_add_event_cb(grid_xtra_btns[i], xtra_pad_touch_cb, LV_EVENT_PRESSED, (void*)(intptr_t)i);
        lv_obj_add_event_cb(grid_xtra_btns[i], xtra_pad_touch_cb, LV_EVENT_PRESSING, (void*)(intptr_t)i);
        lv_obj_add_event_cb(grid_xtra_btns[i], xtra_pad_touch_cb, LV_EVENT_RELEASED, (void*)(intptr_t)i);
        lv_obj_add_event_cb(grid_xtra_btns[i], xtra_pad_touch_cb, LV_EVENT_PRESS_LOST, (void*)(intptr_t)i);

        grid_xtra_lbls[i] = lv_label_create(grid_xtra_btns[i]);
        lv_label_set_text(grid_xtra_lbls[i], "808 A");
        lv_obj_set_width(grid_xtra_lbls[i], main_w - 14);
        lv_obj_set_style_text_font(grid_xtra_lbls[i], &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_align(grid_xtra_lbls[i], LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(grid_xtra_lbls[i], LV_ALIGN_CENTER, 0, -18);

        grid_xtra_slot_lbls[i] = lv_label_create(grid_xtra_btns[i]);
        lv_label_set_text_fmt(grid_xtra_slot_lbls[i], "S%02d", i + 1);
        lv_obj_set_style_text_font(grid_xtra_slot_lbls[i], &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(grid_xtra_slot_lbls[i], theme_text_dim(), 0);
        lv_obj_align(grid_xtra_slot_lbls[i], LV_ALIGN_TOP_LEFT, 10, 8);

        grid_xtra_meta_lbls[i] = lv_label_create(grid_xtra_btns[i]);
        lv_label_set_text(grid_xtra_meta_lbls[i], "PRESET A · XY NOTE");
        lv_obj_set_width(grid_xtra_meta_lbls[i], main_w - 20);
        lv_obj_set_style_text_font(grid_xtra_meta_lbls[i], &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(grid_xtra_meta_lbls[i], theme_text(), 0);
        lv_obj_set_style_text_align(grid_xtra_meta_lbls[i], LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(grid_xtra_meta_lbls[i], LV_ALIGN_BOTTOM_MID, 0, -12);

        int card_x = start_x + col * (pad_w + 16);
        int card_y = start_y + row * (pad_h + 14);
        int side_x = card_x + main_w + 8;

        grid_xtra_change_btns[i] = lv_btn_create(scr_performance);
        lv_obj_set_size(grid_xtra_change_btns[i], 88, 68);
        lv_obj_set_pos(grid_xtra_change_btns[i], side_x, card_y);
        apply_control_button_style(grid_xtra_change_btns[i], theme_accent2(), false, 8);
        lv_obj_add_event_cb(grid_xtra_change_btns[i], xtra_change_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
        lv_obj_t* ch_lbl = lv_label_create(grid_xtra_change_btns[i]);
        lv_label_set_text(ch_lbl, "CARGAR\nSAMPLER");
        lv_obj_set_style_text_font(ch_lbl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_align(ch_lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_center(ch_lbl);

        // Legacy secondary control kept allocated for layout compatibility but
        // hidden: XTRAPADS have one operation only, loading a sampler.
        grid_xtra_delete_btns[i] = lv_btn_create(scr_performance);
        lv_obj_set_size(grid_xtra_delete_btns[i], 88, 68);
        lv_obj_set_pos(grid_xtra_delete_btns[i], side_x, card_y + pad_h - 68);
        apply_control_button_style(grid_xtra_delete_btns[i], theme_accent(), false, 8);
        lv_obj_add_flag(grid_xtra_delete_btns[i], LV_OBJ_FLAG_HIDDEN);
        lv_obj_t* del_lbl = lv_label_create(grid_xtra_delete_btns[i]);
        lv_label_set_text(del_lbl, "PRESET\nA");
        lv_obj_set_style_text_font(del_lbl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_align(del_lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_center(del_lbl);
    }

    // ── Page selector: one page per /data/xtra subfolder on Daisy's SD ──
    // Page 0 ("DEFAULT") is the flat file list already there today; extra
    // pages appear automatically once a folder with up to 4 WAVs is copied
    // onto the card under /data/xtra — RESCAN re-checks without leaving.
    const int page_row_y = start_y + 2 * (pad_h + 14) + 8;

    s_xtra_page_prev_btn = lv_btn_create(scr_performance);
    lv_obj_set_size(s_xtra_page_prev_btn, 60, 48);
    lv_obj_set_pos(s_xtra_page_prev_btn, 20, page_row_y);
    apply_control_button_style(s_xtra_page_prev_btn, theme_accent2(), false, 8);
    lv_obj_add_event_cb(s_xtra_page_prev_btn, xtra_page_prev_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* prev_lbl = lv_label_create(s_xtra_page_prev_btn);
    lv_label_set_text(prev_lbl, LV_SYMBOL_LEFT);
    lv_obj_center(prev_lbl);

    s_xtra_page_next_btn = lv_btn_create(scr_performance);
    lv_obj_set_size(s_xtra_page_next_btn, 60, 48);
    lv_obj_set_pos(s_xtra_page_next_btn, 88, page_row_y);
    apply_control_button_style(s_xtra_page_next_btn, theme_accent2(), false, 8);
    lv_obj_add_event_cb(s_xtra_page_next_btn, xtra_page_next_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* next_lbl = lv_label_create(s_xtra_page_next_btn);
    lv_label_set_text(next_lbl, LV_SYMBOL_RIGHT);
    lv_obj_center(next_lbl);

    lv_obj_t* rescan_btn = lv_btn_create(scr_performance);
    lv_obj_set_size(rescan_btn, 168, 48);
    lv_obj_set_pos(rescan_btn, W - 188, page_row_y);
    apply_control_button_style(rescan_btn, theme_warning(), false, 8);
    lv_obj_add_event_cb(rescan_btn, xtra_page_rescan_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* rescan_lbl = lv_label_create(rescan_btn);
    lv_label_set_text(rescan_lbl, LV_SYMBOL_REFRESH "  CARPETAS");
    lv_obj_set_style_text_font(rescan_lbl, &lv_font_montserrat_14, 0);
    lv_obj_center(rescan_lbl);

    s_xtra_page_lbl = lv_label_create(scr_performance);
    lv_label_set_text(s_xtra_page_lbl, "PAGINA 1/1  ·  DEFAULT");
    lv_obj_set_width(s_xtra_page_lbl, W - 188 - 156 - 20);
    lv_obj_set_style_text_font(s_xtra_page_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_xtra_page_lbl, theme_text(), 0);
    lv_obj_set_style_text_align(s_xtra_page_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(s_xtra_page_lbl, 156, page_row_y + 12);

    xtra_load_state();
    xtra_refresh_panel();
}

// =============================================================================
// PUBLIC API
// =============================================================================
void ui_create_all_screens(void) {
    // Keep boot fast and runtime heap low. Heavy editors (the sequencer alone
    // owns hundreds of LVGL objects) are created on first navigation.
    s_pod_owner_badge_count = 0;
    memset(s_pod_owner_badges, 0, sizeof(s_pod_owner_badges));
    pod_config_store_load(s_pod_config);
    create_boot_screen();
    create_live_screen();

    // Start on boot screen
    lv_scr_load(scr_boot);
    active_screen = 0;
}

// =============================================================================
// THEME RELOAD — delete and recreate all themed screens with new colors
// =============================================================================
// Pantalla de aparcamiento durante el rebuild. Antes se aparcaba en scr_boot
// y el terminal BIOS aparecía en cada cambio de tema como si fuera un glitch;
// ahora se muestra una tarjeta mínima con los colores del tema NUEVO.
static void theme_transition_del_cb(lv_timer_t* t) {
    lv_obj_t* scr = (lv_obj_t*)t->user_data;
    if (scr && scr != lv_scr_act()) lv_obj_del(scr);
}

static lv_obj_t* create_theme_transition_screen(void) {
    lv_obj_t* scr = lv_obj_create(NULL);
    apply_screen_theme_bg(scr);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* tag = lv_label_create(scr);
    lv_label_set_text(tag, "VISUAL THEME");
    lv_obj_set_style_text_font(tag, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_letter_space(tag, 4, 0);
    lv_obj_set_style_text_color(tag, RED808_TEXT_DIM, 0);
    lv_obj_align(tag, LV_ALIGN_CENTER, 0, -38);

    lv_obj_t* name = lv_label_create(scr);
    lv_label_set_text(name, theme_presets[ui_theme_index()].name);
    lv_obj_set_style_text_font(name, &lv_font_montserrat_40, 0);
    lv_obj_set_style_text_letter_space(name, 6, 0);
    lv_obj_set_style_text_color(name, RED808_ACCENT, 0);
    lv_obj_align(name, LV_ALIGN_CENTER, 0, 4);

    lv_obj_t* bar = lv_obj_create(scr);
    lv_obj_set_size(bar, 220, 4);
    lv_obj_align(bar, LV_ALIGN_CENTER, 0, 44);
    lv_obj_set_style_bg_color(bar, RED808_ACCENT2, 0);
    lv_obj_set_style_border_width(bar, 0, 0);
    lv_obj_set_style_radius(bar, 2, 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
    return scr;
}

static void ui_reload_themed_screens(void) {
    int saved_screen = active_screen;
    const int saved_pattern = p4.current_pattern;
    P4_THEME_LOG_PRINTF("[THEME] reload begin current=%u p4=%d screen=%d gen=%u heap=%u psram=%u\n",
                        static_cast<unsigned>(ui_theme_index()), p4.theme,
                        saved_screen, static_cast<unsigned>(s_ui_refresh_gen),
                        static_cast<unsigned>(ESP.getFreeHeap()),
                        static_cast<unsigned>(ESP.getFreePsram()));

    // Stop touch_task from hit-testing/enqueuing against pads that are about
    // to be deleted. ui_navigate_to() at the end restores the flag.
    g_live_screen_active.store(false, std::memory_order_release);

    // Invalidate the prev_* dirty caches in the update functions — the
    // recreated widgets need a full first repaint (see s_ui_refresh_gen).
    s_ui_refresh_gen++;
    s_pod_owner_badge_count = 0;
    memset(s_pod_owner_badges, 0, sizeof(s_pod_owner_badges));

    // Modals live on lv_layer_top(), which SURVIVES the screen deletions
    // below. They must be deleted for real here — just nulling the pointers
    // leaves an orphan fullscreen overlay whose close button no longer works.
    pad_inst_modal_close_cb(NULL);
    pod_status_modal_close_cb(NULL);
    if (s_pad_mode_modal) { lv_obj_del(s_pad_mode_modal); s_pad_mode_modal = NULL; }
    if (midi_summary_modal) { lv_obj_del(midi_summary_modal); midi_summary_modal = NULL; }
    midi_song_confirm_close();
    seq_save_confirm_hide();
    seq_variation_modal_hide();

    // The toast is a child of whichever screen was active — it dies with the
    // screen below. Drop the references so ui_toast_update()/ui_show_toast()
    // don't dereference freed memory.
    s_ui_toast = NULL;
    s_ui_toast_label = NULL;
    s_ui_toast_until_ms = 0;

    // Park on a minimal screen (in the NEW theme) so the active screens can
    // be deleted safely underneath — never scr_boot, see comment above.
    lv_obj_t* trans_scr = create_theme_transition_screen();
    lv_scr_load(trans_scr);

    // Delete all themed screens (nullify pointers before delete to avoid stale refs)
    if (scr_live)        { lv_obj_del(scr_live);        scr_live        = NULL; }
    if (scr_sequencer)   { lv_obj_del(scr_sequencer);   scr_sequencer   = NULL; }
    if (scr_fx)          { lv_obj_del(scr_fx);          scr_fx          = NULL; }
    if (scr_volumes)     { lv_obj_del(scr_volumes);     scr_volumes     = NULL; }
    if (scr_sdcard)      { lv_obj_del(scr_sdcard);      scr_sdcard      = NULL; }
    if (scr_performance) { lv_obj_del(scr_performance); scr_performance = NULL; }
    if (scr_piano)       { lv_obj_del(scr_piano);       scr_piano       = NULL; }
    if (scr_piano_params){ lv_obj_del(scr_piano_params);scr_piano_params= NULL; }
    if (scr_fx_xy)       { lv_obj_del(scr_fx_xy);       scr_fx_xy       = NULL; }

    // Clear widget pointers (prevent stale access in update functions)
    header_bar = NULL; hdr_bpm_label = NULL; hdr_pattern_label = NULL;
    hdr_play_btn = NULL; hdr_play_label = NULL;
    hdr_pattern_minus_btn = NULL; hdr_pattern_plus_btn = NULL;
    for (int i = 0; i < 16; i++) hdr_step_dots[i] = NULL;
    for (int i = 0; i < 16; i++) {
        live_pad_btns[i] = NULL; live_pad_labels[i] = NULL;
        live_pad_num_labels[i] = NULL; live_pad_state_labels[i] = NULL;
        live_pad_inst_labels[i] = NULL;
        live_pad_accent_strips[i] = NULL;
        live_pad_midi_badges[i] = NULL;
        live_spectrum_bars[i] = NULL;
        grid_step_dots[i] = NULL;
    }
    s_live_midi_badge = NULL;
    memset(live_home_panels, 0, sizeof(live_home_panels));
    grid_fx_btn = NULL;
    grid_fx_active_badge = NULL;
    s_pad_back_btn = NULL;
    grid_play_btn = NULL; grid_play_lbl = NULL; grid_bpm_lbl = NULL;
    grid_tempo_ref_lbl = NULL;
    grid_pat_lbl = NULL; grid_step_lbl = NULL;
    grid_nr_btn = NULL; grid_nr_lbl = NULL;
    grid_16l_btn = NULL; grid_16l_lbl = NULL;
    grid_mstr_dot = NULL; grid_mstr_lbl = NULL;
    grid_vol_lbl = NULL; grid_sync_btn = NULL;
    grid_home_vol_lbl = NULL;
    grid_pad_prev_btn = NULL;
    grid_pad_next_btn = NULL;
    grid_pad_lbl = NULL;
    grid_inst_prev_btn = NULL;
    grid_inst_next_btn = NULL;
    grid_inst_lbl = NULL;
    grid_inst_edit_btn = NULL;
    memset(grid_xtra_btns, 0, sizeof(grid_xtra_btns));
    memset(grid_xtra_lbls, 0, sizeof(grid_xtra_lbls));
    memset(grid_xtra_change_btns, 0, sizeof(grid_xtra_change_btns));
    memset(grid_xtra_delete_btns, 0, sizeof(grid_xtra_delete_btns));
    memset(grid_xtra_meta_lbls, 0, sizeof(grid_xtra_meta_lbls));
    memset(grid_xtra_slot_lbls, 0, sizeof(grid_xtra_slot_lbls));
    s_xtra_editor_modal = NULL;
    s_xtra_editor_start = NULL; s_xtra_editor_end = NULL; s_xtra_editor_gate = NULL;
    s_xtra_editor_start_lbl = NULL; s_xtra_editor_end_lbl = NULL;
    s_xtra_editor_gate_lbl = NULL; s_xtra_editor_mode_lbl = NULL;
    s_xtra_editor_wave = NULL; s_xtra_editor_slot = -1;
    s_pad_inst_modal = NULL;
    s_pad_inst_modal_pad_lbl = NULL;
    s_pad_inst_modal_inst_lbl = NULL;
    memset(s_pad_inst_modal_pad_btns, 0, sizeof(s_pad_inst_modal_pad_btns));
    memset(s_pad_inst_modal_inst_btns, 0, sizeof(s_pad_inst_modal_inst_btns));
    for (int e2 = 0; e2 < 3; e2++) {
        s_pad_inst_modal_kit_lbl_eng[e2] = NULL;
        for (int p = 0; p < 5; p++) s_pad_inst_modal_kit_btns[e2][p] = NULL;
    }
    for (int i = 0; i < FX_CARD_COUNT; i++) {
        fx_cards[i] = NULL; fx_arcs[i] = NULL; fx_value_labels[i] = NULL;
        fx_name_labels[i] = NULL; fx_src_labels[i] = NULL; fx_toggle_btns[i] = NULL;
        fx_pct_labels[i] = NULL;
        fx_midi_badges[i] = NULL;
    }
    for (int i = 0; i < FX_PAGE_DOT_COUNT; i++) fx_page_dot[i] = NULL;
    fx_page_lbl = NULL;
    fx_view_btn = NULL;
    fx_view_lbl = NULL;
    fx_pattern_lbl = NULL;
    fx_active_lbl = NULL;
    fx_all_off_btn = NULL;
    s_fx_random_btn = NULL;
    s_fx_random_stop_badge = NULL;
    fx_page = 0;
    fx_view_mode = 0;
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 16; j++) {
            seq_step_btns[i][j] = NULL;
            seq_step_accents[i][j] = NULL;
            seq_step_prob_dot[i][j] = NULL;
        }
        seq_track_labels[i] = NULL; seq_mute_btns[i] = NULL;
        seq_solo_btns[i] = NULL;
        seq_solo_labels[i] = NULL;
        seq_fx_btns[i] = NULL;
        seq_fx_clear_btns[i] = NULL;
        seq_ruler_labels[i] = NULL;
    }
    for (int b = 0; b < 4; b++) seq_beat_bg[b] = NULL;
    seq_step_prob_modal = NULL;
    seq_step_prob_title = NULL;
    for (int i = 0; i < 5; i++) seq_step_prob_btns[i] = NULL;
    for (int i = 0; i < 3; i++) seq_step_lock_btns[i] = NULL;
    seq_step_prob_track = -1;
    seq_step_prob_step = -1;
    s_seq_midi_badge = NULL;
    seq_playhead_line = NULL;
    seq_status_step_lbl = NULL;
    seq_status_pat_lbl = NULL;
    seq_status_name_lbl = NULL;
    seq_status_bpm_lbl = NULL;
    seq_status_mix_lbl = NULL;
    memset(seq_page_btns, 0, sizeof(seq_page_btns));
    memset(seq_page_lbls, 0, sizeof(seq_page_lbls));
    seq_hdr_play_btn = NULL;
    seq_hdr_play_lbl = NULL;
    seq_hdr_pat_lbl = NULL;
    seq_hdr_name_lbl = NULL;
    seq_hdr_queue_btn = NULL;
    seq_hdr_queue_lbl = NULL;
    seq_hdr_var_btn = NULL;
    seq_hdr_song_btn = NULL;
    seq_song_stop_badge = NULL;
    seq_song_modal = NULL;
    for (int i = 0; i < 6; i++) seq_song_style_btns[i] = NULL;
    for (int i = 0; i < 4; i++) seq_song_bars_btns[i] = NULL;
    seq_song_toggle_btn = NULL;
    seq_hdr_evolve_btn = NULL;
    seq_evolve_stop_badge = NULL;
    seq_evolve_modal = NULL;
    seq_evolve_amount_slider = NULL;
    seq_evolve_amount_lbl = NULL;
    for (int i = 0; i < 4; i++) seq_evolve_bars_btns[i] = NULL;
    seq_evolve_toggle_btn = NULL;
    seq_variation_modal = NULL;
    seq_hdr_save_btn = NULL;
    seq_hdr_save_lbl = NULL;
    seq_hdr_kanban_btn = NULL;
    seq_kanban_modal = NULL;
    for (int m = 0; m < KANBAN_COUNT; ++m) {
        seq_kanban_toggle_btns[m] = NULL;
        for (int i = 0; i < 4; ++i) seq_kanban_bars_btns[m][i] = NULL;
    }
    seq_kanban_style_btn = NULL;
    seq_kanban_evolve_slider = NULL;
    seq_kanban_evolve_lbl = NULL;
    seq_kanban_variation_btn = NULL;
    seq_kanban_toast_btn = NULL;
    seq_kanban_song_curated_btn = NULL;
    seq_pattern_list_modal = NULL;
    seq_save_confirm_modal = NULL;
    seq_save_confirm_slot = -1;
    memset(seq_hdr_group_btns, 0, sizeof(seq_hdr_group_btns));
    memset(seq_hdr_group_state, 0xFF, sizeof(seq_hdr_group_state));
    seq_groups_modal = NULL;
    memset(seq_groups_modal_btns, 0, sizeof(seq_groups_modal_btns));
    seq_pattern_modal = NULL;
    seq_pattern_modal_lbl = NULL;
    seq_pattern_modal_spin = NULL;
    seq_pattern_wait_pat = -1;
    seq_pattern_wait_ms = 0;
    seq_pattern_waiting = false;
    for (int i = 0; i < 16; i++) {
        vol_sliders[i] = NULL; vol_labels[i] = NULL;
        vol_name_labels[i] = NULL; vol_mute_dots[i] = NULL;
        vol_strip_panels[i] = NULL;
    }
    mix_master_slider = NULL; mix_seq_slider = NULL;
    mix_live_slider = NULL; mix_bpm_slider = NULL;
    mix_master_lbl = NULL; mix_seq_lbl = NULL;
    mix_live_lbl = NULL; mix_bpm_lbl = NULL;
    mix_pattern_lbl = NULL;
    s_mix_random_btn = NULL;
    s_mix_random_stop_badge = NULL;

    // Clear SD screen widgets
    sd_left_panel = NULL; sd_right_panel = NULL; sd_status_lbl = NULL;
    sd_path_lbl = NULL; sd_file_list = NULL; sd_selected_lbl = NULL;
    sd_assign_lbl = NULL;
    sd_load_btn = NULL; sd_load_lbl = NULL;
    sd_preview_btn = NULL; sd_preview_lbl = NULL;
    for (int i = 0; i < 16; i++) sd_pad_btns[i] = NULL;
    sd_wav_section = NULL; sd_midi_section = NULL;
    sd_midi_info_lbl = NULL; sd_midi_status_lbl = NULL; sd_midi_load_btn = NULL;
    sd_midi_song_btn = NULL;
    memset(sd_midi_pat_btns, 0, sizeof(sd_midi_pat_btns));
    sd_midi_mode_pro_btn = NULL; sd_midi_mode_std_btn = NULL;
    sd_src_sd_btn = NULL; sd_src_mem_btn = NULL; sd_src_daisy_btn = NULL;
    sd_is_midi_mode = false;

    // Recreate LIVE only. The saved active editor is created lazily below;
    // hidden screens remain unallocated until the user opens them again.
    create_live_screen();
    s_piano_keys_container = NULL; s_piano_octave_lbl = NULL;
    s_piano_keys24_btn = NULL; s_piano_keys24_lbl = NULL; s_piano_status_lbl = NULL;
    s_piano_expr_bar = NULL;
    s_piano_rec_btn = NULL; s_piano_rec_lbl = NULL;
    s_piano_pad_lbl = NULL;
    s_piano_glide_btn = NULL; s_piano_glide_lbl = NULL;
    s_piano_bend_btn = NULL; s_piano_bend_lbl = NULL;
    s_piano_gate_btn = NULL; s_piano_gate_lbl = NULL;
    for (int i = 0; i < PIANO_ENGINE_COUNT; i++) s_piano_engine_btns[i] = NULL;
    for (int i = 0; i < 4; i++) s_piano_eng_preset_btns[i] = NULL;
    memset(s_piano_key_obj_by_note, 0, sizeof(s_piano_key_obj_by_note));
    s_piano_grid_container = NULL;
    memset(s_piano_grid_btns, 0, sizeof(s_piano_grid_btns));
    s_piano_play_btn = NULL; s_piano_play_lbl = NULL;
    s_pp_param_panel = NULL; s_pp_title_lbl = NULL;
    for (int i = 0; i < SP_ENGINE_COUNT; i++) s_pp_engine_btns[i] = NULL;
    for (int i = 0; i < 4; i++) s_pp_preset_btns[i] = NULL;
    for (int i = 0; i < PP_MAX_PARAMS_P4; i++) {
        s_pp_sliders[i] = NULL; s_pp_val_lbls[i] = NULL;
        s_pp_cell_bars[i] = NULL;
    }
    s_pp_wave_card = NULL; s_pp_wave_line = NULL; s_pp_wave_lbl = NULL;
    s_fxxy_pad = NULL; s_fxxy_dot = NULL;
    s_fxxy_x_lbl = NULL; s_fxxy_y_lbl = NULL; s_fxxy_mode_lbl = NULL;

    // Restore navigation (go to live if was on unknown screen)
    int nav_to = (saved_screen == 9) ? 9 : 2;  // stay in sdcard if we were there
    if (saved_screen == 6) nav_to = 6;
    if (saved_screen == 3) nav_to = 3;
    if (saved_screen == 7) nav_to = 7;
    if (saved_screen == 8) nav_to = 8;
    if (saved_screen == 10) nav_to = 10;   /* PIANO */
    if (saved_screen == 11) nav_to = 11;   /* PIANO PARAMS (synth editor) */
    if (saved_screen == 13) nav_to = 13;   /* FX XY PAD */
    ui_navigate_to(nav_to);

    /* A visual theme must never select a different musical pattern. Preserve
     * the slot while widgets are rebuilt and request that exact grid again. */
    if (p4.current_pattern != saved_pattern) p4.current_pattern = saved_pattern;
    if (nav_to == 3) {
        control_send_select_pattern(saved_pattern);
        control_send_get_pattern(saved_pattern);
    }

    // ui_navigate_to() fades into the rebuilt screen over 200 ms with the
    // parking screen as the anim's "old screen" — it can't be deleted
    // synchronously here, so a one-shot timer reaps it once the fade is done.
    lv_timer_t* reap = lv_timer_create(theme_transition_del_cb, 450, trans_scr);
    lv_timer_set_repeat_count(reap, 1);
    P4_THEME_LOG_PRINTF("[THEME] reload end current=%u p4=%d screen=%d gen=%u heap=%u psram=%u\n",
                        static_cast<unsigned>(ui_theme_index()), p4.theme,
                        active_screen, static_cast<unsigned>(s_ui_refresh_gen),
                        static_cast<unsigned>(ESP.getFreeHeap()),
                        static_cast<unsigned>(ESP.getFreePsram()));
}

void ui_navigate_to(int screen_id) {
    // Lazy screen creation: create only what the user actually opens.
    switch (screen_id) {
        case 2:  if (!scr_live)         create_live_screen(); break;
        case 3:  if (!scr_sequencer)    create_sequencer_screen(); break;
        case 6:  if (!scr_performance)  create_performance_screen(); break;
        case 7:  if (!scr_volumes)      create_volumes_screen(); break;
        case 8:  if (!scr_fx)           create_fx_screen(); break;
        case 9:  if (!scr_sdcard)       create_sdcard_screen(); break;
        case 10: if (!scr_piano)        create_piano_screen(); break;
        case 11: if (!scr_piano_params) create_piano_params_screen(); break;
        case 13: if (!scr_fx_xy)        create_fx_xy_screen(); break;
        default: break;
    }
    lv_obj_t* targets[] = {
        scr_boot, NULL, scr_live, scr_sequencer, NULL,
        NULL, scr_performance, scr_volumes, scr_fx, scr_sdcard,
        scr_piano,        /* 10 = PIANO (replaces stubbed performance slot) */
        scr_piano_params, /* 11 = PIANO PARAMS (synth editor) */
        NULL,             /* 12 = reserved (guitar screen removed) */
        scr_fx_xy         /* 13 = FX XY PAD */
    };
    int count = sizeof(targets) / sizeof(targets[0]);
    if (screen_id >= 0 && screen_id < count && targets[screen_id]) {
        if (screen_id == 6) {
            xtra_pages_request_folders();
        }
        if (screen_id == 11) {
            if (s_pp_from_xtra && s_pp_xtra_slot >= 0 && s_pp_xtra_slot < 4) {
                pp_refresh_view();
            } else if (s_piano_engine_idx >= 0 && s_piano_engine_idx < SP_ENGINE_COUNT) {
                s_pp_engine_idx = s_piano_engine_idx;
                pp_refresh_view();
                piano_sync_active_engine_state();
            }
        }
        if (screen_id == 10) {
            if (active_screen == 11 && s_pp_engine_idx >= 0 && s_pp_engine_idx < PIANO_ENGINE_COUNT) {
                s_piano_engine_idx = s_pp_engine_idx;
                piano_refresh_engine_chips();
                if (ui_control_available()) {
                    control_send_melody_set_engine(PIANO_ENGINES[s_piano_engine_idx]);
                }
                piano_publish_local_state();
            }
            piano_sync_active_engine_state();
        }
        if (screen_id != 9) s_sd_for_xtra = false;
        if (screen_id != 3) {
            seq_save_confirm_hide();
            seq_pattern_list_hide();
            seq_variation_modal_hide();
        }
        // Entering the XY pad: re-place the dot from the live FX state (the
        // master may have moved cutoff/reso since the screen was created).
        if (screen_id == 13) fxxy_sync_from_state();
        // Leaving a screen: persist any pending XTRA param edits now instead
        // of waiting out the debounce window.
        xtra_param_save_flush();
        bool keep_piano_preview = s_piano_play_active &&
            ((active_screen == 10 && screen_id == 11) || (active_screen == 11 && screen_id == 10));
        // Before leaving most screens, stop active synths to prevent stuck notes.
        // Keep the local Melody preview alive while moving between PIANO and PARAMS.
        if (control_available() && !keep_piano_preview) {
            for (int eng = 0; eng < 8; eng++) {  // valid engines 0..7
                control_send_synth_note_off(eng, 0);  // engine, track=0
            }
        }
        
        lv_scr_load_anim(targets[screen_id], LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, false);
        prev_active_screen = active_screen;
        active_screen = screen_id;
        if (screen_id == 3) {
            // Entering sequencer: enforce fresh pattern pull from Master.
            control_send_select_pattern(p4.current_pattern);
            control_send_get_pattern(p4.current_pattern);
        }
        // Refresh current storage source when entering SD screen
        if (screen_id == 9) sd_refresh_source();
    }
    // Enable/disable direct touch bypass for live pads
    g_live_screen_active.store(screen_id == 2, std::memory_order_release);
}

// =============================================================================
// PAD QUEUE DRAIN — called from loop() on Core 1 (outside LVGL mutex)
// =============================================================================
// Pending note-off for melodic engines (303/WT/FM2/SH101) so a pad-tap does
// not leave the synth voice ringing forever. Index by pad 0..15.
static uint32_t s_pad_noteoff_at[16]    = {0};
static int8_t   s_pad_noteoff_engine[16] = {-1, -1, -1, -1, -1, -1, -1, -1,
                                           -1, -1, -1, -1, -1, -1, -1, -1};

void ui_process_control_queue(void) {
    const int patternStep = s_ctrl_pattern_step_pending.exchange(
        0, std::memory_order_acquire);
    if (patternStep != 0) {
        int nextPattern = (p4.current_pattern + patternStep)
                        % Config::MAX_PATTERNS;
        if (nextPattern < 0) nextPattern += Config::MAX_PATTERNS;
        seq_queued_pattern = -1;
        control_send_select_pattern(nextPattern);
        // This runs on Core 1 (loop): the grid repaint must happen on the
        // LVGL task — request it instead of calling the widget code here.
        ui_request_sequencer_resync();
    }

    if (s_ctrl_pattern_sync_pending.exchange(false, std::memory_order_acquire)) {
        control_sync_current_pattern();
    }

    if (s_ctrl_mute_mask_pending.exchange(false, std::memory_order_acquire)) {
        uint16_t mask = s_ctrl_mute_mask.load(std::memory_order_acquire);
        control_send_mute_mask(mask);
    }

    if (s_ctrl_solo_mask_pending.exchange(false, std::memory_order_acquire)) {
        uint16_t mask = s_ctrl_solo_mask.load(std::memory_order_acquire);
        control_send_solo_mask(mask);
    }

    uint16_t dirty = s_ctrl_mute_dirty.exchange(0, std::memory_order_acquire);
    if (dirty) {
        uint16_t values = s_ctrl_mute_values.load(std::memory_order_acquire);
        for (uint8_t track = 0; track < 16; track++) {
            uint16_t bit = (uint16_t)(1U << track);
            if (dirty & bit) {
                control_send_mute(track, (values & bit) != 0);
            }
        }
    }
}

void ui_process_pad_queue(void) {
    uint32_t now_ms = millis();
    uint8_t t = s_pad_qt.load(std::memory_order_relaxed);
    uint8_t h = s_pad_qh.load(std::memory_order_acquire);
    while (t != h) {
        uint16_t ev = s_pad_q[t & 0x1F];
        t++;
        uint8_t pad      = (uint8_t)(ev & 0xFF);
        uint8_t velocity = (uint8_t)((ev >> 8) & 0xFF);
        if (!velocity) velocity = 100;   // defensive floor
        // Feed DSP spectrum with real velocity
        dsp_notify_pad(pad, velocity);
        // Notify the local controller first. The internal tap event ignores
        // velocity; we still send it for forward compatibility.
        local_apply_message(MSG_TOUCH_CMD, TCMD_PAD_TAP, pad);
        // Then send the DaisyPod3 trigger with MPC-style velocity.
        if (p4.master_connected) {
            // Kit per-pad: si el pad usa un engine drum (808/909/505) y su
            // kit asignado difiere del último aplicado a ese engine en la
            // Daisy, manda CMD_SYNTH_PRESET justo antes del trigger. Esto
            // permite que dos pads del mismo engine suenen con kits distintos
            // (a costa de un cambio de preset por golpe cuando alternan).
            if (pad < 16) {
                int8_t drum = pad_inst_drum_engine_idx(s_pad_inst_sel[pad]);
                if (drum >= 0) {
                    uint8_t kit = s_pad_kit_assigned[pad];
                    if (kit > 4) kit = 0;
                    if (s_engine_kit_last_applied[drum] != (int8_t)kit) {
                        control_send_synth_preset((uint8_t)drum, kit);
                        s_engine_kit_last_applied[drum] = (int8_t)kit;
                    }
                }
            }
            control_send_trigger(pad, velocity);
            // Schedule synth note-off for melodic engines so 303/WT/FM2/
            // SH101 also receives a scheduled note-off after a pad tap. Drum samples
            // (engine -1, 0, 1, 2) already self-terminate.
            if (pad < 16) {
                int8_t engine = pad_inst_engine_code(s_pad_inst_sel[pad]);
                if (engine >= 3 && engine <= 6) {
                    // 303 note-off is engine-global. Cancel an older timer
                    // before a new 303 hit so it cannot cut the fresh note.
                    if (engine == 3) {
                        for (int other = 0; other < 16; other++) {
                            if (other != pad && s_pad_noteoff_engine[other] == engine) {
                                s_pad_noteoff_engine[other] = -1;
                                s_pad_noteoff_at[other] = 0;
                            }
                        }
                    }
                    float bpm = (float)p4.bpm_int + (float)p4.bpm_frac * 0.1f;
                    if (bpm < 40.0f || bpm > 300.0f) bpm = 120.0f;
                    uint32_t sixteenth_ms = (uint32_t)(15000.0f / bpm + 0.5f);
                    uint8_t gate = s_piano_gate_percent.load(std::memory_order_relaxed);
                    uint32_t gate_ms = (uint32_t)((uint64_t)sixteenth_ms * gate / 100U);
                    gate_ms = (uint32_t)constrain((int)gate_ms, 55, 420);
                    s_pad_noteoff_engine[pad] = engine;
                    s_pad_noteoff_at[pad]     = now_ms + gate_ms;
                }
            }
        }
        // Mirror to legacy binary flash timer so screens that still read
        // p4.pad_flash_until (e.g. sequencer sync highlight) keep working.
        p4.pad_flash_until[pad] = millis() + 80;
    }
    s_pad_qt.store(t, std::memory_order_relaxed);

    // Drain pending melodic note-offs.
    for (int pad = 0; pad < 16; pad++) {
        if (!s_pad_noteoff_at[pad]) continue;
        if ((int32_t)(now_ms - s_pad_noteoff_at[pad]) < 0) continue;
        int8_t engine = s_pad_noteoff_engine[pad];
        s_pad_noteoff_at[pad]     = 0;
        s_pad_noteoff_engine[pad] = -1;
        if (!p4.master_connected) continue;
        if (engine == 3) {
            // 303 is a single-voice mono synth on master
            control_send_synth303_note_off();
        } else if (engine >= 0 && engine <= 7) {
            control_send_synth_note_off((uint8_t)engine, (uint8_t)pad);
        }
    }
}

// =============================================================================
// LIVE PAD HIT GEOMETRY — shared between LVGL layout and GT911 touch_task
// =============================================================================
// Pad grid geometry (must match create_live_screen layout below).
// M=8 (margin), CW/CH = pad size, SX/SY = stride (pad + gap).
static constexpr int LIVE_M  = 8;
static constexpr int LIVE_CW = 122;
static constexpr int LIVE_CH = 143;
static constexpr int LIVE_SX = 126;
static constexpr int LIVE_SY = 147;

int ui_pad_from_xy(uint16_t x, uint16_t y, uint8_t* cell_x, uint8_t* cell_y) {
    if (cell_x) *cell_x = 64;
    if (cell_y) *cell_y = 64;
    if (!g_live_screen_active.load(std::memory_order_acquire)) return -1;
    // Hit-test against the cached pad geometry (PAD MODE aware). This runs on
    // touch_task — never touch LVGL objects here (not thread-safe vs render).
    for (int i = 0; i < 16; i++) {
        portENTER_CRITICAL(&s_pad_hit_mux);
        PadHitRect r = s_pad_hit[i];
        portEXIT_CRITICAL(&s_pad_hit_mux);
        if (!r.visible || r.w <= 0 || r.h <= 0) continue;
        int px = r.x;
        int py = r.y;
        int pw = r.w;
        int ph = r.h;
        if ((int)x >= px && (int)x < (px + pw) && (int)y >= py && (int)y < (py + ph)) {
            int dx = (int)x - (int)px;
            int dy = (int)y - (int)py;
            int denom_x = (pw > 1) ? (pw - 1) : 1;
            int denom_y = (ph > 1) ? (ph - 1) : 1;
            if (cell_x) *cell_x = (uint8_t)constrain((dx * 127) / denom_x, 0, 127);
            if (cell_y) *cell_y = (uint8_t)constrain((dy * 127) / denom_y, 0, 127);
            return i;
        }
    }

    // Legacy fallback geometry (default 4x4 layout)
    if (x < LIVE_M || x >= (LIVE_M + 4 * LIVE_SX)) return -1;
    if (y < LIVE_M || y >= (LIVE_M + 4 * LIVE_SY)) return -1;
    int col  = (x - LIVE_M) / LIVE_SX;
    int row  = (y - LIVE_M) / LIVE_SY;
    int x_in = (x - LIVE_M) % LIVE_SX;
    int y_in = (y - LIVE_M) % LIVE_SY;
    if (x_in >= LIVE_CW || y_in >= LIVE_CH) return -1;
    if (col >= 4 || row >= 4) return -1;
    if (cell_x) *cell_x = (uint8_t)constrain((x_in * 127) / (LIVE_CW - 1), 0, 127);
    if (cell_y) *cell_y = (uint8_t)constrain((y_in * 127) / (LIVE_CH - 1), 0, 127);
    return row * 4 + col;
}

static inline uint8_t ui_live_pad_velocity(void) {
    int volume = constrain(p4.live_volume, 0, Config::MAX_VOLUME);
    return (uint8_t)map(volume, 0, Config::MAX_VOLUME, 32, 127);
}

// =============================================================================
// PAD FRAME UPDATE — called from GT911 touch_task (Core 0, 200Hz)
// Rising edge → enqueue event (with 16 Levels remapping if active) and arm
// note-repeat timer. Falling edge → cancel repeat. Held → fire repeats on
// schedule using the current tempo & subdivision.
// =============================================================================
void ui_pad_frame_update(const bool pressed[16], const uint8_t velocity[16],
                         const uint8_t cell_x[16], const uint8_t cell_y[16]) {
    (void)velocity;
    static bool prev_live_active = true;

    // Si el modal PAD SOUND está abierto, ignora los pads físicos:
    // la pantalla está cubierta y cualquier toque debe ir a los
    // botones del modal, no al pad físico que hay debajo.
    if (s_pad_inst_modal) {
        for (int p = 0; p < 16; p++) {
            s_pad_held[p] = false;
            s_pad_repeat_next_ms[p] = 0;
            s_pad_hold_start_ms[p] = 0;
            s_pad_roll_phase[p] = 0;
        }
        return;
    }

    if (!g_live_screen_active.load(std::memory_order_acquire)) {
        // Leaving LIVE screen: release all held pads to Master so they don't sustain
        if (prev_live_active) {  // only on transition OUT of LIVE
            // Send all-notes-off to Master
            if (control_available()) {
                for (int eng = 0; eng < 7; eng++) {
                    control_send_synth_note_off(eng, 0);
                }
            }
            prev_live_active = false;
        }
        
        // Clear state so we don't fire phantom repeats when leaving LIVE
        for (int p = 0; p < 16; p++) {
            s_pad_held[p] = false;
            s_pad_repeat_next_ms[p] = 0;
            s_pad_hold_start_ms[p] = 0;
            s_pad_roll_phase[p] = 0;
        }
        return;
    }
    
    // Entering LIVE screen: reset transition flag
    // Entering/in LIVE screen: reset transition flag for next time we leave
    prev_live_active = true;

    unsigned long now = millis();
    unsigned long nr_interval = ui_nr_interval_ms();    // 0 if NR off

    for (int p = 0; p < 16; p++) {
        bool was_held = s_pad_held[p];
        bool is_held  = pressed[p];
        if (is_held) {
            s_pad_hold_x[p] = cell_x ? cell_x[p] : 64;
            s_pad_hold_y[p] = cell_y ? cell_y[p] : 64;
        }

        if (is_held && !was_held) {
            // ── Rising edge: real finger-down ──
            uint8_t vel = ui_live_pad_velocity();
            s_pad_held_velocity[p] = vel;
            s_pad_hold_start_ms[p] = now;
            s_pad_roll_phase[p] = 0;

            uint8_t send_pad = p;
            uint8_t send_vel = vel;
            if (s_16l_active) {
                // Remap to 16 velocities of the stored source pad
                send_pad = s_16l_src_pad;
                send_vel = (uint8_t)(((p + 1) * 127) / 16);  // 7..127
                if (send_vel < 8) send_vel = 8;
            } else {
                s_16l_src_pad = (uint8_t)p;   // remember for future 16L
            }
            s_pad_inst_focus_pad = (uint8_t)p;
            if (send_pad < 16 && !p4.track_muted[send_pad]) {
                enqueue_pad_event(send_pad, send_vel);
                ui_pad_flash_start(p, vel);
            } else {
                s_pad_flash_vel[p] = 0;
            }

            unsigned long tremolo_interval = ui_pad_tremolo_interval_ms((uint8_t)p, nr_interval);
            s_pad_repeat_next_ms[p] = tremolo_interval
                ? (now + (nr_interval ? tremolo_interval : PAD_TREMOLO_HOLD_MS)) : 0;
        } else if (!is_held && was_held) {
            // ── Falling edge: finger lifted ──
            s_pad_repeat_next_ms[p] = 0;
            s_pad_hold_start_ms[p] = 0;
            s_pad_roll_phase[p] = 0;
        } else if (is_held && s_pad_repeat_next_ms[p]
                   && (int32_t)(now - s_pad_repeat_next_ms[p]) >= 0) {
            // ── Held + tremolo/note-repeat tick ──
            unsigned long tremolo_interval = ui_pad_tremolo_interval_ms((uint8_t)p, nr_interval);
            if (!tremolo_interval) {
                s_pad_repeat_next_ms[p] = 0;
                s_pad_held[p] = is_held;
                continue;
            }
            s_pad_roll_phase[p] = (uint8_t)(s_pad_roll_phase[p] + 1);
            uint8_t vel = ui_pad_tremolo_velocity((uint8_t)p, now);
            uint8_t send_pad = p;
            uint8_t send_vel = vel;
            if (s_16l_active) {
                send_pad = s_16l_src_pad;
                send_vel = (uint8_t)(((p + 1) * 127) / 16);
                if (send_vel < 8) send_vel = 8;
            }
            if (send_pad < 16 && !p4.track_muted[send_pad]) {
                enqueue_pad_event(send_pad, send_vel);
                ui_pad_flash_start(p, vel);
            } else {
                s_pad_flash_vel[p] = 0;
            }
            // Schedule next tick; if we fell behind, catch up without drifting
            // into the far past (e.g. after a blocked frame).
            unsigned long next = s_pad_repeat_next_ms[p] + tremolo_interval;
            if ((int32_t)(next - now) <= 0) next = now + tremolo_interval;
            s_pad_repeat_next_ms[p] = next;
        }

        s_pad_held[p] = is_held;
    }
}

// =============================================================================
// LOCAL STATUS SCREENSAVER — shown after 1 minute without touch.
// =============================================================================
static const uint32_t SCREENSAVER_TIMEOUT_MS = 60UL * 1000UL;
static const uint32_t SCREENSAVER_PAGE_MS = 6500;
static bool     s_screensaver_active = false;
static int      s_screensaver_return = 2;               // pantalla a restaurar
static uint8_t  s_screensaver_page = 0;
static uint32_t s_screensaver_page_started_ms = 0;
static lv_obj_t* s_screensaver_stage = NULL;
static lv_obj_t* s_screensaver_kicker = NULL;
static lv_obj_t* s_screensaver_title = NULL;
static lv_obj_t* s_screensaver_detail = NULL;
static lv_obj_t* s_screensaver_counter = NULL;

struct ScreensaverCreditPage {
    const char* kicker;
    const char* title;
    const char* detail;
};

static const ScreensaverCreditPage SCREENSAVER_CREDITS[] = {
    {
        "RED808 / AGRADECIMIENTOS",
        "GRACIAS",
        "A TODA LA GENTE QUE ME HA AYUDADO,\n"
        "APOYADO Y HECHO POSIBLE ESTE VIAJE."
    },
    {
        "FAMILIA",
        "MoNika",
        "MI MUJER\n\nOTTO  /  MAC  /  BUDY\nMIS BABYS"
    },
    {
        "CREW / THE BOYS",
        "ADRI  /  AITOR  /  BRZUOS\n"
        "FREDY  /  KARZ  /  MARCOS\n"
        "ORIOL  /  VICTURIOSO",
        ""
    },
    {
        "MENTORIA",
        "GUSTAVO PATOW",
        "(UdG)"
    },
    {
        "AMIGOS / FAMILIARES",
        "FRANCESC  /  NONE  /  NANDU\n"
        "LIMA  /  TILLO  /  XARLY\n"
        "ERNEST  /  PAULA  /  NARCIS\n"
        "GRIMAL  /  PANDA  /  GARBIN",
        ""
    },
    {
        "EN MEMORIA DE",
        "JAVI LOBATO  /  ROTEM",
        "DEP"
    }
};
static constexpr uint8_t SCREENSAVER_PAGE_COUNT =
    sizeof(SCREENSAVER_CREDITS) / sizeof(SCREENSAVER_CREDITS[0]);
static lv_obj_t* s_screensaver_progress[SCREENSAVER_PAGE_COUNT] = {};

static void screensaver_fade_exec(void* object, int32_t value) {
    lv_obj_set_style_opa(static_cast<lv_obj_t*>(object),
                         static_cast<lv_opa_t>(value), 0);
}

static void screensaver_translate_exec(void* object, int32_t value) {
    lv_obj_set_style_translate_y(static_cast<lv_obj_t*>(object), value, 0);
}

static void screensaver_show_page(uint8_t page, bool animate) {
    if (!s_screensaver_stage || page >= SCREENSAVER_PAGE_COUNT) return;
    const ScreensaverCreditPage& credit = SCREENSAVER_CREDITS[page];
    const ThemeColors& red808 = theme_presets[THEME_RED808];
    s_screensaver_page = page;
    lv_label_set_text(s_screensaver_kicker, credit.kicker);
    lv_label_set_text(s_screensaver_title, credit.title);
    lv_label_set_text(s_screensaver_detail, credit.detail);
    lv_obj_set_style_text_font(s_screensaver_title,
        page == 0 ? &lv_font_montserrat_40 : &lv_font_montserrat_28, 0);
    lv_label_set_text_fmt(s_screensaver_counter, "%02u / %02u",
                          page + 1u, SCREENSAVER_PAGE_COUNT);
    for (uint8_t i = 0; i < SCREENSAVER_PAGE_COUNT; ++i) {
        lv_obj_set_style_bg_color(s_screensaver_progress[i],
            lv_color_hex(i == page ? red808.accent2 : red808.border), 0);
    }
    if (!animate) return;

    lv_anim_del(s_screensaver_stage, screensaver_fade_exec);
    lv_anim_del(s_screensaver_stage, screensaver_translate_exec);
    lv_obj_set_style_opa(s_screensaver_stage, LV_OPA_20, 0);
    lv_obj_set_style_translate_y(s_screensaver_stage, 16, 0);

    lv_anim_t fade;
    lv_anim_init(&fade);
    lv_anim_set_var(&fade, s_screensaver_stage);
    lv_anim_set_exec_cb(&fade, screensaver_fade_exec);
    lv_anim_set_values(&fade, LV_OPA_20, LV_OPA_COVER);
    lv_anim_set_time(&fade, 520);
    lv_anim_set_path_cb(&fade, lv_anim_path_ease_out);
    lv_anim_start(&fade);

    lv_anim_t rise;
    lv_anim_init(&rise);
    lv_anim_set_var(&rise, s_screensaver_stage);
    lv_anim_set_exec_cb(&rise, screensaver_translate_exec);
    lv_anim_set_values(&rise, 16, 0);
    lv_anim_set_time(&rise, 620);
    lv_anim_set_path_cb(&rise, lv_anim_path_ease_out);
    lv_anim_start(&rise);
}

static void create_screensaver_screen(void) {
    if (scr_screensaver) return;

    const ThemeColors& red808 = theme_presets[THEME_RED808];
    const lv_color_t bg = lv_color_hex(red808.bg);
    const lv_color_t panel = lv_color_hex(red808.panel);
    const lv_color_t border = lv_color_hex(red808.border);
    const lv_color_t text = lv_color_hex(red808.text);
    const lv_color_t dim = lv_color_hex(red808.text_dim);
    const lv_color_t accent = lv_color_hex(red808.accent);
    const lv_color_t accent2 = lv_color_hex(red808.accent2);

    scr_screensaver = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_screensaver, bg, 0);
    lv_obj_clear_flag(scr_screensaver, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* rail = lv_obj_create(scr_screensaver);
    lv_obj_set_size(rail, 308, LCD_V_RES);
    lv_obj_set_pos(rail, 0, 0);
    lv_obj_set_style_radius(rail, 0, 0);
    lv_obj_set_style_bg_color(rail, panel, 0);
    lv_obj_set_style_bg_opa(rail, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(rail, 0, 0);
    lv_obj_clear_flag(rail, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* rail_line = lv_obj_create(scr_screensaver);
    lv_obj_set_size(rail_line, 2, LCD_V_RES);
    lv_obj_set_pos(rail_line, 308, 0);
    lv_obj_set_style_radius(rail_line, 0, 0);
    lv_obj_set_style_bg_color(rail_line, accent, 0);
    lv_obj_set_style_border_width(rail_line, 0, 0);
    lv_obj_clear_flag(rail_line, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(rail_line, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t* brand = lv_label_create(rail);
    lv_label_set_text(brand, "RED808");
    lv_obj_set_style_text_font(brand, &lv_font_montserrat_40, 0);
    lv_obj_set_style_text_color(brand, text, 0);
    lv_obj_set_pos(brand, 30, 30);

    lv_obj_t* product = lv_label_create(rail);
    lv_label_set_text(product, "DRUMMACHINE V2");
    lv_obj_set_style_text_font(product, &lv_font_unscii_16, 0);
    lv_obj_set_style_text_color(product, accent2, 0);
    lv_obj_set_pos(product, 32, 82);

    lv_obj_t* brand_rule = lv_obj_create(rail);
    lv_obj_set_size(brand_rule, 244, 1);
    lv_obj_set_pos(brand_rule, 30, 120);
    lv_obj_set_style_bg_color(brand_rule, border, 0);
    lv_obj_set_style_border_width(brand_rule, 0, 0);
    lv_obj_clear_flag(brand_rule, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(brand_rule, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t* demo_kicker = lv_label_create(rail);
    lv_label_set_text(demo_kicker, "FIRST DEMO");
    lv_obj_set_style_text_font(demo_kicker, &lv_font_unscii_8, 0);
    lv_obj_set_style_text_color(demo_kicker, dim, 0);
    lv_obj_set_pos(demo_kicker, 32, 154);

    lv_obj_t* demo = lv_label_create(rail);
    lv_label_set_text(demo, "01/08/2026\nON/OFF FESTIVAL");
    lv_obj_set_style_text_font(demo, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(demo, text, 0);
    lv_obj_set_style_text_line_space(demo, 8, 0);
    lv_obj_set_pos(demo, 30, 177);

    lv_obj_t* partner_kicker = lv_label_create(rail);
    lv_label_set_text(partner_kicker, "PARTNER TECNOLOGICO");
    lv_obj_set_style_text_font(partner_kicker, &lv_font_unscii_8, 0);
    lv_obj_set_style_text_color(partner_kicker, dim, 0);
    lv_obj_set_pos(partner_kicker, 32, 286);

    lv_obj_t* partner = lv_label_create(rail);
    lv_label_set_text(partner, "Daisy.audio");
    lv_obj_set_style_text_font(partner, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(partner, accent2, 0);
    lv_obj_set_pos(partner, 30, 309);

    lv_obj_t* author_kicker = lv_label_create(rail);
    lv_label_set_text(author_kicker, "DESARROLLADO POR");
    lv_obj_set_style_text_font(author_kicker, &lv_font_unscii_8, 0);
    lv_obj_set_style_text_color(author_kicker, dim, 0);
    lv_obj_set_pos(author_kicker, 32, 408);

    lv_obj_t* author = lv_label_create(rail);
    lv_label_set_text(author, "CESCO FORS");
    lv_obj_set_style_text_font(author, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(author, text, 0);
    lv_obj_set_pos(author, 30, 431);

    lv_obj_t* open_source = lv_label_create(rail);
    lv_label_set_text(open_source, "#opensource");
    lv_obj_set_style_text_font(open_source, &lv_font_unscii_16, 0);
    lv_obj_set_style_text_color(open_source, accent, 0);
    lv_obj_set_pos(open_source, 31, 469);

    lv_obj_t* header = lv_label_create(scr_screensaver);
    lv_label_set_text(header, "AGRADECIMIENTOS / RED808");
    lv_obj_set_style_text_font(header, &lv_font_unscii_16, 0);
    lv_obj_set_style_text_color(header, accent, 0);
    lv_obj_set_style_text_letter_space(header, 2, 0);
    lv_obj_set_pos(header, 354, 36);

    lv_obj_t* header_rule = lv_obj_create(scr_screensaver);
    lv_obj_set_size(header_rule, 620, 1);
    lv_obj_set_pos(header_rule, 354, 78);
    lv_obj_set_style_bg_color(header_rule, border, 0);
    lv_obj_set_style_border_width(header_rule, 0, 0);
    lv_obj_clear_flag(header_rule, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(header_rule, LV_OBJ_FLAG_CLICKABLE);

    s_screensaver_stage = lv_obj_create(scr_screensaver);
    lv_obj_set_size(s_screensaver_stage, 620, 350);
    lv_obj_set_pos(s_screensaver_stage, 354, 116);
    lv_obj_set_style_bg_opa(s_screensaver_stage, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_screensaver_stage, 0, 0);
    lv_obj_set_style_pad_all(s_screensaver_stage, 0, 0);
    lv_obj_clear_flag(s_screensaver_stage, LV_OBJ_FLAG_SCROLLABLE);

    s_screensaver_kicker = lv_label_create(s_screensaver_stage);
    lv_obj_set_width(s_screensaver_kicker, 620);
    lv_obj_set_pos(s_screensaver_kicker, 0, 0);
    lv_obj_set_style_text_font(s_screensaver_kicker, &lv_font_unscii_16, 0);
    lv_obj_set_style_text_color(s_screensaver_kicker, accent2, 0);

    s_screensaver_title = lv_label_create(s_screensaver_stage);
    lv_obj_set_width(s_screensaver_title, 620);
    lv_obj_set_pos(s_screensaver_title, 0, 52);
    lv_label_set_long_mode(s_screensaver_title, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_color(s_screensaver_title, text, 0);
    lv_obj_set_style_text_line_space(s_screensaver_title, 8, 0);

    s_screensaver_detail = lv_label_create(s_screensaver_stage);
    lv_obj_set_width(s_screensaver_detail, 620);
    lv_obj_set_pos(s_screensaver_detail, 0, 216);
    lv_label_set_long_mode(s_screensaver_detail, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(s_screensaver_detail, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(s_screensaver_detail, dim, 0);
    lv_obj_set_style_text_line_space(s_screensaver_detail, 6, 0);

    s_screensaver_counter = lv_label_create(scr_screensaver);
    lv_obj_set_style_text_font(s_screensaver_counter, &lv_font_unscii_8, 0);
    lv_obj_set_style_text_color(s_screensaver_counter, dim, 0);
    lv_obj_set_pos(s_screensaver_counter, 354, 516);

    for (uint8_t i = 0; i < SCREENSAVER_PAGE_COUNT; ++i) {
        s_screensaver_progress[i] = lv_obj_create(scr_screensaver);
        lv_obj_set_size(s_screensaver_progress[i], 42, 3);
        lv_obj_set_pos(s_screensaver_progress[i], 354 + i * 54, 548);
        lv_obj_set_style_radius(s_screensaver_progress[i], 0, 0);
        lv_obj_set_style_border_width(s_screensaver_progress[i], 0, 0);
        lv_obj_clear_flag(s_screensaver_progress[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(s_screensaver_progress[i], LV_OBJ_FLAG_CLICKABLE);
    }
    screensaver_show_page(0, false);
}

static void screensaver_tick(void) {
    // Nunca sobre el boot: tiene su propio flujo de arranque.
    if (active_screen == 0) return;

    // Preferencia del usuario (STATUS > SALVAPANTALLAS). Si lo desactiva
    // mientras esta activo, se restaura la pantalla anterior al instante.
    if (!p4.screensaver_enabled) {
        if (s_screensaver_active) {
            s_screensaver_active = false;
            ui_navigate_to(s_screensaver_return);
        }
        return;
    }

    const uint32_t now = millis();
    uint32_t inact = millis() - lvgl_port_last_touch_ms();
    if (!s_screensaver_active) {
        if (inact >= SCREENSAVER_TIMEOUT_MS) {
            if (!scr_screensaver) create_screensaver_screen();
            if (scr_screensaver) {
                s_screensaver_return = active_screen;
                s_screensaver_page_started_ms = now;
                screensaver_show_page(0, true);
                lv_scr_load_anim(scr_screensaver, LV_SCR_LOAD_ANIM_FADE_ON,
                                 400, 0, false);
                s_screensaver_active = true;
            }
        }
    } else {
        if (inact < SCREENSAVER_TIMEOUT_MS) {
            // Un toque reinició el contador → volver a la pantalla previa.
            s_screensaver_active = false;
            ui_navigate_to(s_screensaver_return);
        } else if (static_cast<uint32_t>(now - s_screensaver_page_started_ms)
                   >= SCREENSAVER_PAGE_MS) {
            s_screensaver_page_started_ms = now;
            screensaver_show_page(
                static_cast<uint8_t>((s_screensaver_page + 1u)
                                     % SCREENSAVER_PAGE_COUNT),
                true);
        }
    }
}

void ui_update_current_screen(void) {
    unsigned long now = millis();
    static unsigned long boot_enter_ms = 0;
    ui_toast_update();

    // Apply state latched by local controller workers on Core 1 —
    // LVGL objects must only be touched from this task.
    ui_live_consume_sync_p4();
    pad_inst_consume_engine_sync();
    pod_process_physical_actions();
    pod_owner_badges_update();

    // Widget refreshes requested by the control task (Core 1) — see the
    // ui_request_* setters near the top of this file. Resync first: it
    // already includes the step-dot pass, so a redundant dots flag from
    // the same control tick collapses into one walk.
    if (s_ui_seq_resync_pending.exchange(false, std::memory_order_acquire)) {
        s_ui_step_dots_pending.store(false, std::memory_order_release);
        ui_sequencer_sync_from_current_pattern();
    }
    if (s_ui_step_dots_pending.exchange(false, std::memory_order_acquire))
        ui_sequencer_refresh_all_step_dots();
    if (s_ui_fx_random_tick_pending.exchange(false, std::memory_order_acquire))
        fx_random_apply(control_auto_toast_enabled());
    if (s_ui_mix_random_tick_pending.exchange(false, std::memory_order_acquire))
        mix_random_apply(control_auto_toast_enabled());
    if (s_ui_matrix_tick_pending.exchange(false, std::memory_order_acquire))
        matrix_apply_column(s_ui_matrix_tick_idx.load(std::memory_order_relaxed));
    if (s_ui_random_song_toast_pending.exchange(false, std::memory_order_acquire))
        ui_show_toast(s_ui_random_song_toast_msg, RED808_CYAN);
    if (s_ui_variation_toast_pending.exchange(false, std::memory_order_acquire))
        ui_show_toast(s_ui_variation_toast_msg, RED808_CYAN);

    // Melody state published by the local controller. Snapshot
    // before clearing pending so a concurrent re-latch is never half-read.
    {
        uint8_t eng = 3, oct = 4, pad = 0;
        bool rec = false;
        if (p4_consume_pending_melody(&eng, &oct, &rec, &pad)) {
            piano_apply_melody_sync(eng, oct, rec, pad);
        }
    }

    // Debounced XTRA param persistence (see xtra_save_param_state).
    xtra_param_save_tick();
    xtra_audio_tick();

    // Results from the async SD upload / Daisy unload workers.
    sd_midi_load_consume_result();
    const bool factoryKitOwnsUpload = sd_factory_autoload_tick();
    if (!factoryKitOwnsUpload) sd_upload_consume_result();
    pad_inst_unload_consume_result();

    // Boot terminal: every status comes from a real setup result or from the
    // live USB protocol state. Timing only controls presentation; it never
    // turns a failed check into OK.
    if (active_screen == 0) {
        if (boot_enter_ms == 0) boot_enter_ms = now;
        uint32_t elapsed = now - boot_enter_ms;
        const auto& transport = daisyUsb.state();
        const bool localReady = p4boot.display_ready && p4boot.lvgl_ready
            && p4boot.ui_ready && p4boot.patterns_ready && p4boot.dsp_ready
            && p4boot.usb_host_ready;
        const bool protocolReady = transport.engine_responding
            && transport.protocol_version == RED808_PROTOCOL_VERSION;
        const bool scanFinished = protocolReady || elapsed >= 5000u;
        int progress = p4boot.setup_complete ? 88 : 8;
        if (transport.link_ready) progress = 94;
        if (scanFinished) progress = 100;
        if (s_boot_progress) lv_bar_set_value(s_boot_progress, progress, LV_ANIM_ON);

        // Cada línea aparece a su tiempo, como un POST de BIOS.
        static const uint32_t revealMs[BOOT_TERM_LINES] =
            { 100, 350, 600, 850, 1100, 1500, 1900, 2300 };
        int lastVisible = -1;
        for (int i = 0; i < BOOT_TERM_LINES; i++) {
            if (!s_boot_term[i]) continue;
            if (elapsed >= revealMs[i]) {
                lv_obj_clear_flag(s_boot_term[i], LV_OBJ_FLAG_HIDDEN);
                lastVisible = i;
            }
        }
        static uint32_t lastBootPaintMs = 0;
        if (lastBootPaintMs == 0 || now - lastBootPaintMs >= 100u) {
            lastBootPaintMs = now;
            auto setBootLine = [](int line, lv_color_t color,
                                  const char* text) {
                if (!s_boot_term[line]) return;
                lv_label_set_text(s_boot_term[line], text);
                lv_obj_set_style_text_color(s_boot_term[line], color, 0);
            };
            char line[96];
            setBootLine(0, p4boot.display_ready ? boot_phosphor() : RED808_ERROR,
                p4boot.display_ready
                    ? "> GFX  MIPI DISPLAY 1024x600 ........ OK"
                    : "> GFX  MIPI DISPLAY ................. ERROR");
            setBootLine(1,
                (p4boot.lvgl_ready && p4boot.ui_ready)
                    ? boot_phosphor() : RED808_ERROR,
                (p4boot.lvgl_ready && p4boot.ui_ready)
                    ? "> UI   LVGL + GT911 TASKS ........... OK"
                    : "> UI   LVGL / TOUCH TASKS ........... ERROR");

            const uint32_t psramTotalMb
                = (p4boot.psram_total_bytes + 524288u) / 1048576u;
            const uint32_t psramFreeMb
                = (p4boot.psram_free_bytes + 524288u) / 1048576u;
            snprintf(line, sizeof(line),
                "> MEM  PSRAM %luMB / FREE %luMB ....... %s",
                static_cast<unsigned long>(psramTotalMb),
                static_cast<unsigned long>(psramFreeMb),
                p4boot.psram_total_bytes > 0 ? "OK" : "WARN");
            setBootLine(2, p4boot.psram_total_bytes > 0
                ? boot_phosphor() : RED808_WARNING, line);

            if (p4boot.spiffs_mounted) {
                snprintf(line, sizeof(line),
                    "> FS   SPIFFS %lu/%luKB ............... OK",
                    static_cast<unsigned long>(p4boot.spiffs_used_bytes / 1024u),
                    static_cast<unsigned long>(p4boot.spiffs_total_bytes / 1024u));
                setBootLine(3, boot_phosphor(), line);
            } else {
                setBootLine(3, RED808_WARNING,
                    "> FS   SPIFFS NOT MOUNTED ............ WARN");
            }

            snprintf(line, sizeof(line),
                "> SEQ  FACTORY PATTERNS %u/%u ........ %s",
                static_cast<unsigned>(p4boot.factory_patterns_found),
                static_cast<unsigned>(p4boot.factory_patterns_expected),
                p4boot.patterns_ready ? "OK" : "ERROR");
            setBootLine(4, p4boot.patterns_ready
                ? boot_phosphor() : RED808_ERROR, line);
            setBootLine(5, p4boot.dsp_ready ? boot_phosphor() : RED808_ERROR,
                p4boot.dsp_ready
                    ? "> DSP  SPECTRUM WORKER TASK .......... OK"
                    : "> DSP  SPECTRUM WORKER TASK .......... ERROR");
            setBootLine(6,
                p4boot.usb_host_ready ? boot_phosphor() : RED808_ERROR,
                p4boot.usb_host_ready
                    ? "> USB  HOST + CDC DRIVER ............. OK"
                    : "> USB  HOST / CDC DRIVER ............. ERROR");

            uint8_t loadedSamples = 0;
            for (uint8_t pad = 0; pad < 16; ++pad)
                if (transport.sample_mask & (1u << pad)) loadedSamples++;
            if (!p4boot.usb_host_ready) {
                setBootLine(7, RED808_ERROR,
                    "> LNK  DAISYPOD3 UNAVAILABLE ......... ERROR");
            } else if (!transport.link_ready) {
                setBootLine(7, RED808_WARNING,
                    "> LNK  DAISYPOD3 USB DEVICE .......... WAIT");
            } else if (!protocolReady) {
                if (transport.protocol_version == 0) {
                    setBootLine(7, RED808_CYAN,
                        "> LNK  USB ENUMERATED / PING ......... SYNC");
                } else {
                    snprintf(line, sizeof(line),
                        "> LNK  PROTOCOL %u.%u (NEED 2.3) ..... ERROR",
                        static_cast<unsigned>(transport.protocol_version >> 8),
                        static_cast<unsigned>(transport.protocol_version & 0xFFu));
                    setBootLine(7, RED808_ERROR, line);
                }
            } else {
                snprintf(line, sizeof(line),
                    "> LNK  DAISY P2.3 / WAV %u/16 ......... %s",
                    static_cast<unsigned>(loadedSamples),
                    loadedSamples > 0 ? "OK" : "WAIT");
                setBootLine(7, loadedSamples > 0
                    ? boot_phosphor() : RED808_WARNING, line);
            }
        }
        // Cursor de bloque parpadeante bajo la última línea visible
        if (s_boot_cursor) {
            int row = lastVisible + 1;
            lv_obj_align(s_boot_cursor, LV_ALIGN_TOP_LEFT, 44, 100 + row * 34);
            bool on = ((now / 400U) & 1U) != 0U;
            lv_obj_set_style_opa(s_boot_cursor, on ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
        }

        // Reveal CONTINUE only when the local POST has been read and either
        // Daisy negotiated the exact protocol or the bounded device scan ended.
        static bool readyShown = false;
        if ((protocolReady || scanFinished || !localReady) && !readyShown
            && elapsed >= revealMs[BOOT_TERM_LINES - 1] + 200) {
            readyShown = true;
            if (s_boot_continue_btn)
                lv_obj_clear_flag(s_boot_continue_btn, LV_OBJ_FLAG_HIDDEN);
        }
        if (s_boot_status_lbl) {
            if (!localReady) {
                lv_label_set_text(s_boot_status_lbl,
                    "LOCAL INIT ERROR - REVIEW THE FAILED CHECK");
                lv_obj_set_style_text_color(s_boot_status_lbl, RED808_ERROR, 0);
            } else if (protocolReady) {
                lv_label_set_text(s_boot_status_lbl,
                    "P4 + DAISYPOD3 READY - PRESS CONTINUE");
                lv_obj_set_style_text_color(s_boot_status_lbl, boot_phosphor(), 0);
            } else if (scanFinished) {
                lv_label_set_text(s_boot_status_lbl,
                    "P4 READY / DAISYPOD3 OFFLINE - PRESS CONTINUE");
                lv_obj_set_style_text_color(s_boot_status_lbl, RED808_WARNING, 0);
            } else {
                lv_label_set_text(s_boot_status_lbl,
                    "LOCAL INIT OK - NEGOTIATING DAISYPOD3 ...");
                lv_obj_set_style_text_color(s_boot_status_lbl, boot_phosphor_dim(), 0);
            }
        }
        // Parpadeo suave del borde del botón mientras espera al operador
        if (readyShown && s_boot_continue_btn) {
            bool blink = ((now / 500U) & 1U) != 0U;
            lv_obj_set_style_border_opa(s_boot_continue_btn,
                blink ? LV_OPA_COVER : LV_OPA_40, 0);
        }
    } else {
        boot_enter_ms = 0;
    }

    // Theme change — recreate all screens with new palette
    static int prev_theme = -1;
    if (p4.theme != prev_theme && prev_theme != -1) {
        P4_THEME_LOG_PRINTF("[THEME] update detected prev=%d p4=%d current=%u\n",
                            prev_theme, p4.theme,
                            static_cast<unsigned>(ui_theme_index()));
        prev_theme = p4.theme;
        ui_theme_apply((VisualTheme)p4.theme);
        ui_reload_themed_screens();
        return;  // screens recreated; update functions have fresh state
    }
    if (prev_theme == -1) prev_theme = p4.theme;

    // Apply a locally requested screen change.
    static int prev_screen = -1;
    if (p4.current_screen != prev_screen) {
        int requested = p4.current_screen;
        prev_screen = requested;
        // Ignore remote BOOT requests once UI has left boot screen.
        // A stale local request may transiently report SCREEN_BOOT.
        if (!(requested == 0 && active_screen != 0)) {
            ui_navigate_to(requested);
        }
    }

    // Status screensaver: show after 5 minutes idle, dismiss on next touch.
    screensaver_tick();

    ui_update_header();
    // STATUS is a global overlay and may be opened from any screen. Keep its
    // hardware values live regardless of the active per-screen updater.
    pod_status_modal_update();
    // MIDI MAP overlay: LEARN capture handoff + pad/knob glow on MIDI input.
    mpd_map_modal_update();

    // Force fx_screen repaint immediately after a local/USB FX change.
    // Must be BEFORE the period throttle so dirty updates aren't delayed up to 33ms.
    // Only when the FX screen is actually visible — the prev_key caches catch
    // up on the next scheduled update after entering the screen.
    if (control_consume_fx_screen_dirty()) {
        if (lv_scr_act() == scr_fx) update_fx_screen();
    }

    if (lv_scr_act() == scr_sdcard && sd_source == 2
        && daisyUsb.state().daisy_sd_revision != sd_daisy_seen_revision) {
        p4sd.needs_refresh.store(true, std::memory_order_release);
    }

    // Per-screen pacing. LIVE and STEPS need 60Hz for pad fades/playhead.
    // Static editors do not: most interaction is handled by event callbacks.
    static unsigned long last_active_update_ms = 0;
    uint32_t period_ms = 33;
    lv_obj_t* active = lv_scr_act();
    if (active == scr_live) period_ms = 16;
    else if (active == scr_sequencer) period_ms = 16;
    else if (active == scr_sdcard) period_ms = p4sd.needs_refresh.load(std::memory_order_acquire) ? 16 : 100;
    else if (active == scr_piano) period_ms = 16;
    else if (active == scr_piano_params) period_ms = 50;
    else if (active == scr_fx) period_ms = 16;
    if (now - last_active_update_ms < period_ms) return;
    last_active_update_ms = now;

    // Update active screen content
    if (active == scr_live) update_live_screen();
    else if (active == scr_sequencer) update_sequencer_screen();
    else if (active == scr_fx) update_fx_screen();
    else if (active == scr_volumes) update_volumes_screen();
    else if (active == scr_piano || active == scr_piano_params) update_piano_screen();
    else if (active == scr_sdcard &&
             p4sd.needs_refresh.exchange(false, std::memory_order_acq_rel)) {
        sd_refresh_ui();
    }
    else if (active == scr_performance) xtra_pages_poll();
}
