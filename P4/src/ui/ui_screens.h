// =============================================================================
// ui_screens.h — P4 screen declarations
// =============================================================================
#pragma once

#include <lvgl.h>

// Screen objects
extern lv_obj_t* scr_boot;
extern lv_obj_t* scr_live;
extern lv_obj_t* scr_sequencer;
extern lv_obj_t* scr_fx;
extern lv_obj_t* scr_volumes;
extern lv_obj_t* scr_sdcard;
extern lv_obj_t* scr_performance;
extern lv_obj_t* scr_piano;     // v2.6 — PIANO live keyboard
extern lv_obj_t* scr_piano_params; // v2.7 — synth engine parameter editor

// Create all screens (call once after LVGL init)
void ui_create_all_screens(void);

// Update current screen from P4State (call every frame)
void ui_update_current_screen(void);

// Drain pad event queue — call from loop() on Core 1 (outside LVGL mutex)
void ui_process_pad_queue(void);

// Drain deferred mute/solo commands — call from loop() outside LVGL mutex.
void ui_process_control_queue(void);

// Map absolute (x,y) touch coordinate to pad index 0..15 on the LIVE screen.
// Returns -1 if not on a pad or the LIVE screen is not active.
int ui_pad_from_xy(uint16_t x, uint16_t y,
				   uint8_t* cell_x = nullptr, uint8_t* cell_y = nullptr);

// Per-frame touch state update — call from GT911 touch_task (Core 0, 200Hz).
// `pressed[p]` is true while any finger currently sits on pad p. `velocity[p]`
// is the instantaneous MIDI velocity (40..127) derived from the GT911 area.
// `cell_x/cell_y` are local pad coordinates scaled 0..127 for expressive hold.
// Handles rising/falling edges, hold tremolo, note-repeat and 16-levels routing.
void ui_pad_frame_update(const bool pressed[16], const uint8_t velocity[16],
						 const uint8_t cell_x[16], const uint8_t cell_y[16]);

// v2.9 — Apply melody_sync payload from master to P4 piano UI.
// Must be called from within lvgl_port_lock.
void piano_apply_melody_sync(uint8_t engine, uint8_t octave, bool rec, uint8_t pad);

// Navigate to a screen
void ui_navigate_to(int screen_id);

// Helper: create styled section shell
lv_obj_t* create_section_shell(lv_obj_t* parent, int x, int y, int w, int h);

// Header bar (shared across screens)
// Returns the floating back button, in case a caller wants to attach
// something to it (e.g. a small corner badge) without new layout space.
lv_obj_t* ui_create_header(lv_obj_t* parent);
void ui_update_header(void);

// Reset the sequencer's temporary multi-bar import state so the current
// 16-step pattern from Master becomes the authoritative view again.
void ui_sequencer_sync_from_current_pattern(void);
// Called when DaisyPod3 commits a queued pattern at step 0.
void ui_pattern_queue_committed(int pattern);

// Install a full external pattern (up to 64 steps) received from Master and
// refresh the sequencer pagination/view around its first page.
void ui_sequencer_load_external_pattern(const bool steps[16][64], int raw_len);

// Sync pads state. Safe to call from any task: the value is latched and
// applied to the LVGL widgets from the LVGL task.
void ui_live_set_sync_p4(bool on);

// Flash a LIVE pad from outside the UI (e.g. a mapped AKAI MPD218 hit
// forwarded by DaisyPod3). Safe to call from any task.
void ui_external_pad_flash(uint8_t pad, uint8_t velocity);

// Apply authoritative per-track synth engine state (track 0..15).
// Engine mapping matches setTrackSynthEngine: -1 sampler, 0..6 synth engines.
// Safe to call from any task: the payload is latched and applied to the
// LVGL widgets from the LVGL task.
void ui_pad_sound_sync_track_engines(const int8_t engines[16]);

// Applies one random FX LAB filter/cutoff/reso/drive/bits/srate pass and one
// random MIXER rebalance, respectively — the same logic the manual RANDOM
// buttons run on tap. LVGL task only (they touch widgets and start
// animations); control code uses the ui_request_* setters below instead.
// showToast is false for AUTO's periodic re-randomization so it does not
// nag with a toast every few bars during a live set; manual taps pass true.
void fx_random_apply(bool showToast = true);
void mix_random_apply(bool showToast = true);

// Refreshes every step's corner "customized" dot (probability<100% or a
// parameter lock) on the sequencer's step grid from the Sequencer's stored
// state. Safe to call even when the sequencer screen was never built (the
// dot pointers are all NULL then, and each refresh no-ops). LVGL task only.
void ui_sequencer_refresh_all_step_dots(void);

// ── Deferred UI requests for the control task (Core 1) ──
// LVGL is single-task: control_process() must never call the widget-touching
// functions above directly (it races lv_timer_handler() on Core 0). These
// setters only flip an atomic; ui_update_current_screen() performs the real
// work on the LVGL task within one frame (~16 ms). Safe from any task.
void ui_request_sequencer_resync(void);   // -> ui_sequencer_sync_from_current_pattern()
void ui_request_step_dots_refresh(void);  // -> ui_sequencer_refresh_all_step_dots()
void ui_request_fx_random_tick(void);     // -> fx_random_apply(false)
void ui_request_mix_random_tick(void);    // -> mix_random_apply(false)
void ui_request_matrix_tick(uint8_t idx); // -> matrix_apply_column(idx)

// Shows a toast with the reason RANDOM SONG picked its last jump target
// (see triggerRandomSongJump in control_api.cpp) — e.g. "SONG -> P034
// Techno III. Acid (siguiente escena)". msg is copied into a fixed-size
// buffer; safe from any task.
void ui_request_random_song_toast(const char* msg);

// Same idea for AUTO VARIATIONS: shows which named variation the last
// automatic pass applied. msg copied into a fixed-size buffer; safe from
// any task.
void ui_request_variation_toast(const char* msg);

// Display name for a SEQ_VAR_* id (e.g. "GHOST GROOVE"), or "VARIACION"
// if unknown. Backs both the manual VAR popup and the AUTO VARIATIONS
// toast so the two paths never drift apart.
const char* ui_variation_name(uint8_t id);
