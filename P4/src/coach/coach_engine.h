// =============================================================================
// coach_engine.h — RED808 Drum Finger Coach (V0.1 demo)
//
// Deterministic teaching engine: ESCUCHA -> MIRA -> TOCA -> ANALIZA -> CORRIGE
// -> REPITE -> PROGRESA. No AI, no dependency on LVGL or the main step
// sequencer/pattern bank — this owns a tiny 5-lesson curriculum and drives
// playback/click directly through control_send_trigger().
//
// Threading: init()/start_level()/repeat()/advance()/open_*() are called
// from the LVGL task (Core 0, holds lvgl_port_lock). tick() is called once
// per loop() iteration (Core 1). on_pad_hit() is called from control_api's
// MIDI monitor drain (Core 1) and from the LIVE pad queue (Core 1) — never
// from an ISR. All state here is plain (non-atomic): every caller runs on
// Core 1 except the UI setters, which only ever read state, so there is no
// concurrent read/write across cores in practice on this build.
// =============================================================================
#pragma once

#include <stdint.h>

namespace coach {

constexpr int kLevelCount = 5;
constexpr int kMaxPatternSteps = 8;
constexpr int kMaxHits = 8;

enum class Screen : uint8_t { Home, LevelSelect, Lesson };

enum class Phase : uint8_t {
    Listen,   // teacher plays the target pattern once
    CountIn,  // "1 - 2 - 3 - 4" click before the student plays
    Perform,  // "AHORA TU" — capturing the student's hits
    Result    // feedback shown; waiting for OTRA VEZ / SIGUIENTE
};

void init();

// Call once per loop() iteration with millis(). Drives playback/count-in/
// perform-window scheduling and the click.
void tick(uint32_t now_ms);

// Feed a resolved drum-pad hit (track 0-15, MIDI velocity 0-127). Ignored
// unless a lesson is currently in Phase::Perform. Safe to call unconditionally
// from every pad-hit source (MIDI-in, on-screen LIVE pads).
void on_pad_hit(uint8_t track, uint8_t velocity, uint32_t now_ms);

// ── Navigation ───────────────────────────────────────────────────────────
void open_home();
void open_level_select();
void start_level(int level);  // begins Listen -> CountIn -> Perform
void repeat();                 // "OTRA VEZ" — re-run CountIn -> Perform
void advance();                // "SIGUIENTE" — next drill rep or next level

Screen screen();
Phase phase();
int current_level();

// ── Level metadata ──────────────────────────────────────────────────────
const char* level_name(int level);
const char* level_focus(int level);   // one-line subtitle ("KICK + SNARE", ...)
int level_best_stars(int level);      // 0..3, best result so far this session
bool level_unlocked(int level);

// ── Pattern currently being demonstrated/practiced (drill subset if active) ─
int pattern_step_count();
int pattern_subdivision();  // steps per beat: 1 = quarter notes, 2 = eighths
bool pattern_step_hit(int step, uint8_t* track_out);  // true if step has a hit
float pattern_bpm();
bool is_drill();

// ── Live progress, for animating the pad/step grid ──────────────────────
int playhead_step();  // -1 idle; else step index currently sounding/expected
int count_in_beat();  // 1..4 during Phase::CountIn, else -1

// ── Result (valid once Phase::Result is entered) ────────────────────────
const char* result_line1();
const char* result_line2();
bool result_is_success();
int accuracy_percent();
int combo();

}  // namespace coach
