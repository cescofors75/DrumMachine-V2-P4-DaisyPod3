// =============================================================================
// coach_engine.cpp — RED808 Drum Finger Coach (V0.1 demo)
// See coach_engine.h for the public contract and threading notes.
// =============================================================================
#include "coach_engine.h"
#include "../control_api.h"

#include <Arduino.h>
#include <stdio.h>
#include <string.h>

namespace coach {

namespace {

// Track indices match the canonical trackNames[] order used across the UI
// (ui_screens.cpp): BD, SD, CH, OH, CY, CP, RS, CB, ...
enum DrumTrack : uint8_t {
    TRACK_BD = 0,
    TRACK_SD = 1,
    TRACK_CH = 2,
    TRACK_OH = 3,
    TRACK_RS = 6,  // reused as the metronome click voice — not part of any
                   // lesson pattern, so it never collides with a target hit.
};

constexpr uint32_t TIMING_OK_MS = 30;
constexpr uint32_t VELOCITY_OK = 25;
constexpr uint8_t DEFAULT_DEMO_VELOCITY = 100;
constexpr uint8_t CLICK_VELOCITY = 90;
constexpr uint8_t CLICK_ACCENT_VELOCITY = 120;
constexpr int DRILL_REPS_NEEDED = 2;
constexpr int AUTO_BPM_STREAK = 2;
constexpr float AUTO_BPM_STEP = 4.0f;

struct LessonHit {
    uint8_t step;
    uint8_t track;
    uint8_t velocity;  // 0 = not graded (level doesn't check velocity yet)
};

struct Level {
    const char* name;
    const char* focus;
    uint8_t step_count;
    uint8_t subdivision;  // steps per beat: 1 = quarter notes, 2 = eighths
    uint8_t hit_count;
    LessonHit hits[kMaxHits];
    float base_bpm;
    float bpm_ceiling;
    bool auto_bpm;
    bool check_velocity;
};

// Base groove: KICK -> HI-HAT -> SNARE -> HI-HAT (the exact demo pattern from
// the spec), extended to a full 8th-note bar with an open hat for level 3+.
const Level kLevels[kLevelCount] = {
    {"NIVEL 1", "KICK + SNARE", 4, 1, 2,
     {{0, TRACK_BD, 0}, {2, TRACK_SD, 0}},
     70.0f, 70.0f, false, false},
    {"NIVEL 2", "ANADE EL HI-HAT", 4, 1, 4,
     {{0, TRACK_BD, 0}, {1, TRACK_CH, 0}, {2, TRACK_SD, 0}, {3, TRACK_CH, 0}},
     72.0f, 72.0f, false, false},
    {"NIVEL 3", "GROOVE COMPLETO", 8, 2, 8,
     {{0, TRACK_BD, 0}, {1, TRACK_CH, 0}, {2, TRACK_SD, 0}, {3, TRACK_CH, 0},
      {4, TRACK_BD, 0}, {5, TRACK_CH, 0}, {6, TRACK_SD, 0}, {7, TRACK_OH, 0}},
     76.0f, 76.0f, false, false},
    {"NIVEL 4", "VELOCITIES", 8, 2, 8,
     {{0, TRACK_BD, 110}, {1, TRACK_CH, 55}, {2, TRACK_SD, 115}, {3, TRACK_CH, 55},
      {4, TRACK_BD, 110}, {5, TRACK_CH, 55}, {6, TRACK_SD, 115}, {7, TRACK_OH, 90}},
     76.0f, 76.0f, false, true},
    {"NIVEL 5", "SUBE EL TEMPO", 8, 2, 8,
     {{0, TRACK_BD, 110}, {1, TRACK_CH, 55}, {2, TRACK_SD, 115}, {3, TRACK_CH, 55},
      {4, TRACK_BD, 110}, {5, TRACK_CH, 55}, {6, TRACK_SD, 115}, {7, TRACK_OH, 90}},
     70.0f, 82.0f, true, true},
};

const char* track_label(uint8_t track) {
    switch (track) {
        case TRACK_BD: return "KICK";
        case TRACK_SD: return "SNARE";
        case TRACK_CH: return "HI-HAT";
        case TRACK_OH: return "HI-HAT ABIERTO";
        default: return "PAD";
    }
}

struct ActiveHit {
    uint8_t step;
    uint8_t track;
    uint8_t velocity;
};

struct CapturedHit {
    uint8_t track;
    uint8_t velocity;
    int32_t t_ms;  // relative to Perform phase start
    bool used;
};

// ── State ──────────────────────────────────────────────────────────────
Screen s_screen = Screen::Home;
Phase s_phase = Phase::Result;
int s_selected_level = 0;
int s_max_unlocked = 0;
int s_best_stars[kLevelCount] = {0, 0, 0, 0, 0};
int s_consecutive_success[kLevelCount] = {0, 0, 0, 0, 0};
float s_bpm = 70.0f;

bool s_drill_active = false;
uint8_t s_drill_tracks[2] = {0, 0};
int s_drill_track_count = 0;
int s_drill_success_count = 0;

ActiveHit s_active_hits[kMaxHits];
int s_active_count = 0;
int s_active_step_count = 0;
int s_active_subdivision = 1;

uint32_t s_phase_start_ms = 0;
int s_last_played_step = -1;  // Listen scheduling cursor
int s_last_click_beat = -1;   // CountIn scheduling cursor

CapturedHit s_captured[kMaxHits];
int s_captured_count = 0;

char s_result_line1[40] = "";
char s_result_line2[40] = "";
bool s_result_success = false;
int s_result_accuracy = 0;
int s_combo = 0;
bool s_last_problem_isolable = false;
uint8_t s_last_problem_track = 0;

float step_duration_ms() {
    return 60000.0f / s_bpm / (float)s_active_subdivision;
}

float beat_duration_ms() { return 60000.0f / s_bpm; }

void safe_trigger(uint8_t track, uint8_t velocity) {
    if (control_available() || control_engine_connected()) {
        control_send_trigger(track, velocity);
    }
}

void refresh_active_pattern() {
    const Level& lvl = kLevels[s_selected_level];
    s_active_step_count = lvl.step_count;
    s_active_subdivision = lvl.subdivision;
    s_active_count = 0;
    for (int i = 0; i < lvl.hit_count && s_active_count < kMaxHits; i++) {
        const LessonHit& h = lvl.hits[i];
        if (s_drill_active) {
            bool keep = false;
            for (int d = 0; d < s_drill_track_count; d++) {
                if (s_drill_tracks[d] == h.track) { keep = true; break; }
            }
            if (!keep) continue;
        }
        s_active_hits[s_active_count++] = {h.step, h.track, h.velocity};
    }
}

void enter_phase(Phase p, uint32_t now_ms) {
    s_phase = p;
    s_phase_start_ms = now_ms;
    s_last_played_step = -1;
    s_last_click_beat = -1;
    if (p == Phase::Perform) s_captured_count = 0;
}

void begin_lesson(int level, uint32_t now_ms) {
    s_selected_level = level;
    s_bpm = kLevels[level].base_bpm;
    s_drill_active = false;
    s_drill_track_count = 0;
    s_drill_success_count = 0;
    refresh_active_pattern();
    s_screen = Screen::Lesson;
    enter_phase(Phase::Listen, now_ms);
}

// Matches captured hits against the active pattern, picks the single most
// important problem to report (missed > wrong pad > timing > velocity >
// extra hits), and records whether the failure is isolable to one voice so
// repeat() can spin up a focused drill.
void finish_perform(uint32_t now_ms) {
    bool matched[kMaxHits] = {false};
    int32_t match_dt[kMaxHits] = {0};
    uint8_t match_track[kMaxHits] = {0};
    uint8_t match_velocity[kMaxHits] = {0};

    const float step_ms = step_duration_ms();
    float window = step_ms * 0.6f;
    if (window < 60.0f) window = 60.0f;
    if (window > 220.0f) window = 220.0f;

    for (int i = 0; i < s_active_count; i++) {
        const float expected = s_active_hits[i].step * step_ms;
        int best_j = -1;
        float best_dt = 1e9f;
        for (int j = 0; j < s_captured_count; j++) {
            if (s_captured[j].used) continue;
            const float dt = (float)s_captured[j].t_ms - expected;
            const float adt = dt < 0 ? -dt : dt;
            if (adt <= window && adt < best_dt) { best_dt = adt; best_j = j; }
        }
        if (best_j >= 0) {
            s_captured[best_j].used = true;
            matched[i] = true;
            match_dt[i] = (int32_t)((float)s_captured[best_j].t_ms - expected);
            match_track[i] = s_captured[best_j].track;
            match_velocity[i] = s_captured[best_j].velocity;
        }
    }

    int missed_count = 0, extra_count = 0, good_count = 0;
    int wrong_pad_idx = -1;
    int worst_timing_idx = -1; uint32_t worst_timing_abs = 0;
    int worst_velocity_idx = -1; int worst_velocity_dev = 0;
    uint8_t touched_tracks[kMaxHits]; int touched_track_count = 0;
    uint8_t problem_tracks[kMaxHits]; int problem_track_count = 0;

    auto mark_touched = [&](uint8_t t) {
        for (int k = 0; k < touched_track_count; k++) if (touched_tracks[k] == t) return;
        touched_tracks[touched_track_count++] = t;
    };
    auto mark_problem = [&](uint8_t t) {
        for (int k = 0; k < problem_track_count; k++) if (problem_tracks[k] == t) return;
        problem_tracks[problem_track_count++] = t;
    };

    for (int i = 0; i < s_active_count; i++) {
        mark_touched(s_active_hits[i].track);
        if (!matched[i]) { missed_count++; mark_problem(s_active_hits[i].track); continue; }
        bool ok = true;
        if (match_track[i] != s_active_hits[i].track) {
            ok = false;
            if (wrong_pad_idx < 0) wrong_pad_idx = i;
            mark_problem(s_active_hits[i].track);
        }
        const uint32_t adt = match_dt[i] < 0 ? (uint32_t)(-match_dt[i]) : (uint32_t)match_dt[i];
        if (adt > TIMING_OK_MS) {
            ok = false;
            if (adt > worst_timing_abs) { worst_timing_abs = adt; worst_timing_idx = i; }
            mark_problem(s_active_hits[i].track);
        }
        if (kLevels[s_selected_level].check_velocity && s_active_hits[i].velocity > 0) {
            const int dev = (int)match_velocity[i] - (int)s_active_hits[i].velocity;
            const int adev = dev < 0 ? -dev : dev;
            if (adev > (int)VELOCITY_OK) {
                ok = false;
                if (adev > worst_velocity_dev) { worst_velocity_dev = adev; worst_velocity_idx = i; }
                mark_problem(s_active_hits[i].track);
            }
        }
        if (ok) good_count++;
    }
    for (int j = 0; j < s_captured_count; j++) if (!s_captured[j].used) extra_count++;

    s_result_accuracy = s_active_count > 0 ? (good_count * 100) / s_active_count : 100;
    const bool success = (missed_count == 0 && extra_count == 0 && wrong_pad_idx < 0 &&
                          worst_timing_idx < 0 && worst_velocity_idx < 0);
    s_result_success = success;
    s_last_problem_isolable = (!s_drill_active && problem_track_count == 1 && touched_track_count > 1);
    if (s_last_problem_isolable) s_last_problem_track = problem_tracks[0];

    if (success) {
        snprintf(s_result_line1, sizeof(s_result_line1), "%s",
                 s_result_accuracy >= 95 ? "PERFECTO" : "MUY BIEN");
        s_result_line2[0] = '\0';
        s_combo++;
    } else {
        s_combo = 0;
        if (missed_count > 0) {
            int i = 0;
            while (i < s_active_count && matched[i]) i++;
            snprintf(s_result_line1, sizeof(s_result_line1), "%s OMITIDO",
                     track_label(s_active_hits[i].track));
            snprintf(s_result_line2, sizeof(s_result_line2), "REPETIMOS ESTA PARTE");
        } else if (wrong_pad_idx >= 0) {
            snprintf(s_result_line1, sizeof(s_result_line1), "%s EQUIVOCADO",
                     track_label(s_active_hits[wrong_pad_idx].track));
            snprintf(s_result_line2, sizeof(s_result_line2), "REPETIMOS ESTA PARTE");
        } else if (worst_timing_idx >= 0) {
            const char* dir = match_dt[worst_timing_idx] > 0 ? "TARDE" : "PRONTO";
            snprintf(s_result_line1, sizeof(s_result_line1), "%s %s %+d ms",
                     track_label(s_active_hits[worst_timing_idx].track), dir,
                     (int)match_dt[worst_timing_idx]);
            snprintf(s_result_line2, sizeof(s_result_line2), "REPETIMOS ESTA PARTE");
        } else if (worst_velocity_idx >= 0) {
            const int dev = (int)match_velocity[worst_velocity_idx] -
                            (int)s_active_hits[worst_velocity_idx].velocity;
            snprintf(s_result_line1, sizeof(s_result_line1), "%s DEMASIADO %s",
                     track_label(s_active_hits[worst_velocity_idx].track),
                     dev > 0 ? "FUERTE" : "SUAVE");
            snprintf(s_result_line2, sizeof(s_result_line2), "PRUEBA MAS %s",
                     dev > 0 ? "SUAVE" : "FUERTE");
        } else {
            snprintf(s_result_line1, sizeof(s_result_line1), "GOLPE DE MAS");
            snprintf(s_result_line2, sizeof(s_result_line2), "ESCUCHA CON ATENCION");
        }
    }

    enter_phase(Phase::Result, now_ms);
}

}  // namespace

void init() {
    s_screen = Screen::Home;
    s_phase = Phase::Result;
    s_selected_level = 0;
    s_max_unlocked = 0;
    for (int i = 0; i < kLevelCount; i++) {
        s_best_stars[i] = 0;
        s_consecutive_success[i] = 0;
    }
    s_bpm = kLevels[0].base_bpm;
    s_drill_active = false;
    s_drill_track_count = 0;
    s_combo = 0;
}

void tick(uint32_t now_ms) {
    if (s_screen != Screen::Lesson) return;
    const uint32_t elapsed = now_ms - s_phase_start_ms;

    switch (s_phase) {
        case Phase::Listen: {
            const float step_ms = step_duration_ms();
            const int step = (int)((float)elapsed / step_ms);
            if (step != s_last_played_step && step < s_active_step_count) {
                s_last_played_step = step;
                for (int i = 0; i < s_active_count; i++) {
                    if (s_active_hits[i].step == step) {
                        const uint8_t vel = s_active_hits[i].velocity > 0
                                                 ? s_active_hits[i].velocity
                                                 : DEFAULT_DEMO_VELOCITY;
                        safe_trigger(s_active_hits[i].track, vel);
                    }
                }
            }
            if (elapsed >= (uint32_t)(s_active_step_count * step_ms)) enter_phase(Phase::CountIn, now_ms);
            break;
        }
        case Phase::CountIn: {
            const float beat_ms = beat_duration_ms();
            const int beat = (int)((float)elapsed / beat_ms);
            if (beat != s_last_click_beat && beat < 4) {
                s_last_click_beat = beat;
                safe_trigger(TRACK_RS, beat == 0 ? CLICK_ACCENT_VELOCITY : CLICK_VELOCITY);
            }
            if (elapsed >= (uint32_t)(4 * beat_ms)) enter_phase(Phase::Perform, now_ms);
            break;
        }
        case Phase::Perform: {
            const float step_ms = step_duration_ms();
            const uint32_t duration = (uint32_t)(s_active_step_count * step_ms + step_ms * 0.6f);
            if (elapsed >= duration) finish_perform(now_ms);
            break;
        }
        case Phase::Result:
            break;
    }
}

void on_pad_hit(uint8_t track, uint8_t velocity, uint32_t now_ms) {
    if (s_screen != Screen::Lesson || s_phase != Phase::Perform) return;
    if (s_captured_count >= kMaxHits) return;
    s_captured[s_captured_count++] = {track, velocity, (int32_t)(now_ms - s_phase_start_ms), false};
}

void open_home() { s_screen = Screen::Home; }
void open_level_select() { s_screen = Screen::LevelSelect; }

void start_level(int level) {
    if (level < 0 || level >= kLevelCount) return;
    begin_lesson(level, millis());
}

void repeat() {
    if (s_screen != Screen::Lesson || s_phase != Phase::Result) return;
    if (!s_result_success) {
        s_consecutive_success[s_selected_level] = 0;
        const Level& lvl = kLevels[s_selected_level];
        if (lvl.auto_bpm && s_result_accuracy < 50 && s_bpm > lvl.base_bpm) {
            s_bpm -= AUTO_BPM_STEP;
            if (s_bpm < lvl.base_bpm) s_bpm = lvl.base_bpm;
        }
        if (!s_drill_active && s_last_problem_isolable) {
            // Isolate the one struggling voice plus an anchor track (kick,
            // or snare if kick itself was the problem) — practice just that
            // pair for a couple of clean reps before returning to the groove.
            s_drill_active = true;
            s_drill_track_count = 0;
            s_drill_tracks[s_drill_track_count++] = s_last_problem_track;
            s_drill_tracks[s_drill_track_count++] =
                (s_last_problem_track == TRACK_BD) ? TRACK_SD : TRACK_BD;
            s_drill_success_count = 0;
            refresh_active_pattern();
        }
    }
    enter_phase(Phase::CountIn, millis());
}

void advance() {
    if (s_screen != Screen::Lesson || s_phase != Phase::Result || !s_result_success) return;

    if (s_drill_active) {
        s_drill_success_count++;
        if (s_drill_success_count >= DRILL_REPS_NEEDED) {
            s_drill_active = false;
            s_drill_track_count = 0;
            refresh_active_pattern();
            enter_phase(Phase::Listen, millis());
        } else {
            enter_phase(Phase::CountIn, millis());
        }
        return;
    }

    const Level& lvl = kLevels[s_selected_level];
    const int stars = s_result_accuracy >= 95 ? 3 : (s_result_accuracy >= 80 ? 2 : 1);
    if (stars > s_best_stars[s_selected_level]) s_best_stars[s_selected_level] = stars;

    bool bumped = false;
    if (lvl.auto_bpm) {
        s_consecutive_success[s_selected_level]++;
        if (s_consecutive_success[s_selected_level] >= AUTO_BPM_STREAK && s_bpm < lvl.bpm_ceiling) {
            s_bpm += AUTO_BPM_STEP;
            if (s_bpm > lvl.bpm_ceiling) s_bpm = lvl.bpm_ceiling;
            s_consecutive_success[s_selected_level] = 0;
            bumped = true;
        }
    }

    // Auto-BPM levels keep looping the same groove, faster each streak,
    // until they land a clean rep at the ceiling BPM without a fresh bump.
    const bool level_complete = !lvl.auto_bpm || (s_bpm >= lvl.bpm_ceiling && !bumped);
    if (!level_complete) {
        enter_phase(Phase::CountIn, millis());
        return;
    }

    if (s_selected_level == s_max_unlocked && s_max_unlocked < kLevelCount - 1) s_max_unlocked++;
    if (s_selected_level < kLevelCount - 1) {
        begin_lesson(s_selected_level + 1, millis());
    } else {
        s_screen = Screen::LevelSelect;
    }
}

Screen screen() { return s_screen; }
Phase phase() { return s_phase; }
int current_level() { return s_selected_level; }

const char* level_name(int level) {
    return (level >= 0 && level < kLevelCount) ? kLevels[level].name : "";
}
const char* level_focus(int level) {
    return (level >= 0 && level < kLevelCount) ? kLevels[level].focus : "";
}
int level_best_stars(int level) {
    return (level >= 0 && level < kLevelCount) ? s_best_stars[level] : 0;
}
bool level_unlocked(int level) { return level >= 0 && level <= s_max_unlocked; }

int pattern_step_count() { return s_active_step_count; }
int pattern_subdivision() { return s_active_subdivision; }

bool pattern_step_hit(int step, uint8_t* track_out) {
    for (int i = 0; i < s_active_count; i++) {
        if (s_active_hits[i].step == step) {
            if (track_out) *track_out = s_active_hits[i].track;
            return true;
        }
    }
    return false;
}

float pattern_bpm() { return s_bpm; }
bool is_drill() { return s_drill_active; }

int playhead_step() {
    if (s_screen != Screen::Lesson) return -1;
    if (s_phase != Phase::Listen && s_phase != Phase::Perform) return -1;
    const uint32_t elapsed = millis() - s_phase_start_ms;
    const int step = (int)((float)elapsed / step_duration_ms());
    return (step >= 0 && step < s_active_step_count) ? step : -1;
}

int count_in_beat() {
    if (s_screen != Screen::Lesson || s_phase != Phase::CountIn) return -1;
    const uint32_t elapsed = millis() - s_phase_start_ms;
    const int beat = (int)((float)elapsed / beat_duration_ms());
    return (beat >= 0 && beat <= 3) ? beat + 1 : -1;
}

const char* result_line1() { return s_result_line1; }
const char* result_line2() { return s_result_line2; }
bool result_is_success() { return s_result_success; }
int accuracy_percent() { return s_result_accuracy; }
int combo() { return s_combo; }

}  // namespace coach
