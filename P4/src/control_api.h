#pragma once

#include <stddef.h>
#include <stdint.h>
#include "master/protocol.h"

namespace mem_midi { struct MidiSongData; }

// Event captured by MIDI LEARN (first note-on or CC after arming).
struct MidiLearnCapture
{
    uint8_t channel; // 0-15
    uint8_t kind;    // MIDI_MAP_KIND_NOTE / MIDI_MAP_KIND_CC
    uint8_t number;  // note or CC number
    uint8_t value;   // velocity / CC value at capture time
};

void control_init();
void control_process();
bool control_available();
bool control_engine_connected();
int control_current_step_raw();

void control_send_trigger(uint8_t pad, uint8_t velocity);
void control_send_start();
void control_send_stop();
void control_send_tempo(float bpm);
void control_send_select_pattern(int index);
void control_send_queue_pattern(int index);
void control_send_cancel_pattern_queue();
void control_send_fill();
void control_send_variation();
enum SequencerVariation : uint8_t {
    SEQ_VAR_NEON_BREAK = 1,
    SEQ_VAR_RATCHET_STORM,
    SEQ_VAR_GHOST_GROOVE,
    SEQ_VAR_POLYRHYTHM,
    SEQ_VAR_HALF_TIME,
    SEQ_VAR_MIRROR,
    SEQ_VAR_TOM_CASCADE,
    SEQ_VAR_ACID_SWITCH,
    SEQ_VAR_HAT_LIFT,
    SEQ_VAR_SPARSE_SPACE,
    SEQ_VAR_UNDO
};
bool control_apply_sequencer_variation(uint8_t variation);
bool control_variation_can_undo();

// "PLAY RANDOM" — generates a brand new pattern from scratch using
// Euclidean-rhythm placement, with per-style hit-count/velocity/probability
// tables (kick/snare/hats/perc). Shares the VARIATIONS undo backup, so
// SEQ_VAR_UNDO also restores the pattern that was active before a random
// generation.
enum RandomPatternStyle : uint8_t {
    RND_STYLE_TECHNO = 1,
    RND_STYLE_HOUSE,
    RND_STYLE_BREAKBEAT,
    RND_STYLE_HIPHOP,
    RND_STYLE_TRAP,
    RND_STYLE_MINIMAL,
};
bool control_apply_random_pattern(uint8_t style);
bool control_sync_current_pattern();
bool control_patterns_ready();
uint8_t control_factory_patterns_found();
uint8_t control_factory_patterns_expected();
void control_send_build4();
void control_send_drop();
void control_send_launch_demo_set();
void control_send_mix_preset(bool club_warm);
void control_send_get_pattern(int pattern);
bool control_save_user_pattern(int source, int destination);
bool control_user_pattern_is_saved(int pattern);
void control_set_pattern_source_tempo(int pattern, float bpm, const char* name);
bool control_install_midi_song(const mem_midi::MidiSongData& song);
bool control_midi_song_ready();
bool control_midi_song_persisted();
void control_cancel_midi_song();
void control_send_unload_daisy(uint8_t pad);
void control_send_set_step(int track, int step, bool active);
void control_send_set_step_velocity(int track, int step, int velocity);
void control_send_mute(int track, bool muted);
void control_send_set_volume(int value);
void control_send_set_seq_volume(int value);
void control_send_set_live_volume(int value);
void control_send_set_track_volume(int track, int volume);
void control_send_set_track_engine(int track, int engine);

// Per-instrument FX (idea 2): one instrument's own filter/drive/crush/sends,
// independent from the FX LAB's global chain.
void control_send_track_filter(int track, int filterType, float cutoffHz,
                               float resonance);
void control_send_track_clear_filter(int track);
void control_send_track_distortion(int track, float amount01);
void control_send_track_bitcrush(int track, int bits);
void control_send_track_reverb_send(int track, int percent);
void control_send_track_delay_send(int track, int percent);
void control_send_track_clear_fx(int track);
void control_send_set_filter(int type);
void control_send_set_filter_cutoff(int hz);
void control_send_set_filter_resonance(float value);
void control_send_set_distortion(float value);
void control_send_set_bitcrush(int bits);
void control_send_set_sample_rate(int rate_hz);
void control_send_set_crush_macro(uint8_t value);
void control_send_fx_enc(int encoder, uint8_t value, bool muted);
void control_send_fx_pot(int pot, uint8_t value, bool muted);
void control_send_all_fx_off();
void control_send_solo(int track, bool soloed);
void control_send_mute_mask(uint16_t mask);
void control_send_solo_mask(uint16_t mask);
void control_mark_fx_screen_dirty();
bool control_consume_fx_screen_dirty();
bool control_pattern_track_uses_sampler(int pattern, int track);
void control_restore_track_engine(int track);

void control_send_synth_note_on_ex(uint8_t engine, uint8_t note,
                                   uint8_t velocity, bool accent, bool slide);
void control_send_synth_note_off(uint8_t engine, uint8_t track);
void control_send_synth_note_off_ex(uint8_t engine, uint8_t track,
                                    uint8_t note);
void control_send_synth303_note_off();
void control_send_synth_trigger(uint8_t engine, uint8_t instrument,
                                uint8_t velocity);
void control_send_synth_param(uint8_t engine, uint8_t instrument,
                              uint8_t parameter, float value);
void control_send_synth_preset(uint8_t engine, uint8_t preset);
void control_send_trim_sample(uint8_t pad, float start, float end);
void control_send_melody_rec_note(uint8_t engine, uint8_t note);
void control_send_melody_assign(uint8_t pad, uint8_t engine, uint8_t octave,
                                const bool grid[16][12],
                                const uint8_t notes[16][12] = nullptr,
                                uint8_t gate_percent = 55);
void control_send_melody_rec_toggle(bool active, uint8_t engine,
                                    uint8_t octave);
void control_send_melody_set_pad(uint8_t pad);
void control_send_melody_set_engine(uint8_t engine);
void control_send_melody_set_octave(uint8_t octave);
void control_send_melody_clear();
void control_send_melody_assign_pad(uint8_t pad, uint8_t engine,
                                    uint8_t octave);
void control_request_sync();

// ── User MIDI map / LEARN (AKAI MPD218) ─────────────────────────────
void control_midi_learn_arm(bool armed);
bool control_midi_learn_armed();
// Bumps whenever an armed LEARN gets auto-disarmed after ~8s with no
// note/CC captured. UI polls this the same way it polls capture_revision.
uint32_t control_midi_learn_timeout_revision();
uint32_t control_midi_capture_revision();
MidiLearnCapture control_midi_capture();
uint32_t control_midi_activity_revision();
void control_midi_last_activity(uint8_t& status, uint8_t& data0,
                                uint8_t& data1);
uint8_t control_midi_map_count();
bool control_midi_map_get(uint8_t index, MidiMapEntry& out);
bool control_midi_map_find(uint8_t channel, uint8_t kind, uint8_t number,
                           MidiMapEntry& out);
bool control_midi_map_assign(const MidiMapEntry& entry);
bool control_midi_map_clear(uint8_t channel, uint8_t kind, uint8_t number);
bool control_midi_map_clear_all();
// Another learned entry already pointing at the same action/target,
// excluding the (channel,kind,number) about to be overwritten.
bool control_midi_map_find_duplicate(uint8_t kind, uint8_t action,
                                     uint8_t arg0, uint8_t arg1,
                                     uint8_t excludeChannel,
                                     uint8_t excludeNumber, MidiMapEntry& out);
// Backup/restore the whole learned map to/from the P4 SD card. Caller
// should confirm the SD card is mounted before calling either.
bool control_midi_map_export_sd();
bool control_midi_map_import_sd();

void local_apply_message(uint8_t type, uint8_t id, uint8_t value);
void local_push_pattern(int pattern, const bool steps[16][16]);
bool local_restore_pattern(uint8_t slot);
void local_stage_pattern(uint8_t slot, const bool steps[16][16]);
void local_lock_tempo(uint32_t duration_ms);
