#pragma once

#include <stddef.h>
#include <stdint.h>

namespace mem_midi { struct MidiSongData; }

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
// FILL and BUILD render a variant of the current pattern into a scratch Daisy
// slot and let the engine return on its own after 1 / 4 bars. They never modify
// the P4 bank. Return false when there is no engine, or when SONG mode owns
// every resident slot.
bool control_send_fill();
void control_send_variation();
// Each variation transforms the pattern that is loaded rather than stamping a
// fixed beat over it, so the result depends on — and preserves — whatever the
// user or the MIDI importer put there.
enum SequencerVariation : uint8_t {
    SEQ_VAR_GHOST_GROOVE = 1,   // quiet notes in the gaps before existing hits
    SEQ_VAR_ROTATE,             // percussion shifted, kick and snare anchored
    SEQ_VAR_HALF_TIME,          // first half stretched over the whole bar
    SEQ_VAR_DOUBLE_TIME,        // first half folded and played twice
    SEQ_VAR_SWAP_HALVES,        // beats 1-2 traded with beats 3-4
    SEQ_VAR_EUCLID,             // same hit count, evenly redistributed
    SEQ_VAR_ACCENT_GROOVE,      // re-articulates dynamics, moves no notes
    SEQ_VAR_RATCHET_STORM,      // rolls on the hits already in the last beat
    SEQ_VAR_SIXTEENTH_LIFT,     // offbeat sixteenths on the busiest track
    SEQ_VAR_THIN_OUT,           // opens space, keeps downbeats
    SEQ_VAR_DICE_PROBABILITY,   // makes offbeats probabilistic
    SEQ_VAR_BREAK,              // syncopated break built from the last beat
    SEQ_VAR_UNDO,
    SEQ_VAR_FIRST = SEQ_VAR_GHOST_GROOVE
};
bool control_apply_sequencer_variation(uint8_t variation);
bool control_variation_can_undo();
bool control_sync_current_pattern();
bool control_patterns_ready();
uint8_t control_factory_patterns_found();
uint8_t control_factory_patterns_expected();
bool control_send_build4();
void control_send_drop();   // toggles: mutes tracks 2..15, then restores
bool control_drop_active();
void control_send_launch_demo_set();
void control_send_get_pattern(int pattern);
bool control_save_user_pattern(int source, int destination);
bool control_user_pattern_is_saved(int pattern);
void control_set_pattern_source_tempo(int pattern, float bpm, const char* name);
bool control_install_midi_song(const mem_midi::MidiSongData& song);
bool control_midi_song_ready();
bool control_midi_song_persisted();
void control_cancel_midi_song();

// SONG mode. An imported arrangement stays resident until it is explicitly
// cancelled; engaging and disengaging only decides whether it drives the
// transport, so the user can audition a single pattern and come back.
bool control_song_available();
bool control_song_engaged();
bool control_song_set_engaged(bool engaged);
bool control_song_loop();
void control_song_set_loop(bool loop);
// Reports the live chain position: `index` of `count`, and the logical P4
// pattern currently playing. Any pointer may be null.
void control_song_status(uint8_t* index, uint8_t* count, uint8_t* pattern);
void control_send_unload_daisy(uint8_t pad);
void control_send_set_step(int track, int step, bool active);
void control_send_set_step_velocity(int track, int step, int velocity);
void control_send_mute(int track, bool muted);
void control_send_set_volume(int value);
void control_send_set_seq_volume(int value);
void control_send_set_live_volume(int value);
void control_send_set_track_volume(int track, int volume);
void control_send_set_track_engine(int track, int engine);
void control_send_set_filter(int type);
void control_send_set_filter_cutoff(int hz);
void control_send_set_filter_resonance(float value);
void control_send_set_distortion(float value);
void control_send_set_bitcrush(int bits);
void control_send_set_sample_rate(int rate_hz);
void control_send_set_crush_macro(uint8_t value);
void control_send_fx_enc(int encoder, uint8_t value, bool muted);
void control_send_fx_pot(int pot, uint8_t value, bool muted);
void control_send_solo(int track, bool soloed);
void control_send_mute_mask(uint16_t mask);
void control_send_solo_mask(uint16_t mask);
void control_mark_fx_screen_dirty();
bool control_consume_fx_screen_dirty();
bool control_pattern_track_uses_sampler(int pattern, int track);
// Re-assert the engine the current pattern assigns to a track. Needed after a
// sample upload, because Daisy switches the pad to the sampler on every load.
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

void local_apply_message(uint8_t type, uint8_t id, uint8_t value);
void local_push_pattern(int pattern, const bool steps[16][16]);
bool local_restore_pattern(uint8_t slot);
void local_stage_pattern(uint8_t slot, const bool steps[16][16]);
void local_lock_tempo(uint32_t duration_ms);
