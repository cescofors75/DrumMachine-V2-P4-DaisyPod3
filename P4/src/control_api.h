#pragma once

#include <stddef.h>
#include <stdint.h>

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
void control_send_build4();
void control_send_drop();
void control_send_launch_demo_set();
void control_send_mix_preset(bool club_warm);
void control_send_get_pattern(int pattern);
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
void control_send_fx_enc(int encoder, uint8_t value, bool muted);
void control_send_fx_pot(int pot, uint8_t value, bool muted);
void control_send_solo(int track, bool soloed);
void control_send_mute_mask(uint16_t mask);
void control_send_solo_mask(uint16_t mask);
void control_mark_fx_screen_dirty();
bool control_consume_fx_screen_dirty();

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
