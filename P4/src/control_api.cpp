#include "control_api.h"

#include "app_state.h"
#include "daisy_usb_transport.h"
#include "master/PatternBank.h"
#include "master/Sequencer.h"
#include "ui/ui_screens.h"
#include "../include/ui_events.h"
#include <Arduino.h>
#include <atomic>
#include <math.h>
#include <string.h>

namespace
{
// Do not construct the sequencer as a global object. Its constructor allocates
// the pattern bank with ps_calloc(), but Arduino adds PSRAM to the heap during
// initArduino(), after global C++ constructors have already run. Construct it
// lazily from control_init(), once setup() and the display are running.
Sequencer& SequencerInstance()
{
    static Sequencer instance;
    return instance;
}

std::atomic<bool> fxDirty{false};
uint32_t tempoLockUntilMs = 0;
uint8_t melodyEngine = 3;
uint8_t melodyOctave = 4;
uint8_t melodyPad = 0;
bool melodyRecording = false;
bool engineWasConnected = false;
uint32_t lastPodRevision = 0;
int queuedLogicalPattern = -1;
uint8_t activeDaisyPattern = 0;
uint8_t queuedDaisyPattern = 0xFF;

template<typename T>
T Clamp(T value, T low, T high)
{
    return value < low ? low : value > high ? high : value;
}

uint8_t DefaultDaisyPattern(int pattern)
{
    const int normalized = pattern < 0 ? 0 : pattern;
    return static_cast<uint8_t>(normalized % 20);
}

void LoadPatternToUi(int pattern)
{
    pattern = Clamp(pattern, 0, MAX_PATTERNS - 1);
    p4.current_pattern = pattern;
    for(int track = 0; track < 16; ++track)
        for(int step = 0; step < 16; ++step)
            p4.steps[track][step] = SequencerInstance().getStep(pattern, track, step);
}

void SendFilterState()
{
    GlobalFilterPayload payload = {};
    payload.filterType = static_cast<uint8_t>(Clamp(p4.filter_type, 0, 14));
    payload.distMode = 0;
    payload.bitDepth = static_cast<uint8_t>(Clamp(p4.bitcrush_bits, 1, 16));
    payload.cutoff = static_cast<float>(Clamp(p4.cutoff_hz, 20, 20000));
    payload.resonance = Clamp(p4.resonance_x10 / 10.0f, 0.1f, 30.0f);
    payload.distortion = static_cast<float>(Clamp(p4.distortion_pct, 0, 100));
    payload.sampleRateReduce = static_cast<uint32_t>(Clamp(p4.sample_rate_hz, 1000, 48000));
    daisyUsb.send(CMD_FILTER_SET, &payload, sizeof(payload));
    fxDirty.store(true, std::memory_order_release);
}

void UploadPattern(uint8_t destination, const bool steps[16][16])
{
    for(uint8_t track = 0; track < 16; ++track)
        daisyUsb.uploadTrack(destination, track, steps[track], 100);
}

void SendCurrentState()
{
    const float bpm = static_cast<float>(p4.bpm_int)
                    + static_cast<float>(p4.bpm_frac) / 10.0f;
    daisyUsb.setTempo(Clamp(bpm, 40.0f, 240.0f));
    daisyUsb.sendU8(CMD_MASTER_VOLUME,
                    static_cast<uint8_t>(Clamp(p4.master_volume, 0, 150)));
    daisyUsb.sendU8(CMD_SEQ_VOLUME,
                    static_cast<uint8_t>(Clamp(p4.seq_volume, 0, 150)));
    daisyUsb.sendU8(CMD_LIVE_VOLUME,
                    static_cast<uint8_t>(Clamp(p4.live_volume, 0, 150)));
    for(uint8_t track = 0; track < 16; ++track)
    {
        daisyUsb.setTrackVolume(
            track, static_cast<uint8_t>(Clamp(p4.track_volume[track], 0, 150)));
    }
    activeDaisyPattern = DefaultDaisyPattern(p4.current_pattern);
    UploadPattern(activeDaisyPattern, p4.steps);
    daisyUsb.selectPattern(activeDaisyPattern);
    if(p4.is_playing) daisyUsb.start(); else daisyUsb.stop();
}
}

void control_init()
{
    Sequencer& sequencer = SequencerInstance();
    initializeProfessionalPatternBank(sequencer);
    sequencer.setTempo(120.0f);
    sequencer.selectPattern(0);
    p4.bpm_int = 120;
    p4.bpm_frac = 0;
    p4.master_volume = 100;
    p4.seq_volume = 100;
    p4.live_volume = 100;
    for(int track = 0; track < 16; ++track)
        p4.track_volume[track] = 100;
    engineWasConnected = false;
    lastPodRevision = 0;
    LoadPatternToUi(0);
    daisyUsb.begin();
}

void control_process()
{
    daisyUsb.process();
    const auto& transport = daisyUsb.state();
    p4.master_connected = transport.engine_responding;
    p4.last_heartbeat_ms = transport.last_response_ms;
    p4.current_step = transport.step & 0x0F;
    p4.is_playing = transport.playing;
    for(uint8_t pad = 0; pad < 16; ++pad)
        p4.sample_loaded[pad] = (transport.sample_mask & (1u << pad)) != 0;
    for(uint8_t pad = 16; pad < 24; ++pad)
        p4.sample_loaded[pad] = (transport.xtra_sample_mask & (1u << (pad - 16))) != 0;
    if(transport.kit_name[0] != '\0')
    {
        memcpy(p4.kit_name, transport.kit_name, sizeof(p4.kit_name));
        p4.kit_name[sizeof(p4.kit_name) - 1] = '\0';
    }
    if(transport.pod_revision != lastPodRevision
       && transport.pod.config.version == 2)
    {
        lastPodRevision = transport.pod_revision;
        p4.master_volume = transport.pod.masterVolume;
        p4.seq_volume = transport.pod.seqVolume;
        p4.live_volume = transport.pod.liveVolume;
        p4.enc_value[1] = transport.pod.delayMixValue;
        p4.enc_muted[1] = (transport.pod.fxActiveBits & 1u) == 0;
        p4.enc_value[2] = transport.pod.reverbMixValue;
        p4.enc_muted[2] = (transport.pod.fxActiveBits & 2u) == 0;
        p4.bpm_int = transport.pod.bpmX10 / 10;
        p4.bpm_frac = transport.pod.bpmX10 % 10;
        p4.is_playing = transport.pod.playing != 0;
        SequencerInstance().setTempo(transport.pod.bpmX10 / 10.0f);
        if(p4.is_playing && !SequencerInstance().isPlaying())
            SequencerInstance().start();
        else if(!p4.is_playing && SequencerInstance().isPlaying())
            SequencerInstance().stop();
    }
    if(transport.engine_responding && queuedLogicalPattern >= 0
       && transport.pattern == queuedDaisyPattern)
    {
        activeDaisyPattern = queuedDaisyPattern;
        SequencerInstance().selectPattern(queuedLogicalPattern);
        LoadPatternToUi(queuedLogicalPattern);
        ui_pattern_queue_committed(queuedLogicalPattern);
        queuedLogicalPattern = -1;
        queuedDaisyPattern = 0xFF;
    }
    else if(transport.engine_responding && queuedLogicalPattern < 0
            && transport.pattern != activeDaisyPattern
            && transport.pattern < MAX_PATTERNS)
    {
        activeDaisyPattern = transport.pattern;
        SequencerInstance().selectPattern(transport.pattern);
        LoadPatternToUi(transport.pattern);
        UploadPattern(activeDaisyPattern, p4.steps);
    }
    if(transport.engine_responding && !engineWasConnected)
        SendCurrentState();
    engineWasConnected = transport.engine_responding;
    SequencerInstance().update();
}

bool control_available() { return daisyUsb.state().link_ready; }
bool control_engine_connected() { return daisyUsb.connected(); }
int control_current_step_raw() { return daisyUsb.state().step; }

void control_send_trigger(uint8_t pad, uint8_t velocity)
{
    daisyUsb.trigger(pad, velocity);
    if(pad < 16)
    {
        p4.pad_velocity[pad] = velocity;
        p4.pad_flash_until[pad] = millis() + 100;
    }
}

void control_send_start()
{
    SequencerInstance().start();
    p4.is_playing = true;
    daisyUsb.start();
}

void control_send_stop()
{
    SequencerInstance().stop();
    p4.is_playing = false;
    daisyUsb.stop();
}

void control_send_tempo(float bpm)
{
    bpm = Clamp(bpm, 40.0f, 240.0f);
    SequencerInstance().setTempo(bpm);
    p4.bpm_int = static_cast<int>(bpm);
    p4.bpm_frac = static_cast<int>((bpm - p4.bpm_int) * 10.0f + 0.5f);
    daisyUsb.setTempo(bpm);
}

void control_send_select_pattern(int index)
{
    index = Clamp(index, 0, MAX_PATTERNS - 1);
    SequencerInstance().selectPattern(index);
    LoadPatternToUi(index);
    activeDaisyPattern = DefaultDaisyPattern(index);
    queuedLogicalPattern = -1;
    queuedDaisyPattern = 0xFF;
    UploadPattern(activeDaisyPattern, p4.steps);
    daisyUsb.selectPattern(activeDaisyPattern);
}

void control_send_queue_pattern(int index)
{
    queuedLogicalPattern = Clamp(index, 0, MAX_PATTERNS - 1);
    queuedDaisyPattern = static_cast<uint8_t>((activeDaisyPattern + 1u) % 20u);
    bool steps[16][16] = {};
    for(int track = 0; track < 16; ++track)
        for(int step = 0; step < 16; ++step)
            steps[track][step] = SequencerInstance().getStep(
                queuedLogicalPattern, track, step);
    UploadPattern(queuedDaisyPattern, steps);
    daisyUsb.queuePattern(queuedDaisyPattern);
}

void control_send_cancel_pattern_queue()
{
    queuedLogicalPattern = -1;
    queuedDaisyPattern = 0xFF;
    daisyUsb.cancelPatternQueue();
}

void control_send_fill()
{
    for(int step = 0; step < 16; ++step)
    {
        const bool active = (step & 1) == 0 || step >= 12;
        control_send_set_step(2, step, active);
    }
}

void control_send_variation()
{
    for(int step = 1; step < 16; step += 4)
        control_send_set_step(5, step, !p4.steps[5][step]);
}

void control_send_build4()
{
    for(int step = 12; step < 16; ++step)
        control_send_set_step(1, step, true);
}

void control_send_drop()
{
    for(int track = 2; track < 16; ++track)
        control_send_mute(track, true);
}

void control_send_launch_demo_set()
{
    control_send_select_pattern(0);
    control_send_start();
}

void control_send_mix_preset(bool club_warm)
{
    control_send_set_volume(club_warm ? 110 : 100);
    control_send_set_filter_cutoff(club_warm ? 12000 : 20000);
    control_send_set_filter_resonance(club_warm ? 1.4f : 1.0f);
}

void control_send_get_pattern(int pattern) { LoadPatternToUi(pattern); }

void control_send_unload_daisy(uint8_t pad)
{
    daisyUsb.sendU8(CMD_SAMPLE_UNLOAD, pad);
    if(pad < 24) p4.sample_loaded[pad] = false;
}

void control_send_set_step(int track, int step, bool active)
{
    if(track < 0 || track >= 16 || step < 0 || step >= 16) return;
    SequencerInstance().setStep(p4.current_pattern, track, step, active, 100);
    p4.steps[track][step] = active;
    daisyUsb.setStep(activeDaisyPattern, track, step, active, 100);
}

void control_send_set_step_velocity(int track, int step, int velocity)
{
    if(track < 0 || track >= 16 || step < 0 || step >= 16) return;
    velocity = Clamp(velocity, 1, 127);
    SequencerInstance().setStepVelocity(p4.current_pattern, track, step, velocity);
    daisyUsb.setStep(activeDaisyPattern, track, step,
                     p4.steps[track][step], static_cast<uint8_t>(velocity));
}

void control_send_mute(int track, bool muted)
{
    if(track < 0 || track >= 16) return;
    p4.track_muted[track] = muted;
    daisyUsb.setTrackMute(track, muted);
}

void control_send_set_volume(int value)
{
    p4.master_volume = Clamp(value, 0, 150);
    daisyUsb.sendU8(CMD_MASTER_VOLUME, p4.master_volume);
}

void control_send_set_seq_volume(int value)
{
    p4.seq_volume = Clamp(value, 0, 150);
    daisyUsb.sendU8(CMD_SEQ_VOLUME, p4.seq_volume);
}

void control_send_set_live_volume(int value)
{
    p4.live_volume = Clamp(value, 0, 150);
    daisyUsb.sendU8(CMD_LIVE_VOLUME, p4.live_volume);
}

void control_send_set_track_volume(int track, int volume)
{
    if(track < 0 || track >= 16) return;
    volume = Clamp(volume, 0, 150);
    p4.track_volume[track] = volume;
    SequencerInstance().setTrackVolume(track, volume);
    daisyUsb.setTrackVolume(track, volume);
}

void control_send_set_track_engine(int track, int engine)
{
    if(track >= 0 && track < 16)
        daisyUsb.setTrackEngine(track, static_cast<int8_t>(engine));
}

void control_send_set_filter(int type)
{
    p4.filter_type = type;
    SendFilterState();
}

void control_send_set_filter_cutoff(int hz)
{
    p4.cutoff_hz = hz;
    SendFilterState();
}

void control_send_set_filter_resonance(float value)
{
    p4.resonance_x10 = static_cast<int>(value * 10.0f + 0.5f);
    SendFilterState();
}

void control_send_set_distortion(float value)
{
    p4.distortion_pct = static_cast<int>(Clamp(value, 0.0f, 1.0f) * 100.0f);
    SendFilterState();
}

void control_send_set_bitcrush(int bits)
{
    p4.bitcrush_bits = bits;
    SendFilterState();
}

void control_send_set_sample_rate(int rate_hz)
{
    p4.sample_rate_hz = rate_hz;
    SendFilterState();
}

void control_send_fx_enc(int encoder, uint8_t value, bool muted)
{
    if(encoder < 0 || encoder >= 3) return;
    p4.enc_value[encoder] = value;
    p4.enc_muted[encoder] = muted;
    const uint8_t active = muted ? 0u : 1u;
    const float unit = value / 127.0f;
    if(encoder == 0)
    {
        daisyUsb.sendU8(CMD_FLANGER_ACTIVE, active);
        daisyUsb.sendFloat(CMD_FLANGER_DEPTH, unit);
    }
    else if(encoder == 1)
    {
        daisyUsb.sendU8(CMD_DELAY_ACTIVE, active);
        daisyUsb.sendFloat(CMD_DELAY_MIX, unit);
    }
    else
    {
        daisyUsb.sendU8(CMD_REVERB_ACTIVE, active);
        daisyUsb.sendFloat(CMD_REVERB_MIX, unit);
    }
    fxDirty.store(true, std::memory_order_release);
}

void control_send_fx_pot(int pot, uint8_t value, bool muted)
{
    if(pot < 0 || pot >= 3) return;
    p4.pot_muted[pot] = muted;
    const float unit = value / 127.0f;
    if(pot == 0)
        daisyUsb.sendFloat(CMD_WAVEFOLDER_GAIN, muted ? 1.0f : 1.0f + unit * 9.0f);
    else if(pot == 1)
        control_send_set_bitcrush(muted ? 16 : static_cast<int>(16.0f - unit * 8.0f));
    else
    {
        daisyUsb.sendU8(CMD_PHASER_ACTIVE, muted ? 0u : 1u);
        daisyUsb.sendFloat(CMD_PHASER_DEPTH, unit);
    }
    fxDirty.store(true, std::memory_order_release);
}

void control_send_solo(int track, bool soloed)
{
    if(track < 0 || track >= 16) return;
    p4.track_solo[track] = soloed;
    daisyUsb.setTrackSolo(track, soloed);
}

void control_send_mute_mask(uint16_t mask)
{
    for(int track = 0; track < 16; ++track)
        control_send_mute(track, (mask & (1u << track)) != 0);
}

void control_send_solo_mask(uint16_t mask)
{
    for(int track = 0; track < 16; ++track)
        control_send_solo(track, (mask & (1u << track)) != 0);
}

void control_mark_fx_screen_dirty() { fxDirty.store(true, std::memory_order_release); }
bool control_consume_fx_screen_dirty()
{
    return fxDirty.exchange(false, std::memory_order_acq_rel);
}

void control_send_synth_note_on_ex(uint8_t engine, uint8_t note,
                                   uint8_t velocity, bool accent, bool slide)
{
    daisyUsb.synthNoteOn(engine, note, velocity, accent, slide);
}

void control_send_synth_note_off(uint8_t engine, uint8_t track)
{
    daisyUsb.synthNoteOff(engine, track);
}

void control_send_synth_note_off_ex(uint8_t engine, uint8_t track, uint8_t note)
{
    daisyUsb.synthNoteOff(engine, track, note);
}

void control_send_synth303_note_off() { daisyUsb.send(CMD_SYNTH_NOTE_OFF); }
void control_send_synth_trigger(uint8_t engine, uint8_t instrument, uint8_t velocity)
{
    daisyUsb.synthTrigger(engine, instrument, velocity);
}

void control_send_synth_param(uint8_t engine, uint8_t instrument,
                              uint8_t parameter, float value)
{
    daisyUsb.synthParam(engine, instrument, parameter, value);
}

void control_send_synth_preset(uint8_t engine, uint8_t preset)
{
    daisyUsb.synthPreset(engine, preset);
}

void control_send_trim_sample(uint8_t, float, float)
{
    // The current RED808 DSP contract has no destructive trim command.
}

void control_send_melody_rec_note(uint8_t engine, uint8_t note)
{
    if(melodyRecording)
        daisyUsb.synthNoteOn(engine, note, 100, false, false);
}

void control_send_melody_assign(uint8_t pad, uint8_t engine, uint8_t octave,
                                const bool grid[16][12],
                                const uint8_t notes[16][12], uint8_t)
{
    const uint8_t pattern = activeDaisyPattern;
    for(uint8_t step = 0; step < 16; ++step)
    {
        uint8_t noteList[4] = {};
        uint8_t count = 0;
        for(uint8_t row = 0; row < 12 && count < 4; ++row)
        {
            if(!grid[step][row]) continue;
            noteList[count++] = notes ? notes[step][row]
                : static_cast<uint8_t>(octave * 12 + 11 - row);
        }
        if(count == 0) continue;
        uint8_t payload[8] = {pattern, pad, step, 0,
                              noteList[0], noteList[1], noteList[2], noteList[3]};
        daisyUsb.setStep(pattern, pad, step, true, 100);
        daisyUsb.send(CMD_DSQ_SET_STEP_NOTES, payload, sizeof(payload));
    }
    daisyUsb.setTrackEngine(pad, engine);
}

void control_send_melody_rec_toggle(bool active, uint8_t engine, uint8_t octave)
{
    melodyRecording = active;
    melodyEngine = engine;
    melodyOctave = octave;
    p4_publish_pending_melody(engine, octave, active, melodyPad);
}
void control_send_melody_set_pad(uint8_t pad) { melodyPad = pad & 15u; }
void control_send_melody_set_engine(uint8_t engine) { melodyEngine = engine; }
void control_send_melody_set_octave(uint8_t octave) { melodyOctave = octave; }
void control_send_melody_clear() { daisyUsb.stopAll(); }
void control_send_melody_assign_pad(uint8_t pad, uint8_t engine, uint8_t octave)
{
    melodyPad = pad & 15u;
    melodyEngine = engine;
    melodyOctave = octave;
}
void control_request_sync() { daisyUsb.send(CMD_GET_STATUS); }

void local_apply_message(uint8_t type, uint8_t id, uint8_t value)
{
    if(type == MSG_SYSTEM && id == SYS_PLAY_STATE)
    {
        if(value) control_send_start(); else control_send_stop();
    }
    else if(type == MSG_SYSTEM && id == SYS_STEP)
        p4.current_step = value & 15u;
    else if(type == MSG_TOUCH_CMD && id == TCMD_PAD_TAP)
        control_send_trigger(value, 127);
    else if(type == MSG_TOUCH_CMD && id == TCMD_PATTERN_SEL)
        control_send_select_pattern(value);
    else if(type == MSG_TRACK && (id & 0xF0u) == TRK_MUTE_BIT)
        control_send_mute(id & 15u, value != 0);
    else if(type == MSG_TRACK && (id & 0xF0u) == TRK_VOLUME)
        control_send_set_track_volume(id & 15u, value);
}

void local_push_pattern(int pattern, const bool steps[16][16])
{
    pattern = Clamp(pattern, 0, MAX_PATTERNS - 1);
    for(int track = 0; track < 16; ++track)
        for(int step = 0; step < 16; ++step)
            SequencerInstance().setStep(
                pattern, track, step, steps[track][step], 100);
    activeDaisyPattern = DefaultDaisyPattern(pattern);
    UploadPattern(activeDaisyPattern, steps);
    daisyUsb.selectPattern(activeDaisyPattern);
}

bool local_restore_pattern(uint8_t slot)
{
    if(slot >= MAX_PATTERNS) return false;
    LoadPatternToUi(slot);
    return true;
}

void local_stage_pattern(uint8_t slot, const bool steps[16][16])
{
    local_push_pattern(slot, steps);
}

void local_lock_tempo(uint32_t duration_ms)
{
    tempoLockUntilMs = millis() + duration_ms;
    (void)tempoLockUntilMs;
}
