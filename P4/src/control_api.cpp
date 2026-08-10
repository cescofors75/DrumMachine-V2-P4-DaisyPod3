#include "control_api.h"

#include "app_state.h"
#include "daisy_usb_transport.h"
#include "drivers/i2c_rotaries.h"
#include "master/PatternBank.h"
#include "master/Sequencer.h"
#include "pattern_store.h"
#include "mem_midi_loader.h"
#include "pod_config_store.h"
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
uint32_t lastPodStateRevision = 0xFFFFFFFFu;
int queuedLogicalPattern = -1;
uint8_t activeDaisyPattern = 0;
uint8_t queuedDaisyPattern = 0xFF;
uint8_t expectedDaisyPattern = 0xFF;
uint32_t expectedDaisyPatternSinceMs = 0;
// `midiSongPrepared` means an arrangement is resident in RAM (and in the user
// pattern slots). `midiSongEngaged` means SONG mode is actually driving the
// transport. Keeping the two apart is what lets the user step out to a single
// pattern and back into the arrangement without re-importing the MIDI file.
std::atomic<bool> midiSongPrepared{false};
std::atomic<bool> midiSongEngaged{false};
std::atomic<bool> midiSongLoop{true};
std::atomic<bool> midiSongPersisted{false};
uint8_t midiSongPatternCount = 0;
uint8_t midiSongChainCount = 0;
uint8_t midiSongIndex = 0;
uint8_t midiSongRepeat = 0;
uint8_t midiSongLogicalPattern[mem_midi::MIDI_SONG_MAX_PATTERNS] = {};
SongEntry midiSongDaisyChain[mem_midi::MIDI_SONG_MAX_CHAIN] = {};

struct VariationSnapshot
{
    bool valid = false;
    int pattern = -1;
    bool active[16][16] = {};
    uint8_t velocity[16][16] = {};
    uint8_t probability[16][16] = {};
    uint8_t ratchet[16][16] = {};
};

VariationSnapshot variationBackup;
bool patternBankReady = false;
uint8_t factoryPatternsFound = 0;

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

void SetOriginalTempo(float bpm)
{
    if(!isfinite(bpm) || bpm < 30.0f || bpm > 300.0f)
    {
        p4.original_bpm_x10 = 0;
        return;
    }
    p4.original_bpm_x10 = Clamp(static_cast<int>(lroundf(bpm * 10.0f)),
                                300, 3000);
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
    payload.bitDepth = static_cast<uint8_t>(Clamp(p4.bitcrush_bits, 4, 16));
    payload.cutoff = static_cast<float>(Clamp(p4.cutoff_hz, 20, 20000));
    payload.resonance = Clamp(p4.resonance_x10 / 10.0f, 0.3f, 30.0f);
    payload.distortion = static_cast<float>(Clamp(p4.distortion_pct, 0, 100));
    payload.sampleRateReduce = p4.sample_rate_hz <= 0
        ? 0u : static_cast<uint32_t>(Clamp(p4.sample_rate_hz, 1000, 48000));
    daisyUsb.send(CMD_FILTER_SET, &payload, sizeof(payload));
    fxDirty.store(true, std::memory_order_release);
}

bool SendWithRetry(uint8_t command, const void* payload, uint16_t length)
{
    for(int attempt = 0; attempt < 200; ++attempt)
    {
        if(!control_available()) return false;
        if(daisyUsb.send(command, payload, length)) return true;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

bool UploadPattern(uint8_t destination, int logicalPattern)
{
    Sequencer& sequencer = SequencerInstance();
    logicalPattern = Clamp(logicalPattern, 0, MAX_PATTERNS - 1);
    const uint8_t length = 16;
    bool uploaded = SendWithRetry(CMD_DSQ_SET_LENGTH, &length, sizeof(length));

    StepUploadData snapshot[16] = {};
    uint8_t trackPayload[4 + 16 * sizeof(DsqStepPkt)] = {};
    auto* steps = reinterpret_cast<DsqStepPkt*>(trackPayload + 4);
    for(uint8_t track = 0; track < 16; ++track)
    {
        sequencer.snapshotTrackForUpload(logicalPattern, track, 16, snapshot);
        trackPayload[0] = destination;
        trackPayload[1] = track;
        trackPayload[2] = 16;
        trackPayload[3] = 0;
        for(uint8_t step = 0; step < 16; ++step)
        {
            steps[step].active = snapshot[step].active ? 1u : 0u;
            steps[step].velocity = snapshot[step].velocity;
            const uint8_t noteLength = snapshot[step].noteLenDiv & 0x0Fu;
            const uint8_t ratchet = Clamp<uint8_t>(snapshot[step].ratchet, 1, 4);
            steps[step].noteLenDiv = noteLength | ((ratchet - 1u) << 4);
            steps[step].probability = snapshot[step].probability;
        }
        if(!SendWithRetry(CMD_DSQ_UPLOAD_TRACK, trackPayload, sizeof(trackPayload)))
            uploaded = false;

        // The track upload intentionally clears melodic information in Daisy;
        // restore the note voices and accent/slide flags exactly as S3 did.
        for(uint8_t step = 0; step < 16; ++step)
        {
            bool hasNotes = false;
            for(uint8_t voice = 0; voice < MELODY_STEP_VOICES; ++voice)
                hasNotes |= snapshot[step].noteVoices[voice] != 0;
            if(hasNotes || snapshot[step].flags != 0)
            {
                DsqSetStepNotesPayload notes = {};
                notes.pattern = destination;
                notes.track = track;
                notes.step = step;
                notes.flags = snapshot[step].flags;
                memcpy(notes.notes, snapshot[step].noteVoices, sizeof(notes.notes));
                if(!SendWithRetry(CMD_DSQ_SET_STEP_NOTES, &notes, sizeof(notes)))
                    uploaded = false;
            }

            if(snapshot[step].cutoffEn || snapshot[step].reverbEn
               || snapshot[step].volumeEn)
            {
                DsqSetParamLockPayload lock = {};
                lock.pattern = destination;
                lock.track = track;
                lock.step = step;
                lock.cutoffEn = snapshot[step].cutoffEn ? 1u : 0u;
                lock.cutoffHi = static_cast<uint8_t>(snapshot[step].cutoffHz >> 8);
                lock.cutoffLo = static_cast<uint8_t>(snapshot[step].cutoffHz);
                lock.reverbEn = snapshot[step].reverbEn ? 1u : 0u;
                lock.reverbSend = snapshot[step].reverbSend;
                lock.volEn = snapshot[step].volumeEn ? 1u : 0u;
                lock.volume = snapshot[step].volume;
                if(!SendWithRetry(CMD_DSQ_SET_PARAM_LOCK, &lock, sizeof(lock)))
                    uploaded = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return uploaded;
}

bool UploadPreparedMidiSong()
{
    if(!midiSongPrepared || midiSongPatternCount == 0
       || midiSongChainCount == 0 || !control_available())
        return false;

    for(uint8_t resident = 0; resident < midiSongPatternCount; ++resident)
        UploadPattern(resident, midiSongLogicalPattern[resident]);

    // MIDI drum imports target the sampler kit. Clear any synth-engine profile
    // left by the previously resident scene before starting the arrangement.
    for(uint8_t track = 0; track < 16; ++track)
        daisyUsb.setTrackEngine(track, -1);

    if(!daisyUsb.uploadSong(midiSongDaisyChain, midiSongChainCount)) return false;
    daisyUsb.controlSong(2, midiSongLoop.load(std::memory_order_acquire));
    activeDaisyPattern = 0;
    expectedDaisyPattern = 0;
    expectedDaisyPatternSinceMs = millis();
    midiSongIndex = 0;
    midiSongRepeat = 0;
    daisyUsb.selectPattern(0);
    return true;
}

void ApplyPatternPerformance(int logicalPattern)
{
    Sequencer& sequencer = SequencerInstance();
    PatternMetadata metadata{};
    if(sequencer.getPatternMetadata(logicalPattern, metadata))
    {
        SetOriginalTempo(static_cast<float>(metadata.recommendedBpm));
        if(metadata.recommendedBpm >= 40 && metadata.recommendedBpm <= 240
           && !i2c_rotaries_owns_function(POD_FUNC_TEMPO))
        {
            sequencer.setTempo((float)metadata.recommendedBpm);
            p4.bpm_int = metadata.recommendedBpm;
            p4.bpm_frac = 0;
            daisyUsb.setTempo((float)metadata.recommendedBpm);
        }
        sequencer.setHumanize(metadata.humanizeTimingMs,
                              metadata.humanizeVelocity);
        SendWithRetry(CMD_DSQ_SET_SWING, &metadata.swing, 1);
        const uint8_t humanize[2] = {metadata.humanizeTimingMs,
                                     metadata.humanizeVelocity};
        SendWithRetry(CMD_DSQ_SET_HUMANIZE, humanize, sizeof(humanize));
    }

    BuiltinPatternSoundProfile sound{};
    if(getBuiltinPatternSoundProfile(logicalPattern, sound))
    {
        for(uint8_t engine = 0; engine < BUILTIN_ENGINE_COUNT; ++engine)
            daisyUsb.synthPreset(engine, sound.presets[engine]);
        for(uint8_t track = 0; track < 16; ++track)
        {
            daisyUsb.setTrackEngine(track, sound.engines[track]);
        }
    }
}

void SendCurrentState()
{
    SequencerInstance().selectPattern(Clamp(p4.current_pattern, 0, MAX_PATTERNS - 1));
    LoadPatternToUi(p4.current_pattern);
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
        SequencerInstance().muteTrack(track, p4.track_muted[track]);
    }
    if(midiSongEngaged && midiSongPatternCount > 0)
    {
        const int firstPattern = midiSongLogicalPattern[0];
        SequencerInstance().selectPattern(firstPattern);
        LoadPatternToUi(firstPattern);
        ApplyPatternPerformance(firstPattern);
        UploadPreparedMidiSong();
        uint16_t muteMask = 0;
        uint16_t soloMask = 0;
        for(uint8_t track = 0; track < 16; ++track)
        {
            if(p4.track_muted[track]) muteMask |= (uint16_t)(1u << track);
            if(p4.track_solo[track]) soloMask |= (uint16_t)(1u << track);
        }
        daisyUsb.setTrackMuteMask(muteMask);
        daisyUsb.setTrackSoloMask(soloMask);
        if(p4.is_playing)
            daisyUsb.controlSong(1, midiSongLoop.load(std::memory_order_acquire));
        else daisyUsb.stop();
        return;
    }
    activeDaisyPattern = DefaultDaisyPattern(p4.current_pattern);
    UploadPattern(activeDaisyPattern, p4.current_pattern);
    ApplyPatternPerformance(p4.current_pattern);
    expectedDaisyPattern = activeDaisyPattern;
    expectedDaisyPatternSinceMs = millis();
    daisyUsb.selectPattern(activeDaisyPattern);
    uint16_t muteMask = 0;
    uint16_t soloMask = 0;
    for(uint8_t track = 0; track < 16; ++track)
    {
        if(p4.track_muted[track]) muteMask |= (uint16_t)(1u << track);
        if(p4.track_solo[track]) soloMask |= (uint16_t)(1u << track);
    }
    daisyUsb.setTrackMuteMask(muteMask);
    daisyUsb.setTrackSoloMask(soloMask);
    if(p4.is_playing) daisyUsb.start(); else daisyUsb.stop();
}
}

void control_init()
{
    Sequencer& sequencer = SequencerInstance();
    initializeEsp32S3FactoryPatternBank(sequencer);
    pattern_store_load_user_bank(sequencer);
    sequencer.setTempo(124.0f);
    sequencer.selectPattern(0);
    p4.bpm_int = 124;
    p4.bpm_frac = 0;
    p4.original_bpm_x10 = 0;
    p4.master_volume = 100;
    p4.seq_volume = 100;
    p4.live_volume = 100;
    for(int track = 0; track < 16; ++track)
        p4.track_volume[track] = 100;
    engineWasConnected = false;
    lastPodStateRevision = 0xFFFFFFFFu;
    expectedDaisyPattern = 0xFF;
    expectedDaisyPatternSinceMs = 0;
    midiSongPrepared = false;
    midiSongEngaged = false;
    midiSongLoop = true;
    midiSongPersisted = false;
    midiSongPatternCount = 0;
    midiSongChainCount = 0;
    midiSongIndex = 0;
    midiSongRepeat = 0;
    LoadPatternToUi(0);
    factoryPatternsFound = 0;
    for(int pattern = 0; pattern < FACTORY_PATTERN_COUNT; ++pattern)
    {
        bool hasHit = false;
        for(int track = 0; track < 16 && !hasHit; ++track)
            for(int step = 0; step < 16; ++step)
                if(sequencer.getStep(pattern, track, step))
                {
                    hasHit = true;
                    break;
                }
        if(hasHit) factoryPatternsFound++;
    }
    patternBankReady = factoryPatternsFound == FACTORY_PATTERN_COUNT;
    daisyUsb.begin();
}

void control_process()
{
    daisyUsb.process();
    const auto& transport = daisyUsb.state();
    p4.master_connected = transport.engine_responding;
    if(!transport.engine_responding && engineWasConnected)
        lastPodStateRevision = 0xFFFFFFFFu;
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
    if(transport.pod.revision != lastPodStateRevision
       && transport.pod.config.version == POD_CONFIG_VERSION)
    {
        lastPodStateRevision = transport.pod.revision;
        p4.master_volume = transport.pod.masterVolume;
        p4.seq_volume = transport.pod.seqVolume;
        p4.live_volume = transport.pod.liveVolume;
        p4.enc_value[0] = transport.pod.flangerDepthValue;
        p4.enc_muted[0] = (transport.pod.fxActiveBits & 4u) == 0;
        p4.enc_value[1] = transport.pod.delayMixValue;
        p4.enc_muted[1] = (transport.pod.fxActiveBits & 1u) == 0;
        p4.enc_value[2] = transport.pod.reverbMixValue;
        p4.enc_muted[2] = (transport.pod.fxActiveBits & 2u) == 0;
        p4.pot_value[3] = transport.pod.wavefolderValue;
        p4.pot_muted[0] = (transport.pod.fxActiveBits & 16u) == 0;
        p4.pot_value[1] = transport.pod.crushValue;
        p4.pot_muted[1] = (transport.pod.fxActiveBits & 32u) == 0;
        p4.pot_value[2] = transport.pod.phaserDepthValue;
        p4.pot_muted[2] = (transport.pod.fxActiveBits & 8u) == 0;
        p4.filter_type = transport.pod.filterType;
        p4.cutoff_hz = transport.pod.cutoffHz;
        p4.resonance_x10 = transport.pod.resonanceX10;
        p4.distortion_pct = transport.pod.distortionPct;
        p4.bitcrush_bits = transport.pod.bitDepth;
        p4.sample_rate_hz = transport.pod.sampleRateHz;
        p4.bpm_int = transport.pod.bpmX10 / 10;
        p4.bpm_frac = transport.pod.bpmX10 % 10;
        p4.is_playing = transport.pod.playing != 0;
        SequencerInstance().setTempo(transport.pod.bpmX10 / 10.0f);
        if(p4.is_playing && !SequencerInstance().isPlaying())
            SequencerInstance().start();
        else if(!p4.is_playing && SequencerInstance().isPlaying())
            SequencerInstance().stop();
        fxDirty.store(true, std::memory_order_release);
    }
    // Daisy only exposes its 20 resident slots. They are not the P4 logical
    // pattern numbers (1..128). A command initiated by P4 is acknowledged by
    // matching the expected resident slot; it must never be reinterpreted as
    // a logical pattern.
    if(expectedDaisyPattern != 0xFF && transport.engine_responding)
    {
        if(transport.pattern == expectedDaisyPattern)
            expectedDaisyPattern = 0xFF;
        else if(millis() - expectedDaisyPatternSinceMs > 1500u)
            expectedDaisyPattern = 0xFF;
    }

    if(midiSongEngaged && transport.engine_responding
       && transport.pattern < midiSongPatternCount)
    {
        // Daisy owns the arrangement clock; P4 follows whichever scene it is
        // playing so the grid on screen is always the bar you are hearing.
        activeDaisyPattern = transport.pattern;
        expectedDaisyPattern = 0xFF;
        midiSongIndex = transport.pod.songIndex;
        midiSongRepeat = transport.pod.songRepeat;
        // The engine drops out of song mode by itself when a non-looping
        // arrangement reaches its last bar. Follow it instead of leaving the
        // header claiming SONG is still running.
        if(transport.pod.config.version == POD_CONFIG_VERSION
           && transport.pod.songLength > 0
           && (transport.pod.songFlags & 0x01u) == 0
           && !transport.playing)
            midiSongEngaged = false;
        const int logicalPattern = midiSongLogicalPattern[transport.pattern];
        if(p4.current_pattern != logicalPattern)
        {
            SequencerInstance().selectPattern(logicalPattern);
            LoadPatternToUi(logicalPattern);
            ui_sequencer_sync_from_current_pattern();
        }
    }
    else if(transport.engine_responding && queuedLogicalPattern >= 0
       && transport.pattern == queuedDaisyPattern)
    {
        activeDaisyPattern = queuedDaisyPattern;
        SequencerInstance().selectPattern(queuedLogicalPattern);
        LoadPatternToUi(queuedLogicalPattern);
        ui_sequencer_sync_from_current_pattern();
        ApplyPatternPerformance(queuedLogicalPattern);
        ui_pattern_queue_committed(queuedLogicalPattern);
        queuedLogicalPattern = -1;
        queuedDaisyPattern = 0xFF;
        expectedDaisyPattern = 0xFF;
    }
    else if(!midiSongEngaged && transport.engine_responding && engineWasConnected
            && queuedLogicalPattern < 0
            && expectedDaisyPattern == 0xFF
            && transport.pattern != activeDaisyPattern)
    {
        // This is a physical Daisy pattern control. Translate the movement of
        // the 20-slot ring into a relative change in the 128-pattern P4 bank,
        // then immediately populate the new Daisy slot with that logical
        // pattern. This preserves Pattern 1 and also handles wrap-around.
        int delta = static_cast<int>(transport.pattern)
                  - static_cast<int>(activeDaisyPattern);
        if(delta > 10) delta -= 20;
        if(delta < -10) delta += 20;
        int logicalPattern = (p4.current_pattern + delta) % MAX_PATTERNS;
        if(logicalPattern < 0) logicalPattern += MAX_PATTERNS;
        activeDaisyPattern = transport.pattern;
        SequencerInstance().selectPattern(logicalPattern);
        LoadPatternToUi(logicalPattern);
        ui_sequencer_sync_from_current_pattern();
        UploadPattern(activeDaisyPattern, logicalPattern);
        ApplyPatternPerformance(logicalPattern);
    }
    if(transport.engine_responding && !engineWasConnected)
    {
        SendCurrentState();
        PodConfigPayload savedConfig{};
        if(!pod_config_store_load(savedConfig))
        {
            pod_config_store_factory_defaults(savedConfig);
            pod_config_store_save(savedConfig);
        }
        SendWithRetry(CMD_POD_SET_CONFIG, &savedConfig, sizeof(savedConfig));
    }
    engineWasConnected = transport.engine_responding;
    SequencerInstance().update();
}

bool control_available() { return daisyUsb.connected(); }
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
    if(midiSongEngaged)
    {
        SequencerInstance().songChainPlay();
        daisyUsb.controlSong(1, midiSongLoop.load(std::memory_order_acquire));
    }
    else
    {
        SequencerInstance().start();
        daisyUsb.start();
    }
    p4.is_playing = true;
}

void control_send_stop()
{
    SequencerInstance().stop();
    if(midiSongEngaged)
    {
        SequencerInstance().songChainStop();
        daisyUsb.controlSong(0);
    }
    p4.is_playing = false;
    daisyUsb.stop();
}

void control_send_tempo(float bpm)
{
    if(i2c_rotaries_owns_function(POD_FUNC_TEMPO)
       && !i2c_rotaries_is_applying())
        return;
    bpm = Clamp(bpm, 40.0f, 240.0f);
    SequencerInstance().setTempo(bpm);
    p4.bpm_int = static_cast<int>(bpm);
    p4.bpm_frac = static_cast<int>((bpm - p4.bpm_int) * 10.0f + 0.5f);
    daisyUsb.setTempo(bpm);
}

void control_send_select_pattern(int index)
{
    // Stepping out to a single pattern used to destroy the whole imported
    // arrangement, so SONG could only ever be used once per import. Leave song
    // mode instead; the chain stays resident and SONG re-arms it.
    control_song_set_engaged(false);
    index = Clamp(index, 0, MAX_PATTERNS - 1);
    SequencerInstance().selectPattern(index);
    LoadPatternToUi(index);
    activeDaisyPattern = DefaultDaisyPattern(index);
    queuedLogicalPattern = -1;
    queuedDaisyPattern = 0xFF;
    UploadPattern(activeDaisyPattern, index);
    ApplyPatternPerformance(index);
    expectedDaisyPattern = activeDaisyPattern;
    expectedDaisyPatternSinceMs = millis();
    daisyUsb.selectPattern(activeDaisyPattern);
}

void control_send_queue_pattern(int index)
{
    control_song_set_engaged(false);
    queuedLogicalPattern = Clamp(index, 0, MAX_PATTERNS - 1);
    queuedDaisyPattern = static_cast<uint8_t>((activeDaisyPattern + 1u) % 20u);
    UploadPattern(queuedDaisyPattern, queuedLogicalPattern);
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
    if(control_apply_sequencer_variation(SEQ_VAR_GHOST_GROOVE))
        control_sync_current_pattern();
}

bool control_apply_sequencer_variation(uint8_t variation)
{
    if(variation < SEQ_VAR_FIRST || variation > SEQ_VAR_UNDO)
        return false;

    Sequencer& sequencer = SequencerInstance();
    const int pattern = Clamp(p4.current_pattern, 0, MAX_PATTERNS - 1);
    bool active[16][16] = {};
    uint8_t velocity[16][16] = {};
    uint8_t probability[16][16] = {};
    uint8_t ratchet[16][16] = {};

    for(int track = 0; track < 16; ++track)
    {
        for(int step = 0; step < 16; ++step)
        {
            active[track][step] = sequencer.getStep(pattern, track, step);
            velocity[track][step] = sequencer.getStepVelocity(pattern, track, step);
            probability[track][step] = sequencer.getStepProbability(pattern, track, step);
            ratchet[track][step] = Clamp<uint8_t>(
                sequencer.getStepRatchet(pattern, track, step), 1, 4);
        }
    }

    if(variation == SEQ_VAR_UNDO)
    {
        if(!variationBackup.valid || variationBackup.pattern != pattern)
            return false;
        memcpy(active, variationBackup.active, sizeof(active));
        memcpy(velocity, variationBackup.velocity, sizeof(velocity));
        memcpy(probability, variationBackup.probability, sizeof(probability));
        memcpy(ratchet, variationBackup.ratchet, sizeof(ratchet));
    }
    else
    {
        variationBackup.valid = true;
        variationBackup.pattern = pattern;
        memcpy(variationBackup.active, active, sizeof(active));
        memcpy(variationBackup.velocity, velocity, sizeof(velocity));
        memcpy(variationBackup.probability, probability, sizeof(probability));
        memcpy(variationBackup.ratchet, ratchet, sizeof(ratchet));

        // Every variation below is a transformation of whatever the pattern
        // already contains. The previous set stamped fixed hits onto fixed
        // track numbers, so applying one to an imported or user-built pattern
        // overwrote the groove with somebody else's beat and produced the same
        // result no matter what was underneath.
        //
        // Track roles follow the canonical RED808 layout that both the factory
        // bank and the MIDI importer target.
        constexpr int TRK_BD = 0, TRK_SD = 1, TRK_CH = 2, TRK_OH = 3;

        auto clearStep = [&](int track, int step)
        {
            active[track][step] = false;
        };

        auto placeHit = [&](int track, int step, uint8_t vel,
                            uint8_t prob, uint8_t rats)
        {
            if(track < 0 || track >= 16 || step < 0 || step >= 16) return;
            active[track][step] = true;
            velocity[track][step] = Clamp<uint8_t>(vel, 1, 127);
            probability[track][step] = Clamp<uint8_t>(prob, 1, 100);
            ratchet[track][step] = Clamp<uint8_t>(rats, 1, 4);
        };

        auto trackHitCount = [&](int track)
        {
            int count = 0;
            for(int step = 0; step < 16; ++step) if(active[track][step]) ++count;
            return count;
        };

        // Typical velocity of a track, used so generated hits sit at the same
        // level as the material around them instead of a hardcoded 104.
        auto trackVelocity = [&](int track, uint8_t fallback)
        {
            int total = 0, count = 0;
            for(int step = 0; step < 16; ++step)
                if(active[track][step] && velocity[track][step] > 0)
                {
                    total += velocity[track][step];
                    ++count;
                }
            return count > 0 ? static_cast<uint8_t>(total / count) : fallback;
        };

        // The busiest percussion track: the one a hat-style transform should
        // act on, whatever the kit assignment happens to be.
        auto busiestPercussionTrack = [&]()
        {
            int best = -1, bestCount = 0;
            for(int track = TRK_CH; track < 16; ++track)
            {
                const int count = trackHitCount(track);
                if(count > bestCount) { bestCount = count; best = track; }
            }
            return bestCount >= 4 ? best : -1;
        };

        auto rotateTrack = [&](int track, int by)
        {
            bool a[16]; uint8_t v[16], pr[16], rt[16];
            for(int step = 0; step < 16; ++step)
            {
                const int src = ((step - by) % 16 + 16) % 16;
                a[step] = active[track][src];
                v[step] = velocity[track][src];
                pr[step] = probability[track][src];
                rt[step] = ratchet[track][src];
            }
            for(int step = 0; step < 16; ++step)
            {
                active[track][step] = a[step];
                velocity[track][step] = v[step];
                probability[track][step] = pr[step];
                ratchet[track][step] = rt[step];
            }
        };

        auto swapSteps = [&](int track, int a, int b)
        {
            bool ta = active[track][a];
            active[track][a] = active[track][b];
            active[track][b] = ta;
            uint8_t tv = velocity[track][a];
            velocity[track][a] = velocity[track][b];
            velocity[track][b] = tv;
            uint8_t tp = probability[track][a];
            probability[track][a] = probability[track][b];
            probability[track][b] = tp;
            uint8_t tr = ratchet[track][a];
            ratchet[track][a] = ratchet[track][b];
            ratchet[track][b] = tr;
        };

        // Bjorklund-style Euclidean distribution of `pulses` over 16 steps.
        auto euclidHasPulse = [](int pulses, int step)
        {
            if(pulses <= 0) return false;
            if(pulses >= 16) return true;
            return ((step * pulses) % 16) < pulses;
        };

        // Deterministic per-pattern jitter: applying the same variation to the
        // same pattern twice must produce the same result, and UNDO has to be
        // able to reverse exactly one application.
        uint32_t rngState = 0x9E3779B9u ^ (static_cast<uint32_t>(pattern) * 2654435761u)
                          ^ (static_cast<uint32_t>(variation) * 40503u);
        auto nextRandom = [&](int modulo)
        {
            rngState ^= rngState << 13;
            rngState ^= rngState >> 17;
            rngState ^= rngState << 5;
            return modulo > 0 ? static_cast<int>(rngState % static_cast<uint32_t>(modulo)) : 0;
        };

        switch(variation)
        {
            case SEQ_VAR_GHOST_GROOVE:
            {
                // Add quiet, low-probability notes in the gaps of the tracks
                // that already carry the groove. Nothing existing is removed.
                for(int track = TRK_SD; track < 16; ++track)
                {
                    const int hits = trackHitCount(track);
                    if(hits == 0 || hits > 10) continue;
                    const uint8_t base = trackVelocity(track, 96);
                    for(int step = 0; step < 16; ++step)
                    {
                        if(active[track][step]) continue;
                        // Only in the sixteenth right before an existing hit:
                        // that is where a ghost note reads as a flam and not
                        // as a new part.
                        const int nextStep = (step + 1) & 15;
                        if(!active[track][nextStep]) continue;
                        if((step & 1) == 0) continue;
                        placeHit(track, step,
                                 static_cast<uint8_t>(Clamp<int>(base / 3, 8, 55)),
                                 static_cast<uint8_t>(55 + nextRandom(25)), 1);
                    }
                }
                break;
            }

            case SEQ_VAR_ROTATE:
            {
                // Push every percussion track three sixteenths later while the
                // kick and snare stay put, so the backbeat survives but the
                // pattern lands somewhere new.
                for(int track = TRK_CH; track < 16; ++track)
                    if(trackHitCount(track) > 0) rotateTrack(track, 3);
                break;
            }

            case SEQ_VAR_HALF_TIME:
            {
                // Stretch the first eight steps over the whole bar. That is the
                // actual definition of half time, and it keeps the source
                // material recognisable.
                for(int track = 0; track < 16; ++track)
                {
                    bool a[16] = {}; uint8_t v[16] = {}, pr[16] = {}, rt[16] = {};
                    for(int step = 0; step < 8; ++step)
                    {
                        a[step * 2] = active[track][step];
                        v[step * 2] = velocity[track][step];
                        pr[step * 2] = probability[track][step];
                        rt[step * 2] = ratchet[track][step];
                    }
                    for(int step = 0; step < 16; ++step)
                    {
                        active[track][step] = a[step];
                        velocity[track][step] = v[step] ? v[step] : 100u;
                        probability[track][step] = pr[step] ? pr[step] : 100u;
                        ratchet[track][step] = rt[step] ? rt[step] : 1u;
                    }
                }
                break;
            }

            case SEQ_VAR_DOUBLE_TIME:
            {
                // The inverse: fold the bar in half and play it twice.
                for(int track = 0; track < 16; ++track)
                {
                    bool a[16] = {}; uint8_t v[16] = {}, pr[16] = {}, rt[16] = {};
                    for(int step = 0; step < 16; ++step)
                    {
                        const int src = (step % 8) * 2;
                        a[step] = active[track][src];
                        v[step] = velocity[track][src];
                        pr[step] = probability[track][src];
                        rt[step] = ratchet[track][src];
                    }
                    for(int step = 0; step < 16; ++step)
                    {
                        active[track][step] = a[step];
                        velocity[track][step] = v[step] ? v[step] : 100u;
                        probability[track][step] = pr[step] ? pr[step] : 100u;
                        ratchet[track][step] = rt[step] ? rt[step] : 1u;
                    }
                }
                break;
            }

            case SEQ_VAR_SWAP_HALVES:
            {
                // Swap beats 1-2 with beats 3-4 on the percussion layer. The
                // old MIRROR reflected the bar around its centre, which moved
                // the downbeat to step 15 and destroyed the pulse.
                for(int track = TRK_CH; track < 16; ++track)
                    for(int step = 0; step < 8; ++step)
                        swapSteps(track, step, step + 8);
                break;
            }

            case SEQ_VAR_EUCLID:
            {
                // Redistribute each track's own hit count evenly across the
                // bar. The density of the pattern is preserved; only the
                // placement changes.
                for(int track = TRK_CH; track < 16; ++track)
                {
                    const int hits = trackHitCount(track);
                    if(hits < 2 || hits > 12) continue;
                    const uint8_t vel = trackVelocity(track, 100);
                    const uint8_t prob = probability[track][0] ? probability[track][0] : 100u;
                    for(int step = 0; step < 16; ++step) clearStep(track, step);
                    for(int step = 0; step < 16; ++step)
                        if(euclidHasPulse(hits, step))
                            placeHit(track, step,
                                     (step % 4) == 0
                                        ? static_cast<uint8_t>(Clamp<int>(vel + 14, 1, 127))
                                        : vel,
                                     prob, 1);
                }
                break;
            }

            case SEQ_VAR_ACCENT_GROOVE:
            {
                // Re-articulate the dynamics of what is already there: strong
                // downbeats, softer offbeats, and a lift into the next bar.
                for(int track = 0; track < 16; ++track)
                {
                    for(int step = 0; step < 16; ++step)
                    {
                        if(!active[track][step]) continue;
                        const uint8_t base = velocity[track][step]
                                           ? velocity[track][step] : 100u;
                        int shaped = base;
                        if((step & 3) == 0)      shaped = base + 16;   // beat
                        else if((step & 1) == 0) shaped = base - 4;    // eighth
                        else                     shaped = base - 18;   // sixteenth
                        if(step == 15) shaped = base + 10;             // pickup
                        velocity[track][step] =
                            static_cast<uint8_t>(Clamp<int>(shaped, 12, 127));
                    }
                }
                break;
            }

            case SEQ_VAR_RATCHET_STORM:
            {
                // Turn the hits that already fall in the last beat into rolls,
                // building towards the bar line, instead of replacing the hats
                // with a generated pattern.
                for(int track = TRK_CH; track < 16; ++track)
                {
                    for(int step = 12; step < 16; ++step)
                    {
                        if(!active[track][step]) continue;
                        ratchet[track][step] =
                            static_cast<uint8_t>(Clamp<int>(step - 10, 2, 4));
                    }
                }
                // If nothing lived in the last beat there is nothing to roll,
                // so seed the busiest percussion track with one.
                bool anyRatchet = false;
                for(int track = TRK_CH; track < 16 && !anyRatchet; ++track)
                    for(int step = 12; step < 16; ++step)
                        if(active[track][step] && ratchet[track][step] > 1)
                        { anyRatchet = true; break; }
                if(!anyRatchet)
                {
                    const int track = busiestPercussionTrack();
                    if(track >= 0)
                        placeHit(track, 15, trackVelocity(track, 104), 100, 3);
                }
                break;
            }

            case SEQ_VAR_SIXTEENTH_LIFT:
            {
                // Fill the offbeat sixteenths of the busiest percussion track,
                // keeping its existing hits and its own velocity level.
                const int track = busiestPercussionTrack();
                if(track < 0) break;
                const uint8_t vel = trackVelocity(track, 96);
                for(int step = 1; step < 16; step += 2)
                    if(!active[track][step])
                        placeHit(track, step,
                                 static_cast<uint8_t>(Clamp<int>(vel - 26, 12, 127)),
                                 100, 1);
                break;
            }

            case SEQ_VAR_THIN_OUT:
            {
                // Open up space by removing offbeat percussion hits, never the
                // downbeats and never the kick or snare.
                for(int track = TRK_CH; track < 16; ++track)
                {
                    if(trackHitCount(track) < 3) continue;
                    for(int step = 0; step < 16; ++step)
                    {
                        if(!active[track][step]) continue;
                        if((step & 3) == 0) continue;
                        if(nextRandom(100) < 55) clearStep(track, step);
                    }
                }
                break;
            }

            case SEQ_VAR_DICE_PROBABILITY:
            {
                // Leave the notes alone and make the bar breathe: offbeat hits
                // become probabilistic, downbeats stay certain.
                for(int track = 0; track < 16; ++track)
                {
                    for(int step = 0; step < 16; ++step)
                    {
                        if(!active[track][step]) continue;
                        if(track <= TRK_SD && (step & 3) == 0)
                        {
                            probability[track][step] = 100;
                            continue;
                        }
                        probability[track][step] = (step & 3) == 0
                            ? static_cast<uint8_t>(88 + nextRandom(12))
                            : static_cast<uint8_t>(45 + nextRandom(40));
                    }
                }
                break;
            }

            case SEQ_VAR_BREAK:
            {
                // A classic break: strip the first beat of the percussion layer
                // and answer it with the material from the last beat, moved
                // one sixteenth earlier so it lands syncopated.
                for(int track = TRK_CH; track < 16; ++track)
                {
                    if(trackHitCount(track) == 0) continue;
                    for(int step = 0; step < 4; ++step) clearStep(track, step);
                    for(int step = 12; step < 16; ++step)
                    {
                        if(!active[track][step]) continue;
                        placeHit(track, step - 12 + 1,
                                 velocity[track][step] ? velocity[track][step]
                                                       : trackVelocity(track, 100),
                                 probability[track][step] ? probability[track][step] : 100u,
                                 ratchet[track][step]);
                    }
                }
                // Keep the pulse readable: the kick still marks the downbeat.
                if(!active[TRK_BD][0])
                    placeHit(TRK_BD, 0, trackVelocity(TRK_BD, 118), 100, 1);
                // Close the break with a roll on the open hat if it is playing.
                if(trackHitCount(TRK_OH) > 0)
                    placeHit(TRK_OH, 15, trackVelocity(TRK_OH, 110), 100, 2);
                break;
            }

            default:
                return false;
        }
    }

    bool changed = false;
    for(int track = 0; track < 16; ++track)
    {
        for(int step = 0; step < 16; ++step)
        {
            const bool oldActive = sequencer.getStep(pattern, track, step);
            const uint8_t oldVelocity = sequencer.getStepVelocity(pattern, track, step);
            const uint8_t oldProbability = sequencer.getStepProbability(pattern, track, step);
            const uint8_t oldRatchet = Clamp<uint8_t>(
                sequencer.getStepRatchet(pattern, track, step), 1, 4);
            if(oldActive == active[track][step]
               && oldVelocity == velocity[track][step]
               && oldProbability == probability[track][step]
               && oldRatchet == ratchet[track][step])
            {
                p4.steps[track][step] = active[track][step];
                continue;
            }

            sequencer.setStep(pattern, track, step, active[track][step],
                              velocity[track][step]);
            sequencer.setStepProbability(pattern, track, step,
                                         probability[track][step]);
            sequencer.setStepRatchet(pattern, track, step, ratchet[track][step]);
            p4.steps[track][step] = active[track][step];
            changed = true;

        }
    }

    if(variation == SEQ_VAR_UNDO && changed) variationBackup.valid = false;
    if(!changed && variation != SEQ_VAR_UNDO) variationBackup.valid = false;
    return changed;
}

bool control_variation_can_undo()
{
    return variationBackup.valid
        && variationBackup.pattern == Clamp(p4.current_pattern, 0, MAX_PATTERNS - 1);
}

bool control_sync_current_pattern()
{
    if(!control_available()) return false;
    return UploadPattern(activeDaisyPattern,
                         Clamp(p4.current_pattern, 0, MAX_PATTERNS - 1));
}

bool control_patterns_ready() { return patternBankReady; }
uint8_t control_factory_patterns_found() { return factoryPatternsFound; }
uint8_t control_factory_patterns_expected() { return FACTORY_PATTERN_COUNT; }

void control_send_build4()
{
    for(int step = 12; step < 16; ++step)
        control_send_set_step(1, step, true);
}

void control_send_drop()
{
    // DROP used to mute tracks 2..15 and leave them muted: the button could
    // only ever be pressed once, and the only way back was to unmute fourteen
    // tracks by hand. It is a toggle, and it remembers what was already muted
    // so releasing the drop restores the mix the user had set up.
    static bool dropActive = false;
    static uint16_t preDropMuteMask = 0;

    if(!dropActive)
    {
        preDropMuteMask = 0;
        for(int track = 0; track < 16; ++track)
            if(p4.track_muted[track]) preDropMuteMask |= (uint16_t)(1u << track);
        for(int track = 2; track < 16; ++track)
            control_send_mute(track, true);
        dropActive = true;
        return;
    }

    for(int track = 2; track < 16; ++track)
        control_send_mute(track, (preDropMuteMask & (1u << track)) != 0);
    dropActive = false;
}

bool control_drop_active()
{
    // Tracks 2..15 all muted is the state DROP leaves behind, whichever way
    // it was reached.
    for(int track = 2; track < 16; ++track)
        if(!p4.track_muted[track]) return false;
    return true;
}

void control_send_launch_demo_set()
{
    control_send_select_pattern(0);
    control_send_start();
}

void control_send_get_pattern(int pattern) { LoadPatternToUi(pattern); }

bool control_save_user_pattern(int source, int destination)
{
    return pattern_store_save_user(SequencerInstance(), source, destination);
}

bool control_user_pattern_is_saved(int pattern)
{
    return pattern_store_is_saved(pattern);
}

void control_set_pattern_source_tempo(int pattern, float bpm, const char* name)
{
    pattern = Clamp(pattern, 0, MAX_PATTERNS - 1);
    PatternMetadata metadata{};
    SequencerInstance().getPatternMetadata(pattern, metadata);
    if(name && name[0] != '\0')
        snprintf(metadata.name, sizeof(metadata.name), "MIDI %.26s", name);
    strncpy(metadata.genre, "MIDI", sizeof(metadata.genre) - 1);
    metadata.recommendedBpm = isfinite(bpm) && bpm >= 30.0f && bpm <= 300.0f
        ? static_cast<uint16_t>(lroundf(bpm)) : 0u;
    SequencerInstance().setPatternMetadata(pattern, metadata);
    if(pattern == p4.current_pattern) SetOriginalTempo(bpm);
}

bool control_install_midi_song(const mem_midi::MidiSongData& song)
{
    if(song.pattern_count == 0
       || song.pattern_count > mem_midi::MIDI_SONG_MAX_PATTERNS
       || song.chain_count == 0
       || song.chain_count > mem_midi::MIDI_SONG_MAX_CHAIN)
        return false;

    Sequencer& sequencer = SequencerInstance();
    sequencer.stop();
    sequencer.songChainStop();
    daisyUsb.controlSong(2);
    daisyUsb.stop();
    p4.is_playing = false;
    midiSongPrepared = false;
    midiSongPatternCount = song.pattern_count;
    midiSongChainCount = song.chain_count;

    bool bulkSteps[MAX_TRACKS][STEPS_PER_PATTERN] = {};
    uint8_t bulkVelocity[MAX_TRACKS][STEPS_PER_PATTERN] = {};
    bool persisted = true;
    for(uint8_t index = 0; index < song.pattern_count; ++index)
    {
        memset(bulkSteps, 0, sizeof(bulkSteps));
        memset(bulkVelocity, 0, sizeof(bulkVelocity));
        for(uint8_t track = 0; track < 16; ++track)
        {
            for(uint8_t step = 0; step < 16; ++step)
            {
                bulkSteps[track][step] = song.patterns[index].steps[track][step];
                bulkVelocity[track][step] = song.patterns[index].velocity[track][step];
            }
        }

        const int logicalPattern = USER_PATTERN_FIRST + index;
        midiSongLogicalPattern[index] = static_cast<uint8_t>(logicalPattern);
        sequencer.setPatternBulk(logicalPattern, bulkSteps, bulkVelocity);

        PatternMetadata metadata{};
        snprintf(metadata.name, sizeof(metadata.name), "MIDI %.19s %02u",
                 song.name[0] ? song.name : "SONG", (unsigned)(index + 1));
        strncpy(metadata.genre, "MIDI SONG", sizeof(metadata.genre) - 1);
        strncpy(metadata.kit, p4.kit_name, sizeof(metadata.kit) - 1);
        metadata.recommendedBpm = song.bpm >= 30.0f && song.bpm <= 300.0f
            ? static_cast<uint16_t>(lroundf(song.bpm)) : 0u;
        metadata.swing = 0;
        metadata.humanizeTimingMs = 0;
        metadata.humanizeVelocity = 0;
        sequencer.setPatternMetadata(logicalPattern, metadata);

        BuiltinPatternSoundProfile samplerProfile{};
        for(uint8_t track = 0; track < MAX_TRACKS; ++track)
            samplerProfile.engines[track] = -1;
        setPatternSoundProfile(logicalPattern, samplerProfile);
        if(!pattern_store_save_user(sequencer, logicalPattern, logicalPattern))
            persisted = false;
    }

    Sequencer::SongChainEntry localChain[mem_midi::MIDI_SONG_MAX_CHAIN] = {};
    for(uint8_t index = 0; index < song.chain_count; ++index)
    {
        if(song.chain[index].pattern >= song.pattern_count) return false;
        localChain[index].pattern
            = midiSongLogicalPattern[song.chain[index].pattern];
        localChain[index].repeats = song.chain[index].repeats == 0
            ? 1u : song.chain[index].repeats;
        midiSongDaisyChain[index].pattern = song.chain[index].pattern;
        midiSongDaisyChain[index].repeats = localChain[index].repeats;
    }
    sequencer.songChainUpload(localChain, song.chain_count);

    const int firstPattern = midiSongLogicalPattern[0];
    sequencer.selectPattern(firstPattern);
    LoadPatternToUi(firstPattern);
    SetOriginalTempo(song.bpm);
    midiSongPrepared = true;
    midiSongEngaged = true;
    midiSongPersisted = persisted;
    midiSongIndex = 0;
    midiSongRepeat = 0;
    queuedLogicalPattern = -1;
    queuedDaisyPattern = 0xFF;
    activeDaisyPattern = 0;

    if(song.bpm >= 40.0f && song.bpm <= 240.0f
       && !i2c_rotaries_owns_function(POD_FUNC_TEMPO))
        control_send_tempo(song.bpm);
    if(control_available()) UploadPreparedMidiSong();
    if(!persisted)
        log_w("[MIDI-SONG] ready in RAM, but one or more user patterns were not persisted");
    return true;
}

bool control_midi_song_ready()
{
    return midiSongPrepared;
}

bool control_song_available()
{
    return midiSongPrepared && midiSongChainCount > 0;
}

bool control_song_engaged()
{
    return midiSongEngaged;
}

bool control_song_loop()
{
    return midiSongLoop.load(std::memory_order_acquire);
}

void control_song_set_loop(bool loop)
{
    midiSongLoop.store(loop, std::memory_order_release);
    if(midiSongEngaged && control_available())
        daisyUsb.controlSong(p4.is_playing ? 1u : 2u, loop);
}

bool control_song_set_engaged(bool engaged)
{
    if(engaged && !control_song_available()) return false;
    if(midiSongEngaged == engaged) return engaged;

    midiSongEngaged = engaged;
    if(!engaged)
    {
        SequencerInstance().songChainStop();
        if(control_available()) daisyUsb.controlSong(0);
        return false;
    }

    // Re-arming has to re-upload: the resident Daisy slots may have been
    // overwritten by whatever single pattern the user was auditioning.
    SequencerInstance().songChainReset();
    const int firstPattern = midiSongLogicalPattern[0];
    SequencerInstance().selectPattern(firstPattern);
    LoadPatternToUi(firstPattern);
    ui_sequencer_sync_from_current_pattern();
    ApplyPatternPerformance(firstPattern);
    if(control_available())
    {
        UploadPreparedMidiSong();
        if(p4.is_playing)
        {
            SequencerInstance().songChainPlay();
            daisyUsb.controlSong(1, midiSongLoop.load(std::memory_order_acquire));
        }
    }
    return true;
}

void control_song_status(uint8_t* index, uint8_t* count, uint8_t* pattern)
{
    const uint8_t chain = midiSongChainCount;
    const uint8_t idx = midiSongIndex < chain ? midiSongIndex : 0u;
    if(index) *index = idx;
    if(count) *count = chain;
    if(pattern)
    {
        const uint8_t resident = chain > 0 ? midiSongDaisyChain[idx].pattern : 0u;
        *pattern = resident < midiSongPatternCount
            ? midiSongLogicalPattern[resident] : 0u;
    }
}

bool control_midi_song_persisted()
{
    return midiSongPrepared && midiSongPersisted;
}

void control_cancel_midi_song()
{
    if(!midiSongPrepared) return;
    SequencerInstance().songChainStop();
    SequencerInstance().songChainReset();
    daisyUsb.controlSong(2);
    midiSongPrepared = false;
    midiSongEngaged = false;
    midiSongPersisted = false;
    midiSongPatternCount = 0;
    midiSongChainCount = 0;
    midiSongIndex = 0;
    midiSongRepeat = 0;
}

void control_send_unload_daisy(uint8_t pad)
{
    daisyUsb.sendU8(CMD_SAMPLE_UNLOAD, pad);
    if(pad < 24) p4.sample_loaded[pad] = false;
}

void control_send_set_step(int track, int step, bool active)
{
    if(track < 0 || track >= 16 || step < 0 || step >= 16) return;
    uint8_t velocity = SequencerInstance().getStepVelocity(
        p4.current_pattern, track, step);
    if(velocity == 0) velocity = 100;
    SequencerInstance().setStep(
        p4.current_pattern, track, step, active, velocity);
    p4.steps[track][step] = active;
    daisyUsb.setStep(activeDaisyPattern, track, step, active, velocity);
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
    SequencerInstance().muteTrack(track, muted);
    daisyUsb.setTrackMute(track, muted);
}

void control_send_set_volume(int value)
{
    if(i2c_rotaries_owns_function(POD_FUNC_MASTER_VOLUME)
       && !i2c_rotaries_is_applying())
        return;
    p4.master_volume = Clamp(value, 0, 150);
    daisyUsb.sendU8(CMD_MASTER_VOLUME, p4.master_volume);
}

void control_send_set_seq_volume(int value)
{
    if(i2c_rotaries_owns_function(POD_FUNC_SEQ_VOLUME)
       && !i2c_rotaries_is_applying())
        return;
    p4.seq_volume = Clamp(value, 0, 150);
    daisyUsb.sendU8(CMD_SEQ_VOLUME, p4.seq_volume);
}

void control_send_set_live_volume(int value)
{
    if(i2c_rotaries_owns_function(POD_FUNC_LIVE_VOLUME)
       && !i2c_rotaries_is_applying())
        return;
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
    if(i2c_rotaries_owns_function(POD_FUNC_FILTER_TYPE)
       && !i2c_rotaries_is_applying()) return;
    const int previous = p4.filter_type;
    p4.filter_type = Clamp(type, 0, 14);
    // A model is inaudible while CUTOFF sits at whichever end of the sweep is
    // transparent for it: a low-pass at 20 kHz, a high-pass at 20 Hz, a
    // band-pass parked at either extreme. Selecting a filter and hearing
    // nothing reads as a broken effect, so park the cutoff somewhere musical
    // the first time a model is engaged. An explicit cutoff the user already
    // dialled in is left alone.
    if(p4.filter_type != FTYPE_NONE && previous != p4.filter_type)
    {
        const bool parkedOpen   = p4.cutoff_hz >= 19000;
        const bool parkedClosed = p4.cutoff_hz <= 40;
        if(parkedOpen || parkedClosed)
        {
            int musical = 1000;
            switch(p4.filter_type)
            {
                case FTYPE_LOWPASS:
                case FTYPE_RESONANT:
                case FTYPE_LADDER:
                case FTYPE_SVF_LP:    musical = 6000; break;
                case FTYPE_HIGHPASS:
                case FTYPE_SVF_HP:    musical = 300;  break;
                case FTYPE_LOWSHELF:  musical = 220;  break;
                case FTYPE_HIGHSHELF: musical = 5000; break;
                case FTYPE_COMB:      musical = 220;  break;
                default:              musical = 1200; break;   // BP/notch/AP/peak
            }
            p4.cutoff_hz = musical;
        }
        // The three EQ models take their gain from RESO. At the bottom of its
        // range that is a full 18 dB cut, which sounds like the model muted the
        // mix rather than a neutral starting point.
        const bool isEq = p4.filter_type == FTYPE_PEAKING
                       || p4.filter_type == FTYPE_LOWSHELF
                       || p4.filter_type == FTYPE_HIGHSHELF;
        // RESO maps 0.7..20.0 onto -18..+18 dB for these three, so a useful
        // starting point is three quarters up the sweep.
        if(isEq && p4.resonance_x10 < 120) p4.resonance_x10 = 152;  // ≈ +9 dB
    }
    SendFilterState();
}

void control_send_set_filter_cutoff(int hz)
{
    if(i2c_rotaries_owns_function(POD_FUNC_FILTER_CUTOFF)
       && !i2c_rotaries_is_applying()) return;
    p4.cutoff_hz = Clamp(hz, 20, 20000);
    const float value = static_cast<float>(p4.cutoff_hz);
    daisyUsb.sendFloat(CMD_FILTER_CUTOFF, value);
    fxDirty.store(true, std::memory_order_release);
}

void control_send_set_filter_resonance(float value)
{
    if(i2c_rotaries_owns_function(POD_FUNC_FILTER_RESONANCE)
       && !i2c_rotaries_is_applying()) return;
    value = Clamp(value, 0.3f, 30.0f);
    p4.resonance_x10 = static_cast<int>(value * 10.0f + 0.5f);
    daisyUsb.sendFloat(CMD_FILTER_RESONANCE, value);
    fxDirty.store(true, std::memory_order_release);
}

void control_send_set_distortion(float value)
{
    if(i2c_rotaries_owns_function(POD_FUNC_DISTORTION)
       && !i2c_rotaries_is_applying()) return;
    p4.distortion_pct = static_cast<int>(Clamp(value, 0.0f, 1.0f) * 100.0f);
    const float percent = static_cast<float>(p4.distortion_pct);
    daisyUsb.sendFloat(CMD_FILTER_DISTORTION, percent);
    fxDirty.store(true, std::memory_order_release);
}

void control_send_set_bitcrush(int bits)
{
    if(i2c_rotaries_owns_function(POD_FUNC_BIT_DEPTH)
       && !i2c_rotaries_is_applying()) return;
    p4.bitcrush_bits = Clamp(bits, 4, 16);
    daisyUsb.sendU8(CMD_FILTER_BITDEPTH,
                    static_cast<uint8_t>(p4.bitcrush_bits));
    fxDirty.store(true, std::memory_order_release);
}

void control_send_set_sample_rate(int rate_hz)
{
    if(i2c_rotaries_owns_function(POD_FUNC_SAMPLE_RATE)
       && !i2c_rotaries_is_applying()) return;
    p4.sample_rate_hz = rate_hz <= 0 ? 0 : Clamp(rate_hz, 1000, 48000);
    const uint32_t rate = static_cast<uint32_t>(p4.sample_rate_hz);
    daisyUsb.send(CMD_FILTER_SR_REDUCE, &rate, sizeof(rate));
    fxDirty.store(true, std::memory_order_release);
}

void control_send_set_crush_macro(uint8_t value)
{
    if(i2c_rotaries_owns_function(POD_FUNC_CRUSH_MACRO)
       && !i2c_rotaries_is_applying()) return;
    value = static_cast<uint8_t>(Clamp(static_cast<int>(value), 0, 127));
    p4.pot_value[1] = value;
    if(value == 0)
    {
        control_send_set_bitcrush(16);
        control_send_set_sample_rate(0);
        return;
    }
    const float normalized = value / 127.0f;
    const int bits = Clamp(static_cast<int>(16.0f - normalized * 10.0f + 0.5f),
                           6, 16);
    const int rate = Clamp(static_cast<int>(42000.0f
        * powf(4000.0f / 42000.0f, normalized) + 0.5f), 4000, 42000);
    control_send_set_bitcrush(bits);
    control_send_set_sample_rate(rate);
}

void control_send_fx_enc(int encoder, uint8_t value, bool muted)
{
    if(encoder < 0 || encoder >= 3) return;
    const uint8_t ownedFunction = encoder == 0 ? POD_FUNC_FLANGER_DEPTH
                                  : encoder == 1 ? POD_FUNC_DELAY_MIX
                                  : encoder == 2 ? POD_FUNC_REVERB_MIX
                                                 : POD_FUNC_NONE;
    if(ownedFunction != POD_FUNC_NONE
       && i2c_rotaries_owns_function(ownedFunction)
       && !i2c_rotaries_is_applying())
        return;
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
    const uint8_t ownedFunction = pot == 0 ? POD_FUNC_WAVEFOLDER_GAIN
                                  : pot == 1 ? POD_FUNC_BIT_DEPTH
                                             : POD_FUNC_PHASER_DEPTH;
    if(i2c_rotaries_owns_function(ownedFunction)
       && !i2c_rotaries_is_applying()) return;
    if(pot == 0) p4.pot_value[3] = value;
    else p4.pot_value[pot] = value;
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
    {
        const bool muted = (mask & (1u << track)) != 0;
        p4.track_muted[track] = muted;
        SequencerInstance().muteTrack(track, muted);
    }
    daisyUsb.setTrackMuteMask(mask);
}

void control_send_solo_mask(uint16_t mask)
{
    for(int track = 0; track < 16; ++track)
        p4.track_solo[track] = (mask & (1u << track)) != 0;
    daisyUsb.setTrackSoloMask(mask);
}

void control_mark_fx_screen_dirty() { fxDirty.store(true, std::memory_order_release); }
bool control_consume_fx_screen_dirty()
{
    return fxDirty.exchange(false, std::memory_order_acq_rel);
}

bool control_pattern_track_uses_sampler(int pattern, int track)
{
    if(pattern < 0 || pattern >= MAX_PATTERNS || track < 0 || track >= 16)
        return false;
    BuiltinPatternSoundProfile profile{};
    return getBuiltinPatternSoundProfile(pattern, profile)
        && profile.engines[track] == -1;
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
        {
            const uint8_t velocity = SequencerInstance().getStepVelocity(
                pattern, track, step);
            SequencerInstance().setStep(
                pattern, track, step, steps[track][step], velocity);
        }
    activeDaisyPattern = DefaultDaisyPattern(pattern);
    UploadPattern(activeDaisyPattern, pattern);
    ApplyPatternPerformance(pattern);
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
