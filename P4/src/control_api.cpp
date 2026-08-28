#include "control_api.h"

#include "app_state.h"
#include "daisy_usb_transport.h"
#include "drivers/i2c_rotaries.h"
#include "master/PatternBank.h"
#include "master/Sequencer.h"
#include "pattern_store.h"
#include "mem_midi_loader.h"
#include "pod_config_store.h"
#include "midi_map_store.h"
#include "../../DaisyPod3/mpd218_mapping.h"
#include "ui/ui_screens.h"
#include "../include/ui_events.h"
#include <Arduino.h>
#include <atomic>
#include <ctype.h>
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
std::atomic<bool> midiSongPrepared{false};
std::atomic<bool> midiSongPersisted{false};
uint8_t midiSongPatternCount = 0;
uint8_t midiSongChainCount = 0;
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

// ── User MIDI map (AKAI MPD218 LEARN) ────────────────────────────────
// P4 owns the persisted map; DaisyPod3 receives a copy (CMD_MIDI_MAP_SET)
// so triggers keep their low-latency path on the audio engine. The UI
// (LVGL task) only reads snapshots or flips the atomics; the map itself is
// mutated from UI callbacks and re-uploaded whole, mirroring how the pod
// config flows through the system.
MidiMapEntry midiMap[MIDI_MAP_MAX_ENTRIES] = {};
uint8_t midiMapCount = 0;
std::atomic<bool> midiLearnArmed{false};
uint32_t midiLearnArmedSinceMs = 0; // loop-task only; guards the timeout below
std::atomic<uint32_t> midiLearnTimeoutRevision{0};
constexpr uint32_t kMidiLearnTimeoutMs = 8000;
std::atomic<uint32_t> midiCaptureRevision{0};
MidiLearnCapture midiCapture = {};
std::atomic<uint32_t> midiActivityRevision{0};
MidiMonitorEvent midiActivity = {};

void SendMidiMapToDaisy()
{
    daisyUsb.sendMidiMap(midiMap, midiMapCount);
}

int MidiMapIndexOf(uint8_t channel, uint8_t kind, uint8_t number)
{
    for(uint8_t i = 0; i < midiMapCount; ++i)
        if(midiMap[i].channel == channel && midiMap[i].kind == kind
           && midiMap[i].number == number)
            return i;
    return -1;
}

// Finds another learned entry that already points at the same target
// (kind+action, and arg0/arg1 too for pad actions that carry a slot
// number), excluding the (channel,kind,number) about to be overwritten.
// Two different physical pads/knobs firing the exact same thing is
// sometimes intentional, sometimes a stray double-assignment — worth a
// heads-up either way.
int MidiMapIndexOfDuplicateAction(uint8_t kind, uint8_t action, uint8_t arg0,
                                  uint8_t arg1, uint8_t excludeChannel,
                                  uint8_t excludeNumber)
{
    for(uint8_t i = 0; i < midiMapCount; ++i)
    {
        const MidiMapEntry& e = midiMap[i];
        if(e.channel == excludeChannel && e.kind == kind
           && e.number == excludeNumber)
            continue; // the slot being overwritten, not a collision
        if(e.kind != kind || e.action != action) continue;
        if(kind == MIDI_MAP_KIND_CC || (e.arg0 == arg0 && e.arg1 == arg1))
            return i;
    }
    return -1;
}

// Resolves which of the 16 visible pads a note event would light up,
// honoring the learned map first and the compiled MPD218 tables second.
int MidiNoteToUiPad(uint8_t channel, uint8_t note)
{
    using namespace red808_mpd218;
    uint8_t action = PAD_NONE, arg0 = 0, arg1 = 0;
    const int learned = MidiMapIndexOf(channel, MIDI_MAP_KIND_NOTE, note);
    if(learned >= 0)
    {
        action = midiMap[learned].action;
        arg0 = midiMap[learned].arg0;
        arg1 = midiMap[learned].arg1;
    }
    else
    {
        uint8_t device = 0, bank = 0, layer = 0, index = 0;
        if(!DecodePad(channel, note, device, bank, layer, index))
            return -1;
        const PadAction& pad = kPadMap[bank][layer][index];
        action = pad.type;
        arg0 = pad.arg0;
        arg1 = pad.arg1;
    }
    switch(action)
    {
        case PAD_TRIGGER_SAMPLE:
        case PAD_SELECT_TRACK:
        case PAD_TOGGLE_TRACK_MUTE:
            return arg0 < 16 ? arg0 : -1;
        case PAD_TRIGGER_SYNTH:
            return arg1 < 16 ? arg1 : -1;
        default:
            return -1;
    }
}

void ProcessMidiMonitor()
{
    MidiMonitorEvent event;
    while(daisyUsb.popMidiEvent(event))
    {
        const uint8_t type = event.status & 0xF0u;
        const uint8_t channel = event.status & 0x0Fu;
        midiActivity = event;
        midiActivityRevision.fetch_add(1, std::memory_order_release);

        const bool noteOn = type == 0x90u && event.data1 > 0;
        const bool cc = type == 0xB0u;
        if(midiLearnArmed.load(std::memory_order_acquire) && (noteOn || cc))
        {
            midiCapture.channel = channel;
            midiCapture.kind = noteOn ? MIDI_MAP_KIND_NOTE : MIDI_MAP_KIND_CC;
            midiCapture.number = event.data0;
            midiCapture.value = event.data1;
            midiLearnArmed.store(false, std::memory_order_release);
            midiCaptureRevision.fetch_add(1, std::memory_order_release);
            continue;
        }

        if(noteOn)
        {
            const int pad = MidiNoteToUiPad(channel, event.data0);
            if(pad >= 0)
                ui_external_pad_flash(static_cast<uint8_t>(pad), event.data1);
        }
    }
}

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
    daisyUsb.controlSong(2);
    activeDaisyPattern = 0;
    expectedDaisyPattern = 0;
    expectedDaisyPatternSinceMs = millis();
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

void PushSoloWithout(int track)
{
    p4.track_solo[track] = false;
    uint16_t soloMask = 0;
    for(int other = 0; other < 16; ++other)
        if(p4.track_solo[other]) soloMask |= (uint16_t)(1u << other);
    daisyUsb.setTrackSoloMask(soloMask);
}

void UnmuteTrack(int track)
{
    p4.track_muted[track] = false;
    SequencerInstance().muteTrack(track, false);
    daisyUsb.setTrackMute(track, false);
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
    if(midiSongPrepared && midiSongPatternCount > 0)
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
        if(p4.is_playing) daisyUsb.controlSong(1);
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
    midiSongPersisted = false;
    midiSongPatternCount = 0;
    midiSongChainCount = 0;
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
    midiMapCount = 0;
    midi_map_store_load(midiMap, midiMapCount);
    daisyUsb.begin();
}

// Forward decl: defined further down, next to the RANDOM SONG/AUTO FX/AUTO
// MIX state it drives; called once per tick from control_process() below.
void control_random_auto_tick();

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

    if(midiSongPrepared && transport.engine_responding
       && transport.pattern < midiSongPatternCount)
    {
        activeDaisyPattern = transport.pattern;
        expectedDaisyPattern = 0xFF;
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
    else if(!midiSongPrepared && transport.engine_responding && engineWasConnected
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
        SendMidiMapToDaisy();
    }
    engineWasConnected = transport.engine_responding;
    ProcessMidiMonitor();
    if(midiLearnArmed.load(std::memory_order_acquire)
       && millis() - midiLearnArmedSinceMs > kMidiLearnTimeoutMs)
    {
        midiLearnArmed.store(false, std::memory_order_release);
        midiLearnTimeoutRevision.fetch_add(1, std::memory_order_release);
    }
    SequencerInstance().update();
    control_random_auto_tick();
}

// ── User MIDI map / LEARN API ────────────────────────────────────────
void control_midi_learn_arm(bool armed)
{
    if(armed)
    {
        // Discard whatever is already queued so LEARN captures the next
        // FRESH press. Without this, a note/CC left over from normal play
        // just before arming (backlog from the 50 ms poll cadence) could be
        // captured instead of the pad/knob the user actually touches.
        MidiMonitorEvent stale;
        while(daisyUsb.popMidiEvent(stale)) {}
        midiLearnArmedSinceMs = millis();
    }
    midiLearnArmed.store(armed, std::memory_order_release);
}

uint32_t control_midi_learn_timeout_revision()
{
    return midiLearnTimeoutRevision.load(std::memory_order_acquire);
}

bool control_midi_learn_armed()
{
    return midiLearnArmed.load(std::memory_order_acquire);
}

uint32_t control_midi_capture_revision()
{
    return midiCaptureRevision.load(std::memory_order_acquire);
}

MidiLearnCapture control_midi_capture() { return midiCapture; }

uint32_t control_midi_activity_revision()
{
    return midiActivityRevision.load(std::memory_order_acquire);
}

void control_midi_last_activity(uint8_t& status, uint8_t& data0,
                                uint8_t& data1)
{
    status = midiActivity.status;
    data0 = midiActivity.data0;
    data1 = midiActivity.data1;
}

uint8_t control_midi_map_count() { return midiMapCount; }

bool control_midi_map_get(uint8_t index, MidiMapEntry& out)
{
    if(index >= midiMapCount) return false;
    out = midiMap[index];
    return true;
}

bool control_midi_map_find(uint8_t channel, uint8_t kind, uint8_t number,
                           MidiMapEntry& out)
{
    const int index = MidiMapIndexOf(channel, kind, number);
    if(index < 0) return false;
    out = midiMap[index];
    return true;
}

bool control_midi_map_assign(const MidiMapEntry& entry)
{
    if(entry.channel > 15u || entry.number > 127u) return false;
    if(entry.kind != MIDI_MAP_KIND_NOTE && entry.kind != MIDI_MAP_KIND_CC)
        return false;
    int index = MidiMapIndexOf(entry.channel, entry.kind, entry.number);
    if(index < 0)
    {
        if(midiMapCount >= MIDI_MAP_MAX_ENTRIES) return false;
        index = midiMapCount++;
    }
    midiMap[index] = entry;
    midi_map_store_save(midiMap, midiMapCount);
    SendMidiMapToDaisy();
    return true;
}

bool control_midi_map_clear(uint8_t channel, uint8_t kind, uint8_t number)
{
    const int index = MidiMapIndexOf(channel, kind, number);
    if(index < 0) return false;
    for(uint8_t i = index; i + 1 < midiMapCount; ++i)
        midiMap[i] = midiMap[i + 1];
    midiMapCount--;
    midi_map_store_save(midiMap, midiMapCount);
    SendMidiMapToDaisy();
    return true;
}

bool control_midi_map_clear_all()
{
    midiMapCount = 0;
    midi_map_store_save(midiMap, midiMapCount);
    SendMidiMapToDaisy();
    return true;
}

bool control_midi_map_find_duplicate(uint8_t kind, uint8_t action,
                                     uint8_t arg0, uint8_t arg1,
                                     uint8_t excludeChannel,
                                     uint8_t excludeNumber, MidiMapEntry& out)
{
    const int index = MidiMapIndexOfDuplicateAction(
        kind, action, arg0, arg1, excludeChannel, excludeNumber);
    if(index < 0) return false;
    out = midiMap[index];
    return true;
}

bool control_midi_map_export_sd()
{
    return midi_map_store_export_sd(midiMap, midiMapCount);
}

bool control_midi_map_import_sd()
{
    MidiMapEntry imported[MIDI_MAP_MAX_ENTRIES] = {};
    uint8_t count = 0;
    if(!midi_map_store_import_sd(imported, count)) return false;
    memcpy(midiMap, imported, count * sizeof(MidiMapEntry));
    midiMapCount = count;
    midi_map_store_save(midiMap, midiMapCount);
    SendMidiMapToDaisy();
    return true;
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
    if(midiSongPrepared)
    {
        SequencerInstance().songChainPlay();
        daisyUsb.controlSong(1);
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
    if(midiSongPrepared)
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
    if(midiSongPrepared) control_cancel_midi_song();
    index = Clamp(index, 0, MAX_PATTERNS - 1);
    if(queuedLogicalPattern >= 0 || queuedDaisyPattern != 0xFF)
        daisyUsb.cancelPatternQueue();
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
    if(midiSongPrepared) control_cancel_midi_song();
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
    if(variation < SEQ_VAR_NEON_BREAK || variation > SEQ_VAR_UNDO)
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

        auto hit = [&](int track, int step, uint8_t vel = 104,
                       uint8_t prob = 100, uint8_t rats = 1)
        {
            if(track < 0 || track >= 16 || step < 0 || step >= 16) return;
            active[track][step] = true;
            velocity[track][step] = Clamp<uint8_t>(vel, 1, 127);
            probability[track][step] = Clamp<uint8_t>(prob, 1, 100);
            ratchet[track][step] = Clamp<uint8_t>(rats, 1, 4);
        };
        auto rest = [&](int track, int step)
        {
            if(track >= 0 && track < 16 && step >= 0 && step < 16)
                active[track][step] = false;
        };

        switch(variation)
        {
            case SEQ_VAR_NEON_BREAK:
                for(int track = 2; track <= 7; ++track)
                    for(int step = 0; step < 4; ++step) rest(track, step);
                for(int step : {3, 7, 11, 15}) hit(3, step, step == 15 ? 118 : 92);
                for(int step : {6, 14}) hit(5, step, 106, 100, step == 14 ? 2 : 1);
                for(int step : {5, 13}) hit(6, step, 78, 78);
                hit(8, 11, 94); hit(9, 13, 104, 100, 2); hit(10, 15, 122, 100, 3);
                break;

            case SEQ_VAR_RATCHET_STORM:
                for(int step = 0; step < 16; ++step) rest(2, step);
                for(int step = 0; step < 16; step += 2)
                    hit(2, step, (step % 4 == 0) ? 112 : 82, 100,
                        step >= 12 ? 4 : 2);
                hit(3, 7, 92); hit(3, 15, 120, 100, 2);
                break;

            case SEQ_VAR_GHOST_GROOVE:
                for(int step : {3, 7, 11, 15}) hit(1, step, 42, 68);
                for(int step : {2, 6, 10, 14}) hit(5, step, 58, 76);
                hit(11, 5, 64, 62); hit(11, 13, 72, 70);
                break;

            case SEQ_VAR_POLYRHYTHM:
                for(int step = 0; step < 16; step += 3) hit(6, step, 82, 86);
                for(int step = 1; step < 16; step += 5) hit(7, step, 76, 78);
                for(int step = 2; step < 16; step += 7) hit(12, step, 70, 72, 2);
                break;

            case SEQ_VAR_HALF_TIME:
                for(int step = 0; step < 16; ++step) {
                    rest(1, step); rest(5, step);
                    if((step & 3) != 0) rest(2, step);
                }
                hit(0, 0, 124); hit(0, 10, 112);
                hit(1, 8, 124); hit(5, 8, 92);
                hit(3, 15, 112, 100, 2);
                break;

            case SEQ_VAR_MIRROR:
                for(int track = 2; track < 16; ++track)
                {
                    for(int step = 0; step < 8; ++step)
                    {
                        const int other = 15 - step;
                        bool a = active[track][step];
                        active[track][step] = active[track][other];
                        active[track][other] = a;
                        uint8_t v = velocity[track][step];
                        velocity[track][step] = velocity[track][other];
                        velocity[track][other] = v;
                        uint8_t p = probability[track][step];
                        probability[track][step] = probability[track][other];
                        probability[track][other] = p;
                        uint8_t r = ratchet[track][step];
                        ratchet[track][step] = ratchet[track][other];
                        ratchet[track][other] = r;
                    }
                }
                break;

            case SEQ_VAR_TOM_CASCADE:
                for(int track = 8; track <= 10; ++track)
                    for(int step = 0; step < 16; ++step) rest(track, step);
                hit(8, 8, 90); hit(8, 11, 98);
                hit(9, 10, 102); hit(9, 13, 108, 100, 2);
                hit(10, 12, 112); hit(10, 14, 118, 100, 2);
                hit(10, 15, 127, 100, 4);
                break;

            case SEQ_VAR_ACID_SWITCH:
                for(int track = 11; track < 16; ++track)
                    for(int step = 0; step < 16; ++step) rest(track, step);
                for(int step = 0; step < 16; ++step)
                {
                    if((step % 3) == 0) hit(11, step, 104, 94);
                    if((step % 5) == 1) hit(12, step, 92, 82);
                    if((step & 3) == 3) hit(13 + ((step >> 2) % 3), step,
                                              98, 88, step >= 12 ? 2 : 1);
                }
                break;

            case SEQ_VAR_HAT_LIFT:
                for(int step = 0; step < 16; ++step) {
                    rest(2, step); rest(3, step);
                }
                for(int step = 0; step < 16; step += 2)
                    hit(2, step, (step % 4 == 0) ? 112 : 76, 100,
                        step == 14 ? 3 : 1);
                hit(3, 7, 94); hit(3, 15, 122, 100, 2);
                break;

            case SEQ_VAR_SPARSE_SPACE:
                for(int track = 2; track < 16; ++track)
                    for(int step = 0; step < 16; ++step)
                        if(((track * 5 + step * 3) % 7) < 5) rest(track, step);
                hit(3, 15, 94, 86);
                break;

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

// ── PLAY RANDOM: fresh pattern generator (Euclidean rhythms, per style) ────
namespace
{
// Track roles: 0 BD 1 SD 2 CH 3 OH 4 CY 5 CP 6 RS 7 CB 8 LT 9 MT 10 HT
//              11 MA 12 CL 13 HC 14 MC 15 LC  (see trackNames in ui_screens.cpp)
struct RandomTrackSpec
{
    uint8_t track;
    uint8_t hitsMin, hitsMax;   // Euclidean pulse count, randomized per generation
    uint8_t velBase, velVar;    // velocity = velBase +/- velVar
    uint8_t probMin, probMax;   // per-step trigger probability range
    uint8_t ratchetPct;         // chance (0-100) a hit gets a 2-4x ratchet
    bool    anchored;           // true: rotation fixed at 0 (kick/snare backbone)
};

const RandomTrackSpec RND_TECHNO[] = {
    {0, 4, 4,  120, 4,  100, 100, 0,  true},
    {2, 10, 13, 78, 18, 80, 95,  10, false},
    {3, 2, 4,   92, 10, 75, 90,  0,  false},
    {5, 2, 2,  104, 6,  90, 100, 0,  false},
    {6, 2, 4,   58, 14, 40, 65,  20, false},
};
const RandomTrackSpec RND_HOUSE[] = {
    {0, 4, 4,  122, 4,  100, 100, 0, true},
    {2, 7, 9,   72, 16, 75, 90,  0, false},
    {3, 3, 5,   96, 8,  80, 95,  0, false},
    {5, 2, 2,  108, 6,  95, 100, 0, false},
    {7, 1, 3,   50, 10, 30, 50,  0, false},
};
// Hit-count centers deliberately kept close to 2,2,3,2,5,5 across BD/SD/CH/OH/CP/RS.
const RandomTrackSpec RND_BREAKBEAT[] = {
    {0, 2, 3,  116, 10, 90, 100, 0,  true},
    {1, 2, 2,  112, 8,  95, 100, 0,  true},
    {2, 2, 4,   74, 18, 70, 85,  15, false},
    {3, 1, 3,   90, 10, 55, 75,  0,  false},
    {5, 4, 6,   96, 12, 60, 80,  10, false},
    {6, 4, 6,   62, 16, 40, 65,  25, false},
};
const RandomTrackSpec RND_HIPHOP[] = {
    {0, 2, 4,  114, 10, 85, 95,  0,  true},
    {1, 2, 2,  110, 8,  95, 100, 0,  true},
    {2, 6, 8,   64, 18, 65, 85,  10, false},
    {5, 1, 3,   88, 10, 25, 45,  0,  false},
    {8, 1, 2,   80, 10, 20, 40,  0,  false},
};
const RandomTrackSpec RND_TRAP[] = {
    {0, 2, 4,  120, 6,  95, 100, 0,  true},
    {1, 2, 2,  112, 6,  95, 100, 0,  true},
    {2, 11, 15, 55, 30, 85, 98,  45, false},
    {3, 1, 3,   85, 10, 45, 65,  0,  false},
    {7, 1, 3,   58, 10, 20, 40,  0,  false},
};
const RandomTrackSpec RND_MINIMAL[] = {
    {0, 2, 4,  100, 8,  70, 90, 0,  true},
    {2, 4, 6,   58, 14, 60, 80, 0,  false},
    {6, 2, 4,   54, 12, 40, 65, 15, false},
    {11, 1, 3,  48, 10, 25, 50, 0,  false},
    {12, 1, 3,  46, 10, 20, 45, 0,  false},
};

template<typename T, size_t N>
constexpr size_t arrayLen(const T (&)[N]) { return N; }

uint32_t randomFastRand()
{
    static uint32_t s = 0;
    if(s == 0) s = (uint32_t)millis() ^ 0x9E3779B9u | 1u;
    s ^= s << 13; s ^= s >> 17; s ^= s << 5;
    return s;
}
long randomRange(long mn, long mx)
{
    if(mx <= mn) return mn;
    return mn + (long)(randomFastRand() % (uint32_t)(mx - mn + 1));
}

// Places `hits` pulses as evenly as possible across `steps` (bucketed
// approximation of Bjorklund's algorithm), then rotates the result.
void euclideanRhythm(uint8_t hits, uint8_t steps, uint8_t rotation, bool out[16])
{
    for(uint8_t i = 0; i < 16; ++i) out[i] = false;
    if(steps == 0 || steps > 16) return;
    if(hits == 0) return;
    if(hits >= steps)
    {
        for(uint8_t i = 0; i < steps; ++i) out[i] = true;
        return;
    }
    bool raw[16] = {};
    int prevBucket = -1;
    for(uint8_t i = 0; i < steps; ++i)
    {
        const int bucket = (int)i * (int)hits / (int)steps;
        if(bucket != prevBucket) raw[i] = true;
        prevBucket = bucket;
    }
    const uint8_t r = rotation % steps;
    for(uint8_t i = 0; i < steps; ++i)
        out[i] = raw[(i + r) % steps];
}
} // namespace

bool control_apply_random_pattern(uint8_t style)
{
    if(style < RND_STYLE_TECHNO || style > RND_STYLE_MINIMAL)
        return false;

    const RandomTrackSpec* specs = RND_TECHNO;
    size_t specCount = arrayLen(RND_TECHNO);
    switch(style)
    {
        case RND_STYLE_TECHNO:    specs = RND_TECHNO;    specCount = arrayLen(RND_TECHNO);    break;
        case RND_STYLE_HOUSE:     specs = RND_HOUSE;     specCount = arrayLen(RND_HOUSE);     break;
        case RND_STYLE_BREAKBEAT: specs = RND_BREAKBEAT; specCount = arrayLen(RND_BREAKBEAT); break;
        case RND_STYLE_HIPHOP:    specs = RND_HIPHOP;    specCount = arrayLen(RND_HIPHOP);    break;
        case RND_STYLE_TRAP:      specs = RND_TRAP;      specCount = arrayLen(RND_TRAP);      break;
        case RND_STYLE_MINIMAL:   specs = RND_MINIMAL;   specCount = arrayLen(RND_MINIMAL);   break;
        default: return false;
    }

    Sequencer& sequencer = SequencerInstance();
    const int pattern = Clamp(p4.current_pattern, 0, MAX_PATTERNS - 1);

    bool active[16][16] = {};
    uint8_t velocity[16][16] = {};
    uint8_t probability[16][16] = {};
    uint8_t ratchet[16][16] = {};

    for(int track = 0; track < 16; ++track)
        for(int step = 0; step < 16; ++step)
        {
            active[track][step] = sequencer.getStep(pattern, track, step);
            velocity[track][step] = sequencer.getStepVelocity(pattern, track, step);
            probability[track][step] = sequencer.getStepProbability(pattern, track, step);
            ratchet[track][step] = Clamp<uint8_t>(
                sequencer.getStepRatchet(pattern, track, step), 1, 4);
        }

    variationBackup.valid = true;
    variationBackup.pattern = pattern;
    memcpy(variationBackup.active, active, sizeof(active));
    memcpy(variationBackup.velocity, velocity, sizeof(velocity));
    memcpy(variationBackup.probability, probability, sizeof(probability));
    memcpy(variationBackup.ratchet, ratchet, sizeof(ratchet));

    // Start from a blank pattern: a fresh generation should read as one
    // coherent groove for the chosen style, not a blend with whatever was
    // on the tracks the style does not use.
    for(int track = 0; track < 16; ++track)
        for(int step = 0; step < 16; ++step)
        {
            active[track][step] = false;
            velocity[track][step] = 100;
            probability[track][step] = 100;
            ratchet[track][step] = 1;
        }

    for(size_t i = 0; i < specCount; ++i)
    {
        const RandomTrackSpec& spec = specs[i];
        if(spec.track >= 16) continue;
        const uint8_t hits = (uint8_t)randomRange(spec.hitsMin, spec.hitsMax);
        if(hits == 0) continue;
        const uint8_t rotation = spec.anchored
            ? 0 : (uint8_t)randomRange(0, 15);
        bool hitSteps[16];
        euclideanRhythm(hits, 16, rotation, hitSteps);
        for(int step = 0; step < 16; ++step)
        {
            if(!hitSteps[step]) continue;
            active[spec.track][step] = true;
            velocity[spec.track][step] = (uint8_t)Clamp<long>(
                spec.velBase + randomRange(-(long)spec.velVar, (long)spec.velVar),
                1, 127);
            probability[spec.track][step] = (uint8_t)randomRange(
                spec.probMin, spec.probMax);
            const bool ratchetHit = spec.ratchetPct > 0
                && randomRange(0, 99) < spec.ratchetPct;
            ratchet[spec.track][step] = ratchetHit
                ? (uint8_t)randomRange(2, 4) : 1;
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

    if(!changed) variationBackup.valid = false;
    return changed;
}

// ── RANDOM SONG / AUTO FX / AUTO MIX: bar-synced auto modes ────────────────
namespace
{
struct BarClock
{
    bool active = false;
    uint8_t bars = 4;
    uint32_t windowStartMs = 0;
    bool windowOpen = false;
};
BarClock songClock;
BarClock fxClock;
BarClock mixClock;
uint8_t songStyle = RND_STYLE_TECHNO;

// Wall-clock bar timer (derived from the current BPM), not a step-sample
// comparison: sampling p4.current_step once per control_process() tick is
// only as fresh as the last USB position poll, and a single missed sample
// (e.g. the main loop briefly busy drawing) silently drops a whole bar —
// invisible at bars=1 (any later wrap still fires immediately) but
// compounds badly at bars=2/4/8, which need several *consecutive* correct
// samples in a row. Real elapsed time doesn't have that failure mode.
bool barClockTick(BarClock& c)
{
    if(!c.active || !p4.is_playing)
    {
        c.windowOpen = false;
        return false;
    }
    const uint32_t now = millis();
    if(!c.windowOpen)
    {
        c.windowStartMs = now;
        c.windowOpen = true;
        return false;
    }
    const float bpm = p4.bpm_int + p4.bpm_frac / 10.0f;
    if(bpm < 20.0f) return false;   // guard against a garbage/unset tempo
    const uint32_t barMs = (uint32_t)(240000.0f / bpm);   // ms per 16-step bar
    const uint32_t targetMs = barMs * (uint32_t)(c.bars < 1 ? 1 : c.bars);
    if(now - c.windowStartMs >= targetMs)
    {
        c.windowStartMs = now;
        return true;
    }
    return false;
}

// Case-insensitive substring search. Avoids relying on strcasestr (a GNU
// extension not guaranteed on the ESP32 toolchain).
bool containsIgnoreCase(const char* haystack, const char* needle)
{
    if(!haystack || !needle || !*needle) return false;
    const size_t needleLen = strlen(needle);
    for(const char* p = haystack; *p; ++p)
    {
        size_t i = 0;
        while(i < needleLen && p[i]
              && tolower((unsigned char)p[i]) == tolower((unsigned char)needle[i]))
            ++i;
        if(i == needleLen) return true;
    }
    return false;
}

const char* songStyleKeyword(uint8_t style)
{
    switch(style)
    {
        case RND_STYLE_TECHNO:    return "techno";
        case RND_STYLE_HOUSE:     return "house";
        case RND_STYLE_BREAKBEAT: return "break";
        case RND_STYLE_HIPHOP:    return "hip";
        case RND_STYLE_TRAP:      return "trap";
        case RND_STYLE_MINIMAL:   return "minimal";
        default:                  return "";
    }
}

// Picks a different existing pattern (factory or saved user slot) and
// queues it for the next bar via the same quantized-queue path the manual
// "Q 1 BAR" control already uses. Prefers patterns whose name/genre
// mentions the chosen style's keyword; when none match (the factory bank
// is one cohesive suite, not a genre library) falls back to any other
// available pattern rather than silently doing nothing.
void triggerRandomSongJump()
{
    Sequencer& sequencer = SequencerInstance();
    const int current = Clamp(p4.current_pattern, 0, MAX_PATTERNS - 1);
    const char* keyword = songStyleKeyword(songStyle);

    int candidates[MAX_PATTERNS];
    int candidateCount = 0;
    int fallback[MAX_PATTERNS];
    int fallbackCount = 0;

    auto consider = [&](int p)
    {
        if(p == current) return;
        PatternMetadata meta{};
        if(!sequencer.getPatternMetadata(p, meta)) return;
        fallback[fallbackCount++] = p;
        if(containsIgnoreCase(meta.genre, keyword) || containsIgnoreCase(meta.name, keyword))
            candidates[candidateCount++] = p;
    };
    for(int p = 0; p < BUILTIN_PATTERN_COUNT; ++p)
        consider(p);
    for(int p = USER_PATTERN_FIRST; p < MAX_PATTERNS; ++p)
        if(pattern_store_is_saved(p))
            consider(p);

    const int* pool = candidateCount > 0 ? candidates : fallback;
    const int poolCount = candidateCount > 0 ? candidateCount : fallbackCount;
    if(poolCount == 0) return;   // nothing else saved to jump to

    const int pick = pool[randomRange(0, poolCount - 1)];
    control_send_queue_pattern(pick);
}
} // namespace

void control_random_song_set_active(bool active)
{
    songClock.active = active;
    songClock.windowOpen = false;
}
bool control_random_song_active() { return songClock.active; }
void control_random_song_set_style(uint8_t style)
{
    if(style >= RND_STYLE_TECHNO && style <= RND_STYLE_MINIMAL) songStyle = style;
}
uint8_t control_random_song_style() { return songStyle; }
void control_random_song_set_bars(uint8_t bars)
{
    songClock.bars = bars < 1 ? 1 : (bars > 8 ? 8 : bars);
}
uint8_t control_random_song_bars() { return songClock.bars; }

void control_random_fx_set_active(bool active)
{
    fxClock.active = active;
    fxClock.windowOpen = false;
}
bool control_random_fx_active() { return fxClock.active; }
void control_random_fx_set_bars(uint8_t bars)
{
    fxClock.bars = bars < 1 ? 1 : (bars > 8 ? 8 : bars);
}
uint8_t control_random_fx_bars() { return fxClock.bars; }

void control_random_mix_set_active(bool active)
{
    mixClock.active = active;
    mixClock.windowOpen = false;
}
bool control_random_mix_active() { return mixClock.active; }
void control_random_mix_set_bars(uint8_t bars)
{
    mixClock.bars = bars < 1 ? 1 : (bars > 8 ? 8 : bars);
}
uint8_t control_random_mix_bars() { return mixClock.bars; }

// Called once per control_process() tick — the auto-mode "conductor". Pure
// control-layer logic for the song jump; FX/mix re-randomization delegates
// to the same LVGL-side apply functions the manual RANDOM buttons use, so
// there is exactly one implementation of what "random" means for each.
void control_random_auto_tick()
{
    if(barClockTick(songClock)) triggerRandomSongJump();
    if(barClockTick(fxClock)) fx_random_apply();
    if(barClockTick(mixClock)) mix_random_apply();
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
    midiSongPersisted = persisted;
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
    midiSongPersisted = false;
    midiSongPatternCount = 0;
    midiSongChainCount = 0;
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
    if(muted && p4.track_solo[track]) PushSoloWithout(track);
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

void control_send_track_filter(int track, int filterType, float cutoffHz,
                               float resonance)
{
    if(track < 0 || track >= 16) return;
    daisyUsb.setTrackFilter(static_cast<uint8_t>(track),
                            static_cast<uint8_t>(Clamp(filterType, 0, 14)),
                            Clamp(cutoffHz, 20.0f, 20000.0f),
                            Clamp(resonance, 0.3f, 40.0f));
}

void control_send_track_clear_filter(int track)
{
    if(track >= 0 && track < 16)
        daisyUsb.clearTrackFilter(static_cast<uint8_t>(track));
}

void control_send_track_distortion(int track, float amount01)
{
    if(track < 0 || track >= 16) return;
    daisyUsb.setTrackDistortion(static_cast<uint8_t>(track),
                                Clamp(amount01, 0.0f, 1.0f));
}

void control_send_track_bitcrush(int track, int bits)
{
    if(track < 0 || track >= 16) return;
    daisyUsb.setTrackBitcrush(static_cast<uint8_t>(track),
                              static_cast<uint8_t>(Clamp(bits, 4, 16)));
}

void control_send_track_reverb_send(int track, int percent)
{
    if(track < 0 || track >= 16) return;
    daisyUsb.setTrackReverbSend(static_cast<uint8_t>(track),
                                static_cast<uint8_t>(Clamp(percent, 0, 100)));
}

void control_send_track_delay_send(int track, int percent)
{
    if(track < 0 || track >= 16) return;
    daisyUsb.setTrackDelaySend(static_cast<uint8_t>(track),
                               static_cast<uint8_t>(Clamp(percent, 0, 100)));
}

void control_send_track_clear_fx(int track)
{
    if(track >= 0 && track < 16)
        daisyUsb.clearTrackFx(static_cast<uint8_t>(track));
}

void control_send_set_filter(int type)
{
    if(i2c_rotaries_owns_function(POD_FUNC_FILTER_TYPE)
       && !i2c_rotaries_is_applying()) return;
    p4.filter_type = Clamp(type, 0, 14);
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

void control_send_all_fx_off()
{
    for(int encoder = 0; encoder < 3; ++encoder)
        p4.enc_muted[encoder] = true;
    p4.pot_muted[0] = true;
    p4.pot_muted[1] = true;
    p4.pot_muted[2] = true;
    p4.pot_value[1] = 0;
    p4.filter_type = 0;
    p4.cutoff_hz = 20000;
    p4.resonance_x10 = 7;
    p4.distortion_pct = 0;
    p4.bitcrush_bits = 16;
    p4.sample_rate_hz = 0;

    daisyUsb.sendU8(CMD_FLANGER_ACTIVE, 0);
    daisyUsb.sendU8(CMD_DELAY_ACTIVE, 0);
    daisyUsb.sendU8(CMD_REVERB_ACTIVE, 0);
    daisyUsb.sendU8(CMD_PHASER_ACTIVE, 0);
    daisyUsb.sendU8(CMD_AUTOWAH_ACTIVE, 0);
    daisyUsb.sendU8(CMD_CHORUS_ACTIVE, 0);
    daisyUsb.sendU8(CMD_TREMOLO_ACTIVE, 0);
    daisyUsb.sendU8(CMD_COMP_ACTIVE, 0);
    daisyUsb.sendU8(CMD_LIMITER_ACTIVE, 0);
    daisyUsb.sendU8(CMD_EARLY_REF_ACTIVE, 0);
    daisyUsb.sendU8(CMD_STEREO_WIDTH, 100);
    daisyUsb.sendU8(CMD_TAPE_STOP, 0);
    daisyUsb.sendU8(CMD_BEAT_REPEAT, 0);
    daisyUsb.sendFloat(CMD_WAVEFOLDER_GAIN, 1.0f);
    SendFilterState();
    fxDirty.store(true, std::memory_order_release);
}

void control_send_solo(int track, bool soloed)
{
    if(track < 0 || track >= 16) return;
    p4.track_solo[track] = soloed;
    daisyUsb.setTrackSolo(track, soloed);
    if(soloed && p4.track_muted[track]) UnmuteTrack(track);
}

void control_send_mute_mask(uint16_t mask)
{
    uint16_t soloMask = 0;
    for(int track = 0; track < 16; ++track)
    {
        const bool muted = (mask & (1u << track)) != 0;
        p4.track_muted[track] = muted;
        SequencerInstance().muteTrack(track, muted);
        if(muted) p4.track_solo[track] = false;
        if(p4.track_solo[track]) soloMask |= (uint16_t)(1u << track);
    }
    daisyUsb.setTrackMuteMask(mask);
    daisyUsb.setTrackSoloMask(soloMask);
}

void control_send_solo_mask(uint16_t mask)
{
    uint16_t muteMask = 0;
    bool muteChanged = false;
    for(int track = 0; track < 16; ++track)
    {
        const bool soloed = (mask & (1u << track)) != 0;
        p4.track_solo[track] = soloed;
        if(soloed && p4.track_muted[track])
        {
            p4.track_muted[track] = false;
            SequencerInstance().muteTrack(track, false);
            muteChanged = true;
        }
        if(p4.track_muted[track]) muteMask |= (uint16_t)(1u << track);
    }
    daisyUsb.setTrackSoloMask(mask);
    if(muteChanged) daisyUsb.setTrackMuteMask(muteMask);
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

void control_restore_track_engine(int track)
{
    if(track < 0 || track >= 16) return;
    const int pattern = Clamp(p4.current_pattern, 0, MAX_PATTERNS - 1);
    BuiltinPatternSoundProfile profile{};
    const int8_t engine = getBuiltinPatternSoundProfile(pattern, profile)
        ? profile.engines[track] : -1;
    daisyUsb.setTrackEngine(track, engine);
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
