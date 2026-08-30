/* ═══════════════════════════════════════════════════════════════════
 *  RED808 DRUM MACHINE V2 — DaisyPod3
 * ─────────────────────────────────────────────────────────────────
 *  Daisy Pod + Seed/Seed3 · STM32H750 + SDRAM | USB CDC device | RED808
 *  48000 Hz · 128 samples/block · 24 pads · 32 voces
 *  Master FX: Delay, Reverb, Chorus, Tremolo, Comp, Wavefolder,
 *             Limiter, Phaser, Flanger, Global Filter
 *  Per-track: Filter, Echo, Flanger, Comp, EQ 3-band, Sends,
 *             Pan, Mute/Solo
 *  Per-pad:   Filter, Distortion, Bitcrush, Loop, Reverse, Pitch,
 *             Stutter
 *  SD Card:   Carga de kits WAV vía SPI3 master (módulo 6-pin)
 *
 *  Enlace de control único: USB-C CDC entre P4 host y Daisy Pod device.
 *  SPI3 (SD card, MASTER):  D0=PB12/CS(GPIO)  D2=PC10/SCK
 *                            D1=PC11/MISO      D6=PC12/MOSI
 * ═══════════════════════════════════════════════════════════════════ */

#include "daisy_seed.h"
#define USE_DAISYSP_LGPL
#include "daisysp.h"
#include "ff_gen_drv.h"
#include "../shared/red808_protocol_codes.h"
#include "mpd218_mapping.h"
#include <string.h>
#include <math.h>
#include <new>
#include <stdio.h>
#include <strings.h>

#ifndef RED808_DSP_BLOCK_PROFILE
#define RED808_DSP_BLOCK_PROFILE 0
#endif

#ifndef RED808_MEM_AUDIT
#define RED808_MEM_AUDIT 0
#endif

/* ═══════════════════════════════════════════════════════════════════
 *  FAST MATH — sinf/expf overrides for synth engine hot paths
 *  sinf: corrected parabolic, max error ~0.06% — inaudible in drums
 *  expf: Schraudolph IEEE 754 bit-trick, ~4% error — perfect for envelopes
 *  These macros ONLY affect the synth headers below.
 * ═══════════════════════════════════════════════════════════════════ */
static inline float __fast_sinf(float x) {
    float phase = x * 0.15915494f;          /* x / (2*pi) */
    phase -= (float)(int)(phase);
    if(phase < 0.0f) phase += 1.0f;
    float p = 2.0f * phase - 1.0f;          /* [-1, 1)    */
    float y = 4.0f * p * (1.0f - fabsf(p));
    return -(0.225f * (y * fabsf(y) - y) + y);
}
static inline float __fast_expf(float x) {
    union { float f; int32_t i; } v;
    v.i = (int32_t)(12102203.0f * x) + 1065353216;
    return (v.i > 0) ? v.f : 0.0f;
}
#define sinf(x) __fast_sinf(x)
#define expf(x) __fast_expf(x)

/* Synth engine libraries */
#include "synth/tr808.h"
#include "synth/tr909.h"
#include "synth/tr505.h"
#include "synth/tb303.h"
#include "synth/wavetable_osc.h"
#include "synth/sh101.h"     /* I1: Roland SH-101 monosynth */
#include "synth/fm2op.h"     /* I2: 2-operator FM Yamaha-style */

#undef sinf
#undef expf

using namespace daisy;
using namespace daisysp;

/* libDaisy's MIDI handler owns a 256-event FIFO (~38 KiB). Keep that FIFO in
 * the otherwise-unused D2 RAM instead of consuming nearly all of DTCMRAM.
 * It is constructed after hardware init because this section is NOLOAD. */
alignas(MidiUartHandler)
static uint8_t mpdMidiStorage[sizeof(MidiUartHandler)]
    __attribute__((section(".heap")));

/* ═══════════════════════════════════════════════════════════════════
 *  1. HARDWARE
 * ═══════════════════════════════════════════════════════════════════ */
struct DaisyPod3Hardware
{
    DaisySeed seed;
    Encoder encoder;
    AnalogControl knob1;
    AnalogControl knob2;
    Switch button1;
    Switch button2;
    RgbLed led1;
    RgbLed led2;
    MidiUartHandler* midi;

    void Init()
    {
        seed.Configure();
        seed.Init();
        seed.SetAudioBlockSize(48);

        button1.Init(seed::D27);
        button2.Init(seed::D28);
        encoder.Init(seed::D26, seed::D25, seed::D13);
        led1.Init(seed::D20, seed::D19, seed::D18, true);
        led2.Init(seed::D17, seed::D24, seed::D23, true);
        led1.Set(0.0f, 0.0f, 0.0f);
        led2.Set(0.0f, 0.0f, 0.0f);
        UpdateLeds();

        AdcChannelConfig adc[2];
        adc[0].InitSingle(seed::D21);
        adc[1].InitSingle(seed::D15);
        seed.adc.Init(adc, 2);
        knob1.Init(seed.adc.GetPtr(0), seed.AudioCallbackRate());
        knob2.Init(seed.adc.GetPtr(1), seed.AudioCallbackRate());

        /* TRS MIDI IN Type A: RX=D14/PB7. TX is deliberately disabled because
         * D13/PB6 is already the encoder push switch on this custom Pod. */
        midi = new (mpdMidiStorage) MidiUartHandler();
        MidiUartHandler::Config midiConfig;
        midiConfig.transport_config.tx = Pin();
        midi->Init(midiConfig);
    }

    void SetAudioBlockSize(size_t size)
    {
        seed.SetAudioBlockSize(size);
        knob1.SetSampleRate(seed.AudioCallbackRate());
        knob2.SetSampleRate(seed.AudioCallbackRate());
    }

    void SetAudioSampleRate(SaiHandle::Config::SampleRate rate)
    {
        seed.SetAudioSampleRate(rate);
        knob1.SetSampleRate(seed.AudioCallbackRate());
        knob2.SetSampleRate(seed.AudioCallbackRate());
    }

    void StartAudio(AudioHandle::AudioCallback callback) { seed.StartAudio(callback); }
    void StartAdc() { seed.adc.Start(); }
    void StartMidi() { midi->StartReceive(); }

    void ProcessAllControls()
    {
        knob1.Process();
        knob2.Process();
        encoder.Debounce();
        button1.Debounce();
        button2.Debounce();
    }

    void UpdateLeds()
    {
        led1.Update();
        led2.Update();
    }
};

DaisyPod3Hardware pod;
DaisySeed& hw = pod.seed;
static CpuLoadMeter audioLoadMeter;

#if RED808_DSP_BLOCK_PROFILE
enum DspProfBlock : uint8_t {
    DSP_PROF_CALLBACK = 0,
    DSP_PROF_SEQ,
    DSP_PROF_LFO,
    DSP_PROF_SAMPLER_VOICES,
    DSP_PROF_SYNTH_808,
    DSP_PROF_SYNTH_909,
    DSP_PROF_SYNTH_505,
    DSP_PROF_SYNTH_303,
    DSP_PROF_SYNTH_WT,
    DSP_PROF_SYNTH_SH101,
    DSP_PROF_SYNTH_FM2OP,
    DSP_PROF_SYNTH_PHYS,
    DSP_PROF_SYNTH_NOISE,
    DSP_PROF_SYNTH_ROUTING,
    DSP_PROF_MASTER_FX,
    DSP_PROF_OUTPUT,
    DSP_PROF_COUNT
};

struct DspProfAccum {
    volatile uint64_t cycles;
    volatile uint32_t calls;
    volatile uint32_t maxCycles;
};

static DspProfAccum dspProf[DSP_PROF_COUNT];
static volatile uint32_t dspProfBlocks = 0;
static constexpr float kCpuClockHz = 480000000.0f;
static constexpr float kDspProfBlockBudgetCycles = kCpuClockHz * (128.0f / 48000.0f);

static inline void DspProfInit()
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t DspProfNow()
{
    return DWT->CYCCNT;
}

static inline void DspProfAdd(DspProfBlock block, uint32_t cycles)
{
    DspProfAccum& a = dspProf[block];
    a.cycles += cycles;
    a.calls++;
    if(cycles > a.maxCycles)
        a.maxCycles = cycles;
}

static inline void DspProfBlockDone()
{
    dspProfBlocks++;
}

struct DspProfSnapshot {
    uint64_t cycles;
    uint32_t calls;
    uint32_t maxCycles;
};

static void DspProfSnapshotAndReset(DspProfSnapshot out[DSP_PROF_COUNT], uint32_t* blocks)
{
    __disable_irq();
    for(uint8_t i = 0; i < DSP_PROF_COUNT; i++){
        out[i].cycles = dspProf[i].cycles;
        out[i].calls = dspProf[i].calls;
        out[i].maxCycles = dspProf[i].maxCycles;
        dspProf[i].cycles = 0;
        dspProf[i].calls = 0;
        dspProf[i].maxCycles = 0;
    }
    *blocks = dspProfBlocks;
    dspProfBlocks = 0;
    __enable_irq();
}

static const char* DspProfName(uint8_t block)
{
    switch(block){
        case DSP_PROF_CALLBACK: return "callback";
        case DSP_PROF_SEQ: return "sequencer";
        case DSP_PROF_LFO: return "track_lfo";
        case DSP_PROF_SAMPLER_VOICES: return "sampler_voices";
        case DSP_PROF_SYNTH_808: return "tr808";
        case DSP_PROF_SYNTH_909: return "tr909";
        case DSP_PROF_SYNTH_505: return "tr505";
        case DSP_PROF_SYNTH_303: return "tb303";
        case DSP_PROF_SYNTH_WT: return "wavetable";
        case DSP_PROF_SYNTH_SH101: return "sh101";
        case DSP_PROF_SYNTH_FM2OP: return "fm2op";
        case DSP_PROF_SYNTH_PHYS: return "phys";
        case DSP_PROF_SYNTH_NOISE: return "noise";
        case DSP_PROF_SYNTH_ROUTING: return "synth_routing";
        case DSP_PROF_MASTER_FX: return "master_fx";
        case DSP_PROF_OUTPUT: return "output";
        default: return "unknown";
    }
}

#define DSP_PROF_SCOPE(name) uint32_t _dsp_prof_start_##name = DspProfNow()
#define DSP_PROF_END(name) DspProfAdd(DSP_PROF_##name, DspProfNow() - _dsp_prof_start_##name)
#else
static inline void DspProfInit() {}
static inline void DspProfBlockDone() {}
#define DSP_PROF_SCOPE(name) do{}while(0)
#define DSP_PROF_END(name) do{}while(0)
#endif

/* ═══════════════════════════════════════════════════════════════════
 *  2. CONFIGURACIÓN
 * ═══════════════════════════════════════════════════════════════════ */
#define SAMPLE_RATE                 48000
#define AUDIO_BLOCK        128
#define MAX_PADS           24
#define CLEAN_TRACK_COUNT  4
#define TOTAL_SAMPLE_SLOTS (MAX_PADS + CLEAN_TRACK_COUNT)
#define MAX_VOICES         32
#define MAX_SAMPLE_BYTES   (8 * 1024 * 1024)   /* 8 MB per sampler */
#define SAMPLE_POOL_BYTES  (48 * 1024 * 1024)  /* global SDRAM pool for all samplers */
#define MAX_DELAY_SAMPLES  96000         /* 2 s @ 48000             */
#define TRACK_ECHO_SIZE    9600          /* 200 ms per track        */

/* ═══════════════════════════════════════════════════════════════════
 *  3. PROTOCOLO RED808 — TODOS los command codes  (protocol.h)
 * ═══════════════════════════════════════════════════════════════════ */
#define SPI_MAGIC_CMD       0xA5
#define SPI_MAGIC_RESP      0x5A

/* Triggers */
#define CMD_TRIGGER_SEQ       0x01
#define CMD_TRIGGER_LIVE      0x02
#define CMD_TRIGGER_STOP      0x03
#define CMD_TRIGGER_STOP_ALL  0x04
#define CMD_TRIGGER_SIDECHAIN 0x05

/* Volume */
#define CMD_MASTER_VOLUME     0x10
#define CMD_SEQ_VOLUME        0x11
#define CMD_LIVE_VOLUME       0x12
#define CMD_TRACK_VOLUME      0x13
#define CMD_LIVE_PITCH        0x14
#define CMD_TEMPO             0x15

/* Global Filter */
#define CMD_FILTER_SET        0x20
#define CMD_FILTER_CUTOFF     0x21
#define CMD_FILTER_RESONANCE  0x22
#define CMD_FILTER_BITDEPTH   0x23
#define CMD_FILTER_DISTORTION 0x24
#define CMD_FILTER_DIST_MODE  0x25
#define CMD_FILTER_SR_REDUCE  0x26
#define CMD_MASTER_FX_ROUTE   0x27  /* [fxId(1), connected(1)] estado de ruteo del grafo */
#define CMD_FILTER_MORPH      0x28  /* float 0.0(LP)-1.0(Notch) — solo con filterType == FTYPE_SVF_MORPH (15) */

/* Master FX */
#define CMD_DELAY_ACTIVE      0x30
#define CMD_DELAY_TIME        0x31
#define CMD_DELAY_FEEDBACK    0x32
#define CMD_DELAY_MIX         0x33
#define CMD_PHASER_ACTIVE     0x34
#define CMD_PHASER_RATE       0x35
#define CMD_PHASER_DEPTH      0x36
#define CMD_PHASER_FEEDBACK   0x37
#define CMD_FLANGER_ACTIVE    0x38
#define CMD_FLANGER_RATE      0x39
#define CMD_FLANGER_DEPTH     0x3A
#define CMD_FLANGER_FEEDBACK  0x3B
#define CMD_FLANGER_MIX       0x3C
#define CMD_COMP_ACTIVE       0x3D
#define CMD_COMP_THRESHOLD    0x3E
#define CMD_COMP_RATIO        0x3F
#define CMD_COMP_ATTACK       0x40
#define CMD_COMP_RELEASE      0x41
#define CMD_COMP_MAKEUP       0x42
#define CMD_REVERB_ACTIVE     0x43
#define CMD_REVERB_FEEDBACK   0x44
#define CMD_REVERB_LPFREQ     0x45
#define CMD_REVERB_MIX        0x46
#define CMD_CHORUS_ACTIVE     0x47
#define CMD_CHORUS_RATE       0x48
#define CMD_CHORUS_DEPTH      0x49
#define CMD_CHORUS_MIX        0x4A
#define CMD_TREMOLO_ACTIVE    0x4B
#define CMD_TREMOLO_RATE      0x4C
#define CMD_TREMOLO_DEPTH     0x4D
#define CMD_WAVEFOLDER_GAIN   0x4E
#define CMD_LIMITER_ACTIVE    0x4F

/* Per-Track FX */
#define CMD_TRACK_FILTER      0x50
#define CMD_TRACK_CLEAR_FILTER 0x51
#define CMD_TRACK_DISTORTION  0x52
#define CMD_TRACK_BITCRUSH    0x53
#define CMD_TRACK_ECHO        0x54
#define CMD_TRACK_FLANGER_FX  0x55
#define CMD_TRACK_COMPRESSOR  0x56
#define CMD_TRACK_CLEAR_LIVE  0x57
#define CMD_TRACK_CLEAR_FX    0x58
#define CMD_TRACK_REVERB_SEND 0x59
#define CMD_TRACK_DELAY_SEND  0x5A
#define CMD_TRACK_CHORUS_SEND 0x5B
#define CMD_TRACK_PAN         0x5C
#define CMD_TRACK_MUTE        0x5D
#define CMD_TRACK_SOLO        0x5E
#define CMD_TRACK_PHASER      0x5F
#define CMD_TRACK_TREMOLO     0x60
#define CMD_TRACK_PITCH       0x61
#define CMD_TRACK_GATE        0x62
#define CMD_TRACK_EQ_LOW      0x63
#define CMD_TRACK_EQ_MID      0x64
#define CMD_TRACK_EQ_HIGH     0x65
#define CMD_TRACK_FX_ROUTE    0x66  /* [track(1), connected(1)] per-track FX routing */

/* Per-Pad FX */
#define CMD_PAD_FILTER        0x70
#define CMD_PAD_CLEAR_FILTER  0x71
#define CMD_PAD_DISTORTION    0x72
#define CMD_PAD_BITCRUSH      0x73
#define CMD_PAD_LOOP          0x74
#define CMD_PAD_REVERSE       0x75
#define CMD_PAD_PITCH         0x76
#define CMD_PAD_STUTTER       0x77
#define CMD_PAD_SCRATCH       0x78
#define CMD_PAD_TURNTABLISM   0x79
#define CMD_PAD_CLEAR_FX      0x7A
#define CMD_PAD_TRIM          0x7B  // [pad(1), startPct(1), endPct(1)] 0-100, non-destructive

/* Sidechain */
#define CMD_SIDECHAIN_SET     0x90
#define CMD_SIDECHAIN_CLEAR   0x91

/* Sample Transfer */
#define CMD_SAMPLE_BEGIN      0xA0
#define CMD_SAMPLE_DATA       0xA1
#define CMD_SAMPLE_END        0xA2
#define CMD_SAMPLE_UNLOAD     0xA3
#define CMD_SAMPLE_UNLOAD_ALL 0xA4

/* SD Card */
#define CMD_SD_LIST_FOLDERS   0xB0
#define CMD_SD_LIST_FILES     0xB1
#define CMD_SD_FILE_INFO      0xB2
#define CMD_SD_LOAD_SAMPLE    0xB3
#define CMD_SD_LOAD_KIT       0xB4
#define CMD_SD_KIT_LIST       0xB5
#define CMD_SD_STATUS         0xB6
#define CMD_SD_UNLOAD_KIT     0xB7
#define CMD_SD_GET_LOADED     0xB8
#define CMD_SD_ABORT          0xB9

/* Status / Query */
#define CMD_GET_STATUS        0xE0
#define CMD_GET_PEAKS         0xE1
#define CMD_GET_CPU_LOAD      0xE2
#define CMD_GET_VOICES        0xE3
#define CMD_GET_EVENTS        0xE4
#define CMD_DIAG_PERF_STRESS  0xE5  /* [mode(1): 0=off,1=on,2=reset metrics] */
#define CMD_POD_GET_STATE     0xE6  /* DaisyPod physical controls/config */
#define CMD_POD_SET_CONFIG    0xE7  /* [PodConfigPayload] */
#define CMD_MIDI_GET_EVENTS   0xE8  /* drain MIDI monitor → [count, {status,d0,d1}×n] */
#define CMD_MIDI_MAP_SET      0xE9  /* [count(1), {ch,kind,num,action,a0,a1}×count] */
#define CMD_PING              0xEE
#define CMD_RESET             0xEF

#define RED808_PROTOCOL_VERSION       0x0203u
#define RED808_CAP_EXTENDED_PONG      0x0001u
#define RED808_CAP_USB_RX_DIAGNOSTICS 0x0002u
#define RED808_CAP_MIDI_MONITOR       0x0004u

/* Synth Engine */
#define CMD_SYNTH_TRIGGER     0xC0  /* [engine(1), instrument(1), velocity(1)] */
#define CMD_SYNTH_PARAM       0xC1  /* [engine(1), instrument(1), paramId(1), value(4)] */
#define CMD_SYNTH_NOTE_ON     0xC2  /* [midiNote(1), accent(1), slide(1)] → 303 */
#define CMD_SYNTH_NOTE_OFF    0xC3  /* 303 note off */
#define CMD_SYNTH_303_PARAM   0xC4  /* [paramId(1), value(4)] → 303 params */
#define CMD_SYNTH_ACTIVE      0xC5  /* [engineMask(1)] enable/disable engines */
#define CMD_SYNTH_PRESET      0xC6  /* [engine(1), preset(1)] apply factory preset */
#define CMD_SYNTH_NOTE_ON_EX  0xC7  /* [engine(1), midiNote(1), velocity(1), accent(1), slide(1)] generic melodic note-on */

/* Synth Engine IDs */
#define SYNTH_ENGINE_808   0
#define SYNTH_ENGINE_909   1
#define SYNTH_ENGINE_505   2
#define SYNTH_ENGINE_303   3
#define SYNTH_ENGINE_WTOSC 4
#define SYNTH_ENGINE_SH101 5  /* I1: Roland SH-101 monosynth */
#define SYNTH_ENGINE_FM2OP 6  /* I2: 2-operator FM */
#define SYNTH_ENGINE_PHYS  7  /* Physical modeling: ModalVoice/StringVoice */
#define SYNTH_ENGINE_NOISE 8  /* Noise/texture: Particle percussion */
#define SYNTH_ENGINE_COUNT 9

enum MasterFxRouteId : uint8_t {
    MASTER_FX_ROUTE_FILTER = 0,
    MASTER_FX_ROUTE_DELAY,
    MASTER_FX_ROUTE_PHASER,
    MASTER_FX_ROUTE_FLANGER,
    MASTER_FX_ROUTE_COMP,
    MASTER_FX_ROUTE_REVERB,
    MASTER_FX_ROUTE_CHORUS,
    MASTER_FX_ROUTE_TREMOLO,
    MASTER_FX_ROUTE_WAVEFOLDER,
    MASTER_FX_ROUTE_LIMITER,
    MASTER_FX_ROUTE_AUTOWAH,
    MASTER_FX_ROUTE_EARLY_REF,
};

/* New Master FX (mega upgrade) */
#define CMD_PITCHSHIFT_ACTIVE  0x27  /* reuse: [1=pitchshift] overloaded with subId */
#define CMD_AUTOWAH_ACTIVE     0xA5
#define CMD_AUTOWAH_LEVEL      0xA6
#define CMD_AUTOWAH_MIX        0xA7
#define CMD_STEREO_WIDTH       0xA8  /* [width 0-200] 100=normal */
#define CMD_TAPE_STOP          0xA9  /* [0=off, 1=stop, 2=start] */
#define CMD_BEAT_REPEAT        0xAA  /* [0=off, div=1/2/4/8/16/32] */
#define CMD_DELAY_STEREO       0xAB  /* [0=mono, 1=pingpong] */
#define CMD_CHORUS_STEREO      0xAC  /* [0=mono, 1=stereo] — now default stereo */
#define CMD_EARLY_REF_ACTIVE   0xAD  /* [0=off, 1=on] early reflections */
#define CMD_EARLY_REF_MIX      0xAE  /* [mix 0-100] */

/* Choke Groups */
#define CMD_CHOKE_GROUP        0xAF  /* [pad(1), group(1)] 0=none 1-8=group */

/* Song Mode */
#define CMD_SONG_UPLOAD        0xF2  /* [count(1), entries×{pattern(1), repeats(1)}] */
#define CMD_SONG_CONTROL       0xF3  /* [0=stop, 1=play, 2=reset] */
#define CMD_SONG_GET_POS       0xF4  /* → [songIdx(1), pattern(1), repeat(1), rsvd(1)] */

/* Expanded per-track LFO targets */
#define CMD_TRACK_LFO_CONFIG   0x67  /* [track, wave, target, rateHi, rateLo, depthHi, depthLo] */
#define CMD_TRACK_MUTE_MASK     0x68  /* [maskLo,maskHi] atomic mixer + sequencer mute state */
#define CMD_TRACK_SOLO_MASK     0x69  /* [maskLo,maskHi] atomic mixer solo state             */

/* Bulk */
#define CMD_BULK_TRIGGERS     0xF0
#define CMD_BULK_FX           0xF1

/* Daisy Sequencer (0xD0-0xDF) */
#define CMD_DSQ_UPLOAD_TRACK    0xD0  /* [pat,trk,stepCount,rsvd + N×{act,vel,div,prob}] */
#define CMD_DSQ_SET_STEP        0xD1  /* [pat,trk,step,active,vel,div,prob,rsvd]          */
#define CMD_DSQ_CONTROL         0xD2  /* [0=stop, 1=play, 2=reset]                       */
#define CMD_DSQ_SELECT_PATTERN  0xD3  /* [pat 0-15]                                       */
#define CMD_DSQ_SET_LENGTH      0xD4  /* [16/32/64]                                       */
#define CMD_DSQ_SET_MUTE        0xD5  /* [track, muted 0/1]                               */
#define CMD_DSQ_GET_POS         0xD6  /* no payload → [step,pat,playing,rsvd]             */
#define CMD_DSQ_SET_SWING       0xD7  /* [swing 0-100]  (global)                           */
#define CMD_DSQ_SET_PARAM_LOCK  0xD8  /* [pat,trk,step,cutoffEn,cutHi,cutLo,revEn,rev,volEn,vol,rsvd,rsvd] */
#define CMD_DSQ_SET_TRACK_ENGINE 0xD9 /* [track(1), engine(1)]  0xFF/-1=sampler 0=808 1=909 2=505 3=303     */
#define CMD_DSQ_SET_TRACK_SWING  0xDA /* E4: [track(1), swing 0-100(1)] per-track swing                    */
#define CMD_DSQ_SET_HUMANIZE     0xDB /* E2: [timingMs(1), velocityAmt(1)] humanizacion global              */
#define CMD_CLEAN_TRACK_ACTIVE   0xDC /* [track(1), active(1)] include/exclude clean track from global transport */
#define CMD_CLEAN_TRACK_MUTE     0xDD /* [track(1), muted(1)] mute clean track audio                        */
#define CMD_DSQ_SET_STEP_NOTES   0xDE /* [pat,trk,step,flags,note0,note1,note2,note3]                      */
#define CMD_DSQ_QUEUE_PATTERN     0xDF /* [pat,bars] 0=normal, 1..16=escena temporal con retorno             */

/* Filter types */
#define FTYPE_NONE       0
#define FTYPE_LOWPASS    1
#define FTYPE_HIGHPASS   2
#define FTYPE_BANDPASS   3
#define FTYPE_NOTCH      4
#define FTYPE_ALLPASS    5
#define FTYPE_PEAKING    6
#define FTYPE_LOWSHELF   7
#define FTYPE_HIGHSHELF  8
#define FTYPE_RESONANT   9   /* 4-pole LP: 2 biquads cascaded, Q up to 40, soft saturation */
#define FTYPE_LADDER    10   /* Moog Ladder 24dB/oct via DaisySP Ladder */
#define FTYPE_SVF_LP    11   /* State Variable Filter LP with drive */
#define FTYPE_SVF_HP    12   /* State Variable Filter HP */
#define FTYPE_SVF_BP    13   /* State Variable Filter BP */
#define FTYPE_COMB      14   /* Comb filter resonator */
#define FTYPE_SVF_MORPH 15   /* Same Svf as SVF_LP/HP/BP, but continuously
                              * crossfades LP->BP->HP->Notch via gFilterMorph
                              * (0..1) instead of picking one fixed output. */

/* Distortion modes */
#define DMODE_SOFT  0
#define DMODE_HARD  1
#define DMODE_TUBE  2
#define DMODE_FUZZ  3

/* ═══════════════════════════════════════════════════════════════════
 *  4. SPI PACKET
 * ═══════════════════════════════════════════════════════════════════ */
struct __attribute__((packed)) SPIPacketHeader {
    uint8_t  magic;
    uint8_t  cmd;
    uint16_t length;
    uint16_t sequence;
    uint16_t checksum;
};

struct __attribute__((packed)) LinkHealthResponse {
    uint32_t echoMs;
    uint32_t uptimeMs;
    uint16_t protocolVersion;
    uint16_t capabilityFlags;
    uint32_t rxDrops;
    uint32_t protocolErrors;
};

struct __attribute__((packed)) CpuLoadResponse {
    float    cpuLoad;
    uint32_t uptime;
    float    cpuAvg;
    float    cpuPeak;
    uint8_t  activeVoices;
    uint8_t  perfStressMode;
    uint16_t spiErrCnt;
    uint16_t spiRingDrops;
    float    masterPeak;
};

#define RX_BUF_SIZE  536
#define TX_BUF_SIZE  768   /* SD responses up to 676 bytes payload */

/* Buffers SPI — ya no necesitan DMA_BUFFER_MEM_SECTION porque usamos
 * polling directo (sin DMA). Pero los dejamos en SRAM1 por si acaso
 * para evitar problemas de D-cache cuando la CPU lee datos que llegan
 * por el periférico SPI.                                              */
static uint8_t DMA_BUFFER_MEM_SECTION rxBuf[RX_BUF_SIZE];
static uint8_t DMA_BUFFER_MEM_SECTION txBuf[TX_BUF_SIZE];
static volatile bool  waitingPayload  = false;
static volatile bool  pendingResponse = false;
static uint16_t       pendingTxLen    = 0;

/* USB CDC is the only P4 <-> DaisyPod3 transport. The callback is deliberately
 * tiny: it only moves bytes into a single-producer/single-consumer ring. Packet
 * framing, CRC validation and command execution stay in the main loop. */
static constexpr uint16_t USB_RX_RING_SIZE = 4096;
static constexpr uint16_t USB_RX_RING_MASK = USB_RX_RING_SIZE - 1;
static volatile uint8_t  usbRxRing[USB_RX_RING_SIZE];
static volatile uint16_t usbRxHead = 0;
static volatile uint16_t usbRxTail = 0;
static volatile uint32_t usbRxDrops = 0;
static uint8_t  usbParseBuf[RX_BUF_SIZE];
static uint16_t usbParseIdx = 0;
static uint32_t usbLastPacketMs = 0;

/* Producer: runs from the USB peripheral IRQ (FS_INTERNAL RX callback),
 * a THIRD execution context alongside the main loop and the audio IRQ.
 * usbRxRing/Head/Tail are `volatile`, which stops the compiler reordering
 * these accesses relative to each other, but on Cortex-M7 that alone does
 * not guarantee the CPU's own memory system publishes the byte write
 * before the index write becomes visible to the main-loop consumer (same
 * reasoning as the AudioCmd queue's __DMB() — see the comment above
 * TriggerPad's definition). Add the same barrier here. */
static void DaisyUsbRxCallback(uint8_t* data, uint32_t* length)
{
    if(data == nullptr || length == nullptr)
        return;
    for(uint32_t i = 0; i < *length; ++i)
    {
        const uint16_t next = (usbRxHead + 1u) & USB_RX_RING_MASK;
        if(next == usbRxTail)
        {
            usbRxDrops++;
            break;
        }
        usbRxRing[usbRxHead] = data[i];
        __DMB(); /* byte write must be visible before the index publishes it */
        usbRxHead = next;
    }
}

/* ═══════════════════════════════════════════════════════════════════
 *  5. SD RESPONSE STRUCTS & PATHS
 * ═══════════════════════════════════════════════════════════════════ */

/* Root path on SD card — firmware tries /data first, then / */
static const char* SD_DATA_ROOT = "/data";

/* Canonical pad→instrument mapping (pads 0-15 = LIVE, 16-23 = XTRA) */
static const char* PAD_FAMILY_NAMES[16] = {
    "BD","SD","CH","OH","CY","CP","RS","CB",
    "LT","MT","HT","MA","CL","HC","MC","LC"
};

/* Keyword table for matching RED 808 KARZ filenames → pad index */
struct InstrKeyword { const char* keyword; uint8_t pad; };
static const InstrKeyword INSTR_KEYWORDS[] = {
    {"BD",  0}, {"KICK", 0},
    {"SD",  1}, {"SNARE",1},
    {"CH",  2}, {"HH",   2}, {"HIHAT",2}, {"CLOSED",2},
    {"OH",  3}, {"OPEN", 3},
    {"CY",  4}, {"CYMBAL",4}, {"CRASH",4}, {"RIDE",4},
    {"CP",  5}, {"CLAP", 5},
    {"RS",  6}, {"RIM",  6},
    {"CB",  7}, {"COW",  7}, {"BELL", 7},
    {"LT",  8}, {"LTOM", 8},
    {"MT",  9}, {"MTOM", 9},
    {"HT", 10}, {"HTOM",10},
    {"MA", 11}, {"MARAC",11},
    {"CL", 12}, {"CLAV", 12}, {"CLAVE",12},
    {"HC", 13}, {"CONGA",13},
    {"MC", 14},
    {"LC", 15},
};
static const int NUM_INSTR_KEYWORDS = sizeof(INSTR_KEYWORDS)/sizeof(INSTR_KEYWORDS[0]);

struct __attribute__((packed)) SdKitListResponse {
    uint8_t count;
    char    kits[16][32];   /* max 16 kits, 32 chars each = 513 bytes */
};

struct __attribute__((packed)) SdLoadKitPayload {
    char    kitName[32];
    uint8_t startPad;
    uint8_t maxPads;
};

struct __attribute__((packed)) SdStatusResponse {
    uint8_t  present;
    uint8_t  reserved;
    uint16_t samplesLoaded;  /* bitmask */
    char     currentKit[32];
};

struct __attribute__((packed)) SdListFilesPayload {
    char folder[32];        /* e.g. "BD", "xtra", "RED 808 KARZ" */
};

struct __attribute__((packed)) SdListFilesResponse {
    uint8_t count;
    char    files[20][32];  /* max 20 files, 32 chars each */
};

struct __attribute__((packed)) SdFileInfoPayload {
    char folder[32];
    char filename[32];
};

struct __attribute__((packed)) SdFileInfoResponse {
    uint32_t sizeBytes;
    uint16_t sampleRate;
    uint16_t bitsPerSample;
    uint8_t  channels;
    uint8_t  reserved[3];
    uint32_t durationMs;    /* estimated */
};

struct __attribute__((packed)) SdLoadSamplePayload {
    char    folder[32];
    char    filename[32];
    uint8_t padIdx;
};

/* ═══════════════════════════════════════════════════════════════════
 *  6. SAMPLES EN SDRAM  (64 MB)
 * ═══════════════════════════════════════════════════════════════════ */
DSY_SDRAM_BSS static int16_t sampleStorage[SAMPLE_POOL_BYTES / 2];

static uint32_t sampleLength[MAX_PADS];
static uint32_t sampleTotalSamples[MAX_PADS];
static uint32_t sampleRateHz[MAX_PADS];
static uint32_t sampleOffsetSamples[MAX_PADS];
static uint32_t sampleCapacitySamples[MAX_PADS];
/* Non-destructive per-pad trim window, as a fraction of sampleLength[pad]
 * (0.0..1.0). Applied at trigger time in TriggerPad() — never touches the
 * uploaded PCM in sampleStorage[]. Both zero-init to 0.0f, which TriggerPad
 * treats as "no trim set" (start>=end) and falls back to the full sample —
 * so a freshly booted pad plays whole without any explicit reset needed.
 * CMD_PAD_TRIM is the only writer; CMD_SAMPLE_END/CMD_SAMPLE_UNLOAD reset
 * a pad back to that same "unset" state on a new/no WAV, so a trim sized
 * for one file can never misapply to a differently-sized one. */
static float padTrimStartPct[MAX_PADS];
static float padTrimEndPct[MAX_PADS];
/* volatile: the ISR (AudioCallback/TriggerPad) gates every read of
 * sampleStorage/sampleLength/sampleOffsetSamples on this flag. Main-loop
 * loaders (CMD_SAMPLE_END, LoadWavToPad, the QSPI boot loader) __DMB()
 * right before publishing it, so the ISR never observes it true while the
 * sample data it guards is still being written — see those call sites. */
static volatile bool sampleLoaded[MAX_PADS];
static volatile bool padLoading[MAX_PADS];  /* true while LoadWavToPad is writing */
static uint32_t cleanTrackLength[CLEAN_TRACK_COUNT];
static uint32_t cleanTrackTotalSamples[CLEAN_TRACK_COUNT];
static uint32_t cleanTrackOffsetSamples[CLEAN_TRACK_COUNT];
static uint32_t cleanTrackCapacitySamples[CLEAN_TRACK_COUNT];
static bool     cleanTrackLoaded[CLEAN_TRACK_COUNT];
static bool     cleanTrackMuted[CLEAN_TRACK_COUNT];
static bool     cleanTrackEnabled[CLEAN_TRACK_COUNT];
static bool     cleanTrackActive[CLEAN_TRACK_COUNT];
static volatile bool cleanTrackLoading[CLEAN_TRACK_COUNT];
static uint32_t cleanTrackPlayhead[CLEAN_TRACK_COUNT];
static volatile bool kitMuteActive = false; /* true → AudioCallback outputs silence */

/* Uploads SPI are accepted sequentially. This keeps validation bounded and
 * guarantees CMD_SAMPLE_END never exposes gaps/uninitialised SDRAM as audio. */
static uint32_t sampleUploadReceivedBytes[TOTAL_SAMPLE_SLOTS];
static bool     sampleUploadValid[TOTAL_SAMPLE_SLOTS];

static inline int16_t* SamplePtr(uint8_t pad)
{
    if(pad >= MAX_PADS || sampleCapacitySamples[pad] == 0)
        return nullptr;
    return &sampleStorage[sampleOffsetSamples[pad]];
}

static inline int16_t* CleanTrackPtr(uint8_t track)
{
    if(track >= CLEAN_TRACK_COUNT || cleanTrackCapacitySamples[track] == 0)
        return nullptr;
    return &sampleStorage[cleanTrackOffsetSamples[track]];
}

static void FreeSampleStorage(uint8_t pad)
{
    if(pad >= MAX_PADS)
        return;
    sampleOffsetSamples[pad] = 0;
    sampleCapacitySamples[pad] = 0;
}

static void FreeCleanTrackStorage(uint8_t track)
{
    if(track >= CLEAN_TRACK_COUNT)
        return;
    cleanTrackOffsetSamples[track] = 0;
    cleanTrackCapacitySamples[track] = 0;
}

static bool AllocAnySampleStorage(uint8_t slot, uint32_t neededSamples)
{
    if(neededSamples == 0 || neededSamples > (MAX_SAMPLE_BYTES / 2))
        return false;

    const bool isClean = slot >= MAX_PADS;
    const uint8_t index = isClean ? (uint8_t)(slot - MAX_PADS) : slot;
    if((isClean && index >= CLEAN_TRACK_COUNT) || (!isClean && index >= MAX_PADS))
        return false;

    uint32_t* capacity = isClean ? cleanTrackCapacitySamples : sampleCapacitySamples;
    uint32_t* offset = isClean ? cleanTrackOffsetSamples : sampleOffsetSamples;
    if(capacity[index] >= neededSamples)
        return true;

    if(isClean) FreeCleanTrackStorage(index);
    else FreeSampleStorage(index);

    struct Segment {
        uint32_t start;
        uint32_t end;
    };

    Segment segments[TOTAL_SAMPLE_SLOTS];
    uint8_t segCount = 0;
    for(uint8_t i = 0; i < MAX_PADS; i++){
        if(!isClean && i == index) continue;
        if(sampleCapacitySamples[i] == 0) continue;
        segments[segCount].start = sampleOffsetSamples[i];
        segments[segCount].end   = sampleOffsetSamples[i] + sampleCapacitySamples[i];
        segCount++;
    }
    for(uint8_t i = 0; i < CLEAN_TRACK_COUNT; i++){
        if(isClean && i == index) continue;
        if(cleanTrackCapacitySamples[i] == 0) continue;
        segments[segCount].start = cleanTrackOffsetSamples[i];
        segments[segCount].end   = cleanTrackOffsetSamples[i] + cleanTrackCapacitySamples[i];
        segCount++;
    }

    for(uint8_t i = 1; i < segCount; i++){
        Segment key = segments[i];
        int8_t j = (int8_t)i - 1;
        while(j >= 0 && segments[j].start > key.start){
            segments[j + 1] = segments[j];
            j--;
        }
        segments[j + 1] = key;
    }

    uint32_t cursor = 0;
    for(uint8_t i = 0; i < segCount; i++){
        if(segments[i].start >= cursor && (segments[i].start - cursor) >= neededSamples){
            offset[index] = cursor;
            capacity[index] = neededSamples;
            return true;
        }
        if(segments[i].end > cursor)
            cursor = segments[i].end;
    }

    const uint32_t poolSamples = SAMPLE_POOL_BYTES / 2;
    if(cursor <= poolSamples && (poolSamples - cursor) >= neededSamples){
        offset[index] = cursor;
        capacity[index] = neededSamples;
        return true;
    }

    return false;
}

static bool AllocSampleStorage(uint8_t pad, uint32_t neededSamples)
{
    if(pad >= MAX_PADS)
        return false;
    return AllocAnySampleStorage(pad, neededSamples);
}

static bool AllocCleanTrackStorage(uint8_t track, uint32_t neededSamples)
{
    if(track >= CLEAN_TRACK_COUNT)
        return false;
    return AllocAnySampleStorage((uint8_t)(MAX_PADS + track), neededSamples);
}

/* ═══════════════════════════════════════════════════════════════════
 *  7. VOCES POLIFÓNICAS
 * ═══════════════════════════════════════════════════════════════════ */
/* ── Voice steal priority (higher = harder to steal) ── */
enum VoicePriority : uint8_t {
    VPRI_LOW    = 0,   /* wavetable, noise — steal first  */
    VPRI_MEDIUM = 1,   /* sampler, FM, SH-101, physical   */
    VPRI_HIGH   = 2,   /* kick, snare, 303 — steal last   */
};

/* PadPriority() defined after dsqTrackEngine declaration */
static inline VoicePriority PadPriority(uint8_t pad);

/* A stolen voice leaves a short decaying residual while its replacement starts
 * immediately. 0.94^112 ~= -60 dB: about 2.3 ms at 48 kHz. */
static constexpr float STEAL_TAIL_COEF  = 0.94f;
static constexpr float STEAL_TAIL_FLOOR = 0.0001f;

struct Voice {
    bool     active;
    uint8_t  pad;
    float    pos;
    float    speed;
    float    gainL;
    float    gainR;
    float    baseGain; // gain antes del pan — actualizado por LFO vol/pan en tiempo real
    float    env;
    float    envAttackInc;
    float    envDecayCoef;
    uint8_t  envStage; /* 0=attack,1=decay,2=bypass */
    uint32_t age;
    uint32_t maxLen;    /* absolute end position (sample index), not a duration */
    uint32_t trimStart; /* absolute start position — non-zero only when the
                         * pad has a trim window set; see padTrimStartPct */
    bool     liveSource; /* true when triggered by CMD_TRIGGER_LIVE */
    /* Last routed sample and click-free residual used by voice stealing. */
    float    lastOutL;
    float    lastOutR;
    float    stealTailL;
    float    stealTailR;
};
static Voice   voices[MAX_VOICES];
static uint32_t voiceAge = 0;

/* ═══════════════════════════════════════════════════════════════════
 *  8. VOLÚMENES
 * ═══════════════════════════════════════════════════════════════════ */
static float masterGain  = 1.0f;
static float seqVolume   = 1.0f;
static float liveVolume  = 1.0f;
static float transportBpm = 120.0f;
static uint8_t podCurrentMasterVolume = 100;
static uint8_t podCurrentSeqVolume = 100;
static uint8_t podCurrentLiveVolume = 100;
static uint16_t podCurrentBpmX10 = 1200;
static float livePitch   = 1.0f;
static float trackGain[MAX_PADS];

/* ═══════════════════════════════════════════════════════════════════
 *  8b. DAISY SEQUENCER  (sample-accurate, BPM clock in AudioCallback)
 * ═══════════════════════════════════════════════════════════════════ */
#define DSQ_PATTERNS   20  /* banco factory: P01–P20, sin wrap de índice */
#define DSQ_TRACKS    16
#define DSQ_MAX_STEPS 64

/* 4-byte step descriptor (used in SPI upload packets) */
struct __attribute__((packed)) DsqStepPkt {
    uint8_t active;       /* 0 or 1                          */
    uint8_t velocity;     /* 1-127                           */
    uint8_t noteLenDiv;   /* low nibble=len, high=ratchet-1 */
    uint8_t probability;  /* 0-100 (100 = always fire)      */
};

/* Full step state (stored in SDRAM, includes param-locks) */
struct DsqStepFull {
    uint8_t  active;
    uint8_t  velocity;
    uint8_t  noteLenDiv;
    uint8_t  probability;
    uint8_t  ratchet;
    uint8_t  flags;       /* bit0 accent, bit1 slide */
    uint8_t  notes[4];    /* MIDI notes; 0 = unused */
    /* param locks */
    bool     cutoffEn;
    uint16_t cutoffHz;
    bool     reverbEn;
    uint8_t  reverbSend;  /* 0-100 */
    bool     volEn;
    uint8_t  volume;      /* 0-150 */
    uint8_t  _pad[1];
};

/* Resident patterns live in SDRAM; richer steps preserve melodic expression. */
DSY_SDRAM_BSS static DsqStepFull dsqSteps[DSQ_PATTERNS][DSQ_TRACKS][DSQ_MAX_STEPS];
static uint32_t dsqLoadedPatternMask = 0;

struct DaisySeqState {
    bool     playing;
    uint8_t  currentPattern;
    int8_t   queuedPattern;
    int8_t   performanceReturnPattern;
    uint8_t  queuedPatternBars;
    uint8_t  performanceBarsRemaining;
    bool     performancePatternActive;
    uint8_t  patternLength;    /* 16, 32, or 64         */
    int16_t  currentStep;      /* -1 = not started      */
    float    tempo;            /* BPM                   */
    uint8_t  swingAmount;      /* 0-100 (global)        */
    bool     trackMuted[DSQ_TRACKS];
    /* BPM clock (sample counter, updated inside AudioCallback) */
    uint32_t samplesElapsed;
    uint32_t samplesPerStep;
    /* E2: Humanization timing */
    uint8_t  humanizeTimingMs; /* 0-5 ms jitter */
    uint8_t  humanizeVelAmt;   /* 0-50 velocity variation */
};
static DaisySeqState dseq;

/* E4: Per-track swing 0-100 (0 = use global swing, >0 = override) */
static uint8_t  dsqTrackSwing[DSQ_TRACKS];
/* Sample-accurate deferred triggers: humanize timing + ratchets. */
struct PendingTrigger {
    bool    active;
    uint8_t pattern;
    uint8_t track;
    uint8_t step;
    uint8_t velocity;
    uint8_t repeatsRemaining;
    uint32_t countdown;
    uint32_t interval;
};
static PendingTrigger pendingTriggers[DSQ_TRACKS];

struct DsqHeldNotes {
    bool active;
    int8_t engine;
    uint8_t notes[4];
    uint32_t samplesRemaining;
};
static DsqHeldNotes dsqHeldNotes[DSQ_TRACKS];

/* Track → synth engine mapping  (-1 = sampler, 0=808, 1=909, 2=505, 3=303)
 * Updated via CMD_DSQ_SET_TRACK_ENGINE.  Default: all tracks use sampler. */
static int8_t dsqTrackEngine[DSQ_TRACKS];

enum PodControlFunction : uint8_t {
    POD_FUNC_NONE = 0,
    POD_FUNC_PLAY_TOGGLE,
    POD_FUNC_STOP,
    POD_FUNC_TRIGGER_SELECTED,
    POD_FUNC_PATTERN_PREV,
    POD_FUNC_PATTERN_NEXT,
    POD_FUNC_MASTER_VOLUME,
    POD_FUNC_SEQ_VOLUME,
    POD_FUNC_LIVE_VOLUME,
    POD_FUNC_TEMPO,
    POD_FUNC_SELECT_PAD,
    POD_FUNC_BACK,
    POD_FUNC_MIXER,
    POD_FUNC_FX,
    POD_FUNC_SEQUENCER,
    POD_FUNC_PAD_GRID,
    POD_FUNC_PAD_SOUNDS,
    POD_FUNC_XTRA_PADS,
    POD_FUNC_DELAY_MIX,
    POD_FUNC_REVERB_MIX,
    POD_FUNC_CONTROL_CONFIG,
    POD_FUNC_SCREEN_BRIGHTNESS,
    POD_FUNC_FLANGER_DEPTH,
    POD_FUNC_WAVEFOLDER_GAIN,
    POD_FUNC_CRUSH_MACRO,
    POD_FUNC_PHASER_DEPTH,
    POD_FUNC_FILTER_CUTOFF,
    POD_FUNC_FILTER_RESONANCE,
    POD_FUNC_DISTORTION,
    POD_FUNC_BIT_DEPTH,
    POD_FUNC_SAMPLE_RATE,
    POD_FUNC_FILTER_TYPE,
    POD_FUNC_COUNT
};

enum PodLedFunction : uint8_t {
    POD_LED_FIXED = 0,
    POD_LED_USB_LINK,
    POD_LED_PLAY_STATE,
    POD_LED_PAD_ACTIVITY,
    POD_LED_SD_STATE,
    POD_LED_SAMPLES_READY,
    POD_LED_COUNT
};

enum PodExtraFxActiveBits : uint8_t {
    POD_FX_EXTRA_AUTOWAH = 1u << 0,
    POD_FX_EXTRA_BEAT_REPEAT = 1u << 1,
    POD_FX_EXTRA_TAPE_STOP = 1u << 2,
    POD_FX_EXTRA_STEREO_WIDTH = 1u << 3,
};

struct __attribute__((packed)) PodConfigPayload {
    uint8_t version;
    uint8_t button1Function;
    uint8_t button2Function;
    uint8_t knob1Function;
    uint8_t knob2Function;
    uint8_t encoderFunction;
    uint8_t encoderButtonFunction;
    uint8_t rotary1Function;
    uint8_t rotary2Function;
    uint8_t rotary3Function;
    uint8_t rotary4Function;
    uint8_t selectorFunction;
    uint8_t led1Function;
    uint8_t led1R;
    uint8_t led1G;
    uint8_t led1B;
    uint8_t led2Function;
    uint8_t led2R;
    uint8_t led2G;
    uint8_t led2B;
    uint8_t faderFunction;
};

static_assert(sizeof(PodConfigPayload) == 21,
              "P4/Daisy PodConfigPayload wire layout changed");

struct __attribute__((packed)) PodStatePayload {
    PodConfigPayload config;
    uint16_t knob1;
    uint16_t knob2;
    int16_t encoderPosition;
    uint8_t buttons;
    uint8_t buttonPressEvents;
    uint8_t selectedPad;
    uint8_t led1R;
    uint8_t led1G;
    uint8_t led1B;
    uint8_t led2R;
    uint8_t led2G;
    uint8_t led2B;
    uint8_t masterVolume;
    uint8_t seqVolume;
    uint8_t liveVolume;
    uint8_t delayMixValue;
    uint8_t reverbMixValue;
    uint8_t fxActiveBits;    /* delay,reverb,flanger,phaser,fold,crush,filter,drive */
    uint8_t flangerDepthValue;
    uint8_t phaserDepthValue;
    uint8_t wavefolderValue;
    uint8_t crushValue;
    uint8_t filterType;
    uint8_t bitDepth;
    uint8_t distortionPct;
    uint8_t reservedFx;
    uint16_t cutoffHz;
    uint16_t resonanceX10;
    uint16_t sampleRateHz;
    uint16_t bpmX10;
    uint8_t playing;
    uint8_t sdPresent;
    uint16_t sampleMask;
    uint32_t revision;
};

static_assert(sizeof(PodStatePayload) == 66,
              "P4/Daisy PodStatePayload wire layout changed");

static constexpr uint8_t POD_CONFIG_VERSION = 7;
static PodConfigPayload podConfig = {
    POD_CONFIG_VERSION,
    POD_FUNC_BACK, POD_FUNC_CONTROL_CONFIG,
    POD_FUNC_MASTER_VOLUME, POD_FUNC_TEMPO,
    POD_FUNC_PATTERN_NEXT, POD_FUNC_PLAY_TOGGLE,
    POD_FUNC_DELAY_MIX, POD_FUNC_REVERB_MIX,
    POD_FUNC_FLANGER_DEPTH, POD_FUNC_PHASER_DEPTH,
    POD_FUNC_NONE,
    POD_LED_USB_LINK, 255, 24, 12,
    POD_LED_SAMPLES_READY, 0, 255, 90,
    POD_FUNC_SCREEN_BRIGHTNESS
};
static uint16_t podKnobRaw[2] = {};
static int16_t podLastKnobRaw[2] = {-1, -1};
static int16_t podEncoderPosition = 0;
static uint8_t podSelectedPad = 0;
static uint8_t podButtonBits = 0;
static uint8_t podButtonPressEvents = 0;
static uint8_t podLedRgb[2][3] = {};
static uint32_t podPadPulseUntilMs = 0;
static uint32_t podStateRevision = 1;
static bool podApplyingCommand = false;

static bool PodFunctionsConflict(uint8_t left, uint8_t right)
{
    if(left == POD_FUNC_NONE || right == POD_FUNC_NONE) return false;
    if(left == right) return true;
    const bool leftPattern = left == POD_FUNC_PATTERN_PREV
                          || left == POD_FUNC_PATTERN_NEXT;
    const bool rightPattern = right == POD_FUNC_PATTERN_PREV
                           || right == POD_FUNC_PATTERN_NEXT;
    if(leftPattern && rightPattern) return true;
    const bool leftCrush = left == POD_FUNC_CRUSH_MACRO;
    const bool rightCrush = right == POD_FUNC_CRUSH_MACRO;
    return (leftCrush && (right == POD_FUNC_BIT_DEPTH
                          || right == POD_FUNC_SAMPLE_RATE))
        || (rightCrush && (left == POD_FUNC_BIT_DEPTH
                           || left == POD_FUNC_SAMPLE_RATE));
}

static bool PodOwnsFunction(uint8_t function)
{
    return podConfig.knob1Function == function
        || podConfig.knob2Function == function;
}

static bool PodOwnsBitDepth()
{
    return PodOwnsFunction(POD_FUNC_CRUSH_MACRO)
        || PodOwnsFunction(POD_FUNC_BIT_DEPTH);
}

static bool PodOwnsSampleRate()
{
    return PodOwnsFunction(POD_FUNC_CRUSH_MACRO)
        || PodOwnsFunction(POD_FUNC_SAMPLE_RATE);
}

/* Map synth-engine (or -1 for sampler) + drum instrument to priority */
static inline VoicePriority PadPriority(uint8_t pad)
{
    if(pad >= DSQ_TRACKS) return VPRI_MEDIUM;
    int8_t eng = dsqTrackEngine[pad];
    switch(eng){
        case SYNTH_ENGINE_808:
        case SYNTH_ENGINE_909:
        case SYNTH_ENGINE_505:
            /* Kick (pad 0) and Snare (pad 1) are HIGH; rest MEDIUM */
            return (pad <= 1) ? VPRI_HIGH : VPRI_MEDIUM;
        case SYNTH_ENGINE_303:  return VPRI_HIGH;
        case SYNTH_ENGINE_SH101:
        case SYNTH_ENGINE_FM2OP:
        case SYNTH_ENGINE_PHYS: return VPRI_MEDIUM;
        case SYNTH_ENGINE_WTOSC:
        case SYNTH_ENGINE_NOISE: return VPRI_LOW;
        default: /* sampler */  return VPRI_MEDIUM;
    }
}

static void DsqUpdateSamplesPerStep() {
    float t = (dseq.tempo > 1.0f) ? dseq.tempo : 120.0f;
    /* 16th note = 60/(bpm×4) seconds */
    float stepSec = 60.0f / (t * 4.0f);
    dseq.samplesPerStep = (uint32_t)(stepSec * (float)SAMPLE_RATE);
    if(dseq.samplesPerStep < 64) dseq.samplesPerStep = 64;
}

static void DsqInit() {
    /* dsqSteps está en SDRAM (DSY_SDRAM_BSS) que NO se zero-inicializa
     * al boot en STM32H7. Sin este memset, volEn/reverbEn/cutoffEn pueden
     * tener basura (=true) con volume/cutoffHz/reverbSend=0, haciendo que
     * DsqFireStep ponga trackGain[t]=0 al disparar el primer step y
     * silenciando todos los live pads permanentemente. */
    memset(dsqSteps, 0, sizeof(dsqSteps));
    dsqLoadedPatternMask = 0;
    memset(&dseq, 0, sizeof(dseq));
    memset(dsqTrackSwing, 0, sizeof(dsqTrackSwing));   /* E4 */
    memset(pendingTriggers, 0, sizeof(pendingTriggers)); /* E4 */
    memset(dsqHeldNotes, 0, sizeof(dsqHeldNotes));
    /* -1 (0xFF) = todos los tracks en modo sampler por defecto */
    memset(dsqTrackEngine, (uint8_t)0xFF, sizeof(dsqTrackEngine));
    for(int i = 0; i < CLEAN_TRACK_COUNT; i++) {
        cleanTrackEnabled[i] = true;
        cleanTrackActive[i] = false;
        cleanTrackMuted[i] = false;
        cleanTrackPlayhead[i] = 0;
    }
    dseq.tempo        = 120.0f;
    dseq.patternLength = 16;
    dseq.currentStep  = -1;
    dseq.queuedPattern = -1;
    dseq.performanceReturnPattern = -1;
    DsqUpdateSamplesPerStep();
}

static int8_t DsqFallbackEngine(uint8_t track)
{
    return track <= 8 ? SYNTH_ENGINE_909 : SYNTH_ENGINE_505;
}

static void DsqEnsureAudibleSources()
{
    for(uint8_t track = 0; track < DSQ_TRACKS; track++)
        dsqTrackEngine[track] = sampleLoaded[track]
            ? static_cast<int8_t>(-1) : DsqFallbackEngine(track);
}

/* ═══════════════════════════════════════════════════════════════════
 *  9. PEAKS
 * ═══════════════════════════════════════════════════════════════════ */
static volatile float trackPeak[MAX_PADS];
static volatile float masterPeak = 0.0f;

/* ═══════════════════════════════════════════════════════════════════
 *  10. BiquadEQ  (Audio EQ Cookbook – LP/HP/BP/Notch/Peak/Shelf)
 * ═══════════════════════════════════════════════════════════════════ */
struct BiquadEQ {
    float b0=1,b1=0,b2=0,a1=0,a2=0;
    float z1=0,z2=0;

    float Process(float in){
        float out = b0*in + z1;
        z1 = b1*in - a1*out + z2;
        z2 = b2*in - a2*out;
        return out;
    }
    void Reset(){ z1=z2=0; }

    void SetType(uint8_t t, float freq, float q, float sr, float gainDb=0.f){
        if(freq<20.f) freq=20.f;
        if(freq>sr*0.45f) freq=sr*0.45f;
        if(q<0.3f) q=0.3f;
        float w = 2.f*(float)M_PI*freq/sr;
        float s_ = sinf(w), c_ = cosf(w);
        float a  = s_/(2.f*q);
        float a0i;
        switch(t){
            case FTYPE_LOWPASS:
                a0i = 1.f/(1.f+a);
                b0 = ((1.f-c_)*0.5f)*a0i;
                b1 = (1.f-c_)*a0i;
                b2 = b0; a1=(-2.f*c_)*a0i; a2=(1.f-a)*a0i;
                break;
            case FTYPE_HIGHPASS:
                a0i = 1.f/(1.f+a);
                b0 = ((1.f+c_)*0.5f)*a0i;
                b1 = -(1.f+c_)*a0i;
                b2 = b0; a1=(-2.f*c_)*a0i; a2=(1.f-a)*a0i;
                break;
            case FTYPE_BANDPASS:
                a0i = 1.f/(1.f+a);
                b0 = a*a0i; b1=0; b2=-b0;
                a1=(-2.f*c_)*a0i; a2=(1.f-a)*a0i;
                break;
            case FTYPE_NOTCH:
                a0i = 1.f/(1.f+a);
                b0 = a0i; b1=(-2.f*c_)*a0i; b2=a0i;
                a1=b1; a2=(1.f-a)*a0i;
                break;
            case FTYPE_PEAKING: {
                float A = pow10f(gainDb / 40.f);
                a0i = 1.f/(1.f + a/A);
                b0 = (1.f + a*A)*a0i;
                b1 = (-2.f*c_)*a0i;
                b2 = (1.f - a*A)*a0i;
                a1 = b1; a2 = (1.f - a/A)*a0i;
                break;
            }
            case FTYPE_LOWSHELF: {
                float A = pow10f(gainDb / 40.f);
                float sq = 2.f*sqrtf(A)*a;
                a0i = 1.f/((A+1.f)+(A-1.f)*c_+sq);
                b0 = A*((A+1.f)-(A-1.f)*c_+sq)*a0i;
                b1 = 2.f*A*((A-1.f)-(A+1.f)*c_)*a0i;
                b2 = A*((A+1.f)-(A-1.f)*c_-sq)*a0i;
                a1 = -2.f*((A-1.f)+(A+1.f)*c_)*a0i;
                a2 = ((A+1.f)+(A-1.f)*c_-sq)*a0i;
                break;
            }
            case FTYPE_HIGHSHELF: {
                float A = pow10f(gainDb / 40.f);
                float sq = 2.f*sqrtf(A)*a;
                a0i = 1.f/((A+1.f)-(A-1.f)*c_+sq);
                b0 = A*((A+1.f)+(A-1.f)*c_+sq)*a0i;
                b1 = -2.f*A*((A-1.f)+(A+1.f)*c_)*a0i;
                b2 = A*((A+1.f)+(A-1.f)*c_-sq)*a0i;
                a1 = 2.f*((A-1.f)-(A+1.f)*c_)*a0i;
                a2 = ((A+1.f)-(A-1.f)*c_+sq)*a0i;
                break;
            }
            case FTYPE_ALLPASS:
                /* Audio EQ Cookbook — all-pass 2nd order */
                a0i = 1.f/(1.f+a);
                b0 = (1.f-a)*a0i; b1=(-2.f*c_)*a0i; b2=1.f;
                a1 = b1; a2 = (1.f-a)*a0i;
                break;
            case FTYPE_RESONANT:
                /* Resonant LP — same pole pair as LOWPASS; second BiquadEQ stage
                 * is applied externally for 24 dB/oct + soft saturation.       */
                a0i = 1.f/(1.f+a);
                b0 = ((1.f-c_)*0.5f)*a0i;
                b1 = (1.f-c_)*a0i;
                b2 = b0; a1=(-2.f*c_)*a0i; a2=(1.f-a)*a0i;
                break;
            default: b0=1;b1=b2=a1=a2=0; break;
        }
    }
};

static inline float GlobalEqGainDb(uint8_t type)
{
    /* The master payload has no EQ-gain field. A fixed musical boost keeps
     * PEAK/LOW SHELF/HIGH SHELF distinct instead of becoming identity filters. */
    return (type == FTYPE_PEAKING || type == FTYPE_LOWSHELF
            || type == FTYPE_HIGHSHELF) ? 6.0f : 0.0f;
}

/* ═══════════════════════════════════════════════════════════════════
 *  11. DaisySP MASTER FX
 * ═══════════════════════════════════════════════════════════════════ */
static DelayLine<float, MAX_DELAY_SAMPLES> DSY_SDRAM_BSS masterDelay;
DSY_SDRAM_BSS static ReverbSc   masterReverb;
DSY_SDRAM_BSS static ChorusEngine masterChorusL;
DSY_SDRAM_BSS static ChorusEngine masterChorusR;
static Tremolo    masterTremolo;
static Compressor masterComp;
DSY_SDRAM_BSS static Phaser     masterPhaserL;
DSY_SDRAM_BSS static Phaser     masterPhaserR;
DSY_SDRAM_BSS static Flanger    masterFlangerL;
DSY_SDRAM_BSS static Flanger    masterFlangerR;

/* Delay */
static bool  delayActive   = false;
static bool  delayRouted   = true;
static float delayTime     = 250.0f;
static float delayFeedback = 0.3f;
static float delayMix      = 0.3f;

/* Reverb */
static bool  reverbActive   = false;
static bool  reverbRouted   = true;
static float reverbFeedback = 0.85f;
static float reverbLpFreq   = 8000.0f;
static float reverbMix      = 0.3f;

/* Chorus */
static bool  chorusActive = false;
static bool  chorusRouted = true;
static float chorusMix    = 0.4f;

/* Tremolo */
static bool  tremoloActive = false;
static bool  tremoloRouted = true;

/* Compressor */
static bool  compActive = false;
static bool  compRouted = true;

/* Phaser */
static bool  phaserActive   = false;
static bool  phaserRouted   = true;
static float phaserDepth    = 0.4f;

/* Flanger (DaisySP) */
static bool  flangerActive  = false;
static bool  flangerRouted  = true;
static float flangerRate    = 0.5f;
static float flangerDepth   = 0.5f;
static float flangerFb      = 0.3f;
static float flangerMix     = 0.3f;

/* Wavefolder + Limiter */
static float waveFolderGain = 1.0f;
static bool  waveFolderRouted = true;
static bool  limiterActive  = false;
static bool  limiterRouted  = true;

/* Autowah (DaisySP) */
static Autowah    masterAutowahL;
static Autowah    masterAutowahR;
static bool  autowahActive  = false;
static bool  autowahRouted  = true;
static float autowahLevel   = 0.5f;
static float autowahMix     = 0.5f;

/* Stereo Width (Mid-Side) — 100 = normal, 0 = mono, 200 = super wide */
static float stereoWidth    = 1.0f;  /* 0..2 mapped from 0..200% */

/* Tape Stop effect — global pitch ramp to 0 */
static bool  tapeStopActive = false;
static float tapeStopSpeed  = 1.0f;  /* current speed multiplier (1→0 on stop) */
static float tapeStopRate   = 0.0001f; /* ramp-down rate per sample */

/* Beat Repeat — circular buffer of master output */
#define BEAT_REPEAT_BUF_SIZE 96000  /* 2 seconds @ 48kHz */
DSY_SDRAM_BSS static float beatRepBufL[BEAT_REPEAT_BUF_SIZE];
DSY_SDRAM_BSS static float beatRepBufR[BEAT_REPEAT_BUF_SIZE];
static bool     beatRepActive = false;
static uint8_t  beatRepDiv    = 0;   /* 0=off, 2=1/2, 4=1/4, 8=1/8, 16=1/16, 32=1/32 */
static uint32_t beatRepLen    = 0;   /* samples per slice */
static uint32_t beatRepPos    = 0;
static bool     beatRepCapturing = false;
static bool     beatRepPlaying = false;

/* Ping-Pong Delay — second delay line for stereo */
static DelayLine<float, MAX_DELAY_SAMPLES> DSY_SDRAM_BSS masterDelayR;
static bool  delayPingPong  = false;

/* Stereo Chorus mode */
static bool  chorusStereoMode = true;  /* default: stereo for wider mix */

/* Early Reflections (4 taps before ReverbSc) */
#define ER_TAPS 6
static DelayLine<float, 4800> DSY_SDRAM_BSS erDelayL;  /* 100ms max */
static DelayLine<float, 4800> DSY_SDRAM_BSS erDelayR;
static DelayLine<float, 4800> DSY_SDRAM_BSS combDelayL;
static DelayLine<float, 4800> DSY_SDRAM_BSS combDelayR;
static bool  erActive  = false;
static bool  erRouted  = true;
static float erMix     = 0.15f;
static const float erTapTimesL[ER_TAPS] = { 7.f, 13.f, 19.f, 29.f, 41.f, 53.f };  /* ms */
static const float erTapTimesR[ER_TAPS] = { 11.f, 17.f, 23.f, 37.f, 47.f, 59.f }; /* ms */
static const float erTapGains[ER_TAPS]  = { 0.8f, 0.65f, 0.5f, 0.4f, 0.3f, 0.22f };

/* Choke Groups — pad → group (0 = none, 1-8 = group) */
static uint8_t chokeGroup[MAX_PADS];

/* Song Mode — chain of pattern+repeats */
#define SONG_MAX_ENTRIES 128
struct SongEntry { uint8_t pattern; uint8_t repeats; };
static SongEntry songChain[SONG_MAX_ENTRIES];
static uint8_t songLength    = 0;
static bool    songPlaying   = false;
static uint8_t songIdx       = 0;
static uint8_t songRepeatCnt = 0;

/* Expanded LFO targets */
enum TrackLfoTargetEx : uint8_t {
    LFO_TGT_GAIN_EX   = 0,
    LFO_TGT_PAN_EX    = 1,
    LFO_TGT_FILTER_EX = 2,
    LFO_TGT_PITCH     = 3,
    LFO_TGT_ECHO_TIME = 4,
    LFO_TGT_DIST_DRIVE= 5,
    LFO_TGT_CRUSH     = 6,
    LFO_TGT_SEND_REV  = 7,
    LFO_TGT_SEND_DEL  = 8,
};

/* DaisySP Ladder filter for master (used when gFilterType == FTYPE_LADDER) */
static LadderFilter  masterLadderL, masterLadderR;

/* DaisySP SVF filter for master (used when gFilterType == FTYPE_SVF_*) */
static Svf     masterSvfL, masterSvfR;

/* ═══════════════════════════════════════════════════════════════════
 *  12. GLOBAL FILTER STATE
 * ═══════════════════════════════════════════════════════════════════ */
static BiquadEQ  gFilterL, gFilterR;
static BiquadEQ  gFilter2L, gFilter2R; /* 2nd stage para FTYPE_RESONANT global */
static bool    gFilterRouted  = true;
static uint8_t gFilterType    = FTYPE_NONE;
static float   gFilterCutoff  = 10000.0f;
static float   gFilterQ       = 0.707f;
/* CMD_FILTER_CUTOFF/RESONANCE (a knob/slider actively moving) write the
 * target above and let UpdateGlobalFilterSmoothing() ease these toward it
 * once per audio block — recomputing full filter coefficients on every raw
 * MIDI-CC step used to jump instantly, which is what made a live sweep
 * sound "brusco" (a series of tiny clicks) instead of a smooth glide.
 * CMD_FILTER_SET (a full preset/kit recall) applies instantly and snaps
 * these to match, same as loading a kit shouldn't visibly "glide" in. */
static float   gFilterCutoffSm = 10000.0f;
static float   gFilterQSm      = 0.707f;
/* CMD_FILTER_MORPH target — only meaningful when gFilterType ==
 * FTYPE_SVF_MORPH. Set directly (not smoothed like cutoff/Q above): it
 * crossfades between the SVF's own outputs sample-by-sample already, so a
 * knob sweep is inherently continuous with no coefficient jump to declick. */
static float   gFilterMorph    = 0.0f;
static uint8_t gFilterBitDepth= 16;
static float   gFilterDist    = 0.0f;
static uint8_t gFilterDistMode= DMODE_SOFT;
static uint32_t gFilterSrReduce = 0;  /* 0 = disabled */
static float   gSrHoldL = 0, gSrHoldR = 0;
static uint32_t gSrPhase = 0;
static bool     gSrPrimed = false;

static bool* GetMasterFxRouteFlag(uint8_t fxId)
{
    switch(fxId)
    {
        case MASTER_FX_ROUTE_FILTER:     return &gFilterRouted;
        case MASTER_FX_ROUTE_DELAY:      return &delayRouted;
        case MASTER_FX_ROUTE_PHASER:     return &phaserRouted;
        case MASTER_FX_ROUTE_FLANGER:    return &flangerRouted;
        case MASTER_FX_ROUTE_COMP:       return &compRouted;
        case MASTER_FX_ROUTE_REVERB:     return &reverbRouted;
        case MASTER_FX_ROUTE_CHORUS:     return &chorusRouted;
        case MASTER_FX_ROUTE_TREMOLO:    return &tremoloRouted;
        case MASTER_FX_ROUTE_WAVEFOLDER: return &waveFolderRouted;
        case MASTER_FX_ROUTE_LIMITER:    return &limiterRouted;
        case MASTER_FX_ROUTE_AUTOWAH:    return &autowahRouted;
        case MASTER_FX_ROUTE_EARLY_REF:  return &erRouted;
        default:                         return nullptr;
    }
}

static inline bool IsGlobalFilterEngaged()
{
    return gFilterRouted
        && (gFilterType != FTYPE_NONE
            || gFilterBitDepth < 16
            || fabsf(gFilterDist) > 0.0001f
            || gFilterSrReduce > 0);
}

static inline bool IsDelayEngaged()      { return delayRouted && delayActive && delayMix > 0.0001f; }
static inline bool IsPhaserEngaged()     { return phaserRouted && phaserActive; }
static inline bool IsFlangerEngaged()    { return flangerRouted && flangerActive && flangerMix > 0.0001f; }
static inline bool IsCompEngaged()       { return compRouted && compActive; }
static inline bool IsReverbEngaged()     { return reverbRouted && reverbActive && reverbMix > 0.0001f; }
static inline bool IsChorusEngaged()     { return chorusRouted && chorusActive && chorusMix > 0.0001f; }
static inline bool IsTremoloEngaged()    { return tremoloRouted && tremoloActive; }
static inline bool IsWaveFolderEngaged() { return waveFolderRouted && waveFolderGain > 1.01f; }
static inline bool IsLimiterEngaged()    { return limiterRouted && limiterActive; }
static inline bool IsAutowahEngaged()    { return autowahRouted && autowahActive; }
static inline bool IsEarlyRefEngaged()   { return erRouted && erActive && erMix > 0.0001f; }

/* ═══════════════════════════════════════════════════════════════════
 *  13. PER-PAD STATE
 * ═══════════════════════════════════════════════════════════════════ */
static bool  padLoop[MAX_PADS];
static bool  padReverse[MAX_PADS];
static float padPitch[MAX_PADS];
static int16_t trkPitchCents[MAX_PADS];  // modulación de pitch por track en centésimas (LFO / UI)

/* Pad filter */
static BiquadEQ  padFilter[MAX_PADS];
static uint8_t padFilterType[MAX_PADS];
static float   padFilterCut[MAX_PADS];
static float   padFilterQ[MAX_PADS];
static float   padFilterGain[MAX_PADS];  /* PEAKING/SHELF only */
/* Actual coefficients driver, eased toward the target above once per audio
 * block by UpdatePadFilterSmoothing() — see gFilterCutoffSm for why. */
static float   padFilterCutSm[MAX_PADS];
static float   padFilterQSm[MAX_PADS];

/* Pad distortion + bitcrush */
static float   padDistDrive[MAX_PADS];
static uint8_t padDistMode[MAX_PADS];   // 0=soft 1=hard 2=tube(asymm) 3=fuzz
static uint8_t padBitDepth[MAX_PADS];

/* Stutter */
static bool     padStutterOn[MAX_PADS];
static uint16_t padStutterIval[MAX_PADS];
static uint16_t padStutterCnt[MAX_PADS];

/* ═══════════════════════════════════════════════════════════════════
 *  14. PER-TRACK MIXER + FX
 * ═══════════════════════════════════════════════════════════════════ */
static float trackReverbSend[MAX_PADS];
static float trackDelaySend[MAX_PADS];
static float trackChorusSend[MAX_PADS];
static float trackPanF[MAX_PADS];          /* -1.0..+1.0 */
static bool  trackMute[MAX_PADS];
static bool  trackSolo[MAX_PADS];
static bool  anySolo = false;

/* Per-track FX routing (false = FX chain bypassed; auto-enabled when ESP32 sends FX commands) */
static bool    trkFxRouted[MAX_PADS];  /* default false; auto-set true on CMD_TRACK_FILTER etc. */

/* Per-track filter */
static BiquadEQ  trkFilter[MAX_PADS];
static BiquadEQ  trkFilter2[MAX_PADS]; /* 2nd stage for FTYPE_RESONANT (24dB/oct) */
static uint8_t trkFilterType[MAX_PADS];
static float   trkFilterCut[MAX_PADS];
static float   trkFilterQ[MAX_PADS];
static float   trkFilterGain[MAX_PADS];  /* PEAKING/SHELF only */
/* Actual coefficients driver, eased toward the target above once per audio
 * block by UpdateTrackFilterSmoothing() — see gFilterCutoffSm for why.
 * Tracks under LFO->filter modulation are skipped: that path already
 * recomputes every sample from trkFilterCut as its center, which is its
 * own, already-continuous form of movement. */
static float   trkFilterCutSm[MAX_PADS];
static float   trkFilterQSm[MAX_PADS];

/* Per-track distortion + bitcrush */
static float   trkDistDrive[MAX_PADS];
static uint8_t trkDistMode[MAX_PADS];
static uint8_t trkBitDepth[MAX_PADS];

/* Per-track echo (delay buf in SDRAM) */
DSY_SDRAM_BSS static float trkEchoBuf[MAX_PADS][TRACK_ECHO_SIZE];
static bool     trkEchoActive[MAX_PADS];
static float    trkEchoDelay[MAX_PADS];
static float    trkEchoFb[MAX_PADS];
static float    trkEchoMix[MAX_PADS];
static uint32_t trkEchoWp[MAX_PADS];

/* Per-track flanger (DaisySP) */
DSY_SDRAM_BSS static Flanger trkFlanger[MAX_PADS];
static bool     trkFlgActive[MAX_PADS];
static float    trkFlgDepth[MAX_PADS];
static float    trkFlgRate[MAX_PADS];
static float    trkFlgFb[MAX_PADS];
static float    trkFlgMix[MAX_PADS];

/* Per-track compressor */
static bool  trkCompActive[MAX_PADS];
static float trkCompThresh[MAX_PADS];
static float trkCompRatio[MAX_PADS];
static float trkCompExp[MAX_PADS];   /* pre-computed: 1.f - 1.f/ratio */
static float trkCompEnv[MAX_PADS];

/* Per-track EQ (3-band: low shelf 200Hz, mid peak 1kHz, high shelf 4kHz) */
static BiquadEQ trkEqLow[MAX_PADS];
static BiquadEQ trkEqMid[MAX_PADS];
static BiquadEQ trkEqHigh[MAX_PADS];
static int8_t trkEqLowDb[MAX_PADS];
static int8_t trkEqMidDb[MAX_PADS];
static int8_t trkEqHighDb[MAX_PADS];

/* Per-track LFO (interno DSP, configurable desde host) */
enum TrackLfoWave : uint8_t { LFO_WAVE_SINE = 0, LFO_WAVE_TRI = 1, LFO_WAVE_SH = 2 };
enum TrackLfoTarget : uint8_t { LFO_TGT_GAIN = 0, LFO_TGT_PAN = 1, LFO_TGT_FILTER = 2 };
static bool    trkLfoActive[MAX_PADS];
static uint8_t trkLfoWave[MAX_PADS];
static uint8_t trkLfoTarget[MAX_PADS];
static float   trkLfoRate[MAX_PADS];
static float   trkLfoDepth[MAX_PADS];
static float   trkLfoPhase[MAX_PADS];
static float   trkLfoSH[MAX_PADS];

/* Per-track AD envelope (aplicada por voz sampler) */
static bool    trkEnvAdActive[MAX_PADS];
static float   trkEnvAttackMs[MAX_PADS];
static float   trkEnvDecayMs[MAX_PADS];

/* ═══════════════════════════════════════════════════════════════════
 *  15. SIDECHAIN
 * ═══════════════════════════════════════════════════════════════════ */
static bool     scActive    = false;
static uint8_t  scSrc       = 0;
static uint16_t scDstMask   = 0;
static float    scAmount    = 0.5f;
static float    scAttackK   = 0.5f;
static float    scReleaseK  = 0.1f;
static float    scEnv       = 0.0f;

/* ═══════════════════════════════════════════════════════════════════
 *  16. SD CARD (SPI3 master — módulo 6 pines)
 *  Conexión: CS=D0(PB12) SCK=D2(PC10) MISO=D1(PC11) MOSI=D6(PC12)
 * ═══════════════════════════════════════════════════════════════════ */
static SpiHandle  sd_spi;         /* SPI3 master for SD card          */
static GPIO       sd_cs;           /* D0 = PB12 for CS (GPIO manual)   */
static FATFS      sdFatFs;        /* FatFS filesystem object           */
static bool    sdPresent = false;
static char    currentKitName[32] = "";
static uint8_t sd_card_type = 0;  /* 0=none 1=SDv1 2=SDv2 6=SDHC      */
/* Boot diagnostics exported in CMD_GET_STATUS bytes 76..79. FatFS result
 * values are kept verbatim so the P4 can distinguish mount/path/WAV errors. */
static uint8_t sdMountResult = (uint8_t)FR_NOT_READY;
static uint8_t sdRootResult = (uint8_t)FR_NOT_READY;
static uint8_t sdBootLoaded = 0;
static uint8_t sdLoadFailures = 0;
/* Low-level SPI diagnostics exported after the FatFS fields. These values
 * identify a wiring/protocol failure without needing a serial terminal. */
enum SdDiagStage : uint8_t {
    SD_DIAG_OK = 0,
    SD_DIAG_SPI_INIT = 1,
    SD_DIAG_POWER_CLOCKS = 2,
    SD_DIAG_CMD0 = 3,
    SD_DIAG_CMD8 = 4,
    SD_DIAG_ACMD41 = 5,
    SD_DIAG_CMD58 = 6,
    SD_DIAG_CMD16 = 7,
    SD_DIAG_READ_CMD = 8,
    SD_DIAG_READ_TOKEN = 9,
    SD_DIAG_READY_TIMEOUT = 10,
    SD_DIAG_SPI_IO = 11
};
static uint8_t  sdDiagStage = SD_DIAG_SPI_INIT;
static uint8_t  sdLastCommand = 0xFF;
static uint8_t  sdLastResponse = 0xFF;
static uint8_t  sdLastDataToken = 0xFF;
static uint16_t sdSpiErrors = 0;

/* ── SD SPI low-level helpers ───────────────────────────────────── */
static inline void SD_CS_LOW()  { sd_cs.Write(false); }
static inline void SD_CS_HIGH() { sd_cs.Write(true);  }

static uint8_t SD_TxRx(uint8_t tx){
    uint8_t rx = 0xFF;
    if(sd_spi.BlockingTransmitAndReceive(&tx, &rx, 1, 10)
       != SpiHandle::Result::OK){
        sdSpiErrors++;
        sdDiagStage = SD_DIAG_SPI_IO;
        return 0xFF;
    }
    return rx;
}

static bool SD_WaitReady(uint32_t timeout_ms){
    uint32_t start = System::GetNow();
    do {
        if(SD_TxRx(0xFF) == 0xFF) return true;
    } while((System::GetNow() - start) < timeout_ms);
    return false;
}

static void SD_Deselect()
{
    SD_CS_HIGH();
    SD_TxRx(0xFF); /* release MISO on the following clock */
}

static bool SD_Select()
{
    SD_CS_LOW();
    SD_TxRx(0xFF);
    if(SD_WaitReady(500)) return true;
    SD_Deselect();
    sdDiagStage = SD_DIAG_READY_TIMEOUT;
    return false;
}

/* ── SD SPI command protocol ────────────────────────────────────── */
#define SD_CMD0    (0x40+0)   /* GO_IDLE_STATE          */
#define SD_CMD8    (0x40+8)   /* SEND_IF_COND           */
#define SD_CMD9    (0x40+9)   /* SEND_CSD               */
#define SD_CMD12   (0x40+12)  /* STOP_TRANSMISSION      */
#define SD_CMD16   (0x40+16)  /* SET_BLOCKLEN           */
#define SD_CMD17   (0x40+17)  /* READ_SINGLE_BLOCK      */
#define SD_CMD18   (0x40+18)  /* READ_MULTIPLE_BLOCK    */
#define SD_CMD24   (0x40+24)  /* WRITE_BLOCK            */
#define SD_CMD25   (0x40+25)  /* WRITE_MULTIPLE_BLOCK   */
#define SD_CMD55   (0x40+55)  /* APP_CMD                */
#define SD_CMD58   (0x40+58)  /* READ_OCR               */
#define SD_ACMD41  (0xC0+41)  /* SD_SEND_OP_COND (app)  */

static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg)
{
    uint8_t n, res;
    if(cmd & 0x80){                       /* ACMD: send CMD55 first */
        cmd &= 0x7F;
        res = SD_SendCmd(SD_CMD55, 0);
        if(res > 1) return res;
    }
    /* CMD12 terminates an active multi-block stream and must keep CS low.
     * Every other command starts from a clean deselect/select cycle and waits
     * until the card releases DO. This is required by cards that remain busy
     * briefly after ACMD41/CMD58. */
    if(cmd != SD_CMD12){
        SD_Deselect();
        if(!SD_Select()){
            sdLastCommand = (uint8_t)(cmd & 0x3Fu);
            sdLastResponse = 0xFF;
            return 0xFF;
        }
    }
    sdLastCommand = (uint8_t)(cmd & 0x3Fu);

    /* Command packet */
    SD_TxRx(cmd);
    SD_TxRx((uint8_t)(arg >> 24));
    SD_TxRx((uint8_t)(arg >> 16));
    SD_TxRx((uint8_t)(arg >> 8));
    SD_TxRx((uint8_t)arg);
    n = 0x01;
    if(cmd == SD_CMD0) n = 0x95;          /* Valid CRC for CMD0(0)  */
    if(cmd == SD_CMD8) n = 0x87;          /* Valid CRC for CMD8     */
    SD_TxRx(n);

    if(cmd == SD_CMD12) SD_TxRx(0xFF);    /* Skip stuff byte        */
    n = 10;
    do { res = SD_TxRx(0xFF); } while((res & 0x80) && --n);
    sdLastResponse = res;
    return res;
}

static bool SD_RxDataBlock(uint8_t* buf, uint32_t cnt)
{
    uint8_t token;
    uint32_t start = System::GetNow();
    do { token = SD_TxRx(0xFF); }
    while(token == 0xFF && (System::GetNow() - start) < 200);
    sdLastDataToken = token;
    if(token != 0xFE) return false;
    for(uint32_t i = 0; i < cnt; i++) buf[i] = SD_TxRx(0xFF);
    SD_TxRx(0xFF); SD_TxRx(0xFF);        /* Discard CRC            */
    return true;
}

static bool SD_TxDataBlock(const uint8_t* buf, uint8_t token)
{
    if(!SD_WaitReady(500)) return false;
    SD_TxRx(token);
    if(token != 0xFD){
        for(uint32_t i = 0; i < 512; i++) SD_TxRx(buf[i]);
        SD_TxRx(0xFF); SD_TxRx(0xFF);    /* Dummy CRC              */
        uint8_t resp = SD_TxRx(0xFF);
        if((resp & 0x1F) != 0x05) return false;
    }
    return true;
}

/* ── FatFS diskio callbacks (registered via FATFS_LinkDriver) ──── */
static DSTATUS SPISD_DiskStatus(BYTE lun){
    return sd_card_type ? 0 : STA_NOINIT;
}

static DSTATUS SPISD_DiskInit(BYTE lun)
{
    uint8_t n, ty, ocr[4];
    SD_CS_HIGH();
    sdDiagStage = SD_DIAG_POWER_CLOCKS;
    /* Give the regulator/card time to settle, then provide considerably more
     * than the minimum 74 clocks with CS high. Cheap 6-pin modules and larger
     * cards are often not ready at the very first 80 clocks after boot. */
    System::Delay(20);
    for(n = 0; n < 20; n++) SD_TxRx(0xFF);

    ty = 0;
    sdDiagStage = SD_DIAG_CMD0;
    if(SD_SendCmd(SD_CMD0, 0) == 1){         /* Enter idle             */
        uint32_t start = System::GetNow();
        sdDiagStage = SD_DIAG_CMD8;
        if(SD_SendCmd(SD_CMD8, 0x1AA) == 1){ /* SDv2 ?                 */
            for(n = 0; n < 4; n++) ocr[n] = SD_TxRx(0xFF);
            if(ocr[2] == 0x01 && ocr[3] == 0xAA){
                sdDiagStage = SD_DIAG_ACMD41;
                while((System::GetNow() - start) < 1000)
                    if(SD_SendCmd(SD_ACMD41, 1UL << 30) == 0) break;
                sdDiagStage = SD_DIAG_CMD58;
                if((System::GetNow() - start) < 1000
                   && SD_SendCmd(SD_CMD58, 0) == 0){
                    for(n = 0; n < 4; n++) ocr[n] = SD_TxRx(0xFF);
                    ty = (ocr[0] & 0x40) ? 6 : 2; /* SDHC(6) or SDv2(2)   */
                }
            }
        } else {
            sdDiagStage = SD_DIAG_ACMD41;
            if(SD_SendCmd(SD_ACMD41, 0) <= 1){ ty = 1; /* SDv1              */
                while((System::GetNow() - start) < 1000)
                    if(SD_SendCmd(SD_ACMD41, 0) == 0) break;
            }
        }
        /* Both SDv1 and non-HC SDv2 cards use byte addressing. FatFs always
         * transfers 512-byte sectors, so explicitly select that block size. */
        if(ty && !(ty & 4)){
            sdDiagStage = SD_DIAG_CMD16;
            if(SD_SendCmd(SD_CMD16, 512) != 0) ty = 0;
        }
    }
    SD_Deselect();
    sd_card_type = ty;
    return ty ? 0 : STA_NOINIT;
}

static DRESULT SPISD_DiskRead(BYTE lun, BYTE* buff, DWORD sector, UINT count)
{
    if(!sd_card_type) return RES_NOTRDY;
    if(!(sd_card_type & 4)) sector *= 512;
    sdDiagStage = SD_DIAG_READ_CMD;
    if(count == 1){
        if(SD_SendCmd(SD_CMD17, sector) == 0){
            sdDiagStage = SD_DIAG_READ_TOKEN;
            if(SD_RxDataBlock(buff, 512)) count = 0;
        }
    } else {
        if(SD_SendCmd(SD_CMD18, sector) == 0){
            sdDiagStage = SD_DIAG_READ_TOKEN;
            do {
                if(!SD_RxDataBlock(buff, 512)) break;
                buff += 512;
            } while(--count);
            SD_SendCmd(SD_CMD12, 0);
        }
    }
    SD_Deselect();
    if(count == 0) sdDiagStage = SD_DIAG_OK;
    return count ? RES_ERROR : RES_OK;
}

static DRESULT SPISD_DiskWrite(BYTE lun, const BYTE* buff, DWORD sector, UINT count)
{
    if(!sd_card_type) return RES_NOTRDY;
    if(!(sd_card_type & 4)) sector *= 512;
    if(count == 1){
        if(SD_SendCmd(SD_CMD24, sector) == 0
           && SD_TxDataBlock(buff, 0xFE)) count = 0;
    } else {
        if(SD_SendCmd(SD_CMD25, sector) == 0){
            do {
                if(!SD_TxDataBlock(buff, 0xFC)) break;
                buff += 512;
            } while(--count);
            SD_TxDataBlock(0, 0xFD);
        }
    }
    SD_Deselect();
    return count ? RES_ERROR : RES_OK;
}

static DRESULT SPISD_DiskIoctl(BYTE lun, BYTE cmd, void* buff)
{
    DRESULT res = RES_ERROR;
    uint8_t csd[16];
    if(!sd_card_type) return RES_NOTRDY;
    switch(cmd){
        case CTRL_SYNC:
            if(SD_Select()) res = RES_OK;
            SD_Deselect();
            break;
        case GET_SECTOR_COUNT:
            if(SD_SendCmd(SD_CMD9, 0) == 0 && SD_RxDataBlock(csd, 16)){
                DWORD n_sec;
                if((csd[0] >> 6) == 1){
                    n_sec = ((DWORD)(csd[7]&0x3F)<<16)|((DWORD)csd[8]<<8)|csd[9];
                    n_sec = (n_sec + 1) << 10;
                } else {
                    uint8_t nn = (csd[5]&0x0F)+((csd[10]&0x80)>>7)+((csd[9]&3)<<1)+2;
                    n_sec = ((DWORD)(csd[8]>>6)+((DWORD)csd[7]<<2)+((DWORD)(csd[6]&3)<<10)+1);
                    n_sec <<= (nn - 9);
                }
                *(DWORD*)buff = n_sec;
                res = RES_OK;
            }
            SD_Deselect();
            break;
        case GET_SECTOR_SIZE:
            *(WORD*)buff = 512; res = RES_OK; break;
        case GET_BLOCK_SIZE:
            *(DWORD*)buff = 1;  res = RES_OK; break;
        default: res = RES_PARERR;
    }
    return res;
}

static const Diskio_drvTypeDef SPISD_Driver = {
    SPISD_DiskInit,
    SPISD_DiskStatus,
    SPISD_DiskRead,
    SPISD_DiskWrite,
    SPISD_DiskIoctl
};

/* ═══════════════════════════════════════════════════════════════════
 *  16b. EVENT NOTIFICATION SYSTEM
 *  La Daisy es SPI esclava → no puede empujar datos al Master.
 *  Solución: cola circular de eventos. El Master descubre que hay
 *  eventos pendientes al ver eventCount > 0 en CMD_GET_STATUS,
 *  y luego llama CMD_GET_EVENTS para drenarlos.
 * ═══════════════════════════════════════════════════════════════════ */
#define EVT_SD_BOOT_DONE       0x01  /* Boot loading complete             */
#define EVT_SD_KIT_LOADED      0x02  /* Kit loaded by CMD_SD_LOAD_KIT     */
#define EVT_SD_SAMPLE_LOADED   0x03  /* Sample cargado por CMD_SD_LOAD_SAMPLE */
#define EVT_SD_KIT_UNLOADED    0x04  /* Kit descargado                    */
#define EVT_SD_ERROR           0x05  /* Error de SD                       */
#define EVT_SD_XTRA_LOADED     0x06  /* XTRA PADS cargados al boot        */

struct __attribute__((packed)) NotifyEvent {
    uint8_t  type;          /* EVT_SD_* */
    uint8_t  padCount;      /* cuántos pads afectados */
    uint8_t  padMaskLo;     /* bitmask pads 0-7  loaded */
    uint8_t  padMaskHi;     /* bitmask pads 8-15 loaded */
    uint8_t  padMaskXtra;   /* bitmask pads 16-23 loaded */
    uint8_t  reserved[3];
    char     name[24];      /* kit name / sample name */
};  /* 32 bytes */

#define EVT_QUEUE_SIZE 8
static NotifyEvent evtQueue[EVT_QUEUE_SIZE];
static volatile uint8_t evtHead = 0;  /* next write position  */
static volatile uint8_t evtTail = 0;  /* next read  position  */
static volatile uint8_t evtCount = 0; /* events in queue      */

static void CopyFixedString(char* dst, size_t dstSize, const char* src)
{
    if(dstSize == 0)
        return;
    size_t i = 0;
    if(src){
        while(i + 1 < dstSize && src[i]){
            dst[i] = src[i];
            i++;
        }
    }
    dst[i++] = 0;
    while(i < dstSize)
        dst[i++] = 0;
}

static bool JoinPath(char* dst, size_t dstSize, const char* left, const char* right)
{
    if(dstSize == 0)
        return false;
    size_t pos = 0;
    const char* parts[2] = { left ? left : "", right ? right : "" };
    for(const char* s = parts[0]; *s; ++s){
        if(pos + 1 >= dstSize){ dst[dstSize - 1] = 0; return false; }
        dst[pos++] = *s;
    }
    if(pos > 0 && dst[pos - 1] != '/'){
        if(pos + 1 >= dstSize){ dst[dstSize - 1] = 0; return false; }
        dst[pos++] = '/';
    }
    for(const char* s = parts[1]; *s; ++s){
        if(pos + 1 >= dstSize){ dst[dstSize - 1] = 0; return false; }
        dst[pos++] = *s;
    }
    dst[pos] = 0;
    return true;
}

static void PushEvent(uint8_t type, uint8_t padCount,
                      uint32_t padMask24, const char* name)
{
    if(evtCount >= EVT_QUEUE_SIZE){
        /* Queue full — overwrite oldest */
        evtTail = (evtTail + 1) % EVT_QUEUE_SIZE;
        evtCount--;
    }
    NotifyEvent& e = evtQueue[evtHead];
    memset(&e, 0, sizeof(e));
    e.type       = type;
    e.padCount   = padCount;
    e.padMaskLo  = (uint8_t)(padMask24 & 0xFF);
    e.padMaskHi  = (uint8_t)((padMask24 >> 8) & 0xFF);
    e.padMaskXtra= (uint8_t)((padMask24 >> 16) & 0xFF);
    CopyFixedString(e.name, sizeof(e.name), name);
    evtHead = (evtHead + 1) % EVT_QUEUE_SIZE;
    evtCount++;
}

static uint8_t PopEvents(NotifyEvent* dst, uint8_t maxEvents)
{
    uint8_t count = 0;
    while(evtCount > 0 && count < maxEvents){
        dst[count++] = evtQueue[evtTail];
        evtTail = (evtTail + 1) % EVT_QUEUE_SIZE;
        evtCount--;
    }
    return count;
}

/* ═══════════════════════════════════════════════════════════════════
 *  17. MISC STATS
 * ═══════════════════════════════════════════════════════════════════ */
static volatile uint32_t spiPktCnt = 0;
static volatile uint16_t spiErrCnt = 0;
static volatile uint16_t spiRingDrops = 0;  /* bytes perdidos por ring lleno */
static volatile uint32_t spiLastPacketMs = 0;
static volatile uint32_t spiLastTriggerMs = 0;

/* ═══════════════════════════════════════════════════════════════════
 *  17b. SYNTH ENGINE INSTANCES
 * ═══════════════════════════════════════════════════════════════════ */
static TR808::Kit synth808;
static TR909::Kit synth909;
static TR505::Kit synth505;
static TB303::Synth acid303;
static WavetableOsc wtOsc;
static SH101::Synth synthSH101;  /* I1: Roland SH-101 */
static FM2Op::Synth synthFM2Op;  /* I2: FM 2-op Yamaha */

/* Physical Modeling engine — DaisySP ModalVoice + StringVoice */
static ModalVoice  physModal;
static StringVoice physString;
static float physModalGain  = 0.8f;
static float physStringGain = 0.8f;
static bool  physModalActive = false;
static bool  physStringActive = false;

/* Noise/Texture engine — DaisySP Particle */
static Particle noisePart;
static float noisePartGain  = 0.6f;
static bool  noisePartActive = false;

static uint8_t trackWtNote[16];   /* nota MIDI por track WT, default C4=60 */
static uint8_t trackSH101Note[16]; /* nota MIDI por track SH101            */
static uint8_t trackFM2OpNote[16]; /* nota MIDI por track FM2Op            */
static float wtFilterCutoffState = 8000.0f;
static float wtFilterQState      = 0.707f;
static float wtLfoRateState      = 2.0f;
static float wtLfoDepthState     = 0.0f;
static WtLfoTarget wtLfoTargetState = WT_LFO_WAVE;

/* Forward-declare sanitizeF (defined in DSP HELPERS section) */
static inline float sanitizeF(float v);

static DcBlock dcBlockL, dcBlockR;

static inline uint8_t AudioCpuPercent()
{
    float load = audioLoadMeter.GetAvgCpuLoad();
    if(!isfinite(load) || load < 0.0f)
        load = 0.0f;
    if(load > 1.0f)
        load = 1.0f;
    return (uint8_t)(load * 100.0f + 0.5f);
}

static inline float AudioCpuPercentFromLoad(float load)
{
    if(!isfinite(load) || load < 0.0f)
        load = 0.0f;
    if(load > 1.0f)
        load = 1.0f;
    return load * 100.0f;
}

static inline float AudioCpuAvgPercent()
{
    return AudioCpuPercentFromLoad(audioLoadMeter.GetAvgCpuLoad());
}

static inline float AudioCpuPeakPercent()
{
    return AudioCpuPercentFromLoad(audioLoadMeter.GetMaxCpuLoad());
}

/* ═══════════════════════════════════════════════════════════════════
 *  Tabla de remap: padIndex del ESP32 → TR808::InstrumentId
 *  ESP32 envía: 0=BD 1=SD 2=CH 3=OH 4=CY 5=CP 6=RS 7=CB
 *               8=LT 9=MT 10=HT 11=MA 12=CL 13=HC 14=MC 15=LC
 * ═══════════════════════════════════════════════════════════════════ */
static const uint8_t padTo808[16] = {
    TR808::INST_KICK,      /* pad 0  = BD  → INST_KICK (0)      */
    TR808::INST_SNARE,     /* pad 1  = SD  → INST_SNARE (1)     */
    TR808::INST_HIHAT_C,   /* pad 2  = CH  → INST_HIHAT_C (3)   */
    TR808::INST_HIHAT_O,   /* pad 3  = OH  → INST_HIHAT_O (4)   */
    TR808::INST_CYMBAL,    /* pad 4  = CY  → INST_CYMBAL (15)   */
    TR808::INST_CLAP,      /* pad 5  = CP  → INST_CLAP (2)      */
    TR808::INST_RIMSHOT,   /* pad 6  = RS  → INST_RIMSHOT (13)  */
    TR808::INST_COWBELL,   /* pad 7  = CB  → INST_COWBELL (14)  */
    TR808::INST_LOW_TOM,   /* pad 8  = LT  → INST_LOW_TOM (5)  */
    TR808::INST_MID_TOM,   /* pad 9  = MT  → INST_MID_TOM (6)  */
    TR808::INST_HI_TOM,    /* pad 10 = HT  → INST_HI_TOM (7)   */
    TR808::INST_MARACAS,   /* pad 11 = MA  → INST_MARACAS (12)  */
    TR808::INST_CLAVES,    /* pad 12 = CL  → INST_CLAVES (11)   */
    TR808::INST_HI_CONGA,  /* pad 13 = HC  → INST_HI_CONGA (10) */
    TR808::INST_MID_CONGA, /* pad 14 = MC  → INST_MID_CONGA (9) */
    TR808::INST_LOW_CONGA, /* pad 15 = LC  → INST_LOW_CONGA (8) */
};

static const uint8_t padTo909[16] = {
    TR909::INST_KICK,      /* 0 BD */
    TR909::INST_SNARE,     /* 1 SD */
    TR909::INST_HIHAT_C,   /* 2 CH */
    TR909::INST_HIHAT_O,   /* 3 OH */
    TR909::INST_CRASH,     /* 4 CY */
    TR909::INST_CLAP,      /* 5 CP */
    TR909::INST_RIMSHOT,   /* 6 RS */
    TR909::INST_RIDE,      /* 7 CB/RD */
    TR909::INST_LOW_TOM,   /* 8 LT */
    TR909::INST_MID_TOM,   /* 9 MT */
    TR909::INST_HI_TOM,    /* 10 HT */
    TR909::INST_SHAKER,    /* 11 MA */
    TR909::INST_CLAVE,     /* 12 CL */
    TR909::INST_HI_PERC,   /* 13 HC */
    TR909::INST_MID_PERC,  /* 14 MC */
    TR909::INST_LOW_PERC,  /* 15 LC */
};

static const uint8_t padTo505[16] = {
    TR505::INST_KICK,      /* 0 BD */
    TR505::INST_SNARE,     /* 1 SD */
    TR505::INST_HIHAT_C,   /* 2 CH */
    TR505::INST_HIHAT_O,   /* 3 OH */
    TR505::INST_CYMBAL,    /* 4 CY */
    TR505::INST_CLAP,      /* 5 CP */
    TR505::INST_RIMSHOT,   /* 6 RS */
    TR505::INST_COWBELL,   /* 7 CB */
    TR505::INST_LOW_TOM,   /* 8 LT */
    TR505::INST_MID_TOM,   /* 9 MT */
    TR505::INST_HI_TOM,    /* 10 HT */
    TR505::INST_SHAKER,    /* 11 MA */
    TR505::INST_CLAVE,     /* 12 CL */
    TR505::INST_HI_PERC,   /* 13 HC */
    TR505::INST_MID_PERC,  /* 14 MC */
    TR505::INST_LOW_PERC,  /* 15 LC */
};

/* Hybrid 909: the original machine uses digital playback for closed/open
 * hats, crash and ride. Canonical pads 2,3,4,7 provide those four PCM slots. */
static bool synth909PcmMode = false;

static bool Is909PcmPad(uint8_t pad)
{
    return pad == 2 || pad == 3 || pad == 4 || pad == 7;
}

static uint8_t Bind909PcmFromLoadedPads()
{
    static const uint8_t pcmPads[] = {2, 3, 4, 7};
    synth909.ClearPcmSamples();
    uint8_t bound = 0;
    for(uint8_t i = 0; i < sizeof(pcmPads); i++){
        uint8_t pad = pcmPads[i];
        if(pad >= MAX_PADS)
            continue;
        int16_t* data = SamplePtr(pad);
        if(!sampleLoaded[pad] || data == nullptr || sampleLength[pad] == 0)
            continue;
        uint32_t sr = sampleRateHz[pad] ? sampleRateHz[pad] : SAMPLE_RATE;
        if(synth909.SetPcmSample(padTo909[pad], data, sampleLength[pad], (float)sr))
            bound++;
    }
    synth909PcmMode = true;
    return bound;
}

static void Unbind909PcmPad(uint8_t pad)
{
    if(Is909PcmPad(pad))
        synth909.ClearPcmSample(padTo909[pad]);
}

/* PCM mode binds the 16 currently loaded canonical pads to the TR-505
 * instrument map. It is opt-in (preset 5), so the default 808 kit can never
 * be mistaken for a 505 ROM. Missing slots keep procedural fallback. */
static bool synth505PcmMode = false;

static uint8_t Bind505PcmFromLoadedPads()
{
    synth505.ClearPcmSamples();
    uint8_t bound = 0;
    for(uint8_t pad = 0; pad < 16; pad++){
        int16_t* data = SamplePtr(pad);
        if(!sampleLoaded[pad] || data == nullptr || sampleLength[pad] == 0)
            continue;
        uint32_t sr = sampleRateHz[pad] ? sampleRateHz[pad] : SAMPLE_RATE;
        if(synth505.SetPcmSample(padTo505[pad], data, sampleLength[pad], (float)sr))
            bound++;
    }
    /* Keep PCM mode armed even when no slots are loaded yet: subsequent SD or
     * SPI loads bind themselves and missing instruments keep the fallback. */
    synth505PcmMode = true;
    return bound;
}

static void Unbind505PcmPad(uint8_t pad)
{
    if(pad < 16)
        synth505.ClearPcmSample(padTo505[pad]);
}

static const uint8_t padTo303Midi[16] = {
    36, 38, 41, 43,
    45, 48, 50, 53,
    55, 57, 60, 62,
    64, 67, 69, 72
};

/* Bitmask: qué engines están activos */
static constexpr float kDrumBusHeadroom = 0.70f;  // evita clipping al mezclar 808/909/505
static uint16_t synthActiveMask = 0x01FF;  /* all 9 engines active */
static uint8_t pianoSelectedEngine = SYNTH_ENGINE_303;

static inline bool IsPianoMelodicEngine(uint8_t engine)
{
    return engine == SYNTH_ENGINE_303 || engine == SYNTH_ENGINE_WTOSC ||
           engine == SYNTH_ENGINE_SH101 || engine == SYNTH_ENGINE_FM2OP ||
           engine == SYNTH_ENGINE_PHYS;
}

#ifndef RED808_ENABLE_INIT_FX
#define RED808_ENABLE_INIT_FX 1
#endif
static constexpr bool kEnableSynth505 = true;
static constexpr bool kAudioSafeMode = false; /* callback de audio real */
#ifndef RED808_BOOT_DIAG_MINIMAL
#define RED808_BOOT_DIAG_MINIMAL 0
#endif
#ifndef RED808_AUDIO_DIAG_MINIMAL
#define RED808_AUDIO_DIAG_MINIMAL 0
#endif
#ifndef RED808_BOOT_PROGRESS_DIAG
#define RED808_BOOT_PROGRESS_DIAG 0
#endif
static constexpr bool kBootDiagMinimal    = (RED808_BOOT_DIAG_MINIMAL    != 0); /* diagnóstico extremo: solo LED, sin audio ni FX */
static constexpr bool kAudioDiagMinimal   = (RED808_AUDIO_DIAG_MINIMAL   != 0); /* diagnóstico: solo audio callback + LED */
static constexpr bool kBootProgressDiag   = (RED808_BOOT_PROGRESS_DIAG  != 0); /* diagnóstico: parpadeos de progreso en boot para localizar crash */
static constexpr bool kEnableAudioStart = true; /* iniciar audio normal */
/* El CDC interno transporta paquetes binarios RED808. Los logs de texto se
 * desactivan para que nunca puedan intercalarse con una respuesta al P4. */
static constexpr bool kEnableStartLog = false;
static constexpr bool kEnableSynthCmdLog = true; /* diagnóstico temporal: preset/note routing */
static constexpr bool kEnableInitFx = (RED808_ENABLE_INIT_FX != 0);    /* diagnóstico: reactivar InitFX para aislar causa */
#ifndef RED808_STARTUP_TONE_TEST
#define RED808_STARTUP_TONE_TEST 0
#endif
#ifndef RED808_STARTUP_808_SELF_TEST
#define RED808_STARTUP_808_SELF_TEST 0
#endif
#ifndef RED808_STARTUP_SHOWCASE_DEMO
#define RED808_STARTUP_SHOWCASE_DEMO 0
#endif
#ifndef RED808_STARTUP_TONE_SECONDS
#define RED808_STARTUP_TONE_SECONDS 3
#endif
#ifndef RED808_STARTUP_STRESS_REPORT
#define RED808_STARTUP_STRESS_REPORT 0
#endif
#ifndef RED808_STARTUP_STRESS_SECONDS
#define RED808_STARTUP_STRESS_SECONDS 18
#endif
static constexpr bool kStartupToneTest = (RED808_STARTUP_TONE_TEST != 0); /* diagnóstico: tono directo 1kHz */
static constexpr bool kStartup808SelfTest = (RED808_STARTUP_808_SELF_TEST != 0); /* diagnóstico: prueba sampler/synth */
static constexpr bool kStartupShowcaseDemo = (RED808_STARTUP_SHOWCASE_DEMO != 0); /* demo musical autónoma */
static constexpr bool kStartupStressReport = (RED808_STARTUP_STRESS_REPORT != 0);
static constexpr uint32_t kStartupStressSeconds = RED808_STARTUP_STRESS_SECONDS;

/* SD boot-load: por defecto DESACTIVADO. Sin tarjeta SD, f_mount() fuerza un
 * acceso a un bus SPI3 con MISO flotante y el driver FatFS se cuelga de forma
 * intermitente en el arranque (se queda en la fase 3-4, no llega a inicializar
 * el SPI esclavo y el ESP32 ve la Daisy OFFLINE). En este montaje no hay SD:
 * los samples vienen de QSPI o por SPI desde el ESP32. Ponlo a 1 solo si vas
 * a usar tarjeta microSD. */
#ifndef RED808_ENABLE_SD_BOOTLOAD
#define RED808_ENABLE_SD_BOOTLOAD 0
#endif
static constexpr bool kEnableSdBootLoad = (RED808_ENABLE_SD_BOOTLOAD != 0);

static void ApplySynthPreset(uint8_t engine, uint8_t presetId);
static void ReleaseAllSynthEngines();
static constexpr bool kBypassIncomingCrc = false; /* producción: validar CRC de comandos entrantes */
static constexpr bool kAcceptOneBasedPadIndex = false; /* ESP32 envía 0-based (pad 0=BD, 1=SD, etc.) */
static constexpr bool kTriggerSynthOnLiveCmd = false; /* producción: no superponer synth al disparo de sampler */
static constexpr bool kForceMasterGainDebug = false; /* producción: respetar master volume del host */
static constexpr bool kSpiSingleFrame10 = true; /* compat: master envía trigger en 1 frame de 10 bytes */
static bool perfStressMode = false;
static uint8_t perfStressProfile = 0;
static uint32_t perfStressNextMs = 0;
static uint8_t perfStressStep = 0;
static bool audioFxShed = false;
static bool startupStressReportActive = false;
static bool startupStressReportDone = false;
static uint32_t startupStressStartMs = 0;
static uint32_t startupStressLastReportMs = 0;
static uint8_t startupStressPhase = 255;
static constexpr uint32_t kStartupStressArmDelayMs = 8000u;

/* PRNG for crackle/noise FX */
static uint32_t noiseState = 0x12345678;
static uint32_t FastRand(){
    noiseState ^= noiseState<<13;
    noiseState ^= noiseState>>17;
    noiseState ^= noiseState<<5;
    return noiseState;
}
static float RandFloat(){
    return ((float)(int32_t)FastRand()) / 2147483648.0f;
}

/* ── Startup section announcer (retro-robótico por formantes) ── */
enum StartupSectionTag : uint8_t {
    SEC_SAMPLERS = 0,
    SEC_808,
    SEC_909,
    SEC_505,
    SEC_303,
    SEC_XTRAS,
    SEC_SAMPLER_FX,
    SEC_TECHNO,
    SEC_ELECTRO,
    SEC_AMBIENT,
    SEC_COUNT
};

static FormantOscillator startupAnnounceOsc;
static bool             startupAnnounceActive = false;
static float            startupAnnounceEnv    = 0.0f;
static uint32_t         startupAnnounceRemain = 0;

static void QueueStartupSectionTag(StartupSectionTag sec)
{
    static const char* kWords[SEC_COUNT] = {
        "SAMPLERS", "808", "909", "505", "303", "XTRAS", "FX JAM", "TECHNO", "ELECTRO", "AMBIENT"
    };
    static const float kCarrier[SEC_COUNT] = {
        86.0f, 92.0f, 98.0f, 104.0f, 110.0f, 116.0f, 94.0f, 88.0f, 96.0f, 80.0f
    };
    static const float kFormant[SEC_COUNT] = {
        820.0f, 940.0f, 980.0f, 910.0f, 860.0f, 760.0f, 1030.0f, 700.0f, 1080.0f, 640.0f
    };
    static const float kPhaseShift[SEC_COUNT] = {
        0.28f, 0.32f, 0.38f, 0.45f, 0.52f, 0.62f, 0.34f, 0.58f, 0.42f, 0.66f
    };

    uint8_t idx = (uint8_t)sec;
    if(idx >= SEC_COUNT) return;

    startupAnnounceOsc.SetCarrierFreq(kCarrier[idx]);
    startupAnnounceOsc.SetFormantFreq(kFormant[idx]);
    startupAnnounceOsc.SetPhaseShift(kPhaseShift[idx]);
    startupAnnounceEnv    = 1.0f;
    startupAnnounceRemain = (uint32_t)(SAMPLE_RATE * 0.22f);
    startupAnnounceActive = true;

    /* Etiqueta textual de sección (si el log USB está activo) */
    hw.PrintLine(">>> %s <<<", kWords[idx]);
}

/* ═══════════════════════════════════════════════════════════════════
 *  18. CRC16 MODBUS
 * ═══════════════════════════════════════════════════════════════════ */
static uint16_t crc16(const uint8_t* d, uint16_t len){
    uint16_t crc = 0xFFFF;
    for(uint16_t i = 0; i < len; i++){
        crc ^= d[i];
        for(uint8_t j = 0; j < 8; j++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

/* ═══════════════════════════════════════════════════════════════════
 *  19. DSP HELPERS
 * ═══════════════════════════════════════════════════════════════════ */
static inline float clampF(float v, float lo, float hi){
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ── Filter parameter smoothing ──────────────────────────────────────────
 * A live cutoff/resonance tweak (knob, touch slider, MIDI CC) used to call
 * BiquadEQ::SetType() immediately with the raw new value: every step of the
 * sweep recomputed the filter's poles from scratch while its z1/z2 history
 * kept running under the OLD coefficients, so each step produced its own
 * small discontinuity — a fast sweep sounded like a series of clicks
 * instead of a glide ("muy brusco"). These three run once per audio block
 * (not per sample — cheap) and ease the *_Sm shadow that actually drives
 * SetType()/SetFreq()/SetRes() toward the raw target a fraction of the way
 * each block, so a sweep crosses several blocks smoothly instead of
 * snapping block-to-block. A full reconfigure (CMD_*_FILTER on a type
 * change, CMD_FILTER_SET, per-step parameter locks, the boot demo sweep)
 * still snaps the _Sm shadow to match immediately right where it applies —
 * only a same-type cutoff/Q tweak actually glides. */
static inline void SmoothFilterParam(float& current, float target, float rate,
                                      float epsilonCut){
    const float d = target - current;
    if(fabsf(d) < epsilonCut) { current = target; return; }
    current += d * rate;
}

static void UpdatePadFilterSmoothing()
{
    const float kRate = 0.25f;
    for(uint8_t p = 0; p < MAX_PADS; p++){
        if(!padFilterType[p]) continue;
        if(padFilterCutSm[p] == padFilterCut[p] && padFilterQSm[p] == padFilterQ[p])
            continue;
        SmoothFilterParam(padFilterCutSm[p], padFilterCut[p], kRate, 0.5f);
        SmoothFilterParam(padFilterQSm[p],   padFilterQ[p],   kRate, 0.002f);
        padFilter[p].SetType(padFilterType[p], padFilterCutSm[p], padFilterQSm[p],
                              (float)SAMPLE_RATE, padFilterGain[p]);
    }
}

static void UpdateTrackFilterSmoothing()
{
    const float kRate = 0.25f;
    for(uint8_t t = 0; t < MAX_PADS; t++){
        if(!trkFilterType[t]) continue;
        if(trkLfoActive[t] && trkLfoTarget[t] == LFO_TGT_FILTER) continue;
        if(trkFilterCutSm[t] == trkFilterCut[t] && trkFilterQSm[t] == trkFilterQ[t])
            continue;
        SmoothFilterParam(trkFilterCutSm[t], trkFilterCut[t], kRate, 0.5f);
        SmoothFilterParam(trkFilterQSm[t],   trkFilterQ[t],   kRate, 0.002f);
        trkFilter[t].SetType(trkFilterType[t], trkFilterCutSm[t], trkFilterQSm[t],
                              (float)SAMPLE_RATE, trkFilterGain[t]);
        if(trkFilterType[t] == FTYPE_RESONANT)
            trkFilter2[t].SetType(FTYPE_RESONANT, trkFilterCutSm[t], trkFilterQSm[t],
                                   (float)SAMPLE_RATE);
    }
}

static void UpdateGlobalFilterSmoothing()
{
    if(!gFilterType) return;
    if(gFilterCutoffSm == gFilterCutoff && gFilterQSm == gFilterQ) return;
    const float kRate = 0.25f;
    SmoothFilterParam(gFilterCutoffSm, gFilterCutoff, kRate, 0.5f);
    SmoothFilterParam(gFilterQSm,      gFilterQ,      kRate, 0.002f);
    if(gFilterType == FTYPE_LADDER){
        masterLadderL.SetFreq(gFilterCutoffSm);
        masterLadderR.SetFreq(gFilterCutoffSm);
        masterLadderL.SetRes(clampF(gFilterQSm / 28.f, 0.f, 1.f));
        masterLadderR.SetRes(clampF(gFilterQSm / 28.f, 0.f, 1.f));
    } else if((gFilterType >= FTYPE_SVF_LP && gFilterType <= FTYPE_SVF_BP)
              || gFilterType == FTYPE_SVF_MORPH){
        masterSvfL.SetFreq(gFilterCutoffSm);
        masterSvfR.SetFreq(gFilterCutoffSm);
        masterSvfL.SetRes(clampF(gFilterQSm / 28.f, 0.f, 1.f));
        masterSvfR.SetRes(clampF(gFilterQSm / 28.f, 0.f, 1.f));
    } else {
        gFilterL.SetType(gFilterType, gFilterCutoffSm, gFilterQSm,
                          (float)SAMPLE_RATE, GlobalEqGainDb(gFilterType));
        gFilterR.SetType(gFilterType, gFilterCutoffSm, gFilterQSm,
                          (float)SAMPLE_RATE, GlobalEqGainDb(gFilterType));
        if(gFilterType == FTYPE_RESONANT){
            gFilter2L.SetType(FTYPE_RESONANT, gFilterCutoffSm, gFilterQSm, (float)SAMPLE_RATE);
            gFilter2R.SetType(FTYPE_RESONANT, gFilterCutoffSm, gFilterQSm, (float)SAMPLE_RATE);
        }
    }
}

static inline void ConfigureFlanger(Flanger& flanger, float rateHz, float depth, float feedback)
{
    flanger.SetLfoFreq(clampF(rateHz, 0.1f, 20.0f));
    flanger.SetLfoDepth(clampF(depth, 0.0f, 1.0f));
    flanger.SetFeedback(clampF(feedback, 0.0f, 0.95f));
    flanger.SetDelay(clampF(depth, 0.0f, 1.0f));
}

static inline void ConfigureMasterFlanger()
{
    ConfigureFlanger(masterFlangerL, flangerRate, flangerDepth, flangerFb);
    ConfigureFlanger(masterFlangerR, flangerRate * 1.013f, flangerDepth, flangerFb);
}

static inline void ConfigureTrackFlanger(uint8_t track)
{
    if(track >= MAX_PADS) return;
    ConfigureFlanger(trkFlanger[track], trkFlgRate[track], trkFlgDepth[track], trkFlgFb[track]);
}

/* Kill NaN AND Inf — with FTZ+DN enabled, denormals are flushed by hardware.
 * Previously this only caught NaN (Inf was assumed to be "clamped at
 * output" by the limiter/soft-clip stage). That assumption was wrong: in
 * the default non-limiter path, SoftClipKnee(Inf) evaluates SoftLimit(Inf)
 * = Inf/Inf = NaN, and its own `if(shaped>1.0f)` safety clamp silently
 * fails to catch NaN (all comparisons with NaN are false). That NaN then
 * reaches dcBlockL/dcBlockR — a stateful 1-pole IIR — and permanently
 * poisons their feedback history: every output sample from then on is NaN
 * regardless of upstream signal, with no code path that ever re-inits
 * them, so the only recovery was rebooting Daisy. Catching Inf here, at
 * every one of this function's ~50 call sites throughout the FX chain,
 * stops it from ever reaching that stage in the first place. */
static inline float sanitizeF(float v){
    return isfinite(v) ? v : 0.0f;
}

static inline float VolumeByteToGain(uint8_t volumePct)
{
    return clampF((float)volumePct, 0.0f, 150.0f) / 100.0f;
}

static inline float SoftClipKnee(float x)
{
    /* Defense in depth: sanitizeF() already guarantees finite input at
     * every call site today, but this guards the one computation in this
     * function that does NOT degrade gracefully for Inf — SoftLimit(Inf)
     * is Inf/Inf = NaN, which then slips past the `shaped>1.0f` clamp
     * below (NaN fails every comparison) and out of this function. */
    if(!isfinite(x)) return 0.0f;
    const float knee  = 0.985f;
    const float drive = 2.2f;
    /* 1.007307f == 1.0f / SoftLimit(2.2f) — pre-computed constant denominator */
    const float invSL = 1.007307f;
    float ax = fabsf(x);
    if(ax <= knee)
        return x;

    float t = (ax - knee) / (1.0f - knee);
    float shaped = knee + (1.0f - knee) * (SoftLimit(drive * t) * invSL);
    if(shaped > 1.0f)
        shaped = 1.0f;
    return copysignf(shaped, x);
}

/* Stateless triangle wavefolder. It is transparent for -1..+1 at gain 1,
 * then folds every excursion back into that range without channel crosstalk. */
static inline float WaveFoldSample(float input, float gain)
{
    float phase = fmodf(input * gain + 1.0f, 4.0f);
    if(phase < 0.0f) phase += 4.0f;
    return phase <= 2.0f ? phase - 1.0f : 3.0f - phase;
}

/* FTYPE_RESONANT cascades two identical RBJ-cookbook lowpass biquads at the
 * same cutoff/Q for a 24 dB/oct slope. Each stage's own resonant peak grows
 * roughly linearly with Q (that parameterization's well-known property:
 * peak gain ~= Q above the Butterworth point Q=0.707, flat/no-peak below
 * it) — so the CASCADE of two such stages peaks roughly with Q^2. At Q=20
 * (the FX Lab RESONANCE knob's max) that is on the order of +20 dB more
 * boost than the old fixed SoftLimit(s*1.4f)*0.714f makeup gain ever
 * accounted for, which is what was driving the resonant filter into
 * constant hard limiting ("satura") instead of just coloring the peak.
 *
 * This brings the cascade's growth back down to linear-in-Q — still
 * audibly louder/more aggressive as resonance rises (that's the point of
 * a resonant filter), just no longer runaway-quadratic. Applied as an
 * extra multiplier AFTER the existing saturation stage, which is left
 * untouched (it stays transparent at low signal levels by design). */
static inline float ResonantMakeupGain(float q)
{
    const float qRef = 0.707f; /* Butterworth Q: no resonant peak below this */
    return q > qRef ? (qRef / q) : 1.0f;
}

static void ResetMasterProcessingState()
{
    delayActive = false;
    reverbActive = false;
    chorusActive = false;
    tremoloActive = false;
    compActive = false;
    phaserActive = false;
    flangerActive = false;
    waveFolderGain = 1.0f;
    phaserDepth = 0.4f;
    masterPhaserL.SetLfoDepth(phaserDepth);
    masterPhaserR.SetLfoDepth(phaserDepth);
    limiterActive = false;
    autowahActive = false;
    erActive = false;

    delayRouted = true;
    reverbRouted = true;
    chorusRouted = true;
    tremoloRouted = true;
    compRouted = true;
    phaserRouted = true;
    flangerRouted = true;
    waveFolderRouted = true;
    limiterRouted = true;
    autowahRouted = true;
    erRouted = true;

    gFilterRouted = true;
    gFilterType = FTYPE_NONE;
    gFilterCutoff = 10000.0f;
    gFilterQ = 0.707f;
    gFilterCutoffSm = gFilterCutoff;
    gFilterQSm      = gFilterQ;
    gFilterBitDepth = 16;
    gFilterDist = 0.0f;
    gFilterDistMode = DMODE_SOFT;
    gFilterSrReduce = 0;
    gSrHoldL = 0.0f;
    gSrHoldR = 0.0f;
    gSrPhase = 0;
    gSrPrimed = false;

    /* Mega upgrade state */
    stereoWidth = 1.0f;
    tapeStopActive = false;
    tapeStopSpeed = 1.0f;
    beatRepActive = false;
    beatRepDiv = 0;
    beatRepPlaying = false;
    beatRepCapturing = false;
    beatRepPos = 0;
    delayPingPong = false;
    chorusStereoMode = true;
}

static inline float TriFromPhase(float ph)
{
    /* 0..1 -> -1..+1 */
    float t = ph < 0.5f ? (ph * 2.0f) : (2.0f - ph * 2.0f);
    return t * 2.0f - 1.0f;
}

static inline float AdDecayCoefFromMs(float decayMs)
{
    float samples = clampF(decayMs, 1.0f, 8000.0f) * (float)SAMPLE_RATE * 0.001f;
    return expf(-1.0f / samples);
}

/* Forward decl: usado por el startup self-test */
static float PadPlaybackSpeed(uint8_t pad, float sourcePitch)
{
    if(pad >= MAX_PADS)
        return 1.0f;
    uint32_t sourceRate = sampleRateHz[pad] ? sampleRateHz[pad] : SAMPLE_RATE;
    return ((float)sourceRate / (float)SAMPLE_RATE)
         * padPitch[pad]
         * powf(2.0f, trkPitchCents[pad] / 1200.0f)
         * clampF(sourcePitch, 0.25f, 4.0f);
}

static void TriggerPad(uint8_t pad, uint8_t velocity,
                       uint8_t trkVol, int8_t pan,
                       uint32_t maxSamples,
                       float sourceVolume = 1.0f,
                       float sourcePitch = 1.0f,
                       bool liveSource = false);

/* ═══════════════════════════════════════════════════════════════════
 *  Startup self-test en fases
 *  1) Samplers WAV RED (pads 0..15, pad por pad)
 *  2) TR808 instrumentos (uno a uno)
 *  3) TR909 instrumentos (uno a uno)
 *  4) TR505 instrumentos (uno a uno)
 *  5) TB303 notas (una a una)
 *  6) Samplers WAV XTRA (pads 16..23, al final)
 * ═══════════════════════════════════════════════════════════════════ */
/* Diagnostic only, off by default (RED808_STARTUP_808_SELF_TEST=0 in the
 * Makefile) — a hardware bring-up self-test that steps through every pad/
 * instrument from the main loop. Unlike the rest of main-loop trigger paths,
 * it calls TriggerPad()/synth Trigger()/acid303 NoteOn()/NoteOff() directly
 * instead of going through AudioCmdPush(), so it does NOT benefit from the
 * lock-free audio command queue and can still race AudioCallback. Left this
 * way deliberately: migrating ~20 call sites across an 11-phase state
 * machine that only ever runs when a developer flips this build flag for
 * bring-up testing isn't worth the risk of a copy/paste mistake in code that
 * can only be exercised by reflashing hardware. If you enable this flag,
 * treat any audio glitch during the self-test as inconclusive for judging
 * the queue itself — it doesn't go through it. */
static void RunStartup808SelfTest(uint32_t nowMs)
{
    if(!kStartup808SelfTest || kStartupShowcaseDemo)
        return;

    enum Phase : uint8_t {
        PH_IDLE = 0,
        PH_SCAN_SAMPLES,
        PH_SCAN_808,
        PH_SCAN_909,
        PH_SCAN_505,
        PH_SCAN_303_ON,
        PH_SCAN_303_OFF,
        PH_SCAN_XTRA,
        PH_SAMPLER_FX_JAM,
        PH_SYNTH_JAM,
        PH_CLEANUP,
        PH_DONE
    };
    static Phase    phase = PH_IDLE;
    static uint32_t nextMs = 0;
    static uint8_t  padIdx = 0;
    static uint8_t  xtraPadIdx = 16;
    static uint8_t  inst808Idx = 0;
    static uint8_t  inst909Idx = 0;
    static uint8_t  inst505Idx = 0;
    static uint8_t  note303Idx = 0;
    static uint8_t  samplerFxStep = 0;
    static uint8_t  synthJamStep  = 0;
    static uint8_t  synthJamStyle = 0; /* 0=Techno 1=Electro 2=Ambient */

    static const uint32_t kPauseSampleMs = 380;
    static const uint32_t kPauseSynthMs  = 360;
    static const uint32_t kPausePhaseMs  = 560;
    static const uint32_t kPause303OnMs  = 320;
    static const uint32_t kPause303OffMs = 220;
    static const uint32_t kPauseSamplerFxMs = 230;
    static const uint32_t kPauseSynthJamMs  = 130;
    static const uint8_t  kSamplerFxSteps   = 14;
    static const uint8_t  kSynthJamSteps    = 32;

    static const uint8_t inst808List[16] = {
        TR808::INST_KICK,
        TR808::INST_SNARE,
        TR808::INST_CLAP,
        TR808::INST_HIHAT_C,
        TR808::INST_HIHAT_O,
        TR808::INST_LOW_TOM,
        TR808::INST_MID_TOM,
        TR808::INST_HI_TOM,
        TR808::INST_LOW_CONGA,
        TR808::INST_MID_CONGA,
        TR808::INST_HI_CONGA,
        TR808::INST_CLAVES,
        TR808::INST_MARACAS,
        TR808::INST_RIMSHOT,
        TR808::INST_COWBELL,
        TR808::INST_CYMBAL,
    };

    static const uint8_t inst909List[] = {
        TR909::INST_KICK,
        TR909::INST_SNARE,
        TR909::INST_CLAP,
        TR909::INST_HIHAT_C,
        TR909::INST_HIHAT_O,
        TR909::INST_LOW_TOM,
        TR909::INST_MID_TOM,
        TR909::INST_HI_TOM,
        TR909::INST_RIDE,
        TR909::INST_CRASH,
        TR909::INST_RIMSHOT,
    };

    static const uint8_t inst505List[] = {
        TR505::INST_KICK,
        TR505::INST_SNARE,
        TR505::INST_CLAP,
        TR505::INST_HIHAT_C,
        TR505::INST_HIHAT_O,
        TR505::INST_LOW_TOM,
        TR505::INST_MID_TOM,
        TR505::INST_HI_TOM,
        TR505::INST_COWBELL,
        TR505::INST_CYMBAL,
        TR505::INST_RIMSHOT,
    };

    static const uint8_t notes303[] = {
        36, 38, 41, 43, 45, 48, 50, 53
    };

    static const uint8_t jamNotes[16] = {
        36, 36, 43, 0,
        41, 41, 48, 0,
        45, 45, 50, 48,
        43, 41, 38, 0
    };

    static const uint8_t jamNotesElectro[16] = {
        36, 43, 36, 0,
        48, 46, 43, 0,
        41, 43, 45, 0,
        50, 48, 46, 43
    };

    static const uint8_t jamNotesAmbient[16] = {
        36, 0, 43, 0,
        48, 0, 50, 0,
        53, 0, 48, 0,
        45, 0, 41, 0
    };

    if(spiPktCnt > 0 && phase != PH_DONE)
    {
        acid303.NoteOff();
        phase = PH_DONE;
        return;
    }

    if(phase == PH_IDLE)
    {
        phase   = PH_SCAN_SAMPLES;
        nextMs  = nowMs + 250;
        padIdx  = 0;
        xtraPadIdx = 16;
        inst808Idx = 0;
        inst909Idx = 0;
        inst505Idx = 0;
        note303Idx = 0;
        samplerFxStep = 0;
        synthJamStep = 0;
        synthJamStyle = 0;
        QueueStartupSectionTag(SEC_SAMPLERS);
        return;
    }

    if(nowMs < nextMs)
        return;

    if(phase == PH_SCAN_SAMPLES)
    {
        while(padIdx < 16 && !sampleLoaded[padIdx])
            padIdx++;

        if(padIdx < 16)
        {
            TriggerPad(padIdx, 115, 100, 0, 0);
            padIdx++;
            nextMs = nowMs + kPauseSampleMs;
            return;
        }

        phase = PH_SCAN_808;
        inst808Idx = 0;
        nextMs = nowMs + kPausePhaseMs;
        QueueStartupSectionTag(SEC_808);
        return;
    }

    if(phase == PH_SCAN_808)
    {
        synth808.Trigger(inst808List[inst808Idx], 0.90f);
        inst808Idx++;
        nextMs = nowMs + kPauseSynthMs;
        if(inst808Idx >= (sizeof(inst808List) / sizeof(inst808List[0])))
        {
            phase = PH_SCAN_909;
            inst909Idx = 0;
            nextMs = nowMs + kPausePhaseMs;
            QueueStartupSectionTag(SEC_909);
        }

        return;
    }

    if(phase == PH_SCAN_909)
    {
        synth909.Trigger(inst909List[inst909Idx], 0.88f);
        inst909Idx++;
        nextMs = nowMs + kPauseSynthMs;
        if(inst909Idx >= (sizeof(inst909List) / sizeof(inst909List[0])))
        {
            phase = PH_SCAN_505;
            inst505Idx = 0;
            nextMs = nowMs + kPausePhaseMs;
            QueueStartupSectionTag(SEC_505);
        }

        return;
    }

    if(phase == PH_SCAN_505)
    {
        synth505.Trigger(inst505List[inst505Idx], 0.88f);
        inst505Idx++;
        nextMs = nowMs + kPauseSynthMs;
        if(inst505Idx >= (sizeof(inst505List) / sizeof(inst505List[0])))
        {
            phase = PH_SCAN_303_ON;
            note303Idx = 0;
            nextMs = nowMs + kPausePhaseMs;
            QueueStartupSectionTag(SEC_303);
        }

        return;
    }

    if(phase == PH_SCAN_303_ON)
    {
        if(note303Idx >= (sizeof(notes303) / sizeof(notes303[0])))
        {
            acid303.NoteOff();
            phase = PH_SCAN_XTRA;
            xtraPadIdx = 16;
            nextMs = nowMs + kPausePhaseMs;
            QueueStartupSectionTag(SEC_XTRAS);
            return;
        }

        bool accent = ((note303Idx & 1u) == 0u);
        bool slide  = ((note303Idx & 3u) == 3u);
        acid303.NoteOn(notes303[note303Idx], accent, slide);
        phase  = PH_SCAN_303_OFF;
        nextMs = nowMs + kPause303OnMs;
        return;
    }

    if(phase == PH_SCAN_303_OFF)
    {
        acid303.NoteOff();
        note303Idx++;
        phase  = PH_SCAN_303_ON;
        nextMs = nowMs + kPause303OffMs;

        return;
    }

    if(phase == PH_SCAN_XTRA)
    {
        while(xtraPadIdx < MAX_PADS && !sampleLoaded[xtraPadIdx])
            xtraPadIdx++;

        if(xtraPadIdx < MAX_PADS)
        {
            TriggerPad(xtraPadIdx, 115, 100, 0, 0);
            xtraPadIdx++;
            nextMs = nowMs + kPauseSampleMs;
            return;
        }

        phase = PH_SAMPLER_FX_JAM;
        samplerFxStep = 0;
        nextMs = nowMs + kPausePhaseMs;
        QueueStartupSectionTag(SEC_SAMPLER_FX);
        return;
    }

    if(phase == PH_SAMPLER_FX_JAM)
    {
        delayActive = true;
        reverbActive = true;
        chorusActive = true;
        delayMix = 0.20f;
        delayFeedback = 0.34f;
        reverbMix = 0.24f;
        chorusMix = 0.16f;
        masterDelay.SetDelay(0.18f * (float)SAMPLE_RATE);
        masterDelayR.SetDelay(0.18f * (float)SAMPLE_RATE);
        masterReverb.SetFeedback(0.83f);
        masterReverb.SetLpFreq(7600.0f);
        masterChorusL.SetLfoFreq(0.35f);
        masterChorusR.SetLfoFreq(0.3535f);
        masterChorusL.SetLfoDepth(0.35f);
        masterChorusR.SetLfoDepth(0.35f);

        int picked = -1;
        for(int k = 0; k < MAX_PADS; k++){
            int cand = (samplerFxStep + k) % MAX_PADS;
            if(sampleLoaded[cand]){ picked = cand; break; }
        }

        if(picked < 0)
        {
            phase = PH_SYNTH_JAM;
            synthJamStyle = 0;
            synthJamStep = 0;
            nextMs = nowMs + kPausePhaseMs;
            QueueStartupSectionTag(SEC_TECHNO);
            return;
        }

        float t = (kSamplerFxSteps <= 1)
            ? 1.0f
            : ((float)samplerFxStep / (float)(kSamplerFxSteps - 1));
        float sweep = 0.5f + 0.5f * sinf(2.0f * (float)M_PI * (0.8f * t + 0.12f));

        uint8_t p = (uint8_t)picked;
        trkEnvAdActive[p] = true;
        trkEnvAttackMs[p] = 1.0f + 45.0f * sweep;
        trkEnvDecayMs[p]  = 140.0f + 980.0f * t;

        trkFilterType[p] = ((samplerFxStep & 3u) == 2u) ? FTYPE_BANDPASS : FTYPE_LOWPASS;
        trkFilterCut[p]  = clampF(260.0f + 11200.0f * sweep, 20.0f, 18000.0f);
        trkFilterQ[p]    = 0.75f + 2.1f * (1.0f - sweep);
        /* This showcase already sweeps in discrete, timed steps of its own —
         * snap the smoothing shadow so UpdateTrackFilterSmoothing() doesn't
         * additionally ease toward each step. */
        trkFilterCutSm[p] = trkFilterCut[p];
        trkFilterQSm[p]   = trkFilterQ[p];
        trkFilter[p].SetType(trkFilterType[p], trkFilterCut[p], trkFilterQ[p], (float)SAMPLE_RATE);

        trkDistMode[p]   = (samplerFxStep & 1u) ? DMODE_TUBE : DMODE_SOFT;
        trkDistDrive[p]  = clampF(0.10f + 0.60f * t, 0.0f, 1.0f);
        trkBitDepth[p]   = (uint8_t)clampF(16.0f - 9.0f * t, 6.0f, 16.0f);

        trackReverbSend[p] = 0.10f + 0.35f * t;
        trackDelaySend[p]  = 0.12f + 0.40f * (1.0f - t);
        trackChorusSend[p] = 0.10f + 0.20f * sweep;
        trackPanF[p]       = clampF(-0.8f + 1.6f * t, -1.0f, 1.0f);

        trkLfoActive[p] = true;
        trkLfoWave[p]   = ((samplerFxStep & 1u) == 0u) ? LFO_WAVE_TRI : LFO_WAVE_SINE;
        trkLfoTarget[p] = ((samplerFxStep & 3u) == 1u) ? LFO_TGT_PAN : LFO_TGT_FILTER;
        trkLfoRate[p]   = 0.45f + 3.2f * t;
        trkLfoDepth[p]  = 0.18f + 0.55f * (1.0f - t);

        uint8_t vel = (uint8_t)clampF(96.0f + 31.0f * sweep, 1.0f, 127.0f);
        TriggerPad(p, vel, 100, 0, 0);

        samplerFxStep++;
        nextMs = nowMs + kPauseSamplerFxMs;
        if(samplerFxStep >= kSamplerFxSteps)
        {
            phase = PH_SYNTH_JAM;
            synthJamStyle = 0;
            synthJamStep = 0;
            nextMs = nowMs + kPausePhaseMs;
            QueueStartupSectionTag(SEC_TECHNO);
        }
        return;
    }

    if(phase == PH_SYNTH_JAM)
    {
        const bool isTechno  = (synthJamStyle == 0);
        const bool isElectro = (synthJamStyle == 1);
        const bool isAmbient = (synthJamStyle == 2);

        const uint8_t styleSteps = isAmbient ? 24 : kSynthJamSteps;
        const uint32_t jamPauseMs = isAmbient ? 190 : (isElectro ? 145 : kPauseSynthJamMs);

        delayActive = true;
        reverbActive = true;
        chorusActive = true;
        tremoloActive = true;
        if(isTechno){
            delayMix = 0.16f + 0.08f * (0.5f + 0.5f * sinf(0.35f * (float)synthJamStep));
            reverbMix = 0.20f + 0.10f * (0.5f + 0.5f * sinf(0.20f * (float)synthJamStep + 1.2f));
            chorusMix = 0.14f + 0.08f * (0.5f + 0.5f * sinf(0.27f * (float)synthJamStep + 0.4f));
            masterTremolo.SetFreq(3.0f + 2.2f * (0.5f + 0.5f * sinf(0.17f * (float)synthJamStep)));
            masterTremolo.SetDepth(0.10f + 0.16f * (0.5f + 0.5f * sinf(0.19f * (float)synthJamStep + 0.7f)));
        } else if(isElectro){
            delayMix = 0.22f;
            reverbMix = 0.14f;
            chorusMix = 0.22f;
            masterTremolo.SetFreq(5.0f);
            masterTremolo.SetDepth(0.11f);
            flangerActive = true;
            flangerRate   = 0.55f;
            flangerDepth  = 0.48f;
            flangerMix    = 0.22f;
        } else {
            delayMix = 0.12f;
            reverbMix = 0.34f;
            chorusMix = 0.30f;
            masterTremolo.SetFreq(1.6f);
            masterTremolo.SetDepth(0.07f);
            flangerActive = false;
            masterReverb.SetLpFreq(6200.0f);
            masterReverb.SetFeedback(0.90f);
        }

        uint8_t st = synthJamStep & 15u;
        if(isTechno){
            if((st % 4u) == 0u) synth808.kick.Trigger(0.92f);
            if(st == 4u || st == 12u) synth909.snare.Trigger(0.82f);
            if((st % 2u) == 1u) synth505.hihatC.Trigger(0.48f);
            if(st == 7u || st == 15u) synth505.clap.Trigger(0.64f);
            if(st == 10u) synth909.ride.Trigger(0.52f);
        } else if(isElectro){
            if(st == 0u || st == 6u || st == 8u || st == 14u) synth909.kick.Trigger(0.90f);
            if(st == 4u || st == 12u) synth505.snare.Trigger(0.72f);
            if((st % 4u) == 2u) synth909.hihatO.Trigger(0.52f);
            if((st % 2u) == 1u) synth505.hihatC.Trigger(0.40f);
            if(st == 11u) synth505.cowbell.Trigger(0.58f);
        } else {
            if(st == 0u || st == 8u) synth808.kick.Trigger(0.66f);
            if(st == 4u || st == 12u) synth808.clap.Trigger(0.48f);
            if((st % 8u) == 6u) synth909.crash.Trigger(0.36f);
            if((st % 4u) == 2u) synth505.hihatO.Trigger(0.34f);
        }

        float u = (styleSteps <= 1)
            ? 1.0f
            : ((float)synthJamStep / (float)(styleSteps - 1));

        float acidCut = isTechno
            ? (420.0f + 4400.0f * u + 900.0f * sinf(2.0f * (float)M_PI * (u * 1.5f)))
            : (isElectro
                ? (700.0f + 3600.0f * (0.5f + 0.5f * sinf(0.30f * (float)synthJamStep)))
                : (260.0f + 2200.0f * (0.5f + 0.5f * sinf(0.18f * (float)synthJamStep))));
        acidCut = clampF(acidCut,
                               120.0f, 14000.0f);
        acid303.SetCutoff(acidCut);
        acid303.SetResonance(clampF(isAmbient
            ? (0.35f + 0.22f * (0.5f + 0.5f * sinf(0.15f * (float)synthJamStep)))
            : (0.45f + 0.40f * (0.5f + 0.5f * sinf(0.41f * (float)synthJamStep))), 0.1f, 0.94f));
        acid303.SetEnvMod(clampF(isAmbient ? 0.30f : (0.38f + 0.55f * u), 0.0f, 1.0f));
        acid303.SetDecay(clampF(isAmbient ? 0.42f : (0.16f + 0.20f * (1.0f - u)), 0.02f, 3.0f));
        acid303.SetAccent(clampF(isAmbient
            ? 0.32f
            : (0.48f + 0.42f * (0.5f + 0.5f * sinf(0.23f * (float)synthJamStep + 0.3f))), 0.0f, 1.0f));
        acid303.SetSlide(clampF(isAmbient ? 0.11f : (0.03f + 0.08f * (0.5f + 0.5f * sinf(0.29f * (float)synthJamStep))), 0.01f, 0.5f));
        acid303.SetWaveform(isAmbient ? TB303::WAVE_SAW : ((synthJamStep & 8u) ? TB303::WAVE_SQUARE : TB303::WAVE_SAW));

        uint8_t n = isTechno ? jamNotes[st] : (isElectro ? jamNotesElectro[st] : jamNotesAmbient[st]);
        if(n == 0u)
        {
            acid303.NoteOff();
        }
        else
        {
            bool accent = isAmbient ? ((st % 8u) == 0u)
                                    : (((st & 3u) == 0u) || (st == 6u) || (st == 14u));
            bool slide  = isAmbient ? ((st % 8u) == 7u)
                                    : (((st & 7u) == 3u) || (st == 11u));
            acid303.NoteOn(n, accent, slide);
        }

        synthJamStep++;
        nextMs = nowMs + jamPauseMs;
        if(synthJamStep >= styleSteps)
        {
            acid303.NoteOff();
            synthJamStep = 0;
            synthJamStyle++;
            if(synthJamStyle < 3){
                nextMs = nowMs + kPausePhaseMs;
                if(synthJamStyle == 1) QueueStartupSectionTag(SEC_ELECTRO);
                else                   QueueStartupSectionTag(SEC_AMBIENT);
            } else {
                phase = PH_CLEANUP;
                nextMs = nowMs + 20;
            }
        }
        return;
    }

    if(phase == PH_CLEANUP)
    {
        delayActive = false;
        reverbActive = false;
        chorusActive = false;
        tremoloActive = false;
        flangerActive = false;

        for(int i = 0; i < MAX_PADS; i++)
        {
            trkFilterType[i] = 0;
            trkFilter[i].Reset();
            trkDistDrive[i] = 0.0f;
            trkDistMode[i]  = DMODE_SOFT;
            trkBitDepth[i]  = 16;
            trackReverbSend[i] = 0.0f;
            trackDelaySend[i]  = 0.0f;
            trackChorusSend[i] = 0.0f;
            trackPanF[i]       = 0.0f;
            trkLfoActive[i]    = false;
            trkLfoDepth[i]     = 0.0f;
            trkLfoPhase[i]     = 0.0f;
            trkEnvAdActive[i]  = false;
            trkEnvAttackMs[i]  = 1.0f;
            trkEnvDecayMs[i]   = 250.0f;
        }

        phase = PH_DONE;
        return;
    }
}

/* AudioNoise (torvalds/AudioNoise): fast tanh approx x/(1+|x|), more stable
 * than polynomial and avoids dangerous overshoot above ±1.5.          */
static inline float MySoftClip(float x){ return x / (1.0f + fabsf(x)); }
/* Asymmetric clip — tube-like even harmonics (AudioNoise/distortion.h)
 * positive: soft / negative: softer → breaks waveform symmetry → warm tone */
static inline float AsymClip(float x){
    if(x >= 0.0f) return x / (1.0f + fabsf(x));
    float n = x * 0.7f;
    return (n / (1.0f + fabsf(n))) * 0.7f;
}

/* Fast pow for compressor ratio: base^exp via IEEE 754 bit-trick (~5% error) */
static inline float fast_powf(float base, float exponent){
    union { float f; int32_t i; } v;
    v.f = base;
    v.i = (int32_t)(exponent * (float)(v.i - 1065353216) + 1065353216.0f);
    return (v.i > 0) ? v.f : 0.0f;
}

static float ApplyDist(float s, float drive, uint8_t mode){
    if(!isfinite(s)) return 0.0f;          // nunca propagar NaN/Inf al DSP
    drive = clampF(drive, 0.f, 1.f);
    if(drive < 0.001f) return s;
    const float dry = s;
    const float preGain = 1.0f + drive * 24.0f;
    s *= preGain;
    switch(mode){
        case DMODE_SOFT: s = MySoftClip(s); break;
        case DMODE_HARD: s = clampF(s,-1.f,1.f); break;
        case DMODE_TUBE: s = AsymClip(s); break;  // asimétrico tube (AudioNoise)
        case DMODE_FUZZ: s = WaveFoldSample(s, 1.0f); break;
        default: s = MySoftClip(s); break;
    }
    const float wet = s * (1.0f - drive * 0.15f);
    const float out = dry * (1.0f - drive) + wet * drive;
    return isfinite(out) ? out : 0.0f;
}

static float BitCrush(float s, uint8_t bits){
    if(bits >= 16) return s;
    bits = bits < 2 ? 2 : bits;
    const float steps = (float)((1u << (bits - 1u)) - 1u);
    return roundf(clampF(s, -1.0f, 1.0f) * steps) / steps;
}

static void StopPadVoices(uint8_t pad)
{
    for(int voiceIndex = 0; voiceIndex < MAX_VOICES; voiceIndex++)
        if(voices[voiceIndex].active && voices[voiceIndex].pad == pad)
            voices[voiceIndex].active = false;
}

static void ReleaseTrackEngine(uint8_t track, int8_t engine)
{
    switch(engine)
    {
        case SYNTH_ENGINE_303:
            acid303.NoteOff();
            break;
        case SYNTH_ENGINE_WTOSC:
            if(track < 16)
                wtOsc.NoteOff(trackWtNote[track]);
            else
                wtOsc.AllNotesOff();
            break;
        case SYNTH_ENGINE_SH101:
            synthSH101.NoteOff();
            break;
        case SYNTH_ENGINE_FM2OP:
            synthFM2Op.NoteOff();
            break;
        case SYNTH_ENGINE_PHYS:
            physModalActive = false;
            physStringActive = false;
            break;
        case SYNTH_ENGINE_NOISE:
            noisePartActive = false;
            break;
        default:
            break;
    }
}

static void ReleaseAllSynthEngines()
{
    acid303.NoteOff();
    wtOsc.AllNotesOff();
    synthSH101.NoteOff();
    synthFM2Op.NoteOff();
    physModalActive = false;
    physStringActive = false;
    noisePartActive = false;
}

static void ReleaseSynthEngineState(uint8_t engine)
{
    switch(engine)
    {
        case SYNTH_ENGINE_808:
        case SYNTH_ENGINE_909:
        case SYNTH_ENGINE_505:
            break;
        case SYNTH_ENGINE_303:
            acid303.NoteOff();
            break;
        case SYNTH_ENGINE_WTOSC:
            wtOsc.AllNotesOff();
            break;
        case SYNTH_ENGINE_SH101:
            synthSH101.NoteOff();
            break;
        case SYNTH_ENGINE_FM2OP:
            synthFM2Op.NoteOff();
            break;
        case SYNTH_ENGINE_PHYS:
            physModalActive = false;
            physStringActive = false;
            break;
        case SYNTH_ENGINE_NOISE:
            noisePartActive = false;
            break;
        default:
            break;
    }
}

static void ApplyWtModState()
{
    wtOsc.SetFilter(wtFilterCutoffState, wtFilterQState);
    wtOsc.SetLfo(wtLfoRateState, wtLfoDepthState, wtLfoTargetState);
}

static void ApplyDrumSynthParam(uint8_t engine, uint8_t instrument, uint8_t paramId, float val)
{
    switch(engine)
    {
        case SYNTH_ENGINE_808:
            switch(instrument){
                case TR808::INST_KICK:
                    if(paramId==0) synth808.kick.SetDecay(val);
                    if(paramId==1) synth808.kick.SetPitch(val);
                    if(paramId==2) synth808.kick.SetDrive(val);
                    if(paramId==3) synth808.kick.volume = clampF(val,0.f,1.f);
                    if(paramId==4) synth808.kick.subLevel  = clampF(val,0.f,0.5f);
                    if(paramId==5) synth808.kick.pitchAmt  = clampF(val,1.f,20.f);
                    if(paramId==6) synth808.kick.SetPitchDecay(val);
                    if(paramId==7) synth808.kick.punchAmt  = clampF(val,0.f,2.5f);
                    break;
                case TR808::INST_SNARE:
                    if(paramId==0) synth808.snare.SetDecay(val);
                    if(paramId==1) synth808.snare.SetPitch(val);
                    if(paramId==2) synth808.snare.SetTone(val);
                    if(paramId==3) synth808.snare.volume = clampF(val,0.f,1.f);
                    if(paramId==4) synth808.snare.SetSnappy(val);
                    break;
                case TR808::INST_CLAP:
                    if(paramId==0) synth808.clap.SetDecay(val);
                    if(paramId==2) synth808.clap.SetSnap(val);
                    if(paramId==3) synth808.clap.volume = clampF(val,0.f,1.f);
                    break;
                case TR808::INST_HIHAT_C:
                    if(paramId==0) synth808.hihatC.SetDecay(val);
                    if(paramId==3) synth808.hihatC.volume = clampF(val,0.f,1.f);
                    break;
                case TR808::INST_HIHAT_O:
                    if(paramId==0) synth808.hihatO.SetDecay(val);
                    if(paramId==3) synth808.hihatO.volume = clampF(val,0.f,1.f);
                    break;
                case TR808::INST_COWBELL:
                    if(paramId==0) synth808.cowbell.SetDecay(val);
                    if(paramId==1) synth808.cowbell.SetTune(val);
                    if(paramId==3) synth808.cowbell.volume = clampF(val,0.f,1.f);
                    break;
                case TR808::INST_CYMBAL:
                    if(paramId==0) synth808.cymbal.SetDecay(val);
                    if(paramId==3) synth808.cymbal.volume = clampF(val,0.f,1.f);
                    break;
                case TR808::INST_LOW_TOM:
                    if(paramId==0) synth808.lowTom.SetDecay(val);
                    if(paramId==1) synth808.lowTom.SetPitch(val);
                    if(paramId==3) synth808.lowTom.volume = clampF(val,0.f,1.f);
                    if(paramId==5) synth808.lowTom.smack = clampF(val,0.f,1.f);
                    break;
                case TR808::INST_MID_TOM:
                    if(paramId==0) synth808.midTom.SetDecay(val);
                    if(paramId==1) synth808.midTom.SetPitch(val);
                    if(paramId==3) synth808.midTom.volume = clampF(val,0.f,1.f);
                    if(paramId==5) synth808.midTom.smack = clampF(val,0.f,1.f);
                    break;
                case TR808::INST_HI_TOM:
                    if(paramId==0) synth808.hiTom.SetDecay(val);
                    if(paramId==1) synth808.hiTom.SetPitch(val);
                    if(paramId==3) synth808.hiTom.volume = clampF(val,0.f,1.f);
                    if(paramId==5) synth808.hiTom.smack = clampF(val,0.f,1.f);
                    break;
                default:
                    if(paramId==3) synth808.SetVolume(instrument, clampF(val,0.f,2.f));
                    break;
            }
            break;

        case SYNTH_ENGINE_909:
            switch(instrument){
                case TR909::INST_KICK:
                    if(paramId==0) synth909.kick.SetDecay(val);
                    if(paramId==1) synth909.kick.SetPitch(val);
                    if(paramId==3) synth909.kick.volume = clampF(val,0.f,1.f);
                    break;
                case TR909::INST_SNARE:
                    if(paramId==0) synth909.snare.SetDecay(val);
                    if(paramId==2) synth909.snare.SetTone(val);
                    if(paramId==3) synth909.snare.volume = clampF(val,0.f,1.f);
                    if(paramId==4) synth909.snare.SetSnappy(val);
                    break;
                case TR909::INST_CLAP:
                    if(paramId==0) synth909.clap.SetDecay(val);
                    if(paramId==3) synth909.clap.volume = clampF(val,0.f,1.f);
                    break;
                case TR909::INST_HIHAT_C:
                    if(paramId==0) synth909.hihatC.SetDecay(val);
                    if(paramId==3) synth909.hihatC.volume = clampF(val,0.f,1.f);
                    break;
                case TR909::INST_HIHAT_O:
                    if(paramId==0) synth909.hihatO.SetDecay(val);
                    if(paramId==3) synth909.hihatO.volume = clampF(val,0.f,1.f);
                    break;
                case TR909::INST_LOW_TOM:
                    if(paramId==0) synth909.lowTom.SetDecay(val);
                    if(paramId==1) synth909.lowTom.SetPitch(val);
                    if(paramId==3) synth909.lowTom.volume = clampF(val,0.f,1.f);
                    break;
                case TR909::INST_MID_TOM:
                    if(paramId==0) synth909.midTom.SetDecay(val);
                    if(paramId==1) synth909.midTom.SetPitch(val);
                    if(paramId==3) synth909.midTom.volume = clampF(val,0.f,1.f);
                    break;
                case TR909::INST_HI_TOM:
                    if(paramId==0) synth909.hiTom.SetDecay(val);
                    if(paramId==1) synth909.hiTom.SetPitch(val);
                    if(paramId==3) synth909.hiTom.volume = clampF(val,0.f,1.f);
                    break;
                case TR909::INST_RIDE:
                    if(paramId==0) synth909.ride.SetDecay(val);
                    if(paramId==3) synth909.ride.volume = clampF(val,0.f,1.f);
                    break;
                case TR909::INST_CRASH:
                    if(paramId==0) synth909.crash.SetDecay(val);
                    if(paramId==3) synth909.crash.volume = clampF(val,0.f,1.f);
                    break;
                case TR909::INST_RIMSHOT:
                    if(paramId==3) synth909.rimshot.volume = clampF(val,0.f,1.f);
                    break;
                case TR909::INST_SHAKER:
                    if(paramId==0) synth909.shaker.SetDecay(val);
                    if(paramId==2) synth909.shaker.SetTone(val);
                    if(paramId==3) synth909.shaker.volume = clampF(val,0.f,1.f);
                    break;
                case TR909::INST_CLAVE:
                    if(paramId==0) synth909.clave.SetDecay(val);
                    if(paramId==1) synth909.clave.SetPitch(val);
                    if(paramId==3) synth909.clave.volume = clampF(val,0.f,1.f);
                    break;
                case TR909::INST_HI_PERC:
                    if(paramId==0) synth909.hiPerc.SetDecay(val);
                    if(paramId==1) synth909.hiPerc.SetPitch(val);
                    if(paramId==2) synth909.hiPerc.SetMetal(val);
                    if(paramId==3) synth909.hiPerc.volume = clampF(val,0.f,1.f);
                    break;
                case TR909::INST_MID_PERC:
                    if(paramId==0) synth909.midPerc.SetDecay(val);
                    if(paramId==1) synth909.midPerc.SetPitch(val);
                    if(paramId==2) synth909.midPerc.SetMetal(val);
                    if(paramId==3) synth909.midPerc.volume = clampF(val,0.f,1.f);
                    break;
                case TR909::INST_LOW_PERC:
                    if(paramId==0) synth909.lowPerc.SetDecay(val);
                    if(paramId==1) synth909.lowPerc.SetPitch(val);
                    if(paramId==2) synth909.lowPerc.SetMetal(val);
                    if(paramId==3) synth909.lowPerc.volume = clampF(val,0.f,1.f);
                    break;
                default:
                    if(paramId==3) synth909.SetVolume(instrument, clampF(val,0.f,2.f));
                    break;
            }
            break;

        case SYNTH_ENGINE_505:
            switch(instrument){
                case TR505::INST_KICK:
                    if(paramId==0) synth505.kick.SetDecay(val);
                    if(paramId==1) synth505.kick.SetPitch(val);
                    if(paramId==3) synth505.kick.volume = clampF(val,0.f,1.f);
                    break;
                case TR505::INST_SNARE:
                    if(paramId==0) synth505.snare.SetDecay(val);
                    if(paramId==2) synth505.snare.SetTone(val);
                    if(paramId==3) synth505.snare.volume = clampF(val,0.f,1.f);
                    break;
                case TR505::INST_CLAP:
                    if(paramId==0) synth505.clap.SetDecay(val);
                    if(paramId==3) synth505.clap.volume = clampF(val,0.f,1.f);
                    break;
                case TR505::INST_HIHAT_C:
                    if(paramId==0) synth505.hihatC.SetDecay(val);
                    if(paramId==3) synth505.hihatC.volume = clampF(val,0.f,1.f);
                    break;
                case TR505::INST_HIHAT_O:
                    if(paramId==0) synth505.hihatO.SetDecay(val);
                    if(paramId==3) synth505.hihatO.volume = clampF(val,0.f,1.f);
                    break;
                case TR505::INST_LOW_TOM:
                    if(paramId==0) synth505.lowTom.SetDecay(val);
                    if(paramId==1) synth505.lowTom.SetPitch(val);
                    if(paramId==3) synth505.lowTom.volume = clampF(val,0.f,1.f);
                    break;
                case TR505::INST_MID_TOM:
                    if(paramId==0) synth505.midTom.SetDecay(val);
                    if(paramId==1) synth505.midTom.SetPitch(val);
                    if(paramId==3) synth505.midTom.volume = clampF(val,0.f,1.f);
                    break;
                case TR505::INST_HI_TOM:
                    if(paramId==0) synth505.hiTom.SetDecay(val);
                    if(paramId==1) synth505.hiTom.SetPitch(val);
                    if(paramId==3) synth505.hiTom.volume = clampF(val,0.f,1.f);
                    break;
                case TR505::INST_COWBELL:
                    if(paramId==0) synth505.cowbell.SetDecay(val);
                    if(paramId==1) synth505.cowbell.SetTune(val);
                    if(paramId==3) synth505.cowbell.volume = clampF(val,0.f,1.f);
                    if(paramId==5) synth505.cowbell.SetLoFi(val);
                    break;
                case TR505::INST_CYMBAL:
                    if(paramId==0) synth505.cymbal.SetDecay(val);
                    if(paramId==3) synth505.cymbal.volume = clampF(val,0.f,1.f);
                    if(paramId==5) synth505.cymbal.SetLoFi(val);
                    break;
                case TR505::INST_RIMSHOT:
                    if(paramId==3) synth505.rimshot.volume = clampF(val,0.f,1.f);
                    if(paramId==5) synth505.rimshot.SetLoFi(val);
                    break;
                case TR505::INST_SHAKER:
                    if(paramId==0) synth505.shaker.SetDecay(val);
                    if(paramId==2) synth505.shaker.SetTone(val);
                    if(paramId==3) synth505.shaker.volume = clampF(val,0.f,1.f);
                    if(paramId==5) synth505.shaker.SetLoFi(val);
                    break;
                case TR505::INST_CLAVE:
                    if(paramId==0) synth505.clave.SetDecay(val);
                    if(paramId==1) synth505.clave.SetPitch(val);
                    if(paramId==3) synth505.clave.volume = clampF(val,0.f,1.f);
                    if(paramId==5) synth505.clave.SetLoFi(val);
                    break;
                case TR505::INST_HI_PERC:
                    if(paramId==0) synth505.hiPerc.SetDecay(val);
                    if(paramId==1) synth505.hiPerc.SetPitch(val);
                    if(paramId==2) synth505.hiPerc.SetClick(val);
                    if(paramId==3) synth505.hiPerc.volume = clampF(val,0.f,1.f);
                    if(paramId==5) synth505.hiPerc.SetLoFi(val);
                    break;
                case TR505::INST_MID_PERC:
                    if(paramId==0) synth505.midPerc.SetDecay(val);
                    if(paramId==1) synth505.midPerc.SetPitch(val);
                    if(paramId==2) synth505.midPerc.SetClick(val);
                    if(paramId==3) synth505.midPerc.volume = clampF(val,0.f,1.f);
                    if(paramId==5) synth505.midPerc.SetLoFi(val);
                    break;
                case TR505::INST_LOW_PERC:
                    if(paramId==0) synth505.lowPerc.SetDecay(val);
                    if(paramId==1) synth505.lowPerc.SetPitch(val);
                    if(paramId==2) synth505.lowPerc.SetClick(val);
                    if(paramId==3) synth505.lowPerc.volume = clampF(val,0.f,1.f);
                    if(paramId==5) synth505.lowPerc.SetLoFi(val);
                    break;
                default:
                    if(paramId==3) synth505.SetVolume(instrument, clampF(val,0.f,2.f));
                    break;
            }
            break;
        default:
            break;
    }
}

static void ApplySh101Preset(uint8_t presetId)
{
    auto set = [](uint8_t paramId, float value) {
        synthSH101.SetParam(paramId, value);
    };

    switch(presetId)
    {
        default:
        case 0: /* Bass Punch */
            set(0, 0.0f);   set(1, 0.50f);  set(2, 0.72f);  set(3, 1.0f);
            set(4, 650.0f); set(5, 0.25f);  set(6, 0.55f);  set(7, 0.001f);
            set(8, 0.18f);  set(9, 0.00f);  set(10, 0.08f); set(11, 0.001f);
            set(12, 0.14f); set(13, 0.10f); set(14, 0.00f); set(15, 0.0f);
            set(16, 0.0f);  set(17, 0.05f); set(18, 0.04f); set(19, 0.85f);
            break;
        case 1: /* Acid Lead */
            set(0, 0.0f);   set(1, 0.42f);  set(2, 0.20f);  set(3, 0.0f);
            set(4, 1800.0f);set(5, 0.70f);  set(6, 0.75f);  set(7, 0.001f);
            set(8, 0.35f);  set(9, 0.25f);  set(10, 0.18f); set(11, 0.001f);
            set(12, 0.25f); set(13, 5.50f); set(14, 0.18f); set(15, 1.0f);
            set(16, 0.0f);  set(17, 0.12f); set(18, 0.07f); set(19, 0.80f);
            break;
        case 2: /* PWM Keys */
            set(0, 1.0f);   set(1, 0.28f);  set(2, 0.15f);  set(3, 0.0f);
            set(4, 2600.0f);set(5, 0.35f);  set(6, 0.45f);  set(7, 0.010f);
            set(8, 0.40f);  set(9, 0.55f);  set(10, 0.28f); set(11, 0.010f);
            set(12, 0.45f); set(13, 3.20f); set(14, 0.32f); set(15, 0.0f);
            set(16, 1.0f);  set(17, 0.00f); set(18, 0.03f); set(19, 0.78f);
            break;
        case 3: /* Drone Pad */
            set(0, 2.0f);   set(1, 0.50f);  set(2, 0.35f);  set(3, 1.0f);
            set(4, 1200.0f);set(5, 0.82f);  set(6, 0.60f);  set(7, 0.120f);
            set(8, 1.20f);  set(9, 0.75f);  set(10, 1.00f); set(11, 0.080f);
            set(12, 1.60f); set(13, 0.35f); set(14, 0.40f); set(15, 1.0f);
            set(16, 0.0f);  set(17, 0.18f); set(18, 0.15f); set(19, 0.72f);
            break;
    }
}

static void ApplyFm2OpPreset(uint8_t presetId)
{
    auto set = [](uint8_t paramId, float value) {
        synthFM2Op.SetParam(paramId, value);
    };

    switch(presetId)
    {
        default:
        case 0: /* FM Bass */
            set(0, 0.001f); set(1, 0.26f); set(2, 0.00f); set(3, 0.14f);
            set(4, 0.001f); set(5, 0.18f); set(6, 0.00f); set(7, 0.12f);
            set(8, 1.00f);  set(9, 4.20f); set(10, 0.06f); set(11, 0.0f);
            set(12, 0.0f);  set(13, 0.50f); set(14, 0.92f);
            break;
        case 1: /* EPiano */
            set(0, 0.001f); set(1, 1.10f); set(2, 0.20f); set(3, 0.90f);
            set(4, 0.001f); set(5, 0.70f); set(6, 0.00f); set(7, 0.48f);
            set(8, 2.00f);  set(9, 2.80f); set(10, 0.04f); set(11, 1.0f);
            set(12, 6.0f);  set(13, 0.85f); set(14, 0.88f);
            break;
        case 2: /* Bell */
            set(0, 0.001f); set(1, 2.20f); set(2, 0.00f); set(3, 1.40f);
            set(4, 0.001f); set(5, 1.10f); set(6, 0.00f); set(7, 0.80f);
            set(8, 3.00f);  set(9, 7.20f); set(10, 0.10f); set(11, 0.0f);
            set(12, 14.0f); set(13, 0.90f); set(14, 0.84f);
            break;
        case 3: /* Growl Lead */
            set(0, 0.004f); set(1, 0.44f); set(2, 0.28f); set(3, 0.22f);
            set(4, 0.001f); set(5, 0.34f); set(6, 0.12f); set(7, 0.22f);
            set(8, 1.50f);  set(9, 9.20f); set(10, 0.34f); set(11, 2.0f);
            set(12, -10.0f); set(13, 0.72f); set(14, 0.90f);
            break;
    }
}

static void ApplyPhysPreset(uint8_t presetId)
{
    auto set = [](uint8_t paramId, float value) {
        switch(paramId)
        {
            case 0: physModal.SetFreq(clampF(value, 20.f, 10000.f));    break;
            case 1: physModal.SetStructure(clampF(value, 0.f, 1.f));   break;
            case 2: physModal.SetBrightness(clampF(value, 0.f, 1.f));  break;
            case 3: physModal.SetDamping(clampF(value, 0.f, 1.f));     break;
            case 4: physModalGain = clampF(value, 0.f, 1.f);           break;
            case 5: physString.SetFreq(clampF(value, 20.f, 10000.f));   break;
            case 6: physString.SetStructure(clampF(value, 0.f, 1.f));  break;
            case 7: physString.SetBrightness(clampF(value, 0.f, 1.f)); break;
            case 8: physString.SetDamping(clampF(value, 0.f, 1.f));    break;
            case 9: physStringGain = clampF(value, 0.f, 1.f);          break;
        }
    };

    switch(presetId)
    {
        default:
        case 0: /* Clasica */
            set(0, 196.0f); set(1, 0.12f); set(2, 0.28f); set(3, 0.78f); set(4, 0.18f);
            set(5, 196.0f); set(6, 0.34f); set(7, 0.48f); set(8, 0.68f); set(9, 0.98f);
            break;
        case 1: /* Flamenco */
            set(0, 220.0f); set(1, 0.20f); set(2, 0.60f); set(3, 0.56f); set(4, 0.16f);
            set(5, 220.0f); set(6, 0.28f); set(7, 0.76f); set(8, 0.42f); set(9, 1.00f);
            break;
        case 2: /* Funky */
            set(0, 164.81f); set(1, 0.34f); set(2, 0.74f); set(3, 0.42f); set(4, 0.12f);
            set(5, 164.81f); set(6, 0.62f); set(7, 0.88f); set(8, 0.22f); set(9, 0.90f);
            break;
        case 3: /* Electrica */
            set(0, 146.83f); set(1, 0.26f); set(2, 0.86f); set(3, 0.30f); set(4, 0.24f);
            set(5, 146.83f); set(6, 0.56f); set(7, 0.98f); set(8, 0.16f); set(9, 0.96f);
            break;
    }
}

static void ApplySynthPreset(uint8_t engine, uint8_t presetId)
{
    const bool pcm909Preset = (engine == SYNTH_ENGINE_909 && presetId == 5);
    const bool pcm505Preset = (engine == SYNTH_ENGINE_505 && presetId == 5);
    uint8_t preset = (presetId < 5) ? presetId : 0;

    switch(engine)
    {
        case SYNTH_ENGINE_808:
        {
            auto set = [](uint8_t inst, uint8_t paramId, float value) {
                ApplyDrumSynthParam(SYNTH_ENGINE_808, inst, paramId, value);
            };
            switch(preset)
            {
                default:
                case 0:
                    synth808.LoadPreset(TR808::Presets::Classic808);
                    set(TR808::INST_KICK, 0, 0.48f); set(TR808::INST_KICK, 1, 50.0f); set(TR808::INST_KICK, 2, 0.45f); set(TR808::INST_KICK, 4, 0.30f); set(TR808::INST_KICK, 5, 6.5f); set(TR808::INST_KICK, 6, 0.045f); set(TR808::INST_KICK, 3, 0.92f);
                    set(TR808::INST_SNARE, 0, 0.18f); set(TR808::INST_SNARE, 1, 185.0f); set(TR808::INST_SNARE, 2, 0.55f); set(TR808::INST_SNARE, 4, 0.75f); set(TR808::INST_SNARE, 3, 0.90f);
                    set(TR808::INST_CLAP, 0, 0.28f); set(TR808::INST_CLAP, 2, 0.70f); set(TR808::INST_CLAP, 3, 0.85f);
                    set(TR808::INST_HIHAT_C, 0, 0.025f); set(TR808::INST_HIHAT_C, 3, 0.95f);
                    set(TR808::INST_HIHAT_O, 0, 0.18f); set(TR808::INST_HIHAT_O, 3, 0.90f);
                    set(TR808::INST_COWBELL, 0, 0.08f); set(TR808::INST_COWBELL, 1, 1.0f); set(TR808::INST_COWBELL, 3, 0.80f);
                    set(TR808::INST_CYMBAL, 0, 0.85f); set(TR808::INST_CYMBAL, 3, 0.80f);
                    break;
                case 1:
                    synth808.LoadPreset(TR808::Presets::HipHop);
                    set(TR808::INST_KICK, 0, 0.82f); set(TR808::INST_KICK, 1, 44.0f); set(TR808::INST_KICK, 2, 0.40f); set(TR808::INST_KICK, 4, 0.28f); set(TR808::INST_KICK, 5, 2.4f); set(TR808::INST_KICK, 6, 0.18f); set(TR808::INST_KICK, 3, 0.92f);
                    set(TR808::INST_SNARE, 0, 0.28f); set(TR808::INST_SNARE, 1, 160.0f); set(TR808::INST_SNARE, 2, 0.35f); set(TR808::INST_SNARE, 4, 0.72f); set(TR808::INST_SNARE, 3, 0.74f);
                    set(TR808::INST_CLAP, 0, 0.34f); set(TR808::INST_CLAP, 2, 0.58f); set(TR808::INST_CLAP, 3, 0.72f);
                    set(TR808::INST_HIHAT_C, 0, 0.030f); set(TR808::INST_HIHAT_C, 3, 0.55f);
                    set(TR808::INST_HIHAT_O, 0, 0.22f); set(TR808::INST_HIHAT_O, 3, 0.60f);
                    set(TR808::INST_LOW_TOM, 0, 0.42f); set(TR808::INST_LOW_TOM, 1, 70.0f); set(TR808::INST_LOW_TOM, 5, 0.22f); set(TR808::INST_LOW_TOM, 3, 0.76f);
                    set(TR808::INST_MID_TOM, 0, 0.34f); set(TR808::INST_MID_TOM, 1, 108.0f); set(TR808::INST_MID_TOM, 5, 0.20f); set(TR808::INST_MID_TOM, 3, 0.70f);
                    set(TR808::INST_HI_TOM, 0, 0.28f); set(TR808::INST_HI_TOM, 1, 162.0f); set(TR808::INST_HI_TOM, 5, 0.16f); set(TR808::INST_HI_TOM, 3, 0.68f);
                    break;
                case 2:
                    synth808.LoadPreset(TR808::Presets::Techno);
                    set(TR808::INST_KICK, 0, 0.34f); set(TR808::INST_KICK, 1, 56.0f); set(TR808::INST_KICK, 2, 0.50f); set(TR808::INST_KICK, 4, 0.12f); set(TR808::INST_KICK, 5, 4.0f); set(TR808::INST_KICK, 6, 0.035f); set(TR808::INST_KICK, 3, 0.95f);
                    set(TR808::INST_SNARE, 0, 0.16f); set(TR808::INST_SNARE, 1, 210.0f); set(TR808::INST_SNARE, 2, 0.68f); set(TR808::INST_SNARE, 4, 0.45f); set(TR808::INST_SNARE, 3, 0.82f);
                    set(TR808::INST_CLAP, 0, 0.20f); set(TR808::INST_CLAP, 2, 0.85f); set(TR808::INST_CLAP, 3, 0.62f);
                    set(TR808::INST_HIHAT_C, 0, 0.050f); set(TR808::INST_HIHAT_C, 3, 0.82f);
                    set(TR808::INST_HIHAT_O, 0, 0.40f); set(TR808::INST_HIHAT_O, 3, 0.74f);
                    set(TR808::INST_COWBELL, 0, 0.05f); set(TR808::INST_COWBELL, 1, 1.18f); set(TR808::INST_COWBELL, 3, 0.55f);
                    set(TR808::INST_CYMBAL, 0, 1.20f); set(TR808::INST_CYMBAL, 3, 0.62f);
                    break;
                case 3:
                    synth808.LoadPreset(TR808::Presets::Latin);
                    set(TR808::INST_KICK, 0, 0.30f); set(TR808::INST_KICK, 1, 54.0f); set(TR808::INST_KICK, 2, 0.16f); set(TR808::INST_KICK, 4, 0.12f); set(TR808::INST_KICK, 5, 2.6f); set(TR808::INST_KICK, 6, 0.08f); set(TR808::INST_KICK, 3, 0.68f);
                    set(TR808::INST_LOW_TOM, 0, 0.48f); set(TR808::INST_LOW_TOM, 1, 92.0f); set(TR808::INST_LOW_TOM, 5, 0.14f); set(TR808::INST_LOW_TOM, 3, 0.86f);
                    set(TR808::INST_MID_TOM, 0, 0.42f); set(TR808::INST_MID_TOM, 1, 144.0f); set(TR808::INST_MID_TOM, 5, 0.14f); set(TR808::INST_MID_TOM, 3, 0.84f);
                    set(TR808::INST_HI_TOM, 0, 0.34f); set(TR808::INST_HI_TOM, 1, 215.0f); set(TR808::INST_HI_TOM, 5, 0.10f); set(TR808::INST_HI_TOM, 3, 0.92f);
                    synth808.SetVolume(TR808::INST_LOW_CONGA, 0.92f); synth808.SetVolume(TR808::INST_MID_CONGA, 0.96f); synth808.SetVolume(TR808::INST_HI_CONGA, 1.00f);
                    synth808.SetVolume(TR808::INST_CLAVES, 0.96f); synth808.SetVolume(TR808::INST_MARACAS, 0.78f); synth808.SetVolume(TR808::INST_RIMSHOT, 0.82f);
                    break;
                case 4: /* Pure 808 — fiel al hardware analogico original */
                    synth808.LoadPreset(TR808::Presets::Pure808);
                    /* Kick: sin sub-osc, drive minimo, sweep corto 1:1, sin drift, punch reducido */
                    synth808.kick.SetDecay(0.45f);
                    synth808.kick.SetPitch(52.0f);
                    synth808.kick.drive     = 0.05f;
                    synth808.kick.subLevel  = 0.0f;
                    synth808.kick.pitchAmt  = 1.0f;
                    synth808.kick.pitchDecay= 0.05f;
                    synth808.kick.punchAmt  = 0.4f;
                    synth808.kick.drift     = 0.0f;
                    synth808.kick.volume    = 0.92f;
                    /* Snare: dos tonos no-armonicos del datasheet, sin drift */
                    synth808.snare.SetDecay(0.18f);
                    synth808.snare.SetPitch(180.0f);
                    synth808.snare.SetTone(0.50f);
                    synth808.snare.SetSnappy(0.50f);
                    synth808.snare.drift  = 0.0f;
                    synth808.snare.volume = 0.85f;
                    /* Clap, hihats, cowbell, cymbal: defaults clasicos */
                    set(TR808::INST_CLAP,    0, 0.28f); set(TR808::INST_CLAP,    2, 0.60f); set(TR808::INST_CLAP,    3, 0.85f);
                    set(TR808::INST_HIHAT_C, 0, 0.025f); set(TR808::INST_HIHAT_C, 3, 0.85f);
                    set(TR808::INST_HIHAT_O, 0, 0.18f);  set(TR808::INST_HIHAT_O, 3, 0.80f);
                    set(TR808::INST_COWBELL, 0, 0.08f); set(TR808::INST_COWBELL, 1, 1.0f); set(TR808::INST_COWBELL, 3, 0.75f);
                    set(TR808::INST_CYMBAL,  0, 0.85f); set(TR808::INST_CYMBAL,  3, 0.70f);
                    /* Toms y congas: niveles equilibrados */
                    set(TR808::INST_LOW_TOM, 0, 0.30f); set(TR808::INST_LOW_TOM, 1, 75.0f);  set(TR808::INST_LOW_TOM, 5, 0.0f); set(TR808::INST_LOW_TOM, 3, 0.80f);
                    set(TR808::INST_MID_TOM, 0, 0.30f); set(TR808::INST_MID_TOM, 1, 120.0f); set(TR808::INST_MID_TOM, 5, 0.0f); set(TR808::INST_MID_TOM, 3, 0.80f);
                    set(TR808::INST_HI_TOM,  0, 0.30f); set(TR808::INST_HI_TOM,  1, 175.0f); set(TR808::INST_HI_TOM,  5, 0.0f); set(TR808::INST_HI_TOM,  3, 0.80f);
                    break;
            }
            break;
        }
        case SYNTH_ENGINE_909:
        {
            if(pcm909Preset){
                synth909.LoadPreset(TR909::Presets::Pure909);
                Bind909PcmFromLoadedPads();
                break;
            }
            synth909PcmMode = false;
            synth909.ClearPcmSamples();
            auto set = [](uint8_t inst, uint8_t paramId, float value) {
                ApplyDrumSynthParam(SYNTH_ENGINE_909, inst, paramId, value);
            };
            switch(preset)
            {
                default:
                case 0:
                    synth909.LoadPreset(TR909::Presets::Classic909);
                    set(TR909::INST_KICK, 0, 0.50f); set(TR909::INST_KICK, 1, 48.0f); set(TR909::INST_KICK, 3, 0.92f);
                    set(TR909::INST_SNARE, 0, 0.25f); set(TR909::INST_SNARE, 2, 0.55f); set(TR909::INST_SNARE, 4, 0.68f); set(TR909::INST_SNARE, 3, 0.90f);
                    set(TR909::INST_CLAP, 0, 0.30f); set(TR909::INST_CLAP, 3, 0.85f);
                    set(TR909::INST_HIHAT_C, 0, 0.025f); set(TR909::INST_HIHAT_C, 3, 0.95f);
                    set(TR909::INST_HIHAT_O, 0, 0.20f); set(TR909::INST_HIHAT_O, 3, 0.90f);
                    set(TR909::INST_LOW_TOM, 0, 0.30f); set(TR909::INST_LOW_TOM, 1, 80.0f); set(TR909::INST_LOW_TOM, 3, 0.80f);
                    set(TR909::INST_MID_TOM, 0, 0.30f); set(TR909::INST_MID_TOM, 1, 120.0f); set(TR909::INST_MID_TOM, 3, 0.80f);
                    set(TR909::INST_HI_TOM, 0, 0.30f); set(TR909::INST_HI_TOM, 1, 180.0f); set(TR909::INST_HI_TOM, 3, 0.80f);
                    set(TR909::INST_RIDE, 0, 0.50f); set(TR909::INST_RIDE, 3, 0.80f);
                    set(TR909::INST_CRASH, 0, 0.80f); set(TR909::INST_CRASH, 3, 0.80f);
                    set(TR909::INST_SHAKER, 0, 0.085f); set(TR909::INST_SHAKER, 2, 0.65f); set(TR909::INST_SHAKER, 3, 0.72f);
                    set(TR909::INST_CLAVE, 0, 0.055f); set(TR909::INST_CLAVE, 1, 1750.0f); set(TR909::INST_CLAVE, 3, 0.76f);
                    set(TR909::INST_HI_PERC, 0, 0.085f); set(TR909::INST_HI_PERC, 1, 820.0f); set(TR909::INST_HI_PERC, 2, 0.28f); set(TR909::INST_HI_PERC, 3, 0.70f);
                    set(TR909::INST_MID_PERC, 0, 0.120f); set(TR909::INST_MID_PERC, 1, 520.0f); set(TR909::INST_MID_PERC, 2, 0.20f); set(TR909::INST_MID_PERC, 3, 0.72f);
                    set(TR909::INST_LOW_PERC, 0, 0.170f); set(TR909::INST_LOW_PERC, 1, 310.0f); set(TR909::INST_LOW_PERC, 2, 0.14f); set(TR909::INST_LOW_PERC, 3, 0.74f);
                    break;
                case 1:
                    synth909.LoadPreset(TR909::Presets::Techno);
                    set(TR909::INST_KICK, 0, 0.55f); set(TR909::INST_KICK, 1, 46.0f); set(TR909::INST_KICK, 3, 0.95f);
                    set(TR909::INST_SNARE, 0, 0.20f); set(TR909::INST_SNARE, 2, 0.68f); set(TR909::INST_SNARE, 4, 0.72f); set(TR909::INST_SNARE, 3, 0.84f);
                    set(TR909::INST_CLAP, 0, 0.18f); set(TR909::INST_CLAP, 3, 0.62f);
                    set(TR909::INST_HIHAT_C, 0, 0.05f); set(TR909::INST_HIHAT_C, 3, 0.88f);
                    set(TR909::INST_HIHAT_O, 0, 0.42f); set(TR909::INST_HIHAT_O, 3, 0.80f);
                    set(TR909::INST_LOW_TOM, 0, 0.22f); set(TR909::INST_LOW_TOM, 1, 76.0f); set(TR909::INST_LOW_TOM, 3, 0.60f);
                    set(TR909::INST_MID_TOM, 0, 0.22f); set(TR909::INST_MID_TOM, 1, 116.0f); set(TR909::INST_MID_TOM, 3, 0.60f);
                    set(TR909::INST_HI_TOM, 0, 0.20f); set(TR909::INST_HI_TOM, 1, 170.0f); set(TR909::INST_HI_TOM, 3, 0.60f);
                    set(TR909::INST_RIDE, 0, 0.85f); set(TR909::INST_RIDE, 3, 0.74f);
                    set(TR909::INST_CRASH, 0, 0.50f); set(TR909::INST_CRASH, 3, 0.56f);
                    set(TR909::INST_SHAKER, 0, 0.060f); set(TR909::INST_SHAKER, 2, 0.88f); set(TR909::INST_SHAKER, 3, 0.86f);
                    set(TR909::INST_CLAVE, 0, 0.035f); set(TR909::INST_CLAVE, 1, 2150.0f); set(TR909::INST_CLAVE, 3, 0.78f);
                    set(TR909::INST_HI_PERC, 0, 0.060f); set(TR909::INST_HI_PERC, 1, 980.0f); set(TR909::INST_HI_PERC, 2, 0.38f); set(TR909::INST_HI_PERC, 3, 0.64f);
                    set(TR909::INST_MID_PERC, 0, 0.095f); set(TR909::INST_MID_PERC, 1, 610.0f); set(TR909::INST_MID_PERC, 2, 0.28f); set(TR909::INST_MID_PERC, 3, 0.66f);
                    set(TR909::INST_LOW_PERC, 0, 0.130f); set(TR909::INST_LOW_PERC, 1, 360.0f); set(TR909::INST_LOW_PERC, 2, 0.20f); set(TR909::INST_LOW_PERC, 3, 0.68f);
                    break;
                case 2:
                    synth909.LoadPreset(TR909::Presets::HousePound);
                    set(TR909::INST_KICK, 0, 0.62f); set(TR909::INST_KICK, 1, 42.0f); set(TR909::INST_KICK, 3, 0.92f);
                    set(TR909::INST_SNARE, 0, 0.22f); set(TR909::INST_SNARE, 2, 0.42f); set(TR909::INST_SNARE, 4, 0.40f); set(TR909::INST_SNARE, 3, 0.70f);
                    set(TR909::INST_CLAP, 0, 0.34f); set(TR909::INST_CLAP, 3, 0.92f);
                    set(TR909::INST_HIHAT_C, 0, 0.032f); set(TR909::INST_HIHAT_C, 3, 0.66f);
                    set(TR909::INST_HIHAT_O, 0, 0.24f); set(TR909::INST_HIHAT_O, 3, 0.74f);
                    set(TR909::INST_LOW_TOM, 0, 0.34f); set(TR909::INST_LOW_TOM, 1, 78.0f); set(TR909::INST_LOW_TOM, 3, 0.68f);
                    set(TR909::INST_MID_TOM, 0, 0.34f); set(TR909::INST_MID_TOM, 1, 118.0f); set(TR909::INST_MID_TOM, 3, 0.68f);
                    set(TR909::INST_HI_TOM, 0, 0.32f); set(TR909::INST_HI_TOM, 1, 176.0f); set(TR909::INST_HI_TOM, 3, 0.68f);
                    set(TR909::INST_RIDE, 0, 0.95f); set(TR909::INST_RIDE, 3, 0.86f);
                    set(TR909::INST_CRASH, 0, 0.58f); set(TR909::INST_CRASH, 3, 0.62f);
                    set(TR909::INST_SHAKER, 0, 0.105f); set(TR909::INST_SHAKER, 2, 0.52f); set(TR909::INST_SHAKER, 3, 0.58f);
                    set(TR909::INST_CLAVE, 0, 0.060f); set(TR909::INST_CLAVE, 1, 1650.0f); set(TR909::INST_CLAVE, 3, 0.62f);
                    set(TR909::INST_HI_PERC, 0, 0.095f); set(TR909::INST_HI_PERC, 1, 740.0f); set(TR909::INST_HI_PERC, 2, 0.20f); set(TR909::INST_HI_PERC, 3, 0.58f);
                    set(TR909::INST_MID_PERC, 0, 0.135f); set(TR909::INST_MID_PERC, 1, 480.0f); set(TR909::INST_MID_PERC, 2, 0.16f); set(TR909::INST_MID_PERC, 3, 0.60f);
                    set(TR909::INST_LOW_PERC, 0, 0.190f); set(TR909::INST_LOW_PERC, 1, 290.0f); set(TR909::INST_LOW_PERC, 2, 0.12f); set(TR909::INST_LOW_PERC, 3, 0.62f);
                    break;
                case 3:
                    synth909.LoadPreset(TR909::Presets::Industrial);
                    set(TR909::INST_KICK, 0, 0.70f); set(TR909::INST_KICK, 1, 58.0f); set(TR909::INST_KICK, 3, 1.00f);
                    set(TR909::INST_SNARE, 0, 0.34f); set(TR909::INST_SNARE, 2, 0.82f); set(TR909::INST_SNARE, 4, 0.86f); set(TR909::INST_SNARE, 3, 0.95f);
                    set(TR909::INST_CLAP, 0, 0.40f); set(TR909::INST_CLAP, 3, 0.88f);
                    set(TR909::INST_HIHAT_C, 0, 0.06f); set(TR909::INST_HIHAT_C, 3, 0.96f);
                    set(TR909::INST_HIHAT_O, 0, 0.52f); set(TR909::INST_HIHAT_O, 3, 0.90f);
                    set(TR909::INST_LOW_TOM, 0, 0.38f); set(TR909::INST_LOW_TOM, 1, 90.0f); set(TR909::INST_LOW_TOM, 3, 0.76f);
                    set(TR909::INST_MID_TOM, 0, 0.38f); set(TR909::INST_MID_TOM, 1, 136.0f); set(TR909::INST_MID_TOM, 3, 0.76f);
                    set(TR909::INST_HI_TOM, 0, 0.36f); set(TR909::INST_HI_TOM, 1, 196.0f); set(TR909::INST_HI_TOM, 3, 0.76f);
                    set(TR909::INST_RIDE, 0, 1.40f); set(TR909::INST_RIDE, 3, 0.66f);
                    set(TR909::INST_CRASH, 0, 1.80f); set(TR909::INST_CRASH, 3, 0.82f);
                    set(TR909::INST_SHAKER, 0, 0.045f); set(TR909::INST_SHAKER, 2, 1.00f); set(TR909::INST_SHAKER, 3, 0.98f);
                    set(TR909::INST_CLAVE, 0, 0.030f); set(TR909::INST_CLAVE, 1, 2450.0f); set(TR909::INST_CLAVE, 3, 0.92f);
                    set(TR909::INST_HI_PERC, 0, 0.050f); set(TR909::INST_HI_PERC, 1, 1120.0f); set(TR909::INST_HI_PERC, 2, 0.60f); set(TR909::INST_HI_PERC, 3, 0.84f);
                    set(TR909::INST_MID_PERC, 0, 0.080f); set(TR909::INST_MID_PERC, 1, 690.0f); set(TR909::INST_MID_PERC, 2, 0.44f); set(TR909::INST_MID_PERC, 3, 0.86f);
                    set(TR909::INST_LOW_PERC, 0, 0.110f); set(TR909::INST_LOW_PERC, 1, 410.0f); set(TR909::INST_LOW_PERC, 2, 0.32f); set(TR909::INST_LOW_PERC, 3, 0.88f);
                    break;
                case 4: /* Pure 909 — fiel al hardware original (kick beater click claro, sin saturacion) */
                    synth909.LoadPreset(TR909::Presets::Pure909);
                    set(TR909::INST_KICK,    0, 0.40f); set(TR909::INST_KICK,    1, 50.0f);  set(TR909::INST_KICK,    3, 0.92f);
                    set(TR909::INST_SNARE,   0, 0.20f); set(TR909::INST_SNARE,   2, 0.55f);  set(TR909::INST_SNARE,   4, 0.55f); set(TR909::INST_SNARE,   3, 0.88f);
                    set(TR909::INST_CLAP,    0, 0.28f); set(TR909::INST_CLAP,    3, 0.82f);
                    set(TR909::INST_HIHAT_C, 0, 0.022f);set(TR909::INST_HIHAT_C, 3, 0.90f);
                    set(TR909::INST_HIHAT_O, 0, 0.18f); set(TR909::INST_HIHAT_O, 3, 0.85f);
                    set(TR909::INST_LOW_TOM, 0, 0.30f); set(TR909::INST_LOW_TOM, 1, 80.0f);  set(TR909::INST_LOW_TOM, 3, 0.80f);
                    set(TR909::INST_MID_TOM, 0, 0.30f); set(TR909::INST_MID_TOM, 1, 120.0f); set(TR909::INST_MID_TOM, 3, 0.80f);
                    set(TR909::INST_HI_TOM,  0, 0.30f); set(TR909::INST_HI_TOM,  1, 180.0f); set(TR909::INST_HI_TOM,  3, 0.80f);
                    set(TR909::INST_RIDE,    0, 0.55f); set(TR909::INST_RIDE,    3, 0.78f);
                    set(TR909::INST_CRASH,   0, 0.85f); set(TR909::INST_CRASH,   3, 0.75f);
                    set(TR909::INST_SHAKER,  0, 0.085f); set(TR909::INST_SHAKER,  2, 0.62f); set(TR909::INST_SHAKER,  3, 0.72f);
                    set(TR909::INST_CLAVE,   0, 0.055f); set(TR909::INST_CLAVE,   1, 1750.0f); set(TR909::INST_CLAVE,   3, 0.74f);
                    set(TR909::INST_HI_PERC, 0, 0.085f); set(TR909::INST_HI_PERC, 1, 820.0f); set(TR909::INST_HI_PERC, 2, 0.24f); set(TR909::INST_HI_PERC, 3, 0.70f);
                    set(TR909::INST_MID_PERC,0, 0.120f); set(TR909::INST_MID_PERC,1, 520.0f); set(TR909::INST_MID_PERC,2, 0.18f); set(TR909::INST_MID_PERC,3, 0.72f);
                    set(TR909::INST_LOW_PERC,0, 0.170f); set(TR909::INST_LOW_PERC,1, 310.0f); set(TR909::INST_LOW_PERC,2, 0.12f); set(TR909::INST_LOW_PERC,3, 0.74f);
                    break;
            }
            break;
        }
        case SYNTH_ENGINE_505:
        {
            if(pcm505Preset){
                synth505.LoadPreset(TR505::Presets::Pure505);
                Bind505PcmFromLoadedPads();
                break;
            }
            synth505PcmMode = false;
            synth505.ClearPcmSamples();
            auto set = [](uint8_t inst, uint8_t paramId, float value) {
                ApplyDrumSynthParam(SYNTH_ENGINE_505, inst, paramId, value);
            };
            switch(preset)
            {
                default:
                case 0:
                    synth505.LoadPreset(TR505::Presets::Classic505);
                    set(TR505::INST_KICK, 0, 0.40f); set(TR505::INST_KICK, 1, 55.0f); set(TR505::INST_KICK, 3, 0.90f);
                    set(TR505::INST_SNARE, 0, 0.25f); set(TR505::INST_SNARE, 2, 0.58f); set(TR505::INST_SNARE, 3, 0.88f);
                    set(TR505::INST_CLAP, 0, 0.30f); set(TR505::INST_CLAP, 3, 0.85f);
                    set(TR505::INST_HIHAT_C, 0, 0.025f); set(TR505::INST_HIHAT_C, 3, 0.95f);
                    set(TR505::INST_HIHAT_O, 0, 0.20f); set(TR505::INST_HIHAT_O, 3, 0.90f);
                    set(TR505::INST_LOW_TOM, 0, 0.30f); set(TR505::INST_LOW_TOM, 1, 80.0f); set(TR505::INST_LOW_TOM, 3, 0.80f);
                    set(TR505::INST_MID_TOM, 0, 0.30f); set(TR505::INST_MID_TOM, 1, 120.0f); set(TR505::INST_MID_TOM, 3, 0.80f);
                    set(TR505::INST_HI_TOM, 0, 0.30f); set(TR505::INST_HI_TOM, 1, 180.0f); set(TR505::INST_HI_TOM, 3, 0.80f);
                    set(TR505::INST_COWBELL, 0, 0.10f); set(TR505::INST_COWBELL, 3, 0.80f);
                    set(TR505::INST_CYMBAL, 0, 0.80f); set(TR505::INST_CYMBAL, 3, 0.80f);
                    set(TR505::INST_SHAKER, 0, 0.095f); set(TR505::INST_SHAKER, 2, 0.55f); set(TR505::INST_SHAKER, 3, 0.76f); set(TR505::INST_SHAKER, 5, 0.35f);
                    set(TR505::INST_CLAVE, 0, 0.050f); set(TR505::INST_CLAVE, 1, 1650.0f); set(TR505::INST_CLAVE, 3, 0.76f); set(TR505::INST_CLAVE, 5, 0.30f);
                    set(TR505::INST_HI_PERC, 0, 0.075f); set(TR505::INST_HI_PERC, 1, 760.0f); set(TR505::INST_HI_PERC, 2, 0.34f); set(TR505::INST_HI_PERC, 3, 0.70f); set(TR505::INST_HI_PERC, 5, 0.34f);
                    set(TR505::INST_MID_PERC, 0, 0.115f); set(TR505::INST_MID_PERC, 1, 480.0f); set(TR505::INST_MID_PERC, 2, 0.26f); set(TR505::INST_MID_PERC, 3, 0.72f); set(TR505::INST_MID_PERC, 5, 0.34f);
                    set(TR505::INST_LOW_PERC, 0, 0.160f); set(TR505::INST_LOW_PERC, 1, 300.0f); set(TR505::INST_LOW_PERC, 2, 0.18f); set(TR505::INST_LOW_PERC, 3, 0.74f); set(TR505::INST_LOW_PERC, 5, 0.34f);
                    break;
                case 1:
                    synth505.LoadPreset(TR505::Presets::NewWave);
                    set(TR505::INST_KICK, 0, 0.24f); set(TR505::INST_KICK, 1, 68.0f); set(TR505::INST_KICK, 3, 0.72f);
                    set(TR505::INST_SNARE, 0, 0.22f); set(TR505::INST_SNARE, 2, 0.62f); set(TR505::INST_SNARE, 3, 0.82f);
                    set(TR505::INST_CLAP, 0, 0.22f); set(TR505::INST_CLAP, 3, 0.66f);
                    set(TR505::INST_HIHAT_C, 0, 0.05f); set(TR505::INST_HIHAT_C, 3, 0.90f);
                    set(TR505::INST_HIHAT_O, 0, 0.24f); set(TR505::INST_HIHAT_O, 3, 0.82f);
                    set(TR505::INST_COWBELL, 0, 0.14f); set(TR505::INST_COWBELL, 3, 0.98f);
                    set(TR505::INST_CYMBAL, 0, 0.42f); set(TR505::INST_CYMBAL, 3, 0.62f);
                    set(TR505::INST_SHAKER, 0, 0.070f); set(TR505::INST_SHAKER, 2, 0.78f); set(TR505::INST_SHAKER, 3, 0.92f); set(TR505::INST_SHAKER, 5, 0.28f);
                    set(TR505::INST_CLAVE, 0, 0.042f); set(TR505::INST_CLAVE, 1, 1900.0f); set(TR505::INST_CLAVE, 3, 0.84f); set(TR505::INST_CLAVE, 5, 0.24f);
                    set(TR505::INST_HI_PERC, 0, 0.065f); set(TR505::INST_HI_PERC, 1, 860.0f); set(TR505::INST_HI_PERC, 2, 0.42f); set(TR505::INST_HI_PERC, 3, 0.76f); set(TR505::INST_HI_PERC, 5, 0.28f);
                    set(TR505::INST_MID_PERC, 0, 0.100f); set(TR505::INST_MID_PERC, 1, 540.0f); set(TR505::INST_MID_PERC, 2, 0.32f); set(TR505::INST_MID_PERC, 3, 0.78f); set(TR505::INST_MID_PERC, 5, 0.28f);
                    set(TR505::INST_LOW_PERC, 0, 0.140f); set(TR505::INST_LOW_PERC, 1, 340.0f); set(TR505::INST_LOW_PERC, 2, 0.22f); set(TR505::INST_LOW_PERC, 3, 0.80f); set(TR505::INST_LOW_PERC, 5, 0.28f);
                    break;
                case 2:
                    synth505.LoadPreset(TR505::Presets::Electro);
                    set(TR505::INST_KICK, 0, 0.30f); set(TR505::INST_KICK, 1, 60.0f); set(TR505::INST_KICK, 3, 0.92f);
                    set(TR505::INST_SNARE, 0, 0.18f); set(TR505::INST_SNARE, 2, 0.70f); set(TR505::INST_SNARE, 3, 0.84f);
                    set(TR505::INST_CLAP, 0, 0.18f); set(TR505::INST_CLAP, 3, 0.60f);
                    set(TR505::INST_HIHAT_C, 0, 0.05f); set(TR505::INST_HIHAT_C, 3, 0.80f);
                    set(TR505::INST_HIHAT_O, 0, 0.22f); set(TR505::INST_HIHAT_O, 3, 0.72f);
                    set(TR505::INST_LOW_TOM, 0, 0.24f); set(TR505::INST_LOW_TOM, 1, 92.0f); set(TR505::INST_LOW_TOM, 3, 0.72f);
                    set(TR505::INST_MID_TOM, 0, 0.24f); set(TR505::INST_MID_TOM, 1, 136.0f); set(TR505::INST_MID_TOM, 3, 0.72f);
                    set(TR505::INST_HI_TOM, 0, 0.22f); set(TR505::INST_HI_TOM, 1, 196.0f); set(TR505::INST_HI_TOM, 3, 0.72f);
                    set(TR505::INST_SHAKER, 0, 0.055f); set(TR505::INST_SHAKER, 2, 0.86f); set(TR505::INST_SHAKER, 3, 0.78f); set(TR505::INST_SHAKER, 5, 0.18f);
                    set(TR505::INST_CLAVE, 0, 0.034f); set(TR505::INST_CLAVE, 1, 2150.0f); set(TR505::INST_CLAVE, 3, 0.76f); set(TR505::INST_CLAVE, 5, 0.16f);
                    set(TR505::INST_HI_PERC, 0, 0.052f); set(TR505::INST_HI_PERC, 1, 960.0f); set(TR505::INST_HI_PERC, 2, 0.50f); set(TR505::INST_HI_PERC, 3, 0.72f); set(TR505::INST_HI_PERC, 5, 0.18f);
                    set(TR505::INST_MID_PERC, 0, 0.082f); set(TR505::INST_MID_PERC, 1, 610.0f); set(TR505::INST_MID_PERC, 2, 0.38f); set(TR505::INST_MID_PERC, 3, 0.74f); set(TR505::INST_MID_PERC, 5, 0.18f);
                    set(TR505::INST_LOW_PERC, 0, 0.112f); set(TR505::INST_LOW_PERC, 1, 390.0f); set(TR505::INST_LOW_PERC, 2, 0.28f); set(TR505::INST_LOW_PERC, 3, 0.76f); set(TR505::INST_LOW_PERC, 5, 0.18f);
                    break;
                case 3:
                    synth505.LoadPreset(TR505::Presets::LoFiHipHop);
                    set(TR505::INST_KICK, 0, 0.55f); set(TR505::INST_KICK, 1, 48.0f); set(TR505::INST_KICK, 3, 0.88f);
                    set(TR505::INST_SNARE, 0, 0.32f); set(TR505::INST_SNARE, 2, 0.32f); set(TR505::INST_SNARE, 3, 0.74f);
                    set(TR505::INST_CLAP, 0, 0.40f); set(TR505::INST_CLAP, 3, 0.64f);
                    set(TR505::INST_HIHAT_C, 0, 0.03f); set(TR505::INST_HIHAT_C, 3, 0.58f);
                    set(TR505::INST_HIHAT_O, 0, 0.18f); set(TR505::INST_HIHAT_O, 3, 0.58f);
                    set(TR505::INST_LOW_TOM, 0, 0.36f); set(TR505::INST_LOW_TOM, 1, 74.0f); set(TR505::INST_LOW_TOM, 3, 0.78f);
                    set(TR505::INST_MID_TOM, 0, 0.34f); set(TR505::INST_MID_TOM, 1, 110.0f); set(TR505::INST_MID_TOM, 3, 0.78f);
                    set(TR505::INST_HI_TOM, 0, 0.30f); set(TR505::INST_HI_TOM, 1, 168.0f); set(TR505::INST_HI_TOM, 3, 0.74f);
                    set(TR505::INST_COWBELL, 0, 0.08f); set(TR505::INST_COWBELL, 3, 0.44f);
                    set(TR505::INST_CYMBAL, 0, 1.10f); set(TR505::INST_CYMBAL, 3, 0.50f);
                    set(TR505::INST_SHAKER, 0, 0.130f); set(TR505::INST_SHAKER, 2, 0.38f); set(TR505::INST_SHAKER, 3, 0.70f); set(TR505::INST_SHAKER, 5, 0.72f);
                    set(TR505::INST_CLAVE, 0, 0.070f); set(TR505::INST_CLAVE, 1, 1450.0f); set(TR505::INST_CLAVE, 3, 0.66f); set(TR505::INST_CLAVE, 5, 0.68f);
                    set(TR505::INST_HI_PERC, 0, 0.110f); set(TR505::INST_HI_PERC, 1, 660.0f); set(TR505::INST_HI_PERC, 2, 0.26f); set(TR505::INST_HI_PERC, 3, 0.66f); set(TR505::INST_HI_PERC, 5, 0.70f);
                    set(TR505::INST_MID_PERC, 0, 0.155f); set(TR505::INST_MID_PERC, 1, 420.0f); set(TR505::INST_MID_PERC, 2, 0.20f); set(TR505::INST_MID_PERC, 3, 0.68f); set(TR505::INST_MID_PERC, 5, 0.70f);
                    set(TR505::INST_LOW_PERC, 0, 0.220f); set(TR505::INST_LOW_PERC, 1, 250.0f); set(TR505::INST_LOW_PERC, 2, 0.14f); set(TR505::INST_LOW_PERC, 3, 0.70f); set(TR505::INST_LOW_PERC, 5, 0.70f);
                    break;
                case 4: /* Pure 505 — fiel al original (sample-based digital limpio, sin lofi) */
                    synth505.LoadPreset(TR505::Presets::Pure505);
                    set(TR505::INST_KICK,    0, 0.40f); set(TR505::INST_KICK,    1, 55.0f);  set(TR505::INST_KICK,    3, 0.90f);
                    set(TR505::INST_SNARE,   0, 0.25f); set(TR505::INST_SNARE,   2, 0.55f);  set(TR505::INST_SNARE,   3, 0.88f);
                    set(TR505::INST_CLAP,    0, 0.28f); set(TR505::INST_CLAP,    3, 0.82f);
                    set(TR505::INST_HIHAT_C, 0, 0.025f);set(TR505::INST_HIHAT_C, 3, 0.92f);
                    set(TR505::INST_HIHAT_O, 0, 0.20f); set(TR505::INST_HIHAT_O, 3, 0.86f);
                    set(TR505::INST_LOW_TOM, 0, 0.30f); set(TR505::INST_LOW_TOM, 1, 80.0f);  set(TR505::INST_LOW_TOM, 3, 0.80f);
                    set(TR505::INST_MID_TOM, 0, 0.30f); set(TR505::INST_MID_TOM, 1, 120.0f); set(TR505::INST_MID_TOM, 3, 0.80f);
                    set(TR505::INST_HI_TOM,  0, 0.30f); set(TR505::INST_HI_TOM,  1, 180.0f); set(TR505::INST_HI_TOM,  3, 0.80f);
                    set(TR505::INST_COWBELL, 0, 0.10f); set(TR505::INST_COWBELL, 3, 0.78f);
                    set(TR505::INST_CYMBAL,  0, 0.80f); set(TR505::INST_CYMBAL,  3, 0.74f);
                    set(TR505::INST_SHAKER,  0, 0.095f); set(TR505::INST_SHAKER,  2, 0.55f); set(TR505::INST_SHAKER,  3, 0.76f); set(TR505::INST_SHAKER,  5, 0.0f);
                    set(TR505::INST_CLAVE,   0, 0.050f); set(TR505::INST_CLAVE,   1, 1650.0f); set(TR505::INST_CLAVE,   3, 0.76f); set(TR505::INST_CLAVE,   5, 0.0f);
                    set(TR505::INST_HI_PERC, 0, 0.075f); set(TR505::INST_HI_PERC, 1, 760.0f); set(TR505::INST_HI_PERC, 2, 0.30f); set(TR505::INST_HI_PERC, 3, 0.70f); set(TR505::INST_HI_PERC, 5, 0.0f);
                    set(TR505::INST_MID_PERC,0, 0.115f); set(TR505::INST_MID_PERC,1, 480.0f); set(TR505::INST_MID_PERC,2, 0.22f); set(TR505::INST_MID_PERC,3, 0.72f); set(TR505::INST_MID_PERC,5, 0.0f);
                    set(TR505::INST_LOW_PERC,0, 0.160f); set(TR505::INST_LOW_PERC,1, 300.0f); set(TR505::INST_LOW_PERC,2, 0.16f); set(TR505::INST_LOW_PERC,3, 0.74f); set(TR505::INST_LOW_PERC,5, 0.0f);
                    break;
            }
            break;
        }
        case SYNTH_ENGINE_303:
            switch(preset)
            {
                default:
                case 0: /* Classic Acid */
                    acid303.SetCutoff(1200.0f);
                    acid303.SetResonance(0.72f);
                    acid303.SetEnvMod(0.65f);
                    acid303.SetDecay(0.35f);
                    acid303.SetAccent(0.60f);
                    acid303.SetSlide(0.09f);
                    acid303.SetWaveform(TB303::WAVE_SAW);
                    acid303.SetVolume(0.80f);
                    acid303.SetAttack(0.001f);
                    acid303.SetSustain(0.00f);
                    acid303.SetRelease(0.15f);
                    acid303.SetOverdrive(0.12f);
                    acid303.SetSubLevel(0.08f);
                    acid303.SetDrift(0.04f);
                    acid303.SetPitchBend(0.0f);
                    break;
                case 1: /* Resonant Squelch */
                    acid303.SetCutoff(900.0f);
                    acid303.SetResonance(0.92f);
                    acid303.SetEnvMod(0.95f);
                    acid303.SetDecay(0.45f);
                    acid303.SetAccent(0.85f);
                    acid303.SetSlide(0.12f);
                    acid303.SetWaveform(TB303::WAVE_SAW);
                    acid303.SetVolume(0.85f);
                    acid303.SetAttack(0.001f);
                    acid303.SetSustain(0.00f);
                    acid303.SetRelease(0.18f);
                    acid303.SetOverdrive(0.28f);
                    acid303.SetSubLevel(0.06f);
                    acid303.SetDrift(0.08f);
                    acid303.SetPitchBend(0.0f);
                    break;
                case 2: /* Sub Bass */
                    acid303.SetCutoff(240.0f);
                    acid303.SetResonance(0.45f);
                    acid303.SetEnvMod(0.25f);
                    acid303.SetDecay(0.60f);
                    acid303.SetAccent(0.25f);
                    acid303.SetSlide(0.06f);
                    acid303.SetWaveform(TB303::WAVE_SQUARE);
                    acid303.SetVolume(0.90f);
                    acid303.SetAttack(0.004f);
                    acid303.SetSustain(0.45f);
                    acid303.SetRelease(0.35f);
                    acid303.SetOverdrive(0.18f);
                    acid303.SetSubLevel(0.45f);
                    acid303.SetDrift(0.02f);
                    acid303.SetPitchBend(0.0f);
                    break;
                case 3: /* Soft Lead */
                    acid303.SetCutoff(2200.0f);
                    acid303.SetResonance(0.58f);
                    acid303.SetEnvMod(0.40f);
                    acid303.SetDecay(0.80f);
                    acid303.SetAccent(0.35f);
                    acid303.SetSlide(0.15f);
                    acid303.SetWaveform(TB303::WAVE_SQUARE);
                    acid303.SetVolume(0.75f);
                    acid303.SetAttack(0.010f);
                    acid303.SetSustain(0.35f);
                    acid303.SetRelease(0.40f);
                    acid303.SetOverdrive(0.08f);
                    acid303.SetSubLevel(0.18f);
                    acid303.SetDrift(0.12f);
                    acid303.SetPitchBend(0.0f);
                    break;
            }
            break;
        case SYNTH_ENGINE_WTOSC:
            switch(preset)
            {
                default:
                case 0: /* Classic Pad */
                    wtOsc.SetWavePos(1.0f);
                    wtOsc.SetAttack(24.0f);
                    wtOsc.SetDecay(1100.0f);
                    wtOsc.volume = 0.84f;
                    wtFilterCutoffState = 7600.0f;
                    wtFilterQState      = 0.70f;
                    wtLfoRateState      = 0.12f;
                    wtLfoDepthState     = 0.06f;
                    wtLfoTargetState    = WT_LFO_WAVE;
                    break;
                case 1: /* Glass Pluck */
                    wtOsc.SetWavePos(2.4f);
                    wtOsc.SetAttack(0.0f);
                    wtOsc.SetDecay(220.0f);
                    wtOsc.volume = 0.92f;
                    wtFilterCutoffState = 7200.0f;
                    wtFilterQState      = 0.90f;
                    wtLfoRateState      = 2.20f;
                    wtLfoDepthState     = 0.04f;
                    wtLfoTargetState    = WT_LFO_WAVE;
                    break;
                case 2: /* Organ Motion */
                    wtOsc.SetWavePos(5.8f);
                    wtOsc.SetAttack(6.0f);
                    wtOsc.SetDecay(1800.0f);
                    wtOsc.volume = 0.88f;
                    wtFilterCutoffState = 9800.0f;
                    wtFilterQState      = 0.65f;
                    wtLfoRateState      = 0.35f;
                    wtLfoDepthState     = 0.10f;
                    wtLfoTargetState    = WT_LFO_VOL;
                    break;
                case 3: /* PWM Bass */
                    wtOsc.SetWavePos(3.6f);
                    wtOsc.SetAttack(0.0f);
                    wtOsc.SetDecay(360.0f);
                    wtOsc.volume = 0.96f;
                    wtFilterCutoffState = 3600.0f;
                    wtFilterQState      = 1.10f;
                    wtLfoRateState      = 1.10f;
                    wtLfoDepthState     = 0.05f;
                    wtLfoTargetState    = WT_LFO_WAVE;
                    break;
            }
            ApplyWtModState();
            break;
        case SYNTH_ENGINE_SH101:
            ApplySh101Preset(preset);
            break;
        case SYNTH_ENGINE_FM2OP:
            ApplyFm2OpPreset(preset);
            break;
        case SYNTH_ENGINE_PHYS:
            ApplyPhysPreset(preset);
            break;
        default:
            break;
    }
}

static void ApplyDefaultSynthPresets()
{
    ApplySynthPreset(SYNTH_ENGINE_808, 0);
    ApplySynthPreset(SYNTH_ENGINE_909, 0);
    ApplySynthPreset(SYNTH_ENGINE_505, 0);
    ApplySynthPreset(SYNTH_ENGINE_303, 0);
    ApplySynthPreset(SYNTH_ENGINE_WTOSC, 0);
    ApplySynthPreset(SYNTH_ENGINE_SH101, 0);
    ApplySynthPreset(SYNTH_ENGINE_FM2OP, 0);
    ApplySynthPreset(SYNTH_ENGINE_PHYS, 0);
}

/* ═══════════════════════════════════════════════════════════════════
 *  20. TRIGGER
 * ═══════════════════════════════════════════════════════════════════ */
static void TriggerPad(uint8_t pad, uint8_t velocity,
                       uint8_t trkVol, int8_t pan,
                       uint32_t maxSamples,
                       float sourceVolume,
                       float sourcePitch,
                       bool liveSource)
{
    if(pad >= MAX_PADS || !sampleLoaded[pad] || padLoading[pad]) return;

    /* ── Choke group: silence any other pad in the same group ── */
    uint8_t grp = chokeGroup[pad];
    if(grp > 0){
        for(int cp = 0; cp < MAX_PADS; cp++){
            if(cp == pad) continue;
            if(chokeGroup[cp] == grp){
                for(int v = 0; v < MAX_VOICES; v++)
                    if(voices[v].active && voices[v].pad == (uint8_t)cp)
                        voices[v].active = false;
            }
        }
    }

    /* Find a free slot or steal by priority+age. */
    int slot = -1;

    /* 1. Free slot */
    for(int i = 0; i < MAX_VOICES; i++)
        if(!voices[i].active){ slot = i; break; }

    /* Priority-aware stealing: prefer same-pad, then lowest priority + oldest. */
    if(slot < 0){
        VoicePriority myPri = PadPriority(pad);
        int best = -1;
        int bestScore = -1; /* lower priority = higher score; tie-break by age */
        for(int i = 0; i < MAX_VOICES; i++){
            /* Same pad → immediate reuse */
            if(voices[i].pad == pad){ best = i; break; }
            VoicePriority vp = PadPriority(voices[i].pad);
            /* Never steal a higher-priority voice */
            if(vp > myPri) continue;
            int score = (2 - (int)vp) * 100000 + (int)(voiceAge - voices[i].age);
            if(score > bestScore){ bestScore = score; best = i; }
        }
        if(best < 0) best = 0; /* absolute fallback */
        slot = best;
    }

    /* Preserve the last routed value of a stolen voice as a short residual.
     * The replacement starts now, so timing remains sample-accurate. */
    float stealTailL = voices[slot].active ? voices[slot].lastOutL : 0.0f;
    float stealTailR = voices[slot].active ? voices[slot].lastOutR : 0.0f;

    /* Non-destructive trim window (see padTrimStartPct comment). start>=end
     * means "no trim set" (covers both the zero-init and any malformed
     * CMD_PAD_TRIM) and falls back to the full sample. */
    uint32_t trimStart = (uint32_t)(padTrimStartPct[pad] * (float)sampleLength[pad]);
    uint32_t trimEnd   = (uint32_t)(padTrimEndPct[pad]   * (float)sampleLength[pad]);
    if(trimEnd > sampleLength[pad]) trimEnd = sampleLength[pad];
    if(trimStart >= trimEnd){ trimStart = 0; trimEnd = sampleLength[pad]; }

    uint32_t len = trimEnd;
    if(maxSamples > 0 && trimStart + maxSamples < len) len = trimStart + maxSamples;

    /* Guardar límite efectivo en la voz */
    voices[slot].maxLen = len;
    voices[slot].trimStart = trimStart;

    float gain = (velocity / 127.0f)
               * VolumeByteToGain(trkVol)
               * trackGain[pad]
               * clampF(sourceVolume, 0.0f, 1.5f);
    float panF = trackPanF[pad] + (pan / 100.0f);
    panF = clampF(panF, -1.0f, 1.0f);
    float gL = gain * (1.0f - clampF(panF, 0.f, 1.f));
    float gR = gain * (1.0f + clampF(panF, -1.f, 0.f));

    voices[slot].active       = true;
    voices[slot].pad          = pad;
    voices[slot].pos          = padReverse[pad] ? (float)(trimEnd - 1) : (float)trimStart;
    voices[slot].speed        = PadPlaybackSpeed(pad, sourcePitch);
    voices[slot].baseGain     = gain;  // gain pre-pan — para LFO vol/pan live update
    voices[slot].gainL        = gL;
    voices[slot].gainR        = gR;
    voices[slot].liveSource   = liveSource;
    voices[slot].lastOutL     = 0.0f;
    voices[slot].lastOutR     = 0.0f;
    voices[slot].stealTailL   = stealTailL;
    voices[slot].stealTailR   = stealTailR;
    if(trkEnvAdActive[pad]){
        float atkMs = clampF(trkEnvAttackMs[pad], 0.0f, 2000.0f);
        voices[slot].env = (atkMs <= 0.01f) ? 1.0f : 0.0f;
        voices[slot].envAttackInc = (atkMs <= 0.01f)
            ? 1.0f
            : (1.0f / (atkMs * (float)SAMPLE_RATE * 0.001f));
        voices[slot].envDecayCoef = AdDecayCoefFromMs(trkEnvDecayMs[pad]);
        voices[slot].envStage = (atkMs <= 0.01f) ? 1 : 0;
    } else {
        voices[slot].env = 1.0f;
        voices[slot].envAttackInc = 1.0f;
        voices[slot].envDecayCoef = 1.0f;
        voices[slot].envStage = 2;
    }
    voices[slot].age    = voiceAge++;
}

/* ═══════════════════════════════════════════════════════════════════
 *  LOCK-FREE AUDIO COMMAND QUEUE (main loop producer -> AudioCallback consumer)
 *
 * Named explicitly in the original race-condition audit as the primary risk:
 * TriggerPad() and the synth Trigger()/NoteOn()/NoteOff() calls below used to
 * run directly from main-loop command handlers (USB packets, MPD218 MIDI —
 * both funnel through ProcessCommand()) while AudioCallback concurrently
 * reads/advances the SAME voices[]/synth engine state on the audio IRQ.
 * Main-loop code now only ENQUEUES a command; AudioCmdDrainAndApply(),
 * called once at the top of every AudioCallback block, performs the actual
 * state mutation on the audio thread itself — so there is only ever one
 * writer to voices[]/the synth engines for these paths.
 *
 * Single producer (Daisy's bare-metal main() and everything it calls —
 * ProcessDaisyUsb/ProcessCommand/ProcessMpdMidi are all invoked from the
 * same main loop, never concurrently with each other), single consumer
 * (AudioCallback, invoked from the SAI/DMA IRQ). head is written only by
 * the producer, tail only by the consumer; __DMB() orders each slot's
 * payload write/read against the index that publishes/consumes it, so the
 * consumer never observes a partially-written slot.
 *
 * SCOPE: this queue covers every main-loop-reachable call site of
 * TriggerPad, the 808/909/505/303/WTOSC/SH101/FM2OP trigger/note-on paths
 * explicitly named in the audit, and PHYS/NOISE (SetFreq, then SetAccent
 * or SetDensity, then an optional Trig, then the Active flag — a different
 * shape than a plain Trigger()/NoteOn() call, but the same "several writes
 * an ISR can read mid-update" risk) — across CMD_TRIGGER_LIVE,
 * CMD_TRIGGER_SEQ, CMD_BULK_TRIGGERS,
 * CMD_SYNTH_TRIGGER, CMD_SYNTH_NOTE_ON(_EX), and RunPerformanceStressMode
 * (CMD_DIAG_PERF_STRESS can enable it at runtime, unlike the boot
 * self-test). NOT routed through this queue, and left as direct main-loop/
 * audio-thread writes exactly as before this change:
 *   - NoteOff/"release" helpers (ReleaseAllSynthEngines,
 *     ReleaseSynthEngineState, ReleaseTrackEngine, DsqReleaseAllHeldNotes,
 *     DsqReleaseHeldNotes, CMD_SYNTH_ACTIVE), voice-stop helpers
 *     (StopPadVoices, SilenceVoicesInPadRange, the several "voices[v].active
 *     = false" loops in ProcessCommand), and scalar parameter setters
 *     (filter type/cutoff, FX mix, track volume, etc.). These are lower
 *     priority: far less frequent than pad/note triggers, and each is a
 *     single scalar/bool write (or several independent ones) rather than a
 *     multi-field voice/engine being brought up in one specific order, so a
 *     torn read is both rarer and far less audible — worst case a note
 *     stops or a parameter updates one block late, not a corrupted voice.
 *   - RunStartup808SelfTest / RunStartupShowcaseDemo: boot-only demos,
 *     compile-time disabled by default (RED808_STARTUP_808_SELF_TEST=0,
 *     RED808_STARTUP_SHOWCASE_DEMO=0).
 *   - DsqTriggerTrackNow/DsqFireStep: already run on the audio thread
 *     (called from inside AudioCallback), so they call TriggerPad and the
 *     synth engines directly — no queue needed, there is no second writer.
 * ═══════════════════════════════════════════════════════════════════ */
enum AudioCmdType : uint8_t
{
    AUDIO_CMD_NONE = 0,
    AUDIO_CMD_TRIGGER_PAD,
    AUDIO_CMD_SYNTH_TRIGGER,  /* 808/909/505 drum hit */
    AUDIO_CMD_SYNTH_NOTE_ON,  /* 303/WTOSC/SH101/FM2OP melodic note-on */
    AUDIO_CMD_SYNTH_NOTE_OFF, /* matching note-off / panic-all (engine=0xFF) */
    AUDIO_CMD_PHYS_NOISE,     /* PHYS/NOISE: SetFreq+SetAccent/SetDensity+[Trig]+Active */
};

struct AudioCmd
{
    uint8_t  type;
    uint8_t  engine;       /* SYNTH_ENGINE_*; 0xFF = legacy "all melodic off" */
    uint8_t  note;         /* MIDI note (melodic); note-off: 0xFF = unspecified */
    uint8_t  velocity;     /* 0-127 */
    uint8_t  accent;       /* bool, 303 note-on */
    uint8_t  slide;        /* bool, 303 note-on */
    uint8_t  track;        /* note-off: track index for WTOSC, 0xFF = none */
    uint8_t  liveSource;   /* bool, pad-trigger */
    uint8_t  pad;          /* pad-trigger */
    uint8_t  trkVol;       /* pad-trigger */
    int8_t   pan;          /* pad-trigger */
    uint8_t  instrument;   /* drum-trigger native instrument index */
    uint8_t  pianoGate;    /* bool: apply piano-engine auto-select before NOTE_ON
                            * (only CMD_SYNTH_NOTE_ON_EX did this originally —
                            * CMD_SYNTH_TRIGGER/CMD_TRIGGER_LIVE/SEQ/BULK never
                            * did, so they must NOT set this). */
    uint8_t  trig;         /* bool: also call .Trig() (PHYS via NOTE_ON_EX only) */
    uint32_t maxSamples;   /* pad-trigger */
    float    sourceVolume; /* pad-trigger */
    float    sourcePitch;  /* pad-trigger */
};

#define AUDIO_CMD_RING_SIZE 64u /* power of two; ample headroom for one block */
static AudioCmd audioCmdRing[AUDIO_CMD_RING_SIZE];
static volatile uint32_t audioCmdHead = 0; /* producer-owned (main loop) */
static volatile uint32_t audioCmdTail = 0; /* consumer-owned (AudioCallback) */
static volatile uint32_t audioCmdDrops = 0;

/* Producer: main loop only (ProcessCommand and friends). Never call from
 * AudioCallback — this is a single-producer ring. */
static bool AudioCmdPush(const AudioCmd& cmd)
{
    const uint32_t head = audioCmdHead;
    const uint32_t next = (head + 1u) & (AUDIO_CMD_RING_SIZE - 1u);
    if(next == audioCmdTail)
    {
        audioCmdDrops++;
        return false; /* full — not expected at depth 64, see comment above */
    }
    audioCmdRing[head] = cmd;
    __DMB(); /* payload write must be visible before the index publishes it */
    audioCmdHead = next;
    return true;
}

static void AudioCmdApplyTriggerPad(const AudioCmd& c)
{
    TriggerPad(c.pad, c.velocity, c.trkVol, c.pan, c.maxSamples,
              c.sourceVolume, c.sourcePitch, c.liveSource != 0);
}

static void AudioCmdApplySynthTrigger(const AudioCmd& c)
{
    const float vel = c.velocity / 127.0f;
    switch(c.engine)
    {
        case SYNTH_ENGINE_808: synth808.Trigger(c.instrument, vel); break;
        case SYNTH_ENGINE_909: synth909.Trigger(c.instrument, vel); break;
        case SYNTH_ENGINE_505: synth505.Trigger(c.instrument, vel); break;
        default: break;
    }
}

/* Used only by the (production-disabled) kTriggerSynthOnLiveCmd debug path:
 * layers a synth808 hit on top of a sampler pad-trigger. Moved below the
 * AudioCmd queue (was defined near the top of the file, calling
 * synth808.Trigger() straight from the main loop) so it can queue through
 * it instead of racing AudioCallback like the rest of main-loop triggers. */
static inline void Synth808TriggerByPad(uint8_t padIdx, float velocity)
{
    AudioCmd cmd{};
    cmd.type = AUDIO_CMD_SYNTH_TRIGGER;
    cmd.engine = SYNTH_ENGINE_808;
    cmd.instrument = (padIdx < 16) ? padTo808[padIdx] : TR808::INST_KICK;
    cmd.velocity = (uint8_t)clampF(velocity * 127.0f, 0.0f, 127.0f);
    AudioCmdPush(cmd);
}

static void AudioCmdApplyNoteOn(const AudioCmd& c)
{
    /* Mirrors the piano-engine auto-select that used to run inline in
     * CMD_SYNTH_NOTE_ON_EX: keep it glued to the note-on it gates, now both
     * happening on the audio thread instead of split across two threads.
     * Gated on pianoGate — CMD_SYNTH_TRIGGER/CMD_TRIGGER_LIVE/SEQ/BULK also
     * produce AUDIO_CMD_SYNTH_NOTE_ON for 303/WTOSC/SH101/FM2OP, but never
     * did this auto-select originally and must keep not doing it. */
    if(c.pianoGate && IsPianoMelodicEngine(c.engine)
       && c.engine != pianoSelectedEngine)
    {
        ReleaseAllSynthEngines();
        pianoSelectedEngine = c.engine;
    }
    const float vel01 = c.velocity / 127.0f;
    switch(c.engine)
    {
        case SYNTH_ENGINE_303:   acid303.NoteOn(c.note, c.accent != 0, c.slide != 0); break;
        case SYNTH_ENGINE_WTOSC: wtOsc.NoteOn(c.note, vel01); break;
        case SYNTH_ENGINE_SH101: synthSH101.NoteOn(c.note, vel01); break;
        case SYNTH_ENGINE_FM2OP: synthFM2Op.NoteOn(c.note, vel01); break;
        default: break;
    }
}

static void AudioCmdApplyNoteOff(const AudioCmd& c)
{
    switch(c.engine)
    {
        case SYNTH_ENGINE_303: acid303.NoteOff(); break;
        case SYNTH_ENGINE_WTOSC:
            if(c.note != 0xFFu)     wtOsc.NoteOff(c.note);
            else if(c.track < 16)   wtOsc.NoteOff(trackWtNote[c.track]);
            else                    wtOsc.AllNotesOff();
            break;
        case SYNTH_ENGINE_SH101: synthSH101.NoteOff(); break;
        case SYNTH_ENGINE_FM2OP: synthFM2Op.NoteOff(); break;
        case 0xFFu: /* legacy panic: every melodic engine off at once */
            acid303.NoteOff();
            synthSH101.NoteOff();
            synthFM2Op.NoteOff();
            wtOsc.AllNotesOff();
            break;
        default: break;
    }
}

static void AudioCmdApplyPhysNoise(const AudioCmd& c)
{
    /* PHYS/NOISE never had a single Trigger()/NoteOn() call — main-loop code
     * did SetFreq + SetAccent/SetDensity + [Trig] + *Active=true as several
     * separate writes. AudioCallback's Process() reads physModal/physString/
     * noisePart state every sample once *Active is true, so doing those
     * writes from the main loop while a voice is already active carries the
     * same "reader observes an inconsistent mid-update state" risk as
     * TriggerPad — just spread across a few calls instead of one struct. */
    const float freq = 440.f * powf(2.f, (c.note - 69) / 12.f);
    const float vel01 = c.velocity / 127.0f;
    if(c.engine == SYNTH_ENGINE_PHYS)
    {
        if(c.pianoGate && c.engine != pianoSelectedEngine)
        {
            ReleaseAllSynthEngines();
            pianoSelectedEngine = c.engine;
        }
        physModal.SetFreq(freq);
        physString.SetFreq(freq);
        physModal.SetAccent(vel01);
        physString.SetAccent(vel01);
        if(c.trig)
        {
            physModal.Trig();
            physString.Trig();
        }
        physModalActive = true;
        physStringActive = true;
    }
    else if(c.engine == SYNTH_ENGINE_NOISE)
    {
        noisePart.SetFreq(freq);
        noisePart.SetDensity(0.5f + vel01 * 0.5f);
        noisePartActive = true;
    }
}

/* Consumer: AudioCallback only, called once at the top of every block —
 * never from anywhere else (single-consumer ring). */
static void AudioCmdDrainAndApply()
{
    while(audioCmdTail != audioCmdHead)
    {
        __DMB(); /* pairs with the producer's __DMB() before it publishes head */
        const AudioCmd cmd = audioCmdRing[audioCmdTail];
        audioCmdTail = (audioCmdTail + 1u) & (AUDIO_CMD_RING_SIZE - 1u);
        switch(cmd.type)
        {
            case AUDIO_CMD_TRIGGER_PAD:    AudioCmdApplyTriggerPad(cmd);   break;
            case AUDIO_CMD_SYNTH_TRIGGER:  AudioCmdApplySynthTrigger(cmd); break;
            case AUDIO_CMD_SYNTH_NOTE_ON:  AudioCmdApplyNoteOn(cmd);       break;
            case AUDIO_CMD_SYNTH_NOTE_OFF: AudioCmdApplyNoteOff(cmd);      break;
            case AUDIO_CMD_PHYS_NOISE:     AudioCmdApplyPhysNoise(cmd);    break;
            default: break;
        }
    }
}

static uint8_t ActiveVoices(){
    uint8_t c = 0;
    for(int i = 0; i < MAX_VOICES; i++) if(voices[i].active) c++;
    return c;
}

static uint8_t CountLoadedPads()
{
    uint8_t count = 0;
    for(uint8_t i = 0; i < MAX_PADS; i++)
        if(sampleLoaded[i]) count++;
    return count;
}

static void SetPerformanceStressProfile(uint8_t profile);

static void SetPerformanceStressMode(bool enabled)
{
    SetPerformanceStressProfile(enabled ? 2 : 0);
}

static void SetPerformanceStressProfile(uint8_t profile)
{
    perfStressProfile = profile;
    perfStressMode = (profile != 0);
    perfStressNextMs = hw.system.GetNow();
    perfStressStep = 0;
    if(perfStressMode){
        delayActive = true;
        delayTime = 280.0f;
        delayFeedback = 0.42f;
        delayMix = 0.18f;
        masterDelay.SetDelay(delayTime / 1000.0f * (float)SAMPLE_RATE);
        masterDelayR.SetDelay(delayTime / 1000.0f * (float)SAMPLE_RATE);
        reverbActive = true;
        reverbMix = 0.22f;
        chorusActive = true;
        chorusMix = 0.16f;
        flangerActive = true;
        flangerRate = 0.35f;
        flangerDepth = 0.72f;
        flangerFb = 0.34f;
        flangerMix = 0.20f;
        ConfigureMasterFlanger();
        compActive = true;
        limiterActive = true;
    }
}

static void RunPerformanceStressMode(uint32_t nowMs)
{
    if(!perfStressMode || nowMs < perfStressNextMs)
        return;

    perfStressNextMs = nowMs + (kStartupStressReport ? 500u : 70u);
    uint8_t step = perfStressStep++;

    /* CMD_DIAG_PERF_STRESS can enable this at runtime (unlike the boot
     * self-test, which is compile-time disabled) — queue every trigger the
     * same way the live/USB/MIDI paths do, so a stress-test session doesn't
     * reopen the exact race this queue exists to close. */
    if(perfStressProfile >= 2){
        for(uint8_t i = 0; i < 4; i++){
            uint8_t pad = (uint8_t)((step + i * 5u) % MAX_PADS);
            if(sampleLoaded[pad] && !padLoading[pad]){
                AudioCmd cmd{};
                cmd.type = AUDIO_CMD_TRIGGER_PAD;
                cmd.pad = pad;
                cmd.velocity = (uint8_t)(96 + (i * 7));
                cmd.trkVol = 100;
                cmd.sourceVolume = 0.72f;
                cmd.sourcePitch = 1.0f;
                AudioCmdPush(cmd);
            }
        }
    }

    {
        AudioCmd cmd{};
        cmd.type = AUDIO_CMD_SYNTH_TRIGGER;
        cmd.engine = SYNTH_ENGINE_808;
        cmd.instrument = padTo808[step & 15u];
        cmd.velocity = (uint8_t)(0.75f * 127.0f + 0.5f);
        AudioCmdPush(cmd);
    }
    {
        AudioCmd cmd{};
        cmd.type = AUDIO_CMD_SYNTH_TRIGGER;
        cmd.engine = SYNTH_ENGINE_909;
        cmd.instrument = padTo909[(step + 3u) & 15u];
        cmd.velocity = (uint8_t)(0.68f * 127.0f + 0.5f);
        AudioCmdPush(cmd);
    }
    {
        AudioCmd cmd{};
        cmd.type = AUDIO_CMD_SYNTH_TRIGGER;
        cmd.engine = SYNTH_ENGINE_505;
        cmd.instrument = padTo505[(step + 7u) & 15u];
        cmd.velocity = (uint8_t)(0.60f * 127.0f + 0.5f);
        AudioCmdPush(cmd);
    }

    if(!kStartupStressReport){
        AudioCmd cmd303{};
        cmd303.type = AUDIO_CMD_SYNTH_NOTE_ON;
        cmd303.engine = SYNTH_ENGINE_303;
        cmd303.note = (uint8_t)(36 + (step % 24u));
        cmd303.accent = ((step & 3u) == 0) ? 1 : 0;
        cmd303.slide  = ((step & 7u) == 0) ? 1 : 0;
        AudioCmdPush(cmd303);

        AudioCmd cmdWt{};
        cmdWt.type = AUDIO_CMD_SYNTH_NOTE_ON;
        cmdWt.engine = SYNTH_ENGINE_WTOSC;
        cmdWt.note = (uint8_t)(48 + (step % 12u));
        cmdWt.velocity = (uint8_t)(0.45f * 127.0f + 0.5f);
        AudioCmdPush(cmdWt);
    }

    if(!kStartupStressReport && perfStressProfile >= 2){
        AudioCmd cmdSh{};
        cmdSh.type = AUDIO_CMD_SYNTH_NOTE_ON;
        cmdSh.engine = SYNTH_ENGINE_SH101;
        cmdSh.note = (uint8_t)(52 + (step % 12u));
        cmdSh.velocity = (uint8_t)(0.42f * 127.0f + 0.5f);
        AudioCmdPush(cmdSh);

        AudioCmd cmdFm{};
        cmdFm.type = AUDIO_CMD_SYNTH_NOTE_ON;
        cmdFm.engine = SYNTH_ENGINE_FM2OP;
        cmdFm.note = (uint8_t)(60 + (step % 12u));
        cmdFm.velocity = (uint8_t)(0.35f * 127.0f + 0.5f);
        AudioCmdPush(cmdFm);
    }
}

static const char* StartupStressPhaseName(uint8_t phase)
{
    switch(phase){
        case 0: return "baseline";
        case 1: return "synth";
        case 2: return "full";
        case 3: return "cooldown";
        default: return "done";
    }
}

static void PrintStartupStressReport(uint32_t elapsedMs, const char* phase)
{
    if(!kEnableStartLog)
        return;

    uint16_t cpuAvg10 = (uint16_t)(AudioCpuAvgPercent() * 10.0f + 0.5f);
    uint16_t cpuPeak10 = (uint16_t)(AudioCpuPeakPercent() * 10.0f + 0.5f);
    uint16_t masterPeak1000 = (uint16_t)(clampF(masterPeak, 0.0f, 4.0f) * 1000.0f + 0.5f);
    hw.PrintLine("STRESS_REPORT ms=%lu phase=%s cpu_avg=%u.%u cpu_peak=%u.%u voices=%u master_peak=%u.%03u clip=%u spi_err=%u spi_drop=%u loaded=%u",
                 (unsigned long)elapsedMs,
                 phase,
                 (unsigned)(cpuAvg10 / 10u),
                 (unsigned)(cpuAvg10 % 10u),
                 (unsigned)(cpuPeak10 / 10u),
                 (unsigned)(cpuPeak10 % 10u),
                 (unsigned)ActiveVoices(),
                 (unsigned)(masterPeak1000 / 1000u),
                 (unsigned)(masterPeak1000 % 1000u),
                 (unsigned)(masterPeak >= 1.0f ? 1 : 0),
                 (unsigned)spiErrCnt,
                 (unsigned)spiRingDrops,
                 (unsigned)CountLoadedPads());
}

#if RED808_DSP_BLOCK_PROFILE
static void PrintDspProfileReport(uint32_t elapsedMs, const char* phase)
{
    if(!kEnableStartLog)
        return;

    DspProfSnapshot snap[DSP_PROF_COUNT];
    uint32_t blocks = 0;
    DspProfSnapshotAndReset(snap, &blocks);
    if(blocks == 0)
        return;

    const float totalBudget = kDspProfBlockBudgetCycles * (float)blocks;
    for(uint8_t i = 0; i < DSP_PROF_COUNT; i++){
        if(snap[i].calls == 0)
            continue;
        float pct = totalBudget > 0.0f ? ((float)snap[i].cycles * 100.0f / totalBudget) : 0.0f;
        float avgCycles = (float)snap[i].cycles / (float)snap[i].calls;
        uint32_t pct10 = (uint32_t)(pct * 10.0f + 0.5f);
        uint32_t avg = (uint32_t)(avgCycles + 0.5f);
        uint32_t peak = snap[i].maxCycles;
        hw.PrintLine("DSP_PROFILE ms=%lu phase=%s block=%s pct=%lu.%lu avg_cycles=%lu peak_cycles=%lu calls=%lu audio_blocks=%lu",
                     (unsigned long)elapsedMs,
                     phase,
                     DspProfName(i),
                     (unsigned long)(pct10 / 10u),
                     (unsigned long)(pct10 % 10u),
                     (unsigned long)avg,
                     (unsigned long)peak,
                     (unsigned long)snap[i].calls,
                     (unsigned long)blocks);
    }
}
#else
static inline void PrintDspProfileReport(uint32_t, const char*) {}
#endif

static void BeginStartupStressReport(uint32_t nowMs)
{
    if(!kStartupStressReport || startupStressReportDone)
        return;
    audioLoadMeter.Reset();
#if RED808_DSP_BLOCK_PROFILE
    DspProfSnapshot discard[DSP_PROF_COUNT];
    uint32_t discardBlocks = 0;
    DspProfSnapshotAndReset(discard, &discardBlocks);
#endif
    masterPeak = 0.0f;
    spiErrCnt = 0;
    spiRingDrops = 0;
    startupStressReportActive = true;
    startupStressStartMs = nowMs + kStartupStressArmDelayMs;
    startupStressLastReportMs = 0;
    startupStressPhase = 255;
    if(kEnableStartLog)
        hw.PrintLine("STRESS_REPORT_BEGIN seconds=%lu profiles=baseline,synth,full,cooldown audio_out=not_required",
                     (unsigned long)kStartupStressSeconds);
}

static void RunStartupStressReport(uint32_t nowMs)
{
    if(!startupStressReportActive)
        return;

    if((int32_t)(nowMs - startupStressStartMs) < 0)
        return;

    uint32_t elapsed = nowMs - startupStressStartMs;
    uint32_t totalMs = kStartupStressSeconds * 1000u;
    if(totalMs < 8000u)
        totalMs = 8000u;

    uint8_t phase = 0;
    uint8_t profile = 0;
    if(elapsed < 2000u){
        phase = 0;
        profile = 0;
    } else if(elapsed < (totalMs / 2u)){
        phase = 1;
        profile = 1;
    } else if(elapsed < (totalMs - 2000u)){
        phase = 2;
        profile = 2;
    } else if(elapsed < totalMs){
        phase = 3;
        profile = 0;
    } else {
        SetPerformanceStressProfile(0);
        PrintStartupStressReport(elapsed, "done");
        if(kEnableStartLog)
        {
            uint16_t cpuAvg10 = (uint16_t)(AudioCpuAvgPercent() * 10.0f + 0.5f);
            uint16_t cpuPeak10 = (uint16_t)(AudioCpuPeakPercent() * 10.0f + 0.5f);
            uint16_t masterPeak1000 = (uint16_t)(clampF(masterPeak, 0.0f, 4.0f) * 1000.0f + 0.5f);
            hw.PrintLine("STRESS_REPORT_END cpu_avg=%u.%u cpu_peak=%u.%u voices=%u master_peak=%u.%03u spi_err=%u spi_drop=%u",
                         (unsigned)(cpuAvg10 / 10u),
                         (unsigned)(cpuAvg10 % 10u),
                         (unsigned)(cpuPeak10 / 10u),
                         (unsigned)(cpuPeak10 % 10u),
                         (unsigned)ActiveVoices(),
                         (unsigned)(masterPeak1000 / 1000u),
                         (unsigned)(masterPeak1000 % 1000u),
                         (unsigned)spiErrCnt,
                         (unsigned)spiRingDrops);
        }
        startupStressReportActive = false;
        startupStressReportDone = true;
        return;
    }

    if(phase != startupStressPhase){
        startupStressPhase = phase;
        SetPerformanceStressProfile(profile);
        if(kEnableStartLog)
            hw.PrintLine("STRESS_PHASE ms=%lu phase=%s profile=%u",
                         (unsigned long)elapsed,
                         StartupStressPhaseName(phase),
                         (unsigned)profile);
    }

    if(startupStressLastReportMs == 0 || (nowMs - startupStressLastReportMs) >= 1000u){
        startupStressLastReportMs = nowMs;
        PrintStartupStressReport(elapsed, StartupStressPhaseName(phase));
        PrintDspProfileReport(elapsed, StartupStressPhaseName(phase));
    }
}

/* ── Reporte periódico del profiler DSP durante uso normal ──
 * Solo activo con RED808_DSP_BLOCK_PROFILE=1; coste cero en producción.
 * Emite por serial cada kDspLiveProfileMs los ciclos/% por engine y FX,
 * medidos con los patrones reales (a diferencia del stress de arranque,
 * que usa patrones sintéticos). Util para decidir qué optimizar. */
#if RED808_DSP_BLOCK_PROFILE
static constexpr uint32_t kDspLiveProfileMs = 3000;
static uint32_t liveDspProfileNextMs = 0;
static void RunLiveDspProfileReport(uint32_t nowMs)
{
    if(!kEnableStartLog)          return;
    if(startupStressReportActive) return;  /* no pisar el stress de arranque */
    if((int32_t)(nowMs - liveDspProfileNextMs) < 0) return;
    liveDspProfileNextMs = nowMs + kDspLiveProfileMs;
    PrintDspProfileReport(nowMs, "live");
}
#else
static inline void RunLiveDspProfileReport(uint32_t) {}
#endif

static void SilenceVoicesInPadRange(uint8_t startPad, uint8_t endPad)
{
    if(endPad > MAX_PADS)
        endPad = MAX_PADS;
    for(int voiceIndex = 0; voiceIndex < MAX_VOICES; voiceIndex++)
    {
        if(!voices[voiceIndex].active)
            continue;
        uint8_t pad = voices[voiceIndex].pad;
        if(pad >= startPad && pad < endPad)
            voices[voiceIndex].active = false;
    }
}

static void ResetTrackRuntimeState(uint8_t track)
{
    if(track >= MAX_PADS)
        return;

    /* Clear per-track FX configuration — prevents stale filter/effects
     * from persisting after kit reload or track clear                  */
    trkFilterType[track] = 0;
    trkFilterCut[track]  = 1000.0f;
    trkFilterQ[track]    = 0.707f;
    trkFilterCutSm[track] = trkFilterCut[track];
    trkFilterQSm[track]   = trkFilterQ[track];
    trkFilter[track].Reset();
    trkFilter2[track].Reset();
    trkDistDrive[track]  = 0.0f;
    trkDistMode[track]   = DMODE_SOFT;
    trkBitDepth[track]   = 16;
    trkEchoActive[track] = false;
    trkEchoWp[track]     = 0;
    trkFlgActive[track]  = false;
    trkFlanger[track].Init((float)SAMPLE_RATE);
    ConfigureTrackFlanger((uint8_t)track);
    trkCompActive[track] = false;
    trkCompEnv[track]    = 0.0f;
    trackPeak[track]     = 0.0f;
    trkFxRouted[track]   = false;
    memset(trkEchoBuf[track], 0, sizeof(trkEchoBuf[track]));
}

static void PreparePadRangeForReload(uint8_t startPad, uint8_t endPad)
{
    if(endPad > MAX_PADS)
        endPad = MAX_PADS;

    /* 1. Mark ALL pads loading FIRST — AudioCallback will skip them entirely.
     *    This MUST happen before SilenceVoicesInPadRange / ResetTrackRuntimeState
     *    to prevent the ISR from re-triggering voices or reading filter state
     *    while we modify it (data race).                                       */
    for(uint8_t pad = startPad; pad < endPad; pad++)
        padLoading[pad] = true;

    /* 2. Now safe to kill active voices and reset track state */
    SilenceVoicesInPadRange(startPad, endPad);
    for(uint8_t pad = startPad; pad < endPad; pad++)
    {
        sampleLoaded[pad] = false;
        sampleLength[pad] = 0;
        sampleTotalSamples[pad] = 0;
        ResetTrackRuntimeState(pad);
    }
}

static void DsqReleaseHeldNotes(uint8_t track)
{
    if(track >= DSQ_TRACKS || !dsqHeldNotes[track].active) return;
    DsqHeldNotes& held = dsqHeldNotes[track];
    switch(held.engine){
        case SYNTH_ENGINE_303:   acid303.NoteOff(); break;
        case SYNTH_ENGINE_WTOSC:
            for(uint8_t v = 0; v < 4; v++) if(held.notes[v]) wtOsc.NoteOff(held.notes[v]);
            break;
        case SYNTH_ENGINE_SH101: synthSH101.NoteOff(); break;
        case SYNTH_ENGINE_FM2OP: synthFM2Op.NoteOff(); break;
        case SYNTH_ENGINE_PHYS:
            physModalActive = false; physStringActive = false; break;
        case SYNTH_ENGINE_NOISE: noisePartActive = false; break;
        default: break;
    }
    memset(&held, 0, sizeof(held));
}

/* Dedicated presentation program. It lives in the audio sequencer instead of
 * running a second millisecond clock in main(), so every hit is sample-accurate
 * and the Master cannot play a second arrangement over it. The musical design
 * is one coherent 32-bar club piece: sample-first drums, two restrained machine
 * layers and two melodic voices. */
enum ShowcaseTrack : uint8_t {
    SHOW_BD = 0, SHOW_SD, SHOW_CH, SHOW_OH, SHOW_CY, SHOW_CP, SHOW_RS, SHOW_CB,
    SHOW_LT, SHOW_MT, SHOW_HT, SHOW_MA, SHOW_CL, SHOW_HC, SHOW_BASS, SHOW_PAD
};

static constexpr uint8_t SHOWCASE_SCENES = 8;

static void ShowcaseHit(uint8_t pattern, uint8_t track, uint8_t step,
                        uint8_t velocity, uint8_t probability = 100,
                        uint8_t ratchet = 1, uint8_t noteLenDiv = 1)
{
    if(pattern >= SHOWCASE_SCENES || track >= DSQ_TRACKS || step >= DSQ_MAX_STEPS) return;
    DsqStepFull& s = dsqSteps[pattern][track][step];
    memset(&s, 0, sizeof(s));
    s.active = 1;
    s.velocity = velocity;
    s.noteLenDiv = noteLenDiv ? noteLenDiv : 1;
    s.probability = probability;
    s.ratchet = (ratchet >= 1 && ratchet <= 4) ? ratchet : 1;
}

static void ShowcaseNote(uint8_t pattern, uint8_t track, uint8_t step,
                         uint8_t note, uint8_t velocity, uint8_t probability = 100)
{
    ShowcaseHit(pattern, track, step, velocity, probability, 1, 1);
    dsqSteps[pattern][track][step].notes[0] = note;
}

static void ShowcaseChord(uint8_t pattern, uint8_t step,
                          uint8_t root, uint8_t third, uint8_t fifth, uint8_t velocity)
{
    ShowcaseHit(pattern, SHOW_PAD, step, velocity, 100, 1, 1);
    DsqStepFull& s = dsqSteps[pattern][SHOW_PAD][step];
    s.notes[0] = root; s.notes[1] = third; s.notes[2] = fifth;
}

static void ShowcaseBackbeat(uint8_t pattern, uint8_t base, uint8_t velocity)
{
    ShowcaseHit(pattern, SHOW_SD, base + 4, velocity);
    ShowcaseHit(pattern, SHOW_SD, base + 12, velocity + 3);
    ShowcaseHit(pattern, SHOW_CP, base + 12, velocity > 18 ? velocity - 18 : velocity, 92);
}

static void ShowcaseFourFloor(uint8_t pattern, uint8_t base, uint8_t velocity)
{
    ShowcaseHit(pattern, SHOW_BD, base + 0, velocity);
    ShowcaseHit(pattern, SHOW_BD, base + 4, velocity > 5 ? velocity - 5 : velocity);
    ShowcaseHit(pattern, SHOW_BD, base + 8, velocity > 3 ? velocity - 3 : velocity);
    ShowcaseHit(pattern, SHOW_BD, base + 12, velocity > 7 ? velocity - 7 : velocity);
}

static void ShowcaseSoftHats(uint8_t pattern, uint8_t base, uint8_t velocity)
{
    for(uint8_t st = 2; st < 16; st += 4)
        ShowcaseHit(pattern, SHOW_CH, base + st, (uint8_t)(velocity + ((st == 14) ? 7 : 0)), 96);
}

static __attribute__((noinline, optimize("O1"))) void BuildStartupShowcaseProgram()
{
    if(!kStartupShowcaseDemo) return;
    memset(dsqSteps, 0, sizeof(dsqSteps));
    memset(dsqTrackEngine, (uint8_t)0xFF, sizeof(dsqTrackEngine));

    /* Samples remain the body of the kit. CY and CB are quiet 909/808 colour;
     * the last two tracks are a mono bass and a soft wavetable pad. */
    dsqTrackEngine[SHOW_CY]   = SYNTH_ENGINE_909;
    dsqTrackEngine[SHOW_CB]   = SYNTH_ENGINE_808;
    dsqTrackEngine[SHOW_BASS] = SYNTH_ENGINE_SH101;
    dsqTrackEngine[SHOW_PAD]  = SYNTH_ENGINE_WTOSC;
    for(uint8_t track = 0; track < CLEAN_TRACK_COUNT; track++){
        cleanTrackEnabled[track] = false;
        cleanTrackActive[track] = false;
        cleanTrackMuted[track] = true;
        cleanTrackPlayhead[track] = 0;
    }

    static const uint8_t chordRoot[4]  = {53, 49, 51, 48}; /* Fm, Db, Eb, Cm */
    static const uint8_t chordThird[4] = {56, 53, 55, 51};
    static const uint8_t chordFifth[4] = {60, 56, 58, 55};

    for(uint8_t bar = 0; bar < 4; bar++){
        const uint8_t b = bar * 16;

        /* 0 · AIR — establish room, samples and harmony before the pulse. */
        ShowcaseSoftHats(0, b, 38);
        ShowcaseHit(0, SHOW_RS, b + 4, 44, 86);
        ShowcaseHit(0, SHOW_CP, b + 12, 54, 90);
        ShowcaseChord(0, b, chordRoot[bar], chordThird[bar], chordFifth[bar], 42);
        if(bar >= 2){
            ShowcaseHit(0, SHOW_BD, b, 96);
            ShowcaseHit(0, SHOW_BD, b + 10, 72, 88);
            ShowcaseNote(0, SHOW_BASS, b, bar == 2 ? 29 : 27, 64);
        }
        if(bar == 0) ShowcaseHit(0, SHOW_CY, 0, 42);

        /* 1 · PULSE — warm four-floor sample pocket, no wall of synths. */
        ShowcaseFourFloor(1, b, 116);
        ShowcaseBackbeat(1, b, 104);
        ShowcaseSoftHats(1, b, 54);
        ShowcaseHit(1, SHOW_OH, b + 6, 60, 88);
        ShowcaseHit(1, SHOW_OH, b + 14, 68, 94);
        ShowcaseNote(1, SHOW_BASS, b + 0, 29, 76);
        ShowcaseNote(1, SHOW_BASS, b + 7, 36, 62, 86);
        ShowcaseNote(1, SHOW_BASS, b + 10, bar & 1u ? 39 : 32, 70);

        /* 2 · HOOK — broken club response with deliberate empty sixteenths. */
        const uint8_t k2a[3] = {0, 6, 10};
        const uint8_t k2b[4] = {0, 7, 11, 14};
        const uint8_t* k2 = (bar & 1u) ? k2b : k2a;
        const uint8_t k2n = (bar & 1u) ? 4 : 3;
        for(uint8_t i = 0; i < k2n; i++) ShowcaseHit(2, SHOW_BD, b + k2[i], i ? 91 : 118);
        ShowcaseBackbeat(2, b, 108);
        const uint8_t h2[6] = {1, 3, 6, 9, 11, 14};
        for(uint8_t i = 0; i < 6; i++) ShowcaseHit(2, SHOW_CH, b + h2[i], (uint8_t)(43 + (i & 1u) * 12), 94);
        ShowcaseHit(2, SHOW_OH, b + 15, 63, 82);
        if((bar & 1u) == 0) ShowcaseHit(2, SHOW_CB, b + 11, 38, 78);
        ShowcaseNote(2, SHOW_BASS, b + 0, 29, 78);
        ShowcaseNote(2, SHOW_BASS, b + 6, 36, 66);
        ShowcaseNote(2, SHOW_BASS, b + 11, 32, 72, 90);

        /* 3 · PRESSURE — same identity, denser hats and restrained machines. */
        ShowcaseFourFloor(3, b, 121);
        ShowcaseBackbeat(3, b, 110);
        for(uint8_t st = 1; st < 16; st += 2)
            ShowcaseHit(3, SHOW_CH, b + st, (uint8_t)(42 + ((st & 3u) == 3u ? 13 : 0)), 97);
        ShowcaseHit(3, SHOW_OH, b + 6, 66);
        ShowcaseHit(3, SHOW_OH, b + 14, 74);
        if(bar == 0) ShowcaseHit(3, SHOW_CY, b, 48);
        if(bar == 3) ShowcaseHit(3, SHOW_CB, b + 14, 42, 86, 2);
        ShowcaseNote(3, SHOW_BASS, b + 0, 29, 82);
        ShowcaseNote(3, SHOW_BASS, b + 10, (bar & 1u) ? 27 : 32, 68);

        /* 4 · SPACE — two bars without kick, then a low broken re-entry. */
        ShowcaseChord(4, b, chordRoot[bar], chordThird[bar], chordFifth[bar], 48);
        ShowcaseHit(4, SHOW_RS, b + 3, 42, 78);
        ShowcaseHit(4, SHOW_CP, b + 12, 60, 92);
        ShowcaseHit(4, SHOW_MA, b + 6, 40, 72);
        ShowcaseHit(4, SHOW_HC, b + 14, 46, 76);
        if(bar >= 2){
            ShowcaseHit(4, SHOW_BD, b, 102);
            ShowcaseHit(4, SHOW_BD, b + 10, 82);
            ShowcaseNote(4, SHOW_BASS, b, bar == 2 ? 29 : 27, 66);
        }

        /* 5 · RETURN — broken first half resolves into the club pulse. */
        if(bar < 2){
            ShowcaseHit(5, SHOW_BD, b, 118);
            ShowcaseHit(5, SHOW_BD, b + 6, 92);
            ShowcaseHit(5, SHOW_BD, b + 10, 102);
        } else {
            ShowcaseFourFloor(5, b, 119);
        }
        ShowcaseBackbeat(5, b, 109);
        ShowcaseSoftHats(5, b, 57);
        ShowcaseHit(5, SHOW_OH, b + 14, 70);
        ShowcaseNote(5, SHOW_BASS, b, 29, 80);
        ShowcaseNote(5, SHOW_BASS, b + 6, 36, 64);
        ShowcaseNote(5, SHOW_BASS, b + 10, (bar & 1u) ? 39 : 32, 74);

        /* 6 · PEAK — sample transients stay in front; machines only highlight. */
        ShowcaseFourFloor(6, b, 123);
        ShowcaseBackbeat(6, b, 114);
        for(uint8_t st = 1; st < 16; st += 2)
            ShowcaseHit(6, SHOW_CH, b + st, (uint8_t)(47 + ((st & 3u) == 3u ? 14 : 0)), 98);
        ShowcaseHit(6, SHOW_OH, b + 6, 70);
        ShowcaseHit(6, SHOW_OH, b + 14, 78);
        if(bar == 0 || bar == 2) ShowcaseHit(6, SHOW_CY, b, 52);
        if(bar == 1 || bar == 3) ShowcaseHit(6, SHOW_CB, b + 11, 43, 84);
        ShowcaseNote(6, SHOW_BASS, b, 29, 84);
        ShowcaseNote(6, SHOW_BASS, b + 7, 36, 67);
        ShowcaseNote(6, SHOW_BASS, b + 10, bar & 1u ? 39 : 32, 76);
        if(bar == 0) ShowcaseChord(6, b, 53, 56, 60, 38);

        /* 7 · RELEASE — subtract, answer with a sample fill, return to AIR. */
        if(bar < 2) ShowcaseFourFloor(7, b, 116);
        else {
            ShowcaseHit(7, SHOW_BD, b, 108);
            if(bar == 2) ShowcaseHit(7, SHOW_BD, b + 10, 78);
        }
        ShowcaseBackbeat(7, b, bar < 2 ? 105 : 88);
        ShowcaseSoftHats(7, b, bar < 2 ? 50 : 38);
        if(bar < 3) ShowcaseNote(7, SHOW_BASS, b, bar & 1u ? 27 : 29, 66);
        if(bar == 2) ShowcaseChord(7, b, 49, 53, 56, 43);
        if(bar == 3){
            ShowcaseHit(7, SHOW_LT, b + 12, 68);
            ShowcaseHit(7, SHOW_MT, b + 13, 76);
            ShowcaseHit(7, SHOW_HT, b + 14, 84);
            ShowcaseHit(7, SHOW_SD, b + 15, 92, 100, 2);
        }
    }

    ApplySynthPreset(SYNTH_ENGINE_808, 1);
    ApplySynthPreset(SYNTH_ENGINE_909, 1);
    ApplySynthPreset(SYNTH_ENGINE_SH101, 2);
    ApplySynthPreset(SYNTH_ENGINE_WTOSC, 3);

    for(uint8_t i = 0; i < SHOWCASE_SCENES; i++){
        songChain[i].pattern = i;
        songChain[i].repeats = 1;
    }
    songLength = SHOWCASE_SCENES;
    songPlaying = false;
    songIdx = 0;
    songRepeatCnt = 0;
    dseq.currentPattern = 0;
    dseq.patternLength = 64;
    dseq.currentStep = -1;
    dseq.tempo = 126.0f;
    dseq.swingAmount = 8;
    dseq.humanizeTimingMs = 0;
    dseq.humanizeVelAmt = 2;
    DsqUpdateSamplesPerStep();
}

static bool ShowcaseBlocksMasterCommand(uint8_t cmd)
{
    if(!kStartupShowcaseDemo) return false;
    if(cmd >= CMD_DSQ_UPLOAD_TRACK && cmd <= CMD_DSQ_SET_STEP_NOTES && cmd != CMD_DSQ_GET_POS)
        return true;
    if(cmd >= CMD_SYNTH_TRIGGER && cmd <= CMD_SYNTH_NOTE_ON_EX)
        return true;
    switch(cmd){
        case CMD_TRIGGER_SEQ:
        case CMD_TRIGGER_LIVE:
        case CMD_TRIGGER_STOP:
        case CMD_TRIGGER_STOP_ALL:
        case CMD_TRIGGER_SIDECHAIN:
        case CMD_BULK_TRIGGERS:
        case CMD_SONG_UPLOAD:
        case CMD_SONG_CONTROL:
        case CMD_RESET:
            return true;
        default:
            return false;
    }
}

static void RunStartupShowcaseDemo(uint32_t nowMs)
{
    if(!kStartupShowcaseDemo) return;
    static bool started = false;
    if(started) return;

    uint8_t loaded = 0;
    for(uint8_t track = 0; track < 14; track++)
        if(sampleLoaded[track] && !padLoading[track]) loaded++;

    /* With a Master attached, wait for its SD kit load. Standalone Showcase
     * falls back quickly to the generated machines instead of staying silent. */
    const bool masterPresent = spiPktCnt > 0;
    if(nowMs < 2200u || kitMuteActive) return;
    if(masterPresent && loaded < 8 && nowMs < 20000u) return;

    for(uint8_t track = 0; track < 14; track++){
        if(dsqTrackEngine[track] != -1 || sampleLoaded[track]) continue;
        dsqTrackEngine[track] = (track <= SHOW_CY) ? SYNTH_ENGINE_909 : SYNTH_ENGINE_505;
    }

    /* Conservative presentation mix: samples lead, machine layers are colour,
     * and ambience remains behind the transient instead of washing it out. */
    for(uint8_t track = 0; track < MAX_PADS; track++){
        trackGain[track] = 0.72f;
        trackPanF[track] = 0.0f;
        trackReverbSend[track] = 0.03f;
        trackDelaySend[track] = 0.0f;
        trackChorusSend[track] = 0.0f;
        trackMute[track] = false;
        trackSolo[track] = false;
    }
    trackGain[SHOW_BD] = 0.92f; trackGain[SHOW_SD] = 0.78f;
    trackGain[SHOW_CH] = 0.52f; trackGain[SHOW_OH] = 0.56f;
    trackGain[SHOW_CY] = 0.38f; trackGain[SHOW_CP] = 0.68f;
    trackGain[SHOW_RS] = 0.58f; trackGain[SHOW_CB] = 0.34f;
    trackGain[SHOW_BASS] = 0.50f; trackGain[SHOW_PAD] = 0.30f;
    trackPanF[SHOW_CH] = 0.10f; trackPanF[SHOW_OH] = 0.24f;
    trackPanF[SHOW_CP] = -0.10f; trackPanF[SHOW_RS] = 0.15f;
    trackPanF[SHOW_LT] = -0.28f; trackPanF[SHOW_HT] = 0.28f;
    trackReverbSend[SHOW_SD] = 0.10f; trackReverbSend[SHOW_CP] = 0.16f;
    trackReverbSend[SHOW_PAD] = 0.25f; trackDelaySend[SHOW_BASS] = 0.06f;

    masterGain = 0.88f;
    seqVolume = 0.92f;
    limiterActive = true;
    compActive = true;
    reverbActive = true;
    delayActive = true;
    chorusActive = true;
    flangerActive = false;
    phaserActive = false;
    reverbMix = 0.11f;
    delayMix = 0.07f;
    delayFeedback = 0.24f;
    chorusMix = 0.035f;
    masterDelay.SetDelay(0.1875f * (float)SAMPLE_RATE);
    masterDelayR.SetDelay(0.1875f * (float)SAMPLE_RATE);
    masterReverb.SetFeedback(0.76f);
    masterReverb.SetLpFreq(6800.0f);

    anySolo = false;
    dseq.currentPattern = songChain[0].pattern;
    dseq.currentStep = -1;
    dseq.samplesElapsed = 0;
    songIdx = 0;
    songRepeatCnt = 0;
    songPlaying = true;
    dseq.playing = true;
    started = true;
}

static void DsqReleaseAllHeldNotes()
{
    for(uint8_t track = 0; track < DSQ_TRACKS; track++) DsqReleaseHeldNotes(track);
    memset(pendingTriggers, 0, sizeof(pendingTriggers));
}

static void DsqTriggerTrackNow(uint8_t track, DsqStepFull& s, uint8_t velocity)
{
    if(track >= DSQ_TRACKS || dseq.trackMuted[track]) return;
    int8_t eng = dsqTrackEngine[track];
    bool isSynth = (eng >= 0 && eng < SYNTH_ENGINE_COUNT);
    if(!isSynth && (!sampleLoaded[track] || padLoading[track])) return;

    const uint8_t div = s.noteLenDiv ? s.noteLenDiv : 1;
    const float vel = velocity / 127.0f;
    if(!isSynth){
        uint32_t maxS = (div > 1) ? (dseq.samplesPerStep / div) : 0;
        if(s.cutoffEn && trkFilterType[track] && trkFxRouted[track]){
            /* Per-step parameter lock: a deliberately instant stab, not a
             * sweep — snap the smoothing shadow too so
             * UpdateTrackFilterSmoothing() doesn't blur it into a glide. */
            float f = clampF((float)s.cutoffHz, 20.f, 20000.f);
            trkFilter[track].SetType(trkFilterType[track], f, trkFilterQ[track], (float)SAMPLE_RATE);
            if(trkFilterType[track] == FTYPE_RESONANT)
                trkFilter2[track].SetType(FTYPE_RESONANT, f, trkFilterQ[track], (float)SAMPLE_RATE);
            trkFilterCut[track] = f;
            trkFilterCutSm[track] = f;
        }
        if(s.reverbEn) trackReverbSend[track] = clampF(s.reverbSend / 100.0f, 0.f, 1.f);
        if(s.volEn) trackGain[track] = VolumeByteToGain(s.volume);
        TriggerPad(track, velocity, 100, 0, maxS, seqVolume);
        return;
    }

    if(eng == SYNTH_ENGINE_808){ synth808.Trigger(padTo808[track], vel); return; }
    if(eng == SYNTH_ENGINE_909){ synth909.Trigger(padTo909[track], vel); return; }
    if(eng == SYNTH_ENGINE_505){ synth505.Trigger(padTo505[track], vel); return; }

    uint8_t notes[4] = {s.notes[0], s.notes[1], s.notes[2], s.notes[3]};
    bool hasNote = false;
    for(uint8_t v = 0; v < 4; v++) if(notes[v]) { hasNote = true; break; }
    if(!hasNote){
        if(eng == SYNTH_ENGINE_303) notes[0] = padTo303Midi[track];
        else if(eng == SYNTH_ENGINE_SH101) notes[0] = trackSH101Note[track];
        else if(eng == SYNTH_ENGINE_FM2OP) notes[0] = trackFM2OpNote[track];
        else notes[0] = trackWtNote[track];
    }

    const bool accent = (s.flags & 0x01) || velocity >= 112;
    const bool slide = (s.flags & 0x02) != 0;
    if(!(eng == SYNTH_ENGINE_303 && slide)) DsqReleaseHeldNotes(track);

    switch(eng){
        case SYNTH_ENGINE_303:
            acid303.NoteOn(notes[0], accent, slide);
            break;
        case SYNTH_ENGINE_WTOSC:
            for(uint8_t v = 0; v < 4; v++) if(notes[v]) wtOsc.NoteOn(notes[v], vel);
            break;
        case SYNTH_ENGINE_SH101:
            synthSH101.NoteOn(notes[0], vel);
            notes[1] = notes[2] = notes[3] = 0;
            break;
        case SYNTH_ENGINE_FM2OP:
            synthFM2Op.NoteOn(notes[0], vel);
            notes[1] = notes[2] = notes[3] = 0;
            break;
        case SYNTH_ENGINE_PHYS: {
            float freq = 440.f * powf(2.f, (notes[0] - 69) / 12.f);
            physModal.SetFreq(freq); physString.SetFreq(freq);
            physModal.SetAccent(vel); physString.SetAccent(vel);
            physModal.Trig(); physString.Trig();
            physModalActive = true; physStringActive = true;
            notes[1] = notes[2] = notes[3] = 0;
            break;
        }
        case SYNTH_ENGINE_NOISE: {
            float freq = 440.f * powf(2.f, (notes[0] - 69) / 12.f);
            noisePart.SetFreq(freq); noisePart.SetDensity(0.5f + vel * 0.5f);
            noisePartActive = true;
            notes[1] = notes[2] = notes[3] = 0;
            break;
        }
        default: return;
    }

    DsqHeldNotes& held = dsqHeldNotes[track];
    held.active = true;
    held.engine = eng;
    memcpy(held.notes, notes, sizeof(held.notes));
    uint32_t gate = dseq.samplesPerStep / div;
    if(slide) gate = dseq.samplesPerStep + 2;
    held.samplesRemaining = gate > 8 ? gate : 8;
}

static void DsqProcessPendingTriggers()
{
    for(uint8_t track = 0; track < DSQ_TRACKS; track++){
        PendingTrigger& pending = pendingTriggers[track];
        if(!pending.active) continue;
        if(pending.countdown > 0){ pending.countdown--; continue; }
        DsqStepFull& s = dsqSteps[pending.pattern][track][pending.step];
        DsqTriggerTrackNow(track, s, pending.velocity);
        if(pending.repeatsRemaining > 0) pending.repeatsRemaining--;
        if(pending.repeatsRemaining == 0){ pending.active = false; continue; }
        pending.velocity = (uint8_t)((pending.velocity * 88u) / 100u);
        if(pending.velocity == 0) pending.velocity = 1;
        pending.countdown = pending.interval;
    }
}

static void DsqProcessHeldNotes()
{
    for(uint8_t track = 0; track < DSQ_TRACKS; track++){
        DsqHeldNotes& held = dsqHeldNotes[track];
        if(!held.active) continue;
        if(held.samplesRemaining > 0) held.samplesRemaining--;
        if(held.samplesRemaining == 0) DsqReleaseHeldNotes(track);
    }
}

/* Fire/schedule the current step. Probability is evaluated once per step;
 * ratchets and timing humanization then remain sample-accurate. */
static void DsqFireStep()
{
    const uint8_t pat = dseq.currentPattern;
    const uint8_t slen = dseq.patternLength;
    const uint8_t step = (uint8_t)((dseq.currentStep % (int)slen + (int)slen) % (int)slen);
    for(uint8_t track = 0; track < DSQ_TRACKS; track++){
        pendingTriggers[track].active = false;
        if(dseq.trackMuted[track]) continue;
        DsqStepFull& s = dsqSteps[pat][track][step];
        if(!s.active || s.velocity == 0 || s.probability == 0) continue;
        if(s.probability < 100 && (uint8_t)(FastRand() % 100) >= s.probability) continue;

        int velocity = s.velocity;
        if(dseq.humanizeVelAmt > 0){
            int range = (velocity * dseq.humanizeVelAmt) / 100;
            int span = range * 2 + 1;
            velocity += span > 1 ? (int)(FastRand() % (uint32_t)span) - range : 0;
            if(velocity < 1) velocity = 1;
            if(velocity > 127) velocity = 127;
        }

        uint32_t delay = 0;
        if(dseq.humanizeTimingMs > 0){
            uint32_t maxDelay = (uint32_t)dseq.humanizeTimingMs * SAMPLE_RATE / 1000u;
            if(track <= 1) maxDelay /= 4u; /* kick/snare stay anchored */
            if(maxDelay > 0) delay = FastRand() % (maxDelay + 1u);
        }
        const uint8_t ratchets = (s.ratchet >= 1 && s.ratchet <= 4) ? s.ratchet : 1;
        PendingTrigger& pending = pendingTriggers[track];
        pending.active = true;
        pending.pattern = pat;
        pending.track = track;
        pending.step = step;
        pending.velocity = (uint8_t)velocity;
        pending.repeatsRemaining = ratchets;
        pending.interval = dseq.samplesPerStep / ratchets;
        if(pending.interval < 8) pending.interval = 8;
        pending.countdown = delay;
        if(delay == 0){
            DsqTriggerTrackNow(track, s, pending.velocity);
            pending.repeatsRemaining--;
            if(pending.repeatsRemaining == 0) pending.active = false;
            else {
                pending.velocity = (uint8_t)((pending.velocity * 88u) / 100u);
                if(pending.velocity == 0) pending.velocity = 1;
                pending.countdown = pending.interval;
            }
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════
 *  21. AUDIO CALLBACK
 * ═══════════════════════════════════════════════════════════════════ */
void AudioCallback(AudioHandle::InputBuffer  /*in*/,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    audioLoadMeter.OnBlockStart();
    DSP_PROF_SCOPE(CALLBACK);

    /* Enforce FZ+DN in ISR context (belt-and-suspenders for FPDSCR) */
    __asm volatile("VMRS r0, FPSCR\n"
                   "ORR  r0, r0, #(1<<24)|(1<<25)\n"
                   "VMSR FPSCR, r0" ::: "r0");

    /* Apply every pad/synth trigger queued by the main loop since the last
     * block, on THIS thread, before anything else touches voices[] or the
     * synth engines this callback. Must run even on the early-exit paths
     * below (tone test / safe mode / kit mute) so queued commands are
     * never starved while one of those states is active. */
    AudioCmdDrainAndApply();

    /* ── STARTUP TONE TEST: tono 1kHz directo, bypasea toda la cadena ── */
    if(kStartupToneTest){
        static uint32_t toneSamples = 0;
        static float    tonePhase   = 0.0f;
        const  uint32_t toneDurSamp = (uint32_t)SAMPLE_RATE * RED808_STARTUP_TONE_SECONDS;
        if(toneSamples < toneDurSamp){
            for(size_t i = 0; i < size; i++){
                float s = 0.7f * sinf(tonePhase);
                out[0][i] = s;
                out[1][i] = s;
                tonePhase += 2.0f * 3.14159265f * 1000.0f / (float)SAMPLE_RATE;
                if(tonePhase > 2.0f * 3.14159265f) tonePhase -= 2.0f * 3.14159265f;
                toneSamples++;
            }
            DSP_PROF_END(CALLBACK);
            DspProfBlockDone();
            audioLoadMeter.OnBlockEnd();
            return;
        }
    }

    for(size_t i = 0; i < size; i++) out[0][i] = out[1][i] = 0.0f;

    if(kAudioSafeMode){
        DSP_PROF_END(CALLBACK);
        DspProfBlockDone();
        audioLoadMeter.OnBlockEnd();
        return;
    }

    /* ── Kit loading: output silence to avoid SDRAM bus contention / data races ── */
    if(kitMuteActive){
        DSP_PROF_END(CALLBACK);
        DspProfBlockDone();
        audioLoadMeter.OnBlockEnd();
        return;
    }

    float mixPeak = 0.0f;

    float blockCpuAvg = AudioCpuAvgPercent();
    if(blockCpuAvg > 86.0f)
        audioFxShed = true;
    else if(blockCpuAvg < 68.0f)
        audioFxShed = false;
    const bool fxShed = audioFxShed;

    /* ═ Pre-calcular: primer track que usa cada motor de síntesis ═ */
    int8_t engTrk[SYNTH_ENGINE_COUNT];
    for(int _ei = 0; _ei < SYNTH_ENGINE_COUNT; _ei++) engTrk[_ei] = -1;
    for(int _t = 0; _t < DSQ_TRACKS; _t++){
        int8_t _e = dsqTrackEngine[_t];
        if(_e >= 0 && _e < SYNTH_ENGINE_COUNT && engTrk[_e] < 0)
            engTrk[_e] = (int8_t)_t;
    }

    const bool revEng = IsReverbEngaged();
    const bool delEng = IsDelayEngaged();
    const bool choEng = !fxShed && IsChorusEngaged();
    bool anyTrackLfo = false;
    for(int _t = 0; _t < MAX_PADS; _t++){
        if(trkLfoActive[_t] && trkLfoDepth[_t] > 0.0001f){
            anyTrackLfo = true;
            break;
        }
    }

    /* Once per block (not per sample — cheap): ease any pad/track/global
     * filter whose target cutoff/Q moved since the last block a step closer,
     * instead of the old behavior of snapping straight to it. See
     * UpdatePadFilterSmoothing() above for why. */
    UpdatePadFilterSmoothing();
    UpdateTrackFilterSmoothing();
    UpdateGlobalFilterSmoothing();

    float lfoVal[MAX_PADS];
    uint8_t trkFilterLfoSet[MAX_PADS];

    for(size_t i = 0; i < size; i++){
        /* ── Daisy Sequencer tick (sample-accurate BPM clock) ─────────────
         *  samplesElapsed==0 → new step boundary: advance and fire.
         *  Called BEFORE voice rendering so new voices are active this sample. */
        DSP_PROF_SCOPE(SEQ);
        if(dseq.playing){
            if(dseq.samplesElapsed == 0){
                const int16_t previousStep = dseq.currentStep;
                dseq.currentStep = (dseq.currentStep + 1) % (int16_t)dseq.patternLength;

                /* Advance the song before firing step zero of the next cycle.
                 * Previously the old pattern fired step 0 and only then changed
                 * pattern, producing a one-step hybrid at every transition. */
                const bool patternWrapped = previousStep >= 0 && dseq.currentStep == 0;
                if(patternWrapped){
                    if(dseq.performancePatternActive && dseq.performanceReturnPattern >= 0){
                        if(dseq.performanceBarsRemaining > 1){
                            dseq.performanceBarsRemaining--;
                        } else {
                            dseq.currentPattern = (uint8_t)dseq.performanceReturnPattern;
                            dseq.performanceReturnPattern = -1;
                            dseq.performanceBarsRemaining = 0;
                            dseq.performancePatternActive = false;
                        }
                    } else if(dseq.queuedPattern >= 0){
                        const uint8_t nextPattern = (uint8_t)dseq.queuedPattern;
                        dseq.queuedPattern = -1;
                        if(dseq.queuedPatternBars > 0){
                            dseq.performanceReturnPattern = (int8_t)dseq.currentPattern;
                            dseq.performanceBarsRemaining = dseq.queuedPatternBars;
                            dseq.performancePatternActive = true;
                        }
                        dseq.queuedPatternBars = 0;
                        dseq.currentPattern = nextPattern;
                    }
                }
                if(songPlaying && patternWrapped && songLength > 0){
                    songRepeatCnt++;
                    if(songRepeatCnt >= songChain[songIdx].repeats){
                        songRepeatCnt = 0;
                        songIdx++;
                        if(songIdx >= songLength){
                            if(kStartupShowcaseDemo){
                                songIdx = 0;
                                dseq.currentPattern = songChain[0].pattern;
                            } else {
                                songPlaying = false;
                                dseq.playing = false;
                            }
                        } else {
                            dseq.currentPattern = songChain[songIdx].pattern;
                        }
                    }
                }
                // Natural song end stops on the bar boundary. Do not fire
                // step zero of the final scene once songPlaying has cleared.
                if(dseq.playing) DsqFireStep();
                /* ── Stems: re-trigger enabled clean tracks from the top at each
                 *    pattern restart so a one-shot stem stays locked to the bar
                 *    (plays in sync with the looping sequencer). Muted tracks are
                 *    armed but stay silent in the mixer until unmuted. ── */
                if(dseq.playing && dseq.currentStep == 0){
                    for(int ct = 0; ct < CLEAN_TRACK_COUNT; ct++){
                        if(cleanTrackEnabled[ct] && cleanTrackLoaded[ct]){
                            cleanTrackPlayhead[ct] = 0;
                            cleanTrackActive[ct] = true;
                        }
                    }
                }
            }
            DsqProcessPendingTriggers();
            DsqProcessHeldNotes();
            dseq.samplesElapsed++;
            /* Swing conserva la duración de cada pareja: el step par se
             * alarga y el impar se acorta en la misma cantidad. Así el onset
             * impar llega tarde sin cambiar el BPM global. */
            uint32_t thr = dseq.samplesPerStep;
            if(dseq.swingAmount > 0){
                const uint32_t offset = dseq.samplesPerStep * (uint32_t)dseq.swingAmount / 200u;
                if((dseq.currentStep & 1) == 0) thr = dseq.samplesPerStep + offset;
                else thr = dseq.samplesPerStep > offset ? dseq.samplesPerStep - offset : 1;
            }
            if(dseq.samplesElapsed >= thr)
                dseq.samplesElapsed = 0;
        }
        DSP_PROF_END(SEQ);

        float busL = 0, busR = 0;
        float reverbBusL = 0, reverbBusR = 0;
        float delayBusL  = 0, delayBusR  = 0;
        float chorusBusL = 0, chorusBusR = 0;
        float sideSrc = 0;

        DSP_PROF_SCOPE(LFO);
        if(anyTrackLfo){
            for(int t = 0; t < MAX_PADS; t++){
                lfoVal[t] = 0.0f;
                trkFilterLfoSet[t] = 0;
                if(!trkLfoActive[t] || trkLfoDepth[t] <= 0.0001f) continue;

                trkLfoPhase[t] += trkLfoRate[t] / (float)SAMPLE_RATE;
                if(trkLfoPhase[t] >= 1.0f){
                    trkLfoPhase[t] -= 1.0f;
                    if(trkLfoWave[t] == LFO_WAVE_SH)
                        trkLfoSH[t] = RandFloat(); /* -1..1 */
                }

                float v = 0.0f;
                if(trkLfoWave[t] == LFO_WAVE_TRI)
                    v = TriFromPhase(trkLfoPhase[t]);
                else if(trkLfoWave[t] == LFO_WAVE_SH)
                    v = trkLfoSH[t];
                else
                    v = __fast_sinf(2.0f * (float)M_PI * trkLfoPhase[t]);

                lfoVal[t] = v * trkLfoDepth[t];
            }
        }
        DSP_PROF_END(LFO);

        for(uint8_t ct = 0; ct < CLEAN_TRACK_COUNT; ct++){
            if(!cleanTrackActive[ct] || cleanTrackMuted[ct] || !cleanTrackLoaded[ct])
                continue;
            int16_t* cleanData = CleanTrackPtr(ct);
            if(cleanData == nullptr || cleanTrackLength[ct] == 0){
                cleanTrackActive[ct] = false;
                continue;
            }
            uint32_t pos = cleanTrackPlayhead[ct];
            if(pos >= cleanTrackLength[ct]){
                cleanTrackActive[ct] = false;
                cleanTrackPlayhead[ct] = 0;
                continue;
            }
            float sample = cleanData[pos] / 32768.0f;
            cleanTrackPlayhead[ct] = pos + 1;
            busL += sample;
            busR += sample;
        }

        /* ── Render voices ── */
        DSP_PROF_SCOPE(SAMPLER_VOICES);
        for(int v = 0; v < MAX_VOICES; v++){
            Voice& vx = voices[v];
            if(!vx.active) continue;
            uint8_t p = vx.pad;

            /* Click-free voice stealing: the previous routed value decays on
             * the dry bus while the replacement voice starts immediately. */
            if(fabsf(vx.stealTailL) > STEAL_TAIL_FLOOR
            || fabsf(vx.stealTailR) > STEAL_TAIL_FLOOR){
                busL += vx.stealTailL;
                busR += vx.stealTailR;
                vx.stealTailL *= STEAL_TAIL_COEF;
                vx.stealTailR *= STEAL_TAIL_COEF;
            } else {
                vx.stealTailL = 0.0f;
                vx.stealTailR = 0.0f;
            }

            /* Position / bounds */
            /* Skip voices on pads being reloaded */
            if(padLoading[p]){ vx.active = false; continue; }

            uint32_t idx = (uint32_t)fabsf(vx.pos);
            uint32_t endLen = (vx.maxLen > 0 && vx.maxLen < sampleLength[p])
                             ? vx.maxLen : sampleLength[p];
            if(padReverse[p]){
                if(vx.pos < (float)vx.trimStart){
                    if(padLoop[p]) vx.pos = (float)(endLen - 1);
                    else { vx.active = false; continue; }
                }
            } else {
                if(idx >= endLen){
                    if(padLoop[p]){ vx.pos = (float)vx.trimStart; idx = vx.trimStart; }
                    else { vx.active = false; continue; }
                }
            }
            idx = (uint32_t)fabsf(vx.pos);
            if(idx >= sampleLength[p]){ vx.active = false; continue; }

            int16_t* sampleData = SamplePtr(p);
            if(sampleData == nullptr){ vx.active = false; continue; }

            /* Interpolation */
            float frac = fabsf(vx.pos) - idx;
            float s0   = sampleData[idx] / 32768.0f;
            float s1   = (idx + 1 < sampleLength[p])
                         ? sampleData[idx + 1] / 32768.0f : 0.0f;
            float s    = s0 + frac * (s1 - s0);

            /* ── Voice AD envelope ── */
            if(vx.envStage == 0){
                vx.env += vx.envAttackInc;
                if(vx.env >= 1.0f){
                    vx.env = 1.0f;
                    vx.envStage = 1;
                }
            } else if(vx.envStage == 1){
                vx.env *= vx.envDecayCoef;
                if(vx.env < 0.0005f){
                    vx.active = false;
                    continue;
                }
            }
            s *= vx.env;

            /* ── Stutter ── */
            if(padStutterOn[p]){
                padStutterCnt[p]++;
                if(padStutterCnt[p] >= padStutterIval[p]){
                    padStutterCnt[p] = 0;
                    if(vx.pos > 100.f) vx.pos -= 100.f; else vx.pos = 0.f;
                }
            }

            /* ── Advance position ── */
            float adv = vx.speed;
            /* ── LFO → Pitch (modulate playback speed) ── */
            if(trkLfoActive[p] && trkLfoTarget[p] == LFO_TGT_PITCH)
                adv *= clampF(1.0f + 0.5f * lfoVal[p], 0.25f, 4.0f);

            vx.pos += padReverse[p] ? -adv : adv;

            /* ── Pad filter ── */
            if(padFilterType[p]){
                s = sanitizeF(padFilter[p].Process(s));
            }

            /* ── Pad distortion + crush ── */
            s = ApplyDist(s, padDistDrive[p], padDistMode[p]);
            s = BitCrush(s, padBitDepth[p]);

            /* ── Per-track FX (only when routed in graph) ── */
            if(trkFxRouted[p]){

            /* ── Per-track filter ── */
            if(trkFilterType[p]){
                if(trkLfoActive[p] && trkLfoTarget[p] == LFO_TGT_FILTER && !trkFilterLfoSet[p]){
                    float modCut = trkFilterCut[p] * (1.0f + 0.9f * lfoVal[p]);
                    modCut = clampF(modCut, 20.f, 20000.f);
                    trkFilter[p].SetType(trkFilterType[p], modCut, trkFilterQ[p], (float)SAMPLE_RATE);
                    if(trkFilterType[p] == FTYPE_RESONANT)
                        trkFilter2[p].SetType(FTYPE_RESONANT, modCut, trkFilterQ[p], (float)SAMPLE_RATE);
                    trkFilterLfoSet[p] = 1;
                }
                s = sanitizeF(trkFilter[p].Process(s));
                if(trkFilterType[p] == FTYPE_RESONANT){
                    s = sanitizeF(trkFilter2[p].Process(s));
                    s = SoftLimit(s * 1.4f) * 0.714f;
                    s *= ResonantMakeupGain(trkFilterQ[p]);
                }
            }

            /* ── Per-track dist + crush ── */
            {
                float drv = trkDistDrive[p];
                if(trkLfoActive[p] && trkLfoTarget[p] == LFO_TGT_DIST_DRIVE)
                    drv = clampF(drv * (1.0f + 0.8f * lfoVal[p]), 0.0f, 64.0f);
                s = ApplyDist(s, drv, trkDistMode[p]);

                float bd = trkBitDepth[p];
                if(trkLfoActive[p] && trkLfoTarget[p] == LFO_TGT_CRUSH)
                    bd = clampF(bd - 6.0f * lfoVal[p], 2.0f, 16.0f);
                s = BitCrush(s, bd);
            }

            /* ── Per-track EQ (3-band) ── */
            if(trkEqLowDb[p])  s = sanitizeF(trkEqLow[p].Process(s));
            if(trkEqMidDb[p])  s = sanitizeF(trkEqMid[p].Process(s));
            if(trkEqHighDb[p]) s = sanitizeF(trkEqHigh[p].Process(s));

            /* ── Per-track echo ── */
            if(!fxShed && trkEchoActive[p]){
                float rawDelay = trkEchoDelay[p];
                if(trkLfoActive[p] && trkLfoTarget[p] == LFO_TGT_ECHO_TIME)
                    rawDelay = clampF(rawDelay * (1.0f + 0.4f * lfoVal[p]), 1.f, (float)(TRACK_ECHO_SIZE-1));
                uint32_t d = (uint32_t)rawDelay;
                if(d == 0)
                    d = 1;
                if(d >= TRACK_ECHO_SIZE)
                    d = TRACK_ECHO_SIZE - 1;
                uint32_t rp = (trkEchoWp[p] + TRACK_ECHO_SIZE - d) % TRACK_ECHO_SIZE;
                float delayed = trkEchoBuf[p][rp];
                trkEchoBuf[p][trkEchoWp[p]] = clampF(s + delayed*trkEchoFb[p], -1.f, 1.f);
                s = s*(1.f - trkEchoMix[p]) + delayed*trkEchoMix[p];
                trkEchoWp[p] = (trkEchoWp[p] + 1) % TRACK_ECHO_SIZE;
            }

            /* ── Per-track flanger ── */
            if(!fxShed && trkFlgActive[p]){
                float wet = sanitizeF(trkFlanger[p].Process(s));
                s = s*(1.f - trkFlgMix[p]) + wet*trkFlgMix[p];
            }

            /* ── Per-track compressor ── */
            if(!fxShed && trkCompActive[p]){
                float absS = fabsf(s);
                if(absS > trkCompEnv[p]) trkCompEnv[p] += (absS - trkCompEnv[p]) * 0.25f;
                else                     trkCompEnv[p] -= (trkCompEnv[p] - absS) * 0.03f;
                if(trkCompEnv[p] > trkCompThresh[p] && trkCompEnv[p] > 0.001f){
                    float g = trkCompThresh[p] / trkCompEnv[p];
                    g = fast_powf(g, trkCompExp[p]);
                    if(g < 0.125f) g = 0.125f;
                    s *= g;
                }
            }

            } /* end trkFxRouted[p] */

            /* ── Sidechain ── */
            float absS = fabsf(s);
            if(scActive && p == scSrc) sideSrc = fmaxf(sideSrc, absS);
            if(scActive && p != scSrc && (scDstMask & (1u << p))){
                float duck = scAmount * scEnv;
                if(duck > 0.88f) duck = 0.88f;
                s *= (1.f - duck);
            }

            /* ── Mute / Solo ── */
            bool muted = trackMute[p];
            if(anySolo && !trackSolo[p]) muted = true;
            if(muted) s = 0;

            float lfoGain = 1.0f;
            if(trkLfoActive[p] && trkLfoTarget[p] == LFO_TGT_GAIN)
                lfoGain = clampF(1.0f + 0.8f * lfoVal[p], 0.0f, 2.0f);

            /* ── Apply voice gain → mix ── */
            float outL = s * vx.gainL * lfoGain;
            float outR = s * vx.gainR * lfoGain;

            /* ── Pan ── */
            float panTrack = trackPanF[p];
            if(trkLfoActive[p] && trkLfoTarget[p] == LFO_TGT_PAN)
                panTrack = clampF(panTrack + 0.9f * lfoVal[p], -1.0f, 1.0f);
            float panL = (1.0f - panTrack) * 0.5f;
            float panR = (1.0f + panTrack) * 0.5f;
            float routedL = outL * panL;
            float routedR = outR * panR;
            busL += routedL;
            busR += routedR;
            vx.lastOutL = routedL;
            vx.lastOutR = routedR;

            /* ── Send buses (stereo) — only accumulate if master FX engaged ── */
            if(revEng){
                float rSend = trackReverbSend[p];
                if(trkLfoActive[p] && trkLfoTarget[p] == LFO_TGT_SEND_REV)
                    rSend = clampF(rSend + 0.5f * lfoVal[p], 0.0f, 1.0f);
                reverbBusL += outL * rSend;
                reverbBusR += outR * rSend;
            }
            if(delEng){
                float dSend = trackDelaySend[p];
                if(trkLfoActive[p] && trkLfoTarget[p] == LFO_TGT_SEND_DEL)
                    dSend = clampF(dSend + 0.5f * lfoVal[p], 0.0f, 1.0f);
                delayBusL  += outL * dSend;
                delayBusR  += outR * dSend;
            }
            if(choEng){
                chorusBusL += outL * trackChorusSend[p];
                chorusBusR += outR * trackChorusSend[p];
            }

            /* ── Track peak ── */
            float pk = fmaxf(fabsf(outL), fabsf(outR));
            if(pk > trackPeak[p]) trackPeak[p] = pk;
        }
        DSP_PROF_END(SAMPLER_VOICES);

        /* ── Sidechain envelope ── */
        if(scActive){
            if(sideSrc > scEnv) scEnv += (sideSrc - scEnv) * scAttackK;
            else                scEnv -= (scEnv - sideSrc) * scReleaseK;
        }

        /* ── SYNTH ENGINES — process + cadena FX per-track ── */
        /* Lambda: aplica filtro/dist/EQ/echo/flanger/comp/vol/pan del track t  */
        /* al sample s y lo suma a busL/busR. Si t<0 -> bus directo sin FX.     */
        auto synthTobus = [&](float s, int8_t t){
            if(t < 0 || t >= MAX_PADS){ busL += s; busR += s; return; }
            if(trkFxRouted[t]){
            /* filtro */
            if(trkFilterType[t]){
                s = sanitizeF(trkFilter[t].Process(s));
                if(trkFilterType[t] == FTYPE_RESONANT){
                    s = sanitizeF(trkFilter2[t].Process(s));
                    s = SoftLimit(s * 1.4f) * 0.714f;
                    s *= ResonantMakeupGain(trkFilterQ[t]);
                }
            }
            /* dist + bitcrush */
            s = ApplyDist(s, trkDistDrive[t], trkDistMode[t]);
            s = BitCrush(s, trkBitDepth[t]);
            /* EQ 3 bandas */
            if(trkEqLowDb[t])  s = sanitizeF(trkEqLow[t].Process(s));
            if(trkEqMidDb[t])  s = sanitizeF(trkEqMid[t].Process(s));
            if(trkEqHighDb[t]) s = sanitizeF(trkEqHigh[t].Process(s));
            /* echo */
            if(!fxShed && trkEchoActive[t]){
                uint32_t d = (uint32_t)trkEchoDelay[t];
                if(d == 0)
                    d = 1;
                if(d >= TRACK_ECHO_SIZE)
                    d = TRACK_ECHO_SIZE - 1;
                uint32_t rpe = (trkEchoWp[t] + TRACK_ECHO_SIZE - d) % TRACK_ECHO_SIZE;
                float delayed = trkEchoBuf[t][rpe];
                trkEchoBuf[t][trkEchoWp[t]] = clampF(s + delayed*trkEchoFb[t], -1.f, 1.f);
                s = s*(1.f - trkEchoMix[t]) + delayed*trkEchoMix[t];
                trkEchoWp[t] = (trkEchoWp[t] + 1) % TRACK_ECHO_SIZE;
            }
            /* flanger */
            if(!fxShed && trkFlgActive[t]){
                float wet = sanitizeF(trkFlanger[t].Process(s));
                s = s*(1.f - trkFlgMix[t]) + wet*trkFlgMix[t];
            }
            /* compressor */
            if(!fxShed && trkCompActive[t]){
                float absS = fabsf(s);
                if(absS > trkCompEnv[t]) trkCompEnv[t] += (absS - trkCompEnv[t]) * 0.25f;
                else                     trkCompEnv[t] -= (trkCompEnv[t] - absS) * 0.03f;
                if(trkCompEnv[t] > trkCompThresh[t] && trkCompEnv[t] > 0.001f){
                    float g = trkCompThresh[t] / trkCompEnv[t];
                    g = fast_powf(g, trkCompExp[t]);
                    if(g < 0.125f) g = 0.125f;
                    s *= g;
                }
            }
            } /* end trkFxRouted */
            /* mute / solo */
            bool muted = trackMute[t];
            if(anySolo && !trackSolo[t]) muted = true;
            if(muted) return;
            /* LFO gain / pan */
            float lfoGain = 1.0f;
            if(trkLfoActive[t] && trkLfoTarget[t] == LFO_TGT_GAIN)
                lfoGain = clampF(1.0f + 0.8f * lfoVal[t], 0.0f, 2.0f);
            float panTrk = trackPanF[t];
            if(trkLfoActive[t] && trkLfoTarget[t] == LFO_TGT_PAN)
                panTrk = clampF(panTrk + 0.9f * lfoVal[t], -1.0f, 1.0f);
            /* vol + pan -> bus */
            float outS = s * trackGain[t] * lfoGain;
            float pL = (1.f - panTrk) * 0.5f;
            float pR = (1.f + panTrk) * 0.5f;
            busL += outS * pL;
            busR += outS * pR;
            /* sends (stereo) — only if master FX engaged */
            float sndL = outS * pL, sndR = outS * pR;
            if(revEng){
                reverbBusL += sndL * trackReverbSend[t];
                reverbBusR += sndR * trackReverbSend[t];
            }
            if(delEng){
                delayBusL  += sndL * trackDelaySend[t];
                delayBusR  += sndR * trackDelaySend[t];
            }
            if(choEng){
                chorusBusL += sndL * trackChorusSend[t];
                chorusBusR += sndR * trackChorusSend[t];
            }
            /* peak */
            float pk = fabsf(outS);
            if(pk > trackPeak[t]) trackPeak[t] = pk;
        };

        if (synthActiveMask & (1 << SYNTH_ENGINE_808)){
            DSP_PROF_SCOPE(SYNTH_808);
            float s = sanitizeF(synth808.Process()) * kDrumBusHeadroom;
            DSP_PROF_END(SYNTH_808);
            DSP_PROF_SCOPE(SYNTH_ROUTING);
            synthTobus(s, engTrk[SYNTH_ENGINE_808]);
            DSP_PROF_END(SYNTH_ROUTING);
        }
        if (synthActiveMask & (1 << SYNTH_ENGINE_909)){
            DSP_PROF_SCOPE(SYNTH_909);
            float s = sanitizeF(synth909.Process()) * kDrumBusHeadroom;
            DSP_PROF_END(SYNTH_909);
            DSP_PROF_SCOPE(SYNTH_ROUTING);
            synthTobus(s, engTrk[SYNTH_ENGINE_909]);
            DSP_PROF_END(SYNTH_ROUTING);
        }
        if (kEnableSynth505 && (synthActiveMask & (1 << SYNTH_ENGINE_505))){
            DSP_PROF_SCOPE(SYNTH_505);
            float s = sanitizeF(synth505.Process()) * kDrumBusHeadroom;
            DSP_PROF_END(SYNTH_505);
            DSP_PROF_SCOPE(SYNTH_ROUTING);
            synthTobus(s, engTrk[SYNTH_ENGINE_505]);
            DSP_PROF_END(SYNTH_ROUTING);
        }
        if ((synthActiveMask & (1 << SYNTH_ENGINE_303)) && acid303.IsActive()){
            /* v2.5: −4dB headroom en synths melódicos para no saturar el bus */
            DSP_PROF_SCOPE(SYNTH_303);
            float s = sanitizeF(acid303.Process()) * 0.63f;
            DSP_PROF_END(SYNTH_303);
            DSP_PROF_SCOPE(SYNTH_ROUTING);
            synthTobus(s, engTrk[SYNTH_ENGINE_303]);
            DSP_PROF_END(SYNTH_ROUTING);
        }
        if ((synthActiveMask & (1 << SYNTH_ENGINE_WTOSC)) && wtOsc.IsActive()){
            DSP_PROF_SCOPE(SYNTH_WT);
            float s = sanitizeF(wtOsc.Process()) * 0.63f;
            DSP_PROF_END(SYNTH_WT);
            DSP_PROF_SCOPE(SYNTH_ROUTING);
            synthTobus(s, engTrk[SYNTH_ENGINE_WTOSC]);
            DSP_PROF_END(SYNTH_ROUTING);
        }
        if ((synthActiveMask & (1 << SYNTH_ENGINE_SH101)) && synthSH101.IsActive()){  /* I1 */
            DSP_PROF_SCOPE(SYNTH_SH101);
            float s = sanitizeF(synthSH101.Process()) * 0.63f;
            DSP_PROF_END(SYNTH_SH101);
            DSP_PROF_SCOPE(SYNTH_ROUTING);
            synthTobus(s, engTrk[SYNTH_ENGINE_SH101]);
            DSP_PROF_END(SYNTH_ROUTING);
        }
        if ((synthActiveMask & (1 << SYNTH_ENGINE_FM2OP)) && synthFM2Op.IsActive()){  /* I2 */
            DSP_PROF_SCOPE(SYNTH_FM2OP);
            float s = sanitizeF(synthFM2Op.Process()) * 0.63f;
            DSP_PROF_END(SYNTH_FM2OP);
            DSP_PROF_SCOPE(SYNTH_ROUTING);
            synthTobus(s, engTrk[SYNTH_ENGINE_FM2OP]);
            DSP_PROF_END(SYNTH_ROUTING);
        }
        if (synthActiveMask & (1 << SYNTH_ENGINE_PHYS)){
            DSP_PROF_SCOPE(SYNTH_PHYS);
            float s = 0;
            if(physModalActive)  s += sanitizeF(physModal.Process())  * physModalGain;
            if(physStringActive) s += sanitizeF(physString.Process()) * physStringGain;
            DSP_PROF_END(SYNTH_PHYS);
            DSP_PROF_SCOPE(SYNTH_ROUTING);
            synthTobus(sanitizeF(s), engTrk[SYNTH_ENGINE_PHYS]);
            DSP_PROF_END(SYNTH_ROUTING);
        }
        if (synthActiveMask & (1 << SYNTH_ENGINE_NOISE)){
            if(noisePartActive){
                DSP_PROF_SCOPE(SYNTH_NOISE);
                float s = sanitizeF(noisePart.Process()) * noisePartGain;
                DSP_PROF_END(SYNTH_NOISE);
                DSP_PROF_SCOPE(SYNTH_ROUTING);
                synthTobus(sanitizeF(s), engTrk[SYNTH_ENGINE_NOISE]);
                DSP_PROF_END(SYNTH_ROUTING);
            }
        }

        /* ── Startup section cue (formante retro-robótico) ── */
        if(startupAnnounceActive)
        {
            float cue = startupAnnounceOsc.Process() * startupAnnounceEnv * 0.16f;
            busL += cue;
            busR += cue;
            startupAnnounceEnv *= 0.9994f;
            if(startupAnnounceRemain > 0) startupAnnounceRemain--;
            if(startupAnnounceRemain == 0 || startupAnnounceEnv < 0.005f)
                startupAnnounceActive = false;
        }

        /* ── MASTER FX CHAIN ── */
        DSP_PROF_SCOPE(MASTER_FX);
        float gainOut = kForceMasterGainDebug ? 1.0f : masterGain;
        /* Sanitize bus accumulators to catch any NaN from voice/synth processing */
        float L = sanitizeF(busL) * gainOut;
        float R = sanitizeF(busR) * gainOut;
        reverbBusL = sanitizeF(reverbBusL);
        reverbBusR = sanitizeF(reverbBusR);
        delayBusL  = sanitizeF(delayBusL);
        delayBusR  = sanitizeF(delayBusR);
        chorusBusL = sanitizeF(chorusBusL);
        chorusBusR = sanitizeF(chorusBusR);

        /* ── Global filter ── */
        if(gFilterRouted && gFilterType != FTYPE_NONE){
            /* Ladder / SVF / Comb filter handled by DaisySP modules */
            if(gFilterType == FTYPE_LADDER){
                L = sanitizeF(masterLadderL.Process(L));
                R = sanitizeF(masterLadderR.Process(R));
            } else if(gFilterType >= FTYPE_SVF_LP && gFilterType <= FTYPE_SVF_BP){
                masterSvfL.Process(L);
                masterSvfR.Process(R);
                if(gFilterType == FTYPE_SVF_LP)      { L = sanitizeF(masterSvfL.Low());  R = sanitizeF(masterSvfR.Low()); }
                else if(gFilterType == FTYPE_SVF_HP)  { L = sanitizeF(masterSvfL.High()); R = sanitizeF(masterSvfR.High()); }
                else /* FTYPE_SVF_BP */               { L = sanitizeF(masterSvfL.Band()); R = sanitizeF(masterSvfR.Band()); }
            } else if(gFilterType == FTYPE_SVF_MORPH){
                /* Same Svf as SVF_LP/HP/BP above, but instead of picking one
                 * fixed output, crossfade continuously across LP -> BP -> HP
                 * -> Notch as gFilterMorph goes 0 -> 1 (three linear
                 * segments), so a knob sweep morphs smoothly instead of
                 * hard-switching between fixed models. Notch = dry - Band is
                 * the standard SVF identity (removing exactly the band a
                 * bandpass keeps reconstructs the notch) — used instead of
                 * a Notch() accessor since this DaisySP version's Svf isn't
                 * vendored in this checkout to confirm it has one; Low/High/
                 * Band are already relied on above (FTYPE_SVF_LP/HP/BP). */
                const float dryL = L, dryR = R;
                masterSvfL.Process(L);
                masterSvfR.Process(R);
                float m = clampF(gFilterMorph, 0.f, 1.f) * 3.0f;
                float lpW = 0.f, bpW = 0.f, hpW = 0.f, nW = 0.f;
                if(m < 1.0f)      { lpW = 1.0f - m; bpW = m; }
                else if(m < 2.0f) { bpW = 2.0f - m; hpW = m - 1.0f; }
                else              { hpW = 3.0f - m; nW  = m - 2.0f; }
                const float bandL = masterSvfL.Band(), bandR = masterSvfR.Band();
                const float notchL = dryL - bandL, notchR = dryR - bandR;
                L = sanitizeF(masterSvfL.Low() * lpW + bandL * bpW
                            + masterSvfL.High() * hpW + notchL * nW);
                R = sanitizeF(masterSvfR.Low() * lpW + bandR * bpW
                            + masterSvfR.High() * hpW + notchR * nW);
            } else if(gFilterType == FTYPE_COMB){
                /* Comb filter via short delay line with feedback */
                float combDelay = clampF(1.f / (gFilterCutoff > 20.f ? gFilterCutoff : 20.f) * (float)SAMPLE_RATE, 1.f, 4799.f);
                float combFb = clampF(gFilterQ / 30.f, 0.f, 0.98f);
                float combL = combDelayL.Read(combDelay);
                float combR = combDelayR.Read(combDelay);
                combDelayL.Write(clampF(L + combL * combFb, -4.f, 4.f));
                combDelayR.Write(clampF(R + combR * combFb, -4.f, 4.f));
                L = L * 0.5f + combL * 0.5f;
                R = R * 0.5f + combR * 0.5f;
            } else {
                L = sanitizeF(gFilterL.Process(L));
                R = sanitizeF(gFilterR.Process(R));
                if(gFilterType == FTYPE_RESONANT){
                    L = sanitizeF(gFilter2L.Process(L));
                    R = sanitizeF(gFilter2R.Process(R));
                    L = SoftLimit(L * 1.4f) * 0.714f;
                    R = SoftLimit(R * 1.4f) * 0.714f;
                    const float mk = ResonantMakeupGain(gFilterQ);
                    L *= mk;
                    R *= mk;
                }
            }
        }

        /* ── Global bitcrush + distortion ── */
        if(gFilterRouted && (gFilterBitDepth < 16
           || fabsf(gFilterDist) > 0.0001f)){
            L = BitCrush(L, gFilterBitDepth);
            R = BitCrush(R, gFilterBitDepth);
            L = ApplyDist(L, gFilterDist, gFilterDistMode);
            R = ApplyDist(R, gFilterDist, gFilterDistMode);
        }

        /* ── Global SAMPLE_RATE reduce ── */
        if(gFilterRouted && gFilterSrReduce > 0
           && gFilterSrReduce < (uint32_t)SAMPLE_RATE){
            if(!gSrPrimed){
                gSrHoldL = L; gSrHoldR = R;
                gSrPrimed = true;
            }
            gSrPhase += gFilterSrReduce;
            if(gSrPhase >= (uint32_t)SAMPLE_RATE){
                gSrPhase -= (uint32_t)SAMPLE_RATE;
                gSrHoldL = L; gSrHoldR = R;
            }
            L = gSrHoldL; R = gSrHoldR;
        }

        /* ── Autowah ── */
        if(!fxShed && IsAutowahEngaged()){
            float awL = sanitizeF(masterAutowahL.Process(L));
            float awR = sanitizeF(masterAutowahR.Process(R));
            L = L * (1.0f - autowahMix) + awL * autowahMix;
            R = R * (1.0f - autowahMix) + awR * autowahMix;
        }

        /* ── Delay (mono or ping-pong stereo) ── */
        if(IsDelayEngaged()){
            float delaySendMono = (delayBusL + delayBusR) * 0.5f;
            if(delayPingPong){
                float wetL = masterDelay.Read();
                float wetR = masterDelayR.Read();
                masterDelay.Write(clampF(L + delaySendMono + wetR * delayFeedback, -4.f, 4.f));
                masterDelayR.Write(clampF(R + delaySendMono + wetL * delayFeedback, -4.f, 4.f));
                L = L * (1.0f - delayMix) + wetL * delayMix;
                R = R * (1.0f - delayMix) + wetR * delayMix;
            } else {
                float wet = masterDelay.Read();
                const float monoIn = (L + R) * 0.5f;
                masterDelay.Write(clampF(monoIn + delaySendMono
                                        + wet * delayFeedback, -4.f, 4.f));
                L = L * (1.0f - delayMix) + wet * delayMix;
                R = R * (1.0f - delayMix) + wet * delayMix;
            }
        }

        /* ── Compressor ── */
        if(!fxShed && IsCompEngaged()){
            L = sanitizeF(masterComp.Process(L));
            R = sanitizeF(masterComp.Apply(R));
        }

        /* ── Wavefolder ── */
        if(!fxShed && IsWaveFolderEngaged()){
            L = WaveFoldSample(L, waveFolderGain);
            R = WaveFoldSample(R, waveFolderGain);
        }

        /* ── Phaser ── */
        if(!fxShed && IsPhaserEngaged()){
            /* DaisySP Phaser sums four engines; normalize the sum to avoid a
             * 12 dB level jump before the output limiter. */
            L = sanitizeF(masterPhaserL.Process(L) * 0.25f);
            R = sanitizeF(masterPhaserR.Process(R) * 0.25f);
        }

        /* ── Flanger (DaisySP) ── */
        if(!fxShed && IsFlangerEngaged()){
            float wetL = sanitizeF(masterFlangerL.Process(L));
            float wetR = sanitizeF(masterFlangerR.Process(R));
            L = L*(1.f - flangerMix) + wetL*flangerMix;
            R = R*(1.f - flangerMix) + wetR*flangerMix;
        }

        /* ── Tremolo ── */
        if(IsTremoloEngaged()){
            float t = masterTremolo.Process(1.0f);
            L *= t; R *= t;
        }

        /* ── Chorus (mono or stereo, with send bus input) ── */
        if(!fxShed && IsChorusEngaged()){
            float chorusSendMono = (chorusBusL + chorusBusR) * 0.5f;
            if(chorusStereoMode){
                float wetL = sanitizeF(masterChorusL.Process(L + chorusSendMono));
                float wetR = sanitizeF(masterChorusR.Process(R + chorusSendMono));
                L = L * (1.0f - chorusMix) + wetL * chorusMix;
                R = R * (1.0f - chorusMix) + wetR * chorusMix;
            } else {
                const float monoIn = (L + R) * 0.5f + chorusSendMono;
                float wet = sanitizeF(masterChorusL.Process(monoIn));
                L = L * (1.0f - chorusMix) + wet * chorusMix;
                R = R * (1.0f - chorusMix) + wet * chorusMix;
            }
        }

        /* ── Early Reflections (before reverb) ── */
        if(!fxShed && IsEarlyRefEngaged()){
            float erL = 0, erR = 0;
            for(int t = 0; t < ER_TAPS; t++){
                erL += erDelayL.Read(erTapTimesL[t] * 0.001f * (float)SAMPLE_RATE) * erTapGains[t];
                erR += erDelayR.Read(erTapTimesR[t] * 0.001f * (float)SAMPLE_RATE) * erTapGains[t];
            }
            erDelayL.Write(sanitizeF(L));
            erDelayR.Write(sanitizeF(R));
            L = L * (1.0f - erMix) + sanitizeF(erL) * erMix;
            R = R * (1.0f - erMix) + sanitizeF(erR) * erMix;
        }

        /* ── Reverb (with send bus input) ── */
        float revL = 0, revR = 0;
        if(IsReverbEngaged()){
            masterReverb.Process(L + reverbBusL, R + reverbBusR,
                                &revL, &revR);
            revL = sanitizeF(revL);
            revR = sanitizeF(revR);
            L = L * (1.0f - reverbMix) + revL * reverbMix;
            R = R * (1.0f - reverbMix) + revR * reverbMix;
        }

        /* ── Stereo Width (Mid-Side processing) ── */
        if(stereoWidth < 0.99f || stereoWidth > 1.01f){
            float mid  = (L + R) * 0.5f;
            float side = (L - R) * 0.5f;
            side *= stereoWidth;
            L = mid + side;
            R = mid - side;
        }

        /* ── Tape Stop effect ── */
        if(tapeStopActive){
            if(tapeStopSpeed > 0.01f){
                tapeStopSpeed -= tapeStopRate;
                if(tapeStopSpeed < 0.0f) tapeStopSpeed = 0.0f;
            }
            L *= tapeStopSpeed;
            R *= tapeStopSpeed;
        }

        /* ── Beat Repeat ── */
        if(beatRepActive && beatRepDiv > 0 && beatRepLen > 0){
            if(beatRepCapturing){
                beatRepBufL[beatRepPos] = L;
                beatRepBufR[beatRepPos] = R;
                if(++beatRepPos >= beatRepLen){
                    beatRepPos = 0;
                    beatRepCapturing = false;
                }
            } else if(beatRepPlaying){
                L = beatRepBufL[beatRepPos];
                R = beatRepBufR[beatRepPos];
                if(++beatRepPos >= beatRepLen) beatRepPos = 0;
            }
        }
        DSP_PROF_END(MASTER_FX);

        /* ── Sanitize before final output (kill NaN/Inf from any DSP module) ── */
        DSP_PROF_SCOPE(OUTPUT);
        L = sanitizeF(L);
        R = sanitizeF(R);

        /* ── Limiter / Soft clip ── */
        if(IsLimiterEngaged()){
            L = clampF(L, -1.0f, 1.0f);
            R = clampF(R, -1.0f, 1.0f);
        } else {
            L = SoftClipKnee(L);
            R = SoftClipKnee(R);
        }

        /* M3: DC offset removal — HP 1-polo ~20 Hz */
        L = dcBlockL.Process(L);
        R = dcBlockR.Process(R);

        /* Self-heal instead of requiring a Daisy reboot: DcBlock is a
         * stateful 1-pole IIR (DaisySP) with no public state reset. If its
         * internal history was ever poisoned by a non-finite sample before
         * the sanitizeF()/SoftClipKnee() fixes above existed — or by any
         * future path this doesn't anticipate — every output sample stays
         * NaN forever regardless of upstream signal, which is exactly the
         * "audio keeps playing but sounds broken and controls do nothing"
         * failure this reboots to fix. Re-init on the spot instead. */
        if(!isfinite(L) || !isfinite(R)){
            dcBlockL.Init((float)SAMPLE_RATE);
            dcBlockR.Init((float)SAMPLE_RATE);
            L = 0.0f;
            R = 0.0f;
        }

        out[0][i] = L;
        out[1][i] = R;

        float pk = fmaxf(fabsf(L), fabsf(R));
        if(pk > mixPeak) mixPeak = pk;
        DSP_PROF_END(OUTPUT);
    }
    masterPeak = mixPeak;
    DSP_PROF_END(CALLBACK);
    DspProfBlockDone();
    audioLoadMeter.OnBlockEnd();
}

/* ── MIDI monitor + user MIDI map (P4 MIDI LEARN) ──────────────────────
 * ProcessMpdMidi() and ProcessCommand() both run in the main loop, so the
 * ring and the map need no ISR protection. The monitor keeps the raw wire
 * events (note-on/off + CC) so P4 can implement LEARN and pad lighting;
 * the user map is uploaded by P4 (CMD_MIDI_MAP_SET) and takes precedence
 * over the compiled MPD218 factory tables, on any MIDI channel. */
#define MIDI_MON_RING_SIZE  64u /* power of two */
#define MIDI_MAP_KIND_NOTE  0u
#define MIDI_MAP_KIND_CC    1u
#define MIDI_MAP_MAX_ENTRIES 64u

struct __attribute__((packed)) MidiMapEntry
{
    uint8_t channel; /* 0-15 zero-based */
    uint8_t kind;    /* MIDI_MAP_KIND_* */
    uint8_t number;  /* note or CC 0-127 */
    uint8_t action;  /* PadActionType (NOTE) / KnobActionType (CC) */
    uint8_t arg0;
    uint8_t arg1;
};

static uint8_t midiMonRing[MIDI_MON_RING_SIZE][3];
static uint8_t midiMonHead = 0;
static uint8_t midiMonTail = 0;

static MidiMapEntry midiUserMap[MIDI_MAP_MAX_ENTRIES];
static uint8_t      midiUserMapCount = 0;

static void MidiMonitorPush(uint8_t status, uint8_t data0, uint8_t data1)
{
    const uint8_t next = (midiMonHead + 1u) & (MIDI_MON_RING_SIZE - 1u);
    if(next == midiMonTail)
        midiMonTail = (midiMonTail + 1u) & (MIDI_MON_RING_SIZE - 1u);
    midiMonRing[midiMonHead][0] = status;
    midiMonRing[midiMonHead][1] = data0;
    midiMonRing[midiMonHead][2] = data1;
    midiMonHead = next;
}

static const MidiMapEntry* MidiUserMapFind(uint8_t channel,
                                           uint8_t kind,
                                           uint8_t number)
{
    for(uint8_t i = 0; i < midiUserMapCount; ++i)
    {
        const MidiMapEntry& entry = midiUserMap[i];
        if(entry.channel == channel && entry.kind == kind
           && entry.number == number)
            return &entry;
    }
    return nullptr;
}

/* ═══════════════════════════════════════════════════════════════════
 *  22. BUILD RESPONSE
 * ═══════════════════════════════════════════════════════════════════ */
static void BuildResponse(uint8_t cmd, uint16_t seq,
                          const uint8_t* payload, uint16_t payloadLen)
{
    if(payloadLen > (TX_BUF_SIZE - sizeof(SPIPacketHeader))){
        spiErrCnt++;
        pendingTxLen = 0;
        pendingResponse = false;
        return;
    }
    SPIPacketHeader* r = (SPIPacketHeader*)txBuf;
    r->magic    = SPI_MAGIC_RESP;
    r->cmd      = cmd;
    r->length   = payloadLen;
    r->sequence = seq;
    r->checksum = payloadLen ? crc16(payload, payloadLen) : 0;
    if(payloadLen && payload) memcpy(txBuf + 8, payload, payloadLen);
    pendingTxLen    = 8 + payloadLen;
    pendingResponse = true;
    /* NUNCA transmitir desde ISR — se hace en main loop */
}

/* Forward declaration — definida more adelante en sección SD */
static bool LoadWavToPad(const char* filepath, uint8_t padIdx);
static int  GuessPadFromFilename(const char* fname);
static bool isWavFile(const char* fname);
static bool SavePodConfigToSD();
static bool LoadPodConfigFromSD();
static uint8_t FillMissingCanonicalPadsFromFamilies(uint8_t startPad, uint8_t maxPads,
                                                     const char* kitPath = nullptr);

static void BuildPodState(PodStatePayload& state)
{
    memset(&state, 0, sizeof(state));
    state.config = podConfig;
    state.knob1 = podKnobRaw[0];
    state.knob2 = podKnobRaw[1];
    state.encoderPosition = podEncoderPosition;
    state.buttons = podButtonBits;
    state.buttonPressEvents = podButtonPressEvents;
    state.selectedPad = podSelectedPad;
    memcpy(&state.led1R, podLedRgb[0], 3);
    memcpy(&state.led2R, podLedRgb[1], 3);
    state.masterVolume = podCurrentMasterVolume;
    state.seqVolume = podCurrentSeqVolume;
    state.liveVolume = podCurrentLiveVolume;
    state.delayMixValue = static_cast<uint8_t>(clampF(delayMix, 0.0f, 1.0f) * 127.0f + 0.5f);
    state.reverbMixValue = static_cast<uint8_t>(clampF(reverbMix, 0.0f, 1.0f) * 127.0f + 0.5f);
    state.flangerDepthValue = static_cast<uint8_t>(
        clampF(flangerDepth, 0.0f, 1.0f) * 127.0f + 0.5f);
    state.phaserDepthValue = static_cast<uint8_t>(
        clampF(phaserDepth, 0.0f, 1.0f) * 127.0f + 0.5f);
    state.wavefolderValue = static_cast<uint8_t>(
        clampF((waveFolderGain - 1.0f) / 9.0f, 0.0f, 1.0f) * 127.0f + 0.5f);
    const float crushBits = clampF((16.0f - static_cast<float>(gFilterBitDepth))
                                   / 10.0f, 0.0f, 1.0f);
    const float crushRate = gFilterSrReduce == 0 ? 0.0f
        : clampF(logf(clampF(static_cast<float>(gFilterSrReduce), 4000.0f,
                             42000.0f) / 42000.0f)
                 / logf(4000.0f / 42000.0f), 0.0f, 1.0f);
    state.crushValue = static_cast<uint8_t>(
        (crushBits > crushRate ? crushBits : crushRate) * 127.0f + 0.5f);
    state.filterType = gFilterType;
    state.bitDepth = gFilterBitDepth;
    state.distortionPct = static_cast<uint8_t>(
        clampF(gFilterDist, 0.0f, 1.0f) * 100.0f + 0.5f);
    state.cutoffHz = static_cast<uint16_t>(
        clampF(gFilterCutoff, 20.0f, 20000.0f) + 0.5f);
    state.resonanceX10 = static_cast<uint16_t>(
        clampF(gFilterQ, 0.3f, 40.0f) * 10.0f + 0.5f);
    state.sampleRateHz = static_cast<uint16_t>(gFilterSrReduce > 48000u
        ? 0u : gFilterSrReduce);
    state.fxActiveBits = (delayActive ? 1u : 0u)
                       | (reverbActive ? 2u : 0u)
                       | (flangerActive ? 4u : 0u)
                       | (phaserActive ? 8u : 0u)
                       | (waveFolderGain > 1.01f ? 16u : 0u)
                       | ((gFilterBitDepth < 16 || gFilterSrReduce > 0) ? 32u : 0u)
                       | (gFilterType != FTYPE_NONE ? 64u : 0u)
                       | (gFilterDist > 0.0001f ? 128u : 0u);
    state.reservedFx = (autowahActive ? POD_FX_EXTRA_AUTOWAH : 0u)
                     | (beatRepActive ? POD_FX_EXTRA_BEAT_REPEAT : 0u)
                     | (tapeStopActive ? POD_FX_EXTRA_TAPE_STOP : 0u)
                     | ((stereoWidth < 0.99f || stereoWidth > 1.01f)
                        ? POD_FX_EXTRA_STEREO_WIDTH : 0u);
    state.bpmX10 = podCurrentBpmX10;
    state.playing = dseq.playing ? 1u : 0u;
    state.sdPresent = sdPresent ? 1u : 0u;
    for(uint8_t track = 0; track < DSQ_TRACKS; track++)
        if(sampleLoaded[track]) state.sampleMask |= (1u << track);
    state.revision = podStateRevision;
}

static void ValidatePodConfig(PodConfigPayload& config)
{
    config.version = POD_CONFIG_VERSION;
    uint8_t* functions[] = {
        &config.button1Function, &config.button2Function,
        &config.knob1Function, &config.knob2Function,
        &config.encoderFunction, &config.encoderButtonFunction,
        &config.rotary1Function, &config.rotary2Function,
        &config.rotary3Function, &config.rotary4Function,
        &config.selectorFunction, &config.faderFunction
    };
    for(uint8_t i = 0; i < 12; i++)
    {
        if(*functions[i] >= POD_FUNC_COUNT) *functions[i] = POD_FUNC_NONE;
        if(*functions[i] == POD_FUNC_NONE) continue;
        for(uint8_t previous = 0; previous < i; previous++)
        {
            if(PodFunctionsConflict(*functions[previous], *functions[i]))
            {
                *functions[i] = POD_FUNC_NONE;
                break;
            }
        }
    }
    if(config.led1Function >= POD_LED_COUNT) config.led1Function = POD_LED_FIXED;
    if(config.led2Function >= POD_LED_COUNT) config.led2Function = POD_LED_FIXED;
}

/* ═══════════════════════════════════════════════════════════════════
 *  23. PROCESS COMMAND  (ALL RED808 commands)
 * ═══════════════════════════════════════════════════════════════════ */
static void ProcessCommand()
{
    SPIPacketHeader* hdr = (SPIPacketHeader*)rxBuf;
    uint8_t* p = rxBuf + 8;
    uint16_t len = hdr->length;
    if(len > (RX_BUF_SIZE - sizeof(SPIPacketHeader))){
        spiErrCnt++;
        return;
    }

    /* CRC check (skip for PING) */
    if(!kBypassIncomingCrc && hdr->cmd != CMD_PING && len > 0){
        uint16_t calc = crc16(p, len);
        if(calc != hdr->checksum){ spiErrCnt++; return; }
    }
    spiPktCnt++;
    spiLastPacketMs = hw.system.GetNow();

    /* Showcase is a dedicated presentation image. Keep transport, notes and
     * sound-profile writes from the Master from becoming a second performance;
     * SD/sample loading, diagnostics, mixer and normal query traffic remain
     * available. Position queries are intentionally not blocked. */
    if(ShowcaseBlocksMasterCommand(hdr->cmd)) return;

    switch(hdr->cmd){

    /* ════════════════════════════════════════════
     *  PING
     * ════════════════════════════════════════════ */
    case CMD_PING: {
        LinkHealthResponse pong = {};
        if(len >= 4) memcpy(&pong.echoMs, p, 4);
        pong.uptimeMs = hw.system.GetNow();
        pong.protocolVersion = RED808_PROTOCOL_VERSION;
        pong.capabilityFlags = RED808_CAP_EXTENDED_PONG
                             | RED808_CAP_USB_RX_DIAGNOSTICS
                             | RED808_CAP_MIDI_MONITOR;
        pong.rxDrops = usbRxDrops;
        pong.protocolErrors = spiErrCnt;
        BuildResponse(CMD_PING, hdr->sequence,
                      reinterpret_cast<const uint8_t*>(&pong), sizeof(pong));
        return;
    }

    /* ════════════════════════════════════════════
     *  TRIGGERS
     * ════════════════════════════════════════════ */
    case CMD_TRIGGER_LIVE:
        if(len >= 2){
            uint8_t pad = p[0];
            uint8_t vel = p[1];
            if(kAcceptOneBasedPadIndex && pad > 0) pad -= 1;
            int8_t livEng = (pad < DSQ_TRACKS) ? dsqTrackEngine[pad] : -1;
            if(livEng >= 0 && livEng < SYNTH_ENGINE_COUNT){
                /* Synth engine activo: disparar synth, NO sampler.
                 * Queued (except PHYS/NOISE — see the AudioCmd scope note
                 * above TriggerPad's definition) so the actual Trigger()/
                 * NoteOn() call happens on the audio thread, not here. */
                float fvel = clampF(vel / 127.0f, 0.0f, 1.0f);
                const uint8_t qvel = (uint8_t)(fvel * 127.0f + 0.5f);
                AudioCmd cmd{};
                cmd.engine = (uint8_t)livEng;
                cmd.velocity = qvel;
                switch(livEng){
                    case SYNTH_ENGINE_808:
                        if(pad < 16){
                            cmd.type = AUDIO_CMD_SYNTH_TRIGGER;
                            cmd.instrument = padTo808[pad];
                            AudioCmdPush(cmd);
                        }
                        break;
                    case SYNTH_ENGINE_909:
                        if(pad < 16){
                            cmd.type = AUDIO_CMD_SYNTH_TRIGGER;
                            cmd.instrument = padTo909[pad];
                            AudioCmdPush(cmd);
                        }
                        break;
                    case SYNTH_ENGINE_505:
                        if(pad < 16){
                            cmd.type = AUDIO_CMD_SYNTH_TRIGGER;
                            cmd.instrument = padTo505[pad];
                            AudioCmdPush(cmd);
                        }
                        break;
                    case SYNTH_ENGINE_303:
                        cmd.type = AUDIO_CMD_SYNTH_NOTE_ON;
                        cmd.note = (pad < 16) ? padTo303Midi[pad] : 48;
                        cmd.accent = (fvel > 0.85f) ? 1 : 0;
                        AudioCmdPush(cmd);
                        break;
                    case SYNTH_ENGINE_WTOSC:
                        cmd.type = AUDIO_CMD_SYNTH_NOTE_ON;
                        cmd.note = (pad < 16) ? trackWtNote[pad] : 60;
                        AudioCmdPush(cmd);
                        break;
                    case SYNTH_ENGINE_SH101:             /* I1 */
                        cmd.type = AUDIO_CMD_SYNTH_NOTE_ON;
                        cmd.note = (pad < 16) ? trackSH101Note[pad] : 60;
                        AudioCmdPush(cmd);
                        break;
                    case SYNTH_ENGINE_FM2OP:             /* I2 */
                        cmd.type = AUDIO_CMD_SYNTH_NOTE_ON;
                        cmd.note = (pad < 16) ? trackFM2OpNote[pad] : 60;
                        AudioCmdPush(cmd);
                        break;
                    case SYNTH_ENGINE_PHYS:
                        cmd.type = AUDIO_CMD_PHYS_NOISE;
                        cmd.note = (pad < 16) ? trackWtNote[pad] : 60;
                        AudioCmdPush(cmd);
                        break;
                    case SYNTH_ENGINE_NOISE:
                        cmd.type = AUDIO_CMD_PHYS_NOISE;
                        cmd.note = (pad < 16) ? trackWtNote[pad] : 60;
                        AudioCmdPush(cmd);
                        break;
                }
            } else {
                /* Modo sampler (por defecto) */
                AudioCmd cmd{};
                cmd.type = AUDIO_CMD_TRIGGER_PAD;
                cmd.pad = pad;
                cmd.velocity = vel;
                cmd.trkVol = 100;
                cmd.sourceVolume = liveVolume;
                cmd.sourcePitch = livePitch;
                cmd.liveSource = 1;
                AudioCmdPush(cmd);
                /* DIAG: si no hay sample cargado, fallback a snare 808 (audible y distinto del kick)
                 * Confirma que el problema es sampleLoaded=false, no el SPI. */
                if(pad < MAX_PADS && !sampleLoaded[pad]){
                    AudioCmd fallback{};
                    fallback.type = AUDIO_CMD_SYNTH_TRIGGER;
                    fallback.engine = SYNTH_ENGINE_808;
                    fallback.instrument = TR808::INST_SNARE;
                    fallback.velocity = (uint8_t)(clampF(vel / 127.0f, 0.1f, 1.0f)
                                                  * 127.0f + 0.5f);
                    AudioCmdPush(fallback);
                }
            }
            spiLastTriggerMs = hw.system.GetNow();
        }
        break;

    case CMD_TRIGGER_SEQ:
        if(len >= 8){
            uint8_t pad = p[0];
            if(kAcceptOneBasedPadIndex && pad > 0) pad -= 1;
            uint32_t maxS = 0; memcpy(&maxS, p + 4, 4);
            /* Si el track tiene un synth engine asignado, enrutar al synth
             * en vez de al sampler (mismo comportamiento que CMD_TRIGGER_LIVE).
             * Sin esto, los pads con 303/WT/SH101/FM2 no sonarian en el
             * sequencer del Master. */
            int8_t seqEng = (pad < DSQ_TRACKS) ? dsqTrackEngine[pad] : -1;
            if(seqEng >= 0 && seqEng < SYNTH_ENGINE_COUNT){
                /* Same queueing as CMD_TRIGGER_LIVE — see the note there. */
                float fvel = clampF(p[1] / 127.0f, 0.0f, 1.0f);
                const uint8_t qvel = (uint8_t)(fvel * 127.0f + 0.5f);
                AudioCmd cmd{};
                cmd.engine = (uint8_t)seqEng;
                cmd.velocity = qvel;
                switch(seqEng){
                    case SYNTH_ENGINE_808:
                        if(pad < 16){
                            cmd.type = AUDIO_CMD_SYNTH_TRIGGER;
                            cmd.instrument = padTo808[pad];
                            AudioCmdPush(cmd);
                        }
                        break;
                    case SYNTH_ENGINE_909:
                        if(pad < 16){
                            cmd.type = AUDIO_CMD_SYNTH_TRIGGER;
                            cmd.instrument = padTo909[pad];
                            AudioCmdPush(cmd);
                        }
                        break;
                    case SYNTH_ENGINE_505:
                        if(pad < 16){
                            cmd.type = AUDIO_CMD_SYNTH_TRIGGER;
                            cmd.instrument = padTo505[pad];
                            AudioCmdPush(cmd);
                        }
                        break;
                    case SYNTH_ENGINE_303:
                        cmd.type = AUDIO_CMD_SYNTH_NOTE_ON;
                        cmd.note = (pad < 16) ? padTo303Midi[pad] : 48;
                        cmd.accent = (fvel > 0.85f) ? 1 : 0;
                        AudioCmdPush(cmd);
                        break;
                    case SYNTH_ENGINE_WTOSC:
                        cmd.type = AUDIO_CMD_SYNTH_NOTE_ON;
                        cmd.note = (pad < 16) ? trackWtNote[pad] : 60;
                        AudioCmdPush(cmd);
                        break;
                    case SYNTH_ENGINE_SH101:
                        cmd.type = AUDIO_CMD_SYNTH_NOTE_ON;
                        cmd.note = (pad < 16) ? trackSH101Note[pad] : 60;
                        AudioCmdPush(cmd);
                        break;
                    case SYNTH_ENGINE_FM2OP:
                        cmd.type = AUDIO_CMD_SYNTH_NOTE_ON;
                        cmd.note = (pad < 16) ? trackFM2OpNote[pad] : 60;
                        AudioCmdPush(cmd);
                        break;
                    case SYNTH_ENGINE_PHYS:
                        cmd.type = AUDIO_CMD_PHYS_NOISE;
                        cmd.note = (pad < 16) ? trackWtNote[pad] : 60;
                        AudioCmdPush(cmd);
                        break;
                    case SYNTH_ENGINE_NOISE:
                        cmd.type = AUDIO_CMD_PHYS_NOISE;
                        cmd.note = (pad < 16) ? trackWtNote[pad] : 60;
                        AudioCmdPush(cmd);
                        break;
                }
            } else {
                AudioCmd cmd{};
                cmd.type = AUDIO_CMD_TRIGGER_PAD;
                cmd.pad = pad;
                cmd.velocity = p[1];
                cmd.trkVol = p[2];
                cmd.pan = (int8_t)p[3];
                cmd.maxSamples = maxS;
                cmd.sourceVolume = seqVolume;
                cmd.sourcePitch = 1.0f;
                AudioCmdPush(cmd);
                if(kTriggerSynthOnLiveCmd)
                    Synth808TriggerByPad(pad, clampF(p[1] / 127.0f, 0.0f, 1.0f));
            }
            spiLastTriggerMs = hw.system.GetNow();
        }
        break;

    case CMD_TRIGGER_STOP:
        if(len >= 1)
        {
            uint8_t pad = p[0];
            StopPadVoices(pad);
            if(pad < DSQ_TRACKS)
                ReleaseTrackEngine(pad, dsqTrackEngine[pad]);
        }
        break;

    case CMD_TRIGGER_STOP_ALL:
        for(int v = 0; v < MAX_VOICES; v++) voices[v].active = false;
        ReleaseAllSynthEngines();
        break;

    case CMD_TRIGGER_SIDECHAIN:
        if(len >= 3) scEnv = clampF(p[2] / 255.0f, 0.f, 1.f);
        break;

    /* ════════════════════════════════════════════
     *  VOLUME
     * ════════════════════════════════════════════ */
    case CMD_MASTER_VOLUME:
        if(len >= 1 && (podApplyingCommand || !PodOwnsFunction(POD_FUNC_MASTER_VOLUME))){
            podCurrentMasterVolume = p[0] > 150 ? 150 : p[0];
            if(!kForceMasterGainDebug) masterGain = VolumeByteToGain(podCurrentMasterVolume);
            podStateRevision++;
        }
        break;
    case CMD_SEQ_VOLUME:
        if(len >= 1 && (podApplyingCommand || !PodOwnsFunction(POD_FUNC_SEQ_VOLUME))){
            podCurrentSeqVolume = p[0] > 150 ? 150 : p[0];
            seqVolume = VolumeByteToGain(podCurrentSeqVolume);
            podStateRevision++;
        }
        break;
    case CMD_LIVE_VOLUME:
        if(len >= 1 && (podApplyingCommand || !PodOwnsFunction(POD_FUNC_LIVE_VOLUME))){
            podCurrentLiveVolume = p[0] > 150 ? 150 : p[0];
            liveVolume = VolumeByteToGain(podCurrentLiveVolume);
            podStateRevision++;
        }
        break;
    case CMD_TRACK_VOLUME:
        if(len >= 2 && p[0] < MAX_PADS) {
            uint8_t t = p[0];
            float oldGain = trackGain[t];
            float newGain = VolumeByteToGain(p[1]);
            trackGain[t] = newGain;
            /* Actualizar voces activas del pad (para LFO vol en tiempo real) */
            if(oldGain > 1e-6f) {
                float ratio = newGain / oldGain;
                float panF  = trackPanF[t];
                for(int v = 0; v < MAX_VOICES; v++) {
                    if(voices[v].active && voices[v].pad == t) {
                        voices[v].baseGain *= ratio;
                        voices[v].gainL = voices[v].baseGain * (1.0f - clampF(panF, 0.f, 1.f));
                        voices[v].gainR = voices[v].baseGain * (1.0f + clampF(panF, -1.f, 0.f));
                    }
                }
            }
        }
        break;
    case CMD_LIVE_PITCH:
        if(len >= 4){
            float pitch; memcpy(&pitch, p, 4);
            if(isfinite(pitch)){
                livePitch = clampF(pitch, 0.25f, 4.0f);
                for(int v = 0; v < MAX_VOICES; v++){
                    if(voices[v].active && voices[v].liveSource)
                        voices[v].speed = PadPlaybackSpeed(voices[v].pad, livePitch);
                }
            }
        }
        break;
    case CMD_TEMPO:
        if(len >= 4 && (podApplyingCommand || !PodOwnsFunction(POD_FUNC_TEMPO))){
            float bpm; memcpy(&bpm, p, 4);
            transportBpm = clampF(bpm, 40.0f, 300.0f);
            dseq.tempo = transportBpm;   /* sync DSQ clock */
            podCurrentBpmX10 = static_cast<uint16_t>(transportBpm * 10.0f + 0.5f);
            DsqUpdateSamplesPerStep();
            podStateRevision++;
        }
        break;

    /* ════════════════════════════════════════════
     *  GLOBAL FILTER (0x20-0x26)
     * ════════════════════════════════════════════ */
    case CMD_FILTER_SET:
        if(len >= 20){
            /* Layout = GlobalFilterPayload de protocol.h (master):
             *   [0] filterType  [1] distMode  [2] bitDepth  [3] reserved
             *   [4..7] cutoff   [8..11] resonance  [12..15] distortion
             *   [16..19] sampleRateReduce
             * Los offsets antiguos (cutoff en +2, Q en +6, bitDepth en +10)
             * NO coincidían con el struct del master: el cutoff aterrizaba
             * como float basura y bitDepth=0 pasaba sin clamp → BitCrush a
             * 0 bits = silencio total. Era el "se cuelga con filtros OUT". */
            uint8_t incomingType = p[0];
            uint8_t incomingDistMode = p[1];
            uint8_t incomingBitDepth = p[2];
            float incomingCutoff = 0.f, incomingQ = 0.f, incomingDist = 0.f;
            uint32_t incomingSrReduce = 0;
            memcpy(&incomingCutoff,   p + 4,  4);
            memcpy(&incomingQ,        p + 8,  4);
            memcpy(&incomingDist,     p + 12, 4);
            memcpy(&incomingSrReduce, p + 16, 4);
            if(podApplyingCommand || !PodOwnsFunction(POD_FUNC_FILTER_TYPE))
                gFilterType = incomingType;
            if(podApplyingCommand || !PodOwnsFunction(POD_FUNC_DISTORTION)){
                gFilterDistMode = incomingDistMode;
                gFilterDist = incomingDist;
            }
            if(podApplyingCommand || !PodOwnsBitDepth())
                gFilterBitDepth = incomingBitDepth;
            if(podApplyingCommand || !PodOwnsFunction(POD_FUNC_FILTER_CUTOFF))
                gFilterCutoff = incomingCutoff;
            if(podApplyingCommand || !PodOwnsFunction(POD_FUNC_FILTER_RESONANCE))
                gFilterQ = incomingQ;
            if(podApplyingCommand || !PodOwnsSampleRate()){
                gFilterSrReduce = incomingSrReduce;
                gSrPhase = 0;
                gSrPrimed = false;
            }
            /* Clamps defensivos: ningún payload puede matar el audio. */
            if(gFilterType > FTYPE_SVF_MORPH) gFilterType = FTYPE_NONE;
            if(gFilterDistMode > DMODE_FUZZ) gFilterDistMode = DMODE_SOFT;
            if(gFilterBitDepth < 4 || gFilterBitDepth > 16) gFilterBitDepth = 16;
            if(gFilterDist > 1.0f) gFilterDist *= 0.01f;
            gFilterDist = clampF(gFilterDist, 0.f, 1.f);
            if(gFilterSrReduce > (uint32_t)SAMPLE_RATE) gFilterSrReduce = 0;
            gFilterCutoff = clampF(gFilterCutoff, 20.f, 20000.f);
            gFilterQ      = (gFilterType == FTYPE_RESONANT) ? clampF(gFilterQ, 0.3f, 40.f) : clampF(gFilterQ, 0.3f, 28.f);
            if(gFilterType == FTYPE_LADDER){
                masterLadderL.SetFreq(gFilterCutoff);
                masterLadderR.SetFreq(gFilterCutoff);
                masterLadderL.SetRes(clampF(gFilterQ / 28.f, 0.f, 1.f));
                masterLadderR.SetRes(clampF(gFilterQ / 28.f, 0.f, 1.f));
            } else if((gFilterType >= FTYPE_SVF_LP && gFilterType <= FTYPE_SVF_BP)
                      || gFilterType == FTYPE_SVF_MORPH){
                masterSvfL.SetFreq(gFilterCutoff);
                masterSvfR.SetFreq(gFilterCutoff);
                masterSvfL.SetRes(clampF(gFilterQ / 28.f, 0.f, 1.f));
                masterSvfR.SetRes(clampF(gFilterQ / 28.f, 0.f, 1.f));
            } else {
                gFilterL.SetType(gFilterType, gFilterCutoff, gFilterQ,
                                 (float)SAMPLE_RATE, GlobalEqGainDb(gFilterType));
                gFilterR.SetType(gFilterType, gFilterCutoff, gFilterQ,
                                 (float)SAMPLE_RATE, GlobalEqGainDb(gFilterType));
                if(gFilterType == FTYPE_RESONANT){
                    gFilter2L.SetType(FTYPE_RESONANT, gFilterCutoff, gFilterQ, (float)SAMPLE_RATE);
                    gFilter2R.SetType(FTYPE_RESONANT, gFilterCutoff, gFilterQ, (float)SAMPLE_RATE);
                }
            }
            /* A full reconfigure (preset/kit recall) applies instantly — it
             * should not visibly "glide" in. Only a lone cutoff/resonance
             * tweak below is left to ease in via UpdateGlobalFilterSmoothing(). */
            gFilterCutoffSm = gFilterCutoff;
            gFilterQSm      = gFilterQ;
            podStateRevision++;
        }
        break;
    case CMD_FILTER_CUTOFF:
        /* A live cutoff sweep used to recompute the filter's coefficients
         * instantly on every raw step, which is what made a fast sweep
         * sound like a series of clicks instead of a glide ("muy brusco").
         * Only the target is set here; UpdateGlobalFilterSmoothing() eases
         * gFilterCutoffSm toward it once per audio block. */
        if(len >= 4 && (podApplyingCommand
           || !PodOwnsFunction(POD_FUNC_FILTER_CUTOFF))){
            memcpy(&gFilterCutoff, p, 4);
            gFilterCutoff = clampF(gFilterCutoff, 20.f, 20000.f);
            podStateRevision++;
        }
        break;
    case CMD_FILTER_RESONANCE:
        /* Same reasoning as CMD_FILTER_CUTOFF above — target only. */
        if(len >= 4 && (podApplyingCommand
           || !PodOwnsFunction(POD_FUNC_FILTER_RESONANCE))){
            memcpy(&gFilterQ, p, 4);
            gFilterQ = (gFilterType == FTYPE_RESONANT) ? clampF(gFilterQ, 0.3f, 40.f) : clampF(gFilterQ, 0.3f, 28.f);
            podStateRevision++;
        }
        break;
    case CMD_FILTER_MORPH:
        if(len >= 4){
            memcpy(&gFilterMorph, p, 4);
            gFilterMorph = clampF(gFilterMorph, 0.f, 1.f);
            podStateRevision++;
        }
        break;
    case CMD_FILTER_BITDEPTH:
        if(len >= 1 && (podApplyingCommand || !PodOwnsBitDepth())){
            gFilterBitDepth = (p[0] < 4) ? 4 : (p[0] > 16 ? 16 : p[0]);
            podStateRevision++;
        }
        break;
    case CMD_FILTER_DISTORTION:
        if(len >= 4 && (podApplyingCommand
           || !PodOwnsFunction(POD_FUNC_DISTORTION))){
            memcpy(&gFilterDist, p, 4);
            if(gFilterDist > 1.0f) gFilterDist *= 0.01f;
            gFilterDist = clampF(gFilterDist, 0.f, 1.f);
            podStateRevision++;
        }
        break;
    case CMD_FILTER_DIST_MODE:
        if(len >= 1 && (podApplyingCommand
           || !PodOwnsFunction(POD_FUNC_DISTORTION)))
            gFilterDistMode = p[0];
        break;
    case CMD_FILTER_SR_REDUCE:
        if(len >= 4 && (podApplyingCommand || !PodOwnsSampleRate())){
            memcpy(&gFilterSrReduce, p, 4);
            if(gFilterSrReduce > (uint32_t)SAMPLE_RATE) gFilterSrReduce = 0;
            gSrPhase = 0;
            gSrPrimed = false;
            podStateRevision++;
        }
        break;
    case CMD_MASTER_FX_ROUTE:
        if(len >= 2){
            if(bool* routed = GetMasterFxRouteFlag(p[0]))
                *routed = (p[1] != 0);
        }
        break;

    /* ════════════════════════════════════════════
     *  DELAY (0x30-0x33)
     * ════════════════════════════════════════════ */
    case CMD_DELAY_ACTIVE:
        if(len >= 1 && (podApplyingCommand || !PodOwnsFunction(POD_FUNC_DELAY_MIX))){
            delayActive = (p[0] != 0);
            podStateRevision++;
        }
        break;
    case CMD_DELAY_TIME:
        if(len >= 4){
            float ms; memcpy(&ms, p, 4);
            delayTime = clampF(ms, 10.f, 2000.f);
            masterDelay.SetDelay(delayTime / 1000.0f * (float)SAMPLE_RATE);
            masterDelayR.SetDelay(delayTime / 1000.0f * (float)SAMPLE_RATE);
        } else if(len >= 2){
            uint16_t ms16 = 0; memcpy(&ms16, p, 2);
            delayTime = clampF((float)ms16, 10.f, 2000.f);
            masterDelay.SetDelay(delayTime / 1000.0f * (float)SAMPLE_RATE);
            masterDelayR.SetDelay(delayTime / 1000.0f * (float)SAMPLE_RATE);
        }
        break;
    case CMD_DELAY_FEEDBACK:
        if(len >= 4){ float v; memcpy(&v, p, 4); delayFeedback = clampF(v, 0.f, 0.95f); }
        else if(len >= 1) delayFeedback = clampF(p[0] / 100.0f, 0.f, 0.95f);
        break;
    case CMD_DELAY_MIX:
        if(podApplyingCommand || !PodOwnsFunction(POD_FUNC_DELAY_MIX)){
            if(len >= 4){ float v; memcpy(&v, p, 4); delayMix = clampF(v, 0.f, 1.f); }
            else if(len >= 1) delayMix = p[0] / 100.0f;
            if(len >= 1) podStateRevision++;
        }
        break;

    /* ════════════════════════════════════════════
     *  PHASER (0x34-0x37)
     * ════════════════════════════════════════════ */
    case CMD_PHASER_ACTIVE:
        if(len >= 1 && (podApplyingCommand
           || !PodOwnsFunction(POD_FUNC_PHASER_DEPTH))){
            phaserActive = (p[0] != 0);
            podStateRevision++;
        }
        break;
    case CMD_PHASER_RATE:
        if(len >= 4){
            float v; memcpy(&v, p, 4); v = clampF(v, 0.1f, 10.f);
            masterPhaserL.SetFreq(v); masterPhaserR.SetFreq(v * 1.013f);
        } else if(len >= 1){
            float v = clampF(p[0] / 10.0f, 0.1f, 10.f);
            masterPhaserL.SetFreq(v); masterPhaserR.SetFreq(v * 1.013f);
        }
        break;
    case CMD_PHASER_DEPTH:
        if(podApplyingCommand || !PodOwnsFunction(POD_FUNC_PHASER_DEPTH)){
            float v = 0.f;
            if(len >= 4) memcpy(&v, p, 4);
            else if(len >= 1) v = p[0] / 100.0f;
            else break;
            v = clampF(v, 0.f, 1.f);
            phaserDepth = v;
            masterPhaserL.SetLfoDepth(v); masterPhaserR.SetLfoDepth(v);
            podStateRevision++;
        }
        break;
    case CMD_PHASER_FEEDBACK:
        if(len >= 4){
            float v; memcpy(&v, p, 4); v = clampF(v, 0.f, 0.95f);
            masterPhaserL.SetFeedback(v); masterPhaserR.SetFeedback(v);
        } else if(len >= 1){
            float v = clampF(p[0] / 100.0f, 0.f, 0.95f);
            masterPhaserL.SetFeedback(v); masterPhaserR.SetFeedback(v);
        }
        break;

    /* ════════════════════════════════════════════
     *  FLANGER (0x38-0x3C)
     * ════════════════════════════════════════════ */
    case CMD_FLANGER_ACTIVE:
        if(len >= 1 && (podApplyingCommand
           || !PodOwnsFunction(POD_FUNC_FLANGER_DEPTH))){
            flangerActive = (p[0] != 0);
            podStateRevision++;
        }
        break;
    case CMD_FLANGER_RATE:
        if(len >= 4){ float v; memcpy(&v, p, 4); flangerRate = clampF(v, 0.1f, 10.f); ConfigureMasterFlanger(); }
        else if(len >= 1) flangerRate = clampF(p[0] * 0.1f, 0.1f, 20.f);
        ConfigureMasterFlanger();
        break;
    case CMD_FLANGER_DEPTH:
        if(podApplyingCommand || !PodOwnsFunction(POD_FUNC_FLANGER_DEPTH)){
            if(len >= 4){ float v; memcpy(&v, p, 4); flangerDepth = clampF(v, 0.f, 1.f); }
            else if(len >= 1) flangerDepth = clampF(p[0] / 100.0f, 0.f, 1.f);
            ConfigureMasterFlanger();
            if(len >= 1) podStateRevision++;
        }
        break;
    case CMD_FLANGER_FEEDBACK:
        if(len >= 4){ float v; memcpy(&v, p, 4); flangerFb = clampF(v, 0.f, 0.95f); ConfigureMasterFlanger(); }
        else if(len >= 1) flangerFb = p[0] / 100.0f;
        ConfigureMasterFlanger();
        break;
    case CMD_FLANGER_MIX:
        if(len >= 4){ float v; memcpy(&v, p, 4); flangerMix = clampF(v, 0.f, 1.f); }
        else if(len >= 1) flangerMix = p[0] / 100.0f;
        break;

    /* ════════════════════════════════════════════
     *  COMPRESSOR (0x3D-0x42)
     * ════════════════════════════════════════════ */
    case CMD_COMP_ACTIVE:
        if(len >= 1) compActive = (p[0] != 0);
        break;
    case CMD_COMP_THRESHOLD:
        if(len >= 4){ float v; memcpy(&v, p, 4); masterComp.SetThreshold(clampF(v, -60.f, 0.f)); }
        else if(len >= 1) masterComp.SetThreshold(-((float)p[0]));
        break;
    case CMD_COMP_RATIO:
        if(len >= 4){ float v; memcpy(&v, p, 4); masterComp.SetRatio(clampF(v, 1.f, 20.f)); }
        else if(len >= 1) masterComp.SetRatio((float)p[0]);
        break;
    case CMD_COMP_ATTACK:
        if(len >= 4){ float v; memcpy(&v, p, 4); masterComp.SetAttack(clampF(v, 0.1f, 100.f) / 1000.f); }
        else if(len >= 1) masterComp.SetAttack((float)p[0] / 1000.0f);
        break;
    case CMD_COMP_RELEASE:
        if(len >= 4){ float v; memcpy(&v, p, 4); masterComp.SetRelease(clampF(v, 10.f, 500.f) / 1000.f); }
        else if(len >= 1) masterComp.SetRelease((float)p[0] / 1000.0f);
        break;
    case CMD_COMP_MAKEUP:
        if(len >= 4){ float v; memcpy(&v, p, 4); masterComp.SetMakeup(clampF(v, 0.f, 30.f)); }
        else if(len >= 1) masterComp.SetMakeup((float)p[0] / 10.0f);
        break;

    /* ════════════════════════════════════════════
     *  REVERB (0x43-0x46)
     * ════════════════════════════════════════════ */
    case CMD_REVERB_ACTIVE:
        if(len >= 1 && (podApplyingCommand || !PodOwnsFunction(POD_FUNC_REVERB_MIX))){
            reverbActive = (p[0] != 0);
            podStateRevision++;
        }
        break;
    case CMD_REVERB_FEEDBACK:
        if(len >= 4){
            float v; memcpy(&v, p, 4);
            reverbFeedback = clampF(v, 0.f, 0.95f);
            masterReverb.SetFeedback(reverbFeedback);
        } else if(len >= 1){
            reverbFeedback = clampF(p[0] / 100.0f, 0.f, 0.95f);
            masterReverb.SetFeedback(reverbFeedback);
        }
        break;
    case CMD_REVERB_LPFREQ:
        if(len >= 4){
            float v; memcpy(&v, p, 4);
            reverbLpFreq = clampF(v, 200.f, 12000.f);
            masterReverb.SetLpFreq(reverbLpFreq);
        } else if(len >= 2){
            uint16_t f = 0; memcpy(&f, p, 2);
            reverbLpFreq = (float)f;
            masterReverb.SetLpFreq(reverbLpFreq);
        }
        break;
    case CMD_REVERB_MIX:
        if(podApplyingCommand || !PodOwnsFunction(POD_FUNC_REVERB_MIX)){
            if(len >= 4){ float v; memcpy(&v, p, 4); reverbMix = clampF(v, 0.f, 1.f); }
            else if(len >= 1) reverbMix = p[0] / 100.0f;
            if(len >= 1) podStateRevision++;
        }
        break;

    /* ════════════════════════════════════════════
     *  CHORUS (0x47-0x4A)
     * ════════════════════════════════════════════ */
    case CMD_CHORUS_ACTIVE:
        if(len >= 1) chorusActive = (p[0] != 0);
        break;
    case CMD_CHORUS_RATE:
        if(len >= 4){
            float v; memcpy(&v, p, 4); v = clampF(v, 0.1f, 10.f);
            masterChorusL.SetLfoFreq(v); masterChorusR.SetLfoFreq(v * 1.013f);
        } else if(len >= 1){
            float v = clampF(p[0] / 10.0f, 0.1f, 10.f);
            masterChorusL.SetLfoFreq(v); masterChorusR.SetLfoFreq(v * 1.013f);
        }
        break;
    case CMD_CHORUS_DEPTH:
        if(len >= 4){
            float v; memcpy(&v, p, 4); v = clampF(v, 0.f, 1.f);
            masterChorusL.SetLfoDepth(v); masterChorusR.SetLfoDepth(v);
        } else if(len >= 1){
            float v = clampF(p[0] / 100.0f, 0.f, 1.f);
            masterChorusL.SetLfoDepth(v); masterChorusR.SetLfoDepth(v);
        }
        break;
    case CMD_CHORUS_MIX:
        if(len >= 4){ float v; memcpy(&v, p, 4); chorusMix = clampF(v, 0.f, 1.f); }
        else if(len >= 1) chorusMix = p[0] / 100.0f;
        break;

    /* ════════════════════════════════════════════
     *  TREMOLO (0x4B-0x4D)
     * ════════════════════════════════════════════ */
    case CMD_TREMOLO_ACTIVE:
        if(len >= 1) tremoloActive = (p[0] != 0);
        break;
    case CMD_TREMOLO_RATE:
        if(len >= 4){ float v; memcpy(&v, p, 4); masterTremolo.SetFreq(clampF(v, 0.1f, 20.f)); }
        else if(len >= 1) masterTremolo.SetFreq(p[0] / 10.0f);
        break;
    case CMD_TREMOLO_DEPTH:
        if(len >= 4){ float v; memcpy(&v, p, 4); masterTremolo.SetDepth(clampF(v, 0.f, 1.f)); }
        else if(len >= 1) masterTremolo.SetDepth(p[0] / 100.0f);
        break;

    /* ════════════════════════════════════════════
     *  WAVEFOLDER + LIMITER (0x4E-0x4F)
     * ════════════════════════════════════════════ */
    case CMD_WAVEFOLDER_GAIN:
        if(podApplyingCommand || !PodOwnsFunction(POD_FUNC_WAVEFOLDER_GAIN)){
            if(len >= 4){ float v; memcpy(&v, p, 4); waveFolderGain = clampF(v, 1.f, 10.f); }
            else if(len >= 1) waveFolderGain = clampF(p[0] / 10.0f, 1.f, 10.f);
            if(len >= 1) podStateRevision++;
        }
        break;
    case CMD_LIMITER_ACTIVE:
        if(len >= 1) limiterActive = (p[0] != 0);
        break;

    /* ════════════════════════════════════════════
     *  PER-TRACK FX (0x50-0x65)
     * ════════════════════════════════════════════ */
    case CMD_TRACK_FILTER:
        if(len >= 12){
            uint8_t t = p[0]; if(t >= MAX_PADS) break;
            uint8_t ftype = p[1];
            const bool typeChanged = (ftype != trkFilterType[t]);
            trkFilterType[t] = ftype;
            if(ftype) trkFxRouted[t] = true;   /* auto-enable per-track FX chain */
            float cut, res, gain = 0.f;
            memcpy(&cut, p + 4, 4);
            memcpy(&res, p + 8, 4);
            if(len >= 16) memcpy(&gain, p + 12, 4);
            trkFilterCut[t] = clampF(cut, 20.f, 20000.f);
            /* RESONANT permite Q hasta 40 para auto-oscilación */
            float qMax = (ftype == FTYPE_RESONANT) ? 40.f : 28.f;
            trkFilterQ[t]   = clampF(res, 0.3f, qMax);
            trkFilterGain[t] = gain;
            if(typeChanged || !ftype){
                /* Switching models (or turning off) applies instantly — a
                 * cutoff/Q tweak within the same type glides in instead, via
                 * UpdateTrackFilterSmoothing() once per audio block. */
                trkFilterCutSm[t] = trkFilterCut[t];
                trkFilterQSm[t]   = trkFilterQ[t];
                trkFilter[t].SetType(ftype, trkFilterCut[t], trkFilterQ[t], (float)SAMPLE_RATE, gain);
                if(ftype == FTYPE_RESONANT)
                    trkFilter2[t].SetType(FTYPE_RESONANT, trkFilterCut[t], trkFilterQ[t], (float)SAMPLE_RATE);
            }
        }
        break;
    case CMD_TRACK_CLEAR_FILTER:
        if(len >= 1 && p[0] < MAX_PADS){
            trkFilterType[p[0]] = 0;
            trkFilter[p[0]].Reset();
            trkFilter2[p[0]].Reset();
        }
        break;
    case CMD_TRACK_DISTORTION:
        /* PadDistortionPayload: [track, distMode, rsvd×2, float amount(4B)] — 8B */
        if(len >= 8 && p[0] < MAX_PADS){
            uint8_t t = p[0];
            trkDistMode[t] = p[1];                    // modo desde byte 1
            float d; memcpy(&d, p + 4, 4);            // amount desde bytes 4-7
            if(d > 1.0f) d /= 100.0f;                // normalizar porcentaje→fracción
            trkDistDrive[t] = clampF(d, 0.f, 1.f);
            if(trkDistDrive[t] > 0.001f) trkFxRouted[t] = true;
        } else if(len >= 2 && p[0] < MAX_PADS){
            trkDistDrive[p[0]] = p[1] / 255.0f;
            if(trkDistDrive[p[0]] > 0.001f) trkFxRouted[p[0]] = true;
        }
        break;
    case CMD_TRACK_BITCRUSH:
        if(len >= 2 && p[0] < MAX_PADS){
            trkBitDepth[p[0]] = (p[1] < 4) ? 4 : (p[1] > 16 ? 16 : p[1]);
            if(trkBitDepth[p[0]] < 16) trkFxRouted[p[0]] = true;
        }
        break;
    case CMD_TRACK_ECHO:
        if(len >= 16 && p[0] < MAX_PADS){
            uint8_t t = p[0];
            trkEchoActive[t] = (p[1] != 0);
            if(trkEchoActive[t]) trkFxRouted[t] = true;
            float timeMs, fb, mix;
            memcpy(&timeMs, p + 4, 4);
            memcpy(&fb,     p + 8, 4);
            memcpy(&mix,    p + 12, 4);
            /* ESP32 sends fb & mix as 0-100 percentage; normalise to 0.0-1.0 */
            if(fb   > 1.0f) fb  /= 100.0f;
            if(mix  > 1.0f) mix /= 100.0f;
            trkEchoDelay[t] = clampF(timeMs * (float)SAMPLE_RATE / 1000.f, 1.f, (float)(TRACK_ECHO_SIZE-1));
            trkEchoFb[t]    = clampF(fb, 0.f, 0.95f);
            trkEchoMix[t]   = clampF(mix, 0.f, 1.f);
        }
        break;
    case CMD_TRACK_FLANGER_FX:
        if(len >= 16 && p[0] < MAX_PADS){
            uint8_t t = p[0];
            trkFlgActive[t] = (p[1] != 0);
            if(trkFlgActive[t]) trkFxRouted[t] = true;
            float depth, rate, fb;
            memcpy(&depth, p + 4, 4);
            memcpy(&rate,  p + 8, 4);
            memcpy(&fb,    p + 12, 4);
            /* ESP32 sends percentage 0-100; normalise to 0.0-1.0 */
            if(depth > 1.0f) depth /= 100.0f;
            if(rate  > 1.0f) rate   = rate / 100.0f * 5.0f; /* 0-100% → 0-5 Hz */
            if(fb    > 1.0f) fb    /= 100.0f;
            trkFlgDepth[t] = clampF(depth, 0.f, 1.f);
            trkFlgRate[t]  = clampF(rate, 0.1f, 10.f);
            trkFlgFb[t]    = clampF(fb, 0.f, 0.95f);
            trkFlgMix[t]   = 0.5f;  /* fixed 50/50; payload has no mix field */
            ConfigureTrackFlanger(t);
        }
        break;
    case CMD_TRACK_COMPRESSOR:
        if(len >= 12 && p[0] < MAX_PADS){
            uint8_t t = p[0];
            trkCompActive[t] = (p[1] != 0);
            if(trkCompActive[t]) trkFxRouted[t] = true;
            float thresh, ratio;
            memcpy(&thresh, p + 4, 4);
            memcpy(&ratio,  p + 8, 4);
            /* ESP32 sends threshold in dB (e.g. -20); convert to linear 0.01-1.0 */
            if(thresh <= 0.f) thresh = powf(10.f, clampF(thresh, -60.f, 0.f) / 20.f);
            trkCompThresh[t] = clampF(thresh, 0.01f, 1.f);
            trkCompRatio[t]  = clampF(ratio, 1.f, 20.f);
            trkCompExp[t]    = 1.f - 1.f/trkCompRatio[t];
        }
        break;
    case CMD_TRACK_CLEAR_LIVE:
        if(len >= 1 && p[0] < MAX_PADS){
            uint8_t t = p[0];
            trkEchoActive[t] = false;
            trkFlgActive[t]  = false;
            trkCompActive[t] = false;
            trkFlanger[t].Init((float)SAMPLE_RATE);
            ConfigureTrackFlanger(t);
            memset(trkEchoBuf[t], 0, sizeof(trkEchoBuf[t]));
        }
        break;
    case CMD_TRACK_CLEAR_FX:
        if(len >= 1 && p[0] < MAX_PADS){
            uint8_t t = p[0];
            trkFxRouted[t]   = false;
            trkFilterType[t] = 0;  trkFilter[t].Reset();
            trkDistDrive[t]  = 0;  trkDistMode[t] = 0;
            trkBitDepth[t]   = 16;
            trkEchoActive[t] = false; trkEchoWp[t] = 0;
            trkFlgActive[t]  = false;
            trkFlanger[t].Init((float)SAMPLE_RATE);
            ConfigureTrackFlanger(t);
            trkCompActive[t] = false; trkCompEnv[t] = 0;
            trackReverbSend[t] = 0; trackDelaySend[t] = 0;
            trackChorusSend[t] = 0;
            trackPanF[t] = 0; trackMute[t] = false; trackSolo[t] = false;
            trkEqLowDb[t] = 0; trkEqMidDb[t] = 0; trkEqHighDb[t] = 0;
            trkLfoActive[t] = false;
            trkLfoWave[t]   = LFO_WAVE_SINE;
            trkLfoTarget[t] = LFO_TGT_GAIN;
            trkLfoRate[t]   = 1.0f;
            trkLfoDepth[t]  = 0.0f;
            trkLfoPhase[t]  = 0.0f;
            trkLfoSH[t]     = 0.0f;
            trkEnvAdActive[t] = false;
            trkEnvAttackMs[t] = 1.0f;
            trkEnvDecayMs[t]  = 250.0f;
            memset(trkEchoBuf[t], 0, sizeof(trkEchoBuf[t]));
        }
        break;

    /* ── Track Sends / Pan / Mute / Solo ── */
    case CMD_TRACK_REVERB_SEND:
        if(len >= 2 && p[0] < MAX_PADS)
            trackReverbSend[p[0]] = p[1] / 100.0f;
        break;
    case CMD_TRACK_DELAY_SEND:
        if(len >= 2 && p[0] < MAX_PADS)
            trackDelaySend[p[0]] = p[1] / 100.0f;
        break;
    case CMD_TRACK_CHORUS_SEND:
        if(len >= 2 && p[0] < MAX_PADS)
            trackChorusSend[p[0]] = p[1] / 100.0f;
        break;
    case CMD_TRACK_PAN:
        if(len >= 2 && p[0] < MAX_PADS) {
            uint8_t t = p[0];
            trackPanF[t] = (int8_t)p[1] / 100.0f;
            float panF = trackPanF[t];
            /* Actualizar voces activas del pad (para LFO pan en tiempo real) */
            for(int v = 0; v < MAX_VOICES; v++) {
                if(voices[v].active && voices[v].pad == t) {
                    voices[v].gainL = voices[v].baseGain * (1.0f - clampF(panF, 0.f, 1.f));
                    voices[v].gainR = voices[v].baseGain * (1.0f + clampF(panF, -1.f, 0.f));
                }
            }
        }
        break;
    case CMD_TRACK_MUTE:
        if(len >= 2 && p[0] < MAX_PADS)
            trackMute[p[0]] = (p[1] != 0);
        break;
    case CMD_TRACK_SOLO:
        if(len >= 2 && p[0] < MAX_PADS){
            trackSolo[p[0]] = (p[1] != 0);
            anySolo = false;
            for(int i = 0; i < MAX_PADS; i++)
                if(trackSolo[i]){ anySolo = true; break; }
        }
        break;
    case CMD_TRACK_MUTE_MASK:
        if(len >= 2){
            const uint16_t mask = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
            for(int i = 0; i < MAX_PADS; ++i){
                const bool muted = (mask & ((uint16_t)1u << i)) != 0;
                trackMute[i] = muted;
                if(i < DSQ_TRACKS) dseq.trackMuted[i] = muted;
            }
        }
        break;
    case CMD_TRACK_SOLO_MASK:
        if(len >= 2){
            const uint16_t mask = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
            anySolo = mask != 0;
            for(int i = 0; i < MAX_PADS; ++i)
                trackSolo[i] = (mask & ((uint16_t)1u << i)) != 0;
        }
        break;

    /* ── Track EQ 3-band ── */
    case CMD_TRACK_EQ_LOW:
        if(len >= 2 && p[0] < MAX_PADS){
            trkEqLowDb[p[0]] = (int8_t)p[1];
            trkEqLow[p[0]].SetType(FTYPE_LOWSHELF, 200.f, 0.707f, (float)SAMPLE_RATE,
                                    (float)(int8_t)p[1]);
            if((int8_t)p[1] != 0) trkFxRouted[p[0]] = true;
        }
        break;
    case CMD_TRACK_EQ_MID:
        if(len >= 2 && p[0] < MAX_PADS){
            trkEqMidDb[p[0]] = (int8_t)p[1];
            trkEqMid[p[0]].SetType(FTYPE_PEAKING, 1000.f, 1.0f, (float)SAMPLE_RATE,
                                   (float)(int8_t)p[1]);
            if((int8_t)p[1] != 0) trkFxRouted[p[0]] = true;
        }
        break;
    case CMD_TRACK_EQ_HIGH:
        if(len >= 2 && p[0] < MAX_PADS){
            trkEqHighDb[p[0]] = (int8_t)p[1];
            trkEqHigh[p[0]].SetType(FTYPE_HIGHSHELF, 4000.f, 0.707f, (float)SAMPLE_RATE,
                                    (float)(int8_t)p[1]);
            if((int8_t)p[1] != 0) trkFxRouted[p[0]] = true;
        }
        break;
    case CMD_TRACK_FX_ROUTE:
        if(len >= 2 && p[0] < MAX_PADS)
            trkFxRouted[p[0]] = (p[1] != 0);
        break;

    /* ── Track Phaser / Tremolo / Pitch / Gate ── */
    case CMD_TRACK_PHASER:
        /* NO IMPLEMENTADO: phaser dedicado por track requiere allpass
         * chain por canal, demasiado costoso con MAX_PADS=16.
         * El phaser maestro global (0x35) sí está activo.         */
        break;
    case CMD_TRACK_TREMOLO:
        /* LFO interno por track (Daisy soberana en modulación)
         * Legacy payload (4B): [track, active, rateByte, depthByte]
         * Extended payload (12B): [track,active,wave,target, rate(float), depth(float)] */
        if(len >= 4 && p[0] < MAX_PADS){
            uint8_t t = p[0];
            trkLfoActive[t] = (p[1] != 0);

            if(len >= 12){
                trkLfoWave[t] = (p[2] > LFO_WAVE_SH) ? LFO_WAVE_SH : p[2];
                trkLfoTarget[t] = (p[3] > LFO_TGT_FILTER) ? LFO_TGT_FILTER : p[3];
                float rate = 0.0f, depth = 0.0f;
                memcpy(&rate,  p + 4, 4);
                memcpy(&depth, p + 8, 4);
                /* ESP32 may send depth as percentage 0-100; normalise */
                if(depth > 1.0f) depth /= 100.0f;
                trkLfoRate[t]  = clampF(rate, 0.05f, 40.0f);
                trkLfoDepth[t] = clampF(depth, 0.0f, 1.0f);
            } else {
                /* Legacy: sine + gain target */
                trkLfoWave[t]   = LFO_WAVE_SINE;
                trkLfoTarget[t] = LFO_TGT_GAIN;
                trkLfoRate[t]   = clampF(p[2] * 0.1f, 0.05f, 40.0f);
                trkLfoDepth[t]  = clampF(p[3] / 100.0f, 0.0f, 1.0f);
            }
        }
        break;
    case CMD_TRACK_PITCH:
        /* Pitch shift por track en centésimas (-1200..+1200)
         * Payload: [uint8_t track, uint8_t reserved, int16_t cents] (4 bytes) */
        if(len >= 4 && p[0] < MAX_PADS){
            uint8_t t = p[0];
            int16_t cents = 0;
            memcpy(&cents, p + 2, 2);
            trkPitchCents[t] = cents;
            /* Actualizar voces activas del pad en tiempo real (LFO modulation) */
            for(int v = 0; v < MAX_VOICES; v++){
                if(voices[v].active && voices[v].pad == (uint8_t)t)
                    voices[v].speed = PadPlaybackSpeed(t, voices[v].liveSource ? livePitch : 1.0f);
            }
        }
        break;
    case CMD_TRACK_GATE:
        /* Track AD gate/envelope para sampler voices
         * Legacy (2B): [track, active]
         * Extended (10B): [track, active, attackMs(float), decayMs(float)] */
        if(len >= 2 && p[0] < MAX_PADS){
            uint8_t t = p[0];
            trkEnvAdActive[t] = (p[1] != 0);

            if(len >= 10){
                float atkMs = 1.0f, decMs = 250.0f;
                memcpy(&atkMs, p + 2, 4);
                memcpy(&decMs, p + 6, 4);
                trkEnvAttackMs[t] = clampF(atkMs, 0.0f, 2000.0f);
                trkEnvDecayMs[t]  = clampF(decMs, 1.0f, 8000.0f);
            } else if(len >= 6){
                uint16_t atkMs16 = 1, decMs16 = 250;
                memcpy(&atkMs16, p + 2, 2);
                memcpy(&decMs16, p + 4, 2);
                trkEnvAttackMs[t] = clampF((float)atkMs16, 0.0f, 2000.0f);
                trkEnvDecayMs[t]  = clampF((float)decMs16, 1.0f, 8000.0f);
            }
        }
        break;

    /* ════════════════════════════════════════════
     *  PER-PAD FX (0x70-0x7A)
     * ════════════════════════════════════════════ */
    case CMD_PAD_FILTER:
        if(len >= 12 && p[0] < MAX_PADS){
            uint8_t pad = p[0];
            uint8_t newType = p[1];
            float cut, res, gain = 0.f;
            memcpy(&cut, p + 4, 4);
            memcpy(&res, p + 8, 4);
            if(len >= 16) memcpy(&gain, p + 12, 4);
            cut = clampF(cut, 20.f, 20000.f);
            res = clampF(res, 0.3f, 10.f);
            const bool typeChanged = (newType != padFilterType[pad]);
            padFilterType[pad] = newType;
            padFilterCut[pad]  = cut;
            padFilterQ[pad]    = res;
            padFilterGain[pad] = gain;
            if(typeChanged || !newType){
                /* Switching models (or turning off) applies instantly — a
                 * cutoff/Q tweak within the same type glides in instead, via
                 * UpdatePadFilterSmoothing() once per audio block. */
                padFilterCutSm[pad] = cut;
                padFilterQSm[pad]   = res;
                padFilter[pad].SetType(newType, cut, res, (float)SAMPLE_RATE, gain);
            }
        }
        break;
    case CMD_PAD_CLEAR_FILTER:
        if(len >= 1 && p[0] < MAX_PADS){
            padFilterType[p[0]] = 0;
            padFilter[p[0]].Reset();
        }
        break;
    case CMD_PAD_DISTORTION:
        /* PadDistortionPayload: [pad, distMode, rsvd×2, float amount(4B)] — 8B */
        if(len >= 8 && p[0] < MAX_PADS){
            uint8_t pad2 = p[0];
            padDistMode[pad2] = p[1];                  // modo desde byte 1
            float d; memcpy(&d, p + 4, 4);             // amount desde bytes 4-7
            if(d > 1.0f) d /= 100.0f;                 // normalizar porcentaje→fracción
            padDistDrive[pad2] = clampF(d, 0.f, 1.f);
        } else if(len >= 2 && p[0] < MAX_PADS){
            padDistDrive[p[0]] = p[1] / 255.0f;
        }
        break;
    case CMD_PAD_BITCRUSH:
        if(len >= 2 && p[0] < MAX_PADS)
            padBitDepth[p[0]] = (p[1] < 4) ? 4 : (p[1] > 16 ? 16 : p[1]);
        break;
    case CMD_PAD_LOOP:
        if(len >= 2 && p[0] < MAX_PADS)
            padLoop[p[0]] = (p[1] != 0);
        break;
    case CMD_PAD_REVERSE:
        if(len >= 2 && p[0] < MAX_PADS)
            padReverse[p[0]] = (p[1] != 0);
        break;
    case CMD_PAD_PITCH:
        if(len >= 3 && p[0] < MAX_PADS){
            uint8_t pad = p[0];
            int16_t cents = 0; memcpy(&cents, p + 1, 2);
            padPitch[pad] = powf(2.0f, cents / 1200.0f);
            for(int v = 0; v < MAX_VOICES; v++){
                if(voices[v].active && voices[v].pad == pad)
                    voices[v].speed = PadPlaybackSpeed(pad, voices[v].liveSource ? livePitch : 1.0f);
            }
        }
        break;
    case CMD_PAD_STUTTER:
        if(len >= 4 && p[0] < MAX_PADS){
            padStutterOn[p[0]] = (p[1] != 0);
            uint16_t ival; memcpy(&ival, p + 2, 2);
            padStutterIval[p[0]] = (ival < 20) ? 20 : (ival > 2000 ? 2000 : ival);
        }
        break;
    case CMD_PAD_SCRATCH:
        /* Removed from Daisy audio engine; command kept as protocol no-op. */
        break;
    case CMD_PAD_TURNTABLISM:
        /* Removed from Daisy audio engine; command kept as protocol no-op. */
        break;
    case CMD_PAD_CLEAR_FX:
        if(len >= 1 && p[0] < MAX_PADS){
            uint8_t pad = p[0];
            padFilterType[pad] = 0; padFilter[pad].Reset();
            padDistDrive[pad] = 0; padDistMode[pad] = 0; padBitDepth[pad] = 16;
            padLoop[pad] = false; padReverse[pad] = false; padPitch[pad] = 1.0f; trkPitchCents[pad] = 0;
            padStutterOn[pad] = false;
        }
        break;
    case CMD_PAD_TRIM:
        /* Non-destructive: only changes where TriggerPad starts/stops
         * reading sampleStorage[pad] for future voices. Already-playing
         * voices keep whatever window they were triggered with. */
        if(len >= 3 && p[0] < MAX_PADS){
            uint8_t pad = p[0];
            uint8_t startPct = p[1] > 100 ? 100 : p[1];
            uint8_t endPct   = p[2] > 100 ? 100 : p[2];
            padTrimStartPct[pad] = (float)startPct / 100.0f;
            padTrimEndPct[pad]   = (float)endPct   / 100.0f;
        }
        break;

    /* ════════════════════════════════════════════
     *  SIDECHAIN (0x90-0x91)
     * ════════════════════════════════════════════ */
    case CMD_SIDECHAIN_SET:
        if(len >= 20){
            float amount = 0.0f, attackK = 0.0f, releaseK = 0.0f;
            uint16_t dstMask = 0;
            memcpy(&dstMask, p + 2, 2);
            memcpy(&amount,   p + 4, 4);
            memcpy(&attackK,  p + 8, 4);
            memcpy(&releaseK, p + 12, 4);
            /* p+16: knee (ignored for now) */
            if(p[0] < MAX_PADS && isfinite(amount)
            && isfinite(attackK) && isfinite(releaseK)){
                scActive = true;
                scSrc = p[0];
                scDstMask = dstMask;
                scAmount = clampF(amount, 0.0f, 1.0f);
                scAttackK = clampF(attackK, 0.0f, 1.0f);
                scReleaseK = clampF(releaseK, 0.0f, 1.0f);
            } else {
                spiErrCnt++;
            }
        }
        break;
    case CMD_SIDECHAIN_CLEAR:
        scActive = false; scEnv = 0;
        break;

    /* ════════════════════════════════════════════
     *  SAMPLE TRANSFER (0xA0-0xA4)
     * ════════════════════════════════════════════ */
    case CMD_SAMPLE_BEGIN:
        if(len >= 12){
            uint8_t pad = p[0];
            if(pad < TOTAL_SAMPLE_SLOTS){
                uint32_t ts = 0; memcpy(&ts, p + 8, 4);
                sampleUploadReceivedBytes[pad] = 0;
                sampleUploadValid[pad] = false;
                if(ts == 0 || ts > MAX_SAMPLE_BYTES / 2)
                    break;
                if(pad < MAX_PADS){
                    StopPadVoices(pad);
                    Unbind909PcmPad(pad);
                    Unbind505PcmPad(pad);
                    sampleLoaded[pad] = false;
                    sampleLength[pad] = 0;
                    sampleTotalSamples[pad] = 0;
                    sampleRateHz[pad] = SAMPLE_RATE; /* SPI PCM is native-rate */
                    if(!AllocSampleStorage(pad, ts)){
                        padLoading[pad] = false;
                        break;
                    }
                    padLoading[pad] = true;
                    sampleTotalSamples[pad] = ts;
                } else {
                    uint8_t track = (uint8_t)(pad - MAX_PADS);
                    cleanTrackLoaded[track] = false;
                    cleanTrackLength[track] = 0;
                    cleanTrackTotalSamples[track] = 0;
                    cleanTrackPlayhead[track] = 0;
                    cleanTrackActive[track] = false;
                    if(!AllocCleanTrackStorage(track, ts)){
                        cleanTrackLoading[track] = false;
                        break;
                    }
                    cleanTrackLoading[track] = true;
                    cleanTrackTotalSamples[track] = ts;
                }
                sampleUploadValid[pad] = true;
            }
        }
        break;

    case CMD_SAMPLE_DATA:
        if(len >= 8){
            uint8_t pad = p[0];
            uint16_t chunkSize = 0; uint32_t offset = 0;
            memcpy(&chunkSize, p + 2, 2);
            memcpy(&offset,    p + 4, 4);
            if(pad < TOTAL_SAMPLE_SLOTS){
                int16_t* sampleData = nullptr;
                bool slotLoading = false;
                uint32_t slotCapacity = 0;
                uint32_t expectedSamples = 0;
                if(pad < MAX_PADS){
                    sampleData = SamplePtr(pad);
                    slotLoading = padLoading[pad];
                    slotCapacity = sampleCapacitySamples[pad];
                    expectedSamples = sampleTotalSamples[pad];
                } else {
                    uint8_t track = (uint8_t)(pad - MAX_PADS);
                    sampleData = CleanTrackPtr(track);
                    slotLoading = cleanTrackLoading[track];
                    slotCapacity = cleanTrackCapacitySamples[track];
                    expectedSamples = cleanTrackTotalSamples[track];
                }

                if(slotLoading){
                    const uint32_t expectedBytes = expectedSamples * 2u;
                    const bool headerValid = sampleData != nullptr
                                          && sampleUploadValid[pad]
                                          && chunkSize > 0
                                          && (chunkSize & 1u) == 0
                                          && (offset & 1u) == 0
                                          && chunkSize <= (uint16_t)(len - 8u)
                                          && offset == sampleUploadReceivedBytes[pad]
                                          && offset <= expectedBytes
                                          && (uint32_t)chunkSize <= (expectedBytes - offset);
                    if(headerValid){
                        const uint32_t startSample = offset / 2u;
                        const uint32_t numSamples  = chunkSize / 2u;
                        if(startSample <= slotCapacity
                        && numSamples <= (slotCapacity - startSample)){
                            memcpy(&sampleData[startSample], p + 8, chunkSize);
                            sampleUploadReceivedBytes[pad] += chunkSize;
                        } else {
                            sampleUploadValid[pad] = false;
                        }
                    } else {
                        sampleUploadValid[pad] = false;
                    }
                }
            }
        }
        break;

    case CMD_SAMPLE_END: {
        uint8_t pad = len >= 1 ? p[0] : 0xFFu;
        bool accepted = false;
        if(len >= 1){
            uint8_t status = (len >= 2) ? p[1] : 0;
            if(pad < MAX_PADS && padLoading[pad]){
                StopPadVoices(pad);
                const uint32_t expectedBytes = sampleTotalSamples[pad] * 2u;
                const bool uploadComplete = sampleUploadValid[pad]
                                         && sampleUploadReceivedBytes[pad] == expectedBytes;
                if(status == 0 && sampleTotalSamples[pad] > 0 && uploadComplete){
                    if(sampleTotalSamples[pad] > MAX_SAMPLE_BYTES / 2)
                        sampleTotalSamples[pad] = MAX_SAMPLE_BYTES / 2;
                    sampleLength[pad] = sampleTotalSamples[pad];
                    /* A trim window sized for the PREVIOUS file at this pad
                     * makes no sense against a new one of different length —
                     * reset to "no trim" (see padTrimStartPct comment). */
                    padTrimStartPct[pad] = 0.0f;
                    padTrimEndPct[pad] = 0.0f;
                    /* Every CMD_SAMPLE_DATA memcpy into sampleData[] and the
                     * sampleLength write above must be visible to the audio
                     * ISR before it sees sampleLoaded[pad]==true and starts
                     * reading this pad's samples (TriggerPad gates on this
                     * flag). Without the barrier a stale/partial read is
                     * possible even though both run on the same core, since
                     * neither sampleData[] nor sampleLength[] is volatile. */
                    __DMB();
                    sampleLoaded[pad] = true;
                    /* A WAV loaded into a LIVE pad always becomes its audible
                     * source. Otherwise the boot fallback 909/505 engine keeps
                     * winning and the valid sample appears to be silent. */
                    if(pad < DSQ_TRACKS)
                        dsqTrackEngine[pad] = -1;
                    if(synth505PcmMode && pad < 16){
                        int16_t* data = SamplePtr(pad);
                        if(data != nullptr)
                            synth505.SetPcmSample(padTo505[pad], data, sampleLength[pad], (float)SAMPLE_RATE);
                    }
                    if(synth909PcmMode && Is909PcmPad(pad)){
                        int16_t* data = SamplePtr(pad);
                        if(data != nullptr)
                            synth909.SetPcmSample(padTo909[pad], data, sampleLength[pad], (float)SAMPLE_RATE);
                    }
                    accepted = true;
                } else {
                    sampleLength[pad] = 0;
                    sampleLoaded[pad] = false;
                    FreeSampleStorage(pad);
                }
                padLoading[pad] = false;
                sampleUploadValid[pad] = false;
                sampleUploadReceivedBytes[pad] = 0;
            } else if(pad >= MAX_PADS && pad < TOTAL_SAMPLE_SLOTS && cleanTrackLoading[pad - MAX_PADS]) {
                uint8_t track = (uint8_t)(pad - MAX_PADS);
                const uint32_t expectedBytes = cleanTrackTotalSamples[track] * 2u;
                const bool uploadComplete = sampleUploadValid[pad]
                                         && sampleUploadReceivedBytes[pad] == expectedBytes;
                if(status == 0 && cleanTrackTotalSamples[track] > 0 && uploadComplete){
                    if(cleanTrackTotalSamples[track] > MAX_SAMPLE_BYTES / 2)
                        cleanTrackTotalSamples[track] = MAX_SAMPLE_BYTES / 2;
                    cleanTrackLength[track] = cleanTrackTotalSamples[track];
                    cleanTrackLoaded[track] = true;
                    accepted = true;
                } else {
                    cleanTrackLength[track] = 0;
                    cleanTrackLoaded[track] = false;
                    FreeCleanTrackStorage(track);
                }
                cleanTrackLoading[track] = false;
                sampleUploadValid[pad] = false;
                sampleUploadReceivedBytes[pad] = 0;
            }
        }
        /* Positive acknowledgement: the sender must not equate USB enqueue
         * success with a sample committed to Daisy SDRAM. */
        const uint8_t response[2] = {pad, (uint8_t)(accepted ? 1u : 0u)};
        BuildResponse(CMD_SAMPLE_END, hdr->sequence, response, sizeof(response));
        return;
    }

    case CMD_SAMPLE_UNLOAD:
        if(len >= 1 && p[0] < MAX_PADS){
            uint8_t pad = p[0];
            StopPadVoices(pad);
            Unbind909PcmPad(pad);
            Unbind505PcmPad(pad);
            padLoading[pad] = false;
            sampleLoaded[pad] = false;
            sampleLength[pad] = 0;
            sampleTotalSamples[pad] = 0;
            sampleRateHz[pad] = SAMPLE_RATE;
            sampleUploadValid[pad] = false;
            sampleUploadReceivedBytes[pad] = 0;
            padTrimStartPct[pad] = 0.0f;
            padTrimEndPct[pad] = 0.0f;
            FreeSampleStorage(pad);
        }
        break;

    case CMD_CLEAN_TRACK_ACTIVE:
        if(len >= 2 && p[0] < CLEAN_TRACK_COUNT){
            uint8_t track = p[0];
            cleanTrackEnabled[track] = (p[1] != 0);
            if(!cleanTrackEnabled[track]){
                cleanTrackActive[track] = false;
                cleanTrackPlayhead[track] = 0;
            } else if(cleanTrackLoaded[track]) {
                // Activate immediately so the stems-screen PLAY button auditions
                // the stem even when the sequencer transport is stopped. Playback
                // is one-shot: the mixer clears cleanTrackActive at end of buffer.
                cleanTrackPlayhead[track] = 0;
                cleanTrackActive[track] = true;
            }
        }
        break;

    case CMD_CLEAN_TRACK_MUTE:
        if(len >= 2 && p[0] < CLEAN_TRACK_COUNT)
            cleanTrackMuted[p[0]] = (p[1] != 0);
        break;

    case CMD_SAMPLE_UNLOAD_ALL:
        synth909.ClearPcmSamples();
        synth909PcmMode = false;
        synth505.ClearPcmSamples();
        synth505PcmMode = false;
        for(int i = 0; i < MAX_PADS; i++){
            padLoading[i] = false;
            sampleLoaded[i] = false;
            sampleLength[i] = 0;
            sampleTotalSamples[i] = 0;
            sampleRateHz[i] = SAMPLE_RATE;
            sampleUploadValid[i] = false;
            sampleUploadReceivedBytes[i] = 0;
            FreeSampleStorage((uint8_t)i);
        }
        for(int i = 0; i < CLEAN_TRACK_COUNT; i++){
            cleanTrackLoading[i] = false;
            cleanTrackLoaded[i] = false;
            cleanTrackLength[i] = 0;
            cleanTrackTotalSamples[i] = 0;
            cleanTrackPlayhead[i] = 0;
            cleanTrackActive[i] = false;
            cleanTrackEnabled[i] = true;
            cleanTrackMuted[i] = false;
            sampleUploadValid[MAX_PADS + i] = false;
            sampleUploadReceivedBytes[MAX_PADS + i] = 0;
            FreeCleanTrackStorage((uint8_t)i);
        }
        for(int v = 0; v < MAX_VOICES; v++) voices[v].active = false;
        break;

    /* ════════════════════════════════════════════
     *  SD CARD (0xB0-0xB9)
     * ════════════════════════════════════════════ */
    case CMD_SD_KIT_LIST: {
        SdKitListResponse resp;
        memset(&resp, 0, sizeof(resp));
        DIR dir; FILINFO fno;
        /* List kit folders inside /data (any directory counts as a kit) */
        char root[16];
        snprintf(root, sizeof(root), "%s", SD_DATA_ROOT);
        if(sdPresent && f_opendir(&dir, root) == FR_OK){
            while(f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != 0){
                if(!(fno.fattrib & AM_DIR)) continue;
                /* Skip single-instrument family folders (2-char names) and xtra */
                size_t nlen = strlen(fno.fname);
                bool isFamily = (nlen <= 2);
                bool isXtra   = (strcasecmp(fno.fname, "xtra") == 0);
                if(!isFamily && !isXtra && resp.count < 16){
                    CopyFixedString(resp.kits[resp.count], sizeof(resp.kits[resp.count]), fno.fname);
                    resp.count++;
                }
            }
            f_closedir(&dir);
        }
        BuildResponse(CMD_SD_KIT_LIST, hdr->sequence,
                      (uint8_t*)&resp, 1 + resp.count * 32);
        return;
    }

    case CMD_SD_LOAD_KIT: {
        if(len >= sizeof(SdLoadKitPayload)){
            SdLoadKitPayload lk;
            memcpy(&lk, p, sizeof(lk));
            lk.kitName[31] = 0;
            char path[96];
            if(!JoinPath(path, sizeof(path), SD_DATA_ROOT, lk.kitName))
                break;
            DIR dir; FILINFO fno;
            uint8_t padIdx = lk.startPad;
            uint8_t maxIdx = lk.startPad + lk.maxPads;
            if(maxIdx > MAX_PADS) maxIdx = MAX_PADS;
            bool canonicalLiveRange = (lk.startPad == 0 && lk.maxPads >= 16);

            /* ── Mute audio output completely during SD loading ── */
            kitMuteActive = true;

            PreparePadRangeForReload(lk.startPad, maxIdx);

            FRESULT openRes = FR_NO_PATH;
            if(sdPresent)
                openRes = f_opendir(&dir, path);
            if(sdPresent && openRes != FR_OK){
                char rootPath[96];
                if(JoinPath(rootPath, sizeof(rootPath), "/", lk.kitName)){
                    FRESULT rootRes = f_opendir(&dir, rootPath);
                    if(rootRes == FR_OK){
                        hw.PrintLine("SD: Kit '%s' using root fallback %s", lk.kitName, rootPath);
                        CopyFixedString(path, sizeof(path), rootPath);
                        openRes = FR_OK;
                    } else {
                        hw.PrintLine("SD: Kit '%s' not found (%s res=%d, %s res=%d)",
                                     lk.kitName, path, (int)openRes, rootPath, (int)rootRes);
                    }
                } else {
                    hw.PrintLine("SD: Kit '%s' path too long", lk.kitName);
                }
            }

            if(sdPresent && openRes == FR_OK){
                if(canonicalLiveRange){
                    bool padUsed[16] = {};
                    while(f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != 0){
                        if(fno.fattrib & AM_DIR) continue;
                        if(!isWavFile(fno.fname)) continue;

                        int pad = GuessPadFromFilename(fno.fname);
                        if(pad < 0 || pad >= 16 || padUsed[pad]) continue;

                        char fpath[160];
                        if(!JoinPath(fpath, sizeof(fpath), path, fno.fname)) continue;
                        if(LoadWavToPad(fpath, (uint8_t)pad))
                            padUsed[pad] = true;
                    }
                } else {
                    while(f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != 0
                          && padIdx < maxIdx){
                        if(fno.fattrib & AM_DIR) continue;
                        if(!isWavFile(fno.fname)) continue;
                        char fpath[160];
                        if(!JoinPath(fpath, sizeof(fpath), path, fno.fname)) continue;
                        if(LoadWavToPad(fpath, padIdx)) padIdx++;
                    }
                }
                f_closedir(&dir);

                if(canonicalLiveRange){
                    FillMissingCanonicalPadsFromFamilies(0, 16, path);
                    padIdx = 16;
                }

                CopyFixedString(currentKitName, sizeof(currentKitName), lk.kitName);
                hw.PrintLine("SD: Kit '%s' loaded pads %d-%d",
                               lk.kitName, lk.startPad, padIdx-1);
                /* Notify Master */
                uint32_t mask = 0;
                for(int i = lk.startPad; i < maxIdx; i++)
                    if(sampleLoaded[i]) mask |= (1u << i);
                uint8_t loadedCount = 0;
                for(int i = lk.startPad; i < maxIdx; i++)
                    if(sampleLoaded[i]) loadedCount++;
                if(loadedCount == 0)
                    hw.PrintLine("SD: WARN kit '%s' loaded 0 pads from %s", lk.kitName, path);
                PushEvent(EVT_SD_KIT_LOADED, loadedCount,
                          mask, lk.kitName);
            } else if(!sdPresent) {
                hw.PrintLine("SD: load kit '%s' ignored, SD not present", lk.kitName);
            }

            /* ── Clear padLoading for range and unmute audio ── */
            for(uint8_t _idx = lk.startPad; _idx < maxIdx; _idx++)
                padLoading[_idx] = false;
            kitMuteActive = false;
        }
        break;
    }

    case CMD_SD_STATUS: {
        SdStatusResponse resp;
        memset(&resp, 0, sizeof(resp));
        resp.present = sdPresent ? 1 : 0;
        for(int i = 0; i < MAX_PADS && i < 16; i++)
            if(sampleLoaded[i]) resp.samplesLoaded |= (1 << i);
        CopyFixedString(resp.currentKit, sizeof(resp.currentKit), currentKitName);
        BuildResponse(CMD_SD_STATUS, hdr->sequence,
                      (uint8_t*)&resp, sizeof(resp));
        return;
    }

    case CMD_SD_UNLOAD_KIT:
        synth909.ClearPcmSamples();
        synth909PcmMode = false;
        synth505.ClearPcmSamples();
        synth505PcmMode = false;
        for(int i = 0; i < MAX_PADS; i++){
            sampleLoaded[i] = false; sampleLength[i] = 0;
            sampleRateHz[i] = SAMPLE_RATE;
        }
        for(int v = 0; v < MAX_VOICES; v++) voices[v].active = false;
        PushEvent(EVT_SD_KIT_UNLOADED, 0, 0, currentKitName);
        currentKitName[0] = 0;
        break;

    case CMD_SD_GET_LOADED: {
        uint8_t resp[4] = {};
        for(int i = 0; i < MAX_PADS && i < 24; i++)
            if(sampleLoaded[i]) resp[i/8] |= (1 << (i%8));
        BuildResponse(CMD_SD_GET_LOADED, hdr->sequence, resp, 4);
        return;
    }

    case CMD_SD_LIST_FOLDERS: {
        /* List subdirectories inside /data — or, when a payload is given
         * (added for XTRA sample-pack paging), inside /data/<parent>
         * instead. Old callers (root kit browser, MIDI file browser) always
         * send a zero-length payload and keep listing /data exactly as
         * before this was added. */
        SdKitListResponse resp;   /* reuse: count + names[16][32] */
        memset(&resp, 0, sizeof(resp));
        char rootPath[96];
        if(len >= sizeof(SdListFilesPayload)){
            SdListFilesPayload pl;
            memcpy(&pl, p, sizeof(pl));
            pl.folder[31] = 0;
            if(!JoinPath(rootPath, sizeof(rootPath), SD_DATA_ROOT, pl.folder)){
                BuildResponse(CMD_SD_LIST_FOLDERS, hdr->sequence, (uint8_t*)&resp, 1);
                return;
            }
        } else {
            CopyFixedString(rootPath, sizeof(rootPath), SD_DATA_ROOT);
        }
        DIR dir; FILINFO fno;
        if(sdPresent && f_opendir(&dir, rootPath) == FR_OK){
            while(f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != 0){
                if((fno.fattrib & AM_DIR) && resp.count < 16){
                    CopyFixedString(resp.kits[resp.count], sizeof(resp.kits[resp.count]), fno.fname);
                    resp.count++;
                }
            }
            f_closedir(&dir);
        }
        BuildResponse(CMD_SD_LIST_FOLDERS, hdr->sequence,
                      (uint8_t*)&resp, 1 + resp.count * 32);
        return;
    }

    case CMD_SD_LIST_FILES: {
        /* List .wav files in a given subfolder of /data */
        SdListFilesResponse resp;
        memset(&resp, 0, sizeof(resp));
        if(len >= sizeof(SdListFilesPayload)){
            SdListFilesPayload pl;
            memcpy(&pl, p, sizeof(pl));
            pl.folder[31] = 0;
            char path[96];
            if(!JoinPath(path, sizeof(path), SD_DATA_ROOT, pl.folder)){
                BuildResponse(CMD_SD_LIST_FILES, hdr->sequence, (uint8_t*)&resp, 1);
                return;
            }
            DIR dir; FILINFO fno;
            if(sdPresent && f_opendir(&dir, path) == FR_OK){
                while(f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != 0
                      && resp.count < 20){
                    if(fno.fattrib & AM_DIR) continue;
                    size_t flen = strlen(fno.fname);
                    if(flen < 4) continue;
                    const char* ext = fno.fname + flen - 4;
                    if(ext[0]=='.' && (ext[1]=='w'||ext[1]=='W')){
                        CopyFixedString(resp.files[resp.count], sizeof(resp.files[resp.count]), fno.fname);
                        resp.count++;
                    }
                }
                f_closedir(&dir);
            }
        }
        BuildResponse(CMD_SD_LIST_FILES, hdr->sequence,
                      (uint8_t*)&resp, 1 + resp.count * 32);
        return;
    }

    case CMD_SD_FILE_INFO: {
        SdFileInfoResponse resp;
        memset(&resp, 0, sizeof(resp));
        if(len >= sizeof(SdFileInfoPayload)){
            SdFileInfoPayload pl;
            memcpy(&pl, p, sizeof(pl));
            pl.folder[31] = 0; pl.filename[31] = 0;
            char path[160];
            snprintf(path, sizeof(path), "%s/%s/%s",
                     SD_DATA_ROOT, pl.folder, pl.filename);
            FIL fil;
            if(sdPresent && f_open(&fil, path, FA_READ) == FR_OK){
                resp.sizeBytes = f_size(&fil);
                uint8_t wh[44]; UINT br;
                if(f_read(&fil, wh, 44, &br)==FR_OK && br>=44
                   && memcmp(wh,"RIFF",4)==0){
                    resp.channels       = wh[22];
                    resp.sampleRate     = wh[24]|(wh[25]<<8);
                    resp.bitsPerSample  = wh[34]|(wh[35]<<8);
                    uint32_t dataBytes  = resp.sizeBytes > 44 ? resp.sizeBytes-44 : 0;
                    uint32_t bytesPerSec= wh[28]|(wh[29]<<8)|(wh[30]<<16)|(wh[31]<<24);
                    if(bytesPerSec > 0)
                        resp.durationMs = (uint32_t)((uint64_t)dataBytes*1000/bytesPerSec);
                }
                f_close(&fil);
            }
        }
        BuildResponse(CMD_SD_FILE_INFO, hdr->sequence,
                      (uint8_t*)&resp, sizeof(resp));
        return;
    }

    case CMD_SD_LOAD_SAMPLE: {
        /* Load a specific .wav file into a specific pad */
        if(len >= sizeof(SdLoadSamplePayload)){
            SdLoadSamplePayload pl;
            memcpy(&pl, p, sizeof(pl));
            pl.folder[31] = 0; pl.filename[31] = 0;
            char path[160];
            snprintf(path, sizeof(path), "%s/%s/%s",
                     SD_DATA_ROOT, pl.folder, pl.filename);
            if(pl.padIdx < MAX_PADS){
                bool ok = LoadWavToPad(path, pl.padIdx);
                hw.PrintLine("SD: Load '%s' → pad %d: %s",
                               pl.filename, pl.padIdx, ok?"OK":"FAIL");
                if(ok){
                    PushEvent(EVT_SD_SAMPLE_LOADED, 1,
                              1u << pl.padIdx, pl.filename);
                } else {
                    PushEvent(EVT_SD_ERROR, 0,
                              1u << pl.padIdx, pl.filename);
                }
            }
        }
        break;
    }

    case CMD_SD_ABORT:
        /* Abort any ongoing SD transfer (currently unused) */
        break;

    /* ════════════════════════════════════════════
     *  STATUS / QUERY (0xE0-0xE3)
     * ════════════════════════════════════════════ */
    case CMD_GET_PEAKS: {
        float buf[17];
        for(int i = 0; i < 16; i++){
            buf[i] = trackPeak[i];
            trackPeak[i] = 0.0f;
        }
        buf[16] = masterPeak;
        BuildResponse(CMD_GET_PEAKS, hdr->sequence, (uint8_t*)buf, 68);
        return;
    }

    case CMD_GET_STATUS: {
        /* Expanded status, including FatFs and raw SD-SPI diagnostics. */
        uint8_t resp[87]; memset(resp, 0, sizeof(resp));
        resp[0] = ActiveVoices();
        resp[1] = AudioCpuPercent();
        /* resp[2-3]: loaded bitmask pads 0-15 */
        for(int i = 0; i < 8; i++)
            if(sampleLoaded[i]) resp[2] |= (1 << i);
        for(int i = 8; i < 16; i++)
            if(sampleLoaded[i]) resp[3] |= (1 << (i-8));
        /* resp[4-7]: uptime ms */
        uint32_t up = hw.system.GetNow();
        memcpy(resp + 4, &up, 4);
        /* resp[8]: SD present */
        resp[8] = sdPresent ? 1 : 0;
        /* resp[9]: loaded bitmask pads 16-23 (XTRA) */
        for(int i = 16; i < 24; i++)
            if(sampleLoaded[i]) resp[9] |= (1 << (i-16));
        /* resp[10]: pending event count → Master sabe si debe llamar CMD_GET_EVENTS */
        resp[10] = evtCount;
        /* resp[11-12]: spiErrCnt (diagnostico CRC/parse errors) */
        resp[11] = (uint8_t)(spiErrCnt & 0xFF);
        resp[12] = (uint8_t)((spiErrCnt >> 8) & 0xFF);
        /* resp[13]: spiRingDrops (bytes perdidos por ring buffer lleno) */
        resp[13] = (uint8_t)(spiRingDrops > 255 ? 255 : spiRingDrops);
        /* resp[14-45]: currentKitName (32 chars) */
        CopyFixedString((char*)(resp + 14), 32, currentKitName);
        /* resp[46-53]: total loaded sample count + total sample bytes (info) */
        uint8_t totalLoaded = 0;
        uint32_t totalBytes = 0;
        for(int i = 0; i < MAX_PADS; i++){
            if(sampleLoaded[i]){
                totalLoaded++;
                totalBytes += sampleLength[i] * 2;
            }
        }
        resp[46] = totalLoaded;
        memcpy(resp + 47, &totalBytes, 4);
        /* resp[51]: MAX_PADS */
        resp[51] = MAX_PADS;
        resp[52] = (uint8_t)(AudioCpuPeakPercent() + 0.5f);
        resp[53] = perfStressMode ? 1 : 0;
        resp[54] = (uint8_t)(AudioCpuAvgPercent() + 0.5f);
        resp[55] = (uint8_t)(masterPeak >= 1.0f ? 1 : 0);
        uint16_t ringDrops = spiRingDrops;
        memcpy(resp + 56, &ringDrops, 2);
        for(int i = 0; i < CLEAN_TRACK_COUNT; i++){
            if(cleanTrackLoaded[i]) resp[58] |= (1u << i);
            if(cleanTrackActive[i]) resp[59] |= (1u << i);
        }
        /* resp[60-75]: Daisy is authoritative for the actual track engines. */
        memcpy(resp + 60, dsqTrackEngine, DSQ_TRACKS);
        /* resp[76-79]: SD boot diagnostics (FatFS codes + real results). */
        resp[76] = sdMountResult;
        resp[77] = sdRootResult;
        resp[78] = sdBootLoaded;
        resp[79] = sdLoadFailures;
        /* resp[80-86]: exact low-level SD failure point. */
        resp[80] = sdDiagStage;
        resp[81] = sd_card_type;
        resp[82] = sdLastCommand;
        resp[83] = sdLastResponse;
        resp[84] = sdLastDataToken;
        memcpy(resp + 85, &sdSpiErrors, 2);
        BuildResponse(CMD_GET_STATUS, hdr->sequence, resp, sizeof(resp));
        return;
    }

    case CMD_POD_GET_STATE: {
        PodStatePayload state;
        BuildPodState(state);
        BuildResponse(CMD_POD_GET_STATE, hdr->sequence,
                      reinterpret_cast<const uint8_t*>(&state), sizeof(state));
        podButtonPressEvents = 0;
        return;
    }

    case CMD_POD_SET_CONFIG: {
        if(len >= sizeof(PodConfigPayload)){
            PodConfigPayload requested;
            memcpy(&requested, p, sizeof(requested));
            ValidatePodConfig(requested);
            podConfig = requested;
            podLastKnobRaw[0] = -1;
            podLastKnobRaw[1] = -1;
            podStateRevision++;
            SavePodConfigToSD();
        }
        PodStatePayload state;
        BuildPodState(state);
        BuildResponse(CMD_POD_SET_CONFIG, hdr->sequence,
                      reinterpret_cast<const uint8_t*>(&state), sizeof(state));
        return;
    }

    case CMD_MIDI_GET_EVENTS: {
        /* Drain the MPD218 monitor ring → up to 32 raw events per poll.
         * Response: [count(1)] + [status,data0,data1] * count. */
        uint8_t buf[1 + 32 * 3];
        uint8_t n = 0;
        while(midiMonTail != midiMonHead && n < 32u)
        {
            buf[1 + n * 3 + 0] = midiMonRing[midiMonTail][0];
            buf[1 + n * 3 + 1] = midiMonRing[midiMonTail][1];
            buf[1 + n * 3 + 2] = midiMonRing[midiMonTail][2];
            midiMonTail = (midiMonTail + 1u) & (MIDI_MON_RING_SIZE - 1u);
            n++;
        }
        buf[0] = n;
        BuildResponse(CMD_MIDI_GET_EVENTS, hdr->sequence, buf,
                      static_cast<uint16_t>(1 + n * 3));
        return;
    }

    case CMD_MIDI_MAP_SET: {
        /* Full replacement of the learned map. Fire-and-forget: P4 persists
         * the map in its NVS and re-uploads it on every reconnection, so no
         * response slot is consumed (telemetry may be pending). */
        if(len < 1) return;
        uint8_t count = p[0];
        if(count > MIDI_MAP_MAX_ENTRIES) count = MIDI_MAP_MAX_ENTRIES;
        if(len < 1u + count * sizeof(MidiMapEntry)) return;
        uint8_t accepted = 0;
        for(uint8_t i = 0; i < count; ++i)
        {
            MidiMapEntry entry;
            memcpy(&entry, p + 1 + i * sizeof(MidiMapEntry), sizeof(entry));
            if(entry.channel > 15u || entry.number > 127u) continue;
            if(entry.kind == MIDI_MAP_KIND_NOTE)
            {
                if(entry.action > red808_mpd218::PAD_CLEAR_SELECTED_FX)
                    continue;
            }
            else if(entry.kind == MIDI_MAP_KIND_CC)
            {
                if(entry.action > red808_mpd218::KNOB_STEREO_WIDTH) continue;
            }
            else
                continue;
            midiUserMap[accepted++] = entry;
        }
        midiUserMapCount = accepted;
        return;
    }

    case CMD_GET_CPU_LOAD: {
        CpuLoadResponse resp;
        resp.cpuLoad = AudioCpuAvgPercent();
        resp.uptime = hw.system.GetNow();
        resp.cpuAvg = AudioCpuAvgPercent();
        resp.cpuPeak = AudioCpuPeakPercent();
        resp.activeVoices = ActiveVoices();
        resp.perfStressMode = perfStressMode ? 1 : 0;
        resp.spiErrCnt = spiErrCnt;
        resp.spiRingDrops = spiRingDrops;
        resp.masterPeak = masterPeak;
        BuildResponse(CMD_GET_CPU_LOAD, hdr->sequence, (const uint8_t*)&resp, sizeof(resp));
        return;
    }

    case CMD_GET_VOICES: {
        uint8_t cnt = ActiveVoices();
        BuildResponse(CMD_GET_VOICES, hdr->sequence, &cnt, 1);
        return;
    }

    case CMD_GET_EVENTS: {
        /* Drain pending events → Master receives up to 4 events per call.
         * Response: [count(1)] + [NotifyEvent(32)] * count
         * El Master llama repetidamente hasta que count == 0. */
        NotifyEvent evts[4];
        uint8_t n = PopEvents(evts, 4);
        uint8_t buf[1 + 4 * 32];
        buf[0] = n;
        if(n > 0) memcpy(buf + 1, evts, n * sizeof(NotifyEvent));
        BuildResponse(CMD_GET_EVENTS, hdr->sequence, buf, 1 + n * 32);
        return;
    }

    case CMD_DIAG_PERF_STRESS: {
        if(len >= 1){
            if(p[0] == 2){
                audioLoadMeter.Reset();
                masterPeak = 0.0f;
                spiErrCnt = 0;
                spiRingDrops = 0;
            } else {
                SetPerformanceStressMode(p[0] != 0);
            }
        }
        uint8_t resp[4] = {
            (uint8_t)(perfStressMode ? 1 : 0),
            (uint8_t)(AudioCpuAvgPercent() + 0.5f),
            (uint8_t)(AudioCpuPeakPercent() + 0.5f),
            ActiveVoices()
        };
        BuildResponse(CMD_DIAG_PERF_STRESS, hdr->sequence, resp, sizeof(resp));
        return;
    }

    /* ════════════════════════════════════════════
     *  RESET
     * ════════════════════════════════════════════ */
    case CMD_RESET:
        SetPerformanceStressMode(false);
        synth909.ClearPcmSamples();
        synth909PcmMode = false;
        synth505.ClearPcmSamples();
        synth505PcmMode = false;
        memset(sampleUploadReceivedBytes, 0, sizeof(sampleUploadReceivedBytes));
        memset(sampleUploadValid, 0, sizeof(sampleUploadValid));
        for(int v = 0; v < MAX_VOICES; v++) voices[v].active = false;
        for(int i = 0; i < MAX_PADS; i++){
            sampleLoaded[i] = false; sampleLength[i] = 0;
            sampleRateHz[i] = SAMPLE_RATE;
            sampleUploadValid[i] = false;
            sampleUploadReceivedBytes[i] = 0;
            trackGain[i]    = 1.0f;  trackPeak[i] = 0;
            padLoop[i] = false; padReverse[i] = false; padPitch[i] = 1.0f; trkPitchCents[i] = 0;
            padFilterType[i] = 0; padDistDrive[i] = 0; padDistMode[i] = 0; padBitDepth[i] = 16;
            padStutterOn[i] = false;
            trkFilterType[i] = 0; trkDistDrive[i] = 0; trkBitDepth[i] = 16;
            trkEchoActive[i] = false; trkFlgActive[i] = false; trkCompActive[i] = false;
            trkFlanger[i].Init((float)SAMPLE_RATE);
            ConfigureTrackFlanger((uint8_t)i);
            trackReverbSend[i] = 0; trackDelaySend[i] = 0; trackChorusSend[i] = 0;
            trackPanF[i] = 0; trackMute[i] = false; trackSolo[i] = false;
            trkEqLowDb[i] = 0; trkEqMidDb[i] = 0; trkEqHighDb[i] = 0;
            trkFxRouted[i] = false;  padLoading[i] = false;
            trkLfoActive[i] = false;
            trkLfoWave[i]   = LFO_WAVE_SINE;
            trkLfoTarget[i] = LFO_TGT_GAIN;
            trkLfoRate[i]   = 1.0f;
            trkLfoDepth[i]  = 0.0f;
            trkLfoPhase[i]  = 0.0f;
            trkLfoSH[i]     = 0.0f;
            trkEnvAdActive[i] = false;
            trkEnvAttackMs[i] = 1.0f;
            trkEnvDecayMs[i]  = 250.0f;
        }
        masterGain = 1.0f; seqVolume = 1.0f; liveVolume = 1.0f; livePitch = 1.0f;
        ResetMasterProcessingState();
        scActive = false; scEnv = 0;
        anySolo = false;
        masterPeak = 0;
        spiPktCnt = 0; spiErrCnt = 0;
        /* Reset synth engines */
        synth808.Init((float)SAMPLE_RATE);
        synth909.Init((float)SAMPLE_RATE);
        synth505.Init((float)SAMPLE_RATE);
        acid303.Init((float)SAMPLE_RATE);
        wtOsc.Init((float)SAMPLE_RATE);
        synthSH101.Init((float)SAMPLE_RATE);
        synthFM2Op.Init((float)SAMPLE_RATE);
        physModal.Init((float)SAMPLE_RATE);
        physString.Init((float)SAMPLE_RATE);
        noisePart.Init((float)SAMPLE_RATE);
        physModalActive = false;
        physStringActive = false;
        noisePartActive = false;
        ApplyDefaultSynthPresets();
        for(int i=0;i<16;i++) trackWtNote[i]    = (uint8_t)(60 + (i % 12));
        for(int i=0;i<16;i++) trackSH101Note[i] = (uint8_t)(60 + (i % 12));
        for(int i=0;i<16;i++) trackFM2OpNote[i] = (uint8_t)(60 + (i % 12));
        /* Reset mega upgrade state */
        masterAutowahL.Init((float)SAMPLE_RATE);
        masterAutowahR.Init((float)SAMPLE_RATE);
        masterAutowahL.SetLevel(1.0f);
        masterAutowahR.SetLevel(1.0f);
        masterAutowahL.SetWah(autowahLevel);
        masterAutowahR.SetWah(autowahLevel);
        masterLadderL.Init((float)SAMPLE_RATE);
        masterLadderR.Init((float)SAMPLE_RATE);
        masterSvfL.Init((float)SAMPLE_RATE);
        masterSvfR.Init((float)SAMPLE_RATE);
        erDelayL.Init();
        erDelayR.Init();
        combDelayL.Init();
        combDelayR.Init();
        masterDelayR.Init();
        masterDelayR.SetDelay(delayTime / 1000.0f * (float)SAMPLE_RATE);
        memset(beatRepBufL, 0, sizeof(beatRepBufL));
        memset(beatRepBufR, 0, sizeof(beatRepBufR));
        memset(chokeGroup, 0, sizeof(chokeGroup));
        songLength = 0; songPlaying = false; songIdx = 0; songRepeatCnt = 0;
        synthActiveMask = 0x01FF;  /* all 9 engines active */
        break;

    /* ════════════════════════════════════════════
     *  BULK (0xF0-0xF1)
     * ════════════════════════════════════════════ */
    /* ════════════════════════════════════════════
     *  SYNTH ENGINES (0xC0-0xC5)
     * ════════════════════════════════════════════ */
    case CMD_SYNTH_TRIGGER:
        if(len >= 3){
            uint8_t engine = p[0];
            uint8_t instrument = p[1];
            /* Queued (except PHYS/NOISE, see the AudioCmd scope note above
             * TriggerPad's definition). cmd.velocity carries the raw byte —
             * AudioCmdApplySynthTrigger/ApplyNoteOn recompute /127.0f, which
             * is bit-for-bit the same as this handler's original
             * `velocity = p[2]/127.0f`, including the lack of clamping. */
            AudioCmd cmd{};
            cmd.engine = engine;
            cmd.velocity = p[2];
            switch(engine){
                case SYNTH_ENGINE_808:
                    cmd.type = AUDIO_CMD_SYNTH_TRIGGER;
                    cmd.instrument = (instrument < 16) ? padTo808[instrument] : (uint8_t)(instrument % TR808::INST_COUNT);
                    AudioCmdPush(cmd);
                    break;
                case SYNTH_ENGINE_909:
                    cmd.type = AUDIO_CMD_SYNTH_TRIGGER;
                    cmd.instrument = (instrument < 16) ? padTo909[instrument] : (uint8_t)(instrument % TR909::INST_COUNT);
                    AudioCmdPush(cmd);
                    break;
                case SYNTH_ENGINE_505:
                    cmd.type = AUDIO_CMD_SYNTH_TRIGGER;
                    cmd.instrument = (instrument < 16) ? padTo505[instrument] : (uint8_t)(instrument % TR505::INST_COUNT);
                    AudioCmdPush(cmd);
                    break;
                case SYNTH_ENGINE_303: {
                    uint8_t slot = (uint8_t)(instrument & 0x0F);
                    float velocity = p[2] / 127.0f;
                    cmd.type = AUDIO_CMD_SYNTH_NOTE_ON;
                    cmd.note = padTo303Midi[slot];
                    cmd.accent = ((slot % 4 == 0) || (velocity > 0.85f)) ? 1 : 0;
                    cmd.slide = (slot % 4 == 3) ? 1 : 0;
                    AudioCmdPush(cmd);
                    break;
                }
                case SYNTH_ENGINE_WTOSC:
                    cmd.type = AUDIO_CMD_SYNTH_NOTE_ON;
                    cmd.note = (instrument < 16) ? trackWtNote[instrument] : 60;
                    AudioCmdPush(cmd);
                    break;
                case SYNTH_ENGINE_SH101:                 /* I1 */
                    cmd.type = AUDIO_CMD_SYNTH_NOTE_ON;
                    cmd.note = (instrument < 16) ? trackSH101Note[instrument] : 60;
                    AudioCmdPush(cmd);
                    break;
                case SYNTH_ENGINE_FM2OP:                 /* I2 */
                    cmd.type = AUDIO_CMD_SYNTH_NOTE_ON;
                    cmd.note = (instrument < 16) ? trackFM2OpNote[instrument] : 60;
                    AudioCmdPush(cmd);
                    break;
                case SYNTH_ENGINE_PHYS:
                    cmd.type = AUDIO_CMD_PHYS_NOISE;
                    cmd.note = (instrument < 16) ? trackWtNote[instrument] : 60;
                    AudioCmdPush(cmd);
                    break;
                case SYNTH_ENGINE_NOISE:
                    cmd.type = AUDIO_CMD_PHYS_NOISE;
                    cmd.note = (instrument < 16) ? trackWtNote[instrument] : 60;
                    AudioCmdPush(cmd);
                    break;
            }
        }
        break;

    case CMD_SYNTH_PARAM:
        if(len >= 7){
            uint8_t engine = p[0];
            uint8_t instrument = p[1];
            uint8_t paramId = p[2];
            float val; memcpy(&val, p + 3, 4);
            /* paramId: 0=decay, 1=pitch, 2=tone, 3=volume, 4=snappy */
            switch(engine){
                /* El byte 'instrument' de CMD_SYNTH_PARAM YA llega como id nativo
                 * del enum: la UI web lo remapea con padToInstrument() antes de
                 * enviarlo. Por eso aqui se pasa DIRECTO, sin volver a remapear.
                 * (CMD_SYNTH_TRIGGER si remapea con padTo*, porque alli el byte
                 *  llega como indice de pad.) */
                case SYNTH_ENGINE_808:
                case SYNTH_ENGINE_909:
                case SYNTH_ENGINE_505:
                    ApplyDrumSynthParam(engine, instrument, paramId, val);
                    break;
                case SYNTH_ENGINE_303:
                    switch(paramId){
                        case 0: acid303.SetCutoff(val);    break;
                        case 1: acid303.SetResonance(val); break;
                        case 2: acid303.SetEnvMod(val);    break;
                        case 3: acid303.SetDecay(val);     break;
                        case 4: acid303.SetAccent(val);    break;
                        case 5: acid303.SetSlide(val);     break;
                        case 6: acid303.SetWaveform(val < 0.5f ? TB303::WAVE_SAW : TB303::WAVE_SQUARE); break;
                        case 7: acid303.SetVolume(val);    break;
                        case 8: acid303.SetAttack(val);    break;
                        case 9: acid303.SetSustain(val);   break;
                        case 10: acid303.SetRelease(val);  break;
                        case 11: acid303.SetOverdrive(val); break;
                        case 12: acid303.SetSubLevel(val); break;
                        case 13: acid303.SetDrift(val);    break;
                        case 14: acid303.SetPitchBend(val); break;
                        default: break;
                    }
                    break;
                case SYNTH_ENGINE_WTOSC:
                    switch(paramId){
                        case 0: wtOsc.SetWavePos(val);                           break;
                        case 1: wtOsc.SetAttack(val);                            break;
                        case 2: wtOsc.SetDecay(val);                             break;
                        case 3: wtOsc.volume = clampF(val, 0.f, 1.f);           break;
                        case 4: { /* filter cutoff Hz */
                            wtFilterCutoffState = clampF(val, 20.f, 18000.f);
                            if(instrument >= 1)
                                wtFilterQState = clampF((float)instrument * 0.1f, 0.1f, 20.f);
                            ApplyWtModState();
                            break; }
                        case 5: { /* lfo rate Hz */
                            wtLfoRateState = clampF(val, 0.01f, 20.f);
                            ApplyWtModState();
                            break; }
                        case 6: { /* lfo depth 0-1 */
                            wtLfoDepthState = clampF(val, 0.f, 1.f);
                            ApplyWtModState();
                            break; }
                        case 7: { /* lfo target */
                            wtLfoTargetState = (WtLfoTarget)clampF(val, 0.f, 2.f);
                            ApplyWtModState();
                            break; }
                        case 8:
                            if(instrument < 16)
                                trackWtNote[instrument] = (uint8_t)clampF(val, 0.f, 127.f);
                            break;
                    }
                    break;
                case SYNTH_ENGINE_SH101:                  /* I1 */
                    if(paramId == 20 && instrument < 16)  /* special: MIDI note assignment */
                        trackSH101Note[instrument] = (uint8_t)clampF(val, 0.f, 127.f);
                    else
                        synthSH101.SetParam(paramId, val);
                    break;
                case SYNTH_ENGINE_FM2OP:                  /* I2 */
                    if(paramId == 20 && instrument < 16)
                        trackFM2OpNote[instrument] = (uint8_t)clampF(val, 0.f, 127.f);
                    else
                        synthFM2Op.SetParam(paramId, val);
                    break;
                case SYNTH_ENGINE_PHYS:
                    switch(paramId){
                        case 0: physModal.SetFreq(clampF(val, 20.f, 10000.f));    break;
                        case 1: physModal.SetStructure(clampF(val, 0.f, 1.f));    break;
                        case 2: physModal.SetBrightness(clampF(val, 0.f, 1.f));   break;
                        case 3: physModal.SetDamping(clampF(val, 0.f, 1.f));      break;
                        case 4: physModalGain = clampF(val, 0.f, 1.f);            break;
                        case 5: physString.SetFreq(clampF(val, 20.f, 10000.f));   break;
                        case 6: physString.SetStructure(clampF(val, 0.f, 1.f));   break;
                        case 7: physString.SetBrightness(clampF(val, 0.f, 1.f));  break;
                        case 8: physString.SetDamping(clampF(val, 0.f, 1.f));     break;
                        case 9: physStringGain = clampF(val, 0.f, 1.f);           break;
                    }
                    break;
                case SYNTH_ENGINE_NOISE:
                    switch(paramId){
                        case 0: noisePart.SetFreq(clampF(val, 20.f, 10000.f));    break;
                        case 1: noisePart.SetResonance(clampF(val, 0.f, 1.f));    break;
                        case 2: noisePart.SetRandomFreq(clampF(val, 0.f, 1.f));   break;
                        case 3: noisePart.SetDensity(clampF(val, 0.f, 1.f));      break;
                        case 4: noisePart.SetGain(clampF(val, 0.f, 1.f));         break;
                        case 5: noisePart.SetSpread(clampF(val, 0.f, 1.f));       break;
                        case 6: noisePartGain = clampF(val, 0.f, 1.f);            break;
                    }
                    break;
            }
        }
        break;

    case CMD_SYNTH_NOTE_ON:
        if(len >= 3){
            AudioCmd cmd{};
            cmd.type = AUDIO_CMD_SYNTH_NOTE_ON;
            cmd.engine = SYNTH_ENGINE_303;
            cmd.note = p[0];
            cmd.accent = (p[1] != 0) ? 1 : 0;
            cmd.slide = (p[2] != 0) ? 1 : 0;
            AudioCmdPush(cmd);
        }
        break;

    case CMD_SYNTH_NOTE_OFF:
        /* v2.5: payload extendido [engine, track] para apagar el synth correcto.
         * Sin payload: apaga TODOS los synths melódicos (panic legacy 303).
         * Queued for 303/WTOSC/SH101/FM2OP (PHYS stays a direct write — see
         * the AudioCmd scope note above TriggerPad's definition). */
        if(len >= 2){
            uint8_t engine = p[0];
            uint8_t track  = p[1];
            uint8_t note   = (len >= 3) ? p[2] : 0xFF;
            if(kEnableSynthCmdLog && track == 0xFF)
                hw.PrintLine("SYNTH_NOTE_OFF_ALL engine=%u", engine);
            switch(engine){
                case SYNTH_ENGINE_303:
                case SYNTH_ENGINE_WTOSC:
                case SYNTH_ENGINE_SH101:
                case SYNTH_ENGINE_FM2OP: {
                    AudioCmd cmd{};
                    cmd.type = AUDIO_CMD_SYNTH_NOTE_OFF;
                    cmd.engine = engine;
                    cmd.note = note;
                    cmd.track = track;
                    AudioCmdPush(cmd);
                    break;
                }
                case SYNTH_ENGINE_PHYS:
                    physModalActive = false;
                    physStringActive = false;
                    break;
                default: break;
            }
        } else {
            /* Legacy: NoteOff genérico (compat firmware antiguo) */
            AudioCmd cmd{};
            cmd.type = AUDIO_CMD_SYNTH_NOTE_OFF;
            cmd.engine = 0xFFu; /* panic-all: 303 + SH101 + FM2OP + WTOSC */
            AudioCmdPush(cmd);
            physModalActive = false;
            physStringActive = false;
        }
        break;

    case CMD_SYNTH_303_PARAM:
        if(len >= 5){
            uint8_t paramId = p[0];
            float val; memcpy(&val, p + 1, 4);
            switch(paramId){
                case 0: acid303.SetCutoff(val);    break;
                case 1: acid303.SetResonance(val);  break;
                case 2: acid303.SetEnvMod(val);     break;
                case 3: acid303.SetDecay(val);      break;
                case 4: acid303.SetAccent(val);     break;
                case 5: acid303.SetSlide(val);      break;
                case 6: acid303.SetWaveform(val < 0.5f ? TB303::WAVE_SAW : TB303::WAVE_SQUARE); break;
                case 7: acid303.SetVolume(val);     break;
                /* v2.0 new params */
                case 8:  acid303.SetAttack(val);    break; /* attack s   */
                case 9:  acid303.SetSustain(val);   break; /* sustain 0-1*/
                case 10: acid303.SetRelease(val);   break; /* release s  */
                case 11: acid303.SetOverdrive(val); break; /* overdrive  */
                case 12: acid303.SetSubLevel(val);  break; /* sub osc    */
                case 13: acid303.SetDrift(val);     break; /* analog drift*/
                case 14: acid303.SetPitchBend(val); break; /* semitones  */
            }
        }
        break;

    case CMD_SYNTH_ACTIVE:
        if(len >= 1)
        {
            uint16_t oldMask = synthActiveMask;
            /* Accept 1 or 2 bytes — backward compatible */
            if(len >= 2)
                synthActiveMask = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
            else
                synthActiveMask = p[0];
            if((oldMask & (1 << SYNTH_ENGINE_303)) && !(synthActiveMask & (1 << SYNTH_ENGINE_303)))
                acid303.NoteOff();
            if((oldMask & (1 << SYNTH_ENGINE_WTOSC)) && !(synthActiveMask & (1 << SYNTH_ENGINE_WTOSC)))
                wtOsc.AllNotesOff();
            if((oldMask & (1 << SYNTH_ENGINE_SH101)) && !(synthActiveMask & (1 << SYNTH_ENGINE_SH101)))
                synthSH101.NoteOff();
            if((oldMask & (1 << SYNTH_ENGINE_FM2OP)) && !(synthActiveMask & (1 << SYNTH_ENGINE_FM2OP)))
                synthFM2Op.NoteOff();
            if((oldMask & (1 << SYNTH_ENGINE_PHYS)) && !(synthActiveMask & (1 << SYNTH_ENGINE_PHYS))){
                physModalActive = false;
                physStringActive = false;
            }
            if((oldMask & (1 << SYNTH_ENGINE_NOISE)) && !(synthActiveMask & (1 << SYNTH_ENGINE_NOISE)))
                noisePartActive = false;
        }
        break;

    case CMD_SYNTH_PRESET:
        if(len >= 2)
        {
            uint8_t engine = p[0];
            uint8_t preset = p[1];
            if(engine < SYNTH_ENGINE_COUNT)
            {
                if(IsPianoMelodicEngine(engine))
                {
                    ReleaseAllSynthEngines();
                    pianoSelectedEngine = engine;
                    if(kEnableSynthCmdLog)
                        hw.PrintLine("SYNTH_PRESET piano engine=%u preset=%u mask=%u", engine, preset, synthActiveMask);
                }
                else
                {
                    ReleaseSynthEngineState(engine);
                    if(kEnableSynthCmdLog)
                        hw.PrintLine("SYNTH_PRESET engine=%u preset=%u mask=%u", engine, preset, synthActiveMask);
                }
                ApplySynthPreset(engine, preset);
            }
        }
        break;

    /* ──────── CMD_SYNTH_NOTE_ON_EX (0xC7) ────────
     * Generic melodic note-on for any synth engine.
     * Payload: [engine(1), midiNote(1), velocity(1), accent(1), slide(1)]
     * Dispatches to the appropriate synth based on engine ID.
     */
    case CMD_SYNTH_NOTE_ON_EX:
        if(len >= 5){
            uint8_t engine   = p[0];
            uint8_t midiNote = p[1];
            uint8_t velocity = p[2];
            bool    accent   = (p[3] != 0);
            bool    slide    = (p[4] != 0);
            /* The piano-engine auto-select (ReleaseAllSynthEngines +
             * pianoSelectedEngine write) moved INTO AudioCmdApplyNoteOn /
             * AudioCmdApplyPhysNoise (cmd.pianoGate below, for 303/WTOSC/
             * SH101/FM2OP/PHYS) so it lands on the audio thread glued to
             * the note-on it gates, instead of racing ahead of it from the
             * main loop. This check is only a would-it-fire read for the
             * diagnostic log — the actual gate runs once, at apply time. */
            if(kEnableSynthCmdLog && IsPianoMelodicEngine(engine)
               && engine != pianoSelectedEngine)
                hw.PrintLine("PIANO_SELECT via=note_on engine=%u", engine);
            switch(engine){
                case SYNTH_ENGINE_303: {
                    AudioCmd cmd{};
                    cmd.type = AUDIO_CMD_SYNTH_NOTE_ON;
                    cmd.engine = engine;
                    cmd.note = midiNote;
                    cmd.accent = accent ? 1 : 0;
                    cmd.slide = slide ? 1 : 0;
                    cmd.pianoGate = 1;
                    AudioCmdPush(cmd);
                    break;
                }
                case SYNTH_ENGINE_WTOSC:
                case SYNTH_ENGINE_SH101:
                case SYNTH_ENGINE_FM2OP: {
                    AudioCmd cmd{};
                    cmd.type = AUDIO_CMD_SYNTH_NOTE_ON;
                    cmd.engine = engine;
                    cmd.note = midiNote;
                    cmd.velocity = velocity;
                    cmd.pianoGate = 1;
                    AudioCmdPush(cmd);
                    break;
                }
                case SYNTH_ENGINE_PHYS: {
                    AudioCmd cmd{};
                    cmd.type = AUDIO_CMD_PHYS_NOISE;
                    cmd.engine = engine;
                    cmd.note = midiNote;
                    cmd.velocity = velocity;
                    cmd.pianoGate = 1;
                    cmd.trig = 1;
                    AudioCmdPush(cmd);
                    break;
                }
                case SYNTH_ENGINE_NOISE: {
                    AudioCmd cmd{};
                    cmd.type = AUDIO_CMD_PHYS_NOISE;
                    cmd.engine = engine;
                    cmd.note = midiNote;
                    cmd.velocity = velocity;
                    AudioCmdPush(cmd);
                    break;
                }
                default:
                    break;
            }
        }
        break;

    case CMD_BULK_TRIGGERS:
        if(len >= 2){
            uint8_t count = p[0];
            /* Formato completo: count(1) + reserved(1) + N×TriggerSeqPayload(8) */
            const uint8_t* tp = p + 2; /* skip count + reserved */
            for(uint8_t i = 0; i < count; i++){
                uint16_t off = i * 8;
                if(off + 8 > (len - 2)) break;
                uint8_t  pad = tp[off];
                if(kAcceptOneBasedPadIndex && pad > 0) pad -= 1;
                uint8_t  vel = tp[off + 1];
                uint8_t  tvol = tp[off + 2];
                int8_t   pan = (int8_t)tp[off + 3];
                uint32_t maxS = 0;
                memcpy(&maxS, tp + off + 4, 4);
                /* Routing: si el track tiene synth engine asignado, dispara
                 * el synth correspondiente (igual que CMD_TRIGGER_LIVE/SEQ). */
                int8_t bEng = (pad < DSQ_TRACKS) ? dsqTrackEngine[pad] : -1;
                if(bEng >= 0 && bEng < SYNTH_ENGINE_COUNT){
                    /* Same queueing as CMD_TRIGGER_LIVE — see the note there. */
                    float fvel = clampF(vel / 127.0f, 0.0f, 1.0f);
                    const uint8_t qvel = (uint8_t)(fvel * 127.0f + 0.5f);
                    AudioCmd cmd{};
                    cmd.engine = (uint8_t)bEng;
                    cmd.velocity = qvel;
                    switch(bEng){
                        case SYNTH_ENGINE_808:
                            if(pad < 16){
                                cmd.type = AUDIO_CMD_SYNTH_TRIGGER;
                                cmd.instrument = padTo808[pad];
                                AudioCmdPush(cmd);
                            }
                            break;
                        case SYNTH_ENGINE_909:
                            if(pad < 16){
                                cmd.type = AUDIO_CMD_SYNTH_TRIGGER;
                                cmd.instrument = padTo909[pad];
                                AudioCmdPush(cmd);
                            }
                            break;
                        case SYNTH_ENGINE_505:
                            if(pad < 16){
                                cmd.type = AUDIO_CMD_SYNTH_TRIGGER;
                                cmd.instrument = padTo505[pad];
                                AudioCmdPush(cmd);
                            }
                            break;
                        case SYNTH_ENGINE_303:
                            cmd.type = AUDIO_CMD_SYNTH_NOTE_ON;
                            cmd.note = (pad < 16) ? padTo303Midi[pad] : 48;
                            cmd.accent = (fvel > 0.85f) ? 1 : 0;
                            AudioCmdPush(cmd);
                            break;
                        case SYNTH_ENGINE_WTOSC:
                            cmd.type = AUDIO_CMD_SYNTH_NOTE_ON;
                            cmd.note = (pad < 16) ? trackWtNote[pad] : 60;
                            AudioCmdPush(cmd);
                            break;
                        case SYNTH_ENGINE_SH101:
                            cmd.type = AUDIO_CMD_SYNTH_NOTE_ON;
                            cmd.note = (pad < 16) ? trackSH101Note[pad] : 60;
                            AudioCmdPush(cmd);
                            break;
                        case SYNTH_ENGINE_FM2OP:
                            cmd.type = AUDIO_CMD_SYNTH_NOTE_ON;
                            cmd.note = (pad < 16) ? trackFM2OpNote[pad] : 60;
                            AudioCmdPush(cmd);
                            break;
                        case SYNTH_ENGINE_PHYS:
                            cmd.type = AUDIO_CMD_PHYS_NOISE;
                            cmd.note = (pad < 16) ? trackWtNote[pad] : 60;
                            AudioCmdPush(cmd);
                            break;
                        case SYNTH_ENGINE_NOISE:
                            cmd.type = AUDIO_CMD_PHYS_NOISE;
                            cmd.note = (pad < 16) ? trackWtNote[pad] : 60;
                            AudioCmdPush(cmd);
                            break;
                    }
                } else {
                    AudioCmd cmd{};
                    cmd.type = AUDIO_CMD_TRIGGER_PAD;
                    cmd.pad = pad;
                    cmd.velocity = vel;
                    cmd.trkVol = tvol;
                    cmd.pan = pan;
                    cmd.maxSamples = maxS;
                    cmd.sourceVolume = seqVolume;
                    cmd.sourcePitch = 1.0f;
                    AudioCmdPush(cmd);
                }
            }
            spiLastTriggerMs = hw.system.GetNow();
        }
        break;

    case CMD_BULK_FX:
        if(len >= 1){
            uint8_t bulkPayload[RX_BUF_SIZE - 8];
            uint8_t savedPacket[RX_BUF_SIZE];
            uint16_t savedPacketLen = 8 + len;
            if(len > sizeof(bulkPayload) || savedPacketLen > sizeof(savedPacket))
                break;

            memcpy(bulkPayload, p, len);
            memcpy(savedPacket, rxBuf, savedPacketLen);
            bool savedPendingResponse = pendingResponse;
            uint16_t savedPendingTxLen = pendingTxLen;

            uint8_t cnt = bulkPayload[0];
            uint16_t off = 1;
            for(uint8_t j = 0; j < cnt; j++){
                if(off + 2 > len) break;
                uint8_t subCmd = bulkPayload[off];
                uint8_t subLen = bulkPayload[off + 1];
                off += 2;
                if(off + subLen > len) break;

                if(subCmd == CMD_BULK_FX){
                    off += subLen;
                    continue;
                }

                SPIPacketHeader* subHdr = (SPIPacketHeader*)rxBuf;
                subHdr->magic = SPI_MAGIC_CMD;
                subHdr->cmd = subCmd;
                subHdr->length = subLen;
                subHdr->sequence = hdr->sequence;
                subHdr->checksum = crc16(bulkPayload + off, subLen);
                if(subLen > 0)
                    memcpy(rxBuf + 8, bulkPayload + off, subLen);
                pendingResponse = false;
                pendingTxLen = 0;
                ProcessCommand();
                pendingResponse = savedPendingResponse;
                pendingTxLen = savedPendingTxLen;
                off += subLen;
            }
            memcpy(rxBuf, savedPacket, savedPacketLen);
        }
        break;

    /* ════════════════════════════════════════════════════════
     *  DAISY SEQUENCER (0xD0-0xD8)
     * ════════════════════════════════════════════════════════ */
    case CMD_DSQ_UPLOAD_TRACK:
        /* [pat(1), trk(1), stepCount(1), rsvd(1)] + stepCount × DsqStepPkt(4) */
        if(len >= 4){
            uint8_t pat  = p[0] % DSQ_PATTERNS;
            uint8_t trk  = p[1] & 15;
            uint8_t cnt  = p[2];
            if(cnt > DSQ_MAX_STEPS) cnt = DSQ_MAX_STEPS;
            const DsqStepPkt* sp = (const DsqStepPkt*)(p + 4);
            for(uint8_t i = 0; i < cnt && (4 + i*4 + 4) <= len; i++){
                DsqStepFull& dst = dsqSteps[pat][trk][i];
                dst.active     = sp[i].active ? 1 : 0;
                dst.velocity   = sp[i].velocity;
                dst.noteLenDiv = sp[i].noteLenDiv & 0x0F;
                if(dst.noteLenDiv == 0) dst.noteLenDiv = 1;
                dst.ratchet = ((sp[i].noteLenDiv >> 4) & 0x03) + 1;
                dst.probability = sp[i].probability;
                dst.flags = 0;
                memset(dst.notes, 0, sizeof(dst.notes));
                /* param locks preserved — only reset on full pattern clear */
            }
            dsqLoadedPatternMask |= (1u << pat);
        }
        break;

    case CMD_DSQ_SET_STEP:
        /* [pat,trk,step,active,vel,div,prob,rsvd] */
        if(len >= 8){
            uint8_t pat  = p[0] % DSQ_PATTERNS;
            uint8_t trk  = p[1] & 15;
            uint8_t step = p[2];
            if(step < DSQ_MAX_STEPS){
                DsqStepFull& s = dsqSteps[pat][trk][step];
                s.active       = p[3] ? 1 : 0;
                s.velocity     = p[4] ? p[4] : 100;
                s.noteLenDiv   = p[5] & 0x0F;
                if(s.noteLenDiv == 0) s.noteLenDiv = 1;
                s.ratchet      = ((p[5] >> 4) & 0x03) + 1;
                s.probability  = p[6];
                dsqLoadedPatternMask |= (1u << pat);
            }
        }
        break;

    case CMD_DSQ_CONTROL:
        /* [0=stop, 1=play, 2=reset] */
        if(len >= 1){
            if(p[0] == 1){
                dseq.samplesElapsed = 0;
                dseq.currentStep    = -1;
                dseq.playing        = true;
                for(int i = 0; i < CLEAN_TRACK_COUNT; i++){
                    cleanTrackPlayhead[i] = 0;
                    cleanTrackActive[i] = cleanTrackEnabled[i] && cleanTrackLoaded[i];
                }
            } else if(p[0] == 0){
                dseq.playing = false;
                DsqReleaseAllHeldNotes();
                for(int i = 0; i < CLEAN_TRACK_COUNT; i++){
                    cleanTrackActive[i] = false;
                    cleanTrackPlayhead[i] = 0;
                }
            } else if(p[0] == 2){
                dseq.playing        = false;
                dseq.currentStep    = -1;
                dseq.samplesElapsed = 0;
                DsqReleaseAllHeldNotes();
                for(int i = 0; i < CLEAN_TRACK_COUNT; i++){
                    cleanTrackActive[i] = false;
                    cleanTrackPlayhead[i] = 0;
                }
            }
            podStateRevision++;
        }
        break;

    case CMD_DSQ_SELECT_PATTERN:
        if(len >= 1){
            dseq.currentPattern = p[0] % DSQ_PATTERNS;
            dseq.queuedPattern = -1;
            dseq.performanceReturnPattern = -1;
            dseq.queuedPatternBars = 0;
            dseq.performanceBarsRemaining = 0;
            dseq.performancePatternActive = false;
        }
        break;

    case CMD_DSQ_QUEUE_PATTERN:
        if(len >= 1){
            if(p[0] == 0xFF){
                dseq.queuedPattern = -1;
                dseq.queuedPatternBars = 0;
            } else {
                dseq.queuedPattern = (int8_t)(p[0] % DSQ_PATTERNS);
                dseq.queuedPatternBars = (len >= 2 && p[1] <= 16) ? p[1] : 0;
            }
        }
        break;

    case CMD_DSQ_SET_LENGTH:
        if(len >= 1){
            uint8_t l = p[0];
            if(l == 16 || l == 32 || l == 64) dseq.patternLength = l;
        }
        break;

    case CMD_DSQ_SET_MUTE:
        if(len >= 2 && p[0] < DSQ_TRACKS)
            dseq.trackMuted[p[0]] = (bool)p[1];
        break;

    case CMD_DSQ_GET_POS:
        /* Respond with [step(1), pattern(1), playing(1), rsvd(1)] */
        {
            uint8_t resp[4] = {
                (uint8_t)((dseq.currentStep < 0) ? 0 : (uint8_t)dseq.currentStep),
                dseq.currentPattern,
                (uint8_t)(dseq.playing ? 1u : 0u),
                0
            };
            BuildResponse(CMD_DSQ_GET_POS, hdr->sequence, resp, sizeof(resp));
        }
        return;  /* BuildResponse is called → skip default no-response path */

    case CMD_DSQ_SET_SWING:
        if(len >= 1) dseq.swingAmount = p[0] > 100 ? 100 : p[0];
        break;

    case CMD_DSQ_SET_PARAM_LOCK:
        /* [pat,trk,step, cutoffEn,cutHi,cutLo, reverbEn,reverb, volEn,vol, rsvd,rsvd] */
        if(len >= 12){
            uint8_t pat  = p[0] % DSQ_PATTERNS;
            uint8_t trk  = p[1] & 15;
            uint8_t step = p[2];
            if(step < DSQ_MAX_STEPS){
                DsqStepFull& s = dsqSteps[pat][trk][step];
                s.cutoffEn  = (bool)p[3];
                s.cutoffHz  = ((uint16_t)p[4] << 8) | p[5];
                s.reverbEn  = (bool)p[6];
                s.reverbSend = p[7];
                s.volEn     = (bool)p[8];
                s.volume    = p[9];
            }
        }
        break;

    case CMD_DSQ_SET_TRACK_ENGINE:
        /* [track(1), engine(1)]  engine: 0xFF/-1=sampler, 0..8=synth engines */
        if(len >= 2 && p[0] < DSQ_TRACKS)
        {
            uint8_t track = p[0];
            int8_t oldEngine = dsqTrackEngine[track];
            int8_t newEngine = (int8_t)p[1]; /* 0xFF → -1 via cast */
            /* A sampler track without a sample must never make PLAY silent. */
            if(newEngine == -1 && !sampleLoaded[track])
                newEngine = DsqFallbackEngine(track);
            if(oldEngine != newEngine)
            {
                pendingTriggers[track].active = false;
                DsqReleaseHeldNotes(track);
                StopPadVoices(track);
                ReleaseTrackEngine(track, oldEngine);
                padLoop[track] = false;
            }
            dsqTrackEngine[track] = newEngine;
        }
        break;

    case CMD_DSQ_SET_TRACK_SWING:              /* E4 */
        /* [track(1), swing 0-100(1)] override swing por track */
        if(len >= 2 && p[0] < DSQ_TRACKS)
            dsqTrackSwing[p[0]] = (p[1] > 100) ? 100 : p[1];
        break;

    case CMD_DSQ_SET_HUMANIZE:                 /* E2 */
        /* [timingMs(1), velocityAmt(1)] 0=off */
        if(len >= 2){
            dseq.humanizeTimingMs = (p[0] > 20) ? 20 : p[0];
            dseq.humanizeVelAmt   = (p[1] > 50) ? 50 : p[1];
        }
        break;

    case CMD_DSQ_SET_STEP_NOTES:
        /* [pat,trk,step,flags,note0,note1,note2,note3] */
        if(len >= 8){
            uint8_t pat = p[0] % DSQ_PATTERNS;
            uint8_t trk = p[1] & 15;
            uint8_t step = p[2];
            if(step < DSQ_MAX_STEPS){
                DsqStepFull& s = dsqSteps[pat][trk][step];
                s.flags = p[3] & 0x03;
                memcpy(s.notes, p + 4, sizeof(s.notes));
            }
        }
        break;

    /* ════════════════════════════════════════════
     *  MEGA UPGRADE — NEW MASTER FX COMMANDS
     * ════════════════════════════════════════════ */
    case CMD_AUTOWAH_ACTIVE:
        if(len >= 1) autowahActive = (bool)p[0];
        break;
    case CMD_AUTOWAH_LEVEL:
        if(len >= 4){
            float lvl; memcpy(&lvl, p, 4);
            autowahLevel = clampF(lvl, 0.f, 1.f);
            masterAutowahL.SetWah(autowahLevel);
            masterAutowahR.SetWah(autowahLevel);
        }
        break;
    case CMD_AUTOWAH_MIX:
        if(len >= 4){
            float mix; memcpy(&mix, p, 4);
            autowahMix = clampF(mix, 0.f, 1.f);
        }
        break;
    case CMD_STEREO_WIDTH:
        if(len >= 1){
            /* p[0] = 0-200 where 100 = normal. Map to 0.0-2.0 */
            stereoWidth = clampF((float)p[0] / 100.0f, 0.0f, 2.0f);
        }
        break;
    case CMD_TAPE_STOP:
        if(len >= 1){
            if(p[0] == 1){
                tapeStopActive = true;
                tapeStopSpeed = 1.0f;
                tapeStopRate = 0.00003f; /* ~0.7s ramp-down @48kHz */
            } else if(p[0] == 2){
                /* Tape start: ramp back up */
                tapeStopActive = true;
                tapeStopRate = -0.00005f; /* ramp up faster */
            } else {
                tapeStopActive = false;
                tapeStopSpeed = 1.0f;
            }
        }
        break;
    case CMD_BEAT_REPEAT:
        if(len >= 1){
            beatRepDiv = p[0]; /* 0=off, 2/4/8/16/32 */
            if(beatRepDiv == 0){
                beatRepActive = false;
                beatRepPlaying = false;
                beatRepCapturing = false;
                beatRepPos = 0;
            } else {
                beatRepActive = true;
                /* Calculate slice len from BPM and division */
                float beatSec = 60.0f / (transportBpm > 1.f ? transportBpm : 120.f);
                float sliceSec = beatSec * (4.0f / (float)beatRepDiv);
                beatRepLen = (uint32_t)(sliceSec * (float)SAMPLE_RATE);
                if(beatRepLen > BEAT_REPEAT_BUF_SIZE) beatRepLen = BEAT_REPEAT_BUF_SIZE;
                if(beatRepLen < 64) beatRepLen = 64;
                beatRepPos = 0;
                beatRepCapturing = true;
                beatRepPlaying = true;
            }
        }
        break;
    case CMD_DELAY_STEREO:
        if(len >= 1) delayPingPong = (bool)p[0];
        break;
    case CMD_CHORUS_STEREO:
        if(len >= 1) chorusStereoMode = (bool)p[0];
        break;
    case CMD_EARLY_REF_ACTIVE:
        if(len >= 1) erActive = (bool)p[0];
        break;
    case CMD_EARLY_REF_MIX:
        if(len >= 1) erMix = clampF((float)p[0] / 100.0f, 0.0f, 1.0f);
        break;

    /* ════════════════════════════════════════════
     *  CHOKE GROUPS
     * ════════════════════════════════════════════ */
    case CMD_CHOKE_GROUP:
        if(len >= 2 && p[0] < MAX_PADS)
            chokeGroup[p[0]] = (p[1] > 8) ? 0 : p[1]; /* 0=none, 1-8 */
        break;

    /* ════════════════════════════════════════════
     *  SONG MODE
     * ════════════════════════════════════════════ */
    case CMD_SONG_UPLOAD:
        /* [count(1), entries×{pattern(1), repeats(1)}] */
        if(len >= 1){
            uint8_t cnt = p[0];
            if(cnt > SONG_MAX_ENTRIES) cnt = SONG_MAX_ENTRIES;
            const uint8_t available = (uint8_t)((len - 1u) / 2u);
            if(cnt > available) cnt = available;
            for(uint8_t si = 0; si < cnt; si++){
                songChain[si].pattern = p[1 + si*2] % DSQ_PATTERNS;
                songChain[si].repeats = p[2 + si*2];
                if(songChain[si].repeats == 0) songChain[si].repeats = 1;
            }
            songLength = cnt;
        }
        break;
    case CMD_SONG_CONTROL:
        if(len >= 1){
            if(p[0] == 1){
                /* Play song mode */
                if(songLength > 0){
                    songPlaying = true;
                    songIdx = 0;
                    songRepeatCnt = 0;
                    dseq.currentPattern = songChain[0].pattern;
                    dseq.currentStep = -1;
                    dseq.samplesElapsed = 0;
                    dseq.playing = true;
                }
            } else if(p[0] == 0){
                songPlaying = false;
            } else if(p[0] == 2){
                songPlaying = false;
                songIdx = 0;
                songRepeatCnt = 0;
            }
        }
        break;
    case CMD_SONG_GET_POS:
        {
            uint8_t resp[4] = {
                songIdx,
                songPlaying ? songChain[songIdx < songLength ? songIdx : 0].pattern : (uint8_t)0,
                songRepeatCnt,
                0
            };
            BuildResponse(CMD_SONG_GET_POS, hdr->sequence, resp, sizeof(resp));
        }
        return;

    /* ════════════════════════════════════════════
     *  EXPANDED PER-TRACK LFO
     * ════════════════════════════════════════════ */
    case CMD_TRACK_LFO_CONFIG:
        /* [track, wave, target, rateHi, rateLo, depthHi, depthLo] */
        if(len >= 7 && p[0] < MAX_PADS){
            uint8_t t = p[0];
            trkLfoWave[t]   = p[1] & 3;
            trkLfoTarget[t] = p[2];
            trkLfoRate[t]   = (float)((p[3] << 8) | p[4]) / 100.0f;
            trkLfoDepth[t]  = (float)((p[5] << 8) | p[6]) / 1000.0f;
            trkLfoActive[t] = (trkLfoDepth[t] > 0.001f);
            trkFxRouted[t]  = true;
        }
        break;

    default: break;
    }
}

/* ═══════════════════════════════════════════════════════════════════
 *  24. USB CDC DEVICE + DAISY POD CONTROLS
 * ═══════════════════════════════════════════════════════════════════ */
static void ApplyPodCommand(uint8_t cmd, const void* payload, uint16_t length)
{
    if(length > (RX_BUF_SIZE - sizeof(SPIPacketHeader)))
        return;
    static uint16_t localSequence = 0x8000u;
    SPIPacketHeader* header = reinterpret_cast<SPIPacketHeader*>(rxBuf);
    header->magic = SPI_MAGIC_CMD;
    header->cmd = cmd;
    header->length = length;
    header->sequence = localSequence++;
    header->checksum = length > 0
        ? crc16(static_cast<const uint8_t*>(payload), length)
        : 0;
    if(length > 0 && payload != nullptr)
        memcpy(rxBuf + sizeof(SPIPacketHeader), payload, length);
    podApplyingCommand = true;
    ProcessCommand();
    podApplyingCommand = false;
}

static void ApplyPodButtonFunction(uint8_t function, uint32_t now)
{
    switch(function)
    {
        case POD_FUNC_PLAY_TOGGLE: {
            const uint8_t action = dseq.playing ? 0u : 1u;
            ApplyPodCommand(CMD_DSQ_CONTROL, &action, sizeof(action));
            break;
        }
        case POD_FUNC_STOP: {
            const uint8_t action = 2u;
            ApplyPodCommand(CMD_DSQ_CONTROL, &action, sizeof(action));
            ApplyPodCommand(CMD_TRIGGER_STOP_ALL, nullptr, 0);
            break;
        }
        case POD_FUNC_TRIGGER_SELECTED: {
            const uint8_t trigger[2] = {podSelectedPad, 120u};
            ApplyPodCommand(CMD_TRIGGER_LIVE, trigger, sizeof(trigger));
            podPadPulseUntilMs = now + 100u;
            break;
        }
        case POD_FUNC_PATTERN_PREV:
        case POD_FUNC_PATTERN_NEXT: {
            int pattern = static_cast<int>(dseq.currentPattern)
                        + (function == POD_FUNC_PATTERN_NEXT ? 1 : -1);
            if(pattern < 0) pattern = DSQ_PATTERNS - 1;
            if(pattern >= DSQ_PATTERNS) pattern = 0;
            const uint8_t value = static_cast<uint8_t>(pattern);
            ApplyPodCommand(CMD_DSQ_SELECT_PATTERN, &value, sizeof(value));
            break;
        }
        default: break;
    }
    podStateRevision++;
}

static void ApplyPodFxNormalized(uint8_t function, float normalized)
{
    normalized = clampF(normalized, 0.0f, 1.0f);
    switch(function)
    {
        case POD_FUNC_FLANGER_DEPTH: {
            const uint8_t enabled = normalized > 0.005f ? 1u : 0u;
            ApplyPodCommand(CMD_FLANGER_ACTIVE, &enabled, sizeof(enabled));
            ApplyPodCommand(CMD_FLANGER_DEPTH, &normalized, sizeof(normalized));
            break;
        }
        case POD_FUNC_WAVEFOLDER_GAIN: {
            const float gain = 1.0f + normalized * 9.0f;
            ApplyPodCommand(CMD_WAVEFOLDER_GAIN, &gain, sizeof(gain));
            break;
        }
        case POD_FUNC_CRUSH_MACRO: {
            const uint8_t bits = normalized <= 0.005f ? 16u
                : static_cast<uint8_t>(clampF(16.0f - normalized * 10.0f,
                                               6.0f, 16.0f) + 0.5f);
            const uint32_t rate = normalized <= 0.005f ? 0u
                : static_cast<uint32_t>(clampF(42000.0f
                    * powf(4000.0f / 42000.0f, normalized),
                    4000.0f, 42000.0f) + 0.5f);
            ApplyPodCommand(CMD_FILTER_BITDEPTH, &bits, sizeof(bits));
            ApplyPodCommand(CMD_FILTER_SR_REDUCE, &rate, sizeof(rate));
            break;
        }
        case POD_FUNC_PHASER_DEPTH: {
            const uint8_t enabled = normalized > 0.005f ? 1u : 0u;
            ApplyPodCommand(CMD_PHASER_ACTIVE, &enabled, sizeof(enabled));
            ApplyPodCommand(CMD_PHASER_DEPTH, &normalized, sizeof(normalized));
            break;
        }
        case POD_FUNC_FILTER_CUTOFF: {
            const float hz = 20.0f * powf(1000.0f, normalized);
            ApplyPodCommand(CMD_FILTER_CUTOFF, &hz, sizeof(hz));
            break;
        }
        case POD_FUNC_FILTER_RESONANCE: {
            const float q = 0.7f + normalized * 19.3f;
            ApplyPodCommand(CMD_FILTER_RESONANCE, &q, sizeof(q));
            break;
        }
        case POD_FUNC_DISTORTION: {
            const float percent = normalized * 100.0f;
            ApplyPodCommand(CMD_FILTER_DISTORTION, &percent, sizeof(percent));
            break;
        }
        case POD_FUNC_BIT_DEPTH: {
            const uint8_t bits = static_cast<uint8_t>(
                clampF(16.0f - normalized * 12.0f, 4.0f, 16.0f) + 0.5f);
            ApplyPodCommand(CMD_FILTER_BITDEPTH, &bits, sizeof(bits));
            break;
        }
        case POD_FUNC_SAMPLE_RATE: {
            const uint32_t rate = normalized <= 0.005f ? 0u
                : static_cast<uint32_t>(clampF(42000.0f
                    * powf(4000.0f / 42000.0f, normalized),
                    4000.0f, 42000.0f) + 0.5f);
            ApplyPodCommand(CMD_FILTER_SR_REDUCE, &rate, sizeof(rate));
            break;
        }
        case POD_FUNC_FILTER_TYPE: {
            struct __attribute__((packed)) PodFilterPayload {
                uint8_t type, distMode, bitDepth, reserved;
                float cutoff, resonance, distortion;
                uint32_t sampleRateReduce;
            } payload = {
                static_cast<uint8_t>(normalized * FTYPE_COMB + 0.5f),
                gFilterDistMode, gFilterBitDepth, 0,
                gFilterCutoff, gFilterQ, gFilterDist, gFilterSrReduce
            };
            static_assert(sizeof(PodFilterPayload) == 20,
                          "Pod filter payload layout changed");
            ApplyPodCommand(CMD_FILTER_SET, &payload, sizeof(payload));
            break;
        }
        default: break;
    }
}

static float PodFxCurrentNormalized(uint8_t function)
{
    switch(function)
    {
        case POD_FUNC_FLANGER_DEPTH: return flangerDepth;
        case POD_FUNC_WAVEFOLDER_GAIN: return (waveFolderGain - 1.0f) / 9.0f;
        case POD_FUNC_CRUSH_MACRO:
            return (16.0f - static_cast<float>(gFilterBitDepth)) / 10.0f;
        case POD_FUNC_PHASER_DEPTH: return phaserDepth;
        case POD_FUNC_FILTER_CUTOFF:
            return logf(clampF(gFilterCutoff, 20.0f, 20000.0f) / 20.0f)
                 / logf(1000.0f);
        case POD_FUNC_FILTER_RESONANCE: return (gFilterQ - 0.7f) / 19.3f;
        case POD_FUNC_DISTORTION: return gFilterDist;
        case POD_FUNC_BIT_DEPTH: return (16.0f - gFilterBitDepth) / 12.0f;
        case POD_FUNC_SAMPLE_RATE:
            return gFilterSrReduce == 0 ? 0.0f
                : logf(clampF(static_cast<float>(gFilterSrReduce), 4000.0f,
                              42000.0f) / 42000.0f)
                  / logf(4000.0f / 42000.0f);
        case POD_FUNC_FILTER_TYPE:
            return static_cast<float>(gFilterType) / FTYPE_COMB;
        default: return 0.0f;
    }
}

static void ApplyPodAbsoluteFunction(uint8_t function, uint16_t raw)
{
    const float normalized = static_cast<float>(raw) / 1000.0f;
    switch(function)
    {
        case POD_FUNC_MASTER_VOLUME:
        case POD_FUNC_SEQ_VOLUME:
        case POD_FUNC_LIVE_VOLUME: {
            const uint8_t value = static_cast<uint8_t>(normalized * 150.0f + 0.5f);
            const uint8_t cmd = function == POD_FUNC_MASTER_VOLUME ? CMD_MASTER_VOLUME
                              : function == POD_FUNC_SEQ_VOLUME ? CMD_SEQ_VOLUME
                              : CMD_LIVE_VOLUME;
            ApplyPodCommand(cmd, &value, sizeof(value));
            break;
        }
        case POD_FUNC_TEMPO: {
            const float value = 40.0f + normalized * 200.0f;
            ApplyPodCommand(CMD_TEMPO, &value, sizeof(value));
            break;
        }
        case POD_FUNC_SELECT_PAD:
            podSelectedPad = static_cast<uint8_t>((raw * 15u + 500u) / 1000u);
            break;
        case POD_FUNC_DELAY_MIX: {
            const uint8_t enabled = normalized > 0.005f ? 1u : 0u;
            ApplyPodCommand(CMD_DELAY_ACTIVE, &enabled, sizeof(enabled));
            ApplyPodCommand(CMD_DELAY_MIX, &normalized, sizeof(normalized));
            break;
        }
        case POD_FUNC_REVERB_MIX: {
            const uint8_t enabled = normalized > 0.005f ? 1u : 0u;
            ApplyPodCommand(CMD_REVERB_ACTIVE, &enabled, sizeof(enabled));
            ApplyPodCommand(CMD_REVERB_MIX, &normalized, sizeof(normalized));
            break;
        }
        case POD_FUNC_FLANGER_DEPTH:
        case POD_FUNC_WAVEFOLDER_GAIN:
        case POD_FUNC_CRUSH_MACRO:
        case POD_FUNC_PHASER_DEPTH:
        case POD_FUNC_FILTER_CUTOFF:
        case POD_FUNC_FILTER_RESONANCE:
        case POD_FUNC_DISTORTION:
        case POD_FUNC_BIT_DEPTH:
        case POD_FUNC_SAMPLE_RATE:
        case POD_FUNC_FILTER_TYPE:
            ApplyPodFxNormalized(function, normalized);
            break;
        default: break;
    }
    podStateRevision++;
}

static void ApplyPodEncoderFunction(uint8_t function, int increment)
{
    if(increment == 0) return;
    switch(function)
    {
        case POD_FUNC_SELECT_PAD: {
            int pad = static_cast<int>(podSelectedPad) + (increment > 0 ? 1 : -1);
            if(pad < 0) pad = 15;
            if(pad > 15) pad = 0;
            podSelectedPad = static_cast<uint8_t>(pad);
            break;
        }
        case POD_FUNC_TEMPO: {
            float value = podCurrentBpmX10 / 10.0f + static_cast<float>(increment);
            value = clampF(value, 40.0f, 240.0f);
            ApplyPodCommand(CMD_TEMPO, &value, sizeof(value));
            break;
        }
        case POD_FUNC_MASTER_VOLUME:
        case POD_FUNC_SEQ_VOLUME:
        case POD_FUNC_LIVE_VOLUME: {
            uint8_t* current = function == POD_FUNC_MASTER_VOLUME ? &podCurrentMasterVolume
                             : function == POD_FUNC_SEQ_VOLUME ? &podCurrentSeqVolume
                             : &podCurrentLiveVolume;
            int value = static_cast<int>(*current) + (increment > 0 ? 2 : -2);
            if(value < 0) value = 0;
            if(value > 150) value = 150;
            const uint8_t sendValue = static_cast<uint8_t>(value);
            const uint8_t cmd = function == POD_FUNC_MASTER_VOLUME ? CMD_MASTER_VOLUME
                              : function == POD_FUNC_SEQ_VOLUME ? CMD_SEQ_VOLUME
                              : CMD_LIVE_VOLUME;
            ApplyPodCommand(cmd, &sendValue, sizeof(sendValue));
            break;
        }
        case POD_FUNC_PATTERN_PREV:
        case POD_FUNC_PATTERN_NEXT:
            return;
        case POD_FUNC_DELAY_MIX: {
            float value = clampF(delayMix + (increment > 0 ? 0.02f : -0.02f),
                                 0.0f, 1.0f);
            const uint8_t enabled = value > 0.005f ? 1u : 0u;
            ApplyPodCommand(CMD_DELAY_ACTIVE, &enabled, sizeof(enabled));
            ApplyPodCommand(CMD_DELAY_MIX, &value, sizeof(value));
            break;
        }
        case POD_FUNC_REVERB_MIX: {
            float value = clampF(reverbMix + (increment > 0 ? 0.02f : -0.02f),
                                 0.0f, 1.0f);
            const uint8_t enabled = value > 0.005f ? 1u : 0u;
            ApplyPodCommand(CMD_REVERB_ACTIVE, &enabled, sizeof(enabled));
            ApplyPodCommand(CMD_REVERB_MIX, &value, sizeof(value));
            break;
        }
        case POD_FUNC_FLANGER_DEPTH:
        case POD_FUNC_WAVEFOLDER_GAIN:
        case POD_FUNC_CRUSH_MACRO:
        case POD_FUNC_PHASER_DEPTH:
        case POD_FUNC_FILTER_CUTOFF:
        case POD_FUNC_FILTER_RESONANCE:
        case POD_FUNC_DISTORTION:
        case POD_FUNC_BIT_DEPTH:
        case POD_FUNC_SAMPLE_RATE:
        case POD_FUNC_FILTER_TYPE: {
            const float step = function == POD_FUNC_FILTER_TYPE
                ? (1.0f / FTYPE_COMB) : 0.02f;
            const float value = clampF(PodFxCurrentNormalized(function)
                + (increment > 0 ? step : -step), 0.0f, 1.0f);
            ApplyPodFxNormalized(function, value);
            break;
        }
        default: break;
    }
    podStateRevision++;
}

/* ──────────────────────────────────────────────────────────────────
 * MERGED MIDI CONTROLLERS (2 devices x 3 programs x 3 A/B/C layers)
 *
 * Program 1/2/3 must transmit on MIDI channel 1/2/3 respectively.
 * The fixed note/CC layout is declared in mpd218_mapping.h. Keeping the
 * table separate from the dispatcher makes all 198 physical positions easy
 * to audit and remap without touching the audio engine.
 * ────────────────────────────────────────────────────────────────── */
static uint8_t  mpdSelectedTrack[red808_mpd218::kDeviceCount] = {0, 0};
static uint8_t  mpdSelectedPad[red808_mpd218::kDeviceCount] = {0, 0};
static uint8_t  mpdSelectedDrumEngine[red808_mpd218::kDeviceCount] = {
    SYNTH_ENGINE_808, SYNTH_ENGINE_808};
static uint8_t  mpdSelectedDrumPad[red808_mpd218::kDeviceCount] = {0, 0};
static uint8_t  mpdSelectedMelodicEngine[red808_mpd218::kDeviceCount] = {
    SYNTH_ENGINE_303, SYNTH_ENGINE_303};
static uint8_t  mpdSelectedSynthPreset[red808_mpd218::kDeviceCount] = {0, 0};
static uint32_t mpdClockStartMs           = 0;
static uint8_t  mpdClockIntervals         = 0;

static uint16_t MpdRaw1000(uint8_t value)
{
    return static_cast<uint16_t>((static_cast<uint32_t>(value) * 1000u + 63u)
                                 / 127u);
}

static uint8_t MpdSelectedNativeDrumInstrument(uint8_t device)
{
    const uint8_t pad = mpdSelectedDrumPad[device] & 0x0Fu;
    switch(mpdSelectedDrumEngine[device])
    {
        case SYNTH_ENGINE_808: return padTo808[pad];
        case SYNTH_ENGINE_909: return padTo909[pad];
        case SYNTH_ENGINE_505: return padTo505[pad];
        default: return 0;
    }
}

static void MpdSendTrackFilter(uint8_t device, float cutoff, float resonance)
{
    const uint8_t track = mpdSelectedTrack[device];
    struct __attribute__((packed)) TrackFilterPayload
    {
        uint8_t track;
        uint8_t type;
        uint8_t reserved[2];
        float cutoff;
        float resonance;
    } payload = {
        track,
        trkFilterType[track] == FTYPE_NONE
            ? static_cast<uint8_t>(FTYPE_LOWPASS)
            : trkFilterType[track],
        {0, 0},
        cutoff,
        resonance,
    };
    static_assert(sizeof(TrackFilterPayload) == 12,
                  "MPD track-filter payload layout changed");
    ApplyPodCommand(CMD_TRACK_FILTER, &payload, sizeof(payload));
}

static void MpdSendSynthParam(uint8_t engine,
                              uint8_t instrument,
                              uint8_t param,
                              float value)
{
    struct __attribute__((packed)) SynthParamPayload
    {
        uint8_t engine;
        uint8_t instrument;
        uint8_t param;
        float value;
    } payload = {engine, instrument, param, value};
    static_assert(sizeof(SynthParamPayload) == 7,
                  "MPD synth-param payload layout changed");
    ApplyPodCommand(CMD_SYNTH_PARAM, &payload, sizeof(payload));
}

static void MpdLoadSynthPreset(uint8_t engine, uint8_t preset)
{
    /* CMD_SYNTH_PRESET intentionally releases every melodic engine for the
     * single-controller web UI. On the merged MIDI bus that would let device A
     * cut device B. Reset only the engine whose preset is being changed. */
    ReleaseSynthEngineState(engine);
    ApplySynthPreset(engine, preset);
    podStateRevision++;
}

static void MpdSelectTrack(uint8_t device, uint8_t track)
{
    mpdSelectedTrack[device] = track % DSQ_TRACKS;
    mpdSelectedPad[device] = mpdSelectedTrack[device];
    podSelectedPad = mpdSelectedTrack[device];
    podStateRevision++;
}

static void MpdApplyPad(uint8_t device,
                        const red808_mpd218::PadAction& action,
                        uint8_t velocity,
                        bool pressed,
                        uint32_t now)
{
    using namespace red808_mpd218;

    /* Only pitched/melodic pads need their release event. */
    if(!pressed)
    {
        if(action.type == PAD_TRIGGER_MELODIC)
        {
            const uint8_t release[3] = {
                mpdSelectedMelodicEngine[device], 0xFFu, action.arg0};
            ApplyPodCommand(CMD_SYNTH_NOTE_OFF, release, sizeof(release));
        }
        return;
    }

    switch(action.type)
    {
        case PAD_TRIGGER_SAMPLE: {
            const uint8_t trigger[2] = {action.arg0, velocity};
            ApplyPodCommand(CMD_TRIGGER_LIVE, trigger, sizeof(trigger));
            mpdSelectedPad[device] = action.arg0;
            podSelectedPad = action.arg0;
            if(action.arg0 < DSQ_TRACKS)
                mpdSelectedTrack[device] = action.arg0;
            podPadPulseUntilMs = now + 100u;
            break;
        }
        case PAD_TRIGGER_MELODIC: {
            const uint8_t noteOn[5] = {
                mpdSelectedMelodicEngine[device],
                action.arg0,
                velocity,
                static_cast<uint8_t>(velocity >= 112u),
                0u,
            };
            ApplyPodCommand(CMD_SYNTH_NOTE_ON_EX, noteOn, sizeof(noteOn));
            podPadPulseUntilMs = now + 100u;
            break;
        }
        case PAD_SELECT_PATTERN: {
            const uint8_t pattern = action.arg0 % DSQ_PATTERNS;
            ApplyPodCommand(CMD_DSQ_SELECT_PATTERN, &pattern, sizeof(pattern));
            break;
        }
        case PAD_TOGGLE_TRACK_MUTE: {
            const uint8_t track = action.arg0 % DSQ_TRACKS;
            const uint8_t mute[2] = {
                track, static_cast<uint8_t>(!dseq.trackMuted[track])};
            ApplyPodCommand(CMD_TRACK_MUTE, mute, sizeof(mute));
            ApplyPodCommand(CMD_DSQ_SET_MUTE, mute, sizeof(mute));
            break;
        }
        case PAD_SELECT_TRACK:
            MpdSelectTrack(device, action.arg0);
            break;
        case PAD_TRIGGER_SYNTH: {
            const uint8_t trigger[3] = {action.arg0, action.arg1, velocity};
            mpdSelectedDrumEngine[device] = action.arg0;
            mpdSelectedDrumPad[device] = action.arg1;
            ApplyPodCommand(CMD_SYNTH_TRIGGER, trigger, sizeof(trigger));
            podPadPulseUntilMs = now + 100u;
            break;
        }
        case PAD_PLAY_TOGGLE:
            ApplyPodButtonFunction(POD_FUNC_PLAY_TOGGLE, now);
            break;
        case PAD_STOP_ALL:
            ApplyPodButtonFunction(POD_FUNC_STOP, now);
            ApplyPodCommand(CMD_SYNTH_NOTE_OFF, nullptr, 0);
            break;
        case PAD_PATTERN_PREV:
            ApplyPodButtonFunction(POD_FUNC_PATTERN_PREV, now);
            break;
        case PAD_PATTERN_NEXT:
            ApplyPodButtonFunction(POD_FUNC_PATTERN_NEXT, now);
            break;
        case PAD_TOGGLE_LOOP: {
            const uint8_t pad = mpdSelectedPad[device];
            const uint8_t payload[2] = {
                pad, static_cast<uint8_t>(!padLoop[pad])};
            ApplyPodCommand(CMD_PAD_LOOP, payload, sizeof(payload));
            break;
        }
        case PAD_TOGGLE_REVERSE: {
            const uint8_t pad = mpdSelectedPad[device];
            const uint8_t payload[2] = {
                pad, static_cast<uint8_t>(!padReverse[pad])};
            ApplyPodCommand(CMD_PAD_REVERSE, payload, sizeof(payload));
            break;
        }
        case PAD_TOGGLE_STUTTER: {
            const uint8_t pad = mpdSelectedPad[device];
            struct __attribute__((packed)) StutterPayload
            {
                uint8_t pad;
                uint8_t enabled;
                uint16_t interval;
            } payload = {
                pad,
                static_cast<uint8_t>(!padStutterOn[pad]),
                240u,
            };
            ApplyPodCommand(CMD_PAD_STUTTER, &payload, sizeof(payload));
            break;
        }
        case PAD_CLEAR_SELECTED_FX: {
            const uint8_t pad = mpdSelectedPad[device];
            ApplyPodCommand(CMD_PAD_CLEAR_FX, &pad, sizeof(pad));
            break;
        }
        case PAD_NONE:
        default:
            break;
    }
    podStateRevision++;
}

static float MpdDrumParamValue(uint8_t device, uint8_t param, float normalized)
{
    switch(param)
    {
        case 0: return 0.01f + normalized * 1.49f;                 /* decay */
        case 1:
            /* The 808 cowbell exposes a tune ratio, while pitched drums use Hz. */
            return mpdSelectedDrumEngine[device] == SYNTH_ENGINE_808
                       && mpdSelectedDrumPad[device] == 7
                ? 0.7f + normalized * 0.8f
                : 30.0f * powf(100.0f, normalized);
        case 2: return normalized;                                /* tone/drive */
        case 3: return normalized;                                /* volume */
        case 4: return normalized;                                /* snappy/sub */
        case 5: return mpdSelectedDrumPad[device] == 0
                     ? 1.0f + normalized * 19.0f : normalized;    /* punch/smack */
        default: return normalized;
    }
}

static void MpdApplyMelodicParam(uint8_t device,
                                 uint8_t slot,
                                 float normalized)
{
    uint8_t param = slot;
    float value = normalized;

    const uint8_t engine = mpdSelectedMelodicEngine[device];
    switch(engine)
    {
        case SYNTH_ENGINE_303:
            if(slot == 0) value = 60.0f * powf(200.0f, normalized);
            else if(slot == 3) value = 0.03f + normalized * 1.97f;
            break;
        case SYNTH_ENGINE_WTOSC:
            if(slot == 1) value = normalized * 2.0f;
            else if(slot == 2) value = 0.01f + normalized * 3.0f;
            else if(slot == 4) value = 20.0f * powf(900.0f, normalized);
            else if(slot == 5) value = 0.01f * powf(2000.0f, normalized);
            break;
        case SYNTH_ENGINE_SH101: {
            static const uint8_t params[6] = {4, 5, 6, 8, 10, 13};
            param = params[slot];
            if(slot == 0) value = 20.0f * powf(500.0f, normalized);
            else if(slot == 1) value = normalized * 0.95f;
            else if(slot == 3) value = 0.01f + normalized * 2.99f;
            else if(slot == 4) value = 0.005f + normalized * 1.995f;
            else if(slot == 5) value = 0.1f * powf(200.0f, normalized);
            break;
        }
        case SYNTH_ENGINE_FM2OP: {
            static const uint8_t params[6] = {8, 9, 10, 0, 1, 14};
            param = params[slot];
            if(slot == 0) value = 0.5f * powf(32.0f, normalized);
            else if(slot == 1) value = normalized * 20.0f;
            else if(slot == 3) value = 0.001f + normalized * 1.999f;
            else if(slot == 4) value = 0.01f + normalized * 4.99f;
            break;
        }
        case SYNTH_ENGINE_PHYS:
            if(slot == 0 || slot == 5)
                value = 20.0f * powf(500.0f, normalized);
            break;
        default:
            break;
    }

    MpdSendSynthParam(engine, mpdSelectedTrack[device], param, value);
}

static void MpdApplyKnob(uint8_t device,
                         const red808_mpd218::KnobAction& action,
                         uint8_t midiValue)
{
    using namespace red808_mpd218;
    const float normalized = static_cast<float>(midiValue) / 127.0f;
    const uint16_t raw = MpdRaw1000(midiValue);
    const uint8_t track = mpdSelectedTrack[device];

    switch(action.type)
    {
        case KNOB_MASTER_VOLUME:
            ApplyPodAbsoluteFunction(POD_FUNC_MASTER_VOLUME, raw); break;
        case KNOB_LIVE_VOLUME:
            ApplyPodAbsoluteFunction(POD_FUNC_LIVE_VOLUME, raw); break;
        case KNOB_SEQ_VOLUME:
            ApplyPodAbsoluteFunction(POD_FUNC_SEQ_VOLUME, raw); break;
        case KNOB_TEMPO:
        case KNOB_SEQ_TEMPO:
            ApplyPodAbsoluteFunction(POD_FUNC_TEMPO, raw); break;
        case KNOB_DELAY_MIX:
            ApplyPodAbsoluteFunction(POD_FUNC_DELAY_MIX, raw); break;
        case KNOB_REVERB_MIX:
            ApplyPodAbsoluteFunction(POD_FUNC_REVERB_MIX, raw); break;
        case KNOB_FILTER_CUTOFF:
            ApplyPodAbsoluteFunction(POD_FUNC_FILTER_CUTOFF, raw); break;
        case KNOB_FILTER_RESONANCE:
            ApplyPodAbsoluteFunction(POD_FUNC_FILTER_RESONANCE, raw); break;
        case KNOB_DISTORTION:
            ApplyPodAbsoluteFunction(POD_FUNC_DISTORTION, raw); break;
        case KNOB_BIT_DEPTH:
            ApplyPodAbsoluteFunction(POD_FUNC_BIT_DEPTH, raw); break;
        case KNOB_SAMPLE_RATE:
            ApplyPodAbsoluteFunction(POD_FUNC_SAMPLE_RATE, raw); break;
        case KNOB_FILTER_TYPE:
            ApplyPodAbsoluteFunction(POD_FUNC_FILTER_TYPE, raw); break;
        case KNOB_FLANGER_DEPTH:
            ApplyPodAbsoluteFunction(POD_FUNC_FLANGER_DEPTH, raw); break;
        case KNOB_PHASER_DEPTH:
            ApplyPodAbsoluteFunction(POD_FUNC_PHASER_DEPTH, raw); break;
        case KNOB_WAVEFOLDER:
            ApplyPodAbsoluteFunction(POD_FUNC_WAVEFOLDER_GAIN, raw); break;
        case KNOB_CRUSH_MACRO:
            ApplyPodAbsoluteFunction(POD_FUNC_CRUSH_MACRO, raw); break;
        case KNOB_LIVE_PITCH: {
            const float pitch = 0.5f * powf(4.0f, normalized);
            ApplyPodCommand(CMD_LIVE_PITCH, &pitch, sizeof(pitch));
            break;
        }
        case KNOB_SELECT_PAD:
            ApplyPodAbsoluteFunction(POD_FUNC_SELECT_PAD, raw);
            MpdSelectTrack(device, podSelectedPad);
            break;
        case KNOB_TRACK_VOLUME: {
            const uint8_t payload[2] = {
                track, static_cast<uint8_t>(normalized * 150.0f + 0.5f)};
            ApplyPodCommand(CMD_TRACK_VOLUME, payload, sizeof(payload));
            break;
        }
        case KNOB_TRACK_PAN: {
            const int8_t pan = static_cast<int8_t>(normalized * 200.0f - 100.0f);
            const uint8_t payload[2] = {track, static_cast<uint8_t>(pan)};
            ApplyPodCommand(CMD_TRACK_PAN, payload, sizeof(payload));
            break;
        }
        case KNOB_TRACK_REVERB_SEND:
        case KNOB_TRACK_DELAY_SEND:
        case KNOB_TRACK_CHORUS_SEND: {
            const uint8_t payload[2] = {
                track, static_cast<uint8_t>(normalized * 100.0f + 0.5f)};
            const uint8_t cmd = action.type == KNOB_TRACK_REVERB_SEND
                ? CMD_TRACK_REVERB_SEND
                : action.type == KNOB_TRACK_DELAY_SEND
                    ? CMD_TRACK_DELAY_SEND : CMD_TRACK_CHORUS_SEND;
            ApplyPodCommand(cmd, payload, sizeof(payload));
            break;
        }
        case KNOB_TRACK_PITCH: {
            struct __attribute__((packed)) TrackPitchPayload
            {
                uint8_t track;
                uint8_t reserved;
                int16_t cents;
            } payload = {
                track, 0,
                static_cast<int16_t>(normalized * 2400.0f - 1200.0f)};
            ApplyPodCommand(CMD_TRACK_PITCH, &payload, sizeof(payload));
            break;
        }
        case KNOB_TRACK_FILTER_CUTOFF: {
            const float cutoff = 20.0f * powf(1000.0f, normalized);
            const float q = trkFilterQ[track] >= 0.3f ? trkFilterQ[track] : 0.707f;
            MpdSendTrackFilter(device, cutoff, q);
            break;
        }
        case KNOB_TRACK_FILTER_RESONANCE: {
            const float cutoff = trkFilterCut[track] >= 20.0f
                ? trkFilterCut[track] : 12000.0f;
            MpdSendTrackFilter(device, cutoff, 0.3f + normalized * 27.7f);
            break;
        }
        case KNOB_TRACK_DISTORTION: {
            struct __attribute__((packed)) TrackDistPayload
            {
                uint8_t track;
                uint8_t mode;
                uint8_t reserved[2];
                float amount;
            } payload = {track, DMODE_SOFT, {0, 0}, normalized};
            ApplyPodCommand(CMD_TRACK_DISTORTION, &payload, sizeof(payload));
            break;
        }
        case KNOB_TRACK_BIT_DEPTH: {
            const uint8_t payload[2] = {
                track,
                static_cast<uint8_t>(16.0f - normalized * 12.0f + 0.5f)};
            ApplyPodCommand(CMD_TRACK_BITCRUSH, payload, sizeof(payload));
            break;
        }
        case KNOB_TRACK_EQ_LOW:
        case KNOB_TRACK_EQ_HIGH: {
            const int8_t db = static_cast<int8_t>(normalized * 24.0f - 12.0f);
            const uint8_t payload[2] = {track, static_cast<uint8_t>(db)};
            ApplyPodCommand(action.type == KNOB_TRACK_EQ_LOW
                                ? CMD_TRACK_EQ_LOW : CMD_TRACK_EQ_HIGH,
                            payload, sizeof(payload));
            break;
        }
        case KNOB_SEQ_SWING: {
            const uint8_t value = static_cast<uint8_t>(normalized * 100.0f + 0.5f);
            ApplyPodCommand(CMD_DSQ_SET_SWING, &value, sizeof(value));
            break;
        }
        case KNOB_HUMANIZE_TIMING: {
            const uint8_t payload[2] = {
                static_cast<uint8_t>(normalized * 20.0f + 0.5f),
                dseq.humanizeVelAmt};
            ApplyPodCommand(CMD_DSQ_SET_HUMANIZE, payload, sizeof(payload));
            break;
        }
        case KNOB_HUMANIZE_VELOCITY: {
            const uint8_t payload[2] = {
                dseq.humanizeTimingMs,
                static_cast<uint8_t>(normalized * 50.0f + 0.5f)};
            ApplyPodCommand(CMD_DSQ_SET_HUMANIZE, payload, sizeof(payload));
            break;
        }
        case KNOB_PATTERN_SELECT: {
            const uint8_t pattern = static_cast<uint8_t>(
                normalized * static_cast<float>(DSQ_PATTERNS - 1) + 0.5f);
            ApplyPodCommand(CMD_DSQ_SELECT_PATTERN, &pattern, sizeof(pattern));
            break;
        }
        case KNOB_TRACK_SELECT:
            MpdSelectTrack(device, static_cast<uint8_t>(normalized
                           * static_cast<float>(DSQ_TRACKS - 1) + 0.5f));
            break;
        case KNOB_DRUM_PARAM_1:
        case KNOB_DRUM_PARAM_2:
        case KNOB_DRUM_PARAM_3:
        case KNOB_DRUM_PARAM_4:
        case KNOB_DRUM_PARAM_5:
        case KNOB_DRUM_PARAM_6: {
            const uint8_t param = static_cast<uint8_t>(
                action.type - KNOB_DRUM_PARAM_1);
            MpdSendSynthParam(mpdSelectedDrumEngine[device],
                              MpdSelectedNativeDrumInstrument(device),
                              param,
                              MpdDrumParamValue(device, param, normalized));
            break;
        }
        case KNOB_MELODIC_PARAM_1:
        case KNOB_MELODIC_PARAM_2:
        case KNOB_MELODIC_PARAM_3:
        case KNOB_MELODIC_PARAM_4:
        case KNOB_MELODIC_PARAM_5:
        case KNOB_MELODIC_PARAM_6:
            MpdApplyMelodicParam(device, static_cast<uint8_t>(
                action.type - KNOB_MELODIC_PARAM_1), normalized);
            break;
        case KNOB_SYNTH_ENGINE: {
            static const uint8_t engines[5] = {
                SYNTH_ENGINE_303, SYNTH_ENGINE_WTOSC, SYNTH_ENGINE_SH101,
                SYNTH_ENGINE_FM2OP, SYNTH_ENGINE_PHYS};
            const uint8_t index = static_cast<uint8_t>(normalized * 4.0f + 0.5f);
            const uint8_t previousEngine = mpdSelectedMelodicEngine[device];
            if(previousEngine != engines[index])
                ReleaseSynthEngineState(previousEngine);
            mpdSelectedMelodicEngine[device] = engines[index];
            MpdLoadSynthPreset(engines[index], mpdSelectedSynthPreset[device]);
            break;
        }
        case KNOB_SYNTH_PRESET: {
            mpdSelectedSynthPreset[device] = static_cast<uint8_t>(
                normalized * 4.0f + 0.5f);
            MpdLoadSynthPreset(mpdSelectedMelodicEngine[device],
                               mpdSelectedSynthPreset[device]);
            break;
        }
        case KNOB_CHORUS_MIX: {
            const uint8_t enabled = normalized > 0.005f ? 1u : 0u;
            ApplyPodCommand(CMD_CHORUS_ACTIVE, &enabled, sizeof(enabled));
            ApplyPodCommand(CMD_CHORUS_MIX, &normalized, sizeof(normalized));
            break;
        }
        case KNOB_TREMOLO_DEPTH: {
            const uint8_t enabled = normalized > 0.005f ? 1u : 0u;
            ApplyPodCommand(CMD_TREMOLO_ACTIVE, &enabled, sizeof(enabled));
            ApplyPodCommand(CMD_TREMOLO_DEPTH, &normalized, sizeof(normalized));
            break;
        }
        case KNOB_AUTOWAH_MIX: {
            const uint8_t enabled = normalized > 0.005f ? 1u : 0u;
            ApplyPodCommand(CMD_AUTOWAH_ACTIVE, &enabled, sizeof(enabled));
            ApplyPodCommand(CMD_AUTOWAH_MIX, &normalized, sizeof(normalized));
            break;
        }
        case KNOB_STEREO_WIDTH: {
            const uint8_t width = static_cast<uint8_t>(normalized * 200.0f + 0.5f);
            ApplyPodCommand(CMD_STEREO_WIDTH, &width, sizeof(width));
            break;
        }
        case KNOB_NONE:
        default:
            break;
    }
    podStateRevision++;
}

static void MpdHandleClock(uint32_t now)
{
    if(mpdClockStartMs == 0u)
    {
        mpdClockStartMs = now;
        mpdClockIntervals = 0;
        return;
    }

    if(++mpdClockIntervals >= 24u)
    {
        const uint32_t elapsed = now - mpdClockStartMs;
        mpdClockStartMs = now;
        mpdClockIntervals = 0;
        if(elapsed >= 80u && elapsed <= 1500u)
        {
            const float bpm = clampF(60000.0f / static_cast<float>(elapsed),
                                     40.0f, 300.0f);
            ApplyPodCommand(CMD_TEMPO, &bpm, sizeof(bpm));
        }
    }
}

static void ProcessMpdMidi()
{
    pod.midi->Listen();
    uint8_t budget = 192;
    while(pod.midi->HasEvents() && budget-- > 0u)
    {
        MidiEvent event = pod.midi->PopEvent();
        uint8_t device = 0, bank = 0, layer = 0, index = 0;
        const uint32_t now = hw.system.GetNow();

        switch(event.type)
        {
            case NoteOn:
            case NoteOff: {
                const uint8_t channel = static_cast<uint8_t>(event.channel);
                const bool pressed = event.type == NoteOn && event.data[1] != 0u;
                /* Only mirror actual presses into the monitor ring — LEARN
                 * and the on-screen pad glow only care about note-on, and
                 * note-off/vel=0 events (roughly half of all pad traffic)
                 * would otherwise fill the ring and delay/skew what a
                 * freshly armed LEARN captures. */
                if(pressed)
                    MidiMonitorPush(static_cast<uint8_t>(0x90u
                                                         | (channel & 0x0Fu)),
                                    event.data[0], event.data[1]);
                /* A learned assignment beats the compiled factory map and
                 * works on any channel/note the controller happens to send. */
                if(const MidiMapEntry* learned
                   = MidiUserMapFind(channel, MIDI_MAP_KIND_NOTE,
                                     event.data[0]))
                {
                    uint8_t mapDevice = 0, mapBank = 0;
                    red808_mpd218::DecodeDeviceAndBank(channel, mapDevice,
                                                       mapBank);
                    const red808_mpd218::PadAction action = {
                        static_cast<red808_mpd218::PadActionType>(
                            learned->action),
                        learned->arg0, learned->arg1};
                    MpdApplyPad(mapDevice, action, event.data[1], pressed,
                                now);
                }
                else if(red808_mpd218::DecodePad(channel, event.data[0],
                                                 device, bank, layer, index))
                    MpdApplyPad(device,
                                red808_mpd218::kPadMap[bank][layer][index],
                                event.data[1], pressed, now);
                break;
            }
            case ControlChange: {
                const uint8_t channel = static_cast<uint8_t>(event.channel);
                MidiMonitorPush(static_cast<uint8_t>(0xB0u
                                                     | (channel & 0x0Fu)),
                                event.data[0], event.data[1]);
                if(const MidiMapEntry* learned
                   = MidiUserMapFind(channel, MIDI_MAP_KIND_CC,
                                     event.data[0]))
                {
                    uint8_t mapDevice = 0, mapBank = 0;
                    red808_mpd218::DecodeDeviceAndBank(channel, mapDevice,
                                                       mapBank);
                    const red808_mpd218::KnobAction action = {
                        static_cast<red808_mpd218::KnobActionType>(
                            learned->action)};
                    MpdApplyKnob(mapDevice, action, event.data[1]);
                }
                else if(red808_mpd218::DecodeKnob(channel, event.data[0],
                                                  device, bank, layer, index))
                    MpdApplyKnob(device,
                                 red808_mpd218::kKnobMap[bank][layer][index],
                                 event.data[1]);
                break;
            }
            case ChannelMode:
                if(event.cm_type == AllSoundOff || event.cm_type == AllNotesOff)
                {
                    uint8_t modeDevice = 0, modeBank = 0;
                    if(red808_mpd218::DecodeDeviceAndBank(
                           static_cast<uint8_t>(event.channel),
                           modeDevice, modeBank))
                    {
                        (void)modeBank;
                        const uint8_t release[2] = {
                            mpdSelectedMelodicEngine[modeDevice], 0xFFu};
                        ApplyPodCommand(CMD_SYNTH_NOTE_OFF,
                                        release, sizeof(release));
                    }
                }
                break;
            case SystemRealTime:
                if(event.srt_type == TimingClock)
                    MpdHandleClock(now);
                else if(event.srt_type == Start || event.srt_type == Continue)
                {
                    const uint8_t play = 1u;
                    ApplyPodCommand(CMD_DSQ_CONTROL, &play, sizeof(play));
                }
                else if(event.srt_type == Stop)
                {
                    const uint8_t stop = 0u;
                    ApplyPodCommand(CMD_DSQ_CONTROL, &stop, sizeof(stop));
                }
                else if(event.srt_type == Reset)
                {
                    const uint8_t reset = 2u;
                    ApplyPodCommand(CMD_DSQ_CONTROL, &reset, sizeof(reset));
                    ApplyPodCommand(CMD_TRIGGER_STOP_ALL, nullptr, 0);
                    ApplyPodCommand(CMD_SYNTH_NOTE_OFF, nullptr, 0);
                }
                break;
            default:
                break;
        }
    }
}

static bool PodLedIsActive(uint8_t function, uint32_t now)
{
    switch(function)
    {
        case POD_LED_FIXED: return true;
        case POD_LED_USB_LINK:
            return usbLastPacketMs != 0u && (now - usbLastPacketMs) < 3000u;
        case POD_LED_PLAY_STATE: return dseq.playing;
        case POD_LED_PAD_ACTIVITY:
            return now < podPadPulseUntilMs || ActiveVoices() > 0;
        case POD_LED_SD_STATE: return sdPresent;
        case POD_LED_SAMPLES_READY:
        {
            bool samplesReady = false;
            for(uint8_t i = 0; i < DSQ_TRACKS; i++)
                if(sampleLoaded[i]) { samplesReady = true; break; }
            const uint8_t pattern = dseq.currentPattern % DSQ_PATTERNS;
            const bool patternReady =
                (dsqLoadedPatternMask & (1u << pattern)) != 0;
            return samplesReady && patternReady;
        }
        default: return false;
    }
}

static void ApplyPodLed(uint8_t index, uint8_t function,
                        uint8_t red, uint8_t green, uint8_t blue,
                        uint32_t now)
{
    const bool active = PodLedIsActive(function, now);
    const float scale = active ? 0.85f : 0.015f;
    podLedRgb[index][0] = active ? red : 0;
    podLedRgb[index][1] = active ? green : 0;
    podLedRgb[index][2] = active ? blue : 0;
    const float r = (red / 255.0f) * scale;
    const float g = (green / 255.0f) * scale;
    const float b = (blue / 255.0f) * scale;
    if(index == 0) pod.led1.Set(r, g, b);
    else           pod.led2.Set(r, g, b);
}

static void ProcessDaisyUsb()
{
    uint16_t budget = USB_RX_RING_SIZE;
    while(usbRxTail != usbRxHead && budget-- > 0)
    {
        __DMB(); /* pairs with the producer's __DMB() before it publishes head */
        const uint8_t byte = usbRxRing[usbRxTail];
        usbRxTail = (usbRxTail + 1u) & USB_RX_RING_MASK;

        if(usbParseIdx == 0)
        {
            if(byte == SPI_MAGIC_CMD)
                usbParseBuf[usbParseIdx++] = byte;
            continue;
        }

        if(usbParseIdx >= RX_BUF_SIZE)
        {
            usbParseIdx = 0;
            continue;
        }
        usbParseBuf[usbParseIdx++] = byte;
        if(usbParseIdx < sizeof(SPIPacketHeader))
            continue;

        const SPIPacketHeader* header
            = reinterpret_cast<const SPIPacketHeader*>(usbParseBuf);
        const uint16_t packetLength
            = sizeof(SPIPacketHeader) + header->length;
        if(packetLength > RX_BUF_SIZE)
        {
            usbParseIdx = 0;
            continue;
        }
        if(usbParseIdx != packetLength)
            continue;

        memcpy(rxBuf, usbParseBuf, packetLength);
        usbParseIdx = 0;
        usbLastPacketMs = hw.system.GetNow();
        ProcessCommand();
        /* txBuf is a single response slot. Stop consuming the RX ring as soon
         * as a command produces a response, transmit it below, and continue
         * with the queued command on the next main-loop pass. This prevents a
         * following PING/STATUS from overwriting SAMPLE_END's commit ACK. */
        if(pendingResponse)
            break;
    }

    if(pendingResponse && pendingTxLen > 0)
    {
        if(hw.usb_handle.TransmitInternal(txBuf, pendingTxLen)
           == UsbHandle::Result::OK)
        {
            pendingResponse = false;
            pendingTxLen = 0;
        }
    }
}

static void ProcessPodControls(uint32_t now)
{
    static uint32_t lastScanMs = 0;
    static uint32_t lastLedMs = 0;

    if(now - lastScanMs < 1u)
        return;
    lastScanMs = now;
    pod.ProcessAllControls();

    const int increment = pod.encoder.Increment();
    if(increment != 0)
    {
        if(podConfig.encoderFunction == POD_FUNC_PATTERN_PREV
           || podConfig.encoderFunction == POD_FUNC_PATTERN_NEXT)
        {
            podEncoderPosition += increment;
            podStateRevision++;
        }
        else
        {
            ApplyPodEncoderFunction(podConfig.encoderFunction, increment);
        }
    }

    if(pod.button1.RisingEdge())
    {
        podButtonPressEvents |= 1u;
        ApplyPodButtonFunction(podConfig.button1Function, now);
    }

    if(pod.button2.RisingEdge())
    {
        podButtonPressEvents |= 2u;
        ApplyPodButtonFunction(podConfig.button2Function, now);
    }

    if(pod.encoder.RisingEdge())
    {
        podButtonPressEvents |= 4u;
        ApplyPodButtonFunction(podConfig.encoderButtonFunction, now);
    }

    podKnobRaw[0] = static_cast<uint16_t>(pod.knob1.Value() * 1000.0f + 0.5f);
    podKnobRaw[1] = static_cast<uint16_t>(pod.knob2.Value() * 1000.0f + 0.5f);
    for(uint8_t knob = 0; knob < 2; knob++)
    {
        if(podLastKnobRaw[knob] < 0
           || abs(static_cast<int>(podKnobRaw[knob]) - podLastKnobRaw[knob]) >= 8)
        {
            ApplyPodAbsoluteFunction(knob == 0 ? podConfig.knob1Function
                                               : podConfig.knob2Function,
                                     podKnobRaw[knob]);
            podLastKnobRaw[knob] = static_cast<int16_t>(podKnobRaw[knob]);
        }
    }

    podButtonBits = (pod.button1.Pressed() ? 1u : 0u)
                  | (pod.button2.Pressed() ? 2u : 0u)
                  | (pod.encoder.Pressed() ? 4u : 0u);

    if(now - lastLedMs >= 10u)
    {
        lastLedMs = now;
        ApplyPodLed(0, podConfig.led1Function,
                    podConfig.led1R, podConfig.led1G, podConfig.led1B, now);
        ApplyPodLed(1, podConfig.led2Function,
                    podConfig.led2R, podConfig.led2G, podConfig.led2B, now);
        pod.UpdateLeds();
    }
}

/* ═══════════════════════════════════════════════════════════════════
 *  25. SD CARD INIT (SPI3 master) + AUTO-LOAD
 * ═══════════════════════════════════════════════════════════════════ */
static bool InitSD()
{
    /* ── CS pin (D0 = PB12) as GPIO output, start HIGH ── */
    sd_cs.Init(hw.GetPin(0), GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL);
    SD_CS_HIGH();

    /* ── SPI3 master — slow for card init (<=400 kHz) ── */
    SpiHandle::Config sc;
    sc.periph         = SpiHandle::Config::Peripheral::SPI_3;
    sc.mode           = SpiHandle::Config::Mode::MASTER;
    sc.direction      = SpiHandle::Config::Direction::TWO_LINES;
    sc.datasize       = 8;
    sc.clock_polarity = SpiHandle::Config::ClockPolarity::LOW;
    sc.clock_phase    = SpiHandle::Config::ClockPhase::ONE_EDGE;
    sc.nss            = SpiHandle::Config::NSS::SOFT;
    sc.baud_prescaler = SpiHandle::Config::BaudPrescaler::PS_256;  /* ~400 kHz */
    sc.pin_config.sclk = hw.GetPin(2);   /* D2 = PC10 */
    sc.pin_config.miso = hw.GetPin(1);   /* D1 = PC11 */
    sc.pin_config.mosi = hw.GetPin(6);   /* D6 = PC12 */
    sc.pin_config.nss  = Pin();            /* CS manual via GPIO */
    sdDiagStage = SD_DIAG_SPI_INIT;
    sdLastCommand = 0xFF;
    sdLastResponse = 0xFF;
    sdLastDataToken = 0xFF;
    sdSpiErrors = 0;
    sd_spi.Init(sc);

    /* ── Register SPI SD driver with FatFS ── */
    char sdPath[4];
    FATFS_LinkDriver(&SPISD_Driver, sdPath);

    /* Some cards are not ready at the first command immediately after power
     * up. Retry the real mount without registering another FatFS driver. */
    FRESULT fr = FR_NOT_READY;
    for(uint8_t attempt = 0; attempt < 3; attempt++){
        sd_card_type = 0;
        fr = f_mount(&sdFatFs, sdPath, 1);
        sdMountResult = (uint8_t)fr;
        if(fr == FR_OK){
            sdPresent = true;
            /* ── Switch to fast SPI for data transfer ── */
            /* External six-pin modules and Dupont wiring are much more stable
             * around 6 MHz than at 12 MHz, while still loading WAVs quickly. */
            SD_Deselect();
            sc.baud_prescaler = SpiHandle::Config::BaudPrescaler::PS_16;  /* ~6 MHz */
            sd_spi.Init(sc);
            sdDiagStage = SD_DIAG_OK;
            return true;
        }
        SD_CS_HIGH();
        System::Delay(120u * (attempt + 1u));
    }
    sdPresent = false;
    return false;
}

static constexpr uint32_t POD_CONFIG_FILE_MAGIC = 0x36444F50u; /* "POD6" */
static constexpr const char* POD_CONFIG_FILE_PATH = "/pod_controls.cfg";

struct __attribute__((packed)) PodConfigFile {
    uint32_t magic;
    uint16_t checksum;
    uint16_t reserved;
    PodConfigPayload config;
};

static bool SavePodConfigToSD()
{
    if(!sdPresent) return false;
    PodConfigFile stored = {};
    stored.magic = POD_CONFIG_FILE_MAGIC;
    stored.config = podConfig;
    stored.checksum = crc16(reinterpret_cast<const uint8_t*>(&stored.config),
                            sizeof(stored.config));
    FIL file;
    if(f_open(&file, POD_CONFIG_FILE_PATH, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
        return false;
    UINT written = 0;
    const FRESULT result = f_write(&file, &stored, sizeof(stored), &written);
    if(result == FR_OK && written == sizeof(stored)) f_sync(&file);
    f_close(&file);
    return result == FR_OK && written == sizeof(stored);
}

static bool LoadPodConfigFromSD()
{
    if(!sdPresent) return false;
    FIL file;
    if(f_open(&file, POD_CONFIG_FILE_PATH, FA_READ) != FR_OK)
        return false;
    PodConfigFile stored = {};
    UINT read = 0;
    const FRESULT result = f_read(&file, &stored, sizeof(stored), &read);
    f_close(&file);
    if(result != FR_OK || read != sizeof(stored)
       || stored.magic != POD_CONFIG_FILE_MAGIC
       || stored.config.version != POD_CONFIG_VERSION
       || stored.checksum != crc16(reinterpret_cast<const uint8_t*>(&stored.config),
                                   sizeof(stored.config)))
        return false;
    ValidatePodConfig(stored.config);
    podConfig = stored.config;
    podLastKnobRaw[0] = -1;
    podLastKnobRaw[1] = -1;
    podStateRevision++;
    return true;
}

/* Try to load the first .wav from a directory directly into pad slot */
static bool LoadWavToPad(const char* filepath, uint8_t padIdx)
{
    if(padIdx >= MAX_PADS) return false;

    bool ok = false;
    bool opened = false;
    FIL fil;
    UINT br = 0;
    uint8_t riff[12];
    uint16_t audioFormat = 0;
    uint16_t ch = 0;
    uint16_t bps = 0;
    uint32_t sr = 0;
    uint32_t dataOffset = 0;
    uint32_t dataSize = 0;
    uint32_t bytesPerFrame = 0;
    uint32_t totalFrames = 0;
    int16_t* sampleData = nullptr;
    bool fmtFound = false;

    StopPadVoices(padIdx);
    Unbind909PcmPad(padIdx);
    Unbind505PcmPad(padIdx);
    sampleLoaded[padIdx] = false;
    sampleLength[padIdx] = 0;
    sampleTotalSamples[padIdx] = 0;
    sampleRateHz[padIdx] = SAMPLE_RATE;
    /* A trim window sized for whatever was on this pad before makes no
     * sense against a new file of different length — reset to "no trim"
     * (see padTrimStartPct comment) regardless of how this load turns out. */
    padTrimStartPct[padIdx] = 0.0f;
    padTrimEndPct[padIdx] = 0.0f;
    FreeSampleStorage(padIdx);
    padLoading[padIdx] = true;

    if(f_open(&fil, filepath, FA_READ) != FR_OK)
        goto done;
    opened = true;

    /* RIFF/WAVE parser: fmt/data may be separated by LIST/JUNK metadata. */
    if(f_read(&fil, riff, sizeof(riff), &br) != FR_OK || br != sizeof(riff))
        goto done;
    if(memcmp(riff, "RIFF", 4) != 0 || memcmp(riff + 8, "WAVE", 4) != 0)
        goto done;

    while(f_tell(&fil) + 8u <= f_size(&fil)){
        uint8_t ck[8];
        if(f_read(&fil, ck, 8, &br) != FR_OK || br < 8) break;
        uint32_t ckSz = ck[4]|(ck[5]<<8)|(ck[6]<<16)|(ck[7]<<24);
        const uint32_t chunkData = (uint32_t)f_tell(&fil);
        const uint32_t fileSize = (uint32_t)f_size(&fil);
        if(chunkData > fileSize || ckSz > (fileSize - chunkData))
            break;

        if(memcmp(ck, "fmt ", 4) == 0 && ckSz >= 16u){
            uint8_t fmt[40] = {};
            uint32_t fmtRead = ckSz < sizeof(fmt) ? ckSz : sizeof(fmt);
            if(f_read(&fil, fmt, fmtRead, &br) != FR_OK || br != fmtRead)
                break;
            audioFormat = fmt[0] | (fmt[1] << 8);
            ch  = fmt[2] | (fmt[3] << 8);
            sr  = fmt[4] | (fmt[5] << 8) | (fmt[6] << 16) | (fmt[7] << 24);
            bps = fmt[14] | (fmt[15] << 8);
            /* WAVE_FORMAT_EXTENSIBLE with PCM sub-format GUID. */
            if(audioFormat == 0xFFFEu && fmtRead >= 40u
            && fmt[24] == 1u && fmt[25] == 0u)
                audioFormat = 1u;
            fmtFound = true;
        } else if(memcmp(ck, "data", 4) == 0){
            dataOffset = chunkData;
            dataSize = ckSz;
        }

        if(fmtFound && dataOffset != 0u)
            break;
        uint32_t nextChunk = chunkData + ckSz + (ckSz & 1u);
        if(nextChunk < chunkData || nextChunk > fileSize)
            break;
        if(f_lseek(&fil, nextChunk) != FR_OK)
            break;
    }

    if(!fmtFound || dataSize == 0u || dataOffset == 0u || audioFormat != 1u)
        goto done;
    if(ch == 0 || ch > 2 || sr < 1000u || sr > 384000u
    || (bps != 8 && bps != 16 && bps != 24))
        goto done;
    if(f_lseek(&fil, dataOffset) != FR_OK)
        goto done;

    bytesPerFrame = (bps/8) * ch;
    if(bytesPerFrame == 0)
        goto done;
    totalFrames = dataSize / bytesPerFrame;
    if(totalFrames > MAX_SAMPLE_BYTES / 2) totalFrames = MAX_SAMPLE_BYTES / 2;
    if(!AllocSampleStorage(padIdx, totalFrames))
        goto done;

    sampleData = SamplePtr(padIdx);
    if(sampleData == nullptr)
        goto done;

    /* Read and convert to mono 16-bit */
    if(bps == 16 && ch == 1){
        /* Optimal: direct read */
        if(f_read(&fil, sampleData, totalFrames * 2, &br) != FR_OK)
            goto done;
        sampleLength[padIdx] = br / 2;
    } else {
        /* Convert: read in chunks */
        uint8_t buf[512];
        uint32_t frames = 0;
        while(frames < totalFrames){
            uint32_t want = (totalFrames - frames) * bytesPerFrame;
            if(want > sizeof(buf))
                want = sizeof(buf) - (sizeof(buf) % bytesPerFrame);
            if(want == 0)
                break;
            if(f_read(&fil, buf, want, &br) != FR_OK || br == 0) break;
            if((br % bytesPerFrame) != 0)
                break;
            uint32_t got = br / bytesPerFrame;
            for(uint32_t i = 0; i < got && frames < totalFrames; i++){
                const uint8_t* s = buf + i * bytesPerFrame;
                int32_t sample = 0;
                if(bps == 16){
                    sample = (int16_t)(s[0]|(s[1]<<8));
                    if(ch == 2) sample = (sample + (int16_t)(s[2]|(s[3]<<8))) / 2;
                } else if(bps == 24){
                    sample = (int32_t)(((uint32_t)s[0]<<8)|((uint32_t)s[1]<<16)|((uint32_t)s[2]<<24));
                    sample >>= 16;
                    if(ch == 2){
                        int32_t s2 = (int32_t)(((uint32_t)s[3]<<8)|((uint32_t)s[4]<<16)|((uint32_t)s[5]<<24));
                        sample = (sample + (s2>>16)) / 2;
                    }
                } else if(bps == 8){
                    sample = ((int32_t)s[0] - 128) * 256;
                    if(ch == 2) sample = (sample + ((int32_t)s[1]-128)*256) / 2;
                }
                sampleData[frames++] =
                    (int16_t)(sample < -32768 ? -32768 : (sample > 32767 ? 32767 : sample));
            }
        }
        sampleLength[padIdx] = frames;
    }

    sampleTotalSamples[padIdx] = sampleLength[padIdx];
    {
        /* Braced so `loaded` goes out of scope before the `done:` label
         * below — the goto's above it would otherwise jump into (skip)
         * this initialization, which C++ rejects. */
        const bool loaded = sampleLength[padIdx] > 0;
        if(loaded)
            sampleRateHz[padIdx] = sr;
        if(loaded && synth505PcmMode && padIdx < 16)
            synth505.SetPcmSample(padTo505[padIdx], SamplePtr(padIdx),
                                  sampleLength[padIdx], (float)sampleRateHz[padIdx]);
        if(loaded && synth909PcmMode && Is909PcmPad(padIdx))
            synth909.SetPcmSample(padTo909[padIdx], SamplePtr(padIdx),
                                  sampleLength[padIdx], (float)sampleRateHz[padIdx]);

        /* Publish last: every write above (the sample data filled earlier
         * in this function, sampleLength/sampleTotalSamples/sampleRateHz,
         * and the PCM pointers just handed to synth505/909) must be
         * visible before the audio ISR can see sampleLoaded[padIdx]==true
         * and start using this pad — TriggerPad and the PCM playback path
         * both gate on this flag. */
        __DMB();
        sampleLoaded[padIdx] = loaded;
    }

    ok = sampleLoaded[padIdx];
    if(ok && padIdx < DSQ_TRACKS)
        dsqTrackEngine[padIdx] = -1;

done:
    if(!ok){
        if(sdLoadFailures < 255u) sdLoadFailures++;
        StopPadVoices(padIdx);
        sampleLoaded[padIdx] = false;
        sampleLength[padIdx] = 0;
        sampleTotalSamples[padIdx] = 0;
        FreeSampleStorage(padIdx);
    }
    padLoading[padIdx] = false;
    if(opened)
        f_close(&fil);
    return ok;
}

/* ── Helper: case-insensitive substring match ─────────────────── */
static bool containsCI(const char* haystack, const char* needle)
{
    for(const char* h = haystack; *h; h++){
        const char* hp = h;
        const char* np = needle;
        while(*np && *hp && (toupper((uint8_t)*hp) == toupper((uint8_t)*np))){
            hp++; np++;
        }
        if(!*np) return true;
    }
    return false;
}

/* ── Helper: guess pad index from a filename using keyword table ── */
static int GuessPadFromFilename(const char* fname)
{
    /* Try each keyword — longer keywords checked implicitly because
       the table is ordered from most-specific to least-specific */
    for(int k = 0; k < NUM_INSTR_KEYWORDS; k++){
        if(containsCI(fname, INSTR_KEYWORDS[k].keyword))
            return INSTR_KEYWORDS[k].pad;
    }
    return -1;
}

/* ── Helper: check if .wav extension ─────────────────────────── */
static bool isWavFile(const char* fname)
{
    size_t len = strlen(fname);
    if(len < 4) return false;
    const char* ext = fname + len - 4;
    return (ext[0] == '.') && (ext[1]=='w'||ext[1]=='W')
        && (ext[2]=='a'||ext[2]=='A') && (ext[3]=='v'||ext[3]=='V');
}

static int compareCI(const char* a, const char* b)
{
    while(*a && *b){
        int da = toupper((uint8_t)*a);
        int db = toupper((uint8_t)*b);
        if(da != db) return da - db;
        a++; b++;
    }
    return (int)((uint8_t)*a) - (int)((uint8_t)*b);
}

static bool LoadFirstWavFromFolderToPad(const char* folderPath, uint8_t padIdx)
{
    DIR dir;
    FILINFO fno;
    char bestName[64] = {};
    bool found = false;

    if(f_opendir(&dir, folderPath) != FR_OK)
        return false;

    while(f_readdir(&dir, &fno) == FR_OK && fno.fname[0]){
        if(fno.fattrib & AM_DIR) continue;
        if(!isWavFile(fno.fname)) continue;

        if(!found || compareCI(fno.fname, bestName) < 0){
            CopyFixedString(bestName, sizeof(bestName), fno.fname);
            found = true;
        }
    }
    f_closedir(&dir);

    if(!found)
        return false;

    char fpath[192];
    if(!JoinPath(fpath, sizeof(fpath), folderPath, bestName))
        return false;
    return LoadWavToPad(fpath, padIdx);
}

static uint8_t FillMissingCanonicalPadsFromFamilies(uint8_t startPad, uint8_t maxPads,
                                                     const char* kitPath)
{
    uint8_t endPad = startPad + maxPads;
    if(endPad > 16) endPad = 16;
    uint8_t filled = 0;

    for(uint8_t pad = startPad; pad < endPad; pad++){
        if(sampleLoaded[pad]) continue;

        /* Try 1: kitPath/familyName/ (e.g. /RED 808 KARZ/BD/) */
        bool ok = false;
        if(kitPath && kitPath[0]){
            char famPath[96];
            if(JoinPath(famPath, sizeof(famPath), kitPath, PAD_FAMILY_NAMES[pad]))
                ok = LoadFirstWavFromFolderToPad(famPath, pad);
        }
        /* Try 2: SD_DATA_ROOT/familyName/ */
        if(!ok){
            char famPath[96];
            if(JoinPath(famPath, sizeof(famPath), SD_DATA_ROOT, PAD_FAMILY_NAMES[pad]))
                ok = LoadFirstWavFromFolderToPad(famPath, pad);
        }
        if(ok) filled++;
    }

    return filled;
}

/* Auto-load default kit from SD at boot */
static void AutoLoadFromSD()
{
    if(!sdPresent) return;

    sdBootLoaded = 0;
    sdLoadFailures = 0;
    DIR dataRoot;
    const FRESULT dataRootOpen = f_opendir(&dataRoot, SD_DATA_ROOT);
    sdRootResult = (uint8_t)dataRootOpen;
    if(dataRootOpen == FR_OK)
        f_closedir(&dataRoot);

    /* ── PHASE 1: Load LIVE PADS 0-15 from default kit ─────────── */
    /* Try "RED 808 KARZ" first, then any folder in /data          */
    static const char* defaultKitNames[] = {
        "RED 808 KARZ", nullptr
    };

    bool liveLoaded = false;

    for(int k = 0; defaultKitNames[k]; k++){
        char kitPath[96];
        if(!JoinPath(kitPath, sizeof(kitPath), SD_DATA_ROOT, defaultKitNames[k]))
            continue;
        DIR dir;
        if(f_opendir(&dir, kitPath) != FR_OK) continue;

        /* Pass 1: smart-map by instrument keyword */
        bool padUsed[16] = {};
        FILINFO fno;
        uint8_t loaded = 0;
        while(f_readdir(&dir, &fno) == FR_OK && fno.fname[0]){
            if(fno.fattrib & AM_DIR) continue;
            if(!isWavFile(fno.fname)) continue;
            int pad = GuessPadFromFilename(fno.fname);
            if(pad >= 0 && pad < 16 && !padUsed[pad]){
                char fpath[192];
                if(!JoinPath(fpath, sizeof(fpath), kitPath, fno.fname)) continue;
                if(LoadWavToPad(fpath, (uint8_t)pad)){
                    padUsed[pad] = true;
                    loaded++;
                }
            }
        }
        f_closedir(&dir);

        /* Deterministic boot mapping:
           only canonical keyword->pad assignment is loaded for LIVE pads.
           Duplicates/unknown filenames are ignored here to preserve
           stable track identity across boots and kits. */

        /* Diagnostic: report missing canonical pads */
        uint8_t missing = 0;
        for(int i = 0; i < 16; i++){
            if(!padUsed[i]) missing++;
        }
        if(missing > 0){
            hw.PrintLine("SD: Kit '%s' missing %d canonical pads",
                         defaultKitNames[k], missing);
        }

        if(loaded > 0){
            CopyFixedString(currentKitName, sizeof(currentKitName), defaultKitNames[k]);
            hw.PrintLine("SD: Loaded %d LIVE PADS from '%s'",
                           loaded, defaultKitNames[k]);
            /* Build pad mask for event */
            uint32_t bootMask = 0;
            for(int i = 0; i < 16; i++)
                if(sampleLoaded[i]) bootMask |= (1u << i);
            PushEvent(EVT_SD_BOOT_DONE, loaded, bootMask,
                      defaultKitNames[k]);
            liveLoaded = true;
            break;
        }
    }

    /* Fallback if default kit not found: try first directory with WAVs */
    if(!liveLoaded){
        DIR root; FILINFO fno;
        if(f_opendir(&root, SD_DATA_ROOT) == FR_OK){
            while(f_readdir(&root, &fno) == FR_OK && fno.fname[0]){
                if(!(fno.fattrib & AM_DIR)) continue;
                if(strlen(fno.fname) <= 2) continue;  /* skip family folders */
                if(strcasecmp(fno.fname, "xtra") == 0) continue;
                char kitPath[96];
                if(!JoinPath(kitPath, sizeof(kitPath), SD_DATA_ROOT, fno.fname)) continue;
                DIR kdir; FILINFO kfno;
                uint8_t padIdx = 0;
                if(f_opendir(&kdir, kitPath) == FR_OK){
                    while(f_readdir(&kdir, &kfno) == FR_OK && kfno.fname[0]
                          && padIdx < 16){
                        if(kfno.fattrib & AM_DIR) continue;
                        if(!isWavFile(kfno.fname)) continue;
                        char fpath[192];
                        if(!JoinPath(fpath, sizeof(fpath), kitPath, kfno.fname)) continue;
                        if(LoadWavToPad(fpath, padIdx)) padIdx++;
                    }
                    f_closedir(&kdir);
                }
                if(padIdx > 0){
                    CopyFixedString(currentKitName, sizeof(currentKitName), fno.fname);
                    hw.PrintLine("SD: Fallback loaded %d LIVE PADS from '%s'",
                                   padIdx, fno.fname);
                    uint32_t fbMask = 0;
                    for(int i = 0; i < padIdx; i++)
                        if(sampleLoaded[i]) fbMask |= (1u << i);
                    PushEvent(EVT_SD_BOOT_DONE, padIdx, fbMask, fno.fname);
                    liveLoaded = true;
                    break;
                }
            }
            f_closedir(&root);
        }
    }

    {
        uint8_t recovered = FillMissingCanonicalPadsFromFamilies(0, 16);
        if(recovered > 0){
            hw.PrintLine("SD: Recovered %d missing LIVE pads from /data families",
                         recovered);
        }
    }

    /* ── PHASE 2: Load XTRA PADS 16-23 from /data/xtra ─────────── */
    {
        char xtraPath[48];
        snprintf(xtraPath, sizeof(xtraPath), "%s/xtra", SD_DATA_ROOT);
        DIR dir; FILINFO fno;
        uint8_t xtraIdx = 16;  /* pads 16-23 */
        if(f_opendir(&dir, xtraPath) == FR_OK){
            while(f_readdir(&dir, &fno) == FR_OK && fno.fname[0]
                  && xtraIdx < MAX_PADS){
                if(fno.fattrib & AM_DIR) continue;
                if(!isWavFile(fno.fname)) continue;
                char fpath[160];
                if(!JoinPath(fpath, sizeof(fpath), xtraPath, fno.fname)) continue;
                if(LoadWavToPad(fpath, xtraIdx)) xtraIdx++;
            }
            f_closedir(&dir);
            if(xtraIdx > 16){
                hw.PrintLine("SD: Loaded %d XTRA PADS from /data/xtra",
                               xtraIdx - 16);
                uint32_t xtraMask = 0;
                for(int i = 16; i < xtraIdx; i++)
                    if(sampleLoaded[i]) xtraMask |= (1u << i);
                PushEvent(EVT_SD_XTRA_LOADED, xtraIdx - 16,
                          xtraMask, "xtra");
            }
        }
    }

    for(uint8_t pad = 0; pad < DSQ_TRACKS; pad++)
        if(sampleLoaded[pad]) sdBootLoaded++;
}

/* ═══════════════════════════════════════════════════════════════════
 *  26. INIT HELPERS
 * ═══════════════════════════════════════════════════════════════════ */
static void InitArrays()
{
    memset(sampleUploadReceivedBytes, 0, sizeof(sampleUploadReceivedBytes));
    memset(sampleUploadValid, 0, sizeof(sampleUploadValid));
    for(int i = 0; i < MAX_PADS; i++){
        sampleLoaded[i] = false;
        sampleLength[i] = 0;
        sampleTotalSamples[i] = 0;
        sampleRateHz[i] = SAMPLE_RATE;
        trackGain[i]  = 1.0f;
        trackPeak[i]  = 0.0f;
        padLoop[i]    = false;
        padReverse[i] = false;
        padPitch[i]   = 1.0f;
        trkPitchCents[i] = 0;
        padFilterType[i] = 0;
        padFilterCut[i]  = 10000.f;
        padFilterQ[i]    = 0.707f;
        padFilterCutSm[i] = padFilterCut[i];
        padFilterQSm[i]   = padFilterQ[i];
        padFilterGain[i]  = 0.f;
        padDistDrive[i]  = 0;
        padDistMode[i]   = 0;
        padBitDepth[i]   = 16;
        padStutterOn[i]  = false;
        trackReverbSend[i] = 0;
        trackDelaySend[i]  = 0;
        trackChorusSend[i] = 0;
        trackPanF[i]  = 0;
        trackMute[i]  = false;
        trackSolo[i]  = false;
        trkFilterType[i] = 0;
        trkFilterCut[i]  = 10000.f;
        trkFilterQ[i]    = 0.707f;
        trkFilterCutSm[i] = trkFilterCut[i];
        trkFilterQSm[i]   = trkFilterQ[i];
        trkFilterGain[i]  = 0.f;
        trkDistDrive[i]  = 0;
        trkDistMode[i]   = 0;
        trkBitDepth[i]   = 16;
        trkEchoActive[i] = false;
        trkEchoWp[i] = 0;
        trkFlgActive[i]  = false;
        trkFlanger[i].Init((float)SAMPLE_RATE);
        ConfigureTrackFlanger((uint8_t)i);
        trkCompActive[i] = false;
        trkCompThresh[i] = 0.6f;
        trkCompRatio[i]  = 4.0f;
        trkCompExp[i]    = 1.f - 1.f/4.0f;
        trkCompEnv[i]    = 0;
        trkEqLowDb[i]  = 0;
        trkEqMidDb[i]  = 0;
        trkEqHighDb[i] = 0;
        trkLfoActive[i] = false;
        trkLfoWave[i]   = LFO_WAVE_SINE;
        trkLfoTarget[i] = LFO_TGT_GAIN;
        trkLfoRate[i]   = 1.0f;
        trkLfoDepth[i]  = 0.0f;
        trkLfoPhase[i]  = 0.0f;
        trkLfoSH[i]     = 0.0f;
        trkEnvAdActive[i] = false;
        trkEnvAttackMs[i] = 1.0f;
        trkEnvDecayMs[i]  = 250.0f;
    }
    for(int i = 0; i < MAX_VOICES; i++) voices[i].active = false;
}

static void InitFX()
{
    float sr = (float)SAMPLE_RATE;

    ResetMasterProcessingState();

    for(int i = 0; i < MAX_PADS; i++){
        trkFxRouted[i] = false;
        padLoading[i]  = false;
    }

    masterDelay.Init();
    masterDelay.SetDelay(sr * 0.25f);

    masterReverb.Init(sr);
    masterReverb.SetFeedback(0.6f);
    masterReverb.SetLpFreq(8000.0f);

    masterChorusL.Init(sr);
    masterChorusR.Init(sr);
    masterChorusL.SetLfoFreq(0.3f);
    masterChorusR.SetLfoFreq(0.3039f);
    masterChorusL.SetLfoDepth(0.4f);
    masterChorusR.SetLfoDepth(0.4f);
    masterChorusL.SetDelay(0.72f);
    masterChorusR.SetDelay(0.78f);

    masterTremolo.Init(sr);
    masterTremolo.SetFreq(4.0f);
    masterTremolo.SetDepth(0.5f);
    masterTremolo.SetWaveform(Oscillator::WAVE_SIN);

    masterComp.Init(sr);
    masterComp.SetThreshold(-20.0f);
    masterComp.SetRatio(4.0f);
    masterComp.SetAttack(0.01f);
    masterComp.SetRelease(0.1f);
    masterComp.SetMakeup(1.0f);
    masterComp.AutoMakeup(true);

    masterPhaserL.Init(sr);
    masterPhaserR.Init(sr);
    masterPhaserL.SetFreq(0.5f);
    masterPhaserR.SetFreq(0.5065f);
    masterPhaserL.SetLfoDepth(phaserDepth);
    masterPhaserR.SetLfoDepth(phaserDepth);
    masterPhaserL.SetFeedback(0.5f);
    masterPhaserR.SetFeedback(0.5f);

    masterFlangerL.Init(sr);
    masterFlangerR.Init(sr);
    ConfigureMasterFlanger();

    for(int i = 0; i < MAX_PADS; i++){
        memset(trkEchoBuf[i], 0, sizeof(trkEchoBuf[i]));
        trkFlanger[i].Init(sr);
        ConfigureTrackFlanger((uint8_t)i);
    }

    /* ── Mega Upgrade: init new master FX modules ── */
    masterAutowahL.Init(sr);
    masterAutowahR.Init(sr);
    masterAutowahL.SetLevel(1.0f);
    masterAutowahR.SetLevel(1.0f);
    masterAutowahL.SetWah(autowahLevel);
    masterAutowahR.SetWah(autowahLevel);

    masterLadderL.Init(sr);
    masterLadderR.Init(sr);
    masterLadderL.SetFreq(10000.f);
    masterLadderR.SetFreq(10000.f);
    masterLadderL.SetRes(0.3f);
    masterLadderR.SetRes(0.3f);

    masterSvfL.Init(sr);
    masterSvfR.Init(sr);
    masterSvfL.SetFreq(10000.f);
    masterSvfR.SetFreq(10000.f);
    masterSvfL.SetRes(0.3f);
    masterSvfR.SetRes(0.3f);
    masterSvfL.SetDrive(0.0f);
    masterSvfR.SetDrive(0.0f);

    erDelayL.Init();
    erDelayR.Init();
    combDelayL.Init();
    combDelayR.Init();

    masterDelayR.Init();
    masterDelayR.SetDelay(sr * 0.25f);

    memset(beatRepBufL, 0, sizeof(beatRepBufL));
    memset(beatRepBufR, 0, sizeof(beatRepBufR));
    beatRepActive = false;
    beatRepDiv = 0;
    beatRepPos = 0;
    beatRepCapturing = false;
    beatRepPlaying = false;

    memset(chokeGroup, 0, sizeof(chokeGroup));

    songLength = 0;
    songPlaying = false;
    songIdx = 0;
    songRepeatCnt = 0;

    autowahActive = false;
    autowahRouted = true;
    erActive = false;
    erRouted = true;
    stereoWidth = 1.0f;
    tapeStopActive = false;
    tapeStopSpeed = 1.0f;
    delayPingPong = false;
    chorusStereoMode = true;

    /* ── Synth Engines Init ── */
    synth808.Init(sr);
    synth909.Init(sr);
    synth505.Init(sr);
    acid303.Init(sr);
    wtOsc.Init(sr);
    synthSH101.Init(sr);  /* I1 */
    synthFM2Op.Init(sr);  /* I2 */

    /* Physical Modeling engine */
    physModal.Init(sr);
    physModal.SetFreq(220.f);
    physModal.SetAccent(0.5f);
    physModal.SetStructure(0.5f);
    physModal.SetBrightness(0.5f);
    physModal.SetDamping(0.5f);
    physModalActive = false;

    physString.Init(sr);
    physString.SetFreq(220.f);
    physString.SetAccent(0.5f);
    physString.SetStructure(0.5f);
    physString.SetBrightness(0.5f);
    physString.SetDamping(0.5f);
    physStringActive = false;

    /* Noise/Texture engine */
    noisePart.Init(sr);
    noisePart.SetFreq(220.f);
    noisePart.SetResonance(0.5f);
    noisePart.SetRandomFreq(0.3f);
    noisePart.SetDensity(0.5f);
    noisePart.SetGain(0.6f);
    noisePart.SetSpread(0.3f);
    noisePartActive = false;

    ApplyDefaultSynthPresets();
    dcBlockL.Init(sr);    /* M3 */
    dcBlockR.Init(sr);
    for(int i=0; i<16; i++) trackWtNote[i]    = (uint8_t)(60 + (i % 12));
    for(int i=0; i<16; i++) trackSH101Note[i] = (uint8_t)(60 + (i % 12));
    for(int i=0; i<16; i++) trackFM2OpNote[i] = (uint8_t)(60 + (i % 12));
    startupAnnounceOsc.Init(sr);
    startupAnnounceOsc.SetCarrierFreq(100.0f);
    startupAnnounceOsc.SetFormantFreq(900.0f);
    startupAnnounceOsc.SetPhaseShift(0.4f);
}

/* ═══════════════════════════════════════════════════════════════════
 *  26b. BOOT DIAGNOSTIC HELPERS
 * ═══════════════════════════════════════════════════════════════════ */

/* Blink LED N veces con 150ms on/off — útil para marcar etapas de boot */
static void BootBlinkN(int n)
{
    for(int i = 0; i < n; i++){
        hw.SetLed(true);  System::Delay(150);
        hw.SetLed(false); System::Delay(150);
    }
    System::Delay(400);
}

/* HardFault handler personalizado — patrón SOS (3 corto · 3 largo · 3 corto).
 * Reemplaza el "2 parpadeos y fijo" del handler por defecto de libdaisy.
 * Con este handler el usuario verá SOS en lugar del patrón ambiguo.
 * Usa busy-loop delay (no SysTick) para funcionar incluso con stack corrupto. */

static void FaultDelay(uint32_t ms)
{
    /* STM32H750 @ 480MHz ≈ 480000 ciclos/ms (sin cache effects en fault) */
    volatile uint32_t cycles = ms * 240000u;  /* ~0.5M ciclos/ms conservador */
    while(cycles--) __asm volatile("nop");
}

static void FaultSosLoop(void)
{
    __disable_irq();
    while(1)
    {
        for(int i = 0; i < 3; i++){ hw.SetLed(true); FaultDelay(120); hw.SetLed(false); FaultDelay(120); }
        FaultDelay(250);
        for(int i = 0; i < 3; i++){ hw.SetLed(true); FaultDelay(450); hw.SetLed(false); FaultDelay(150); }
        FaultDelay(250);
        for(int i = 0; i < 3; i++){ hw.SetLed(true); FaultDelay(120); hw.SetLed(false); FaultDelay(120); }
        FaultDelay(1500);
    }
}

extern "C" void HardFault_Handler(void)   { FaultSosLoop(); }
extern "C" void MemManage_Handler(void)    { FaultSosLoop(); }
extern "C" void BusFault_Handler(void)     { FaultSosLoop(); }
extern "C" void UsageFault_Handler(void)   { FaultSosLoop(); }

/* ═══════════════════════════════════════════════════════════════════
 *  27. MAIN
 * ═══════════════════════════════════════════════════════════════════ */
int main()
{
    /* ── Enable FPU Flush-to-Zero + Default-NaN to prevent denormals ── */
    /* Thread-mode FPSCR (for main loop) */
    __asm volatile("VMRS r0, FPSCR\n"
                   "ORR  r0, r0, #(1<<24)|(1<<25)\n"  /* FZ=1, DN=1 */
                   "VMSR FPSCR, r0" ::: "r0");
    /* FPDSCR — default FPSCR for ALL exception/ISR contexts (AudioCallback!) */
    *(volatile uint32_t*)0xE000EF3Cu |= (1u << 24) | (1u << 25);

    /* ── Hardware init ── */
    pod.Init();
    DspProfInit();

    /* ── Boot progress markers — solo activos con RED808_BOOT_PROGRESS_DIAG=1 ──
     * Compila con: make RED808_BOOT_PROGRESS_DIAG=1
     * Si el LED muestra SOS → HardFault antes del primer parpadeo.
     * Cuenta de parpadeos al crash: 1=hw.Init OK, 2=InitArrays OK,
     * 3=InitFX OK (SDRAM OK), 4=QSPI/SD OK, 5=Audio+USB OK → main loop. */
#define BOOT_BLINK(n) do { if(kBootProgressDiag) BootBlinkN(n); } while(0)
    BOOT_BLINK(1);  /* hw.Init + DspProfInit completados */

    if(kBootDiagMinimal)
    {
        bool led = false;
        while(1)
        {
            led = !led;
            hw.SetLed(led);
            System::Delay(125);
        }
    }

    if(kAudioDiagMinimal)
    {
        pod.SetAudioBlockSize(AUDIO_BLOCK);
        pod.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
        pod.StartAudio(AudioCallback);
        while(1)
        {
            const uint32_t now = hw.system.GetNow();
            hw.SetLed(((now / 250u) & 1u) != 0u);
            System::Delay(1);
        }
    }

    pod.SetAudioBlockSize(AUDIO_BLOCK);
    pod.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    audioLoadMeter.Init(hw.AudioSampleRate(), hw.AudioBlockSize());

    auto Log = [&](const char* fmt, auto... args)
    {
        if(kEnableStartLog)
            hw.PrintLine(fmt, args...);
    };

    /* USB serial debug (false = no bloquear esperando terminal) */
    if(kEnableStartLog)
        hw.StartLog(false);
    else
        hw.usb_handle.Init(UsbHandle::UsbPeriph::FS_INTERNAL);
    hw.usb_handle.SetReceiveCallback(
        DaisyUsbRxCallback, UsbHandle::UsbPeriph::FS_INTERNAL);
    Log("══════════════════════════════════════════");
    Log("  DrumMachine V2 — DaisyPod3 USB device");
    Log("  %d pads · %d voices · %d Hz · %d block",
        MAX_PADS, MAX_VOICES, SAMPLE_RATE, AUDIO_BLOCK);
    Log("  Synth: TR808 · TR909 · TR505 · TB303");
    Log("══════════════════════════════════════════");

    /* ── Init state ── */
    InitArrays();
    BOOT_BLINK(2);  /* InitArrays completado */
    if(kEnableInitFx)
        InitFX();
    BOOT_BLINK(3);  /* InitFX completado (SDRAM OK) */

    /* ── Cargar WAVs desde QSPI Flash (blob en 0x900C0000) → SDRAM ── */
    if(kStartupStressReport)
    {
        Log("StressReport: sample preload omitido (QSPI/SD no requerido)");
    }
    else
    {
        /* El blob se flashea con pack_wavs.py + dfu-util a 0x900C0000.
         * IMPORTANTE: app vive en 0x90040000 (BOOT_SRAM copia 512KB a SRAM),
         * por eso el blob empieza en 0x900C0000 (app+512KB) y NO en 0x90080000
         * (que solo dejaba 256KB y el firmware ya pasa de ese tamaño).
         * Formato: magic "WAV\0"(4B) | ver(2B) | count(2B)
         *          entries[count]: padIdx(1B) | rsv(1B) | offset(4B) | size(4B)  [10 bytes]
         *          WAV files raw (con header) back-to-back, align 4
         * La QSPI es memory-mapped: leemos directamente como punteros. */
        static const uint8_t* QSPI_SAMPLES = (const uint8_t*)0x900C0000;
        static const uint32_t QSPI_END_ADDR = 0x90800000;
        const uint8_t* blob = QSPI_SAMPLES;
        bool blobOk = (blob[0]=='W' && blob[1]=='A' && blob[2]=='V' && blob[3]==0);
        if(blobOk){
            uint16_t blobCount = blob[6] | (blob[7] << 8);
            const uint32_t maxBlobBytes = QSPI_END_ADDR - (uint32_t)QSPI_SAMPLES;
            const uint32_t entrySize = 10;
            Log("QSPI WAV blob detectado: %d samples", blobCount);
            for(uint16_t i = 0; i < blobCount && i < MAX_PADS; i++){
                const uint8_t* e = blob + 8 + i * entrySize;
                uint8_t  padIdx  = e[0];
                uint32_t offset  = e[2]|(e[3]<<8)|(e[4]<<16)|(e[5]<<24);
                uint32_t fsize   = e[6]|(e[7]<<8)|(e[8]<<16)|(e[9]<<24);
                if(padIdx >= MAX_PADS || fsize < 44) continue;
                if(offset >= maxBlobBytes || fsize > maxBlobBytes || (offset + fsize) > maxBlobBytes)
                    continue;
                /* Parsear WAV header directamente desde QSPI */
                const uint8_t* wav = blob + offset;
                if(memcmp(wav, "RIFF", 4) != 0 || memcmp(wav+8, "WAVE", 4) != 0){
                    Log("  Pad %d: WAV header invalido", padIdx);
                    continue;
                }
                uint16_t audioFormat = wav[20] | (wav[21]<<8);
                uint16_t ch  = wav[22] | (wav[23]<<8);
                uint32_t sr  = wav[24] | (wav[25]<<8) | (wav[26]<<16) | (wav[27]<<24);
                uint16_t bps = wav[34] | (wav[35]<<8);
                if(audioFormat != 1u || ch == 0 || ch > 2
                || sr < 1000u || sr > 384000u
                || (bps != 8 && bps != 16 && bps != 24))
                    continue;
                /* Buscar chunk "data" */
                uint32_t pos = 12;
                uint32_t dataSize = 0;
                const uint8_t* dataPtr = nullptr;
                while((pos + 8) <= fsize){
                    const uint8_t* ck = wav + pos;
                    uint32_t ckSz = ck[4]|(ck[5]<<8)|(ck[6]<<16)|(ck[7]<<24);
                    if((pos + 8 + ckSz) > fsize)
                        break;
                    if(memcmp(ck, "data", 4) == 0){
                        dataSize = ckSz;
                        dataPtr  = ck + 8;
                        break;
                    }
                    pos += 8 + ckSz;
                    if(ckSz & 1) pos++;  /* WAV chunks padded to even */
                }
                if(!dataPtr || dataSize == 0) continue;
                uint32_t bytesPerFrame = (bps / 8) * ch;
                if(bytesPerFrame == 0) continue;
                uint32_t totalFrames = dataSize / bytesPerFrame;
                if(totalFrames > MAX_SAMPLE_BYTES / 2)
                    totalFrames = MAX_SAMPLE_BYTES / 2;
                FreeSampleStorage(padIdx);
                if(!AllocSampleStorage(padIdx, totalFrames)){
                    sampleLength[padIdx] = 0;
                    sampleTotalSamples[padIdx] = 0;
                    sampleLoaded[padIdx] = false;
                    Log("  Pad %2d: sin SDRAM para %lu frames", padIdx, totalFrames);
                    continue;
                }
                int16_t* sampleData = SamplePtr(padIdx);
                if(sampleData == nullptr){
                    sampleLength[padIdx] = 0;
                    sampleTotalSamples[padIdx] = 0;
                    sampleLoaded[padIdx] = false;
                    continue;
                }
                /* Convertir a mono 16-bit en SDRAM */
                if(bps == 16 && ch == 1){
                    memcpy(sampleData, dataPtr, totalFrames * 2);
                    sampleLength[padIdx] = totalFrames;
                } else {
                    uint32_t frames = 0;
                    for(uint32_t f = 0; f < totalFrames; f++){
                        const uint8_t* s = dataPtr + f * bytesPerFrame;
                        int32_t sample = 0;
                        if(bps == 16){
                            sample = (int16_t)(s[0]|(s[1]<<8));
                            if(ch == 2) sample = (sample + (int16_t)(s[2]|(s[3]<<8))) / 2;
                        } else if(bps == 24){
                            sample = (int32_t)(((uint32_t)s[0]<<8)|((uint32_t)s[1]<<16)|((uint32_t)s[2]<<24));
                            sample >>= 16;
                            if(ch == 2){
                                int32_t s2 = (int32_t)(((uint32_t)s[3]<<8)|((uint32_t)s[4]<<16)|((uint32_t)s[5]<<24));
                                sample = (sample + (s2>>16)) / 2;
                            }
                        } else if(bps == 8){
                            sample = ((int32_t)s[0] - 128) * 256;
                            if(ch == 2) sample = (sample + ((int32_t)s[1]-128)*256) / 2;
                        }
                        sampleData[frames++] =
                            (int16_t)(sample < -32768 ? -32768 : (sample > 32767 ? 32767 : sample));
                    }
                    sampleLength[padIdx] = frames;
                }
                sampleTotalSamples[padIdx] = sampleLength[padIdx];
                const bool loaded = sampleLength[padIdx] > 0;
                sampleRateHz[padIdx] = loaded ? sr : SAMPLE_RATE;
                /* Audio is already running at this point in boot (StartAudio
                 * happened earlier in main()) — publish last, after a
                 * barrier, same reasoning as LoadWavToPad/CMD_SAMPLE_END. */
                __DMB();
                sampleLoaded[padIdx] = loaded;
                if(loaded)
                    Log("  Pad %2d: %lu frames OK", padIdx, sampleLength[padIdx]);
            }
        } else {
            Log("No hay WAV blob en QSPI (0x900C0000)");
        }
    }
    bool sdOk = false;
    sdPresent = false;

    /* ── Conteo inicial de samples cargados ── */
    uint8_t loadedCount = 0;
    for(int i = 0; i < MAX_PADS; i++) if(sampleLoaded[i]) loadedCount++;
    Log("Samples precargados: %d / %d", loadedCount, MAX_PADS);

    /* ── SD boot load ──
     * Si no hay blob QSPI usable, intentamos recuperar el kit por defecto
     * desde /data en la microSD para evitar un arranque completamente mudo. */
    if(kEnableSdBootLoad && !kStartupStressReport)
    {
        Log("Daisy SD autoritativa: intentando init + autoload...");
        sdOk = InitSD();
        if(sdOk)
        {
            LoadPodConfigFromSD();
            Log("SD OK, cargando kit por defecto...");
            AutoLoadFromSD();
        }
        else
        {
            Log("SD init fallo; sin muestras locales al arranque");
        }

        loadedCount = 0;
        for(int i = 0; i < MAX_PADS; i++) if(sampleLoaded[i]) loadedCount++;
    }
    else if(loadedCount == 0)
    {
        Log("Sin samples en QSPI y SD boot-load desactivado: arranque sin muestras locales (OK, llegan por USB desde P4)");
    }

    Log("Samples cargados: %d / %d", loadedCount, MAX_PADS);
    BOOT_BLINK(4);  /* QSPI/SD load completado */

    Log("USB CDC device listo para P4 host");

    /* ── Inicializar secuenciador Daisy ── */
    DsqInit();
    DsqEnsureAudibleSources();
    Log("DsqInit: %d patrones x %d tracks x %d steps en SDRAM",
        DSQ_PATTERNS, DSQ_TRACKS, DSQ_MAX_STEPS);
    pod.StartMidi();
    Log("MIDI merge listo: 2 dispositivos x 3 bancos x 3 capas");
    if(kStartupShowcaseDemo){
        BuildStartupShowcaseProgram();
        Log("Showcase programado: 8 escenas / 32 compases / sample-first");
    }

    /* ── Start Audio ── */
    if(kEnableAudioStart)
    {
        Log("Iniciando audio @ %d Hz, %d samples/block", SAMPLE_RATE, AUDIO_BLOCK);
        pod.StartAudio(AudioCallback);

    }
    else
    {
        Log("Audio: DESHABILITADO (diagnostico StartAudio)");
    }
    BOOT_BLINK(5);  /* Audio started + SPI init OK */
#undef BOOT_BLINK

    pod.StartAdc();

    /* LED apagado por defecto; se enciende por actividad de transporte */
    hw.SetLed(false);
    Log(">>> RED808 DRUM MACHINE READY <<<");

    Log("STARTUP TONE TEST: tono 1kHz durante 3s (kStartupToneTest=true)");
    Log("STARTUP SELF-TEST: DESACTIVADO (DaisyPod3 listo para P4)");
    BeginStartupStressReport(hw.system.GetNow());

    /* ── Main loop ── */
    while(1){

        /* Control simultaneo: USB-C CDC con el P4 + MIDI TRS Type A. */
        ProcessDaisyUsb();
        ProcessMpdMidi();
        ProcessPodControls(hw.system.GetNow());

        /* ── LED diagnóstico ── */
        uint32_t now = hw.system.GetNow();
        RunStartupShowcaseDemo(now);
        RunStartup808SelfTest(now);
        RunStartupStressReport(now);
        RunPerformanceStressMode(now);
        RunLiveDspProfileReport(now);
        if(kStartupToneTest)
            hw.SetLed(((now / 125u) & 1u) != 0u);
        else
            hw.SetLed(usbLastPacketMs != 0u && (now - usbLastPacketMs) < 3000u);
    }
}
