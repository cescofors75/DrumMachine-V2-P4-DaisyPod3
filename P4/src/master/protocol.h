/*
 * protocol.h
 * DrumMachine V2 USB command protocol — shared command definitions
 * ESP32-P4 host ↔ DaisyPod3 device
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include "../../../shared/red808_protocol_codes.h"

// ═══════════════════════════════════════════════════════
// BINARY USB PACKET STRUCTURE
// ═══════════════════════════════════════════════════════

#define SPI_MAGIC_CMD      0xA5   // Command from Master
#define SPI_MAGIC_RESP     0x5A   // Response from Slave
#define SPI_MAGIC_SAMPLE   0xDA   // Sample data transfer
#define SPI_MAGIC_BULK     0xBB   // Bulk multi-command

#define SPI_MAX_PAYLOAD    760    // Daisy TX supports 768-byte packets (SD file lists)

// Packet header (8 bytes, packed)
typedef struct __attribute__((packed)) {
    uint8_t  magic;       // SPI_MAGIC_CMD / SPI_MAGIC_RESP / SPI_MAGIC_SAMPLE
    uint8_t  cmd;         // Command code
    uint16_t length;      // Payload length in bytes
    uint16_t sequence;    // Sequence number for verification
    uint16_t checksum;    // CRC16 of payload
} SPIPacketHeader;

// ═══════════════════════════════════════════════════════
// COMMANDS: TRIGGER (0x01 - 0x0F)
// ═══════════════════════════════════════════════════════
#define CMD_TRIGGER_SEQ       0x01  // Trigger from sequencer
#define CMD_TRIGGER_LIVE      0x02  // Trigger from live pad
#define CMD_TRIGGER_STOP      0x03  // Stop specific sample
#define CMD_TRIGGER_STOP_ALL  0x04  // Stop all samples
#define CMD_TRIGGER_SIDECHAIN 0x05  // Trigger sidechain envelope

// ═══════════════════════════════════════════════════════
// COMMANDS: VOLUME (0x10 - 0x1F)
// ═══════════════════════════════════════════════════════
#define CMD_MASTER_VOLUME     0x10  // Master volume (0-100)
#define CMD_SEQ_VOLUME        0x11  // Sequencer volume (0-150)
#define CMD_LIVE_VOLUME       0x12  // Live pad volume (0-180)
#define CMD_TRACK_VOLUME      0x13  // Per-track volume (0-150)
#define CMD_LIVE_PITCH        0x14  // Live pitch shift
#define CMD_TEMPO             0x15  // Global transport BPM (ESP32 master clock -> Daisy sync)

// ───────────────────────────────────────────────────────
// PROTOCOL BOUNDARY (ESP32 master -> Daisy slave)
//   TRIGGER         -> CMD_TRIGGER_SEQ / CMD_TRIGGER_LIVE
//   PARAM_RT        -> existing param commands (filter/fx/track/pad/synth)
//   SYNC_PATTERN    -> optional metadata command (timing stays on ESP32)
//   TEMPO           -> CMD_TEMPO
//   LOAD_SAMPLE     -> CMD_SD_LOAD_SAMPLE / CMD_SD_LOAD_KIT
//   UPLOAD_*        -> CMD_SAMPLE_BEGIN / CMD_SAMPLE_DATA / CMD_SAMPLE_END
//   MUTE            -> CMD_TRACK_MUTE
//   REQ_DIR         -> CMD_SD_LIST_FOLDERS / CMD_SD_LIST_FILES
// Daisy -> ESP32 should return only ACK/ERR + directory/load status/events.
// Audio calculations, LFO values, waveform buffers, step timing never cross wire.
// ───────────────────────────────────────────────────────

// ═══════════════════════════════════════════════════════
// COMMANDS: GLOBAL FILTER (0x20 - 0x2F)
// ═══════════════════════════════════════════════════════
#define CMD_FILTER_SET        0x20  // Set global filter (full params)
#define CMD_FILTER_CUTOFF     0x21  // Cutoff frequency
#define CMD_FILTER_RESONANCE  0x22  // Resonance (Q)
#define CMD_FILTER_BITDEPTH   0x23  // Bit crush
#define CMD_FILTER_DISTORTION 0x24  // Global distortion amount
#define CMD_FILTER_DIST_MODE  0x25  // Distortion mode (soft/hard/tube/fuzz)
#define CMD_FILTER_SR_REDUCE  0x26  // Sample rate reduction
#define CMD_MASTER_FX_ROUTE   0x27  // Master FX routing: [fxId(1), connected(1)]

// ═══════════════════════════════════════════════════════
// COMMANDS: MASTER EFFECTS (0x30 - 0x4F)
// ═══════════════════════════════════════════════════════
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

// ═══════════════════════════════════════════════════════
// COMMANDS: REVERB (0x43 - 0x46)
// ═══════════════════════════════════════════════════════
#define CMD_REVERB_ACTIVE     0x43  // Master reverb on/off
#define CMD_REVERB_FEEDBACK   0x44  // Room size / decay (0.0-1.0)
#define CMD_REVERB_LPFREQ     0x45  // Damp: LP filter Hz (200-12000)
#define CMD_REVERB_MIX        0x46  // Dry/Wet mix (0.0-1.0)

// ═══════════════════════════════════════════════════════
// COMMANDS: CHORUS (0x47 - 0x4A)
// ═══════════════════════════════════════════════════════
#define CMD_CHORUS_ACTIVE     0x47  // Master chorus on/off
#define CMD_CHORUS_RATE       0x48  // LFO rate Hz (0.1-10.0)
#define CMD_CHORUS_DEPTH      0x49  // LFO depth (0.0-1.0)
#define CMD_CHORUS_MIX        0x4A  // Dry/Wet mix (0.0-1.0)

// ═══════════════════════════════════════════════════════
// COMMANDS: TREMOLO (0x4B - 0x4D)
// ═══════════════════════════════════════════════════════
#define CMD_TREMOLO_ACTIVE    0x4B  // Master tremolo on/off
#define CMD_TREMOLO_RATE      0x4C  // Rate Hz (0.1-20.0)
#define CMD_TREMOLO_DEPTH     0x4D  // Depth (0.0-1.0)

// ═══════════════════════════════════════════════════════
// COMMANDS: WAVEFOLDER + LIMITER (0x4E - 0x4F)
// ═══════════════════════════════════════════════════════
#define CMD_WAVEFOLDER_GAIN   0x4E  // WaveFolder input gain (1.0-10.0)
#define CMD_LIMITER_ACTIVE    0x4F  // Brick-wall limiter on/off (threshold = 0dBFS)

// ═══════════════════════════════════════════════════════
// COMMANDS: PER-TRACK FX (0x50 - 0x6F)
// ═══════════════════════════════════════════════════════
#define CMD_TRACK_FILTER      0x50  // Per-track filter
#define CMD_TRACK_CLEAR_FILTER 0x51 // Clear track filter
#define CMD_TRACK_DISTORTION  0x52  // Per-track distortion
#define CMD_TRACK_BITCRUSH    0x53  // Per-track bitcrush
#define CMD_TRACK_ECHO        0x54  // Per-track echo
#define CMD_TRACK_FLANGER_FX  0x55  // Per-track flanger
#define CMD_TRACK_COMPRESSOR  0x56  // Per-track compressor
#define CMD_TRACK_CLEAR_LIVE  0x57  // Clear per-track live FX
#define CMD_TRACK_CLEAR_FX    0x58  // Clear all per-track FX

// Per-track FX send levels + mixer
#define CMD_TRACK_REVERB_SEND 0x59  // Per-track reverb send (0-100)
#define CMD_TRACK_DELAY_SEND  0x5A  // Per-track delay send (0-100)
#define CMD_TRACK_CHORUS_SEND 0x5B  // Per-track chorus send (0-100)
#define CMD_TRACK_PAN         0x5C  // Per-track pan (-100 L .. 0 C .. +100 R)
#define CMD_TRACK_MUTE        0x5D  // Per-track mute (0/1)
#define CMD_TRACK_SOLO        0x5E  // Per-track solo (0/1)
#define CMD_TRACK_PHASER      0x5F  // Per-track phaser
#define CMD_TRACK_TREMOLO     0x60  // Per-track tremolo
#define CMD_TRACK_PITCH       0x61  // Per-track pitch shift (cents)
#define CMD_TRACK_GATE        0x62  // Per-track noise gate
#define CMD_TRACK_EQ_LOW      0x63  // Per-track 3-band EQ low  (-12..+12 dB)
#define CMD_TRACK_EQ_MID      0x64  // Per-track 3-band EQ mid  (-12..+12 dB)
#define CMD_TRACK_EQ_HIGH     0x65  // Per-track 3-band EQ high (-12..+12 dB)
#define CMD_TRACK_FX_ROUTE    0x66  // Per-track FX routing: [track(1), connected(1)]
#define CMD_TRACK_LFO_CONFIG  0x67  // Per-track LFO config (7 bytes)

// ═══════════════════════════════════════════════════════
// COMMANDS: PER-PAD FX (0x70 - 0x7F)
// ═══════════════════════════════════════════════════════
#define CMD_PAD_FILTER        0x70  // Per-pad filter
#define CMD_PAD_CLEAR_FILTER  0x71  // Clear pad filter
#define CMD_PAD_DISTORTION    0x72  // Per-pad distortion
#define CMD_PAD_BITCRUSH      0x73  // Per-pad bitcrush
#define CMD_PAD_LOOP          0x74  // Pad continuous loop
#define CMD_PAD_REVERSE       0x75  // Reverse sample
#define CMD_PAD_PITCH         0x76  // Per-pad pitch shift
#define CMD_PAD_STUTTER       0x77  // Stutter effect
#define CMD_PAD_CLEAR_FX      0x7A  // Clear all pad FX

// ═══════════════════════════════════════════════════════
// COMMANDS: PER-PAD LFO (0x80 - 0x8F)
//   NOTA ARQUITECTURAL: Estos command codes son INTERNOS del ESP32.
//   NUNCA se envían por SPI a la Daisy Seed.
//   Los LFOs se computan en LFOEngine.cpp; su resultado se aplica
//   modulando parámetros existentes (ej: pitch → CMD_TRACK_PITCH 0x61,
//   pan → CMD_TRACK_PAN 0x60, vol → CMD_TRACK_VOLUME 0x10).
//   Estos defines SOLO existen para la UI y el serializador de preset.
// ═══════════════════════════════════════════════════════
#define CMD_PAD_LFO_ACTIVE    0x80  // [pad, on/off]
#define CMD_PAD_LFO_WAVE      0x81  // [pad, waveform] 0=sin 1=tri 2=sq 3=saw 4=s&h
#define CMD_PAD_LFO_RATE      0x82  // [pad, division] 0=1/4 1=1/8 2=1/16 3=1/32 4=free
#define CMD_PAD_LFO_DEPTH     0x83  // [pad, depth 0-100]
#define CMD_PAD_LFO_TARGET    0x84  // [pad, target] 0=pitch 1=decay 2=filter 3=pan 4=vol
#define CMD_PAD_LFO_FREE_HZ   0x85 // [pad, hz_x10] for free-running mode (0.1-20.0 Hz)
#define CMD_PAD_LFO_PHASE     0x86  // [pad, phase 0-255] (0=0°, 128=180°, 255≈360°)
#define CMD_PAD_LFO_RETRIG    0x87  // [pad, on/off] retrigger phase on note-on

// ═══════════════════════════════════════════════════════
// COMMANDS: SIDECHAIN (0x90 - 0x9F)
// ═══════════════════════════════════════════════════════
#define CMD_SIDECHAIN_SET     0x90  // Configure sidechain
#define CMD_SIDECHAIN_CLEAR   0x91  // Deactivate sidechain

// ═══════════════════════════════════════════════════════
// COMMANDS: SAMPLE TRANSFER & NEW MASTER FX (0xA0 - 0xAF)
// ═══════════════════════════════════════════════════════
#define CMD_SAMPLE_BEGIN      0xA0  // Begin sample transfer
#define CMD_SAMPLE_DATA       0xA1  // Sample data chunk
#define CMD_SAMPLE_END        0xA2  // End sample transfer
#define CMD_SAMPLE_UNLOAD     0xA3  // Unload one sample
#define CMD_SAMPLE_UNLOAD_ALL 0xA4  // Unload all samples

// New master FX commands (0xA5 - 0xAF)
#define CMD_AUTOWAH_ACTIVE    0xA5  // Auto-Wah on/off
#define CMD_AUTOWAH_LEVEL     0xA6  // Auto-Wah sensitivity 0-127
#define CMD_AUTOWAH_MIX       0xA7  // Auto-Wah wet/dry 0-100
#define CMD_STEREO_WIDTH      0xA8  // Stereo width 0-200
#define CMD_TAPE_STOP         0xA9  // Tape stop/start effect
#define CMD_BEAT_REPEAT       0xAA  // Beat repeat division
#define CMD_DELAY_STEREO      0xAB  // Delay stereo mode (0=mono, 1=ping-pong)
#define CMD_CHORUS_STEREO     0xAC  // Chorus stereo mode (0=mono, 1=stereo)
#define CMD_EARLY_REF_ACTIVE  0xAD  // Early reflections on/off
#define CMD_EARLY_REF_MIX     0xAE  // Early reflections mix 0-100
#define CMD_CHOKE_GROUP       0xAF  // Choke groups: [pad(1), group(1)]

// ═══════════════════════════════════════════════════════
// COMMANDS: DAISY SD FILE SYSTEM (0xB0 - 0xBF)
//   Permite al ESP32 explorar y cargar samples desde
//   la SD card conectada al SPI2 de la Daisy Seed.
//   La Daisy carga directo a SDRAM — mucho más rápido
//   que transferir PCM por SPI.
// ═══════════════════════════════════════════════════════
#define CMD_SD_LIST_FOLDERS   0xB0  // List root-level folders (kit dirs)
#define CMD_SD_LIST_FILES     0xB1  // List WAV files in a folder
#define CMD_SD_FILE_INFO      0xB2  // Get sample info (length, SR, bits)
#define CMD_SD_LOAD_SAMPLE    0xB3  // Load one WAV from SD → SDRAM pad slot
#define CMD_SD_LOAD_KIT       0xB4  // Load entire kit folder → all pads
#define CMD_SD_KIT_LIST       0xB5  // Get list of available kit names
#define CMD_SD_STATUS         0xB6  // SD card status (present, size, free)
#define CMD_SD_UNLOAD_KIT     0xB7  // Unload current kit from SDRAM
#define CMD_SD_GET_LOADED     0xB8  // Get currently loaded kit name + pad map
#define CMD_SD_ABORT          0xB9  // Abort ongoing SD load operation

// ═══════════════════════════════════════════════════════
// COMMANDS: SYNTH ENGINES (0xC0 - 0xCF)
//   Daisy Seed onboard synthesis: TR-808, TR-909, TR-505,
//   TB-303, WTOSC, SH-101 y FM2Op.
//   engine IDs: 0=TR-808, 1=TR-909, 2=TR-505, 3=TB-303,
//               4=WTOSC, 5=SH-101, 6=FM2Op
// ═══════════════════════════════════════════════════════
#define CMD_SYNTH_TRIGGER    0xC0  // Trigger one synth instrument
#define CMD_SYNTH_PARAM      0xC1  // Set synth instrument parameter
#define CMD_SYNTH_NOTE_ON    0xC2  // TB-303 note on (MIDI note + accent/slide)
#define CMD_SYNTH_NOTE_OFF   0xC3  // TB-303 note off (no payload)
#define CMD_SYNTH_303_PARAM  0xC4  // TB-303 global parameter
#define CMD_SYNTH_ACTIVE     0xC5  // Engine active mask (1 or 2 bytes)
#define CMD_SYNTH_PRESET     0xC6  // Apply factory preset [engine(1), preset(1)]
#define CMD_SYNTH_NOTE_ON_EX 0xC7  // Generic synth note on (any engine: 303/SH101/FM2Op/WTOSC)

// Synth engine IDs
#define SYNTH_ENGINE_808   0
#define SYNTH_ENGINE_909   1
#define SYNTH_ENGINE_505   2
#define SYNTH_ENGINE_303   3
#define SYNTH_ENGINE_WTOSC 4  // Wavetable Oscillator
#define SYNTH_ENGINE_SH101 5  // I1: Roland SH-101 monosynth
#define SYNTH_ENGINE_FM2OP 6  // I2: 2-operator FM Yamaha-style
#define SYNTH_ENGINE_PHYS  7  // Physical modeling: ModalVoice/StringVoice
#define SYNTH_ENGINE_NOISE 8  // Noise/texture: Particle percussion
#define SYNTH_ENGINE_COUNT 9

// TR-808 instrument IDs (engine=0)
#define SYNTH_808_KICK     0
#define SYNTH_808_SNARE    1
#define SYNTH_808_CLAP     2
#define SYNTH_808_HH_CL    3
#define SYNTH_808_HH_OP    4
#define SYNTH_808_LO_TOM   5
#define SYNTH_808_MID_TOM  6
#define SYNTH_808_HI_TOM   7
#define SYNTH_808_LO_CONGA 8
#define SYNTH_808_MID_CONGA 9
#define SYNTH_808_HI_CONGA 10
#define SYNTH_808_CLAVES   11
#define SYNTH_808_MARACAS  12
#define SYNTH_808_RIMSHOT  13
#define SYNTH_808_COWBELL  14
#define SYNTH_808_CYMBAL   15

// Synth parameter IDs (CMD_SYNTH_PARAM — applies to 808/909/505)
#define SYNTH_PARAM_DECAY   0
#define SYNTH_PARAM_PITCH   1
#define SYNTH_PARAM_TONE    2   // also "saturation" depending on engine
#define SYNTH_PARAM_VOLUME  3
#define SYNTH_PARAM_SNAPPY  4

// TB-303 parameter IDs (CMD_SYNTH_303_PARAM)
#define SYNTH_303_CUTOFF    0   // filter cutoff frequency (Hz)
#define SYNTH_303_RESONANCE 1   // resonance / Q
#define SYNTH_303_ENV_MOD   2   // envelope modulation depth
#define SYNTH_303_DECAY     3   // decay time (seconds)
#define SYNTH_303_ACCENT    4   // accent level (0.0-1.0)
#define SYNTH_303_SLIDE_T   5   // slide/portamento time (seconds)
#define SYNTH_303_WAVEFORM  6   // waveform: 0=sawtooth, 1=square
#define SYNTH_303_VOLUME    7   // output volume (0.0-1.0)

// ─── Synth Payload Structs ───────────────────────────

// CMD_SYNTH_TRIGGER (0xC0) — 3 bytes
typedef struct __attribute__((packed)) {
    uint8_t  engine;      // SYNTH_ENGINE_*
    uint8_t  instrument;  // 0-15 (varies by engine)
    uint8_t  velocity;    // 0-127
} SynthTriggerPayload;

// CMD_SYNTH_PARAM (0xC1) — 7 bytes
typedef struct __attribute__((packed)) {
    uint8_t  engine;      // SYNTH_ENGINE_*  (0/1/2; not used for 303)
    uint8_t  instrument;  // instrument ID
    uint8_t  paramId;     // SYNTH_PARAM_*
    uint8_t  reserved;
    float    value;       // IEEE 754 little-endian
} SynthParamPayload;

// CMD_SYNTH_NOTE_ON (0xC2) — 3 bytes (TB-303 only)
typedef struct __attribute__((packed)) {
    uint8_t  midiNote;    // 0-127
    uint8_t  accent;      // 0=normal, 1=accent
    uint8_t  slide;       // 0=normal, 1=slide/portamento
} SynthNoteOnPayload;

// CMD_SYNTH_NOTE_OFF (0xC3) — legacy: no payload; v2.6+: [engine, track] or [engine, track, note]

// CMD_SYNTH_NOTE_ON_EX (0xC7) — 5 bytes (generic melodic synth note-on)
typedef struct __attribute__((packed)) {
    uint8_t  engine;      // SYNTH_ENGINE_303 / _WTOSC / _SH101 / _FM2OP
    uint8_t  midiNote;    // 0-127
    uint8_t  velocity;    // 0-127
    uint8_t  accent;      // 0=normal, 1=accent (303 only)
    uint8_t  slide;       // 0=normal, 1=slide  (303 only)
} SynthNoteOnExPayload;

// CMD_SYNTH_303_PARAM (0xC4) — 5 bytes
typedef struct __attribute__((packed)) {
    uint8_t  paramId;     // SYNTH_303_*
    uint8_t  reserved[3];
    float    value;       // IEEE 754 little-endian
} Synth303ParamPayload;

// CMD_SYNTH_ACTIVE (0xC5) — 1 or 2 bytes
typedef struct __attribute__((packed)) {
    uint8_t  maskLo;      // bits 0-7 (engines 0-7)
    uint8_t  maskHi;      // bit 0 = engine 8 (NOISE)
} SynthActivePayload16;

// Legacy 1-byte variant (kept for backward compat)
typedef struct __attribute__((packed)) {
    uint8_t  engineMask;  // bit0=808..bit6=FM2Op
} SynthActivePayload;

// CMD_SYNTH_PRESET (0xC6) — 2 bytes
typedef struct __attribute__((packed)) {
    uint8_t  engine;
    uint8_t  preset;
} SynthPresetPayload;

// ═══════════════════════════════════════════════════════
// COMMANDS: DAISY SEQUENCER (0xD0 - 0xDF)
// ═══════════════════════════════════════════════════════
#define CMD_DSQ_UPLOAD_TRACK    0xD0  // Upload full track steps for one pattern
#define CMD_DSQ_SET_STEP        0xD1  // Set/update a single step
#define CMD_DSQ_CONTROL         0xD2  // 0=stop, 1=play, 2=reset
#define CMD_DSQ_SELECT_PATTERN  0xD3  // Select active pattern (0-15)
#define CMD_DSQ_SET_LENGTH      0xD4  // Set pattern length (16/32/64)
#define CMD_DSQ_SET_MUTE        0xD5  // Mute/unmute a track
#define CMD_DSQ_GET_POS         0xD6  // Query current step/pattern (→ response)
#define CMD_DSQ_SET_SWING       0xD7  // Set swing amount (0-100)
#define CMD_DSQ_SET_PARAM_LOCK  0xD8  // Set per-step parameter locks
#define CMD_DSQ_SET_TRACK_ENGINE 0xD9  // [track(1), engine(1)]  -1=sampler 0=808 1=909 2=505 3=303
#define CMD_DSQ_SET_TRACK_SWING  0xDA  // E4: [track(1), swing 0-100(1)] per-track swing override
#define CMD_DSQ_SET_HUMANIZE     0xDB  // E2: [timingMs(1), velocityAmt(1)] humanization
#define CMD_CLEAN_TRACK_ACTIVE   0xDC  // [track(1), active(1)] include/exclude clean track from global transport
#define CMD_CLEAN_TRACK_MUTE     0xDD  // [track(1), muted(1)] mute clean track audio
#define CMD_DSQ_SET_STEP_NOTES   0xDE  // [pattern,track,step,flags,note0..note3]
#define CMD_DSQ_QUEUE_PATTERN    0xDF  // [pattern,bars] 0=normal, 1..16=temp scene

// ─── DSQ Step packet (4 bytes) – used in upload track ───
#define DSQ_PATTERNS   20
#define DSQ_TRACKS     16
#define DSQ_MAX_STEPS  64

typedef struct __attribute__((packed)) {
    uint8_t  active;       // 1 = step is active
    uint8_t  velocity;     // 1-127 (0 = use track default)
    uint8_t  noteLenDiv;   // low nibble=length divisor; high nibble=ratchet-1
    uint8_t  probability;  // 0-100 (100 = always fire)
} DsqStepPkt;

// CMD_DSQ_UPLOAD_TRACK (0xD0)  payload: 4 + stepCount×4 bytes (max 260)
typedef struct __attribute__((packed)) {
    uint8_t   pattern;     // 0-15
    uint8_t   track;       // 0-15
    uint8_t   stepCount;   // 16/32/64
    uint8_t   reserved;
    DsqStepPkt steps[DSQ_MAX_STEPS];  // only first stepCount entries are sent
} DsqUploadTrackPayload;

// CMD_DSQ_SET_STEP (0xD1)  8 bytes
typedef struct __attribute__((packed)) {
    uint8_t  pattern;
    uint8_t  track;
    uint8_t  step;
    uint8_t  active;
    uint8_t  velocity;
    uint8_t  noteLenDiv;
    uint8_t  probability;
    uint8_t  reserved;
} DsqSetStepPayload;

// CMD_DSQ_SET_PARAM_LOCK (0xD8)  12 bytes
typedef struct __attribute__((packed)) {
    uint8_t  pattern;
    uint8_t  track;
    uint8_t  step;
    uint8_t  cutoffEn;      // 1 = apply per-step cutoff
    uint8_t  cutoffHi;      // cutoff Hz high byte
    uint8_t  cutoffLo;      // cutoff Hz low byte
    uint8_t  reverbEn;      // 1 = apply per-step reverb send
    uint8_t  reverbSend;    // reverb send 0-100
    uint8_t  volEn;         // 1 = apply per-step volume
    uint8_t  volume;        // volume 0-127
    uint8_t  reserved[2];
} DsqSetParamLockPayload;

// CMD_DSQ_SET_STEP_NOTES (0xDE)  8 bytes
typedef struct __attribute__((packed)) {
    uint8_t pattern;
    uint8_t track;
    uint8_t step;
    uint8_t flags;         // bit0=accent, bit1=slide
    uint8_t notes[4];      // MIDI notes, 0 = unused/rest
} DsqSetStepNotesPayload;

// CMD_DSQ_GET_POS response (4 bytes)
typedef struct __attribute__((packed)) {
    uint8_t  currentStep;    // 0-63
    uint8_t  currentPattern; // 0-15
    uint8_t  playing;        // 1 = running
    uint8_t  reserved;
} DsqPosResponse;

typedef struct __attribute__((packed)) {
    uint8_t  track;
    uint8_t  value;
} CleanTrackControlPayload;

// ═══════════════════════════════════════════════════════
// COMMANDS: STATUS / QUERY (0xE0 - 0xEF)
// ═══════════════════════════════════════════════════════
#define CMD_GET_STATUS        0xE0  // Get general status (87 bytes response)
#define CMD_GET_PEAKS         0xE1  // Get VU meters (16 tracks + master)
#define CMD_GET_CPU_LOAD      0xE2  // Get CPU load
#define CMD_GET_VOICES        0xE3  // Get active voices
#define CMD_GET_EVENTS        0xE4  // Get pending notification events from slave
#define CMD_DIAG_PERF_STRESS  0xE5  // Daisy performance stress mode / metrics reset
#define CMD_POD_GET_STATE     0xE6  // Get physical Pod controls/config (Daisy authoritative)
#define CMD_POD_SET_CONFIG    0xE7  // Set Pod assignments; Daisy returns canonical state
#define CMD_PING              0xEE  // Ping/Pong
#define CMD_RESET             0xEF  // Full DSP reset

// ═══════════════════════════════════════════════════════
// COMMANDS: BULK (0xF0 - 0xFF)
// ═══════════════════════════════════════════════════════
#define CMD_BULK_TRIGGERS     0xF0  // Multiple triggers in one packet
#define CMD_BULK_FX           0xF1  // Multiple FX changes
#define CMD_SONG_UPLOAD       0xF2  // Song chain upload: [count(1), {pattern(1), repeats(1)}×count]
#define CMD_SONG_CONTROL      0xF3  // Song control: [action(1)] 0=stop, 1=play, 2=reset
#define CMD_SONG_GET_POS      0xF4  // Song position query → 4 bytes response

// ═══════════════════════════════════════════════════════
// PAYLOAD STRUCTURES
// ═══════════════════════════════════════════════════════

// DaisyPod physical-control functions. Absolute controls on Daisy are the
// source of truth; P4 mirrors their values but never pretends to move them.
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

typedef struct __attribute__((packed)) {
    uint8_t version;
    uint8_t button1Function;
    uint8_t button2Function;
    uint8_t knob1Function;
    uint8_t knob2Function;
    uint8_t encoderFunction;
    uint8_t encoderButtonFunction;
    uint8_t led1Function;
    uint8_t led1R;
    uint8_t led1G;
    uint8_t led1B;
    uint8_t led2Function;
    uint8_t led2R;
    uint8_t led2G;
    uint8_t led2B;
    uint8_t reserved;
} PodConfigPayload;

typedef struct __attribute__((packed)) {
    PodConfigPayload config;
    uint16_t knob1;          // physical position 0..1000
    uint16_t knob2;          // physical position 0..1000
    int16_t encoderPosition;
    uint8_t buttons;         // bit0=B1, bit1=B2, bit2=encoder push
    uint8_t buttonPressEvents; // latched rising edges, consumed by P4
    uint8_t selectedPad;
    uint8_t led1R;
    uint8_t led1G;
    uint8_t led1B;
    uint8_t led2R;
    uint8_t led2G;
    uint8_t led2B;
    uint8_t masterVolume;    // canonical Daisy values
    uint8_t seqVolume;
    uint8_t liveVolume;
    uint8_t delayMixValue;   // canonical Daisy 0..127
    uint8_t reverbMixValue;  // canonical Daisy 0..127
    uint8_t fxActiveBits;    // bit0=delay, bit1=reverb
    uint16_t bpmX10;
    uint8_t playing;
    uint8_t sdPresent;
    uint16_t sampleMask;
    uint32_t revision;
} PodStatePayload;

// --- Triggers ---
typedef struct __attribute__((packed)) {
    uint8_t  padIndex;       // 0-15 (sequencer track)
    uint8_t  velocity;       // 1-127
    uint8_t  trackVolume;    // 0-150
    uint8_t  reserved;
    uint32_t maxSamples;     // 0 = full sample, >0 = cut after N samples
} TriggerSeqPayload;

typedef struct __attribute__((packed)) {
    uint8_t  padIndex;       // 0-23 (16 seq + 8 XTRA)
    uint8_t  velocity;       // 1-127
} TriggerLivePayload;

typedef struct __attribute__((packed)) {
    uint8_t  count;          // Number of triggers (1-16)
    uint8_t  reserved;
    TriggerSeqPayload triggers[16];
} BulkTriggersPayload;

// --- Volume ---
typedef struct __attribute__((packed)) {
    uint8_t  volume;         // 0-150
} VolumePayload;

typedef struct __attribute__((packed)) {
    uint8_t  track;          // 0-15
    uint8_t  volume;         // 0-150
} TrackVolumePayload;

typedef struct __attribute__((packed)) {
    float    pitch;          // 0.25 - 3.0
} PitchPayload;

// --- Global Filter ---
typedef struct __attribute__((packed)) {
    uint8_t  filterType;     // enum FilterType (0-14)
    uint8_t  distMode;       // enum DistortionMode (0-3)
    uint8_t  bitDepth;       // 1-16
    uint8_t  reserved;
    float    cutoff;         // Hz (20.0 - 20000.0)
    float    resonance;      // Q (0.1 - 30.0)
    float    distortion;     // 0.0 - 100.0
    uint32_t sampleRateReduce; // Hz (for SR reduction)
} GlobalFilterPayload;

// --- Single float param (for individual FX params) ---
typedef struct __attribute__((packed)) {
    float    value;
} FloatPayload;

// --- Single bool param ---
typedef struct __attribute__((packed)) {
    uint8_t  active;         // 0/1
} BoolPayload;

// --- Single uint32 param ---
typedef struct __attribute__((packed)) {
    uint32_t value;
} Uint32Payload;

// --- Per-Track Filter ---
typedef struct __attribute__((packed)) {
    uint8_t  track;          // 0-15
    uint8_t  filterType;     // enum FilterType
    uint8_t  reserved[2];
    float    cutoff;         // Hz
    float    resonance;      // Q
    float    gain;           // dB (for peaking/shelf)
} TrackFilterPayload;

// --- Per-Track Echo ---
typedef struct __attribute__((packed)) {
    uint8_t  track;          // 0-15
    uint8_t  active;         // 0/1
    uint8_t  reserved[2];
    float    time;           // ms (10-200)
    float    feedback;       // 0.0-90.0 (%)
    float    mix;            // 0.0-100.0 (%)
} TrackEchoPayload;

// --- Per-Track Flanger ---
typedef struct __attribute__((packed)) {
    uint8_t  track;          // 0-15
    uint8_t  active;         // 0/1
    uint8_t  reserved[2];
    float    rate;           // 0.0-100.0 (%)
    float    depth;          // 0.0-100.0 (%)
    float    feedback;       // 0.0-100.0 (%)
} TrackFlangerPayload;

// --- Per-Track Compressor ---
typedef struct __attribute__((packed)) {
    uint8_t  track;          // 0-15
    uint8_t  active;         // 0/1
    uint8_t  reserved[2];
    float    threshold;      // dB
    float    ratio;          // 1.0-20.0
} TrackCompressorPayload;

// --- Per-Pad Filter ---
typedef struct __attribute__((packed)) {
    uint8_t  pad;            // 0-23
    uint8_t  filterType;     // enum FilterType
    uint8_t  reserved[2];
    float    cutoff;         // Hz
    float    resonance;      // Q
    float    gain;           // dB
} PadFilterPayload;

// --- Per-Pad Distortion ---
typedef struct __attribute__((packed)) {
    uint8_t  pad;            // 0-23
    uint8_t  distMode;       // enum DistortionMode
    uint8_t  reserved[2];
    float    amount;         // 0.0-100.0
} PadDistortionPayload;

// --- Per-Pad BitCrush ---
typedef struct __attribute__((packed)) {
    uint8_t  pad;            // 0-23
    uint8_t  bits;           // 1-16
} PadBitCrushPayload;

// --- Per-Pad Loop ---
typedef struct __attribute__((packed)) {
    uint8_t  pad;            // 0-23
    uint8_t  enabled;        // 0/1
} PadLoopPayload;

// --- Per-Pad Reverse ---
typedef struct __attribute__((packed)) {
    uint8_t  pad;            // 0-23
    uint8_t  reverse;        // 0/1
} PadReversePayload;

// --- Per-Pad Pitch ---
typedef struct __attribute__((packed)) {
    uint8_t  pad;            // 0-23
    uint8_t  reserved;
    uint16_t reserved2;
    float    pitch;          // 0.25-3.0
} PadPitchPayload;

// --- Per-Pad Stutter ---
typedef struct __attribute__((packed)) {
    uint8_t  pad;            // 0-23
    uint8_t  active;         // 0/1
    uint16_t intervalMs;     // ms
} PadStutterPayload;

// --- Per-Pad LFO ---
typedef struct __attribute__((packed)) {
    uint8_t  pad;            // 0-23
    uint8_t  active;         // 0/1
} PadLfoActivePayload;

typedef struct __attribute__((packed)) {
    uint8_t  pad;            // 0-23
    uint8_t  waveform;       // 0=sin 1=tri 2=sq 3=saw 4=s&h
} PadLfoWavePayload;

// LFO rate divisions (sync to BPM)
#define LFO_DIV_1_4   0     // quarter note
#define LFO_DIV_1_8   1     // eighth note
#define LFO_DIV_1_16  2     // sixteenth note
#define LFO_DIV_1_32  3     // thirty-second note
#define LFO_DIV_FREE  4     // free-running Hz

typedef struct __attribute__((packed)) {
    uint8_t  pad;            // 0-23
    uint8_t  division;       // LFO_DIV_*
} PadLfoRatePayload;

typedef struct __attribute__((packed)) {
    uint8_t  pad;            // 0-23
    uint8_t  depth;          // 0-100 (percentage)
} PadLfoDepthPayload;

// LFO modulation targets
#define LFO_TARGET_PITCH   0
#define LFO_TARGET_DECAY   1
#define LFO_TARGET_FILTER  2
#define LFO_TARGET_PAN     3
#define LFO_TARGET_VOLUME  4

typedef struct __attribute__((packed)) {
    uint8_t  pad;            // 0-23
    uint8_t  target;         // LFO_TARGET_*
} PadLfoTargetPayload;

typedef struct __attribute__((packed)) {
    uint8_t  pad;            // 0-23
    uint8_t  reserved;
    uint16_t hzX10;          // Hz * 10 (1 = 0.1 Hz, 200 = 20.0 Hz)
} PadLfoFreeHzPayload;

// --- Sidechain ---
typedef struct __attribute__((packed)) {
    uint8_t  active;         // 0/1
    uint8_t  sourceTrack;    // 0-15
    uint16_t destMask;       // Bitmask of destination tracks
    float    amount;         // 0.0-1.0
    float    attackMs;       // 0.1-80.0 ms
    float    releaseMs;      // 10.0-1200.0 ms
    float    knee;           // 0.0-1.0
} SidechainPayload;

// --- Sidechain Trigger ---
typedef struct __attribute__((packed)) {
    uint8_t  sourceTrack;    // 0-15  (which track triggers the ducking)
    uint8_t  reserved;
} SidechainTriggerPayload;

// --- Reverb ---
typedef struct __attribute__((packed)) {
    uint8_t  active;         // 0/1
    uint8_t  reserved[3];
    float    feedback;       // 0.0-1.0 (room size / decay time)
    float    lpFreq;         // Hz (200-12000, damp color)
    float    mix;            // 0.0-1.0 (dry/wet)
} ReverbPayload;

// --- Chorus ---
typedef struct __attribute__((packed)) {
    uint8_t  active;         // 0/1
    uint8_t  reserved[3];
    float    rate;           // Hz (0.1-10.0)
    float    depth;          // 0.0-1.0
    float    mix;            // 0.0-1.0
} ChorusPayload;

// --- Tremolo ---
typedef struct __attribute__((packed)) {
    uint8_t  active;         // 0/1
    uint8_t  reserved[3];
    float    rate;           // Hz (0.1-20.0)
    float    depth;          // 0.0-1.0
} TremoloPayload;

// --- Per-Track FX Send ---
typedef struct __attribute__((packed)) {
    uint8_t  track;          // 0-15
    uint8_t  sendLevel;      // 0-100 (percentage)
} TrackSendPayload;

// --- Per-Track Pan ---
typedef struct __attribute__((packed)) {
    uint8_t  track;          // 0-15
    int8_t   pan;            // -100 (full L) .. 0 (center) .. +100 (full R)
} TrackPanPayload;

// --- Per-Track Mute/Solo ---
typedef struct __attribute__((packed)) {
    uint8_t  track;          // 0-15
    uint8_t  enabled;        // 0/1
} TrackMuteSoloPayload;

// --- Per-Track 3-band EQ ---
typedef struct __attribute__((packed)) {
    uint8_t  track;          // 0-15
    int8_t   gainLow;        // -12..+12 dB
    int8_t   gainMid;        // -12..+12 dB
    int8_t   gainHigh;       // -12..+12 dB
} TrackEqPayload;

// --- Per-Track Gate ---
typedef struct __attribute__((packed)) {
    uint8_t  track;          // 0-15
    uint8_t  active;         // 0/1
    uint8_t  reserved[2];
    float    threshold;      // dB (-60..-6)
    float    attackMs;       // 0.1-50
    float    releaseMs;      // 10-500
} TrackGatePayload;

// --- SD: List Folders Request (no payload, response below) ---
// --- SD: List Files Request ---
typedef struct __attribute__((packed)) {
    char     folderName[32]; // null-terminated folder name
} SdListFilesPayload;

// --- SD: File Info Request ---
typedef struct __attribute__((packed)) {
    char     folderName[32]; // kit folder
    char     fileName[32];   // WAV filename
} SdFileInfoPayload;

// --- SD: Load Sample (65 bytes — folder[32] + filename[32] + padIdx) ---
typedef struct __attribute__((packed)) {
    char     folderName[32]; // kit folder on SD (e.g. "BD", "xtra", "RED 808 KARZ")
    char     fileName[32];   // WAV file name
    uint8_t  padIndex;       // 0-23 destination pad slot
} SdLoadSamplePayload;

// --- SD: Load Kit (all WAVs in a folder) ---
typedef struct __attribute__((packed)) {
    char     kitName[32];    // folder name = kit name
    uint8_t  startPad;       // first pad slot to fill (usually 0)
    uint8_t  maxPads;        // max pads to load (usually 16 or 24)
} SdLoadKitPayload;

// --- SD: Folder List Response (from Daisy → ESP32) ---
typedef struct __attribute__((packed)) {
    uint8_t  count;          // number of folders (max 16)
    char     names[16][32];  // up to 16 folder names, null-terminated
} SdFolderListResponse;

// --- SD: File List Response ---
typedef struct __attribute__((packed)) {
    uint8_t  count;          // number of files (max 20)
    char     files[20][32];  // exact Daisy USB wire format
} SdFileListResponse;

// --- SD: File Info Response (16 bytes) ---
typedef struct __attribute__((packed)) {
    uint32_t sizeBytes;      // file size in bytes
    uint16_t sampleRate;     // Hz (44100, 48000...)
    uint16_t bitsPerSample;  // 8, 16, 24
    uint8_t  channels;       // 1=mono, 2=stereo
    uint8_t  reserved[3];
    uint32_t durationMs;     // duration in milliseconds
} SdFileInfoResponse;

// --- SD: Status Response ---
typedef struct __attribute__((packed)) {
    uint8_t  present;        // 0=no SD, 1=SD present
    uint8_t  reserved;
    uint16_t samplesLoaded;  // bitmask of loaded pad slots 0..15
    char     currentKit[32]; // currently loaded kit name
} SdStatusResponse;

// --- SD: Kit List Response ---
typedef struct __attribute__((packed)) {
    uint8_t  count;          // number of kits found
    char     kits[16][32];   // up to 16 kit names
} SdKitListResponse;

// --- CPU Load Response (CMD_GET_CPU_LOAD 0xE2) ---
typedef struct __attribute__((packed)) {
    float    cpuLoad;        // 0.0-100.0 percent
    uint32_t uptime;         // milliseconds since boot
    float    cpuAvg;         // smoothed CPU percent
    float    cpuPeak;        // peak CPU percent since reset
    uint8_t  activeVoices;
    uint8_t  perfStressMode; // 0/1
    uint16_t spiErrCnt;
    uint16_t spiRingDrops;
    float    masterPeak;
} CpuLoadResponse;

// --- Active Voices Response (CMD_GET_VOICES 0xE3) ---
typedef struct __attribute__((packed)) {
    uint8_t  activeVoices;   // 0-MAX_VOICES
    uint8_t  reserved[3];
} VoicesResponse;

// --- Sample Transfer ---
typedef struct __attribute__((packed)) {
    uint8_t  padIndex;       // 0-23
    uint8_t  bitsPerSample;  // 16
    uint16_t sampleRate;     // 48000
    uint32_t totalBytes;     // Total PCM data size
    uint32_t totalSamples;   // Total int16_t samples
} SampleBeginPayload;

typedef struct __attribute__((packed)) {
    uint8_t  padIndex;       // 0-23
    uint8_t  reserved;
    uint16_t chunkSize;      // Bytes in this chunk (max 512)
    uint32_t offset;         // Byte offset from start
    // int16_t data[] follows (up to 256 samples = 512 bytes)
} SampleDataHeader;

typedef struct __attribute__((packed)) {
    uint8_t  padIndex;       // 0-23
    uint8_t  status;         // 0=OK, 1=error
    uint16_t reserved;
    uint32_t checksum;       // CRC32 of all data
} SampleEndPayload;

typedef struct __attribute__((packed)) {
    uint8_t  padIndex;       // 0-23
} SampleUnloadPayload;

// --- Status Responses ---
typedef struct __attribute__((packed)) {
    float trackPeaks[16];    // 0.0-1.0 per track
    float masterPeak;        // 0.0-1.0 master
} PeaksResponse;

// --- Legacy StatusResponse (kept for reference) ---
// Was 20 bytes. Slave now sends 54 bytes (StatusResponseV2).

// --- StatusResponse V4 (60 bytes — first 58 bytes remain V3-compatible) ---
typedef struct __attribute__((packed)) {
    uint8_t  activeVoices;       // byte 0: active polyphonic voices
    uint8_t  cpuLoadPercent;     // byte 1: CPU % (0-100)
    uint16_t padsLoadedMask;     // bytes 2-3: bitmask pads 0-15 loaded
    uint32_t uptime;             // bytes 4-7: uptime in ms
    uint8_t  sdPresent;          // byte 8: SD card present (0/1)
    uint8_t  xtraPadsMask;       // byte 9: bitmask pads 16-23 (XTRA)
    uint8_t  evtCount;           // byte 10: pending notification events
    uint16_t spiErrCnt;          // bytes 11-12: Daisy-side CRC/parse error count
    uint8_t  spiRingDropsSat;    // byte 13: saturated Daisy SPI ring drops
    char     currentKitName[32]; // bytes 14-45: null-terminated kit name
    uint8_t  totalPadsLoaded;    // byte 46: total pads loaded count
    uint32_t totalBytesUsed;     // bytes 47-50: total bytes in SDRAM
    uint8_t  maxPads;            // byte 51: MAX_PADS value (=24)
    uint8_t  cpuPeakPercent;     // byte 52: peak CPU % since reset
    uint8_t  perfStressMode;     // byte 53: performance stress mode active
    uint8_t  cpuAvgPercent;      // byte 54: smoothed CPU %
    uint8_t  masterClipFlag;     // byte 55: master peak >= 1.0
    uint16_t spiRingDrops;       // bytes 56-57: full Daisy SPI ring drop count
    uint8_t  cleanTrackLoadedMask;   // byte 58: clean tracks 0-3 loaded in Daisy
    uint8_t  cleanTrackPlayingMask;  // byte 59: clean tracks 0-3 currently running
} StatusResponse;  // 60 bytes total

// --- Notification Event (32 bytes — returned by CMD_GET_EVENTS) ---
#define EVT_SD_BOOT_DONE       0x01  // Boot: LIVE PADS loaded from SD
#define EVT_SD_KIT_LOADED      0x02  // Kit loaded by CMD_SD_LOAD_KIT
#define EVT_SD_SAMPLE_LOADED   0x03  // Single sample loaded by CMD_SD_LOAD_SAMPLE
#define EVT_SD_KIT_UNLOADED    0x04  // Kit unloaded by CMD_SD_UNLOAD_KIT
#define EVT_SD_ERROR           0x05  // Error loading sample
#define EVT_SD_XTRA_LOADED     0x06  // Boot: XTRA PADS loaded from /data/xtra

typedef struct __attribute__((packed)) {
    uint8_t  type;           // EVT_SD_* event type
    uint8_t  padCount;       // pads affected
    uint8_t  padMaskLo;      // bitmask pads 0-7
    uint8_t  padMaskHi;      // bitmask pads 8-15
    uint8_t  padMaskXtra;    // bitmask pads 16-23
    uint8_t  reserved[3];
    char     name[24];       // kit/sample name, null-terminated
} NotifyEvent;

// CMD_GET_EVENTS response: [count(1)] + [NotifyEvent(32)] × count
// Maximum 4 events per call. Poll again if evtCount > returned count.
#define MAX_EVENTS_PER_CALL  4
typedef struct __attribute__((packed)) {
    uint8_t      count;      // number of events in this response (0-4)
    NotifyEvent  events[MAX_EVENTS_PER_CALL]; // up to 4 events
} EventsResponse;  // 1 + 4*32 = 129 bytes max

typedef struct __attribute__((packed)) {
    uint32_t timestamp;      // millis() from ESP32
} PingPayload;

typedef struct __attribute__((packed)) {
    uint32_t echoTimestamp;  // Same timestamp returned
    uint32_t stm32Uptime;   // millis() from STM32
} PongResponse;

// ═══════════════════════════════════════════════════════
// FILTER & DISTORTION ENUMS (shared between ESP32 & STM32)
// ═══════════════════════════════════════════════════════

// Filter types
#define FTYPE_NONE         0
#define FTYPE_LOWPASS      1
#define FTYPE_HIGHPASS     2
#define FTYPE_BANDPASS     3
#define FTYPE_NOTCH        4
#define FTYPE_ALLPASS      5
#define FTYPE_PEAKING      6
#define FTYPE_LOWSHELF     7
#define FTYPE_HIGHSHELF    8
#define FTYPE_RESONANT     9
#define FTYPE_LADDER       10  // Moog Ladder 24dB/oct
#define FTYPE_SVF_LP       11  // State Variable Filter LP
#define FTYPE_SVF_HP       12  // State Variable Filter HP
#define FTYPE_SVF_BP       13  // State Variable Filter BP
#define FTYPE_COMB         14  // Comb filter resonator
#define FTYPE_REVERSE      17
#define FTYPE_HALFSPEED    18
#define FTYPE_STUTTER      19

// Distortion modes
#define DMODE_SOFT   0
#define DMODE_HARD   1
#define DMODE_TUBE   2
#define DMODE_FUZZ   3

// Filter preset structure (for ESP32 side UI)
typedef struct {
    uint8_t type;
    float cutoff;
    float resonance;
    float gain;
    const char* name;
} FilterPresetInfo;

// ═══════════════════════════════════════════════════════
// MASTER FX ROUTE IDs (CMD_MASTER_FX_ROUTE 0x27)
// ═══════════════════════════════════════════════════════
enum MasterFxRouteId : uint8_t {
    MASTER_FX_ROUTE_FILTER     = 0,
    MASTER_FX_ROUTE_DELAY      = 1,
    MASTER_FX_ROUTE_PHASER     = 2,
    MASTER_FX_ROUTE_FLANGER    = 3,
    MASTER_FX_ROUTE_COMP       = 4,
    MASTER_FX_ROUTE_REVERB     = 5,
    MASTER_FX_ROUTE_CHORUS     = 6,
    MASTER_FX_ROUTE_TREMOLO    = 7,
    MASTER_FX_ROUTE_WAVEFOLDER = 8,
    MASTER_FX_ROUTE_LIMITER    = 9,
    MASTER_FX_ROUTE_AUTOWAH    = 10,
    MASTER_FX_ROUTE_EARLY_REF  = 11,
};

typedef struct __attribute__((packed)) {
    uint8_t  fxId;       // MasterFxRouteId
    uint8_t  connected;  // 0=bypass, 1=connected
} MasterFxRoutePayload;

// ═══════════════════════════════════════════════════════
// NEW MASTER FX PAYLOADS
// ═══════════════════════════════════════════════════════

// CMD_CHOKE_GROUP (0xAF) — 2 bytes
typedef struct __attribute__((packed)) {
    uint8_t  pad;        // 0-15
    uint8_t  group;      // 0=none, 1-8=group
} ChokeGroupPayload;

// CMD_SONG_UPLOAD (0xF2)
#define SONG_MAX_ENTRIES  32
typedef struct __attribute__((packed)) {
    uint8_t  pattern;    // 0-15
    uint8_t  repeats;    // 1-255 (0 treated as 1)
} SongEntry;

// CMD_SONG_GET_POS (0xF4) response
typedef struct __attribute__((packed)) {
    uint8_t  songIdx;       // position in chain
    uint8_t  currentPattern;
    uint8_t  songRepeatCnt;
    uint8_t  reserved;
} SongPosResponse;

// CMD_TRACK_LFO_CONFIG (0x67) — 7 bytes
typedef struct __attribute__((packed)) {
    uint8_t  track;      // 0-15
    uint8_t  wave;       // 0=sine, 1=triangle, 2=sample&hold
    uint8_t  target;     // 0-8 (TrackLfoTargetEx)
    uint8_t  rateHi;     // rate >> 8
    uint8_t  rateLo;     // rate & 0xFF
    uint8_t  depthHi;    // depth >> 8
    uint8_t  depthLo;    // depth & 0xFF
} TrackLfoConfigPayload;

// LFO target IDs (expanded for per-track LFO)
enum TrackLfoTargetEx : uint8_t {
    TLFO_TGT_GAIN       = 0,
    TLFO_TGT_PAN        = 1,
    TLFO_TGT_FILTER     = 2,
    TLFO_TGT_PITCH      = 3,
    TLFO_TGT_ECHO_TIME  = 4,
    TLFO_TGT_DIST_DRIVE = 5,
    TLFO_TGT_CRUSH      = 6,
    TLFO_TGT_SEND_REV   = 7,
    TLFO_TGT_SEND_DEL   = 8,
};

#endif // PROTOCOL_H
