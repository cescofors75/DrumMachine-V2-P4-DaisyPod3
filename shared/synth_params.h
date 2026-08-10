// =============================================================================
// shared/synth_params.h
// Read-only parameter & preset definitions for PIANO synth engines.
// Used by both BlueSlaveV2 (ESP32-S3) and BlueSlaveP4 to render
// the PIANO PARAMS LVGL screen and emit UDP {synth303Param|synthParam|synthPreset}.
//
// Mirrors RedMaster_ESP32S3/data/web/synth-editor.js.
// =============================================================================
#pragma once
#include <stdint.h>
#include <stddef.h>

// Engine indices (compatible with Daisy synthNoteOnEx / synthParam)
//   0 = 808 sampler   1 = 909 sampler   2 = 505 sampler   (NOT exposed here)
//   3 = TB-303         4 = Wavetable    5 = SH-101         6 = FM 2-Op
//   7 = reserved legacy physical-model engine (not exposed in the UI)

#define SP_ENGINE_303    3
#define SP_ENGINE_WT     4
#define SP_ENGINE_SH101  5
#define SP_ENGINE_FM2OP  6
#define SP_ENGINE_PHYS   7

#define SP_ENGINE_COUNT  4   // exposed engines 3..6

typedef struct {
    uint8_t      param_id;
    const char*  name;
    float        vmin;
    float        vmax;
    float        vdef;
    uint8_t      step_int;   // 1 if integer-stepped (waveform select etc.)
    const char*  unit;
} SynthParamDef;

typedef struct {
    uint8_t  param_id;
    float    value;
} SynthPresetVal;

typedef struct {
    const char*           name;
    const SynthPresetVal* values;
    uint8_t               count;
} SynthPreset;

typedef struct {
    uint8_t              engine;        // SP_ENGINE_*
    const char*          label;         // "303" / "WT" / "SH101" / "FM2"
    const char*          long_name;     // "TB-303 Bass" etc
    const SynthParamDef* params;
    uint8_t              param_count;
    const SynthPreset*   presets;
    uint8_t              preset_count;
} SynthEngineDef;

// ---------------------------------------------------------------------------
// 303 — 15 params
// ---------------------------------------------------------------------------
static const SynthParamDef SP_PARAMS_303[] = {
    { 0,  "Cutoff",     20.f,    5000.f, 800.f,   0, "Hz" },
    { 1,  "Resonance",  0.f,     0.97f,  0.5f,    0, "" },
    { 2,  "Env Mod",    0.f,     1.f,    0.5f,    0, "" },
    { 3,  "Decay",      0.02f,   3.f,    0.3f,    0, "s" },
    { 8,  "Attack",     0.001f,  2.f,    0.001f,  0, "s" },
    { 9,  "Sustain",    0.f,     1.f,    0.f,     0, "" },
    { 10, "Release",    0.005f,  2.f,    0.05f,   0, "s" },
    { 4,  "Accent",     0.f,     1.f,    0.5f,    0, "" },
    { 5,  "Slide",      0.01f,   0.5f,   0.06f,   0, "s" },
    { 11, "Overdrive",  0.f,     1.f,    0.f,     0, "" },
    { 12, "Sub Osc",    0.f,     1.f,    0.f,     0, "" },
    { 13, "Drift",      0.f,     1.f,    0.f,     0, "" },
    { 14, "PitchBend", -12.f,    12.f,   0.f,     0, "st" },
    { 6,  "Wave",       0.f,     1.f,    0.f,     1, "" },   // 0=SAW 1=SQR
    { 7,  "Volume",     0.f,     1.f,    0.7f,    0, "" },
};

static const SynthPresetVal SP_PRES_303_0[] = {
    {0,1200.f},{1,0.72f},{2,0.65f},{3,0.35f},{4,0.60f},{5,0.09f},{6,0},{7,0.80f},
    {8,0.001f},{9,0},{10,0.15f},{11,0.12f},{12,0.08f},{13,0.04f},{14,0}};
static const SynthPresetVal SP_PRES_303_1[] = {
    {0,900.f},{1,0.92f},{2,0.95f},{3,0.45f},{4,0.85f},{5,0.12f},{6,0},{7,0.85f},
    {8,0.001f},{9,0},{10,0.18f},{11,0.28f},{12,0.06f},{13,0.08f},{14,0}};
static const SynthPresetVal SP_PRES_303_2[] = {
    {0,240.f},{1,0.45f},{2,0.25f},{3,0.60f},{4,0.25f},{5,0.06f},{6,1.f},{7,0.90f},
    {8,0.004f},{9,0.45f},{10,0.35f},{11,0.18f},{12,0.45f},{13,0.02f},{14,0}};
static const SynthPresetVal SP_PRES_303_3[] = {
    {0,2200.f},{1,0.58f},{2,0.40f},{3,0.80f},{4,0.35f},{5,0.15f},{6,1.f},{7,0.75f},
    {8,0.010f},{9,0.35f},{10,0.40f},{11,0.08f},{12,0.18f},{13,0.12f},{14,0}};

static const SynthPreset SP_PRESETS_303[] = {
    { "Acid",      SP_PRES_303_0, 15 },
    { "Squelch",   SP_PRES_303_1, 15 },
    { "Sub Bass",  SP_PRES_303_2, 15 },
    { "Soft Lead", SP_PRES_303_3, 15 },
};

// ---------------------------------------------------------------------------
// WT — 8 params
// ---------------------------------------------------------------------------
static const SynthParamDef SP_PARAMS_WT[] = {
    { 0, "Wave Pos",   0.f,    7.f,    0.f,     0, ""   },
    { 1, "Attack",     0.f,    2000.f, 5.f,     1, "ms" },
    { 2, "Decay",      1.f,    4000.f, 300.f,   1, "ms" },
    { 3, "Volume",     0.f,    1.f,    0.75f,   0, ""   },
    { 4, "Filter Cut", 0.f,    18000.f,8000.f,  1, "Hz" },
    { 5, "LFO Rate",   0.01f,  20.f,   2.f,     0, "Hz" },
    { 6, "LFO Depth",  0.f,    1.f,    0.f,     0, ""   },
    { 7, "LFO Target", 0.f,    2.f,    0.f,     1, ""   }, // 0=Wave 1=Pitch 2=Vol
};

static const SynthPresetVal SP_PRES_WT_0[] = { {0,1.0f},{1,24},{2,1100},{3,0.84f},{4,7600},{5,0.12f},{6,0.06f},{7,0} };
static const SynthPresetVal SP_PRES_WT_1[] = { {0,2.4f},{1,0},{2,220},{3,0.92f},{4,7200},{5,2.20f},{6,0.04f},{7,0} };
static const SynthPresetVal SP_PRES_WT_2[] = { {0,5.8f},{1,6},{2,1800},{3,0.88f},{4,9800},{5,0.35f},{6,0.10f},{7,2} };
static const SynthPresetVal SP_PRES_WT_3[] = { {0,3.6f},{1,0},{2,360},{3,0.96f},{4,3600},{5,1.10f},{6,0.05f},{7,0} };
static const SynthPreset SP_PRESETS_WT[] = {
    { "Pad",      SP_PRES_WT_0, 8 },
    { "Pluck",    SP_PRES_WT_1, 8 },
    { "Organ",    SP_PRES_WT_2, 8 },
    { "PWM Bass", SP_PRES_WT_3, 8 },
};

// ---------------------------------------------------------------------------
// SH101 — 20 params
// ---------------------------------------------------------------------------
static const SynthParamDef SP_PARAMS_SH101[] = {
    { 0,  "Wave",       0.f,    2.f,    0.f,     1, "" },     // SAW/SQR/PUL
    { 1,  "PWM Width",  0.1f,   0.9f,   0.5f,    0, "" },
    { 2,  "Sub Lvl",    0.f,    1.f,    0.3f,    0, "" },
    { 3,  "Sub Oct",    0.f,    1.f,    0.f,     1, "" },     // 0=-1oct 1=-2oct
    { 4,  "VCF Cut",    20.f,   18000.f,2000.f,  1, "Hz" },
    { 5,  "VCF Res",    0.f,    1.f,    0.4f,    0, "" },
    { 6,  "Env→VCF",    0.f,    1.f,    0.5f,    0, "" },
    { 7,  "VCA Atk",    0.001f, 2.f,    0.005f,  0, "s" },
    { 8,  "VCA Dec",    0.01f,  3.f,    0.3f,    0, "s" },
    { 9,  "VCA Sus",    0.f,    1.f,    0.6f,    0, "" },
    { 10, "VCA Rel",    0.005f, 3.f,    0.15f,   0, "s" },
    { 11, "VCF Atk",    0.001f, 2.f,    0.005f,  0, "s" },
    { 12, "VCF Dec",    0.01f,  3.f,    0.2f,    0, "s" },
    { 13, "LFO Rate",   0.1f,   20.f,   4.f,     0, "Hz" },
    { 14, "LFO Depth",  0.f,    1.f,    0.f,     0, "" },
    { 15, "LFO Tgt",    0.f,    2.f,    0.f,     1, "" },     // Pitch/Cut/PWM
    { 16, "LFO Wave",   0.f,    3.f,    0.f,     1, "" },     // SIN/TRI/SQR/SAW
    { 17, "Portament",  0.f,    1.f,    0.f,     0, "" },
    { 18, "A-Drift",    0.f,    1.f,    0.1f,    0, "" },
    { 19, "Volume",     0.f,    1.f,    0.75f,   0, "" },
};

static const SynthPresetVal SP_PRES_SH_0[] = {
    {0,0},{1,0.48f},{2,0.62f},{3,1.f},{4,720.f},{5,0.22f},{6,0.44f},{7,0.001f},{8,0.16f},
    {9,0.08f},{10,0.10f},{11,0.001f},{12,0.16f},{13,0.15f},{14,0.00f},{15,0},{16,0},
    {17,0.03f},{18,0.03f},{19,0.90f}};
static const SynthPresetVal SP_PRES_SH_1[] = {
    {0,0},{1,0.44f},{2,0.24f},{3,0.f},{4,2200.f},{5,0.62f},{6,0.70f},{7,0.001f},{8,0.28f},
    {9,0.20f},{10,0.16f},{11,0.001f},{12,0.24f},{13,4.20f},{14,0.10f},{15,1.f},
    {16,0},{17,0.10f},{18,0.06f},{19,0.86f}};
static const SynthPresetVal SP_PRES_SH_2[] = {
    {0,1.f},{1,0.30f},{2,0.18f},{3,0.f},{4,3400.f},{5,0.28f},{6,0.34f},{7,0.008f},{8,0.34f},
    {9,0.62f},{10,0.24f},{11,0.008f},{12,0.34f},{13,2.40f},{14,0.18f},{15,0},
    {16,1.f},{17,0},{18,0.03f},{19,0.84f}};
static const SynthPresetVal SP_PRES_SH_3[] = {
    {0,2.f},{1,0.52f},{2,0.38f},{3,1.f},{4,1400.f},{5,0.78f},{6,0.58f},{7,0.060f},{8,0.92f},
    {9,0.68f},{10,0.72f},{11,0.050f},{12,1.10f},{13,0.45f},{14,0.28f},{15,1.f},
    {16,0},{17,0.16f},{18,0.12f},{19,0.80f}};
static const SynthPreset SP_PRESETS_SH101[] = {
    { "Bass",   SP_PRES_SH_0, 20 },
    { "Acid",   SP_PRES_SH_1, 20 },
    { "Keys",   SP_PRES_SH_2, 20 },
    { "Drone",  SP_PRES_SH_3, 20 },
};

// ---------------------------------------------------------------------------
// FM 2-Op — 15 params
// ---------------------------------------------------------------------------
static const SynthParamDef SP_PARAMS_FM2OP[] = {
    { 0,  "C Atk",      0.001f, 2.f,    0.005f,  0, "s" },
    { 1,  "C Dec",      0.01f,  3.f,    0.4f,    0, "s" },
    { 2,  "C Sus",      0.f,    1.f,    0.3f,    0, "" },
    { 3,  "C Rel",      0.005f, 3.f,    0.2f,    0, "s" },
    { 4,  "M Atk",      0.001f, 2.f,    0.001f,  0, "s" },
    { 5,  "M Dec",      0.01f,  3.f,    0.25f,   0, "s" },
    { 6,  "M Sus",      0.f,    1.f,    0.f,     0, "" },
    { 7,  "M Rel",      0.005f, 3.f,    0.1f,    0, "s" },
    { 8,  "M/C Ratio",  0.5f,   8.f,    2.f,     0, "x" },
    { 9,  "FM Index",   0.f,    12.f,   3.f,     0, "" },
    { 10, "Feedback",   0.f,    1.f,    0.f,     0, "" },
    { 11, "Algorithm",  0.f,    2.f,    0.f,     1, "" }, // FM/ADD/RING
    { 12, "Detune",   -50.f,   50.f,    0.f,     0, "ct" },
    { 13, "VelSens",    0.f,    1.f,    0.7f,    0, "" },
    { 14, "Volume",     0.f,    1.f,    0.75f,   0, "" },
};

static const SynthPresetVal SP_PRES_FM_0[] = {
    {0,0.001f},{1,0.26f},{2,0.00f},{3,0.14f},{4,0.001f},{5,0.18f},{6,0.00f},{7,0.12f},
    {8,1.00f},{9,4.20f},{10,0.06f},{11,0},{12,0.00f},{13,0.50f},{14,0.92f}};
static const SynthPresetVal SP_PRES_FM_1[] = {
    {0,0.001f},{1,1.10f},{2,0.20f},{3,0.90f},{4,0.001f},{5,0.70f},{6,0.00f},{7,0.48f},
    {8,2.00f},{9,2.80f},{10,0.04f},{11,1.f},{12,6.00f},{13,0.85f},{14,0.88f}};
static const SynthPresetVal SP_PRES_FM_2[] = {
    {0,0.001f},{1,2.20f},{2,0.00f},{3,1.40f},{4,0.001f},{5,1.10f},{6,0.00f},{7,0.80f},
    {8,3.00f},{9,7.20f},{10,0.10f},{11,0},{12,14.00f},{13,0.90f},{14,0.84f}};
static const SynthPresetVal SP_PRES_FM_3[] = {
    {0,0.004f},{1,0.44f},{2,0.28f},{3,0.22f},{4,0.001f},{5,0.34f},{6,0.12f},{7,0.22f},
    {8,1.50f},{9,9.20f},{10,0.34f},{11,2.f},{12,-10.00f},{13,0.72f},{14,0.90f}};
static const SynthPreset SP_PRESETS_FM2OP[] = {
    { "FM Bass",  SP_PRES_FM_0, 15 },
    { "EPiano",   SP_PRES_FM_1, 15 },
    { "Bell",     SP_PRES_FM_2, 15 },
    { "Growl",    SP_PRES_FM_3, 15 },
};

// ---------------------------------------------------------------------------
// PHYS / GTR — 8 params
// ---------------------------------------------------------------------------
static const SynthParamDef SP_PARAMS_PHYS[] = {
    { 1, "M Struct",   0.f, 1.f, 0.14f, 0, "" },
    { 2, "M Bright",   0.f, 1.f, 0.30f, 0, "" },
    { 3, "M Damp",     0.f, 1.f, 0.78f, 0, "" },
    { 4, "M Gain",     0.f, 1.f, 0.12f, 0, "" },
    { 6, "S Struct",   0.f, 1.f, 0.34f, 0, "" },
    { 7, "S Bright",   0.f, 1.f, 0.64f, 0, "" },
    { 8, "S Damp",     0.f, 1.f, 0.58f, 0, "" },
    { 9, "S Gain",     0.f, 1.f, 0.95f, 0, "" },
};

static const SynthPresetVal SP_PRES_PHYS_0[] = {
    {1,0.12f},{2,0.28f},{3,0.78f},{4,0.18f},{6,0.34f},{7,0.48f},{8,0.68f},{9,0.98f}
};
static const SynthPresetVal SP_PRES_PHYS_1[] = {
    {1,0.20f},{2,0.60f},{3,0.56f},{4,0.16f},{6,0.28f},{7,0.76f},{8,0.42f},{9,1.00f}
};
static const SynthPresetVal SP_PRES_PHYS_2[] = {
    {1,0.34f},{2,0.74f},{3,0.42f},{4,0.12f},{6,0.62f},{7,0.88f},{8,0.22f},{9,0.90f}
};
static const SynthPresetVal SP_PRES_PHYS_3[] = {
    {1,0.26f},{2,0.86f},{3,0.30f},{4,0.24f},{6,0.56f},{7,0.98f},{8,0.16f},{9,0.96f}
};
static const SynthPreset SP_PRESETS_PHYS[] = {
    { "Clasica",  SP_PRES_PHYS_0, 8 },
    { "Flamenco", SP_PRES_PHYS_1, 8 },
    { "Funky",    SP_PRES_PHYS_2, 8 },
    { "Electrica",SP_PRES_PHYS_3, 8 },
};

// ---------------------------------------------------------------------------
// Engine table
// ---------------------------------------------------------------------------
static const SynthEngineDef SP_ENGINES[SP_ENGINE_COUNT] = {
    { SP_ENGINE_303,   "303",   "TB-303 Bass",
      SP_PARAMS_303,   (uint8_t)(sizeof(SP_PARAMS_303)/sizeof(SP_PARAMS_303[0])),
      SP_PRESETS_303,  4 },
    { SP_ENGINE_WT,    "WT",    "Wavetable OSC",
      SP_PARAMS_WT,    (uint8_t)(sizeof(SP_PARAMS_WT)/sizeof(SP_PARAMS_WT[0])),
      SP_PRESETS_WT,   4 },
    { SP_ENGINE_SH101, "SH101", "SH-101 Lead",
      SP_PARAMS_SH101, (uint8_t)(sizeof(SP_PARAMS_SH101)/sizeof(SP_PARAMS_SH101[0])),
      SP_PRESETS_SH101,4 },
    { SP_ENGINE_FM2OP, "FM2",   "FM 2-Op",
      SP_PARAMS_FM2OP, (uint8_t)(sizeof(SP_PARAMS_FM2OP)/sizeof(SP_PARAMS_FM2OP[0])),
            SP_PRESETS_FM2OP,4 },
};
