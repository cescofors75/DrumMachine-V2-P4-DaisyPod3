/* ═══════════════════════════════════════════════════════════════════
 *  wavetable_osc.h  —  RED808 Drum Machine
 *  Wavetable Oscillator Engine  (SYNTH_ENGINE_WTOSC = 4)
 *
 *  ARM Cortex-M7 (STM32H750) optimised:
 *    • 8 bandlimited wavetables × 1024 samples = 32 KB static SRAM
 *    • Tables generadas una sola vez en Init() – sin sinf() en callback
 *    • Phase accumulator entero enmascarado (potencia de 2)
 *    • Interpolación lineal entre muestras y entre tablas (morph)
 *    • 8 voces polifónicas con voz robada (oldest steal)
 *    • Envolvente AD per-voz
 *    • Filtro biquad LP compartido por engine
 *    • LFO interno: modula wave_position / pitch / volumen
 *
 *  Arquitectura de flujo:
 *    Phase Acc → Table Lookup+Interp → Morph A↔B → Env → LFO apply
 *      → Filter → gain → mix
 *
 *  Control vía CMD_SYNTH_PARAM (engine=4):
 *    paramId  0  wave_position  0.0-7.0  (fractional = morph)
 *    paramId  1  attack_ms      0-2000
 *    paramId  2  decay_ms       1-8000
 *    paramId  3  volume         0.0-1.0
 *    paramId  4  filter_cutoff  20-18000 Hz  (0=bypass)
 *    paramId  5  filter_q       0.2-20.0
 *    paramId  6  lfo_rate_hz    0.01-20.0
 *    paramId  7  lfo_depth      0.0-1.0
 *    paramId  8  lfo_target     0=wave 1=pitch 2=vol
 *
 *  Triggers desde DSQ:
 *    engine=4 en dsqTrackEngine[t]  →  wtOsc.NoteOn(note, vel)
 *    note MIDI viene de trackWtNote[t]  (default: cromático desde C3)
 *
 * ═══════════════════════════════════════════════════════════════════ */
#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>

/* ─── Constantes ─────────────────────────────────────────────────── */
#define WT_TABLE_SIZE   1024          /* potencia de 2 → bitmask barato  */
#define WT_TABLE_MASK   (WT_TABLE_SIZE - 1)
#define WT_NUM_WAVES    8             /* 8 formas × 1024 × 4B = 32 KB   */
#define WT_MAX_VOICES   8             /* polifonía máxima                */

#ifndef WTOSC_PI
#define WTOSC_PI  3.14159265358979f
#endif

#ifndef WTOSC_CLAMP
#define WTOSC_CLAMP(x,lo,hi)  ((x)<(lo)?(lo):(x)>(hi)?(hi):(x))
#endif

/* ─── Wave IDs ───────────────────────────────────────────────────── */
enum WtWaveId : uint8_t {
    WT_WAVE_SINE     = 0,  /* Seno puro                               */
    WT_WAVE_TRI      = 1,  /* Triángulo                               */
    WT_WAVE_SAW      = 2,  /* Sierra BL (8 armónicos)                 */
    WT_WAVE_SQUARE   = 3,  /* Cuadrada BL (armónicos impares ≤9)      */
    WT_WAVE_PULSE25  = 4,  /* Pulso 25% (timbre nasal)                */
    WT_WAVE_SOFTSINE = 5,  /* Seno + 2º armónico (órgano 8'+4')       */
    WT_WAVE_ORGAN    = 6,  /* 8' + 4' + 2.67' (Hammond lite)         */
    WT_WAVE_SOFTSQ   = 7,  /* Cuadrada suavizada (tanh shaping)       */
};

/* ─── LFO targets ────────────────────────────────────────────────── */
enum WtLfoTarget : uint8_t {
    WT_LFO_WAVE  = 0,
    WT_LFO_PITCH = 1,
    WT_LFO_VOL   = 2,
};

/* ═══════════════════════════════════════════════════════════════════
 *  Biquad LP — Direct Form 1 (AudioNoise / torvalds/biquad.h style)
 *  DF1 es más estable numéricamente que DF2 con coeficientes flotantes
 * ═══════════════════════════════════════════════════════════════════ */
struct WtBiquad {
    float b0 = 1.f, b1 = 0.f, b2 = 0.f;
    float a1 = 0.f, a2 = 0.f;
    float x1 = 0.f, x2 = 0.f;
    float y1 = 0.f, y2 = 0.f;

    void SetLPF(float freq, float Q, float sr) {
        float w0    = 2.0f * WTOSC_PI * WTOSC_CLAMP(freq, 20.f, sr * 0.499f) / sr;
        float sinw  = sinf(w0);
        float cosw  = cosf(w0);
        float alpha = sinw / (2.0f * WTOSC_CLAMP(Q, 0.1f, 20.0f));
        float inv   = 1.0f / (1.0f + alpha);
        float b1t   = (1.0f - cosw) * inv;
        b0 =  b1t * 0.5f;
        b1 =  b1t;
        b2 =  b0;
        a1 = -2.0f * cosw * inv;
        a2 = (1.0f - alpha) * inv;
    }

    inline float Process(float x0) {
        float y0 = b0*x0 + b1*x1 + b2*x2 - a1*y1 - a2*y2;
        x2 = x1;  x1 = x0;
        y2 = y1;  y1 = y0;
        return y0;
    }

    void Reset() { x1 = x2 = y1 = y2 = 0.0f; }
};

/* ═══════════════════════════════════════════════════════════════════
 *  Per-Voice State
 * ═══════════════════════════════════════════════════════════════════ */
struct WtVoice {
    bool     active    = false;
    uint8_t  note      = 69;       /* MIDI A4                          */
    float    phase     = 0.0f;     /* 0 .. TABLE_SIZE (float)          */
    float    phase_inc = 0.0f;     /* samples per sample at note freq  */
    float    wave_pos  = 0.0f;     /* 0..(NUM_WAVES-1), fractional=morph */
    float    gainL     = 0.5f;
    float    gainR     = 0.5f;
    float    env       = 0.0f;
    float    envAtkInc = 1.0f;     /* per-sample attack increment      */
    float    envDecCoef= 0.9995f;  /* per-sample decay multiplier      */
    uint8_t  envStage  = 2;        /* 0=atk  1=dec  2=idle             */
    uint32_t age       = 0;
};

/* ═══════════════════════════════════════════════════════════════════
 *  WavetableOsc  — main engine class
 * ═══════════════════════════════════════════════════════════════════ */
class WavetableOsc {
public:
    float volume = 0.75f;

    /* ── Init (call once before audio starts) ────────────────────── */
    void Init(float sample_rate) {
        sr_           = sample_rate;
        voiceAge_     = 0;
        globalWavePos_= 0.0f;
        atkMs_        = 5.0f;
        decMs_        = 300.0f;
        lfoPhase_     = 0.0f;
        lfoRate_      = 2.0f;
        lfoDepth_     = 0.0f;
        lfoTarget_    = WT_LFO_WAVE;
        filterCutoff_ = 8000.0f;
        filterQ_      = 0.707f;
        filterActive_ = false;
        for(int i = 0; i < WT_MAX_VOICES; i++) voices_[i] = WtVoice();
        filter_.Reset();
        filter_.SetLPF(filterCutoff_, filterQ_, sr_);
        GenerateTables();
    }

    /* ── Trigger a voice ─────────────────────────────────────────── */
    void NoteOn(uint8_t note, float vel, float wave_pos_override = -1.0f) {
        int slot = AllocVoice(note);
        WtVoice& v = voices_[slot];

        float freq     = MidiToHz(note);
        v.active       = true;
        v.note         = note;
        v.phase        = 0.0f;
        v.phase_inc    = freq * (float)WT_TABLE_SIZE / sr_;
        v.wave_pos     = (wave_pos_override >= 0.0f) ? wave_pos_override : globalWavePos_;

        float gain     = WTOSC_CLAMP(vel, 0.0f, 1.0f) * volume;
        v.gainL = v.gainR = gain;

        float atkSamp  = atkMs_ * 0.001f * sr_;
        v.envAtkInc    = (atkSamp > 1.0f) ? (1.0f / atkSamp) : 1.0f;
        v.envDecCoef   = DecayCoef(decMs_, sr_);
        v.envStage     = (atkSamp <= 1.0f) ? 1 : 0;
        v.env          = (v.envStage == 1) ? 1.0f : 0.0f;
        v.age          = voiceAge_++;
    }

    /* ── Release voice (move to decay stage) ────────────────────── */
    void NoteOff(uint8_t note) {
        for(int i = 0; i < WT_MAX_VOICES; i++)
            if(voices_[i].active && voices_[i].note == note && voices_[i].envStage == 0)
                voices_[i].envStage = 1;
    }

    void AllNotesOff() {
        for(int i = 0; i < WT_MAX_VOICES; i++)
            if(voices_[i].active) voices_[i].envStage = 1;
    }

    /* ── Main DSP — call once per sample from AudioCallback ──────── */
    float Process() {
        if(ActiveVoiceCount() == 0)
            return 0.0f;

        /* Advance global LFO */
        float lfoVal = 0.0f;
        if(lfoDepth_ > 0.001f) {
            lfoPhase_ += lfoRate_ / sr_;
            if(lfoPhase_ >= 1.0f) lfoPhase_ -= 1.0f;
            lfoVal = sinf(2.0f * WTOSC_PI * lfoPhase_) * lfoDepth_;
        }

        float out = 0.0f;

        for(int vi = 0; vi < WT_MAX_VOICES; vi++) {
            WtVoice& v = voices_[vi];
            if(!v.active) continue;

            /* ── Envelope ── */
            if(v.envStage == 0) {
                v.env += v.envAtkInc;
                if(v.env >= 1.0f) { v.env = 1.0f; v.envStage = 1; }
            } else if(v.envStage == 1) {
                v.env *= v.envDecCoef;
                if(v.env < 0.0002f) { v.env = 0.0f; v.active = false; continue; }
            }

            /* ── LFO modulation ── */
            float phase_inc_mod = v.phase_inc;
            float wpos          = v.wave_pos;
            float vol_mod       = 1.0f;

            if(lfoDepth_ > 0.001f) {
                switch(lfoTarget_) {
                    case WT_LFO_WAVE:
                        /* modula posición en la wavetable: morph entre ondas */
                        wpos = WTOSC_CLAMP(wpos + lfoVal * (float)(WT_NUM_WAVES - 1) * 0.5f,
                                           0.0f, (float)(WT_NUM_WAVES - 1));
                        break;
                    case WT_LFO_PITCH:
                        /* ±0.5 octava de vibrato */
                        phase_inc_mod *= powf(2.0f, lfoVal * 0.5f);
                        break;
                    case WT_LFO_VOL:
                        vol_mod = WTOSC_CLAMP(1.0f + lfoVal, 0.0f, 1.5f);
                        break;
                }
            }

            /* ── Phase índice entero + fracción (lineal interp) ── */
            int   idxA   = (int)v.phase & WT_TABLE_MASK;
            int   idxB   = (idxA + 1)   & WT_TABLE_MASK;
            float frac   = v.phase - (float)(int)v.phase;

            /* ── Morph entre dos tablas adyacentes ── */
            int   wA     = (int)wpos;
            int   wB     = wA + 1;
            if(wB >= WT_NUM_WAVES) wB = WT_NUM_WAVES - 1;
            float wMix   = wpos - (float)wA;

            /* Interpolación intra-tabla × inter-tabla */
            float sA = wavetable_[wA][idxA] + frac * (wavetable_[wA][idxB] - wavetable_[wA][idxA]);
            float sB = wavetable_[wB][idxA] + frac * (wavetable_[wB][idxB] - wavetable_[wB][idxA]);
            float s  = sA + wMix * (sB - sA);

            /* ── Advance phase accumulator ── */
            v.phase += phase_inc_mod;
            if(v.phase >= (float)WT_TABLE_SIZE) v.phase -= (float)WT_TABLE_SIZE;

            /* ── Acumular en salida ── */
            out += s * v.env * v.gainL * vol_mod;
        }

        /* ── Filter compartido ── */
        if(filterActive_)
            out = filter_.Process(out);

        return out;
    }

    /* ── Global wave position (morph) ───────────────────────────── */
    void SetWavePos(float pos) {
        globalWavePos_ = WTOSC_CLAMP(pos, 0.0f, (float)(WT_NUM_WAVES - 1));
        /* Actualizar voces activas para cambio en tiempo real */
        for(int i = 0; i < WT_MAX_VOICES; i++)
            if(voices_[i].active) voices_[i].wave_pos = globalWavePos_;
    }

    /* ── Envelope ─────────────────────────────────────────────── */
    void SetAttack(float ms) { atkMs_ = WTOSC_CLAMP(ms, 0.0f, 2000.0f); }
    void SetDecay (float ms) { decMs_ = WTOSC_CLAMP(ms, 1.0f, 8000.0f); }

    /* ── Filter (0 Hz = bypass) ─────────────────────────────────── */
    void SetFilter(float cutoffHz, float Q) {
        filterCutoff_ = cutoffHz;
        filterQ_      = Q;
        filterActive_ = (cutoffHz > 20.0f && cutoffHz < 17000.0f);
        if(filterActive_)
            filter_.SetLPF(cutoffHz, Q, sr_);
        else
            filter_.Reset();
    }

    /* ── Internal LFO ───────────────────────────────────────────── */
    void SetLfo(float rateHz, float depth, WtLfoTarget target) {
        lfoRate_   = WTOSC_CLAMP(rateHz, 0.01f, 20.0f);
        lfoDepth_  = WTOSC_CLAMP(depth, 0.0f, 1.0f);
        lfoTarget_ = target;
    }

    /* ── Query ──────────────────────────────────────────────────── */
    uint8_t ActiveVoiceCount() const {
        uint8_t c = 0;
        for(int i = 0; i < WT_MAX_VOICES; i++) if(voices_[i].active) c++;
        return c;
    }

    bool IsActive() const { return ActiveVoiceCount() > 0; }

    float GetWavePos() const { return globalWavePos_; }

private:
    float    sr_ = 48000.0f;

    /* ── 32 KB de tablas: static para ir a SRAM, no al stack ─────── */
    static float wavetable_[WT_NUM_WAVES][WT_TABLE_SIZE];
    static bool  tablesReady_;

    WtVoice  voices_[WT_MAX_VOICES];
    uint32_t voiceAge_      = 0;
    float    globalWavePos_ = 0.0f;
    float    atkMs_         = 5.0f;
    float    decMs_         = 300.0f;
    float    lfoPhase_      = 0.0f;
    float    lfoRate_       = 2.0f;
    float    lfoDepth_      = 0.0f;
    WtLfoTarget lfoTarget_  = WT_LFO_WAVE;
    float    filterCutoff_  = 8000.0f;
    float    filterQ_       = 0.707f;
    bool     filterActive_  = false;
    WtBiquad filter_;

    /* ── Voice allocation — robo de la más antigua ─────────────── */
    int AllocVoice(uint8_t note) {
        /* 1: reutilizar misma nota (legato / retrigger) */
        for(int i = 0; i < WT_MAX_VOICES; i++)
            if(voices_[i].active && voices_[i].note == note) return i;
        /* 2: slot libre */
        for(int i = 0; i < WT_MAX_VOICES; i++)
            if(!voices_[i].active) return i;
        /* 3: robar la más antigua */
        uint32_t oldest = UINT32_MAX; int best = 0;
        for(int i = 0; i < WT_MAX_VOICES; i++)
            if(voices_[i].age < oldest) { oldest = voices_[i].age; best = i; }
        return best;
    }

    /* ── MIDI note → Hz ─────────────────────────────────────────── */
    static float MidiToHz(uint8_t note) {
        return 440.0f * powf(2.0f, ((float)(int)note - 69.0f) / 12.0f);
    }

    /* ── Decay coefficient: llega a -60 dB en `ms` milliseconds ─── */
    static float DecayCoef(float ms, float sr) {
        if(ms <= 0.0f) return 0.0f;
        return powf(0.001f, 1.0f / (ms * 0.001f * sr));
    }

    /* ═══════════════════════════════════════════════════════════════
     *  Wavetable generation — ejecutado una sola vez en Init()
     *  Todas las tablas son bandlimited vía suma de armónicos.
     *  Sin malloc. Sin sinf() en el callback.
     * ═══════════════════════════════════════════════════════════════ */
    void GenerateTables() {
        if(tablesReady_) return;
        tablesReady_ = true;

        const float inv_ts = 1.0f / (float)WT_TABLE_SIZE;

        for(int i = 0; i < WT_TABLE_SIZE; i++) {
            float phase = (float)i * inv_ts;          /* 0..1 */
            float phi   = phase * 2.0f * WTOSC_PI;   /* 0..2π */

            /* 0: Seno puro ──────────────────────────────────────── */
            wavetable_[WT_WAVE_SINE][i] = sinf(phi);

            /* 1: Triángulo ─────────────────────────────────────── */
            if      (phase < 0.25f)  wavetable_[WT_WAVE_TRI][i] =  phase * 4.0f;
            else if (phase < 0.75f)  wavetable_[WT_WAVE_TRI][i] =  2.0f - phase * 4.0f;
            else                     wavetable_[WT_WAVE_TRI][i] =  phase * 4.0f - 4.0f;

            /* 2: Sierra BL — suma de primeros 8 armónicos ──────── */
            {
                float s = 0.0f;
                for(int h = 1; h <= 8; h++)
                    s += sinf(phi * (float)h) / (float)h;
                wavetable_[WT_WAVE_SAW][i] = s * (2.0f / WTOSC_PI);
            }

            /* 3: Cuadrada BL — armónicos impares 1,3,5,7,9 ─────── */
            {
                float s = 0.0f;
                for(int h = 1; h <= 9; h += 2)
                    s += sinf(phi * (float)h) / (float)h;
                wavetable_[WT_WAVE_SQUARE][i] = WTOSC_CLAMP(s * (4.0f / WTOSC_PI), -1.0f, 1.0f);
            }

            /* 4: Pulso 25% — suma de cuadrada + cuadrada desplazada */
            {
                float s = 0.0f, q = 0.0f;
                float phi2 = (phase + 0.25f) * 2.0f * WTOSC_PI;
                for(int h = 1; h <= 9; h += 2) {
                    float fh = (float)h;
                    s += sinf(phi  * fh) / fh;
                    q += sinf(phi2 * fh) / fh;
                }
                wavetable_[WT_WAVE_PULSE25][i] = WTOSC_CLAMP((s + q) * (2.0f / WTOSC_PI), -1.0f, 1.0f);
            }

            /* 5: SoftSine — 8' + 4' (órgano suave) ────────────── */
            wavetable_[WT_WAVE_SOFTSINE][i] = 0.70f * sinf(phi) + 0.30f * sinf(phi * 2.0f);

            /* 6: Organ — 8' + 4' + 2.67' (Hammond lite) ────────── */
            wavetable_[WT_WAVE_ORGAN][i] = WTOSC_CLAMP(
                0.58f * sinf(phi) +
                0.25f * sinf(phi * 2.0f) +
                0.17f * sinf(phi * 3.0f),
                -1.0f, 1.0f);

            /* 7: Soft Square — sierra → tanh shaping (analógico) ──
             *    Reemplazamos tanhf() por la aproximación rápida
             *    x/(1+|x|) de AudioNoise/distortion.h, pero para
             *    generación de tabla podemos usar tanhf() una vez.  */
            {
                float saw = 2.0f * phase - 1.0f;  /* -1..+1 */
                float drive = 3.0f;
                float shaped = tanhf(saw * drive) / tanhf(drive);
                wavetable_[WT_WAVE_SOFTSQ][i] = shaped;
            }
        }
    }
};

/* ─── Definición de static members (una sola vez, en este header) ── */
float WavetableOsc::wavetable_[WT_NUM_WAVES][WT_TABLE_SIZE];
bool  WavetableOsc::tablesReady_ = false;
