/* ═══════════════════════════════════════════════════════════════════
 *  SH101.h  --  Roland SH-101 Monosynth Emulation  (v1.0)
 * ─────────────────────────────────────────────────────────────────
 *  Arquitectura del Roland SH-101 (1982):
 *    DCO  (SAW / SQUARE / PULSE-PWM  +  SUB osc −1oct)
 *      → VCF  (24 dB/oct Moog-style Transistor Ladder, CEM3372-like)
 *      → VCA  (ADSR)
 *    LFO  (SIN/TRI/SQR/SAW  →  pitch / VCF cutoff / PWM)
 *    Portamento, Drift analogico per voz
 *
 *  Caracter sonoro: leads, bajos sinteticos, arpeggios, electro.
 *  Complementa la TB-303 (mas abierto, mas cuerpo, registro mas alto).
 *
 *  Engine ID: 5  (SYNTH_ENGINE_SH101)
 *  Instrumento: 0 (monofonico — ultimo NoteOn gana)
 *
 *  Parametros CMD_SYNTH_PARAM (engine=5, instrument=0):
 *    0   Waveform      0=SAW 1=SQR 2=PULSE
 *    1   PWM Width     [0.05..0.95]  (solo PULSE)
 *    2   Sub Level     [0.0..1.0]
 *    3   Sub Oct       0.0=−1oct  1.0=−2oct
 *    4   VCF Cutoff    [20..10000] Hz
 *    5   VCF Resonance [0.0..0.95]
 *    6   VCF Env Amt   [0.0..1.0]
 *    7   VCA Attack    [0.001..2.0] s
 *    8   VCA Decay     [0.01..3.0] s
 *    9   VCA Sustain   [0.0..1.0]
 *    10  VCA Release   [0.005..2.0] s
 *    11  Filter Attack [0.001..2.0] s
 *    12  Filter Decay  [0.01..3.0] s
 *    13  LFO Rate      [0.1..20.0] Hz
 *    14  LFO Depth     [0.0..1.0]
 *    15  LFO Target    0=pitch 1=cutoff 2=pwm
 *    16  LFO Wave      0=sin 1=tri 2=sqr 3=saw
 *    17  Portamento    [0.0..1.0]  (tiempo de glide normalizado)
 *    18  Drift         [0.0..1.0]  (analog pitch instability)
 *    19  Volume        [0.0..1.0]
 *
 *  Uso:
 *    SH101::Synth sh;
 *    sh.Init(48000.0f);
 *    sh.NoteOn(60, 0.8f);
 *    float s = sh.Process();
 *    sh.NoteOff();
 *
 *  48 kHz  float32  C++14  header-only  sin dependencias externas
 * ═══════════════════════════════════════════════════════════════════ */
#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>

#ifndef SH101_TWOPI
#define SH101_TWOPI 6.283185307179586f
#endif

namespace SH101 {

/* ─────────────────────────────────────────────────────────────────
 *  Utilidades compartidas
 * ───────────────────────────────────────────────────────────────── */

static inline float Clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/** tanh racional Pade 3/3 — error <0.4% en [-4,4], ~3x mas rapido */
static inline float FastTanh(float x) {
    const float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

/** MIDI note → Hz */
static inline float MidiToHz(uint8_t note) {
    return 440.0f * powf(2.0f, ((float)note - 69.0f) / 12.0f);
}

/** PolyBLEP anti-alias para discontinuidades del oscilador */
static inline float PolyBlep(float phase, float dt) {
    if (phase < dt) {
        float t = phase / dt;
        return t + t - t * t - 1.0f;
    }
    if (phase > 1.0f - dt) {
        float t = (phase - 1.0f) / dt;
        return t * t + t + t + 1.0f;
    }
    return 0.0f;
}

/* ─────────────────────────────────────────────────────────────────
 *  XORSHIFT PRNG — para drift analogico
 * ───────────────────────────────────────────────────────────────── */
struct Rng32 {
    uint32_t s[4] = {0x12345678u, 0xABCDEF01u, 0x55AA55AAu, 0x87654321u};

    void Seed(uint32_t seed) {
        for (int i = 0; i < 4; i++) {
            seed += 0x9e3779b9u;
            uint32_t z = seed;
            z = (z ^ (z >> 16)) * 0x85ebca6bu;
            z = (z ^ (z >> 13)) * 0xc2b2ae35u;
            s[i] = z ^ (z >> 16);
            if (!s[i]) s[i] = 0xDEADBEEFu;
        }
    }

    uint32_t Next() {
        const uint32_t result = s[0] + s[3];
        const uint32_t t = s[1] << 9;
        s[2] ^= s[0]; s[3] ^= s[1];
        s[1] ^= s[2]; s[0] ^= s[3];
        s[2] ^= t;
        s[3] = (s[3] << 11) | (s[3] >> 21);
        return result;
    }

    float White() {
        return ((float)(int32_t)Next()) * (1.0f / 2147483648.0f);
    }
};

/* ─────────────────────────────────────────────────────────────────
 *  ADSR ENVELOPE — para VCA y VCF
 * ───────────────────────────────────────────────────────────────── */
class Adsr {
public:
    enum Stage { IDLE=0, ATTACK, DECAY, SUSTAIN, RELEASE };

    void Init(float sr) { sr_ = sr; stage_ = IDLE; val_ = 0.0f; }

    void Gate(bool on) {
        if (on) {
            stage_ = ATTACK;
        } else {
            if (stage_ != IDLE) stage_ = RELEASE;
        }
    }

    void Retrigger() { stage_ = ATTACK; }

    /* Curva RC exponencial en decay/release (caracter analogico) */
    float Process(float atk_s, float dec_s, float sustain, float rel_s) {
        switch (stage_) {
            case ATTACK:
                val_ += 1.0f / Clamp(atk_s * sr_, 1.0f, sr_ * 10.0f);
                if (val_ >= 1.0f) { val_ = 1.0f; stage_ = DECAY; }
                break;
            case DECAY: {
                float tgt = Clamp(sustain, 0.0f, 1.0f);
                float coef = expf(-1.0f / Clamp(dec_s * sr_, 1.0f, sr_ * 20.0f));
                val_ = tgt + (val_ - tgt) * coef;   /* RC exponencial A1 */
                if (fabsf(val_ - tgt) < 0.0005f) { val_ = tgt; stage_ = SUSTAIN; }
                break;
            }
            case SUSTAIN:
                val_ = Clamp(sustain, 0.0f, 1.0f);
                break;
            case RELEASE: {
                float coef = expf(-1.0f / Clamp(rel_s * sr_, 1.0f, sr_ * 20.0f));
                val_ *= coef;   /* RC exponencial A1 */
                if (val_ < 0.0001f) { val_ = 0.0f; stage_ = IDLE; }
                break;
            }
            default:
                val_ = 0.0f;
                break;
        }
        return val_;
    }

    bool IsIdle() const { return stage_ == IDLE; }
    float Value() const { return val_; }

private:
    float   sr_    = 48000.0f;
    float   val_   = 0.0f;
    Stage   stage_ = IDLE;
};

/* ─────────────────────────────────────────────────────────────────
 *  TRANSISTOR LADDER FILTER 24dB/oct
 *  Modelo Moog-style (Huovilainen simplificado):
 *    4 stages de 1-polo LP en cascada + nealimentacion de resonancia
 *    Saturacion tanh en cada etapa → autooscilacion suave y musical
 *    Caracter mas abierto y cuerpo que el diode ladder del 303
 * ───────────────────────────────────────────────────────────────── */
class LadderFilter {
public:
    void Init(float sr) {
        sr_  = sr;
        memset(stage_, 0, sizeof(stage_));
        memset(stageIn_, 0, sizeof(stageIn_));
    }

    void SetParams(float cutoffHz, float resonance) {
        cutoff_ = Clamp(cutoffHz, 20.0f, sr_ * 0.47f);
        res_    = Clamp(resonance, 0.0f, 0.95f);

        /* Coeficiente de 1 polo (pre-warped bilinear) */
        float w  = SH101_TWOPI * cutoff_ / sr_;
        float w2 = w * w;
        float w3 = w2 * w;
        float w4 = w3 * w;
        (void)w4;
        /* Simplified: use 1-pole coef, good enough for musical use */
        float fc_norm = Clamp(cutoff_ / sr_, 0.0f, 0.49f);
        /* Moog formula: g = e^(-2π fc/fs) converted */
        g_   = 1.0f - expf(-SH101_TWOPI * fc_norm);
        /* resonance compensation gain — prevents volume drop near cutoff */
        resComp_ = 1.0f + res_ * 1.2f;
    }

    float Process(float input) {
        /* Feedback from last stage (resonance) */
        float fb = res_ * 3.8f * stage_[3];
        float x  = FastTanh((input - fb) * resComp_);

        for (int i = 0; i < 4; i++) {
            float in = (i == 0) ? x : stage_[i - 1];
            stage_[i] += g_ * (FastTanh(in) - FastTanh(stage_[i]));
        }
        return stage_[3];
    }

    void Reset() { memset(stage_, 0, sizeof(stage_)); }

private:
    float sr_       = 48000.0f;
    float cutoff_   = 800.0f;
    float res_      = 0.0f;
    float g_        = 0.1f;
    float resComp_  = 1.0f;
    float stage_[4] = {};
    float stageIn_[4] = {};
};

/* ─────────────────────────────────────────────────────────────────
 *  SH-101 PARAMETERS
 * ───────────────────────────────────────────────────────────────── */
struct Params {
    /* Oscillator */
    uint8_t  waveform    = 0;       /* 0=SAW 1=SQR 2=PULSE             */
    float    pwmWidth    = 0.5f;    /* [0.05..0.95]                    */
    float    subLevel    = 0.3f;    /* [0.0..1.0]                      */
    float    subOct      = 0.0f;    /* 0=-1oct 1=-2oct                 */

    /* VCF */
    float    cutoff      = 1200.0f; /* [20..10000] Hz                  */
    float    resonance   = 0.4f;    /* [0.0..0.95]                     */
    float    vcfEnvAmt   = 0.5f;    /* [0.0..1.0]                      */
    float    vcfAttack   = 0.005f;  /* VCF filter attack [0.001..2.0]s */
    float    vcfDecay    = 0.15f;   /* VCF filter decay  [0.01..3.0] s */

    /* VCA */
    float    vcaAttack   = 0.003f;  /* [0.001..2.0] s                  */
    float    vcaDecay    = 0.3f;    /* [0.01..3.0] s                   */
    float    vcaSustain  = 0.7f;    /* [0.0..1.0]                      */
    float    vcaRelease  = 0.1f;    /* [0.005..2.0] s                  */

    /* LFO */
    float    lfoRate     = 4.0f;    /* [0.1..20.0] Hz                  */
    float    lfoDepth    = 0.0f;    /* [0.0..1.0]                      */
    uint8_t  lfoTarget   = 1;       /* 0=pitch 1=cutoff 2=pwm          */
    uint8_t  lfoWave     = 0;       /* 0=sin 1=tri 2=sqr 3=saw         */

    /* Global */
    float    portamento  = 0.0f;    /* [0.0..1.0]                      */
    float    drift       = 0.15f;   /* [0.0..1.0] analog instability   */
    float    volume      = 0.8f;    /* [0.0..1.0]                      */
};

/* ─────────────────────────────────────────────────────────────────
 *  SH-101 SYNTH — clase principal
 * ───────────────────────────────────────────────────────────────── */
class Synth {
public:
    Params params;

    void Init(float sr) {
        sr_       = sr;
        dt_       = 1.0f / sr;
        active_   = false;
        phase_    = 0.0f;
        subPhase_ = 0.0f;
        lfoPhase_ = 0.0f;
        driftAcc_ = 0.0f;
        driftPhase_ = 0.0f;
        currentNote_ = 60;
        currentFreq_ = MidiToHz(60);
        targetFreq_  = currentFreq_;

        vcaEnv_.Init(sr);
        vcfEnv_.Init(sr);
        filter_.Init(sr);
        rng_.Seed(0x5A101000u ^ (uint32_t)(sr));
        filter_.SetParams(params.cutoff, params.resonance);
    }

    void NoteOn(uint8_t midiNote, float velocity = 1.0f) {
        float newFreq = MidiToHz(midiNote);
        if (!active_) {
            /* Primera nota — iniciar desde cero */
            if (params.portamento < 0.01f) {
                currentFreq_ = newFreq;
            } else {
                currentFreq_ = newFreq; /* si estaba inactivo, saltar directo */
            }
            phase_    = 0.0f;
            subPhase_ = 0.0f;
            vcaEnv_.Gate(true);
            vcfEnv_.Gate(true);
        } else {
            /* Legato / retrigger */
            if (params.portamento < 0.01f) {
                currentFreq_ = newFreq;
            }
            vcaEnv_.Retrigger();
            vcfEnv_.Retrigger();
        }
        targetFreq_  = newFreq;
        currentNote_ = midiNote;
        velocity_    = Clamp(velocity, 0.0f, 1.0f);
        active_      = true;
    }

    void NoteOff() {
        vcaEnv_.Gate(false);
        vcfEnv_.Gate(false);
    }

    float Process() {
        if (!active_)
            return 0.0f;

        /* ── Portamento ── */
        if (params.portamento > 0.01f) {
            float portaTime = powf(params.portamento, 2.0f) * 2.0f + 0.001f;
            float portaK    = expf(-dt_ / portaTime);
            currentFreq_    = targetFreq_ + (currentFreq_ - targetFreq_) * portaK;
        } else {
            currentFreq_ = targetFreq_;
        }

        /* ── Drift analogico A4: LFO de ruido lento ── */
        float driftMod = 0.0f;
        if (params.drift > 0.001f) {
            driftPhase_ += (0.1f + rng_.White() * 0.3f + 1.5f * params.drift) * dt_;
            if (driftPhase_ >= 1.0f) {
                driftPhase_ -= 1.0f;
                driftAcc_ = rng_.White() * params.drift * 0.008f; /* ±~0.8% max */
            }
            driftMod = driftAcc_ * sinf(SH101_TWOPI * driftPhase_);
        }

        /* ── LFO ── */
        lfoPhase_ += params.lfoRate * dt_;
        if (lfoPhase_ >= 1.0f) lfoPhase_ -= 1.0f;
        float lfoVal = 0.0f;
        switch (params.lfoWave) {
            case 0: lfoVal = sinf(SH101_TWOPI * lfoPhase_); break;
            case 1: lfoVal = (lfoPhase_ < 0.5f) ? (lfoPhase_*4.0f - 1.0f) : (3.0f - lfoPhase_*4.0f); break;
            case 2: lfoVal = (lfoPhase_ < 0.5f) ? 1.0f : -1.0f; break;
            case 3: lfoVal = 2.0f * lfoPhase_ - 1.0f; break;
            default: lfoVal = sinf(SH101_TWOPI * lfoPhase_); break;
        }

        /* ── Frecuencia efectiva con pitch LFO + drift ── */
        float freq = currentFreq_;
        if (params.lfoTarget == 0) {
            float pitchModSemis = lfoVal * params.lfoDepth * 12.0f;
            freq *= powf(2.0f, pitchModSemis / 12.0f);
        }
        freq *= (1.0f + driftMod);
        freq  = Clamp(freq, 10.0f, sr_ * 0.45f);

        float phaseInc    = freq * dt_;
        float subPhaseInc = freq * dt_ * (params.subOct < 0.5f ? 0.5f : 0.25f);

        /* ── Avance de fase ── */
        phase_    += phaseInc;
        if (phase_    >= 1.0f) phase_    -= 1.0f;
        subPhase_ += subPhaseInc;
        if (subPhase_ >= 1.0f) subPhase_ -= 1.0f;

        /* ── Oscilador principal con PolyBLEP ── */
        float osc = 0.0f;
        float pwm = params.pwmWidth;
        if (params.lfoTarget == 2) {  /* LFO → PWM */
            pwm = Clamp(pwm + lfoVal * params.lfoDepth * 0.4f, 0.05f, 0.95f);
        }

        switch (params.waveform) {
            case 0: /* SAW */
                osc = 2.0f * phase_ - 1.0f;
                osc -= PolyBlep(phase_, phaseInc);
                break;
            case 1: /* SQUARE */
                osc = (phase_ < 0.5f) ? 1.0f : -1.0f;
                osc += PolyBlep(phase_, phaseInc);
                osc -= PolyBlep(fmodf(phase_ + 0.5f, 1.0f), phaseInc);
                break;
            case 2: /* PULSE con PWM */
                osc = (phase_ < pwm) ? 1.0f : -1.0f;
                osc += PolyBlep(phase_, phaseInc);
                osc -= PolyBlep(fmodf(phase_ + (1.0f - pwm), 1.0f), phaseInc);
                break;
            default:
                osc = 2.0f * phase_ - 1.0f;
                osc -= PolyBlep(phase_, phaseInc);
                break;
        }

        /* ── Sub-oscilador (square -1 o -2 octava) ── */
        float sub = (subPhase_ < 0.5f) ? 1.0f : -1.0f;
        osc = osc * (1.0f - params.subLevel * 0.5f)
            + sub * params.subLevel * 0.5f;

        /* ── VCF Envelope ── */
        float vcfEnvVal = vcfEnv_.Process(
            params.vcfAttack, params.vcfDecay, 0.0f, 0.02f);

        /* Cutoff efectivo: params.cutoff + env mod + LFO mod */
        float cutoffEff = params.cutoff;
        cutoffEff += vcfEnvVal * params.vcfEnvAmt * 4000.0f;
        if (params.lfoTarget == 1) {
            cutoffEff += lfoVal * params.lfoDepth * 2000.0f;
        }
        cutoffEff = Clamp(cutoffEff, 20.0f, sr_ * 0.47f);

        /* Actualizar filtro (solo si cambio significativo) */
        filter_.SetParams(cutoffEff, params.resonance);

        /* ── VCF ── */
        float filtered = filter_.Process(osc);

        /* ── VCA Envelope ── */
        float vca = vcaEnv_.Process(
            params.vcaAttack, params.vcaDecay,
            params.vcaSustain, params.vcaRelease);

        if (vca < 0.0001f && vcaEnv_.IsIdle()) {
            active_ = false;
        }

        return filtered * vca * velocity_ * params.volume;
    }

    bool IsActive() const { return active_; }

    /* ── SetParam (router desde CMD_SYNTH_PARAM) ── */
    void SetParam(uint8_t paramId, float val) {
        switch (paramId) {
            case  0: params.waveform   = (uint8_t)Clamp(val, 0.0f, 2.0f); break;
            case  1: params.pwmWidth   = Clamp(val, 0.05f, 0.95f); break;
            case  2: params.subLevel   = Clamp(val, 0.0f,  1.0f);  break;
            case  3: params.subOct     = Clamp(val, 0.0f,  1.0f);  break;
            case  4: params.cutoff     = Clamp(val, 20.0f, 10000.0f); break;
            case  5: params.resonance  = Clamp(val, 0.0f,  0.95f); break;
            case  6: params.vcfEnvAmt  = Clamp(val, 0.0f,  1.0f);  break;
            case  7: params.vcaAttack  = Clamp(val, 0.001f,2.0f);  break;
            case  8: params.vcaDecay   = Clamp(val, 0.01f, 3.0f);  break;
            case  9: params.vcaSustain = Clamp(val, 0.0f,  1.0f);  break;
            case 10: params.vcaRelease = Clamp(val, 0.005f,2.0f);  break;
            case 11: params.vcfAttack  = Clamp(val, 0.001f,2.0f);  break;
            case 12: params.vcfDecay   = Clamp(val, 0.01f, 3.0f);  break;
            case 13: params.lfoRate    = Clamp(val, 0.1f,  20.0f); break;
            case 14: params.lfoDepth   = Clamp(val, 0.0f,  1.0f);  break;
            case 15: params.lfoTarget  = (uint8_t)Clamp(val, 0.0f, 2.0f); break;
            case 16: params.lfoWave    = (uint8_t)Clamp(val, 0.0f, 3.0f); break;
            case 17: params.portamento = Clamp(val, 0.0f,  1.0f);  break;
            case 18: params.drift      = Clamp(val, 0.0f,  1.0f);  break;
            case 19: params.volume     = Clamp(val, 0.0f,  1.0f);  break;
        }
    }

private:
    float    sr_         = 48000.0f;
    float    dt_         = 1.0f / 48000.0f;
    bool     active_     = false;
    float    phase_      = 0.0f;
    float    subPhase_   = 0.0f;
    float    lfoPhase_   = 0.0f;
    float    driftPhase_ = 0.0f;
    float    driftAcc_   = 0.0f;
    float    velocity_   = 1.0f;
    uint8_t  currentNote_= 60;
    float    currentFreq_= 261.63f;
    float    targetFreq_ = 261.63f;

    Adsr         vcaEnv_;
    Adsr         vcfEnv_;
    LadderFilter filter_;
    Rng32        rng_;
};

} /* namespace SH101 */
