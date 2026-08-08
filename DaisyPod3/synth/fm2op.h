/* ═══════════════════════════════════════════════════════════════════
 *  FM2Op.h  --  2-Operator FM Synthesizer (Yamaha-style)  (v1.0)
 * ─────────────────────────────────────────────────────────────────
 *  2-operador FM clasico (estilo Yamaha DX / OPL2):
 *    Modulator  →  Carrier  →  VCA  →  salida
 *
 *  Timbres caracteristicos:
 *    - Bells, campanas (ratio 14, index alto)
 *    - Electric bass / bajos metalicos (ratio 1, index medio)
 *    - Plucks, pizzicato (index alto, decay rapido)
 *    - Flutes, marimba (ratio 1, index bajo)
 *    - Metallic percussion (ratio 3.5 / 7, index alto)
 *
 *  Engine ID: 6  (SYNTH_ENGINE_FM2OP)
 *  Instrumento: 0 (monofonico — ultimo NoteOn gana)
 *
 *  Parametros CMD_SYNTH_PARAM (engine=6, instrument=0):
 *    0   C Attack   [0.001..2.0] s  carrier envelope attack
 *    1   C Decay    [0.01..5.0]  s  carrier envelope decay
 *    2   C Sustain  [0.0..1.0]      carrier sustain level
 *    3   C Release  [0.005..3.0] s  carrier release
 *    4   M Attack   [0.001..2.0] s  modulator envelope attack
 *    5   M Decay    [0.01..5.0]  s  modulator decay
 *    6   M Sustain  [0.0..1.0]      modulator sustain
 *    7   M Release  [0.005..3.0] s  modulator release
 *    8   Ratio      [0.5..16.0]     modulador/carrier freq ratio
 *    9   Index      [0.0..20.0]     FM depth (modulation index)
 *    10  Feedback   [0.0..1.0]      modulator self-feedback
 *    11  Algorithm  0=M→C  1=M+C (additive)  2=C+C ring-mod
 *    12  Detune     [-50..+50] cents desintonizacion del modulador
 *    13  Velocity   sensitividad index con velocidad [0.0..1.0]
 *    14  Volume     [0.0..1.0]
 *
 *  Uso:
 *    FM2Op::Synth fm;
 *    fm.Init(48000.0f);
 *    fm.params.ratio = 14.0f;   // bell
 *    fm.params.index = 3.5f;
 *    fm.NoteOn(69, 0.9f);
 *    float s = fm.Process();
 *
 *  48 kHz  float32  C++14  header-only  sin dependencias externas
 * ═══════════════════════════════════════════════════════════════════ */
#pragma once

#include <math.h>
#include <stdint.h>

#ifndef FM2OP_TWOPI
#define FM2OP_TWOPI 6.283185307179586f
#endif

namespace FM2Op {

/* ─────────────────────────────────────────────────────────────────
 *  Utilidades
 * ───────────────────────────────────────────────────────────────── */

static inline float Clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline float MidiToHz(uint8_t note) {
    return 440.0f * powf(2.0f, ((float)note - 69.0f) / 12.0f);
}

static inline float SemitoneRatio(float cents) {
    return expf(cents * 0.00057762265f); /* ln(2) / 1200 */
}

/* ─────────────────────────────────────────────────────────────────
 *  ADSR con curva RC exponencial (caracter analogico - A1)
 * ───────────────────────────────────────────────────────────────── */
class Adsr {
public:
    enum Stage { IDLE=0, ATTACK, DECAY, SUSTAIN, RELEASE };

    void Init(float sr)  {
        sr_ = sr;
        stage_ = IDLE;
        val_ = 0.0f;
        cachedAtk_ = cachedDec_ = cachedRel_ = -1.0f;
    }
    void Gate(bool on)   { stage_ = on ? ATTACK : (stage_ != IDLE ? RELEASE : IDLE); }
    void Retrigger()     { stage_ = ATTACK; }
    bool IsIdle() const  { return stage_ == IDLE; }
    float Value() const  { return val_; }

    float Process(float atk, float dec, float sus, float rel) {
        /* Envelope times only change at control rate. Cache the expensive
         * reciprocal/exp calculations instead of repeating them per sample. */
        if (atk != cachedAtk_) {
            cachedAtk_ = atk;
            attackInc_ = 1.0f / Clamp(atk * sr_, 1.0f, sr_ * 20.0f);
        }
        if (dec != cachedDec_) {
            cachedDec_ = dec;
            decayCoef_ = expf(-1.0f / Clamp(dec * sr_, 1.0f, sr_ * 30.0f));
        }
        if (rel != cachedRel_) {
            cachedRel_ = rel;
            releaseCoef_ = expf(-1.0f / Clamp(rel * sr_, 1.0f, sr_ * 30.0f));
        }
        switch (stage_) {
            case ATTACK:
                val_ += attackInc_;
                if (val_ >= 1.0f) { val_ = 1.0f; stage_ = DECAY; }
                break;
            case DECAY: {
                float tgt  = Clamp(sus, 0.0f, 1.0f);
                val_ = tgt + (val_ - tgt) * decayCoef_;
                if (fabsf(val_ - tgt) < 0.0003f) { val_ = tgt; stage_ = SUSTAIN; }
                break;
            }
            case SUSTAIN:
                val_ = Clamp(sus, 0.0f, 1.0f);
                break;
            case RELEASE: {
                val_ *= releaseCoef_;
                if (val_ < 0.0001f) { val_ = 0.0f; stage_ = IDLE; }
                break;
            }
            default:
                val_ = 0.0f;
                break;
        }
        return val_;
    }

private:
    float  sr_    = 48000.0f;
    float  val_   = 0.0f;
    float  cachedAtk_ = -1.0f, cachedDec_ = -1.0f, cachedRel_ = -1.0f;
    float  attackInc_ = 1.0f, decayCoef_ = 0.0f, releaseCoef_ = 0.0f;
    Stage  stage_ = IDLE;
};

/* ─────────────────────────────────────────────────────────────────
 *  FM 2-OP PARAMETERS
 * ───────────────────────────────────────────────────────────────── */
struct Params {
    /* Carrier envelope */
    float    cAtk    = 0.002f;    /* [0.001..2.0] s  */
    float    cDec    = 0.4f;      /* [0.01..5.0]  s  */
    float    cSus    = 0.0f;      /* [0.0..1.0]      */
    float    cRel    = 0.05f;     /* [0.005..3.0] s  */

    /* Modulator envelope */
    float    mAtk    = 0.001f;    /* [0.001..2.0] s  */
    float    mDec    = 0.2f;      /* [0.01..5.0]  s  */
    float    mSus    = 0.0f;      /* [0.0..1.0]      */
    float    mRel    = 0.05f;     /* [0.005..3.0] s  */

    /* FM parameters */
    float    ratio   = 1.0f;      /* [0.5..16.0]  mod:car ratio   */
    float    index   = 3.0f;      /* [0.0..20.0]  FM depth        */
    float    feedback= 0.0f;      /* [0.0..1.0]   mod self-fb     */
    uint8_t  algo    = 0;         /* 0=M→C 1=M+C  2=ring          */
    float    detune  = 0.0f;      /* [-50..+50] cents             */
    float    velSens = 0.7f;      /* [0.0..1.0] vel→index scale   */

    float    volume  = 0.8f;      /* [0.0..1.0]                   */
};

/* ─────────────────────────────────────────────────────────────────
 *  FM2Op SYNTH — clase principal
 * ───────────────────────────────────────────────────────────────── */
class Synth {
public:
    Params params;

    void Init(float sr) {
        sr_        = sr;
        dt_        = 1.0f / sr;
        active_    = false;
        carPhase_  = 0.0f;
        modPhase_  = 0.0f;
        modFbBuf_  = 0.0f;
        velocity_  = 1.0f;
        baseFreq_  = MidiToHz(60);
        UpdateCachedFreqs();
        carEnv_.Init(sr);
        modEnv_.Init(sr);
    }

    void NoteOn(uint8_t midiNote, float velocity = 1.0f) {
        baseFreq_  = MidiToHz(midiNote);
        UpdateCachedFreqs();
        velocity_  = Clamp(velocity, 0.0f, 1.0f);
        carPhase_  = 0.0f;
        modPhase_  = 0.0f;
        modFbBuf_  = 0.0f;
        active_    = true;
        carEnv_.Gate(true);
        modEnv_.Gate(true);
    }

    void NoteOff() {
        carEnv_.Gate(false);
        modEnv_.Gate(false);
    }

    float Process() {
        if (!active_) return 0.0f;

        /* ── Envelopes ── */
        float cEnv = carEnv_.Process(params.cAtk, params.cDec, params.cSus, params.cRel);
        float mEnv = modEnv_.Process(params.mAtk, params.mDec, params.mSus, params.mRel);

        /* Index escala con velocidad y with modulator envelope */
        float velScale = 1.0f - params.velSens * (1.0f - velocity_);
        float effIndex = params.index * mEnv * velScale;

        /* ── Modulator con self-feedback ── */
        float modSelf = modFbBuf_ * params.feedback * 1.5f;
        float modOsc  = sinf(FM2OP_TWOPI * modPhase_ + modSelf);
        modFbBuf_ = (modFbBuf_ * 0.5f) + (modOsc * 0.5f); /* lowpass suave */

        /* ── Phase advance ── */
        modPhase_ += modFreq_ * dt_;
        if (modPhase_ >= 1.0f) modPhase_ -= (uint32_t)modPhase_;

        carPhase_ += carFreq_ * dt_;
        if (carPhase_ >= 1.0f) carPhase_ -= 1.0f;

        /* ── Carrier output segun algoritmo ── */
        float output = 0.0f;
        switch (params.algo) {
            case 0: /* FM puro: modulator modula carrier */
                output = sinf(FM2OP_TWOPI * carPhase_ + effIndex * modOsc);
                break;
            case 1: /* Aditivo: carrier + modulator en paralelo */
                output  = sinf(FM2OP_TWOPI * carPhase_) * 0.7f;
                output += modOsc * Clamp(effIndex, 0.0f, 1.0f) * 0.3f;
                break;
            case 2: /* Ring modulation */
                output = sinf(FM2OP_TWOPI * carPhase_) * modOsc
                       * Clamp(effIndex, 0.0f, 1.0f);
                break;
            default:
                output = sinf(FM2OP_TWOPI * carPhase_ + effIndex * modOsc);
                break;
        }

        output *= cEnv * velocity_ * params.volume;

        /* Desactivar cuando el carrier envelope termina en idle */
        if (cEnv < 0.0001f && carEnv_.IsIdle()) {
            active_ = false;
        }

        return output;
    }

    bool IsActive() const { return active_; }

    /* ── SetParam (router desde CMD_SYNTH_PARAM) ── */
    void SetParam(uint8_t paramId, float val) {
        switch (paramId) {
            case  0: params.cAtk     = Clamp(val, 0.001f, 2.0f);  break;
            case  1: params.cDec     = Clamp(val, 0.01f,  5.0f);  break;
            case  2: params.cSus     = Clamp(val, 0.0f,   1.0f);  break;
            case  3: params.cRel     = Clamp(val, 0.005f, 3.0f);  break;
            case  4: params.mAtk     = Clamp(val, 0.001f, 2.0f);  break;
            case  5: params.mDec     = Clamp(val, 0.01f,  5.0f);  break;
            case  6: params.mSus     = Clamp(val, 0.0f,   1.0f);  break;
            case  7: params.mRel     = Clamp(val, 0.005f, 3.0f);  break;
            case  8: params.ratio    = Clamp(val, 0.5f,   16.0f); UpdateCachedFreqs(); break;
            case  9: params.index    = Clamp(val, 0.0f,   20.0f); break;
            case 10: params.feedback = Clamp(val, 0.0f,   1.0f);  break;
            case 11: params.algo     = (uint8_t)Clamp(val, 0.0f, 2.0f); break;
            case 12: params.detune   = Clamp(val, -50.0f, 50.0f); UpdateCachedFreqs(); break;
            case 13: params.velSens  = Clamp(val, 0.0f,   1.0f);  break;
            case 14: params.volume   = Clamp(val, 0.0f,   1.0f);  break;
        }
    }

private:
    void UpdateCachedFreqs() {
        const float detuneMult = (params.detune != 0.0f) ? SemitoneRatio(params.detune) : 1.0f;
        carFreq_ = baseFreq_;
        modFreq_ = baseFreq_ * params.ratio * detuneMult;
    }

    float    sr_       = 48000.0f;
    float    dt_       = 1.0f / 48000.0f;
    bool     active_   = false;
    float    carPhase_ = 0.0f;
    float    modPhase_ = 0.0f;
    float    modFbBuf_ = 0.0f;
    float    velocity_ = 1.0f;
    float    baseFreq_ = 261.63f;
    float    carFreq_  = 261.63f;
    float    modFreq_  = 261.63f;
    Adsr     carEnv_;
    Adsr     modEnv_;
};

} /* namespace FM2Op */
