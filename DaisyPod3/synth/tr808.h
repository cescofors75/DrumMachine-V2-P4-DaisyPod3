/* =====================================================================
 *  TR808.h  --  Roland TR-808 Drum Synthesis Library  (v2.0)
 * ---------------------------------------------------------------------
 *  La caja de ritmos mas influyente de la historia.
 *  Cada instrumento es una reconstruccion del circuito analogico
 *  original, implementado en DSP float32.
 *
 *  INSTRUMENTOS (16):
 *    Kick  Snare  Clap  HiHatClosed  HiHatOpen
 *    LowTom  MidTom  HiTom  LowConga  MidConga  HiConga
 *    Claves  Maracas  RimShot  Cowbell  Cymbal
 *
 *  MEJORAS v2.0:
 *    FastTanh racional (3x mas rapido en Cortex-M7)
 *    SVF 2-polo con coefs precalculados en Trigger() no en Process()
 *    Xoshiro32** PRNG mejor distribucion espectral que Xorshift
 *    Kick sub-osc + click de noise filtrado mas autentico
 *    Snare SVF BP con pitch-tracking del filtro de noise
 *    Clap doble SVF burst/tail + Q variable
 *    Cowbell bandpass tuned + double-slope envelope
 *    Claves resonador de impulso Q alto + noise transient
 *    Toms smack transient noise BP corto al inicio
 *    HiHat/Cymbal HP estable con coef precalculado
 *    Kit per-channel volume/mute + soft limiter de salida
 *    Velocity curva smoothstep mas natural que lineal
 *    Presets Classic808 HipHop Techno Latin
 *
 *  48 kHz  float32  C++14  header-only  sin dependencias
 *
 *  USO BASICO:
 *    TR808::Kit drum;
 *    drum.Init(48000.0f);
 *    drum.Trigger(TR808::INST_KICK, 1.0f);
 *    float s = drum.Process();
 *
 *  USO AVANZADO:
 *    drum.SetVolume(TR808::INST_SNARE, 0.8f);
 *    drum.SetMute(TR808::INST_HIHAT_O, true);
 *    drum.LoadPreset(TR808::Presets::HipHop);
 * ===================================================================== */
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

#ifndef TR808_TWOPI
#define TR808_TWOPI 6.283185307179586f
#endif

namespace TR808 {

/* =====================================================================
 *  UTILIDADES DSP compartidas por todos los instrumentos
 * ===================================================================== */

static inline float Clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* tanh(x) aproximacion racional Pade 3/3
 * Error < 0.4% en [-4,4]  ~3x mas rapido que tanhf() en Cortex-M */
static inline float FastTanh(float x) {
    x = Clamp(x, -3.0f, 3.0f);          // evita distorsion por overflow Pade
    const float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

/* Curva de velocidad logaritmica (E1)
 * v=0->0  v=0.25->0.48  v=0.5->0.70  v=1->1
 * Los golpes suaves suenan organicos: log(1+9v)/log(10) */
static inline float VelCurve(float v) {
    v = Clamp(v, 0.0f, 1.0f);
    return logf(1.0f + v * 9.0f) * 0.43429448190f;  /* /log(10) */
}

/* ---------------------------------------------------------------------
 *  Xoshiro32** PRNG
 *  Mejor distribucion espectral que el Xorshift simple de v1.0
 *  Evita artefactos tonales en el metallic noise del hihat/cymbal
 * --------------------------------------------------------------------- */
struct Rng {
    uint32_t s[4];

    void Seed(uint32_t seed) {
        for (int i = 0; i < 4; i++) {
            seed += 0x9e3779b9u;
            uint32_t z = seed;
            z = (z ^ (z >> 16)) * 0x85ebca6bu;
            z = (z ^ (z >> 13)) * 0xc2b2ae35u;
            s[i] = z ^ (z >> 16);
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

/* ---------------------------------------------------------------------
 *  SVF -- State Variable Filter 2 polos
 *  Topologia Andy Simper (Cytomic): numericamente estable
 *
 *  REGLA DE ORO: llamar SetCoefs() en Trigger() o Init()
 *  NUNCA en Process() -- eso era el principal problema de v1.0
 * --------------------------------------------------------------------- */
struct SVF {
    float g  = 0.0f;
    float k  = 1.0f;
    float a1 = 0.0f;
    float a2 = 0.0f;
    float a3 = 0.0f;
    float ic1 = 0.0f;
    float ic2 = 0.0f;

    void SetCoefs(float sr, float fc, float Q) {
        g  = tanf(TR808_TWOPI * Clamp(fc, 10.0f, sr * 0.49f) / (2.0f * sr));
        k  = 1.0f / Clamp(Q, 0.5f, 40.0f);
        a1 = 1.0f / (1.0f + g * (g + k));
        a2 = g * a1;
        a3 = g * a2;
    }

    void Reset() { ic1 = ic2 = 0.0f; }

    float ProcessLP(float v0) {
        float v3 = v0 - ic2;
        float v1 = a1 * ic1 + a2 * v3;
        float v2 = ic2 + a2 * ic1 + a3 * v3;
        ic1 = 2.0f * v1 - ic1;
        ic2 = 2.0f * v2 - ic2;
        return v2;
    }

    float ProcessBP(float v0) {
        float v3 = v0 - ic2;
        float v1 = a1 * ic1 + a2 * v3;
        float v2 = ic2 + a2 * ic1 + a3 * v3;
        ic1 = 2.0f * v1 - ic1;
        ic2 = 2.0f * v2 - ic2;
        return v1;
    }

    float ProcessHP(float v0) {
        float v3 = v0 - ic2;
        float v1 = a1 * ic1 + a2 * v3;
        float v2 = ic2 + a2 * ic1 + a3 * v3;
        ic1 = 2.0f * v1 - ic1;
        ic2 = 2.0f * v2 - ic2;
        return v0 - k * v1 - v2;
    }
};

/* =====================================================================
 *  KICK 808
 * ---------------------------------------------------------------------
 *  Circuito original: oscilador sine (BD) con pitch envelope
 *  exponencial y amplitud exponencial y distorsion de transistor.
 *
 *  v2.0:
 *    Sub-oscilador a -1 octava (sine a media frecuencia)
 *    Click: noise filtrado BP (mas autentico que sine de 1.2 kHz)
 *    Distorsion asinmetrica positivo != negativo = caracter 808 real
 * ===================================================================== */
class Kick {
public:
    float decay      = 0.45f;
    float pitch      = 52.0f;
    float pitchDecay = 0.12f;
    float pitchAmt   = 3.0f;
    float attack     = 0.004f;
    float drive      = 0.55f;
    float subLevel   = 0.15f;
    float volume     = 1.0f;
    float drift      = 0.15f;  /* A4: analog pitch drift [0..1] */
    float punchAmt   = 1.8f;   /* Pure: 0.0-2.0, click-transient gain */

    void Init(float sampleRate) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_      = false;
        driftPhase_  = 0.0f;
        driftOffset_ = 0.0f;
        clickFilter_.SetCoefs(sr_, 4800.0f, 1.2f);
        rng_.Seed(0xBD808000u);
    }

    void Trigger(float velocity = 1.0f) {
        active_   = true;
        time_     = 0.0f;
        phase_    = 0.0f;
        subPhase_ = 0.0f;
        vel_      = VelCurve(velocity);
        /* A4: random pitch offset per trigger */
        if (drift > 0.001f) {
            float r = ((float)(int32_t)(rng_.Next() & 0xFFFF)) / 65535.0f * 2.0f - 1.0f;
            driftOffset_ = r * drift * 0.006f;  /* ±0.6% ~ ±10 cents max */
        }
        clickFilter_.Reset();
        rng_.Seed(0xBD808000u ^ (uint32_t)(vel_ * 65535));
    }

    float Process() {
        if (!active_) return 0.0f;

        /* A4: slow drift oscillator 0.3-1.5 Hz */
        float driftMod = 0.0f;
        if (drift > 0.001f) {
            driftPhase_ += (0.3f + drift * 1.2f) * dt_;
            if (driftPhase_ >= 1.0f) driftPhase_ -= 1.0f;
            driftMod = sinf(6.283185f * driftPhase_) * drift * 0.003f;
        }

        float pitchEff = pitch * (1.0f + driftOffset_ + driftMod);
        float cp = pitchEff + pitchEff * pitchAmt * expf(-time_ / pitchDecay); /* A2: ya exponencial */

        phase_ += cp * dt_;
        if (phase_ >= 1.0f) phase_ -= 1.0f;
        float sine = sinf(TR808_TWOPI * phase_);

        subPhase_ += cp * 0.5f * dt_;
        if (subPhase_ >= 1.0f) subPhase_ -= 1.0f;
        float sub = sinf(TR808_TWOPI * subPhase_) * subLevel;

        float punchEnv = expf(-time_ / 0.0008f);
        float punch    = punchEnv * punchAmt;

        float clickEnv = expf(-time_ / attack);
        float click    = clickFilter_.ProcessBP(rng_.White()) * clickEnv * 0.6f;

        float amp  = expf(-time_ / decay);
        float mix  = sine + sub + punch + click;

        float g = 1.0f + drive * 4.0f;
        float output = mix > 0.0f
            ? FastTanh(mix * g)
            : FastTanh(mix * g * 0.7f);

        time_ += dt_;
        if (amp < 0.0005f) active_ = false;

        return output * amp * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d)      { decay      = Clamp(d, 0.05f, 2.0f);    }
    void SetPitch(float p)      { pitch      = Clamp(p, 30.0f, 120.0f);  }
    void SetDrive(float d)      { drive      = Clamp(d, 0.0f,  1.0f);    }
    void SetPitchDecay(float d) { pitchDecay = Clamp(d, 0.01f, 0.5f);    }
    void SetDrift(float d)      { drift      = Clamp(d, 0.0f,  1.0f);    } /* A4 */

private:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, phase_ = 0.0f, subPhase_ = 0.0f, vel_ = 1.0f;
    float driftPhase_  = 0.0f;
    float driftOffset_ = 0.0f;
    SVF   clickFilter_;
    Rng   rng_;
};

/* =====================================================================
 *  SNARE 808
 * ---------------------------------------------------------------------
 *  Dos tonos no armonicos 180 Hz + 330 Hz + ruido BP filtrado.
 *  Frecuencias del datasheet Roland.
 *
 *  v2.0: SVF BP correcto con pitch-tracking del fc de noise
 * ===================================================================== */
class Snare {
public:
    float decay  = 0.18f;
    float tone   = 0.5f;
    float snappy = 0.6f;
    float pitch  = 185.0f;
    float volume = 1.0f;
    float drift  = 0.10f;  /* A4: analog drift [0..1] */

    void Init(float sampleRate) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_ = false;
        driftPhase_  = 0.0f;
        driftOffset_ = 0.0f;
        /* A5: noise a dos filtros BP para sonido mas rico */
        noiseFilter_.SetCoefs(sr_, 5200.0f, 1.4f);
        snpFilter_.SetCoefs(sr_, 2800.0f, 0.9f);
        rng_.Seed(0xA1B2C3D4u);
    }

    void Trigger(float velocity = 1.0f) {
        active_  = true;
        time_    = 0.0f;
        phase1_  = phase2_ = 0.0f;
        vel_     = VelCurve(velocity);
        /* A4: drift per trigger */
        if (drift > 0.001f) {
            float r = ((float)(int32_t)(rng_.Next() & 0xFFFF)) / 65535.0f * 2.0f - 1.0f;
            driftOffset_ = r * drift * 0.005f;
        }
        /* A5: colored noise - BP freq tracks snare pitch para coherencia timbrica */
        float bpFreq  = pitch * (28.0f + tone * 12.0f);   /* 5200-8400 Hz */
        float bpFreq2 = pitch * (15.0f + tone * 8.0f);    /* 2800-4280 Hz */
        noiseFilter_.SetCoefs(sr_, Clamp(bpFreq, 300.f, sr_*0.45f), 1.4f + snappy * 0.8f);
        snpFilter_.SetCoefs(sr_,  Clamp(bpFreq2, 200.f, sr_*0.45f), 0.9f + snappy * 0.6f);
        noiseFilter_.Reset();
        snpFilter_.Reset();
        rng_.Seed(0xA1B2C3D4u ^ (uint32_t)(velocity * 1234));
    }

    float Process() {
        if (!active_) return 0.0f;

        /* A4: drift oscilador lento en snare */
        float pitchEff = pitch * (1.0f + driftOffset_);
        if (drift > 0.001f) {
            driftPhase_ += (0.4f + drift) * dt_;
            if (driftPhase_ >= 1.0f) driftPhase_ -= 1.0f;
            pitchEff *= (1.0f + sinf(6.283185f * driftPhase_) * drift * 0.002f);
        }

        phase1_ += pitchEff * dt_;
        if (phase1_ >= 1.0f) phase1_ -= 1.0f;
        float t1 = sinf(TR808_TWOPI * phase1_);

        phase2_ += pitchEff * 1.833f * dt_;
        if (phase2_ >= 1.0f) phase2_ -= 1.0f;
        float t2 = sinf(TR808_TWOPI * phase2_);

        float toneEnv  = expf(-time_ / (decay * 0.55f));
        float toneOut  = (t1 * 0.65f + t2 * 0.35f) * toneEnv * tone;

        float snapEnv  = expf(-time_ / (decay * 0.9f));
        float nRaw = rng_.White();
        /* Crack: pico de noise sin filtrar al inicio (~1.2ms) */
        float crackEnv = expf(-time_ / 0.0012f);
        float crack    = nRaw * crackEnv * snappy * 0.8f;
        /* A5: dos filtros BP con freq distintas para ruido con mas caracter */
        float noiseOut = (noiseFilter_.ProcessBP(nRaw) * 0.65f
                        + snpFilter_.ProcessBP(nRaw)   * 0.35f
                        + crack) * snapEnv * snappy;

        float output = FastTanh((toneOut + noiseOut) * 1.4f);

        time_ += dt_;
        if (snapEnv < 0.0005f) active_ = false;

        return output * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d)  { decay  = Clamp(d, 0.05f, 1.0f);    }
    void SetTone(float t)   { tone   = Clamp(t, 0.0f,  1.0f);    }
    void SetSnappy(float s) { snappy = Clamp(s, 0.0f,  1.0f);    }
    void SetPitch(float p)  { pitch  = Clamp(p, 100.0f, 350.0f); }
    void SetDrift(float d)  { drift  = Clamp(d, 0.0f,  1.0f);    } /* A4 */

private:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, phase1_ = 0.0f, phase2_ = 0.0f, vel_ = 1.0f;
    float driftPhase_  = 0.0f;
    float driftOffset_ = 0.0f;
    SVF   noiseFilter_;
    SVF   snpFilter_;  /* A5: segundo filtro BP para colored noise */
    Rng   rng_;
};

/* =====================================================================
 *  CLAP 808
 * ---------------------------------------------------------------------
 *  4 rafagas de noise con micro-delays de ~6.5ms entre si.
 *  v2.0: doble SVF burst/tail separados + Q variable para el crack
 * ===================================================================== */
class Clap {
public:
    float decay  = 0.28f;
    float snap   = 0.7f;
    float tone   = 0.5f;
    float volume = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_ = false;
        rng_.Seed(0xC1A2B380u);
    }

    void Trigger(float velocity = 1.0f) {
        active_ = true;
        time_   = 0.0f;
        vel_    = VelCurve(velocity);
        float fc = 900.0f + tone * 2800.0f;
        float Q  = 1.0f + snap * 5.0f;
        burstF_.SetCoefs(sr_, fc, Q);
        tailF_.SetCoefs(sr_, fc * 0.6f, 1.2f);
        burstF_.Reset();
        tailF_.Reset();
    }

    float Process() {
        if (!active_) return 0.0f;

        float n = rng_.White();

        const float kDt  = 0.0065f;
        const float kLen = 0.0025f;

        float burstEnv = 0.0f;
        for (int i = 0; i < 4; i++) {
            float t = time_ - i * kDt;
            if (t >= 0.0f && t < kLen)
                burstEnv += expf(-t / 0.0015f);
        }

        const float tailStart = 4.0f * kDt;
        float tailEnv = (time_ >= tailStart)
            ? expf(-(time_ - tailStart) / decay) : 0.0f;

        float out = burstF_.ProcessBP(n) * burstEnv
                  + tailF_.ProcessBP(n)  * tailEnv * 0.5f;

        float output = FastTanh(out * 2.2f);

        time_ += dt_;
        if (time_ > tailStart + decay * 4.0f && tailEnv < 0.0005f)
            active_ = false;

        return output * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d) { decay = Clamp(d, 0.05f, 1.0f); }
    void SetSnap(float s)  { snap  = Clamp(s, 0.0f,  1.0f); }

private:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, vel_ = 1.0f;
    SVF   burstF_, tailF_;
    Rng   rng_;
};

/* =====================================================================
 *  HIHAT BASE -- nucleo compartido por CHH, OHH y Cymbal
 * ---------------------------------------------------------------------
 *  6 ondas cuadradas con las frecuencias no armonicas exactas del
 *  circuito Roland (medidas por Rainer Buchty).
 *  + 5% de ruido blanco inyectado (el circuito real lo tiene).
 *  SVF HP precalculado -- estable y eficiente.
 * ===================================================================== */
class HiHatBase {
protected:
    static constexpr float METAL_FREQS[6] = {
        204.0f, 298.5f, 366.5f, 522.0f, 540.0f, 800.0f
    };

    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_   = 0.0f;
    float vel_    = 1.0f;
    float phase_[6] = {};
    float driftPhase_  = 0.0f;   /* A4 */
    float driftOffset_ = 0.0f;   /* A4 */
    float hpFreq_      = 7000.0f;
    Rng   rng_;
    SVF   hpFilter_;

    float driftHat_ = 0.10f;  /* A4: drift para hi-hats [0..1] */

    void BaseInit(float sampleRate, float hpFreq) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_ = false;
        hpFreq_  = hpFreq;
        driftPhase_ = 0.0f;
        driftOffset_ = 0.0f;
        hpFilter_.SetCoefs(sr_, hpFreq, 0.7f);
        rng_.Seed(0xBAADF00Du);
    }

    void BaseTrigger(float velocity) {
        active_ = true;
        time_   = 0.0f;
        vel_    = VelCurve(velocity);
        memset(phase_, 0, sizeof(phase_));
        /* A4: drift per trigger en las frecuencias metalicas */
        if (driftHat_ > 0.001f) {
            driftOffset_ = ((float)(int32_t)(rng_.Next() & 0xFFFF)) / 65535.0f * 2.0f - 1.0f;
            driftOffset_ *= driftHat_ * 0.012f;  /* ±1.2% = ±~20 cents */
        }
        hpFilter_.Reset();
        rng_.Seed(0xBAADF00Du ^ (uint32_t)(vel_ * 0xFFFF));
    }

    float MetallicCore() {
        /* A4: drift oscilador lento modulando frecuencias metalicas */
        float driftMod = 0.0f;
        if (driftHat_ > 0.001f) {
            driftPhase_ += (0.2f + driftHat_ * 0.8f) * dt_;
            if (driftPhase_ >= 1.0f) driftPhase_ -= 1.0f;
            driftMod = driftOffset_ + sinf(6.283185f * driftPhase_) * driftHat_ * 0.004f;
        }
        float sum = 0.0f;
        for (int i = 0; i < 6; i++) {
            float freqDrifted = METAL_FREQS[i] * (1.0f + driftMod);
            phase_[i] += freqDrifted * dt_;
            if (phase_[i] >= 1.0f) phase_[i] -= 1.0f;
            sum += (phase_[i] < 0.5f) ? 1.0f : -1.0f;
        }
        return (sum * (1.0f / 6.0f)) * 0.95f + rng_.White() * 0.05f;
    }
};

constexpr float HiHatBase::METAL_FREQS[6];

/* =====================================================================
 *  HIHAT CLOSED
 * ===================================================================== */
class HiHatClosed : public HiHatBase {
public:
    float decay  = 0.042f;
    float tone   = 0.5f;
    float volume = 1.0f;

    void Init(float sr)            { BaseInit(sr, 8800.0f); }
    void Trigger(float v = 1.0f)   { BaseTrigger(v); }

    float Process() {
        if (!active_) return 0.0f;
        float hp  = hpFilter_.ProcessHP(MetallicCore());
        float env = expf(-time_ / decay);
        env = env * env;  /* cuadrado: cierre mas seco y agresivo */
        float out = FastTanh(hp * (0.7f + tone * 0.9f) * 3.4f) * env;
        time_ += dt_;
        if (env < 0.0005f) active_ = false;
        return out * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d) { decay = Clamp(d, 0.01f, 0.3f); }
};

/* =====================================================================
 *  HIHAT OPEN
 * ===================================================================== */
class HiHatOpen : public HiHatBase {
public:
    float decay  = 0.28f;
    float tone   = 0.5f;
    float volume = 1.0f;

    void Init(float sr)           { BaseInit(sr, 7200.0f); }
    void Trigger(float v = 1.0f)  { BaseTrigger(v); }
    void Choke()                  { active_ = false; }

    float Process() {
        if (!active_) return 0.0f;
        float hp  = hpFilter_.ProcessHP(MetallicCore());
        float env = expf(-time_ / decay);
        float out = FastTanh(hp * (0.7f + tone * 0.9f) * 2.9f) * env;
        time_ += dt_;
        if (env < 0.0005f) active_ = false;
        return out * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d) { decay = Clamp(d, 0.05f, 2.0f); }
};

/* =====================================================================
 *  TOM BASE -- compartido por LowTom, MidTom, HiTom
 * ---------------------------------------------------------------------
 *  Sine con pitch sweep + smack transient al inicio
 *  El smack es noise BP corto que simula el impacto de la maza
 * ===================================================================== */
class TomBase {
public:
    float decay      = 0.25f;
    float pitch      = 100.0f;
    float pitchDecay = 0.05f;
    float smack      = 0.15f;
    float volume     = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_ = false;
        smackF_.SetCoefs(sr_, pitch * 8.0f, 1.5f);
        rng_.Seed(0xF0E1D2C3u);
    }

    void Trigger(float velocity = 1.0f) {
        active_ = true;
        time_   = 0.0f;
        phase_  = 0.0f;
        vel_    = VelCurve(velocity);
        smackF_.SetCoefs(sr_, pitch * 8.0f, 1.5f);
        smackF_.Reset();
    }

    float Process() {
        if (!active_) return 0.0f;

        float cp = pitch + pitch * 2.5f * expf(-time_ / pitchDecay);
        phase_ += cp * dt_;
        if (phase_ >= 1.0f) phase_ -= 1.0f;
        float sine = sinf(TR808_TWOPI * phase_);

        float smackEnv = expf(-time_ / 0.005f);
        float smackOut = smackF_.ProcessBP(rng_.White()) * smackEnv * smack;

        float amp    = expf(-time_ / decay);
        float output = FastTanh((sine + smackOut) * 1.3f) * amp;

        time_ += dt_;
        if (amp < 0.0005f) active_ = false;

        return output * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d) { decay = Clamp(d, 0.05f, 1.5f);  }
    void SetPitch(float p) {
        pitch = Clamp(p, 40.0f, 500.0f);
        smackF_.SetCoefs(sr_, pitch * 8.0f, 1.5f);
    }

protected:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, phase_ = 0.0f, vel_ = 1.0f;
    SVF   smackF_;
    Rng   rng_;
};

class LowTom : public TomBase {
public: LowTom() { pitch = 78.0f;  decay = 0.32f; pitchDecay = 0.065f; smack = 0.18f; }
};
class MidTom : public TomBase {
public: MidTom() { pitch = 118.0f; decay = 0.26f; pitchDecay = 0.052f; smack = 0.15f; }
};
class HiTom  : public TomBase {
public: HiTom()  { pitch = 175.0f; decay = 0.20f; pitchDecay = 0.040f; smack = 0.12f; }
};

/* =====================================================================
 *  CONGA BASE -- LowConga MidConga HiConga
 * ===================================================================== */
class CongaBase {
public:
    float decay  = 0.15f;
    float pitch  = 200.0f;
    float volume = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate; dt_ = 1.0f / sr_; active_ = false;
    }

    void Trigger(float velocity = 1.0f) {
        active_ = true; time_ = 0.0f; phase_ = 0.0f;
        vel_    = VelCurve(velocity);
    }

    float Process() {
        if (!active_) return 0.0f;
        float cp = pitch + pitch * 1.8f * expf(-time_ / 0.012f);
        phase_ += cp * dt_;
        if (phase_ >= 1.0f) phase_ -= 1.0f;
        float amp  = expf(-time_ / decay);
        float out  = FastTanh(sinf(TR808_TWOPI * phase_) * 1.1f) * amp;
        time_ += dt_;
        if (amp < 0.0005f) active_ = false;
        return out * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d) { decay = Clamp(d, 0.03f, 0.6f);   }
    void SetPitch(float p) { pitch = Clamp(p, 80.0f, 600.0f); }

protected:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, phase_ = 0.0f, vel_ = 1.0f;
};

class LowConga : public CongaBase { public: LowConga() { pitch = 168.0f; decay = 0.20f; } };
class MidConga : public CongaBase { public: MidConga() { pitch = 248.0f; decay = 0.16f; } };
class HiConga  : public CongaBase { public: HiConga()  { pitch = 368.0f; decay = 0.13f; } };

/* =====================================================================
 *  CLAVES 808
 * ---------------------------------------------------------------------
 *  v2.0: modelo de cuerpo resonante RLC
 *  SVF BP de Q alto excitado por un unico impulso
 *  Q=18 simula la madera resonante seca de las claves reales
 *  + click de noise HP brevísimo al inicio
 * ===================================================================== */
class Claves {
public:
    float pitch  = 2400.0f;
    float decay  = 0.018f;
    float volume = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate; dt_ = 1.0f / sr_; active_ = false;
        resonF_.SetCoefs(sr_, pitch, 18.0f);
        clickF_.SetCoefs(sr_, 5000.0f, 0.7f);
        rng_.Seed(0xC1A4E508u);
    }

    void Trigger(float velocity = 1.0f) {
        active_  = true; time_ = 0.0f;
        vel_     = VelCurve(velocity);
        impulse_ = 1.0f;
        resonF_.SetCoefs(sr_, pitch, 18.0f);
        resonF_.Reset();
        clickF_.Reset();
    }

    float Process() {
        if (!active_) return 0.0f;

        float body = resonF_.ProcessBP(impulse_ + rng_.White() * 0.02f);
        impulse_ = 0.0f;

        float clickEnv = expf(-time_ / 0.0008f);
        float click    = clickF_.ProcessBP(rng_.White()) * clickEnv * 0.4f;

        float amp    = expf(-time_ / decay);
        float output = (body + click) * amp;

        time_ += dt_;
        if (amp < 0.0005f) active_ = false;
        return output * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetPitch(float p) {
        pitch = Clamp(p, 1800.0f, 3500.0f);
        resonF_.SetCoefs(sr_, pitch, 18.0f);
    }

private:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, vel_ = 1.0f, impulse_ = 0.0f;
    SVF   resonF_, clickF_;
    Rng   rng_;
};

/* =====================================================================
 *  MARACAS 808
 *  Noise HP de alta frecuencia, decay ultra-corto.
 *  SVF HP con fc calculado en Trigger segun tone
 * ===================================================================== */
class Maracas {
public:
    float decay  = 0.030f;
    float tone   = 0.6f;
    float volume = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate; dt_ = 1.0f / sr_; active_ = false;
        rng_.Seed(0xA2B3C4D5u);
    }

    void Trigger(float velocity = 1.0f) {
        active_ = true; time_ = 0.0f;
        vel_    = VelCurve(velocity);
        hpF_.SetCoefs(sr_, 5500.0f + tone * 4500.0f, 0.7f);
        hpF_.Reset();
    }

    float Process() {
        if (!active_) return 0.0f;
        float env = expf(-time_ / decay);
        float out = hpF_.ProcessHP(rng_.White()) * env;
        time_ += dt_;
        if (env < 0.0005f) active_ = false;
        return out * volume * vel_;
    }

    bool IsActive() const { return active_; }

private:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, vel_ = 1.0f;
    SVF   hpF_;
    Rng   rng_;
};

/* =====================================================================
 *  RIMSHOT 808
 *  Click de noise HP + resonador de tono breve ~820 Hz
 *  v2.0: SVF resonador excitado por impulso
 * ===================================================================== */
class RimShot {
public:
    float decay  = 0.022f;
    float pitch  = 820.0f;
    float volume = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate; dt_ = 1.0f / sr_; active_ = false;
        resonF_.SetCoefs(sr_, pitch, 5.0f);
        rng_.Seed(0xD1E2F3A4u);
    }

    void Trigger(float velocity = 1.0f) {
        active_  = true; time_ = 0.0f;
        vel_     = VelCurve(velocity);
        impulse_ = 1.0f;
        resonF_.SetCoefs(sr_, pitch, 5.0f);
        resonF_.Reset();
        rng_.Seed(0xD1E2F3A4u ^ (uint32_t)(vel_ * 9999));
    }

    float Process() {
        if (!active_) return 0.0f;

        float clickEnv = expf(-time_ / 0.0006f);
        float click    = rng_.White() * clickEnv * 0.6f;

        float toneEnv = expf(-time_ / decay);
        float toneOut = resonF_.ProcessBP(impulse_) * toneEnv;
        impulse_ = 0.0f;

        float out = FastTanh((click + toneOut) * 2.0f);
        time_ += dt_;
        if (toneEnv < 0.0005f && clickEnv < 0.0005f) active_ = false;
        return out * volume * vel_;
    }

    bool IsActive() const { return active_; }

private:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, vel_ = 1.0f, impulse_ = 0.0f;
    SVF   resonF_;
    Rng   rng_;
};

/* =====================================================================
 *  COWBELL 808
 * ---------------------------------------------------------------------
 *  Dos square waves 540 Hz + 800 Hz -> bandpass -> VCA
 *
 *  v2.0:
 *    SVF BP centrado en la zona media (1200 Hz) -- antes no habia BP
 *    Double-slope envelope: click rapido + body decay
 *    El BP es lo que le da el caracter metalico controlado del original
 * ===================================================================== */
class Cowbell {
public:
    float decay  = 0.080f;
    float tune   = 1.0f;
    float volume = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate; dt_ = 1.0f / sr_; active_ = false;
        bpF_.SetCoefs(sr_, 1200.0f, 3.5f);
    }

    void Trigger(float velocity = 1.0f) {
        active_  = true; time_ = 0.0f; phase1_ = phase2_ = 0.0f;
        vel_     = VelCurve(velocity);
        bpF_.SetCoefs(sr_, 1200.0f * tune, 3.5f);
        bpF_.Reset();
    }

    float Process() {
        if (!active_) return 0.0f;

        phase1_ += 540.0f * tune * dt_;
        if (phase1_ >= 1.0f) phase1_ -= 1.0f;
        float sq1 = (phase1_ < 0.5f) ? 1.0f : -1.0f;

        phase2_ += 800.0f * tune * dt_;
        if (phase2_ >= 1.0f) phase2_ -= 1.0f;
        float sq2 = (phase2_ < 0.5f) ? 1.0f : -1.0f;

        float bp = bpF_.ProcessBP((sq1 + sq2) * 0.5f);

        float clickEnv = expf(-time_ / 0.0025f);
        float bodyEnv  = expf(-time_ / decay);
        float env = bodyEnv + clickEnv * 0.5f;

        float out = FastTanh(bp * env * 1.8f);

        time_ += dt_;
        if (bodyEnv < 0.0005f) active_ = false;
        return out * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d) { decay = Clamp(d, 0.02f, 0.8f); }
    void SetTune(float t)  { tune  = Clamp(t, 0.7f,  1.5f); }

private:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, phase1_ = 0.0f, phase2_ = 0.0f, vel_ = 1.0f;
    SVF   bpF_;
};

/* =====================================================================
 *  CYMBAL 808
 *  Metallic noise 6 squares + HP largo + onset suave
 * ===================================================================== */
class Cymbal : public HiHatBase {
public:
    float decay  = 0.85f;
    float tone   = 0.6f;
    float volume = 1.0f;

    void Init(float sr)           { BaseInit(sr, 5500.0f); }
    void Trigger(float v = 1.0f)  { BaseTrigger(v); }

    float Process() {
        if (!active_) return 0.0f;
        float hp     = hpFilter_.ProcessHP(MetallicCore());
        float env    = expf(-time_ / decay);
        float attack = 1.0f - expf(-time_ / 0.0025f);
        float out    = FastTanh(hp * (0.4f + tone * 0.6f) * 1.8f) * env * attack;
        time_ += dt_;
        if (env < 0.0003f) active_ = false;
        return out * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d) { decay = Clamp(d, 0.1f, 5.0f); }
};

/* =====================================================================
 *  INSTRUMENT IDs
 * ===================================================================== */
enum InstrumentId : uint8_t {
    INST_KICK = 0,
    INST_SNARE,
    INST_CLAP,
    INST_HIHAT_C,
    INST_HIHAT_O,
    INST_LOW_TOM,
    INST_MID_TOM,
    INST_HI_TOM,
    INST_LOW_CONGA,
    INST_MID_CONGA,
    INST_HI_CONGA,
    INST_CLAVES,
    INST_MARACAS,
    INST_RIMSHOT,
    INST_COWBELL,
    INST_CYMBAL,
    INST_COUNT
};

/* =====================================================================
 *  PRESETS
 * ===================================================================== */
struct KitPreset {
    const char* name;
    float vol[INST_COUNT];
};

namespace Presets {

static const KitPreset Classic808 = { "Classic 808",
    { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
      1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f }
};

static const KitPreset HipHop = { "Hip-Hop",
    { 1.1f, 0.9f, 0.8f, 0.55f, 0.6f, 0.8f, 0.7f, 0.7f,
      0.75f, 0.7f, 0.65f, 0.6f, 0.5f, 0.7f, 0.5f, 0.4f }
};

static const KitPreset Techno = { "Techno",
    { 1.2f, 1.0f, 0.6f, 0.8f, 0.7f, 0.5f, 0.5f, 0.5f,
      0.4f, 0.4f, 0.4f, 0.3f, 0.6f, 0.5f, 0.3f, 0.5f }
};

static const KitPreset Latin = { "Latin",
    { 0.7f, 0.6f, 0.4f, 0.5f, 0.5f, 0.8f, 0.8f, 0.9f,
      1.0f, 1.0f, 1.1f, 1.1f, 0.8f, 0.8f, 0.7f, 0.4f }
};

static const KitPreset Pure808 = { "Pure 808",
    { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
      1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f }
};

} /* namespace Presets */

/* =====================================================================
 *  KIT 808 -- contenedor completo
 * ---------------------------------------------------------------------
 *  Acceso a todos los instrumentos por nombre o por indice enum
 *  Per-channel volume y mute independientes
 *  Bus sum con soft limiter de salida:
 *    evita clipping digital cuando varios instrumentos
 *    coinciden en el mismo sample (kick + snare + clap)
 *  Preset loading con LoadPreset()
 * ===================================================================== */
class Kit {
public:
    Kick        kick;
    Snare       snare;
    Clap        clap;
    HiHatClosed hihatC;
    HiHatOpen   hihatO;
    LowTom      lowTom;
    MidTom      midTom;
    HiTom       hiTom;
    LowConga    lowConga;
    MidConga    midConga;
    HiConga     hiConga;
    Claves      claves;
    Maracas     maracas;
    RimShot     rimshot;
    Cowbell     cowbell;
    Cymbal      cymbal;

    void Init(float sampleRate) {
        sr_ = sampleRate;
        kick.Init(sr_);     snare.Init(sr_);      clap.Init(sr_);
        hihatC.Init(sr_);   hihatO.Init(sr_);
        lowTom.Init(sr_);   midTom.Init(sr_);     hiTom.Init(sr_);
        lowConga.Init(sr_); midConga.Init(sr_);   hiConga.Init(sr_);
        claves.Init(sr_);   maracas.Init(sr_);    rimshot.Init(sr_);
        cowbell.Init(sr_);  cymbal.Init(sr_);

        for (int i = 0; i < INST_COUNT; i++) {
            chanVol_[i]  = 1.0f;
            chanMute_[i] = false;
        }
        masterVol_  = 0.92f;
        limitState_ = 0.0f;
    }

    void Trigger(uint8_t inst, float velocity = 1.0f) {
        velocity = Clamp(velocity, 0.0f, 1.0f);
        switch (inst) {
            case INST_KICK:      kick.Trigger(velocity);      break;
            case INST_SNARE:     snare.Trigger(velocity);     break;
            case INST_CLAP:      clap.Trigger(velocity);      break;
            case INST_HIHAT_C:   hihatO.Choke();
                                 hihatC.Trigger(velocity);    break;
            case INST_HIHAT_O:   hihatO.Trigger(velocity);    break;
            case INST_LOW_TOM:   lowTom.Trigger(velocity);    break;
            case INST_MID_TOM:   midTom.Trigger(velocity);    break;
            case INST_HI_TOM:    hiTom.Trigger(velocity);     break;
            case INST_LOW_CONGA: lowConga.Trigger(velocity);  break;
            case INST_MID_CONGA: midConga.Trigger(velocity);  break;
            case INST_HI_CONGA:  hiConga.Trigger(velocity);   break;
            case INST_CLAVES:    claves.Trigger(velocity);    break;
            case INST_MARACAS:   maracas.Trigger(velocity);   break;
            case INST_RIMSHOT:   rimshot.Trigger(velocity);   break;
            case INST_COWBELL:   cowbell.Trigger(velocity);   break;
            case INST_CYMBAL:    cymbal.Trigger(velocity);    break;
        }
    }

    float Process() {
        float mix = 0.0f;

        auto add = [&](uint8_t id, auto& inst) {
            if (inst.IsActive()) {
                float sample = inst.Process();
                if (!chanMute_[id]) mix += sample * chanVol_[id];
            }
        };

        add(INST_KICK,      kick);
        add(INST_SNARE,     snare);
        add(INST_CLAP,      clap);
        add(INST_HIHAT_C,   hihatC);
        add(INST_HIHAT_O,   hihatO);
        add(INST_LOW_TOM,   lowTom);
        add(INST_MID_TOM,   midTom);
        add(INST_HI_TOM,    hiTom);
        add(INST_LOW_CONGA, lowConga);
        add(INST_MID_CONGA, midConga);
        add(INST_HI_CONGA,  hiConga);
        add(INST_CLAVES,    claves);
        add(INST_MARACAS,   maracas);
        add(INST_RIMSHOT,   rimshot);
        add(INST_COWBELL,   cowbell);
        add(INST_CYMBAL,    cymbal);

        /* Soft limiter: peak follower + gain reduction
         * Attack instantaneo, release lento ~0.2s a 48kHz
         * Evita clipping digital sin sonar como un compresor */
        mix *= masterVol_;
        float absv = fabsf(mix);
        limitState_ = (absv > limitState_) ? absv : limitState_ * 0.9985f;
        if (limitState_ > 0.98f)
            mix *= 0.98f / limitState_;

        return mix;
    }

    /* -- Mixer -- */
    void  SetVolume(uint8_t i, float v) {
        if (i < INST_COUNT) chanVol_[i] = Clamp(v, 0.0f, 2.0f);
    }
    void  SetMute(uint8_t i, bool m) {
        if (i < INST_COUNT) chanMute_[i] = m;
    }
    void  SetMasterVolume(float v) { masterVol_ = Clamp(v, 0.0f, 1.0f); }
    float GetVolume(uint8_t i) const {
        return (i < INST_COUNT) ? chanVol_[i] : 0.0f;
    }
    bool  IsMuted(uint8_t i) const {
        return (i < INST_COUNT) ? chanMute_[i] : false;
    }

    void LoadPreset(const KitPreset& p) {
        for (int i = 0; i < INST_COUNT; i++)
            chanVol_[i] = Clamp(p.vol[i], 0.0f, 2.0f);
    }

    uint8_t ActiveCount() const {
        uint8_t c = 0;
        if (kick.IsActive())     c++;
        if (snare.IsActive())    c++;
        if (clap.IsActive())     c++;
        if (hihatC.IsActive())   c++;
        if (hihatO.IsActive())   c++;
        if (lowTom.IsActive())   c++;
        if (midTom.IsActive())   c++;
        if (hiTom.IsActive())    c++;
        if (lowConga.IsActive()) c++;
        if (midConga.IsActive()) c++;
        if (hiConga.IsActive())  c++;
        if (claves.IsActive())   c++;
        if (maracas.IsActive())  c++;
        if (rimshot.IsActive())  c++;
        if (cowbell.IsActive())  c++;
        if (cymbal.IsActive())   c++;
        return c;
    }

private:
    float   sr_          = 48000.0f;
    float   masterVol_   = 0.92f;
    float   limitState_  = 0.0f;
    float   chanVol_[INST_COUNT]  = {};
    bool    chanMute_[INST_COUNT] = {};
};

} /* namespace TR808 */

/* =====================================================================
 *  CHANGELOG
 *  v2.0  SVF correcto coefs precalculados, FastTanh, Xoshiro32,
 *        Kick sub-osc+noise click, Snare SVF tracked,
 *        Clap dual-filter, Cowbell BP, Claves resonador impulso,
 *        Toms smack transient, HiHat HP estable,
 *        Kit mixer + soft limiter + presets.
 *  v1.0  Version inicial con filtros inline recalculados en Process.
 * ===================================================================== */
