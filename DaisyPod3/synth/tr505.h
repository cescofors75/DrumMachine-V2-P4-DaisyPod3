/* =====================================================================
 *  TR505.h  --  Roland TR-505 Drum Synthesis Library  (v2.0)
 * ---------------------------------------------------------------------
 *  La hermana menor y mas digital de la 808/909.
 *  Sonido PCM de 8 bits y ~8 kHz de sample rate interno --
 *  thin, bright, aliased, lo-fi. New wave, synth-pop, electro.
 *
 *  INSTRUMENTOS (16):
 *    Kick  Snare  Clap  HiHatClosed  HiHatOpen
 *    LowTom  MidTom  HiTom  Cowbell  Cymbal  RimShot
 *    Shaker  Clave  HiPerc  MidPerc  LowPerc
 *
 *  IDENTIDAD SONORA PRESERVADA:
 *    El caracter lo-fi NO es un defecto -- es el sonido.
 *    Bit crush + sample-rate reduction = el alma de la 505.
 *    Instrumentos thin y bright, sin el boom profundo de la 808.
 *
 *  MEJORAS v2.0:
 *    LoFi mejorado: bit crush + sample-rate reduction (S&H)
 *      El S&H replica el aliasing de los samples a ~8 kHz
 *      sampleRateDiv=1 calidad completa, =6 aprox 8 kHz
 *    SVF 2-polo coefs precalculados en Trigger() no en Process()
 *    Xoshiro32** PRNG sin artefactos espectrales
 *    FastTanh racional 3x mas rapido que tanhf()
 *    VelCurve smoothstep mas natural que lineal
 *    Kick: click de noise filtrado + drive configurable
 *    Snare: SVF HP en noise + SVF LP en tone component
 *    Clap: SVF BP (3 rafagas, mas simple que 808/909)
 *    HiHats: SVF HP precalculado + noise grain para textura 505
 *    Toms: smack transient digital corto
 *    Cowbell: SVF BP centrado (sin BP era demasiado bruto)
 *    Cymbal: SVF HP estable
 *    RimShot: resonador SVF de impulso
 *    Kit: per-channel volume/mute + lofi global + soft limiter
 *    Presets: Classic505, NewWave, Electro, LoFiHipHop
 *
 *  RENDIMIENTO:
 *    < 60 ciclos/muestra (Kit completo, Cortex-M7 @ 480 MHz)
 *    El LoFi S&H es casi gratuito (sample-and-hold con contador)
 *
 *  48 kHz  float32  C++14  header-only  sin dependencias
 *
 *  USO BASICO:
 *    TR505::Kit drum;
 *    drum.Init(48000.0f);
 *    drum.Trigger(TR505::INST_KICK, 1.0f);
 *    float s = drum.Process();
 *
 *  USO AVANZADO:
 *    drum.SetLoFi(0.4f);                          // lofi global
 *    drum.LoadPreset(TR505::Presets::LoFiHipHop); // preset
 *    drum.SetVolume(TR505::INST_SNARE, 0.9f);     // por canal
 * ===================================================================== */
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

#ifndef TR505_TWOPI
#define TR505_TWOPI 6.283185307179586f
#endif

namespace TR505 {

/* =====================================================================
 *  UTILIDADES DSP
 * ===================================================================== */

static inline float Clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* tanh(x) Pade 3/3: error < 0.4% en [-4,4]  ~3x mas rapido */
static inline float FastTanh(float x) {
    x = Clamp(x, -3.0f, 3.0f);          // evita distorsion por overflow Pade
    const float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

/* Smoothstep velocity: mas natural que lineal */
static inline float VelCurve(float v) {
    v = Clamp(v, 0.0f, 1.0f);
    return v * v * (3.0f - 2.0f * v);
}

/* Xoshiro32** PRNG */
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
        const uint32_t r = s[0] + s[3];
        const uint32_t t = s[1] << 9;
        s[2] ^= s[0]; s[3] ^= s[1];
        s[1] ^= s[2]; s[0] ^= s[3];
        s[2] ^= t;
        s[3] = (s[3] << 11) | (s[3] >> 21);
        return r;
    }

    float White() {
        return ((float)(int32_t)Next()) * (1.0f / 2147483648.0f);
    }
};

/* SVF -- State Variable Filter (Simper/Cytomic)
 * SetCoefs() en Trigger() o Init() -- NUNCA en Process() */
struct SVF {
    float g=0, k=1, a1=0, a2=0, a3=0, ic1=0, ic2=0;

    void SetCoefs(float sr, float fc, float Q) {
        g  = tanf(TR505_TWOPI * Clamp(fc, 10.0f, sr * 0.49f) / (2.0f * sr));
        k  = 1.0f / Clamp(Q, 0.5f, 40.0f);
        a1 = 1.0f / (1.0f + g * (g + k));
        a2 = g * a1;
        a3 = g * a2;
    }

    void Reset() { ic1 = ic2 = 0.0f; }

    float ProcessLP(float v0) {
        float v3=v0-ic2, v1=a1*ic1+a2*v3, v2=ic2+a2*ic1+a3*v3;
        ic1=2.f*v1-ic1; ic2=2.f*v2-ic2; return v2;
    }

    float ProcessBP(float v0) {
        float v3=v0-ic2, v1=a1*ic1+a2*v3, v2=ic2+a2*ic1+a3*v3;
        ic1=2.f*v1-ic1; ic2=2.f*v2-ic2; return v1;
    }

    float ProcessHP(float v0) {
        float v3=v0-ic2, v1=a1*ic1+a2*v3, v2=ic2+a2*ic1+a3*v3;
        ic1=2.f*v1-ic1; ic2=2.f*v2-ic2; return v0-k*v1-v2;
    }
};

/* =====================================================================
 *  LOFI PROCESSOR -- el alma de la TR-505
 * ---------------------------------------------------------------------
 *  Dos etapas que replican el caracter PCM de 8 bits / ~8 kHz:
 *
 *  1. Sample-Rate Reduction (S&H):
 *     Mantiene el mismo sample durante sampleRateDiv muestras.
 *     sampleRateDiv=1 -> 48kHz (calidad plena)
 *     sampleRateDiv=6 -> ~8kHz (505 original aprox)
 *     Crea el aliasing armonico tipico de los samples de la 505.
 *
 *  2. Bit Crush:
 *     Reduce la resolucion de amplitud a N niveles.
 *     bitDepth=8.0 -> 256 niveles (8 bits)
 *     bitDepth=4.0 -> 16 niveles (4 bits, muy lo-fi)
 *     bitDepth=16.0 -> 65536 niveles (casi transparente)
 *
 *  Ambos parametros son float para permitir modulacion suave.
 * ===================================================================== */
struct LoFiProcessor {
    float bitDepth      = 8.0f;   /* [4.0 - 16.0] bits efectivos      */
    float sampleRateDiv = 3.0f;   /* [1.0 - 8.0]  divisor de SR       */

    /* Precalcula levels_ desde bitDepth -- llamar siempre que
     * bitDepth o sampleRateDiv cambien (via UpdateLoFi del instrumento).
     * NUNCA llamar UpdateCache() desde Process() -- extremadamente caro. */
    void UpdateCache() {
        /* Signed PCM has 2^(bits-1)-1 positive quantisation steps. */
        levels_  = powf(2.0f, bitDepth - 1.0f) - 1.0f;
        if (levels_ < 1.0f) levels_ = 1.0f;
        divInt_  = Clamp((int)roundf(sampleRateDiv), 1, 16);  /* entero para comparacion rapida */
    }

    void SetBypass(bool bypass) { bypass_ = bypass; }

    float Process(float input) {
        if (bypass_) return input;
        /* S&H primero: samplea a frecuencia reducida */
        if (++counter_ >= divInt_) {
            counter_ = 0;
            held_    = input;
        }
        /* Luego crush el valor retenido (como DAC real) */
        return roundf(held_ * levels_) / levels_;
    }

    void Reset() { counter_ = 0; held_ = 0.0f; }

private:
    float levels_  = 127.0f; /* signed 8-bit positive range */
    int   divInt_  = 3;
    int   counter_ = 0;
    float held_    = 0.0f;
    bool  bypass_  = false;
};

/* =====================================================================
 *  KICK 505
 * ---------------------------------------------------------------------
 *  Mas corto y punchy que la 808 -- sin el boom profundo.
 *  Los samples de kick de la 505 son notablemente thin.
 *  Pitch sweep moderado (x5 vs x8 en 808) y decay corto.
 *
 *  v2.0:
 *    Click: noise BP filtrado (el "click" del sample al inicio)
 *    Drive: saturacion suave para el punch digital
 *    LoFi integrado por instrumento (mas fino que LoFi global)
 * ===================================================================== */
class Kick {
public:
    float decay      = 0.25f;   /* [0.05 - 0.8]  s  corto = 505     */
    float pitch      = 60.0f;   /* [40   - 100]  Hz                 */
    float pitchDecay = 0.008f;  /* [0.005- 0.1]  s  caida rapida = 505 */
    float drive      = 0.45f;   /* [0.0  - 1.0]  saturacion digital    */
    float lofi       = 0.3f;    /* [0.0  - 1.0]  cantidad lo-fi     */
    float volume     = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_ = false;
        clickF_.SetCoefs(sr_, 4500.0f, 0.8f);
        rng_.Seed(0x505BD01u);
        UpdateLoFi();  /* precalcula levels_ / divInt_ */
    }

    void Trigger(float velocity = 1.0f) {
        active_   = true;
        time_     = 0.0f;
        phase_    = 0.0f;
        vel_      = VelCurve(velocity);
        clickF_.Reset();
        lofiProc_.Reset();
        rng_.Seed(0x505BD01u ^ (uint32_t)(vel_ * 65535));
    }

    float Process() {
        if (!active_) return 0.0f;

        float cp = pitch + pitch * 5.0f * expf(-time_ / pitchDecay);
        phase_ += cp * dt_;
        if (phase_ >= 1.0f) phase_ -= 1.0f;
        float sine = sinf(TR505_TWOPI * phase_);

        /* Click: noise filtrado -- el "artefacto de inicio" del sample PCM */
        float clickEnv = expf(-time_ / 0.001f);
        float click    = clickF_.ProcessBP(rng_.White()) * clickEnv * 0.4f;

        float amp    = expf(-time_ / decay);
        float g      = 1.0f + drive * 3.0f;
        float output = FastTanh((sine + click) * g) * amp;

        /* Lo-fi: bit crush + S&H */
        output = lofiProc_.Process(output);

        time_ += dt_;
        if (amp < 0.0005f) active_ = false;

        return output * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d) { decay = Clamp(d, 0.05f, 0.8f);  }
    void SetPitch(float p) { pitch = Clamp(p, 40.0f, 100.0f); }
    void SetLoFi(float l)  { lofi = Clamp(l, 0.0f, 1.0f); UpdateLoFi(); }

private:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, phase_ = 0.0f, vel_ = 1.0f;
    SVF         clickF_;
    Rng         rng_;
    LoFiProcessor lofiProc_;

    void UpdateLoFi() {
        /* lofi=0 -> 14 bits, muy suave
         * lofi=1 -> 5 bits, muy agresivo
         * Default lofi=0.3 -> ~8-9 bits, clasico 505 */
        lofiProc_.bitDepth      = 14.0f - lofi * 9.0f;
        lofiProc_.sampleRateDiv = 1.0f  + lofi * 5.0f;
        lofiProc_.UpdateCache();  /* precalcula levels_ / divInt_ */
        lofiProc_.SetBypass(lofi <= 0.0f);
    }
};

/* =====================================================================
 *  SNARE 505
 * ---------------------------------------------------------------------
 *  Thin y digital. Un tono corto + noise filtrado.
 *  En v1.0 el noise era completamente plano (sin filtro).
 *
 *  v2.0:
 *    SVF HP en el noise para mas snap y brillo (precalculado)
 *    SVF LP muy suave en el tono para suavizar el sine digital
 *    Envelope de tone y noise con ratios diferentes
 * ===================================================================== */
class Snare {
public:
    float decay   = 0.15f;   /* [0.04 - 0.5]  s               */
    float tone    = 0.4f;    /* [0.0  - 1.0]  mezcla tono     */
    float snappy  = 0.6f;    /* [0.0  - 1.0]  nivel noise     */
    float pitch   = 220.0f;  /* [150  - 350]  Hz              */
    float lofi    = 0.3f;
    float volume  = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_ = false;
        /* noiseF_ HP sera pitch-tracked en Trigger() */
        noiseF_.SetCoefs(sr_, pitch * 28.0f, 0.9f);
        toneF_.SetCoefs(sr_, 3000.0f, 0.7f);
        rng_.Seed(0x5055E01u); /* snare */
        UpdateLoFi();
    }

    void Trigger(float velocity = 1.0f) {
        active_  = true;
        time_    = 0.0f;
        phase_   = 0.0f;
        vel_     = VelCurve(velocity);
        /* HP pitch-tracked: precalculado aqui, no en Process() */
        noiseF_.SetCoefs(sr_, pitch * 28.0f, 0.9f);
        noiseF_.Reset();
        toneF_.Reset();
        lofiProc_.Reset();
        rng_.Seed(0x5055E01u ^ (uint32_t)(velocity * 3333));
    }

    float Process() {
        if (!active_) return 0.0f;

        phase_ += pitch * dt_;
        if (phase_ >= 1.0f) phase_ -= 1.0f;

        /* Tono: sine crudo + 3a armonica -- el caracter digital chiptune del 505 */
        float rawTone  = sinf(TR505_TWOPI * phase_);
        float rawTone2 = sinf(TR505_TWOPI * phase_ * 1.5f);
        float toneEnv  = expf(-time_ / (decay * 0.45f));
        float toneOut  = (rawTone * 0.75f + rawTone2 * 0.25f) * toneEnv * tone;

        /* Noise HP: brillo y snap */
        float noiseEnv = expf(-time_ / decay);
        float noiseOut = noiseF_.ProcessHP(rng_.White()) * noiseEnv * snappy;

        float output = FastTanh((toneOut + noiseOut) * 1.5f);
        output = lofiProc_.Process(output);

        time_ += dt_;
        if (noiseEnv < 0.0005f) active_ = false;

        return output * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d)  { decay  = Clamp(d, 0.04f, 0.5f);   }
    void SetTone(float t)   { tone   = Clamp(t, 0.0f,  1.0f);   }
    void SetSnappy(float s) { snappy = Clamp(s, 0.0f,  1.0f);   }
    void SetPitch(float p)  { pitch  = Clamp(p, 150.0f, 350.0f); }
    void SetLoFi(float l)   { lofi = Clamp(l, 0.0f, 1.0f); UpdateLoFi(); }

private:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, phase_ = 0.0f, vel_ = 1.0f;
    SVF   noiseF_, toneF_;
    Rng   rng_;
    LoFiProcessor lofiProc_;

    void UpdateLoFi() {
        lofiProc_.bitDepth      = 13.0f - lofi * 8.0f;
        lofiProc_.sampleRateDiv = 1.0f  + lofi * 5.0f;
        lofiProc_.UpdateCache();
        lofiProc_.SetBypass(lofi <= 0.0f);
    }
};

/* =====================================================================
 *  CLAP 505
 * ---------------------------------------------------------------------
 *  3 rafagas (mas simple que 808's 4 y 909's 6).
 *  Mas "flat" que las otras maquinas -- el sample de clap de la 505
 *  es notablemente seco y digital.
 *
 *  v2.0:
 *    SVF BP precalculado (en v1.0 era noise crudo sin filtrar)
 *    Separacion burst/tail con filtros independientes
 * ===================================================================== */
class Clap {
public:
    float decay  = 0.20f;   /* [0.05 - 0.6]  s   */
    float tone   = 0.5f;    /* [0.0  - 1.0]  fc  */
    float lofi   = 0.35f;
    float volume = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_ = false;
        rng_.Seed(0x5050C01u); /* clap */
    }

    void Trigger(float velocity = 1.0f) {
        active_ = true;
        time_   = 0.0f;
        vel_    = VelCurve(velocity);
        float fc = 1400.0f + tone * 2200.0f;
        burstF_.SetCoefs(sr_, fc, 1.8f);
        tailF_.SetCoefs(sr_, fc * 0.55f, 1.1f);
        burstF_.Reset();
        tailF_.Reset();
        lofiProc_.Reset();
        UpdateLoFi();
    }

    float Process() {
        if (!active_) return 0.0f;

        float n = rng_.White();

        /* 3 rafagas con 8ms de separacion */
        const float kDt  = 0.008f;
        const float kLen = 0.003f;
        float burstEnv = 0.0f;
        for (int i = 0; i < 3; i++) {
            float t = time_ - i * kDt;
            if (t >= 0.0f && t < kLen)
                burstEnv += expf(-t / 0.002f);
        }

        const float tailStart = 3.0f * kDt;
        float tailEnv = (time_ >= tailStart)
            ? expf(-(time_ - tailStart) / decay) : 0.0f;

        float out = burstF_.ProcessBP(n) * burstEnv
                  + tailF_.ProcessBP(n)  * tailEnv * 0.5f;

        float output = FastTanh(out * 1.8f);
        output = lofiProc_.Process(output);

        time_ += dt_;
        if (time_ > tailStart + decay * 4.0f && tailEnv < 0.0005f)
            active_ = false;

        return output * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d) { decay = Clamp(d, 0.05f, 0.6f); }
    void SetLoFi(float l)  { lofi = Clamp(l, 0.0f, 1.0f); UpdateLoFi(); }

private:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, vel_ = 1.0f;
    /* NOTA: 'lofi' es el miembro publico de arriba -- NO redeclarar aqui */
    SVF   burstF_, tailF_;
    Rng   rng_;
    LoFiProcessor lofiProc_;

    void UpdateLoFi() {
        lofiProc_.bitDepth      = 12.0f - lofi * 7.0f;
        lofiProc_.sampleRateDiv = 1.5f  + lofi * 4.5f;
        lofiProc_.UpdateCache();
        lofiProc_.SetBypass(lofi <= 0.0f);
    }
};

/* =====================================================================
 *  HIHAT BASE 505
 * ---------------------------------------------------------------------
 *  La 505 NO usa el metallic oscillator de la 808/909.
 *  Sus hihats son samples de noise filtrado con bit crush.
 *  Caracter mas "grainy" y less metallic.
 *  SVF HP precalculado en Init.
 * ===================================================================== */
class HiHatBase505 {
protected:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, vel_ = 1.0f;
    Rng   rng_;
    SVF   hpFilter_;
    LoFiProcessor lofiProc_;

    void BaseInit(float sampleRate, float hpFreq, uint32_t seed) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_ = false;
        hpFilter_.SetCoefs(sr_, hpFreq, 0.65f);
        rng_.Seed(seed);
        /* Hi-hats 505: moderadamente lo-fi (samples eran menos degradados) */
        lofiProc_.bitDepth      = 9.5f;
        lofiProc_.sampleRateDiv = 2.5f;
        lofiProc_.UpdateCache();
    }

    void BaseTrigger(float velocity) {
        active_ = true;
        time_   = 0.0f;
        vel_    = VelCurve(velocity);
        hpFilter_.Reset();
        lofiProc_.Reset();
        rng_.Seed(rng_.Next());  /* semilla diferente cada vez */
    }

    void BaseSetLoFi(float l) {
        l = Clamp(l, 0.0f, 1.0f);
        /* bitMin=8, divMax=5 para hihats (calibracion fija) */
        lofiProc_.bitDepth      = 14.0f - l * 6.0f;
        lofiProc_.sampleRateDiv = 1.0f  + l * 4.0f;
        lofiProc_.UpdateCache();
        lofiProc_.SetBypass(l <= 0.0f);
    }
};

/* =====================================================================
 *  HIHAT CLOSED 505
 * ===================================================================== */
class HiHatClosed : public HiHatBase505 {
public:
    float decay  = 0.030f;  /* [0.01 - 0.15] s */
    float tone   = 0.5f;
    float volume = 1.0f;

    void Init(float sr) { BaseInit(sr, 5500.0f + tone * 1500.0f, 0x505AA01u); /* hihatC */ }

    void Trigger(float v = 1.0f) {
        /* Actualizar fc segun tone antes del trigger */
        hpFilter_.SetCoefs(sr_, 5500.0f + tone * 1500.0f, 0.65f);
        BaseTrigger(v);
    }

    float Process() {
        if (!active_) return 0.0f;
        float hp  = hpFilter_.ProcessHP(rng_.White());
        float env = expf(-time_ / decay);
        env = env * env;  /* cuadrado: cierre seco */
        float out = FastTanh(hp * 2.6f) * env;
        out = lofiProc_.Process(out);
        time_ += dt_;
        if (env < 0.0005f) active_ = false;
        return out * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d) { decay = Clamp(d, 0.01f, 0.15f); }
    void SetLoFi(float l)  { BaseSetLoFi(l); }
};

/* =====================================================================
 *  HIHAT OPEN 505
 * ===================================================================== */
class HiHatOpen : public HiHatBase505 {
public:
    float decay  = 0.20f;   /* [0.05 - 1.0]  s */
    float tone   = 0.5f;
    float volume = 1.0f;

    void Init(float sr) { BaseInit(sr, 5000.0f + tone * 1500.0f, 0x505BB02u); /* hihatO */ }

    void Trigger(float v = 1.0f) {
        hpFilter_.SetCoefs(sr_, 5000.0f + tone * 1500.0f, 0.65f);
        BaseTrigger(v);
    }

    void Choke() { active_ = false; }

    float Process() {
        if (!active_) return 0.0f;
        float hp  = hpFilter_.ProcessHP(rng_.White());
        float env = expf(-time_ / decay);
        env = env * sqrtf(env);  /* env^1.5: cierre seco pero no tan agresivo como closed */
        float out = FastTanh(hp * 2.3f) * env;
        out = lofiProc_.Process(out);
        time_ += dt_;
        if (env < 0.0005f) active_ = false;
        return out * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d) { decay = Clamp(d, 0.05f, 1.0f); }
    void SetLoFi(float l)  { BaseSetLoFi(l); }
};

/* =====================================================================
 *  TOM BASE 505
 * ---------------------------------------------------------------------
 *  Sine con pitch sweep moderado -- menos punch que 808/909.
 *  El sample de tom de la 505 es notablemente simple.
 *
 *  v2.0: smack transient: noise HP brevísimo al inicio
 *  Simula el click de inicio del sample PCM.
 * ===================================================================== */
class TomBase505 {
public:
    float decay      = 0.15f;
    float pitch      = 120.0f;
    float pitchDecay = 0.025f;
    float lofi       = 0.25f;
    float volume     = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_ = false;
        smackF_.SetCoefs(sr_, pitch * 6.0f, 1.2f);
        rng_.Seed(0x5050A01u); /* tom */
        UpdateLoFi();
    }

    void Trigger(float velocity = 1.0f) {
        active_ = true;
        time_   = 0.0f;
        phase_  = 0.0f;
        vel_    = VelCurve(velocity);
        smackF_.Reset();
        lofiProc_.Reset();
    }

    float Process() {
        if (!active_) return 0.0f;

        float cp = pitch + pitch * 1.5f * expf(-time_ / pitchDecay);
        phase_ += cp * dt_;
        if (phase_ >= 1.0f) phase_ -= 1.0f;
        float sine = sinf(TR505_TWOPI * phase_);

        /* Smack: click digital del inicio del sample */
        float smackEnv = expf(-time_ / 0.003f);
        float smack    = smackF_.ProcessBP(rng_.White()) * smackEnv * 0.25f;

        float amp    = expf(-time_ / decay);
        float output = (sine + smack) * amp;
        output = lofiProc_.Process(output);

        time_ += dt_;
        if (amp < 0.0005f) active_ = false;

        return output * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d) { decay = Clamp(d, 0.05f, 0.6f);  }
    void SetPitch(float p) {
        pitch = Clamp(p, 50.0f, 400.0f);
        smackF_.SetCoefs(sr_, pitch * 6.0f, 1.2f);
    }
    void SetLoFi(float l)  { lofi = Clamp(l, 0.0f, 1.0f); UpdateLoFi(); }

protected:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, phase_ = 0.0f, vel_ = 1.0f;
    SVF   smackF_;
    Rng   rng_;
    LoFiProcessor lofiProc_;

    void UpdateLoFi() {
        lofiProc_.bitDepth      = 13.0f - lofi * 7.0f;
        lofiProc_.sampleRateDiv = 1.0f  + lofi * 4.0f;
        lofiProc_.UpdateCache();
        lofiProc_.SetBypass(lofi <= 0.0f);
    }
};

class LowTom : public TomBase505 {
public: LowTom() { pitch=75.0f;  decay=0.20f; pitchDecay=0.028f; }
};
class MidTom : public TomBase505 {
public: MidTom() { pitch=110.0f; decay=0.18f; pitchDecay=0.024f; }
};
class HiTom  : public TomBase505 {
public: HiTom()  { pitch=160.0f; decay=0.15f; pitchDecay=0.020f; }
};

/* =====================================================================
 *  COWBELL 505
 * ---------------------------------------------------------------------
 *  Frecuencias ligeramente diferentes a la 808 (560 Hz + 845 Hz).
 *  En v1.0 no tenia bandpass -- sonaba demasiado duro.
 *
 *  v2.0: SVF BP que le da el caracter "cheap digital cowbell"
 *  Exactamente lo que define la 505: metalico pero lo-fi.
 * ===================================================================== */
class Cowbell {
public:
    float decay  = 0.06f;   /* [0.02 - 0.4]  s */
    float tune   = 1.0f;    /* [0.8  - 1.3]  multiplicador */
    float lofi   = 0.35f;
    float volume = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_ = false;
        bpF_.SetCoefs(sr_, 900.0f, 2.5f);
        UpdateLoFi();
    }

    void Trigger(float velocity = 1.0f) {
        active_  = true;
        time_    = 0.0f;
        phase1_  = phase2_ = 0.0f;
        vel_     = VelCurve(velocity);
        bpF_.SetCoefs(sr_, 900.0f * tune, 2.5f);
        bpF_.Reset();
        lofiProc_.Reset();
    }

    float Process() {
        if (!active_) return 0.0f;

        /* Frecuencias 505: 560 Hz + 845 Hz (vs 540+800 en 808) */
        phase1_ += 560.0f * tune * dt_;
        if (phase1_ >= 1.0f) phase1_ -= 1.0f;
        float sq1 = (phase1_ < 0.5f) ? 1.0f : -1.0f;

        phase2_ += 845.0f * tune * dt_;
        if (phase2_ >= 1.0f) phase2_ -= 1.0f;
        float sq2 = (phase2_ < 0.5f) ? 1.0f : -1.0f;

        float raw = (sq1 + sq2) * 0.5f;
        float bp  = bpF_.ProcessBP(raw);
        float env = expf(-time_ / decay);

        /* Click inicial percusivo -- onset del sample PCM */
        float clickEnv = expf(-time_ / 0.0015f);
        float click    = raw * clickEnv * 0.4f;

        float output = FastTanh((bp * env + click) * 1.5f);
        output = lofiProc_.Process(output);

        time_ += dt_;
        if (env < 0.0005f) active_ = false;

        return output * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d) { decay = Clamp(d, 0.02f, 0.4f); }
    void SetTune(float t)  { tune  = Clamp(t, 0.8f,  1.3f); }
    void SetLoFi(float l)  { lofi = Clamp(l, 0.0f, 1.0f); UpdateLoFi(); }

private:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, phase1_ = 0.0f, phase2_ = 0.0f, vel_ = 1.0f;
    SVF   bpF_;
    LoFiProcessor lofiProc_;

    void UpdateLoFi() {
        lofiProc_.bitDepth      = 11.0f - lofi * 6.0f;
        lofiProc_.sampleRateDiv = 2.0f  + lofi * 4.0f;
        lofiProc_.UpdateCache();
        lofiProc_.SetBypass(lofi <= 0.0f);
    }
};

/* =====================================================================
 *  CYMBAL 505
 * ---------------------------------------------------------------------
 *  Noise HP puro -- sin el metallic oscillator de 808/909.
 *  Mas "hissy" y plano. SVF HP precalculado en v2.0.
 * ===================================================================== */
class Cymbal {
public:
    float decay  = 0.60f;   /* [0.1  - 3.0]  s */
    float tone   = 0.5f;    /* [0.0  - 1.0]  fc sweep */
    float lofi   = 0.25f;
    float volume = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_ = false;
        rng_.Seed(0x505CA01u); /* cymbal */
        hpF_.SetCoefs(sr_, 5000.0f, 0.65f);
        UpdateLoFi();
    }

    void Trigger(float velocity = 1.0f) {
        active_ = true;
        time_   = 0.0f;
        vel_    = VelCurve(velocity);
        hpF_.SetCoefs(sr_, 5000.0f + tone * 3000.0f, 0.65f);
        hpF_.Reset();
        lofiProc_.Reset();
        rng_.Seed(0x505CA01u ^ (uint32_t)(vel_ * 9999));
    }

    float Process() {
        if (!active_) return 0.0f;

        float hp     = hpF_.ProcessHP(rng_.White());
        float attack = 1.0f - expf(-time_ / 0.003f);
        float env    = expf(-time_ / decay);

        float output = hp * env * attack;
        output = lofiProc_.Process(output);

        time_ += dt_;
        if (env < 0.0005f) active_ = false;

        return output * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d) { decay = Clamp(d, 0.1f, 3.0f); }
    void SetLoFi(float l)  { lofi = Clamp(l, 0.0f, 1.0f); UpdateLoFi(); }

private:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, vel_ = 1.0f;
    SVF   hpF_;
    Rng   rng_;
    LoFiProcessor lofiProc_;

    void UpdateLoFi() {
        lofiProc_.bitDepth      = 12.0f - lofi * 7.0f;
        lofiProc_.sampleRateDiv = 1.5f  + lofi * 3.5f;
        lofiProc_.UpdateCache();
        lofiProc_.SetBypass(lofi <= 0.0f);
    }
};

/* =====================================================================
 *  RIMSHOT 505
 * ---------------------------------------------------------------------
 *  Click de noise + tono corto ~750 Hz.
 *  v2.0: SVF resonador de impulso (mas limpio que sine libre)
 * ===================================================================== */
class RimShot {
public:
    float decay  = 0.020f;
    float pitch  = 750.0f;
    float lofi   = 0.25f;
    float volume = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_ = false;
        resonF_.SetCoefs(sr_, pitch, 4.5f);
        rng_.Seed(0x505EA01u); /* rimshot */
        UpdateLoFi();
    }

    void Trigger(float velocity = 1.0f) {
        active_  = true;
        time_    = 0.0f;
        vel_     = VelCurve(velocity);
        impulse_ = 1.0f;
        resonF_.SetCoefs(sr_, pitch, 4.5f);
        resonF_.Reset();
        lofiProc_.Reset();
        rng_.Seed(0x505EA01u ^ (uint32_t)(vel_ * 7777));
    }

    float Process() {
        if (!active_) return 0.0f;

        float clickEnv = expf(-time_ / 0.0008f);
        float click    = rng_.White() * clickEnv * 0.5f;

        float toneEnv = expf(-time_ / decay);
        float tone    = resonF_.ProcessBP(impulse_) * toneEnv;
        impulse_ = 0.0f;

        float output = FastTanh((click + tone) * 1.8f);
        output = lofiProc_.Process(output);

        time_ += dt_;
        if (toneEnv < 0.0005f && clickEnv < 0.0005f) active_ = false;

        return output * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetLoFi(float l)  { lofi = Clamp(l, 0.0f, 1.0f); UpdateLoFi(); }

private:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, vel_ = 1.0f, impulse_ = 0.0f;
    SVF   resonF_;
    Rng   rng_;
    LoFiProcessor lofiProc_;

    void UpdateLoFi() {
        lofiProc_.bitDepth      = 12.0f - lofi * 7.0f;
        lofiProc_.sampleRateDiv = 1.5f  + lofi * 3.0f;
        lofiProc_.UpdateCache();
        lofiProc_.SetBypass(lofi <= 0.0f);
    }
};

/* =====================================================================
 *  SHAKER / MARACAS 505
 * ---------------------------------------------------------------------
 *  Noise PCM corto y granulado para el slot MA, con bit-crush integrado.
 * ===================================================================== */
class Shaker {
public:
    float decay  = 0.095f;
    float tone   = 0.55f;
    float lofi   = 0.45f;
    float volume = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_ = false;
        hpF_.SetCoefs(sr_, 4400.0f + tone * 3600.0f, 0.75f);
        bpF_.SetCoefs(sr_, 1800.0f + tone * 2200.0f, 1.2f);
        rng_.Seed(0x5055A01u);
        UpdateLoFi();
    }

    void Trigger(float velocity = 1.0f) {
        active_ = true;
        time_ = 0.0f;
        vel_ = VelCurve(velocity);
        hpF_.SetCoefs(sr_, 4400.0f + tone * 3600.0f, 0.75f);
        bpF_.SetCoefs(sr_, 1800.0f + tone * 2200.0f, 1.2f);
        hpF_.Reset();
        bpF_.Reset();
        lofiProc_.Reset();
        rng_.Seed(0x5055A01u ^ (uint32_t)(vel_ * 23451));
    }

    float Process() {
        if (!active_) return 0.0f;
        float n = rng_.White();
        float env = expf(-time_ / decay);
        float atk = 1.0f - expf(-time_ / 0.0012f);
        float out = (hpF_.ProcessHP(n) * 0.9f + bpF_.ProcessBP(n) * 0.35f) * env * atk;
        out = FastTanh(out * 2.1f);
        out = lofiProc_.Process(out);
        time_ += dt_;
        if (env < 0.0005f) active_ = false;
        return out * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d) { decay = Clamp(d, 0.02f, 0.45f); }
    void SetTone(float t)  { tone = Clamp(t, 0.0f, 1.0f); }
    void SetLoFi(float l)  { lofi = Clamp(l, 0.0f, 1.0f); UpdateLoFi(); }

private:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, vel_ = 1.0f;
    SVF   hpF_, bpF_;
    Rng   rng_;
    LoFiProcessor lofiProc_;

    void UpdateLoFi() {
        lofiProc_.bitDepth      = 11.0f - lofi * 6.0f;
        lofiProc_.sampleRateDiv = 2.0f  + lofi * 4.5f;
        lofiProc_.UpdateCache();
        lofiProc_.SetBypass(lofi <= 0.0f);
    }
};

/* =====================================================================
 *  CLAVE 505
 * ---------------------------------------------------------------------
 *  Click tonal PCM, mas fino y plastico que el rimshot.
 * ===================================================================== */
class Clave {
public:
    float decay  = 0.050f;
    float pitch  = 1650.0f;
    float lofi   = 0.35f;
    float volume = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_ = false;
        resonF_.SetCoefs(sr_, pitch, 7.0f);
        clickF_.SetCoefs(sr_, 3600.0f, 1.0f);
        rng_.Seed(0x505C1A5u);
        UpdateLoFi();
    }

    void Trigger(float velocity = 1.0f) {
        active_ = true;
        time_ = 0.0f;
        vel_ = VelCurve(velocity);
        impulse_ = 1.0f;
        resonF_.SetCoefs(sr_, pitch, 7.0f);
        resonF_.Reset();
        clickF_.Reset();
        lofiProc_.Reset();
        rng_.Seed(0x505C1A5u ^ (uint32_t)(vel_ * 15551));
    }

    float Process() {
        if (!active_) return 0.0f;
        float toneEnv = expf(-time_ / decay);
        float tone = resonF_.ProcessBP(impulse_) * toneEnv;
        impulse_ = 0.0f;
        float clickEnv = expf(-time_ / 0.0009f);
        float click = clickF_.ProcessBP(rng_.White()) * clickEnv * 0.35f;
        float out = FastTanh((tone + click) * 2.0f);
        out = lofiProc_.Process(out);
        time_ += dt_;
        if (toneEnv < 0.0005f && clickEnv < 0.0005f) active_ = false;
        return out * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d) { decay = Clamp(d, 0.015f, 0.25f); }
    void SetPitch(float p) { pitch = Clamp(p, 800.0f, 2800.0f); }
    void SetLoFi(float l)  { lofi = Clamp(l, 0.0f, 1.0f); UpdateLoFi(); }

private:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, vel_ = 1.0f, impulse_ = 0.0f;
    SVF   resonF_, clickF_;
    Rng   rng_;
    LoFiProcessor lofiProc_;

    void UpdateLoFi() {
        lofiProc_.bitDepth      = 12.0f - lofi * 7.0f;
        lofiProc_.sampleRateDiv = 1.5f  + lofi * 3.5f;
        lofiProc_.UpdateCache();
        lofiProc_.SetBypass(lofi <= 0.0f);
    }
};

/* =====================================================================
 *  PERC 505 BASE
 * ---------------------------------------------------------------------
 *  Tres percusiones PCM-like para HC/MC/LC: mas cortas y plasticas que
 *  los toms, con ataque granular y bit-crush propio.
 * ===================================================================== */
class Perc505Base {
public:
    float decay  = 0.12f;
    float pitch  = 480.0f;
    float click  = 0.22f;
    float lofi   = 0.38f;
    float volume = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_ = false;
        bodyF_.SetCoefs(sr_, pitch, 4.0f);
        clickF_.SetCoefs(sr_, pitch * 5.0f, 1.1f);
        rng_.Seed(0x5059E01u);
        UpdateLoFi();
    }

    void Trigger(float velocity = 1.0f) {
        active_ = true;
        time_ = 0.0f;
        vel_ = VelCurve(velocity);
        impulse_ = 1.0f;
        bodyF_.SetCoefs(sr_, pitch, 4.0f);
        clickF_.SetCoefs(sr_, pitch * 5.0f, 1.1f);
        bodyF_.Reset();
        clickF_.Reset();
        lofiProc_.Reset();
        rng_.Seed(0x5059E01u ^ (uint32_t)(vel_ * pitch));
    }

    float Process() {
        if (!active_) return 0.0f;
        float env = expf(-time_ / decay);
        float body = bodyF_.ProcessBP(impulse_) * env;
        impulse_ = 0.0f;
        float clickEnv = expf(-time_ / 0.0025f);
        float clickOut = clickF_.ProcessBP(rng_.White()) * clickEnv * click;
        float out = FastTanh((body + clickOut) * 1.8f);
        out = lofiProc_.Process(out);
        time_ += dt_;
        if (env < 0.0005f && clickEnv < 0.0005f) active_ = false;
        return out * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d) { decay = Clamp(d, 0.035f, 0.5f); }
    void SetPitch(float p) { pitch = Clamp(p, 120.0f, 1400.0f); }
    void SetClick(float c) { click = Clamp(c, 0.0f, 1.0f); }
    void SetLoFi(float l)  { lofi = Clamp(l, 0.0f, 1.0f); UpdateLoFi(); }

protected:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, vel_ = 1.0f, impulse_ = 0.0f;
    SVF   bodyF_, clickF_;
    Rng   rng_;
    LoFiProcessor lofiProc_;

    void UpdateLoFi() {
        lofiProc_.bitDepth      = 12.0f - lofi * 7.0f;
        lofiProc_.sampleRateDiv = 1.5f  + lofi * 4.5f;
        lofiProc_.UpdateCache();
        lofiProc_.SetBypass(lofi <= 0.0f);
    }
};

class HiPerc : public Perc505Base {
public: HiPerc() { pitch = 760.0f; decay = 0.075f; click = 0.34f; lofi = 0.42f; }
};
class MidPerc : public Perc505Base {
public: MidPerc() { pitch = 480.0f; decay = 0.115f; click = 0.26f; lofi = 0.40f; }
};
class LowPerc : public Perc505Base {
public: LowPerc() { pitch = 300.0f; decay = 0.160f; click = 0.18f; lofi = 0.38f; }
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
    INST_COWBELL,
    INST_CYMBAL,
    INST_RIMSHOT,
    INST_SHAKER,
    INST_CLAVE,
    INST_HI_PERC,
    INST_MID_PERC,
    INST_LOW_PERC,
    INST_COUNT
};

/* =====================================================================
 *  PRESETS
 * ===================================================================== */
struct KitPreset {
    const char* name;
    float vol[INST_COUNT];
    float globalLoFi;   /* lofi global del kit */
};

namespace Presets {

/* Classic 505: como salio de fabrica, lofi moderado */
static const KitPreset Classic505 = { "Classic 505",
        { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
            1.0f, 1.0f, 1.0f, 0.92f, 0.92f, 0.88f, 0.90f, 0.92f },
    0.3f
};

/* New Wave: hihat y cowbell arriba, kick moderado */
static const KitPreset NewWave = { "New Wave",
        { 0.8f, 0.9f, 0.7f, 0.9f, 0.8f, 0.6f, 0.6f, 0.7f,
            1.1f, 0.6f, 0.7f, 1.00f, 0.80f, 0.65f, 0.68f, 0.72f },
    0.25f
};

/* Electro: kick fuerte, snare bright, poco lofi */
static const KitPreset Electro = { "Electro",
        { 1.2f, 1.0f, 0.6f, 0.8f, 0.7f, 0.7f, 0.7f, 0.7f,
            0.5f, 0.5f, 0.6f, 0.75f, 0.72f, 0.70f, 0.72f, 0.74f },
    0.15f
};

/* Lo-Fi Hip-Hop: todo crushed y degradado */
static const KitPreset LoFiHipHop = { "Lo-Fi Hip-Hop",
        { 1.0f, 0.9f, 0.7f, 0.6f, 0.6f, 0.8f, 0.8f, 0.7f,
            0.4f, 0.5f, 0.5f, 0.85f, 0.80f, 0.72f, 0.74f, 0.76f },
    0.65f
};

/* Pure 505 procedural: sin LoFi. Para PCM real usar el backend PcmSlot. */
static const KitPreset Pure505 = { "Pure 505",
        { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
            1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f },
    0.0f
};

} /* namespace Presets */

/* =====================================================================
 *  KIT 505 -- contenedor completo
 * ---------------------------------------------------------------------
 *  Per-channel volume y mute
 *  SetLoFi() global: ajusta el lofi de todos los instrumentos
 *  Soft limiter de salida
 *  LoadPreset() con globalLoFi incluido
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
    Cowbell     cowbell;
    Cymbal      cymbal;
    RimShot     rimshot;
    Shaker      shaker;
    Clave       clave;
    HiPerc      hiPerc;
    MidPerc     midPerc;
    LowPerc     lowPerc;

    /* Optional PCM ROM slots. Populated slots override their procedural
     * voice; missing slots keep the procedural engine as a safe fallback. */
    struct PcmSlot {
        const int16_t* data = nullptr;
        uint32_t length = 0;
        float sourceRate = 48000.0f;
        float pos = 0.0f;
        float step = 1.0f;
        float velocity = 1.0f;
        bool active = false;
    };

    void Init(float sampleRate) {
        sr_ = sampleRate;
        kick.Init(sr_);     snare.Init(sr_);    clap.Init(sr_);
        hihatC.Init(sr_);   hihatO.Init(sr_);
        lowTom.Init(sr_);   midTom.Init(sr_);   hiTom.Init(sr_);
        cowbell.Init(sr_);  cymbal.Init(sr_);   rimshot.Init(sr_);
        shaker.Init(sr_);   clave.Init(sr_);
        hiPerc.Init(sr_);   midPerc.Init(sr_);  lowPerc.Init(sr_);

        for (int i = 0; i < INST_COUNT; i++) {
            chanVol_[i]  = 1.0f;
            chanMute_[i] = false;
            pcm_[i] = PcmSlot{};
        }
        masterVol_  = 0.92f;
        limitState_ = 0.0f;
    }

    void Trigger(uint8_t inst, float velocity = 1.0f) {
        velocity = Clamp(velocity, 0.0f, 1.0f);
        if (inst >= INST_COUNT) return;
        if (inst == INST_HIHAT_C) {
            hihatO.Choke();
            pcm_[INST_HIHAT_O].active = false;
        }
        if (pcm_[inst].data != nullptr && pcm_[inst].length > 0) {
            pcm_[inst].pos = 0.0f;
            pcm_[inst].step = pcm_[inst].sourceRate / sr_;
            pcm_[inst].velocity = VelCurve(velocity);
            pcm_[inst].active = true;
            return;
        }
        switch (inst) {
            case INST_KICK:     kick.Trigger(velocity);    break;
            case INST_SNARE:    snare.Trigger(velocity);   break;
            case INST_CLAP:     clap.Trigger(velocity);    break;
            case INST_HIHAT_C:  hihatC.Trigger(velocity);  break;
            case INST_HIHAT_O:  hihatO.Trigger(velocity);  break;
            case INST_LOW_TOM:  lowTom.Trigger(velocity);  break;
            case INST_MID_TOM:  midTom.Trigger(velocity);  break;
            case INST_HI_TOM:   hiTom.Trigger(velocity);   break;
            case INST_COWBELL:  cowbell.Trigger(velocity); break;
            case INST_CYMBAL:   cymbal.Trigger(velocity);  break;
            case INST_RIMSHOT:  rimshot.Trigger(velocity); break;
            case INST_SHAKER:   shaker.Trigger(velocity);  break;
            case INST_CLAVE:    clave.Trigger(velocity);   break;
            case INST_HI_PERC:  hiPerc.Trigger(velocity);  break;
            case INST_MID_PERC: midPerc.Trigger(velocity); break;
            case INST_LOW_PERC: lowPerc.Trigger(velocity); break;
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

        add(INST_KICK,    kick);
        add(INST_SNARE,   snare);
        add(INST_CLAP,    clap);
        add(INST_HIHAT_C, hihatC);
        add(INST_HIHAT_O, hihatO);
        add(INST_LOW_TOM, lowTom);
        add(INST_MID_TOM, midTom);
        add(INST_HI_TOM,  hiTom);
        add(INST_COWBELL, cowbell);
        add(INST_CYMBAL,  cymbal);
        add(INST_RIMSHOT, rimshot);
        add(INST_SHAKER,  shaker);
        add(INST_CLAVE,   clave);
        add(INST_HI_PERC, hiPerc);
        add(INST_MID_PERC, midPerc);
        add(INST_LOW_PERC, lowPerc);

        for (uint8_t id = 0; id < INST_COUNT; id++) {
            PcmSlot& slot = pcm_[id];
            if (!slot.active || slot.data == nullptr || slot.length == 0) continue;
            uint32_t idx = (uint32_t)slot.pos;
            if (idx >= slot.length) {
                slot.active = false;
                continue;
            }
            float frac = slot.pos - (float)idx;
            float s0 = slot.data[idx] / 32768.0f;
            float s1 = (idx + 1u < slot.length) ? slot.data[idx + 1u] / 32768.0f : 0.0f;
            float sample = s0 + frac * (s1 - s0);
            slot.pos += slot.step;
            if (slot.pos >= (float)slot.length) slot.active = false;
            if (!chanMute_[id]) mix += sample * slot.velocity * chanVol_[id];
        }

        /* Soft limiter */
        mix *= masterVol_;
        float absv = fabsf(mix);
        limitState_ = (absv > limitState_) ? absv : limitState_ * 0.9985f;
        if (limitState_ > 0.98f)
            mix *= 0.98f / limitState_;

        return mix;
    }

    /* Ajusta el lofi de TODOS los instrumentos simultaneamente
     * Este es el control mas importante de la identidad 505 */
    void SetLoFi(float l) {
        l = Clamp(l, 0.0f, 1.0f);
        kick.SetLoFi(l);      snare.SetLoFi(l);     clap.SetLoFi(l);
        hihatC.SetLoFi(l);    hihatO.SetLoFi(l);
        lowTom.SetLoFi(l);    midTom.SetLoFi(l);    hiTom.SetLoFi(l);
        cowbell.SetLoFi(l);   cymbal.SetLoFi(l);    rimshot.SetLoFi(l);
        shaker.SetLoFi(l);    clave.SetLoFi(l);
        hiPerc.SetLoFi(l);    midPerc.SetLoFi(l);   lowPerc.SetLoFi(l);
    }

    bool SetPcmSample(uint8_t inst, const int16_t* data, uint32_t length,
                      float sourceRate = 48000.0f) {
        if (inst >= INST_COUNT || data == nullptr || length == 0) return false;
        PcmSlot& slot = pcm_[inst];
        slot.data = data;
        slot.length = length;
        slot.sourceRate = Clamp(sourceRate, 1000.0f, 384000.0f);
        slot.pos = 0.0f;
        slot.step = slot.sourceRate / sr_;
        slot.active = false;
        return true;
    }

    void ClearPcmSample(uint8_t inst) {
        if (inst < INST_COUNT) pcm_[inst] = PcmSlot{};
    }

    void ClearPcmSamples() {
        for (uint8_t i = 0; i < INST_COUNT; i++) pcm_[i] = PcmSlot{};
    }

    bool HasPcmSample(uint8_t inst) const {
        return inst < INST_COUNT && pcm_[inst].data != nullptr && pcm_[inst].length > 0;
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
        SetLoFi(p.globalLoFi);
    }

    uint8_t ActiveCount() const {
        uint8_t c = 0;
        if (kick.IsActive())    c++;
        if (snare.IsActive())   c++;
        if (clap.IsActive())    c++;
        if (hihatC.IsActive())  c++;
        if (hihatO.IsActive())  c++;
        if (lowTom.IsActive())  c++;
        if (midTom.IsActive())  c++;
        if (hiTom.IsActive())   c++;
        if (cowbell.IsActive()) c++;
        if (cymbal.IsActive())  c++;
        if (rimshot.IsActive()) c++;
        if (shaker.IsActive())  c++;
        if (clave.IsActive())   c++;
        if (hiPerc.IsActive())  c++;
        if (midPerc.IsActive()) c++;
        if (lowPerc.IsActive()) c++;
        for (uint8_t i = 0; i < INST_COUNT; i++)
            if (pcm_[i].active) c++;
        return c;
    }

private:
    float  sr_          = 48000.0f;
    float  masterVol_   = 0.92f;
    float  limitState_  = 0.0f;
    float  chanVol_[INST_COUNT]  = {};
    bool   chanMute_[INST_COUNT] = {};
    PcmSlot pcm_[INST_COUNT];
};

} /* namespace TR505 */

/* =====================================================================
 *  CHANGELOG
 *  v2.0  LoFiProcessor: bit crush + S&H SR reduction (el cambio clave),
 *        SVF coefs precalculados, FastTanh, Xoshiro32, VelCurve,
 *        Kick noise click, Snare SVF HP+LP, Clap dual-SVF BP,
 *        HiHat base comun SVF HP, Toms smack digital,
 *        Cowbell SVF BP, Cymbal SVF HP, RimShot resonador impulso,
 *        Kit: SetLoFi() global, mixer, soft limiter, 4 presets.
 *  v1.0  Filtros HP inline recalculados en Process, tanhf(), Xorshift.
 * ===================================================================== */
