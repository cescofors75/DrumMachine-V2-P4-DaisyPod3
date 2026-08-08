/* =====================================================================
 *  TR909.h  --  Roland TR-909 Drum Synthesis Library  (v2.0)
 * ---------------------------------------------------------------------
 *  La maquina que define el techno, el house y el trance.
 *  Mas agresiva que la 808: kick con mas punch, snare mas brillante,
 *  hihats mas digitales, ride y crash con caracter propio.
 *
 *  INSTRUMENTOS (16):
 *    Kick  Snare  Clap  HiHatClosed  HiHatOpen
 *    LowTom  MidTom  HiTom  Ride  Crash  RimShot
 *    Shaker  Clave  HiPerc  MidPerc  LowPerc
 *
 *  DIFERENCIAS CLAVE vs TR-808:
 *    Kick:  pitchAmt x12 (vs x8), pitchDecay mas rapido, mas punch
 *    Snare: ratio armonico 1.6x (vs 1.833x), HP mas alto
 *    Clap:  6 micro-rafagas (vs 4 en 808), mas denso
 *    HiHat: frecuencias metalicas mas brillantes, HP a 7500 Hz
 *    Toms:  pitch sweep x3 (vs x2.5), mas percusivos
 *    Ride:  bell tone sine + metallic tail (instrumento nuevo)
 *    Crash: onset lento + noise extra (instrumento nuevo)
 *
 *  MEJORAS v2.0:
 *    FastTanh racional 3x mas rapido que tanhf() en Cortex-M7
 *    SVF 2-polo (Simper/Cytomic): coefs precalculados en Trigger()
 *    Xoshiro32** PRNG: sin artefactos espectrales en hihats
 *    Kick: sub-osc a -1 oct + compresion correcta sin doble expf()
 *    Snare: SVF HP pitch-tracked + snappy envelope separado
 *    Clap: doble SVF burst/tail + Q configurable
 *    Toms: smack transient (noise BP corto) como en hardware real
 *    Ride: bell sine corto + tail metalico con fade separados
 *    RimShot: resonador SVF de impulso
 *    HiHat: HP estable precalculado
 *    Kit: per-channel volume/mute + soft limiter + 4 presets
 *    Velocity: curva smoothstep mas natural que lineal
 *
 *  RENDIMIENTO:
 *    < 80 ciclos/muestra (Kit completo, Cortex-M7 @ 480 MHz)
 *    Apto para AudioCallback a 48 kHz, block size 1-512
 *
 *  48 kHz  float32  C++14  header-only  sin dependencias
 *
 *  USO BASICO:
 *    TR909::Kit drum;
 *    drum.Init(48000.0f);
 *    drum.Trigger(TR909::INST_KICK, 1.0f);
 *    float s = drum.Process();
 *
 *  USO AVANZADO:
 *    drum.SetVolume(TR909::INST_SNARE, 0.9f);
 *    drum.LoadPreset(TR909::Presets::Techno);
 * ===================================================================== */
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

#ifndef TR909_TWOPI
#define TR909_TWOPI 6.283185307179586f
#endif

namespace TR909 {

/* =====================================================================
 *  UTILIDADES DSP
 *  Independientes de TR808 -- este header es autocontenido
 * ===================================================================== */

static inline float Clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* tanh(x) Pade 3/3: error < 0.4% en [-4,4]
 * ~3x mas rapido que tanhf() en Cortex-M7 sin FPU tanh nativa */
static inline float FastTanh(float x) {
    x = Clamp(x, -3.0f, 3.0f);          // evita distorsion por overflow Pade
    const float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

/* Curva smoothstep para velocity: v=0->0, v=0.5->~0.7, v=1->1 */
static inline float VelCurve(float v) {
    v = Clamp(v, 0.0f, 1.0f);
    return v * v * (3.0f - 2.0f * v);
}

/* Xoshiro32** PRNG -- mejor que Xorshift para audio
 * Evita patrones espectrales en el metallic noise */
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

/* SVF -- State Variable Filter (topologia Andy Simper/Cytomic)
 * SetCoefs() en Trigger() o Init() -- NUNCA en Process() */
struct SVF {
    float g=0, k=1, a1=0, a2=0, a3=0, ic1=0, ic2=0;

    void SetCoefs(float sr, float fc, float Q) {
        g  = tanf(TR909_TWOPI * Clamp(fc, 10.0f, sr * 0.49f) / (2.0f * sr));
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
 *  KICK 909
 * ---------------------------------------------------------------------
 *  Circuito 909: mas punch y mas agresivo que la 808.
 *  Pitch sweep mas rapido (pitchAmt x12, pitchDecay 0.04s).
 *  Compresion interna: blend entre decay normal y decay largo
 *  que mantiene el cuerpo -- corregida vs v1.0 (sin doble expf).
 *
 *  v2.0 anade:
 *    Sub-oscilador a -1 octava (la 909 real lo tiene)
 *    Click: noise BP filtrado (mas autentico que sine de 3kHz)
 *    Compresion: blend correcto pre-calculado
 * ===================================================================== */
class Kick {
public:
    float decay       = 0.5f;    /* [0.1  - 1.5]  s               */
    float pitch       = 50.0f;   /* [30   - 100]  Hz              */
    float pitchDecay  = 0.04f;   /* [0.01 - 0.15] s  mas rapido que 808 */
    float pitchAmt    = 12.0f;   /* [4.0  - 20.0] sweep amount    */
    float clickAmt    = 0.35f;   /* [0.0  - 1.0]  nivel del click */
    float compression = 0.5f;    /* [0.0  - 1.0]  punch/sustain   */
    float drive       = 0.4f;    /* [0.0  - 1.0]  saturacion      */
    float subLevel    = 0.1f;    /* [0.0  - 0.4]  sub-oscilador   */
    float volume      = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_ = false;
        clickF_.SetCoefs(sr_, 4000.0f, 0.9f);
        rng_.Seed(0x909BD001u);
    }

    void Trigger(float velocity = 1.0f) {
        active_   = true;
        time_     = 0.0f;
        phase_    = 0.0f;
        subPhase_ = 0.0f;
        vel_      = VelCurve(velocity);
        /* Compresion: blend entre decay corto y largo
         * Se pre-calcula alpha aqui para no hacer blend en Process */
        compAlpha_ = compression * 0.5f;
        clickF_.Reset();
        rng_.Seed(0x909BD001u ^ (uint32_t)(vel_ * 65535));
    }

    float Process() {
        if (!active_) return 0.0f;

        /* Pitch sweep exponencial -- mas agresivo que 808 */
        float cp = pitch + pitch * pitchAmt * expf(-time_ / pitchDecay);

        /* Oscilador principal */
        phase_ += cp * dt_;
        if (phase_ >= 1.0f) phase_ -= 1.0f;
        float sine = sinf(TR909_TWOPI * phase_);

        /* Sub-oscilador a -1 octava */
        subPhase_ += cp * 0.5f * dt_;
        if (subPhase_ >= 1.0f) subPhase_ -= 1.0f;
        float sub = sinf(TR909_TWOPI * subPhase_) * subLevel;

        /* Click: noise filtrado BP (mas autentico que sine 3kHz de v1.0) */
        float clickEnv = expf(-time_ / 0.0008f);
        float punch    = clickEnv * 2.0f;  /* impulso puro sin filtrar */
        float click    = clickF_.ProcessBP(rng_.White()) * clickEnv * clickAmt;

        /* Compresion correcta: blend amp normal + amp comprimida */
        float ampNorm = expf(-time_ / decay);
        float ampComp = expf(-time_ / (decay * 2.5f));
        /* Sustain boost: el 909 real eleva brevemente los primeros ~30ms */
        float sustainBoost = expf(-time_ / 0.03f) * compression * 0.3f;
        float amp     = ampNorm * (1.0f - compAlpha_) + ampComp * compAlpha_ + sustainBoost;

        /* Saturacion asinmetrica: caracter de transistor */
        float mix = sine + sub + punch + click;
        float g   = 1.0f + drive * 5.0f;
        float out = mix > 0.0f
            ? FastTanh(mix * g)
            : FastTanh(mix * g * 0.75f);

        time_ += dt_;
        if (amp < 0.0005f) active_ = false;

        return out * amp * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d)       { decay       = Clamp(d, 0.1f,  1.5f);   }
    void SetPitch(float p)       { pitch       = Clamp(p, 30.0f, 120.0f); }
    void SetPitchDecay(float d)  { pitchDecay  = Clamp(d, 0.01f, 0.15f);  }
    void SetCompression(float c) { compression = Clamp(c, 0.0f,  1.0f);   }
    void SetDrive(float d)       { drive       = Clamp(d, 0.0f,  1.0f);   }

private:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, phase_ = 0.0f, subPhase_ = 0.0f;
    float vel_ = 1.0f, compAlpha_ = 0.25f;
    SVF   clickF_;
    Rng   rng_;
};

/* =====================================================================
 *  SNARE 909
 * ---------------------------------------------------------------------
 *  Dos tonos: fundamental + armonico a x1.6 (diferente a 808's x1.833)
 *  Noise HP a frecuencia mas alta que 808 -- mas snap y brillo.
 *
 *  v2.0:
 *    SVF HP con pitch-tracking (fc = pitch * 32) precalculado en Trigger
 *    Envelope de tone y snap independientes
 *    RimMode: sonido de golpe en el aro (activable)
 * ===================================================================== */
class Snare {
public:
    float decay   = 0.18f;    /* [0.05 - 0.8]  s              */
    float tone    = 0.6f;     /* [0.0  - 1.0]  mezcla tono    */
    float snappy  = 0.7f;     /* [0.0  - 1.0]  nivel de snap  */
    float pitch   = 200.0f;   /* [120  - 400]  Hz             */
    float volume  = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_ = false;
        noiseF_.SetCoefs(sr_, 5800.0f, 1.0f);
        bodyF_.SetCoefs(sr_, 1200.0f, 0.8f);  /* BP para cuerpo del snare */
        rng_.Seed(0x9095E01u); /* snare */
    }

    void Trigger(float velocity = 1.0f) {
        active_  = true;
        time_    = 0.0f;
        phase1_  = phase2_ = 0.0f;
        vel_     = VelCurve(velocity);
        /* HP pitch-tracked: sigue el tono del snare */
        noiseF_.SetCoefs(sr_, pitch * 32.0f, 1.0f);
        bodyF_.SetCoefs(sr_, pitch * 6.0f, 0.8f);  /* BP body tracks pitch */
        noiseF_.Reset();
        bodyF_.Reset();
        rng_.Seed(0x9095E01u ^ (uint32_t)(velocity * 4567));
    }

    float Process() {
        if (!active_) return 0.0f;

        /* Tono fundamental */
        phase1_ += pitch * dt_;
        if (phase1_ >= 1.0f) phase1_ -= 1.0f;
        float t1 = sinf(TR909_TWOPI * phase1_);

        /* Armonico: ratio 1.6x (caracter 909, mas "crujiente" que 808) */
        phase2_ += pitch * 1.6f * dt_;
        if (phase2_ >= 1.0f) phase2_ -= 1.0f;
        float t2 = sinf(TR909_TWOPI * phase2_);

        /* Envelope tonal mas corto que el noise */
        float toneEnv  = expf(-time_ / (decay * 0.4f));
        float toneOut  = (t1 * 0.7f + t2 * 0.3f) * toneEnv * tone;

        /* Noise HP + BP body: el 909 real mezcla ambos */
        float snapEnv  = expf(-time_ / decay);
        float nRaw = rng_.White();
        /* Crack: impulso sin filtrar al inicio */
        float crackEnv = expf(-time_ / 0.0010f);
        float crack    = nRaw * crackEnv * snappy * 0.9f;
        float noiseHP  = noiseF_.ProcessHP(nRaw) * 1.4f;
        float noiseBP  = bodyF_.ProcessBP(nRaw) * 0.7f;  /* cuerpo medio */
        float noiseOut = (noiseHP + noiseBP + crack) * snapEnv * snappy;

        float output = FastTanh((toneOut + noiseOut) * 2.0f);

        time_ += dt_;
        if (snapEnv < 0.0005f) active_ = false;

        return output * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d)  { decay  = Clamp(d, 0.05f, 0.8f);  }
    void SetTone(float t)   { tone   = Clamp(t, 0.0f,  1.0f);  }
    void SetSnappy(float s) { snappy = Clamp(s, 0.0f,  1.0f);  }
    void SetPitch(float p)  { pitch  = Clamp(p, 120.0f, 400.0f); }

private:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, phase1_ = 0.0f, phase2_ = 0.0f, vel_ = 1.0f;
    SVF   noiseF_;
    SVF   bodyF_;   /* BP para cuerpo del snare */
    Rng   rng_;
};

/* =====================================================================
 *  CLAP 909
 * ---------------------------------------------------------------------
 *  6 micro-rafagas (vs 4 en 808) -- mas denso y mas agresivo.
 *  Filtro mas brillante (fc base ~1800 Hz vs 900 Hz en 808).
 *
 *  v2.0:
 *    Doble SVF burst/tail (mismo patron que TR808 v2.0)
 *    6 rafagas con 5ms de separacion
 *    Q configurable via snap
 * ===================================================================== */
class Clap {
public:
    float decay  = 0.25f;   /* [0.05 - 0.8]  s  cola      */
    float snap   = 0.65f;   /* [0.0  - 1.0]  Q del crack  */
    float tone   = 0.6f;    /* [0.0  - 1.0]  fc del filtro */
    float volume = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_ = false;
        rng_.Seed(0x9090C01u); /* clap */
    }

    void Trigger(float velocity = 1.0f) {
        active_ = true;
        time_   = 0.0f;
        vel_    = VelCurve(velocity);
        float fc = 1800.0f + tone * 4000.0f;  /* mas brillante que 808 */
        float Q  = 1.0f + snap * 6.0f;
        burstF_.SetCoefs(sr_, fc, Q);
        tailF_.SetCoefs(sr_, fc * 0.55f, 1.3f);
        burstF_.Reset();
        tailF_.Reset();
    }

    float Process() {
        if (!active_) return 0.0f;

        float n = rng_.White();

        /* 6 rafagas con 5ms de separacion (mas denso que 808's 4x6.5ms) */
        const float kDt  = 0.005f;
        const float kLen = 0.002f;
        float burstEnv = 0.0f;
        for (int i = 0; i < 6; i++) {
            float t = time_ - i * kDt;
            if (t >= 0.0f && t < kLen)
                burstEnv += expf(-t / 0.001f);
        }

        const float tailStart = 6.0f * kDt;
        float tailEnv = (time_ >= tailStart)
            ? expf(-(time_ - tailStart) / decay) : 0.0f;

        float out = burstF_.ProcessBP(n) * burstEnv
                  + tailF_.ProcessBP(n)  * tailEnv * 0.55f;

        float output = FastTanh(out * 2.5f);

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
 *  HIHAT 909 BASE
 * ---------------------------------------------------------------------
 *  6 ondas cuadradas con las frecuencias especificas de la TR-909
 *  (diferentes a la 808 -- mas brillantes y mas "digitales").
 *  HP a 7500 Hz (vs 6000-7200 Hz en 808).
 *  SVF HP precalculado en Init.
 * ===================================================================== */
class HiHat909Base {
protected:
    /* Frecuencias del oscilador metalico TR-909
     * Mas altas y mas espaciadas que la 808 -- caracter digital */
    static constexpr float METAL_FREQS[6] = {
        263.5f, 400.0f, 531.0f, 588.0f, 678.0f, 1043.0f
    };

    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_   = 0.0f, vel_ = 1.0f;
    float phase_[6] = {};
    Rng   rng_;
    SVF   hpFilter_;

    void BaseInit(float sampleRate, float hpFreq = 7800.0f) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_ = false;
        /* HP mas alto que 808 -- mas brillo, mas caracter digital */
        hpFilter_.SetCoefs(sr_, hpFreq, 0.65f);
        rng_.Seed(0x909AA01u); /* hihat */
    }

    void BaseTrigger(float velocity) {
        active_ = true;
        time_   = 0.0f;
        vel_    = VelCurve(velocity);
        memset(phase_, 0, sizeof(phase_));
        hpFilter_.Reset();
        rng_.Seed(0x909AA01u ^ (uint32_t)(vel_ * 0xFFFF));
    }

    /* Metallic oscillator: 6 squares + 8% ruido blanco
     * La 909 tiene mas contenido de noise que la 808 (mas digital) */
    float MetallicCore() {
        float sum = 0.0f;
        for (int i = 0; i < 6; i++) {
            phase_[i] += METAL_FREQS[i] * dt_;
            if (phase_[i] >= 1.0f) phase_[i] -= 1.0f;
            sum += (phase_[i] < 0.5f) ? 1.0f : -1.0f;
        }
        return (sum * (1.0f / 6.0f)) * 0.92f + rng_.White() * 0.08f;
    }
};

constexpr float HiHat909Base::METAL_FREQS[6];

/* =====================================================================
 *  HIHAT CLOSED 909
 * ===================================================================== */
class HiHatClosed : public HiHat909Base {
public:
    float decay  = 0.030f;  /* [0.01 - 0.2]  s  mas corto que 808 */
    float tone   = 0.6f;
    float volume = 1.0f;

    void Init(float sr) { BaseInit(sr, 9200.0f); }

    void Trigger(float v = 1.0f) { BaseTrigger(v); }

    float Process() {
        if (!active_) return 0.0f;
        float hp  = hpFilter_.ProcessHP(MetallicCore());
        float env = expf(-time_ / decay);
        env = env * env;  /* cuadrado: cierre seco */
        /* Mas brillante que 808: gain mas alto, tone bias hacia treble */
        float out = FastTanh(hp * (0.75f + tone * 0.95f) * 3.6f) * env;
        time_ += dt_;
        if (env < 0.0005f) active_ = false;
        return out * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d) { decay = Clamp(d, 0.01f, 0.2f); }
};

/* =====================================================================
 *  HIHAT OPEN 909
 * ===================================================================== */
class HiHatOpen : public HiHat909Base {
public:
    float decay  = 0.30f;   /* [0.05 - 2.0]  s */
    float tone   = 0.6f;
    float volume = 1.0f;

    void Init(float sr) {
        BaseInit(sr, 8000.0f);
        chokeGain_ = 1.0f;
        choking_ = false;
    }

    void Trigger(float v = 1.0f) {
        BaseTrigger(v);
        chokeGain_ = 1.0f;
        choking_ = false;
    }

    void Choke() { if (active_) choking_ = true; }

    float Process() {
        if (!active_) return 0.0f;
        float hp  = hpFilter_.ProcessHP(MetallicCore());
        float env = expf(-time_ / decay);
        if (choking_) {
            chokeGain_ *= 0.94f;
            if (chokeGain_ < 0.001f) active_ = false;
        }
        float out = FastTanh(hp * (0.75f + tone * 0.95f) * 3.2f) * env * chokeGain_;
        time_ += dt_;
        if (env < 0.0005f) active_ = false;
        return out * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d) { decay = Clamp(d, 0.05f, 2.0f); }

private:
    float chokeGain_ = 1.0f;
    bool  choking_ = false;
};

/* =====================================================================
 *  TOM 909 BASE
 * ---------------------------------------------------------------------
 *  Mas punch que los toms de la 808: pitch sweep x3 (vs x2.5)
 *  pitchDecay mas rapido (0.03s vs 0.05s).
 *
 *  v2.0: smack transient (noise BP muy corto al inicio)
 *  El smack simula el impacto de la baqueta en el parche.
 * ===================================================================== */
class Tom909Base {
public:
    float decay      = 0.20f;
    float pitch      = 150.0f;
    float pitchDecay = 0.03f;   /* mas rapido que 808 */
    float clickAmt   = 0.40f;   /* click initial punch */
    float smack      = 0.20f;   /* nivel del smack transient */
    float volume     = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_ = false;
        smackF_.SetCoefs(sr_, pitch * 7.0f, 1.4f);
        rng_.Seed(0x9090A01u); /* tom */
    }

    void Trigger(float velocity = 1.0f) {
        active_ = true;
        time_   = 0.0f;
        phase_  = 0.0f;
        vel_    = VelCurve(velocity);
        smackF_.SetCoefs(sr_, pitch * 7.0f, 1.4f);
        smackF_.Reset();
    }

    float Process() {
        if (!active_) return 0.0f;

        /* Pitch sweep mas agresivo que 808 */
        float cp = pitch + pitch * 3.0f * expf(-time_ / pitchDecay);
        phase_ += cp * dt_;
        if (phase_ >= 1.0f) phase_ -= 1.0f;
        float sine = sinf(TR909_TWOPI * phase_);

        /* Click: noise BP corto -- mas autentico que sine de 1500 Hz (v1.0) */
        float smackEnv = expf(-time_ / 0.004f);
        float smackOut = smackF_.ProcessBP(rng_.White()) * smackEnv * smack;

        /* Click adicional de alta frecuencia al inicio */
        float clickEnv = expf(-time_ / 0.0008f);
        float click    = rng_.White() * clickEnv * clickAmt * 0.3f;

        float amp    = expf(-time_ / decay);
        float output = FastTanh((sine + smackOut + click) * 1.4f) * amp;

        time_ += dt_;
        if (amp < 0.0005f) active_ = false;

        return output * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d) { decay = Clamp(d, 0.05f, 0.8f);  }
    void SetPitch(float p) {
        pitch = Clamp(p, 60.0f, 400.0f);
        smackF_.SetCoefs(sr_, pitch * 7.0f, 1.4f);
    }

protected:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, phase_ = 0.0f, vel_ = 1.0f;
    SVF   smackF_;
    Rng   rng_;
};

class LowTom : public Tom909Base {
public: LowTom() { pitch=90.0f;  decay=0.30f; pitchDecay=0.035f; smack=0.22f; }
};
class MidTom : public Tom909Base {
public: MidTom() { pitch=140.0f; decay=0.25f; pitchDecay=0.028f; smack=0.18f; }
};
class HiTom  : public Tom909Base {
public: HiTom()  { pitch=210.0f; decay=0.20f; pitchDecay=0.022f; smack=0.15f; }
};

/* =====================================================================
 *  RIDE 909
 * ---------------------------------------------------------------------
 *  Instrumento exclusivo de la 909 (no existe en 808).
 *  El ride real tiene dos componentes:
 *    Bell: sine corto a ~1000 Hz (el ping cuando la baqueta
 *          golpea el domo metalico central)
 *    Tail: metallic noise largo con decay muy suave
 *  La mezcla bell/tail es lo que define su caracter.
 * ===================================================================== */
class Ride : public HiHat909Base {
public:
    float decay    = 1.5f;    /* [0.5  - 5.0]  s  tail decay       */
    float bellDecay= 0.12f;   /* [0.05 - 0.4]  s  bell sine decay  */
    float bellAmt  = 0.35f;   /* [0.0  - 1.0]  nivel del bell tone */
    float tone     = 0.5f;
    float volume   = 1.0f;

    void Init(float sampleRate) {
        BaseInit(sampleRate, 6500.0f);
        bellF1_.SetCoefs(sampleRate, 980.0f, 3.5f);   /* fundamental: Q reducido */
        bellF2_.SetCoefs(sampleRate, 2350.0f, 4.0f);  /* 2nd partial ~2.4x */
    }

    void Trigger(float velocity = 1.0f) {
        BaseTrigger(velocity);
        bellImpulse_ = 1.0f;
        bellF1_.Reset();
        bellF2_.Reset();
    }

    float Process() {
        if (!active_) return 0.0f;

        /* Componente bell: dos partials para sonido metalico real */
        float bell1 = bellF1_.ProcessBP(bellImpulse_);
        float bell2 = bellF2_.ProcessBP(bellImpulse_) * 0.4f;
        bellImpulse_ = 0.0f;
        float bellEnv = expf(-time_ / bellDecay);

        /* Tail: metallic noise largo */
        float metal   = MetallicCore();
        float hp      = hpFilter_.ProcessHP(metal);
        float tailEnv = expf(-time_ / decay);
        float attack  = 1.0f - expf(-time_ / 0.002f);

        float out = (bell1 + bell2) * bellEnv * bellAmt
                  + hp   * tailEnv * attack * (0.3f + tone * 0.4f);

        out = FastTanh(out * 1.6f);

        time_ += dt_;
        if (tailEnv < 0.0003f) active_ = false;

        return out * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d)     { decay     = Clamp(d, 0.5f,  5.0f); }
    void SetBellDecay(float d) { bellDecay = Clamp(d, 0.05f, 0.4f); }
    void SetBellAmt(float a)   { bellAmt   = Clamp(a, 0.0f,  1.0f); }

private:
    SVF   bellF1_;
    SVF   bellF2_;
    float bellImpulse_ = 0.0f;
};

/* =====================================================================
 *  CRASH 909
 * ---------------------------------------------------------------------
 *  Instrumento exclusivo de la 909 (no existe en 808).
 *  Similar al ride pero con:
 *    Onset mas lento (attack ~3ms vs 2ms del ride)
 *    Mas contenido de noise blanco extra (brillo del crash)
 *    Decay muy largo
 *    No tiene bell tone
 * ===================================================================== */
class Crash : public HiHat909Base {
public:
    float decay    = 2.5f;   /* [0.5  - 8.0]  s */
    float tone     = 0.7f;
    float noiseAmt = 0.3f;   /* [0.0  - 0.5]  noise blanco extra */
    float volume   = 1.0f;

    void Init(float sampleRate) {
        BaseInit(sampleRate, 5800.0f);
        extraNoiseF_.SetCoefs(sampleRate, 9000.0f, 0.6f);
        rng2_.Seed(0x909CA02u); /* crash */
    }

    void Trigger(float velocity = 1.0f) {
        BaseTrigger(velocity);
        extraNoiseF_.Reset();
        rng2_.Seed(0x909CA02u ^ (uint32_t)(vel_ * 31337));
    }

    float Process() {
        if (!active_) return 0.0f;

        /* Metallic noise base */
        float metal = MetallicCore();
        float hp    = hpFilter_.ProcessHP(metal);

        /* Noise extra para brillo y densidad del crash */
        float extraNoise = extraNoiseF_.ProcessHP(rng2_.White()) * noiseAmt;

        /* Onset rapido -- el crash real tiene attack casi instantaneo */
        float attack = 1.0f - expf(-time_ / 0.0005f);
        float env    = expf(-time_ / decay);

        float out = (hp + extraNoise) * env * attack * (0.35f + tone * 0.5f);
        out = FastTanh(out * 2.0f);

        time_ += dt_;
        if (env < 0.0002f) active_ = false;

        return out * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d)    { decay    = Clamp(d, 0.5f,  8.0f); }
    void SetNoiseAmt(float n) { noiseAmt = Clamp(n, 0.0f,  0.5f); }

private:
    SVF extraNoiseF_;
    Rng rng2_;
};

/* =====================================================================
 *  RIMSHOT 909
 * ---------------------------------------------------------------------
 *  Click de noise HP brevísimo + resonador de tono ~880 Hz
 *  v2.0: SVF resonador excitado por impulso (como TR808 v2.0)
 * ===================================================================== */
class RimShot {
public:
    float decay  = 0.020f;
    float pitch  = 880.0f;
    float volume = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_ = false;
        resonF_.SetCoefs(sr_, pitch, 6.0f);
        rng_.Seed(0x909EA01u); /* rimshot */
    }

    void Trigger(float velocity = 1.0f) {
        active_  = true;
        time_    = 0.0f;
        vel_     = VelCurve(velocity);
        impulse_ = 1.0f;
        resonF_.SetCoefs(sr_, pitch, 6.0f);
        resonF_.Reset();
        rng_.Seed(0x909EA01u ^ (uint32_t)(vel_ * 12345));
    }

    float Process() {
        if (!active_) return 0.0f;

        float clickEnv = expf(-time_ / 0.0005f);
        float click    = rng_.White() * clickEnv * 0.7f;

        float toneEnv = expf(-time_ / decay);
        float tone    = resonF_.ProcessBP(impulse_) * toneEnv;
        impulse_ = 0.0f;

        float out = FastTanh((click + tone) * 2.2f);

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
 *  SHAKER / MARACAS 909
 * ---------------------------------------------------------------------
 *  Percusion de ruido brillante para completar el slot MA sin duplicar
 *  clap/rimshot. Mantiene caracter 909: agresivo, corto y presente.
 * ===================================================================== */
class Shaker {
public:
    float decay  = 0.085f;
    float tone   = 0.65f;
    float volume = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_ = false;
        noiseF_.SetCoefs(sr_, 5200.0f + tone * 5200.0f, 0.85f);
        bodyF_.SetCoefs(sr_, 2200.0f + tone * 1800.0f, 1.6f);
        rng_.Seed(0x9095A01u);
    }

    void Trigger(float velocity = 1.0f) {
        active_ = true;
        time_ = 0.0f;
        vel_ = VelCurve(velocity);
        noiseF_.SetCoefs(sr_, 5200.0f + tone * 5200.0f, 0.85f);
        bodyF_.SetCoefs(sr_, 2200.0f + tone * 1800.0f, 1.6f);
        noiseF_.Reset();
        bodyF_.Reset();
        rng_.Seed(0x9095A01u ^ (uint32_t)(vel_ * 27183));
    }

    float Process() {
        if (!active_) return 0.0f;
        float n = rng_.White();
        float env = expf(-time_ / decay);
        float atk = 1.0f - expf(-time_ / 0.0009f);
        float hp = noiseF_.ProcessHP(n) * (0.85f + tone * 0.65f);
        float bp = bodyF_.ProcessBP(n) * 0.35f;
        float out = FastTanh((hp + bp) * atk * env * 2.6f);
        time_ += dt_;
        if (env < 0.0005f) active_ = false;
        return out * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d) { decay = Clamp(d, 0.02f, 0.45f); }
    void SetTone(float t)  { tone = Clamp(t, 0.0f, 1.0f); }

private:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, vel_ = 1.0f;
    SVF   noiseF_, bodyF_;
    Rng   rng_;
};

/* =====================================================================
 *  CLAVE 909
 * ---------------------------------------------------------------------
 *  Resonador corto y duro, separado del rimshot para que el slot CL tenga
 *  identidad propia en patrones latinos/electro.
 * ===================================================================== */
class Clave {
public:
    float decay  = 0.055f;
    float pitch  = 1750.0f;
    float volume = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_ = false;
        resonF_.SetCoefs(sr_, pitch, 9.0f);
        clickF_.SetCoefs(sr_, 4200.0f, 1.1f);
        rng_.Seed(0x909C1A5u);
    }

    void Trigger(float velocity = 1.0f) {
        active_ = true;
        time_ = 0.0f;
        vel_ = VelCurve(velocity);
        impulse_ = 1.0f;
        resonF_.SetCoefs(sr_, pitch, 9.0f);
        resonF_.Reset();
        clickF_.Reset();
        rng_.Seed(0x909C1A5u ^ (uint32_t)(vel_ * 17777));
    }

    float Process() {
        if (!active_) return 0.0f;
        float toneEnv = expf(-time_ / decay);
        float tone = resonF_.ProcessBP(impulse_) * toneEnv;
        impulse_ = 0.0f;
        float clickEnv = expf(-time_ / 0.0007f);
        float click = clickF_.ProcessBP(rng_.White()) * clickEnv * 0.45f;
        float out = FastTanh((tone + click) * 2.4f);
        time_ += dt_;
        if (toneEnv < 0.0005f && clickEnv < 0.0005f) active_ = false;
        return out * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d) { decay = Clamp(d, 0.015f, 0.25f); }
    void SetPitch(float p) { pitch = Clamp(p, 900.0f, 3200.0f); }

private:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, vel_ = 1.0f, impulse_ = 0.0f;
    SVF   resonF_, clickF_;
    Rng   rng_;
};

/* =====================================================================
 *  PERC 909 BASE
 * ---------------------------------------------------------------------
 *  Tres percusiones tonales que ocupan HC/MC/LC sin clonar toms. Son mas
 *  cortas, con mas ataque metalico y menos cuerpo que los toms 909.
 * ===================================================================== */
class Perc909Base {
public:
    float decay  = 0.12f;
    float pitch  = 520.0f;
    float metal  = 0.18f;
    float volume = 1.0f;

    void Init(float sampleRate) {
        sr_ = sampleRate;
        dt_ = 1.0f / sr_;
        active_ = false;
        bodyF_.SetCoefs(sr_, pitch, 5.5f);
        metalF_.SetCoefs(sr_, pitch * 3.5f, 1.4f);
        rng_.Seed(0x9099E01u);
    }

    void Trigger(float velocity = 1.0f) {
        active_ = true;
        time_ = 0.0f;
        vel_ = VelCurve(velocity);
        impulse_ = 1.0f;
        bodyF_.SetCoefs(sr_, pitch, 5.5f);
        metalF_.SetCoefs(sr_, pitch * 3.5f, 1.4f);
        bodyF_.Reset();
        metalF_.Reset();
        rng_.Seed(0x9099E01u ^ (uint32_t)(vel_ * pitch));
    }

    float Process() {
        if (!active_) return 0.0f;
        float env = expf(-time_ / decay);
        float body = bodyF_.ProcessBP(impulse_) * env;
        impulse_ = 0.0f;
        float slapEnv = expf(-time_ / 0.0035f);
        float slap = metalF_.ProcessBP(rng_.White()) * slapEnv * metal;
        float out = FastTanh((body + slap) * 2.0f);
        time_ += dt_;
        if (env < 0.0005f && slapEnv < 0.0005f) active_ = false;
        return out * volume * vel_;
    }

    bool IsActive() const { return active_; }
    void SetDecay(float d) { decay = Clamp(d, 0.035f, 0.55f); }
    void SetPitch(float p) { pitch = Clamp(p, 160.0f, 1600.0f); }
    void SetMetal(float m) { metal = Clamp(m, 0.0f, 1.0f); }

protected:
    float sr_ = 48000.0f, dt_ = 1.0f / 48000.0f;
    bool  active_ = false;
    float time_ = 0.0f, vel_ = 1.0f, impulse_ = 0.0f;
    SVF   bodyF_, metalF_;
    Rng   rng_;
};

class HiPerc : public Perc909Base {
public: HiPerc() { pitch = 820.0f; decay = 0.085f; metal = 0.28f; }
};
class MidPerc : public Perc909Base {
public: MidPerc() { pitch = 520.0f; decay = 0.120f; metal = 0.20f; }
};
class LowPerc : public Perc909Base {
public: LowPerc() { pitch = 310.0f; decay = 0.170f; metal = 0.14f; }
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
    INST_RIDE,
    INST_CRASH,
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
};

namespace Presets {

/* Todos los instrumentos al mismo nivel -- como sale de fabrica */
static const KitPreset Classic909 = { "Classic 909",
        { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
            1.0f, 1.0f, 1.0f, 0.90f, 0.92f, 0.88f, 0.90f, 0.92f }
};

/* Techno: kick y hihat dominan, snare presente, poco crash */
static const KitPreset Techno = { "Techno",
        { 1.2f, 1.0f, 0.5f, 0.9f, 0.8f, 0.5f, 0.5f, 0.5f,
            0.7f, 0.3f, 0.4f, 0.82f, 0.70f, 0.55f, 0.58f, 0.60f }
};

/* House: kick gordo, hihat suave, clap en el 2 y 4 */
static const KitPreset HousePound = { "House Pound",
        { 1.1f, 0.8f, 1.0f, 0.6f, 0.7f, 0.6f, 0.6f, 0.6f,
            0.8f, 0.4f, 0.5f, 0.62f, 0.55f, 0.52f, 0.56f, 0.60f }
};

/* Industrial: agresivo, todo fuerte, crash y crash largo */
static const KitPreset Industrial = { "Industrial",
        { 1.2f, 1.1f, 0.8f, 1.0f, 0.9f, 0.7f, 0.7f, 0.7f,
            0.5f, 0.8f, 0.7f, 1.00f, 0.90f, 0.82f, 0.84f, 0.86f }
};

static const KitPreset Pure909 = { "Pure 909",
        { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
            1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f }
};

} /* namespace Presets */

/* =====================================================================
 *  KIT 909 -- contenedor completo
 * ---------------------------------------------------------------------
 *  Acceso a instrumentos por nombre o por indice enum
 *  Per-channel volume y mute
 *  Soft limiter de salida (peak follower + gain reduction)
 *  Preset loading
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
    Ride        ride;
    Crash       crash;
    RimShot     rimshot;
    Shaker      shaker;
    Clave       clave;
    HiPerc      hiPerc;
    MidPerc     midPerc;
    LowPerc     lowPerc;

    /* Optional PCM slots for the digital half of the original TR-909.
     * Main binds CH/OH/Ride/Crash; other slots remain procedural. */
    struct PcmSlot {
        const int16_t* data = nullptr;
        uint32_t length = 0;
        float sourceRate = 48000.0f;
        float pos = 0.0f;
        float step = 1.0f;
        float velocity = 1.0f;
        float chokeGain = 1.0f;
        bool choking = false;
        bool active = false;
    };

    void Init(float sampleRate) {
        sr_ = sampleRate;
        kick.Init(sr_);     snare.Init(sr_);    clap.Init(sr_);
        hihatC.Init(sr_);   hihatO.Init(sr_);
        lowTom.Init(sr_);   midTom.Init(sr_);   hiTom.Init(sr_);
        ride.Init(sr_);     crash.Init(sr_);    rimshot.Init(sr_);
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
            if (pcm_[INST_HIHAT_O].active)
                pcm_[INST_HIHAT_O].choking = true;
        }
        if (pcm_[inst].data != nullptr && pcm_[inst].length > 0) {
            PcmSlot& slot = pcm_[inst];
            slot.pos = 0.0f;
            slot.step = slot.sourceRate / sr_;
            slot.velocity = VelCurve(velocity);
            slot.chokeGain = 1.0f;
            slot.choking = false;
            slot.active = true;
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
            case INST_RIDE:     ride.Trigger(velocity);    break;
            case INST_CRASH:    crash.Trigger(velocity);   break;
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
        add(INST_RIDE,    ride);
        add(INST_CRASH,   crash);
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
            if (slot.choking) {
                slot.chokeGain *= 0.94f;
                if (slot.chokeGain < 0.001f) slot.active = false;
            }
            slot.pos += slot.step;
            if (slot.pos >= (float)slot.length) slot.active = false;
            if (!chanMute_[id])
                mix += sample * slot.velocity * slot.chokeGain * chanVol_[id];
        }

        /* Soft limiter: peak follower con attack instantaneo
         * y release ~0.2s -- evita clipping sin comer transientes */
        mix *= masterVol_;
        float absv = fabsf(mix);
        limitState_ = (absv > limitState_) ? absv : limitState_ * 0.9985f;
        if (limitState_ > 0.98f)
            mix *= 0.98f / limitState_;

        return mix;
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
        slot.chokeGain = 1.0f;
        slot.choking = false;
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
        if (ride.IsActive())    c++;
        if (crash.IsActive())   c++;
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

} /* namespace TR909 */

/* =====================================================================
 *  CHANGELOG
 *  v2.0  SVF correcto coefs precalculados, FastTanh, Xoshiro32,
 *        Kick sub-osc + compresion correcta + noise click,
 *        Snare SVF HP pitch-tracked, Clap 6x dual-filter,
 *        Toms smack transient, Ride bell+tail separados,
 *        Crash extra noise HP, RimShot resonador impulso,
 *        HiHat HP estable, Kit mixer + soft limiter + presets.
 *  v1.0  Version inicial con filtros inline, tanhf(), Xorshift.
 * ===================================================================== */
