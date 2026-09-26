#pragma once
// Included after filter type constants and CMSIS/math headers.
struct BiquadEQ {
    float b0=1,b1=0,b2=0,a1=0,a2=0;
    float z1=0,z2=0;

    float Process(float in){
        float out = b0*in + z1;
        if(!isfinite(out)) { Reset(); return 0.f; }
        z1 = b1*in - a1*out + z2;
        z2 = b2*in - a2*out;
        return out;
    }
    void Reset(){
        const uint32_t irq=__get_PRIMASK(); __disable_irq();
        z1=z2=0;
        __set_PRIMASK(irq);
    }

    void SetType(uint8_t t, float freq, float q, float sr, float gainDb=0.f){
        if(!isfinite(freq) || !isfinite(q) || !isfinite(sr) || sr<=0.f || !isfinite(gainDb)) return;
        // Compute off-line: AudioCallback must never observe half of a new
        // coefficient set when a foreground USB command interrupts a sweep.
        float b0=1.f,b1=0.f,b2=0.f,a1=0.f,a2=0.f;
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
        if(!isfinite(b0) || !isfinite(b1) || !isfinite(b2) || !isfinite(a1) || !isfinite(a2)) return;
        const uint32_t irq=__get_PRIMASK(); __disable_irq();
        this->b0=b0; this->b1=b1; this->b2=b2; this->a1=a1; this->a2=a2;
        __set_PRIMASK(irq);
    }
};
