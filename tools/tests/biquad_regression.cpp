#include <cmath>
#include <cstdint>
#include <cassert>
#include <cstdio>
#include <limits>
#include <initializer_list>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
using std::isfinite;
static uint32_t irq=0,commits=0;
uint32_t __get_PRIMASK() { return irq; }
void __disable_irq() { irq=1; }
void __set_PRIMASK(uint32_t value) { assert(irq==1); irq=value; ++commits; }
float pow10f(float x) { return std::pow(10.f,x); }
enum { FTYPE_NONE, FTYPE_LOWPASS, FTYPE_HIGHPASS, FTYPE_BANDPASS,
       FTYPE_NOTCH, FTYPE_ALLPASS, FTYPE_PEAKING, FTYPE_LOWSHELF,
       FTYPE_HIGHSHELF, FTYPE_RESONANT };
#include "../../DaisyPod3/biquad_eq.h"
int main() {
    BiquadEQ filter;
    for(int type=0;type<=9;++type) for(float q:{0.3f,0.7f,10.f,40.f}) {
        filter.Reset();
        for(int sweep=0;sweep<200;++sweep) {
            const float hz=20.f*std::pow(1000.f,(sweep%100)/99.f);
            filter.SetType(type,hz,q,48000.f,6.f);
            assert(irq==0);
            for(int s=0;s<128;++s) assert(isfinite(filter.Process(s==0 ? 1.f : 0.f)));
        }
    }
    filter.SetType(FTYPE_LOWPASS,1000.f,0.7f,48000.f);
    const float before=filter.b0;
    filter.SetType(FTYPE_LOWPASS,std::numeric_limits<float>::quiet_NaN(),1,48000);
    assert(filter.b0==before);
    filter.SetType(FTYPE_LOWPASS,1000,1,0); assert(filter.b0==before);
    filter.z1=std::numeric_limits<float>::infinity();
    assert(filter.Process(1)==0 && filter.z1==0 && filter.z2==0);
    assert(isfinite(filter.Process(1)));
    irq=1; filter.SetType(FTYPE_HIGHPASS,800,1,48000); assert(irq==1);
    filter.Reset(); assert(irq==1); irq=0;
    assert(commits>8000);
    std::puts("Biquad: model/cutoff/Q sweeps, invalid parameters, nonfinite recovery and IRQ restoration PASS");
}
