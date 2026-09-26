#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
// Inspect the production envelope to distinguish a stuck accent from a
// deliberately resonant output; no alternative DSP implementation in this test.
#define private public
#include "../../DaisyPod3/synth/tb303.h"
#undef private
#include "../../DaisyPod3/synth/tr808.h"
#include "../../DaisyPod3/synth/tr909.h"
#include "../../DaisyPod3/synth/tr505.h"
template<class Kit> void stress(const char* name) {
    Kit kit; kit.Init(48000.f);
    float peak=0.f;
    // 20 seconds, 300 BPM, sixteenth notes with x4 ratchets: 80 hits/s
    // on each of the 16 instruments. Deliberately denser than the factory bank.
    for(int i=0;i<48000*20;++i) {
        if(i%600==0) for(uint8_t id=0;id<16;++id) kit.Trigger(id,(i/600)%4 ? .7f : 1.f);
        const float sample=kit.Process();
        assert(std::isfinite(sample));
        peak=std::fmax(peak,std::fabs(sample));
        assert(std::fabs(sample)<=.981f);
    }
    std::printf("%s: dense x4 ratchets, 20 seconds, peak %.6f PASS\n",name,peak);
}
int main() {
    TB303::Synth acid; acid.Init(48000.f);
    acid.params.accentAmt=1.f;
    acid.NoteOn(uint8_t(36),true,false);
    acid.Process();
    const float initial=acid.accentChirpEnv_;
    assert(initial>7000.f);
    acid.NoteOn(uint8_t(38),false,false);
    for(int i=0;i<12000;++i) assert(std::isfinite(acid.Process()));
    assert(acid.accentChirpEnv_<1.f);
    std::printf("303: accent chirp decays after non-accented retrigger (%.2f -> %.3f Hz) PASS\n",initial,acid.accentChirpEnv_);
    float peak=0;
    for(int i=0;i<48000*20;++i) {
        if(i%600==0) {
            acid.NoteOff();
            acid.SetCutoff((i/600)%2 ? 20.f : 20000.f);
            acid.SetResonance(.97f);
            acid.NoteOn(uint8_t(24+(i/600)%48),(i/600)%4==0,(i/600)%7==0);
        }
        const float sample=acid.Process();
        assert(std::isfinite(sample));
        peak=std::fmax(peak,std::fabs(sample));
    }
    std::printf("303: x4, maximum resonance, cutoff jumps, peak %.6f, finite PASS\n",peak);
    stress<TR808::Kit>("808"); stress<TR909::Kit>("909"); stress<TR505::Kit>("505");
}
