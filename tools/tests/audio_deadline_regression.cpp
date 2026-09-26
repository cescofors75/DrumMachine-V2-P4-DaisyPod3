#include <cassert>
#include <cstdio>
#include <cmath>
#include "../../shared/audio_deadline.h"
int main() {
    assert(!audio::deadlineReached(100,189,90));
    assert(audio::deadlineReached(100,190,90));
    assert(!audio::deadlineReached(0xfffffff0u,0x20u,49));
    assert(audio::deadlineReached(0xfffffff0u,0x21u,49));
    for(unsigned count=1;count<=256;++count) {
        float previous=1.f;
        for(unsigned i=0;i<count;++i) {
            const float gain=audio::remainingFade(i,count);
            assert(std::isfinite(gain) && gain>=0.f && gain<previous);
            previous=gain;
        }
        assert(previous==0.f);
    }
    std::puts("Audio deadline: boundary, timer wrap and bounded fade to zero PASS");
}
