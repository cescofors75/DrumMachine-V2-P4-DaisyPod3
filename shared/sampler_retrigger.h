#pragma once
#include <stddef.h>
#include <stdint.h>
namespace sampler {
// A sequencer track retriggers its existing voice; live pads retain polyphony.
template<class Voice>
int sequencedSlot(const Voice* voices, size_t count, uint8_t pad) {
    for(size_t i=0;i<count;++i)
        if(voices[i].active && voices[i].sequenced && voices[i].pad==pad) return int(i);
    return -1;
}
inline float replacementGain(float& residualWeight) {
    const float gain=1.f-residualWeight;
    residualWeight*=0.94f;
    if(residualWeight<0.0001f) residualWeight=0.f;
    return gain;
}
}
