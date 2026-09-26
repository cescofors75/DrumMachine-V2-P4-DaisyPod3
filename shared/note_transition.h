#pragma once
#include <stdint.h>
namespace audio {
// Short bounded crossfade only on an already sounding voice, not every sample.
struct NoteTransition {
    float last=0,held=0;
    uint8_t left=0;
    void restart(bool sounding) { held=last; left=sounding ? 32 : 0; }
    float process(float next) {
        if(left) { const float old=left/32.f; next=held*old+next*(1.f-old); --left; }
        return last=next;
    }
};
}
