#include <cassert>
#include <cmath>
#include <cstdio>
#include "../../shared/sampler_retrigger.h"
struct Voice { bool active=false,sequenced=false; unsigned char pad=0; };
int main() {
    Voice voices[32];
    voices[0]={true,false,2}; // a live hit on the same pad must survive
    for(int hit=0;hit<10000;++hit) {
        int slot=sampler::sequencedSlot(voices,32,2);
        if(slot<0) { for(int i=0;i<32;++i) if(!voices[i].active) { slot=i; break; } }
        assert(slot==1);
        voices[slot]={true,true,2};
    }
    assert(sampler::sequencedSlot(voices,32,3)==-1);
    int active=0; for(const auto& v:voices) active+=v.active;
    assert(active==2);
    float peak=0;
    for(int old=-10;old<=10;++old) for(int next=-10;next<=10;++next) {
        float fade=1.f,tail=old/10.f;
        for(int i=0;i<300;++i) {
            const float mixed=tail+next/10.f*sampler::replacementGain(fade);
            if(i==0) assert(std::fabs(mixed-old/10.f)<1e-6f);
            assert(std::isfinite(mixed) && std::fabs(mixed)<=1.00011f);
            peak=std::fmax(peak,std::fabs(mixed));
            tail*=0.94f;
        }
    }
    std::printf("Sampler: 10000 retriggers reuse one sequencer voice; LIVE preserved; complementary transition peak %.5f PASS\n",peak);
}
