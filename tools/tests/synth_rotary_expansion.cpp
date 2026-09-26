#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include "../../shared/rotary_steps.h"
#include "../../shared/sequence_group_variations.h"
#include "../../DaisyPod3/synth/tb303.h"
#include "../../DaisyPod3/synth/sh101.h"
#include "../../DaisyPod3/synth/fm2op.h"
#include "../../DaisyPod3/synth/wavetable_osc.h"
template<class Synth> void check(const char* name,Synth& synth) {
    synth.Init(48000.f);
    synth.NoteOn(uint8_t(48),.8f);
    float previous=0;
    for(int i=0;i<800;++i) previous=synth.Process();
    synth.NoteOff(); synth.NoteOn(uint8_t(60),.3f);
    const float first=synth.Process();
    assert(std::fabs(first-previous)<1e-6f);
    for(int i=0;i<240000;++i) {
        if(i%1000==0) { synth.NoteOff(); synth.NoteOn(uint8_t(36+(i/1000)%48),.8f); }
        assert(std::isfinite(synth.Process()));
    }
    std::printf("%s: retrigger continuity and 5s finite render PASS\n",name);
}
int main() {
    for(int d=-24;d<=24;++d) assert(rotary::detents(d)==0);
    for(int n=1;n<10;++n) { assert(rotary::detents(n*51)==n); assert(rotary::detents(-n*51)==-n); }
    seqgroups::Step base[16],out[16];
    for(int s=0;s<16;++s) base[s]={s%4==0,90,75,1};
    for(int v=0;v<16;++v) {
        seqgroups::variation(base,out,v);
        for(const auto& x:out) assert(x.velocity<=127 && x.probability<=100 && x.ratchet>=1 && x.ratchet<=4);
        if(v>=11 && v<=13) { int count=0; for(auto x:out) count+=x.active; assert(count==(v==11 ? 3 : v==12 ? 5 : 7)); }
    }
    for(int v=0;v<12;++v) {
        seqgroups::glitch(base,out,v);
        for(const auto& x:out) assert(x.ratchet>=1 && x.ratchet<=4);
        unsigned mask=0; for(auto p:seqgroups::orderMap[v]) mask|=1u<<p;
        assert(mask==15);
        for(int prev=0;prev<v;++prev) assert(std::memcmp(seqgroups::orderMap[prev],seqgroups::orderMap[v],4)!=0);
    }
    SH101::Synth sh; FM2Op::Synth fm;
    check("SH101",sh); check("FM2OP",fm);
    TB303::Synth acid; acid.Init(48000.f); acid.NoteOn(uint8_t(48),true,false);
    float previous=0; for(int i=0;i<800;++i) previous=acid.Process();
    acid.NoteOff(); acid.NoteOn(uint8_t(60),false,false);
    assert(std::fabs(acid.Process()-previous)<1e-6f);
    WavetableOsc wt; wt.Init(48000.f); wt.NoteOn(48,.8f);
    for(int i=0;i<800;++i) previous=wt.Process();
    wt.NoteOff(48); wt.NoteOn(48,.3f);
    assert(std::fabs(wt.Process()-previous)<1e-6f);
    for(int i=0;i<240000;++i) {
        if(i%1000==0) {
            acid.NoteOff(); acid.NoteOn(uint8_t(36+(i/1000)%48),(i/1000)%4==0,false);
            wt.AllNotesOff(); wt.NoteOn(uint8_t(36+(i/1000)%48),.7f);
        }
        assert(std::isfinite(acid.Process())); assert(std::isfinite(wt.Process()));
    }
    std::puts("303/WT continuity, rotary detents and expanded musical banks PASS");
}
