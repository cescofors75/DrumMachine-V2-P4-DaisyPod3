#ifdef NDEBUG
#undef NDEBUG
#endif
#include "../../shared/sequence_group_variations.h"
#include <cassert>
#include <cstdio>
#include <cstring>
template<class T> T Clamp(T v,T lo,T hi) { return v<lo ? lo : v>hi ? hi : v; }
constexpr int MAX_PATTERNS=2;
struct { int current_pattern=0; bool steps[16][16]{}; } p4;
struct FakeSequencer {
    seqgroups::Step data[2][16][16]{};
    bool getStep(int p,int t,int s) { return data[p][t][s].active; }
    uint8_t getStepVelocity(int p,int t,int s) { return data[p][t][s].velocity; }
    uint8_t getStepProbability(int p,int t,int s) { return data[p][t][s].probability; }
    uint8_t getStepRatchet(int p,int t,int s) { return data[p][t][s].ratchet; }
    void setStep(int p,int t,int s,bool a,uint8_t v) { data[p][t][s].active=a; data[p][t][s].velocity=v; }
    void setStepProbability(int p,int t,int s,uint8_t v) { data[p][t][s].probability=v; }
    void setStepRatchet(int p,int t,int s,uint8_t v) { data[p][t][s].ratchet=v; }
} sequencer;
FakeSequencer& SequencerInstance() { return sequencer; }
static seqgroups::Step undo[16][16];
void control_variation_snapshot_current() { memcpy(undo,sequencer.data[p4.current_pattern],sizeof(undo)); }
bool control_apply_group_variation(uint8_t group,uint8_t variation,uint8_t page=0);
#include "../../P4/src/control_group_variations.inc"
int main() {
    uint16_t covered=0;
    for(auto mask:seqgroups::masks) { assert(!(covered&mask)); covered|=mask; }
    assert(covered==0xffff && seqgroups::masks[0]==1 && seqgroups::masks[1]==0x22);
    seqgroups::Step base[16]{},out[16]{};
    for(unsigned s=0;s<16;++s) base[s]={s%4==0,uint8_t(40+s),100,1};
    seqgroups::transform(base,out,0);
    for(int s=0;s<16;++s) assert(seqgroups::equal(base[s],out[s]));
    seqgroups::transform(base,out,1); assert(out[1].active && !out[0].active && out[1].velocity==40);
    seqgroups::transform(base,out,2); assert(out[15].active && out[15].velocity==40);
    seqgroups::transform(base,out,3); assert(out[0].active && out[8].active && !out[4].active);
    seqgroups::transform(base,out,4); assert(out[3].active && out[3].velocity==42 && out[3].probability==75);
    seqgroups::transform(base,out,5); assert(out[0].ratchet==2 && out[1].ratchet==1);
    // Always transform the baseline, so selecting another variant is reversible.
    seqgroups::transform(base,out,0); assert(out[0].ratchet==1 && !out[3].active);
    for(int p=0;p<2;++p) for(int t=0;t<16;++t) for(int s=0;s<16;++s)
        sequencer.data[p][t][s]={s%4==0,uint8_t(90+t),100,1};
    assert(control_apply_group_variation(0,1));
    assert(!sequencer.data[0][0][0].active && sequencer.data[0][0][1].active);
    for(int t=1;t<16;++t) for(int s=0;s<16;++s) assert(seqgroups::equal(sequencer.data[0][t][s],undo[t][s]));
    assert(control_apply_group_variation(0,0));
    assert(sequencer.data[0][0][0].active && !sequencer.data[0][0][1].active);
    assert(control_apply_group_variation(1,5));
    assert(sequencer.data[0][1][0].ratchet==2 && sequencer.data[0][5][0].ratchet==2);
    assert(sequencer.data[0][0][0].ratchet==1);
    // External edits establish a new baseline instead of being overwritten.
    sequencer.data[0][0][0].velocity=55;
    assert(control_apply_group_variation(0,1));
    assert(sequencer.data[0][0][1].velocity==55);
    p4.current_pattern=1;
    assert(control_apply_group_variation(0,1));
    assert(sequencer.data[1][0][1].velocity==90);
    // A newly staged page may have identical hits but must get its own base.
    assert(control_apply_group_variation(0,1,1));
    assert(sequencer.data[1][0][2].active);
    assert(!control_apply_group_variation(0,1,4));
    assert(control_apply_group_variation(2,125));
    assert(sequencer.data[1][2][0].probability==25 && sequencer.data[1][3][0].probability==25);
    assert(sequencer.data[1][4][0].probability==100);
    assert(control_apply_group_variation(2,100)); assert(sequencer.data[1][2][0].probability==0);
    assert(control_apply_group_variation(2,12)); assert(sequencer.data[1][2][0].ratchet==4);
    assert(sequencer.data[1][2][0].probability==0);
    const auto first=sequencer.data[1][4][0], fourth=sequencer.data[1][7][0], outside=sequencer.data[1][8][0];
    assert(control_apply_group_variation(1,21));
    assert(seqgroups::equal(sequencer.data[1][4][0],fourth));
    assert(seqgroups::equal(sequencer.data[1][5][0],first));
    assert(seqgroups::equal(sequencer.data[1][8][0],outside));
    assert(control_apply_group_variation(1,20)); assert(seqgroups::equal(sequencer.data[1][4][0],first));
    assert(!control_apply_group_variation(4,0)); assert(!control_apply_group_variation(0,7));
    // Explicit reset must not adopt a later EVOLVE edit as the effect's base.
    assert(control_apply_group_variation(3,12));
    sequencer.data[1][6][0].velocity=51;
    assert(control_apply_group_variation(3,31));
    assert(sequencer.data[1][6][0].ratchet==1);
    assert(sequencer.data[1][6][0].velocity==96);
    assert(control_apply_group_variation(2,33));
    assert(sequencer.data[1][2][0].probability==100);
    assert(control_apply_group_variation(0,51)); // Euclid 3 through production adapter
    int pulses=0; for(int s=0;s<16;++s) pulses+=sequencer.data[1][0][s].active;
    assert(pulses==3);
    assert(control_apply_group_variation(0,65)); // glitch x3
    for(int s=0;s<16;++s) if(sequencer.data[1][0][s].active) assert(sequencer.data[1][0][s].ratchet==3);
    assert(!control_apply_group_variation(0,56));
    std::puts("Sequence groups: disjoint coverage, baseline restore, timing and dynamics PASS");
}
