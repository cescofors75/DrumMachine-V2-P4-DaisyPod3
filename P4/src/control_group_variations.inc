bool control_apply_group_variation(uint8_t group,uint8_t variation,uint8_t page)
{
    // Explicit button reset restores the captured pre-effect rhythm even when
    // EVOLVE changed it later. Ordinary turns still adopt external edits.
    const bool explicitReset=variation>=30 && variation<=33;
    if(explicitReset) {
        const uint8_t resets[4]={0,10,20,200};
        variation=resets[variation-30];
    }
    const bool expandedVar=variation>=40 && variation<=55;
    const bool expandedGlitch=variation>=60 && variation<=71;
    const bool expandedOrder=variation>=80 && variation<=91;
    const bool order=(variation>=20 && variation<=23) || expandedOrder;
    const bool probability=variation>=100 && variation<=200;
    const bool glitch=(variation>=10 && variation<=14) || expandedGlitch;
    if(group>=4 || (variation>=7 && !order && !probability && !glitch && !expandedVar) || page>=4) return false;
    // Each bank detects changes made by the other banks as a new baseline.
    // For example, GLITCH must retain a probability chosen in PROBABILIDAD.
    const int family=order ? 2 : probability ? 3 : glitch ? 1 : 0;
    static seqgroups::Step bases[4][16][16]{},lasts[4][16][16]{};
    static int identities[4][4]={{-1,-1,-1,-1},{-1,-1,-1,-1},{-1,-1,-1,-1},{-1,-1,-1,-1}};
    auto& base=bases[family]; auto& last=lasts[family];
    auto& patterns=identities[family];
    const uint16_t mask=order ? uint16_t(15u<<(group*4)) : seqgroups::masks[group];
    const int pattern=Clamp(p4.current_pattern,0,MAX_PATTERNS-1);
    auto& seq=SequencerInstance();
    seqgroups::Step current[16][16]{};
    const int identity=pattern*4+page;
    bool reset=patterns[group]!=identity;
    for(int t=0;t<16;++t) if(mask & (1u<<t))
        for(int s=0;s<16;++s) {
            current[t][s]={seq.getStep(pattern,t,s),seq.getStepVelocity(pattern,t,s),
                seq.getStepProbability(pattern,t,s),uint8_t(Clamp<int>(seq.getStepRatchet(pattern,t,s),1,4))};
            if(!explicitReset && !seqgroups::equal(current[t][s],last[t][s])) reset=true;
        }
    if(reset) {
        patterns[group]=identity;
        for(int t=0;t<16;++t) if(mask & (1u<<t))
            memcpy(base[t],current[t],sizeof(base[t]));
    }
    control_variation_snapshot_current();
    bool changed=false;
    for(int t=0;t<16;++t) if(mask & (1u<<t)) {
        if(order) memcpy(last[t],base[group*4+seqgroups::orderMap[expandedOrder ? variation-80 : variation-20][t%4]],sizeof(last[t]));
        else if(probability) {
            memcpy(last[t],current[t],sizeof(last[t]));
            for(auto& step:last[t]) if(step.active) step.probability=variation-100;
        } else if(expandedVar) seqgroups::variation(base[t],last[t],variation-40);
        else if(expandedGlitch) seqgroups::glitch(base[t],last[t],variation-60);
        else seqgroups::transform(base[t],last[t],variation);
        for(int s=0;s<16;++s) {
            const auto& v=last[t][s];
            if(seqgroups::equal(current[t][s],v)) continue;
            seq.setStep(pattern,t,s,v.active,v.velocity);
            seq.setStepProbability(pattern,t,s,v.probability);
            seq.setStepRatchet(pattern,t,s,v.ratchet);
            p4.steps[t][s]=v.active; changed=true;
        }
    }
    return changed;
}
