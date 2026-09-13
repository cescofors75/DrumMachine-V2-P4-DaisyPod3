#pragma once
#include "synth_params.h"

// Parameter IDs, never positions in the editor. Ranges/defaults remain in synth_params.h.
struct SynthBankDef { const char* title; int8_t ids[4]; };
static constexpr SynthBankDef SB_303[] = {
    {"FILTER",{0,1,2,3}}, {"AMP",{8,9,10,4}},
    {"CHARACTER",{5,11,12,13}}, {"PERFORMANCE",{14,6,7,-1}}
};
static constexpr SynthBankDef SB_WT[] = {
    {"OSC / ENV",{0,1,2,3}}, {"FILTER / LFO",{4,5,6,7}}
};
static constexpr SynthBankDef SB_SH[] = {
    {"OSC",{0,1,2,3}}, {"FILTER",{4,5,6,11}}, {"AMP ENV",{7,8,9,10}},
    {"MOD",{12,13,14,15}}, {"CHARACTER",{16,17,18,19}}
};
static constexpr SynthBankDef SB_FM[] = {
    {"CARRIER ENV",{0,1,2,3}}, {"MODULATOR ENV",{4,5,6,7}},
    {"FM CORE",{8,9,10,11}}, {"OUTPUT / PERFORMANCE",{12,13,14,-1}}
};
inline const SynthBankDef* synthBanks(uint8_t engine, uint8_t& count) {
    switch(engine) {
        case SP_ENGINE_303: count=4; return SB_303;
        case SP_ENGINE_WT: count=2; return SB_WT;
        case SP_ENGINE_SH101: count=5; return SB_SH;
        case SP_ENGINE_FM2OP: count=4; return SB_FM;
        default: count=0; return nullptr;
    }
}
inline const SynthParamDef* synthBankParam(const SynthEngineDef& engine, int id) {
    for(uint8_t i=0;i<engine.param_count;++i) if(engine.params[i].param_id==id) return &engine.params[i];
    return nullptr;
}
