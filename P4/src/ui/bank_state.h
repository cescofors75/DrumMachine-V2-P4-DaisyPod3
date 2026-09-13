#pragma once
#include <atomic>
#include <stdint.h>
#include <math.h>
namespace bank_state {
struct Pad {
    std::atomic<bool> velocityEnabled{false};
    std::atomic<int> velocity{100}, probability{100}, repeat{0}, gate{0}, accent{0};
    std::atomic<int> pan{0}, pitch{0}, mod{0}, fadeIn{0}, fadeOut{0};
};
inline Pad pads[16];
inline uint8_t hitVelocity(uint8_t input,int target,bool enabled,int reference,bool accent) {
    if(!input) return 0;
    int value=input;
    if(enabled) value=value*target/(reference>0 ? reference : 1);
    if(accent) value+=15;
    return uint8_t(value<1 ? 1 : value>127 ? 127 : value);
}
inline std::atomic<int> fxValues[16][7]{};
inline std::atomic<uint8_t> fxDirty[16]{};
inline void fx(uint8_t pad, uint8_t field, int value) {
    if(pad>=16 || field>=7) return;
    fxValues[pad][field]=value; fxDirty[pad].fetch_or(1u<<field);
}
void observe(uint8_t command, const void* data, uint16_t size);
inline std::atomic<float> synth[4][32]{};
inline std::atomic<uint32_t> synthDirty[4]{};
inline void param(uint8_t engine, uint8_t id, float v) {
    if(engine<3 || engine>6 || id>=32 || !isfinite(v)) return;
    synth[engine-3][id].store(v);
    synthDirty[engine-3].fetch_or(1u<<id);
}
}
