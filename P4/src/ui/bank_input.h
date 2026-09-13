#pragma once
#include <atomic>
#include <stdint.h>

// Hardware task -> LVGL task. Only LVGL owns contexts and invokes setters.
namespace bank_input {
inline std::atomic<uint32_t> epoch{1};
inline std::atomic<uint64_t> deltas[4]{};
inline std::atomic<uint64_t> presses[4]{};
inline std::atomic<uint16_t> leds[4]{};
inline std::atomic<bool> active{false};
inline void publish(std::atomic<uint64_t>* queue, uint8_t i, int delta, uint32_t generation) {
    if(i >= 4 || !delta || generation != epoch.load()) return;
    uint64_t old = queue[i].load();
    do {
        if(generation != epoch.load()) return;
        int previous = uint32_t(old >> 32) == generation ? int32_t(old) : 0;
        int next = previous + delta;
        if(next > 128) next = 128; if(next < -128) next = -128;
        uint64_t packed = (uint64_t(generation)<<32) | uint32_t(next);
        if(queue[i].compare_exchange_weak(old,packed)) return;
    } while(true);
}
inline void rotate(uint8_t i,int delta,uint32_t generation) { publish(deltas,i,delta,generation); }
inline void press(uint8_t i,uint32_t generation) { if(active.load()) publish(presses,i,1,generation); }
inline void clear() {
    ++epoch;
    for(uint8_t i=0;i<4;++i) { deltas[i]=0; presses[i]=0; }
}
inline int take(uint8_t i) {
    const uint64_t v=deltas[i].exchange(0);
    return uint32_t(v>>32)==epoch.load() ? int32_t(v) : 0;
}
inline int takePress(uint8_t i) {
    const uint64_t v=presses[i].exchange(0);
    return uint32_t(v>>32)==epoch.load() ? int32_t(v) : 0;
}
}
