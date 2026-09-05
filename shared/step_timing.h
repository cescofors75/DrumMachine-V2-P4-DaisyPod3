#pragma once
#include <stdint.h>
inline uint32_t SwingOffset(uint32_t base, uint8_t swing) {
    return base * (swing > 100 ? 100u : swing) / 200u;
}
// A pair always lasts 2*base. Humanization stays within the step; all
// requested ratchets fit before the next onset (including short swung steps).
inline uint32_t RatchetInterval(uint32_t duration, uint32_t& delay, uint8_t count) {
    if(count < 1) count = 1;
    if(count > 4) count = 4;
    if(duration < count) duration = count;
    if(delay > duration - count) delay = duration - count;
    return (duration - delay) / count;
}
