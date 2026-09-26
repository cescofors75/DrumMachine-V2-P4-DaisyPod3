#pragma once
#include <stdint.h>
namespace audio {
inline bool deadlineReached(uint32_t start,uint32_t now,uint32_t budget) {
    return uint32_t(now-start)>=budget;
}
inline float remainingFade(uint32_t offset,uint32_t count) {
    return count ? float(count-offset-1)/float(count) : 0.f;
}
}
