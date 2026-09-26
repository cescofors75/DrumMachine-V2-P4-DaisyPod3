#pragma once
#include <atomic>
#include <stdint.h>

// USB producer / LVGL consumer. Unlike the latest telemetry snapshot, these
// counters retain presses while rendering is busy, including two quick taps.
class PodButtonEvents {
public:
    void publish(uint8_t mask) {
        for(unsigned i=0;i<3;++i)
            if(mask & (1u<<i)) pending_[i].fetch_add(1,std::memory_order_relaxed);
    }
    uint32_t take(unsigned button) {
        return button<3 ? pending_[button].exchange(0,std::memory_order_relaxed) : 0;
    }
    void clear() { for(auto& count:pending_) count.store(0,std::memory_order_relaxed); }
private:
    std::atomic<uint32_t> pending_[3]{};
};
