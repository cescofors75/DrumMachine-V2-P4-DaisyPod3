#include "../../P4/src/pod_button_events.h"
#include <cassert>
#include <cstdio>
#include <thread>

int main() {
    PodButtonEvents events;
    bool visible=true;
    auto refresh=[&] { if(events.take(1)&1u) visible=!visible; };
    // A press followed by empty telemetry must survive a delayed UI frame.
    events.publish(2);
    for(int i=0;i<20;++i) events.publish(0);
    refresh(); assert(!visible);
    refresh(); assert(!visible); // no replay on the next frame
    events.publish(2); refresh(); assert(visible);
    // Two distinct taps before rendering must retain even toggle parity.
    events.publish(2); events.publish(2); refresh(); assert(visible);
    events.publish(7);
    assert(events.take(0)==1); assert(events.take(2)==1);
    refresh(); assert(!visible);
    events.publish(7); events.clear();
    for(unsigned i=0;i<3;++i) assert(events.take(i)==0);
    // USB producer and UI consumer cannot overwrite a concurrent press.
    std::atomic<bool> done{false};
    std::thread producer([&] {
        for(int i=0;i<100000;++i) events.publish(2);
        done.store(true);
    });
    uint32_t received=0;
    while(!done.load()) received+=events.take(1);
    producer.join(); received+=events.take(1);
    assert(received==100000);
    std::puts("Pod button events: delayed frames, toggle parity, no replay and concurrent delivery PASS");
}
