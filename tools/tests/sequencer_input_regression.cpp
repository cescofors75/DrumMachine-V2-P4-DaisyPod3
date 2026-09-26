#include <cassert>
#include <initializer_list>
#include "../../P4/src/ui/bank_input.h"
#include "../../shared/rotary_steps.h"

int main() {
    using namespace bank_input;
    clear();
    const auto generation=epoch.load();
    rotate(0,4,generation);
    for(int i=0;i<4;++i) assert(takeStep(0)==1);
    assert(takeStep(0)==0);
    rotate(1,-10,generation);
    assert(takeStep(1)==-1);
    assert(take(1)==-9); // No loss after draining a single selector step.
    rotate(2,5,generation);
    rotate(2,-3,generation);
    assert(takeStep(2)==1);
    assert(takeStep(2)==1);
    assert(takeStep(2)==0);
    rotate(3,4,generation);
    clear();
    rotate(3,10,generation); // Events from the old context are rejected.
    assert(takeStep(3)==0);
    rotate(0,7,epoch.load());
    deltas[0]=0; // Rotary press reset discards preceding motion.
    assert(takeStep(0)==0);
    // Headroom at the physical counter boundaries permits a full detent
    // in either direction even when the parameter is at 0 or 100 percent.
    for(int feedback: {51,972}) {
        assert(rotary::detents((feedback+51)-feedback)==1);
        assert(rotary::detents((feedback-51)-feedback)==-1);
    }
}
