#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <cstdio>
#include "../../shared/bank_controller.h"
#include "../../shared/synth_banks.h"
#include "../../P4/src/ui/bank_input.h"
using namespace bank4;
struct State { float v[4]={50,800,0,3}; int writes=0; };
static float get(void* p,uint16_t id) { return static_cast<State*>(p)->v[id]; }
static void set(void* p,uint16_t id,float v) { auto& s=*static_cast<State*>(p); s.v[id]=v; ++s.writes; }
int main() {
    State state; Bank banks[10];
    for(auto& b:banks) b={"TEST",{
        {0,"LEVEL",Type::Continuous,0,100,1,50,0,0},
        {1,"FREQ",Type::Log,20,20000,1,800,0,0},
        {2,"ON",Type::Toggle,0,1,1,0,0,0},
        {3,"ENUM",Type::Enum,0,7,1,0,0,0}}};
    for(int n=1;n<=10;++n) {
        Controller c; c.setContext({uint32_t(n),"T",banks,uint8_t(n),&state,get,set});
        const int before=state.writes;
        c.fader(1023,0); c.fader(1023,200); assert(c.active==0); // Entering cannot move recalled bank.
        c.fader(0,201); c.fader(0,300); c.fader(1023,400); c.fader(1023,500);
        assert(c.active==n-1); assert(state.writes==before);
        c.select(0); c.fader(1023,600); c.fader(1023,900); assert(c.active==0); // Touch wins until fader moves.
        if(n>1) {
            c.fader(0,1000); c.fader(0,1100);
            int boundary=1024/n;
            for(int t=0;t<1000;++t) c.fader(uint16_t(boundary+(t%17)-8),1200+t);
            assert(c.active==0); // ADC jitter cannot flip a bank.
        }
    }
    Controller c; c.setContext({100,"A",banks,4,&state,get,set}); c.select(2);
    c.setContext({101,"B",banks,2,&state,get,set}); c.select(1);
    c.setContext({100,"A",banks,4,&state,get,set}); assert(c.active==2);
    state.v[0]=73; assert(c.rotate(0,1)); assert(state.v[0]==74); // Latest touch/external value, not a remembered knob position.
    c.press(0); c.rotate(0,1); assert(fabsf(state.v[0]-74.1f)<.001f);
    c.rotate(0,100000); assert(state.v[0]==100); c.rotate(0,-100000); assert(state.v[0]==0);
    c.rotate(1,1); assert(state.v[1]>800 && state.v[1]<900);
    c.rotate(2,1); assert(state.v[2]==1); c.press(2); assert(state.v[2]==0);
    c.rotate(3,-99); assert(state.v[3]==0); c.rotate(3,99); assert(state.v[3]==7);
    const int before=state.writes;
    banks[2].slots[0].type=Type::Action; c.rotate(0,1); c.press(0); assert(state.writes==before);
    banks[2].slots[1].flags=Disabled; c.rotate(1,1); assert(state.writes==before);
    c.setContext({}); c.rotate(0,1); c.press(0); assert(state.writes==before);
    Pickup pickup; assert(!pickup.accept(.1f,.6f)); assert(!pickup.accept(.4f,.6f));
    assert(pickup.accept(.8f,.6f)); pickup.reset(); assert(!pickup.accept(.8f,.2f));
    for(const auto& engine:SP_ENGINES) {
        bool seen[32]={}; uint8_t count=0; const auto* defs=synthBanks(engine.engine,count);
        assert(count==(engine.engine==3 ? 4 : engine.engine==4 ? 2 : engine.engine==5 ? 5 : 4));
        int mapped=0;
        for(uint8_t b=0;b<count;++b) for(int id:defs[b].ids) if(id>=0) {
            const auto* p=synthBankParam(engine,id); assert(p); assert(!seen[id]); seen[id]=true; ++mapped;
            assert(p->vmin<=p->vdef && p->vdef<=p->vmax);
        }
        assert(mapped==engine.param_count);
    }
    bank_input::clear(); const uint32_t oldEpoch=bank_input::epoch.load();
    bank_input::rotate(0,2,oldEpoch); bank_input::rotate(0,-1,oldEpoch); assert(bank_input::take(0)==1);
    bank_input::rotate(0,5,oldEpoch); bank_input::clear(); assert(bank_input::take(0)==0);
    bank_input::rotate(0,5,oldEpoch); assert(bank_input::take(0)==0); // Old context's in-flight I2C read.
    bank_input::active=true;
    bank_input::press(0,oldEpoch); assert(bank_input::takePress(0)==0);
    bank_input::press(0,bank_input::epoch.load()); assert(bank_input::takePress(0)==1);
    bank_input::press(0,bank_input::epoch.load()); bank_input::clear(); assert(bank_input::takePress(0)==0);
    puts("BANK: fader 1-10 banks, jitter, debounce, touch override, context recall, relative/fine/log, enums, protected actions, pickup, synth coverage, stale events PASS");
}
