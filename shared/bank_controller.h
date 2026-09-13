#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>

namespace bank4 {
enum class Type : uint8_t { Continuous, Integer, Enum, Toggle, Log, Time, Action };
enum Flags : uint16_t { Disabled = 1, Protected = 2 };
struct Param {
    uint16_t id = 0;
    const char* label = nullptr;
    Type type = Type::Continuous;
    float min = 0, max = 1, step = .01f, initial = 0;
    uint32_t color = 0x00e5ff;
    uint16_t flags = 0;
};
struct Bank { const char* title = ""; Param slots[4]; };
struct Context {
    uint32_t key = 0;
    const char* title = "";
    const Bank* banks = nullptr;
    uint8_t count = 0;
    void* state = nullptr;
    float (*get)(void*, uint16_t) = nullptr;
    void (*set)(void*, uint16_t, float) = nullptr;
};
inline float clamp(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }
// Also used by future absolute controls. Navigation must explicitly re-arm pickup.
struct Pickup {
    bool acquired = false, havePrevious = false;
    float previous = 0;
    void reset() { acquired = havePrevious = false; }
    bool accept(float physical, float target, float tolerance = .015f) {
        if(!isfinite(physical) || !isfinite(target)) return false;
        if(fabsf(physical-target) <= tolerance || (havePrevious &&
           ((previous <= target && physical >= target) || (previous >= target && physical <= target)))) acquired = true;
        previous = physical; havePrevious = true;
        return acquired;
    }
};
class Controller {
public:
    Context context;
    uint8_t active = 0;
    uint32_t revision = 0;
    bool fine[4] = {};
    bool setContext(const Context& next) {
        if(context.key == next.key && context.count == next.count) { context = next; return false; }
        remember(); context = next; active = 0;
        for(const auto& m : memory) if(m.key == next.key) active = m.bank;
        if(!next.count || active >= next.count) active = 0;
        for(bool& f : fine) f = false;
        faderArmed = false; candidate = -1; ++revision;
        return true;
    }
    void select(int index) {
        if(!context.count) return;
        const uint8_t next = uint8_t(clamp(float(index), 0, context.count-1));
        if(next == active) return;
        active = next; candidate = -1; faderArmed = false;
        for(bool& f : fine) f = false;
        remember(); ++revision;
    }
    // Call continuously with filtered 0..1023 ADC, even when stationary, to debounce.
    void fader(uint16_t raw, uint32_t now) {
        raw = raw > 1023 ? 1023 : raw;
        if(!faderArmed) { anchor = raw; faderArmed = true; moved = false; candidate = -1; return; }
        if(!moved) {
            if(abs(int(raw)-int(anchor)) < 30) return;
            moved = true;
        }
        if(context.count <= 1) return;
        int zone = int(uint32_t(raw)*context.count/1024);
        if(zone == active) { candidate = -1; return; }
        // 4% full scale, limited to one quarter of a zone for large bank counts.
        const int margin = int(context.count > 6 ? 256/context.count : 41);
        const int boundary = zone > active ? (active+1)*1024/context.count : active*1024/context.count;
        if((zone > active && int(raw) < boundary+margin) ||
           (zone < active && int(raw) > boundary-margin)) { candidate = -1; return; }
        if(candidate != zone) { candidate = zone; since = now; return; }
        if(uint32_t(now-since) >= 80) { select(zone); faderArmed = true; moved = true; }
    }
    const Param* slot(uint8_t i) const {
        return context.banks && context.count && i < 4 ? &context.banks[active].slots[i] : nullptr;
    }
    float value(uint8_t i) const {
        const auto* p = slot(i);
        return p && p->label && context.get ? context.get(context.state,p->id) : 0;
    }
    float normalized(uint8_t i) const {
        const auto* p = slot(i); if(!p || !p->label || p->max <= p->min) return 0;
        const float current = value(i);
        if(!isfinite(current)) return 0;
        float v = clamp(current,p->min,p->max);
        if((p->type == Type::Log || p->type == Type::Time) && p->min > 0)
            return logf(v/p->min)/logf(p->max/p->min);
        return (v-p->min)/(p->max-p->min);
    }
    bool rotate(uint8_t i, int delta) {
        const auto* p = slot(i);
        if(!p || !p->label || !delta || !context.get || !context.set ||
           (p->flags & (Disabled|Protected)) || p->type == Type::Action) return false;
        const float old = value(i);
        if(!isfinite(old)) return false;
        float step = p->step * (fine[i] ? .1f : 1.f);
        float v;
        if(p->type == Type::Toggle) v = delta > 0 ? 1 : 0;
        else if(p->type == Type::Enum || p->type == Type::Integer) v = roundf(old)+delta;
        else if((p->type == Type::Log || p->type == Type::Time) && p->min > 0)
            v = clamp(old,p->min,p->max)*powf(p->max/p->min, delta*(fine[i] ? .001f : .01f));
        else v = old+delta*step;
        v = clamp(v,p->min,p->max);
        if(fabsf(v-old) < 1e-7f) return false;
        context.set(context.state,p->id,v); return true;
    }
    void press(uint8_t i) {
        const auto* p = slot(i);
        if(!p || !p->label || (p->flags & (Disabled|Protected)) || p->type == Type::Action) return;
        if(p->type == Type::Toggle) rotate(i,value(i) > .5f ? -1 : 1);
        else if(p->type != Type::Integer && p->type != Type::Enum) { fine[i] = !fine[i]; ++revision; }
    }
private:
    struct Memory { uint32_t key = 0; uint8_t bank = 0; } memory[128];
    uint8_t nextMemory = 0;
    bool faderArmed = false, moved = false;
    uint16_t anchor = 0;
    int candidate = -1;
    uint32_t since = 0;
    void remember() {
        if(!context.key) return;
        for(auto& m : memory) if(m.key == context.key) { m.bank = active; return; }
        memory[nextMemory] = {context.key,active}; nextMemory = (nextMemory+1)%128;
    }
};
}
