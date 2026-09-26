#include "../../shared/midi_input.h"
#include <cassert>

int main() {
    MidiInputParser p;
    MidiInputMessage m{};
    assert(!p.push(42, m)); // Orphan data
    assert(!p.push(0x92, m));
    assert(!p.push(60, m));
    assert(p.push(0xF8, m) && m.status == 0xF8);
    assert(p.push(100, m) && m.status == 0x92 && m.data0 == 60 && m.data1 == 100);
    assert(!p.push(61, m));
    assert(p.push(0, m) && m.status == 0x92 && m.data1 == 0);
    assert(!p.push(0xC3, m));
    assert(p.push(10, m) && m.status == 0xC3 && m.data1 == 0);
    assert(p.push(11, m) && m.status == 0xC3);
    assert(!p.push(0xF0, m)); // SysEx cancels running status
    assert(!p.push(61, m));
    assert(p.push(0xFC, m) && m.status == 0xFC);
    assert(!p.push(0xF7, m));
    assert(!p.push(100, m));
    assert(!p.push(0xB1, m));
    assert(!p.push(120, m));
    assert(!p.push(0xF9, m)); // Undefined realtime does not reset pending CC
    assert(p.push(0, m) && m.status == 0xB1 && m.data0 == 120);
    assert(!p.push(64, m));
    p.reset();
    assert(!p.push(127, m));
    assert(midiInputMessageValid({0x80, 60, 0}));
    assert(midiInputMessageValid({0xFA, 0, 0}));
    assert(!midiInputMessageValid({0xF9, 0, 0}));
    assert(!midiInputMessageValid({0x90, 128, 0}));
    assert(!midiInputMessageValid({0x01, 60, 1}));
}
