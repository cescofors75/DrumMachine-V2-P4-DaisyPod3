#pragma once
#include <stdint.h>

// Short MIDI messages, normalized to an explicit status (no running status
// crosses USB packet boundaries). SysEx is deliberately not forwarded.
struct MidiInputMessage { uint8_t status, data0, data1; };
static_assert(sizeof(MidiInputMessage) == 3, "MIDI USB wire size");

class MidiInputParser {
public:
    void reset() { status_ = count_ = first_ = 0; }
    bool push(uint8_t byte, MidiInputMessage& out) {
        // Realtime can interrupt any message, including SysEx.
        if(byte >= 0xF8) {
            if(byte == 0xF9 || byte == 0xFD) return false;
            out = {byte, 0, 0};
            return true;
        }
        if(byte & 0x80) {
            count_ = 0;
            status_ = byte < 0xF0 ? byte : 0;
            return false;
        }
        if(!status_) return false;
        const uint8_t kind = status_ & 0xF0;
        if(kind == 0xC0 || kind == 0xD0) {
            out = {status_, byte, 0};
            return true;
        }
        if(!count_) { first_ = byte; count_ = 1; return false; }
        count_ = 0;
        out = {status_, first_, byte};
        return true;
    }
private:
    uint8_t status_ = 0, count_ = 0, first_ = 0;
};

inline bool midiInputMessageValid(const MidiInputMessage& m) {
    if(m.data0 >= 128 || m.data1 >= 128) return false;
    if(m.status >= 0x80 && m.status < 0xF0) return true;
    return m.status == 0xF8 || m.status == 0xFA || m.status == 0xFB
        || m.status == 0xFC || m.status == 0xFE || m.status == 0xFF;
}
