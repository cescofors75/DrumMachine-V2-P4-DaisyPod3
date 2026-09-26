#include "midi_din.h"
#include "../include/config.h"
#include "daisy_usb_transport.h"
#include "../../shared/midi_input.h"
#include <HardwareSerial.h>

namespace {
HardwareSerial midiPort(1);
MidiInputParser parser;
bool ready = false;
bool wasConnected = false;
MidiInputMessage pending;
bool hasPending = false;
}

void midi_din_begin() {
#if P4_MIDI_RX_GPIO >= 0
    midiPort.setRxBufferSize(2048);
    midiPort.begin(31250, SERIAL_8N1, P4_MIDI_RX_GPIO, P4_MIDI_TX_GPIO);
    ready = true;
#endif
}

void midi_din_process() {
    if(!ready) return;
    const bool connected = daisyUsb.connected()
        && (daisyUsb.state().capability_flags & RED808_CAP_MIDI_INPUT);
    if(!connected || !wasConnected) {
        // Never replay notes played while the engine was disconnected.
        parser.reset();
        hasPending = false;
        while(midiPort.available()) midiPort.read();
        wasConnected = connected;
        return;
    }
    // Keep a complete message if USB is temporarily backpressured; never
    // split running status across retries or silently discard a note-off.
    unsigned budget = 192;
    while(budget--) {
        if(hasPending) {
            if(!daisyUsb.send(CMD_MIDI_INPUT, &pending, sizeof(pending))) return;
            hasPending = false;
        }
        if(!midiPort.available()) break;
        hasPending = parser.push(static_cast<uint8_t>(midiPort.read()), pending);
    }
}
