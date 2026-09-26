#pragma once
// Called only on the Arduino loop thread, alongside the USB transport.
void midi_din_begin();
void midi_din_process();
