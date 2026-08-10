#include "app_state.h"

P4State p4 = {
    .enc_muted = {true, true, true},
    .pot_muted = {true, true, true},
    .filter_type = 0,
    .cutoff_hz = 20000,
    .resonance_x10 = 10,
    .distortion_pct = 0,
    .bitcrush_bits = 16,
    .sample_rate_hz = 0,
};

P4BootState p4boot = {};

P4SdState p4sd = {};

static std::atomic<uint32_t> pendingMelody{0};

void p4_publish_pending_melody(uint8_t engine, uint8_t octave,
                               bool recording, uint8_t pad)
{
    const uint32_t packed = 0x80000000u
        | static_cast<uint32_t>(engine)
        | (static_cast<uint32_t>(octave) << 8)
        | (static_cast<uint32_t>(recording ? 1u : 0u) << 16)
        | (static_cast<uint32_t>(pad) << 17);
    pendingMelody.store(packed, std::memory_order_release);
}

bool p4_consume_pending_melody(uint8_t* engine, uint8_t* octave,
                               bool* recording, uint8_t* pad)
{
    const uint32_t packed
        = pendingMelody.exchange(0, std::memory_order_acq_rel);
    if((packed & 0x80000000u) == 0)
        return false;
    if(engine) *engine = static_cast<uint8_t>(packed & 0xFFu);
    if(octave) *octave = static_cast<uint8_t>((packed >> 8) & 0xFFu);
    if(recording) *recording = ((packed >> 16) & 1u) != 0;
    if(pad) *pad = static_cast<uint8_t>((packed >> 17) & 0xFFu);
    return true;
}
