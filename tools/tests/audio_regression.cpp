#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include "../../shared/step_timing.h"
#include "../../shared/pattern_transfer.h"
#include "../../DaisyPod3/synth/tr808.h"
#include "../../DaisyPod3/synth/tr909.h"
#include "../../DaisyPod3/synth/tr505.h"

template<class Kit> void checkKit(const char* name) {
    Kit kit;
    kit.Init(48000);
    kit.Trigger(0, 1.f);
    double energy = 0;
    for(int i = 0; i < 4800; ++i) {
        float channels[16];
        float mix = kit.Process(channels), sum = 0;
        for(int ch = 0; ch < 16; ++ch) {
            assert(std::isfinite(channels[ch]));
            if(ch != 0) assert(channels[ch] == 0.f);
            sum += channels[ch];
        }
        assert(std::fabs(sum - mix) < 1e-5f);
        energy += std::fabs(channels[0]);
    }
    assert(energy > 1);
    kit.Init(48000);
    for(int ch = 0; ch < 16; ++ch) kit.Trigger(ch, .8f);
    for(int i = 0; i < 4800; ++i) {
        float channels[16], sum = 0;
        float mix = kit.Process(channels);
        for(float value : channels) { assert(std::isfinite(value)); sum += value; }
        assert(std::fabs(sum - mix) < 1e-5f);
    }
    std::printf("%s: isolated kick, full kit, sum(outputs)==legacy mix PASS\n", name);
}
template<class Kit> void checkPcm() {
    Kit kit; kit.Init(48000);
    const int16_t samples[8] = {16000, 8000, -4000, -2000, 1000, 500, 250, 0};
    assert(kit.SetPcmSample(0, samples, 8));
    kit.Trigger(0, 1.f);
    float energy = 0;
    for(int i = 0; i < 16; ++i) {
        float out[16]; const float sum = kit.Process(out);
        assert(std::fabs(sum - out[0]) < 1e-5f);
        for(int c = 1; c < 16; ++c) assert(out[c] == 0.f);
        energy += std::fabs(out[0]);
    }
    assert(energy > .01f);
}
int main() {
    unsigned cases = 0;
    for(uint32_t bpm = 40; bpm <= 300; ++bpm) for(uint8_t swing = 0; swing <= 100; ++swing) {
        const uint32_t base = 720000 / bpm;
        const uint32_t offset = SwingOffset(base, swing);
        assert((base + offset) + (base - offset) == 2 * base);
        for(uint8_t phase = 0; phase < 2; ++phase) for(uint8_t count = 1; count <= 4; ++count) {
            const uint32_t duration = phase ? base - offset : base + offset;
            for(uint32_t humanize : {0u, 240u, 960u, 50000u}) {
                uint32_t delay = humanize;
                uint32_t interval = RatchetInterval(duration, delay, count);
                assert(interval > 0);
                assert(delay + (count - 1) * interval < duration);
                ++cases;
            }
        }
    }
    for(uint8_t ratchet = 1; ratchet <= 4; ++ratchet) for(uint8_t div = 1; div <= 15; ++div) {
        const auto wire = PackStepDivision(div, ratchet);
        assert((wire & 15) == div && ((wire >> 4) + 1) == ratchet);
    }
    const char data[] = "hello";
    assert(PatternHash(data, 5) == 0x4f9f2cabu);
    assert(PatternHash(data + 2, 3, PatternHash(data, 2)) == PatternHash(data, 5));
    PatternWireStep wire[16] = {};
    for(int drop = -1; drop < 16; ++drop) {
        PatternTransferCheck transfer;
        transfer.begin(4, 99);
        uint32_t hash = 2166136261u;
        for(uint8_t track = 0; track < 16; ++track) {
            wire[0].velocity = track;
            hash = PatternHash(wire, sizeof(wire), hash);
            if(track != drop) transfer.track(4, 99, track, wire, sizeof(wire));
        }
        assert(transfer.ready(4, 99, hash) == (drop == -1));
        assert(!transfer.ready(4, 100, hash));
        assert(!transfer.ready(4, 99, hash ^ 1u));
        assert(!transfer.ready(5, 99, hash));
    }
    std::puts("Transfer: complete upload, each missing track, stale token, bad checksum PASS");
    std::printf("Timing: %u combinations PASS; division packing and checksum PASS\n", cases);
    checkKit<TR808::Kit>("808");
    checkKit<TR909::Kit>("909");
    checkKit<TR505::Kit>("505");
    checkPcm<TR909::Kit>();
    checkPcm<TR505::Kit>();
    std::puts("909/505 PCM individual output PASS");
}
