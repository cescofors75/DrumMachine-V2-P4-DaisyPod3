// Standalone host test; no Daisy SDK or existing regression runner required.
// From the repository root:
// g++ -std=c++14 -O2 tools/tests/drum_idle_regression.cpp -o drum_idle_regression.exe
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <cstdio>
#include "../../DaisyPod3/synth/tr808.h"
#include "../../DaisyPod3/synth/tr909.h"
#include "../../DaisyPod3/synth/tr505.h"

namespace {
constexpr int kChannels = 16;
constexpr int kMaxTail = 48000 * 30;
constexpr uint8_t kClosedHat = 3;
constexpr uint8_t kOpenHat = 4;

template<class Kit>
float ProcessChecked(Kit& kit, float* outputs) {
    const bool before = kit.IsActive(); // Caller tests this BEFORE Process().
    assert(before || kit.ActiveCount() == 0);
    float guarded[kChannels + 2];
    for (float& value : guarded) value = 12345.0f;
    const float mix = kit.Process(guarded + 1);
    assert(guarded[0] == 12345.0f && guarded[kChannels + 1] == 12345.0f);
    assert(std::isfinite(mix));
    float sum = 0.0f;
    for (int i = 0; i < kChannels; ++i) {
        outputs[i] = guarded[i + 1];
        assert(std::isfinite(outputs[i]));
        sum += outputs[i];
        if (outputs[i] != 0.0f) {
            assert(before);
            assert(kit.IsActive()); // Also retain the final nonzero sample.
        }
        if (!before) assert(outputs[i] == 0.0f);
    }
    assert(std::fabs(sum - mix) < 1e-5f);
    assert(kit.IsActive() || kit.ActiveCount() == 0);
    return mix;
}

template<class Kit>
void CheckIdle(Kit& kit) {
    assert(!kit.IsActive());
    assert(kit.ActiveCount() == 0);
    float outputs[kChannels];
    for (int i = 0; i < 8; ++i) {
        assert(ProcessChecked(kit, outputs) == 0.0f);
        for (float value : outputs) assert(value == 0.0f);
        assert(kit.Process() == 0.0f); // Null outputs fast path.
        assert(!kit.IsActive());
    }
}

template<class Kit>
double Drain(Kit& kit, int isolatedChannel = -1) {
    double energy = 0.0;
    int frames = 0;
    while (kit.IsActive()) {
        assert(++frames < kMaxTail);
        float outputs[kChannels];
        ProcessChecked(kit, outputs);
        for (int i = 0; i < kChannels; ++i) {
            if (isolatedChannel >= 0 && i != isolatedChannel)
                assert(outputs[i] == 0.0f);
            energy += std::fabs(outputs[i]);
        }
    }
    CheckIdle(kit);
    return energy;
}

template<class Kit>
void CheckProcedural(const char* name) {
    Kit kit;
    kit.Init(48000.0f);
    CheckIdle(kit);
    kit.Trigger(16);
    kit.Trigger(255);
    CheckIdle(kit);

    for (uint8_t id = 0; id < kChannels; ++id) {
        kit.Init(48000.0f);
        kit.Trigger(id, 0.8f);
        assert(kit.IsActive());
        assert(Drain(kit, id) > 0.0);
        kit.Trigger(id); // Wake again after retirement.
        assert(kit.IsActive());
        assert(Drain(kit, id) > 0.0);
    }

    // Compare every kick sample against the unguarded voice, including the
    // sample where its own IsActive() becomes false (505 may quantize it to 0).
    kit.Init(48000.0f);
    kit.SetMasterVolume(0.1f); // Keep this comparison below limiter threshold.
    auto referenceKick = kit.kick;
    referenceKick.Trigger();
    kit.Trigger(0);
    int kickFrames = 0;
    while (referenceKick.IsActive()) {
        assert(++kickFrames < kMaxTail);
        const float expected = referenceKick.Process() * 0.1f;
        float outputs[kChannels];
        assert(std::fabs(ProcessChecked(kit, outputs) - expected) < 1e-6f);
        assert(kit.IsActive());
    }
    assert(!kit.kick.IsActive());
    Drain(kit);

    kit.Init(48000.0f);
    for (uint8_t id = 0; id < kChannels; ++id) kit.Trigger(id);
    Kit sumOnly = kit;
    for (int i = 0; i < 512; ++i) {
        float outputs[kChannels];
        assert(std::fabs(ProcessChecked(kit, outputs) - sumOnly.Process()) < 1e-6f);
        assert(kit.IsActive() == sumOnly.IsActive());
    }
    assert(Drain(kit) > 0.0);

    // Mute, zero gain, and zero velocity must not stop envelope advancement.
    for (int mode = 0; mode < 3; ++mode) {
        kit.Init(48000.0f);
        if (mode == 0) kit.SetMute(0, true);
        if (mode == 1) kit.SetMasterVolume(0.0f);
        kit.Trigger(0, mode == 2 ? 0.0f : 1.0f);
        assert(kit.IsActive());
        assert(Drain(kit) == 0.0);
    }
    kit.Trigger(0);
    kit.Init(48000.0f); // Re-init cancels voices and cached activity.
    CheckIdle(kit);

    kit.Trigger(kOpenHat);
    for (int i = 0; i < 32; ++i) kit.Process();
    kit.Trigger(kClosedHat);
    assert(kit.IsActive());
    // 909 has a short choke fade; 808/505 choke immediately.
    for (int i = 0; i < 256; ++i) {
        float outputs[kChannels];
        ProcessChecked(kit, outputs);
    }
    assert(!kit.hihatO.IsActive());
    Drain(kit);
    kit.Trigger(kOpenHat);
    assert(kit.IsActive());
    assert(Drain(kit, kOpenHat) > 0.0);
    std::printf("%s: procedural lifecycle, silence, sum and hat choke PASS\n", name);
}

template<class Kit>
void CheckLimiterRelease(const char* name) {
    Kit idle;
    idle.Init(48000.0f);
    idle.hihatO.volume = 10000.0f; // Charge the peak follower well above threshold.
    idle.Trigger(kOpenHat);
    bool limited = false;
    for (int i = 0; i < 64; ++i)
        limited = (std::fabs(idle.Process()) > 0.9f) || limited;
    assert(limited);
    idle.hihatO.Choke();
    Drain(idle);
    idle.hihatO.volume = 1.0f;
    Kit busy = idle;
    Kit frozen = idle;
    busy.Trigger(0, 0.0f); // Same limiter release via the normal rendering path.
    for (int i = 0; i < 512; ++i) {
        float outputs[kChannels];
        assert(ProcessChecked(idle, outputs) == 0.0f);
        assert(busy.Process() == 0.0f);
        assert(busy.IsActive());
        assert(!idle.IsActive());
    }
    idle.Trigger(kOpenHat);
    busy.Trigger(kOpenHat);
    frozen.Trigger(kOpenHat);
    double releasedEnergy = 0.0, frozenEnergy = 0.0;
    for (int i = 0; i < 64; ++i) {
        float outputs[kChannels];
        const float sample = ProcessChecked(idle, outputs);
        assert(std::fabs(sample - busy.Process()) < 1e-6f);
        releasedEnergy += std::fabs(sample);
        frozenEnergy += std::fabs(frozen.Process());
    }
    assert(frozenEnergy > 0.0);
    assert(releasedEnergy > frozenEnergy * 1.2);
    std::printf("%s: idle limiter release matches active silent rendering PASS\n", name);
}

template<class Kit>
void CheckPcm(const char* name, bool fadeChoke) {
    Kit kit;
    const int16_t clip[] = {0, 0, 8192, -4096, 4096};
    const int16_t one[] = {16384};
    const float rates[] = {24000.0f, 48000.0f, 96000.0f};
    for (uint8_t id = 0; id < kChannels; ++id) {
        for (float rate : rates) {
            kit.Init(48000.0f);
            assert(kit.SetPcmSample(id, clip, 5, rate));
            CheckIdle(kit); // Binding a sample alone must not wake the kit.
            kit.Trigger(id);
            assert(kit.IsActive());
            assert(Drain(kit, id) > 0.0); // Leading zeros are not end-of-sample.
            kit.Trigger(id);
            assert(kit.IsActive());
            assert(Drain(kit, id) > 0.0);
        }
    }

    kit.Init(48000.0f);
    assert(kit.SetPcmSample(0, one, 1));
    kit.Trigger(0);
    float outputs[kChannels];
    assert(ProcessChecked(kit, outputs) > 0.0f);
    assert(kit.ActiveCount() == 0);
    assert(kit.IsActive()); // Last PCM sample was emitted on deactivation.
    assert(ProcessChecked(kit, outputs) == 0.0f);
    CheckIdle(kit);

    kit.Trigger(0);
    assert(!kit.SetPcmSample(0, nullptr, 1));
    assert(!kit.SetPcmSample(0, one, 0));
    assert(!kit.SetPcmSample(255, one, 1));
    kit.ClearPcmSample(255);
    assert(kit.IsActive());
    assert(kit.SetPcmSample(0, clip, 5)); // Replacement cancels playback.
    assert(kit.Process() == 0.f);
    CheckIdle(kit);
    kit.Trigger(0);
    kit.ClearPcmSample(0);
    assert(kit.Process() == 0.f);
    CheckIdle(kit);
    kit.Trigger(0); // Procedural fallback after PCM removal.
    assert(kit.IsActive());
    assert(Drain(kit, 0) > 0.0);

    kit.Init(48000.0f);
    assert(kit.SetPcmSample(0, clip, 5));
    assert(kit.SetPcmSample(1, clip, 5));
    kit.Trigger(0);
    kit.Trigger(1);
    kit.ClearPcmSample(0);
    assert(kit.IsActive()); // Another PCM slot is still playing.
    assert(Drain(kit, 1) > 0.0);
    kit.Trigger(1);
    kit.ClearPcmSamples();
    assert(kit.Process() == 0.f);
    CheckIdle(kit);

    kit.Trigger(0); // Preserve a procedural voice under a PCM overlay.
    assert(kit.SetPcmSample(0, clip, 5));
    kit.Trigger(0);
    kit.ClearPcmSample(0);
    assert(kit.IsActive() && kit.kick.IsActive());
    assert(kit.SetPcmSample(1, clip, 5));
    kit.Trigger(1);
    kit.ClearPcmSamples();
    assert(kit.IsActive() && kit.kick.IsActive());
    assert(Drain(kit, 0) > 0.0);

    int16_t longHat[1024];
    for (int16_t& value : longHat) value = 8192;
    kit.Init(48000.0f);
    assert(kit.SetPcmSample(kOpenHat, longHat, 1024));
    assert(kit.SetPcmSample(kClosedHat, one, 1));
    kit.Trigger(kOpenHat);
    kit.Process();
    kit.Trigger(kClosedHat);
    assert(kit.IsActive());
    ProcessChecked(kit, outputs);
    assert((outputs[kOpenHat] > 0.0f) == fadeChoke);
    int frames = 0;
    while (kit.IsActive()) {
        assert(++frames < 256); // Choke, not the 1024-frame sample end.
        ProcessChecked(kit, outputs);
    }
    CheckIdle(kit);
    kit.Trigger(kOpenHat);
    assert(kit.IsActive());
    ProcessChecked(kit, outputs);
    assert(std::fabs(outputs[kOpenHat] - 0.25f * 0.92f) < 1e-6f);
    assert(Drain(kit, kOpenHat) > 0.0);
    kit.Trigger(kOpenHat);
    kit.Init(48000.0f);
    assert(!kit.HasPcmSample(kOpenHat));
    CheckIdle(kit);
    std::printf("%s: PCM lifecycle, final sample, clear/replace and choke PASS\n", name);
}
} // namespace

int main() {
    CheckProcedural<TR808::Kit>("808");
    CheckProcedural<TR909::Kit>("909");
    CheckProcedural<TR505::Kit>("505");
    CheckLimiterRelease<TR808::Kit>("808");
    CheckLimiterRelease<TR909::Kit>("909");
    CheckLimiterRelease<TR505::Kit>("505");
    CheckPcm<TR909::Kit>("909", true);
    CheckPcm<TR505::Kit>("505", false);
    std::puts("Drum idle regression PASS");
}