#pragma once

#include "Sequencer.h"

static constexpr uint8_t BUILTIN_PATTERN_COUNT = 16;
// Slots 0..19: the original S3 JSON bank (untouched, still what "20
// patrones" refers to). Slots 20..99: the new genre expansion bank added
// on top of it. FACTORY_PATTERN_COUNT is the TOTAL now live on the device;
// LEGACY_FACTORY_PATTERN_COUNT is only the size of the old pad-8 refinement
// tables, which apply to the original 20 alone.
static constexpr uint8_t LEGACY_FACTORY_PATTERN_COUNT = 20;
static constexpr uint8_t FACTORY_PATTERN_COUNT = 100;
static constexpr uint8_t BUILTIN_ENGINE_COUNT = 9;

struct BuiltinPatternSoundProfile {
  int8_t engines[MAX_TRACKS];
  uint8_t presets[BUILTIN_ENGINE_COUNT];
};

void initializeProfessionalPatternBank(Sequencer& sequencer);
// Exact factory bank used by RedMaster_ESP32S3 after loading
// /patterns/20_patrones_factory_daisy.json, including its historical pad-8
// refinement. Embedded here so a normal P4 firmware flash is sufficient.
void initializeEsp32S3FactoryPatternBank(Sequencer& sequencer);
// Adds 80 new professional patterns in slots 20..99, ten genres x eight
// scenes each (one genre = one consecutive 8-pattern "song"). Leaves the
// original 20-pattern bank above completely untouched; call right after
// initializeEsp32S3FactoryPatternBank at boot.
void initializeFactoryExpansionBank(Sequencer& sequencer);
bool getBuiltinPatternSoundProfile(int pattern, BuiltinPatternSoundProfile& out);
void resetPatternSoundProfiles(void);
void setPatternSoundProfile(int pattern, const BuiltinPatternSoundProfile& profile);
uint8_t getConfiguredPatternCount(void);

// Musical cleanup applied only to the shipped 20-pattern JSON bank. Pad 8 is
// the generated 303 voice there; these lines replace the old 13/16-step gate
// wall with deliberate, velocity-shaped phrases.
void refineFactoryTwentyPatternBank(Sequencer& sequencer);
