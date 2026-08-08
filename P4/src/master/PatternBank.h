#pragma once

#include "Sequencer.h"

static constexpr uint8_t BUILTIN_PATTERN_COUNT = 16;
static constexpr uint8_t FACTORY_PATTERN_COUNT = 20;
static constexpr uint8_t BUILTIN_ENGINE_COUNT = 9;

struct BuiltinPatternSoundProfile {
  int8_t engines[MAX_TRACKS];
  uint8_t presets[BUILTIN_ENGINE_COUNT];
};

void initializeProfessionalPatternBank(Sequencer& sequencer);
bool getBuiltinPatternSoundProfile(int pattern, BuiltinPatternSoundProfile& out);
void resetPatternSoundProfiles(void);
void setPatternSoundProfile(int pattern, const BuiltinPatternSoundProfile& profile);
uint8_t getConfiguredPatternCount(void);

// Musical cleanup applied only to the shipped 20-pattern JSON bank. Pad 8 is
// the generated 303 voice there; these lines replace the old 13/16-step gate
// wall with deliberate, velocity-shaped phrases.
void refineFactoryTwentyPatternBank(Sequencer& sequencer);
