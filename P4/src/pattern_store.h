#pragma once

#include "master/Sequencer.h"

static constexpr int USER_PATTERN_FIRST = 100; // display slot P101

// Restore every valid P101..P128 file after the factory bank is initialized.
void pattern_store_load_user_bank(Sequencer& sequencer);

// Copy the complete source pattern (steps, locks, melody and sound profile) to
// a user slot and persist it in the P4 internal filesystem.
bool pattern_store_save_user(Sequencer& sequencer, int source, int destination);

bool pattern_store_is_user_slot(int pattern);
bool pattern_store_is_saved(int pattern);

