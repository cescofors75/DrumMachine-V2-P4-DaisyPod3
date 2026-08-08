#pragma once

// Internal UI event identifiers retained from BlueSlaveP4. They no longer
// describe a wire protocol; local_apply_message() handles them in-process.
#define MSG_SYSTEM       0x04
#define MSG_TRACK        0x06
#define MSG_TOUCH_CMD    0x08
#define MSG_PATTERN_PUSH 0x0C

#define SYS_PLAY_STATE 0x03
#define SYS_STEP       0x04

#define TRK_MUTE_BIT 0x00
#define TRK_VOLUME   0x20

#define TCMD_PAD_TAP       0x00
#define TCMD_PATTERN_SEL   0x02
#define TCMD_THEME_NEXT    0x04
#define TCMD_SYNC_PADS     0x0B
#define TCMD_MELODY_ENGINE 0x12
#define TCMD_MELODY_OCTAVE 0x13
#define TCMD_MELODY_REC    0x14
#define TCMD_MELODY_PAD    0x16
#define TCMD_MELODY_NOTE   0x17
