#pragma once

#include <stdint.h>

/*
 * RED808 factory map for the Akai MPD218.
 *
 * The MPD218 has 16 programs. RED808 uses programs 1..3 as BANK 1..3.
 * Every program contains three pad banks and three control banks (A/B/C).
 * Two controllers can share the H4MIDI merge bus. Their presets must use:
 *
 *   Program / RED808 bank   Controller A channel   Controller B channel
 *             1                       1                      4
 *             2                       2                      5
 *             3                       3                      6
 *
 *   Pad layer A: notes 36..51    Control layer A: CC 20..25
 *   Pad layer B: notes 52..67    Control layer B: CC 26..31
 *   Pad layer C: notes 68..83    Control layer C: CC 32..37
 *
 * Combining channel + note/CC gives every controller's 144 pad positions and
 * 54 knob positions a unique address without relying on bank-change messages.
 */
namespace red808_mpd218
{
constexpr uint8_t kDeviceCount     = 2;
constexpr uint8_t kBankCount       = 3;
constexpr uint8_t kLayerCount      = 3;
constexpr uint8_t kPadsPerLayer    = 16;
constexpr uint8_t kKnobsPerLayer   = 6;
constexpr uint8_t kFirstMidiChannel = 0; /* libDaisy channels are zero-based. */
constexpr uint8_t kChannelsPerDevice = kBankCount;

constexpr uint8_t kPadNoteBase[kLayerCount] = {36, 52, 68};
constexpr uint8_t kKnobCcBase[kLayerCount]  = {20, 26, 32};

enum PadActionType : uint8_t
{
    PAD_NONE = 0,
    PAD_TRIGGER_SAMPLE,
    PAD_TRIGGER_MELODIC,
    PAD_SELECT_PATTERN,
    PAD_TOGGLE_TRACK_MUTE,
    PAD_SELECT_TRACK,
    PAD_TRIGGER_SYNTH,
    PAD_PLAY_TOGGLE,
    PAD_STOP_ALL,
    PAD_PATTERN_PREV,
    PAD_PATTERN_NEXT,
    PAD_TOGGLE_LOOP,
    PAD_TOGGLE_REVERSE,
    PAD_TOGGLE_STUTTER,
    PAD_CLEAR_SELECTED_FX,
};

struct PadAction
{
    PadActionType type;
    uint8_t       arg0;
    uint8_t       arg1;
};

enum KnobActionType : uint8_t
{
    KNOB_NONE = 0,

    /* Bank 1: live mixer and master effects. */
    KNOB_MASTER_VOLUME,
    KNOB_LIVE_VOLUME,
    KNOB_SEQ_VOLUME,
    KNOB_TEMPO,
    KNOB_DELAY_MIX,
    KNOB_REVERB_MIX,
    KNOB_FILTER_CUTOFF,
    KNOB_FILTER_RESONANCE,
    KNOB_DISTORTION,
    KNOB_BIT_DEPTH,
    KNOB_SAMPLE_RATE,
    KNOB_FILTER_TYPE,
    KNOB_FLANGER_DEPTH,
    KNOB_PHASER_DEPTH,
    KNOB_WAVEFOLDER,
    KNOB_CRUSH_MACRO,
    KNOB_LIVE_PITCH,
    KNOB_SELECT_PAD,

    /* Bank 2: selected-track mixer/FX and sequencer. */
    KNOB_TRACK_VOLUME,
    KNOB_TRACK_PAN,
    KNOB_TRACK_REVERB_SEND,
    KNOB_TRACK_DELAY_SEND,
    KNOB_TRACK_CHORUS_SEND,
    KNOB_TRACK_PITCH,
    KNOB_TRACK_FILTER_CUTOFF,
    KNOB_TRACK_FILTER_RESONANCE,
    KNOB_TRACK_DISTORTION,
    KNOB_TRACK_BIT_DEPTH,
    KNOB_TRACK_EQ_LOW,
    KNOB_TRACK_EQ_HIGH,
    KNOB_SEQ_TEMPO,
    KNOB_SEQ_SWING,
    KNOB_HUMANIZE_TIMING,
    KNOB_HUMANIZE_VELOCITY,
    KNOB_PATTERN_SELECT,
    KNOB_TRACK_SELECT,

    /* Bank 3: selected drum/melodic synth and extra master effects. */
    KNOB_DRUM_PARAM_1,
    KNOB_DRUM_PARAM_2,
    KNOB_DRUM_PARAM_3,
    KNOB_DRUM_PARAM_4,
    KNOB_DRUM_PARAM_5,
    KNOB_DRUM_PARAM_6,
    KNOB_MELODIC_PARAM_1,
    KNOB_MELODIC_PARAM_2,
    KNOB_MELODIC_PARAM_3,
    KNOB_MELODIC_PARAM_4,
    KNOB_MELODIC_PARAM_5,
    KNOB_MELODIC_PARAM_6,
    KNOB_SYNTH_ENGINE,
    KNOB_SYNTH_PRESET,
    KNOB_CHORUS_MIX,
    KNOB_TREMOLO_DEPTH,
    KNOB_AUTOWAH_MIX,
    KNOB_STEREO_WIDTH,
};

struct KnobAction
{
    KnobActionType type;
};

#define MPD_PAD(action, a0, a1) {action, a0, a1}
#define MPD_SAMPLE(slot) MPD_PAD(PAD_TRIGGER_SAMPLE, slot, 0)
#define MPD_PATTERN(pattern) MPD_PAD(PAD_SELECT_PATTERN, pattern, 0)
#define MPD_MUTE(track) MPD_PAD(PAD_TOGGLE_TRACK_MUTE, track, 0)
#define MPD_SELECT_TRACK(track) MPD_PAD(PAD_SELECT_TRACK, track, 0)
#define MPD_SYNTH(engine, instrument) MPD_PAD(PAD_TRIGGER_SYNTH, engine, instrument)
#define MPD_MELODIC(note) MPD_PAD(PAD_TRIGGER_MELODIC, note, 0)

/*
 * PAD MAP
 * Bank 1: LIVE       A=main samples, B=xtra+transport+pad FX, C=melodic keyboard
 * Bank 2: SEQUENCER  A=patterns, B=track mutes, C=selected track
 * Bank 3: SYNTHS     A=TR-808, B=TR-909, C=TR-505
 */
constexpr PadAction kPadMap[kBankCount][kLayerCount][kPadsPerLayer] = {
    {
        {
            MPD_SAMPLE(0),  MPD_SAMPLE(1),  MPD_SAMPLE(2),  MPD_SAMPLE(3),
            MPD_SAMPLE(4),  MPD_SAMPLE(5),  MPD_SAMPLE(6),  MPD_SAMPLE(7),
            MPD_SAMPLE(8),  MPD_SAMPLE(9),  MPD_SAMPLE(10), MPD_SAMPLE(11),
            MPD_SAMPLE(12), MPD_SAMPLE(13), MPD_SAMPLE(14), MPD_SAMPLE(15),
        },
        {
            MPD_SAMPLE(16), MPD_SAMPLE(17), MPD_SAMPLE(18), MPD_SAMPLE(19),
            MPD_SAMPLE(20), MPD_SAMPLE(21), MPD_SAMPLE(22), MPD_SAMPLE(23),
            MPD_PAD(PAD_PLAY_TOGGLE, 0, 0),
            MPD_PAD(PAD_STOP_ALL, 0, 0),
            MPD_PAD(PAD_PATTERN_PREV, 0, 0),
            MPD_PAD(PAD_PATTERN_NEXT, 0, 0),
            MPD_PAD(PAD_TOGGLE_LOOP, 0, 0),
            MPD_PAD(PAD_TOGGLE_REVERSE, 0, 0),
            MPD_PAD(PAD_TOGGLE_STUTTER, 0, 0),
            MPD_PAD(PAD_CLEAR_SELECTED_FX, 0, 0),
        },
        {
            MPD_MELODIC(48), MPD_MELODIC(49), MPD_MELODIC(50), MPD_MELODIC(51),
            MPD_MELODIC(52), MPD_MELODIC(53), MPD_MELODIC(54), MPD_MELODIC(55),
            MPD_MELODIC(56), MPD_MELODIC(57), MPD_MELODIC(58), MPD_MELODIC(59),
            MPD_MELODIC(60), MPD_MELODIC(61), MPD_MELODIC(62), MPD_MELODIC(63),
        },
    },
    {
        {
            MPD_PATTERN(0),  MPD_PATTERN(1),  MPD_PATTERN(2),  MPD_PATTERN(3),
            MPD_PATTERN(4),  MPD_PATTERN(5),  MPD_PATTERN(6),  MPD_PATTERN(7),
            MPD_PATTERN(8),  MPD_PATTERN(9),  MPD_PATTERN(10), MPD_PATTERN(11),
            MPD_PATTERN(12), MPD_PATTERN(13), MPD_PATTERN(14), MPD_PATTERN(15),
        },
        {
            MPD_MUTE(0),  MPD_MUTE(1),  MPD_MUTE(2),  MPD_MUTE(3),
            MPD_MUTE(4),  MPD_MUTE(5),  MPD_MUTE(6),  MPD_MUTE(7),
            MPD_MUTE(8),  MPD_MUTE(9),  MPD_MUTE(10), MPD_MUTE(11),
            MPD_MUTE(12), MPD_MUTE(13), MPD_MUTE(14), MPD_MUTE(15),
        },
        {
            MPD_SELECT_TRACK(0),  MPD_SELECT_TRACK(1),
            MPD_SELECT_TRACK(2),  MPD_SELECT_TRACK(3),
            MPD_SELECT_TRACK(4),  MPD_SELECT_TRACK(5),
            MPD_SELECT_TRACK(6),  MPD_SELECT_TRACK(7),
            MPD_SELECT_TRACK(8),  MPD_SELECT_TRACK(9),
            MPD_SELECT_TRACK(10), MPD_SELECT_TRACK(11),
            MPD_SELECT_TRACK(12), MPD_SELECT_TRACK(13),
            MPD_SELECT_TRACK(14), MPD_SELECT_TRACK(15),
        },
    },
    {
        {
            MPD_SYNTH(0, 0),  MPD_SYNTH(0, 1),  MPD_SYNTH(0, 2),  MPD_SYNTH(0, 3),
            MPD_SYNTH(0, 4),  MPD_SYNTH(0, 5),  MPD_SYNTH(0, 6),  MPD_SYNTH(0, 7),
            MPD_SYNTH(0, 8),  MPD_SYNTH(0, 9),  MPD_SYNTH(0, 10), MPD_SYNTH(0, 11),
            MPD_SYNTH(0, 12), MPD_SYNTH(0, 13), MPD_SYNTH(0, 14), MPD_SYNTH(0, 15),
        },
        {
            MPD_SYNTH(1, 0),  MPD_SYNTH(1, 1),  MPD_SYNTH(1, 2),  MPD_SYNTH(1, 3),
            MPD_SYNTH(1, 4),  MPD_SYNTH(1, 5),  MPD_SYNTH(1, 6),  MPD_SYNTH(1, 7),
            MPD_SYNTH(1, 8),  MPD_SYNTH(1, 9),  MPD_SYNTH(1, 10), MPD_SYNTH(1, 11),
            MPD_SYNTH(1, 12), MPD_SYNTH(1, 13), MPD_SYNTH(1, 14), MPD_SYNTH(1, 15),
        },
        {
            MPD_SYNTH(2, 0),  MPD_SYNTH(2, 1),  MPD_SYNTH(2, 2),  MPD_SYNTH(2, 3),
            MPD_SYNTH(2, 4),  MPD_SYNTH(2, 5),  MPD_SYNTH(2, 6),  MPD_SYNTH(2, 7),
            MPD_SYNTH(2, 8),  MPD_SYNTH(2, 9),  MPD_SYNTH(2, 10), MPD_SYNTH(2, 11),
            MPD_SYNTH(2, 12), MPD_SYNTH(2, 13), MPD_SYNTH(2, 14), MPD_SYNTH(2, 15),
        },
    },
};

#define MPD_KNOB(action) {action}

constexpr KnobAction kKnobMap[kBankCount][kLayerCount][kKnobsPerLayer] = {
    {
        {MPD_KNOB(KNOB_MASTER_VOLUME), MPD_KNOB(KNOB_LIVE_VOLUME),
         MPD_KNOB(KNOB_SEQ_VOLUME), MPD_KNOB(KNOB_TEMPO),
         MPD_KNOB(KNOB_DELAY_MIX), MPD_KNOB(KNOB_REVERB_MIX)},
        {MPD_KNOB(KNOB_FILTER_CUTOFF), MPD_KNOB(KNOB_FILTER_RESONANCE),
         MPD_KNOB(KNOB_DISTORTION), MPD_KNOB(KNOB_BIT_DEPTH),
         MPD_KNOB(KNOB_SAMPLE_RATE), MPD_KNOB(KNOB_FILTER_TYPE)},
        {MPD_KNOB(KNOB_FLANGER_DEPTH), MPD_KNOB(KNOB_PHASER_DEPTH),
         MPD_KNOB(KNOB_WAVEFOLDER), MPD_KNOB(KNOB_CRUSH_MACRO),
         MPD_KNOB(KNOB_LIVE_PITCH), MPD_KNOB(KNOB_SELECT_PAD)},
    },
    {
        {MPD_KNOB(KNOB_TRACK_VOLUME), MPD_KNOB(KNOB_TRACK_PAN),
         MPD_KNOB(KNOB_TRACK_REVERB_SEND), MPD_KNOB(KNOB_TRACK_DELAY_SEND),
         MPD_KNOB(KNOB_TRACK_CHORUS_SEND), MPD_KNOB(KNOB_TRACK_PITCH)},
        {MPD_KNOB(KNOB_TRACK_FILTER_CUTOFF), MPD_KNOB(KNOB_TRACK_FILTER_RESONANCE),
         MPD_KNOB(KNOB_TRACK_DISTORTION), MPD_KNOB(KNOB_TRACK_BIT_DEPTH),
         MPD_KNOB(KNOB_TRACK_EQ_LOW), MPD_KNOB(KNOB_TRACK_EQ_HIGH)},
        {MPD_KNOB(KNOB_SEQ_TEMPO), MPD_KNOB(KNOB_SEQ_SWING),
         MPD_KNOB(KNOB_HUMANIZE_TIMING), MPD_KNOB(KNOB_HUMANIZE_VELOCITY),
         MPD_KNOB(KNOB_PATTERN_SELECT), MPD_KNOB(KNOB_TRACK_SELECT)},
    },
    {
        {MPD_KNOB(KNOB_DRUM_PARAM_1), MPD_KNOB(KNOB_DRUM_PARAM_2),
         MPD_KNOB(KNOB_DRUM_PARAM_3), MPD_KNOB(KNOB_DRUM_PARAM_4),
         MPD_KNOB(KNOB_DRUM_PARAM_5), MPD_KNOB(KNOB_DRUM_PARAM_6)},
        {MPD_KNOB(KNOB_MELODIC_PARAM_1), MPD_KNOB(KNOB_MELODIC_PARAM_2),
         MPD_KNOB(KNOB_MELODIC_PARAM_3), MPD_KNOB(KNOB_MELODIC_PARAM_4),
         MPD_KNOB(KNOB_MELODIC_PARAM_5), MPD_KNOB(KNOB_MELODIC_PARAM_6)},
        {MPD_KNOB(KNOB_SYNTH_ENGINE), MPD_KNOB(KNOB_SYNTH_PRESET),
         MPD_KNOB(KNOB_CHORUS_MIX), MPD_KNOB(KNOB_TREMOLO_DEPTH),
         MPD_KNOB(KNOB_AUTOWAH_MIX), MPD_KNOB(KNOB_STEREO_WIDTH)},
    },
};

#undef MPD_KNOB
#undef MPD_MELODIC
#undef MPD_SYNTH
#undef MPD_SELECT_TRACK
#undef MPD_MUTE
#undef MPD_PATTERN
#undef MPD_SAMPLE
#undef MPD_PAD

inline bool DecodeDeviceAndBank(uint8_t channel,
                                uint8_t& device,
                                uint8_t& bank)
{
    if(channel < kFirstMidiChannel
       || channel >= kFirstMidiChannel + kDeviceCount * kChannelsPerDevice)
        return false;
    const uint8_t relative = static_cast<uint8_t>(channel - kFirstMidiChannel);
    device = static_cast<uint8_t>(relative / kChannelsPerDevice);
    bank = static_cast<uint8_t>(relative % kChannelsPerDevice);
    return true;
}

inline bool DecodePad(uint8_t channel,
                      uint8_t note,
                      uint8_t& device,
                      uint8_t& bank,
                      uint8_t& layer,
                      uint8_t& pad)
{
    if(!DecodeDeviceAndBank(channel, device, bank))
        return false;
    for(uint8_t candidate = 0; candidate < kLayerCount; ++candidate)
    {
        const uint8_t base = kPadNoteBase[candidate];
        if(note >= base && note < base + kPadsPerLayer)
        {
            layer = candidate;
            pad = static_cast<uint8_t>(note - base);
            return true;
        }
    }
    return false;
}

inline bool DecodeKnob(uint8_t channel,
                       uint8_t cc,
                       uint8_t& device,
                       uint8_t& bank,
                       uint8_t& layer,
                       uint8_t& knob)
{
    if(!DecodeDeviceAndBank(channel, device, bank))
        return false;
    for(uint8_t candidate = 0; candidate < kLayerCount; ++candidate)
    {
        const uint8_t base = kKnobCcBase[candidate];
        if(cc >= base && cc < base + kKnobsPerLayer)
        {
            layer = candidate;
            knob = static_cast<uint8_t>(cc - base);
            return true;
        }
    }
    return false;
}

} // namespace red808_mpd218
