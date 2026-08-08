#ifndef RED808_PROTOCOL_CODES_H
#define RED808_PROTOCOL_CODES_H

#include <stdint.h>

#define SPI_MAGIC_CMD      0xA5
#define SPI_MAGIC_RESP     0x5A
#define SPI_MAGIC_SAMPLE   0xDA
#define SPI_MAGIC_BULK     0xBB

#define CMD_TRIGGER_SEQ       0x01
#define CMD_TRIGGER_LIVE      0x02
#define CMD_TRIGGER_STOP      0x03
#define CMD_TRIGGER_STOP_ALL  0x04
#define CMD_TRIGGER_SIDECHAIN 0x05

#define CMD_MASTER_VOLUME     0x10
#define CMD_SEQ_VOLUME        0x11
#define CMD_LIVE_VOLUME       0x12
#define CMD_TRACK_VOLUME      0x13
#define CMD_LIVE_PITCH        0x14
#define CMD_TEMPO             0x15

#define CMD_FILTER_SET        0x20
#define CMD_FILTER_CUTOFF     0x21
#define CMD_FILTER_RESONANCE  0x22
#define CMD_FILTER_BITDEPTH   0x23
#define CMD_FILTER_DISTORTION 0x24
#define CMD_FILTER_DIST_MODE  0x25
#define CMD_FILTER_SR_REDUCE  0x26
#define CMD_MASTER_FX_ROUTE   0x27

#define CMD_DELAY_ACTIVE      0x30
#define CMD_DELAY_TIME        0x31
#define CMD_DELAY_FEEDBACK    0x32
#define CMD_DELAY_MIX         0x33
#define CMD_PHASER_ACTIVE     0x34
#define CMD_PHASER_RATE       0x35
#define CMD_PHASER_DEPTH      0x36
#define CMD_PHASER_FEEDBACK   0x37
#define CMD_FLANGER_ACTIVE    0x38
#define CMD_FLANGER_RATE      0x39
#define CMD_FLANGER_DEPTH     0x3A
#define CMD_FLANGER_FEEDBACK  0x3B
#define CMD_FLANGER_MIX       0x3C
#define CMD_COMP_ACTIVE       0x3D
#define CMD_COMP_THRESHOLD    0x3E
#define CMD_COMP_RATIO        0x3F
#define CMD_COMP_ATTACK       0x40
#define CMD_COMP_RELEASE      0x41
#define CMD_COMP_MAKEUP       0x42
#define CMD_REVERB_ACTIVE     0x43
#define CMD_REVERB_FEEDBACK   0x44
#define CMD_REVERB_LPFREQ     0x45
#define CMD_REVERB_MIX        0x46
#define CMD_CHORUS_ACTIVE     0x47
#define CMD_CHORUS_RATE       0x48
#define CMD_CHORUS_DEPTH      0x49
#define CMD_CHORUS_MIX        0x4A
#define CMD_TREMOLO_ACTIVE    0x4B
#define CMD_TREMOLO_RATE      0x4C
#define CMD_TREMOLO_DEPTH     0x4D
#define CMD_WAVEFOLDER_GAIN   0x4E
#define CMD_LIMITER_ACTIVE    0x4F

#define CMD_TRACK_FILTER       0x50
#define CMD_TRACK_CLEAR_FILTER 0x51
#define CMD_TRACK_DISTORTION   0x52
#define CMD_TRACK_BITCRUSH     0x53
#define CMD_TRACK_ECHO         0x54
#define CMD_TRACK_FLANGER_FX   0x55
#define CMD_TRACK_COMPRESSOR   0x56
#define CMD_TRACK_CLEAR_LIVE   0x57
#define CMD_TRACK_CLEAR_FX     0x58
#define CMD_TRACK_REVERB_SEND  0x59
#define CMD_TRACK_DELAY_SEND   0x5A
#define CMD_TRACK_CHORUS_SEND  0x5B
#define CMD_TRACK_PAN          0x5C
#define CMD_TRACK_MUTE         0x5D
#define CMD_TRACK_SOLO         0x5E
#define CMD_TRACK_PHASER       0x5F
#define CMD_TRACK_TREMOLO      0x60
#define CMD_TRACK_PITCH        0x61
#define CMD_TRACK_GATE         0x62
#define CMD_TRACK_EQ_LOW       0x63
#define CMD_TRACK_EQ_MID       0x64
#define CMD_TRACK_EQ_HIGH      0x65
#define CMD_TRACK_FX_ROUTE     0x66
#define CMD_TRACK_LFO_CONFIG   0x67

#define CMD_PAD_FILTER       0x70
#define CMD_PAD_CLEAR_FILTER 0x71
#define CMD_PAD_DISTORTION   0x72
#define CMD_PAD_BITCRUSH     0x73
#define CMD_PAD_LOOP         0x74
#define CMD_PAD_REVERSE      0x75
#define CMD_PAD_PITCH        0x76
#define CMD_PAD_STUTTER      0x77
#define CMD_PAD_SCRATCH      0x78
#define CMD_PAD_TURNTABLISM  0x79
#define CMD_PAD_CLEAR_FX     0x7A

#define CMD_PAD_LFO_ACTIVE  0x80
#define CMD_PAD_LFO_WAVE    0x81
#define CMD_PAD_LFO_RATE    0x82
#define CMD_PAD_LFO_DEPTH   0x83
#define CMD_PAD_LFO_TARGET  0x84
#define CMD_PAD_LFO_FREE_HZ 0x85
#define CMD_PAD_LFO_PHASE   0x86
#define CMD_PAD_LFO_RETRIG  0x87

#define CMD_SIDECHAIN_SET   0x90
#define CMD_SIDECHAIN_CLEAR 0x91

#define CMD_SAMPLE_BEGIN      0xA0
#define CMD_SAMPLE_DATA       0xA1
#define CMD_SAMPLE_END        0xA2
#define CMD_SAMPLE_UNLOAD     0xA3
#define CMD_SAMPLE_UNLOAD_ALL 0xA4
#define CMD_AUTOWAH_ACTIVE    0xA5
#define CMD_AUTOWAH_LEVEL     0xA6
#define CMD_AUTOWAH_MIX       0xA7
#define CMD_STEREO_WIDTH      0xA8
#define CMD_TAPE_STOP         0xA9
#define CMD_BEAT_REPEAT       0xAA
#define CMD_DELAY_STEREO      0xAB
#define CMD_CHORUS_STEREO     0xAC
#define CMD_EARLY_REF_ACTIVE  0xAD
#define CMD_EARLY_REF_MIX     0xAE
#define CMD_CHOKE_GROUP       0xAF

#define CMD_SD_LIST_FOLDERS 0xB0
#define CMD_SD_LIST_FILES   0xB1
#define CMD_SD_FILE_INFO    0xB2
#define CMD_SD_LOAD_SAMPLE  0xB3
#define CMD_SD_LOAD_KIT     0xB4
#define CMD_SD_KIT_LIST     0xB5
#define CMD_SD_STATUS       0xB6
#define CMD_SD_UNLOAD_KIT   0xB7
#define CMD_SD_GET_LOADED   0xB8
#define CMD_SD_ABORT        0xB9

#define CMD_SYNTH_TRIGGER    0xC0
#define CMD_SYNTH_PARAM      0xC1
#define CMD_SYNTH_NOTE_ON    0xC2
#define CMD_SYNTH_NOTE_OFF   0xC3
#define CMD_SYNTH_303_PARAM  0xC4
#define CMD_SYNTH_ACTIVE     0xC5
#define CMD_SYNTH_PRESET     0xC6
#define CMD_SYNTH_NOTE_ON_EX 0xC7

#define SYNTH_ENGINE_808   0
#define SYNTH_ENGINE_909   1
#define SYNTH_ENGINE_505   2
#define SYNTH_ENGINE_303   3
#define SYNTH_ENGINE_WTOSC 4
#define SYNTH_ENGINE_SH101 5
#define SYNTH_ENGINE_FM2OP 6
#define SYNTH_ENGINE_PHYS  7
#define SYNTH_ENGINE_NOISE 8
#define SYNTH_ENGINE_COUNT 9

#define CMD_DSQ_UPLOAD_TRACK     0xD0
#define CMD_DSQ_SET_STEP         0xD1
#define CMD_DSQ_CONTROL          0xD2
#define CMD_DSQ_SELECT_PATTERN   0xD3
#define CMD_DSQ_SET_LENGTH       0xD4
#define CMD_DSQ_SET_MUTE         0xD5
#define CMD_DSQ_GET_POS          0xD6
#define CMD_DSQ_SET_SWING        0xD7
#define CMD_DSQ_SET_PARAM_LOCK   0xD8
#define CMD_DSQ_SET_TRACK_ENGINE 0xD9
#define CMD_DSQ_SET_TRACK_SWING  0xDA
#define CMD_DSQ_SET_HUMANIZE     0xDB
#define CMD_CLEAN_TRACK_ACTIVE   0xDC
#define CMD_CLEAN_TRACK_MUTE     0xDD
#define CMD_DSQ_QUEUE_PATTERN    0xDF

#define CMD_GET_STATUS   0xE0
#define CMD_GET_PEAKS    0xE1
#define CMD_GET_CPU_LOAD 0xE2
#define CMD_GET_VOICES   0xE3
#define CMD_GET_EVENTS   0xE4
#define CMD_PING         0xEE
#define CMD_RESET        0xEF

#define CMD_BULK_TRIGGERS 0xF0
#define CMD_BULK_FX       0xF1
#define CMD_SONG_UPLOAD   0xF2
#define CMD_SONG_CONTROL  0xF3
#define CMD_SONG_GET_POS  0xF4

#define DSQ_PATTERNS  20
#define DSQ_TRACKS    16
#define DSQ_MAX_STEPS 64

#endif
