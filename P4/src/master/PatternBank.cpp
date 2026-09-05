#include "PatternBank.h"
#include "LegacyFactoryBank.generated.h"

#include <cstring>

namespace {
enum Track : uint8_t {
  BD = 0, SD, CH, OH, CY, CP, RS, CB, LT, MT, HT, MA, CL, HC, MC, LC
};

enum Engine : int8_t {
  SMP = -1, E808 = 0, E909, E505, E303, EWT, ESH, EFM, EPHYS, ENOISE
};

BuiltinPatternSoundProfile ACTIVE_SOUND_PROFILE[MAX_PATTERNS] = {};
bool ACTIVE_SOUND_PROFILE_VALID[MAX_PATTERNS] = {};

struct MetaSeed {
  const char* name;
  const char* genre;
  const char* kit;
  uint16_t bpm;
  uint8_t swing;
  uint8_t timing;
  uint8_t velocity;
};

/* One song, sixteen scenes. The factory bank now reads as a single live set
 * — the NIGHT DRIVE suite in F minor, everything at 124 BPM so a set can run
 * front to back without touching the tempo. The grooves the bank was loved
 * for (I-III, XII) are untouched; the rest are variations of the same
 * material, and scenes IX-X hand the spotlight to a piano lead (PHYS). */
constexpr MetaSeed META[BUILTIN_PATTERN_COUNT] = {
  {"I. Pulse Bloom",  "Suite: intro",     "RED 808 KARZ + SH", 124, 10, 1, 5},
  {"II. Transit",     "Suite: groove A",  "Samples + 909",     124,  4, 0, 3},
  {"III. Glass",      "Suite: groove B",  "Samples + 505",     124, 26, 2, 6},
  {"IV. Pressure",    "Suite: house",     "Samples + 303",     124, 12, 1, 4},
  {"V. Carbon",       "Suite: breaks",    "Samples + 505",     124,  2, 1, 6},
  {"VI. Motorik",     "Suite: drive",     "Samples + SH/WT",   124,  4, 1, 4},
  {"VII. Elastic",    "Suite: electro",   "Samples + 808",     124, 14, 1, 5},
  {"VIII. Gravity",   "Suite: breakdown", "Samples + SH/WT",   124, 18, 2, 7},
  {"IX. Piano Rise",  "Suite: piano",     "Smp + PHYS/WT",     124,  8, 1, 4},
  {"X. Piano Bloom",  "Suite: piano",     "Smp + PHYS/WT",     124,  8, 1, 4},
  {"XI. Voltage",     "Suite: peak drive","Samples + SH",      124,  6, 1, 4},
  {"XII. Organ Dust", "Suite: raw house", "Samples + 909/WT",  124, 14, 1, 4},
  {"XIII. Rupture",   "Suite: hard breaks","Samples + 909/SH", 124,  4, 1, 5},
  {"XIV. Afterglow",  "Suite: cooldown",  "Samples + FM/WT",   124, 20, 3, 8},
  {"XV. Peak Relay",  "Suite: peak",      "Samples + 909/505", 124,  2, 1, 5},
  {"XVI. Ritual",     "Suite: finale",    "Samples + 909/FM",  124,  4, 0, 3},
};

/* Sample-first contract. At least twelve tracks in every factory pattern use
 * the SD kit; generated machines are accent colours, never the whole kit. */
constexpr int8_t ENGINE_PROFILE[BUILTIN_PATTERN_COUNT][MAX_TRACKS] = {
  {SMP,SMP,SMP,SMP,E909,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,ESH,EWT},
  {SMP,SMP,SMP,E909,E909,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,EFM,ESH},
  {SMP,SMP,SMP,E505,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,ESH,EWT},
  {SMP,SMP,SMP,E909,E909,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,EWT,E303},
  {SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,E505,SMP,SMP,SMP,ESH,ENOISE},
  {SMP,SMP,SMP,SMP,E505,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,ESH,EWT},
  {SMP,SMP,SMP,SMP,SMP,SMP,SMP,E808,SMP,SMP,SMP,SMP,SMP,SMP,EFM,ESH},
  {SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,ESH,EWT},
  {SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,EWT,EPHYS},
  {SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,EWT,EPHYS},
  {SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,ESH},
  {SMP,SMP,SMP,E909,E909,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,EWT},
  {SMP,SMP,SMP,SMP,E909,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,ESH},
  {SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,EFM,EWT},
  {SMP,SMP,SMP,E909,SMP,SMP,SMP,SMP,SMP,SMP,E505,SMP,SMP,SMP,ESH,ENOISE},
  {SMP,SMP,SMP,E909,E909,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,EFM,ESH},
};

/* Preset order: 808, 909, 505, 303, WT, SH, FM, PHYS, NOISE. */
constexpr uint8_t PRESET_PROFILE[BUILTIN_PATTERN_COUNT][BUILTIN_ENGINE_COUNT] = {
  {1,1,1,0,3,2,1,0,0}, {1,1,1,0,1,2,2,0,0},
  {1,1,2,0,3,2,1,0,0}, {1,2,1,1,2,1,1,0,0},
  {1,1,2,0,1,2,1,0,1}, {1,1,1,0,3,2,1,0,0},
  {1,1,1,0,1,2,2,0,0}, {1,1,1,0,3,1,1,0,0},
  {1,1,1,0,1,1,1,0,0}, {1,1,1,0,1,1,1,0,0},
  {1,1,1,0,1,1,1,0,0}, {1,2,1,0,2,1,1,0,0},
  {1,1,1,0,1,1,1,0,0}, {1,1,1,0,3,1,1,0,0},
  {1,2,2,0,1,2,1,0,2}, {1,2,1,0,1,2,3,0,0},
};

void setMeta(Sequencer& seq, int pattern) {
  PatternMetadata meta{};
  strncpy(meta.name, META[pattern].name, sizeof(meta.name) - 1);
  strncpy(meta.genre, META[pattern].genre, sizeof(meta.genre) - 1);
  strncpy(meta.kit, META[pattern].kit, sizeof(meta.kit) - 1);
  meta.recommendedBpm = META[pattern].bpm;
  meta.swing = META[pattern].swing;
  meta.humanizeTimingMs = META[pattern].timing;
  meta.humanizeVelocity = META[pattern].velocity;
  seq.setPatternMetadata(pattern, meta);
}

void hit(Sequencer& seq, uint8_t track, uint8_t step, uint8_t velocity = 108,
         uint8_t probability = 100, uint8_t ratchet = 1, uint8_t noteLen = 1) {
  seq.setStep(track, step, true, velocity);
  seq.setStepProbability(track, step, probability);
  seq.setStepRatchet(track, step, ratchet);
  seq.setStepNoteLen(track, step, noteLen);
}

template <size_t N>
void hits(Sequencer& seq, uint8_t track, const uint8_t (&steps)[N],
          uint8_t velocity = 100, int8_t alternate = -7) {
  for (size_t i = 0; i < N; ++i) {
    int shaped = velocity + ((i & 1u) ? alternate : 0) + ((i % 4u == 0u) ? 5 : 0);
    if (shaped < 1) shaped = 1;
    if (shaped > 127) shaped = 127;
    hit(seq, track, steps[i], (uint8_t)shaped);
  }
}

void melodicHit(Sequencer& seq, uint8_t track, uint8_t step, uint8_t note,
                uint8_t velocity, uint8_t flags = 0, uint8_t probability = 100,
                uint8_t ratchet = 1, uint8_t noteLen = 1) {
  hit(seq, track, step, velocity, probability, ratchet, noteLen);
  seq.setStepNote(track, step, note);
  seq.clearStepNoteVoices(track, step);
  seq.setStepNoteVoice(track, step, 0, note);
  seq.setStepFlags(track, step, flags);
}

void chordHit(Sequencer& seq, uint8_t track, uint8_t step,
              uint8_t root, uint8_t third, uint8_t fifth,
              uint8_t velocity, uint8_t probability = 100) {
  hit(seq, track, step, velocity, probability, 1, 1);
  seq.setStepNote(track, step, root);
  seq.clearStepNoteVoices(track, step);
  seq.setStepNoteVoice(track, step, 0, root);
  seq.setStepNoteVoice(track, step, 1, third);
  seq.setStepNoteVoice(track, step, 2, fifth);
}

void fourFloor(Sequencer& seq, uint8_t velocity = 116) {
  const uint8_t steps[] = {0,4,8,12,16,20,24,28};
  hits(seq, BD, steps, velocity, -4);
}

void backbeat(Sequencer& seq, uint8_t velocity = 108) {
  const uint8_t steps[] = {4,12,20,28};
  hits(seq, SD, steps, velocity, -3);
}

void softOffbeats(Sequencer& seq, uint8_t velocity = 66) {
  const uint8_t steps[] = {2,6,10,14,18,22,26,30};
  hits(seq, CH, steps, velocity, -10);
}

void sampleMotion(Sequencer& seq, uint8_t track, uint16_t low, uint16_t high,
                  uint8_t reverbSend = 12) {
  seq.setStepCutoffLock(track, 0, true, low);
  seq.setStepCutoffLock(track, 16, true, high);
  seq.setStepReverbSendLock(track, 28, true, reverbSend);
}

// Track 7 is physical/display pad 8. The original factory JSON gated its 303
// on 13-16 of every 16 steps in most scenes. Apart from masking the drums, a
// stale engine assignment could make that wall sound like an endless hi-hat.
// A mask per scene gives every bass phrase air and keeps the density bounded.
constexpr uint16_t FACTORY_PAD8_MASK[LEGACY_FACTORY_PATTERN_COUNT] = {
  0x4949, 0x9595, 0x0000, 0x4D4D, 0x4921,
  0x9529, 0xA9A5, 0x0000, 0xA549, 0x4491,
  0x2121, 0x1081, 0x0000, 0x4441, 0xADAD,
  0xE595, 0x5555, 0x0000, 0x0000, 0xA525
};

constexpr uint8_t FACTORY_PAD8_ROOT[LEGACY_FACTORY_PATTERN_COUNT] = {
  36,36,36,36,36, 38,38,38,38,38, 41,41,41,41,36, 41,36,36,36,36
};

void rewriteFactoryPad8Line(Sequencer& seq, int pattern) {
  static constexpr uint8_t scale[] = {0,3,5,7,10,12,15,17};
  const uint16_t mask = FACTORY_PAD8_MASK[pattern];
  uint8_t phraseIndex = 0;
  for (uint8_t step = 0; step < 16; ++step) {
    seq.setStep(pattern, CB, step, false, 1);
    seq.setStepNote(pattern, CB, step, 0);
    seq.clearStepNoteVoices(pattern, CB, step);
    seq.setStepFlags(pattern, CB, step, 0);
    if ((mask & (uint16_t)(1u << step)) == 0) continue;

    const uint8_t note = FACTORY_PAD8_ROOT[pattern] +
        scale[(phraseIndex + (uint8_t)pattern) & 0x07u];
    int velocity = 62 + ((pattern >= 14) ? 18 : (pattern >= 5 ? 10 : 4));
    velocity += (step % 4 == 0) ? 24 : ((phraseIndex & 1u) ? -7 : 3);
    velocity = constrain(velocity, 45, 122);
    uint8_t flags = (step % 4 == 0) ? 0x01 : 0;
    if ((pattern == 3 || pattern == 8 || pattern >= 14) &&
        step > 0 && (mask & (uint16_t)(1u << (step - 1))) &&
        (phraseIndex % 3u == 2u)) {
      flags |= 0x02;
    }
    seq.setStep(pattern, CB, step, true, (uint8_t)velocity);
    seq.setStepNote(pattern, CB, step, note);
    seq.setStepNoteVoice(pattern, CB, step, 0, note);
    seq.setStepFlags(pattern, CB, step, flags);
    ++phraseIndex;
  }
}

void buildPattern(Sequencer& seq, int p) {
  seq.selectPattern(p);
  seq.clearPattern(p);
  setMeta(seq, p);

  switch (p) {
    case 0: { // Pulse Bloom — broken sample pocket with a single harmonic breath.
      const uint8_t k[] = {0,6,10,16,23,26};
      const uint8_t s[] = {4,12,20,28};
      const uint8_t h[] = {2,6,9,14,18,22,25,30};
      hits(seq, BD, k, 113); hits(seq, SD, s, 109); hits(seq, CH, h, 59, -12);
      hit(seq, SD, 11, 38, 48); hit(seq, SD, 27, 43, 58);
      hit(seq, OH, 15, 62, 76); hit(seq, OH, 31, 70, 86);
      melodicHit(seq, MC, 0, 29, 76); melodicHit(seq, MC, 10, 36, 62, 0, 86);
      melodicHit(seq, MC, 16, 27, 78); melodicHit(seq, MC, 26, 32, 67);
      chordHit(seq, LC, 0, 53,56,60, 42); chordHit(seq, LC, 16, 49,53,56, 38);
      sampleMotion(seq, CH, 4200, 7600, 10);
      break;
    }
    case 1: { // Night Transit — minimal pressure; FM answers only twice per bar.
      fourFloor(seq, 120); backbeat(seq, 105); softOffbeats(seq, 61);
      const uint8_t hats[] = {1,5,9,13,17,21,25,29}; hits(seq, OH, hats, 54, -8);
      hit(seq, RS, 11, 47, 62); hit(seq, RS, 27, 54, 72);
      melodicHit(seq, MC, 6, 55, 58, 0, 82); melodicHit(seq, MC, 22, 58, 64, 0, 88);
      melodicHit(seq, LC, 0, 29, 72); melodicHit(seq, LC, 10, 32, 61);
      melodicHit(seq, LC, 16, 29, 74); melodicHit(seq, LC, 27, 27, 64);
      sampleMotion(seq, CH, 6800, 11200, 7);
      break;
    }
    case 2: { // Glass Garage — two-step drums, quiet WT air and no kick wall.
      const uint8_t k[] = {0,7,10,16,22,27,30};
      const uint8_t c[] = {4,12,20,28};
      const uint8_t h[] = {1,3,6,9,11,14,17,19,22,25,27,30};
      hits(seq, BD, k, 114); hits(seq, CP, c, 106); hits(seq, CH, h, 55, -11);
      hit(seq, SD, 11, 40, 52); hit(seq, SD, 19, 44, 58);
      hit(seq, OH, 15, 64, 78); hit(seq, OH, 31, 72, 86);
      melodicHit(seq, MC, 0, 29, 72); melodicHit(seq, MC, 7, 36, 61);
      melodicHit(seq, MC, 16, 27, 74); melodicHit(seq, MC, 22, 39, 62, 0, 82);
      chordHit(seq, LC, 0, 53,56,60, 34); chordHit(seq, LC, 16, 51,55,58, 32);
      break;
    }
    case 3: { // Warm Pressure — sample house body with short acid punctuation.
      fourFloor(seq, 117); backbeat(seq, 107); softOffbeats(seq, 67);
      hit(seq, CP, 12, 72); hit(seq, CP, 28, 78);
      const uint8_t sh[] = {3,7,11,15,19,23,27,31}; hits(seq, MA, sh, 45, -8);
      chordHit(seq, MC, 2, 53,56,60, 48); chordHit(seq, MC, 18, 49,53,56, 45);
      melodicHit(seq, LC, 0, 29, 76, 1); melodicHit(seq, LC, 6, 36, 64);
      melodicHit(seq, LC, 13, 32, 68, 2, 88); melodicHit(seq, LC, 16, 29, 78, 1);
      melodicHit(seq, LC, 27, 39, 68, 2, 82);
      break;
    }
    case 4: { // Carbon Breaks — chopped samples; 505 toms only form the turnaround.
      const uint8_t k[] = {0,6,10,16,19,24,27,30};
      const uint8_t s[] = {4,12,20,25,28};
      const uint8_t h[] = {0,3,6,10,14,17,19,22,26,29,31};
      hits(seq, BD, k, 117); hits(seq, SD, s, 112); hits(seq, CH, h, 61, -13);
      hit(seq, SD, 15, 48, 58, 2); hit(seq, OH, 7, 64, 72);
      hit(seq, LT, 29, 62); hit(seq, MT, 30, 70); hit(seq, HT, 31, 82, 100, 2);
      melodicHit(seq, MC, 0, 29, 78); melodicHit(seq, MC, 10, 36, 67);
      melodicHit(seq, MC, 16, 27, 80); melodicHit(seq, MC, 27, 32, 69);
      melodicHit(seq, LC, 15, 72, 35, 0, 58); melodicHit(seq, LC, 31, 79, 42, 0, 72);
      break;
    }
    case 5: { // Neon Motorik — dry samples, restrained bass and two soft chords.
      const uint8_t k[] = {0,5,8,12,16,21,24,28};
      hits(seq, BD, k, 111); backbeat(seq, 110); softOffbeats(seq, 58);
      hit(seq, CP, 12, 65); hit(seq, CP, 28, 72);
      hit(seq, LT, 29, 54); hit(seq, MT, 30, 62); hit(seq, HT, 31, 72);
      melodicHit(seq, MC, 0, 29, 75); melodicHit(seq, MC, 8, 36, 62);
      melodicHit(seq, MC, 16, 32, 72); melodicHit(seq, MC, 24, 27, 66);
      chordHit(seq, LC, 0, 53,56,60, 40); chordHit(seq, LC, 16, 51,55,58, 37);
      seq.setStepReverbSendLock(SD, 28, true, 18);
      break;
    }
    case 6: { // Elastic Electro — syncopated sample chassis and tiny machine details.
      const uint8_t k[] = {0,3,7,10,16,19,23,27,30};
      const uint8_t s[] = {4,12,20,28};
      const uint8_t h[] = {2,6,10,14,18,22,26,30};
      hits(seq, BD, k, 117); hits(seq, SD, s, 108); hits(seq, CH, h, 62, -9);
      hit(seq, CB, 11, 46, 76); hit(seq, CB, 27, 50, 82);
      hit(seq, SD, 31, 66, 78, 2);
      melodicHit(seq, MC, 6, 60, 52, 0, 76); melodicHit(seq, MC, 22, 63, 58, 0, 84);
      melodicHit(seq, LC, 0, 29, 72); melodicHit(seq, LC, 7, 36, 65);
      melodicHit(seq, LC, 16, 27, 74); melodicHit(seq, LC, 26, 32, 70);
      break;
    }
    case 7: { // Low Gravity — half-time weight, dusty ghosts and long harmonic space.
      const uint8_t k[] = {0,6,10,16,23,27};
      const uint8_t s[] = {4,12,20,28};
      const uint8_t h[] = {2,6,9,14,18,22,25,30};
      hits(seq, BD, k, 108); hits(seq, SD, s, 103); hits(seq, CH, h, 48, -10);
      hit(seq, SD, 11, 34, 44); hit(seq, SD, 27, 38, 52);
      hit(seq, RS, 7, 38, 48); hit(seq, OH, 31, 58, 72);
      melodicHit(seq, MC, 0, 29, 72); melodicHit(seq, MC, 10, 36, 58);
      melodicHit(seq, MC, 16, 27, 74); melodicHit(seq, MC, 27, 32, 62);
      chordHit(seq, LC, 0, 53,56,60, 38); chordHit(seq, LC, 16, 49,53,56, 36);
      sampleMotion(seq, CH, 2800, 5200, 16);
      break;
    }
    case 8: { // IX. Piano Rise — the floor thins out and a piano takes the lead.
      const uint8_t k[] = {0,8,16,24};
      hits(seq, BD, k, 102, -2); softOffbeats(seq, 46);
      hit(seq, RS, 12, 40, 62); hit(seq, RS, 28, 44, 70);
      hit(seq, OH, 31, 56, 74);
      /* F minor pentatonic lead, legato phrasing (PHYS voice as the piano). */
      melodicHit(seq, LC, 0, 65, 84, 0, 100, 1, 2);
      melodicHit(seq, LC, 3, 68, 72, 0, 100, 1, 2);
      melodicHit(seq, LC, 6, 72, 78, 0, 100, 1, 2);
      melodicHit(seq, LC, 10, 70, 66, 0, 88, 1, 2);
      melodicHit(seq, LC, 14, 68, 62, 0, 100, 1, 2);
      melodicHit(seq, LC, 16, 65, 80, 0, 100, 1, 2);
      melodicHit(seq, LC, 19, 63, 68, 0, 100, 1, 2);
      melodicHit(seq, LC, 22, 68, 72, 0, 88, 1, 2);
      melodicHit(seq, LC, 26, 70, 74, 0, 100, 1, 2);
      melodicHit(seq, LC, 30, 63, 58, 0, 76, 1, 2);
      chordHit(seq, MC, 0, 53,56,60, 36); chordHit(seq, MC, 16, 49,53,56, 34);
      sampleMotion(seq, CH, 3200, 6400, 14);
      break;
    }
    case 9: { // X. Piano Bloom — same piano theme an octave up, full floor under it.
      fourFloor(seq, 112); backbeat(seq, 100); softOffbeats(seq, 56);
      hit(seq, CP, 12, 62); hit(seq, CP, 28, 68); hit(seq, OH, 15, 58, 76);
      melodicHit(seq, LC, 0, 72, 86, 0, 100, 1, 2);
      melodicHit(seq, LC, 3, 75, 74, 0, 100, 1, 2);
      melodicHit(seq, LC, 7, 77, 80, 0, 100, 1, 2);
      melodicHit(seq, LC, 10, 75, 68, 0, 88, 1, 2);
      melodicHit(seq, LC, 13, 72, 70, 0, 100, 1, 2);
      melodicHit(seq, LC, 16, 70, 78, 0, 100, 1, 2);
      melodicHit(seq, LC, 20, 68, 66, 0, 100, 1, 2);
      melodicHit(seq, LC, 23, 65, 72, 0, 88, 1, 2);
      melodicHit(seq, LC, 26, 68, 70, 0, 100, 1, 2);
      melodicHit(seq, LC, 30, 70, 64, 0, 76, 1, 2);
      chordHit(seq, MC, 2, 53,56,60, 40); chordHit(seq, MC, 10, 51,55,58, 36, 88);
      chordHit(seq, MC, 18, 49,53,56, 38); chordHit(seq, MC, 26, 48,51,55, 36, 90);
      break;
    }
    case 10: { // XI. Voltage — peak-hour slammer: all samples, one low SH pulse.
      fourFloor(seq, 122); backbeat(seq, 114);
      hit(seq, BD, 7, 98, 80); hit(seq, BD, 23, 102, 86);
      hit(seq, CP, 4, 110); hit(seq, CP, 12, 110);
      hit(seq, CP, 20, 112); hit(seq, CP, 28, 114);
      const uint8_t oh[] = {2,6,10,14,18,22,26,30}; hits(seq, OH, oh, 74, -6);
      const uint8_t h[] = {1,3,5,7,9,11,13,15,17,19,21,23,25,27,29,31};
      hits(seq, CH, h, 52, -9);
      hit(seq, RS, 11, 58, 76); hit(seq, RS, 27, 62, 82);
      hit(seq, CY, 0, 96);
      melodicHit(seq, LC, 0, 29, 58); melodicHit(seq, LC, 10, 29, 46, 0, 80);
      melodicHit(seq, LC, 16, 29, 60); melodicHit(seq, LC, 26, 32, 50, 0, 76);
      break;
    }
    case 11: { // Organ Dust — raw sample house with quiet WT chord stabs.
      fourFloor(seq, 117); backbeat(seq, 108); softOffbeats(seq, 65);
      hit(seq, CP, 12, 72); hit(seq, CP, 28, 78);
      const uint8_t sh[] = {3,7,11,15,19,23,27,31}; hits(seq, MA, sh, 43, -8);
      chordHit(seq, LC, 2, 53,56,60, 50); chordHit(seq, LC, 10, 51,55,58, 44, 88);
      chordHit(seq, LC, 18, 49,53,56, 48); chordHit(seq, LC, 26, 48,51,55, 46, 90);
      hit(seq, LT, 29, 48, 58); hit(seq, MT, 30, 56, 66); hit(seq, HT, 31, 64, 76);
      break;
    }
    case 12: { // XIII. Rupture — hard sample breaks, 909 ride glue, quiet SH bass.
      const uint8_t k[] = {0,3,10,16,19,26,29};
      const uint8_t s[] = {4,12,20,28};
      const uint8_t h[] = {0,2,5,7,8,10,13,16,18,21,23,24,26,29};
      hits(seq, BD, k, 120); hits(seq, SD, s, 116); hits(seq, CH, h, 60, -10);
      hit(seq, SD, 7, 44, 56); hit(seq, SD, 15, 48, 62); hit(seq, SD, 23, 44, 58);
      hit(seq, SD, 30, 88, 100, 2);
      hit(seq, OH, 15, 76); hit(seq, OH, 31, 80);
      const uint8_t ride[] = {0,8,16,24}; hits(seq, CY, ride, 54, -6);
      hit(seq, CL, 6, 48, 78); hit(seq, CL, 22, 52, 84);
      hit(seq, LT, 30, 70); hit(seq, MT, 31, 80, 100, 2);
      melodicHit(seq, LC, 0, 29, 62); melodicHit(seq, LC, 8, 32, 52, 0, 84);
      melodicHit(seq, LC, 16, 29, 64); melodicHit(seq, LC, 24, 27, 54, 0, 84);
      break;
    }
    case 13: { // Afterglow — slow sample pocket and barely-there harmonic light.
      const uint8_t k[] = {0,6,10,16,23,27};
      const uint8_t s[] = {4,12,20,28};
      const uint8_t h[] = {2,6,10,14,18,22,26,30};
      hits(seq, BD, k, 103); hits(seq, SD, s, 99); hits(seq, CH, h, 44, -9);
      hit(seq, SD, 11, 31, 42); hit(seq, RS, 7, 34, 46); hit(seq, OH, 31, 54, 66);
      melodicHit(seq, MC, 6, 60, 45, 0, 72); melodicHit(seq, MC, 22, 58, 48, 0, 78);
      chordHit(seq, LC, 0, 53,56,60, 39); chordHit(seq, LC, 16, 49,53,56, 36);
      sampleMotion(seq, CH, 2400, 4600, 20);
      break;
    }
    case 14: { // Peak Relay — fast sample break with a single controlled machine fill.
      const uint8_t k[] = {0,6,10,16,19,24,27,30};
      const uint8_t s[] = {4,12,20,25,28};
      const uint8_t h[] = {0,2,3,6,8,10,11,14,16,18,19,22,24,26,27,30};
      hits(seq, BD, k, 120); hits(seq, SD, s, 115); hits(seq, CH, h, 62, -10);
      hit(seq, CY, 0, 52); hit(seq, OH, 15, 67); hit(seq, OH, 31, 76);
      hit(seq, SD, 30, 62, 78, 2);
      hit(seq, LT, 29, 58); hit(seq, MT, 30, 68); hit(seq, HT, 31, 80, 100, 2);
      melodicHit(seq, MC, 0, 29, 80); melodicHit(seq, MC, 10, 36, 66);
      melodicHit(seq, MC, 16, 27, 82); melodicHit(seq, MC, 27, 32, 69);
      melodicHit(seq, LC, 31, 79, 40, 0, 66);
      break;
    }
    case 15: { // Machine Ritual — warehouse pulse, samples lead and FM stays low.
      fourFloor(seq, 122); backbeat(seq, 111); softOffbeats(seq, 68);
      const uint8_t hats[] = {1,5,9,13,17,21,25,29}; hits(seq, OH, hats, 56, -8);
      hit(seq, RS, 3, 48, 68); hit(seq, RS, 11, 54, 76);
      hit(seq, CY, 0, 48); hit(seq, CY, 16, 42, 72);
      melodicHit(seq, MC, 6, 48, 52, 0, 74); melodicHit(seq, MC, 14, 55, 58, 0, 80);
      melodicHit(seq, MC, 22, 51, 54, 0, 76); melodicHit(seq, MC, 30, 58, 62, 0, 84);
      melodicHit(seq, LC, 0, 29, 70); melodicHit(seq, LC, 10, 32, 64);
      melodicHit(seq, LC, 16, 27, 72); melodicHit(seq, LC, 27, 36, 68);
      sampleMotion(seq, CH, 6200, 10800, 8);
      break;
    }
  }
}
} // namespace

void initializeProfessionalPatternBank(Sequencer& sequencer) {
  resetPatternSoundProfiles();
  sequencer.setPatternLength(32);
  for (int pattern = 0; pattern < BUILTIN_PATTERN_COUNT; ++pattern) {
    buildPattern(sequencer, pattern);
    BuiltinPatternSoundProfile profile{};
    memcpy(profile.engines, ENGINE_PROFILE[pattern], sizeof(profile.engines));
    memcpy(profile.presets, PRESET_PROFILE[pattern], sizeof(profile.presets));
    setPatternSoundProfile(pattern, profile);
  }
  sequencer.selectPattern(0);
  sequencer.setHumanize(META[0].timing, META[0].velocity);
}

void initializeEsp32S3FactoryPatternBank(Sequencer& sequencer) {
  // The S3 firmware first built its integrated bank, then the factory JSON
  // replaced the musical data while retaining metadata defaults (swing and
  // humanize) for slots 0..15. Reproduce that order exactly.
  initializeProfessionalPatternBank(sequencer);
  sequencer.setPatternLength(16);
  sequencer.setTempo((float)LEGACY_FACTORY_TEMPO);
  resetPatternSoundProfiles();

  for (const LegacyFactoryPatternSeed& pattern : LEGACY_FACTORY_PATTERNS) {
    if (pattern.slot >= MAX_PATTERNS) continue;

    PatternMetadata metadata{};
    sequencer.getPatternMetadata(pattern.slot, metadata);
    strncpy(metadata.name, pattern.name, sizeof(metadata.name) - 1);
    metadata.name[sizeof(metadata.name) - 1] = '\0';
    metadata.recommendedBpm = LEGACY_FACTORY_TEMPO;

    sequencer.clearPattern(pattern.slot);
    sequencer.setPatternMetadata(pattern.slot, metadata);

    BuiltinPatternSoundProfile profile{};
    for (int track = 0; track < MAX_TRACKS; ++track)
      profile.engines[track] = -1;

    const uint16_t end = pattern.firstTrack + pattern.trackCount;
    for (uint16_t index = pattern.firstTrack; index < end; ++index) {
      const LegacyFactoryTrackSeed& source = LEGACY_FACTORY_TRACKS[index];
      if (source.track >= MAX_TRACKS) continue;
      profile.engines[source.track] = source.engine;
      if (source.engine >= 0 && source.engine < BUILTIN_ENGINE_COUNT)
        profile.presets[source.engine] = source.preset;

      for (uint8_t step = 0; step < 16; ++step) {
        const bool active = (source.activeMask & (uint16_t)(1u << step)) != 0;
        sequencer.setStep(pattern.slot, source.track, step, active,
                          source.velocities[step]);
        sequencer.setStepNote(pattern.slot, source.track, step,
                              source.notes[step]);
        sequencer.setStepFlags(pattern.slot, source.track, step,
                               source.flags[step]);
      }
    }
    setPatternSoundProfile(pattern.slot, profile);
  }

  // This cleanup was also executed by the S3 JSON loader for this exact bank.
  refineFactoryTwentyPatternBank(sequencer);
  sequencer.selectPattern(0);
  PatternMetadata initial{};
  if (sequencer.getPatternMetadata(0, initial))
    sequencer.setHumanize(initial.humanizeTimingMs, initial.humanizeVelocity);
}

bool getBuiltinPatternSoundProfile(int pattern, BuiltinPatternSoundProfile& out) {
  if (pattern < 0 || pattern >= MAX_PATTERNS || !ACTIVE_SOUND_PROFILE_VALID[pattern]) return false;
  memcpy(&out, &ACTIVE_SOUND_PROFILE[pattern], sizeof(out));
  return true;
}

void resetPatternSoundProfiles(void) {
  memset(ACTIVE_SOUND_PROFILE, 0, sizeof(ACTIVE_SOUND_PROFILE));
  memset(ACTIVE_SOUND_PROFILE_VALID, 0, sizeof(ACTIVE_SOUND_PROFILE_VALID));
}

void setPatternSoundProfile(int pattern, const BuiltinPatternSoundProfile& profile) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return;
  memcpy(&ACTIVE_SOUND_PROFILE[pattern], &profile, sizeof(profile));
  ACTIVE_SOUND_PROFILE_VALID[pattern] = true;
}

void clearPatternSoundProfile(int pattern) {
  if (pattern >= 0 && pattern < MAX_PATTERNS) ACTIVE_SOUND_PROFILE_VALID[pattern] = false;
}

uint8_t getConfiguredPatternCount(void) {
  for (int pattern = MAX_PATTERNS - 1; pattern >= 0; --pattern) {
    if (ACTIVE_SOUND_PROFILE_VALID[pattern]) return (uint8_t)(pattern + 1);
  }
  return 0;
}

void refineFactoryTwentyPatternBank(Sequencer& sequencer) {
  for (int pattern = 0; pattern < LEGACY_FACTORY_PATTERN_COUNT; ++pattern) {
    rewriteFactoryPad8Line(sequencer, pattern);
  }
}

// =============================================================================
// FACTORY EXPANSION BANK — slots 20..99, ten genres x eight scenes each.
// Each genre is one consecutive 8-pattern "song": a short arc from bare
// skeleton through groove/hook/build to a peak and back down, the same idea
// as the NIGHT DRIVE suite above but written directly at 16 steps, because
// patternLength is a single global the device only ever sets to 16 (see
// initializeEsp32S3FactoryPatternBank) — nothing longer would ever play.
// The original 20-pattern bank is untouched; this only adds new material.
// =============================================================================
namespace {

constexpr uint8_t EXP_SONGS = 10;
constexpr uint8_t EXP_SCENES = 8;
constexpr uint8_t EXP_COUNT = EXP_SONGS * EXP_SCENES; // 80

struct ExpansionSong {
  const char* genre;   // matched by RANDOM SONG's style keywords where one exists
  const char* kit;
  uint16_t bpm;
  uint8_t swing;
  uint8_t timing;
  uint8_t velocity;
  int8_t engines[MAX_TRACKS];
  uint8_t presets[BUILTIN_ENGINE_COUNT];
  uint8_t bassRoot;
  uint8_t chordRoot;
};

/* Preset order: 808, 909, 505, 303, WT, SH, FM, PHYS, NOISE — same as the
 * NIGHT DRIVE tables above. Every song keeps a flat, conservative profile
 * (preset 1 almost everywhere) since there is no way to audition these in
 * this environment; the per-song character comes from the engine choice on
 * LC/MC (bass/lead) rather than exotic preset indices. */
constexpr ExpansionSong EXP_SONG[EXP_SONGS] = {
  // 0 House — 124 BPM, four-on-the-floor, acid bass, warm chord stabs.
  {"House", "Smp+303/WT", 124, 12, 1, 4,
   {SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,EWT,E303},
   {1,1,1,1,1,1,1,0,0}, 33, 57},
  // 1 Techno — 130 BPM, relentless kick, 303 acid line, sparse stabs.
  {"Techno", "Smp+909/303", 130, 4, 1, 3,
   {SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,ESH,E303},
   {1,1,1,1,1,1,1,0,0}, 31, 55},
  // 2 Hip-Hop / Boom Bap — 92 BPM, dusty sample kit, FM sub only.
  {"Hip-Hop", "Samples", 92, 18, 2, 6,
   {SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,EFM},
   {1,1,1,1,1,1,1,0,0}, 29, 53},
  // 3 Trap — 140 BPM, half-time clap, crisp 909 hats, FM 808 sub.
  {"Trap", "Smp+FM sub", 140, 6, 1, 5,
   {SMP,SMP,E909,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,EFM},
   {1,1,1,1,1,1,1,0,0}, 26, 50},
  // 4 Drum & Bass — 174 BPM, chopped break, reese bass on WT.
  {"Drum & Bass", "Smp+WT reese", 174, 2, 1, 4,
   {SMP,SMP,SMP,SMP,E909,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,EWT},
   {1,1,1,1,1,1,1,0,0}, 28, 52},
  // 5 Dubstep — 140 BPM half-time, wobble/growl bass across SH+WT.
  {"Dubstep", "Smp+SH/WT", 140, 4, 1, 5,
   {SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,EWT,ESH},
   {1,1,1,1,1,1,1,0,0}, 26, 50},
  // 6 Funk / Disco — 112 BPM, syncopated kick, 303 funk bass, 909 stabs.
  {"Funk/Disco", "Smp+303", 112, 16, 1, 5,
   {SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,E909,E303},
   {1,1,1,1,1,1,1,0,0}, 33, 57},
  // 7 Reggaeton — 95 BPM, dembow-shaped kick/clap, FM sub + WT pluck.
  {"Reggaeton", "Smp+FM/WT", 95, 8, 1, 5,
   {SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,EWT,EFM},
   {1,1,1,1,1,1,1,0,0}, 31, 55},
  // 8 Afrobeats — 105 BPM, log-drum bass, interlocking hand percussion.
  {"Afrobeats", "Smp+FM log", 105, 10, 1, 6,
   {SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,EWT,EFM},
   {1,1,1,1,1,1,1,0,0}, 29, 53},
  // 9 Jungle / Breakbeat — 160 BPM, amen-style chop, WT sub bass.
  {"Breakbeat", "Smp+WT/909", 160, 2, 1, 5,
   {SMP,SMP,SMP,SMP,E909,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,SMP,EWT},
   {1,1,1,1,1,1,1,0,0}, 28, 52},
};

constexpr const char* EXP_SCENE_NAME[EXP_SONGS][EXP_SCENES] = {
  { "House I. Skeleton","House II. Groove","House III. Hook","House IV. Shuffle",
    "House V. Rise","House VI. Peak","House VII. Break","House VIII. Fade" },
  { "Techno I. Pulse","Techno II. Drive","Techno III. Acid","Techno IV. Shift",
    "Techno V. Charge","Techno VI. Overdrive","Techno VII. Void","Techno VIII. Reset" },
  { "Boom Bap I. Dust","Boom Bap II. Groove","Boom Bap III. Sample Flip","Boom Bap IV. Swing",
    "Boom Bap V. Layers","Boom Bap VI. Full Crate","Boom Bap VII. Breakdown","Boom Bap VIII. Outro" },
  { "Trap I. Skeleton","Trap II. Hi-Hat Roll","Trap III. 808 Slide","Trap IV. Half-Time",
    "Trap V. Triplets","Trap VI. Drop","Trap VII. Space","Trap VIII. Fade Out" },
  { "DnB I. Break","DnB II. Reese","DnB III. Rolling","DnB IV. Amen Chop",
    "DnB V. Rise","DnB VI. Drop","DnB VII. Half-Step","DnB VIII. Outro" },
  { "Dubstep I. Intro","Dubstep II. Half-Time","Dubstep III. Wobble","Dubstep IV. Growl",
    "Dubstep V. Tension","Dubstep VI. Drop","Dubstep VII. Space","Dubstep VIII. Fade" },
  { "Funk I. Pocket","Funk II. Groove","Funk III. Horns Cue","Funk IV. Syncopation",
    "Funk V. Break","Funk VI. Peak","Funk VII. Bridge","Funk VIII. Outro" },
  { "Reggaeton I. Dembow","Reggaeton II. Groove","Reggaeton III. Perreo","Reggaeton IV. Variation",
    "Reggaeton V. Build","Reggaeton VI. Drop","Reggaeton VII. Break","Reggaeton VIII. Outro" },
  { "Afrobeats I. Log Drum","Afrobeats II. Groove","Afrobeats III. Call-Response","Afrobeats IV. Layers",
    "Afrobeats V. Build","Afrobeats VI. Peak","Afrobeats VII. Break","Afrobeats VIII. Outro" },
  { "Jungle I. Amen","Jungle II. Chop","Jungle III. Rolling","Jungle IV. Sub Drop",
    "Jungle V. Build","Jungle VI. Drop","Jungle VII. Breakdown","Jungle VIII. Outro" },
};

constexpr uint8_t EXP_SCALE[8] = {0,3,5,7,10,12,15,19};
uint8_t scaleNote(uint8_t root, uint8_t degree) {
  return (uint8_t)(root + EXP_SCALE[degree & 0x07u]);
}

void buildHouseScene(Sequencer& seq, int scene, uint8_t bassRoot, uint8_t chordRoot) {
  switch (scene) {
    case 0: {
      const uint8_t k[] = {0,4,8,12};
      const uint8_t h[] = {2,6,10,14};
      hits(seq, BD, k, 112, -3); hits(seq, CH, h, 52, -8);
      hit(seq, OH, 14, 58, 70);
      melodicHit(seq, LC, 0, bassRoot, 70); melodicHit(seq, LC, 8, bassRoot, 62);
      break;
    }
    case 1: {
      const uint8_t k[] = {0,4,8,12};
      const uint8_t c[] = {4,12};
      const uint8_t h[] = {2,6,10,14};
      hits(seq, BD, k, 116, -3); hits(seq, CP, c, 108, -4); hits(seq, CH, h, 56, -9);
      melodicHit(seq, LC, 0, bassRoot, 88); melodicHit(seq, LC, 3, scaleNote(bassRoot,2), 66);
      melodicHit(seq, LC, 6, bassRoot, 78); melodicHit(seq, LC, 11, scaleNote(bassRoot,4), 62);
      break;
    }
    case 2: {
      const uint8_t k[] = {0,4,8,12};
      const uint8_t c[] = {4,12};
      const uint8_t h[] = {2,6,10,14};
      hits(seq, BD, k, 116, -3); hits(seq, CP, c, 108, -4); hits(seq, CH, h, 56, -9);
      melodicHit(seq, LC, 0, bassRoot, 88); melodicHit(seq, LC, 6, bassRoot, 78);
      chordHit(seq, MC, 2, chordRoot, chordRoot+3, chordRoot+7, 62);
      chordHit(seq, MC, 10, chordRoot-2, chordRoot+1, chordRoot+5, 58);
      break;
    }
    case 3: {
      const uint8_t k[] = {0,4,8,12};
      const uint8_t c[] = {4,12};
      const uint8_t h[] = {2,7,10,15};
      hits(seq, BD, k, 114, -3); hits(seq, CP, c, 106, -4); hits(seq, CH, h, 58, -9);
      hit(seq, RS, 6, 48, 72); hit(seq, RS, 14, 52, 78);
      melodicHit(seq, LC, 0, bassRoot, 86); melodicHit(seq, LC, 3, scaleNote(bassRoot,2), 64);
      melodicHit(seq, LC, 7, bassRoot, 76); melodicHit(seq, LC, 10, scaleNote(bassRoot,3), 60);
      chordHit(seq, MC, 2, chordRoot, chordRoot+3, chordRoot+7, 60);
      break;
    }
    case 4: {
      const uint8_t k[] = {0,4,8,12};
      const uint8_t c[] = {4,12};
      const uint8_t h[] = {0,2,4,6,8,10,12,14};
      hits(seq, BD, k, 118, -2); hits(seq, CP, c, 110, -3); hits(seq, CH, h, 50, -10);
      const uint8_t sh[] = {1,3,5,7,9,11,13,15}; hits(seq, MA, sh, 42, -8);
      melodicHit(seq, LC, 0, bassRoot, 90); melodicHit(seq, LC, 6, bassRoot, 84);
      chordHit(seq, MC, 2, chordRoot, chordRoot+3, chordRoot+7, 64);
      break;
    }
    case 5: {
      const uint8_t k[] = {0,4,8,12};
      const uint8_t c[] = {4,12};
      const uint8_t h[] = {0,2,4,6,8,10,12,14};
      hits(seq, BD, k, 122, -2); hits(seq, CP, c, 114, -3); hits(seq, CH, h, 54, -9);
      const uint8_t oh[] = {2,6,10,14}; hits(seq, OH, oh, 74, -6);
      hit(seq, CY, 0, 98);
      melodicHit(seq, LC, 0, bassRoot, 96); melodicHit(seq, LC, 3, scaleNote(bassRoot,2), 78);
      melodicHit(seq, LC, 6, bassRoot, 92); melodicHit(seq, LC, 11, scaleNote(bassRoot,4), 76);
      chordHit(seq, MC, 2, chordRoot, chordRoot+3, chordRoot+7, 70);
      chordHit(seq, MC, 10, chordRoot-2, chordRoot+1, chordRoot+5, 66);
      break;
    }
    case 6: {
      const uint8_t h[] = {2,6,10,14};
      hits(seq, CH, h, 48, -8);
      melodicHit(seq, LC, 0, bassRoot, 74); melodicHit(seq, LC, 8, bassRoot, 62, 0, 82);
      chordHit(seq, MC, 2, chordRoot, chordRoot+3, chordRoot+7, 54, 88);
      chordHit(seq, MC, 10, chordRoot-2, chordRoot+1, chordRoot+5, 50, 84);
      seq.setStepCutoffLock(CH, 0, true, 2200);
      seq.setStepCutoffLock(CH, 8, true, 8800);
      seq.setStepReverbSendLock(CH, 14, true, 24);
      break;
    }
    case 7: {
      const uint8_t k[] = {0,8};
      const uint8_t h[] = {2,6,10,14};
      hits(seq, BD, k, 102, -4); hits(seq, CH, h, 44, -10);
      melodicHit(seq, LC, 0, bassRoot, 70); melodicHit(seq, LC, 8, bassRoot, 56);
      hit(seq, CY, 15, 62, 60);
      break;
    }
  }
}

void buildTechnoScene(Sequencer& seq, int scene, uint8_t bassRoot, uint8_t chordRoot) {
  switch (scene) {
    case 0: {
      const uint8_t k[] = {0,4,8,12};
      const uint8_t h[] = {2,6,10,14};
      hits(seq, BD, k, 116, -2); hits(seq, CH, h, 50, -10);
      melodicHit(seq, LC, 0, bassRoot, 72); melodicHit(seq, LC, 8, bassRoot, 66);
      break;
    }
    case 1: {
      const uint8_t k[] = {0,4,8,12};
      const uint8_t h[] = {0,2,4,6,8,10,12,14};
      hits(seq, BD, k, 120, -2); hits(seq, CH, h, 46, -11);
      const uint8_t b[] = {0,2,4,6,8,10,12,14};
      hits(seq, LC, b, 78, -8);
      break;
    }
    case 2: {
      const uint8_t k[] = {0,4,8,12};
      hits(seq, BD, k, 120, -2);
      const uint8_t h[] = {2,6,10,14}; hits(seq, CH, h, 48, -10);
      melodicHit(seq, LC, 0, bassRoot, 90); melodicHit(seq, LC, 2, scaleNote(bassRoot,1), 62, 0, 80);
      melodicHit(seq, LC, 3, bassRoot, 70); melodicHit(seq, LC, 5, scaleNote(bassRoot,3), 84);
      melodicHit(seq, LC, 8, bassRoot, 92); melodicHit(seq, LC, 10, scaleNote(bassRoot,2), 60, 0, 78);
      melodicHit(seq, LC, 11, bassRoot, 68); melodicHit(seq, LC, 13, scaleNote(bassRoot,4), 86, 0, 100, 2);
      break;
    }
    case 3: {
      const uint8_t k[] = {0,4,7,8,12};
      const uint8_t h[] = {2,6,10,14};
      hits(seq, BD, k, 118, -2); hits(seq, CH, h, 50, -9);
      hit(seq, RS, 6, 52, 72); hit(seq, RS, 14, 56, 78);
      melodicHit(seq, LC, 0, bassRoot, 88); melodicHit(seq, LC, 8, bassRoot, 84);
      break;
    }
    case 4: {
      const uint8_t k[] = {0,4,8,12};
      const uint8_t h[] = {0,2,4,6,8,10,12,14};
      hits(seq, BD, k, 120, -2); hits(seq, CH, h, 52, -8);
      const uint8_t sh[] = {1,3,5,7,9,11,13,15}; hits(seq, MA, sh, 44, -8);
      melodicHit(seq, LC, 0, bassRoot, 92); melodicHit(seq, LC, 4, bassRoot, 80);
      melodicHit(seq, LC, 8, bassRoot, 92); melodicHit(seq, LC, 12, bassRoot, 80);
      break;
    }
    case 5: {
      const uint8_t k[] = {0,4,8,12};
      const uint8_t h[] = {0,2,4,6,8,10,12,14};
      hits(seq, BD, k, 124, -1); hits(seq, CH, h, 56, -8);
      const uint8_t ride[] = {0,4,8,12}; hits(seq, CY, ride, 62, -6);
      hit(seq, CY, 0, 100);
      melodicHit(seq, LC, 0, bassRoot, 96); melodicHit(seq, LC, 2, scaleNote(bassRoot,1), 66, 0, 78);
      melodicHit(seq, LC, 8, bassRoot, 96); melodicHit(seq, LC, 10, scaleNote(bassRoot,2), 66, 0, 78);
      chordHit(seq, MC, 6, chordRoot, chordRoot+3, chordRoot+6, 58, 80);
      break;
    }
    case 6: {
      const uint8_t h[] = {2,6,10,14};
      hits(seq, CH, h, 44, -10);
      melodicHit(seq, LC, 0, bassRoot, 70, 0, 84); melodicHit(seq, LC, 8, bassRoot, 62, 0, 78);
      seq.setStepCutoffLock(CH, 0, true, 1800);
      seq.setStepCutoffLock(CH, 12, true, 9200);
      seq.setStepReverbSendLock(LC, 8, true, 20);
      break;
    }
    case 7: {
      const uint8_t k[] = {0,8};
      const uint8_t h[] = {2,6,10,14};
      hits(seq, BD, k, 104, -3); hits(seq, CH, h, 40, -11);
      melodicHit(seq, LC, 0, bassRoot, 68);
      break;
    }
  }
}

void buildBoomBapScene(Sequencer& seq, int scene, uint8_t bassRoot, uint8_t chordRoot) {
  switch (scene) {
    case 0: {
      const uint8_t k[] = {0,10};
      const uint8_t s[] = {4,12};
      hits(seq, BD, k, 110, -4); hits(seq, SD, s, 104, -5);
      const uint8_t h[] = {2,6,10,14}; hits(seq, CH, h, 54, -12);
      break;
    }
    case 1: {
      const uint8_t k[] = {0,7,10};
      const uint8_t s[] = {4,12};
      const uint8_t h[] = {0,2,4,6,8,10,12,14};
      hits(seq, BD, k, 112, -4); hits(seq, SD, s, 106, -5); hits(seq, CH, h, 50, -14);
      melodicHit(seq, LC, 0, bassRoot, 78); melodicHit(seq, LC, 10, scaleNote(bassRoot,2), 62);
      break;
    }
    case 2: {
      const uint8_t k[] = {0,3,10};
      const uint8_t s[] = {4,12};
      hits(seq, BD, k, 112, -4); hits(seq, SD, s, 106, -5);
      hit(seq, SD, 9, 44, 55); hit(seq, SD, 15, 40, 48);
      const uint8_t h[] = {2,6,10,14}; hits(seq, CH, h, 52, -12);
      melodicHit(seq, LC, 0, bassRoot, 80); melodicHit(seq, LC, 6, scaleNote(bassRoot,3), 60);
      break;
    }
    case 3: {
      const uint8_t k[] = {0,7,10};
      const uint8_t s[] = {4,12};
      const uint8_t h[] = {1,3,6,9,11,14};
      hits(seq, BD, k, 112, -4); hits(seq, SD, s, 106, -5); hits(seq, CH, h, 52, -12);
      hit(seq, RS, 8, 42, 64);
      melodicHit(seq, LC, 0, bassRoot, 82); melodicHit(seq, LC, 7, scaleNote(bassRoot,2), 62);
      melodicHit(seq, LC, 10, bassRoot, 76);
      break;
    }
    case 4: {
      const uint8_t k[] = {0,7,10};
      const uint8_t s[] = {4,12};
      const uint8_t h[] = {0,2,4,6,8,10,12,14};
      hits(seq, BD, k, 114, -4); hits(seq, SD, s, 108, -5); hits(seq, CH, h, 52, -12);
      hit(seq, CB, 6, 46, 68); hit(seq, CL, 14, 40, 62);
      melodicHit(seq, LC, 0, bassRoot, 84); melodicHit(seq, LC, 6, scaleNote(bassRoot,3), 64);
      chordHit(seq, MC, 2, chordRoot, chordRoot+3, chordRoot+7, 48, 82);
      break;
    }
    case 5: {
      const uint8_t k[] = {0,3,7,10};
      const uint8_t s[] = {4,12};
      const uint8_t h[] = {0,2,4,6,8,10,12,14};
      hits(seq, BD, k, 116, -3); hits(seq, SD, s, 110, -4); hits(seq, CH, h, 56, -11);
      hit(seq, SD, 15, 70, 82, 2);
      hit(seq, CB, 6, 48, 70); hit(seq, CL, 14, 44, 64);
      melodicHit(seq, LC, 0, bassRoot, 90); melodicHit(seq, LC, 6, scaleNote(bassRoot,3), 68);
      melodicHit(seq, LC, 10, bassRoot, 84);
      chordHit(seq, MC, 2, chordRoot, chordRoot+3, chordRoot+7, 54, 84);
      break;
    }
    case 6: {
      const uint8_t k[] = {0,10};
      hits(seq, BD, k, 96, -5);
      melodicHit(seq, LC, 0, bassRoot, 70, 0, 88); melodicHit(seq, LC, 10, scaleNote(bassRoot,2), 56, 0, 78);
      const uint8_t h[] = {6,14}; hits(seq, CH, h, 38, -14);
      break;
    }
    case 7: {
      hit(seq, BD, 0, 100);
      hit(seq, SD, 4, 88, 68);
      melodicHit(seq, LC, 0, bassRoot, 66);
      hit(seq, CB, 12, 44, 55);
      break;
    }
  }
}

void buildTrapScene(Sequencer& seq, int scene, uint8_t bassRoot, uint8_t chordRoot) {
  switch (scene) {
    case 0: {
      const uint8_t k[] = {0,8};
      hits(seq, BD, k, 110, -3);
      hit(seq, CP, 8, 100, 80);
      const uint8_t h[] = {0,2,4,6,8,10,12,14}; hits(seq, CH, h, 48, -10);
      break;
    }
    case 1: {
      const uint8_t k[] = {0,7,8};
      hits(seq, BD, k, 114, -3);
      hit(seq, CP, 8, 104, 82);
      const uint8_t h[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
      hits(seq, CH, h, 42, -12);
      melodicHit(seq, LC, 0, bassRoot, 92); melodicHit(seq, LC, 8, bassRoot, 96, 0, 100, 1, 4);
      break;
    }
    case 2: {
      const uint8_t k[] = {0,7,8,13};
      hits(seq, BD, k, 114, -3);
      hit(seq, CP, 8, 104, 82);
      const uint8_t h[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
      hits(seq, CH, h, 42, -12);
      melodicHit(seq, LC, 0, bassRoot, 96); melodicHit(seq, LC, 6, scaleNote(bassRoot,2), 70, 0, 76);
      melodicHit(seq, LC, 8, bassRoot, 100, 0, 100, 1, 4); melodicHit(seq, LC, 14, scaleNote(bassRoot,1), 66, 0, 72);
      break;
    }
    case 3: {
      const uint8_t k[] = {0,8};
      hits(seq, BD, k, 112, -3);
      hit(seq, CP, 8, 106, 84);
      melodicHit(seq, LC, 0, bassRoot, 100); melodicHit(seq, LC, 8, bassRoot, 104, 0, 100, 1, 4);
      break;
    }
    case 4: {
      const uint8_t k[] = {0,7,8};
      hits(seq, BD, k, 114, -3);
      hit(seq, CP, 8, 106, 84);
      const uint8_t h[] = {0,2,4,6,8,10,12,14}; hits(seq, CH, h, 46, -10);
      hit(seq, CH, 14, 70, 90, 2); hit(seq, CH, 15, 62, 84, 2);
      melodicHit(seq, LC, 0, bassRoot, 98); melodicHit(seq, LC, 8, bassRoot, 102, 0, 100, 1, 4);
      break;
    }
    case 5: {
      const uint8_t k[] = {0,3,7,8,11};
      hits(seq, BD, k, 118, -2);
      hit(seq, CP, 8, 110, 88);
      const uint8_t h[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
      hits(seq, CH, h, 46, -12);
      hit(seq, CY, 0, 96);
      melodicHit(seq, LC, 0, bassRoot, 104); melodicHit(seq, LC, 8, bassRoot, 108, 0, 100, 1, 4);
      chordHit(seq, MC, 8, chordRoot, chordRoot+3, chordRoot+7, 60, 84);
      break;
    }
    case 6: {
      hit(seq, BD, 0, 100);
      hit(seq, CP, 8, 88, 70);
      const uint8_t h[] = {4,12}; hits(seq, CH, h, 34, -14);
      melodicHit(seq, LC, 0, bassRoot, 84, 0, 84); melodicHit(seq, LC, 8, bassRoot, 78, 0, 76, 1, 4);
      break;
    }
    case 7: {
      hit(seq, BD, 0, 92);
      hit(seq, CP, 8, 78, 60);
      melodicHit(seq, LC, 0, bassRoot, 70);
      break;
    }
  }
}

void buildDnbScene(Sequencer& seq, int scene, uint8_t bassRoot, uint8_t chordRoot) {
  switch (scene) {
    case 0: {
      const uint8_t k[] = {0,10};
      const uint8_t s[] = {4,12};
      hits(seq, BD, k, 108, -3); hits(seq, SD, s, 104, -4);
      const uint8_t h[] = {2,6,10,14}; hits(seq, CH, h, 50, -10);
      break;
    }
    case 1: {
      const uint8_t k[] = {0,10};
      const uint8_t s[] = {4,12};
      const uint8_t h[] = {0,2,4,6,8,10,12,14};
      hits(seq, BD, k, 112, -3); hits(seq, SD, s, 108, -4); hits(seq, CH, h, 48, -11);
      melodicHit(seq, LC, 0, bassRoot, 88, 0, 100, 1, 3);
      break;
    }
    case 2: {
      const uint8_t k[] = {0,10};
      const uint8_t s[] = {4,12};
      const uint8_t h[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
      hits(seq, BD, k, 112, -3); hits(seq, SD, s, 108, -4); hits(seq, CH, h, 44, -12);
      hit(seq, SD, 7, 46, 55); hit(seq, SD, 15, 42, 50);
      melodicHit(seq, LC, 0, bassRoot, 90, 0, 100, 1, 3);
      break;
    }
    case 3: {
      const uint8_t k[] = {0,6,10};
      const uint8_t s[] = {4,9,12};
      const uint8_t h[] = {0,2,4,6,8,10,12,14};
      hits(seq, BD, k, 114, -3); hits(seq, SD, s, 108, -4); hits(seq, CH, h, 48, -11);
      hit(seq, OH, 14, 66, 74);
      melodicHit(seq, LC, 0, bassRoot, 92, 0, 100, 1, 3);
      break;
    }
    case 4: {
      const uint8_t k[] = {0,10};
      const uint8_t s[] = {4,12};
      const uint8_t h[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
      hits(seq, BD, k, 116, -2); hits(seq, SD, s, 110, -3); hits(seq, CH, h, 50, -10);
      hit(seq, CY, 14, 72, 90, 2); hit(seq, CY, 15, 80, 96, 2);
      melodicHit(seq, LC, 0, bassRoot, 96, 0, 100, 1, 3);
      break;
    }
    case 5: {
      const uint8_t k[] = {0,6,10};
      const uint8_t s[] = {4,9,12};
      const uint8_t h[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
      hits(seq, BD, k, 118, -2); hits(seq, SD, s, 112, -3); hits(seq, CH, h, 50, -11);
      hit(seq, CY, 0, 92); hit(seq, OH, 14, 70, 78);
      melodicHit(seq, LC, 0, bassRoot, 100, 0, 100, 1, 3);
      melodicHit(seq, LC, 8, scaleNote(bassRoot,2), 86, 0, 100, 1, 3);
      chordHit(seq, MC, 6, chordRoot, chordRoot+3, chordRoot+6, 56, 78);
      break;
    }
    case 6: {
      const uint8_t k[] = {0,10};
      hits(seq, BD, k, 106, -3); hit(seq, SD, 8, 100, 78);
      const uint8_t h[] = {4,12}; hits(seq, CH, h, 40, -12);
      melodicHit(seq, LC, 0, bassRoot, 92, 0, 100, 1, 4);
      break;
    }
    case 7: {
      hit(seq, BD, 0, 100); hit(seq, SD, 4, 84, 68);
      melodicHit(seq, LC, 0, bassRoot, 74);
      break;
    }
  }
}

void buildDubstepScene(Sequencer& seq, int scene, uint8_t bassRoot, uint8_t chordRoot) {
  switch (scene) {
    case 0: {
      hit(seq, BD, 0, 108); hit(seq, SD, 8, 100, 80);
      const uint8_t h[] = {4,12}; hits(seq, CH, h, 42, -12);
      break;
    }
    case 1: {
      const uint8_t k[] = {0,6};
      hits(seq, BD, k, 110, -3); hit(seq, SD, 8, 104, 84);
      const uint8_t h[] = {0,4,8,12}; hits(seq, CH, h, 40, -12);
      melodicHit(seq, LC, 0, bassRoot, 88, 0, 100, 1, 4);
      break;
    }
    case 2: {
      const uint8_t k[] = {0,6};
      hits(seq, BD, k, 112, -3); hit(seq, SD, 8, 106, 86);
      const uint8_t h[] = {4,12}; hits(seq, CH, h, 38, -12);
      melodicHit(seq, LC, 0, bassRoot, 100, 0, 100, 2, 1);
      melodicHit(seq, LC, 8, bassRoot, 104, 0, 100, 2, 1);
      break;
    }
    case 3: {
      const uint8_t k[] = {0,6};
      hits(seq, BD, k, 112, -3); hit(seq, SD, 8, 106, 86);
      melodicHit(seq, LC, 0, bassRoot, 100, 0, 100, 2, 1);
      melodicHit(seq, LC, 6, scaleNote(bassRoot,2), 72, 0, 80);
      melodicHit(seq, LC, 8, scaleNote(bassRoot,1), 104, 0, 100, 2, 1);
      chordHit(seq, MC, 12, chordRoot, chordRoot+3, chordRoot+6, 50, 76);
      break;
    }
    case 4: {
      const uint8_t k[] = {0,6};
      hits(seq, BD, k, 114, -3); hit(seq, SD, 8, 108, 88);
      const uint8_t h[] = {0,2,4,6,8,10,12,14}; hits(seq, CH, h, 40, -12);
      hit(seq, CH, 14, 66, 88, 2); hit(seq, CH, 15, 74, 92, 2);
      melodicHit(seq, LC, 0, bassRoot, 98, 0, 100, 1, 2);
      break;
    }
    case 5: {
      const uint8_t k[] = {0,3,6,11};
      hits(seq, BD, k, 118, -2); hit(seq, SD, 8, 112, 92);
      hit(seq, CY, 0, 96);
      const uint8_t h[] = {0,2,4,6,8,10,12,14}; hits(seq, CH, h, 44, -12);
      melodicHit(seq, LC, 0, bassRoot, 108, 0, 100, 2, 1);
      melodicHit(seq, LC, 8, bassRoot, 112, 0, 100, 2, 1);
      break;
    }
    case 6: {
      hit(seq, BD, 0, 96); hit(seq, SD, 8, 90, 72);
      melodicHit(seq, LC, 0, bassRoot, 80, 0, 84);
      seq.setStepCutoffLock(LC, 0, true, 300);
      seq.setStepCutoffLock(LC, 8, true, 4200);
      break;
    }
    case 7: {
      hit(seq, BD, 0, 90); hit(seq, SD, 8, 76, 60);
      melodicHit(seq, LC, 0, bassRoot, 66);
      break;
    }
  }
}

void buildFunkScene(Sequencer& seq, int scene, uint8_t bassRoot, uint8_t chordRoot) {
  switch (scene) {
    case 0: {
      const uint8_t k[] = {0,8};
      const uint8_t s[] = {4,12};
      hits(seq, BD, k, 108, -3); hits(seq, SD, s, 104, -4);
      const uint8_t h[] = {2,6,10,14}; hits(seq, CH, h, 56, -10);
      break;
    }
    case 1: {
      const uint8_t k[] = {0,6,8,14};
      const uint8_t s[] = {4,12};
      const uint8_t h[] = {0,2,4,6,8,10,12,14};
      hits(seq, BD, k, 112, -3); hits(seq, SD, s, 106, -4); hits(seq, CH, h, 54, -10);
      melodicHit(seq, LC, 0, bassRoot, 86); melodicHit(seq, LC, 3, scaleNote(bassRoot,2), 66);
      melodicHit(seq, LC, 6, bassRoot, 78); melodicHit(seq, LC, 10, scaleNote(bassRoot,4), 62);
      break;
    }
    case 2: {
      const uint8_t k[] = {0,6,8,14};
      const uint8_t s[] = {4,12};
      hits(seq, BD, k, 112, -3); hits(seq, SD, s, 106, -4);
      const uint8_t h[] = {0,2,4,6,8,10,12,14}; hits(seq, CH, h, 54, -10);
      chordHit(seq, MC, 2, chordRoot, chordRoot+4, chordRoot+7, 66);
      chordHit(seq, MC, 10, chordRoot+2, chordRoot+5, chordRoot+9, 62);
      melodicHit(seq, LC, 0, bassRoot, 88); melodicHit(seq, LC, 6, bassRoot, 80);
      break;
    }
    case 3: {
      const uint8_t k[] = {0,3,6,8,11,14};
      const uint8_t s[] = {4,12};
      hits(seq, BD, k, 112, -3); hits(seq, SD, s, 106, -4);
      hit(seq, RS, 7, 48, 70); hit(seq, RS, 15, 52, 76);
      const uint8_t h[] = {0,2,4,6,8,10,12,14}; hits(seq, CH, h, 54, -10);
      melodicHit(seq, LC, 0, bassRoot, 88); melodicHit(seq, LC, 3, scaleNote(bassRoot,2), 66);
      melodicHit(seq, LC, 6, bassRoot, 80); melodicHit(seq, LC, 11, scaleNote(bassRoot,5), 64);
      break;
    }
    case 4: {
      const uint8_t k[] = {0,8};
      hits(seq, BD, k, 104, -4);
      melodicHit(seq, LC, 0, bassRoot, 92); melodicHit(seq, LC, 2, scaleNote(bassRoot,1), 70, 0, 82);
      melodicHit(seq, LC, 4, bassRoot, 84); melodicHit(seq, LC, 7, scaleNote(bassRoot,3), 74);
      melodicHit(seq, LC, 8, bassRoot, 90); melodicHit(seq, LC, 11, scaleNote(bassRoot,5), 68);
      const uint8_t h[] = {2,6,10,14}; hits(seq, CH, h, 44, -12);
      break;
    }
    case 5: {
      const uint8_t k[] = {0,6,8,14};
      const uint8_t s[] = {4,12};
      const uint8_t h[] = {0,2,4,6,8,10,12,14};
      hits(seq, BD, k, 116, -2); hits(seq, SD, s, 110, -3); hits(seq, CH, h, 58, -9);
      hit(seq, CY, 0, 92);
      chordHit(seq, MC, 2, chordRoot, chordRoot+4, chordRoot+7, 70);
      chordHit(seq, MC, 10, chordRoot+2, chordRoot+5, chordRoot+9, 66);
      melodicHit(seq, LC, 0, bassRoot, 96); melodicHit(seq, LC, 6, bassRoot, 88);
      break;
    }
    case 6: {
      hit(seq, BD, 0, 98); hit(seq, SD, 8, 90, 70);
      melodicHit(seq, LC, 0, bassRoot, 78, 0, 86);
      chordHit(seq, MC, 4, chordRoot, chordRoot+4, chordRoot+7, 54, 80);
      break;
    }
    case 7: {
      hit(seq, BD, 0, 96); hit(seq, SD, 4, 82, 64);
      melodicHit(seq, LC, 0, bassRoot, 70);
      break;
    }
  }
}

void buildReggaetonScene(Sequencer& seq, int scene, uint8_t bassRoot, uint8_t chordRoot) {
  switch (scene) {
    case 0: {
      const uint8_t k[] = {0,6,10};
      hits(seq, BD, k, 110, -3);
      const uint8_t c[] = {4,12}; hits(seq, CP, c, 100, -4);
      break;
    }
    case 1: {
      const uint8_t k[] = {0,6,10};
      const uint8_t c[] = {4,8,12,14};
      const uint8_t h[] = {2,6,10,14};
      hits(seq, BD, k, 112, -3); hits(seq, CP, c, 104, -4); hits(seq, CH, h, 52, -10);
      melodicHit(seq, LC, 0, bassRoot, 84); melodicHit(seq, LC, 10, scaleNote(bassRoot,2), 66);
      break;
    }
    case 2: {
      const uint8_t k[] = {0,6,10};
      const uint8_t c[] = {4,8,12,14};
      const uint8_t h[] = {0,2,4,6,8,10,12,14};
      hits(seq, BD, k, 114, -3); hits(seq, CP, c, 106, -4); hits(seq, CH, h, 50, -10);
      melodicHit(seq, MC, 2, scaleNote(chordRoot,0), 62); melodicHit(seq, MC, 10, scaleNote(chordRoot,2), 58);
      melodicHit(seq, LC, 0, bassRoot, 86); melodicHit(seq, LC, 6, scaleNote(bassRoot,3), 68);
      break;
    }
    case 3: {
      const uint8_t k[] = {0,3,6,10,13};
      const uint8_t c[] = {4,8,12,14};
      hits(seq, BD, k, 114, -3); hits(seq, CP, c, 106, -4);
      const uint8_t h[] = {2,6,10,14}; hits(seq, CH, h, 50, -10);
      hit(seq, RS, 15, 48, 70);
      melodicHit(seq, LC, 0, bassRoot, 88); melodicHit(seq, LC, 6, scaleNote(bassRoot,3), 70);
      break;
    }
    case 4: {
      const uint8_t k[] = {0,6,10};
      const uint8_t c[] = {4,8,12,14};
      const uint8_t h[] = {0,2,4,6,8,10,12,14};
      hits(seq, BD, k, 116, -2); hits(seq, CP, c, 108, -3); hits(seq, CH, h, 52, -9);
      const uint8_t sh[] = {1,3,5,7,9,11,13,15}; hits(seq, MA, sh, 40, -8);
      melodicHit(seq, LC, 0, bassRoot, 90); melodicHit(seq, LC, 10, scaleNote(bassRoot,2), 72);
      break;
    }
    case 5: {
      const uint8_t k[] = {0,6,10};
      const uint8_t c[] = {4,8,12,14};
      const uint8_t h[] = {0,2,4,6,8,10,12,14};
      hits(seq, BD, k, 118, -2); hits(seq, CP, c, 110, -3); hits(seq, CH, h, 54, -9);
      hit(seq, CY, 0, 90);
      melodicHit(seq, MC, 2, scaleNote(chordRoot,0), 68); melodicHit(seq, MC, 6, scaleNote(chordRoot,3), 64);
      melodicHit(seq, MC, 10, scaleNote(chordRoot,2), 70); melodicHit(seq, MC, 14, scaleNote(chordRoot,5), 62);
      melodicHit(seq, LC, 0, bassRoot, 94); melodicHit(seq, LC, 6, scaleNote(bassRoot,3), 76);
      break;
    }
    case 6: {
      const uint8_t c[] = {4,12};
      hits(seq, CP, c, 90, -6);
      melodicHit(seq, LC, 0, bassRoot, 76, 0, 86);
      melodicHit(seq, MC, 6, scaleNote(chordRoot,2), 56, 0, 80);
      break;
    }
    case 7: {
      hit(seq, BD, 0, 96); hit(seq, CP, 4, 82, 66);
      melodicHit(seq, LC, 0, bassRoot, 70);
      break;
    }
  }
}

void buildAfrobeatsScene(Sequencer& seq, int scene, uint8_t bassRoot, uint8_t chordRoot) {
  switch (scene) {
    case 0: {
      const uint8_t k[] = {0,10};
      hits(seq, BD, k, 106, -3);
      const uint8_t sh[] = {0,2,4,6,8,10,12,14}; hits(seq, MA, sh, 44, -8);
      break;
    }
    case 1: {
      const uint8_t k[] = {0,10,13};
      const uint8_t c[] = {4,12};
      hits(seq, BD, k, 108, -3); hits(seq, CP, c, 96, -6);
      const uint8_t sh[] = {0,2,4,6,8,10,12,14}; hits(seq, MA, sh, 46, -8);
      melodicHit(seq, LC, 0, bassRoot, 82); melodicHit(seq, LC, 10, scaleNote(bassRoot,2), 68);
      break;
    }
    case 2: {
      const uint8_t k[] = {0,10,13};
      const uint8_t c[] = {4,12};
      hits(seq, BD, k, 110, -3); hits(seq, CP, c, 98, -6);
      const uint8_t sh[] = {0,2,4,6,8,10,12,14}; hits(seq, MA, sh, 46, -8);
      melodicHit(seq, LC, 0, bassRoot, 86); melodicHit(seq, LC, 6, scaleNote(bassRoot,2), 68);
      hit(seq, HC, 3, 54, 74); hit(seq, HC, 11, 58, 78);
      break;
    }
    case 3: {
      const uint8_t k[] = {0,10,13};
      const uint8_t c[] = {4,12};
      hits(seq, BD, k, 110, -3); hits(seq, CP, c, 98, -6);
      const uint8_t sh[] = {0,2,4,6,8,10,12,14}; hits(seq, MA, sh, 44, -8);
      hit(seq, HC, 3, 52, 72); hit(seq, MC, 7, 50, 70); hit(seq, LC, 15, 48, 66, 2);
      melodicHit(seq, LC, 0, bassRoot, 88); melodicHit(seq, LC, 6, scaleNote(bassRoot,2), 70);
      break;
    }
    case 4: {
      const uint8_t k[] = {0,6,10,13};
      const uint8_t c[] = {4,12};
      const uint8_t h[] = {2,6,10,14};
      hits(seq, BD, k, 112, -2); hits(seq, CP, c, 100, -5); hits(seq, CH, h, 42, -10);
      const uint8_t sh[] = {0,2,4,6,8,10,12,14}; hits(seq, MA, sh, 48, -8);
      melodicHit(seq, LC, 0, bassRoot, 92); melodicHit(seq, LC, 6, scaleNote(bassRoot,2), 74);
      break;
    }
    case 5: {
      const uint8_t k[] = {0,6,10,13};
      const uint8_t c[] = {4,12};
      const uint8_t h[] = {0,2,4,6,8,10,12,14};
      hits(seq, BD, k, 116, -2); hits(seq, CP, c, 104, -4); hits(seq, CH, h, 46, -9);
      hit(seq, CY, 0, 86);
      const uint8_t sh[] = {0,2,4,6,8,10,12,14}; hits(seq, MA, sh, 50, -7);
      chordHit(seq, MC, 2, chordRoot, chordRoot+3, chordRoot+7, 60, 84);
      melodicHit(seq, LC, 0, bassRoot, 96); melodicHit(seq, LC, 6, scaleNote(bassRoot,2), 78);
      break;
    }
    case 6: {
      const uint8_t sh[] = {0,4,8,12}; hits(seq, MA, sh, 40, -8);
      hit(seq, HC, 6, 48, 68); hit(seq, MC, 10, 46, 66);
      melodicHit(seq, LC, 0, bassRoot, 74, 0, 84);
      break;
    }
    case 7: {
      hit(seq, BD, 0, 96);
      const uint8_t sh[] = {0,4,8,12}; hits(seq, MA, sh, 36, -8);
      melodicHit(seq, LC, 0, bassRoot, 68);
      break;
    }
  }
}

void buildJungleScene(Sequencer& seq, int scene, uint8_t bassRoot, uint8_t chordRoot) {
  switch (scene) {
    case 0: {
      const uint8_t k[] = {0,10};
      const uint8_t s[] = {4,12};
      hits(seq, BD, k, 108, -3); hits(seq, SD, s, 104, -4);
      const uint8_t h[] = {2,6,10,14}; hits(seq, CH, h, 48, -11);
      break;
    }
    case 1: {
      const uint8_t k[] = {0,10};
      const uint8_t s[] = {4,12};
      hits(seq, BD, k, 112, -3); hits(seq, SD, s, 108, -4);
      hit(seq, SD, 7, 44, 55); hit(seq, SD, 15, 40, 50);
      const uint8_t h[] = {0,2,4,6,8,10,12,14}; hits(seq, CH, h, 46, -12);
      melodicHit(seq, LC, 0, bassRoot, 88, 0, 100, 1, 3);
      break;
    }
    case 2: {
      const uint8_t k[] = {0,6,10};
      const uint8_t s[] = {4,9,12};
      hits(seq, BD, k, 114, -3); hits(seq, SD, s, 108, -4);
      const uint8_t h[] = {0,2,4,6,8,10,12,14}; hits(seq, CH, h, 46, -12);
      hit(seq, OH, 14, 64, 74);
      melodicHit(seq, LC, 0, bassRoot, 90, 0, 100, 1, 3);
      break;
    }
    case 3: {
      const uint8_t k[] = {0,6,10};
      const uint8_t s[] = {4,9,12};
      const uint8_t h[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
      hits(seq, BD, k, 114, -2); hits(seq, SD, s, 108, -3); hits(seq, CH, h, 40, -13);
      melodicHit(seq, LC, 0, bassRoot, 92, 0, 100, 1, 3);
      break;
    }
    case 4: {
      const uint8_t k[] = {0,10};
      hits(seq, BD, k, 108, -3);
      melodicHit(seq, LC, 0, bassRoot, 100, 0, 100, 1, 2);
      melodicHit(seq, LC, 8, scaleNote(bassRoot,2), 90, 0, 100, 1, 2);
      const uint8_t h[] = {4,12}; hits(seq, CH, h, 34, -14);
      break;
    }
    case 5: {
      const uint8_t k[] = {0,6,10};
      const uint8_t s[] = {4,9,12};
      const uint8_t h[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
      hits(seq, BD, k, 116, -2); hits(seq, SD, s, 110, -3); hits(seq, CH, h, 44, -12);
      hit(seq, CY, 14, 74, 90, 2); hit(seq, CY, 15, 84, 96, 2);
      melodicHit(seq, LC, 0, bassRoot, 96, 0, 100, 1, 2);
      break;
    }
    case 6: {
      const uint8_t k[] = {0,6,10};
      const uint8_t s[] = {4,9,12};
      const uint8_t h[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
      hits(seq, BD, k, 120, -2); hits(seq, SD, s, 114, -3); hits(seq, CH, h, 48, -11);
      const uint8_t ride[] = {0,4,8,12}; hits(seq, CY, ride, 58, -6);
      hit(seq, CY, 0, 94);
      melodicHit(seq, LC, 0, bassRoot, 104, 0, 100, 1, 2);
      melodicHit(seq, LC, 8, scaleNote(bassRoot,2), 92, 0, 100, 1, 2);
      chordHit(seq, MC, 6, chordRoot, chordRoot+3, chordRoot+6, 54, 78);
      break;
    }
    case 7: {
      hit(seq, BD, 0, 98); hit(seq, SD, 4, 84, 66);
      melodicHit(seq, LC, 0, bassRoot, 74);
      break;
    }
  }
}

void buildExpansionPattern(Sequencer& seq, int p) {
  const int local = p - 20;
  const int songIdx = local / EXP_SCENES;
  const int scene = local % EXP_SCENES;
  const ExpansionSong& song = EXP_SONG[songIdx];

  seq.selectPattern(p);
  seq.clearPattern(p);

  PatternMetadata meta{};
  strncpy(meta.name, EXP_SCENE_NAME[songIdx][scene], sizeof(meta.name) - 1);
  strncpy(meta.genre, song.genre, sizeof(meta.genre) - 1);
  strncpy(meta.kit, song.kit, sizeof(meta.kit) - 1);
  meta.recommendedBpm = song.bpm;
  meta.swing = song.swing;
  meta.humanizeTimingMs = song.timing;
  meta.humanizeVelocity = song.velocity;
  seq.setPatternMetadata(p, meta);

  const uint8_t bassRoot = song.bassRoot;
  const uint8_t chordRoot = song.chordRoot;

  switch (songIdx) {
    case 0: buildHouseScene(seq, scene, bassRoot, chordRoot); break;
    case 1: buildTechnoScene(seq, scene, bassRoot, chordRoot); break;
    case 2: buildBoomBapScene(seq, scene, bassRoot, chordRoot); break;
    case 3: buildTrapScene(seq, scene, bassRoot, chordRoot); break;
    case 4: buildDnbScene(seq, scene, bassRoot, chordRoot); break;
    case 5: buildDubstepScene(seq, scene, bassRoot, chordRoot); break;
    case 6: buildFunkScene(seq, scene, bassRoot, chordRoot); break;
    case 7: buildReggaetonScene(seq, scene, bassRoot, chordRoot); break;
    case 8: buildAfrobeatsScene(seq, scene, bassRoot, chordRoot); break;
    case 9: buildJungleScene(seq, scene, bassRoot, chordRoot); break;
  }
}

} // namespace

void initializeFactoryExpansionBank(Sequencer& sequencer) {
  for (int local = 0; local < EXP_COUNT; ++local) {
    const int pattern = 20 + local;
    buildExpansionPattern(sequencer, pattern);
    const ExpansionSong& song = EXP_SONG[local / EXP_SCENES];
    BuiltinPatternSoundProfile profile{};
    memcpy(profile.engines, song.engines, sizeof(profile.engines));
    memcpy(profile.presets, song.presets, sizeof(profile.presets));
    setPatternSoundProfile(pattern, profile);
  }
  sequencer.selectPattern(0);
}

uint8_t getFactoryPatternKey(int pattern) {
  if (pattern >= 0 && pattern < LEGACY_FACTORY_PATTERN_COUNT)
    return (uint8_t)(FACTORY_PAD8_ROOT[pattern] % 12u);
  if (pattern >= LEGACY_FACTORY_PATTERN_COUNT && pattern < FACTORY_PATTERN_COUNT)
    return (uint8_t)(EXP_SONG[(pattern - LEGACY_FACTORY_PATTERN_COUNT)
                              / EXP_SCENES].bassRoot % 12u);
  return 255;
}
