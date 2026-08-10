/*
 * Sequencer.cpp
 * Implementació del sequencer de 16 steps
 */

#include "Sequencer.h"
#include <esp_heap_caps.h>    // ps_calloc / heap_caps_malloc
#include <stdio.h>

static constexpr uint32_t kDaisyEngineSampleRate = 48000;

// PRNG local (xorshift32) para el hot path de audio. El random() de Arduino
// usa un estado global compartido (no thread-safe entre nucleos: tambien lo
// llama codigo en Core0) y es mas costoso; aqui se llama hasta
// MAX_TRACKS x ratchet veces por step. Calidad sobrada para humanize/probability.
static inline uint32_t seqFastRand() {
  static uint32_t s = 0;
  if (s == 0) s = (uint32_t)micros() | 1u;   // seed lazy, nunca 0
  s ^= s << 13; s ^= s >> 17; s ^= s << 5;
  return s;
}
// Equivalentes a random(n) / random(min, max): rango [0,n) y [min,max)
static inline long seqRand(long n) { return n > 0 ? (long)(seqFastRand() % (uint32_t)n) : 0; }
static inline long seqRand(long mn, long mx) { return mn + seqRand(mx - mn); }

Sequencer::Sequencer() : 
  playing(false), 
  currentPattern(0), 
  currentStep(0), 
  queuedPattern(-1),
  performanceReturnPattern(-1),
  queuedPatternBars(0),
  performanceBarsRemaining(0),
  performancePatternActive(false),
  tempo(120.0f),
  lastStepTime(0),
  nextStepInterval(0),
  humanizeTimingMs(0),
  humanizeVelocityAmount(0),
  stepCallback(nullptr),
  stepAutomationCallback(nullptr),
  stepChangeCallback(nullptr),
  patternChangeCallback(nullptr),
  songMode(false),
  songLength(1),
  songChainActive(false),
  songChainCount(0),
  songChainIdx(0),
  songChainRepeatCnt(0),
  patternLength(16) {

  memset(songChain, 0, sizeof(songChain));

  patternMutex = xSemaphoreCreateRecursiveMutex();
  stateMutex = xSemaphoreCreateRecursiveMutex();

  // ── Allocate pattern storage in PSRAM. Internal DRAM is too small for this
  // block plus pattern/UI state, so PSRAM is a hard requirement.
  pd = (PatternData*)ps_calloc(1, sizeof(PatternData));
  if (!pd) {
    Serial.println("[SEQ] FATAL: PSRAM allocation failed for PatternData");
    delay(100);
    ESP.restart();
  }
  // steps[] already zeroed by calloc; set non-zero defaults
  for (int p = 0; p < MAX_PATTERNS; p++) {
    snprintf(pd->metadata[p].name, sizeof(pd->metadata[p].name), "Pattern %d", p + 1);
    strncpy(pd->metadata[p].genre, "User", sizeof(pd->metadata[p].genre) - 1);
    strncpy(pd->metadata[p].kit, "Hybrid", sizeof(pd->metadata[p].kit) - 1);
    pd->metadata[p].recommendedBpm = 120;
    // Daisy interpreta swing como cantidad de desplazamiento: 0 = recto.
    pd->metadata[p].swing = 0;
    pd->metadata[p].humanizeTimingMs = 0;
    pd->metadata[p].humanizeVelocity = 0;
    for (int t = 0; t < MAX_TRACKS; t++) {
      for (int s = 0; s < STEPS_PER_PATTERN; s++) {
        pd->velocities[p][t][s] = 127;
        pd->noteLenDivs[p][t][s] = 1;  // Default: full note
        pd->probabilities[p][t][s] = 100;
        pd->ratchets[p][t][s] = 1;
        pd->stepVolumeLockEnabled[p][t][s] = false;
        pd->stepVolumeLockValue[p][t][s] = 100;
        pd->stepCutoffLockEnabled[p][t][s] = false;
        pd->stepCutoffLockHz[p][t][s] = 1000;
        pd->stepReverbSendLockEnabled[p][t][s] = false;
        pd->stepReverbSendLockValue[p][t][s] = 0;
      }
    }
  }
  
  for (int t = 0; t < MAX_TRACKS; t++) {
    trackMuted[t] = false;
    trackVolume[t] = 100;
    loopActive[t] = false;
    loopPaused[t] = false;
    loopType[t] = LOOP_EVERY_STEP;
    loopStepCounter[t] = 0;
  }
  
  calculateStepInterval();
  nextStepInterval = stepInterval;
}

Sequencer::~Sequencer() {
  if (pd) free(pd);
  if (patternMutex) vSemaphoreDelete(patternMutex);
  if (stateMutex) vSemaphoreDelete(stateMutex);
}

void Sequencer::start() {
  lockState();
  playing = true;
  lastStepTime = micros();
  unlockState();
}

void Sequencer::stop() {
  lockState();
  playing = false;
  unlockState();
}

void Sequencer::reset() {
  lockState();
  currentStep = 0;
  lastStepTime = micros();
  unlockState();
}

bool Sequencer::isPlaying() {
  lockState();
  const bool value = playing;
  unlockState();
  return value;
}

void Sequencer::setTempo(float bpm) {
  if (bpm < 40.0f) bpm = 40.0f;
  if (bpm > 300.0f) bpm = 300.0f;
  lockState();
  tempo = bpm;
  calculateStepInterval();
  unlockState();
}

float Sequencer::getTempo() {
  lockState();
  const float value = tempo;
  unlockState();
  return value;
}

void Sequencer::calculateStepInterval() {
  // 16th notes
  // 1 beat = 60/BPM seconds
  // 1 16th note = (60/BPM) / 4 seconds
  // Convert to microseconds
  stepInterval = (uint32_t)((60.0f / tempo / 4.0f) * 1000000.0f);
  if (nextStepInterval == 0) nextStepInterval = stepInterval;
}

void Sequencer::update() {
  uint32_t now = micros();
  int stepToProcess = 0;
  int patternToProcess = 0;
  lockState();
  const uint32_t intervalToUse = nextStepInterval > 0 ? nextStepInterval : stepInterval;
  if (!playing || now - lastStepTime < intervalToUse) {
    unlockState();
    return;
  }
  lastStepTime = now;
  stepToProcess = currentStep;
  patternToProcess = currentPattern;
  unlockState();

  // Notify and run callbacks outside stateMutex; they may enqueue SPI and must
  // never block a Core0 control operation while doing so.
  if (stepChangeCallback != nullptr) stepChangeCallback(stepToProcess);
  processStep(patternToProcess, stepToProcess);

  bool patternChanged = false;
  int changedPattern = 0;
  int changedLength = 0;
  lockState();
  // A concurrent reset/select may have changed the transport while callbacks
  // ran. Only advance the step we actually processed.
  if (currentStep == stepToProcess) {
    currentStep = currentStep + 1;
    if (currentStep >= patternLength) {
      currentStep = 0;

      // Live queue has priority over song modes. A one-bar scene (FILL/VAR)
      // automatically returns at the following wrap. Kick/bass safety for VAR
      // is handled when its temporary scene is prepared.
      if (performancePatternActive && performanceReturnPattern >= 0) {
        if (performanceBarsRemaining > 1) {
          performanceBarsRemaining = (uint8_t)(performanceBarsRemaining - 1u);
        } else {
          currentPattern = performanceReturnPattern;
          performanceReturnPattern = -1;
          performanceBarsRemaining = 0;
          performancePatternActive = false;
          patternChanged = true;
          changedPattern = currentPattern;
          changedLength = 1;
        }
      } else if (queuedPattern >= 0) {
        const int next = queuedPattern;
        const uint8_t bars = queuedPatternBars;
        queuedPattern = -1;
        queuedPatternBars = 0;
        if (bars > 0) {
          performanceReturnPattern = currentPattern;
          performanceBarsRemaining = bars;
          performancePatternActive = true;
        }
        currentPattern = next;
        patternChanged = true;
        changedPattern = currentPattern;
        changedLength = 1;
      }

      // Song Chain mode: custom pattern chain with repeats
      if (!patternChanged && songChainActive && songChainCount > 0) {
        songChainRepeatCnt++;
        uint8_t needed = songChain[songChainIdx].repeats;
        if (needed == 0) needed = 1;
        if (songChainRepeatCnt >= needed) {
          // Advance to next entry in chain
          songChainIdx++;
          songChainRepeatCnt = 0;
          if (songChainIdx >= songChainCount) {
            // Chain ended — stop song chain
            songChainActive = false;
          } else {
            currentPattern = songChain[songChainIdx].pattern % MAX_PATTERNS;
            patternChanged = true;
            changedPattern = currentPattern;
            changedLength = songChainCount;
          }
        }
      }
      // Legacy song mode: linear pattern advance
      else if (!patternChanged && songMode && songLength > 1) {
        int nextPattern = currentPattern + 1;
        if (nextPattern >= songLength) {
          nextPattern = 0; // Loop back to start
        }
        currentPattern = nextPattern;
        patternChanged = true;
        changedPattern = currentPattern;
        changedLength = songLength;
      }
    }

    if (humanizeTimingMs > 0) {
      int32_t jitterUs = (int32_t)seqRand(-(long)humanizeTimingMs, (long)humanizeTimingMs + 1) * 1000;
      int32_t candidate = (int32_t)stepInterval + jitterUs;
      int32_t minStep = (int32_t)stepInterval / 2;
      if (candidate < minStep) candidate = minStep;
      nextStepInterval = (uint32_t)candidate;
    } else {
      nextStepInterval = stepInterval;
    }
  }
  unlockState();
  if (patternChanged && patternChangeCallback) {
    patternChangeCallback(changedPattern, changedLength);
  }
}

void Sequencer::processStep(int pat, int st) {
  // First: Process looped tracks
  processLoops();

  // Snapshot del step que va a disparar, bajo lock. Asi una edicion
  // concurrente del patron (setPatternBulk/clearPattern desde la tarea
  // otra tarea no puede "romper" los datos que estamos a punto de disparar.
  // Los callbacks (que hacen SPI) se ejecutan FUERA del lock.
  struct StepSnap {
    bool     active;
    uint8_t  velocity;
    uint8_t  div;
    uint8_t  ratchet;
    uint8_t  probability;
    bool     volLockEn;
    uint8_t  volLockVal;
    bool     cutoffEn;
    uint16_t cutoffHz;
    bool     revEn;
    uint8_t  revVal;
    bool     muted;
    uint8_t  trackVol;
  } snap[MAX_TRACKS];

  uint32_t intervalUs = 0;
  uint8_t velocityHumanize = 0;
  lockState();
  intervalUs = stepInterval;
  velocityHumanize = humanizeVelocityAmount;
  for (int track = 0; track < MAX_TRACKS; ++track) {
    snap[track].muted = trackMuted[track];
    snap[track].trackVol = trackVolume[track];
  }
  unlockState();

  lockPattern();
  for (int track = 0; track < MAX_TRACKS; track++) {
    snap[track].active      = pd->steps[pat][track][st];
    snap[track].velocity    = pd->velocities[pat][track][st];
    snap[track].div         = pd->noteLenDivs[pat][track][st];
    snap[track].ratchet     = pd->ratchets[pat][track][st];
    snap[track].probability = pd->probabilities[pat][track][st];
    snap[track].volLockEn   = pd->stepVolumeLockEnabled[pat][track][st];
    snap[track].volLockVal  = pd->stepVolumeLockValue[pat][track][st];
    snap[track].cutoffEn    = pd->stepCutoffLockEnabled[pat][track][st];
    snap[track].cutoffHz    = pd->stepCutoffLockHz[pat][track][st];
    snap[track].revEn       = pd->stepReverbSendLockEnabled[pat][track][st];
    snap[track].revVal      = pd->stepReverbSendLockValue[pat][track][st];
  }
  unlockPattern();

  // Trigger all active tracks at current step (desde el snapshot)
  for (int track = 0; track < MAX_TRACKS; track++) {
    // Check sequencer steps
    if (snap[track].active && !snap[track].muted) {
      uint8_t probability = snap[track].probability;
      if (probability < 100) {
        long roll = seqRand(0, 100);
        if (roll >= probability) {
          continue;
        }
      }

      uint8_t velocity = snap[track].velocity;
      uint8_t div = snap[track].div;
      uint8_t ratchet = snap[track].ratchet;
      if (ratchet < 1) ratchet = 1;
      if (ratchet > 4) ratchet = 4;
      uint8_t outTrackVolume = snap[track].volLockEn
                ? snap[track].volLockVal
                : snap[track].trackVol;

      if (stepAutomationCallback != nullptr) {
        stepAutomationCallback(
          track,
          st,
          snap[track].cutoffEn,
          snap[track].cutoffHz,
          snap[track].revEn,
          snap[track].revVal,
          snap[track].volLockEn,
          outTrackVolume
        );
      }

      // Compute max samples for note length (0 = full sample)
      uint32_t noteLenSamples = 0;
      if (div > 1) {
        noteLenSamples = (uint32_t)(((uint64_t)intervalUs * kDaisyEngineSampleRate) / ((uint32_t)div * 1000000UL));
        if (noteLenSamples < 64) noteLenSamples = 64;  // minimum
      }
      
      // Call callback if set
      if (stepCallback != nullptr) {
        for (uint8_t r = 0; r < ratchet; r++) {
          uint8_t outVelocity = velocity;
          if (velocityHumanize > 0) {
            int maxDelta = (int)((127 * velocityHumanize) / 100);
            int jitter = (int)seqRand(-maxDelta, maxDelta + 1);
            int v = (int)velocity + jitter;
            if (v < 1) v = 1;
            if (v > 127) v = 127;
            outVelocity = (uint8_t)v;
          }

          uint32_t subNoteLen = noteLenSamples;
          if (ratchet > 1 && noteLenSamples > 0) {
            subNoteLen = noteLenSamples / ratchet;
            if (subNoteLen < 64) subNoteLen = 64;
          }
          stepCallback(track, outVelocity, outTrackVolume, subNoteLen);
        }
      }
    }
  }
}

void Sequencer::setStep(int track, int step, bool active, uint8_t velocity) {
  if (track < 0 || track >= MAX_TRACKS) return;
  if (step < 0 || step >= STEPS_PER_PATTERN) return;

  lockPattern();
  pd->steps[currentPattern][track][step] = active;
  pd->velocities[currentPattern][track][step] = velocity;
  unlockPattern();
}

bool Sequencer::getStep(int track, int step) {
  if (track < 0 || track >= MAX_TRACKS) return false;
  if (step < 0 || step >= STEPS_PER_PATTERN) return false;

  lockPattern();
  bool v = pd->steps[currentPattern][track][step];
  unlockPattern();
  return v;
}

bool Sequencer::getStep(int pattern, int track, int step) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return false;
  if (track < 0 || track >= MAX_TRACKS) return false;
  if (step < 0 || step >= STEPS_PER_PATTERN) return false;

  lockPattern();
  bool v = pd->steps[pattern][track][step];
  unlockPattern();
  return v;
}

void Sequencer::clearPattern(int pattern) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return;

  lockPattern();
  // Bulk memset — orders of magnitude faster than element-by-element on PSRAM
  memset(pd->steps[pattern],                  0, sizeof(pd->steps[pattern]));           // false
  memset(pd->velocities[pattern],           127, sizeof(pd->velocities[pattern]));      // 127
  memset(pd->noteLenDivs[pattern],            1, sizeof(pd->noteLenDivs[pattern]));     // 1
  memset(pd->probabilities[pattern],        100, sizeof(pd->probabilities[pattern]));   // 100
  memset(pd->ratchets[pattern],               1, sizeof(pd->ratchets[pattern]));        // 1
  memset(pd->stepVolumeLockEnabled[pattern],  0, sizeof(pd->stepVolumeLockEnabled[pattern]));
  memset(pd->stepVolumeLockValue[pattern],  100, sizeof(pd->stepVolumeLockValue[pattern]));
  memset(pd->stepCutoffLockEnabled[pattern],  0, sizeof(pd->stepCutoffLockEnabled[pattern]));
  memset(pd->stepReverbSendLockEnabled[pattern], 0, sizeof(pd->stepReverbSendLockEnabled[pattern]));
  memset(pd->stepReverbSendLockValue[pattern],   0, sizeof(pd->stepReverbSendLockValue[pattern]));
  memset(pd->stepNotes[pattern],                  0, sizeof(pd->stepNotes[pattern]));              // 0 = rest
  memset(pd->stepNoteVoices[pattern],             0, sizeof(pd->stepNoteVoices[pattern]));
  memset(pd->stepFlags[pattern],                  0, sizeof(pd->stepFlags[pattern]));              // no accent/slide
  // stepCutoffLockHz is uint16_t — memset can't set 1000, need loop
  for (int t = 0; t < MAX_TRACKS; t++) {
    for (int s = 0; s < STEPS_PER_PATTERN; s++) {
      pd->stepCutoffLockHz[pattern][t][s] = 1000;
    }
  }
  unlockPattern();
}

void Sequencer::clearPattern() {
  clearPattern(currentPattern);
}

void Sequencer::clearTrack(int track) {
  if (track < 0 || track >= MAX_TRACKS) return;

  lockPattern();
  for (int s = 0; s < STEPS_PER_PATTERN; s++) {
    pd->steps[currentPattern][track][s] = false;
  }
  unlockPattern();
}

void Sequencer::snapshotTrackForUpload(int pattern, int track, int stepCount, StepUploadData* out) {
  if (!out) return;
  if (pattern < 0 || pattern >= MAX_PATTERNS) return;
  if (track < 0 || track >= MAX_TRACKS) return;
  if (stepCount < 0) stepCount = 0;
  if (stepCount > STEPS_PER_PATTERN) stepCount = STEPS_PER_PATTERN;
  // Copia toda la pista bajo UN solo lock: el resultado es internamente
  // consistente aunque la web edite otra pista entre llamadas (cada pista se
  // sube por separado, asi que esta es la granularidad correcta).
  lockPattern();
  for (int s = 0; s < stepCount; s++) {
    out[s].active      = pd->steps[pattern][track][s];
    out[s].velocity    = pd->velocities[pattern][track][s];
    out[s].noteLenDiv  = pd->noteLenDivs[pattern][track][s];
    out[s].probability = pd->probabilities[pattern][track][s];
    out[s].ratchet     = pd->ratchets[pattern][track][s];
    out[s].flags       = pd->stepFlags[pattern][track][s];
    memcpy(out[s].noteVoices, pd->stepNoteVoices[pattern][track][s],
           sizeof(out[s].noteVoices));
    out[s].cutoffEn    = pd->stepCutoffLockEnabled[pattern][track][s];
    out[s].cutoffHz    = pd->stepCutoffLockHz[pattern][track][s];
    out[s].reverbEn    = pd->stepReverbSendLockEnabled[pattern][track][s];
    out[s].reverbSend  = pd->stepReverbSendLockValue[pattern][track][s];
    out[s].volumeEn    = pd->stepVolumeLockEnabled[pattern][track][s];
    out[s].volume      = pd->stepVolumeLockValue[pattern][track][s];
  }
  unlockPattern();
}

bool Sequencer::snapshotPatternForStorage(int pattern, PatternStorageData* out) {
  if (!out || pattern < 0 || pattern >= MAX_PATTERNS) return false;
  lockPattern();
  out->metadata = pd->metadata[pattern];
  for (int t = 0; t < MAX_TRACKS; ++t) {
    for (int s = 0; s < STEPS_PER_PATTERN; ++s) {
      PatternStorageStep& dst = out->steps[t][s];
      dst.active       = pd->steps[pattern][t][s] ? 1u : 0u;
      dst.velocity     = pd->velocities[pattern][t][s];
      dst.noteLenDiv   = pd->noteLenDivs[pattern][t][s];
      dst.probability  = pd->probabilities[pattern][t][s];
      dst.ratchet      = pd->ratchets[pattern][t][s];
      dst.flags        = pd->stepFlags[pattern][t][s];
      memcpy(dst.noteVoices, pd->stepNoteVoices[pattern][t][s],
             sizeof(dst.noteVoices));
      dst.cutoffEn     = pd->stepCutoffLockEnabled[pattern][t][s] ? 1u : 0u;
      dst.cutoffHz     = pd->stepCutoffLockHz[pattern][t][s];
      dst.reverbEn     = pd->stepReverbSendLockEnabled[pattern][t][s] ? 1u : 0u;
      dst.reverbSend   = pd->stepReverbSendLockValue[pattern][t][s];
      dst.volumeEn     = pd->stepVolumeLockEnabled[pattern][t][s] ? 1u : 0u;
      dst.volume       = pd->stepVolumeLockValue[pattern][t][s];
    }
  }
  unlockPattern();
  return true;
}

bool Sequencer::restorePatternFromStorage(int pattern, const PatternStorageData* in) {
  if (!in || pattern < 0 || pattern >= MAX_PATTERNS) return false;
  lockPattern();
  pd->metadata[pattern] = in->metadata;
  pd->metadata[pattern].name[sizeof(pd->metadata[pattern].name) - 1] = '\0';
  pd->metadata[pattern].genre[sizeof(pd->metadata[pattern].genre) - 1] = '\0';
  pd->metadata[pattern].kit[sizeof(pd->metadata[pattern].kit) - 1] = '\0';
  for (int t = 0; t < MAX_TRACKS; ++t) {
    for (int s = 0; s < STEPS_PER_PATTERN; ++s) {
      const PatternStorageStep& src = in->steps[t][s];
      pd->steps[pattern][t][s] = src.active != 0;
      pd->velocities[pattern][t][s] = constrain(src.velocity, 1, 127);
      pd->noteLenDivs[pattern][t][s] = src.noteLenDiv;
      pd->probabilities[pattern][t][s] = constrain(src.probability, 0, 100);
      pd->ratchets[pattern][t][s] = constrain(src.ratchet, 1, 4);
      pd->stepFlags[pattern][t][s] = src.flags;
      memcpy(pd->stepNoteVoices[pattern][t][s], src.noteVoices,
             sizeof(src.noteVoices));
      pd->stepNotes[pattern][t][s] = src.noteVoices[0];
      pd->stepCutoffLockEnabled[pattern][t][s] = src.cutoffEn != 0;
      pd->stepCutoffLockHz[pattern][t][s] = constrain(src.cutoffHz, 20, 20000);
      pd->stepReverbSendLockEnabled[pattern][t][s] = src.reverbEn != 0;
      pd->stepReverbSendLockValue[pattern][t][s] = src.reverbSend;
      pd->stepVolumeLockEnabled[pattern][t][s] = src.volumeEn != 0;
      pd->stepVolumeLockValue[pattern][t][s] = constrain(src.volume, 0, 150);
    }
  }
  unlockPattern();
  return true;
}

// ============= VELOCITY EDITING =============

void Sequencer::setStepVelocity(int track, int step, uint8_t velocity) {
  if (track < 0 || track >= MAX_TRACKS) return;
  if (step < 0 || step >= STEPS_PER_PATTERN) return;
  lockPattern();
  pd->velocities[currentPattern][track][step] = constrain(velocity, 1, 127);
  unlockPattern();
}

void Sequencer::setStepVelocity(int pattern, int track, int step, uint8_t velocity) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return;
  if (track < 0 || track >= MAX_TRACKS) return;
  if (step < 0 || step >= STEPS_PER_PATTERN) return;
  lockPattern();
  pd->velocities[pattern][track][step] = constrain(velocity, 1, 127);
  unlockPattern();
}

uint8_t Sequencer::getStepVelocity(int track, int step) {
  if (track < 0 || track >= MAX_TRACKS) return 127;
  if (step < 0 || step >= STEPS_PER_PATTERN) return 127;
  lockPattern();
  uint8_t v = pd->velocities[currentPattern][track][step];
  unlockPattern();
  return v;
}

uint8_t Sequencer::getStepVelocity(int pattern, int track, int step) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return 127;
  if (track < 0 || track >= MAX_TRACKS) return 127;
  if (step < 0 || step >= STEPS_PER_PATTERN) return 127;
  lockPattern();
  uint8_t v = pd->velocities[pattern][track][step];
  unlockPattern();
  return v;
}

void Sequencer::setStepNoteLen(int track, int step, uint8_t div) {
  setStepNoteLen(currentPattern, track, step, div);
}

void Sequencer::setStepNoteLen(int pattern, int track, int step, uint8_t div) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return;
  if (track < 0 || track >= MAX_TRACKS) return;
  if (step < 0 || step >= STEPS_PER_PATTERN) return;
  if (div == 0) div = 1;  // Sanitize
  lockPattern();
  pd->noteLenDivs[pattern][track][step] = div;
  unlockPattern();
}

uint8_t Sequencer::getStepNoteLen(int track, int step) {
  if (track < 0 || track >= MAX_TRACKS) return 1;
  if (step < 0 || step >= STEPS_PER_PATTERN) return 1;
  lockPattern();
  uint8_t v = pd->noteLenDivs[currentPattern][track][step];
  unlockPattern();
  return v;
}

uint8_t Sequencer::getStepNoteLen(int pattern, int track, int step) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return 1;
  if (track < 0 || track >= MAX_TRACKS) return 1;
  if (step < 0 || step >= STEPS_PER_PATTERN) return 1;
  lockPattern();
  uint8_t v = pd->noteLenDivs[pattern][track][step];
  unlockPattern();
  return v;
}

// ============= MELODY NOTE PER STEP =============

void Sequencer::setStepNote(int track, int step, uint8_t midiNote) {
  if (track < 0 || track >= MAX_TRACKS) return;
  if (step < 0 || step >= STEPS_PER_PATTERN) return;
  lockPattern();
  pd->stepNotes[currentPattern][track][step] = midiNote;
  pd->stepNoteVoices[currentPattern][track][step][0] = midiNote;
  unlockPattern();
}

void Sequencer::setStepNote(int pattern, int track, int step, uint8_t midiNote) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return;
  if (track < 0 || track >= MAX_TRACKS) return;
  if (step < 0 || step >= STEPS_PER_PATTERN) return;
  lockPattern();
  pd->stepNotes[pattern][track][step] = midiNote;
  pd->stepNoteVoices[pattern][track][step][0] = midiNote;
  unlockPattern();
}

uint8_t Sequencer::getStepNote(int track, int step) {
  if (track < 0 || track >= MAX_TRACKS) return 0;
  if (step < 0 || step >= STEPS_PER_PATTERN) return 0;
  lockPattern();
  uint8_t v = pd->stepNotes[currentPattern][track][step];
  unlockPattern();
  return v;
}

uint8_t Sequencer::getStepNote(int pattern, int track, int step) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return 0;
  if (track < 0 || track >= MAX_TRACKS) return 0;
  if (step < 0 || step >= STEPS_PER_PATTERN) return 0;
  lockPattern();
  uint8_t v = pd->stepNotes[pattern][track][step];
  unlockPattern();
  return v;
}

void Sequencer::setStepNoteVoice(int track, int step, int voice, uint8_t midiNote) {
  setStepNoteVoice(currentPattern, track, step, voice, midiNote);
}

void Sequencer::setStepNoteVoice(int pattern, int track, int step, int voice, uint8_t midiNote) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return;
  if (track < 0 || track >= MAX_TRACKS) return;
  if (step < 0 || step >= STEPS_PER_PATTERN) return;
  if (voice < 0 || voice >= MELODY_STEP_VOICES) return;
  lockPattern();
  pd->stepNoteVoices[pattern][track][step][voice] = midiNote;
  if (voice == 0) pd->stepNotes[pattern][track][step] = midiNote;
  unlockPattern();
}

uint8_t Sequencer::getStepNoteVoice(int track, int step, int voice) {
  return getStepNoteVoice(currentPattern, track, step, voice);
}

uint8_t Sequencer::getStepNoteVoice(int pattern, int track, int step, int voice) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return 0;
  if (track < 0 || track >= MAX_TRACKS) return 0;
  if (step < 0 || step >= STEPS_PER_PATTERN) return 0;
  if (voice < 0 || voice >= MELODY_STEP_VOICES) return 0;
  lockPattern();
  uint8_t v = pd->stepNoteVoices[pattern][track][step][voice];
  unlockPattern();
  return v;
}

void Sequencer::clearStepNoteVoices(int track, int step) {
  clearStepNoteVoices(currentPattern, track, step);
}

void Sequencer::clearStepNoteVoices(int pattern, int track, int step) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return;
  if (track < 0 || track >= MAX_TRACKS) return;
  if (step < 0 || step >= STEPS_PER_PATTERN) return;
  lockPattern();
  memset(pd->stepNoteVoices[pattern][track][step], 0, MELODY_STEP_VOICES);
  pd->stepNotes[pattern][track][step] = 0;
  unlockPattern();
}

void Sequencer::setStepFlags(int track, int step, uint8_t flags) {
  if (track < 0 || track >= MAX_TRACKS) return;
  if (step < 0 || step >= STEPS_PER_PATTERN) return;
  lockPattern();
  pd->stepFlags[currentPattern][track][step] = flags;
  unlockPattern();
}

void Sequencer::setStepFlags(int pattern, int track, int step, uint8_t flags) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return;
  if (track < 0 || track >= MAX_TRACKS) return;
  if (step < 0 || step >= STEPS_PER_PATTERN) return;
  lockPattern();
  pd->stepFlags[pattern][track][step] = flags;
  unlockPattern();
}

uint8_t Sequencer::getStepFlags(int track, int step) {
  if (track < 0 || track >= MAX_TRACKS) return 0;
  if (step < 0 || step >= STEPS_PER_PATTERN) return 0;
  lockPattern();
  uint8_t v = pd->stepFlags[currentPattern][track][step];
  unlockPattern();
  return v;
}

uint8_t Sequencer::getStepFlags(int pattern, int track, int step) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return 0;
  if (track < 0 || track >= MAX_TRACKS) return 0;
  if (step < 0 || step >= STEPS_PER_PATTERN) return 0;
  lockPattern();
  uint8_t v = pd->stepFlags[pattern][track][step];
  unlockPattern();
  return v;
}

void Sequencer::setPatternBulk(int pattern, const bool stepsData[MAX_TRACKS][STEPS_PER_PATTERN], const uint8_t velsData[MAX_TRACKS][STEPS_PER_PATTERN]) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return;

  lockPattern();
  for (int t = 0; t < MAX_TRACKS; t++) {
    for (int s = 0; s < STEPS_PER_PATTERN; s++) {
      pd->steps[pattern][t][s] = stepsData[t][s];
      pd->velocities[pattern][t][s] = velsData[t][s];
      pd->noteLenDivs[pattern][t][s] = 1;
      pd->probabilities[pattern][t][s] = 100;
      pd->ratchets[pattern][t][s] = 1;
      pd->stepVolumeLockEnabled[pattern][t][s] = false;
      pd->stepVolumeLockValue[pattern][t][s] = 100;
      pd->stepCutoffLockEnabled[pattern][t][s] = false;
      pd->stepCutoffLockHz[pattern][t][s] = 1000;
      pd->stepReverbSendLockEnabled[pattern][t][s] = false;
      pd->stepReverbSendLockValue[pattern][t][s] = 0;
      pd->stepNotes[pattern][t][s] = 0;
      memset(pd->stepNoteVoices[pattern][t][s], 0,
             sizeof(pd->stepNoteVoices[pattern][t][s]));
      pd->stepFlags[pattern][t][s] = 0;
    }
  }
  unlockPattern();
}

void Sequencer::selectPattern(int pattern) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return;
  lockState();
  currentPattern = pattern;
  queuedPattern = -1;
  queuedPatternBars = 0;
  performanceReturnPattern = -1;
  performanceBarsRemaining = 0;
  performancePatternActive = false;
  unlockState();
}

void Sequencer::setStep(int pattern, int track, int step, bool active, uint8_t velocity) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return;
  if (track < 0 || track >= MAX_TRACKS) return;
  if (step < 0 || step >= STEPS_PER_PATTERN) return;
  lockPattern();
  pd->steps[pattern][track][step] = active;
  pd->velocities[pattern][track][step] = velocity;
  unlockPattern();
}

void Sequencer::queuePattern(int pattern) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return;
  lockState();
  queuedPattern = pattern;
  queuedPatternBars = 0;
  unlockState();
}

void Sequencer::queueOneBarPattern(int pattern) {
  queuePatternForBars(pattern, 1);
}

void Sequencer::queuePatternForBars(int pattern, uint8_t bars) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return;
  lockState();
  queuedPattern = pattern;
  queuedPatternBars = constrain(bars, 1, 16);
  unlockState();
}

int Sequencer::getQueuedPattern() {
  lockState();
  const int value = queuedPattern;
  unlockState();
  return value;
}

void Sequencer::cancelQueuedPattern() {
  lockState();
  queuedPattern = -1;
  queuedPatternBars = 0;
  unlockState();
}

int Sequencer::getPerformancePattern() {
  lockState();
  const int value = (performancePatternActive && performanceReturnPattern >= 0)
      ? performanceReturnPattern : currentPattern;
  unlockState();
  return value;
}

void Sequencer::muteTrack(int track, bool muted) {
  if (track >= 0 && track < MAX_TRACKS) {
    lockState();
    trackMuted[track] = muted;
    unlockState();
  }
}

bool Sequencer::isTrackMuted(int track) {
  if (track >= 0 && track < MAX_TRACKS) {
    lockState();
    const bool value = trackMuted[track];
    unlockState();
    return value;
  }
  return false;
}

void Sequencer::setTrackVolume(int track, uint8_t volume) {
  if (track >= 0 && track < MAX_TRACKS) {
    lockState();
    trackVolume[track] = constrain(volume, 0, 150);
    unlockState();
  }
}

uint8_t Sequencer::getTrackVolume(int track) {
  if (track >= 0 && track < MAX_TRACKS) {
    lockState();
    const uint8_t value = trackVolume[track];
    unlockState();
    return value;
  }
  return 100; // Default volume
}

int Sequencer::getCurrentPattern() {
  lockState();
  const int value = currentPattern;
  unlockState();
  return value;
}

void Sequencer::copyPattern(int src, int dst) {
  if (src < 0 || src >= MAX_PATTERNS) return;
  if (dst < 0 || dst >= MAX_PATTERNS) return;

  lockPattern();
  pd->metadata[dst] = pd->metadata[src];
  for (int t = 0; t < MAX_TRACKS; t++) {
    for (int s = 0; s < STEPS_PER_PATTERN; s++) {
      pd->steps[dst][t][s] = pd->steps[src][t][s];
      pd->velocities[dst][t][s] = pd->velocities[src][t][s];
      pd->noteLenDivs[dst][t][s] = pd->noteLenDivs[src][t][s];
      pd->probabilities[dst][t][s] = pd->probabilities[src][t][s];
      pd->ratchets[dst][t][s] = pd->ratchets[src][t][s];
      pd->stepVolumeLockEnabled[dst][t][s] = pd->stepVolumeLockEnabled[src][t][s];
      pd->stepVolumeLockValue[dst][t][s] = pd->stepVolumeLockValue[src][t][s];
      pd->stepCutoffLockEnabled[dst][t][s] = pd->stepCutoffLockEnabled[src][t][s];
      pd->stepCutoffLockHz[dst][t][s] = pd->stepCutoffLockHz[src][t][s];
      pd->stepReverbSendLockEnabled[dst][t][s] = pd->stepReverbSendLockEnabled[src][t][s];
      pd->stepReverbSendLockValue[dst][t][s] = pd->stepReverbSendLockValue[src][t][s];
      pd->stepNotes[dst][t][s] = pd->stepNotes[src][t][s];
      memcpy(pd->stepNoteVoices[dst][t][s], pd->stepNoteVoices[src][t][s], MELODY_STEP_VOICES);
      pd->stepFlags[dst][t][s] = pd->stepFlags[src][t][s];
    }
  }
  unlockPattern();
}

int Sequencer::getCurrentStep() {
  return currentStep;
}

void Sequencer::setStepVolumeLock(int track, int step, bool enabled, uint8_t volume) {
  if (track < 0 || track >= MAX_TRACKS) return;
  if (step < 0 || step >= STEPS_PER_PATTERN) return;
  lockPattern();
  pd->stepVolumeLockEnabled[currentPattern][track][step] = enabled;
  pd->stepVolumeLockValue[currentPattern][track][step] = constrain(volume, 0, 150);
  unlockPattern();
}

void Sequencer::setStepVolumeLock(int pattern, int track, int step, bool enabled, uint8_t volume) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return;
  if (track < 0 || track >= MAX_TRACKS) return;
  if (step < 0 || step >= STEPS_PER_PATTERN) return;
  lockPattern();
  pd->stepVolumeLockEnabled[pattern][track][step] = enabled;
  pd->stepVolumeLockValue[pattern][track][step] = constrain(volume, 0, 150);
  unlockPattern();
}

bool Sequencer::hasStepVolumeLock(int track, int step) {
  if (track < 0 || track >= MAX_TRACKS) return false;
  if (step < 0 || step >= STEPS_PER_PATTERN) return false;
  lockPattern();
  bool v = pd->stepVolumeLockEnabled[currentPattern][track][step];
  unlockPattern();
  return v;
}

bool Sequencer::hasStepVolumeLock(int pattern, int track, int step) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return false;
  if (track < 0 || track >= MAX_TRACKS) return false;
  if (step < 0 || step >= STEPS_PER_PATTERN) return false;
  lockPattern();
  bool v = pd->stepVolumeLockEnabled[pattern][track][step];
  unlockPattern();
  return v;
}

uint8_t Sequencer::getStepVolumeLock(int track, int step) {
  // lock recursivo: hasStepVolumeLock vuelve a tomarlo en la misma task
  lockPattern();
  uint8_t v = hasStepVolumeLock(track, step) ? pd->stepVolumeLockValue[currentPattern][track][step] : 0;
  unlockPattern();
  return v;
}

uint8_t Sequencer::getStepVolumeLock(int pattern, int track, int step) {
  lockPattern();
  uint8_t v = hasStepVolumeLock(pattern, track, step) ? pd->stepVolumeLockValue[pattern][track][step] : 0;
  unlockPattern();
  return v;
}

void Sequencer::setStepCutoffLock(int track, int step, bool enabled, uint16_t cutoffHz) {
  if (track < 0 || track >= MAX_TRACKS) return;
  if (step < 0 || step >= STEPS_PER_PATTERN) return;
  cutoffHz = constrain(cutoffHz, 20, 20000);
  lockPattern();
  pd->stepCutoffLockEnabled[currentPattern][track][step] = enabled;
  pd->stepCutoffLockHz[currentPattern][track][step] = cutoffHz;
  unlockPattern();
}

void Sequencer::setStepCutoffLock(int pattern, int track, int step, bool enabled, uint16_t cutoffHz) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return;
  if (track < 0 || track >= MAX_TRACKS) return;
  if (step < 0 || step >= STEPS_PER_PATTERN) return;
  cutoffHz = constrain(cutoffHz, 20, 20000);
  lockPattern();
  pd->stepCutoffLockEnabled[pattern][track][step] = enabled;
  pd->stepCutoffLockHz[pattern][track][step] = cutoffHz;
  unlockPattern();
}

bool Sequencer::hasStepCutoffLock(int track, int step) {
  if (track < 0 || track >= MAX_TRACKS) return false;
  if (step < 0 || step >= STEPS_PER_PATTERN) return false;
  lockPattern();
  bool v = pd->stepCutoffLockEnabled[currentPattern][track][step];
  unlockPattern();
  return v;
}

bool Sequencer::hasStepCutoffLock(int pattern, int track, int step) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return false;
  if (track < 0 || track >= MAX_TRACKS) return false;
  if (step < 0 || step >= STEPS_PER_PATTERN) return false;
  lockPattern();
  bool value = pd->stepCutoffLockEnabled[pattern][track][step];
  unlockPattern();
  return value;
}

uint16_t Sequencer::getStepCutoffLock(int track, int step) {
  if (track < 0 || track >= MAX_TRACKS) return 1000;
  if (step < 0 || step >= STEPS_PER_PATTERN) return 1000;
  lockPattern();
  uint16_t v = pd->stepCutoffLockHz[currentPattern][track][step];
  unlockPattern();
  return v;
}

uint16_t Sequencer::getStepCutoffLock(int pattern, int track, int step) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return 1000;
  if (track < 0 || track >= MAX_TRACKS) return 1000;
  if (step < 0 || step >= STEPS_PER_PATTERN) return 1000;
  lockPattern();
  uint16_t v = pd->stepCutoffLockHz[pattern][track][step];
  unlockPattern();
  return v;
}

void Sequencer::setStepReverbSendLock(int track, int step, bool enabled, uint8_t sendLevel) {
  if (track < 0 || track >= MAX_TRACKS) return;
  if (step < 0 || step >= STEPS_PER_PATTERN) return;
  sendLevel = constrain(sendLevel, 0, 100);
  lockPattern();
  pd->stepReverbSendLockEnabled[currentPattern][track][step] = enabled;
  pd->stepReverbSendLockValue[currentPattern][track][step] = sendLevel;
  unlockPattern();
}

void Sequencer::setStepReverbSendLock(int pattern, int track, int step, bool enabled, uint8_t sendLevel) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return;
  if (track < 0 || track >= MAX_TRACKS) return;
  if (step < 0 || step >= STEPS_PER_PATTERN) return;
  sendLevel = constrain(sendLevel, 0, 100);
  lockPattern();
  pd->stepReverbSendLockEnabled[pattern][track][step] = enabled;
  pd->stepReverbSendLockValue[pattern][track][step] = sendLevel;
  unlockPattern();
}

bool Sequencer::hasStepReverbSendLock(int track, int step) {
  if (track < 0 || track >= MAX_TRACKS) return false;
  if (step < 0 || step >= STEPS_PER_PATTERN) return false;
  lockPattern();
  bool v = pd->stepReverbSendLockEnabled[currentPattern][track][step];
  unlockPattern();
  return v;
}

bool Sequencer::hasStepReverbSendLock(int pattern, int track, int step) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return false;
  if (track < 0 || track >= MAX_TRACKS) return false;
  if (step < 0 || step >= STEPS_PER_PATTERN) return false;
  lockPattern();
  bool value = pd->stepReverbSendLockEnabled[pattern][track][step];
  unlockPattern();
  return value;
}

uint8_t Sequencer::getStepReverbSendLock(int track, int step) {
  if (track < 0 || track >= MAX_TRACKS) return 0;
  if (step < 0 || step >= STEPS_PER_PATTERN) return 0;
  lockPattern();
  uint8_t v = pd->stepReverbSendLockValue[currentPattern][track][step];
  unlockPattern();
  return v;
}

uint8_t Sequencer::getStepReverbSendLock(int pattern, int track, int step) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return 0;
  if (track < 0 || track >= MAX_TRACKS) return 0;
  if (step < 0 || step >= STEPS_PER_PATTERN) return 0;
  lockPattern();
  uint8_t v = pd->stepReverbSendLockValue[pattern][track][step];
  unlockPattern();
  return v;
}

void Sequencer::setStepProbability(int track, int step, uint8_t probability) {
  if (track < 0 || track >= MAX_TRACKS) return;
  if (step < 0 || step >= STEPS_PER_PATTERN) return;
  lockPattern();
  pd->probabilities[currentPattern][track][step] = constrain(probability, 0, 100);
  unlockPattern();
}

void Sequencer::setStepProbability(int pattern, int track, int step, uint8_t probability) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return;
  if (track < 0 || track >= MAX_TRACKS) return;
  if (step < 0 || step >= STEPS_PER_PATTERN) return;
  lockPattern();
  pd->probabilities[pattern][track][step] = constrain(probability, 0, 100);
  unlockPattern();
}

uint8_t Sequencer::getStepProbability(int track, int step) {
  if (track < 0 || track >= MAX_TRACKS) return 100;
  if (step < 0 || step >= STEPS_PER_PATTERN) return 100;
  lockPattern();
  uint8_t v = pd->probabilities[currentPattern][track][step];
  unlockPattern();
  return v;
}

uint8_t Sequencer::getStepProbability(int pattern, int track, int step) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return 100;
  if (track < 0 || track >= MAX_TRACKS) return 100;
  if (step < 0 || step >= STEPS_PER_PATTERN) return 100;
  lockPattern();
  uint8_t v = pd->probabilities[pattern][track][step];
  unlockPattern();
  return v;
}

void Sequencer::setStepRatchet(int track, int step, uint8_t ratchet) {
  if (track < 0 || track >= MAX_TRACKS) return;
  if (step < 0 || step >= STEPS_PER_PATTERN) return;
  lockPattern();
  pd->ratchets[currentPattern][track][step] = constrain(ratchet, 1, 4);
  unlockPattern();
}

void Sequencer::setStepRatchet(int pattern, int track, int step, uint8_t ratchet) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return;
  if (track < 0 || track >= MAX_TRACKS) return;
  if (step < 0 || step >= STEPS_PER_PATTERN) return;
  lockPattern();
  pd->ratchets[pattern][track][step] = constrain(ratchet, 1, 4);
  unlockPattern();
}

uint8_t Sequencer::getStepRatchet(int track, int step) {
  if (track < 0 || track >= MAX_TRACKS) return 1;
  if (step < 0 || step >= STEPS_PER_PATTERN) return 1;
  lockPattern();
  uint8_t v = pd->ratchets[currentPattern][track][step];
  unlockPattern();
  return v;
}

uint8_t Sequencer::getStepRatchet(int pattern, int track, int step) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return 1;
  if (track < 0 || track >= MAX_TRACKS) return 1;
  if (step < 0 || step >= STEPS_PER_PATTERN) return 1;
  lockPattern();
  uint8_t v = pd->ratchets[pattern][track][step];
  unlockPattern();
  return v;
}

void Sequencer::setHumanize(uint8_t timingMs, uint8_t velocityAmount) {
  humanizeTimingMs = constrain(timingMs, 0, 40);
  humanizeVelocityAmount = constrain(velocityAmount, 0, 60);
}

uint8_t Sequencer::getHumanizeTimingMs() {
  return humanizeTimingMs;
}

uint8_t Sequencer::getHumanizeVelocityAmount() {
  return humanizeVelocityAmount;
}

void Sequencer::setStepCallback(StepCallback callback) {
  stepCallback = callback;
}

void Sequencer::setStepAutomationCallback(StepAutomationCallback callback) {
  stepAutomationCallback = callback;
}

void Sequencer::setStepChangeCallback(StepChangeCallback callback) {
  stepChangeCallback = callback;
}

void Sequencer::setPatternChangeCallback(PatternChangeCallback callback) {
  patternChangeCallback = callback;
}

// ============= PATTERN LENGTH =============

void Sequencer::setPatternLength(int len) {
  if (len != 16 && len != 32 && len != 64) return;
  lockState();
  patternLength = len;
  if (currentStep >= patternLength) {
    currentStep = 0;
  }
  unlockState();
}

void Sequencer::setPatternMetadata(int pattern, const PatternMetadata& metadata) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return;
  lockPattern();
  pd->metadata[pattern] = metadata;
  pd->metadata[pattern].name[sizeof(pd->metadata[pattern].name) - 1] = '\0';
  pd->metadata[pattern].genre[sizeof(pd->metadata[pattern].genre) - 1] = '\0';
  pd->metadata[pattern].kit[sizeof(pd->metadata[pattern].kit) - 1] = '\0';
  // 0 is a deliberate sentinel for imported MIDI files with no Set Tempo
  // event. All defined source tempos remain constrained to the playable range.
  if (pd->metadata[pattern].recommendedBpm != 0)
    pd->metadata[pattern].recommendedBpm = constrain(pd->metadata[pattern].recommendedBpm, 30, 300);
  pd->metadata[pattern].swing = constrain(pd->metadata[pattern].swing, 0, 100);
  pd->metadata[pattern].humanizeTimingMs = constrain(pd->metadata[pattern].humanizeTimingMs, 0, 30);
  pd->metadata[pattern].humanizeVelocity = constrain(pd->metadata[pattern].humanizeVelocity, 0, 40);
  unlockPattern();
}

bool Sequencer::getPatternMetadata(int pattern, PatternMetadata& metadata) {
  if (pattern < 0 || pattern >= MAX_PATTERNS) return false;
  lockPattern();
  metadata = pd->metadata[pattern];
  unlockPattern();
  return true;
}

int Sequencer::getPatternLength() {
  lockState();
  const int value = patternLength;
  unlockState();
  return value;
}

// ============= SONG MODE =============

void Sequencer::setSongMode(bool enabled) {
  lockState();
  songMode = enabled;
  if (songMode) {
    // When entering song mode, start from pattern 0
    currentPattern = 0;
  } else {
  }
  unlockState();
}

bool Sequencer::isSongMode() {
  lockState();
  const bool value = songMode;
  unlockState();
  return value;
}

void Sequencer::setSongLength(int length) {
  if (length < 1) length = 1;
  if (length > MAX_PATTERNS) length = MAX_PATTERNS;
  lockState();
  songLength = length;
  unlockState();
}

int Sequencer::getSongLength() {
  lockState();
  const int value = songLength;
  unlockState();
  return value;
}

// ============= LOOP SYSTEM =============

void Sequencer::toggleLoop(int track) {
  if (track >= 0 && track < MAX_TRACKS) {
    lockState();
    loopActive[track] = !loopActive[track];
    loopPaused[track] = false; // Reset pause state
    loopStepCounter[track] = 0; // Reset counter
    unlockState();
  }
}

void Sequencer::setLoopType(int track, LoopType type) {
  if (track >= 0 && track < MAX_TRACKS) {
    lockState();
    loopType[track] = type;
    loopStepCounter[track] = 0;
    unlockState();
  }
}

LoopType Sequencer::getLoopType(int track) {
  if (track >= 0 && track < MAX_TRACKS) {
    lockState();
    const LoopType value = loopType[track];
    unlockState();
    return value;
  }
  return LOOP_EVERY_STEP;
}

void Sequencer::pauseLoop(int track) {
  if (track >= 0 && track < MAX_TRACKS) {
    lockState();
    if (loopActive[track]) {
      loopPaused[track] = !loopPaused[track];
    }
    unlockState();
  }
}

bool Sequencer::isLooping(int track) {
  if (track >= 0 && track < MAX_TRACKS) {
    lockState();
    const bool value = loopActive[track];
    unlockState();
    return value;
  }
  return false;
}

bool Sequencer::isLoopPaused(int track) {
  if (track >= 0 && track < MAX_TRACKS) {
    lockState();
    const bool value = loopPaused[track];
    unlockState();
    return value;
  }
  return false;
}

void Sequencer::processLoops() {
  bool trigger[MAX_TRACKS] = {};
  uint8_t volume[MAX_TRACKS] = {};
  lockState();
  for (int track = 0; track < MAX_TRACKS; track++) {
    if (loopActive[track] && !loopPaused[track] && !trackMuted[track]) {
      bool shouldTrigger = false;
      
      switch (loopType[track]) {
        case LOOP_EVERY_STEP:
          // Trigger on every step (16th note)
          shouldTrigger = true;
          break;
        case LOOP_EVERY_BEAT:
          // Trigger every 4 steps (quarter note)
          shouldTrigger = (loopStepCounter[track] % 4 == 0);
          break;
        case LOOP_HALF_BEAT:
          // Trigger every 2 steps (8th note)
          shouldTrigger = (loopStepCounter[track] % 2 == 0);
          break;
        case LOOP_ARRHYTHMIC:
          // Random trigger ~40% chance per step
          shouldTrigger = (seqRand(100) < 40);
          break;
      }
      trigger[track] = shouldTrigger;
      volume[track] = trackVolume[track];
      loopStepCounter[track] = (loopStepCounter[track] + 1) % patternLength;
    }
  }
  unlockState();

  for (int track = 0; track < MAX_TRACKS; ++track) {
    if (trigger[track] && stepCallback != nullptr) {
      stepCallback(track, 100, volume[track], 0);
    }
  }
}

// ============= SONG CHAIN MODE =============

void Sequencer::songChainUpload(const SongChainEntry* entries, uint8_t count) {
  lockState();
  if (!entries || count == 0) {
    songChainCount = 0;
    songChainActive = false;
    unlockState();
    return;
  }
  if (count > SONG_CHAIN_MAX) count = SONG_CHAIN_MAX;
  for (uint8_t i = 0; i < count; ++i) {
    songChain[i].pattern = entries[i].pattern % MAX_PATTERNS;
    songChain[i].repeats = entries[i].repeats == 0 ? 1 : entries[i].repeats;
  }
  songChainCount = count;
  songChainIdx = 0;
  songChainRepeatCnt = 0;
  unlockState();
}

void Sequencer::songChainPlay() {
  lockState();
  if (songChainCount == 0) {
    unlockState();
    return;
  }
  songChainActive = true;
  songChainIdx = 0;
  songChainRepeatCnt = 0;
  currentPattern = songChain[0].pattern % MAX_PATTERNS;
  currentStep = 0;
  if (!playing) {
    playing = true;
    lastStepTime = micros();
  }
  const int pattern = currentPattern;
  const int count = songChainCount;
  unlockState();
  if (patternChangeCallback) {
    patternChangeCallback(pattern, count);
  }
}

void Sequencer::songChainStop() {
  lockState();
  songChainActive = false;
  unlockState();
}

void Sequencer::songChainReset() {
  lockState();
  songChainIdx = 0;
  songChainRepeatCnt = 0;
  if (songChainCount > 0) {
    currentPattern = songChain[0].pattern % MAX_PATTERNS;
  }
  unlockState();
}

bool Sequencer::isSongChainActive() {
  lockState(); const bool value = songChainActive; unlockState(); return value;
}

uint8_t Sequencer::getSongChainIdx() {
  lockState(); const uint8_t value = songChainIdx; unlockState(); return value;
}

uint8_t Sequencer::getSongChainRepeatCnt() {
  lockState(); const uint8_t value = songChainRepeatCnt; unlockState(); return value;
}

uint8_t Sequencer::getSongChainCount() {
  lockState(); const uint8_t value = songChainCount; unlockState(); return value;
}

