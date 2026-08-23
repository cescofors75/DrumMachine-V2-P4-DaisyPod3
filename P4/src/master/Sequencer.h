/*
 * Sequencer.h
 * Sequencer de 16 steps per Drum Machine (8 tracks)
 * (OPCIONAL - per afegir funcionalitat de sequencer)
 *
 * ROLE ON THE P4: PATTERN DATA STORE ONLY — NOT the playback engine.
 *
 * DaisyPod3 owns playback: its step clock is a sample counter advanced
 * inside AudioCallback (see DsqFireStep() in DaisyPod3/main.cpp), which is
 * sample-accurate and drives the actual voices. The P4 learns the current
 * step/pattern from Daisy telemetry (transport.step), not from this class.
 *
 * So on the P4 only the pattern-storage half of this class is live: the
 * accessors for steps, velocities, notes, flags, per-step locks, plus
 * selectPattern()/currentPattern which those 1-argument accessors use as
 * their implicit target.
 *
 * The playback half — update(), processStep(), processLoops(), the four
 * *Callback hooks, the loop system, song/queued-pattern advance — is NOT
 * driven here. Do not call update() from the P4: none of the callbacks are
 * ever registered (so every step it computes is discarded), and worse, its
 * wrap logic writes currentPattern from its own micros()-based clock. That
 * made it a second writer racing selectPattern(), and since currentPattern
 * is the implicit target for step edits, a wrap at a slightly different
 * moment than Daisy's could land an edit on the wrong pattern.
 *
 * The playback half is kept (it compiles, it is just never driven) because
 * it predates the split — this class started life as the standalone engine.
 * Removing it outright is a separate, larger cleanup.
 */

#ifndef SEQUENCER_H
#define SEQUENCER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#define MAX_PATTERNS 128
#define STEPS_PER_PATTERN 64
#define MAX_TRACKS 16
#define MELODY_STEP_VOICES 4

struct PatternMetadata {
  char name[32];
  char genre[24];
  char kit[16];
  uint16_t recommendedBpm;
  uint8_t swing;
  uint8_t humanizeTimingMs;
  uint8_t humanizeVelocity;
};

// Loop types for pads
enum LoopType {
  LOOP_EVERY_STEP = 0,   // Trigger every step (16th note)
  LOOP_EVERY_BEAT = 1,   // Trigger every beat (4 steps = quarter note)
  LOOP_HALF_BEAT = 2,    // Trigger 2x per beat (8th note = every 2 steps)
  LOOP_ARRHYTHMIC = 3    // Random timing
};

// -----------------------------------------------------------------------
// PatternData — all large per-step arrays in one heap-block (PSRAM).
// Kept OUTSIDE the Sequencer class so the class BSS is small.
// Allocated with ps_calloc() in Sequencer::Sequencer().
// -----------------------------------------------------------------------
struct PatternData {
  PatternMetadata metadata[MAX_PATTERNS];
  bool    steps[MAX_PATTERNS][MAX_TRACKS][STEPS_PER_PATTERN];
  uint8_t velocities[MAX_PATTERNS][MAX_TRACKS][STEPS_PER_PATTERN];
  uint8_t noteLenDivs[MAX_PATTERNS][MAX_TRACKS][STEPS_PER_PATTERN];
  uint8_t probabilities[MAX_PATTERNS][MAX_TRACKS][STEPS_PER_PATTERN];
  uint8_t ratchets[MAX_PATTERNS][MAX_TRACKS][STEPS_PER_PATTERN];
  bool    stepVolumeLockEnabled[MAX_PATTERNS][MAX_TRACKS][STEPS_PER_PATTERN];
  uint8_t stepVolumeLockValue[MAX_PATTERNS][MAX_TRACKS][STEPS_PER_PATTERN];
  bool    stepCutoffLockEnabled[MAX_PATTERNS][MAX_TRACKS][STEPS_PER_PATTERN];
  uint16_t stepCutoffLockHz[MAX_PATTERNS][MAX_TRACKS][STEPS_PER_PATTERN];
  bool    stepReverbSendLockEnabled[MAX_PATTERNS][MAX_TRACKS][STEPS_PER_PATTERN];
  uint8_t stepReverbSendLockValue[MAX_PATTERNS][MAX_TRACKS][STEPS_PER_PATTERN];
  // Melody: per-step MIDI note voices (0 = rest/inherit) and flags (bit0=accent, bit1=slide)
  uint8_t stepNotes[MAX_PATTERNS][MAX_TRACKS][STEPS_PER_PATTERN];
  uint8_t stepNoteVoices[MAX_PATTERNS][MAX_TRACKS][STEPS_PER_PATTERN][MELODY_STEP_VOICES];
  uint8_t stepFlags[MAX_PATTERNS][MAX_TRACKS][STEPS_PER_PATTERN];
};
// sizeof(PatternData) ≈ 241 KB  →  allocated from 8 MB PSRAM, not DRAM

// Snapshot consistente de un step para subir a la Daisy. Permite que
// dsqUploadPattern copie una pista entera bajo un solo lock y luego haga el
// transfer SPI (con sus vTaskDelay) sin mantener el mutex tomado.
struct StepUploadData {
  bool     active;
  uint8_t  velocity;
  uint8_t  noteLenDiv;
  uint8_t  probability;
  uint8_t  ratchet;
  uint8_t  flags;
  uint8_t  noteVoices[MELODY_STEP_VOICES];
  bool     cutoffEn;
  uint16_t cutoffHz;
  bool     reverbEn;
  uint8_t  reverbSend;
  bool     volumeEn;
  uint8_t  volume;
};

// Stable, packed representation used by the P4 user-pattern store. It is
// deliberately independent from PatternData's compiler-specific bool layout.
struct __attribute__((packed)) PatternStorageStep {
  uint8_t  active;
  uint8_t  velocity;
  uint8_t  noteLenDiv;
  uint8_t  probability;
  uint8_t  ratchet;
  uint8_t  flags;
  uint8_t  noteVoices[MELODY_STEP_VOICES];
  uint8_t  cutoffEn;
  uint16_t cutoffHz;
  uint8_t  reverbEn;
  uint8_t  reverbSend;
  uint8_t  volumeEn;
  uint8_t  volume;
};

struct __attribute__((packed)) PatternStorageData {
  PatternMetadata metadata;
  PatternStorageStep steps[MAX_TRACKS][STEPS_PER_PATTERN];
};

class Sequencer {
public:
  Sequencer();
  ~Sequencer();
  
  // Control
  void start();
  void stop();
  void reset();
  bool isPlaying();
  
  // Timing
  void setTempo(float bpm);
  float getTempo();
  void update(); // Call from loop
  
  // Pattern editing
  void setStep(int track, int step, bool active, uint8_t velocity = 127);
  void setStep(int pattern, int track, int step, bool active, uint8_t velocity = 127);
  bool getStep(int track, int step);
  bool getStep(int pattern, int track, int step);  // Get step from specific pattern
  void clearPattern(int pattern);
  void clearPattern(); // Clear current pattern
  void clearTrack(int track);

  // Copia consistente (bajo un solo lock) de los datos de una pista de un
  // patron para subirlos a la Daisy sin races con ediciones de la web.
  void snapshotTrackForUpload(int pattern, int track, int stepCount, StepUploadData* out);
  bool snapshotPatternForStorage(int pattern, PatternStorageData* out);
  bool restorePatternFromStorage(int pattern, const PatternStorageData* in);
  
  // Velocity editing per step
  void setStepVelocity(int track, int step, uint8_t velocity);
  void setStepVelocity(int pattern, int track, int step, uint8_t velocity);
  uint8_t getStepVelocity(int track, int step);
  uint8_t getStepVelocity(int pattern, int track, int step);
  
  // Note length per step (divider: 1=full, 2=half, 4=quarter, 8=eighth)
  void setStepNoteLen(int track, int step, uint8_t div);
  void setStepNoteLen(int pattern, int track, int step, uint8_t div);
  uint8_t getStepNoteLen(int track, int step);
  uint8_t getStepNoteLen(int pattern, int track, int step);

  // Probability and ratchet per step
  void setStepProbability(int track, int step, uint8_t probability);
  void setStepProbability(int pattern, int track, int step, uint8_t probability);
  uint8_t getStepProbability(int track, int step);
  uint8_t getStepProbability(int pattern, int track, int step);
  void setStepRatchet(int track, int step, uint8_t ratchet);
  void setStepRatchet(int pattern, int track, int step, uint8_t ratchet);
  uint8_t getStepRatchet(int track, int step);
  uint8_t getStepRatchet(int pattern, int track, int step);

  // Humanize (global)
  void setHumanize(uint8_t timingMs, uint8_t velocityAmount);
  uint8_t getHumanizeTimingMs();
  uint8_t getHumanizeVelocityAmount();

  // Parameter locks per step (currently volume lock)
  void setStepVolumeLock(int track, int step, bool enabled, uint8_t volume);
  void setStepVolumeLock(int pattern, int track, int step, bool enabled, uint8_t volume);
  bool hasStepVolumeLock(int track, int step);
  bool hasStepVolumeLock(int pattern, int track, int step);
  uint8_t getStepVolumeLock(int track, int step);
  uint8_t getStepVolumeLock(int pattern, int track, int step);

  // Parameter automation per-step (timing source = sequencer)
  void setStepCutoffLock(int track, int step, bool enabled, uint16_t cutoffHz);
  void setStepCutoffLock(int pattern, int track, int step, bool enabled, uint16_t cutoffHz);
  bool hasStepCutoffLock(int track, int step);
  bool hasStepCutoffLock(int pattern, int track, int step);
  uint16_t getStepCutoffLock(int track, int step);
  uint16_t getStepCutoffLock(int pattern, int track, int step);

  void setStepReverbSendLock(int track, int step, bool enabled, uint8_t sendLevel);
  void setStepReverbSendLock(int pattern, int track, int step, bool enabled, uint8_t sendLevel);
  bool hasStepReverbSendLock(int track, int step);
  bool hasStepReverbSendLock(int pattern, int track, int step);
  uint8_t getStepReverbSendLock(int track, int step);
  uint8_t getStepReverbSendLock(int pattern, int track, int step);
  
  // Melody note per step (MIDI note 0-127, 0 = rest/no note)
  void setStepNote(int track, int step, uint8_t midiNote);
  void setStepNote(int pattern, int track, int step, uint8_t midiNote);
  uint8_t getStepNote(int track, int step);
  uint8_t getStepNote(int pattern, int track, int step);
  void setStepNoteVoice(int track, int step, int voice, uint8_t midiNote);
  void setStepNoteVoice(int pattern, int track, int step, int voice, uint8_t midiNote);
  uint8_t getStepNoteVoice(int track, int step, int voice);
  uint8_t getStepNoteVoice(int pattern, int track, int step, int voice);
  void clearStepNoteVoices(int track, int step);
  void clearStepNoteVoices(int pattern, int track, int step);
  // Melody flags per step (bit0=accent, bit1=slide)
  void setStepFlags(int track, int step, uint8_t flags);
  void setStepFlags(int pattern, int track, int step, uint8_t flags);
  uint8_t getStepFlags(int track, int step);
  uint8_t getStepFlags(int pattern, int track, int step);

  // Bulk pattern writing (for reliable MIDI import)
  void setPatternBulk(int pattern, const bool stepsData[MAX_TRACKS][STEPS_PER_PATTERN], const uint8_t velsData[MAX_TRACKS][STEPS_PER_PATTERN]);
  
  // Pattern management
  void selectPattern(int pattern);
  // Live performance changes. They are committed only when the playhead wraps
  // to step 0, so a remote display can prepare the next scene without moving
  // the audible transport in the middle of a bar.
  void queuePattern(int pattern);
  void queueOneBarPattern(int pattern);
  void queuePatternForBars(int pattern, uint8_t bars);
  int getQueuedPattern();
  void cancelQueuedPattern();
  int getPerformancePattern(); // hides temporary FILL/VAR scratch scenes from UIs
  int getCurrentPattern();
  void copyPattern(int src, int dst);
  void setPatternMetadata(int pattern, const PatternMetadata& metadata);
  bool getPatternMetadata(int pattern, PatternMetadata& metadata);
  
  // Mute tracks
  void muteTrack(int track, bool muted);
  bool isTrackMuted(int track);
  
  // Track volumes (0-150)
  void setTrackVolume(int track, uint8_t volume);
  uint8_t getTrackVolume(int track);
  
  // Song mode (auto-chain patterns)
    // Pattern length (16/32/64)
  void setPatternLength(int len);
  int getPatternLength();

  void setSongMode(bool enabled);
  bool isSongMode();
  void setSongLength(int length);
  int getSongLength();

  // Song Chain mode (pattern chain with repeats)
  static constexpr int SONG_CHAIN_MAX = 128;
  struct SongChainEntry { uint8_t pattern; uint8_t repeats; };
  void songChainUpload(const SongChainEntry* entries, uint8_t count);
  void songChainPlay();
  void songChainStop();
  void songChainReset();
  bool isSongChainActive();
  uint8_t getSongChainIdx();
  uint8_t getSongChainRepeatCnt();
  uint8_t getSongChainCount();
  
  // Playback
  int getCurrentStep();
  
  // Loop system for live pads
  void toggleLoop(int track);
  void setLoopType(int track, LoopType type);
  LoopType getLoopType(int track);
  void pauseLoop(int track);
  bool isLooping(int track);
  bool isLoopPaused(int track);
  void processLoops(); // Called internally
  
  // Callbacks
  // noteLenSamples: 0 = full sample, >0 = cut after N samples (note length)
  typedef void (*StepCallback)(int track, uint8_t velocity, uint8_t trackVolume, uint32_t noteLenSamples);
  typedef void (*StepAutomationCallback)(int track, int step,
                                         bool cutoffEnabled, uint16_t cutoffHz,
                                         bool reverbEnabled, uint8_t reverbSend,
                                         bool volumeEnabled, uint8_t volume);
  typedef void (*StepChangeCallback)(int newStep);
  typedef void (*PatternChangeCallback)(int newPattern, int songLength);
  void setStepCallback(StepCallback callback);
  void setStepAutomationCallback(StepAutomationCallback callback);
  void setStepChangeCallback(StepChangeCallback callback);
  void setPatternChangeCallback(PatternChangeCallback callback);
  
private:
  // Pattern data: all stored in PSRAM (PatternData* pd)
  PatternData* pd;

  // Protege pd[] entre la tarea de audio/secuencia (loop → processStep)
  // y otras tareas que editan patrones.
  // Mutex RECURSIVO: muchos accessors se delegan entre si (p.ej.
  // getStepVolumeLock -> hasStepVolumeLock, o las variantes de 1 patron ->
  // variante con patron). Con un mutex normal eso provocaria deadlock; el
  // recursivo permite que la misma task lo tome anidado.
  SemaphoreHandle_t patternMutex = nullptr;
  inline void lockPattern()   { if (patternMutex) xSemaphoreTakeRecursive(patternMutex, portMAX_DELAY); }
  inline void unlockPattern() { if (patternMutex) xSemaphoreGiveRecursive(patternMutex); }

  // Short-lived synchronization for transport, mixer, loop and song-chain
  // state shared by Core0 web handlers and the Core1 sequencer hot path.
  SemaphoreHandle_t stateMutex = nullptr;
  inline void lockState()   { if (stateMutex) xSemaphoreTakeRecursive(stateMutex, portMAX_DELAY); }
  inline void unlockState() { if (stateMutex) xSemaphoreGiveRecursive(stateMutex); }
  
  // volatile: leidos/escritos desde Core1 (loop → update/processStep) y Core0
  // (start/stop/selectPattern/setTempo desde web). Evita que el compilador los
  // cachee en registro dentro del while(true) y garantiza visibilidad cruzada.
  volatile bool playing;
  int patternLength;  // Active step count: 16, 32, or 64
  volatile int currentPattern;
  volatile int currentStep;
  volatile int queuedPattern;
  volatile int performanceReturnPattern;
  volatile uint8_t queuedPatternBars;
  volatile uint8_t performanceBarsRemaining;
  volatile bool performancePatternActive;
  float tempo; // BPM
  uint32_t lastStepTime;
  volatile uint32_t stepInterval; // microseconds
  volatile uint32_t nextStepInterval;
  uint8_t humanizeTimingMs;
  uint8_t humanizeVelocityAmount;
  bool trackMuted[MAX_TRACKS];
  uint8_t trackVolume[MAX_TRACKS]; // Volume per track (0-150)
  
  StepCallback stepCallback;
  StepAutomationCallback stepAutomationCallback;
  StepChangeCallback stepChangeCallback;
  PatternChangeCallback patternChangeCallback;
  
  // Song mode
  bool songMode;
  int songLength; // 1-16 pattern count for song

  // Song Chain mode
  bool songChainActive;
  uint8_t songChainCount;     // number of entries (0-128)
  uint8_t songChainIdx;       // current index in chain
  uint8_t songChainRepeatCnt; // current repeat count for current entry
  SongChainEntry songChain[SONG_CHAIN_MAX];
  
  // Loop system
  bool loopActive[MAX_TRACKS];
  bool loopPaused[MAX_TRACKS];
  LoopType loopType[MAX_TRACKS];
  uint8_t loopStepCounter[MAX_TRACKS];
  
  void calculateStepInterval();
  void processStep(int pattern, int step);
};

#endif // SEQUENCER_H
