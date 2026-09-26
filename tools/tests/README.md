# Audit regression tests

`midi_input_regression.cpp` checks DIN running status, interleaved realtime,
velocity-zero note release, SysEx cancellation and USB message validation.

Run from any directory:

```text
python tools/run_audit_tests.py --compiler /path/to/g++
python tools/run_audit_tests.py --compiler /path/to/zig
```

The runner builds into a temporary directory, checks compiler and test exit
codes, and removes its binaries on completion. Assertions remain enabled even
with compilers that define NDEBUG in optimized builds.

- `audio_regression.cpp`: production timing/packing/hash helpers, transaction
  validation (missing tracks, old token, corrupt checksum), all three native
  drum kit outputs, preservation of aggregate signal, and 909/505 PCM outputs.
- `drum_idle_regression.cpp`: idle silence, procedural and PCM voice lifecycles,
  final samples, hat choke, sample replacement and limiter recovery for all kits.
- `storage_regression.cpp`: includes the **production pattern_store.cpp** with
  an in-memory filesystem and small opaque pattern payload. Injects interrupted
  writes at every byte boundary, open failure and corruption; checks that failed
  saves leave destination RAM intact and reboot restores the last valid copy.
  Also tests migration of existing V1 files. This does not simulate a damaged
  SPIFFS partition or flash-controller failure.

- `bank_controller_regression.cpp`: production BANK controller and hardware
  mailbox; fader hysteresis/debounce, context recall, touch override, relative
  edits, bounds, fine/log modes, protected actions and complete synth mapping.
- `bank_state_regression.cpp`: production wire observer; packed/truncated
  payloads, invalid floats, signed values, FX reset, round-trip of every send
  and bitcrush detent, and preservation of default LIVE velocity dynamics.

These are host tests, not measurements of real-time CPU load, touch response,
USB latency or power-loss behavior of the physical filesystem.

`xtra_directories_regression.cpp` runs the production XTRA directory selector
against a simulated transport: exact paths, case-insensitive WAV filtering,
alphabetical order, unrelated response rejection, empty folders, wraparound,
debounced and paced loading, held-pad deferral and timeout.

`pod_button_events_regression.cpp` tests the production USB-to-UI press mailbox:
delayed rendering after empty telemetry, hide/show parity, no event replay,
independent buttons, reset, and 100,000 concurrent producer/consumer events.

`sequence_groups_regression.cpp` exercises group transforms and the production
control adapter: disjoint track masks, baseline restoration, preserving other
groups, external edits, pattern changes, velocity/probability/ratchet and invalid inputs.

The group tests also cover probability zero, glitch x4 preserving probabilities,
and rotating/restoring rhythms within four-row blocks without touching other rows.
`fx_rotary_selection_regression.cpp` covers independent selection by column,
whole-row replacement, all 18 effects, and the final row's two empty slots.

`biquad_regression.cpp` uses the production Daisy biquad: sweeps all ten types,
cutoff and Q, rejects invalid parameters, recovers nonfinite state and checks
interrupt-mask restoration. It does not measure device CPU or reproduce a hang.

`sampler_retrigger_regression.cpp` checks the production sequencer voice selection
and complementary replacement gain: 10,000 retriggers, live-voice isolation and
positive/negative transition bounds.

`ratchet_audio_stress.cpp` checks the production TB303 accent decay after a
non-accented note, then renders dense x4 ratchets on 303/808/909/505 for 20 seconds
per engine. This test and the extended sequence reset test compile, but their
execution was blocked by Windows Application Control on 15 September 2026.
They are not recorded as passing. Sampler retrigger execution passed.

`audio_deadline_regression.cpp` covers timer wrapping, deadline boundaries and
fade-to-zero bounds. Compilation succeeded; Windows Application Control blocked
execution on 15 September 2026. It does not establish a hardware timing margin.

`synth_rotary_expansion.cpp`: initial execution passed retrigger continuity for
303/SH/FM/WT, five-second finite SH/FM renders, rotary deadband and expanded-bank
invariants. The later version adding five-second 303/WT renders compiled but
Windows blocked execution. Do not count those added long renders as passing.


## Revisión de sequencer — SEQUENCER_REVIEW

`sequencer_input_regression.cpp`: conservación de pasos pendientes, cambio
 de sentido, invalidez de eventos de un contexto anterior, reset y margen de
contador. Validación de compilación; ejecución pendiente (Windows bloqueó los
EXE de pruebas de esta sesión; no se intenta eludir la política).
Ambos firmwares compilan. La prueba 4/8/12/16 pistas y el tacto físico de los
rotarys quedan pendientes. Hoja: `GUIA_SEQUENCER_NONE.md`.
