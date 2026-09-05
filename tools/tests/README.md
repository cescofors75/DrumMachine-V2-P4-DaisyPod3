# Audit regression tests

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
- `storage_regression.cpp`: includes the **production pattern_store.cpp** with
  an in-memory filesystem and small opaque pattern payload. Injects interrupted
  writes at every byte boundary, open failure and corruption; checks that failed
  saves leave destination RAM intact and reboot restores the last valid copy.
  Also tests migration of existing V1 files. This does not simulate a damaged
  SPIFFS partition or flash-controller failure.

These are host tests, not measurements of real-time CPU load, touch response,
USB latency or power-loss behavior of the physical filesystem.
