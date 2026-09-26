"""Run portable audit regression tests using g++/clang++ or `--compiler zig`."""
import argparse
import shutil
import subprocess
import tempfile
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--compiler", default=shutil.which("g++") or shutil.which("clang++"))
args = parser.parse_args()
if not args.compiler:
    parser.error("Pass --compiler with the path to g++, clang++ or zig")
root = Path(__file__).resolve().parent.parent
compiler = [args.compiler]
if Path(args.compiler).stem.lower() == "zig":
    compiler.append("c++")
with tempfile.TemporaryDirectory(prefix="drum-audit-tests-") as output:
    for test in ("sequencer_input_regression", "synth_rotary_expansion", "audio_deadline_regression", "sampler_retrigger_regression", "ratchet_audio_stress", "biquad_regression", "audio_regression", "storage_regression", "drum_idle_regression", "bank_controller_regression", "bank_state_regression", "xtra_directories_regression", "pod_button_events_regression", "sequence_groups_regression", "fx_rotary_selection_regression"):
        executable = Path(output) / (test + ".exe")
        subprocess.run(compiler + ["-std=c++17", "-O2", "-Itools/tests/stubs",
            "tools/tests/" + test + ".cpp", "-o", str(executable)], cwd=root, check=True)
        subprocess.run([str(executable)], check=True)
