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
    for test in ("audio_regression", "storage_regression"):
        executable = Path(output) / (test + ".exe")
        subprocess.run(compiler + ["-std=c++17", "-O2", "-Itools/tests/stubs",
            "tools/tests/" + test + ".cpp", "-o", str(executable)], cwd=root, check=True)
        subprocess.run([str(executable)], check=True)
