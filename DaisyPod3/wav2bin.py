#!/usr/bin/env python3
"""
wav2bin.py - Convierte WAVs de RED 808 KARZ a un blob binario para QSPI Flash.

El blob se flashea a QSPI en 0x90100000 (768 KB offset desde inicio QSPI).
El firmware Daisy lo lee directamente desde esa direccion memory-mapped.

Formato del blob:
  [Header]  magic "RED8" (4B) | version (2B) | count (2B) = 8 bytes
  [Entries] count * 12 bytes cada una:
            padIdx(1B) | reserved(1B) | offset(4B) | length(4B)
  [Data]    raw int16_t samples back-to-back, alineados a 4 bytes

Uso:  python wav2bin.py
Genera: build/samples.bin
"""

import wave
import struct
import os
import sys

WAV_DIR = os.path.join("data", "RED 808 KARZ")
OUTPUT  = os.path.join("build", "samples.bin")

# Mapeo EXACTO para RED 808 KARZ por prefijo "808 XX"
EXACT_MAP = {
    "808 BD":  0,   # Bass Drum
    "808 SD":  1,   # Snare Drum
    "808 HH":  2,   # Closed HiHat
    "808 OH":  3,   # Open HiHat
    "808 CY":  4,   # Cymbal
    "808 CP":  5,   # Clap
    "808 RS":  6,   # Rimshot
    "808 COW": 7,   # Cowbell
    "808 LT":  8,   # Low Tom
    "808 MT":  9,   # Mid Tom
    "808 HT": 10,   # High Tom
    "808 MA": 11,   # Maracas
    "808 CL": 12,   # Claves
    "808 HC": 13,   # Hi Conga
    "808 MC": 14,   # Mid Conga
    "808 LC": 15,   # Low Conga
}

KEYWORD_MAP = [
    ("BD",  0), ("KICK", 0),
    ("SD",  1), ("SNARE",1),
    ("CH",  2), ("HH",   2),
    ("OH",  3), ("OPEN", 3),
    ("CY",  4), ("CYMBAL",4),
    ("CP",  5), ("CLAP", 5),
    ("RS",  6), ("RIM",  6),
    ("CB",  7), ("COW",  7),
    ("LT",  8), ("MT",  9),
    ("HT", 10), ("MA", 11),
    ("CL", 12), ("HC", 13),
    ("MC", 14), ("LC", 15),
]

PAD_NAMES = [
    "BD","SD","CH","OH","CY","CP","RS","CB",
    "LT","MT","HT","MA","CL","HC","MC","LC"
]

def guess_pad(filename):
    upper = filename.upper()
    for prefix, pad in EXACT_MAP.items():
        if upper.startswith(prefix.upper()):
            return pad
    for kw, pad in KEYWORD_MAP:
        if kw in upper:
            return pad
    return -1

def read_wav_mono16(filepath):
    """Read WAV file, convert to mono 16-bit signed samples."""
    with wave.open(filepath, 'rb') as wf:
        ch = wf.getnchannels()
        sw = wf.getsampwidth()
        sr = wf.getframerate()
        nframes = wf.getnframes()
        raw = wf.readframes(nframes)

    samples = []
    if sw == 2:
        fmt = f"<{nframes * ch}h"
        data = struct.unpack(fmt, raw)
        if ch == 1:
            samples = list(data)
        else:
            for i in range(0, len(data), ch):
                avg = sum(data[i:i+ch]) // ch
                samples.append(max(-32768, min(32767, avg)))
    elif sw == 1:
        for i in range(0, len(raw), ch):
            vals = [raw[i+c] for c in range(ch)]
            avg = sum(vals) // ch
            samples.append((avg - 128) * 256)
    elif sw == 3:
        for i in range(0, len(raw), sw * ch):
            channel_vals = []
            for c in range(ch):
                off = i + c * 3
                b0, b1, b2 = raw[off], raw[off+1], raw[off+2]
                val = (b0 << 8) | (b1 << 16) | (b2 << 24)
                val = struct.unpack('<i', struct.pack('<I', val))[0]
                channel_vals.append(val >> 16)
            avg = sum(channel_vals) // ch
            samples.append(max(-32768, min(32767, avg)))

    # Limit to 8 MB per sampler once converted to mono int16.
    MAX_SAMPLES = (8 * 1024 * 1024) // 2
    if len(samples) > MAX_SAMPLES:
        samples = samples[:MAX_SAMPLES]

    return samples, sr

def main():
    if not os.path.isdir(WAV_DIR):
        print(f"ERROR: No se encuentra {WAV_DIR}")
        sys.exit(1)

    wavfiles = sorted([f for f in os.listdir(WAV_DIR) if f.lower().endswith('.wav')])
    if not wavfiles:
        print(f"ERROR: No hay WAV files en {WAV_DIR}")
        sys.exit(1)

    print(f"Encontrados {len(wavfiles)} archivos WAV")

    # Two-pass assignment
    file_with_pad = [(fname, guess_pad(fname)) for fname in wavfiles]

    entries = []  # (pad, samples_int16, filename)
    pad_used = set()
    assigned = set()

    # Pass 1: unique instruments
    for fname, pad in file_with_pad:
        if pad >= 0 and pad < 16 and pad not in pad_used:
            filepath = os.path.join(WAV_DIR, fname)
            samples, sr = read_wav_mono16(filepath)
            pad_used.add(pad)
            assigned.add(fname)
            entries.append((pad, samples, fname))
            print(f"  Pad {pad:2d} ({PAD_NAMES[pad]:2s}) <- {fname} ({len(samples)} samples, {sr} Hz)")

    # Pass 2: duplicates to free slots
    for fname, pad in file_with_pad:
        if fname in assigned:
            continue
        free = next((s for s in range(16) if s not in pad_used), -1)
        if free < 0:
            print(f"  SKIP {fname}")
            continue
        filepath = os.path.join(WAV_DIR, fname)
        samples, sr = read_wav_mono16(filepath)
        pad_used.add(free)
        entries.append((free, samples, fname))
        print(f"  Pad {free:2d} ({PAD_NAMES[free]:2s}) <- {fname} [extra] ({len(samples)} samples, {sr} Hz)")

    entries.sort(key=lambda e: e[0])

    total_samples = sum(len(e[1]) for e in entries)
    total_bytes = total_samples * 2
    print(f"\nTotal: {len(entries)} pads, {total_samples} muestras, {total_bytes} bytes ({total_bytes/1024:.1f} KB)")

    # Build binary blob
    HEADER_SIZE = 8   # magic(4) + version(2) + count(2)
    ENTRY_SIZE  = 12  # padIdx(1) + reserved(1) + offset(4) + length(4)
    count = len(entries)

    # Calculate data start offset (aligned to 4 bytes)
    data_start = HEADER_SIZE + count * ENTRY_SIZE
    if data_start % 4 != 0:
        data_start += 4 - (data_start % 4)

    # Build sample data and track offsets
    sample_data = bytearray()
    entry_info = []  # (pad, offset, length)

    for pad, samples, fname in entries:
        offset = data_start + len(sample_data)
        length = len(samples)
        raw = struct.pack(f"<{length}h", *samples)
        sample_data.extend(raw)
        # Align to 4 bytes
        while len(sample_data) % 4 != 0:
            sample_data.extend(b'\x00')
        entry_info.append((pad, offset, length))

    # Write blob
    os.makedirs(os.path.dirname(OUTPUT), exist_ok=True)
    with open(OUTPUT, 'wb') as f:
        # Header
        f.write(b'RED8')                          # magic
        f.write(struct.pack('<H', 1))             # version
        f.write(struct.pack('<H', count))         # count

        # Entries
        for pad, offset, length in entry_info:
            f.write(struct.pack('<B', pad))        # padIdx
            f.write(struct.pack('<B', 0))          # reserved
            f.write(struct.pack('<I', offset))     # offset from blob start
            f.write(struct.pack('<I', length))     # num int16_t samples

        # Padding to data_start
        pos = HEADER_SIZE + count * ENTRY_SIZE
        while pos < data_start:
            f.write(b'\x00')
            pos += 1

        # Sample data
        f.write(sample_data)

    blob_size = data_start + len(sample_data)
    print(f"\nGenerado: {OUTPUT}")
    print(f"  Blob size: {blob_size} bytes ({blob_size/1024:.1f} KB)")
    print(f"  Firmware QSPI: 0x90040000")
    print(f"  Samples QSPI:  0x90100000  (768 KB offset)")
    print(f"  QSPI libre:    {(7936*1024 - 768*1024 - blob_size)/1024:.0f} KB restantes de 7936 KB")

if __name__ == "__main__":
    main()
