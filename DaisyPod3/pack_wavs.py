#!/usr/bin/env python3
"""
pack_wavs.py - Empaqueta WAV files tal cual en un blob para QSPI Flash.

El firmware Daisy lee directamente desde QSPI memory-mapped (0x90080000).
Los WAV se guardan intactos con sus headers; el firmware los parsea al arrancar.

Formato del blob:
  [0x00] magic    "WAV\0"  (4 bytes)
  [0x04] version  uint16   (1)
  [0x06] count    uint16   (num files)
    [0x08] entries[count]:
         padIdx   uint8
         reserved uint8
         offset   uint32   (desde inicio del blob)
         size     uint32   (bytes del WAV completo)
  [...]  WAV files raw, alineados a 4 bytes

Genera: build/samples.bin
"""

import os
import sys
import struct

WAV_DIR = os.path.join("data", "RED 808 KARZ")
XTRA_DIR = os.path.join("data", "xtra")
OUTPUT  = os.path.join("build", "samples.bin")

# Mapeo RED 808 KARZ por prefijo "808 XX"
EXACT_MAP = {
    "808 BD":  0,  "808 SD":  1,  "808 HH":  2,  "808 OH":  3,
    "808 CY":  4,  "808 CP":  5,  "808 RS":  6,  "808 COW": 7,
    "808 LT":  8,  "808 MT":  9,  "808 HT": 10,  "808 MA": 11,
    "808 CL": 12,  "808 HC": 13,  "808 MC": 14,  "808 LC": 15,
}

KEYWORD_MAP = [
    ("BD",0),("KICK",0),("SD",1),("SNARE",1),("CH",2),("HH",2),
    ("OH",3),("CY",4),("CP",5),("RS",6),("CB",7),("COW",7),
    ("LT",8),("MT",9),("HT",10),("MA",11),("CL",12),("HC",13),
    ("MC",14),("LC",15),
]

PAD_NAMES = ["BD","SD","CH","OH","CY","CP","RS","CB",
             "LT","MT","HT","MA","CL","HC","MC","LC",
             "X0","X1","X2","X3","X4","X5","X6","X7"]

def guess_pad(filename):
    upper = filename.upper()
    for prefix, pad in EXACT_MAP.items():
        if upper.startswith(prefix.upper()):
            return pad
    for kw, pad in KEYWORD_MAP:
        if kw in upper:
            return pad
    return -1

def align4(n):
    return n + (4 - n % 4) % 4

def main():
    if not os.path.isdir(WAV_DIR):
        print(f"ERROR: No se encuentra {WAV_DIR}")
        sys.exit(1)

    wavfiles = sorted(f for f in os.listdir(WAV_DIR) if f.lower().endswith('.wav'))
    if not wavfiles:
        print(f"ERROR: No hay WAV files en {WAV_DIR}")
        sys.exit(1)

    print(f"Encontrados {len(wavfiles)} archivos WAV en {WAV_DIR}")

    # Two-pass pad assignment for RED kit (pads 0..15)
    file_pad = [(f, guess_pad(f)) for f in wavfiles]
    entries = []   # (pad, filename, raw_bytes)
    pad_used = set()
    assigned = set()

    # Pass 1: unique instruments
    for fname, pad in file_pad:
        if 0 <= pad < 16 and pad not in pad_used:
            path = os.path.join(WAV_DIR, fname)
            raw = open(path, 'rb').read()
            pad_used.add(pad)
            assigned.add(fname)
            entries.append((pad, fname, raw))
            print(f"  Pad {pad:2d} ({PAD_NAMES[pad]:2s}) <- {fname} ({len(raw)} bytes)")

    # Pass 2: extras to free slots
    for fname, pad in file_pad:
        if fname in assigned:
            continue
        free = next((s for s in range(16) if s not in pad_used), -1)
        if free < 0:
            print(f"  SKIP {fname}")
            continue
        path = os.path.join(WAV_DIR, fname)
        raw = open(path, 'rb').read()
        pad_used.add(free)
        entries.append((free, fname, raw))
        print(f"  Pad {free:2d} ({PAD_NAMES[free]:2s}) <- {fname} [extra] ({len(raw)} bytes)")

    # Add XTRA folder at pads 16..23
    if os.path.isdir(XTRA_DIR):
        xtra_files = sorted(f for f in os.listdir(XTRA_DIR) if f.lower().endswith('.wav'))
        if xtra_files:
            print(f"Añadiendo {len(xtra_files)} archivos WAV de {XTRA_DIR}")
        xtra_pad = 16
        for fname in xtra_files:
            if xtra_pad >= 24:
                print(f"  SKIP XTRA {fname} (sin pads libres en 16..23)")
                continue
            path = os.path.join(XTRA_DIR, fname)
            raw = open(path, 'rb').read()
            entries.append((xtra_pad, fname, raw))
            print(f"  Pad {xtra_pad:2d} ({PAD_NAMES[xtra_pad]:2s}) <- {fname} [xtra] ({len(raw)} bytes)")
            xtra_pad += 1

    entries.sort(key=lambda e: e[0])
    count = len(entries)

    # Calculate layout
    HEADER_SIZE = 8
    ENTRY_SIZE  = 10   # 1+1+4+4
    toc_size = align4(HEADER_SIZE + count * ENTRY_SIZE)

    # Calculate offsets for each WAV file
    offsets = []
    cur = toc_size
    for pad, fname, raw in entries:
        offsets.append(cur)
        cur = align4(cur + len(raw))

    total_blob = cur
    total_wav = sum(len(e[2]) for e in entries)

    # Write blob
    os.makedirs(os.path.dirname(OUTPUT), exist_ok=True)
    with open(OUTPUT, 'wb') as f:
        # Header
        f.write(b'WAV\x00')                       # magic
        f.write(struct.pack('<H', 1))              # version
        f.write(struct.pack('<H', count))          # count

        # TOC entries
        for i, (pad, fname, raw) in enumerate(entries):
            f.write(struct.pack('<B', pad))         # padIdx
            f.write(struct.pack('<B', 0))           # reserved
            f.write(struct.pack('<I', offsets[i]))  # offset
            f.write(struct.pack('<I', len(raw)))    # size (bytes)

        # Pad TOC to alignment
        pos = HEADER_SIZE + count * ENTRY_SIZE
        while pos < toc_size:
            f.write(b'\x00')
            pos += 1

        # WAV data
        for i, (pad, fname, raw) in enumerate(entries):
            f.write(raw)
            # Align to 4 bytes
            pad_bytes = align4(len(raw)) - len(raw)
            if pad_bytes:
                f.write(b'\x00' * pad_bytes)

    print(f"\n{'='*50}")
    print(f"  Blob: {OUTPUT}")
    print(f"  WAVs: {total_wav:,} bytes ({total_wav/1024:.1f} KB)")
    print(f"  Blob: {total_blob:,} bytes ({total_blob/1024:.1f} KB)")
    redCount  = sum(1 for p, _, _ in entries if p < 16)
    xtraCount = sum(1 for p, _, _ in entries if p >= 16)
    print(f"  Pads total: {count} (RED={redCount}, XTRA={xtraCount})")
    print(f"  QSPI firmware:  0x90040000")
    print(f"  QSPI samples:   0x90080000  (256K offset)")
    print(f"  QSPI disponible: {7936 - 256 - total_blob//1024:.0f} KB restantes")
    print(f"{'='*50}")

if __name__ == "__main__":
    main()
