#!/usr/bin/env python3
"""
wav2header.py — Convierte todos los WAV de data/RED 808 KARZ a un header C
con arrays int16_t (mono, 16-bit) listos para embeber en el firmware Daisy Seed.

Uso:  python wav2header.py
Genera: embedded_samples.h
"""

import wave
import struct
import os
import sys
import re

WAV_DIR = os.path.join("data", "RED 808 KARZ")
OUTPUT  = "embedded_samples.h"

# Mapeo ESPECÍFICO para RED 808 KARZ (nombres exactos del kit)
# Primero intentamos matching exacto por prefijo "808 XX"
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

# Fallback genérico por keyword (si no matchea exacto)
KEYWORD_MAP = [
    ("BD",  0), ("KICK", 0),
    ("SD",  1), ("SNARE",1),
    ("CH",  2), ("HH",   2), ("HIHAT",2), ("CLOSED",2),
    ("OH",  3), ("OPEN", 3),
    ("CY",  4), ("CYMBAL",4), ("CRASH",4), ("RIDE",4),
    ("CP",  5), ("CLAP", 5),
    ("RS",  6), ("RIM",  6),
    ("CB",  7), ("COW",  7), ("BELL", 7),
    ("LT",  8), ("LTOM", 8),
    ("MT",  9), ("MTOM", 9),
    ("HT", 10), ("HTOM",10),
    ("MA", 11), ("MARAC",11),
    ("CL", 12), ("CLAV", 12), ("CLAVE",12),
    ("HC", 13), ("CONGA",13),
    ("MC", 14),
    ("LC", 15),
]

PAD_NAMES = [
    "BD","SD","CH","OH","CY","CP","RS","CB",
    "LT","MT","HT","MA","CL","HC","MC","LC"
]

def guess_pad(filename):
    """Guess pad index from filename using exact prefix matching first, then keywords."""
    upper = filename.upper()
    # Try exact prefix match first (e.g. "808 BD" → pad 0)
    for prefix, pad in EXACT_MAP.items():
        if upper.startswith(prefix.upper()):
            return pad
    # Fallback to keyword match
    for kw, pad in KEYWORD_MAP:
        if kw in upper:
            return pad
    return -1

def sanitize_name(filename):
    """Convert filename to valid C identifier."""
    name = os.path.splitext(filename)[0]
    name = re.sub(r'[^a-zA-Z0-9]', '_', name)
    name = re.sub(r'_+', '_', name).strip('_')
    return "wav_" + name

def read_wav_mono16(filepath):
    """Read WAV file, convert to mono 16-bit signed samples."""
    with wave.open(filepath, 'rb') as wf:
        ch = wf.getnchannels()
        sw = wf.getsampwidth()  # bytes per sample
        sr = wf.getframerate()
        nframes = wf.getnframes()
        raw = wf.readframes(nframes)
    
    samples = []
    
    if sw == 2:  # 16-bit
        fmt = f"<{nframes * ch}h"
        data = struct.unpack(fmt, raw)
        if ch == 1:
            samples = list(data)
        else:
            for i in range(0, len(data), ch):
                avg = sum(data[i:i+ch]) // ch
                samples.append(max(-32768, min(32767, avg)))
    elif sw == 1:  # 8-bit unsigned
        for i in range(0, len(raw), ch):
            vals = [raw[i+c] for c in range(ch)]
            avg = sum(vals) // ch
            samples.append((avg - 128) * 256)
    elif sw == 3:  # 24-bit
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
    
    # TWO-PASS approach for correct pad assignment:
    # Pass 1: Assign each unique instrument to its correct pad (first file wins)
    # Pass 2: Put duplicates into remaining free slots
    
    file_with_pad = []  # (filename, guessed_pad)
    for fname in wavfiles:
        pad = guess_pad(fname)
        file_with_pad.append((fname, pad))
    
    entries = []  # (pad_idx, var_name, samples, sr, filename)
    pad_used = set()
    assigned_files = set()
    
    # Pass 1: unique instruments get their correct pad
    for fname, pad in file_with_pad:
        if pad >= 0 and pad < 16 and pad not in pad_used:
            filepath = os.path.join(WAV_DIR, fname)
            samples, sr = read_wav_mono16(filepath)
            var_name = sanitize_name(fname)
            pad_used.add(pad)
            assigned_files.add(fname)
            entries.append((pad, var_name, samples, sr, fname))
            print(f"  Pad {pad:2d} ({PAD_NAMES[pad]:2s}) <- {fname} ({len(samples)} samples, {sr} Hz)")
    
    # Pass 2: duplicates/unmatched go to free slots
    for fname, pad in file_with_pad:
        if fname in assigned_files:
            continue
        # Find first free slot
        free = -1
        for s in range(16):
            if s not in pad_used:
                free = s
                break
        if free < 0:
            print(f"  SKIP {fname} (no free pad slot)")
            continue
        filepath = os.path.join(WAV_DIR, fname)
        samples, sr = read_wav_mono16(filepath)
        var_name = sanitize_name(fname)
        pad_used.add(free)
        entries.append((free, var_name, samples, sr, fname))
        print(f"  Pad {free:2d} ({PAD_NAMES[free]:2s}) <- {fname} [extra] ({len(samples)} samples, {sr} Hz)")
    
    # Sort by pad index
    entries.sort(key=lambda e: e[0])
    
    # Calculate total size
    total_samples = sum(len(e[2]) for e in entries)
    total_bytes = total_samples * 2
    print(f"\nTotal: {len(entries)} samples, {total_samples} muestras, {total_bytes} bytes ({total_bytes/1024:.1f} KB)")
    
    # Generate header
    with open(OUTPUT, 'w', encoding='utf-8') as f:
        f.write("/* ═══════════════════════════════════════════════════════════════\n")
        f.write(" *  embedded_samples.h — RED 808 KARZ WAV samples embebidos\n")
        f.write(" *  Generado automáticamente por wav2header.py\n")
        f.write(" *  NO EDITAR A MANO\n")
        f.write(" * ═══════════════════════════════════════════════════════════════ */\n")
        f.write("#pragma once\n")
        f.write("#include <stdint.h>\n\n")
        f.write("/* Los arrays van en QSPI Flash (memory-mapped, 7936 KB)\n")
        f.write("   para no ocupar SRAM (480 KB, que es solo para codigo). */\n")
        f.write("#define DSY_QSPI_DATA __attribute__((section(\".qspiflash_data\")))\n\n")
        
        f.write(f"#define EMBEDDED_SAMPLE_COUNT {len(entries)}\n\n")
        
        # Write each sample array
        for pad, var_name, samples, sr, fname in entries:
            f.write(f"/* Pad {pad} ({PAD_NAMES[pad]}) — {fname} — {len(samples)} samples @ {sr} Hz */\n")
            f.write(f"static const DSY_QSPI_DATA int16_t {var_name}[] = {{\n")
            
            # Write in rows of 16
            for i in range(0, len(samples), 16):
                chunk = samples[i:i+16]
                line = ", ".join(f"{s}" for s in chunk)
                if i + 16 < len(samples):
                    f.write(f"    {line},\n")
                else:
                    f.write(f"    {line}\n")
            f.write("};\n\n")
        
        # Write index table
        f.write("/* ── Index table: pad → embedded sample ──────────────────── */\n")
        f.write("struct EmbeddedSample {\n")
        f.write("    uint8_t        padIdx;\n")
        f.write("    const int16_t* data;\n")
        f.write("    uint32_t       length;  /* number of int16_t samples */\n")
        f.write("    const char*    name;\n")
        f.write("};\n\n")
        
        f.write(f"static const EmbeddedSample embeddedSamples[{len(entries)}] = {{\n")
        for pad, var_name, samples, sr, fname in entries:
            safe_fname = fname.replace('"', '\\"')
            f.write(f"    {{ {pad:2d}, {var_name}, {len(samples):6d}, \"{safe_fname}\" }},\n")
        f.write("};\n")
    
    print(f"\n✓ Generado: {OUTPUT}")
    print(f"  Incluir en main.cpp: #include \"embedded_samples.h\"")

if __name__ == "__main__":
    main()
