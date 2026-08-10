import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const here = path.dirname(fileURLToPath(import.meta.url));
const workspace = path.resolve(here, "../..");
const source = path.join(
  workspace,
  "RedMaster_ESP32S3/data/patterns/20_patrones_factory_daisy.json",
);
const output = path.join(
  here,
  "../P4/src/master/LegacyFactoryBank.generated.h",
);

const bank = JSON.parse(fs.readFileSync(source, "utf8"));
if (bank.stepCount !== 16 || bank.patterns?.length !== 20) {
  throw new Error("Expected the original 20-pattern, 16-step S3 factory bank");
}

const q = (value) => JSON.stringify(String(value));
const u8 = (values, fallback = 0) => {
  const data = Array.from({ length: 16 }, (_, index) => {
    const value = Number(values?.[index] ?? fallback);
    return Math.max(0, Math.min(255, value | 0));
  });
  return `{${data.join(",")}}`;
};

const tracks = [];
const patterns = [];
for (const pattern of bank.patterns) {
  const firstTrack = tracks.length;
  for (const track of pattern.tracks ?? []) {
    let activeMask = 0;
    for (let step = 0; step < 16; step++) {
      if (track.steps?.[step]) activeMask |= (1 << step);
    }
    const velocities = Array.from({ length: 16 }, (_, step) => {
      if (!track.velocities) return 127;
      return Math.max(1, Math.min(127, Number(track.velocities[step] ?? 0) | 0));
    });
    tracks.push({
      track: Number(track.track) | 0,
      engine: Math.max(-1, Math.min(8, Number(track.engine ?? -1) | 0)),
      preset: Math.max(0, Math.min(31, Number(track.preset ?? 0) | 0)),
      activeMask,
      velocities,
      notes: track.notes,
      flags: track.flags,
    });
  }
  patterns.push({
    slot: Number(pattern.slot) | 0,
    name: pattern.name,
    firstTrack,
    trackCount: tracks.length - firstTrack,
  });
}

const lines = [
  "#pragma once",
  "",
  "// Generated from RedMaster_ESP32S3/data/patterns/20_patrones_factory_daisy.json.",
  "// Do not edit by hand; regenerate with tools/generate_legacy_factory_bank.mjs.",
  "struct LegacyFactoryTrackSeed {",
  "  uint8_t track;",
  "  int8_t engine;",
  "  uint8_t preset;",
  "  uint16_t activeMask;",
  "  uint8_t velocities[16];",
  "  uint8_t notes[16];",
  "  uint8_t flags[16];",
  "};",
  "",
  "struct LegacyFactoryPatternSeed {",
  "  uint8_t slot;",
  "  const char* name;",
  "  uint16_t firstTrack;",
  "  uint8_t trackCount;",
  "};",
  "",
  `static constexpr uint16_t LEGACY_FACTORY_TEMPO = ${Number(bank.tempo) | 0};`,
  "static constexpr LegacyFactoryTrackSeed LEGACY_FACTORY_TRACKS[] = {",
];

for (const track of tracks) {
  lines.push(
    `  {${track.track},${track.engine},${track.preset},0x${track.activeMask.toString(16).padStart(4, "0")},`
      + `{${track.velocities.join(",")}},${u8(track.notes)},${u8(track.flags)}},`,
  );
}
lines.push("};", "", "static constexpr LegacyFactoryPatternSeed LEGACY_FACTORY_PATTERNS[] = {");
for (const pattern of patterns) {
  lines.push(
    `  {${pattern.slot},${q(pattern.name)},${pattern.firstTrack},${pattern.trackCount}},`,
  );
}
lines.push("};", "");

const generated = `${lines.join("\n")}\n`;
if (process.argv.includes("--check")) {
  const current = fs.readFileSync(output, "utf8");
  if (current !== generated) {
    throw new Error(`Generated factory bank is stale: ${output}`);
  }
  console.log(`Verified ${output}`);
} else {
  fs.writeFileSync(output, generated, "utf8");
  console.log(`Generated ${output}`);
}
console.log(`${patterns.length} patterns, ${tracks.length} track records`);
