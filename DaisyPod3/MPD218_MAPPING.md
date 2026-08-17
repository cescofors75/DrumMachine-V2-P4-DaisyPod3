# RED808 — mapa completo Akai MPD218 + CME H4MIDI WC

Cada controlador usa **3 programas RED808** como bancos y, dentro de cada
programa, las **3 capas A/B/C**. El firmware admite **dos controladores MIDI
simultáneos** en el flujo fusionado del H4MIDI. Cada uno conserva su propio
contexto y dispone de 144 usos de pad (`3 × 3 × 16`) y 54 usos de potenciómetro
(`3 × 3 × 6`): 288 pads y 108 knobs direccionables entre ambos.

Los botones `PAD BANK` y `CTRL BANK` del MPD218 son independientes: se puede,
por ejemplo, tocar los pads de la capa A mientras los potenciómetros están en C.

## 1. Cableado

```text
Controlador A ── USB-A HOST o MIDI IN 1 ─┐
                                         ├─ CME H4MIDI WC (MERGE)
Controlador B ── hub USB o MIDI IN 2 ────┘          │
                                                   │ MIDI OUT 1
                                                   ▼
                                    DIN ↔ TRS MIDI Type A
                                                   │
                                                   ▼
                           Daisy Pod MIDI IN — D14/PB7
```

- Conectar el MPD218 al puerto **USB HOST** del H4MIDI. Para dos controladores
  USB, usar un hub USB alimentado si el consumo conjunto se aproxima a 1 A.
- También se pueden usar `MIDI IN 1` y `MIDI IN 2` para dispositivos DIN.
- En HxMIDI Tools, fusionar las entradas empleadas hacia `MIDI OUT 1`.
- La entrada de la Daisy es **TRS MIDI Type A**. Un adaptador Type B no funciona.
- La implementación es MIDI IN solamente. D13/PB6 continúa dedicado al pulsador
  del encoder de este hardware.
- El flujo fusionado no conserva una identidad de puerto utilizable por la
  Daisy. Los dos dispositivos deben quedar separados por canales MIDI.

## 2. Programación común en Akai MPD218 Editor

Crear y guardar tres programas en cada controlador. El A usa los canales 1–3 y
el B los canales 4–6:

| Programa | Nombre sugerido | Canal controlador A | Canal controlador B | Banco RED808 |
|---:|---|---:|---:|---:|
| 1 | `R8-LIVE` | 1 | 4 | 1 — Live/master |
| 2 | `R8-SEQ` | 2 | 5 | 2 — Secuenciador/pistas |
| 3 | `R8-SYN` | 3 | 6 | 3 — Sintetizadores |

Si el segundo dispositivo no permite seleccionar esos canales, usar el MIDI
Mapper de HxMIDI Tools para convertir sus canales de entrada a 4/5/6 antes de
fusionarlos. Nunca dejar ambos controladores en 1/2/3: después del merge serían
indistinguibles.

En **cada uno de los tres programas de ambos dispositivos**, asignar estos
números. Las notas y CC son iguales; solo cambia el canal indicado arriba:

### Pads

| Pad físico | Capa A — Note | Capa B — Note | Capa C — Note |
|---:|---:|---:|---:|
| 1 | 36 | 52 | 68 |
| 2 | 37 | 53 | 69 |
| 3 | 38 | 54 | 70 |
| 4 | 39 | 55 | 71 |
| 5 | 40 | 56 | 72 |
| 6 | 41 | 57 | 73 |
| 7 | 42 | 58 | 74 |
| 8 | 43 | 59 | 75 |
| 9 | 44 | 60 | 76 |
| 10 | 45 | 61 | 77 |
| 11 | 46 | 62 | 78 |
| 12 | 47 | 63 | 79 |
| 13 | 48 | 64 | 80 |
| 14 | 49 | 65 | 81 |
| 15 | 50 | 66 | 82 |
| 16 | 51 | 67 | 83 |

Configurar los pads como mensajes `Note`, con velocidad activada y comportamiento
momentáneo. El firmware admite `Note On`, `Note Off` y `Note On velocity 0`.

### Potenciómetros

| Knob físico | Capa A — CC | Capa B — CC | Capa C — CC |
|---:|---:|---:|---:|
| K1 | 20 | 26 | 32 |
| K2 | 21 | 27 | 33 |
| K3 | 22 | 28 | 34 |
| K4 | 23 | 29 | 35 |
| K5 | 24 | 30 | 36 |
| K6 | 25 | 31 | 37 |

Configurar todos los knobs con rango MIDI completo `0..127`.

## 3. Banco 1 — `R8-LIVE` — A: canal 1 / B: canal 4

### Pads, capa A — samples principales

| Pad | Note | Acción |
|---:|---:|---|
| 1 | 36 | Sample 1 |
| 2 | 37 | Sample 2 |
| 3 | 38 | Sample 3 |
| 4 | 39 | Sample 4 |
| 5 | 40 | Sample 5 |
| 6 | 41 | Sample 6 |
| 7 | 42 | Sample 7 |
| 8 | 43 | Sample 8 |
| 9 | 44 | Sample 9 |
| 10 | 45 | Sample 10 |
| 11 | 46 | Sample 11 |
| 12 | 47 | Sample 12 |
| 13 | 48 | Sample 13 |
| 14 | 49 | Sample 14 |
| 15 | 50 | Sample 15 |
| 16 | 51 | Sample 16 |

La velocidad MIDI controla la fuerza del disparo. El sample tocado pasa a ser la
pista/pad seleccionado para las operaciones contextuales **de ese controlador**.

### Pads, capa B — samples extra, transporte y FX del pad seleccionado

| Pad | Note | Acción |
|---:|---:|---|
| 1 | 52 | Sample 17 |
| 2 | 53 | Sample 18 |
| 3 | 54 | Sample 19 |
| 4 | 55 | Sample 20 |
| 5 | 56 | Sample 21 |
| 6 | 57 | Sample 22 |
| 7 | 58 | Sample 23 |
| 8 | 59 | Sample 24 |
| 9 | 60 | Play/Pause del secuenciador |
| 10 | 61 | Stop + panic de samples y sintes |
| 11 | 62 | Patrón anterior |
| 12 | 63 | Patrón siguiente |
| 13 | 64 | Loop ON/OFF del pad seleccionado |
| 14 | 65 | Reverse ON/OFF del pad seleccionado |
| 15 | 66 | Stutter ON/OFF del pad seleccionado |
| 16 | 67 | Borrar FX del pad seleccionado |

### Pads, capa C — teclado melódico

| Pad | Note enviada al mapa | Nota musical generada |
|---:|---:|---|
| 1 | 68 | C3 (MIDI 48) |
| 2 | 69 | C#3 (49) |
| 3 | 70 | D3 (50) |
| 4 | 71 | D#3 (51) |
| 5 | 72 | E3 (52) |
| 6 | 73 | F3 (53) |
| 7 | 74 | F#3 (54) |
| 8 | 75 | G3 (55) |
| 9 | 76 | G#3 (56) |
| 10 | 77 | A3 (57) |
| 11 | 78 | A#3 (58) |
| 12 | 79 | B3 (59) |
| 13 | 80 | C4 (60) |
| 14 | 81 | C#4 (61) |
| 15 | 82 | D4 (62) |
| 16 | 83 | D#4 (63) |

Toca el motor melódico actualmente seleccionado: TB-303, Wavetable, SH-101,
FM 2-op o Physical. Usa velocidad, acento desde velocidad alta y Note Off real.

### Knobs

| Knob | Capa A / CC 20..25 | Capa B / CC 26..31 | Capa C / CC 32..37 |
|---:|---|---|---|
| K1 | Master volume | Master filter cutoff | Flanger depth |
| K2 | Live volume | Master filter resonance | Phaser depth |
| K3 | Sequencer volume | Master distortion | Wavefolder gain |
| K4 | Tempo 40..240 BPM | Master bit depth 16..4 bit | Crush macro |
| K5 | Delay mix | Master sample-rate reduction | Live pitch 0,5x..2x |
| K6 | Reverb mix | Master filter type | Seleccionar pad/pista 1..16 |

## 4. Banco 2 — `R8-SEQ` — A: canal 2 / B: canal 5

### Pads

| Pad | Capa A / Notes 36..51 | Capa B / Notes 52..67 | Capa C / Notes 68..83 |
|---:|---|---|---|
| 1 | Seleccionar patrón 1 | Mute pista 1 | Seleccionar pista 1 |
| 2 | Seleccionar patrón 2 | Mute pista 2 | Seleccionar pista 2 |
| 3 | Seleccionar patrón 3 | Mute pista 3 | Seleccionar pista 3 |
| 4 | Seleccionar patrón 4 | Mute pista 4 | Seleccionar pista 4 |
| 5 | Seleccionar patrón 5 | Mute pista 5 | Seleccionar pista 5 |
| 6 | Seleccionar patrón 6 | Mute pista 6 | Seleccionar pista 6 |
| 7 | Seleccionar patrón 7 | Mute pista 7 | Seleccionar pista 7 |
| 8 | Seleccionar patrón 8 | Mute pista 8 | Seleccionar pista 8 |
| 9 | Seleccionar patrón 9 | Mute pista 9 | Seleccionar pista 9 |
| 10 | Seleccionar patrón 10 | Mute pista 10 | Seleccionar pista 10 |
| 11 | Seleccionar patrón 11 | Mute pista 11 | Seleccionar pista 11 |
| 12 | Seleccionar patrón 12 | Mute pista 12 | Seleccionar pista 12 |
| 13 | Seleccionar patrón 13 | Mute pista 13 | Seleccionar pista 13 |
| 14 | Seleccionar patrón 14 | Mute pista 14 | Seleccionar pista 14 |
| 15 | Seleccionar patrón 15 | Mute pista 15 | Seleccionar pista 15 |
| 16 | Seleccionar patrón 16 | Mute pista 16 | Seleccionar pista 16 |

Los mutes actualizan a la vez el mixer y el secuenciador. RED808 dispone de 20
patrones; los patrones 17..20 también se alcanzan con el knob C5.

### Knobs

| Knob | Capa A / CC 20..25 — pista seleccionada | Capa B / CC 26..31 — FX pista | Capa C / CC 32..37 — secuenciador |
|---:|---|---|---|
| K1 | Volumen | Filter cutoff | Tempo |
| K2 | Pan L..R | Filter resonance | Swing 0..100 % |
| K3 | Reverb send | Distortion | Humanize timing 0..20 ms |
| K4 | Delay send | Bit depth 16..4 bit | Humanize velocity 0..50 |
| K5 | Chorus send | EQ low -12..+12 dB | Patrón 1..20 |
| K6 | Pitch -12..+12 semitonos | EQ high -12..+12 dB | Seleccionar pista 1..16 |

## 5. Banco 3 — `R8-SYN` — A: canal 3 / B: canal 6

### Pads — tres cajas de ritmos

| Pad | Capa A / Notes 36..51 — TR-808 | Capa B / Notes 52..67 — TR-909 | Capa C / Notes 68..83 — TR-505 |
|---:|---|---|---|
| 1 | Kick | Kick | Kick |
| 2 | Snare | Snare | Snare |
| 3 | Closed hi-hat | Closed hi-hat | Closed hi-hat |
| 4 | Open hi-hat | Open hi-hat | Open hi-hat |
| 5 | Cymbal | Crash | Cymbal |
| 6 | Clap | Clap | Clap |
| 7 | Rimshot | Rimshot | Rimshot |
| 8 | Cowbell | Ride | Cowbell |
| 9 | Low tom | Low tom | Low tom |
| 10 | Mid tom | Mid tom | Mid tom |
| 11 | High tom | High tom | High tom |
| 12 | Maracas | Shaker | Shaker |
| 13 | Claves | Clave | Clave |
| 14 | High conga | High percussion | High percussion |
| 15 | Mid conga | Mid percussion | Mid percussion |
| 16 | Low conga | Low percussion | Low percussion |

El último pad de caja tocado selecciona motor e instrumento para los seis knobs
de la capa A del mismo controlador.

### Knobs, capa A — instrumento de batería seleccionado

| Knob | CC | Parámetro contextual |
|---:|---:|---|
| K1 | 20 | Decay |
| K2 | 21 | Pitch/Tune |
| K3 | 22 | Tone/Drive |
| K4 | 23 | Volume |
| K5 | 24 | Snappy/Sub |
| K6 | 25 | Punch/Smack |

Algunos instrumentos no implementan los seis parámetros; en esos casos el knob
no altera nada, para conservar el modelo real del instrumento.

### Knobs, capa B — motor melódico seleccionado

| Knob | CC | TB-303 | Wavetable | SH-101 | FM 2-op | Physical |
|---:|---:|---|---|---|---|---|
| K1 | 26 | Cutoff | Wave position | Cutoff | Ratio | Modal frequency |
| K2 | 27 | Resonance | Attack | Resonance | Index | Modal structure |
| K3 | 28 | Env mod | Decay | VCF env amount | Feedback | Modal brightness |
| K4 | 29 | Decay | Volume | VCA decay | Carrier attack | Modal damping |
| K5 | 30 | Accent | Filter cutoff | VCA release | Carrier decay | Modal level |
| K6 | 31 | Slide | LFO rate | LFO rate | Volume | String frequency |

### Knobs, capa C — selección de sinte y FX adicionales

| Knob | CC | Acción |
|---:|---:|---|
| K1 | 32 | Motor melódico: 303 / Wavetable / SH-101 / FM2OP / Physical |
| K2 | 33 | Preset 1..5 del motor melódico |
| K3 | 34 | Chorus mix |
| K4 | 35 | Tremolo depth |
| K5 | 36 | Auto-wah mix |
| K6 | 37 | Stereo width 0..200 % |

## 6. Convivencia de los dos controladores

Se mantienen dos contextos independientes:

- pista seleccionada;
- pad seleccionado, incluidos los samples 17–24;
- caja 808/909/505 e instrumento seleccionado;
- motor melódico y preset seleccionado.

Por ello, A puede editar la pista 1 mientras B edita la 12, o A puede tocar la
303 mientras B toca el SH-101. Los recursos globales —volumen master, tempo,
transporte y FX master— son deliberadamente compartidos: el último movimiento
recibido establece su valor.

Los motores monofónicos son recursos únicos. Si ambos controladores eligen a la
vez la misma 303, SH-101 o FM2OP, comparten esa voz y un Note Off puede cerrar la
nota del otro. Para interpretación simultánea, seleccionar motores distintos o
usar Wavetable para las partes polifónicas.

### Un único reloj maestro

`MIDI Clock`, `Start`, `Continue`, `Stop` y `Reset` no incluyen canal MIDI. Tras
el merge la Daisy no puede saber qué dispositivo los originó. En HxMIDI Tools:

1. elegir un único dispositivo como maestro de clock/transporte;
2. dejar pasar sus mensajes System Real-Time a `MIDI OUT 1`;
3. filtrar Clock/Start/Stop/Reset del otro dispositivo.

Dos relojes simultáneos producirían cambios de tempo y transporte impredecibles.

## 7. MIDI adicional admitido

- `MIDI Start` y `Continue`: inicia el secuenciador.
- `MIDI Stop`: detiene el secuenciador.
- `MIDI Timing Clock`: tras 24 pulsos calcula y aplica el tempo del único reloj
  maestro configurado en el H4MIDI.
- `All Sound Off` y `All Notes Off`: liberan el motor melódico del dispositivo
  correspondiente sin cortar el otro contexto.
- `MIDI Reset`: panic global de samples y sintes.
- El control USB-C del P4 y MIDI TRS funcionan simultáneamente.

## 8. Prueba rápida del lunes

1. Cargar los tres programas A en canales 1/2/3 y los tres B en 4/5/6.
2. En HxMIDI Tools, fusionar ambos dispositivos a `MIDI OUT 1` y permitir clock
   solamente desde el maestro elegido.
3. Controlador A, programa 1, Pad Bank A: comprobar los samples con dinámica.
4. Controlador B, programa 3, Pad Bank A/B/C: comprobar 808, 909 y 505.
5. En ambos, programa 1, Pad Bank C: mantener una nota y soltarla; el sinte debe cerrar la
   envolvente con Note Off.
6. Seleccionar pistas diferentes con A y B y confirmar que sus knobs no cambian
   de pista al mover el otro controlador.
7. Programa 2, Pad Bank B: confirmar mute/unmute de las 16 pistas.
8. Mover los seis knobs en cada `CTRL BANK` A/B/C de ambos dispositivos.

El mapa fuente del firmware está en `mpd218_mapping.h`; toda la interpretación
MIDI está en la sección `AKAI MPD218 FACTORY MAP` de `main.cpp`.
