# Auditoría P4 + DaisyPod3 — FX, Sequencer, Song, Piano y XTRA

Fecha: 10 de agosto de 2026
Rama: `claude/p4-daisypod3-audit-97l195`

Esta revisión sustituye a la auditoría anterior, que solo cubría la cadena de
FX. Aquí están los fallos encontrados en esta pasada, con la corrección
aplicada en cada caso.

---

## 1. FX — efectos que no sonaban o sonaban mal

### AUTOWAH no hacía absolutamente nada

`InitFX()` llamaba a `masterAutowahL.SetWah(0.0f)` y **ningún comando volvía a
tocar ese parámetro nunca**. Mirando el `Process()` de DaisySP:

```c
float fSlow3 = (1.0f - 0.01f * wet_dry_) + (1.f - wah_);
out = (wah_ * (rec0_[0] - rec0_[1])) + (fSlow3 * in);
```

Con `wah_ = 0` y `wet_dry_ = 100` queda `out = in`: un cable. El control
`CMD_AUTOWAH_LEVEL` movía `SetLevel()`, que solo escala la rama húmeda — es
decir, escalaba algo que valía cero.

**Corregido:** `CMD_AUTOWAH_LEVEL` ahora escribe `SetWah()`, que es el barrido
real, y el nivel interno queda fijo en 1.0. El `autowahMix` del firmware sigue
haciendo la mezcla final.

### ALLPASS era inaudible por definición

Un filtro all-pass tiene respuesta de magnitud plana. En el bus master, con
solo `gFilterL.Process(L)`, no cambia nada perceptible: únicamente rota fase.
Seleccionar `ALLPASS` en la tarjeta FILTER no producía ningún efecto.

**Corregido:** la rama de ALLPASS mezcla la salida con la señal seca. Para un
all-pass de segundo orden, `(dry + allpass) / 2` es exactamente el notch
correspondiente, así que ahora el modelo es un notch sintonizable con CUTOFF y
profundidad controlada por RESO.

### PEAK / LOW SHELF / HIGH SHELF: +6 dB fijos y RESO muerto

El payload de filtro no tiene campo de ganancia, así que estos tres modelos
recibían `+6 dB` constantes. Apenas se oye, no se puede cortar, y RESO solo
cambiaba el ancho de banda — con lo que en la práctica no hacía nada útil.

**Corregido:** en estos tres modelos **RESO es la ganancia**, de −18 dB a
+18 dB con el centro del recorrido plano, y la Q queda fija en 0,9 (musical).
La tarjeta RESO de `FX LAB` cambia su etiqueta a `EQ GAIN` y su lectura a dB
cuando hay un modelo de EQ activo, y su "OFF" pasa a ser el centro plano en vez
del corte total.

### Resonancia de LADDER / SVF / COMB: recorrido inútil

- Ladder y SVF: `SetRes(gFilterQ / 28)`. Con la Q por defecto de 0,707 eso da
  **0,025**, y el tope del recorrido de la UI (Q 20) solo llegaba a 0,71.
- Comb: `feedback = gFilterQ / 30` → **0,023** por defecto. No se oía peine
  hasta empujar RESO por encima del 90 %.

**Corregido:** una única curva `NormalizedResonance()` normaliza contra el
rango que la UI produce de verdad (Q 0,7–20) con perfil raíz, llega a 0,5 a
mitad de recorrido y se queda en 0,95 para no auto-oscilar. El comb deriva su
realimentación de esa misma curva (0,15–0,97).

### Seleccionar un modelo de filtro no se oía

Con CUTOFF en su extremo neutro (20 kHz por defecto), un LOWPASS está
completamente abierto y un HIGHPASS completamente cerrado: elegir el modelo no
producía ningún cambio audible.

**Corregido:** `control_send_set_filter()` coloca el cutoff en un punto musical
la primera vez que se activa un modelo si estaba aparcado en un extremo
(LP/RESONANT/LADDER/SVF-LP → 6 kHz, HP → 300 Hz, LOW SHELF → 220 Hz,
HIGH SHELF → 5 kHz, COMB → 220 Hz, resto → 1,2 kHz). Un cutoff que el usuario
ya haya ajustado no se toca.

### Otros arreglos de la cadena de filtro

| Fallo | Corrección |
|---|---|
| Los tres manejadores (`FILTER_SET`, `FILTER_CUTOFF`, `FILTER_RESONANCE`) duplicaban el bloque de aplicación. | Un único `ApplyGlobalFilterSettings()`. |
| Cambiar de modelo dejaba el historial del biquad anterior: clic audible y colas de Q alta sonando en un filtro sin polos para amortiguarlas. | Reset de los cuatro biquads al cambiar de tipo. |
| El clamp de Q a 40 (solo válido para RESONANT) se aplicaba por comando: pasar de RESONANT a LOWPASS dejaba Q 40 en un solo biquad → pico brutal. | El clamp se recalcula con el tipo vigente en cada aplicación. |
| SVF podía recibir cutoff por encima de `sr/3`, donde no es incondicionalmente estable. | Clamp a `SAMPLE_RATE / 3.2`. |
| `CMD_RESET` volvía a `Init()` los módulos DaisySP sin reaplicar cutoff/resonancia: el filtro volvía a sus defaults mientras el estado reportado seguía mostrando los valores del usuario. | `ApplyGlobalFilterSettings(true)` tras el reset. |
| La tarjeta BITS escribía en `p4.pot_value[1]`, que pertenece a la macro CRUSH: mover BITS hacía saltar el arco de CRUSH. | BITS ya no escribe ese slot. |

### Comprobado y correcto (no tocado)

El `* 0.25f` del phaser **es correcto**: `Phaser::Process()` de DaisySP suma
cuatro `PhaserEngine` sin normalizar, así que sin ese factor habría un salto de
unos +12 dB. Se verificó contra el código fuente de la revisión fijada.

---

## 2. Sequencer — las variaciones

Las diez variaciones anteriores no eran transformaciones: eran **sellos fijos**.
`NEON BREAK` escribía golpes en las pistas 3, 5, 6, 8, 9 y 10 en pasos
concretos; `ACID SWITCH` borraba las pistas 11–15 enteras y las reescribía. El
resultado era idéntico dieras el patrón que dieras, y aplicarlas sobre un
patrón importado o propio lo destruía y lo sustituía por el beat de otro.

**Reescritas por completo.** Las trece opciones nuevas operan sobre el
contenido que ya hay, deducen el nivel de velocity de cada pista y respetan los
roles de bombo y caja:

| Variación | Qué hace con *tu* patrón |
|---|---|
| GHOST GROOVE | Añade fantasmas en el semicorchea previo a cada golpe existente, con probabilidad. No borra nada. |
| ACCENT GROOVE | Redibuja las dinámicas (tiempo fuerte, corchea, semicorchea, pickup). No mueve ni una nota. |
| ROTATE 3/16 | Gira la percusión tres semicorcheas; bombo y caja quedan anclados. |
| SWAP HALVES | Intercambia tiempos 1–2 con 3–4. Sustituye a `MIRROR`, que reflejaba el compás y mandaba el tiempo fuerte al paso 15. |
| HALF TIME | Estira el primer medio compás sobre el compás entero. |
| DOUBLE TIME | Pliega el compás a la mitad y lo toca dos veces. |
| EUCLID SPREAD | Redistribuye uniformemente los golpes que cada pista ya tiene; conserva la densidad. |
| 16TH LIFT | Rellena contratiempos en la pista percusiva más ocupada, a su propio nivel. |
| RATCHET STORM | Convierte en redobles los golpes que ya caen en el último tiempo. |
| THIN OUT | Abre huecos quitando contratiempos, nunca tiempos fuertes ni bombo/caja. |
| DICE | Deja las notas y da probabilidad a los contratiempos. |
| BREAK | Vacía el primer tiempo de la percusión y lo responde con el material del último, desplazado. |
| UNDO | Igual que antes. |

Son deterministas: la misma variación sobre el mismo patrón da siempre el mismo
resultado, y UNDO revierte exactamente una aplicación.

**Verificado** con un banco de pruebas compilado aparte
(`vartest.cpp`, no incluido en el firmware) sobre un patrón de referencia:
las trece producen cambio, ninguna es aleatoria entre ejecuciones, y UNDO
restaura pasos, velocities y probabilidades. Única diferencia residual tras
UNDO: un ratchet almacenado como `0` puede volver como `1` en pasos que estaban
inactivos. Los dos valores significan "sin redoble" en todo el pipeline
(`Clamp<uint8_t>(ratchet, 1, 4)` tanto al leer como al subir a Daisy), así que
es indistinguible musicalmente. Es comportamiento preexistente, no introducido
aquí.

---

## 3. Modo DRY / CLUB WARM — eliminado

Hacía exactamente esto:

```cpp
control_send_set_volume(club_warm ? 110 : 100);
control_send_set_filter_cutoff(club_warm ? 12000 : 20000);
control_send_set_filter_resonance(club_warm ? 1.4f : 1.0f);
```

Un poco de volumen y un lowpass a 12 kHz — que además solo se oía si ya había
un modelo de filtro seleccionado, porque `FILTER=OFF` ignora el cutoff. Y
pisaba en silencio tres controles que el usuario podía tener ajustados en
`FX LAB`.

Eliminado por completo: `seq_club_warm`, `seq_mix_preset_cb`,
`control_send_mix_preset()` y el botón de cabecera.

---

## 4. Modo SONG

El encadenado existía en las dos placas pero era prácticamente inalcanzable:

- No había ningún control en pantalla. Solo se activaba como efecto colateral
  de importar un `.mid` con `FULL SONG`.
- `control_send_select_pattern()` llamaba a `control_cancel_midi_song()`, que
  **destruía el arrangement entero**. Tocar `PATTERN +` una vez y la canción
  desaparecía; había que reimportar el MIDI.
- El bucle estaba atado a `kStartupShowcaseDemo`, una constante de compilación:
  una canción importada siempre paraba al llegar al final, sin opción de
  repetir.
- P4 no sabía en qué punto de la cadena estaba Daisy.

**Corregido:**

- Separación entre *cargada* (`midiSongPrepared`) y *activa*
  (`midiSongEngaged`). Salir a un patrón suelto ahora solo desactiva el modo;
  la cadena sigue residente y se puede volver a ella.
- Botón `SONG` en la cabecera del sequencer, en el hueco que dejó DRY/CLUB.
  Cicla `SONG OFF` → `SONG n/N` → `LOOP n/N` → `SONG OFF`, y muestra la
  posición real de la cadena mientras suena.
- `songLoop` en Daisy, con la bandera viajando en `CMD_SONG_CONTROL` como
  segundo byte (los emisores antiguos siguen funcionando, sin bucle).
- `PodStatePayload` crece de 66 a 70 bytes con `songIndex`, `songLength`,
  `songRepeat` y `songFlags`. Los campos van **al final** a propósito: el
  receptor acepta cualquier paquete de al menos su propio tamaño, así que las
  dos placas se pueden grabar por separado sin que el enlace deje de enumerar.
- Cuando una canción sin bucle termina, Daisy sale de modo song y P4 lo sigue
  en vez de dejar la cabecera diciendo que sigue sonando.
- Reactivar SONG vuelve a subir las escenas residentes, porque los slots de
  Daisy pueden haber quedado pisados por el patrón que se estuviera
  escuchando. La subida se hace desde `ui_process_control_queue()` en el core
  1, nunca desde el callback de LVGL.

---

## 5. Piano

El motor de modelado físico (engine 7) estaba **completamente implementado en
las dos placas** — `ApplyPhysPreset()`, `ModalVoice` + `StringVoice`,
enrutado en el `AudioCallback`, `IsPianoMelodicEngine()` lo acepta — y tenía su
tabla de parámetros y sus cuatro presets en `shared/synth_params.h`. Pero
`SP_ENGINE_COUNT` valía 4 y el comentario decía "reserved legacy
physical-model engine (not exposed in the UI)". Nadie podía llegar a él.

**Corregido:** `SP_ENGINE_COUNT = 5`. `PIANO` pasa a cinco motores
(303 / WT / SH101 / FM2 / PHYS), y `PIANO PARAMS` gana su pestaña con los ocho
parámetros y los presets Clásica / Flamenco / Funky / Eléctrica.

También: `s_pp_preset_idx[SP_ENGINE_COUNT] = { -1 }` solo inicializaba el
elemento 0. Los demás motores arrancaban en el preset 0 mientras su fila de
chips no mostraba ninguno seleccionado.

---

## 6. XTRA PADS

**Los presets estaban desalineados.** Cada motor tiene cuatro presets, pero los
pads exponían tres (`XTRA_PRESET_LABELS[3] = {"A","B","C"}`) y envolvían el
índice con `% 3` en cinco sitios. Eligiendo el cuarto preset desde
`PIANO PARAMS` para un slot XTRA se guardaba un `3` que después:

- se enviaba a Daisy como **preset 0** (`preset_idx % 3`), y
- cargaba en la instantánea local de parámetros los valores del **preset 3**
  (`constrain(preset_idx, 0, preset_count - 1)`).

El pad no sonaba como ninguno de los dos.

**Corregido:** cuatro presets A/B/C/D en todas partes, con una única función
`xtra_preset_index()` que fija el rango, de modo que la etiqueta, la
instantánea de parámetros y el preset enviado por USB no puedan volver a
discrepar. Las tablas `XTRA_DRUM_INSTRUMENTS` y `XTRA_MELODIC_BASE_NOTES`
crecen a cuatro variantes.

**PHYS también en los pads:** `xtra_engine_idx_from_pp_engine()` ya devolvía 7,
pero `xtra_slot_engine_code()` recortaba a `0..6` y lo convertía en FM2 sin
avisar. El modulador XY ya tenía su rama `case 7:` escrita y sin usar.

---

## 7. DROP

`control_send_drop()` silenciaba las pistas 2..15 y las dejaba así. El botón
solo se podía pulsar una vez, y la única salida era desmutear catorce pistas a
mano.

**Corregido:** es un toggle y recuerda qué pistas ya estaban silenciadas, así
que soltar el drop devuelve la mezcla que el usuario tenía.

---

## Pendiente / no tocado en esta pasada

Encontrado pero fuera del alcance pedido, documentado para la siguiente:

- **`control_send_fill()` y `control_send_build4()`** son destructivos igual que
  lo era DROP: reescriben permanentemente la pista 2 (fill) y los pasos 12–15
  de la pista 1 (build), sin retorno. El toast dice "1 compás + retorno" y
  "4 compases", pero no hay ni retorno ni temporización.
- **Beat repeat** (`beatRepRp` / `beatRepWp`): la condición de bucle de slice
  mezcla dos comparaciones de ventana y puede saltar el punto de lectura de
  forma no obvia al envolver el buffer circular.
- **Autowah doble dry/wet**: DaisySP ya mezcla internamente vía `wah_`, y el
  firmware vuelve a mezclar con `autowahMix`. Suena bien pero la curva del
  control no es lineal en la mezcla.
- **Presupuesto de CPU**: la revisión sigue siendo estática. Hay que medir en
  placa con `CMD_GET_CPU_LOAD` y modo stress, 16 pistas y los seis FX master
  activos, para fijar un presupuesto cuantitativo.

## Pruebas recomendadas en hardware

1. `FX LAB` → FILTER: recorrer los 15 modelos y confirmar que **todos** se oyen
   nada más seleccionarlos, sin tocar CUTOFF.
2. Con PEAK / LOW SHELF / HIGH SHELF, barrer RESO de extremo a extremo y
   comprobar corte y realce reales, con la lectura en dB.
3. AUTOWAH: activarlo y confirmar el barrido con el nivel a distintos valores.
4. Cargar un `.mid` con `FULL SONG`, pulsar `SONG`, dejar que encadene, pulsar
   otra vez para `LOOP`, salir a un patrón suelto con `PATTERN +` y volver a
   `SONG`: la canción debe seguir ahí.
5. Aplicar cada variación sobre un patrón de fábrica y sobre uno importado, y
   confirmar que el resultado depende del patrón de partida.
6. `PIANO` → `PHYS`, y un slot XTRA en PHYS con los cuatro presets A/B/C/D.
