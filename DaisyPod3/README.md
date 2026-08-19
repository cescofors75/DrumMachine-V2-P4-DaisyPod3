# DaisyPod3

Firmware de audio para Daisy Pod con Daisy Seed/Seed3.

Contiene el motor RED808 de 64 MB: sampler de 24 pads, secuenciador residente, motores TR-808/TR-909/TR-505/TB-303, wavetable, SH-101, FM de 2 operadores, modelado físico, ruido/texturas, mixer y efectos. Conserva la carga local de kits desde la tarjeta SD del Pod.

El USB interno del Seed funciona como dispositivo CDC binario con el P4. En
paralelo, la entrada UART de D14 acepta un bus MIDI TRS Type A fusionado por el
CME H4MIDI WC. Admite dos controladores simultáneos: A en canales 1–3 y B en
4–6, cada uno con contexto independiente. El mapa completo está en
[MPD218_MAPPING.md](MPD218_MAPPING.md).

## Controles físicos

- Encoder: selecciona pad 1–16.
- Botón 1: play/stop.
- Botón 2: dispara el pad seleccionado.
- Pulsación del encoder: reset de transporte y stop de todas las voces.
- Knob 1: volumen master, 0–150.
- Knob 2: tempo, 40–240 BPM.
- LED 1: estado USB y transporte.
- LED 2: grupo del pad seleccionado y pulso de trigger.

La SPI3 permanece dedicada únicamente a la tarjeta SD local.

## Sincronización main loop / ISR de audio

`AudioCallback` corre como IRQ de audio (SAI/DMA) mientras el resto del
firmware (USB CDC, MIDI TRS, comandos del secuenciador) se procesa en el bucle
principal de `main()`. Disparar un pad o una nota directamente desde el bucle
principal mientras el IRQ recorre `voices[]`/los motores de síntesis en el
mismo instante es una condición de carrera real: una escritura a medias en
`voices[slot]` puede audible como click, silencio o, en el peor caso,
`HardFault`.

Para las rutas de mayor tráfico — disparo de pad (`TriggerPad`), disparo/
note-on de los sintetizadores 808/909/505/303/WTOSC/SH101/FM2OP, y el disparo
de PHYS/NOISE (SetFreq + SetAccent/SetDensity + `Trig()` opcional + flag
`*Active`) — el bucle principal ya no llama a esas funciones directamente. En
su lugar encola un `AudioCmd` en un ring SPSC (`AudioCmdPush`, ver `main.cpp`
justo después de `TriggerPad`) y `AudioCmdDrainAndApply()`, invocado al
inicio de cada bloque de `AudioCallback`, ejecuta la llamada real ya en el
hilo de audio. Solo hay un escritor de `voices[]`/los motores para esas
rutas.

Quedan fuera de esta cola, deliberadamente:

- **Helpers de "release"/NoteOff** (`ReleaseAllSynthEngines`,
  `ReleaseSynthEngineState`, `ReleaseTrackEngine`, `CMD_SYNTH_ACTIVE`,
  `DsqReleaseAllHeldNotes`...) y los bucles que paran voces
  (`StopPadVoices`, `SilenceVoicesInPadRange`,
  `voices[v].active = false` sueltos). Investigado y descartado a
  propósito: en `CMD_SYNTH_PRESET`, por ejemplo, el release va seguido
  *síncronamente* de `ApplySynthPreset(engine, preset)` en el mismo
  handler. Encolar solo el release dejaría una ventana de ~2 ms donde
  `ApplySynthPreset` corre en el main loop ANTES de que el release llegue
  al hilo de audio — el preset nuevo podría aplicarse y luego el release
  cortar una nota que acababa de empezar con él. Arreglarlo bien exigiría
  encolar el preset-load *junto con* el release como una única operación
  atómica (el mismo patrón que ya se usa para el piano-gate), lo cual es
  más cambio y más riesgo del que justifica cerrar una race de un solo
  booleano (una voz que se corta unos ms tarde, no una voz corrupta).
- **Setters de parámetros escalares** (tipo de filtro, mezcla de FX,
  volumen de pista...). Mucho menos frecuentes que un trigger y tocan un
  único campo escalar en vez de una voz/motor completo con varios campos
  en un orden concreto, así que una lectura a medias es tanto más rara
  como mucho menos audible.
- **`RunStartup808SelfTest`** (autotest de arranque, desactivado por
  defecto vía `RED808_STARTUP_808_SELF_TEST=0` en el `Makefile`): recorre
  todos los pads/instrumentos desde el bucle principal llamando a
  `TriggerPad`/`Trigger`/`NoteOn`/`NoteOff` directamente, sin pasar por
  `AudioCmdPush`. Es deuda técnica conocida y documentada in situ (no un
  descuido): son ~20 sitios de llamada repartidos en una máquina de
  estados de 11 fases que solo se ejecuta si alguien activa ese flag para
  bring-up de hardware — migrarla a ciegas, sin poder verificarla más que
  reflasheando, no compensaba el riesgo de un error de copia/pega. Si se
  activa el flag, cualquier glitch durante el self-test no dice nada sobre
  la cola de audio: ese camino no la usa.

Sí se corrigió `Synth808TriggerByPad` (usada solo por el flag de debug
`kTriggerSynthOnLiveCmd`, `false` en producción, que superpone un golpe de
synth808 sobre el disparo normal de un pad en vivo): antes llamaba a
`synth808.Trigger()` directo desde `ProcessCommand()`; ahora encola un
`AudioCmd` como todo lo demás. A diferencia del self-test de arranque, este
camino comparte hot path con `CMD_TRIGGER_LIVE` en tiempo real, así que si
alguna vez se reactiva el flag para depurar, no debía quedar como el único
disparo en vivo fuera de la cola.

Esta migración se verificó con una compilación limpia (`make`, sin
warnings) y una revisión manual campo a campo contra el código original,
pero **no se ha probado en hardware real**. Antes de darla por buena en
producción: cargar samples por USB mientras suena el secuenciador, un
stress test con `CMD_DIAG_PERF_STRESS`, e inyección de MIDI rápido (redobles)
mientras se reasignan pads desde P4.

## Ring USB (tercer contexto de ejecución)

`DaisyUsbRxCallback` corre en la IRQ del periférico USB (`FS_INTERNAL`) —
un tercer contexto además del bucle principal y la IRQ de audio. El ring
`usbRxRing`/`usbRxHead`/`usbRxTail` que lo conecta con `ProcessDaisyUsb`
(bucle principal) ya usaba variables `volatile`, pero le faltaba `__DMB()`
entre escribir el byte y publicar el índice — igual que la cola de audio,
sin la barrera el consumidor podía en teoría ver el índice actualizado
antes de que el byte fuera visible. Añadido en productor y consumidor.

## Bandera `sampleLoaded[]` y coherencia de caché D en SDRAM

Verificado a fondo (el audit original pedía `SCB_CleanDCache_by_Addr` tras
cargar samples en SDRAM):

- **`sampleLoaded[]` no era `volatile`** (a diferencia de `padLoading[]`, que
  sí lo es) y se publicaba sin barrera tras escribir los datos del sample —
  esto sí era una carencia real. Corregido: ahora es `volatile`, y
  `CMD_SAMPLE_END`, `LoadWavToPad` y el cargador QSPI de arranque hacen
  `__DMB()` justo antes de publicar la bandera, después de escribir todos
  los campos relacionados (incluido `sampleRateHz`, que antes se escribía
  *después* de publicar `sampleLoaded`, dejando una ventana donde el ISR
  podía leer el sample con el pitch equivocado).
- **La coherencia de caché D en SDRAM ya estaba resuelta**, pero no en
  `main.cpp`: `third_party/libDaisy/src/util/sd_diskio.c` (`SD_read`/
  `SD_write`) ya hace `SCB_CleanDCache_by_Addr` +
  `SCB_InvalidateDCache_by_Addr` alrededor de toda lectura/escritura DMA a
  SD (`ENABLE_SD_DMA_CACHE_MAINTENANCE` está fijo a `1`) — es el único sitio
  donde DMA escribe samples directo en SDRAM (el *fast path* de WAV mono
  16-bit en `LoadWavToPad`, vía `f_read`). La carga desde QSPI (lectura
  *memory-mapped* por CPU) y `CMD_SAMPLE_DATA` por USB (`memcpy` por CPU
  desde `rxBuf`) nunca usan DMA hacia SDRAM: mismo núcleo escribe, mismo
  núcleo lee más tarde en `AudioCallback` — coherente por construcción, sin
  necesitar mantenimiento de caché.
