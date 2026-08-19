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

Para las rutas de mayor tráfico — disparo de pad (`TriggerPad`) y disparo/
note-on de los sintetizadores 808/909/505/303/WTOSC/SH101/FM2OP — el bucle
principal ya no llama a esas funciones directamente. En su lugar encola un
`AudioCmd` en un ring SPSC (`AudioCmdPush`, ver `main.cpp` justo después de
`TriggerPad`) y `AudioCmdDrainAndApply()`, invocado al inicio de cada bloque de
`AudioCallback`, ejecuta la llamada real ya en el hilo de audio. Solo hay un
escritor de `voices[]`/los motores para esas rutas.

Quedan fuera de esta cola (documentado en el comentario junto a la
definición): los motores PHYS/NOISE, los helpers de "release"/NoteOff
(`ReleaseAllSynthEngines`, `CMD_SYNTH_ACTIVE`, etc.) y los setters de
parámetros escalares (tipo de filtro, mezcla de FX, volumen de pista...).
Son de menor prioridad: mucho menos frecuentes que un trigger y tocan un
único campo escalar en vez de una voz completa de varios campos, así que una
lectura a medias es tanto más rara como mucho menos audible.

Esta migración se verificó con una compilación limpia (`make`, sin
warnings) y una revisión manual campo a campo contra el código original,
pero **no se ha probado en hardware real**. Antes de darla por buena en
producción: cargar samples por USB mientras suena el secuenciador, un
stress test con `CMD_DIAG_PERF_STRESS`, e inyección de MIDI rápido (redobles)
mientras se reasignan pads desde P4.
