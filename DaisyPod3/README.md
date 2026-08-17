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
