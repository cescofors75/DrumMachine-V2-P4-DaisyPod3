# Protocolo USB P4 ↔ DaisyPod3

El enlace usa USB CDC Full Speed. P4 enumera como host y DaisyPod3 como dispositivo (`VID 0x0483`, `PID 0x5740`).

Cada comando usa una cabecera binaria empaquetada de 8 bytes:

| Campo | Tamaño | Descripción |
|---|---:|---|
| magic | 1 | `0xA5` comando, `0x5A` respuesta |
| command | 1 | código RED808 |
| length | 2 | tamaño del payload |
| sequence | 2 | contador del emisor |
| checksum | 2 | CRC-16/Modbus (`0xA001`, inicio `0xFFFF`) del payload |

El payload máximo es de 528 bytes. La carga de audio usa `SAMPLE_BEGIN`, fragmentos `SAMPLE_DATA` de hasta 512 bytes PCM y `SAMPLE_END`.

El P4 envía `PING` cada segundo y consulta posición cada 40 ms. Una respuesta válida establece el estado `DAISY`; tras tres segundos sin respuesta el motor pasa a desconectado y se resincroniza automáticamente al volver.

Desde la versión de protocolo `2.1`, `PING` devuelve además una respuesta de salud
de 20 bytes. Conserva los primeros 8 bytes históricos (eco y uptime), de modo que
las herramientas y firmwares anteriores siguen siendo compatibles, y añade versión,
capacidades, descartes RX y errores de protocolo. P4 mide el tiempo de ida y vuelta
mediante la secuencia exacta de cada consulta y muestra esa latencia en HOME.

Solo puede existir una consulta de telemetría pendiente. Si no llega su secuencia en
400 ms se libera sin bloquear audio ni controles; las respuestas atrasadas se
contabilizan y descartan. Asimismo, la telemetría cede prioridad cuando la cola USB
de salida supera ocho paquetes, evitando que los sondeos retrasen notas, transporte,
edición de pasos o transferencia de samples.

La reproducción de canciones usa `0xF2 SONG_UPLOAD` y `0xF3 SONG_CONTROL`.
`SONG_UPLOAD` transporta hasta 128 pares `{residentPattern, repeats}` en un
único payload (257 bytes como máximo); cada repetición admite `1..255`. Daisy
valida el número de pares realmente presentes antes de activar la cadena y
detiene el secuenciador al completar la última sección.

Las consultas que esperan respuesta se escalonan para que Daisy no tenga que
encolar más de una respuesta CDC a la vez:

- `0xE0 GET_STATUS`: SD, máscara de samples y engines reales de las 16 pistas.
- `0xE6 POD_GET_STATE`: configuración, posiciones físicas, LED y valores canónicos.
- `0xE7 POD_SET_CONFIG`: nueva asignación; Daisy valida y devuelve su estado canónico.

`PodConfigPayload` usa la versión 7 e incluye las seis asignaciones físicas de
DaisyPod, cuatro asignaciones para los SEN0502 y una para el Mini Fader conectado
directamente al ADC GPIO20 de P4. El antiguo campo del selector se conserva en
`NONE` únicamente para mantener el payload de configuración en 21 bytes.

Desde el protocolo `2.3`, `PodStatePayload` ocupa 66 bytes y devuelve el estado
canónico de los 12 controles FX: valores y bypass de delay, reverb, flanger,
phaser y wavefolder, además de crush, tipo/cutoff/resonancia del filtro,
distorsión, bit depth y sample rate. Daisy incrementa `revision` al aceptar un
cambio físico o un comando P4; P4 sólo aplica revisiones nuevas, evitando que
una respuesta antigua pise un ajuste recién enviado.

El estado físico de DaisyPod es autoritativo. Si un knob está asignado a
MASTER, SEQ, LIVE o TEMPO, Daisy rechaza para ese parámetro los valores
digitales incompatibles enviados por P4. Los rotaries I2C y el fader de P4
tienen la segunda prioridad y bloquean el control táctil equivalente
mientras estén detectados y asignados. Cada SEN0502 publica su botón por separado,
con debounce, y al estar asignado a un FX su pulsación conmuta el bypass de ese FX.
P4 se corrige con la siguiente revisión nueva de `POD_GET_STATE`.
