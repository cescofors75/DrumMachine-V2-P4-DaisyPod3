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

Las consultas que esperan respuesta se escalonan para que Daisy no tenga que
encolar más de una respuesta CDC a la vez:

- `0xE0 GET_STATUS`: SD, máscara de samples y engines reales de las 16 pistas.
- `0xE6 POD_GET_STATE`: configuración, posiciones físicas, LED y valores canónicos.
- `0xE7 POD_SET_CONFIG`: nueva asignación; Daisy valida y devuelve su estado canónico.

El estado físico de DaisyPod es autoritativo. Si un knob está asignado a
MASTER, SEQ, LIVE o TEMPO, Daisy rechaza para ese parámetro los valores
digitales incompatibles enviados por P4 y P4 se corrige con la siguiente
respuesta `POD_GET_STATE`.
