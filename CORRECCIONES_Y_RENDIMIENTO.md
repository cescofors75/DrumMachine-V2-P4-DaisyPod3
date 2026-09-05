# Correcciones y rendimiento — 2026-09-05

Implementadas las correcciones de los diez hallazgos de la auditoría, más el
disparo duplicado de pads encontrado durante la revisión. Ambos firmwares
compilan y generan binarios. No se han flasheado las placas ni medido CPU,
latencia o FPS en hardware.

## Correcciones

| Problema | Cambio |
| --- | --- |
| Pistas 808/909/505 mezcladas por la primera pista | Los kits entregan salidas por instrumento; se acumulan por pista y se procesa su volumen, mute, solo, pan y FX una vez por muestra. Se conserva la señal agregada del kit. |
| Editar velocidad destruye división, probabilidad y ratchet | Un serializador conserva todos los atributos. Las ediciones durante una carga solicitan resincronización. |
| Locks heredados de otro patrón | La carga completa sustituye también notas y locks desactivados. |
| Guardado destruye la versión anterior | Dos copias alternas, generación, checksum, reapertura y verificación. La copia anterior y el destino en RAM sobreviven a una escritura fallida. Lectura compatible con V1. |
| Seleccionar una carga parcial | BEGIN/TRACK/COMMIT con token, orden de pistas y checksum. Daisy prepara un buffer separado, lo intercambia en el límite del bloque y confirma después. P4 conserva solicitudes fallidas para reintentar. |
| Swing pierde ratchets | Planificación por parejas y duración efectiva por pista; humanize se limita al espacio disponible. |
| Locks ignorados en síntesis y persistentes en samples | Valores efectivos separados del mezclador base, aplicados a ambos caminos y restablecidos en el siguiente paso o silencio. |
| SEQ VOL no afecta a síntesis secuenciada | Se conserva el origen LIVE/SEQ por instrumento de batería y por motor melódico. |
| Notas sostenidas al finalizar SONG | El final natural libera las notas y cancela los disparos pendientes. |
| Swing por pista sin efecto | Se usa el valor de cada pista al planificar su offbeat; cero sigue significando heredar el swing global. |
| Doble disparo táctil | Eliminado el disparo previo a velocidad 127: se envía una sola nota con la velocidad real. |

Los motores melódicos monofónicos mantienen su arquitectura de última nota;
el propietario de la nota sustituye la selección de la primera pista. No se
han creado dieciséis instancias independientes de cada sintetizador.

## UI y respuesta de los controles

- Guardado de patrones en una tarea de baja prioridad, fuera del callback de
  LVGL. El editor actual permanece abierto y no se marcan como guardadas las
  ediciones posteriores al snapshot.
- Mensajes de sincronización pendiente, confirmada o fallida. El modal espera
  el ACK de carga cuando hay conexión.
- Los pads y la cola de controles se atienden entre pistas y durante la espera
  del ACK. La importación de canciones delega la transferencia a la tarea de
  control; no crea un segundo consumidor del parser USB.
- PLAY de una canción pendiente se difiere hasta preparar sus escenas; STOP
  cancela esa intención.
- El salvapantallas respeta PLAY y la actividad de MIDI, knobs, encoder,
  botones, rotaries y fader.

## Rendimiento: cambios cuantificables

Los siguientes recuentos son cálculos del formato de transferencia, no un
benchmark de velocidad del cable. Incluyen cabeceras P4→Daisy y excluyen
telemetría, perfiles y el ACK de retorno de 12 bytes.

| Métrica | Antes | Ahora | Resultado |
| --- | ---: | ---: | --- |
| Paquetes de patrón denso: 256 pasos con notas y locks | 529 | 18 | 96,6% menos paquetes |
| Bytes del mismo patrón denso | 10.441 | 4.314 | 58,7% menos bytes |
| Patrón sin notas ni locks: paquetes | 17 | 18 | Se añade la transacción verificable |
| Patrón sin notas ni locks: bytes | 1.225 | 4.314 | El formato completo cuesta más en patrones simples |
| Disparos USB por toque de pad conectado | 2 | 1 | Sin retrigger duplicado ni velocidad 127 espuria |
| Buffer adicional de patrón en Daisy | 0 | 20.480 bytes | Publicación por intercambio de punteros; no copia un patrón completo dentro del callback |
| Ratchet solicitado 4, 120 BPM, swing 100, paso corto | 2 golpes | 4 golpes | Sin pérdidas por el límite del paso |

La mezcla correcta procesa FX por pista en lugar de compartirlos entre todo
un kit: puede consumir **más CPU** en patrones con muchas pistas de síntesis.
No se afirma una reducción de CPU. Se conserva el perfilador DSP existente
para medir esa carga en placa.

## Memoria y archivos generados

Resultados de compilación del código corregido:

| Firmware / región | Uso | Capacidad | Porcentaje |
| --- | ---: | ---: | ---: |
| P4 RAM estática | 95.520 B | 327.680 B | 29,2% |
| P4 aplicación en flash | 1.419.938 B | 6.553.600 B | 21,7% |
| Daisy DTCMRAM | 97.140 B | 131.072 B | 74,11% |
| Daisy SRAM | 396.296 B | 491.520 B | 80,63% |
| Daisy SDRAM estática | 53.966.732 B | 67.108.864 B | 80,42% |

Estos valores no incluyen el máximo de pila/heap observado durante ejecución.
El bloque de audio configurado es 128 muestras a 48 kHz: **2,667 ms de
presupuesto por callback**, no una medición de latencia extremo a extremo.

Binarios:

- `DaisyPod3/build/DrumMachineV2_DaisyPod3.bin`: 396.296 bytes.
- `P4/.pio/build/esp32p4-upload/firmware.bin`: 1.481.520 bytes.

## Validación

- Daisy: compilación, enlace y generación de BIN/HEX correctos con ARM GCC y
  `build.ps1 -MakeBin C:\msys64\usr\bin`.
- P4: entorno `esp32p4-upload`, compilación, enlace y BIN correctos. Se ejecutó
  PlatformIO mediante `python -m platformio`. El script de construcción usa
  el módulo Python de esptool en Windows para evitar depender del launcher
  `.exe` bloqueado por Application Control. No se desactivó esa protección.
- Pruebas C++ con aserciones habilitadas: **843.552 combinaciones** de timing,
  empaquetado de división/ratchet, hash, cada pista ausente en una transferencia,
  token antiguo, checksum incorrecto, salida individual de los tres kits y
  reproducción PCM 909/505.
- Almacenamiento: **177 puntos de interrupción de escritura** con payload
  reducido, fallo de apertura, copia más reciente corrupta, recuperación tras
  reinicio simulado y migración V1. Se ejecuta el código real del almacén contra
  un filesystem en memoria; no simula corrupción de toda la partición SPIFFS.
- `git diff --check` sin errores de whitespace.

Repetir las pruebas con `tools/run_audit_tests.py`; instrucciones en
`tools/tests/README.md`.

## Comprobación pendiente en las placas

Actualizar **ambos firmwares a protocolo 2.4**. Verificar una sesión con las
16 pistas de síntesis y FX, mute/solo individual, SEQ VOL a cero, ratchet con
swing/humanize máximo, último paso con slide, desconexión durante carga,
guardado con poco espacio y actuación MIDI prolongada.

Registrar CPU media/pico, underruns, máximo de pila, FPS, latencia táctil→audio
y latencia de cambio de patrón. La compilación y las pruebas de host no
demuestran que el callback cumpla su plazo con la carga máxima real.
