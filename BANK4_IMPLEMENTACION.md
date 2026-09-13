# BANK4 — integración P4

Implementación de `RED808_BANK4_SPEC_v1.docx`, versión 1.0 del 12/09/2026.
La navegación, los bancos y el feedback se gestionan en P4. No se han modificado
el firmware Daisy ni los códigos del protocolo; se conserva su mezcla de audio
actual. Este cambio requiere actualizar P4, manteniendo la Daisy compatible que
ya estaba funcionando.

## Uso

- Seleccionar pantalla, pad o motor por touch. La barra inferior muestra el
  contexto, nombre de banco, N/M y los cuatro parámetros R1–R4.
- Mover el fader para cambiar de banco. `BANK <` y `BANK >` hacen lo mismo por
  touch. Cambiar de banco no envía parámetros al motor de audio.
- Girar un encoder para editar desde el valor actual. Los botones `−`/`+`
  permiten editar sin hardware. Pulsar un parámetro continuo activa/desactiva
  el ajuste fino (indicador `[F]`); un toggle alterna ON/OFF. Los parámetros
  enteros/enumerados avanzan por pasos completos y su pulsación no los cambia.
- `P−`/`P+` seleccionan el pad en los contextos de pista. Tocar un pad LIVE o
  editar su volumen en Mixer también lo selecciona.
- Engine/Sample abre la asignación táctil. Ningún giro ni pulsación física
  confirma una asignación. Los slots no disponibles muestran `--`.
- Los diálogos administrativos desactivan la edición musical por BANK.

La selección de banco se recuerda por contexto durante la sesión (hasta 128
contextos). No se escribe en flash. Al volver a una pantalla o seleccionar un
banco por touch, un fader inmóvil no sustituye la selección recuperada: hay que
moverlo al menos 30 puntos sobre 1023. A continuación se aplica cuantización
uniforme, histéresis del 4 % (limitada para bancos numerosos) y debounce de 80 ms.

## Bancos

| Contexto | Bancos |
|---|---|
| FX LAB | MOD / SPACE; CRUSH / FILTER; COLOR: exactamente los 12 efectos de referencia |
| Mixer | CHANNEL: volumen, pan, reverb, delay; STATUS: mute, solo, asignación |
| LIVE | PAD; PLAY; SEND, relativos al último pad seleccionado |
| PadSound sample | BASIC; AMP ENV; FILTER; FX SEND; PERF |
| PadSound sintetizador | BASIC; bancos del motor; FILTER; FX SEND; PERF |
| Pad FX | FILTER: cutoff, resonancia, drive, bits; SEND |
| Piano | PERFORMANCE: octava, velocity, motor, volumen; bancos del motor |
| TB-303 Params | FILTER; AMP; CHARACTER; PERFORMANCE |
| Wavetable Params | OSC / ENV; FILTER / LFO |
| SH-101 Params | OSC; FILTER; AMP ENV; MOD; CHARACTER |
| FM2 Params | CARRIER ENV; MODULATOR ENV; FM CORE; OUTPUT / PERFORMANCE |

Los identificadores, límites y valores por defecto de los sintetizadores
proceden de `shared/synth_params.h`. Los bancos están en
`shared/synth_banks.h`; las pruebas verifican que cada parámetro aparece una
sola vez y que no falta ninguno. Los editores conservan sus controles táctiles;
las pantallas extensas usan desplazamiento vertical por encima de la barra.

FX LAB ofrece VIEW 4, VIEW 8 y VIEW 12: una, dos o tres filas de cuatro.
Cada fila incluye su nombre de grupo; los primeros tres grupos muestran R1–R4
y permiten seleccionar BANK tocando el encabezado. Cambiar BANK muestra la
página correspondiente y marca el grupo activo. Los 18 efectos siguen
accesibles; el último grupo mantiene dos posiciones libres. La cuadrícula
reserva el espacio inferior de BANK y conserva ARC, LED y BAR.

### Adaptaciones al hardware y motor existentes

El repositorio tiene 18 cards FX, frente a las 12 del documento. Los seis efectos
adicionales conservan su acceso táctil; los tres bancos de referencia no cambian.

El sampler expone fade-in/fade-out como ataque/release (0–255 ms), sin inventar
decay/sustain. Pitch por pad se ofrece al sampler; los sintetizadores usan sus
parámetros disponibles en los bancos de motor. Env Amount, Dry/Wet por pista y
Output/Route quedan vacíos cuando no existe un control equivalente. Gate se
ofrece al sampler y motores melódicos; `AUTO` conserva la duración previa.

Velocity, accent, probability, repeat y gate son ajustes de interpretación LIVE
en P4; no sobrescriben los pasos guardados del secuenciador ni transforman notas
MIDI que Daisy recibe directamente. Sus valores iniciales conservan el
comportamiento anterior. Los ajustes nuevos no se guardan entre reinicios.

Los cuatro SEN0502 usan el contador de 0–1023 para el aro: el hardware no ofrece
RGB independiente. Los colores de pista/FX se reflejan en pantalla. El feedback
de un slot editable usa 51–972, dejando margen para detectar ambos sentidos
incluso en los extremos; un slot vacío o protegido apaga el aro. Se lee el giro
antes de escribir feedback y la escritura confirmada actualiza la referencia,
para que no se interprete como un giro del usuario. Véase la
[biblioteca del fabricante](https://github.com/DFRobot/DFRobot_VisualRotaryEncoder).

Se mantiene la conexión de fader ya implementada: ADC GPIO20, 12 bits,
normalizado a 0–1023, lectura cada 10 ms; barra de 14 LED en GPIO45. La detección
es la del driver existente: una entrada ADC no puede confirmar por sí sola que
el fader está conectado. La histéresis reduce jitter, pero la prueba de
desconexión física sigue siendo necesaria.

## Estado y rendimiento

- Los giros y pulsaciones cruzan de la tarea I²C a LVGL mediante buzones atómicos
  con generación de contexto; se descartan eventos del banco anterior.
- El controlador no asigna memoria dinámica al girar o cambiar de banco.
- Los setters envían cambios efectivos; navegar, pintar y actualizar aros no
  generan comandos USB de parámetros. Las actualizaciones de estado no se
  reenvían al audio.
- Los textos y bordes solo se repintan cuando cambian. El resaltado dura 500 ms.
  El contador del aro solo se escribe si difiere; la barra del fader evita
  repetir una trama LED idéntica.
- Touch, parámetros enviados desde P4 y presets actualizan la misma vista.
  La telemetría existente sigue siendo la fuente de confirmación disponible.
  Los parámetros que Daisy no devuelve (por ejemplo, determinados cambios
  directos de pan/send por MIDI conectado a Daisy) no pueden reconstruirse en
  P4; el reflejo local de esos parámetros es optimista, no una lectura del DSP.

No se han medido CPU de audio, latencia USB, carga I²C ni respuesta táctil en
placa durante esta implementación. Los porcentajes del compilador representan
ocupación estática, no rendimiento en tiempo real ni el máximo de heap/PSRAM.

Compilación final del 13/09/2026 (`esp32p4-upload`):

| Recurso | Uso | Capacidad del entorno |
|---|---:|---:|
| RAM estática | 75.748 bytes (23,1 %) | 327.680 bytes |
| Programa flash | 1.442.422 bytes (22,0 %) | 6.553.600 bytes |

Binario preparado: `P4/.pio/build/esp32p4-upload/firmware_BANK4.bin`.

## Validación

- Compilación del entorno `esp32p4-upload`: correcta.
- BANK: contextos, memoria, 1–10 bancos, jitter, debounce, navegación sin
  escrituras, valores relativos, fine, log, enums, toggles, acciones protegidas,
  pickup futuro y descarte de eventos viejos: correctos.
- Estado BANK: paquetes truncados, floats inválidos, pan/pitch con signo,
  reset FX, todos los pasos de send/bitcrush y dinámica LIVE por defecto:
  correctos.
- Regresiones de audio, transferencia y ciclos de voces: correctas.
- La prueba de almacenamiento compila, pero Windows bloqueó su ejecutable
  mediante Control de aplicaciones (error 4551). No se cambió esa política;
  la suite completa no se puede declarar aprobada en este equipo.

Pendiente de aceptación en placa: verificar los cuatro sentidos/aros al cambiar
contexto, fader en fronteras, touch sin dispositivos, legibilidad y desplazamiento
de las pantallas, y alternar patrones con PLAY y detenido mientras se editan
parámetros. No se ha flasheado ningún dispositivo en esta implementación.

## Archivos principales

- `shared/bank_controller.h`: navegación, límites, takeover y edición genérica.
- `shared/synth_banks.h`: agrupación semántica de los motores.
- `P4/src/ui/ui_bank_contexts.inc`: adaptadores y barra LVGL.
- `P4/src/ui/bank_input.h`: eventos físicos y feedback.
- `P4/src/ui/bank_state.*`: reflejo de parámetros y estado de interpretación.
- `P4/src/drivers/i2c_rotaries.cpp`: lectura relativa y aros SEN0502.
- `tools/tests/bank_*_regression.cpp`: comprobaciones portables.
