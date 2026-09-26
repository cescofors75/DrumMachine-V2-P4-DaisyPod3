# BANK4 — integración P4

Última revisión: **SEQUENCER_REVIEW** (15/09/2026). Corrige pérdida de clics,
margen del encoder y rotarys en el popup EVOLVE; optimiza filtros LFO y FX de
pistas silenciadas; añade XRUN/protección de carga en DASHBOARD.

- Guía de parámetros y pruebas: [GUIA_SEQUENCER_NONE.md](GUIA_SEQUENCER_NONE.md).
- Firmware P4: `P4/.pio/build/esp32p4-upload/firmware_SEQUENCER_REVIEW.bin`.
- Firmware Daisy: `DaisyPod3/build/DrumMachineV2_DaisyPod3_SEQUENCER_REVIEW.bin`.
- P4 RAM/flash: 93.208 / 1.455.516 bytes. Daisy DTCM/SRAM: 97.012 / 394.836 bytes.
- Compilan ambos. Sin flashear ni validar acústicamente. Véase la auditoría
  para el límite de aislamiento de pistas que comparten motor de síntesis.

Las secciones siguientes conservan el historial de la implementación.


Implementación de `RED808_BANK4_SPEC_v1.docx`, versión 1.0 del 12/09/2026.
La navegación, los bancos y el feedback se gestionan en P4. Esta revisión requiere
actualizar P4 y Daisy: corrige la exclusión de controles USB por los potenciómetros
Pod y protege la actualización de biquads. Se conserva la suma del mezclador de
audio y no se cambian los códigos del protocolo.

## Uso

- Seleccionar pantalla, pad o motor por touch. La barra superpuesta al header
  muestra el banco y los nombres y valores de R1–R4. Es solo informativa:
  no contiene botones de edición ni intercepta el touch.
- Botón 1 de DaisyPod: BACK. Botón 2: mostrar/ocultar la barra. Los rotarys
  siguen funcionando cuando está oculta. Esta visibilidad se conserva durante
  la sesión, incluso al cambiar de pantalla o tema.
- La barra ocupa 72 px superpuestos, con fondo al 80 % de opacidad. Fondo,
  texto, bordes y resaltado siguen el tema visual, incluido GREY.
- Mover el fader para cambiar de banco. Cambiar de banco no envía parámetros
  al motor de audio. Los encabezados de grupos FX también seleccionan banco.
- Girar un encoder edita desde el valor actual. Pulsar un rotary continuo
  activa/desactiva el ajuste fino ([F]); un toggle alterna ON/OFF. Los valores
  enteros/enumerados avanzan por pasos completos y su pulsación no los cambia.
- En FX, girar activa el efecto y ajusta su valor; pulsar el rotary alterna
  ON/OFF y recupera el último valor activo. FILTER conmuta solo el filtro.
- Tocar un pad LIVE o editar su volumen en Mixer selecciona la pista.
  La asignación de motor/sample se realiza desde los controles de la pantalla.
- Los diálogos administrativos desactivan la edición musical por BANK.

Al cargar configuraciones antiguas se reservan los dos botones: BACK y NONE
para Daisy. P4 utiliza el evento del segundo botón para la barra, evitando
que ejecute el antiguo UNDO. Las demás asignaciones y los LED se conservan.

La selección de banco se recuerda por contexto durante la sesión (hasta 128
contextos). No se escribe en flash. Al volver a una pantalla o seleccionar un
banco por touch, un fader inmóvil no sustituye la selección recuperada: hay que
moverlo al menos 30 puntos sobre 1023. A continuación se aplica cuantización
uniforme, histéresis del 4 % (limitada para bancos numerosos) y debounce de 80 ms.

## Bancos

| Contexto | Bancos |
|---|---|
| FX LAB | Cinco filas para los 18 efectos; selección independiente por columna |
| Mixer | CHANNEL: volumen, pan, reverb, delay; STATUS: mute, solo, asignación |
| HOME / LIVE | HOME: BPM, volumen general, patrón (pulsación PLAY/PAUSE), brillo; después PAD, PLAY y SEND |
| Sequencer | VARIACIONES; EVOLVE; PROBABILIDAD; GLITCH; ORDEN DE RITMOS |
| PadSound selección | Engine; preset (pitch si sampler); carácter; volumen |
| PadSound MANUAL | BASIC; AMP ENV si sampler; bancos del motor melódico; FILTER; FX SEND; PERF |
| Pad FX | FILTER FAMILY: tipo, cutoff, resonancia, drive; COLOR / SEND: bits, reverb, delay, chorus |
| Piano | PERFORMANCE: octava, velocity, motor, volumen; bancos del motor |
| TB-303 Params | FILTER; AMP; CHARACTER; PERFORMANCE |
| Wavetable Params | OSC / ENV; FILTER / LFO |
| SH-101 Params | OSC; FILTER; AMP ENV; MOD; CHARACTER |
| FM2 Params | CARRIER ENV; MODULATOR ENV; FM CORE; OUTPUT / PERFORMANCE |

Los identificadores, límites y valores por defecto de los sintetizadores
proceden de `shared/synth_params.h`. Los bancos están en
`shared/synth_banks.h`; las pruebas verifican que cada parámetro aparece una
sola vez y que no falta ninguno. Los editores conservan sus controles táctiles;
la barra no reserva espacio ni desplaza sus zonas táctiles.

FX LAB ofrece VIEW 4, VIEW 8 y VIEW 12: una, dos o tres filas de cuatro.
Cada fila incluye su nombre de grupo; las tarjetas asignadas muestran R1–R4.
Se puede seleccionar una fila completa tocando su encabezado. Cambiar BANK muestra la
página correspondiente y marca el grupo activo. Los 18 efectos siguen
accesibles; el último grupo mantiene dos posiciones libres. La cuadrícula
utiliza toda la altura disponible y conserva ARC, LED y BAR.

### Adaptaciones al hardware y motor existentes

El repositorio tiene 18 cards FX. Los cinco bancos permiten acceder a todos
con rotarys; la última fila tiene dos slots vacíos.

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
- Los botones de DaisyPod usan contadores atómicos independientes del último
  estado USB: una pantalla ocupada no pierde las pulsaciones ya recibidas.
  Solo GET_STATE entrega eventos; SET_CONFIG no los duplica al repetir su eco.
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

Compilación validada del 15/09/2026 (`esp32p4-upload`), con mejoras de retrigger en
sintetizadores, rotarys unitarios y bancos ampliados del sequencer:

| Recurso | Uso | Capacidad del entorno |
|---|---:|---:|
| RAM estática | 93.224 bytes (28,4 %) | 327.680 bytes |
| Programa flash | 1.454.384 bytes (22,2 %) | 6.553.600 bytes |

Binario actualizado: `P4/.pio/build/esp32p4-upload/firmware_SYNTH_BANKS.bin`.
También se actualiza `firmware_BANK4.bin`.
Daisy: `DaisyPod3/build/DrumMachineV2_DaisyPod3_SYNTH_BANKS.bin`.
Daisy ocupa 97.012 bytes DTCM (74,01 %), 392.252 bytes SRAM (79,80 %) y
53.966.732 bytes SDRAM (80,42 %). Son ocupaciones, no medidas de carga de audio.

### Correcciones HOME, filtros y Mixer

- Los potenciómetros asignados en Daisy ya no bloquean las órdenes USB/MIDI:
  manda el último control utilizado. El tempo P4 admite el mismo rango 40–300.
- Los coeficientes biquad se calculan fuera de la sección crítica y se publican
  juntos; el estado no finito se reinicia. La prueba portable de barridos de los
  diez tipos, parámetros inválidos y recuperación pasa. El bloqueo del patrón 20
  no se ha reproducido en hardware; esta corrección elimina un riesgo identificado,
  sin confirmar todavía la causa exacta del incidente.
- Mixer elimina los sliders globales MAIN/BPM. Los 16 canales ganan 78 píxeles
  de altura y el ancho del fader pasa de 14 a 24 píxeles.

## Validación

- Botones Pod: frames demorados, ocultar/mostrar, dos pulsaciones antes de
  repintar, ausencia de repetición y 100.000 eventos concurrentes: correctos
  en la prueba portable `pod_button_events_regression.cpp`.
- Compilación de P4 con BANK4, vistas 4/8/12, barra superior y XTRA: correcta.
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
de las pantallas, BACK y mostrar/ocultar la barra sin mover el contenido, y alternar patrones con PLAY y detenido mientras se editan
parámetros. No se ha flasheado ningún dispositivo en esta implementación.

## Archivos principales

### HOME

El primer banco visible al iniciar HOME/LIVE es HOME: R1 BPM, R2 volumen general,
R3 selector de patrón y PLAY/PAUSE al pulsar, R4 brillo de pantalla (0–100 %).
La fila permanece arriba, sobre el header. El brillo se aplica inmediatamente
con el controlador PWM existente, en pasos del 1 %, y muestra su valor actual.
No modifica el tema. Se conserva durante la sesión; no se guarda entre reinicios.
El selector acepta como máximo un patrón por evento cada 100 ms, sin saltos por
ticks acumulados. Tras 450 ms sin giro, en PLAY queda pendiente hasta el siguiente
compás; detenido se aplica directamente. Los patrones se cargan
en la tarea de control existente, fuera de LVGL. Los siguientes bancos conservan
la edición PAD/PLAY/SEND y el fader permite volver a HOME.

La fila se monta al construir HOME, con sus cuatro etiquetas y valores iniciales.
Es hija de la página (o del modal musical activo) y se mantiene delante de su
contenido. No mueve, redimensiona ni reagrupa el contenido: mostrarla u ocultarla
conserva las posiciones y coordenadas táctiles originales de la pantalla.

### XTRA: cuatro directorios independientes

| Rotary / pad | Carpeta en la SD de Daisy |
|---|---|
| R1 / XTRA 1 | `/data/xtra/Drums` |
| R2 / XTRA 2 | `/data/xtra/Vocals` |
| R3 / XTRA 3 | `/data/xtra/FX` |
| R4 / XTRA 4 | `/data/xtra/Music` |

Al entrar en XTRA se leen las cuatro listas, sin sustituir sonidos. Cada giro
selecciona el WAV anterior/siguiente de su carpeta, con vuelta al principio/final.
Se filtra `.wav` sin distinguir mayúsculas y se ordenan los nombres alfabéticamente. CARPETAS relee
las listas. La carga espera 350 ms desde el último giro y a soltar el pad; hay
un mínimo de 700 ms entre solicitudes de carga. Una lista vacía no descarga
el sonido anterior. Los cuatro destinos son los pads de sampler 16–19.

Se reutiliza el protocolo existente: **máximo 20 nombres por carpeta y 31
caracteres por nombre**, sin paginación. Esto no recorre una biblioteca mayor
que ese límite. La SD debe contener las rutas indicadas; no se crean carpetas
ni se copian archivos. La carga actualiza la UI de forma optimista después de
enviar el comando; no constituye confirmación de carga correcta de Daisy.

La prueba `xtra_directories_regression.cpp` ejecuta el controlador de producción
con SD simulada: rutas, respuesta correlacionada por secuencia, filtro/orden,
carpetas vacías, giros acumulados, límites temporales y pad mantenido.

### Sequencer y Pad Sound — 15/09/2026

Sequencer usa R1 para BD, R2 para SD/CP, R3 para CH/OH y R4 para los demás
instrumentos. Las siete posiciones son ORIGINAL, DESPLAZAR, ESPEJO, HALF TIME,
GHOST, RATCHET y SPARSE. Se trabaja desde una base por grupo: volver a ORIGINAL
restaura esa base. Una edición externa del grupo establece una nueva base.
Los cambios esperan 250 ms sin giro y se procesan en la tarea de control;
las solicitudes de otro patrón o página se descartan. Se modifica el compás
visible y se conservan la página y los demás compases. Se conserva UNDO y el patrón
queda marcado como modificado. No hay cambios en el DSP de Daisy.

El botón FX de una fila abre esa pista. Dos familias de cuatro controles
coinciden con los bancos del fader: TYPE/CUTOFF/RESONANCE/DRIVE y
CRUSH/REVERB/DELAY/CHORUS. Tocar el título de familia o uno de sus
controles también selecciona su banco. Los filtros son tipos alternativos del filtro
existente de pista, no cuatro filtros DSP simultáneos.

Pad Sound muestra una cuadrícula táctil 4×4, cuatro lecturas grandes y acciones
BACK, MANUAL, SAMPLER, FX, PREVIEW y ASIGNAR. Engine, preset, carácter y volumen
son selecciones pendientes; PREVIEW aplica y escucha, ASIGNAR aplica y cierra.
MANUAL aplica la preparación y permite edición en vivo con bancos de cuatro.
El carácter modifica filtro/drive de la pista: neutro, oscuro, brillante, suave
o agresivo. El sampler usa pitch en R2 porque no tiene presets de síntesis.

MANUAL ofrece parámetros de pista para todos los sonidos y los bancos nativos
existentes de TB-303, Wavetable, SH-101 y FM. No añade editores nativos de
osciladores 808/909/505 ni nuevas instancias DSP independientes por pad.
Los parámetros nativos melódicos siguen compartidos por engine; se indica en
la pantalla. No se promete aislamiento de patches entre pads del mismo engine.

Prueba `sequence_groups_regression.cpp`: ejecuta el adaptador de producción
con secuenciador simulado; verifica aislamiento entre grupos, restauración,
dinámica, edición externa, cambio de patrón y entradas inválidas.
La disposición y el tacto deben comprobarse en la pantalla física.

### Código

- `shared/bank_controller.h`: navegación, límites, takeover y edición genérica.
- `shared/synth_banks.h`: agrupación semántica de los motores.
- `P4/src/ui/ui_bank_contexts.inc`: adaptadores y barra LVGL.
- `P4/src/ui/bank_input.h`: eventos físicos y feedback.
- `P4/src/ui/bank_state.*`: reflejo de parámetros y estado de interpretación.
- `P4/src/drivers/i2c_rotaries.cpp`: lectura relativa y aros SEN0502.
- `tools/tests/bank_*_regression.cpp`: comprobaciones portables.


### FX por columnas y bancos ampliados de Sequencer

Tocar una tarjeta FX (incluidos sus controles) asigna ese efecto al rotary de
su columna, sin alterar los otros tres. R1 puede apuntar a la fila 1 y R2 a la
fila 3. Las tarjetas asignadas muestran R1–R4. Cambiar VIEW o página conserva
la mezcla. El fader selecciona una fila completa; el título de grupo también
lo hace. La selección no envía cambios de audio y se descartan giros pendientes
del mapa anterior. Al regresar desde otra pantalla se conserva la mezcla hasta
mover el fader. Los botones y arcos conservan sus funciones táctiles existentes.

Sequencer tiene cinco bancos:
- VARIACIONES: 16 opciones por grupo, incluidos Euclid 3/5/7, desplazamientos y fill final.
- EVOLVE: R1 OFF/8 modos; R2 cantidad 0–100; R3 intervalo 1/2/3/4/6/8/12/16
  compases; R4 grupo de notas (todos, bombo, caja/clap, hats, percusión).
  Pulsar R1 desactiva, R2 pone cantidad cero, R3 vuelve a cuatro compases y
  R4 vuelve a TODOS. Al apagar se restaura la humanización propia anterior.

Las pulsaciones de VAR/GLITCH/ORDEN restauran la base capturada del grupo;
PROBABILIDAD vuelve al 100 %. Los botones del popup FX de pista desactivan
el filtro o neutralizan el parámetro. Véase
[auditoría de ratchet](AUDITORIA_RATCHET_SEQUENCER.md) para los cambios de
audio, las pruebas realizadas y los límites pendientes de validación en placa.
- PROBABILIDAD: 0–100 % para los pasos activos de cada grupo.
- GLITCH: 12 opciones, incluidos x2/x3/x4, stutter, offbeats y drop 25 %.
- ORDEN DE RITMOS: R1 filas 1–4, R2 5–8, R3 9–12, R4 13–16. Cada giro selecciona una de 12 permutaciones de
  pasos, velocidad, probabilidad y ratchet dentro de ese bloque. Conserva
  los engines, notas, locks y demás ajustes propios de las pistas.

Cambiar de banco no aplica transformaciones. Probabilidad, glitch y orden usan
la misma espera de 250 ms y sincronización diferida de patrones. Sus bases se
separan para que una operación posterior conserve los cambios de otro banco.

Pruebas nuevas: selección mixta FX, sustitución de fila completa, columnas
fijas, slots vacíos, probabilidad cero, repetición x4, rotación/restauración de
ritmos y aislamiento entre grupos: correctas. La repetición de la prueba BANK
general compiló, pero Windows bloqueó su ejecución mediante Control de
aplicaciones. No se declara la suite completa aprobada ni verificación física.
