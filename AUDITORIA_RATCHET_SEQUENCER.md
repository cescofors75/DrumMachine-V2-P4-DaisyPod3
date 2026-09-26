# Auditoría de ratchet, variaciones y EVOLVE — 15/09/2026

## Revisión del sequencer: sensibilidad, EVOLVE y carga con muchas pistas

Firmwares `SEQUENCER_REVIEW`, posteriores a `SYNTH_BANKS`.

- Eliminado el descarte de giros durante 65 ms. Los selectores consumen un
  clic por actualización y conservan el resto; porcentajes consumen el lote.
  Pulsar reset descarta el giro que quedaba pendiente.
- El contador/aro SEN0502 comparte registro. Se reserva margen 51..972 para
  que un giro completo pueda leerse en ambos sentidos cerca de 0/100 %.
  La ganancia sigue en 51, según la especificación del fabricante:
  https://wiki.dfrobot.com/sen0502/docs/21517
- EVOLVE ya admite rotarys con su popup abierto. Sus peticiones conservan el
  valor visible hasta aplicarse; cambios de cantidad, intervalo, activación y
  «APLICAR AHORA» del popup se difieren fuera de LVGL.
- El array de LFO estaba sin inicializar si no había ningún LFO con profundidad
  positiva, pero ciertas rutas consultaban el valor con LFO activo/profundidad
  cero. Se inicializa a cero, evitando valores indeterminados en ese caso.
- Los coeficientes del filtro modulado se actualizan cada ocho muestras,
  compartiendo el cálculo entre sampler y sintetizadores. Antes se recalculaban
  cada muestra en sampler y no se aplicaba esa modulación a la ruta synth.
  Reduce ocho veces la frecuencia de ese cálculo concreto, no toda la CPU.
- Mute/solo evitan los FX de pistas inaudibles tras avanzar el estado de la voz.
  Los estados de los FX silenciados quedan pausados; al recuperar la pista hay
  que comprobar también sus colas. No se han cambiado ganancias ni sumas master.
- Status añade XRUN (agotamiento del presupuesto de audio) y protección FX,
  visibles en DASHBOARD/CPU-MEMORIA. No se necesita otro canal USB ni logs de
  audio. Un XRUN creciente confirma cortes del bloque por presupuesto, no la
  causa exacta. Cero XRUN no descarta saturación, discontinuidades u otros fallos.

Revisión de rutas: los mutes individuales y por grupo y el solo exclusivo
actualizan máscaras; solo desmutea la pista elegida y mute elimina su solo.
CLEAR FX también neutraliza chorus en el espejo de estado. Los locks de paso,
variaciones y páginas se mantienen. Los diálogos de guardar/cargar siguen
bloqueando los rotarys de fondo por diseño.

**Límite pendiente importante:** synths compartidos se mezclan por motor y se
rutan al primer track asignado. Repetir motor en varias filas no garantiza
FX/solo independientes por fila. Corregirlo necesita separar las salidas por
instrumento/voz y comprobar la mezcla; no se ha sustituido aquí esa arquitectura.

P4 ya ejecuta transformaciones, aleatorización y gestión de patrones. Mover
DSP de audio a P4 requiere un transporte de audio sincronizado adicional;
el enlace actual intercambia control/patrones, no un bus PCM de ida/vuelta.
No se atribuye el chirrido a ocho voces sin medición: la guía propone una
matriz 4/8/12/16 pistas, motores, FX, ratchets y transiciones 16/19/20.

Compilación P4 correcta: RAM 93.208 bytes; flash 1.455.516 bytes.
Compilación Daisy correcta: DTCM 97.012 bytes; SRAM 394.836 bytes;
SDRAM 53.966.732 bytes. Sin flashear. No son medidas de carga CPU.
La prueba nueva de cola de rotarys se valida por compilación; su ejecución
queda pendiente por el bloqueo de ejecutables de pruebas observado en Windows.
Ver `GUIA_SEQUENCER_NONE.md` para parámetros y pruebas manuales.

## Seguimiento más reciente: motores melódicos y bancos ampliados

Se conservan 303, SH101, FM2OP y WT. No hay evidencia suficiente para eliminar
uno. Los cambios se centran en discontinuidades de retrigger y coste calculable:

- Transición de 32 muestras (0,67 ms a 48 kHz) al redisparar una voz que ya
  suena. Se aplica a la salida de 303/SH/FM y a cada voz WT antes del filtro.
  Es una interpolación acotada, no un limitador sobre toda la mezcla.
- FM y WT conservan la fase al reutilizar una voz activa. FM también conserva
  su estado de feedback. Las voces nuevas mantienen su arranque original.
- SH almacena los coeficientes de decay/release y portamento; solo recalcula
  cuando cambian los tiempos. Su filtro se actualiza cada 8 muestras (6 kHz de
  control a 48 kHz de audio), frente a una exponencial por muestra.
- WT calcula la modulación de pitch una vez por muestra y la comparte entre
  voces. No cambia la fórmula ni la frecuencia de modulación.

Rotarys: se descartan diferencias menores a medio paso de la ganancia
configurada (51), en lugar de convertir cualquier diferencia en movimiento.
En sequencer se acepta un avance unitario por lectura de UI, separado por
65 ms, sin aplicar paquetes acumulados de cuatro opciones. Los giros rápidos
se limitan deliberadamente; no se ha medido aún el encoder físico.

| Banco | Nuevas opciones |
|---|---|
| VAR | 16: original, desplazamiento, espejo, half-time, ghost, ratchet, sparse, shifts 2/4/8, swap pares, Euclid 3/5/7, fill final, acentos |
| GLITCH | 12: original, x2/x4, reverse bloques, chop, x3, stutter, fill x2, offbeats, reverse total, swap mitades, drop 25 % |
| ORDEN | 12 permutaciones distintas dentro de cada bloque de cuatro pistas |
| PROBABILIDAD | 0–100 %, sin salto acelerado |
| EVOLVE R1 | OFF y 8 modos: MIX, probabilidad, timing, velocidad, ghost, sparse, denso, suave |
| EVOLVE R2 | Cantidad 0–100 % |
| EVOLVE R3 | 1/2/3/4/6/8/12/16 compases |
| EVOLVE R4 | Grupo de notas: todos, bombo, caja/clap, hats, percusión |

R4 de EVOLVE sustituye al anterior STOP redundante. Pulsarlo vuelve a TODOS;
pulsar R1 apaga AUTO, R2 pone cantidad cero y R3 vuelve a cuatro compases.
El grupo selecciona qué notas/probabilidades se modifican. Timing y velocidad
humanizada siguen siendo globales porque el protocolo los define así.
TODOS conserva la protección del bombo/caja; al elegir explícitamente un grupo
se permite generar notas fantasma también en él. El popup táctil ofrece los
mismos ocho intervalos de compases que el rotary.
Las transformaciones de orden siguen moviendo ritmo/dinámica, no engines ni notas MIDI.

Validación: la primera versión de `synth_rotary_expansion.cpp` pasó continuidad
en los cuatro motores, cinco segundos de render finito de SH/FM, detents y
bancos ampliados (incluidas permutaciones únicas y pulsos Euclid). Al extender
el render largo a 303/WT, Windows bloqueó el ejecutable actualizado. Esos dos
renders largos y la prueba ampliada del adaptador siguen sin validación ejecutada.
No se ha medido CPU física ni reproducido aquí la combinación exacta del usuario.

Binarios de esta revisión: `firmware_SYNTH_BANKS.bin` y
`DrumMachineV2_DaisyPod3_SYNTH_BANKS.bin`. Ambas compilaciones completadas.
P4: 93.224 bytes RAM y 1.454.384 bytes flash. Daisy: 97.012 bytes DTCM,
392.252 bytes SRAM y 53.966.732 bytes SDRAM. Estos tamaños no miden CPU.
No se ha flasheado el dispositivo.

## Seguimiento: bloqueo al pasar de 19 a 20 con FX/EVOLVE

El usuario ha reproducido un bloqueo con error de sincronización y audio
sostenido. La revisión anterior no resolvió ese caso. Nuevos hallazgos:

- `hw.PrintLine` usa el mismo CDC que el protocolo. En libDaisy el logger pasa
  a `TransmitSync`, con espera sin límite, después de dos envíos correctos.
  Los logs de preset seguían activos aunque no se iniciase el log de arranque.
  Todos los mensajes de diagnóstico quedan ahora desactivados en producción.
- `CDC_Transmit_FS` accede a `pClassData` sin comprobar desconexión/reset. El
  envío del protocolo comprueba configuración y puntero, y protege la secuencia
  comprobar/enviar frente al IRQ USB. No espera a que termine el envío.
- Cambiar el engine de una pista cancelaba triggers y modificaba voces/notas
  desde main. Ahora se encola y se ejecuta al comienzo del bloque de audio.
- La protección de presets no cubría la liberación previa. Ahora abarca toda
  la operación sobre el engine afectado, conserva guardas anidadas y evita que
  el temporizador de notas toque un engine en modificación. Los parámetros
  genéricos también se protegen y rechazan floats no finitos.
- La reducción por sobrecarga no retiraba delay/reverb principales. Ahora sí,
  con margen mayor y recuperación de aproximadamente un segundo. Un límite al
  90 % del tiempo de bloque desvanece el tramo sin renderizar y devuelve CPU al
  hilo principal. Es una degradación de emergencia: puede perder avance musical
  en las muestras omitidas, no promete audio íntegro durante una sobrecarga.
  `audioDeadlineTrips` cuenta activaciones para depuración; no se ha ampliado el
  protocolo de telemetría para mostrarlo en pantalla.
- El handler de fault dejaba al DMA repitiendo audio. Ahora fuerza cero digital
  en SAI1 antes del SOS; esto silencia el fallo, no recupera la CPU.
- P4 pausa las mutaciones automáticas mientras se transfiere o confirma un
  cambio de patrón. R4 de EVOLVE indica claramente PARAR EVOLVE / PULSA: STOP
  y DETENIDO. Su función es detener AUTO.

Son riesgos identificados en código, no una reproducción del fallo físico.
La prueba portable del deadline compila, pero Windows bloqueó su ejecución;
no se registra como aprobada. Queda pendiente repetir 19 → 20 en la placa,
observando si hay SOS, y medir el margen real del callback.

Binarios de este seguimiento: `firmware_TRANSITION_USB_FIX.bin` (P4) y
`DrumMachineV2_DaisyPod3_TRANSITION_USB_FIX.bin` (Daisy). Los binarios anteriores
se conservan. No se ha flasheado el equipo.
Las compilaciones finales P4 y Daisy han terminado correctamente. La ejecución
de la nueva prueba de deadline sigue bloqueada por la política de Windows.

Se han encontrado y corregido fallos de control, retrigger y continuidad. No se
ha reproducido el incidente del equipo físico ni medido su carga de CPU. Por eso
no se considera demostrado que todos los picos del patrón 16 estén resueltos.

## Hallazgos y correcciones

| Prioridad | Fallo comprobado en código | Corrección |
|---|---|---|
| Alta | EVOLVE enviaba primero la activación de una nota fantasma y después su velocidad/probabilidad. El audio podía disparar el estado intermedio, con velocidad antigua y ratchet heredado. | Preparar todos los campos y enviar un solo paso completo, con ratchet 1 para la nueva nota fantasma. |
| Alta | Daisy actualizaba los campos de un paso USB por separado mientras el callback podía copiarlo. | Publicar los campos en una sección crítica breve, conservando el estado anterior de interrupciones. |
| Alta | Actualizar el patrón activo cancelaba todos los ratchets pendientes y liberaba notas melódicas. | Conservar los snapshots del par ya programado; el siguiente par lee el patrón actualizado. Cambiar de patrón y STOP siguen liberando notas. |
| Alta | Cada golpe WAV del sequencer abría otra voz si quedaba espacio. Ratchets y EVOLVE acumulaban colas, nivel y trabajo DSP. | Retrigger monofónico por pad del sequencer, reutilizando su voz. Las voces LIVE mantienen su asignación polifónica. |
| Alta | Al robar/reutilizar una voz se sumaban la cola anterior y el ataque nuevo a nivel completo. | Entrada complementaria al decaimiento del residual; se recuerda la salida conjunta para el siguiente retrigger. |
| Media | El choke entre pads cortaba una voz directamente a cero, especialmente audible en hi-hats repetidos. | Liberar un residual corto, sin seguir leyendo samples ni procesando los FX de esa voz. |
| Media | El chirp de acento de TB303 dejaba de decaer al llegar una nota sin acento, pero seguía sumándose al cutoff. | Decaimiento continuo también después de desactivar el acento. |
| Media | Un gate WAV corto podía terminar en una muestra no nula; reverse arrancaba fuera del límite acortado. | Fade final mínimo de 32 muestras del sample y arranque reverse en el límite efectivo. |
| Media | Apagar EVOLVE no retiraba su humanización global. | Restaurar la humanización anterior si todavía pertenece a EVOLVE y al mismo patrón; respetar un cambio manual posterior. |
| Media | La pulsación de un rotary entero/enum no hacía nada; el reset podía adoptar como base las modificaciones posteriores de EVOLVE. | Reset explícito por familia que recupera su base capturada, sin convertir esas modificaciones posteriores en el nuevo original. |

## Pulsaciones en el sequencer

- Variaciones: ORIGINAL del grupo.
- Probabilidad: 100 % en los pasos activos del grupo.
- Glitch y orden: recuperar el ritmo capturado antes de aplicar esa familia.
- EVOLVE: R1 desactiva AUTO, R2 lleva cantidad a cero, R3 vuelve a un compás y
  R4 detiene AUTO. La evolución manual sigue disponible en los controles de pantalla.
- En el popup FX de pista: TYPE/CUTOFF/RESONANCE desactivan el filtro; DRIVE y
  envíos vuelven a cero; CRUSH vuelve a 16 bits.

El reset de una familia restaura su snapshot: puede retirar cambios de EVOLVE
hechos después en ese mismo grupo. Apagar AUTO por sí solo detiene futuras
evoluciones y retira la humanización propia; no deshace las notas ya generadas.

## Rendimiento y sonido

El sequencer WAV utiliza como máximo una voz por pad, frente a acumular hasta
agotar el pool de 32. Esto reduce trabajo y solapamientos con ratchet, pero cambia
deliberadamente las colas: el siguiente golpe sustituye al anterior. No es una
medición de porcentaje de CPU ahorrado.

Una nota fantasma de EVOLVE pasa de tres mensajes de paso a uno; su retirada,
de dos a uno. La transición del sampler no añade memoria de audio ni buffers
grandes. No se han cambiado las ganancias generales ni la suma de los kits.

## Verificación y límites

- Compilaciones P4 y Daisy completas, sin flashear. Binarios nuevos:
  `P4/.pio/build/esp32p4-upload/firmware_RATCHET_RESET.bin` y
  `DaisyPod3/build/DrumMachineV2_DaisyPod3_RATCHET_RESET.bin`.
- PASS: 10.000 retriggers usando el selector de voces de producción; aislamiento
  de una voz LIVE; continuidad y límites de transición en 441 pares de amplitud.
- PASS: regresiones existentes de transferencia, 843.552 combinaciones de timing
  y suma/salidas individuales de 808/909/505. No cubren el callback completo.
- Las pruebas ampliadas de reset y estrés de motores compilan. Windows bloqueó
  ambos ejecutables mediante Control de aplicaciones: no se han ejecutado ni
  se ha intentado eludir la política.
- La prueba de estrés preparada cubre 20 segundos por motor, 300 BPM y ratchet x4
  denso, además del chirp TB303. No sustituye una medición del callback de placa.
- El banco de fábrica actual es LegacyFactoryBank: slot 15 es ACID DORIAN FALL y
  slot 16 ACID OCTAVE. No se presupone cuál contiene el patrón guardado del usuario
  ni que el preset/engine siga siendo el de fábrica.

Persisten dos limitaciones de arquitectura que esta revisión no oculta: varias
voces LIVE del mismo pad siguen compartiendo estados de FX; los kits de batería
sintéticos se mezclan antes de la ruta del primer track asignado al motor. No hay
aislamiento completo de FX por voz/instrumento. La corrección monofónica evita el
primer problema para las voces propias del sequencer, pero no para LIVE simultáneo.

Aceptación pendiente: patrón 16 sin FX, luego cada grupo VAR/RATCHET/GLITCH,
EVOLVE y sus resets; repetir con filtros resonantes y con PLAY/STOP. Registrar
CPU media/pico, clipping y errores USB existentes en diagnóstico, junto con el
engine/preset y el filtro exactos. Distinguir clic de retrigger, saturación sostenida
y bloqueo de control. No se ha flasheado ningún dispositivo.
