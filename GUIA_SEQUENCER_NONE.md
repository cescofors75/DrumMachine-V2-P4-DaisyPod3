# Sequencer — guía y hoja de pruebas para NONE

Revisión: 15/09/2026. Corresponde a los firmwares `SEQUENCER_REVIEW`.
Las pruebas de esta hoja deben realizarse en el equipo; no están marcadas como superadas.

## 1. Manejo básico

| Control | Función |
|---|---|
| PLAY / PAUSE | Inicia o detiene la reproducción. |
| P− / P+ y nombre del patrón | Cambia de patrón o abre la lista. |
| Q 1 BAR / Q OFF | Cambio de patrón al compás / cambio inmediato. |
| Celda de la cuadrícula | Activa o desactiva un golpe. |
| Pulsación larga de una celda | Probabilidad y parámetros fijados para ese paso. |
| Nombre de pista | MUTE: silencia o recupera esa pista. |
| S | SOLO exclusivo: escucha esa pista. Otro S cambia de pista; repetir S elimina el solo. Activar solo quita el mute de esa pista. |
| FX de una fila | Abre los efectos de esa pista. |
| X de una fila | Borra los FX de esa pista. No borra sus notas ni los FX master. |
| Fader físico | Selecciona uno de los cinco bancos de rotarys de la tabla siguiente. |
| Botón del rotary | Restaura/anula su parámetro según la tabla. |
| Botones DaisyPod | BACK y mostrar/ocultar indicadores. Ocultarlos no cambia la asignación de los rotarys. |
| LIST / SAVE | Cargar patrón / guardar en un slot; revisar la confirmación de reemplazo. |
| MIDI | Importar desde la biblioteca MIDI. Hasta cuatro páginas de 16 pasos; cambiar de página carga ese bloque de 16 pasos, no crea reproducción continua de 64 pasos. |

Los selectores avanzan un paso por actualización de pantalla y conservan los clics pendientes. Cantidad y probabilidad aceptan los clics acumulados. No hay aceleración artificial. Al pulsar para resetear se descarta el movimiento anterior pendiente.

## 2. Los cinco bancos de rotarys

| Banco | R1 | R2 | R3 | R4 | Al pulsar |
|---|---|---|---|---|---|
| VARIACIONES | Bombo | Caja / clap | Hi-hats | Percusión / resto | Restaura la base capturada de ese grupo. |
| EVOLVE | Modo / OFF | Cantidad 0–100 % | Intervalo | Grupo de notas | R1 OFF; R2 0 %; R3 4 compases; R4 TODOS. |
| PROBABILIDAD | Bombo | Caja / clap | Hi-hats | Percusión / resto | 100 % para los pasos activos del grupo. |
| GLITCH | Bombo | Caja / clap | Hi-hats | Percusión / resto | Restaura la base capturada. |
| ORDEN DE RITMOS | Filas 1–4 | Filas 5–8 | Filas 9–12 | Filas 13–16 | Restaura el orden capturado. |

Grupos de rotarys por fila: bombo 1; caja/clap 2 y 6; hats 3 y 4; resto 5 y 7–16. Son asignaciones por fila, aunque cambies el motor de esa fila.

VAR, GLITCH y ORDEN se aplican después de unos 250 ms sin giro. Otras modificaciones pueden convertirse en una nueva base: «ORIGINAL» no equivale siempre al patrón de fábrica. Para una comparación limpia, vuelve a cargar una copia guardada del patrón.

**VARIACIONES — 16 opciones:** ORIGINAL, DESPLAZAR (+1 paso), ESPEJO, HALF TIME, GHOST, RATCHET, SPARSE, SHIFT +2, SHIFT +4, SHIFT +8, SWAP PARES, EUCLID 3, EUCLID 5, EUCLID 7, FILL FINAL, ACENTOS.

**GLITCH — 12 opciones:** ORIGINAL, RATCHET x2, RATCHET x4, REV BLOQUES, CHOP, RATCHET x3, STUTTER, FILL x2, OFFBEATS, REV TOTAL, SWAP MITADES, DROP 25 %.

**ORDEN — 12 opciones:** ORIGINAL, ROTAR +1/+2/+3, INVERTIR, SWAP 1-2, SWAP 3-4, SWAP 1-4, SWAP 2-3, ZIGZAG, PARES, CRUZADO. Intercambia golpes, velocidad, probabilidad y ratchet; conserva motores, notas MIDI y locks propios de las pistas.

**PROBABILIDAD:** 0 % nunca dispara; 100 % siempre dispara, si el paso está activo y la pista es audible. Los valores intermedios son aleatorios: no garantizan un número fijo de golpes por compás.

## 3. EVOLVE

| R1: modo | Qué modifica |
|---|---|
| OFF | Detiene nuevas evoluciones. No deshace las notas ya modificadas. |
| MIX | Probabilidades, notas fantasma y humanización. |
| PROBABILIDAD | Varía probabilidades de notas existentes; puede retirar fantasmas de evoluciones anteriores. |
| TIMING | Humanización temporal global. |
| VELOCIDAD | Humanización de velocidad global. |
| GHOST | Añade y retira notas fantasma. |
| SPARSE | Reduce probabilidades, con suelo de 40 %, sin añadir fantasmas. |
| DENSO | Genera fantasmas con mayor frecuencia. |
| SUAVE | MIX con la mitad de intensidad. |

- R2: intensidad 0–100 %. Cero suspende la transformación.
- R3: cada 1, 2, 3, 4, 6, 8, 12 o 16 compases.
- R4: TODOS, BOMBO, CAJA / CLAP, HI-HATS o PERCUSIÓN.
- TODOS protege más el bombo/caja. Elegir expresamente un grupo permite generar fantasmas también en él.
- El grupo limita las notas y probabilidades. TIMING y VELOCIDAD humanizada siguen siendo globales.
- El popup EVOLVE permite usar los cuatro rotarys. «APLICAR AHORA» solicita una evolución manual; el procesamiento se realiza fuera de la tarea gráfica.
- Apagar EVOLVE restaura la humanización que gestionaba, pero no revierte todas las notas. Para recuperar exactamente el patrón de prueba, recarga la copia guardada.

## 4. Efectos de cada pista

Abre FX en la fila deseada. El fader cambia entre estos dos bancos:

| Banco | R1 | R2 | R3 | R4 |
|---|---|---|---|---|
| FILTER FAMILY | Tipo de filtro | Cutoff 0–127 | Resonancia 0–127 | Drive 0–127 |
| COLOR / SEND | Bitcrush 4–16 bits | Reverb 0–100 % | Delay 0–100 % | Chorus 0–100 % |

Tipos: OFF, LOWPASS, HIGHPASS, BANDPASS, NOTCH, ALLPASS, RESONANT. Cutoff es una escala de control, no Hz lineales.

Pulsar tipo/cutoff/resonancia apaga el filtro. Pulsar drive o un envío lo pone a cero. Pulsar bitcrush vuelve a 16 bits. RANDOM FX genera ajustes; CLEAR FX los neutraliza. Los envíos necesitan que el efecto master correspondiente esté activo.

**FX LAB master**, separado de los FX de pista: FLANGE, DELAY, REVERB, FOLD, CRUSH, PHASER, CUTOFF, RESO, DRIVE, BITS, SRATE, FILTER, TREMOLO, CHORUS, COMP, A.WAH, MORPH, STUTTER. VIEW muestra 4/8/12 tarjetas. Tocar una tarjeta la asigna al rotary de su columna; el fader asigna una fila completa. Pulsar el rotary conmuta el efecto asignado.

## 5. Opciones táctiles y automatismos

| Opción | Valores / comportamiento |
|---|---|
| Probabilidad de un paso | 100, 75, 50, 25 o 10 %. |
| Lock CUTOFF | ON/OFF; fija 6000 Hz en ese paso. |
| Lock REVERB | ON/OFF; fija envío 80 %. |
| Lock VOLUMEN | ON/OFF; fija valor 127. |
| VAR táctil | NEON BREAK, RATCHET STORM, GHOST GROOVE, POLYRHYTHM 3x5, HALF-TIME DROP, MIRROR BEAT, TOM CASCADE, ACID SWITCH, HAT LIFT, SPARSE SPACE, UNDO LAST VAR. Son presets distintos del banco VAR de rotarys. |
| GROUPS | Mute de DRUMS (1–7, 9–11), BASS (8), SYNTH (15–16), XTRA (12–14). Sus grupos son diferentes de los grupos de variaciones. |
| SONG | Cambios automáticos de patrón; estilos TECHNO, HOUSE, BREAK, HIP-HOP, TRAP y MINIMAL; intervalos 1/2/4/8 compases. |
| AUTO | Panel conjunto de SONG, AUTO FX, AUTO MIX, EVOLVE y VAR; controles de activación, intervalo, notificaciones y opciones específicas. |
| MATRIX | Hasta 16 columnas con patrón y presets de filtro, mixer y melodía; ocho slots de preset por categoría; intervalo 1/2/4/8 compases. Columnas sin patrón se omiten. NONE en un preset significa no aplicar uno nuevo: no limpia el anterior. |

## 6. Prueba base y prueba de carga

Usar una copia guardada del patrón; evitar sobrescribir el original durante las pruebas.

1. Apagar SONG, MATRIX, AUTO FX, AUTO MIX, EVOLVE y VAR automática. Recargar el patrón. Q 1 BAR.
2. Quitar solos, neutralizar FX de pista y master, comprobar probabilidad 100 % y ratchet normal. NONE en MATRIX no sustituye este paso.
3. Probar diez clics lentos hacia cada lado en cada rotary/banco. Comprobar todos los extremos, especialmente 0/1 % y 99/100 %.
4. Repetir con giros rápidos, pulsación para resetear y cambio de banco. No debe seguir cambiando después del reset ni aplicarse el giro al banco anterior.
5. Abrir/cerrar EVOLVE y repetir R1–R4 mientras reproduce. Repetir en FX de pista. Los diálogos de guardar/cargar y otros modales mantienen bloqueados los controles de fondo intencionadamente.
6. Probar cada mute y solo; cambiar de solo A a B; pulsar solo otra vez; mutear la pista en solo; repetir con GROUPS. Comprobar sonido y estado visual.
7. Reproducir 60 segundos con 4, 8, 12 y 16 pistas, activándolas gradualmente. Repetir primero con samples y después añadiendo 303, SH, FM y WT de uno en uno.
8. Repetir con EVOLVE suave, MIX/denso, ratchets y FX, incorporándolos de uno en uno. Mantener el mismo BPM y nivel de salida para comparar.
9. Probar patrones 16, 19 y 20, y transiciones 19→20 y 20→19. Registrar si hay error de sincronización, chasquido aislado, distorsión continua o tono sostenido.
10. Abrir DASHBOARD → CPU / MEMORIA y anotar CPU media/pico, AUDIO XRUN y FX LOAD antes/después. XRUN cuenta interrupciones por agotamiento del presupuesto de audio desde el arranque. FX LOAD REDUCIDOS indica protección por carga; N/D indica firmware sin ese diagnóstico. Ninguno de estos indicadores detecta todos los tipos de distorsión.

| Patrón / BPM | Pistas y motores | FX / EVOLVE / ratchet | CPU media / pico | XRUN antes→después | Resultado y pasos para repetir |
|---|---|---|---|---|---|
| | | | | | |
| | | | | | |
| | | | | | |

## 7. Límites que hay que comprobar

Los motores de síntesis son compartidos entre pistas: asignar el mismo motor a varias filas no crea instancias independientes. La ruta actual usa el primer track del motor para parte del mezclado/FX. Probar explícitamente solos y FX con motores repetidos; no asumir aislamiento completo por pista en esa configuración.

Un chirrido con ocho pistas no demuestra por sí solo sobrecarga: comparar XRUN, CPU y la combinación exacta. El firmware mantiene la mezcla y las ganancias anteriores; no elimina motores. La revisión reduce trabajo del filtro modulado y de FX inaudibles, pero falta medir el resultado acústico en hardware.
