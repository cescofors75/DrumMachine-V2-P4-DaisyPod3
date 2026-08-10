# Auditoría de FX — DrumMachine V2

Fecha: 10 de agosto de 2026

## Alcance y conclusión

La pantalla `FX LAB` contiene 12 controles, pero no representa 12 algoritmos
independientes. Hay seis efectos master directos (`FLANGE`, `DELAY`, `REVERB`,
`FOLD`, `CRUSH`, `PHASER`), cinco parámetros del bloque global de
filtro/distorsión (`CUTOFF`, `RESO`, `DRIVE`, `BITS`, `SRATE`) y un selector de
15 modelos (`FILTER`).

La auditoría encontró fallos audibles reales: `FOLD` no era un wavefolder,
`CRUSH` duplicaba a `BITS`, el phaser podía añadir unos 12 dB, la reducción de
sample rate no respetaba valores como 32 kHz, y tres modelos de EQ eran
prácticamente transparentes. También había estado DSP compartido entre canales
en phaser, chorus, autowah y comb. Todos estos puntos quedaron corregidos.

## Resultado por control

| Control | Hallazgo | Corrección aplicada |
|---|---|---|
| FLANGE | El rate recibido como `float` se multiplicaba por 10 y la modulación estéreo no era independiente. | Rate en Hz sin escalado doble; instancias L/R con una ligera desviación de rate para más anchura. |
| DELAY | El modo mono alimentaba el delay solo con L y algunos cambios de tiempo no actualizaban la línea R. | Entrada mono `(L+R)/2`; tiempo sincronizado en las dos líneas; ping-pong conserva feedback cruzado. |
| REVERB | La mezcla era correcta, pero podía ser sobreescrita por la pantalla aun teniendo dueño mecánico. | Propiedad física completa y bloqueo del control digital cuando aparece `HW`. |
| FOLD | `DaisySP::Fold` es un sample-and-hold por incremento, no un wavefolder; además se reutilizaba un estado para L/R. | Wavefolder triangular real, sin estado y estéreo, transparente a ganancia 1. |
| CRUSH | Controlaba la misma profundidad que `BITS`; no existía una identidad propia. | Macro dual: baja bits de 16 a 6 y sample rate de 42 a 4 kHz con curvas musicales. Conflicta explícitamente con `BITS` y `SRATE` para impedir dos dueños. |
| PHASER | Solo L era procesado y R recibía una copia parcial. DaisySP sumaba cuatro motores sin normalizar. | Phaser independiente L/R, rates ligeramente distintos y salida normalizada a 0,25 para evitar el salto aproximado de +12 dB. |
| CUTOFF | El porcentaje ocultaba una frecuencia perceptualmente no lineal. | Curva logarítmica 20 Hz–20 kHz y lectura real en Hz/kHz. |
| RESO | Escala poco clara y lectura porcentual. | Rango útil Q 0,7–20 y lectura numérica de Q. |
| DRIVE | Había dos escalas, 0–1 y 0–100; la compensación anterior hacía caer mucho el volumen al aumentar drive. | Estado DSP único 0–1, compatibilidad de entrada porcentual, preganancia gradual, mezcla dry/wet y compensación moderada. |
| BITS | La cuantización usaba `2^bits` sobre un intervalo bipolar y entregaba aproximadamente un bit más del indicado. | Cuantización bipolar con `2^(bits-1)-1`; rango 4–16 bits y bypass real a 16. |
| SRATE | El divisor entero hacía que 32 kHz a 48 kHz pudiera comportarse como bypass. | Acumulador de fase fraccional: tasa media exacta entre 4 y 42 kHz; `0` es bypass. |
| FILTER | PEAK/LOW SHELF/HIGH SHELF recibían 0 dB y eran inaudibles; COMB compartía memoria con early reflections. | Los tres EQ master usan +6 dB y el comb tiene líneas L/R propias. Los 15 modelos se muestran por nombre. |

## Otras correcciones de la cadena

- Chorus usa un `ChorusEngine` independiente por canal. Esto evita procesar dos
  veces el mismo estado y usa dos líneas de 2400 muestras en vez de cuatro.
- Autowah usa estados separados L/R.
- Bitcrush, drive y sample-rate reduction ya no obligan a ejecutar un filtro
  Biquad cuando `FILTER` está en `OFF`.
- Los FX caros siguen sujetos al mecanismo `fxShed` cuando la carga de audio se
  aproxima al límite. Delay, tremolo y reverb permanecen fuera de ese descarte
  para evitar cortes evidentes en colas y tempo.
- Todos los parámetros de la pantalla usan comandos específicos. Cambiar un
  valor ya no reenvía el bloque global completo ni pisa parámetros cuyo dueño
  es un control mecánico.

## CPU y memoria

La revisión de CPU de esta entrega es estática; el porcentaje real depende del
número de voces, sintes, FX por pista y FX master simultáneos y debe medirse en
la placa mediante `CMD_GET_CPU_LOAD`/modo stress.

Resultados de compilación:

- P4: 82.100 bytes de RAM (25,1 %) y 1.294.732 bytes de flash de aplicación
  (19,8 %).
- DaisyPod3: binario de 364.816 bytes; `text` 357.112, `data` 7.696 y BSS total
  54.049.736 bytes.
- La sección SDRAM de Daisy ocupa `0x337278c`, aproximadamente 51,45 MiB de
  64 MiB (80,4 %), dejando unos 12,55 MiB. El linker termina sin overflow.
- El chorus estéreo optimizado ahorra aproximadamente 19,2 KiB respecto a usar
  dos objetos `Chorus` completos. El phaser estéreo y las líneas comb propias
  aumentan SDRAM, pero eliminan contaminación entre canales/efectos.

## Prueba recomendada en hardware

1. Medir CPU media y pico con 16 pistas, máximo de voces habitual y los seis FX
   master activos.
2. Repetir con `FILTER=RESONANT`, `FILTER=LADDER` y `FILTER=COMB`, que son los
   modelos más costosos o con feedback.
3. Barrer cada control de 0 a 100 verificando continuidad, ausencia de saltos
   de volumen y pico master por debajo del clipping sostenido.
4. Comparar L/R con material mono y estéreo para confirmar que phaser, chorus,
   autowah y comb ya no colapsan ni contaminan canales.
5. Registrar `cpuAvg`, `cpuPeak`, drops USB/SPI y cualquier activación de
   `fxShed`; esos datos permitirán fijar un presupuesto de CPU cuantitativo para
   la siguiente afinación.
