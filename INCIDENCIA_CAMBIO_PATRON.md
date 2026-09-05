# Saturación al cambiar de patrón

## Segunda revisión: retirada del mezclador nuevo

El usuario comunica que el síntoma persiste. Se retira la mezcla por instrumento
introducida hoy: el bloque completo de mezcla de sintetizadores vuelve al commit
`670f908` (30 de agosto), conservando únicamente la máscara de protección durante
la recarga de presets. Se verificó por comparación exacta del bloque de código.
La versión retirada se conserva en `tools/diagnostics/synth_routing_before_rollback.txt`.

Consecuencia: los kits vuelven a usar el volumen y los efectos de su primera pista
asignada. Se retiran de ese bloque los factores LIVE/SEQ y los cambios de ganancia
introducidos hoy. Es una retirada para recuperar el comportamiento anterior;
la independencia de efectos por instrumento queda pendiente de una implementación
que se valide con medidas en hardware.

Coste de encaminamiento de baterías: vuelve a 3 llamadas por muestra (una por kit
habilitado), frente al recorrido nuevo de hasta 16 pistas y sus efectos, además
de los motores melódicos. No equivale a un porcentaje medido de CPU.
El firmware de esta retirada compila correctamente: `DaisyPod3/build/DrumMachineV2_DaisyPod3_rollback_mixer.bin`. SRAM: 392.948 B (79,95%). No se ha flasheado.
No se ha confirmado todavía en placa que esta retirada elimine el síntoma.
El historial de la primera revisión se conserva debajo; sus cifras de build
corresponden a aquella revisión.

---

Se revisa el fallo comunicado al cambiar patrones, con PLAY y también detenido.
No se ha reproducido ni medido todavía en la placa: las causas siguientes son
riesgos confirmados en el código, no una atribución definitiva del síntoma.

## Cambios de esta revisión

- Daisy protege cada motor durante la escritura de su preset. El audio no procesa
  ese motor con parámetros o referencias PCM a medio actualizar. Los comandos
  de notas esperan en su cola y los disparos del secuenciador se aplazan mientras
  ese motor está ocupado. El resto de motores sigue procesándose.
- La selección inmediata se aplica al principio del bloque de audio y libera
  notas retenidas, repeticiones pendientes y locks del patrón anterior.
  También se limpian al sustituir el patrón activo y en transiciones de canción.
- Los locks de cutoff usan el suavizado existente por bloque: se elimina el salto
  de coeficientes y el cálculo trigonométrico dentro de cada repetición.
- P4 carga presets únicamente de los motores asignados al patrón y los posibles motores
  de respaldo 909/505 para pistas sampler sin muestra.
- El ejecutor de pruebas incluye ahora la regresión existente de reposo de los
  tres kits, PCM, finales de muestra, choke y recuperación del limitador.

## Rendimiento: qué se puede afirmar

| Operación | Antes de esta revisión | Ahora |
| --- | --- | --- |
| Presets enviados al cambiar patrón | 9 | Motores usados y posibles respaldos (1–9) |
| Recálculo de filtro dentro de una repetición | Hasta 2 biquads cuando cambia cutoff | 0; suavizado por bloque |
| Selección inmediata | Escritura desde control durante audio | Inicio del siguiente bloque |
| Presupuesto de audio a 48 kHz / 128 muestras | 2,667 ms | 2,667 ms |

La protección de presets puede pausar brevemente el motor que se está editando;
no es un crossfade ni garantiza transiciones sin clics. No se atribuyen porcentajes
de mejora de CPU sin medición en Daisy.

## Verificación

Las pruebas portables pasan: 843.552 combinaciones de temporización, transferencia
con ACK y rechazos, aislamiento de instrumentos, suma de salidas, almacenamiento
ante 177 escrituras interrumpidas y ciclos completos de reposo/PCM de los kits.
Estas pruebas no simulan la interrupción de audio del STM32 ni validan el sonido
físico de la transición. Pendiente probar cambios repetidos con y sin PLAY,
colas largas, filtros resonantes y kits PCM, observando CPU y escuchando la salida.

Ambos firmwares compilan y generan binarios; no se han flasheado.
Memoria del build actual: Daisy DTCMRAM 96.900 B (73,93%), SRAM 393.980 B
(80,16%), SDRAM 53.966.732 B (80,42%). P4 RAM 95.520 B (29,2%) y flash
1.420.562 B (21,7%). Estos valores describen el estado completo del repositorio,
que ya incluía otras optimizaciones; no miden el ahorro exclusivo de esta revisión.
