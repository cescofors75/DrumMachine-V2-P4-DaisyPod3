# Auditoría DaisyPod3 y ESP32-P4

Seguimiento: las correcciones y su validación posterior están documentadas en
[CORRECCIONES_Y_RENDIMIENTO.md](CORRECCIONES_Y_RENDIMIENTO.md). Este archivo
conserva los hallazgos originales y los límites de la revisión inicial.

Fecha: 2026-09-05. Revisión estática del código propio y del contrato entre placas. No se ha modificado el firmware. No constituye una validación completa en hardware ni una revisión exhaustiva de todas las dependencias.

Se revisaron principalmente transporte USB, sincronización y almacenamiento de patrones, secuenciador, mezcla de audio, automatizaciones, rutas de sintetizadores y controles de UI. También se inspeccionaron el importador MIDI, FM2Op y persistencia de ajustes; no se atribuyen fallos a esas partes sin evidencia suficiente.

Prioridades: P1 = alta, afecta reproducción o datos; P2 = media, comportamiento incorrecto o experiencia de uso. Las referencias son relativas a la raíz del repositorio.

## Hallazgos

### 1. [P1] Varias pistas de síntesis comparten el mezclador de la primera

**Código:** `DaisyPod3/main.cpp:5549`, `:6030`, `:6065`.

`engTrk` elige únicamente la primera pista asignada a cada motor. Después, el resultado completo de `synth808.Process()`, `synth909.Process()` o `synth505.Process()` pasa por `synthTobus` con esa pista. Por tanto, volumen, mute, solo, panorama y efectos no se aplican individualmente a los instrumentos del mismo motor.

**Reproducción:** asignar 808 a BD y SD; programar ambos; silenciar BD. El mute de la primera pista elimina también la salida de SD. Cambiar el volumen o panorama de SD no controla su salida de forma independiente.

**Corrección:** obtener salidas por instrumento/voz y mezclarlas por pista. Para motores monofónicos compartidos, definir y mostrar explícitamente su exclusividad o instanciarlos por pista.

### 2. [P1] Cambiar velocidad o activar un paso reinicia otros atributos en Daisy

**Código:** `P4/src/control_api.cpp:2137–2169`, `P4/src/daisy_usb_transport.h` (`setStep`), `DaisyPod3/main.cpp:9155–9170`.

Las llamadas de edición de velocidad y activación omiten división y probabilidad, usando los valores predeterminados 1 y 100. Daisy también reconstruye el ratchet desde el nibble superior de división, que llega a cero: ratchet 1. La edición de probabilidad conserva la división, pero tampoco empaqueta el ratchet. El modelo local conserva atributos que el motor acaba de perder.

**Reproducción:** cargar un paso con ratchet 4, división 2 y probabilidad 30%; editar solo su velocidad. Daisy queda con ratchet 1, división 1 y probabilidad 100%. Resincronizar puede recuperar los valores anteriores, produciendo otro cambio inesperado.

**Corrección:** utilizar un único serializador del estado completo del paso, incluyendo el empaquetado del ratchet, en todas las ediciones.

### 3. [P1] Los slots reutilizados heredan automatizaciones antiguas

**Código:** `P4/src/control_api.cpp:323–340`, `DaisyPod3/main.cpp:9131–9149`.

La carga de pista preserva los parameter locks en Daisy. P4 solo envía locks cuando al menos uno está activado en el patrón nuevo. No existe un envío para limpiar un paso que ahora no tiene ninguno.

**Reproducción:** cargar un patrón con un lock de volumen cero y después otro sin locks en el mismo slot Daisy. El segundo puede seguir silenciado en ese paso. La reutilización ocurre tanto al seleccionar patrones que coinciden módulo 20 como al preparar colas.

**Corrección:** limpiar todos los locks al comenzar una carga completa o transmitir explícitamente los estados desactivados para todos los pasos.

### 4. [P1] Guardar puede destruir la copia anterior antes de asegurar la nueva

**Código:** `P4/src/pattern_store.cpp:113–149`.

Se modifica el patrón destino en RAM antes de asegurar el almacenamiento y se borra el archivo existente antes de abrir/escribir el nuevo. Un error de apertura, escritura incompleta o pérdida de alimentación puede eliminar la versión anterior. En caso de error, `savedMask` tampoco se invalida, de modo que un slot antes guardado puede continuar marcado como disponible aunque su archivo ya no sea válido.

**Reproducción:** sobrescribir un slot existente con almacenamiento sin espacio o un fallo de escritura. La función devuelve error después de haber alterado RAM y eliminado el archivo anterior.

**Corrección:** crear y verificar una nueva versión antes de sustituir la anterior; usar dos copias con generación y checksum si se necesita recuperación robusta ante cortes. Publicar el cambio en RAM y estado visual tras el éxito.

### 5. [P1] Se seleccionan patrones aunque su carga haya fallado

**Código:** `P4/src/control_api.cpp:264–273`, `:493–518`; `P4/src/usb_cdc_handler.cpp` (`usb_cdc_write`).

`UploadPattern` devuelve éxito/error, pero selección, cola y sincronización descartan el resultado. Además, `SendWithRetry` confirma únicamente la entrada en la cola USB, no la aplicación del patrón en Daisy. Se puede seleccionar o encolar un slot parcialmente actualizado y consumir la petición pendiente sin recuperarla.

**Reproducción:** provocar congestión o desconectar durante la carga de varias pistas. El estado local puede avanzar aunque el contenido residente no esté completo.

**Corrección:** respetar el resultado, conservar la solicitud para recuperación y añadir confirmación de carga completa con revisión/checksum. Preparar un slot inactivo y activarlo solo cuando esté confirmado.

### 6. [P2] Swing y ratchet no utilizan la misma duración de paso

**Código:** `DaisyPod3/main.cpp:5434`, `:5465`, `:5654–5662`.

El ratchet usa `samplesPerStep / ratchets`, pero el swing acorta los pasos impares. Al comenzar el paso siguiente, `DsqFireStep` cancela los triggers pendientes. Así se pierden las últimas repeticiones. Humanize puede aumentar el problema al añadir demora.

**Comprobación ejecutada:** simulación del contador y de los límites con PowerShell, a 120 BPM/48 kHz y ratchet 4, sin humanize:

| Swing | Duración del paso impar | Golpes efectivos |
| --- | ---: | ---: |
| 0 | 6000 muestras | 4 |
| 50 | 4500 muestras | 3 |
| 100 | 3000 muestras | 2 |

Esta comprobación reproduce el algoritmo, no ejecuta el firmware.

**Corrección:** calcular las repeticiones sobre la duración efectiva del paso y el margen disponible después de humanize; definir también cómo se relaciona el gate con esa duración.

### 7. [P2] Los locks se ignoran en sintetizadores y persisten en samples

**Código:** `DaisyPod3/main.cpp:5320–5342`.

Cutoff, reverb y volumen por paso solo se aplican dentro de `if(!isSynth)`. En un sintetizador no tienen efecto. En samples escriben directamente los valores globales de la pista y no restauran el valor base al llegar a un paso sin lock.

**Reproducción:** colocar un lock de volumen cero en un paso de sample y dejar el siguiente sin lock: el siguiente hereda el cero. Cambiar la pista a síntesis: el lock deja de aplicarse.

**Corrección:** separar valores base y valores efectivos por paso, restaurarlos según una semántica documentada y aplicar la automatización antes de bifurcar las rutas de sonido. Si se desea automatización persistente, indicarlo expresamente en la UI.

### 8. [P2] El volumen SEQ no controla las notas secuenciadas de sintetizadores

**Código:** `DaisyPod3/main.cpp:5317–5342`, `:6041`, `:6065–6138`.

La ruta de sample entrega `seqVolume` a `TriggerPad`; las ramas de síntesis disparan con velocity y su mezcla usa `trackGain`, sin aplicar `seqVolume`. SEQ a cero no silencia el conjunto si hay sintetizadores secuenciados.

**Corrección:** conservar el origen LIVE/SEQ por voz o usar buses diferenciados para aplicar los controles de volumen de manera consistente.

### 9. [P2] El final natural de SONG puede dejar notas sostenidas

**Código:** `DaisyPod3/main.cpp:5622–5635`, `:5649–5650`, `:5396–5397`.

Al terminar la cadena se desactiva `dseq.playing`, pero no se llama a `DsqReleaseAllHeldNotes`. El descuento de gates solo sucede dentro del bloque de reproducción. Una nota del último paso con slide puede conservar muestras pendientes —su gate es `samplesPerStep + 2`— y no recibir nunca su NoteOff.

**Reproducción:** terminar una canción con nota 303 con slide y sustain audible; comprobar si continúa tras finalizar SONG. Es una consecuencia del flujo estático pendiente de verificación auditiva.

**Corrección:** realizar una transición de fin que libere notas y cancele pendientes, manteniendo únicamente las colas de efectos deseadas.

### 10. [P2] El comando de swing por pista no tiene efecto

**Código:** `DaisyPod3/main.cpp:1009`, `:1254`, `:9298–9301`.

La búsqueda de todas las referencias a `dsqTrackSwing` encuentra declaración, inicialización y escritura del comando, pero ninguna lectura en reproducción. El motor acepta el ajuste y no lo aplica.

**Corrección:** incorporarlo a la planificación por pista o retirar/anunciar como no soportada esta capacidad del protocolo. No se ha comprobado que la UI actual lo exponga como control independiente.

## UI, UX y optimizaciones propuestas

- **Estado real de sincronización:** mostrar «pendiente», «sincronizando», «confirmado» o «error». Una selección local no acredita que Daisy esté reproduciendo ese contenido; enlaza con el hallazgo 5.
- **Guardar fuera del callback de LVGL:** `ui_screens.cpp:9369–9372` ejecuta el guardado SPIFFS síncronamente. Puede detener la respuesta táctil durante escritura o recolección del sistema de archivos. Usar un trabajador y un indicador de progreso; medir la pausa en placa.
- **Salvapantallas sensible a la interpretación:** `ui_screens.cpp:20182` mide solo el último toque. Puede cubrir la pantalla después de 60 segundos aunque se esté tocando MIDI o usando knobs. Integrar actividad física/MIDI o permitir mantener la vista durante PLAY.
- **Procesamiento seguro entre loop e interrupción:** la cola de audio ya existe para disparos, pero comandos como carga de pasos, cambio de motor y liberación de notas siguen modificando estructuras consumidas por el callback. Extender la propiedad del estado al hilo de audio y publicar snapshots completos al límite de bloque; evitar deshabilitar interrupciones durante cargas grandes.
- **Latencia de controles durante cargas:** mover una carga fuera de LVGL evita congelar el dibujo, pero `ProcessPendingPatternWork` todavía ocupa `control_process`; `main.cpp::loop` atiende pads y controles después. Fraccionar el trabajo por iteración y reservar prioridad a notas/transporte.
- **Optimizar con perfilado:** medir coste por bloque, picos, underruns y latencia percentil 99 con polifonía y FX máximos antes de cambiar algoritmos. Aprovechar el perfilador existente. Separar parámetros calculados al cambiar controles de operaciones por muestra.
- **Mantenibilidad:** dividir los archivos grandes `DaisyPod3/main.cpp` y `P4/src/ui/ui_screens.cpp` por transporte, secuenciación, mezcla, almacenamiento y pantallas. Centralizar serialización y rangos compartidos para impedir divergencias como la del hallazgo 2.

## Validación y límites

- Revisadas las rutas emisor/receptor de los hallazgos y ejecutada la simulación numérica swing/ratchet.
- Intentada la compilación Daisy con `build.ps1`: detenida porque no se encuentra `make`.
- Intentada la compilación P4 con el entorno `esp32p4-upload`: Windows Application Control bloquea `platformio.exe`. No es un resultado de compilación del código.
- No se han flasheado placas, medido tiempos reales, escuchado audio ni validado visualmente la pantalla física.
- No se ha ejecutado el test C existente de conversión WAV24. No se señala esa conversión como defectuosa.

Orden recomendado: corregir mezcla y preservación de pasos/locks; proteger almacenamiento; confirmar cargas completas; corregir temporización y gates; después mejorar UX y medir rendimiento. Añadir regresiones del protocolo y pruebas de audio para estos casos, incluyendo desconexión USB y escritura fallida.
