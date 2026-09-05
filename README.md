# DrumMachine V2 — P4 + DaisyPod3

Proyecto nuevo de batería y sintetizador dividido en dos firmwares:

- `P4`: interfaz completa, secuenciador, patrones, MIDI y host USB.
- `DaisyPod3`: motor de audio RED808 de 64 MB, sintetizadores, sampler, efectos y todos los controles físicos del Daisy Pod.

El único enlace de control entre placas es un cable USB-C de datos:

La versión de protocolo actual es **2.4**: actualizar ambos firmwares juntos.
Las cargas completas de patrón se verifican con checksum y se activan en Daisy
al comenzar un bloque de audio; una carga incompleta conserva el patrón anterior.
Los guardados de usuario usan dos copias verificadas y mantienen compatibilidad
de lectura con los archivos antiguos. Véase [correcciones y rendimiento](CORRECCIONES_Y_RENDIMIENTO.md).

```text
ESP32-P4 (USB host)  <──── USB-C CDC binario ────>  DaisyPod3 (USB device)
```

No se utiliza ESP32-S3. El ESP32-C6 integrado en la pantalla P4 queda sin uso. No hay Wi-Fi, web, HTTP, WebSocket, UDP ni enlace serie entre placas.

## Estructura

```text
DrumMachine_V2_P4_DPod3/
├── P4/          Firmware PlatformIO para Guition JC1060P470C
├── DaisyPod3/   Firmware libDaisy para Daisy Pod + Seed/Seed3
└── shared/      Códigos de comando comunes al enlace USB
```

## Compilación

Clona el repositorio incluyendo sus dependencias:

```powershell
git clone --recurse-submodules https://github.com/cescofors75/DrumMachine-V2-P4-DaisyPod3.git
```

Si ya lo clonaste sin submódulos:

```powershell
git submodule update --init --recursive
```

P4:

```powershell
C:\Users\cesco\.platformio\penv\Scripts\platformio.exe run -d .\P4 -e esp32p4-upload
```

DaisyPod3:

```powershell
.\DaisyPod3\build.ps1
```

El firmware Daisy usa las revisiones fijadas de `libDaisy` y `DaisySP` incluidas
como submódulos bajo `third_party`.

## Conexión

1. Grabar cada firmware en su placa.
2. Alimentar el conjunto conforme al montaje definitivo.
3. Conectar el puerto USB host/OTG del P4 al USB interno del Daisy Seed mediante un cable USB-C con datos.
4. El P4 detecta el dispositivo `0483:5740`, envía un `PING` y resincroniza BPM, mezcla, patrón y estado de transporte.

La pantalla muestra `USB WAIT` hasta enumerar el dispositivo y `DAISY` cuando el motor responde.

## MicroSD y samples

La microSD de **P4 es la única fuente de archivos de audio**. Al enlazar con
DaisyPod3, P4 busca automáticamente los WAV del kit en
`/data/RED 808 KARZ`, los convierte a PCM mono de 48 kHz y carga los 16 pads en
Daisy por el mismo enlace USB-C. La microSD de Daisy no se monta ni interviene
en el arranque.

El estado de la pantalla indica `P4 KIT SCAN`, el progreso `KIT n/16` y
finalmente `READY`. Si una muestra falta o no es válida, esa pista conserva la
voz de síntesis de respaldo para que PLAY no quede mudo.

## Patrones de fábrica

P4 incorpora exactamente el banco final de la versión ESP32-S3 procedente de
`RedMaster_ESP32S3/data/patterns/20_patrones_factory_daisy.json`: 20 patrones,
16 pasos y 107 registros de pista. La sincronización con Daisy incluye pasos,
velocidades, notas, flags, motores, presets, BPM, swing y humanize.

Daisy dispone de 20 slots de ejecución, pero P4 mantiene 128 patrones lógicos.
El firmware traduce ambos espacios sin confundir el número de slot Daisy con
el número de patrón P4, por lo que un cambio o una resincronización USB no
puede sustituir el contenido de `P001`.

Los slots `P101..P128` están reservados para el usuario. En `SEQUENCER > LIST`
pulsa `SAVE USER` y elige el destino: se guarda una copia completa del patrón
actual (steps, velocidades, notas, ratchets, probability, parameter locks,
metadata y perfil de motores/presets). Los archivos viven en el SPIFFS interno
de P4 y se restauran automáticamente al arrancar; no dependen de ninguna SD.

## Importación MIDI desde la SD de P4

Al seleccionar un `.mid`, `PATTERN` conserva la importación rápida de hasta
cuatro compases. `FULL SONG` analiza el arrangement completo en segundo plano,
cuantiza al semicorchea más cercana, conserva velocity y silencios, reutiliza
compases equivalentes y compacta sus repeticiones. Las escenas distintas se
guardan en `P101..P120` y Daisy reproduce una cadena de hasta 128 secciones;
antes de sustituir esos patrones la pantalla pide confirmación.

El mapa de control MIDI externo se consulta desde `STATUS > CONTROL MAP > MIDI
MAP`. La pantalla replica el MPD218 (16 pads con PAD 13-16 arriba y 6 knobs en
filas 5·6 / 3·4 / 1·2) e incluye ambos dispositivos, bancos 1–3, capas A/B/C
separadas para pads y knobs y todas las acciones del mapa compilado de
DaisyPod3. La tecla roja `LEARN` captura el siguiente evento del AKAI (nota o
CC, en cualquier canal) y abre el selector de asignación: samples, 808/909/505,
patrones, mutes, transporte o acciones de knob. Las asignaciones se guardan en
la NVS de P4, se reenvían a Daisy en cada conexión y mandan sobre el mapa de
fábrica; al tocar el pad mapeado del AKAI suena la Daisy y el pad se ilumina
tanto en la pantalla LIVE del P4 como en la vista MPD218.

El importador admite SMF 0/1 con PPQN. `STD` usa el canal GM de batería y `PRO`
convierte todos los canales en las 16 pistas percusivas. Daisy dispone de 20
escenas residentes: si una canción necesita más de 20 compases realmente
distintos o más de 128 secciones, el resumen indica claramente la parte
importada. Si existen varios eventos de tempo, se conserva el primero como BPM
original de referencia.

HOME y SEQUENCER comparan ese BPM original —también el recomendado por los
patrones normales— con el tempo efectivo. Cuando un knob físico o control
digital lo cambia, aparece una flecha, la diferencia con signo y el valor
original; el control físico continúa siendo la autoridad.

## Controles físicos DaisyPod

Pulsa la celda `STATUS` de la pantalla LIVE para abrir `PHYSICAL CONTROL MAP`.
Se pueden asignar los dos botones, los dos knobs, el giro y pulsación del
encoder, y función/color de los dos LED RGB. Cada cambio se envía a Daisy y la
pantalla vuelve a leer la configuración canónica. Los knobs absolutos de Daisy
tienen prioridad sobre los sliders o botones digitales de P4. La configuración
se guarda inmediatamente en NVS de P4 y se restaura en Daisy en cada conexión;
no depende de una microSD en Daisy.

Una función solo puede tener un dueño físico. Cuando ya está asignada deja de
aparecer en los desplegables de los demás controles; para moverla hay que
poner primero el dueño anterior en `NONE`. Las celdas digitales equivalentes
muestran una insignia `HW` mientras tienen un botón, knob, encoder, rotary o
fader físico asignado.

Mapa inicial: encoder `PATTERN +`, pulsación del encoder `PLAY / PAUSE`, knob 1
`MASTER VOL`, knob 2 `TEMPO`, botón 1 `BACK` y botón 2 `BUTTON CONFIG` (abre o
cierra el propio mapa de controles en la pantalla P4). Los cuatro rotaries P4
arrancan como `DELAY MIX`, `REVERB MIX`, `FLANGER DEPTH` y `PHASER DEPTH`.

## Hub I2C y cuatro rotaries P4

Los controles externos usan un bus independiente del táctil. El multiplexor
activo PCA9548A/TCA9548A se conecta al segundo controlador I2C de la P4:

- `D / SDA` del hub -> `GPIO3` de la P4.
- `C / SCL` del hub -> `GPIO4` de la P4.
- `+ / VCC` del hub -> `3V3` de la P4 para la prueba con un solo SEN0502.
- `- / GND` del hub -> `GND` de la P4.

No alimentar a 5 V el bus que entra directamente en GPIO3/4: las resistencias
pull-up de un módulo I2C pueden elevar también SDA/SCL a esa tensión. Para los
cuatro rotaries, si el consumo de los anillos LED supera lo disponible en 3V3,
usar una fuente regulada externa de `3.3 V` con `GND` común; nunca inyectar 5 V
en SDA/SCL.

No conectar el hub a los pines laterales `I2C_SDA/I2C_SCL` (`GPIO7/8`): ese
bus queda reservado exclusivamente al táctil GT911. El hub trabaja mediante
`Wire1` a 100 kHz en una tarea independiente. Los cuatro DFRobot SEN0502 van
en los canales `0`, `1`, `2` y `3`; al estar aislados por canal pueden conservar
todos su dirección de fábrica `0x54`.

El firmware detecta el multiplexor en `0x70..0x77` verificando su registro de
control con dos máscaras distintas y restaurando después todos los canales.
Solo entonces muestra `PCA9548A SIGNATURE OK`; una vez identificado mantiene
esa topología hasta el siguiente arranque para que un fallo I2C aislado no
cambie a modo pasivo.
También mantiene compatibilidad con el DFR0759 pasivo; solo en ese caso los
DIP deben dar direcciones diferentes:

| Rotary | DIP 1 | DIP 2 | Dirección |
|---|---:|---:|---:|
| 1 | 0 | 0 | `0x54` |
| 2 | 0 | 1 | `0x55` |
| 3 | 1 | 0 | `0x56` |
| 4 | 1 | 1 | `0x57` |

Los cuatro arrancan asignados a `DELAY MIX`, `REVERB MIX`, `FLANGER DEPTH` y
`PHASER DEPTH`. Se reasignan directamente desde los desplegables de `STATUS` y
también pueden controlar volumen, tempo, wavefolder, crush, cutoff, resonancia,
distorsión, profundidad de bits, reducción de frecuencia y modelo de filtro.
Una vez asignado y detectado, el valor físico `0..1023` del rotary manda sobre
el control digital equivalente. La prioridad completa es: knob físico
DaisyPod, rotary físico P4 y, por último, pantalla P4.
El estado inferior del modal muestra cuántos rotaries están presentes y sus
cuatro valores en tiempo real.

Los canales `0..3` del PCA9548A/TCA9548A quedan reservados para los cuatro
SEN0502. Los canales `4..7` permanecen libres; los dos arrays M5Stack Unit
8Encoder ya no forman parte del proyecto ni del firmware.

## Mini Fader M5Stack directo a P4

El selector DFRobot de 12 posiciones y el ADS1115 se han eliminado. El ADC real
de `GPIO20`, ya comprobado en esta placa, queda dedicado al Mini Fader M5Stack:

- `5V` del fader → `5V` de P4.
- `GND` del fader → `GND` común.
- `OUT` del fader (salida analógica 0–3,3 V) → `GPIO20` de P4.
- `IN` del fader (entrada de datos de los 14 SK6812) → `GPIO45` de P4.

`STATUS` muestra el ADC bruto de 12 bits y el valor normalizado `0..1023`. El
fader aparece en `PHYSICAL CONTROL MAP`, arranca asignado a `SCREEN BRIGHTNESS`
y su posición física manda sobre el control digital equivalente. Los cuatro
rotaries siguen siendo los únicos dispositivos conectados al PCA9548A.
