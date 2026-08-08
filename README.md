# DrumMachine V2 — P4 + DaisyPod3

Proyecto nuevo de batería y sintetizador dividido en dos firmwares:

- `P4`: interfaz completa, secuenciador, patrones, MIDI y host USB.
- `DaisyPod3`: motor de audio RED808 de 64 MB, sintetizadores, sampler, efectos y todos los controles físicos del Daisy Pod.

El único enlace de control entre placas es un cable USB-C de datos:

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

La microSD de **DaisyPod3 es la fuente de audio oficial**. Daisy monta la tarjeta
al arrancar, busca el kit `/data/RED 808 KARZ` y completa pads ausentes desde
las carpetas de familias bajo `/data`. El contenido listo para copiar está en
`DaisyPod3/data/`: copia esa carpeta completa a la raíz de una tarjeta FAT32.

Si una muestra falta o no es un WAV válido, esa pista recibe automáticamente
una voz 909/505 de respaldo. Por tanto, PLAY nunca debe quedar mudo por una SD
vacía o un pad sin sample. El P4 consulta a Daisy qué pads están realmente
cargados y refleja ese estado; su almacenamiento local queda para importación
MIDI/compatibilidad y no es la verdad del motor de audio.

## Controles físicos DaisyPod

Pulsa la celda `STATUS` de la pantalla LIVE para abrir `PHYSICAL CONTROL MAP`.
Se pueden asignar los dos botones, los dos knobs, el giro y pulsación del
encoder, y función/color de los dos LED RGB. Cada cambio se envía a Daisy y la
pantalla vuelve a leer la configuración canónica. Los knobs absolutos de Daisy
tienen prioridad sobre los sliders o botones digitales de P4. Daisy guarda el
mapa en `/pod_controls.cfg` dentro de su propia microSD y lo recupera al arrancar.
