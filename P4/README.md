# P4

Firmware de control para la pantalla Guition JC1060P470C con ESP32-P4.

Incluye la interfaz táctil completa, pads, secuenciador, banco de patrones, mixer, efectos, piano, edición de sintetizadores, navegador SD y carga de WAV. La lógica que antes residía en un controlador separado ahora se ejecuta localmente mediante `control_api` y las clases `Sequencer`/`PatternBank`.

El P4 funciona como host USB CDC. `daisy_usb_transport` encapsula los comandos binarios, consulta la posición del secuenciador y mantiene el estado de conexión. La SD de P4 es la fuente de samples: al conectar DaisyPod3 se escanea `/data/RED 808 KARZ`, se asigna un WAV a cada familia de pad, se convierte a PCM mono de 48 kHz y se transfiere automáticamente por USB. El navegador conserva además la carga manual de WAV.

El banco de fábrica se genera desde el JSON final de la antigua versión ESP32-S3. Se transmiten a Daisy no sólo los pasos, sino también velocidades, notas, flags, motores, presets, BPM, swing y humanize.

El navegador MIDI ofrece importación `PATTERN` y `FULL SONG`. La segunda crea
escenas persistentes `P101..P120`, deduplica compases con pequeñas variaciones
de velocity y prepara una cadena Daisy de hasta 128 secciones. El parser corre
fuera del hilo LVGL, valida longitudes SMF y muestra límites o truncamiento en
el resumen. El BPM original queda separado del BPM efectivo para poder ver la
diferencia cuando manda un control físico o digital.

En la celda `STATUS`, `CONTROL MAP > MIDI MAP` muestra el mapa de referencia
completo de los dos Akai MPD218: dispositivo, banco/programa, canal MIDI, las
tres capas independientes de pads y knobs, las 16 notas y los 6 CC con su
acción. El dispositivo 1 ocupa los canales 1/2/3 y el 2 los canales 4/5/6; la
pantalla recuerda además que el H4MIDI debe dejar pasar reloj y transporte desde
un único maestro.

No se inicializa el coprocesador inalámbrico de la pantalla y no existe transporte alternativo.

Compilar:

```powershell
C:\Users\cesco\.platformio\penv\Scripts\platformio.exe run -e esp32p4
```
