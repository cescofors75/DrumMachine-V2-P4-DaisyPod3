# P4

Firmware de control para la pantalla Guition JC1060P470C con ESP32-P4.

Incluye la interfaz táctil completa, pads, secuenciador, banco de patrones, mixer, efectos, piano, edición de sintetizadores, navegador SD y carga de WAV. La lógica que antes residía en un controlador separado ahora se ejecuta localmente mediante `control_api` y las clases `Sequencer`/`PatternBank`.

El P4 funciona como host USB CDC. `daisy_usb_transport` encapsula los comandos binarios, consulta la posición del secuenciador y mantiene el estado de conexión. Los WAV seleccionados en SD se decodifican, convierten a PCM mono de 48 kHz y se transfieren directamente a DaisyPod3 por USB.

No se inicializa el coprocesador inalámbrico de la pantalla y no existe transporte alternativo.

Compilar:

```powershell
C:\Users\cesco\.platformio\penv\Scripts\platformio.exe run -e esp32p4
```

