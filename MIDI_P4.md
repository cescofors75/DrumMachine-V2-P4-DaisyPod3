# Migración DIN MIDI a P4

La nueva ruta de entrada es placa MIDI IN → UART1 P4 (31250, 8N1) →
USB CDC `CMD_MIDI_INPUT` → motor Daisy. Conserva los mapas MPD218,
asignaciones de usuario, LEARN y monitor existentes en Daisy.

## Estado de esta preparación

La entrada P4 queda desactivada hasta definir `P4_MIDI_RX_GPIO` en los
build_flags de PlatformIO. No se han deducido pines ni alimentación a partir
de las fotos. `P4_MIDI_TX_GPIO` también queda en -1: todavía no se genera
MIDI OUT. Verificar modelo/esquema de la placa y adaptación a lógica de
3,3 V antes de conectarla a los GPIO.

Una vez confirmado el montaje, compilar Daisy con
`DAISY_LEGACY_MIDI_UART=0` para retirar la antigua entrada D14. Por ahora
se conserva para que la configuración sin pines P4 no deje al instrumento
sin entrada MIDI. No conectar simultáneamente el mismo flujo a ambas entradas.

La nueva capacidad `RED808_CAP_MIDI_INPUT` permite que P4 detecte si Daisy
acepta esta ruta. Si no la anuncia, o se desconecta, se descarta la entrada
sin reproducir notas antiguas al reconectar. Hay un buffer UART de 2048 bytes
y reintento de mensajes completos ante saturación de la cola USB; una parada
prolongada aún puede agotar el buffer UART.

El parser admite running status, realtime intercalado, mensajes de canal y
note-on con velocidad cero. SysEx y system-common no se encaminan. Los
mensajes sin acción en el mapa existente no adquieren nuevas funciones.

## Pendiente para maestro de reloj

Esta ruta mueve la recepción MIDI; **no traslada el reloj del secuenciador**.
Daisy sigue temporizando en audio. Falta concretar si P4 debe generar
Clock de 24 PPQN y Start/Continue/Stop por OUT y cómo sincronizar la fase
del secuenciador Daisy. No se debe generar un segundo reloj libre desde
el bucle de pantalla y dar por sincronizados ambos motores.

## Protocolo

`CMD_MIDI_INPUT = 0xEA`, dirección P4 → Daisy, payload de 1–64 triples
`{status, data0, data1}`, con status explícito y datos de 7 bits. Daisy
valida el paquete completo antes de ejecutarlo. Capacidad PONG `0x0008`.
Los eventos de monitor vuelven por el comando existente `0xE8`.

Pruebas: `tools/tests/midi_input_regression.cpp`. La compilación y las pruebas
de parser no sustituyen la verificación de cableado, latencia o reloj físico.
