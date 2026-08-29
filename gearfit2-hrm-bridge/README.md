# HRM Viability — Gear Fit2 (fase 1)

App Tizen Wearable minima: lee el sensor HRM y muestra el BPM en pantalla.
Objetivo unico: validar si el toolchain (SDK, certificados, instalacion en el
reloj) funciona en un Gear Fit2 (2016) antes de invertir mas tiempo.

## Que hace

- `config.xml`: manifest Tizen, perfil `wearable`, requiere el privilegio
  `http://tizen.org/privilege/healthinfo` y la feature
  `sensor.heart_rate_monitor`.
- `index.html` + `js/main.js`: arrancan `tizen.humanactivitymonitor` en modo
  `HRM` y pintan `heartRate` en pantalla grande.

## Pasos para probarlo (fuera de este entorno, requieren el reloj físico)

1. Instalar **Tizen Studio** con el paquete "Wearable" (SDK 2.3.1 / 2.3.2,
   el que soporte Gear Fit2). Las versiones antiguas de Tizen Studio pueden
   no estar ya en el listado por defecto del Package Manager — si falta,
   añadir manualmente el repositorio de imagenes wearable antiguas.
2. Generar un certificado **author** + **distributor** (perfil Samsung) desde
   Tizen Studio → Certificate Manager. Esto requiere una cuenta Samsung y
   acceso al Samsung Certificate Portal — es el paso con más riesgo de
   bloqueo (el portal ha tenido caidas intermitentes). Hacer esto primero,
   antes de nada más, como prueba de humo.
3. Activar el modo desarrollador en el Gear Fit2 (Ajustes → Info del reloj,
   tocar la versión varias veces) y conectar por Wi-Fi con la IP que muestra
   el reloj:
   ```
   sdb connect <ip-del-reloj>:26101
   sdb devices
   ```
4. Compilar y desplegar:
   ```
   tizen build-web
   tizen package -t wgt -s <perfil-de-firma>
   sdb install HrmViability.wgt
   ```
5. Lanzar la app desde el reloj. Si el sensor no da lectura, ajustar la
   correa (el HRM óptico necesita buen contacto con la piel).

## Criterio de corte

Si en una tarde de trabajo se consigue ver el propio BPM en pantalla,
la fase 1 está validada y se pasa a sacar el dato por red (fase 2). Si el
toolchain (paso 1 o 2) bloquea sin salida clara en ese tiempo, no continuar.
