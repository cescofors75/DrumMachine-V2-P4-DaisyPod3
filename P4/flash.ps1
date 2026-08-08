param(
    [string]$Port,
    [switch]$DetectOnly
)

$ErrorActionPreference = 'Stop'
$projectDir = $PSScriptRoot
$pio = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\platformio.exe'
$python = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\python.exe'

if(-not (Test-Path -LiteralPath $pio)) {
    throw "No se encontro PlatformIO: $pio"
}

Write-Host 'Buscando ESP32-P4 conectado...' -ForegroundColor Cyan
$json = & $pio device list --json-output
if($LASTEXITCODE -ne 0) {
    throw 'PlatformIO no pudo enumerar los puertos serie.'
}
$devices = @($json | ConvertFrom-Json)

function Test-P4Port([string]$ComPort) {
    try {
        $result = & $python -m esptool --port $ComPort chip_id 2>&1 |
            Out-String
        return $result -match 'ESP32-P4'
    }
    catch {
        return $false
    }
}

if($Port) {
    $selected = $devices | Where-Object { $_.port -eq $Port } |
        Select-Object -First 1
    if(-not $selected) {
        throw "El puerto solicitado $Port no esta conectado."
    }
}
else {
    $candidates = @($devices | Where-Object {
        $hwid = [string]$_.hwid
        $description = [string]$_.description
        # 0012 = ROM download mode; 1001 = running USB Serial/JTAG app.
        # Recognize both directly: probing 0012 with chip_id resets the USB
        # device and can leave PlatformIO holding a stale COM port.
        $hwid -match 'VID:PID=303A:(0012|1001)' -or
        $description -match 'ESP32-P4|\bP4\b|USB JTAG'
    })

    if($candidates.Count -eq 1) {
        $selected = $candidates[0]
    }
    elseif($candidates.Count -gt 1) {
        foreach($candidate in $candidates) {
            Write-Host "Verificando $($candidate.port)..." -ForegroundColor DarkGray
            if(Test-P4Port $candidate.port) {
                $selected = $candidate
                break
            }
        }
    }

    if(-not $selected) {
        foreach($device in $devices) {
            Write-Host "Probando $($device.port) ($($device.description))..." `
                -ForegroundColor DarkGray
            if(Test-P4Port $device.port) {
                $selected = $device
                break
            }
        }
    }
}

if(-not $selected) {
    Write-Host 'No se detecta ningun ESP32-P4 ni ningun puerto COM compatible.' `
        -ForegroundColor Red
    if($devices.Count -gt 0) {
        Write-Host 'Puertos presentes:' -ForegroundColor Yellow
        $devices | ForEach-Object {
            Write-Host "  $($_.port)  $($_.description)  [$($_.hwid)]"
        }
    }
    Write-Host ''
    Write-Host 'Pon el P4 en modo descarga: mantén BOOT, pulsa RESET,' `
        -ForegroundColor Yellow
    Write-Host 'suelta RESET y después suelta BOOT. Luego repite la tarea.' `
        -ForegroundColor Yellow
    exit 1
}

Write-Host "ESP32-P4 detectado en $($selected.port)" -ForegroundColor Green
Write-Host "  $($selected.description)  [$($selected.hwid)]"
if($DetectOnly) { exit 0 }

Write-Host 'Flasheando DrumMachineV2 P4 con no-stub...' -ForegroundColor Cyan
& $pio run `
    --project-dir $projectDir `
    --environment esp32p4-upload `
    --target upload `
    --upload-port $selected.port
exit $LASTEXITCODE
