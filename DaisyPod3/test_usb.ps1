param(
    [ValidateRange(1, 15)]
    [int]$TimeoutSeconds = 4
)

$ErrorActionPreference = 'Stop'
$pio = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\platformio.exe'

if(-not (Test-Path -LiteralPath $pio)) {
    throw "No se encontro PlatformIO: $pio"
}

Write-Host 'Buscando DaisyPod3 USB CDC (0483:5740)...' -ForegroundColor Cyan
$json = & $pio device list --json-output
if($LASTEXITCODE -ne 0) {
    throw 'PlatformIO no pudo enumerar los puertos serie.'
}

$devices = @($json | ConvertFrom-Json)
$candidates = @($devices | Where-Object {
    ([string]$_.hwid) -match 'VID:PID=0483:5740'
})

if($candidates.Count -eq 0) {
    Write-Host 'No se detecta DaisyPod3 como USB CDC.' -ForegroundColor Red
    Write-Host 'Conecta Daisy directamente al PC y pulsa RESET sin BOOT.' `
        -ForegroundColor Yellow
    exit 1
}

if($candidates.Count -gt 1) {
    Write-Host 'Hay varias Daisy CDC; se probara la primera:' -ForegroundColor Yellow
    $candidates | ForEach-Object {
        Write-Host "  $($_.port)  $($_.description)  [$($_.hwid)]"
    }
}

$device = $candidates[0]
$portName = [string]$device.port
Write-Host "DaisyPod3 detectada en $portName" -ForegroundColor Green

$serial = [System.IO.Ports.SerialPort]::new($portName, 115200, 'None', 8, 'One')
$serial.ReadTimeout = 100
$serial.WriteTimeout = 1000
$serial.DtrEnable = $true
$serial.RtsEnable = $true
$received = [System.Collections.Generic.List[byte]]::new()
$pongOk = $false
$statusOk = $false

try {
    $serial.Open()
    Start-Sleep -Milliseconds 250
    $serial.DiscardInBuffer()

    # RED808 PING: packed 8-byte header + uint32 payload. Daisy deliberately
    # accepts PING without validating the incoming CRC.
    [byte[]]$ping = @(
        0xA5, 0xEE,             # magic, command
        0x04, 0x00,             # payload length = 4
        0x34, 0x12,             # sequence = 0x1234
        0x00, 0x00,             # checksum (ignored for PING)
        0x44, 0x4D, 0x56, 0x32  # "DMV2"
    )
    $serial.Write($ping, 0, $ping.Length)
    Write-Host 'PING binario enviado; esperando PONG...' -ForegroundColor Cyan

    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
    while([DateTime]::UtcNow -lt $deadline) {
        $available = $serial.BytesToRead
        if($available -gt 0) {
            [byte[]]$chunk = New-Object byte[] $available
            $count = $serial.Read($chunk, 0, $chunk.Length)
            for($i = 0; $i -lt $count; ++$i) { $received.Add($chunk[$i]) }
        }

        $bytes = $received.ToArray()
        for($offset = 0; $offset + 8 -le $bytes.Length; ++$offset) {
            if($bytes[$offset] -ne 0x5A) { continue }
            $payloadLength = [int]$bytes[$offset + 2] -bor `
                ([int]$bytes[$offset + 3] -shl 8)
            $packetLength = 8 + $payloadLength
            if($offset + $packetLength -gt $bytes.Length) { continue }
            $command = $bytes[$offset + 1]
            $sequence = [int]$bytes[$offset + 4] -bor `
                ([int]$bytes[$offset + 5] -shl 8)
            if($command -eq 0xEE -and $sequence -eq 0x1234) {
                $pongOk = $true
                break
            }
        }
        if($pongOk) { break }
        Start-Sleep -Milliseconds 25
    }

    if($pongOk) {
        Write-Host 'PONG correcto: DaisyPod3 USB y protocolo funcionan.' `
            -ForegroundColor Green

        $received.Clear()
        $serial.DiscardInBuffer()
        # GET_STATUS has an empty payload, therefore checksum is zero.
        [byte[]]$getStatus = @(
            0xA5, 0xE0,             # magic, command
            0x00, 0x00,             # payload length = 0
            0x35, 0x12,             # sequence = 0x1235
            0x00, 0x00              # checksum
        )
        $serial.Write($getStatus, 0, $getStatus.Length)
        Write-Host 'Leyendo estado de SD y samples...' -ForegroundColor Cyan
        $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
        while([DateTime]::UtcNow -lt $deadline -and -not $statusOk) {
            $available = $serial.BytesToRead
            if($available -gt 0) {
                [byte[]]$chunk = New-Object byte[] $available
                $count = $serial.Read($chunk, 0, $chunk.Length)
                for($i = 0; $i -lt $count; ++$i) { $received.Add($chunk[$i]) }
            }

            $bytes = $received.ToArray()
            for($offset = 0; $offset + 8 -le $bytes.Length; ++$offset) {
                if($bytes[$offset] -ne 0x5A -or $bytes[$offset + 1] -ne 0xE0) { continue }
                $payloadLength = [int]$bytes[$offset + 2] -bor `
                    ([int]$bytes[$offset + 3] -shl 8)
                if($payloadLength -lt 76 -or $offset + 8 + $payloadLength -gt $bytes.Length) { continue }
                $payload = $offset + 8
                $liveMask = [int]$bytes[$payload + 2] -bor `
                    ([int]$bytes[$payload + 3] -shl 8)
                $liveCount = 0
                for($bit = 0; $bit -lt 16; ++$bit) {
                    if(($liveMask -band (1 -shl $bit)) -ne 0) { $liveCount++ }
                }
                $kitBytes = New-Object byte[] 32
                [Array]::Copy($bytes, $payload + 14, $kitBytes, 0, 32)
                $kit = [Text.Encoding]::ASCII.GetString($kitBytes).Trim([char]0)
                if([string]::IsNullOrWhiteSpace($kit)) { $kit = '--' }
                $sd = $bytes[$payload + 8] -ne 0
                $xtraMask = $bytes[$payload + 9]
                Write-Host ("SD Daisy: {0} | LIVE WAV: {1}/16 | XTRA mask: 0x{2:X2} | KIT: {3}" -f `
                    $(if($sd){'OK'}else{'NO'}), $liveCount, $xtraMask, $kit) `
                    -ForegroundColor $(if($sd){'Green'}else{'Red'})
                if($payloadLength -ge 80) {
                    $mount = $bytes[$payload + 76]
                    $root = $bytes[$payload + 77]
                    $boot = $bytes[$payload + 78]
                    $fail = $bytes[$payload + 79]
                    Write-Host "Diagnostico FatFS: mount=$mount  /data=$root  boot=$boot  wav_errors=$fail"
                    Write-Host 'Codigos: 0=OK, 3=NOT_READY, 4=NO_FILE, 5=NO_PATH, 13=NO_FILESYSTEM.' `
                        -ForegroundColor DarkGray
                }
                $statusOk = $true
                break
            }
            Start-Sleep -Milliseconds 25
        }
    }
}
finally {
    if($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
}

if($pongOk -and $statusOk) { exit 0 }
if($pongOk) {
    Write-Host 'USB funciona, pero Daisy no devolvio GET_STATUS.' -ForegroundColor Yellow
    exit 3
}

if($received.Count -gt 0) {
    $previewCount = [Math]::Min($received.Count, 64)
    $preview = ($received.GetRange(0, $previewCount) | ForEach-Object {
        $_.ToString('X2')
    }) -join ' '
    Write-Host "Daisy envio $($received.Count) bytes, pero no un PONG valido:" `
        -ForegroundColor Red
    Write-Host "  $preview"
} else {
    Write-Host 'Daisy enumera como CDC, pero no responde ningun byte.' `
        -ForegroundColor Red
}
Write-Host 'El firmware Daisy no llega al bucle USB o no es la imagen DrumMachineV2.' `
    -ForegroundColor Yellow
exit 2
