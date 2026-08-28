param(
    [switch]$SkipBuild,
    [switch]$Clean,
    [switch]$SkipClean,
    [ValidateRange(10, 180)]
    [int]$TimeoutSeconds = 45
)

$ErrorActionPreference = 'Stop'
$projectDir = $PSScriptRoot
$repoDir = (Resolve-Path (Join-Path $projectDir '..')).Path
$buildScript = Join-Path $projectDir 'build.ps1'
$bootloaderDir = Join-Path $repoDir 'third_party\libDaisy\core'
$bootloaders = @(
    (Join-Path $bootloaderDir 'dsy_bootloader_v6_4-intdfu-2000ms.bin'),
    (Join-Path $bootloaderDir 'dsy_bootloader_v6_4-extdfu-2000ms.bin')
)
$firmware = Join-Path $projectDir 'build\DrumMachineV2_DaisyPod3.bin'

if(-not $SkipBuild) {
    Write-Host 'Compilando DrumMachineV2 DaisyPod3...' -ForegroundColor Cyan
    $buildArgs = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $buildScript)
    if($Clean -and -not $SkipClean) { $buildArgs += '-Clean' }
    else {
        Write-Host 'Build incremental de DaisyPod3 (usa -Clean para reconstruir todo).' `
            -ForegroundColor DarkGray
    }
    & powershell.exe @buildArgs
    if($LASTEXITCODE -ne 0) {
        throw "La compilacion de DaisyPod3 fallo con codigo $LASTEXITCODE"
    }
}

if(-not (Test-Path -LiteralPath $firmware)) {
    throw "No existe el firmware: $firmware"
}

$dfuCandidates = @(
    'C:\Espressif\tools\dfu-util\0.11\dfu-util-0.11-win64\dfu-util.exe',
    'C:\msys64\mingw64\bin\dfu-util.exe',
    'C:\msys64\usr\bin\dfu-util.exe'
)
$dfuCommand = Get-Command dfu-util.exe -ErrorAction SilentlyContinue
if($dfuCommand) { $dfuCandidates += $dfuCommand.Source }
$dfu = $dfuCandidates | Where-Object { Test-Path -LiteralPath $_ } |
    Select-Object -First 1
if(-not $dfu) {
    throw 'No se encontro dfu-util 0.11 para flashear DaisyPod3.'
}

$token = [Guid]::NewGuid().ToString('N')
$stdoutFile = Join-Path $env:TEMP "drummachinev2-dfu-$token.out"
$stderrFile = Join-Path $env:TEMP "drummachinev2-dfu-$token.err"
$dfuArgs = @(
    '-w', '-a', '0',
    '-s', '0x90040000:leave',
    '-D', $firmware,
    '-d', '0483:df11'
)

try {
    Write-Host ''
    Write-Host 'DAISYPOD3: pulsa RESET una vez, SIN mantener BOOT.' -ForegroundColor Yellow
    Write-Host "Esperando el bootloader DFU durante $TimeoutSeconds segundos..." -ForegroundColor Cyan

    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
    $dfuDevices = ''
    while([DateTime]::UtcNow -lt $deadline) {
        $dfuDevices = (& $dfu -l -d '0483:df11' 2>&1 | Out-String)
        if($dfuDevices -match 'Found DFU:') { break }
        Start-Sleep -Milliseconds 200
    }

    if($dfuDevices -notmatch 'Found DFU:') {
        throw "Timeout: DaisyPod3 no entro en el bootloader DFU. Pulsa RESET sin BOOT y repite la tarea."
    }
    if($dfuDevices -notmatch '0x900[0-9A-Fa-f]+') {
        if($dfuDevices -notmatch '@Internal Flash\s+/0x08000000') {
            throw "Se detecto un dispositivo DFU desconocido; no se escribira en el. Dispositivo detectado:`n$($dfuDevices.Trim())"
        }

        Write-Host 'DFU ROM de STM32 detectado. Restaurando bootloader Daisy v6.4...' -ForegroundColor Cyan
        $bootloaderInstalled = $false
        foreach($bootloader in $bootloaders) {
            if(-not (Test-Path -LiteralPath $bootloader)) { continue }
            Write-Host "Probando $([System.IO.Path]::GetFileName($bootloader))..." -ForegroundColor Cyan
            # Keep the STM32 in ROM DFU after writing. With :leave the device
            # resets immediately and Windows removes it before dfu-util can
            # read the final status, producing "Error during download
            # get_status" even though all bytes were written successfully.
            $bootOutput = (& $dfu -a 0 -s '0x08000000' -D $bootloader -d '0483:df11' 2>&1 | Out-String)
            $bootExitCode = $LASTEXITCODE
            if($bootOutput) { Write-Host $bootOutput.Trim() }
            $bootDownloadComplete =
                $bootOutput -match 'Download done' -and
                $bootOutput -match 'File downloaded successfully'
            if($bootDownloadComplete) {
                $bootloaderInstalled = $true
                break
            }
            Write-Host "El bootloader no se completo (dfu-util=$bootExitCode)." `
                -ForegroundColor DarkYellow
        }
        if(-not $bootloaderInstalled) {
            throw 'No se pudo instalar el bootloader Daisy v6.4 desde el DFU ROM.'
        }

        Write-Host 'Bootloader instalado. Suelta BOOT y pulsa RESET una vez.' -ForegroundColor Yellow
        Write-Host 'Esperando la interfaz QSPI del bootloader Daisy...' -ForegroundColor Cyan
        $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
        do {
            Start-Sleep -Milliseconds 200
            $dfuDevices = (& $dfu -l -d '0483:df11' 2>&1 | Out-String)
        } while($dfuDevices -notmatch '0x900[0-9A-Fa-f]+' -and [DateTime]::UtcNow -lt $deadline)

        if($dfuDevices -notmatch '0x900[0-9A-Fa-f]+') {
            throw 'El bootloader se instalo, pero no aparecio su interfaz QSPI. Suelta BOOT, pulsa RESET y repite la tarea.'
        }
        Write-Host 'Bootloader Daisy QSPI detectado.' -ForegroundColor Green
    }

    $process = Start-Process -FilePath $dfu -ArgumentList $dfuArgs `
        -PassThru -WindowStyle Hidden `
        -RedirectStandardOutput $stdoutFile `
        -RedirectStandardError $stderrFile

    # El dispositivo ya fue detectado arriba. NO pedir otro RESET aquí: al
    # hacerlo mientras dfu-util ejecuta ERASE_PAGE Windows corta el endpoint
    # USB y devuelve LIBUSB_ERROR_IO (codigo 74).
    Write-Host 'Flasheando QSPI... NO pulses RESET ni desconectes el USB.' -ForegroundColor Yellow
    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
    while(-not $process.HasExited -and [DateTime]::UtcNow -lt $deadline) {
        Start-Sleep -Milliseconds 200
        $process.Refresh()
    }

    if(-not $process.HasExited) {
        Stop-Process -Id $process.Id -Force -ErrorAction SilentlyContinue
        $process.WaitForExit()
        throw "Timeout: DaisyPod3 no entro en el bootloader DFU. Pulsa RESET sin BOOT y repite la tarea."
    }

    $stdout = if(Test-Path -LiteralPath $stdoutFile) {
        Get-Content -LiteralPath $stdoutFile -Raw
    } else { '' }
    $stderr = if(Test-Path -LiteralPath $stderrFile) {
        Get-Content -LiteralPath $stderrFile -Raw
    } else { '' }
    if($stdout) { Write-Host $stdout.Trim() }
    if($stderr) { Write-Host $stderr.Trim() }

    $combinedOutput = "$stdout`n$stderr"
    $downloadComplete =
        $combinedOutput -match 'Download done' -and
        $combinedOutput -match 'File downloaded successfully'

    if($process.ExitCode -ne 0 -and -not $downloadComplete) {
        if($combinedOutput -match 'ERASE_PAGE|LIBUSB_ERROR_IO') {
            throw "Se interrumpio el borrado DFU (codigo $($process.ExitCode)). Vuelve a ejecutar la tarea, pulsa RESET solo cuando lo pide al principio y NO lo pulses durante 'Flasheando QSPI'."
        }
        throw "dfu-util fallo con codigo $($process.ExitCode). Comprueba el cable USB-C y que el bootloader Daisy este instalado."
    }
    if($process.ExitCode -ne 0 -and $downloadComplete) {
        Write-Host 'La descarga termino; Windows perdio DFU al reiniciar Daisy (:leave). Se acepta como correcta.' -ForegroundColor DarkYellow
    }

    Write-Host 'DrumMachineV2 DaisyPod3 flasheado correctamente.' -ForegroundColor Green
}
finally {
    Remove-Item -LiteralPath $stdoutFile -Force -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $stderrFile -Force -ErrorAction SilentlyContinue
}
