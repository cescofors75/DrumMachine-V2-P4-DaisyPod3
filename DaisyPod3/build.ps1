param(
    [switch]$Clean,
    [switch]$VerboseBuild,
    [string]$ToolchainBin,
    [string]$MakeBin
)

$ErrorActionPreference = 'Stop'
$projectDir = $PSScriptRoot
$repoDir = (Resolve-Path (Join-Path $projectDir '..')).Path
$dependencyDirs = @(
    (Join-Path $repoDir 'third_party\libDaisy'),
    (Join-Path $repoDir 'third_party\DaisySP'),
    (Join-Path $repoDir 'third_party\DaisySP\DaisySP-LGPL')
)
$dependencyLibraries = @(
    (Join-Path $dependencyDirs[0] 'build\libdaisy.a'),
    (Join-Path $dependencyDirs[1] 'build\libdaisysp.a'),
    (Join-Path $dependencyDirs[2] 'build\libdaisysp-lgpl.a')
)

function Find-FirstExistingFile {
    param([string[]]$Patterns)

    foreach($pattern in $Patterns) {
        $match = Get-ChildItem -Path $pattern -ErrorAction SilentlyContinue |
            Select-Object -First 1
        if($match) { return $match.FullName }
    }
    return $null
}

function Resolve-ToolDir {
    param(
        [string]$ExplicitDir,
        [string[]]$ExecutablePatterns,
        [string[]]$CommandNames,
        [string]$FriendlyName
    )

    if($ExplicitDir) {
        foreach($commandName in $CommandNames) {
            if(Test-Path (Join-Path $ExplicitDir $commandName)) {
                return (Resolve-Path $ExplicitDir).Path
            }
        }
        throw "$FriendlyName no encontrado en: $ExplicitDir"
    }

    foreach($commandName in $CommandNames) {
        $fromPath = Get-Command $commandName -ErrorAction SilentlyContinue
        if($fromPath) { return Split-Path $fromPath.Source -Parent }
    }

    $executable = Find-FirstExistingFile $ExecutablePatterns
    if($executable) { return Split-Path $executable -Parent }

    throw "$FriendlyName no encontrado. Instala STM32CubeIDE o pasa la ruta correspondiente."
}

$armPatterns = @(
    'C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*\tools\bin\arm-none-eabi-gcc.exe',
    'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*\tools\bin\arm-none-eabi-gcc.exe',
    'C:\Program Files*\Arm GNU Toolchain arm-none-eabi\*\bin\arm-none-eabi-gcc.exe'
)
$makePatterns = @(
    'C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.*\tools\bin\make.exe',
    'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.*\tools\bin\make.exe',
    'C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.*\tools\bin\mingw32-make.exe'
)

$gccDir = Resolve-ToolDir -ExplicitDir $ToolchainBin -ExecutablePatterns $armPatterns `
    -CommandNames @('arm-none-eabi-gcc.exe') -FriendlyName 'arm-none-eabi-gcc'
$makeDir = Resolve-ToolDir -ExplicitDir $MakeBin -ExecutablePatterns $makePatterns `
    -CommandNames @('make.exe', 'mingw32-make.exe') -FriendlyName 'make'
$env:PATH = "$gccDir;$makeDir;$env:PATH"

$makeCommand = Get-Command make.exe -ErrorAction SilentlyContinue
if(-not $makeCommand) {
    $makeCommand = Get-Command mingw32-make.exe -ErrorAction SilentlyContinue
}
if(-not $makeCommand) { throw 'make.exe/mingw32-make.exe no disponible.' }

foreach($dependencyDir in $dependencyDirs) {
    if(-not (Test-Path (Join-Path $dependencyDir 'Makefile'))) {
        throw 'Faltan submodulos. Ejecuta: git submodule update --init --recursive'
    }
}

Write-Host "Daisy Project: $projectDir" -ForegroundColor Cyan
Write-Host "ARM GCC:       $gccDir" -ForegroundColor Cyan
Write-Host "Make:          $($makeCommand.Source)" -ForegroundColor Cyan

for($index = 0; $index -lt $dependencyDirs.Count; $index++) {
    if($Clean -or -not (Test-Path -LiteralPath $dependencyLibraries[$index])) {
        $dependencyName = Split-Path $dependencyDirs[$index] -Leaf
        Write-Host "Compilando dependencia $dependencyName..." -ForegroundColor Cyan
        Push-Location $dependencyDirs[$index]
        try {
            if($Clean) {
                & $makeCommand.Source clean
                if($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
            }
            for($attempt = 1; $attempt -le 3; $attempt++) {
                & $makeCommand.Source
                if($LASTEXITCODE -eq 0 -and
                    (Test-Path -LiteralPath $dependencyLibraries[$index])) {
                    break
                }
                if($attempt -lt 3) {
                    Write-Host "Reintentando $dependencyName ($($attempt + 1)/3)..." `
                        -ForegroundColor Yellow
                }
            }
            if(-not (Test-Path -LiteralPath $dependencyLibraries[$index])) {
                throw "No se pudo generar $($dependencyLibraries[$index])"
            }
        }
        finally {
            Pop-Location
        }
    }
}

Push-Location $projectDir
try {
    if($Clean) {
        & $makeCommand.Source clean
        if($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    }

    $makeArgs = @()
    if($VerboseBuild) { $makeArgs += 'VERBOSE=1' }
    & $makeCommand.Source @makeArgs
    exit $LASTEXITCODE
}
finally {
    Pop-Location
}
