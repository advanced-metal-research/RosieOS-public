# Official Windows helper for Offline Programming V1.
# Ensures the Vite UI on 127.0.0.1:5189, cross-compiles the Linux OLP binary
# if WSL has no Go, then invokes WSL Ubuntu-22.04 to start Motion Server +
# pixi Tesseract via start-offline-programming.sh (API 127.0.0.1:8794).
# Does not enable connected execution / physical arming.
$ErrorActionPreference = "Stop"
$script:OlpLinuxRebuilt = $false

$Distro = if ($env:WSL_DISTRO) { $env:WSL_DISTRO } else { "Ubuntu-22.04" }
$OlpDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$UiPort = 5189
$ApiPort = 8794
$AppUrl = "http://127.0.0.1:$UiPort/offline-programming/v1/ui/"
$NodeHome = "C:\Program Files\nodejs"
if (Test-Path $NodeHome) {
    $env:Path = "$NodeHome;" + $env:Path
}

function ConvertTo-WslPath([string]$WinPath) {
    $resolved = (Resolve-Path $WinPath).Path
    $drive = $resolved.Substring(0, 1).ToLowerInvariant()
    $rest = $resolved.Substring(2).Replace("\", "/")
    return "/mnt/$drive$rest"
}

function Test-Http([string]$Url) {
    try {
        $null = Invoke-WebRequest -Uri $Url -UseBasicParsing -TimeoutSec 2
        return $true
    } catch {
        return $false
    }
}

function Ensure-OlpLinuxBinary {
    $linuxBin = Join-Path $OlpDir "bin\offline-programming-linux"
    $goFiles = @(Get-ChildItem -Path $OlpDir -Filter *.go -File) + @(Get-ChildItem -Path (Join-Path $OlpDir "internal") -Filter *.go -File -Recurse -ErrorAction SilentlyContinue)
    $newestGo = $goFiles | Sort-Object LastWriteTime -Descending | Select-Object -First 1
    $needsBuild = -not (Test-Path $linuxBin)
    if (-not $needsBuild -and $newestGo -and (Get-Item $linuxBin).LastWriteTime -lt $newestGo.LastWriteTime) {
        $needsBuild = $true
        Write-Host "olp-start: Go sources newer than linux binary; rebuilding so jog uses motion-server/v1"
    }
    if (-not $needsBuild) {
        Write-Host "olp-start: OLP linux binary present: $linuxBin"
        return
    }
    if (-not (Get-Command go -ErrorAction SilentlyContinue)) {
        throw "OLP linux binary missing and Windows Go is unavailable. Install Go, then re-run this script."
    }
    Write-Host "olp-start: cross-compiling offline-programming-linux for WSL"
    $binDir = Join-Path $OlpDir "bin"
    if (-not (Test-Path $binDir)) {
        New-Item -ItemType Directory -Path $binDir | Out-Null
    }
    $previousGoos = $env:GOOS
    $previousGoarch = $env:GOARCH
    $previousCgo = $env:CGO_ENABLED
    try {
        $env:GOOS = "linux"
        $env:GOARCH = "amd64"
        $env:CGO_ENABLED = "0"
        Push-Location $OlpDir
        & go build -o $linuxBin .
        if ($LASTEXITCODE -ne 0) {
            throw "go build of offline-programming-linux failed"
        }
        $script:OlpLinuxRebuilt = $true
    } finally {
        Pop-Location
        if ($null -eq $previousGoos) { Remove-Item Env:GOOS -ErrorAction SilentlyContinue } else { $env:GOOS = $previousGoos }
        if ($null -eq $previousGoarch) { Remove-Item Env:GOARCH -ErrorAction SilentlyContinue } else { $env:GOARCH = $previousGoarch }
        if ($null -eq $previousCgo) { Remove-Item Env:CGO_ENABLED -ErrorAction SilentlyContinue } else { $env:CGO_ENABLED = $previousCgo }
    }
}

function Ensure-ViteUi {
    if (Test-Http $AppUrl) {
        Write-Host "olp-start: reusing Vite UI at $AppUrl"
        return
    }
    $uiDir = Join-Path $OlpDir "ui"
    $nodeModules = Join-Path $uiDir "node_modules"
    if (-not (Test-Path $nodeModules)) {
        if (-not (Get-Command npm -ErrorAction SilentlyContinue)) {
            throw "ui/node_modules is missing and npm is unavailable. Next: add Node to PATH, then npm install in offline-programming\v1\ui"
        }
        Write-Host "olp-start: npm install in ui/"
        Push-Location $uiDir
        try {
            & npm install
            if ($LASTEXITCODE -ne 0) { throw "npm install failed" }
        } finally {
            Pop-Location
        }
    }
    if (-not (Get-Command go -ErrorAction SilentlyContinue)) {
        throw "Vite UI is down and Go is unavailable. Next: go run . ui --host 127.0.0.1 --port 5189 from offline-programming\v1"
    }
    Write-Host "olp-start: starting Vite UI on 127.0.0.1:$UiPort"
    $nodePath = $env:Path
    Start-Process -FilePath "powershell.exe" -WorkingDirectory $OlpDir -WindowStyle Minimized -ArgumentList @(
        "-NoProfile",
        "-Command",
        "`$env:Path = '$nodePath'; Set-Location -LiteralPath '$OlpDir'; go run . ui --host 127.0.0.1 --port $UiPort"
    ) | Out-Null
    $deadline = (Get-Date).AddSeconds(45)
    while ((Get-Date) -lt $deadline) {
        if (Test-Http $AppUrl) {
            Write-Host "olp-start: Vite UI ready at $AppUrl"
            return
        }
        Start-Sleep -Seconds 1
    }
    throw "Vite UI did not become ready at $AppUrl"
}

Write-Host "olp-start: Windows helper; WSL distro $Distro"
Ensure-OlpLinuxBinary
Ensure-ViteUi

$wslScript = ConvertTo-WslPath (Join-Path $OlpDir "start-offline-programming.sh")
Write-Host "olp-start: invoking WSL $wslScript"
$wslArgs = @("-d", $Distro)
if ($script:OlpLinuxRebuilt) {
    $wslArgs += @("--env", "OLP_RESTART_SERVE=1")
}
$wslArgs += @("--", "bash", $wslScript)
& wsl.exe @wslArgs
if ($LASTEXITCODE -ne 0) {
    throw "WSL start-offline-programming.sh exited $LASTEXITCODE"
}

Write-Host ""
Write-Host "olp-start: Open the app: $AppUrl"
if (Test-Http "http://127.0.0.1:$ApiPort/api/offline-programming/v1/health") {
    Write-Host "olp-start: API / Tesseract: http://127.0.0.1:$ApiPort/api/offline-programming/v1/health"
} else {
    Write-Host "olp-start: API on :$ApiPort was not healthy after WSL return; check WSL serve log"
}
