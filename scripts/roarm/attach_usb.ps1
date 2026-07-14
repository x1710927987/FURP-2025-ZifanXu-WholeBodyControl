# Attach RoArm-M3 CP2102 serial USB to WSL RoArm_M3 (run in Admin PowerShell).
# Prerequisite: usbipd-win installed; RoArm-M3 powered on and USB connected.

$ErrorActionPreference = "Stop"
$usbipd = "C:\Program Files\usbipd-win\usbipd.exe"

if (-not (Test-Path $usbipd)) {
    Write-Error "usbipd-win not found. Install: winget install dorssel.usbipd-win"
}

Write-Host "==> Ensuring RoArm_M3 WSL distro is running..."
Start-Process wsl -ArgumentList "-d", "RoArm_M3" -WindowStyle Hidden | Out-Null
Start-Sleep -Seconds 2

Write-Host "==> USB devices:"
& $usbipd list

$busid = (& $usbipd list | Select-String "CP2102|CP210x|UART Bridge" | ForEach-Object {
    if ($_ -match '^\s*(\d+-\d+)\s') { $matches[1] }
} | Select-Object -First 1)

if (-not $busid) {
    Write-Error "No CP2102 serial device found. Check USB cable and arm power."
}

Write-Host "==> Binding busid $busid (ignore error if already shared)..."
& $usbipd bind --busid $busid 2>$null

Write-Host "==> Attaching to RoArm_M3..."
& $usbipd attach --wsl RoArm_M3 --busid $busid

Write-Host "==> Done. In RoArm_M3 terminal run: ls /dev/ttyUSB*"
