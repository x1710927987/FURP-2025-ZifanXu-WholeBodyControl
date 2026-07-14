# Attach Berxel iHawk100 USB camera to WSL2 (run as Administrator).
# Usage: powershell -ExecutionPolicy Bypass -File scripts/berxel/camera/attach_camera.ps1

$ErrorActionPreference = "Stop"
$usbipd = "C:\Program Files\usbipd-win\usbipd.exe"

if (-not (Test-Path $usbipd)) {
    Write-Error "usbipd-win not found. Install: winget install dorssel.usbipd-win"
}

Write-Host "=== Berxel iHawk100 USB camera -> WSL2 ===" -ForegroundColor Cyan
Start-Process wsl -ArgumentList "-d", "Ubuntu-24.04" -WindowStyle Hidden | Out-Null
Start-Sleep -Seconds 2

Write-Host "==> USB devices:"
& $usbipd list

$busid = (& $usbipd list | Select-String "Berxel" | ForEach-Object {
    if ($_ -match '^\s*(\d+-\d+)\s') { $matches[1] }
} | Select-Object -First 1)

if (-not $busid) {
    Write-Error "No Berxel camera found. Check USB connection."
}

$state = (& $usbipd list | Select-String $busid)
if ($state -match "Attached") {
    Write-Host "Camera already attached (busid $busid)." -ForegroundColor Yellow
} else {
    Write-Host "==> Binding and attaching busid $busid..."
    $prevEap = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    & $usbipd bind --busid $busid 2>&1 | Out-Null
    $ErrorActionPreference = $prevEap
    & $usbipd attach --wsl --busid $busid
    Write-Host "Attached successfully." -ForegroundColor Green
}

Write-Host ""
Write-Host "Next in WSL (Ubuntu-24.04):" -ForegroundColor Yellow
Write-Host "  ls /dev/video*" -ForegroundColor White
Write-Host "  bash /mnt/d/Repository/internship_and_research/FURP-2025-ZifanXu-WholeBodyControl/scripts/berxel/calibration/collect.sh" -ForegroundColor White
