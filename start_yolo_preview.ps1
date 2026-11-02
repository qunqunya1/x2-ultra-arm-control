param(
    [string]$Model = "",
    [string]$Python = "",
    [string]$CameraUrl = 'http://127.0.0.1:8765',
    [ValidateRange(1024,65535)][int]$Port = 8774
)
$ErrorActionPreference='Stop'
if (-not $Python) { throw 'Specify -Python using your YOLO environment.' }
if (-not $Model) { throw 'Specify -Model using your trained weights.' }
if (-not (Test-Path -LiteralPath $Python)) { throw 'Specify -Python using your YOLO environment.' }
if (-not (Test-Path -LiteralPath $Model)) { throw 'Specify -Model using the trained best.pt file.' }
$env:PYTHONUTF8='1'
Push-Location -LiteralPath $PSScriptRoot
try { & $Python -B -u -m x2_kinematics.aimdk_bridge.yolo_preview --model $Model --camera-url $CameraUrl --port $Port }
finally { Pop-Location }
