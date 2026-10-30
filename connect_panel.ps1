param(
    [Parameter(Mandatory = $true)][string]$RobotHost,
    [ValidateRange(1024, 65535)][int]$Port = 8765
)
$ErrorActionPreference = 'Stop'
Write-Host "After SSH connects, open http://127.0.0.1:$Port/ in your browser."
& ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=10 -L "127.0.0.1:${Port}:127.0.0.1:${Port}" $RobotHost
exit $LASTEXITCODE
