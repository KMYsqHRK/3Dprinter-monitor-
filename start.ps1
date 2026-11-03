# Start the monitor reachable from other devices (Tailscale / LAN), token-protected.
# The token is generated once into token.txt next to this script.
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$tokenFile = Join-Path $here 'token.txt'
if (-not (Test-Path $tokenFile)) {
    $bytes = New-Object byte[] 24
    [Security.Cryptography.RandomNumberGenerator]::Create().GetBytes($bytes)
    [Convert]::ToBase64String($bytes).Replace('+', '-').Replace('/', '_').TrimEnd('=') |
        Out-File -Encoding ascii $tokenFile
}
$env:PM_TOKEN = (Get-Content $tokenFile -Raw).Trim()
$env:PM_HOST = '0.0.0.0'
uv run --directory $here printer_monitor.py
