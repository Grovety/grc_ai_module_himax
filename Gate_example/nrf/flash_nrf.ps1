param (
    [string]$Port = "COM1"
)

$ErrorActionPreference = "Stop"

$BaudRate   = 115200
$Mtu        = 1024
$ConnName   = "testDK"
$ImageFile  = "./bin/zephyr.signed.bin"

# Colors
function Write-Green ([string]$Text) { Write-Host $Text -ForegroundColor Green }
function Write-Red   ([string]$Text) { Write-Host $Text -ForegroundColor Red }

function Run-Command {
    param([string]$Cmd, [string[]]$ArgsList, [bool]$IgnoreError = $false)
    
    $process = Start-Process -FilePath $Cmd -ArgumentList $ArgsList -Wait -NoNewWindow -PassThru
    
    if ($process.ExitCode -ne 0 -and -not $IgnoreError) {
        throw "Command failed"
    }
}

try {
    Write-Host "Using Port: " -NoNewline
    Write-Green $Port

    if (-not (Test-Path $ImageFile)) {
        Write-Red "Error: Firmware file not found: $ImageFile"
        exit 1
    }

    Run-Command "mcumgr" @("conn", "delete", $ConnName) -IgnoreError $true

    Write-Host "Configuring connection..."
    $connString = "dev=$Port,baud=$BaudRate,mtu=$Mtu"
    
    try {
        Run-Command "mcumgr" @("conn", "add", $ConnName, "type=serial", "connstring=$connString")
    } catch {
        Write-Red "Error: Failed to configure mcumgr connection."
        exit 1
    }

    Start-Sleep -Seconds 1

    Write-Host "Uploading image..."
    try {
        Run-Command "mcumgr" @("-c", $ConnName, "image", "upload", $ImageFile)
    } catch {
        Write-Red "Error: Image upload failed."
        exit 1
    }

    Start-Sleep -Seconds 1

    Write-Host "Resetting device..."
    try {
        Run-Command "mcumgr" @("-c", $ConnName, "reset")
    } catch {
        Write-Red "Error: Reset failed."
        exit 1
    }

    Write-Green "nRF Flashing Complete."

} catch {
    Write-Red "Error: $($_.Exception.Message)"
    exit 1
}