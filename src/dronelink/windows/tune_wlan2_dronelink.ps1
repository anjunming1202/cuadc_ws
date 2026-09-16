# One-time (or occasional) Windows tune so WLAN 2 finds DroneLink fast.
# Run in PowerShell (Admin recommended for power-management):
#   cd e:\cuadc_ws\src\dronelink\windows
#   .\tune_wlan2_dronelink.ps1
#
# What it does:
# - Puts DroneLink first on WLAN 2
# - Sets every OTHER profile on WLAN 2 to manual (stops hotel/phone SSIDs
#   from stealing scan/connect time on the USB adapter)
# - Keeps DroneLink on auto-connect, no autoswitch
# - Tries to disable adapter power saving

param(
    [string]$Iface = "WLAN 2",
    [string]$Ssid = "DroneLink"
)

$ErrorActionPreference = "Continue"
Write-Host "Tuning interface '$Iface' for SSID '$Ssid' ..."

netsh wlan set profileparameter name="$Ssid" interface="$Iface" connectionmode=auto | Out-Host
netsh wlan set profileparameter name="$Ssid" interface="$Iface" autoswitch=no | Out-Host
netsh wlan set profileorder name="$Ssid" interface="$Iface" priority=1 | Out-Host

$profiles = @()
foreach ($line in (netsh wlan show profiles interface="$Iface")) {
    # EN: "All User Profile     : Name"  / ZH: "所有用户配置文件 : Name"
    if ($line -match "(?:All User Profile|所有用户配置文件)\s*:\s*(.+)\s*$") {
        $name = $Matches[1].Trim()
        if ($name) { $profiles += $name }
    }
}
$profiles = $profiles | Select-Object -Unique
Write-Host "Profiles on ${Iface}: $($profiles -join ', ')"

foreach ($p in $profiles) {
    if ($p -eq $Ssid) { continue }
    Write-Host "  -> manual: $p"
    netsh wlan set profileparameter name="$p" interface="$Iface" connectionmode=manual | Out-Null
}

try {
    Disable-NetAdapterPowerManagement -Name $Iface -ErrorAction Stop
    Write-Host "Disabled power management on '$Iface'."
} catch {
    Write-Host "Could not change adapter power management (try Run as Administrator): $_"
}

Write-Host ""
Write-Host "Done. Next boots: WLAN 2 should only auto-hunt DroneLink."
Write-Host "For an immediate reconnect now, run: .\connect_dronelink.bat"
