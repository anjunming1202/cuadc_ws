# Fix Pi DroneLink AP over Ethernet maintenance link (10.10.10.1).
# Run on Windows while Ethernet to Pi is connected. You will be prompted for SSH password.
#
#   cd e:\cuadc_ws\src\dronelink\windows
#   .\fix_pi_ap_over_ethernet.ps1

param(
    [string]$PiHost = "10.10.10.1",
    [string]$User = "wly"
)

$ErrorActionPreference = "Stop"
$Root = "e:\cuadc_ws"
$Dl = Join-Path $Root "src\dronelink"
$Hostapd = Join-Path $Dl "raspberrypi\hostapd\hostapd.conf"
$NetSh = Join-Path $Dl "raspberrypi\scripts\dronelink-network.sh"
$NetUnit = Join-Path $Dl "raspberrypi\systemd\dronelink-network.service"
$Target = "${User}@${PiHost}"

Write-Host "Ping $PiHost ..."
$ping = Test-Connection -ComputerName $PiHost -Count 1 -Quiet
if (-not $ping) {
    throw "Cannot ping $PiHost. Plug the maintenance Ethernet cable (Windows 10.10.10.2 <-> Pi 10.10.10.1)."
}

Write-Host "Copying hostapd + network script to $Target ..."
scp $Hostapd "${Target}:~/hostapd.conf"
scp $NetSh "${Target}:~/dronelink-network.sh"
scp $NetUnit "${Target}:~/dronelink-network.service"

$Pub = $null
foreach ($name in @("id_ed25519.pub", "id_rsa.pub")) {
    $p = Join-Path $env:USERPROFILE ".ssh\$name"
    if (Test-Path $p) { $Pub = $p; break }
}
if ($Pub) {
    scp $Pub "${Target}:~/windows_ground.pub"
}

$Remote = @'
set -e
echo "=== BEFORE ==="
systemctl is-active dronelink-network.service || true
ip -br link | grep -E "wlx|wlan|eth" || true
echo "--- current hostapd (if any) ---"
sudo sed -n "1,30p" /etc/dronelink/hostapd.conf 2>/dev/null || echo "(no /etc/dronelink/hostapd.conf)"
echo "--- journal ---"
journalctl -u dronelink-network.service -n 40 --no-pager || true

sudo mkdir -p /etc/dronelink
sudo cp ~/hostapd.conf /etc/dronelink/hostapd.conf
sudo cp ~/dronelink-network.sh /usr/local/sbin/dronelink-network.sh
sudo chmod +x /usr/local/sbin/dronelink-network.sh
if [ -f ~/dronelink-network.service ]; then
  sudo cp ~/dronelink-network.service /etc/systemd/system/dronelink-network.service
fi

if [ -f ~/windows_ground.pub ]; then
  mkdir -p ~/.ssh
  chmod 700 ~/.ssh
  touch ~/.ssh/authorized_keys
  chmod 600 ~/.ssh/authorized_keys
  KEY=$(cat ~/windows_ground.pub)
  grep -qxF "$KEY" ~/.ssh/authorized_keys || echo "$KEY" >> ~/.ssh/authorized_keys
  rm -f ~/windows_ground.pub
  echo "authorized_keys updated"
fi

sudo systemctl daemon-reload
sudo systemctl enable dronelink-network.service
sudo systemctl restart dronelink-network.service
sleep 4

echo "=== AFTER ==="
systemctl is-active dronelink-network.service || true
systemctl status dronelink-network.service --no-pager -l | head -30
echo "--- restored hostapd ---"
grep -E "^(interface|ssid|channel|wpa_passphrase|country_code|hw_mode|beacon_int)=" /etc/dronelink/hostapd.conf || true
echo "--- iface ---"
IFACE=$(grep "^interface=" /etc/dronelink/hostapd.conf | cut -d= -f2)
ip -br addr show "$IFACE" 2>/dev/null || ip -br link | grep -i wlx || true
journalctl -u dronelink-network.service -n 25 --no-pager
'@

ssh -t $Target $Remote
Write-Host ""
Write-Host "Done. On Windows, rescan Wi-Fi; SSID should be DroneLink, password 12345678."
Write-Host "Then:  cd e:\cuadc_ws\src\dronelink\windows ; .\start_dronelink_ground.bat"
