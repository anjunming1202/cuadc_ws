# Deploy / refresh DroneLink telemetry on the Pi over SSH.
# Usage (PowerShell):
#   .\deploy_pi_telemetry.ps1
#   .\deploy_pi_telemetry.ps1 -PiHost RaspberryPi5.local
# You will be prompted for the Pi password once (or use an installed SSH key).
# Also installs this PC's ~/.ssh/*.pub into the Pi authorized_keys so later
# agent/SSH access can be passwordless.

param(
    [string]$PiHost = "192.168.50.1",
    [string]$User = "wly"
)

$ErrorActionPreference = "Stop"
$Root = "e:\cuadc_ws"
$Dl = Join-Path $Root "src\dronelink"
$Script = Join-Path $Dl "raspberrypi\scripts\dronelink-telemetry.py"
$Unit = Join-Path $Dl "raspberrypi\systemd\dronelink-telemetry.service"
$Target = "${User}@${PiHost}"

$Pub = $null
foreach ($name in @("id_ed25519.pub", "id_rsa.pub")) {
    $p = Join-Path $env:USERPROFILE ".ssh\$name"
    if (Test-Path $p) { $Pub = $p; break }
}

Write-Host "Deploying to $Target ..."
scp $Script "${Target}:~/dronelink-telemetry.py"
scp $Unit "${Target}:~/dronelink-telemetry.service"
if ($Pub) {
    scp $Pub "${Target}:~/windows_ground.pub"
    Write-Host "Copied pubkey: $Pub"
}

$Remote = @'
set -e
sudo cp ~/dronelink-telemetry.py /usr/local/sbin/dronelink-telemetry.py
sudo chmod +x /usr/local/sbin/dronelink-telemetry.py
sudo cp ~/dronelink-telemetry.service /etc/systemd/system/dronelink-telemetry.service
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
sudo systemctl enable dronelink-telemetry.service
sudo systemctl restart dronelink-telemetry.service
sleep 3
systemctl is-active dronelink-network.service dronelink-video.service dronelink-telemetry.service || true
journalctl -u dronelink-telemetry.service -n 30 --no-pager
ping -c 2 -W 1 192.168.50.2 || true
'@

ssh -t $Target $Remote
Write-Host "Done. Check HUD: http://127.0.0.1:8765/"
