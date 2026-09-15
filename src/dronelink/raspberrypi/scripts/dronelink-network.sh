#!/bin/bash
set -e

IFACE="wlxa85e4565195f"

# Wait for the USB Wi-Fi interface to appear.
for i in $(seq 1 30); do
    if [ -e "/sys/class/net/$IFACE" ]; then
        break
    fi
    sleep 1
done

if [ ! -e "/sys/class/net/$IFACE" ]; then
    echo "DroneLink: $IFACE did not appear within 30 seconds" >&2
    exit 1
fi

# Give the 88XXau driver time to settle.
sleep 3

# Realtek RTL8812AU China channel plan.
/usr/sbin/iwpriv "$IFACE" channel_plan 0x48

# Bring the interface up.
/usr/sbin/ip link set "$IFACE" up

# Start hostapd first. The 88XXau AP-mode transition may clear IPv4 addresses.
/usr/sbin/hostapd /etc/dronelink/hostapd.conf &
HOSTAPD_PID=$!

# Allow the adapter to complete the AP-mode transition.
sleep 3

# Assign the DroneLink address after AP mode is established.
/usr/sbin/ip addr replace 192.168.50.1/24 dev "$IFACE"

# Keep systemd tracking hostapd through this wrapper.
wait "$HOSTAPD_PID"
