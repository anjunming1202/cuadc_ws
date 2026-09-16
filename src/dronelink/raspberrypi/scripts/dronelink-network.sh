#!/bin/bash
set -e

IFACE="wlxa85e4565195f"

# Wait for the USB Wi-Fi interface (poll fast; was up to 30x1s).
for i in $(seq 1 40); do
    if [ -e "/sys/class/net/$IFACE" ]; then
        break
    fi
    sleep 0.25
done

if [ ! -e "/sys/class/net/$IFACE" ]; then
    echo "DroneLink: $IFACE did not appear within 10 seconds" >&2
    exit 1
fi

# Brief settle for 88XXau (was sleep 3).
sleep 1

# Realtek RTL8812AU China channel plan.
/usr/sbin/iwpriv "$IFACE" channel_plan 0x48

# Bring the interface up.
/usr/sbin/ip link set "$IFACE" up

# Start hostapd first. The 88XXau AP-mode transition may clear IPv4 addresses.
/usr/sbin/hostapd /etc/dronelink/hostapd.conf &
HOSTAPD_PID=$!

# Wait until AP iface is up / beaconing instead of a fixed 3s sleep.
for i in $(seq 1 20); do
    if ip link show "$IFACE" 2>/dev/null | grep -q "state UP\|LOWER_UP"; then
        break
    fi
    sleep 0.25
done
sleep 0.5

# Assign the DroneLink address after AP mode is established.
/usr/sbin/ip addr replace 192.168.50.1/24 dev "$IFACE"

echo "DroneLink AP ready on $IFACE (192.168.50.1)" >&2

# Keep systemd tracking hostapd through this wrapper.
wait "$HOSTAPD_PID"
