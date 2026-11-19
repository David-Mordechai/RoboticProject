#!/bin/bash
# Car Wi-Fi: return from the car's own access point to the phone hotspot (installed 2026-11-16, user approved).
# Why: NetworkManager never leaves an active connection, so after the hotspot dropped once (2026-11-16 14:32 PC time)
# car #1 stayed on its fallback AP HW-54007663 even when AndroidAP came back (priorities: AndroidAP 10, own AP 0).
# The Pi's Wi-Fi cannot scan while it is an AP (scan shows only itself, tested), so we TRY instead of looking first:
#   only when wlan0 runs the car's own AP AND no device is connected to it (never cut off someone using the AP),
#   try AndroidAP (30 s); if that fails, bring the own AP straight back.
# Run every 60 s by wifi-return.timer (root). Log: journalctl -t wifi_return
AP=HW-54007663; HOT=AndroidAP
active=$(nmcli -t -f NAME,DEVICE connection show --active | awk -F: '$2=="wlan0"{print $1}')
[ "$active" = "$AP" ] || exit 0
[ -z "$(iw dev wlan0 station dump)" ] || exit 0
logger -t wifi_return "on own AP with no clients: trying $HOT"
if timeout 45 nmcli --wait 30 connection up "$HOT" >/dev/null 2>&1; then
  logger -t wifi_return "joined $HOT: $(ip -4 -br addr show wlan0 | awk '{print $3}')"
else
  logger -t wifi_return "$HOT not reachable: back to $AP"
  nmcli --wait 20 connection up "$AP" >/dev/null 2>&1 || logger -t wifi_return "WARNING: could not restore $AP"
fi
