# Find the MentorPi cars on every network the PC is on, by MAC address.
# The phone hotspot (AndroidAP) changes its subnet between sessions (10.71.88.x on 2026-11-11,
# 10.164.108.x on 2026-11-16) and cannot reserve addresses, so stored IPs go stale. MACs don't.
# MACs: car #1 eth0 98:FE:54:00:76:63, wlan0 98:FE:54:00:76:64 (memory note robot1, read on the car);
#       car #2 eth0 98:FE:54:00:82:0C (memory note robot2); car #2 wlan0 not recorded, matched by prefix.
# Also reports the cars' own access points (HW-<mac>) if visible, which means a car did NOT join the hotspot.
# Usage: pwsh car/pc/find_cars.ps1
param([int]$TimeoutMs = 800)

$cars = @{ '98-fe-54-00-76' = 'car1'; '98-fe-54-00-82' = 'car2' }

# Every up IPv4 interface; sweep its /24 (the office LAN is a /23, so sweep both halves there).
$nets = Get-NetIPAddress -AddressFamily IPv4 |
    Where-Object { $_.IPAddress -notlike '127.*' -and $_.IPAddress -notlike '169.254.*' -and $_.PrefixLength -ge 16 }
$prefixes = @()
foreach ($n in $nets) {
    $o = $n.IPAddress.Split('.')
    if ($n.PrefixLength -le 23) {
        $base = [int]$o[2] - ([int]$o[2] % 2)
        $prefixes += "$($o[0]).$($o[1]).$base", "$($o[0]).$($o[1]).$($base + 1)"
    } else {
        $prefixes += "$($o[0]).$($o[1]).$($o[2])"
    }
}
$prefixes = $prefixes | Sort-Object -Unique
Write-Host "Sweeping: $($prefixes -join '.x, ').x"

$tasks = foreach ($p in $prefixes) { 1..254 | ForEach-Object { [System.Net.NetworkInformation.Ping]::new().SendPingAsync("$p.$_", $TimeoutMs) } }
[System.Threading.Tasks.Task]::WaitAll($tasks)

$found = @()
foreach ($line in (arp -a)) {
    if ($line -match '^\s*(\d+\.\d+\.\d+\.\d+)\s+([0-9a-f-]{17})\s+dynamic') {
        $ip = $Matches[1]; $mac = $Matches[2].ToLower()
        $key = $mac.Substring(0, 14)
        if ($cars.ContainsKey($key)) { $found += [pscustomobject]@{ Car = $cars[$key]; IP = $ip; MAC = $mac } }
    }
}
if ($found) { $found | Sort-Object Car | Format-Table -AutoSize } else { Write-Host 'No car found on any connected network.' }

$aps = netsh wlan show networks | Select-String 'SSID \d+ : (HW-\S+)' | ForEach-Object { $_.Matches[0].Groups[1].Value }
if ($aps) { Write-Host "Car access points visible (car did not join the hotspot): $($aps -join ', ')" }
