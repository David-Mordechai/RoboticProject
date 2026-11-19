#!/bin/bash
# Wait (max 60 s) until both Nav2 lifecycle managers report active, or a failure appears; then print the log's key lines.
L=~/docker/tmp/nav/nav_a.log
for i in $(seq 30); do
  a=$(grep -c 'Managed nodes are active' $L 2>/dev/null); f=$(grep -c -E 'Failed to bring up|process has died' $L 2>/dev/null)
  [ "${a:-0}" -ge 2 ] || [ "${f:-0}" -ge 1 ] && break; sleep 2
done
grep -i -E 'error|fail|active|lethal|exception|warn|died' $L | grep -v -i deactivat | tail -40
