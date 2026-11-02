#!/bin/bash
# On the car host: wait for the stack, then measure camera rates + health + CPU. Arg: label
L=$1
for i in $(seq 1 30); do
  sudo -n docker exec -u ubuntu MentorPi zsh -c "source ~/.zshrc >/dev/null 2>&1; ros2 node list 2>/dev/null" | grep -q camera_publisher && break; sleep 3
done
sleep 15
echo "=== $L"; sudo -n docker exec -u ubuntu MentorPi zsh -c "source ~/.zshrc >/dev/null 2>&1; timeout 60 python3 /home/ubuntu/shared/cam_rate.py" 2>&1 | grep -v pygame &
sleep 18; echo "--- top (during image phase)"; top -bn1 -o %CPU | sed -n '3p;7,12p'; wait
echo "--- load"; uptime
