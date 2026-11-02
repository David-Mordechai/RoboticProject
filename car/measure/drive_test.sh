source ~/.zshrc >/dev/null 2>&1
T=/controller/cmd_vel
pub() { ros2 topic pub -r 10 -t $3 $T geometry_msgs/msg/Twist "{linear: {x: $1}, angular: {z: $2}}" >/dev/null; }
stop() { pub 0.0 0.0 10; }
trap stop EXIT
step() { echo "$(date +%T) $1"; pub $2 $3 20; stop; }
step "1/5 forward  v=0.15 m/s"            0.15  0.0
step "2/5 reverse  v=-0.15 m/s"          -0.15  0.0
step "3/5 forward + steer LEFT  w=0.5"     0.15  0.5
step "4/5 forward + steer RIGHT w=-0.5"    0.15 -0.5
step "5/5 forward straight (re-center)"    0.15  0.0
echo "$(date +%T) done, stopped"
