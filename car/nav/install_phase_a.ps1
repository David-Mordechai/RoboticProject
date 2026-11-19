# Copy all Phase A files from the PC to car #1 (Windows OpenSSH, key login as pi). Run from the repo root:
#   pwsh car/nav/install_phase_a.ps1 [-Car 10.71.88.196]
# Container paths: /home/ubuntu/shared = ~/docker/tmp on the host. Host scripts go to ~ (pi).
# The old depth_scan.py is kept as depth_scan_before_floorfit.py on the car (rollback: copy it back).
param([string]$Car = "10.71.88.196")
$ErrorActionPreference = "Stop"
$r = "pi@$Car"
python car/nav/make_params_a.py
ssh $r "mkdir -p docker/tmp/nav docker/tmp/maps docker/tmp/perception; cd docker/tmp/perception; [ -f depth_scan_before_floorfit.py ] || cp depth_scan.py depth_scan_before_floorfit.py"
scp -q car/nav/nav2_params_a.yaml car/nav/navigate_ackermann_a.xml car/nav/navigate_ackermann_b.xml `
       car/nav/nav_a.launch.py car/nav/nav_trip.py car/nav/set_pose.py car/nav/amcl_check.py car/nav/scan_map_match.py `
       car/nav/undock.py car/nav/dock.py car/nav/home_legs.sh car/nav/nav_a_check.py car/nav/dsnow.py car/nav/nav_guard.py car/nav/analysis/zone_pts.py `
       "${r}:docker/tmp/nav/"
scp -q car/maps/office_goto/goto6.pgm car/maps/office_goto/goto6.yaml "${r}:docker/tmp/maps/"
scp -q car/perception/depth_scan.py "${r}:docker/tmp/perception/"
scp -q car/measure/steer_limit.py "${r}:docker/tmp/"
scp -q car/nav/nav_a_start.sh car/nav/nav_a_stop.sh car/nav/nav_a_wait.sh car/nav/nav_a_ready.sh car/nav/nav_a_trial.sh car/nav/nav_a_c1.sh `
       car/nav/nav_a_home.sh car/nav/stop_trial.sh car/nav/gate_a_loop.sh car/measure/rec_run.sh "${r}:"
ssh $r "chmod +x nav_a_*.sh stop_trial.sh rec_run.sh gate_a_loop.sh; sed -i 's/\r$//' *.sh docker/tmp/nav/*.py docker/tmp/nav/*.sh docker/tmp/nav/*.yaml docker/tmp/perception/depth_scan.py; ls -la nav_a_*.sh docker/tmp/nav/navigate_ackermann_b.xml docker/tmp/perception/depth_scan*.py"
Write-Host "installed on $Car"
