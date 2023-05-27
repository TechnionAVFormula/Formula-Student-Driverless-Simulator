# Connecting ROS Bridge to Sim
1. Run the sim (if on linux see `docker` folder)
2. Copy `settings.json` from the root of the repo into `~/Formula-Student-Driverless-Simulator`
2. If using a conda environment, activate it now
3. In this `ros2` directory:
```
colcon build --symlink-install
source install/setup.sh
ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py manual_mode:=True
```