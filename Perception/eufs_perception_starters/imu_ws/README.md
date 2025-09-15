# IMU Viewer (ROS 2 Galactic)

This package subscribes to `/imu/data` and visualises IMU orientation/acceleration.

## Build

```bash
cd ~/Documents/NOTTSAIR-25-26/Perception/eufs_perception_starters/imu_ws
rm -rf build install log
colcon build --packages-select imu
```

## Run

```bash
source ~/eufs_ws/install/setup.bash
source install/setup.bash
ros2 launch imu imu.launch.py
```

## Notes
- Displays raw IMU orientation/linear acceleration.
- Useful for debugging noisy or unstable IMU data.
