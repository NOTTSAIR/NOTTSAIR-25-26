# LiDAR BEV Diagnostic (ROS 2 Galactic)

This package subscribes to `/velodyne_points` and shows a bird’s-eye view visualisation.

## Build

```bash
cd ~/Documents/NOTTSAIR-25-26/Perception/eufs_perception_starters/lidar_ws
rm -rf build install log
colcon build --packages-select lidar
```

## Run

```bash
source ~/eufs_ws/install/setup.bash
source install/setup.bash
ros2 launch lidar lidar.launch.py
```

## Notes
- Shows a top-down scatter plot of LiDAR points.
- Arrow keys pan, `q` quits.
