# GPS Viewer (ROS 2 Galactic)

This package subscribes to `/gps` (`sensor_msgs/NavSatFix`) and displays the car’s live position with a fading trail.

## Build

```bash
cd ~/Documents/NOTTSAIR-25-26/Perception/eufs_perception_starters/gps_ws
rm -rf build install log
colcon build --packages-select gps
```

## Run

```bash
source ~/eufs_ws/install/setup.bash
source install/setup.bash
ros2 launch gps gps.launch.py
```

## Notes
- Origin is fixed at the starting point (0,0).
- Trail fades over time for clarity.
