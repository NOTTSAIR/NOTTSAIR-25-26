# Camera Viewer (ROS 2 Galactic)

This package subscribes to ZED camera topics and displays camera feeds.

## Build

```bash
cd ~/Documents/NOTTSAIR-25-26/Perception/eufs_perception_starters/camera_ws
rm -rf build install log
colcon build --packages-select camera
```

## Run

In a new terminal (after starting the EUFS sim):

```bash
source ~/eufs_ws/install/setup.bash
source install/setup.bash
ros2 launch camera camera.launch.py
```

## Notes
- Subscribes to `/zed/left/image_rect_color` and `/zed/right/image_rect_color`.
- QoS is set to BEST_EFFORT to match the simulator publishers.
