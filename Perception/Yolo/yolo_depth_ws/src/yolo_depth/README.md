# YOLO Depth POC — Quick Start

A single ROS 2 node that runs YOLO on the RGB stream, samples the aligned depth image inside each bounding box, overlays labels like `Blue cone 5.20 m`, and publishes both the annotated image and `vision_msgs/Detection2DArray`. Opens a live OpenCV window for debugging.

This guide assumes you already have the package in place and the directory structure is correct. Only the bits you may need to tweak are listed here.

## 1) Edit your parameters

Open your parameter file, for example:
```
$(ros2 pkg prefix yolo_depth)/share/yolo_depth/config/yolo_depth.yaml
```
Key parameters to check:

```yaml
yolo_depth_node:
  ros__parameters:
    # Inputs
    rgb_topic: "/zed/left/image_rect_color"
    depth_topic: "/zed/depth/image_raw"   # 32FC1 metres or 16UC1 millimetres

    # Outputs
    annotated_image_topic: "/cones/annotated_depth"
    detections_topic: "/cones/detections"

    # Model
    model_path: "/absolute/path/to/best.torchscript"
    input_width: 640
    input_height: 640
    conf_threshold: 0.35
    nms_threshold: 0.45
    use_cuda: true

    # Drawing
    draw_boxes: true
    draw_labels: true
    window_title: "YOLO Depth POC"

    # Colour vote inside bbox
    min_colour_fraction: 0.10
    colour_shrink_px: 4

    # Depth sampling
    sample_mode: "grid"     # "grid" or "centre"
    grid_n: 5
    inner_ratio: 0.4
    min_valid_frac: 0.2
    min_range_m: 0.5
    max_range_m: 60.0
```

What you will likely change:
- `model_path` to your TorchScript file.
- `rgb_topic` and `depth_topic` if your camera topics differ.
- Set `use_cuda` to false if you do not have a working CUDA runtime.
- If you change `input_width` or `input_height`, re export the TorchScript with the same size.

Depth handling:
- `32FC1` is treated as metres.
- `16UC1` is converted from millimetres to metres inside the node.

## 2) Environment exports

Set LibTorch paths so the build links and the binary finds `libc10.so` at runtime:

```bash
export Torch_DIR=/opt/libtorch/share/cmake/Torch
export LD_LIBRARY_PATH=/opt/libtorch/lib:$LD_LIBRARY_PATH
```

Adjust `/opt/libtorch` if yours lives elsewhere.

## 3) Build

From your workspace root where the package already exists:

```bash
cd ~/Yolo/yolo_depth_ws
colcon build --packages-select yolo_depth --cmake-args -DTorch_DIR=$Torch_DIR
```

If you see a runtime linker error later about `libc10.so`, your `LD_LIBRARY_PATH` is not set.

## 4) Run

Launch file:
```bash
source install/setup.bash
ros2 launch yolo_depth yolo_depth.launch.py
```

Direct run with a specific params file:
```bash
ros2 run yolo_depth yolo_depth_node --ros-args --params-file $(ros2 pkg prefix yolo_depth)/share/yolo_depth/config/yolo_depth.yaml
```

You should see an OpenCV window titled `YOLO Depth POC`. The annotated image is also on `/cones/annotated_depth` for `rqt_image_view`.

## 5) Quick sanity checks

Confirm RGB is publishing:
```bash
ros2 topic hz /zed/left/image_rect_color
```

Check depth encoding once:
```bash
ros2 topic echo -n 1 /zed/depth/image_raw | grep encoding
# Expect "32FC1" or "16UC1"
```

View annotated output:
```bash
rqt_image_view   # then select /cones/annotated_depth
```

## 6) Performance tips

- On a GTX 1650 you should be fine at 640. If needed:
  - Set `sample_mode: centre` or reduce `grid_n` to 3.
  - Use a smaller model input, but re export the TorchScript to match the new size.
- Keep `use_cuda: true` if CUDA is available. The node falls back to CPU if not.

## 7) Common issues

- **`libc10.so: cannot open shared object file`**
  - Export `LD_LIBRARY_PATH` to your libtorch `lib` folder.

- **YAML parse error on launch**
  - Fix indentation and ensure the top keys are `yolo_depth_node:` and under it `ros__parameters:`.

- **No window appears**
  - You are in a headless shell. Use `rqt_image_view` to see `/cones/annotated_depth` or run in a GUI session.

- **Model forward failed**
  - Re export your YOLO TorchScript with the same `input_width` and `input_height` you set in the YAML.

## 8) Handy commands

```bash
# List topics
ros2 topic list

# Echo one message to inspect fields
ros2 topic echo -n 1 /zed/depth/image_raw

# Rebuild only this package
colcon build --packages-select yolo_depth --cmake-args -DTorch_DIR=$Torch_DIR

# Run with a custom params file
ros2 run yolo_depth yolo_depth_node --ros-args --params-file /path/to/yolo_depth.yaml
```

All set. Build, run, and you should see captions like `Blue cone 5.20 m` over your detections.
