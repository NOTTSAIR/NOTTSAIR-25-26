# YOLO Inference on ROS2 (GPU, TorchScript)

This package (`yolo_infer`) runs YOLO-based cone detection inside ROS2 using LibTorch (C++).  
It was tested with **CUDA 12.8**, **LibTorch cu128**, **ROS2 Galactic**, and a **GeForce RTX 3060**.

---

## Requirements

- **Ubuntu 20.04**
- **ROS2 Galactic**
- **NVIDIA GPU + driver**
- **CUDA 12.8 toolkit**
- **LibTorch (prebuilt with cu128)**
- **Python 3.10 venv** (for exporting YOLO models)
- **Ultralytics >= 8.3**
- **colcon build tools**

---

## Installation

### 1. CUDA 12.8 + Driver
Make sure your driver and CUDA toolkit are installed:
```bash
nvidia-smi   # should report CUDA 12.8
```
Export environment variables:
```bash
export CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.8
export CUDACXX=/usr/local/cuda-12.8/bin/nvcc
export PATH=/usr/local/cuda-12.8/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64:$LD_LIBRARY_PATH
```

### 2. LibTorch
Download **LibTorch cu128 (shared with deps)** and extract to `/opt/libtorch`:
```bash
sudo unzip libtorch-shared-with-deps-2.8.0+cu128.zip -d /opt
```
Then export:
```bash
export Torch_DIR=/opt/libtorch/share/cmake/Torch
export LD_LIBRARY_PATH=/opt/libtorch/lib:$LD_LIBRARY_PATH
```

### 3. Python venv for YOLO
```bash
python3.10 -m venv ~/venvs/yolo
source ~/venvs/yolo/bin/activate
pip install --upgrade pip setuptools wheel
pip install ultralytics opencv-python torch torchvision matplotlib polars pyyaml requests scipy psutil
```


Update `config/defaults.yaml` (change to your linux username):
```yaml
model_path: "/home/tyler/Documents/NOTTSAIR-25-26/Perception/Yolo/best.torchscript"
use_cuda: true
input_width: 640
input_height: 640
```

---

## Build

Deactivate venv first (colcon must not use the venv Python):
```bash
deactivate 2>/dev/null || true
cd ~/Documents/NOTTSAIR-25-26/Perception/Yolo/yolo_ws
source /opt/ros/galactic/setup.bash
rm -rf build/ install/ log/
colcon build --symlink-install --cmake-args \
  -DTorch_DIR=/opt/libtorch/share/cmake/Torch \
  -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.8 \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.8/bin/nvcc
```

---

## Run

```bash
source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 launch yolo_infer yolo_infer.launch.py
```

---

## Bash Helpers

Append this block to your `~/.bashrc`:

```bash
########## YOLO GPU WORKFLOW ##########
export YOLO_WS="$HOME/Documents/NOTTSAIR-25-26/Perception/Yolo/yolo_ws"
export YOLO_PY_VENV="$HOME/venvs/yolo"

export CUDA_TOOLKIT_ROOT_DIR="/usr/local/cuda-12.8"
export CUDACXX="/usr/local/cuda-12.8/bin/nvcc"
export PATH="/usr/local/cuda-12.8/bin:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda-12.8/lib64:/opt/libtorch/lib:$LD_LIBRARY_PATH"

export Torch_DIR="/opt/libtorch/share/cmake/Torch"

yolo_ros_base() { source /opt/ros/galactic/setup.bash; }

yolo_venv() { source "$YOLO_PY_VENV/bin/activate" && echo "venv on $(python -V)"; }

yolo_export_cuda() {
  local size="${1:-640}"
  yolo_venv
  cd "$YOLO_WS/src/yolo_infer" || return
  yolo export model=best.pt format=torchscript imgsz="$size" device=0
  mv best.torchscript "best_${size}_cuda.torchscript"
}

yolo_build() {
  deactivate 2>/dev/null || true
  yolo_ros_base
  cd "$YOLO_WS" || return
  rm -rf build/ install/ log/
  colcon build --symlink-install --cmake-args \
    -DTorch_DIR="$Torch_DIR" \
    -DCUDA_TOOLKIT_ROOT_DIR="$CUDA_TOOLKIT_ROOT_DIR" \
    -DCMAKE_CUDA_COMPILER="$CUDACXX"
  source install/setup.bash
}

yolo_run() {
  deactivate 2>/dev/null || true
  yolo_ros_base
  cd "$YOLO_WS" || return
  source install/setup.bash
  ros2 launch yolo_infer yolo_infer.launch.py "$@"
}

yolo_br() { yolo_build && yolo_run "$@"; }

yolo_clean() { cd "$YOLO_WS" && rm -rf build/ install/ log/; }
########## END YOLO GPU WORKFLOW ##########
```

---

## Notes

- Always export models with `device=0` to avoid CPU-only TorchScript.
- Always build with venv **off**.
- At runtime, the node opens one OpenCV window with YOLO cone detections.
