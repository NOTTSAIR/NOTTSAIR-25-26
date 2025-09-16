# static_a

ROS 2 package for **Static Inspection A**. Publishes Ackermann commands for EUFS.  
Tested on **Ubuntu 20.04** with **ROS 2 Galactic**.

This guide assumes you cloned the repository into:

```
~/Documents/NOTTSAIR-25-26/Missions/Static Inspection A/static_a_ws
```

with the following structure:

```
~/Documents/NOTTSAIR-25-26/Missions/Static Inspection A/static_a_ws/
└─ src/
   └─ static_a/
      ├─ package.xml
      ├─ CMakeLists.txt
      ├─ src/
      │   └─ static_inspection_a.cpp
      ├─ launch/
      │   └─ static_a.launch.py
      └─ config/
          └─ static_a.yaml    # optional
```

---

## Prerequisites
- ROS 2 Galactic installed and working
- EUFS sim workspace at `~/eufs_ws` built and sourced when running
- Git and build tools

---

## 1) Change to the workspace root
```bash
cd ~/Documents/NOTTSAIR-25-26/Missions/Static\ Inspection\ A/static_a_ws
```

---

## 2) Install dependencies (first time on a machine)
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## 3) Build
```bash
colcon build --packages-select static_a
```

---

## 4) Source workspaces
Always source EUFS first, then this workspace:

```bash
source ~/eufs_ws/install/setup.bash
source ~/Documents/NOTTSAIR-25-26/Missions/Static\ Inspection\ A/static_a_ws/install/setup.bash
```

---

## 5) Running with the simulator

1. **Start the simulator** (Gazebo + EUFS sim stack).  
   Make sure `~/eufs_ws` is built and sourced before launching.  

2. In a new terminal, **run Static A**:
   ```bash
   ros2 launch static_a static_a.launch.py
   ```

   The terminal will now show **"waiting"** and **"gate opened"** messages once the mission starts.

3. Open the **Mission Control GUI** (window name: `eufs_sim.perspective - rqt`).  
   From the **Mission** dropdown, select:
   ```
   DDT_INSPECTION_A
   ```

4. Once selected, Static A will respond (takes a few seconds), and you can watch the car moving in **Gazebo**.  

---

## 6) Quick rebuild
```bash
cd ~/Documents/NOTTSAIR-25-26/Missions/Static\ Inspection\ A/static_a_ws
colcon build --packages-select static_a
source install/setup.bash
```

---

## 7) Clean rebuild
```bash
cd ~/Documents/NOTTSAIR-25-26/Missions/Static\ Inspection\ A/static_a_ws
rm -rf build install log
colcon build --packages-select static_a
source install/setup.bash
```

---

## Helpful bash aliases
Add these to your `~/.bashrc` so you can build and run easily.

```bash
# --- static_a helpers ---
source_static_a_env() {
  # Source EUFS if present
  if [ -f "$HOME/eufs_ws/install/setup.bash" ]; then
    . "$HOME/eufs_ws/install/setup.bash"
  fi
  # Source this workspace
  if [ -f "$HOME/Documents/NOTTSAIR-25-26/Missions/Static Inspection A/static_a_ws/install/setup.bash" ]; then
    . "$HOME/Documents/NOTTSAIR-25-26/Missions/Static Inspection A/static_a_ws/install/setup.bash"
  fi
}

build_static_a() {
  cd "$HOME/Documents/NOTTSAIR-25-26/Missions/Static Inspection A/static_a_ws" || return 1
  colcon build --packages-select static_a || return 1
  . "$HOME/Documents/NOTTSAIR-25-26/Missions/Static Inspection A/static_a_ws/install/setup.bash"
}

run_static_a() {
  source_static_a_env || return 1
  ros2 launch static_a static_a.launch.py
}

static_a() {
  build_static_a && run_static_a
}
# --- end static_a helpers ---
```

After editing `~/.bashrc`, reload it:
```bash
source ~/.bashrc
```

Now you can run:
```bash
static_a        # builds then launches
build_static_a  # build only
run_static_a    # launch only
```

---

## Troubleshooting
- **Package not found**: Source both workspaces again:
  ```bash
  source ~/eufs_ws/install/setup.bash
  source ~/Documents/NOTTSAIR-25-26/Missions/Static\ Inspection\ A/static_a_ws/install/setup.bash
  ```
- **Executable not found**: Rebuild and check the binary exists:
  ```bash
  ls ~/Documents/NOTTSAIR-25-26/Missions/Static\ Inspection\ A/static_a_ws/install/static_a/lib/static_a/static_inspection_a
  ```
- **Missing dependencies**: Install with:
  ```bash
  rosdep install --from-paths src --ignore-src -r -y
  ```

---

## .gitignore (recommended in `static_a/`)
```
build/
install/
log/
# CMake
CMakeFiles/
CMakeCache.txt
cmake_install.cmake
Makefile
# Python cache
__pycache__/
*.py[cod]
# IDEs
.vscode/
.idea/
# OS
.DS_Store
Thumbs.db
```

---

## Licence
MIT
