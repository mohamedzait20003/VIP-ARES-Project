# ARES-Project

**Autonomous Drone Control System** built on ROS2 and ArduPilot.

Maintainer: Mohammed Zaitoun — mzaitoun@purdue.edu  
License: MIT

---

## Prerequisites

Make sure the following are installed on your system before setting up this workspace.

### 1. ROS2

This package targets **ROS2 Humble** (Ubuntu 22.04) or **ROS2 Iron/Jazzy** (Ubuntu 24.04).

Follow the official installation guide:  
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

Source your ROS2 installation in every new terminal (or add it to `~/.bashrc`):

```bash
source /opt/ros/humble/setup.bash
```

### 2. MAVROS

MAVROS provides the bridge between ROS2 and ArduPilot via MAVLink.

```bash
sudo apt update
sudo apt install ros-humble-mavros ros-humble-mavros-extras
```

Install the required GeographicLib datasets (needed by MAVROS):

```bash
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

### 3. Additional ROS2 Dependencies

```bash
sudo apt install \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-geometry-msgs \
  ros-humble-sensor-msgs \
  ros-humble-nav-msgs \
  ros-humble-std-msgs
```

### 4. Build Tools

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep
```

### 5. Python Environment

This project uses a Python virtual environment to manage non-ROS Python dependencies.

```bash
sudo apt install python3-venv python3-pip
```

Create the virtual environment in the repo root:

```bash
cd ~/ros2_ws/src/ARES-Project
python3 -m venv .venv
```

Activate it:

```bash
source .venv/bin/activate
```

Install project dependencies:

```bash
pip install -r requirements.txt
```

> The virtual environment should be activated whenever running Python scripts in this project.  
> Add `.venv/` to `.gitignore` — it should never be committed.

---

## Workspace Setup

### 1. Create and enter your ROS2 workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone this repository

```bash
git clone https://github.com/mohamedzait20003/VIP-ARES-Project.git ARES-Project
```

### 3. Set up the Python virtual environment

```bash
cd ~/ros2_ws/src/ARES-Project
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### 4. Initialize rosdep and install dependencies

```bash
cd ~/ros2_ws
sudo rosdep init        # skip if already done
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build the workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

### 6. Source the workspace

```bash
source ~/ros2_ws/install/setup.bash
```

Add this line to your `~/.bashrc` so it is sourced automatically in every new terminal:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Connecting to ArduPilot

### Simulation (SITL)

If testing with ArduPilot SITL, start MAVROS pointing at the simulator:

```bash
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555
```

### Hardware (Flight Controller via USB/Serial)

```bash
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:57600
```

Replace `/dev/ttyACM0` and baud rate with the values appropriate for your setup.

---

## Project Structure

```
ARES-Project/
├── CMakeLists.txt       # Build configuration
├── package.xml          # Package metadata and dependencies
├── requirements.txt     # Python dependencies
├── .venv/               # Python virtual environment (not committed)
├── include/
│   └── ARES-Project/    # C++ header files
└── src/                 # C++ and Python source files
```

---

## Verifying the Installation

After building and sourcing, confirm the package is found by ROS2:

```bash
ros2 pkg list | grep ARES
```

---

## Contributing

1. Fork the repository and create a feature branch from `main`.
2. Follow the existing code style and keep headers under `include/ARES-Project/`.
3. Run the linter before submitting a PR:
   ```bash
   colcon test --packages-select ARES-Project
   colcon test-result --verbose
   ```
4. Open a pull request with a clear description of your changes.

---

## Troubleshooting

| Issue | Fix |
|---|---|
| `rosdep init` fails | Run `sudo rosdep init` only once; skip if already initialized |
| MAVROS GeographicLib error | Run the `install_geographiclib_datasets.sh` script (see Prerequisites) |
| Package not found after build | Re-source the workspace: `source ~/ros2_ws/install/setup.bash` |
| Serial port permission denied | Add your user to the `dialout` group: `sudo usermod -aG dialout $USER` then log out and back in |
| Python imports fail | Make sure the virtual environment is activated: `source .venv/bin/activate` |
| `pip install` errors | Upgrade pip first: `pip install --upgrade pip`, then retry |
