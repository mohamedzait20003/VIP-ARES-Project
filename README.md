# ARES-Project

**Autonomous Drone Control System** built on ROS2 and ArduPilot.

Maintainer: Mohammed Zaitoun — mzaitoun@purdue.edu  
License: MIT

> **Environment:** This project is developed and run inside **WSL2** (Windows Subsystem for Linux) on Ubuntu 24.04.

---

## WSL2 Setup

Before anything else, make sure you are on **WSL2** (not WSL1) and running **Ubuntu 24.04**.

### Verify WSL version (run in PowerShell on Windows)

```powershell
wsl --list --verbose
```

If the VERSION column shows `1`, upgrade:

```powershell
wsl --set-version Ubuntu-24.04 2
```

### Configure your git identity (one-time, inside WSL)

```bash
git config --global user.name "Your Name"
git config --global user.email "you@example.com"
```

### Enable systemd (required by some ROS2 services)

Add the following to `/etc/wsl.conf` inside WSL (create the file if it doesn't exist):

```ini
[boot]
systemd=true
```

Then restart WSL from PowerShell:

```powershell
wsl --shutdown
```

---

## Prerequisites

Make sure the following are installed on your system before setting up this workspace.

### 1. ROS2

This package targets **ROS2 Jazzy** (Ubuntu 24.04).

Follow the official installation guide:  
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html

Source your ROS2 installation in every new terminal (or add it to `~/.bashrc`):

```bash
source /opt/ros/jazzy/setup.bash
```

### 2. MAVROS

MAVROS provides the bridge between ROS2 and ArduPilot via MAVLink.

```bash
sudo apt update
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras
```

Install the required GeographicLib datasets (needed by MAVROS):

```bash
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
```

### 3. Additional ROS2 Dependencies

```bash
sudo apt install \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-nav-msgs \
  ros-jazzy-std-msgs
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
cd ~/ARES-Project
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

### 1. Clone this repository

```bash
git clone https://github.com/mohamedzait20003/VIP-ARES-Project.git ARES-Project
```

### 2. Set up the Python virtual environment

```bash
cd ~/ARES-Project
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### 4. Initialize rosdep and install dependencies

```bash
cd ~/ARES-Project
sudo rosdep init
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

If running ArduPilot SITL **on Windows** (outside WSL), use the WSL2 host IP instead of `127.0.0.1`. Find it with:

```bash
cat /etc/resolv.conf | grep nameserver | awk '{print $2}'
```

Then launch MAVROS with that IP:

```bash
ros2 launch mavros apm.launch fcu_url:=udp://<host-ip>:14550@14555
```

If running SITL **inside WSL** (same machine), `127.0.0.1` works directly:

```bash
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555
```

### Hardware (Flight Controller via USB/Serial)

WSL2 does not expose USB devices by default. You must first forward the USB port from Windows using **usbipd-win**.

#### 1. Install usbipd-win (run in PowerShell as Administrator on Windows)

```powershell
winget install usbipd
```

#### 2. Attach the flight controller to WSL (run in PowerShell as Administrator on Windows)

```powershell
usbipd list                        # find your device's BUSID
usbipd bind --busid <BUSID>        # one-time, marks device as shareable
usbipd attach --wsl --busid <BUSID>
```

#### 3. Verify the device is visible in WSL

```bash
ls /dev/ttyACM*   # or /dev/ttyUSB*
```

#### 4. Launch MAVROS

```bash
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:57600
```

Replace `/dev/ttyACM0` and baud rate with the values appropriate for your setup.

> **Note:** `usbipd attach` must be re-run every time you reconnect the device or restart WSL.

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
| Serial port permission denied | Add your user to the `dialout` group: `sudo usermod -aG dialout $USER`, then restart WSL |
| Python imports fail | Make sure the virtual environment is activated: `source .venv/bin/activate` |
| `pip install` errors | Upgrade pip first: `pip install --upgrade pip`, then retry |
| `/dev/ttyACM0` not found in WSL | Re-attach USB with `usbipd attach --wsl --busid <BUSID>` from an Admin PowerShell |
| SITL can't connect (UDP) | Use the WSL2 host IP from `/etc/resolv.conf` instead of `127.0.0.1` |
| WSL version is 1 | Run `wsl --set-version Ubuntu-24.04 2` in PowerShell, then restart |
| ROS2 daemon issues | Restart with `ros2 daemon stop && ros2 daemon start` |
