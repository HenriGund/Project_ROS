# TurtleBot2 – Autonomous Medication Delivery System

> **ROS 1 Noetic · TurtleBot2 / Kobuki · Kinect v1 · Hospital environment**

---

## Table of Contents

1. [Project Description](#1-project-description)
2. [System Architecture](#2-system-architecture)
3. [Sensor List](#3-sensor-list)
4. [Package Structure](#4-package-structure)
5. [ROS Topic Map](#5-ros-topic-map)
6. [State Machine](#6-state-machine)
7. [Installation and Workspace Setup](#7-installation-and-workspace-setup)
8. [Connecting to the Physical Robot](#8-connecting-to-the-physical-robot)
9. [Step-by-Step Usage](#9-step-by-step-usage)
10. [Configuration Reference](#10-configuration-reference)
11. [Troubleshooting](#11-troubleshooting)

---

## 1. Project Description

### Context

Hospital nursing staff spend a significant portion of their shift walking between the
**medication stock room** and the **nursing ward** to fetch and deliver medication boxes.
These repetitive trips are time-consuming and take nurses away from direct patient care.

### Goal

Replace those trips with an **autonomous mobile robot** — a TurtleBot2 — that handles
end-to-end transport of medication boxes.

### Mission flow

```
Nurse types request in terminal
        |
        v
Robot leaves base --> navigates to stock room
        |
        v  (text alert: "please place the box on the tray")
Robot waits at stock room
        |
        v  (Kinect RGB camera detects box placed on tray)
Robot navigates to ward
        |
        v  (text alert: "medication arrived, please remove the box")
Robot waits at ward
        |
        v  (Kinect RGB camera detects box removed from tray)
Robot returns to base
```

### Key design decision

The **Kinect v1** does double duty:

- **Depth image** --> converted to `/scan` by `depthimage_to_laserscan` for SLAM and
  obstacle avoidance. No separate LiDAR needed.
- **RGB image** --> OpenCV colour-blob detection confirms the box is on the tray.
  No RFID, no pressure sensor needed.

Alerts are **software-only** (text on the nurse terminal). No Arduino, no buzzer,
no extra wiring.

---

## 2. System Architecture

```
  Nurse terminal (hmi_interface)
       |  /delivery_request
       v
  mission_manager  <---  /load_status  <---  load_detector
       |                                           ^
       | /alert_cmd                                | /camera/rgb/image_raw
       v                                           |
  alert_node                              KINECT v1 (openni_launch)
       |  /mission_alert                           |
       v                                           | /camera/depth_registered/image_raw
  hmi_interface (displays alert)                   v
                                         depthimage_to_laserscan
                                                   |
                                                   | /scan
                                                   v
                                         amcl + move_base
                                                   |
                                                   | /cmd_vel
                                                   v
                                            Kobuki base
                                                   |
                                                   | /odom
                                                   v
                                            mission_manager
```

---

## 3. Sensor List

| Sensor | Role | ROS topic |
|--------|------|-----------|
| Kinect v1 depth | SLAM + obstacle avoidance (fake /scan) | `/camera/depth_registered/image_raw` |
| Kinect v1 RGB | Box presence detection on the tray | `/camera/rgb/image_raw` |
| Kobuki wheel encoders | Odometry (built-in, no wiring) | `/odom` |

> No LiDAR. No Arduino. No buzzer. No IR sensors. One Kinect handles everything.

### Kinect limitations

| Limitation | Impact | Mitigation |
|------------|--------|------------|
| Horizontal FOV ~57 degrees | Blind spots left/right and behind | Rotate 360 degrees at every junction during SLAM |
| Minimum range ~0.45 m | Cannot detect objects closer than 45 cm | Keep inflation_radius >= 0.35 m in costmap |
| Fails in direct sunlight | Unusable near bright windows | Hospital corridors with artificial lighting are fine |
| USB 2.0 power ~500 mA | May fail on a weak USB hub | Connect Kinect directly to the onboard PC |

---

## 4. Package Structure

```
PROJECT_ROS/                           <-- catkin workspace root
  src/
    turtlebot2_delivery/               <-- this package
      README.md
      package.xml
      CMakeLists.txt
      launch/
        slam.launch          Phase 1 - Kinect + gmapping + teleop
        navigation.launch    Phase 2 - Kinect + amcl + move_base
        delivery.launch      Phase 3 - full mission bringup
      src/
        mission_manager.py   Central state machine
        load_detector.py     OpenCV box detection --> /load_status
        hmi_interface.py     Nurse terminal keyboard interface
        alert_node.py        Text alerts --> /mission_alert
      config/
        waypoints.yaml       base / stock_room / ward poses
        costmap_params.yaml  Global + local costmap (Kinect-tuned)
        move_base_params.yaml  DWA local planner settings
      maps/
        hospital.pgm         Occupancy grid (replace after SLAM)
        hospital.yaml        Map metadata
```

---

## 5. ROS Topic Map

| Topic | Type | Publisher | Subscriber(s) |
|-------|------|-----------|---------------|
| `/delivery_request` | std_msgs/String | hmi_interface | mission_manager |
| `/mission_state` | std_msgs/String | mission_manager | hmi_interface, load_detector |
| `/load_status` | std_msgs/Bool | load_detector | mission_manager |
| `/alert_cmd` | std_msgs/String | mission_manager | alert_node |
| `/mission_alert` | std_msgs/String | alert_node | hmi_interface |
| `/camera/rgb/image_raw` | sensor_msgs/Image | openni_launch | load_detector |
| `/camera/depth_registered/image_raw` | sensor_msgs/Image | openni_launch | depthimage_to_laserscan |
| `/scan` | sensor_msgs/LaserScan | depthimage_to_laserscan | gmapping / amcl / move_base |
| `/odom` | nav_msgs/Odometry | Kobuki base | move_base, gmapping |
| `/cmd_vel` | geometry_msgs/Twist | move_base | Kobuki base |
| `/load_detector/debug_image` | sensor_msgs/Image | load_detector | RViz (optional) |

---

## 6. State Machine

States: IDLE --> TO_STOCK --> WAITING_LOAD --> TO_WARD --> DELIVERING --> RETURNING --> IDLE

```
IDLE
  |-- /delivery_request received --> TO_STOCK
        |-- arrived at stock room --> WAITING_LOAD
              |-- /load_status = True (box detected) --> TO_WARD
                    |-- arrived at ward --> DELIVERING
                          |-- /load_status = False (box removed) --> RETURNING
                                |-- arrived at base --> IDLE
```

On any navigation failure: robot falls back to RETURNING and goes home.

---

## 7. Installation and Workspace Setup

### 7.1 Prerequisites

Follow the tutorial at:
  https://perso.ensta.fr/~battesti/website/teachings_others/turtlebot2_noetic/

This gives you two source workspaces:

```
~/kobuki_ws/      Kobuki base drivers
~/turtlebot2_ws/  TurtleBot2 packages (turtlebot_bringup, turtlebot_teleop ...)
```

Verify both are ready:

```bash
echo $ROS_PACKAGE_PATH
# Must include paths to kobuki_ws/devel and turtlebot2_ws/devel
```

### 7.2 Install additional ROS packages

```bash
sudo apt update
sudo apt install \
  ros-noetic-gmapping \
  ros-noetic-amcl \
  ros-noetic-move-base \
  ros-noetic-dwa-local-planner \
  ros-noetic-map-server \
  ros-noetic-navfn \
  ros-noetic-costmap-2d \
  ros-noetic-openni-launch \
  ros-noetic-depthimage-to-laserscan \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  python3-opencv \
  python3-yaml
```

IMPORTANT: use `ros-noetic-openni-launch` (OpenNI 1.x), NOT openni2.
The turtlebot_bringup/3dsensor.launch uses openni_launch for the Kinect v1.

### 7.3 Configure ~/.bashrc

Add all of the following to `~/.bashrc`:

```bash
# ROS base
source /opt/ros/noetic/setup.bash

# TurtleBot2 workspace chain (order matters)
source ~/kobuki_ws/devel/setup.bash
source ~/turtlebot2_ws/devel/setup.bash
source ~/PROJECT_ROS/devel/setup.bash

# TurtleBot2 hardware configuration
export TURTLEBOT_BASE=kobuki
export TURTLEBOT_STACKS=hexagons
export TURTLEBOT_3D_SENSOR=kinect
export TURTLEBOT_SERIAL_PORT=/dev/kobuki   # udev symlink (see Section 8)
```

Reload:

```bash
source ~/.bashrc
```

### 7.4 Build PROJECT_ROS

Source the upstream workspaces first, then build:

```bash
source ~/kobuki_ws/devel/setup.bash
source ~/turtlebot2_ws/devel/setup.bash

cd ~/PROJECT_ROS
catkin_make

source ~/PROJECT_ROS/devel/setup.bash
```

Verify:

```bash
rospack find turtlebot2_delivery
# Expected output: /home/student/PROJECT_ROS/src/turtlebot2_delivery
```

---

## 8. Connecting to the Physical Robot

### 8.1 Physical USB connections

| Device | Port | How to verify |
|--------|------|---------------|
| Kobuki base (FTDI) | `/dev/ttyUSB0` (or `/dev/kobuki` after udev) | `ls /dev/ttyUSB*` |
| Kinect v1 | USB bulk device - no serial port | `lsusb \| grep -i microsoft` |

Expected output of `lsusb | grep -i microsoft`:

```
Bus ... Device ...: ID 045e:02ae Microsoft Corp. Xbox NUI Camera
Bus ... Device ...: ID 045e:02b0 Microsoft Corp. Xbox NUI Motor
```

### 8.2 udev rule for Kobuki

Check if the TurtleBot2 package already installed one:

```bash
ls /etc/udev/rules.d/ | grep kobuki
```

If not present, create it:

```bash
sudo nano /etc/udev/rules.d/99-kobuki.rules
```

Content:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="kobuki", MODE="0666"
```

Apply:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Add your user to the dialout group (once):

```bash
sudo usermod -aG dialout $USER
# Log out and back in
```

### 8.3 ROS network (two-machine setup)

On the **robot PC** (the ROS master), add to `~/.bashrc`:

```bash
export ROS_MASTER_URI=http://192.168.1.10:11311   # robot PC IP
export ROS_IP=192.168.1.10
```

On the **operator laptop**, add to `~/.bashrc`:

```bash
export ROS_MASTER_URI=http://192.168.1.10:11311   # same robot IP
export ROS_IP=192.168.1.20                         # this laptop's IP
```

Single-machine (no network):

```bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1
```

### 8.4 Pre-flight checks

Run these after connecting the hardware, before launching:

```bash
# 1. Kinect is seen by the OS
lsusb | grep -i microsoft

# 2. (After starting navigation.launch) Kinect depth stream
rostopic hz /camera/depth_registered/image_raw   # expect ~30 Hz

# 3. Fake laser scan
rostopic hz /scan                                 # expect ~30 Hz

# 4. Kinect RGB stream
rostopic hz /camera/rgb/image_raw                 # expect ~30 Hz

# 5. Kobuki odometry
rostopic echo /odom -n 1
```

---

## 9. Step-by-Step Usage

### Phase 1 - SLAM (mapping)

Run once to build the hospital floor map.

```bash
roslaunch turtlebot2_delivery slam.launch
```

Starts: Kobuki base, Kinect (openni_launch + depthimage_to_laserscan), gmapping,
keyboard teleop, RViz.

Drive with the keyboard:

```
i = forward      , = backward
j = rotate left  l = rotate right
k = stop
```

Mapping strategy for the Kinect 57-degree FOV:
- Drive at max 0.2 m/s.
- At every corridor junction: stop and rotate 360 degrees.
- Cover all rooms: stock room entrance, ward entrance, base/dock area.

Save the map when complete:

```bash
rosrun map_server map_saver -f $(rospack find turtlebot2_delivery)/maps/hospital
```

This creates `maps/hospital.pgm` and `maps/hospital.yaml`. Stop slam.launch.

---

### Phase 2 - Record waypoints

Start navigation on the saved map:

```bash
# Terminal 1
roslaunch turtlebot2_delivery navigation.launch
```

In RViz, click "2D Pose Estimate" and click the robot's current position.

Drive the robot to each location using teleop in a second terminal:

```bash
# Terminal 2
roslaunch turtlebot_teleop keyboard_teleop.launch
```

At each location, read the pose:

```bash
rostopic echo /amcl_pose -n 1
```

Copy position.x, position.y, orientation.z, orientation.w into
`config/waypoints.yaml`. Example:

```yaml
waypoints:
  base:
    position:    {x: 0.00, y: 0.00}
    orientation: {x: 0.0, y: 0.0, z: 0.000, w: 1.000}

  stock_room:
    position:    {x: 3.52, y: 1.18}
    orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}

  ward:
    position:    {x: 7.95, y: 4.47}
    orientation: {x: 0.0, y: 0.0, z: 0.000, w: 1.000}
```

---

### Phase 3 - Tune camera colour detection

The load_detector identifies the medication box by its colour in HSV.

Run a live HSV tuner while looking at the box on the tray:

```bash
python3 - <<'EOF'
import cv2, numpy as np
cap = cv2.VideoCapture(0)
def nothing(x): pass
cv2.namedWindow('HSV Tuner')
for n, v in [('H_lo',100),('H_hi',130),('S_lo',80),('S_hi',255),('V_lo',50),('V_hi',255)]:
    cv2.createTrackbar(n, 'HSV Tuner', v, 255, nothing)
while True:
    ret, frame = cap.read()
    if not ret: break
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lo = np.array([cv2.getTrackbarPos(n,'HSV Tuner') for n in ('H_lo','S_lo','V_lo')])
    hi = np.array([cv2.getTrackbarPos(n,'HSV Tuner') for n in ('H_hi','S_hi','V_hi')])
    cv2.imshow('HSV Tuner', cv2.bitwise_and(frame, frame, mask=cv2.inRange(hsv, lo, hi)))
    if cv2.waitKey(1) == 27: break
cap.release(); cv2.destroyAllWindows()
EOF
```

Update `src/load_detector.py` with your values:

```python
HSV_LOWER = np.array([100,  80,  50], dtype=np.uint8)   # replace
HSV_UPPER = np.array([130, 255, 255], dtype=np.uint8)   # replace
PIXEL_THRESHOLD = 3000                                   # tune for your camera distance
```

Check the detection live (while delivery.launch is running):

```bash
rosrun image_view image_view image:=/load_detector/debug_image
```

---

### Phase 4 - Run the full mission

Two terminals needed.

**Terminal 1 - Navigation stack**

```bash
source ~/.bashrc
roslaunch turtlebot2_delivery navigation.launch
```

Wait until:
- RViz shows the Kinect point cloud and the map.
- AMCL prints "Received a 2D pose estimate".

Set the initial robot position with "2D Pose Estimate" in RViz.

**Terminal 2 - Delivery mission**

```bash
source ~/.bashrc
roslaunch turtlebot2_delivery delivery.launch
```

An xterm window opens as the nurse terminal. The Kinect is already streaming
from Terminal 1 - delivery.launch does NOT restart the Kinect.

**Nurse terminal example session:**

```
--------------------------------------------------------
  TurtleBot2 Medication Delivery - Nurse Terminal
--------------------------------------------------------
[HMI] Enter delivery request: Paracetamol 500mg to Room 3
[HMI] Request sent: "Paracetamol 500mg to Room 3"

[STATUS] Robot is heading to the stock room...

========================================================
[ALERT] Robot is at the STOCK ROOM. Please place the medication box on the tray.
========================================================
    <-- place the box on the tray -->
    (Kinect RGB detects the box automatically - no button to press)

[STATUS] Robot is delivering medication to the ward...

========================================================
[ALERT] Robot is at the WARD. Please remove the medication box from the tray.
========================================================
    <-- remove the box from the tray -->
    (Kinect RGB detects the absence automatically)

[STATUS] Robot is returning to base...
[STATUS] Robot is at base - ready for a new request.
```

---

## 10. Configuration Reference

### config/waypoints.yaml

| Key | Description |
|-----|-------------|
| `base` | Charging dock and home position |
| `stock_room` | Robot stops here to receive the box |
| `ward` | Robot stops here to deliver the box |

All coordinates are in the `map` frame (metres). Use `rostopic echo /amcl_pose -n 1`
to read values after driving to each location.

### config/costmap_params.yaml

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_radius` | 0.23 m | Kobuki radius + 5 cm safety margin |
| `inflation_radius` (global) | 0.35 m | Minimum clearance from walls in global plan |
| `inflation_radius` (local) | 0.30 m | Minimum clearance in local obstacle layer |
| `obstacle_range` | 4.0 m | Max range at which Kinect marks obstacles |
| `raytrace_range` | 4.5 m | Max range for clearing free space |
| `sensor_frame` | `camera_depth_frame` | TF frame output by depthimage_to_laserscan |

### config/move_base_params.yaml

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_vel_x` | 0.35 m/s | Reduced for indoor safety |
| `xy_goal_tolerance` | 0.15 m | Acceptable position error at goal |
| `yaw_goal_tolerance` | 0.15 rad | Acceptable angle error at goal (~8 deg) |
| `sim_time` | 1.5 s | DWA trajectory simulation horizon |

### src/load_detector.py

| Constant | Default | Description |
|----------|---------|-------------|
| `IMAGE_TOPIC` | `/camera/rgb/image_raw` | Kinect RGB topic from openni_launch |
| `HSV_LOWER` | `[100, 80, 50]` | Lower HSV bound for box colour |
| `HSV_UPPER` | `[130, 255, 255]` | Upper HSV bound for box colour |
| `PIXEL_THRESHOLD` | 3000 | Min coloured pixels to confirm box present |
| `ACTIVE_STATES` | `WAITING_LOAD, DELIVERING` | States in which detection runs (saves CPU) |

---

## 11. Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `rospack find turtlebot2_delivery` fails | Workspace not built or not sourced | `cd ~/PROJECT_ROS && catkin_make` with all workspaces sourced; run `source ~/.bashrc` |
| `cannot find minimal.launch` or `3dsensor.launch` | turtlebot2_ws not in ROS_PACKAGE_PATH | Check `echo $ROS_PACKAGE_PATH`; source `~/turtlebot2_ws/devel/setup.bash` |
| Kinect not found by openni_launch | USB not powered or wrong driver | Connect directly to PC (no hub); run `lsusb \| grep -i microsoft` to confirm |
| `openni_launch` package not found | Package not installed | `sudo apt install ros-noetic-openni-launch` |
| `/scan` not publishing | depthimage_to_laserscan not started | Check `TURTLEBOT_3D_SENSOR=kinect`; run `rostopic hz /scan` after navigation.launch |
| AMCL does not converge (robot lost on map) | Initial pose not set | Use "2D Pose Estimate" in RViz; drive the robot 0.5 m to gather scan data |
| Robot navigates but grazes walls | Inflation radius too small | Increase `inflation_radius` in costmap_params.yaml to 0.40 |
| `camera_depth_frame` TF missing | openni_launch publish_tf conflict | Ensure `publish_tf:=false` in 3dsensor include inside navigation.launch |
| Box never detected | HSV range does not match box colour | Re-run HSV tuner (Phase 3); check `/load_detector/debug_image` in RViz |
| Box always detected | PIXEL_THRESHOLD too low or bright ambient light | Increase `PIXEL_THRESHOLD`; cover the tray from direct light |
| `catkin_make` fails with missing dependency | Wrong build order | Source kobuki_ws and turtlebot2_ws before running catkin_make in PROJECT_ROS |
