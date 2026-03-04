# IRB120 (ROS 2 + Gazebo Sim + MoveIt 2)

This repository contains a **ROS 2 simulation + MoveIt 2 planning setup** for the **ABB IRB120** robot, plus a small toolset of Python nodes for:

- running a Gazebo Sim (Fortress/Garden) scene with `gz_ros2_control`
- streaming simple joint trajectories to a `JointTrajectoryController`
- sending MoveIt `MoveGroup` action goals (single goal + pick & place sequence)
- spawning and bridging a **static RGB/Depth/Stereo camera rig** and visualizing it with an OpenCV viewer

> ⚠️ Note: The **pick & place demo is still clunky** (tuning/robustness is not finished). See **Known issues & limitations**.

## Demo video

https://github.com/user-attachments/assets/e5eb057e-8b84-4c72-8c68-88a2e95a0e2d
---





## Repo layout (as in this zip)

```
IRB120/                         # ROS 2 package (ament_python)
  package.xml
  setup.py
  config/
    IRB120_controller.yaml
  launch/
    demo.launch.py              # RViz-only: fake joint_state publisher
    gazebo.launch.py            # Gazebo Sim + controllers + optional trajectory streamer + camera spawn/bridge
    moveit.launch.py            # Gazebo Sim + controllers + MoveIt move_group + RViz
    pnp.launch.py               # Gazebo Sim + controllers + MoveIt move_group + pick & place sender
  urdf/
    IRB120.urdf.xml             # robot + gz_ros2_control plugin
    IRB120.rviz                 # RViz config for demo.launch.py
    camera.sdf                  # static camera rig (RGB + depth + stereo)
    camera.urdf.xml             # URDF version of the camera rig (mostly for visualization)
    meshes/
  worlds/
    IRB120_empty_world.sdf      # includes Sensors system plugin
    IRB120_green_cube.sdf
  IRB120/
    state_publisher.py
    send_trajectory.py
    send_trajectory_pnp.py
    camera_viewer.py

IRB120_moveit_config/           # MoveIt Setup Assistant output (ament_cmake)
  config/
  launch/
  package.xml
  CMakeLists.txt
```

### Important detail about MoveIt config files
The **Python package `IRB120` installs the MoveIt config files** from `IRB120_moveit_config/config/*` into:

```
share/IRB120/moveit_config/
```

That’s why the custom launch files under `IRB120/launch/` reference `.../share/IRB120/moveit_config/...`.

---

## Requirements

This codebase is intended for:

- **ROS 2 Humble**
- **Gazebo Sim** (Fortress / Garden via `ros_gz_sim`)
- **MoveIt 2**
- `ros2_control` + `gz_ros2_control`
- OpenCV + `cv_bridge` (for the camera viewer)

If you already have ROS 2 set up, the most reliable way to pull dependencies is:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

> Note: `IRB120/package.xml` currently lists only a subset of runtime deps (some nodes also use `moveit_msgs`, `control_msgs`, `trajectory_msgs`, `geometry_msgs`, `shape_msgs`, `cv_bridge`, etc.). If `rosdep` misses anything, install the missing ROS packages based on the error messages.

---

## Build

1) Create a workspace and put the repo in `src/`:

```bash
mkdir -p ~/irb120_ws/src
cd ~/irb120_ws/src
# copy/clone this repo here so you have: ~/irb120_ws/src/IRB120/
```

2) Build:

```bash
cd ~/irb120_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Launch files (what each one does)

### 1) RViz-only demo (no Gazebo)
Publishes a **synthetic /joint_states** stream (sinusoidal joints) so you can view the model in RViz without sim.

```bash
ros2 launch IRB120 demo.launch.py
```

What runs:
- `robot_state_publisher` using `urdf/IRB120.urdf.xml`
- `IRB120/state_publisher.py` publishing `/joint_states`
- RViz using `urdf/IRB120.rviz`

---

### 2) Gazebo simulation (empty world) + controllers + optional joint trajectory streamer + camera

```bash
ros2 launch IRB120 gazebo.launch.py
```

What runs:
- Gazebo Sim (`ros_gz_sim`) with `worlds/IRB120_empty_world.sdf`
- spawns the robot from `/robot_description`
- spawns controllers (see **Controllers** section)
- bridges `/clock` from Gazebo → ROS 2
- spawns a **static camera rig** (from `urdf/camera.sdf`)
- bridges camera topics (RGB + depth + stereo)
- optionally runs `send_trajectory` after a short delay (continuous motion demo)

Useful launch args (current behavior):

```bash
# disable trajectory streamer
ros2 launch IRB120 gazebo.launch.py run_streamer:=false

# change which topic the OpenCV viewer should subscribe to (viewer is usually run separately)
ros2 launch IRB120 gazebo.launch.py camera_topic:=/cam/rgb/image_raw
```

> Known quirk: `gazebo.launch.py` declares a `world:=...` argument, but the current code overwrites the variable and does **not** actually wire the `LaunchConfiguration` into `gz_args`. If you need a different world, edit the launch file (or fix it to use `LaunchConfiguration('world')`).

---

### 3) MoveIt + Gazebo simulation (green cube world)

```bash
ros2 launch IRB120 moveit.launch.py
```

What runs:
- Gazebo Sim with `worlds/IRB120_green_cube.sdf`
- robot spawn + controllers + `/clock` bridge
- MoveIt `move_group`
- RViz using `moveit_config/moveit.rviz`

---

### 4) Pick & place demo (MoveIt action client)

```bash
ros2 launch IRB120 pnp.launch.py
```

What runs:
- Gazebo Sim with `worlds/IRB120_green_cube.sdf`
- robot spawn + controllers + `/clock` bridge
- MoveIt `move_group`
- after ~6 seconds, runs `send_trajectory_pnp` (pick/place sequence)

Tip: this launch does **not** start RViz. You can start it manually:

```bash
RVIZ_CFG=$(ros2 pkg prefix IRB120)/share/IRB120/moveit_config/moveit.rviz
rviz2 -d "$RVIZ_CFG"
```

---

## Controllers

Controller configuration file:

- `IRB120/config/IRB120_controller.yaml`

Controllers used by launch files:

- `arm_controller` (JointTrajectoryController)
- `gripper_controller` (JointTrajectoryController)
- `joint_state_broadcaster` (JointStateBroadcaster)

Check controller status:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

> Known quirk: `IRB120_controller.yaml` currently defines `arm_broadcaster` but the launch files spawn `joint_state_broadcaster`. If the spawner fails, align the names (either rename the controller in YAML or spawn the one that exists).

---

## Nodes / console scripts

These executables are registered in `setup.py`:

- `state_publisher` → publishes fake `/joint_states` for RViz-only demo
- `send_trajectory` → sends alternating A/B joint goals via `/arm_controller/follow_joint_trajectory`
- `send_trajectory_moveit` → sends a single Cartesian end-effector goal via MoveIt `MoveGroup` action (`/move_action`)
- `send_trajectory_pnp` → pick & place sequence via MoveIt `MoveGroup` action + optional gripper open/close
- `camera_viewer` → OpenCV visualizer for RGB/depth/stereo topics

Run a node directly (after `source install/setup.bash`):

```bash
ros2 run IRB120 send_trajectory
```

---

## Camera system

### Camera model
The camera rig is defined in:

- `urdf/camera.sdf` (used for spawning into Gazebo)
- `urdf/camera.urdf.xml` (URDF version; mostly for visualization)

It contains:

- RGB camera:  `/cam/rgb/image_raw` (1280×720 @ 30 Hz)
- Depth camera: `/cam/depth/image_raw` (640×480 @ 30 Hz)
- Stereo cameras:
  - `/cam/stereo/left/image_raw`
  - `/cam/stereo/right/image_raw`

Gazebo will also publish corresponding `camera_info` topics (bridged in `gazebo.launch.py`):

- `/cam/rgb/camera_info`, `/cam/depth/camera_info`, `/cam/stereo/left/camera_info`, `/cam/stereo/right/camera_info`

### Start the camera (recommended: via gazebo.launch)

```bash
ros2 launch IRB120 gazebo.launch.py
```

This launch **spawns** the camera and starts a `ros_gz_bridge parameter_bridge` for the topics.

### Start the camera viewer (RGB / Depth / Stereo / Combined)

> Gazebo camera streams are typically **BEST_EFFORT QoS**, so make sure your subscribers handle that (the provided viewer does).

**RGB only**

```bash
ros2 run IRB120 camera_viewer --ros-args \
  -p topic:=/cam/rgb/image_raw \
  -p window_name:='IRB120 RGB'
```

**Depth (colormapped)**

```bash
ros2 run IRB120 camera_viewer --ros-args \
  -p topic:=/cam/depth/image_raw \
  -p depth:=true \
  -p window_name:='IRB120 Depth'
```

**Stereo depth-from-disparity preview**

```bash
ros2 run IRB120 camera_viewer --ros-args \
  -p stereo_mode:=true \
  -p left_topic:=/cam/stereo/left/image_raw \
  -p right_topic:=/cam/stereo/right/image_raw \
  -p stereo_num_disparities:=128 \
  -p stereo_block_size:=7 \
  -p window_name:='IRB120 Stereo'
```

**Combined dashboard (Left + Right + RGB + Depth)**

```bash
ros2 run IRB120 camera_viewer --ros-args \
  -p combined_mode:=true \
  -p window_name:='IRB120 Camera Dashboard'
```

Quit the viewer by focusing the OpenCV window and pressing **`q`**.

### Camera debugging commands

List camera topics:

```bash
ros2 topic list | grep -E '^/cam'
```

Echo camera messages (use BEST_EFFORT):

```bash
ros2 topic echo /cam/rgb/image_raw --qos-reliability best_effort
ros2 topic echo /cam/rgb/camera_info --qos-reliability best_effort
```

If you see topics but no frames:
- confirm the **Sensors** system plugin is present in the world (it is included in `IRB120_empty_world.sdf`)
- confirm the `ros_gz_bridge parameter_bridge` is running

### Running the camera in MoveIt / pick&place worlds
`moveit.launch.py` and `pnp.launch.py` **do not** spawn the camera rig by default.

If you want the camera there too, you can either:
- add the `spawn_camera` + `camera_bridge` blocks from `gazebo.launch.py` to those launch files, **or**
- spawn and bridge it manually:

```bash
# 1) spawn the camera model
CAMERA_SDF=$(ros2 pkg prefix IRB120)/share/IRB120/urdf/camera.sdf
ros2 run ros_gz_sim create \
  -name static_camera \
  -x 1.0 -y 0.0 -z 0.5 \
  -R 0.0 -P 0.0 -Y 3.14159 \
  -file "$CAMERA_SDF"

# 2) bridge the topics
ros2 run ros_gz_bridge parameter_bridge \
  /cam/rgb/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
  /cam/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /cam/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
  /cam/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /cam/stereo/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
  /cam/stereo/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /cam/stereo/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
  /cam/stereo/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo
```

---

## Pick & place parameters (pnp.launch.py)

`pnp.launch.py` starts `send_trajectory_pnp` with parameters like:

- `cube_xyz`: where the cube is expected (in `base_link` frame)
- `approach_above_top`, `pick_above_top`: approach offsets
- `lift_height`: z height for lifting
- `place_xyz`: placement location
- tolerances and MoveIt planning params

Run it manually with different tuning:

```bash
ros2 run IRB120 send_trajectory_pnp --ros-args \
  -p cube_xyz:='[0.60, 0.00, 0.30]' \
  -p place_xyz:='[0.15, -0.25, 0.10]' \
  -p approach_above_top:=0.12 \
  -p position_tolerance:=0.03 \
  -p max_velocity_scaling_factor:=0.10 \
  -p max_acceleration_scaling_factor:=0.10
```

---

## Known issues & limitations

1) **Pick & place is still clunky / not fully perfect**
- The demo is open-loop: it does not *perceive* the object; it relies on the hard-coded `cube_xyz`.
- The `IRB120_green_cube.sdf` cube pose/size may not match the default `cube_xyz` used in `pnp.launch.py`, so you may need to retune `cube_xyz`, approach heights, and tolerances.
- The robot can fail MoveIt planning due to reachability, collision, or constraints.

2) **Controller naming mismatch**
- YAML defines `arm_broadcaster` but launch files spawn `joint_state_broadcaster`.

3) **`gazebo.launch.py` has args not fully wired**
- `world:=...` is declared but currently not respected (world path is hard-coded in the file).
- `view_camera:=true` is declared, but the `camera_viewer` node is currently commented out in the `LaunchDescription` list.

4) **/clock bridge syntax differs between launch files**
- `gazebo.launch.py` uses the common bracket form: `...Clock[gz.msgs.Clock]`
- `moveit.launch.py` and `pnp.launch.py` currently use `...Clock@gz.msgs.Clock` (may break /clock bridging depending on ros_gz_bridge version).

5) **URDF plugin uses a relative controller YAML path**
- In `IRB120.urdf.xml`, the gz_ros2_control plugin references:
  `./IRB120/config/IRB120_controller.yaml`
  which may not resolve when running from a different working directory.

---

## Troubleshooting

### Controllers won’t start / action server not available

- Check controller status:
  ```bash
  ros2 control list_controllers
  ```
- Check the action server:
  ```bash
  ros2 action list | grep follow_joint_trajectory
  ```
- If `joint_state_broadcaster` fails to spawn, align YAML/controller names.

### No camera frames

- Confirm the camera rig is spawned and topics exist:
  ```bash
  ros2 topic list | grep /cam
  ```
- Use BEST_EFFORT QoS when echoing:
  ```bash
  ros2 topic echo /cam/rgb/image_raw --qos-reliability best_effort
  ```
- Make sure the **Sensors** system plugin is in your world.

### Sim time /clock problems

- Make sure `/clock` is bridged:
  ```bash
  ros2 topic echo /clock
  ```
- If it’s missing in `moveit.launch.py` / `pnp.launch.py`, update the bridge string to the bracket form:
  ```
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]
  ```

---

## Next steps (recommended improvements)

- Fix the YAML/launch controller name mismatch.
- Wire `world:=...` and re-enable `camera_viewer` auto-start option in `gazebo.launch.py`.
- Add the camera rig to `moveit.launch.py` / `pnp.launch.py` or provide a dedicated `camera.launch.py`.
- Make pick & place less clunky by:
  - matching cube pose/size to the world model, or
  - using the camera topics for detection/pose estimation and feeding that into `cube_xyz`.
