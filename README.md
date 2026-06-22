# VLM Object-Goal Navigation — ROS2 Humble + Gazebo

Tell a mobile robot **"go to the red box"** and it finds the object with a local
vision-language model and drives there. There is no hardcoded table of object
locations: the robot searches, the VLM detects the named object in the live
RGB-D stream, and the sighting is back-projected through depth and TF into a
metric goal for Nav2.

A diff-drive robot with a 2D lidar and an RGB-D camera runs in a small Gazebo
world. Everything — including the VLM — runs CPU-only;
YOLO-World on torch-CPU is ~0.16 s per frame.

**NOTE:** Two model files are missing due to file size constraints. In order to run this project, you need to download those files. You may find them [here](https://drive.google.com/drive/folders/1MYLxP5tOjvTqHSZ6vxf3P_Akh41utirH?usp=drive_link). Place "yolov8s-world.pt" in the parent folder and copy the "/weights" folder entirely, as is. **The project will not run as intended without the models.**

## How the VLM goal-finding works

The detector ([vlm_detector.py](src/vlm_navigation/vlm_navigation/vlm_detector.py))
is a **hybrid** that combines an open-vocabulary VLM with a cheap color anchor:

- **YOLO-World (the VLM) directs.** Given a free-form phrase ("the red box"), it
  decides whether the target class is present in a frame and roughly where.
- **HSV anchors.** Inside the VLM's region of interest, an HSV pass finds the
  exact pixel centroid, which is what gets back-projected to a metric goal.

The VLM does the semantic *find*; HSV refines it to a precise pixel. This keeps
the open-vocabulary phrasing while staying robust on small, flat-shaded objects
the VLM only detects at low confidence. Add a new object by editing
`TARGET_VOCAB` / `HSV_RANGES` in the detector.

## Packages

- **vlm_description** — URDF/xacro robot, Gazebo sensors and plugins, the sim
  world, ros_gz bridge, RViz config, and gazebo bringup.
- **vlm_navigation** — slam_toolbox config, Nav2 params, SLAM + Nav2 launch
  files, and the `object_goal_nav` node.

## Build

```bash
cd ~/vlm_ws
source /opt/ros/humble/setup.bash
colcon build
```

The VLM node also needs YOLO-World and the CLIP text encoder (torch CPU assumed
present):

```bash
pip install ultralytics clip-anytorch
```

The first run downloads the YOLO-World and CLIP weights once.

## Run

Every terminal first sources the workspace and pins the RMW to FastDDS (the
default CycloneDDS breaks Nav2 on loopback):

```bash
cd ~/vlm_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Start clean between runs (clears stale sim/nav processes and the ROS daemon):

```bash
./kill_sim.sh
```

**Terminal 1 — bring up Gazebo + Nav2** (one command; Nav2 starts a few seconds
after Gazebo so sensors and TF are ready first):

```bash
ros2 launch vlm_navigation bringup.launch.py
```

Wait ~30–45 s for all Nav2 nodes to activate. Args: `rviz:=false` to skip RViz,
`nav2_delay:=<seconds>` to change the Gazebo→Nav2 gap.

<details>
<summary>Or launch the two stacks separately</summary>

```bash
# Terminal 1 — Gazebo
ros2 launch vlm_description gazebo.launch.py rviz:=false
#   software_render:=true   # Mesa llvmpipe, if sensors don't render on an iGPU
#   render_engine:=ogre     # older engine fallback

# Terminal 2 — Nav2 + AMCL (a prebuilt map is included)
ros2 launch vlm_navigation nav2.launch.py rviz:=true
```

To build a new map instead, use `vlm_navigation slam.launch.py`, drive around,
and save with `map_saver_cli`.
</details>

**Terminal 2 — object-goal navigation:**

```bash
ros2 run vlm_navigation object_goal_nav --ros-args -p target:="red box"
```

Free-form phrases work: `red box`, `the green cube`, `blue block`, etc. The robot
searches (rotates in place, then drives to vantage waypoints), the VLM finds the
object, back-projects the sighting to a metric goal, drives to a ~1 m standoff,
and re-confirms:

```
Searching (rotate in place)...
  VLM sighting at step 1: Detection(red, px=(82,290), area=2929, vlm_conf=0.04)
Localized 'red box' at map (3.48,2.97), range 4.10m.
Navigating to standoff pose (2.50,2.13)...
✓ CONFIRMED 'red box' in view
```

Parameters:
- `target` — the phrase to find (default `red box`).
- `skip_nav:=true` — detect from where the robot stands, no driving (needs only
  Gazebo, not Nav2).
- `search_in_place_only:=true` — rotate at the start pose, don't drive to the
  search waypoints.

## Notes

- **FastDDS is required.** Export `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` in every
  shell that talks to the stack, or nodes won't discover each other.
- **One Gazebo at a time.** A leftover bridge publishing a second `/clock` makes
  sim time jump and produces "extrapolation into the past" planner errors;
  `./kill_sim.sh` clears it.
- **Search waypoints are world-specific.** `SEARCH_WAYPOINTS` in the node are
  open spots that collectively see every corner; retune them if you change the
  world.
- **VLM confidence is low on flat-shaded primitives (~0.04–0.12).** `VLM_CONF` is
  deliberately low and HSV does the precise localization; raise it for textured
  or larger objects.
- **CameraInfo intrinsics are published at half resolution** by ros_gz (K is for
  320×240 while images are 640×480, and `width/height` don't reflect it). The
  back-projection rescales K from the principal point before un-projecting;
  without it, off-center goals land in the wrong place.
