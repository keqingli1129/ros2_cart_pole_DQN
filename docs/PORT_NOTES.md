# ROS1 → ROS2 Port Notes

This workspace was originally written for ROS1 (catkin, Gazebo classic,
`ros_control`, `rospy`). It was ported to **ROS2 Jazzy + Gazebo Harmonic
(`gz-sim8`)** and merged to `master` on 2026-07-20. This doc captures the
architecture decisions and the real bugs found while getting it running live,
since the working notes from that process (subagent reports, review diffs)
were scratch files that didn't survive past the port itself.

The original implementation plan is at
[`docs/superpowers/plans/2026-07-20-ros1-to-ros2-port.md`](superpowers/plans/2026-07-20-ros1-to-ros2-port.md)
if you want the full task-by-task breakdown; this doc is the shorter "what
and why" version plus what changed after that plan during live testing.

## How to build and run

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch robot_launch launch_simulation.launch.py
```

No unit test suite exists (this is a robotics simulation project) — the
verification method is `colcon build` succeeding plus a live run showing
`/joint_states` publishing and the DQN node completing training episodes
with no tracebacks.

## Package summary

| Package | ROS1 build type | ROS2 build type | Notes |
|---|---|---|---|
| `robot_description` | catkin | `ament_cmake` | URDF/xacro/meshes only, plus the two gz-sim plugins (see below) |
| `robot_control` | catkin | `ament_cmake` | Now just launches `robot_state_publisher` |
| `commander` | catkin | `ament_python` | DQN node ported `rospy` → `rclpy` |
| `robot_launch` | catkin | `ament_cmake` | Top-level launch, world file, `ros_gz_bridge` wiring |

## Key architecture decision: no `ros2_control`

`ros2_control`/`gz_ros2_control` are not installed on this machine, and
installing new system packages was out of scope for the port. Instead of
the ROS1 `ros_control` + `gazebo_ros_control` stack, actuation and
joint-state feedback use **gz-sim's built-in system plugins**, baked
directly into `robot_description/robot/cart_pole.urdf.xacro`:

- `gz::sim::systems::ApplyJointForce` on `cart_joint` — applies the DQN's
  raw force commands.
- `gz::sim::systems::JointStatePublisher` — publishes joint position/velocity
  for every joint in the model.

These are bridged to ROS2 by `ros_gz_bridge parameter_bridge`, configured in
`robot_launch/launch/launch_simulation.launch.py`.

If this workspace ever needs more sophisticated control (position/velocity
control, controller switching, multiple joints coordinated together), a
`ros2_control`/`gz_ros2_control` migration would be the natural next step —
this port deliberately kept things minimal for a single force-driven joint.

## Bridge topic map

| Gazebo transport topic | ROS2 topic (after remap) | Type | Direction |
|---|---|---|---|
| `/model/cart_pole/joint/cart_joint/cmd_force` | `/cart_controller/command` | `std_msgs/msg/Float64` ↔ `gz.msgs.Double` | ROS → GZ |
| `/world/robomaster_rale/model/cart_pole/joint_state` | `/joint_states` | `sensor_msgs/msg/JointState` ↔ `gz.msgs.Model` | GZ → ROS |
| `/world/robomaster_rale/control` | (same) | `ros_gz_interfaces/srv/ControlWorld` | service, ROS → GZ |

**Non-obvious quirk:** these two gz-transport topics are scoped
differently on this gz-sim8 version — `cmd_force` is *not* world-scoped,
but `joint_state` *is* (confirmed empirically with `gz topic -l` while the
sim was running; the plain `/model/cart_pole/joint_state` form doesn't
exist). Don't assume symmetry here if the bridge config ever needs to change.

`commander`'s DQN node (`commander/commander/dqn_learning.py`) subscribes
`/joint_states`, publishes `/cart_controller/command`, and calls
`/world/robomaster_rale/control` to reset between episodes — using the same
topic/service names the original ROS1 script used, just re-pointed at the
bridge.

## Bugs found during live integration testing (beyond the original plan)

Static review (matching the plan's file contents, checking cross-task
topic/type consistency) caught zero issues across all four packages. Six
real problems only surfaced once the system was actually run:

1. **Stale Gazebo-classic world file.** `robomaster_rale.world` used
   `<include><uri>model://sun</uri></include>` / `model://ground_plane`,
   which don't resolve under gz-sim. Replaced with inline `light`/`model`
   SDF (copied from gz-sim's own shipped example worlds).
2. **Mesh resource path.** gz-sim rewrites the URDF's `package://` mesh URIs
   to `model://` and resolves them via `GZ_SIM_RESOURCE_PATH`, which by
   default doesn't include this workspace's install space. Fixed by
   prepending `robot_description`'s share-directory parent to that env var
   in the launch file (portably, via `get_package_share_directory`, not a
   hardcoded path).
3. **DQN output invisible under `ros2 launch`.** Python's stdout is fully
   buffered when non-interactive. Fixed with `emulate_tty=True` +
   `PYTHONUNBUFFERED=1` on the `commander` `Node` action.
4. **`/joint_states` topic scoping wrong** (the asymmetry noted above) —
   the bridge was originally configured with the plain, unscoped topic name;
   corrected to the world-scoped form.
5. **A full world reset (`reset.all`) permanently kills the joint-state
   feed.** Confirmed via `gz topic -i` before/after: after one
   `reset.all=True` call, `JointStatePublisher`'s gz-transport publisher
   is torn down and never re-advertised, freezing `/joint_states` forever.
   Switched to `reset.model_only=True` (resets the model's pose/velocity
   without tearing down the world) — this is a deliberate, explicitly
   approved deviation from resetting sim time/world state on every episode.
   Since the DQN loop is wall-clock (`time.sleep`) driven, not sim-time
   driven, this has no known effect on training behavior.
6. **Startup race.** All launch actions started concurrently; `commander`'s
   first act (`reset_simulation()`) could fail if Gazebo/the bridge weren't
   up within its 5s service-wait timeout on a slow machine. Fixed by
   sequencing `commander` to start only after the `ros_gz_sim create` spawn
   action exits (`RegisterEventHandler(OnProcessExit(...))`), instead of
   relying on `torch`'s import time as an accidental delay.

A seventh, smaller robustness fix was added during final review: the
`reset_simulation()` future-poll loop had no deadline and could hang
forever if the reset service accepted a call but never responded; it now
raises after 5 seconds instead.

## Known minor items (not fixed, low priority)

- `robot_description/robot/cart_pole.urdf.xacro` still includes
  `cart_trans_v0` (a `ros_control`-era `<transmission>` block). It's inert
  under gz-sim/`robot_state_publisher` but is stale — safe to remove along
  with `urdf/cart/cart.transmission.xacro` if you want to clean it up.
- Two `robot_state_publisher`-equivalent nodes run simultaneously (one to
  expose `robot_description` on a topic for spawning, one from
  `robot_control` for TF) — harmless duplication, not wrong.
- `use_sim_time` isn't set anywhere; fine today since nothing consumes TF
  for control, but worth revisiting if RViz or another sim-time consumer
  is added later.
