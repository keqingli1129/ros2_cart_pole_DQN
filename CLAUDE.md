# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A ROS2 Jazzy + Gazebo Harmonic (gz-sim8) cart-pole simulation where a DQN
(deep Q-network) agent, written with PyTorch, learns to balance a pole on a
cart by applying a horizontal force. Originally a ROS1/Gazebo-classic
project, ported to ROS2 on 2026-07-20 (see `docs/PORT_NOTES.md` for the full
rationale and every bug found during the port — read it before touching the
launch/bridge wiring, it documents several non-obvious gz-sim quirks).

## Build and run

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch robot_launch launch_simulation.launch.py
```

There is no unit test suite (robotics sim project). Verification = `colcon
build` succeeds, then a live run shows `/joint_states` publishing and the
`DQN_simulation` node completing training episodes with no tracebacks. If
the DQN's per-episode prints (`Episode: N Finished after ...`) aren't
appearing in the terminal, that's the buffering issue described below, not a
training failure.

To iterate on just the DQN node without rebuilding the whole workspace,
edit `src/commander/commander/dqn_learning.py` and re-run
`colcon build --packages-select commander` (it's an `ament_python` package,
so a plain re-source often picks up changes too since files are symlinked
into `install/` by `--symlink-install`, if the workspace was built that way).

## Launch sequence

`ros2 launch robot_launch launch_simulation.launch.py`
(`src/robot_launch/launch/launch_simulation.launch.py`) is the single entry
point. It brings up five things, with one explicit ordering dependency:

1. **`gz_sim`** — includes `ros_gz_sim`'s `gz_sim.launch.py` to start Gazebo
   with `src/robot_launch/worlds/robomaster_rale.world`. Before doing this,
   the launch file prepends `robot_description`'s share-directory *parent*
   to `GZ_SIM_RESOURCE_PATH` (via `get_package_share_directory`, not a
   hardcoded path) — required so gz-sim can resolve the URDF's
   `package://robot_description/meshes/...` URIs, which it rewrites to
   `model://robot_description/meshes/...`.
2. **`robot_state_publisher_source`** — a `robot_state_publisher` node
   (named `robot_description_publisher`) that publishes the xacro-expanded
   URDF on the `robot_description` topic, purely so `spawn_entity` can spawn
   from a topic rather than a file.
3. **`bridge`** — `ros_gz_bridge parameter_bridge`, wiring the three
   gz-transport topics/services listed below to ROS2 topics/services.
4. **`spawn_entity`** — `ros_gz_sim create`, spawns `cart_pole` into the
   `robomaster_rale` world from the `robot_description` topic.
5. **`robot_control`** — includes `robot_control_launch.py`, which runs a
   second `robot_state_publisher` (this one for TF, named plain
   `robot_state_publisher`). Yes, two `robot_state_publisher`-equivalent
   nodes run at once; this is harmless duplication, not a bug.
6. **`commander`** (the DQN node) — deliberately **not** started
   concurrently with the rest. It's wired via
   `RegisterEventHandler(OnProcessExit(target_action=spawn_entity, ...))` to
   start only after `spawn_entity` exits. Without this, `commander`'s first
   action (`reset_simulation()`, a 5s-timeout service call) can race Gazebo/
   the bridge coming up on a slow machine. Do not reorder this to start
   `commander` alongside the other actions.

The `commander` node's stdout is forced unbuffered
(`emulate_tty=True` + `additional_env={'PYTHONUNBUFFERED': '1'}` in
`src/commander/launch/commander_launch.py`) — without this, Python fully
buffers stdout under `ros2 launch` and per-episode prints never appear.

## Bridge topic map (gz-transport ↔ ROS2)

| Gazebo transport topic | ROS2 topic (after remap) | Type | Direction |
|---|---|---|---|
| `/model/cart_pole/joint/cart_joint/cmd_force` | `/cart_controller/command` | `std_msgs/msg/Float64` ↔ `gz.msgs.Double` | ROS → GZ |
| `/world/robomaster_rale/model/cart_pole/joint_state` | `/joint_states` | `sensor_msgs/msg/JointState` ↔ `gz.msgs.Model` | GZ → ROS |
| `/world/robomaster_rale/control` | (same) | `ros_gz_interfaces/srv/ControlWorld` | service, ROS → GZ |

**Non-obvious quirk:** these two topics are scoped differently on this
gz-sim8 version — `cmd_force` is *not* world-scoped, but `joint_state` *is*
(the plain `/model/cart_pole/joint_state` form doesn't exist). Don't assume
symmetry if this bridge config ever needs to change; re-verify with
`gz topic -l` while the sim is running.

## No `ros2_control`

`ros2_control`/`gz_ros2_control` aren't installed and weren't added during
the port. Actuation and joint feedback instead come from gz-sim's built-in
system plugins, declared directly in
`src/robot_description/robot/cart_pole.urdf.xacro`:
`gz::sim::systems::ApplyJointForce` (applies the DQN's force commands to
`cart_joint`) and `gz::sim::systems::JointStatePublisher` (publishes
position/velocity for every joint). `robot_control` package exists only to
run `robot_state_publisher` — there's no controller manager. If this
project ever needs position/velocity control, controller switching, or
multiple coordinated joints, migrating to `ros2_control` is the natural
next step; today's setup is intentionally minimal for one force-driven
joint.

## DQN training (`src/commander/commander/dqn_learning.py`)

Single file, no config files — hyperparameters are literals in `main()`.
Entry point: `ros2 run commander dqn_learning` (installed via
`console_scripts` in `src/commander/setup.py`; also what
`commander_launch.py` runs).

**Network (`QNet`)**: MLP, 4 → 64 → 64 → 10, ReLU activations, no output
activation (raw Q-values). Runs on CUDA if `torch.cuda.is_available()`,
else CPU.

**State (4-dim, all `float16`)**: `[cart_pose_x, cart_vel_x, yaw_angle,
y_angular]`. `cart_pose_x`/`cart_vel_x` come straight from the bridged
`/joint_states` for `cart_joint`. `yaw_angle` is **not** measured directly —
it's dead-reckoned each step as `yaw_angle += y_angular * time_interval`
(`time_interval = 0.02`) inside the Python loop, using `y_angular` (the
`pole_joint` angular velocity from `/joint_states`). It resets to all-zero
at `step == 0` of each episode.

**Action space**: 10 discrete actions (`num_actions = 10`), mapped to a
continuous force via `force = action * 16/9 - 8`, spanning roughly [-8, 8]
Newtons, published to `/cart_controller/command`.

**Reward shaping**: `+6 - abs(yaw_angle) * 10` per step while upright;
`-200` on early termination (`abs(yaw_angle) > 0.6` rad, i.e. pole fell);
if the episode survives the full `max_step = 200` steps, the *last* step's
reward is replaced with the accumulated `episode_reward` instead of the
per-step shaped value.

**Q-learning update (`Brain.updateQnet`)**: this is a **single shared
network** used as both the online and target net (no separate target
network, no experience replay buffer) — every step does one online SGD
update immediately after acting, using the standard Q-learning target
`reward + gamma * max(next_q)` with `gamma = 0.7`. Because there's no replay
buffer, samples are used exactly once and are highly correlated
(consecutive timesteps of the same trajectory) — if training stability ever
becomes a problem, adding replay + a frozen target net is the standard fix,
not a hyperparameter tweak.

**Exploration**: epsilon-greedy, `eps` starts at `1.0`, decays
multiplicatively by `r = 0.99` after every action taken *during training*
(not per-episode), floored at `0.1` (decay stops once `eps <= 0.1`, but
`eps` can end up dropping just below that floor since the check happens
before the multiply).

**Episode loop (`DQNSimulationNode.simulate`)**: calls
`reset_simulation()` (see below) at the start of every episode, then steps
in real time — each of the up to 200 steps sleeps out the remainder of
`time_interval = 0.02` s using `time.time()`/`time.sleep()`, i.e. training
speed is wall-clock-bound by real-time Gazebo simulation, not
sim-time-accelerated. `main()` runs `num_episodes = 1000` training episodes
in a loop, then plots total episode durations with matplotlib
(`plot_durations`, updated live every episode via `plt.pause`, plus a final
blocking `plt.show()` after all 1000 episodes).

**Episode reset**: `reset_simulation()` calls the bridged
`/world/robomaster_rale/control` service with
`request.world_control.reset.model_only = True` — deliberately *not*
`reset.all`. A full `reset.all` permanently kills gz-sim's
`JointStatePublisher` gz-transport advertisement after the first call,
freezing `/joint_states` for the rest of the run (confirmed empirically,
see `docs/PORT_NOTES.md`). `model_only` resets pose/velocity without that
teardown. The call has a 5s deadline and raises `RuntimeError` if the
service doesn't respond in time — don't remove that deadline, it was added
specifically after an early version hung indefinitely on a slow reset.

## Known stale/low-priority items (not bugs, don't "fix" without asking)

- `cart_pole.urdf.xacro` still includes `cart_trans_v0`, a `ros_control`-era
  `<transmission>` block — inert under gz-sim, safe to remove along with
  `urdf/cart/cart.transmission.xacro` but not currently causing problems.
- `use_sim_time` isn't set anywhere. Fine today since nothing consumes TF
  for control; revisit if RViz or another sim-time consumer is added.
