# ROS1-to-ROS2 Port of cart_pole_DQN Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port the four ROS1/catkin packages in this workspace (`robot_description`, `robot_control`, `commander`, `robot_launch`) to build and run natively on the installed ROS2 Jazzy + Gazebo Harmonic (`gz-sim8`) stack, so `colcon build` succeeds and `ros2 launch robot_launch launch_simulation.launch.py` runs the DQN cart-pole simulation end to end.

**Architecture:** Drop `ros_control`/`gazebo_ros` entirely (neither is installed, and `gz_ros2_control`/`ros2_control` aren't installed either). Instead drive the cart joint with Gazebo Sim's built-in `ApplyJointForce` system plugin and read joint state from its built-in `JointStatePublisher` system plugin — both already shipped with the installed `gz-sim8-cli` package, confirmed via `/usr/share/gz/gz-sim8/worlds/apply_joint_force.sdf` and `joint_controller.sdf`. These are bridged to ROS2 topics with `ros_gz_bridge parameter_bridge`, which is already installed. World reset (`/gazebo/reset_simulation` in ROS1) becomes the `ros_gz_interfaces/srv/ControlWorld` service, bridged the same way (this exact bridge invocation is printed verbatim by `ros2 run ros_gz_bridge parameter_bridge --help`). `commander`'s `DQN_learning.py` moves from `rospy` to `rclpy`, subscribing to a bridged `sensor_msgs/JointState` on `/joint_states` (so `robot_state_publisher` can consume the same topic, matching the original architecture) instead of parsing `gazebo_msgs/LinkStates`.

**Tech Stack:** ROS2 Jazzy, Gazebo Sim (Harmonic, `gz-sim8`), `ros_gz_bridge`, `ros_gz_sim`, `ament_cmake` / `ament_python`, `rclpy`, PyTorch (existing DQN code, untouched).

## Global Constraints

- Do not install new system packages (no `ros2_control`, `gz_ros2_control`, or ROS1 packages are available or to be installed) — the whole design avoids needing them.
- Preserve the original DQN algorithm and RL loop behavior (`QNet`, `Brain`, `Agent`, reward shaping, episode structure, PID gains/limits in the URDF) exactly — only the ROS/Gazebo interfacing changes.
- World name is `robomaster_rale` (from `robot_launch/worlds/robomaster_rale.world:3`). Model name is `cart_pole` (from `robot_description/robot/cart_pole.urdf.xacro:2`). These exact strings appear in gz-transport topic/service names below — do not rename either without updating every bridge string.
- `robot_description/robot/cart_pole.sdf` and `cart_pole.urdf` are stale, previously-exported artifacts not referenced by any launch file — leave them alone, do not spend time fixing them.
- Every package.xml uses `format="3"`.

---

### Task 0: Initialize git so the port is revertible

This workspace currently has no `.git` — every later task's commit steps depend on one existing, and having the pre-port ROS1 state committed is the safety net if any later step needs to be reverted.

**Files:**
- Create: `/home/keqing-li/Documents/ros2_cart_pole_DQN/.gitignore`

- [ ] **Step 1: Check there really is no existing repo**

```bash
cd /home/keqing-li/Documents/ros2_cart_pole_DQN
git status
```
Expected: `fatal: not a git repository ...`. If this instead prints a normal `git status` output, STOP — a repo already exists, skip Task 0 entirely and re-check whether `build/`, `install/`, `log/` are already tracked/ignored before continuing to Task 1.

- [ ] **Step 2: Create `.gitignore`**

```
build/
install/
log/
__pycache__/
*.pyc
```

- [ ] **Step 3: Init and make the baseline commit**

```bash
git init
git add -A
git commit -m "baseline: ROS1 catkin packages before ROS2 port"
```
Expected: a commit containing `src/`, `docs/`, and `.gitignore`, with `build/`, `install/`, `log/` excluded.

---

### Task 1: Port `robot_description` to `ament_cmake`

**Files:**
- Modify: `src/robot_description/CMakeLists.txt` (full rewrite)
- Modify: `src/robot_description/package.xml` (full rewrite)
- Modify: `src/robot_description/robot/cart_pole.urdf.xacro:38-41` (replace the `gazebo_ros_control` plugin block)

**Interfaces:**
- Produces: an installed share directory `install/robot_description/share/robot_description/{robot,urdf,meshes}` that `$(find robot_description)` (via `ament_index_python`, which ROS2's `xacro` still supports — confirmed in `/opt/ros/jazzy/lib/python3.12/site-packages/xacro/substitution_args.py:139-141`) can resolve at xacro-processing time.

- [ ] **Step 1: Rewrite `package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>robot_description</name>
  <version>0.0.0</version>
  <description>URDF/xacro/mesh description of the cart_pole robot</description>
  <maintainer email="robomania@todo.todo">robomania</maintainer>
  <license>TODO</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>xacro</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: Rewrite `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY robot urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

- [ ] **Step 3: Replace the ROS1 Gazebo plugin block in `cart_pole.urdf.xacro`**

Replace lines 38-41 (the `<gazebo><plugin name="gazebo_ros_control" .../></gazebo>` block) with:

```xml
  <!-- =============== Gazebo (gz-sim) =============== -->
  <gazebo>
    <plugin filename="gz-sim-apply-joint-force-system" name="gz::sim::systems::ApplyJointForce">
      <joint_name>cart_joint</joint_name>
    </plugin>
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>
  </gazebo>
```

This is the exact plugin usage shown in `/usr/share/gz/gz-sim8/worlds/apply_joint_force.sdf:146-150` and `/usr/share/gz/gz-sim8/worlds/lift_drag.sdf:249-251`. `ApplyJointForce` opens a gz-transport topic `/model/cart_pole/joint/cart_joint/cmd_force` (`gz.msgs.Double`) that applies a raw force to `cart_joint` — this is the direct equivalent of the `force` variable the DQN script already computes, replacing the old `libgazebo_ros_control.so` + `effort_controllers/JointPositionController` indirection. `JointStatePublisher` publishes all joint positions/velocities on `/model/cart_pole/joint_state` (`gz.msgs.Model`).

- [ ] **Step 4: Verify xacro still processes cleanly**

Run:
```bash
cd /home/keqing-li/Documents/ros2_cart_pole_DQN
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_description
source install/setup.bash
xacro src/robot_description/robot/cart_pole.urdf.xacro > /tmp/cart_pole_test.urdf
python3 -c "import xml.dom.minidom as m; m.parse('/tmp/cart_pole_test.urdf'); print('OK: well-formed URDF')"
grep -c "ApplyJointForce\|JointStatePublisher" /tmp/cart_pole_test.urdf
```
Expected: `colcon build` reports `Summary: 1 package finished`; `xacro` exits 0; minidom prints `OK: well-formed URDF`; grep prints `2`.

- [ ] **Step 5: Commit**

```bash
git add src/robot_description
git commit -m "port robot_description to ament_cmake and gz-sim plugins"
```

---

### Task 2: Port `robot_control` to `ament_cmake`

Since actuation (`ApplyJointForce`) and joint-state publishing (`JointStatePublisher`) now come from gz-sim plugins baked into the URDF (Task 1), `robot_control`'s only remaining job is running `robot_state_publisher` so TF gets published from `/joint_states`. `ros_control`'s `controller.yaml` (PID gains for a position controller that no longer exists) has no equivalent — delete it.

**Files:**
- Modify: `src/robot_control/CMakeLists.txt` (full rewrite)
- Modify: `src/robot_control/package.xml` (full rewrite)
- Create: `src/robot_control/launch/robot_control_launch.py`
- Delete: `src/robot_control/launch/robot_control.launch`
- Delete: `src/robot_control/config/controller.yaml`

**Interfaces:**
- Produces: a `generate_launch_description()` in `robot_control_launch.py` containing exactly one `robot_state_publisher` `Node`, includable from `robot_launch`.

- [ ] **Step 1: Rewrite `package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>robot_control</name>
  <version>0.0.0</version>
  <description>Launches robot_state_publisher for the cart_pole robot</description>
  <maintainer email="robomania@todo.todo">robomania</maintainer>
  <license>TODO</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: Rewrite `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.8)
project(robot_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

- [ ] **Step 3: Delete the obsolete ros_control files**

```bash
rm src/robot_control/launch/robot_control.launch
rm src/robot_control/config/controller.yaml
rmdir src/robot_control/config
```

- [ ] **Step 4: Create `src/robot_control/launch/robot_control_launch.py`**

```python
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    robot_description = LaunchConfiguration('robot_description')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_description',
            description='Robot description XML (URDF) as a string',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
    ])
```

- [ ] **Step 5: Verify build**

Run:
```bash
cd /home/keqing-li/Documents/ros2_cart_pole_DQN
colcon build --packages-select robot_control
```
Expected: `Summary: 1 package finished`.

- [ ] **Step 6: Commit**

```bash
git add src/robot_control
git commit -m "port robot_control to ament_cmake, drop ros_control in favor of gz-sim plugins"
```

---

### Task 3: Port `commander` to `ament_python` and rewrite `DQN_learning.py` for `rclpy`

**Files:**
- Create: `src/commander/setup.py`
- Create: `src/commander/setup.cfg`
- Create: `src/commander/commander/__init__.py`
- Create: `src/commander/commander/dqn_learning.py` (ported from `scripts/DQN_learning.py`)
- Modify: `src/commander/package.xml` (full rewrite)
- Create: `src/commander/launch/commander_launch.py`
- Delete: `src/commander/CMakeLists.txt`
- Delete: `src/commander/scripts/DQN_learning.py`
- Delete: `src/commander/launch/commander.launch`

**Interfaces:**
- Produces: console-script entry point `dqn_learning` runnable via `ros2 run commander dqn_learning`.
- Consumes: `/joint_states` (`sensor_msgs/msg/JointState`, names `cart_joint`/`pole_joint`), publishes `/cart_controller/command` (`std_msgs/msg/Float64`), calls `/world/robomaster_rale/control` (`ros_gz_interfaces/srv/ControlWorld`) — all three names must exactly match the bridge remappings configured in Task 4's `launch_simulation.launch.py`.

- [ ] **Step 1: Remove the catkin build files**

```bash
rm src/commander/CMakeLists.txt
```

- [ ] **Step 2: Create `src/commander/package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>commander</name>
  <version>0.0.0</version>
  <description>DQN cart-pole balancing agent</description>
  <maintainer email="robomania@todo.todo">robomania</maintainer>
  <license>TODO</license>

  <buildtool_depend>ament_python</buildtool_depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>ros_gz_interfaces</depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 3: Create `src/commander/setup.py`**

```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'commander'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robomania',
    maintainer_email='robomania@todo.todo',
    description='DQN cart-pole balancing agent',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dqn_learning = commander.dqn_learning:main',
        ],
    },
)
```

- [ ] **Step 4: Create `src/commander/setup.cfg`**

```ini
[develop]
script_dir=$base/lib/commander
[install]
install_scripts=$base/lib/commander
```

- [ ] **Step 5: Create the `ament_python` resource marker and package `__init__.py`**

```bash
mkdir -p src/commander/resource
touch src/commander/resource/commander
touch src/commander/commander/__init__.py
```

- [ ] **Step 6: Delete the old script and launch file**

```bash
rm src/commander/scripts/DQN_learning.py
rmdir src/commander/scripts
rm src/commander/launch/commander.launch
```

- [ ] **Step 7: Create `src/commander/commander/dqn_learning.py`**

This preserves `QNet`, `Brain`, `Agent`, the reward/episode logic, and `plot_durations` verbatim from the original script. Only the ROS interfacing changes: a `DQNSimulationNode` class replaces the module-level `rospy` globals/callback, `/joint_states` (`sensor_msgs/JointState`) replaces `/gazebo/link_states` (`gazebo_msgs/LinkStates`) as the observation source (`cart_joint` position/velocity is exactly the old `cart_pose_x`/`cart_vel_x`; `pole_joint` velocity is exactly the old `y_angular` — both were already derived from the same physical DOFs, just read through link poses before), and `ros_gz_interfaces/srv/ControlWorld` replaces the `std_srvs/Empty` `/gazebo/reset_simulation` call. Because `rclpy` (unlike `rospy`) does not spin callbacks automatically, the node is spun on a background thread so the synchronous training loop below keeps working exactly like the original blocking rospy script.

```python
#!/usr/bin/env python3

import threading
import time

import numpy as np
import matplotlib.pyplot as plt
import torch
import torch.nn as nn
import torch.optim as optim

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from ros_gz_interfaces.srv import ControlWorld


class QNet(nn.Module):
    def __init__(self, num_states, dim_mid, num_actions):
        super().__init__()

        self.fc = nn.Sequential(
            nn.Linear(num_states, dim_mid),
            nn.ReLU(),
            nn.Linear(dim_mid, dim_mid),
            nn.ReLU(),
            nn.Linear(dim_mid, num_actions)
        )

    def forward(self, x):
        x = self.fc(x)
        return x


class Brain:
    def __init__(self, num_states, num_actions, gamma, r, lr):
        self.num_states = num_states
        self.num_actions = num_actions
        self.eps = 1.0
        self.gamma = gamma
        self.r = r

        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        print("self.device = ", self.device)
        self.q_net = QNet(num_states, 64, num_actions)
        self.q_net.to(self.device)
        self.criterion = nn.MSELoss()
        self.optimizer = optim.Adam(self.q_net.parameters(), lr=lr)

    def updateQnet(self, obs_numpy, action, reward, next_obs_numpy):
        obs_tensor = torch.from_numpy(obs_numpy).float()
        obs_tensor.unsqueeze_(0)
        obs_tensor = obs_tensor.to(self.device)

        next_obs_tensor = torch.from_numpy(next_obs_numpy).float()
        next_obs_tensor.unsqueeze_(0)
        next_obs_tensor = next_obs_tensor.to(self.device)

        self.optimizer.zero_grad()

        self.q_net.train()
        q = self.q_net(obs_tensor)

        with torch.no_grad():
            self.q_net.eval()
            label = self.q_net(obs_tensor)
            next_q = self.q_net(next_obs_tensor)

            label[:, action] = reward + self.gamma * np.max(next_q.cpu().detach().numpy(), axis=1)[0]

        loss = self.criterion(q, label)
        loss.backward()
        self.optimizer.step()

    def getAction(self, obs_numpy, is_training):
        if is_training and np.random.rand() < self.eps:
            action = np.random.randint(self.num_actions)
        else:
            obs_tensor = torch.from_numpy(obs_numpy).float()
            obs_tensor.unsqueeze_(0)
            obs_tensor = obs_tensor.to(self.device)
            with torch.no_grad():
                self.q_net.eval()
                q = self.q_net(obs_tensor)
                action = np.argmax(q.cpu().detach().numpy(), axis=1)[0]

        if is_training and self.eps > 0.1:
            self.eps *= self.r
        return action


class Agent:
    def __init__(self, num_states, num_actions, gamma, r, lr):
        self.brain = Brain(num_states, num_actions, gamma, r, lr)

    def updateQnet(self, obs, action, reward, next_obs):
        self.brain.updateQnet(obs, action, reward, next_obs)

    def getAction(self, obs, is_training):
        action = self.brain.getAction(obs, is_training)
        return action


class DQNSimulationNode(Node):
    def __init__(self):
        super().__init__('DQN_simulation')

        self.cart_pose_x = 0.0
        self.cart_vel_x = 0.0
        self.y_angular = 0.0

        self.pub_cart = self.create_publisher(Float64, '/cart_controller/command', 10)
        self.create_subscription(JointState, '/joint_states', self._joint_state_callback, 10)
        self.reset_client = self.create_client(ControlWorld, '/world/robomaster_rale/control')

        self.number_of_steps = []

    def _joint_state_callback(self, msg):
        if 'cart_joint' in msg.name:
            idx = msg.name.index('cart_joint')
            self.cart_pose_x = msg.position[idx]
            self.cart_vel_x = msg.velocity[idx]
        if 'pole_joint' in msg.name:
            idx = msg.name.index('pole_joint')
            self.y_angular = msg.velocity[idx]

    def reset_simulation(self):
        if not self.reset_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('/world/robomaster_rale/control service unavailable')
        request = ControlWorld.Request()
        request.world_control.reset.all = True
        future = self.reset_client.call_async(request)
        while not future.done():
            time.sleep(0.001)

    def simulate(self, episode, is_training, agent):
        reward = 0
        yaw_angle = 0
        time_interval = 0.02

        self.reset_simulation()

        obs = np.array([0, 0, 0, 0], dtype='float16')
        next_obs = np.array([0, 0, 0, 0], dtype='float16')
        max_step = 200
        is_done = False
        episode_reward = 0

        for step in range(max_step):
            time1 = time.time()
            yaw_angle += self.y_angular * time_interval

            next_obs[0] = self.cart_pose_x
            next_obs[1] = self.cart_vel_x
            next_obs[2] = yaw_angle
            next_obs[3] = self.y_angular

            if step == 0:
                next_obs[0] = 0
                next_obs[1] = 0
                next_obs[2] = 0
                next_obs[3] = 0

            action = agent.getAction(obs, is_training)

            if abs(yaw_angle) > 0.6 or step == max_step - 1:
                is_done = True

            if is_done:
                if step < max_step - 1:
                    reward = -200
                else:
                    reward = episode_reward
            else:
                reward = 6 - abs(yaw_angle) * 10

            episode_reward += reward

            force = action * 16 / 9 - 8

            if is_training:
                agent.updateQnet(obs, action, reward, next_obs)

            obs = np.copy(next_obs)

            self.pub_cart.publish(Float64(data=float(force)))

            time2 = time.time()
            interval = time2 - time1
            if interval < time_interval:
                time.sleep(time_interval - interval)

            if is_done and is_training:
                print('Episode: {0} Finished after {1} time steps with reward {2}'.format(
                    episode, step + 1, episode_reward))
                self.plot_durations(step)
                break
            elif is_done and not is_training:
                print('Evaluation: Finished after {} time steps'.format(step + 1))
                break

    def plot_durations(self, step):
        plt.figure(2)
        plt.clf()
        self.number_of_steps.append(step)
        x = np.arange(0, len(self.number_of_steps))
        plt.title('Training')
        plt.xlabel('Episode')
        plt.ylabel('Duration')
        plt.plot(x, self.number_of_steps)

        plt.pause(0.001)


def main():
    rclpy.init()
    node = DQNSimulationNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    gamma = 0.7
    r = 0.99
    lr = 0.001
    num_states = 4
    num_actions = 10

    agent = Agent(num_states, num_actions, gamma, r, lr)

    num_episodes = 1000
    is_training = True
    try:
        for i_episode in range(num_episodes):
            node.simulate(i_episode, is_training, agent)

        x = np.arange(0, len(node.number_of_steps))
        plt.plot(x, node.number_of_steps)
        plt.show()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 8: Create `src/commander/launch/commander_launch.py`**

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='commander',
            executable='dqn_learning',
            name='commander_node',
            output='screen',
        ),
    ])
```

- [ ] **Step 9: Verify build and import**

Run:
```bash
cd /home/keqing-li/Documents/ros2_cart_pole_DQN
colcon build --packages-select commander
source install/setup.bash
python3 -c "import commander.dqn_learning; print('OK: module imports cleanly')"
```
Expected: `Summary: 1 package finished`; prints `OK: module imports cleanly` (this only proves imports/syntax are correct — running training end-to-end happens in Task 5 once Gazebo is actually up).

- [ ] **Step 10: Commit**

```bash
git add src/commander
git commit -m "port commander to ament_python and rclpy"
```

---

### Task 4: Port `robot_launch` to `ament_cmake` and write the ROS2 launch file

**Files:**
- Modify: `src/robot_launch/CMakeLists.txt` (full rewrite)
- Modify: `src/robot_launch/package.xml` (full rewrite)
- Create: `src/robot_launch/launch/launch_simulation.launch.py`
- Delete: `src/robot_launch/launch/launch_simulation.launch`

**Interfaces:**
- Consumes: `robot_control_launch.py` from Task 2 (arg `robot_description`), `commander_launch.py` from Task 3 (no args).
- Produces: the running system — gz-sim server with world `robomaster_rale`, spawned `cart_pole` model, three `ros_gz_bridge` mappings, `robot_state_publisher`, `commander`.

- [ ] **Step 1: Rewrite `package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>robot_launch</name>
  <version>0.0.0</version>
  <description>Top-level launch file for the cart_pole DQN simulation</description>
  <maintainer email="robomania@todo.todo">robomania</maintainer>
  <license>TODO</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>ros_gz_interfaces</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>robot_description</exec_depend>
  <exec_depend>robot_control</exec_depend>
  <exec_depend>commander</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: Rewrite `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.8)
project(robot_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

- [ ] **Step 3: Delete the ROS1 launch file**

```bash
rm src/robot_launch/launch/launch_simulation.launch
```

- [ ] **Step 4: Create `src/robot_launch/launch/launch_simulation.launch.py`**

Bridge topic/service names are exactly the gz-transport names documented in `/usr/share/gz/gz-sim8/worlds/apply_joint_force.sdf:8` (`/model/<model>/joint/<joint>/cmd_force`) and confirmed by `ros2 run ros_gz_bridge parameter_bridge --help` for the `ControlWorld` service form. `remappings` rename the ROS-visible topics to the names `commander`/`robot_state_publisher` expect (`/cart_controller/command`, `/joint_states`) — matching the original ROS1 topic names.

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    robot_description_path = PathJoinSubstitution([
        get_package_share_directory('robot_description'),
        'robot', 'cart_pole.urdf.xacro',
    ])
    robot_description = Command(['xacro ', robot_description_path])

    world_path = os.path.join(
        get_package_share_directory('robot_launch'), 'worlds', 'robomaster_rale.world')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_control'), 'launch',
                         'robot_control_launch.py')
        ),
        launch_arguments={'robot_description': robot_description}.items(),
    )

    commander = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('commander'), 'launch',
                         'commander_launch.py')
        )
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'robomaster_rale',
            '-topic', 'robot_description',
            '-name', 'cart_pole',
            '-x', '0', '-y', '0', '-z', '1.225',
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/cart_pole/joint/cart_joint/cmd_force'
            '@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/cart_pole/joint_state'
            '@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/world/robomaster_rale/control@ros_gz_interfaces/srv/ControlWorld',
        ],
        remappings=[
            ('/model/cart_pole/joint/cart_joint/cmd_force', '/cart_controller/command'),
            ('/model/cart_pole/joint_state', '/joint_states'),
        ],
        output='screen',
    )

    robot_state_publisher_source = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_description_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher_source,
        bridge,
        spawn_entity,
        robot_control,
        commander,
    ])
```

Note: `robot_state_publisher_source` above publishes the `robot_description` parameter itself onto the `robot_description` *topic* (a `std_msgs/String`, ROS2's `robot_state_publisher` does this automatically) so `ros_gz_sim create -topic robot_description` can read the URDF from it — mirroring the original `<param name="robot_description" .../>` + `-param robot_description` ROS1 pattern via a topic instead of the (ROS1-only) parameter server. `robot_control`'s own `robot_state_publisher` node (started via the `robot_control` include) is the one that actually publishes TF as `/joint_states` messages arrive.

- [ ] **Step 5: Verify build**

Run:
```bash
cd /home/keqing-li/Documents/ros2_cart_pole_DQN
colcon build
```
Expected: `Summary: 4 packages finished` and no `stderr` output beyond CMake's usual deprecation notice.

- [ ] **Step 6: Commit**

```bash
git add src/robot_launch
git commit -m "port robot_launch to ament_cmake and gz-sim launch/bridge setup"
```

---

### Task 5: Full workspace build and integration smoke test

**Files:** none (verification only).

- [ ] **Step 1: Clean build all four packages**

```bash
cd /home/keqing-li/Documents/ros2_cart_pole_DQN
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build
```
Expected: `Summary: 4 packages finished`, zero packages failed/aborted.

- [ ] **Step 2: Launch the simulation**

```bash
source install/setup.bash
timeout 25 ros2 launch robot_launch launch_simulation.launch.py
```
Expected: Gazebo GUI window opens showing the `robomaster_rale` world with the `cart_pole` model spawned upright; no Python tracebacks in the terminal; the DQN node prints `self.device = ...` followed by `Episode: 0 Finished after N time steps with reward ...`.

- [ ] **Step 3: In a second terminal while it's running, verify the bridge topics and node graph**

```bash
source install/setup.bash
ros2 topic list
ros2 topic hz /joint_states --window 20
ros2 node list
```
Expected: `/joint_states` and `/cart_controller/command` both appear in `ros2 topic list`; `/joint_states` publishes at a nonzero rate; `ros2 node list` shows `/DQN_simulation`, `/robot_description_publisher`, `/robot_state_publisher`, `/parameter_bridge`.

- [ ] **Step 4: If any step fails, capture the exact error and fix the specific broken piece**

Do not proceed to declare the port complete on a failure. Common failure points to check first, in order: (a) exact `/model/cart_pole/...` topic scoping in `gz topic -l` (Gazebo may nest the topic under `/world/robomaster_rale/model/cart_pole/...` instead of the plain `/model/cart_pole/...` form — if so, update the two bridge topic strings in `launch_simulation.launch.py` Step 4 above to match, using `gz topic -l` output as ground truth); (b) whether `cart_joint`/`pole_joint` actually appear in `ros2 topic pub` sample of `/joint_states` (`ros2 topic echo /joint_states --once`); (c) whether the `ControlWorld` service actually resets the model pose (episode 2 should start from the same spawn height as episode 1, not fall through the floor).

- [ ] **Step 5: Commit any fixes from Step 4, then do a final commit noting the port is verified end-to-end**

```bash
git add -A
git commit -m "verify ros1-to-ros2 port builds and runs end-to-end"
```
