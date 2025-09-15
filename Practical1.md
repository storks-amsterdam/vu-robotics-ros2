# Practical 1 – ROS 2 Franka Arm: Introspection, Command, and TF

## Objectives
- Launch a simulated Franka arm with controllers.
- Inspect nodes, topics, message types, and relationships.
- Understand key components of a ROS 2 control + Gazebo bridge stack.
- Publish joint commands manually and via a custom keyboard teleoperation node.
- Query and print end-effector pose using TF transforms.

## Prerequisites
- Ubuntu + ROS 2 (Jazzy or compatible) OR the provided Docker image runtime.
- Workspace sourced (e.g., source /opt/ros/jazzy/setup.bash) if native install.
- franka_gazebo_bringup (or provided meta‑package) from franka-ros2 library installed.
- Basic Python familiarity.
- Docker users (preferred for course):
  - Ensure Docker is installed (see README.md).
  - Pull the latest teaching image before starting:
    ```bash
    docker pull storkslab/ros2-jazzy-franka:latest
    ```
  - Start a fresh detached container (do not restart old ones, since the new image contains new scripts):
    ```bash
    docker run -d -p 8888:8888 storkslab/ros2-jazzy-franka:latest jupyter server
    ```
  - Access via browser: https://\<IP Address\>:8888/lab (password: robotics2025), then open Desktop.


# Task 1: Understand Core Components of ROS 2 Control Stack
1. Launch simulation:
   ```bash
   ros2 launch franka_gazebo_bringup joint_group_position_controller.launch.py
   ```
2. Open a new terminal.
3. List topics:
   ```bash
   ros2 topic list
   ```
4. List nodes:
   ```bash
   ros2 node list
   ```
5. Inspect several topics (example):
   ```bash
   ros2 topic info /joint_states
   ros2 topic info /tf
   ros2 topic info /joint_group_position_controller/commands
   ```
6. Echo joint states briefly:
   ```bash
   ros2 topic echo /joint_states --qos-profile sensor_data
   ```
7. Use rqt to visualize nodes and topics:
   ```bash
   rqt
   ```
   - Switch to Plugins > Introspection > Node Graph.
   - Select "Nodes/Topics (all)" in the dropdown to see all nodes and topics.
   - Switch to Namespaces: hide leaf topics off if cluttered.
   - Take a screenshot (Deliverable).

## Launch files and controllers

Inspect the launch file for context:
```bash
cd ~/franka_ros2_ws/src/franka_gazebo/franka_gazebo_bringup/launch
```
Open: joint_group_position_controller.launch.py

For more details on controllers, see:
https://control.ros.org/jazzy/doc/ros2_controllers/doc/controllers_index.html#controllers-for-manipulators-and-other-robots

Franka gazebo package controllers are defined in:
```bash
cd ~/franka_ros2_ws/src/franka_gazebo/franka_gazebo_bringup/config/
nano franka_gazebo_controllers.yaml
```


## Questions (Task 1):

For each item below write 2–4 sentences. You should explain:
1. Role/purpose in the system.
2. What it publishes and/or subscribes to (topics + message types).
3. How it fits in the overall data/control flow.

Topics (3):
- /joint_states
- /tf
- /joint_group_position_controller/commands

Nodes (3):
- /robot_state_publisher
- /controller_manager
- /joint_state_broadcaster


# Coding Tasks

## Task 2 (Implementation Part): Create Joint Control Package
You will create franka_joint_control with two Python nodes.

A. Create package:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python --license Apache-2.0 franka_joint_control
cd franka_joint_control
mkdir -p franka_joint_control
```

B. Node 1: keyboard_controller.py (teleoperation)
Goal: Press keys to increment/decrement joint angles and publish updated command vector.

Suggested behavior:
- Maintain an internal list of 7 joint targets (start at current joint states [0.0, -0.79, 0.0, -2.36, 0.0, 1.57, 0.79] OR zeros).
- Key mappings (increment / decrement):
```text
  Joint 1: 1 / q
  Joint 2: 2 / w
  Joint 3: 3 / e
  Joint 4: 4 / r
  Joint 5: 5 / t
  Joint 6: 6 / y
  Joint 7: 7 / u
  space: zero all
  p: print current command
  Ctrl+C: exit cleanly
```
- Publish to /joint_group_position_controller/commands (std_msgs/Float64MultiArray).
- Use non-blocking keyboard capture (e.g., termios + sys.stdin, no external dependencies).

Structure outline (pseudo-code example):
```text
rclpy.init()
node = rclpy.create_node('keyboard_controller')
publisher = node.create_publisher(msgType, topic, qos)
loop:
  read key
  update list
  msg = msgType(data=targets)
  publisher.publish(msg)
```

C. Update setup.py entry_points:
'keyboard_controller = franka_joint_control.keyboard_controller:main'

D. Dependencies in package.xml:
<depend>rclpy</depend>
<depend>std_msgs</depend>

E. Build & test:
```bash
cd ~/ros2_ws
colcon build --packages-select franka_joint_control
source install/setup.bash
ros2 run franka_joint_control keyboard_controller
```
Move joints while simulation runs.

Deliverables:
- keyboard_controller.py file.

## Task 3: End-Effector Pose Node (get_pose.py)
Goal: Periodically print end-effector pose using TF.

A. Add get_pose.py in franka_joint_control/ directory (same package).

B. Use:
- rclpy
- tf2_ros.Buffer + TransformListener
- timer (e.g., 2 Hz)

C. Frames:
- Base frame commonly: world or fr3_link0 (confirm via: ros2 run tf2_tools view_frames or ros2 topic echo /tf | grep)
- End-effector frame: fr3_link7, fr3_hand, or similar; inspect URDF or TF tree.

D. Lookup:
transform = buffer.lookup_transform(base_frame, ee_frame, rclpy.time.Time())
Extract translation (x,y,z) + quaternion (x,y,z,w).

Optional: Convert quaternion to RPY (use tf_transformations if available or custom math).

E. Print formatted line:

End-Effector Pose: pos=[x y z] quat=[qx qy qz qw]
Handle exceptions (tf2.LookupException, tf2.ExtrapolationException).

Add setup.py entry point:
'get_pose = franka_joint_control.get_pose:main'

Rebuild & run:
```bash
cd ~/ros2_ws
colcon build --packages-select franka_joint_control
source install/setup.bash
ros2 run franka_joint_control get_pose
```

Deliverables:
- get_pose.py file.

## Suggested File Layout
franka_joint_control/
  package.xml
  setup.py
  franka_joint_control/
    __init__.py
    keyboard_controller.py
    get_pose.py


## Submission Package
Provide:
1. nodes.txt (output of ros2 node list)
2. topics.txt (output of ros2 topic list)
3. rqt_graph.png
4. answers.txt (Task 1 questions + component descriptions)
5. keyboard_controller.py
6. get_pose.py

For full marks: you should see a changing end-effector pose printed in the terminal as you move the robot arm using the keyboard. Check with a TA.

Compress into: practical1_\<studentid\>.zip

## 11. Evaluation
- Task 1: Component descriptions & answers: 2pts
- Task 2: keyboard_controller script: 1.5pts
- Task 3: get_pose script: 1.5pts
