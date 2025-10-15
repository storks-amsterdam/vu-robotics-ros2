# Practical 2 – MoveIt + Gazebo: Planning, Block Pushing, and Collision Avoidance

## Objectives
- Explore MoveIt planning via RViz with a simulated Franka arm in Gazebo.
- Implement a block pushing sequence using MoveIt (no RViz).
- Use the planning scene (collision objects) to solve a pushing task with only 2 moves.

Final deliverables:
- One or two screenshots showing the block fully inside the target zone in Gazebo (from Task 1 and Task 2).

## Prerequisites (DO NOT SKIP!)
- Environment from README.md (Docker locally or on VM). Gazebo Harmonic and ROS 2 Jazzy.
- IMPORTANT: Pull latest Docker image. The new image includes a `push_block` package with launch files and scripts for this practical.
```bash
docker pull storkslab/ros2-jazzy-franka:latest
```
- Start a container with Jupyter Server and Desktop environemnt as described in README.md.
- The push_block package is present at:
  `~/ros2_ws/src/push_block`


Build once before starting:
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select push_block
source install/setup.bash
```

---

## Part A – RViz + MoveIt Exploration (warm-up)

Goal: Inspect the MoveIt Motion Planning pipeline and plan a few motions in RViz while Gazebo runs.

MoveIt 2 is the standard manipulation stack for ROS 2. The move_group node runs a motion planning pipeline that combines: kinematics (IK/FK), collision checking against the “planning scene” (robot + attached objects + world geometry), and a planner backend (OMPL by default). In this repo the default planner is OMPL’s RRTConnect (planner_id: RRTConnectkConfigDefault) with multiple planning attempts and a 5 s planning time; trajectories are post-processed and time-parameterized. RViz’s MotionPlanning panel sends pose or joint goals to move_group and visualizes the resulting trajectory; Plan runs the solver only, Execute streams the trajectory to controllers in Gazebo.

Docs: https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html

1) In the desktop environment, launch MoveIt + Gazebo + RViz:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch push_block moveit.launch.py
```

2) In RViz:
- Ensure the MotionPlanning panel is visible (Panels > Add New Panel > MotionPlanning if needed).
- You might need to click/enable the "Planning Scene" item from the list of plugins.
- Select planning group: fr3_arm.
- Use the interactive marker to move the end-effector goal.
- Plan and Execute a few motions. Keep the end-effector pointing down for realism.

3) Observe:
- Gazebo renders the robot; RViz shows planned trajectories.
- Topics/nodes: move_group, robot_state_publisher, controllers.
- You can use this environment to manulally move the robot arm around and push the block if you want.
- This is just for exploration and testing controllers. Normally, you would not use RViz for teleoperation.

No submission required for Part A.

---

## Part B – Task 1: Block Pushing with MoveIt (no RViz)

You will use a prepared script and complete the fill-ins inside it.

Launch (no RViz) to examine the scene:
```bash
ros2 launch push_block pusher.launch.py
```

In the script, only one move is implemented. You will fill implement the rest of the pushing sequence.
Since the moveit planning pipeline does not know about anyother obejcts in the scene, you will need to break the push into multiple small moves.

Script to edit:
- File: `~/ros2_ws/src/push_block/push_block/block_pusher.py`
- The script already:
  - Sets a “point-down” end-effector orientation.
  - Moves to a pre-push pose above the block.
  - Exposes a helper `_plan_to_pose` for pose goals with link `fr3_link8` in frame `world`.

Based on the first step as an example, implement all the TODOs (it's not necessary to change any other code):
- Step 2: Move down to a pushing height (lower z, same x/y as pre-push).
- Step 3: Perform a straight push along the table plane towards the target zone (adjust y or x while keeping z constant and orientation unchanged).
- Step 4: Retract up (increase z back to a safe height).

Move sequence parts to fill in (based on the step 1 example):
- Create new pose (PoseStamped())
- Set position.x, position.y, position.z
- Set orientation (copy from ee_pose.pose.orientation)
- Call `_plan_to_pose()` and check for success


Hints:
- Maintain `ee_pose.pose.orientation` for all poses.
- Cartesian-like behavior is achieved by planning to successive pose waypoints with small changes in x/y and fixed z.
- Example magnitudes that tend to work: z push height about 0.11–0.14 m; retract around 0.22–0.28 m. Adjust in small increments if planning fails.
- Keep frame_id as "world".
- If arm hits the table or block, slightly relax the approach position (±5 in x/y,  +2-5 cm in z) and keep orientation constant.

Don't forget to rebuild the pacakge after editing the script, and source the workspace.

Rebuild and run:
```bash
cd ~/ros2_ws
colcon build --packages-select push_block
source install/setup.bash
ros2 launch push_block pusher.launch.py
```

Deliverable:
- Screenshot of Gazebo showing the block partially touching the target zone.

---

## Part C – Task 2: Collision-Aware Push (2 moves only)

This task adds a wall and leverages the MoveIt planning scene and collision avoidance so the problem can be solved with just two moves.

Launch (no RViz) to examine the scene:
```bash
ros2 launch push_block pusher_collision.launch.py
```

Script to edit:
- File: `~/ros2_ws/src/push_block/push_block/block_pusher_collision.py`
- Provided helpers:
  - `add_box_obstacle()` adds the block to the planning scene.
  - `add_wall_obstacle()` adds the wall.
  - `remove_box_obstacle()` can clear objects if needed.
  - `_plan_to_pose()` plans to a PoseStamped target using link `fr3_link8` in frame `world`.

Goal:
- Use only two moves to move the block in the target zone.
- Use the collision objects to make planning safe and straightforward.

What to fill in (do not change any other code):
1) Before planning, add collision objects:
   - Call `self.add_wall_obstacle()` and `self.add_box_obstacle()` to populate the planning scene.
2) Move 1: Move to a pre-push pose in front of the box but beyond the wall (x > 0.4 is a good starting point). Keep the “point-down” orientation. Lower to push height.
3) Move 2: Execute a single straight push to the final target position. Keep the same orientation and push-height z. Question: Can you push the block if it is part of the planning scene?

Hints:
- Maintain `ee_pose.pose.orientation` for both moves.
- Use coordinates that keep the end-effector clear of the wall while approaching the block from the free side. Try small adjustments in x/y if planning fails.
- Keep frame_id as "world".
- Build planning incrementally: first try just moving to the pre-push pose, then add the push.
- Compare behavior with and without the block as a collision object.
- If planning reports collision with the box, ensure the pre-push pose is offset just enough to touch and push without intersecting.

Rebuild and run:
```bash
cd ~/ros2_ws
colcon build --packages-select push_block
source install/setup.bash
ros2 launch push_block pusher_collision.launch.py
```

Deliverable:
- Screenshot of Gazebo showing the block partially touching the target zone.

---

## Submission
- Upload screenshot(s) showing the block clearly touching the target zone in Gazebo for:
  - Task 1 (block pushing) without the wall, and
  - Task 2 (collision-aware pushing with 2 moves) with the wall.
- Name your files clearly (e.g., p2_task1.png, p2_task2.png).
- Name your files clearly (e.g., p2_task1.png, p2_task2.png).
