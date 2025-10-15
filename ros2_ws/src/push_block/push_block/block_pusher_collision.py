import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from rclpy.duration import Duration
from ament_index_python.packages import get_package_share_directory
import os
import math


def plan_and_execute(
        robot,
        planning_component,
        logger,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        ):
        """A helper function to plan and execute a motion."""
        # plan to goal
        logger.info("Planning trajectory")
        if multi_plan_parameters is not None:
                plan_result = planning_component.plan(
                        multi_plan_parameters=multi_plan_parameters
                )
        elif single_plan_parameters is not None:
                plan_result = planning_component.plan(
                        single_plan_parameters=single_plan_parameters
                )
        else:
                plan_result = planning_component.plan()

        # execute the plan
        if plan_result:
                logger.info("Executing plan")
                robot_trajectory = plan_result.trajectory
                robot.execute(robot_trajectory, controllers=[])
                return True
        else:
                logger.error("Planning failed")
                return False


class BlockPusher(Node):
    def __init__(self):
        super().__init__("block_pusher", allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        # Initialize MoveItPy
        self.moveit_py = MoveItPy(node_name="block_pusher")
        self.logger = self.get_logger()
        self.panda_arm = self.moveit_py.get_planning_component("fr3_arm")

        self.planning_scene_monitor = self.moveit_py.get_planning_scene_monitor()
        self._pose_link = "fr3_link8"
        self.logger.info("MoveItPy instance created, waiting to start push sequence...")
        # Defer start a bit to allow move_group/controllers to come up
        self._start_timer = self.create_timer(2.0, self._on_start_timer)

    def _on_start_timer(self):
        self._start_timer.cancel()
        self.execute_push()

    def _joint_state_callback(self, msg: JointState):
        self._latest_joint_state = msg

    def add_box_obstacle(self):
        """Adds a box obstacle to the planning scene."""
        position = (0.5, 0.2, 0.155)  # x, y, z
        size = (0.15, 0.15, 0.15)
        with self.planning_scene_monitor.read_write() as scene:
            self.collision_object = CollisionObject()
            self.collision_object.header.frame_id = "world"
            self.collision_object.id = "box"

            box_pose = Pose()
            box_pose.position.x = position[0]
            box_pose.position.y = position[1]
            box_pose.position.z = position[2]

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = size

            self.collision_object.primitives.append(box)
            self.collision_object.primitive_poses.append(box_pose)
            self.collision_object.operation = CollisionObject.ADD

            scene.apply_collision_object(self.collision_object)
            scene.current_state.update()  # Important to ensure the scene is updated

    def add_wall_obstacle(self):
        """Adds a wall obstacle to the planning scene."""
        position = (0.4, 0.0, 0.16)  # x, y, z
        size = (0.05, 1, 0.16)
        with self.planning_scene_monitor.read_write() as scene:
            collision_object = CollisionObject()
            collision_object.header.frame_id = "world"
            collision_object.id = "wall"

            wall_pose = Pose()
            wall_pose.position.x = position[0]
            wall_pose.position.y = position[1]
            wall_pose.position.z = position[2]

            wall = SolidPrimitive()
            wall.type = SolidPrimitive.BOX
            wall.dimensions = size

            collision_object.primitives.append(wall)
            collision_object.primitive_poses.append(wall_pose)
            collision_object.operation = CollisionObject.ADD

            scene.apply_collision_object(collision_object)
            scene.current_state.update()  # Important to ensure the scene is updated

    def remove_box_obstacle(self):
        """Removes the box obstacle from the planning scene."""
        with self.planning_scene_monitor.read_write() as scene:
            # self.collision_object.operation = CollisionObject.REMOVE
            # scene.apply_collision_object(self.collision_object)
            scene.remove_all_collision_objects()
            scene.current_state.update()  # Important to ensure the scene is updated

    def _plan_to_pose(self, target_pose: PoseStamped, single_plan_parameters=None):
        self.panda_arm.set_start_state_to_current_state()
        self.panda_arm.set_goal_state(pose_stamped_msg=target_pose, pose_link=self._pose_link)
        plan_and_execute(self.moveit_py, self.panda_arm, self.logger, single_plan_parameters=single_plan_parameters)

    def execute_push(self):
        """Executes the sequence of movements to push the block."""

        # End-Effector orientation to point down
        q_x = 1.0 * math.sin(math.pi/2)
        q_y = 0.0 * math.sin(math.pi/2)
        q_z = 0.0 * math.sin(math.pi/2)
        q_w = math.cos(math.pi/2)
        ee_pose = PoseStamped()
        ee_pose.header.frame_id = "world"
        ee_pose.pose.orientation
        ee_pose.pose.orientation.x = q_x
        ee_pose.pose.orientation.y = q_y
        ee_pose.pose.orientation.z = q_z
        ee_pose.pose.orientation.w = q_w


        # 1. Move to a position in front of the box, after the wall
        # TODO: Define target position, and execute the move
        # Hint: The wall is at x=0.4, so position.x should be > 0.4 to be in front of it
        # Hint: Add collision objects before planning, see class methods definied above.
        # ...

        # 2. Push the block
        # TODO: Define target position, and execute the move
        # Hint: Can you push the block if it is part of the planning scene?
        # ...


        # 3. Retract the arm upwards
        # TODO: Define target position, and execute the move
        # ...

        self.logger.info("Push sequence completed.")


def main(args=None):
    rclpy.init(args=args)
    block_pusher_node = BlockPusher()
    
    # Keep the node alive to allow for asynchronous operations to complete
    # Since the logic is sequential, we can spin once or just shutdown.
    # For more complex scenarios, you might spin indefinitely.
    rclpy.spin(block_pusher_node)
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
