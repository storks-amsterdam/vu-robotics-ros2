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

        self._pose_link = "fr3_link8"
        self.logger.info("MoveItPy instance created, waiting to start push sequence...")
        # Defer start a bit to allow move_group/controllers to come up
        self._start_timer = self.create_timer(2.0, self._on_start_timer)

    def _on_start_timer(self):
        self._start_timer.cancel()
        self.execute_push()

    def _joint_state_callback(self, msg: JointState):
        self._latest_joint_state = msg

    def _prepare_start_state(self):
        if not self._manipulator_group_name or not self._group_joint_names:
            self.panda_arm.set_start_state_to_current_state()
            return
        if self._latest_joint_state is None:
            self.panda_arm.set_start_state_to_current_state()
            return
        joint_map = dict(zip(self._latest_joint_state.name, self._latest_joint_state.position))
        if not all(name in joint_map for name in self._group_joint_names):
            self.panda_arm.set_start_state_to_current_state()
            return
        bounded_state = RobotState(self._robot_model)
        bounded_state.set_to_default_values()
        positions = [joint_map[name] for name in self._group_joint_names]
        bounded_state.set_joint_group_positions(self._manipulator_group_name, positions)
        bounded_state.enforce_bounds()
        self.panda_arm.set_start_state(robot_state=bounded_state)

    def _plan_to_pose(self, target_pose: PoseStamped, single_plan_parameters=None):
        # self._prepare_start_state()
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


        # 1. Move to a pre-push position above the block
        pre_push_pose = PoseStamped()
        pre_push_pose.header.frame_id = "world"
        pre_push_pose.pose.position.x = 0.35
        pre_push_pose.pose.position.y = 0.38
        pre_push_pose.pose.position.z = 0.25
        pre_push_pose.pose.orientation = ee_pose.pose.orientation
        self.logger.info("1. Moving to pre-push position.")
        self._plan_to_pose(pre_push_pose)

        # 2. Move down to pushing height
        # TODO: Define target position, and execute the move

        # 3. Push the block
        # TODO: Define target position, and execute the move
        # ...

        # 4. Retract up
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
