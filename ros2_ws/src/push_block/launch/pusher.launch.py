#  Copyright (c) 2024 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

# This file is an adapted version of
# https://github.com/ros-planning/moveit_resources/blob/ca3f7930c630581b5504f3b22c40b4f82ee6369d/panda_moveit_config/launch/demo.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* Windows Error where available
        return None


def generate_launch_description():
    # Configure ROS nodes for launch
    load_gripper_name = 'load_gripper'
    franka_hand_name = 'franka_hand'
    arm_id_name = 'arm_id'
    namespace_name = 'namespace'

    load_gripper = LaunchConfiguration(load_gripper_name)
    franka_hand = LaunchConfiguration(franka_hand_name)
    arm_id = LaunchConfiguration(arm_id_name)
    namespace = LaunchConfiguration(namespace_name)

    load_gripper_launch_argument = DeclareLaunchArgument(
            load_gripper_name,
            default_value='false',
            description='true/false for activating the gripper')
    franka_hand_launch_argument = DeclareLaunchArgument(
            franka_hand_name,
            default_value='franka_hand',
            description='Default value: franka_hand')
    arm_id_launch_argument = DeclareLaunchArgument(
            arm_id_name,
            default_value='fr3',
            description='Available values: fr3, fp3 and fer')
    namespace_launch_argument = DeclareLaunchArgument(
        namespace_name,
        default_value='',
        description='Namespace for the robot. If not set, the robot will be launched in the root namespace.')

    db_arg = DeclareLaunchArgument(
        'db', default_value='False', description='Database flag'
    )

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.urdf.xacro'
    )

    robot_description_command = Command(
        [
            FindExecutable(name='xacro'),
            ' ',
            franka_xacro_file,
            ' ros2_control:=true',
            ' hand:=', load_gripper,
            ' arm_id:=fr3',
            ' robot_ip:=dont-care',
            ' use_fake_hardware:=false',
            ' fake_sensor_commands:=false',
            ' gazebo:=true',
        ]
    )

    robot_description = {'robot_description': ParameterValue(
        robot_description_command, value_type=str)}

    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.srdf.xacro'
    )

    robot_description_semantic_command = Command(
        [FindExecutable(name='xacro'), ' ',
         franka_semantic_xacro_file, ' hand:=', load_gripper]
    )

    # Use ParameterValue here as well if needed
    robot_description_semantic = {'robot_description_semantic': ParameterValue(
        robot_description_semantic_command, value_type=str)}

    kinematics_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/kinematics.yaml')

    kinematics_config = {
        'robot_description_kinematics': kinematics_yaml
    }

    joint_limits_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/fr3_joint_limits.yaml'
    )

    joint_limits_config = {
        'robot_description_planning': joint_limits_yaml
    }

    # Planning Functionality
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
                                ],
            'response_adapters': [
                'default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/ValidateSolution',
                'default_planning_response_adapters/DisplayMotionPath'
                                  ],
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/fr3_controllers.yaml'
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Config for MoveItPy
    planning_pipelines_config = {
        'planning_pipelines': {
            'pipeline_names': ['ompl'],
        },
        'default_planning_pipeline': 'ompl',
        "plan_request_params": {
            "planning_pipeline": "ompl",
            "planner_id": "RRTConnectkConfigDefault",
            "planning_attempts": 10,
            "planning_time": 5.0,
            "max_velocity_scaling_factor": 0.5,
            "max_acceleration_scaling_factor": 0.5,
        },
        'ompl': {
                'planning_plugins': ['ompl_interface/OMPLPlanner'],
                'request_adapters': [
                    'default_planning_request_adapters/ResolveConstraintFrames',
                    'default_planning_request_adapters/ValidateWorkspaceBounds',
                    'default_planning_request_adapters/CheckStartStateBounds',
                    'default_planning_request_adapters/CheckStartStateCollision',
                                    ],
                'response_adapters': [
                    'default_planning_response_adapters/AddTimeOptimalParameterization',
                    'default_planning_response_adapters/ValidateSolution',
                    'default_planning_response_adapters/DisplayMotionPath'
                                    ],
                'start_state_max_bounds_error': 0.1,
                **ompl_planning_yaml
            },
    }

    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=namespace,
        parameters=[
            {'use_sim_time': True},  # <<< ensure sim time
            robot_description,
            robot_description_semantic,
            kinematics_config,
            joint_limits_config,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )
    
    block_pusher_node = Node(
        package='push_block',
        executable='block_pusher',
        name='block_pusher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            robot_description,
            robot_description_semantic,
            kinematics_config,
            joint_limits_config,
            ompl_planning_pipeline_config,
            trajectory_execution,
            planning_scene_monitor_parameters,
            planning_pipelines_config,
            moveit_controllers,
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},  # <<< ensure sim time
            robot_description,
        ],
    )

    # Gazebo Sim
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(get_package_share_directory('franka_description'))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Use the custom world
    world_file = os.path.join(get_package_share_directory('push_block'), 'models', 'push_block', 'world.sdf')
    
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'{world_file} -r'}.items(),
    )

    # Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    # # Spawn block
    # block_sdf_path = os.path.join(get_package_share_directory('push_block'), 'models', 'push_block', 'model.sdf')
    # spawn_block = Node(package='ros_gz_sim', executable='create',
    #                  arguments=['-file', block_sdf_path, '-name', 'push_block', '-x', '0.5', '-y', '0.0', '-z', '0.025'],
    #                  output='screen')


    load_joint_state_broadcaster = ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'controller_manager', 'spawner', 'joint_state_broadcaster',
                    '--controller-manager-timeout', '60',
                    '--controller-manager',
                    PathJoinSubstitution([namespace, 'controller_manager'])
                ],
                output='screen'
            )
    load_fr3_arm_controller = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner', 'fr3_arm_controller',
            '--controller-manager-timeout', '60',
            '--controller-manager',
            PathJoinSubstitution([namespace, 'controller_manager'])
        ],
        output='screen'
    )

    # Start block_pusher only after fr3_arm_controller spawner exits
    start_block_pusher_after_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=load_fr3_arm_controller,
            on_exit=[block_pusher_node],
        )
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    return LaunchDescription(
        [
            load_gripper_launch_argument,
            franka_hand_launch_argument,
            arm_id_launch_argument,
            namespace_launch_argument,
            db_arg,
            robot_state_publisher,
            run_move_group_node,
            gazebo_sim,
            spawn,
            load_joint_state_broadcaster,
            load_fr3_arm_controller,
            start_block_pusher_after_arm_controller,
            clock_bridge,
        ]
    )
