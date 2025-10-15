# Copyright (c) 2024 Franka Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch.actions import IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return {}

def setup_moveit_nodes(context: LaunchContext, arm_id, load_gripper, franka_hand, namespace):
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)
    namespace_str = context.perform_substitution(namespace)
    namespace_kwargs = {}
    if namespace_str:
        namespace_kwargs['namespace'] = namespace_str

    ros2_controllers_path = os.path.join(
        get_package_share_directory('push_block'),
        'config',
        'fr3_ros_controllers.yaml',
    )

    franka_share = get_package_share_directory('franka_description')
    franka_xacro_file = os.path.join(franka_share, 'robots', arm_id_str, f'{arm_id_str}.urdf.xacro')
    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'arm_id': arm_id_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': franka_hand_str,
            'controller_config': ros2_controllers_path,
            'ros2_control_plugin_parameters_file': ros2_controllers_path,
        },
    ).toxml()
    robot_description = {'robot_description': robot_description_config}

    franka_semantic_file = os.path.join(franka_share, 'robots', arm_id_str, f'{arm_id_str}.srdf.xacro')
    semantic_config = xacro.process_file(
        franka_semantic_file,
        mappings={'hand': load_gripper_str},
    ).toxml()
    robot_description_semantic = {'robot_description_semantic': semantic_config}

    kinematics_config = {
        'robot_description_kinematics': load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')
    }
    joint_limits_config = {
        'robot_description_planning': load_yaml('franka_fr3_moveit_config', 'config/fr3_joint_limits.yaml')
    }
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
                'default_planning_response_adapters/DisplayMotionPath',
            ],
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline_config['move_group'].update(
        load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml')
    )

    moveit_controllers = {
        'moveit_simple_controller_manager': load_yaml(
            'franka_fr3_moveit_config', 'config/fr3_controllers.yaml'
        ),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
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
        'publish_robot_description': True,
        'publish_robot_description_semantic': True,
        'publish_robot_description_kinematics': True,
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
        **namespace_kwargs,
    )
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        arguments=['--ros-args', '-r', '__ns:=/'],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            joint_limits_config,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
        **namespace_kwargs,
    )
    rviz_config = os.path.join(
        get_package_share_directory('franka_fr3_moveit_config'),
        'rviz',
        'moveit.rviz',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', rviz_config, '-f', 'world'],
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_config,
        ],
        **namespace_kwargs,
    )
    
    return [robot_state_publisher, move_group, rviz]


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
            default_value='true',
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
    # gazebo_sim = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    #     launch_arguments={'gz_args': 'empty.sdf -r', }.items(),
    # )

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
    
    moveit_stack = OpaqueFunction(
        function=setup_moveit_nodes,
        args=[arm_id, load_gripper, franka_hand, namespace],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    return LaunchDescription([
        clock_bridge,
        load_gripper_launch_argument,
        franka_hand_launch_argument,
        arm_id_launch_argument,
        namespace_launch_argument,
        gazebo_sim,
        moveit_stack,
        spawn,
        # spawn_block,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_fr3_arm_controller],
            )
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            parameters=[{'source_list': ['joint_states'], 'rate': 30}],
        ),
    ])
