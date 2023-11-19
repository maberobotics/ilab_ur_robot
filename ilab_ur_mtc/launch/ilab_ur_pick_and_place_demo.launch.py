# Copyright 2023 MABE-ROBOTICS
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
        # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'arm_id',
            default_value='ur',
            description='arm_id of the joint names, useful for multi-robot setup. \
                         If changed than also joint names in the controllers \
                         configuration have to be updated. Expected format "<prefix>/"',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'base_frame_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('urman_description'), 'config', 'base_frame.yaml']
            ),
            description='Configuration file of robot base frame wrt World.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Start robot with fake hardware mirroring command to its states.',
        )
    )

    # Initialize Arguments
    arm_id = LaunchConfiguration('arm_id')
    start_rviz = LaunchConfiguration('start_rviz')
    base_frame_file = LaunchConfiguration('base_frame_file')
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')


    robot_description_content = Command(
      [
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution(
            [FindPackageShare('urman_description'), 'urdf', 'urman.urdf.xacro']),
        ' ',
        'name:=',
        arm_id,
        ' ',
        'sim_gazebo:=',
        use_sim,
        ' ',
        'ur_type:=ur5e',
        ' ',
        'use_fake_hardware:=',
        use_fake_hardware,
        ' ',
        'base_frame_file:=',
        base_frame_file
      ]
    )

    robot_description = {'robot_description': robot_description_content}

    # Get SRDF via xacro
    robot_description_semantic_content = Command(
      [
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare('urman_description'), 'srdf', 'urman.srdf.xacro']
        ),
        ' ',
        'name:=',
        arm_id,
      ]
    )

    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }

    # Get planning parameters
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }

        # Get planning parameters
    robot_description_planning_joint_limits = PathJoinSubstitution([
            FindPackageShare('urman_description'), "moveit2", "joint_limits.yaml",
        ]
    )

    robot_description_kinematics = PathJoinSubstitution(
      [FindPackageShare("urman_description"), "moveit2", "kinematics.yaml"]
    )

    robot_description_planning_cartesian_limits = PathJoinSubstitution([
        FindPackageShare('urman_description'), 'config', 'cartesian_limits.yaml',
      ]
    )

    move_group_capabilities = {
      'capabilities': '''pilz_industrial_motion_planner/MoveGroupSequenceAction \
            pilz_industrial_motion_planner/MoveGroupSequenceService \
            move_group/ExecuteTaskSolutionCapability'''
    }

    planning_pipelines_config = PathJoinSubstitution([
      FindPackageShare('urman_description'), 'moveit2', 'planning_pipelines_config.yaml'
    ])

    ompl_planning_config = PathJoinSubstitution([
      FindPackageShare('urman_description'), 'moveit2', 'ompl_planning.yaml'
    ])

    moveit_controllers = PathJoinSubstitution(
      [FindPackageShare("urman_description"), "moveit2", "moveit_controller_config.yaml"]
    )

    trajectory_execution = {
      'moveit_manage_controllers': False,
      'trajectory_execution.allowed_execution_duration_scaling': 1.2,
      'trajectory_execution.allowed_goal_duration_margin': 0.5,
      'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
      'publish_planning_scene': True,
      'publish_geometry_updates': True,
      'publish_state_updates': True,
      'publish_transforms_updates': True,
      "planning_scene_monitor_options": {
        "name": "planning_scene_monitor",
        "robot_description": "robot_description",
        "joint_state_topic": "/joint_states",
        "attached_collision_object_topic": "/move_group/planning_scene_monitor",
        "publish_planning_scene_topic": "/move_group/publish_planning_scene",
        "monitored_planning_scene_topic": "/move_group/monitored_planning_scene",
        "wait_for_initial_state_timeout": 10.0,
      },
    }

    # MTC Demo node
    pick_place_demo = Node(
        package="ilab_ur_mtc",
        executable="ilab_ur_mtc_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning_cartesian_limits,
            robot_description_planning_joint_limits,
            planning_pipelines_config,
            ompl_planning_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities,
        ],
    )

    return LaunchDescription(declared_arguments + [pick_place_demo])