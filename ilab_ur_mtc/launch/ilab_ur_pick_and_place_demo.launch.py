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
            'base_frame_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('ilab_ur_description'), 'config', 'base_frame.yaml']
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
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='/',
            description='Namespace of launched nodes, useful for multi-robot setup. \
                         If changed than also the namespace in the controllers \
                         configuration needs to be updated. Expected format "<ns>/".',
        )
    )

    # Initialize Arguments
    arm_id = LaunchConfiguration('arm_id')
    base_frame_file = LaunchConfiguration('base_frame_file')
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    namespace = LaunchConfiguration('namespace')


    robot_description_content = Command(
      [
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution(
            [FindPackageShare('ilab_ur_description'), 'urdf', 'ilab_ur.urdf.xacro']),
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
            [FindPackageShare('ilab_ur_description'), 'srdf', 'ilab_ur.srdf.xacro']
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

    robot_description_kinematics = PathJoinSubstitution(
      [FindPackageShare("ilab_ur_description"), "moveit2", "kinematics.yaml"]
    )

    # MTC Demo node
    pick_place_demo = Node(
        package="ilab_ur_mtc",
        executable="ilab_ur_mtc_node",
        output="screen",
        namespace=namespace,
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    return LaunchDescription(declared_arguments + [pick_place_demo])