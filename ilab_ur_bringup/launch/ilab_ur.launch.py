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
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros import parameter_descriptions
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart


def generate_launch_description():
    
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'arm_id',
            default_value='ur',
            description='Prefix of the joint names, useful for multi-robot setup. \
                         If changed than also joint names in the controllers \
                         configuration have to be updated. Expected format "<prefix>/"',
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
            default_value='false',
            description='Start robot with fake hardware mirroring command to its states.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_planning',
            default_value='false',
            description='Start robot with Moveit2 `move_group` planning \
                         config for Pilz and OMPL.',
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
            'robot_ip',
            default_value='0.0.0.0',
            description='Robot IP of FRI interface',
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
            'namespace',
            default_value='/',
            description='Namespace of launched nodes, useful for multi-robot setup. \
                         If changed than also the namespace in the controllers \
                         configuration needs to be updated. Expected format "<ns>/".',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_gazebo',
            default_value='false',
            description='Start Gazebo simulation.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'merge_joint_states',
            default_value='true',
            description='Merge joint states topics into /joint_states.',
        )
    )

    # Initialize Arguments
    arm_id = LaunchConfiguration('arm_id')
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_planning = LaunchConfiguration('use_planning')
    start_rviz = LaunchConfiguration('start_rviz')
    robot_ip = LaunchConfiguration('robot_ip')
    base_frame_file = LaunchConfiguration('base_frame_file')
    namespace = LaunchConfiguration('namespace')
    start_gazebo = LaunchConfiguration('start_gazebo')
    merge_joint_states = LaunchConfiguration('merge_joint_states')

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
        'robot_ip:=',
        robot_ip,
        ' ',
        'base_frame_file:=',
        base_frame_file,
        ' ',
        'namespace:=',
        namespace,
      ]
    )
    robot_description = {
      'robot_description': parameter_descriptions.ParameterValue(robot_description_content, value_type=str)
    }

     # Running with Moveit2 planning
    ilab_ur_planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ilab_ur_bringup'),
            '/launch',
            '/ilab_ur_planning.launch.py'
        ]),
        launch_arguments={
            'arm_id': arm_id,
            'start_rviz': start_rviz,
            'base_frame_file': base_frame_file,
            'use_sim': use_sim,
            'use_fake_hardware': use_fake_hardware,
            'namespace': namespace,
        }.items(),
        condition=IfCondition(use_planning),
    )

    robot_controllers = PathJoinSubstitution(
      [FindPackageShare('ilab_ur_description'), 'config', 'controllers.yaml']
    )

    rviz_config_file = PathJoinSubstitution(
      [FindPackageShare('ilab_ur_description'), 'rviz', 'ilab_ur.rviz']
    )

    robot_state_publisher_node = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      output='both',
      parameters=[robot_description],
      namespace=namespace,
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[
            {'source_list': ['/ur/joint_states'], 'rate': 30}],
        condition=IfCondition(merge_joint_states),
    )

    rviz_node = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      output='log',
      arguments=['-d', rviz_config_file],
      parameters=[
        robot_description,
      ],
      condition=UnlessCondition(use_planning),
    )

    ilab_ur_simulation_world = PathJoinSubstitution(
        [FindPackageShare('ilab_ur_description'),
            'gazebo/worlds', 'empty.world']
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution(
                [FindPackageShare('gazebo_ros'),
                    'launch', 'gazebo.launch.py']
            )]
        ),
        launch_arguments={'verbose': 'true', 'world': ilab_ur_simulation_world}.items(),
        condition=IfCondition(start_gazebo),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', [namespace, 'robot_description'], '-entity', [namespace, arm_id]],
        output='screen',
        condition=IfCondition(use_sim),
    )

    control_node = Node(
      package='controller_manager',
      executable='ros2_control_node',
      parameters=[robot_description, robot_controllers],
      output='both',
      remappings=[('/joint_states', '/ur/joint_states')],
      namespace=namespace,
      condition=UnlessCondition(use_sim),
    )

    robot_controller_spawner = Node(
      package='controller_manager',
      executable='spawner',
      arguments=['scaled_joint_trajectory_controller', '-c', [namespace, 'controller_manager']]
    )

    joint_state_broadcaster_spawner = Node(
      package='controller_manager',
      executable='spawner',
      arguments=['joint_state_broadcaster', '-c', [namespace, 'controller_manager']]
    )

    gripper_controller_spawner = Node(
      package='controller_manager',
      executable='spawner',
      arguments=['gripper_controller', '-c', [namespace, 'controller_manager']]
    )

    # Delay `joint_state_broadcaster` after spawn_entity
    delay_joint_state_broadcaster_spawner_after_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        ),
        condition=IfCondition(use_sim),
    )

    # Delay `joint_state_broadcaster` after control_node
    delay_joint_state_broadcaster_spawner_after_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[joint_state_broadcaster_spawner],
        ),
        condition=UnlessCondition(use_sim),
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(start_rviz),
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # Delay start of gripper_controller after `joint_state_broadcaster`
    delay_gripper_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    nodes_to_start = [
      ilab_ur_planning_launch,
      gazebo,
      spawn_entity,
      delay_joint_state_broadcaster_spawner_after_spawn_entity,
      delay_joint_state_broadcaster_spawner_after_control_node,
      robot_state_publisher_node,
      joint_state_publisher,
      delay_rviz_after_joint_state_broadcaster_spawner,
      control_node,
      delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
      delay_gripper_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
