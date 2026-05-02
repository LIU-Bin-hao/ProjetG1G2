import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'simple_avoidance_sim'
    pkg_share = get_package_share_directory(package_name)

    world_file = os.path.join(pkg_share, 'worlds', 'simple_test.world')
    xacro_file = os.path.join(pkg_share, 'urdf', 'simple_car.urdf.xacro')

    robot_description = Command(['xacro ', xacro_file])

    # 1) Gazebo
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    # 2) robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description}
        ]
    )

    # 3) spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'simple_car',
            '-topic', 'robot_description',
            '-x', '4.5',
            '-y', '0.0',
            '-z', '0.5',
            '-Y', '3.1416'
        ],
        output='screen'
    )

    # 4) controllers
    joint_state_broadcaster = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'joint_state_broadcaster'
        ],
        output='screen'
    )

    front_steering_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'front_steering_controller'
        ],
        output='screen'
    )

    rear_wheel_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'rear_wheel_controller'
        ],
        output='screen'
    )

    # 5) ackermann controller node
    ackermann_controller = Node(
        package='simple_avoidance_sim',
        executable='ackermann_controller_node',
        name='ackermann_controller_node',
        output='screen',
        parameters=[
            {'wheelbase': 0.8},
            {'wheel_radius': 0.12},
            {'max_steering_angle': 0.5}
        ]
    )

    # 6) goal_navigation node
    goal_navigation = Node(
        package='simple_avoidance_sim',
        executable='goal_navigation_node',
        name='goal_navigation_node',
        output='screen'
    )

    # 顺序控制：
    # 先启动 Gazebo 和 robot_state_publisher
    # 再 spawn
    # spawn 成功后加载 controllers
    # controllers 再起来后启动 ackermann + obstacle avoidance

    load_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                TimerAction(period=2.0, actions=[joint_state_broadcaster])
            ]
        )
    )

    load_front_steering = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[
                TimerAction(period=2.0, actions=[front_steering_controller])
            ]
        )
    )

    load_rear_wheel = RegisterEventHandler(
        OnProcessExit(
            target_action=front_steering_controller,
            on_exit=[
                TimerAction(period=2.0, actions=[rear_wheel_controller])
            ]
        )
    )

    start_ackermann = RegisterEventHandler(
        OnProcessExit(
            target_action=rear_wheel_controller,
            on_exit=[
                TimerAction(period=2.0, actions=[ackermann_controller])
            ]
        )
    )

    start_goal_navigation = RegisterEventHandler(
        OnProcessExit(
            target_action=rear_wheel_controller,
            on_exit=[
                TimerAction(period=3.0, actions=[goal_navigation])
            ]
        )
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        TimerAction(period=3.0, actions=[spawn_entity]),
        load_joint_state_broadcaster,
        load_front_steering,
        load_rear_wheel,
        start_ackermann,
        start_goal_navigation,
    ])
