import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():

    package_name = "simple_avoidance_sim"
    pkg_share = get_package_share_directory(package_name)

    gazebo_model_path = "/home/luc/ProjetG1G2/simpletest_ws/src/gazebo_models_worlds_collection/models"

    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=gazebo_model_path + ":" + os.environ.get("GAZEBO_MODEL_PATH", "")
    )

    world_file = "/home/luc/ProjetG1G2/simpletest_ws/src/gazebo_models_worlds_collection/worlds/small_city.world"
    xacro_file = os.path.join(pkg_share, "urdf", "simple_car.urdf.xacro")

    robot_description = Command(["xacro ", xacro_file])

    # Gazebo
    gazebo = ExecuteProcess(
        cmd=[
            "gazebo",
            world_file,
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
    )

    # robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_description},
        ],
    )

    # spawn vehicle
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "simple_car",
            "-topic", "robot_description",
            "-x", "44.5",
            "-y", "40.0",
            "-z", "0.5",
            "-Y", "4.7124",
        ],
        output="screen",
    )

    # controllers
    joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    front_steering_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "front_steering_controller",
        ],
        output="screen",
    )

    rear_wheel_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "rear_wheel_controller",
        ],
        output="screen",
    )

    # Ackermann controller
    # 保留它：它负责把 /cmd_vel 转成前轮转角和后轮速度
    ackermann_controller = Node(
        package="simple_avoidance_sim",
        executable="ackermann_controller_node",
        name="ackermann_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": True},
        ],
    )

    # New obstacle avoidance node
    # 注意：这里启动的是新的 town_laser_avoidance_node
    # 不再启动旧的 obstacle_avoidance_node
    town_laser_avoidance = Node(
        package="simple_avoidance_sim",
        executable="town_laser_avoidance_node",
        name="town_obstacle_avoidance_node",
        output="screen",
        parameters=[
            {"use_sim_time": True},

            {"scan_topic": "/scan"},
            {"cmd_vel_topic": "/cmd_vel"},

            {"control_frequency": 20.0},

            {"slow_down_distance": 2.5},
            {"stop_distance": 0.8},

            {"cruise_speed": 0.45},
            {"slow_speed": 0.18},

            {"turn_speed": 0.6},
            {"avoidance_turn_gain": 0.25},
            {"max_angular_z": 0.35},

            {"front_angle_deg": 20.0},
            {"side_angle_min_deg": 20.0},
            {"side_angle_max_deg": 70.0},

            {"scan_timeout": 0.5},
            {"self_ignore_distance": 0.35},
            
            {"road_center_error_topic": "/road/center_error"},
            {"road_steer_gain": 1.2},
        ],
    )
    
    black_road_follower = Node(
	    package="simple_avoidance_sim",
	    executable="black_road_follower_node",
	    name="black_road_follower_node",
	    output="screen",
	    parameters=[
		{"use_sim_time": True},
		{"image_topic": "/camera/front_camera/image_raw"},
		{"center_error_topic": "/road/center_error"},

		{"roi_start_ratio": 0.55},
		{"roi_end_ratio": 0.95},
		{"black_threshold": 60},
		{"min_road_area": 500},
	    ],
	)

    return LaunchDescription(
        [
            set_gazebo_model_path,
            gazebo,
            robot_state_publisher,

            TimerAction(period=3.0, actions=[spawn_entity]),

            TimerAction(period=6.0, actions=[joint_state_broadcaster]),
            TimerAction(period=7.0, actions=[front_steering_controller]),
            TimerAction(period=8.0, actions=[rear_wheel_controller]),

            TimerAction(period=10.0, actions=[ackermann_controller]),

            # 这里启动新避障节点
            TimerAction(period=11.0, actions=[black_road_follower]),
            TimerAction(period=12.0, actions=[town_laser_avoidance]),
        ]
    )
