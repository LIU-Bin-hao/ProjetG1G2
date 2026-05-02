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
            "-Y", "3.1416",
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
    ackermann_controller = Node(
        package="simple_avoidance_sim",
        executable="ackermann_controller_node",
        output="screen",
    )

    # simple obstacle avoidance
    obstacle_avoidance = Node(
        package="simple_avoidance_sim",
        executable="obstacle_avoidance_node",
        output="screen",
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
        TimerAction(period=11.0, actions=[obstacle_avoidance]),
    ]
)
