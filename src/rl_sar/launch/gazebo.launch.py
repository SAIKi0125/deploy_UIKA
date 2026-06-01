# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rname = LaunchConfiguration("rname")
    catchball_debug = LaunchConfiguration("catchball_debug")

    wname = "earth"
    robot_name = ParameterValue(Command(["echo -n ", rname]), value_type=str)
    ros_namespace = ParameterValue(Command(["echo -n ", "/", rname, "_gazebo"]), value_type=str)
    gazebo_entity_name = Command(["echo -n ", rname, "_gazebo"])
    gazebo_model_name = ParameterValue(gazebo_entity_name, value_type=str)

    robot_description = ParameterValue(
        Command([
            "xacro ",
            Command(["echo -n ", Command(["ros2 pkg prefix ", rname, "_description"])]),
            "/share/", rname, "_description/xacro/robot.xacro"
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            # "verbose": "true",
            # "pause": "true",  # Not Available
            "world": os.path.join(get_package_share_directory("rl_sar"), "worlds", wname + ".world"),
        }.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "/robot_description",
            "-entity", gazebo_entity_name,
            "-z", "1.0",
        ],
        output="screen",
    )

    spawn_catchball = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file", os.path.join(get_package_share_directory("rl_sar"), "models", "catchball", "model.sdf"),
            "-entity", "catchball",
            "-z", "-2.0",
        ],
        output="screen",
    )

    catchball_thrower_node = Node(
        package="rl_sar",
        executable="catchball_thrower.py",
        name="catchball_thrower",
        output="screen",
        parameters=[{
            "robot_model_name": gazebo_model_name,
            "enabled": ParameterValue(
                PythonExpression(["'", catchball_debug, "'.lower() not in ['true', '1', 'yes', 'on']"]),
                value_type=bool,
            ),
        }],
    )

    catchball_fov_visualizer_node = Node(
        package="rl_sar",
        executable="catchball_fov_visualizer.py",
        name="catchball_fov_visualizer",
        output="screen",
        parameters=[{
            "robot_model_name": gazebo_model_name,
            "enabled": True,
        }],
    )

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable='spawner.py' if os.environ.get('ROS_DISTRO', '') == 'foxy' else 'spawner',
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    robot_joint_controller_node = Node(
        package="controller_manager",
        executable='spawner.py' if os.environ.get('ROS_DISTRO', '') == 'foxy' else 'spawner',
        arguments=["robot_joint_controller"],
        output="screen",
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'deadzone': 0.1,
            'autorepeat_rate': 0.0,
        }],
    )

    param_node = Node(
        package="demo_nodes_cpp",
        executable="parameter_blackboard",
        name="param_node",
        parameters=[{
            "robot_name": robot_name,
            "gazebo_model_name": gazebo_model_name,
            "catchball_debug": ParameterValue(catchball_debug, value_type=bool),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rname",
            description="Robot name (e.g., a1, go2)",
            default_value=TextSubstitution(text=""),
        ),
        DeclareLaunchArgument(
            "catchball_debug",
            description="Enable manual ball_pos_b debug and disable real Gazebo ball throws.",
            default_value=TextSubstitution(text="false"),
        ),
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        spawn_catchball,
        joint_state_broadcaster_node,
        # robot_joint_controller_node,  # Spawn in rl_sim.cpp
        joy_node,
        catchball_thrower_node,
        catchball_fov_visualizer_node,
        param_node,
    ])
