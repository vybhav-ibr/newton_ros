# Copyright 2024 Open Source Robotics Foundation, Inc.
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    namespace = LaunchConfiguration("namespace", default="robot")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("newton_ros2_control_demos"),
                    "urdf",
                    "test_diff_drive.xacro.urdf",
                ]
            ),
            " ",
            f"namespace:={namespace}",
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("newton_ros2_control_demos"),
            "config",
            "diff_drive_controller.yaml",
        ]
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/r1/controller_manager"],
    )
    diff_drive_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_base_controller",
            "--param-file",
            robot_controllers,
            "-c",
            "/r1/controller_manager",
        ],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            (
                "/robot/controller_manager/robot_description",
                f"/robot/robot_description",
            ),
        ],
        output="screen",
        namespace=namespace,
    )

    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[diff_drive_base_controller_spawner],
                )
            ),
            node_robot_state_publisher,
            ros2_control_node,
            # Launch Arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="If true, use simulated clock",
            ),
            DeclareLaunchArgument(
                "namespace", default_value=namespace, description="Namespace to be used"
            ),
        ]
    )
