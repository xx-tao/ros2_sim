# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Package Directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    ros2_sim = get_package_share_directory('ros2_sim')

    # Parse robot description from xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ros2_sim"),
                    "urdf",
                    "example_d415_gazebo.urdf.xacro",
                ]
            ),
            " ",
            "camera_name:=gz_camera",
            " ",
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": robot_description_content,
            }
        ],
    )
    small_checkerboard_file = os.path.join(ros2_sim, 'urdf', 'small_checkerboard.sdf')

    world_file = os.path.join(ros2_sim, 'worlds', 'gravity_free.sdf')

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": robot_description_content,
            }
        ],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        # launch_arguments={'gz_args': '-r empty.sdf'}.items(),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(ros2_sim, 'rviz', 'joint_states.rviz')],
    )

    # Spawn
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "realsense_camera",
            "-allow_renaming",
            "true",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0",
        ],
    )

    spawn_2 = Node(
        package='ros_gz_sim',
        executable='create',
        parameters=[{'name': 'small_checkerboard',
                     'file': small_checkerboard_file}],  #
        output='screen',
    )

    # Gz - ROS Bridge
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        remappings=[
            (
                "/camera/image",
                "/camera/camera/color/image_raw",
            ),
            (
                "/camera/depth_image",
                "/camera/camera/depth_registered/image_rect",
            ),
            (
                "/camera/points",
                "/camera/camera/depth/color/points",
            ),
            (
                "/camera/camera_info",
                "/camera/camera/color/camera_info",
            ),
            (
                "/camera/camera_info",
                "/camera/camera/depth_registered/camera_info",
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            # Nodes and Launches
            gazebo,
            spawn,
            spawn_2,
            gazebo_bridge,
            robot_state_publisher_node,
            # rviz,
            joint_state_publisher,
        ]
    )
