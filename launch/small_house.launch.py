# /*******************************************************************************
# * Copyright 2019 ROBOTIS CO., LTD.
# *
# * Licensed under the Apache License, Version 2.0 (the "License");
# * you may not use this file except in compliance with the License.
# * You may obtain a copy of the License at
# *
# *     http://www.apache.org/licenses/LICENSE-2.0
# *
# * Unless required by applicable law or agreed to in writing, software
# * distributed under the License is distributed on an "AS IS" BASIS,
# * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# * See the License for the specific language governing permissions and
# * limitations under the License.
# *******************************************************************************/

# /* Author: Darby Lim */

import os

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    AndSubstitution,
)
from launch_ros.actions import Node

def generate_launch_description():
    world_file_name = 'small_house.world'
    package_dir = get_package_share_directory('aws_robomaker_small_house_world')
    gz_version = LaunchConfiguration('gz_version')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    headless = LaunchConfiguration("headless", default="false")
    world_path = os.path.join(package_dir, 'worlds', world_file_name)
    gz_models_path = os.path.join(package_dir, "models", ":/home/phoenix/.gazebo/models")

    box_bot_urdf = os.path.join(get_package_share_directory('aws_robomaker_small_house_world'), 'models', 'box_bot', 'urdf', 'box_bot_description.urdf')
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": Command(["xacro ", LaunchConfiguration("model")])}
        ],
    )

    # gazebo have to be executed with shell=False, or test_launch won't terminate it
    #   see: https://github.com/ros2/launch/issues/545
    # This code is form taken ros_gz_sim and modified to work with shell=False
    #   see: https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim/launch/gz_sim.launch.py.in
    gz_env = {'GZ_SIM_SYSTEM_PLUGIN_PATH':
           ':'.join([os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                     os.environ.get('LD_LIBRARY_PATH', default='')]),
           'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':  # TODO(CH3): To support pre-garden. Deprecated.
                      ':'.join([os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                                os.environ.get('LD_LIBRARY_PATH', default='')])}
    gazebo =  ExecuteProcess(
            condition=launch.conditions.IfCondition(headless),
            cmd=['ruby', FindExecutable(name="gz"), 'sim',  '-r', '-s', '--headless-rendering', world_path, '--force-version', gz_version],
            output='screen',
            additional_env=gz_env, # type: ignore
            shell=False,
        )
    gazebo =  ExecuteProcess(
            condition=launch.conditions.UnlessCondition(headless),
            cmd=['ruby', FindExecutable(name="gz"), 'sim',  '-r', world_path, '--force-version', gz_version],
            output='screen',
            additional_env=gz_env, # type: ignore
            shell=False,
        )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "box_bot",
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.50",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/rgb/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/rgb/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )
    return LaunchDescription([
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=gz_models_path,
        ),
        SetEnvironmentVariable(
            name="GZ_SIM_MODEL_PATH",
            value=gz_models_path,
        ),
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(package_dir, 'worlds', world_file_name), ''],
          description='SDF world file'),
        DeclareLaunchArgument(
            name="model",
            default_value=box_bot_urdf,
            description="Absolute path to robot urdf file",
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name="headless",
            default_value="False",
            description="Start GZ in hedless mode and don't start RViz (overrides use_rviz)",
        ),
        DeclareLaunchArgument(
            name='gz_version', 
            default_value='8',
            description='Gazebo Sim\'s major version'
        ),
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        ros_gz_bridge,
    ])


# if __name__ == '__main__':
#     generate_launch_description()
