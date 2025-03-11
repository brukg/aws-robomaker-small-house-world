import os

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import FindExecutable, LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
import xacro
import pprint
def generate_launch_description():
    world_file_name = 'small_house.world'
    world_file_name = 'empty.world'
    package_dir = get_package_share_directory('aws_robomaker_small_house_world')
    desc_pkg_share = get_package_share_directory('unitree_robots_description')
    gz_version = LaunchConfiguration('gz_version')
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    headless = LaunchConfiguration("headless", default="false")
    world_path = os.path.join(package_dir, 'worlds', world_file_name)
    # gz_models_path = os.getenv('GZ_SIM_RESOURCE_PATH')

    gz_models_path = os.path.join(package_dir, "models", ":/home/phoenix/.gazebo/models", ":"+desc_pkg_share+"/models")
    
    robot = os.path.join(desc_pkg_share, "models/h1/urdf/robot.urdf")
    desc = xacro.parse(open(robot))
    xacro.process_doc(desc)
    # with open(H1_robot, 'r') as file:
    #     h1_description = file.read()
    
    # with open(A1_robot, 'r') as file:
    #     a1_description = file.read()

    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            # {'robot_description': h1_description}
            {'robot_description': desc.toxml()}
        ],
    )

    gazebo =  ExecuteProcess(
            condition=launch.conditions.IfCondition(headless),
            cmd=['ruby', FindExecutable(name="gz"), 'sim',  '-r', '-s', '--headless-rendering', world_path, '--force-version', gz_version],
            output='screen',
            # additional_env=gz_env, # type: ignore
            shell=False,
        )
    gazebo =  ExecuteProcess(
            condition=launch.conditions.UnlessCondition(headless),
            cmd=['ruby', FindExecutable(name="gz"), 'sim',  '-r', world_path, '--force-version', gz_version],
            output='screen',
            # additional_env=gz_env, # type: ignore
            shell=False,
        )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "h1_robot",
            "-topic", "robot_description",
            # '-string', desc.toxml(),
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "1.50",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )
    

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'
    )

    load_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'effort_controller'],
        output='screen'
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(desc_pkg_share, "rviz", "h1.rviz")],
    )
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/top_camera/rgb/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            "/top_camera/rgb/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
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
        DeclareLaunchArgument(
            name='model',
            default_value="a1",
            description='Robot description',
            choices=['a1', 'h1']
        ),
        gazebo,
        robot_state_publisher_node,
        # joint_state_publisher_gui_node,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_controller,
        #         on_exit=[load_effort_controller],
        #     )
        # ),
        ros_gz_bridge,
        rviz_node,
    ])


# if __name__ == '__main__':
#     generate_launch_description()
