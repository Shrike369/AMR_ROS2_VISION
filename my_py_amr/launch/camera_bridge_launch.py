"""
Launch Gazebo-ROS bridge and camera bridge node.

This launch file starts the ``parameter_bridge`` from ``ros_gz_bridge`` to
bridge the Gazebo camera topics into ROS ``sensor_msgs`` types, and then
launches our ``camera_bridge`` node which republished the camera image to
``/camera/image_raw``.
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # Bridge the raw Gazebo image and camera_info to ROS types
    bridge_args = [
        '/world/empty/model/camera_bot/link/base_link/sensor/top_camera/image'
        + '@sensor_msgs/msg/Image@gz.msgs.Image',
        '/world/empty/model/camera_bot/link/base_link/sensor/top_camera/camera_info'
        + '@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            # Bridge ROS cmd_vel -> Gazebo Twist and Gazebo odom/joint_state -> ROS
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/amr_with_markers/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/amr_with_markers/joint_state@sensor_msgs/msg/JointState@gz.msgs.JointState',
    ]

    parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        arguments=bridge_args,
        output='screen',
    )

    camera_bridge = Node(
        package='my_py_amr',
        executable='camera_bridge',
        name='camera_bridge',
        output='screen',
    )

    # Convert incoming camera images to an OccupancyGrid map
    image_to_occupancy = Node(
        package='my_py_amr',
        executable='image_to_occupancy',
        name='image_to_occupancy',
        output='screen',
    )

    # Vision-based transform: detect red/blue markers on robot top and publish map->base_link
    top_marker_tf = Node(
        package='my_py_amr',
        executable='top_down_marker_tf',
        name='top_down_marker_tf',
        output='screen',
        parameters=[{
            'camera_topic': '/camera/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'camera_frame': 'map',
            'base_frame': 'base_link',
            'camera_height': 10.13,
                    'invert_y': True,
            'debug': False,
            'publish_pose': True,
        }],
    )

    # Small helper: forward Nav2 velocity topic -> /cmd_vel so Gazebo receives commands
    republish_cmd = Node(
        package='my_py_amr',
        executable='republish_cmd_vel',
        name='republish_cmd_vel',
        output='screen',
        parameters=[{
            'in_topic': '/cmd_vel_nav',
            'out_topic': '/cmd_vel',
        }],
    )

    # Republish Gazebo model odometry -> /odom for ROS consumers
    republish_odom = Node(
        package='my_py_amr',
        executable='republish_odometry',
        name='republish_odometry',
        output='screen',
        parameters=[{
            'in_topic': '/model/amr_with_markers/odometry',
            'out_topic': '/odom',
        }],
    )

    # Republish joint states by synthesizing wheel angles from odometry
    republish_joints = Node(
        package='my_py_amr',
        executable='republish_joint_states',
        name='republish_joint_states',
        output='screen',
        parameters=[{
            'in_topic': '/odom',
            'left_joint': 'left_wheel_joint',
            'right_joint': 'right_wheel_joint',
            'wheel_separation': 0.40,
            'wheel_radius': 0.09,
        }],
    )

    # Include Nav2 bringup (use a params file in this package)
    nav2_share = get_package_share_directory('nav2_bringup')
    my_share = get_package_share_directory('my_py_amr')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'params_file': os.path.join(my_share, 'params', 'nav2_params.yaml'),
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([parameter_bridge, camera_bridge, image_to_occupancy, top_marker_tf, republish_cmd, republish_odom, republish_joints, nav2_launch])
