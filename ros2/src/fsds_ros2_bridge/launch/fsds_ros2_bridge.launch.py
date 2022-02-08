from os.path import expanduser, join
import json 

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python import get_package_share_directory

CAMERA_FRAMERATE = 30.0

def generate_launch_description():
    with open(expanduser("~")+'/Formula-Student-Driverless-Simulator/settings.json', 'r') as file:
        settings = json.load(file)

    camera_configs = settings['Vehicles']['FSCar']['Cameras']
    if(not camera_configs):
        print('no cameras configured in ~/Formula-Student-Driverless-Simulator/settings.json')

    camera_nodes = [
        Node(
            package='fsds_ros2_bridge',
            executable='fsds_ros2_bridge_camera',
            namespace="fsds/camera", 
            name=camera_name,
            output='screen',
            parameters=[
                {'camera_name': camera_name},
                {'depthcamera': camera_config["CaptureSettings"][0]["ImageType"] == 2},
                {'framerate': CAMERA_FRAMERATE},
                {'host_ip': LaunchConfiguration('host')},
            ]
        ) for camera_name, camera_config in camera_configs.items()]

    host_arg = DeclareLaunchArgument(name='host',default_value='localhost')
    mission_arg = DeclareLaunchArgument(name='mission_name', default_value='trackdrive')
    track_arg = DeclareLaunchArgument(name='track_name', default_value='A')
    competition_arg = DeclareLaunchArgument(name='competition_mode', default_value='false')
    manual_arg = DeclareLaunchArgument(name='manual_mode', default_value='false')
    rviz_arg = DeclareLaunchArgument(name='rviz', default_value='false')

    bridge_node = Node(
        package='fsds_ros2_bridge',
        executable='fsds_ros2_bridge',
        name='ros_bridge',
        output='screen',
        parameters=[
            {
                'update_odom_every_n_sec': 0.004
            },
            {
                'update_imu_every_n_sec': 0.004
            },
            {
                'update_gps_every_n_sec': 0.1
            },
            {
                'update_gss_every_n_sec': 0.01
            },
            {
                'publish_static_tf_every_n_sec': 1.0
            },
            {
                'update_lidar_every_n_sec': 0.1
            },
            {
                'host_ip': LaunchConfiguration('host')
            },
            {
                'mission_name': LaunchConfiguration('mission_name')
            },
            {
                'track_name': LaunchConfiguration('track_name')
            },
            {
                'competition_mode': LaunchConfiguration('competition_mode')
            },
            {
                'manual_mode': LaunchConfiguration('manual_mode')
            },
        ]
    )

    tf_node = Node(
        package='fsds_transforms',
        executable='transform', 
        output='screen',
    )

    rviz_pkg = get_package_share_directory('launch_rviz')
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_pkg, '/launch/display.launch.py']),
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        host_arg,
        mission_arg,
        track_arg,
        competition_arg,
        manual_arg,
        rviz_arg,
        bridge_node,
        *camera_nodes,
        tf_node,
        rviz_launch,
    ])


if __name__ == '__main__':
    generate_launch_description()
