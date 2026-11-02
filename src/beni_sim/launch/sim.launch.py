from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def start(context):
    world = str(Path(get_package_share_directory('beni_sim')) / 'worlds' / 'room.sdf')
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    cmd = ['ign', 'gazebo', '-r', world]
    if not gui:
        cmd += ['-s', '--headless-rendering']
    topics = [
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
        '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
    ]
    return [ExecuteProcess(cmd=cmd, output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=topics, output='screen'),
        Node(package='beni_sim', executable='drive_guard', output='screen'),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Show Gazebo GUI'),
        OpaqueFunction(function=start),
    ])
