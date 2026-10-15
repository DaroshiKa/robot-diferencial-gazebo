"""
Un solo launch: Gazebo (pista) + robot_state_publisher + spawn + bridge ROS<->Gazebo
+ nodo de cinemática (comparación de odometrías) + RViz2.

    ros2 launch robot_diferencial simulacion.launch.py
    ros2 launch robot_diferencial simulacion.launch.py rviz:=false
    ros2 launch robot_diferencial simulacion.launch.py csv:=/tmp/odometrias.csv

Teleoperación (otra terminal, con el workspace cargado):
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

NOMBRE_ROBOT = 'robot_diferencial'
NOMBRE_MUNDO = 'pista'


def generate_launch_description():
    pkg = get_package_share_directory('robot_diferencial')
    xacro_file = os.path.join(pkg, 'urdf', 'diferencial.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'pista.sdf')
    rviz_file = os.path.join(pkg, 'rviz', 'control.rviz')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    rviz = LaunchConfiguration('rviz')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    # Punto de spawn = línea Start/Finish de la pista (origen del mundo, yaw 0 = tangente).
    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', NOMBRE_ROBOT, '-topic', 'robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '0.07', '-Y', '0.0'],
        output='screen',
    )

    # ]  ROS -> Gazebo     [  Gazebo -> ROS
    joint_state_gz = f'/world/{NOMBRE_MUNDO}/model/{NOMBRE_ROBOT}/joint_state'
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            f'{joint_state_gz}@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # camera_info: según la versión de gz-sensors sale en /camera/camera_info o en
            # /camera/image_raw/camera_info; se puentean ambos y se unifica el nombre en ROS.
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/image_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            (joint_state_gz, '/joint_states'),
            ('/camera/image_raw/camera_info', '/camera/camera_info'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    cinematica = Node(
        package='diferencial_control', executable='cinematica_diferencial',
        parameters=[{'use_sim_time': True, 'csv': LaunchConfiguration('csv')}], output='screen',
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}], condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('csv', default_value='',
                              description='ruta de un .csv donde el nodo guarda ambas odometrias'),
        gazebo, robot_state_publisher, spawn, bridge, cinematica, rviz_node,
    ])
