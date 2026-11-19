"""Pone (o quita) un obstáculo en la pista con la simulación ya corriendo: prueba del AEB.

    ros2 launch robot_diferencial obstaculo.launch.py                 # x = 4.0, y = 0.0 (recta principal)
    ros2 launch robot_diferencial obstaculo.launch.py x:=2.5 y:=0.1 nombre:=obs2
    gz service -s /world/<world>/remove ...  (o desde la GUI de Gazebo) para quitarlo
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sdf = os.path.join(get_package_share_directory('robot_diferencial'), 'models', 'obstaculo', 'model.sdf')
    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='4.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('nombre', default_value='obstaculo'),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-file', sdf, '-name', LaunchConfiguration('nombre'),
                        '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'), '-z', '0.0']),
    ])
