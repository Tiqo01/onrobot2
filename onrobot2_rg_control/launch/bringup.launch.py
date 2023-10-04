import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction

from launch_ros.actions import Node

def generate_launch_description():

    OnRobotRGStatusListener = Node(
        package='onrobot2_rg_control',
        executable='OnRobotRGStatusListener.py',
        name='OnRobotRGStatusListener',
        output='screen',
        emulate_tty=True
    )
    

    OnRobotRGTcpNode = Node(
        package='onrobot2_rg_control',
        executable='OnRobotRGTcpNode.py',
        name='OnRobotRGTcpNode',
        output='screen',
        emulate_tty=True,
        parameters=[{"/onrobot/ip": "192.168.1.1"},
                    {"/onrobot/port" : 502},
                    {"/onrobot/gripper": "rg2"},
                    {"/onrobot/changer_addr": 65},
                    {"/onrobot/dummy": False}]

    )
    

    return LaunchDescription([
        OnRobotRGStatusListener,
        OnRobotRGTcpNode,        
    ])
