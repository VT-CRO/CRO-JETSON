import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'crobot-bringup'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('crobot-gazebo'), 'launch', 'launch_sim.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('crobot-navigation'), 'launch', 'rgbd.launch.py'
        ))
    )

    nav2_dir = get_package_share_directory('nav2_bringup')
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',

            # need to specify a params file otherwise nav2 doesnt work
            'params_file': os.path.join(
                get_package_share_directory('crobot-navigation'), 'config', 'nav2_params.yaml'
            )
        }.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[
            ("cmd_vel_out", "diff_drive_controller/cmd_vel_unstamped")
        ]
    )

    return LaunchDescription([
        gazebo,
        slam,
        nav,
        twist_mux
    ])