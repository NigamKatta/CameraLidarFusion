#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose_turtlebot = LaunchConfiguration('x_pose_turtlebot', default='-2.0')
    y_pose_turtlebot = LaunchConfiguration('y_pose_turtlebot', default='0.0')
    
    x_pose_chessboard = LaunchConfiguration('x_pose_chessboard', default='0.0')
    y_pose_chessboard = LaunchConfiguration('y_pose_chessboard', default='0.0')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'camera_calibrate.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose_turtlebot,
            'y_pose': y_pose_turtlebot
        }.items()
    )

    spawn_chessboard_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_chessboard.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose_chessboard,
            'y_pose': y_pose_chessboard
        }.items()
    )

    # ## Run this Node separatly if you want to calibrate a real-time camera.
    # spawn_cameracalibrate_cmd = Node(
        
    #     package = 'camera_calibration',
    #     executable='cameracalibrator',
    #     arguments=[
    #         '-p', 'checkerboard',
    #         '--no-service-check',
    #         '--size', '8x6',
    #         '--square', '0.02',
    #         '--ros-args',
    #     ],
    #     remappings=[
    #             ('image', '/camera/image_raw'),
    #         ],  
    # )

    # ## Camera position-orientation change.
    # spawn_chessstate_cmd = TimerAction( 
    #     period=2.0, # delay bu 2 seconds
    #     actions=[
    #                 Node(
    #                         package = 'turtlebot3_gazebo',
    #                         executable='chess_state_publisher.py',
    #                     ),
    #             ],
    # )
    

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(spawn_chessboard_cmd)
    #ld.add_action(spawn_cameracalibrate_cmd)
    #ld.add_action(spawn_chessstate_cmd)

    return ld
