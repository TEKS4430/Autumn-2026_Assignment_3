#!/usr/bin/env python3
"""
assignment.launch.py — TEKS4430 Hybrid Architecture Assignment

What gets launched:
    1. WebotsController: TurtleBot3Burger  — sensor topics + motor control
    2. task2_tree                          — STUDENT: behavior tree node

This package is self-contained. It includes:
    - robot_driver.py  (copied from Assignment 1)
    - TurtleBot3Burger.urdf  (copied from Assignment 1)
    - break_room.wbt  (copied from Assignment 1, open manually in Webots)

Prerequisites:
    - Webots is open with worlds/break_room.wbt from this package
    - py_trees is installed:
        sudo apt install ros-jazzy-py-trees ros-jazzy-py-trees-ros-interfaces ros-jazzy-py-trees-ros

Usage:
    ros2 launch hybrid_assignment assignment.launch.py
"""

import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():

    # URDF from this package — self-contained, no dependency on sensing_assignment
    hybrid_pkg = get_package_share_directory('hybrid_assignment')
    robot_urdf = os.path.join(
        hybrid_pkg, 'resource', 'TurtleBot3Burger.urdf')

    # TurtleBot3 Webots driver
    # Publishes: /scan /imu /odom
    # Subscribes: /cmd_vel (behavior tree publishes here)
    turtlebot_driver = WebotsController(
        robot_name='TurtleBot3Burger',
        parameters=[
            {'robot_description': robot_urdf},
        ],
        output='screen',
    )

    # Student behavior tree node
    bt_node = Node(
        package='hybrid_assignment',
        executable='task2_tree',
        name='hybrid_behaviour_tree',
        output='screen',
    )

    return LaunchDescription([
        turtlebot_driver,
        bt_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=turtlebot_driver,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown()
                )],
            )
        ),
    ])
