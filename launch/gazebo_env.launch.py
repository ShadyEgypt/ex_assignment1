#!/usr/bin/env python3

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():

    pkg_ex_assignment1 = get_package_share_directory("ex_assignment1")
    pkg_sensors = get_package_share_directory("bme_gazebo_sensors")

    # Absolute path to your NON-ROS aruco folder
    aruco_path = "~/gz_ros2_aruco_ws/src/ros2-gazebo-aruco/gz-world/aruco_box"

    # ---------------------------------------------
    # Extend MODEL PATH
    # ---------------------------------------------
    model_paths = [
        os.path.join(pkg_ex_assignment1, "models"),
        os.path.join(pkg_sensors, "models"),
        aruco_path,  # <-- USE ABSOLUTE PATH
        os.environ.get("GAZEBO_MODEL_PATH", "")
    ]

    os.environ["GAZEBO_MODEL_PATH"] = ":".join(model_paths)

    # ---------------------------------------------
    # Extend PLUGIN PATH
    # ---------------------------------------------
    plugin_paths = [
        os.path.join(get_package_prefix("ex_assignment1"), "lib"),
        os.path.join(get_package_prefix("bme_gazebo_sensors"), "lib"),
        os.environ.get("GAZEBO_PLUGIN_PATH", "")
    ]

    os.environ["GAZEBO_PLUGIN_PATH"] = ":".join(plugin_paths)

    print("GAZEBO_MODEL_PATH =", os.environ["GAZEBO_MODEL_PATH"])
    print("GAZEBO_PLUGIN_PATH =", os.environ["GAZEBO_PLUGIN_PATH"])

    return LaunchDescription([])
