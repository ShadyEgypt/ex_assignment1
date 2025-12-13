from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    FrontendLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_prefix
import os


def generate_launch_description():

    # --------------------------------------------------
    # Launch arguments
    # --------------------------------------------------
    world = LaunchConfiguration(
        'world',
        default=os.path.join(
            FindPackageShare('erl1').find('erl1'),
            'worlds',
            'my_world.sdf'
        )
    )

    rviz_config = LaunchConfiguration(
        'rviz_config',
        default='assignment.rviz'
    )

    # --------------------------------------------------
    # Gazebo Sim resource path (CRITICAL FIX)
    # --------------------------------------------------
    aruco_models_path = os.path.join(
        get_package_prefix('aruco_marker_gazebo'),
        'share',
        'aruco_marker_gazebo',
        'models'
    )

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=aruco_models_path
    )

    # --------------------------------------------------
    # Gazebo environment
    # --------------------------------------------------
    gazebo_env = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('ex_assignment1').find('ex_assignment1'),
                'launch',
                'gazebo_env.launch.py'
            )
        )
    )

    # --------------------------------------------------
    # Spawn robot
    # --------------------------------------------------
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('bme_gazebo_sensors').find('bme_gazebo_sensors'),
                'launch',
                'spawn_robot.launch.py'
            )
        ),
        launch_arguments={
            'rviz_config': rviz_config,
            'rviz': 'true',
        }.items()
    )

    # --------------------------------------------------
    # ArUco tracker
    # --------------------------------------------------
    aruco_tracker = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('aruco_opencv').find('aruco_opencv'),
                'launch',
                'aruco_tracker.launch.xml'
            )
        )
    )


    # --------------------------------------------------
    # Image modifier node
    # --------------------------------------------------
    image_modifier = Node(
        package='ex_assignment1',
        executable='image_modifier',
        name='image_modifier',
        output='screen'
    )

    # --------------------------------------------------
    # Action server node
    # --------------------------------------------------
    action_server = Node(
        package='ex_assignment1',
        executable='action_server',
        name='servo_marker_server',
        output='screen'
    )

    # --------------------------------------------------
    # Launch description
    # --------------------------------------------------
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world),
        DeclareLaunchArgument('rviz_config', default_value=rviz_config),

        # ENV VAR MUST COME FIRST
        set_gz_resource_path,

        gazebo_env,
        spawn_robot,
        aruco_tracker,
        image_modifier,
        action_server,
    ])
