import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- Paths ---
    pkg_robotaksi_controller = get_package_share_directory('robotaksi_controller')
    pkg_robotaksi_description = get_package_share_directory('robotaksi_description')
    #pkg_robotaksi_mapping = get_package_share_directory('robotaksi_mapping')

    # --- Launch Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    #run_slam = LaunchConfiguration('run_slam', default='true')

    # --- Simulation ---
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robotaksi_description, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # --- Controller ---
    # Twist filter is not used with the new controller setup for now
    # twist_filter_node = Node(...)

    # --- Controller Spawners ---
    controller_spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['position_controller', '--controller-manager', '/controller_manager'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['velocity_controller', '--controller-manager', '/controller_manager'],
            parameters=[{'use_sim_time': use_sim_time}],
        )
    ]

    # --- Localization (EKF) ---


 

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'))
    #ld.add_action(DeclareLaunchArgument('run_slam', default_value='true', description='Whether to start the SLAM toolbox'))
    
    ld.add_action(simulation_launch)
    #ld.add_action(robot_localization_node)
    
    
    for spawner in controller_spawners:
        ld.add_action(spawner)

    return ld 
