import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Read from YAML via environment variable PARAMS_FILE
SPEC_FILE = os.environ["PARAMS_FILE"]

def generate_launch_description():

    # ------------------------------------------------------------------
    # (A) Read the YAML file to extract map_path, nav_config, spawn pose
    # ------------------------------------------------------------------
    with open(SPEC_FILE, 'r') as f:
        robot_specs = yaml.safe_load(f)

    map_path   = robot_specs['map_path']
    nav_config = robot_specs['nav_config']
    localization_params_path = robot_specs['localization_params_path']
   
    # Decide spawn pose from the trajectory type
    if robot_specs['trajectory_type'] == 'user_defined':
        x   = robot_specs['user_defined_trajectories'][0]["spawn_pose"]["x"]
        y   = robot_specs['user_defined_trajectories'][0]["spawn_pose"]["y"]
        yaw = robot_specs['user_defined_trajectories'][0]["spawn_pose"]["yaw"]
    # else:  # e.g. 'auto_generated'
        x   = robot_specs['auto_generated_trajectory']["spawn_pose"]["x"]
        y   = robot_specs['auto_generated_trajectory']["spawn_pose"]["y"]
        yaw = robot_specs['auto_generated_trajectory']["spawn_pose"]["yaw"]


    # ------------------------------------------------------------------
    # (B) Keep your existing logic for GZ_SIM_RESOURCE_PATH, etc.
    # ------------------------------------------------------------------
    pkg_petra_robot = get_package_share_directory('nav2bench')

    # If you need to set GZ_SIM_RESOURCE_PATH
    gazebo_models_path, _ = os.path.split(pkg_petra_robot)
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path
    else:
        os.environ["GZ_SIM_RESOURCE_PATH"] = gazebo_models_path

    # ------------------------------------------------------------------
    # (C) Declare your launch arguments (RViz on/off, config, sim_time)
    # ------------------------------------------------------------------
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='navigation.rviz',
        description='RViz config file'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    # ------------------------------------------------------------------
    # (D) If you still want a separate AMCL / Navigation config,
    #     you can keep them. Or you can override with 'nav_config' from YAML
    # ------------------------------------------------------------------
    # Example: use the user-specified nav_config for *navigation*,
    # but keep a separate AMCL config for *localization*, or
    # unify them if you prefer. Adjust as needed.
    # localization_params_path = os.path.join(
    #     pkg_petra_robot,
    #     'config',
    #     'amcl_localization.yaml'
    # )
    # We'll use `nav_config` from the YAML for navigation:
    navigation_params_path = nav_config  # path from your YAML
    localization_params_path = localization_params_path

    # If you want to unify everything from the YAML, just do:
    # localization_params_path = nav_config
    # navigation_params_path   = nav_config

    # We can also override the map with the YAML-provided path:
    map_file_path = map_path  # from your YAML

    # ------------------------------------------------------------------
    # (E) Paths for localization_launch.py and navigation_launch.py
    # ------------------------------------------------------------------
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_localization_launch_path = os.path.join(
        nav2_bringup_dir, 'launch', 'localization_launch.py'
    )
    nav2_navigation_launch_path = os.path.join(
        nav2_bringup_dir, 'launch', 'navigation_launch.py'
    )

    # ------------------------------------------------------------------
    # (F) Example: Keep your interactive marker config, if needed
    # ------------------------------------------------------------------
    interactive_marker_config_file_path = os.path.join(
        pkg_petra_robot,
        'config',
        'linear.yaml'
    )
    interactive_marker_twist_server_node = Node(
        package='interactive_marker_twist_server',
        executable='marker_server',
        name='twist_server_node',
        parameters=[interactive_marker_config_file_path],
        output='screen',
    )

    # ------------------------------------------------------------------
    # (G) Launch RViz with a user-selectable config
    # ------------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            PathJoinSubstitution([
                pkg_petra_robot,
                'rviz',
                LaunchConfiguration('rviz_config')
            ])
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # ------------------------------------------------------------------
    # (H) Include the nav2 localization
    # ------------------------------------------------------------------
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file':  localization_params_path,
            'map':          map_file_path
        }.items()
    )

    # ------------------------------------------------------------------
    # (I) Include the nav2 navigation
    # ------------------------------------------------------------------
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file':  navigation_params_path,
        }.items()
    )

    # ------------------------------------------------------------------
    # (J) Build and return your final LaunchDescription
    # ------------------------------------------------------------------
    ld = LaunchDescription()

    # 1. Register your launch args
    ld.add_action(rviz_launch_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(sim_time_arg)

    # 2. Add your nodes (uncomment interactive marker if needed)
    ld.add_action(rviz_node)
    # ld.add_action(interactive_marker_twist_server_node)

    # 3. Add your nav2 includes
    ld.add_action(localization_launch)
    ld.add_action(navigation_launch)

    return ld
