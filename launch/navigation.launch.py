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
SPEC_FILE = os.environ.get("PARAMS_FILE")
if not SPEC_FILE:
    raise RuntimeError("Environment variable 'PARAMS_FILE' is not set. Please set it to the path of the YAML file.")

try:
    with open(SPEC_FILE, 'r') as f:
        robot_specs = yaml.safe_load(f)
except FileNotFoundError:
    raise RuntimeError(f"Specified YAML file '{SPEC_FILE}' not found.")
except yaml.YAMLError as e:
    raise RuntimeError(f"Error parsing YAML file '{SPEC_FILE}': {e}")

required_keys = ['map_path', 'nav_config', 'localization_params_path', 'trajectory_type']
for key in required_keys:
    if key not in robot_specs:
        raise RuntimeError(f"Missing required key '{key}' in YAML file '{SPEC_FILE}'.")

def generate_launch_description():

    # ------------------------------------------------------------------
    # (A) Read the YAML file to extract map_path, nav_config, spawn pose
    # ------------------------------------------------------------------
    # region

    map_path   = robot_specs['map_path']
    nav_config = robot_specs['nav_config']
    localization_params_path = robot_specs['localization_params_path']
   
    # Decide spawn pose from the trajectory type
    if robot_specs['trajectory_type'] == 'user_defined':
        x = robot_specs['user_defined_trajectories'][0]["spawn_pose"]["x"]
        y = robot_specs['user_defined_trajectories'][0]["spawn_pose"]["y"]
        yaw = robot_specs['user_defined_trajectories'][0]["spawn_pose"]["yaw"]
    elif robot_specs['trajectory_type'] == 'auto_generated':
        x = robot_specs['auto_generated_trajectory']["spawn_pose"]["x"]
        y = robot_specs['auto_generated_trajectory']["spawn_pose"]["y"]
        yaw = robot_specs['auto_generated_trajectory']["spawn_pose"]["yaw"]
    else:
        raise RuntimeError(f"Unknown trajectory_type: {robot_specs['trajectory_type']}")

    print(f"Using SPEC_FILE: {SPEC_FILE}")
    print(f"Map path: {map_path}")
    print(f"Navigation config: {nav_config}")
    print(f"Localization params path: {localization_params_path}")
    # endregion

    # ------------------------------------------------------------------
    # (B) Keep your existing logic for GZ_SIM_RESOURCE_PATH, etc.
    # ------------------------------------------------------------------
    # region
    
    pkg_petra_robot = get_package_share_directory('nav2bench')

    # If you need to set GZ_SIM_RESOURCE_PATH
    gazebo_models_path, _ = os.path.split(pkg_petra_robot)
    if os.path.exists(gazebo_models_path):
        if "GZ_SIM_RESOURCE_PATH" in os.environ:
            os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path
        else:
            os.environ["GZ_SIM_RESOURCE_PATH"] = gazebo_models_path
    else:
        raise RuntimeError(f"Gazebo models path '{gazebo_models_path}' does not exist.")
    # endregion

    # ------------------------------------------------------------------
    # (C) Declare your launch arguments (RViz on/off, config, sim_time)
    # ------------------------------------------------------------------
    # region

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
    # endregion
 
    # ------------------------------------------------------------------
    # (D) If you still want a separate AMCL / Navigation config,
    #     you can keep them. Or you can override with 'nav_config' from YAML
    # ------------------------------------------------------------------
    # region

    # Example: use the user-specified nav_config for *navigation*,
    # but keep a separate AMCL config for *localization*, or
    # unify them if you prefer. Adjust as needed.
    # localization_params_path = os.path.join(
    #   pkg_petra_robot,
    #   'config',
    #   'amcl_localization.yaml'
    # )
    # We'll use `nav_config` from the YAML for navigation:
    # navigation_params_path = nav_config  # path from your YAML
    # localization_params_path = localization_params_path

    # If you want to unify everything from the YAML, just do:
    # localization_params_path = nav_config
    navigation_params_path   = nav_config

    # We can also override the map with the YAML-provided path:
    map_file_path = map_path  # from your YAML
    # endregion

    # ------------------------------------------------------------------
    # (E) Paths for localization_launch.py and navigation_launch.py
    # ------------------------------------------------------------------
    # region

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_localization_launch_path = os.path.join(
        nav2_bringup_dir, 'launch', 'localization_launch.py'
    )
    nav2_navigation_launch_path = os.path.join(
        nav2_bringup_dir, 'launch', 'navigation_launch.py'
    )
    # endregion

    # ------------------------------------------------------------------
    # (F) Example: Keep your interactive marker config, if needed
    # ------------------------------------------------------------------
    # region

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
    # endregion

    # ------------------------------------------------------------------
    # (G) Launch RViz with a user-selectable config
    # ------------------------------------------------------------------
    # region

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
    # endregion

    # ------------------------------------------------------------------
    # (H) Include the nav2 localization
    # ------------------------------------------------------------------
    # region

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file':  localization_params_path,
            'map':          map_file_path
        }.items()
    )
    # endregion

    # ------------------------------------------------------------------
    # (I) Include the nav2 navigation
    # ------------------------------------------------------------------
    # region

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file':  navigation_params_path,
        }.items()
    )
    # endregion

    # ------------------------------------------------------------------
    # (J) Build and return your final LaunchDescription
    # ------------------------------------------------------------------
    # region

    ld = LaunchDescription()

    # 1. Register your launch args
    ld.add_action(rviz_launch_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(sim_time_arg)

    # 2. Add your nodes (uncomment interactive marker if needed)
    ld.add_action(rviz_node)
    ld.add_action(interactive_marker_twist_server_node)

    # 3. Add your nav2 includes
    ld.add_action(localization_launch)
    ld.add_action(navigation_launch)

    return ld
    # endregion
