import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the name of config file of the current experiment from an environment variable
    specs_file = os.environ.get('PARAMS_FILE', None)

    if specs_file is None:
        raise RuntimeError("PARAMS_FILE environment variable is not set. Please provide a valid YAML file.")

    # Opening the config file to extract experiment parameters
    try:
        with open(specs_file, 'r', encoding='utf-8') as file:
            robot_specs = yaml.safe_load(file)
    except Exception as e:
        raise RuntimeError(f"Failed to read the YAML configuration file: {e}")

    # Extract controller and planner types
    controller_type = robot_specs.get('controller_type', [])
    planner_type = robot_specs.get('planner_type', [])
    instances_num = robot_specs.get('instances_num', 1)

    if not controller_type or not planner_type:
        raise RuntimeError("Controller or Planner types are missing in the YAML file.")

    # Extract trajectory type
    trajectory_type = robot_specs.get('trajectory_type', 'user_defined')
    trajectories = (robot_specs.get('user_defined_trajectories', []) 
                    if trajectory_type == 'user_defined' 
                    else robot_specs.get('auto_generated_trajectory', {}).get('types', []))

    if not trajectories:
        raise RuntimeError(f"No valid trajectories found for trajectory type: {trajectory_type}")

    # Declare launch arguments (instead of environment variables)
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('planner', default_value=planner_type[0], description="Planner type"))
    ld.add_action(DeclareLaunchArgument('controller', default_value=controller_type[0], description="Controller type"))
    ld.add_action(DeclareLaunchArgument('trajectory_num', default_value="0", description="Current trajectory index"))
    ld.add_action(DeclareLaunchArgument('round_num', default_value="1", description="Current round number"))
    ld.add_action(DeclareLaunchArgument('iteration_id', default_value="0", description="Iteration ID"))

    # Resolve package paths
    nav2bench_path = get_package_share_directory("nav2bench")

    # Include the robot spawning launch file
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2bench_path, 'launch', 'petra_urdf_v7.launch.py'))
    )
    ld.add_action(spawn_robot)

    # Include the navigation launch file
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2bench_path, 'launch', 'navigation.launch.py'))
    )
    ld.add_action(nav2)

    # Trajectory generator node
    trajectory_generator = Node(
        package='nav2bench',
        executable='trajectory_generator.py',
        name='trajectory_generator',
        output='screen',
        parameters=[{
            "planner_type": LaunchConfiguration('planner'),
            "controller_type": LaunchConfiguration('controller'),
        }]
    )
    ld.add_action(trajectory_generator)

    # PDF Generator node (final step)
    pdf_generator = Node(
        package='nav2bench',
        executable='pdf_generator.py',
        name='pdf_generator',
        output='screen'
    )

    # Create nodes for controller, goal following, and resetting the robot
    controller_nodes = []
    follow_path_nodes = []
    reset_nodes = []

    total_experiments = len(controller_type) * len(planner_type) * instances_num * len(trajectories)

    for index in range(total_experiments):
        controller_nodes.append(Node(
            package='nav2bench',
            executable='marker_publisher.py',
            name=f'marker_publisher_{index}',
            output='screen'
        ))

        follow_path_nodes.append(Node(
            package='nav2bench',
            executable='follow_path.py',
            name=f'follow_path_{index}',
            output='screen'
        ))

        reset_nodes.append(Node(
            package='nav2bench',
            executable='reset_robot.py',
            name=f'reset_robot_{index}',
            output='screen'
        ))

    # Event handling for process sequence
    for exp_index in range(total_experiments):
        if exp_index == 0:
            # First iteration: link trajectory generator -> first reset -> first goal execution
            ld.add_action(RegisterEventHandler(
                OnProcessExit(target_action=trajectory_generator, on_exit=reset_nodes[0])
            ))
            ld.add_action(RegisterEventHandler(
                OnProcessExit(target_action=reset_nodes[0], on_exit=follow_path_nodes[0])
            ))
            ld.add_action(RegisterEventHandler(
                OnProcessExit(target_action=follow_path_nodes[0], on_exit=controller_nodes[0])
            ))
        else:
            prev_index = exp_index - 1
            ld.add_action(RegisterEventHandler(
                OnProcessExit(target_action=follow_path_nodes[prev_index], on_exit=[
                    DeclareLaunchArgument('planner', default_value=planner_type[exp_index % len(planner_type)]),
                    DeclareLaunchArgument('controller', default_value=controller_type[exp_index % len(controller_type)]),
                    DeclareLaunchArgument('iteration_id', default_value=str(exp_index % instances_num)),
                    DeclareLaunchArgument('trajectory_num', default_value=str(exp_index % len(trajectories))),
                    DeclareLaunchArgument('round_num', default_value=str(exp_index + 1)),
                    reset_nodes[exp_index]
                ])
            ))
            ld.add_action(RegisterEventHandler(
                OnProcessExit(target_action=reset_nodes[exp_index], on_exit=[follow_path_nodes[exp_index], controller_nodes[exp_index]])
            ))

    # Final PDF generation after last experiment
    ld.add_action(RegisterEventHandler(
        OnProcessExit(target_action=follow_path_nodes[-1], on_exit=[pdf_generator])
    ))

    return ld
