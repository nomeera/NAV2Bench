import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET

    # -------------------------------------------------------------------------
    # 1modify_sdf_file to fix ball joint error appear in rviz
    # -------------------------------------------------------------------------
def modify_sdf_file(sdf_file):
    # Parse the SDF XML
    tree = ET.parse(sdf_file)
    root = tree.getroot()

    # Find all joints in the SDF
    for joint in root.findall(".//joint"):
        joint_type = joint.get("type")
        if joint_type == "ball":
            print(f"Changing joint '{joint.get('name')}' from 'ball' to 'fixed'")
            joint.set("type", "fixed")

    # Write back to a temporary file
    modified_sdf_file = sdf_file.replace(".sdf", "_modified.sdf")
    tree.write(modified_sdf_file)
    return modified_sdf_file

def generate_launch_description():
    # -------------------------------------------------------------------------
    # 1. Read YAML file from environment variable PARAMS_FILE
    # -------------------------------------------------------------------------
    # region
    specs_file = os.environ["PARAMS_FILE"]
    with open(specs_file, "r") as file:
        robot_specs = yaml.safe_load(file)

    # Extract fields from YAML
    world_path = robot_specs["world_path"]
    models_path = robot_specs["models_path"]
    # urdf_file   = robot_specs['urdf_file']
    model_file = robot_specs["model_file"]

    # Decide spawn pose from the trajectory type
    if robot_specs["trajectory_type"] == "user_defined":
        x = robot_specs["user_defined_trajectories"][0]["spawn_pose"]["x"]
        y = robot_specs["user_defined_trajectories"][0]["spawn_pose"]["y"]
        yaw = robot_specs["user_defined_trajectories"][0]["spawn_pose"]["yaw"]
        # else:  # e.g. 'auto_generated'
        x = robot_specs["auto_generated_trajectory"]["spawn_pose"]["x"]
        y = robot_specs["auto_generated_trajectory"]["spawn_pose"]["y"]
        yaw = robot_specs["auto_generated_trajectory"]["spawn_pose"]["yaw"]
    # endregion

    # -------------------------------------------------------------------------
    # 2. Update GZ_SIM_RESOURCE_PATH so Harmonic finds your models
    # -------------------------------------------------------------------------
    # region
    if models_path and os.path.exists(models_path):
        if "GZ_SIM_RESOURCE_PATH" in os.environ:
            os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + models_path
        else:
            os.environ["GZ_SIM_RESOURCE_PATH"] = models_path
    # endregion

    # -------------------------------------------------------------------------
    # 3. Create Launch Arguments for world + URDF and include world.launch.py
    # -------------------------------------------------------------------------
    # region
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=world_path,
        description="Path to the .sdf world file to load in Gazebo Garden",
    )

    """
    model_arg = DeclareLaunchArgument(
        'model_urdf',
        default_value=urdf_file,
        description='Path to the robot URDF or Xacro file'
    )
    """
    # Reads the content of the SDF file into a string for use in other components.

    # setup paths
    pkg_petra_robot = get_package_share_directory("nav2bench")

    # load sdf file
    sdf_file = os.path.join(pkg_petra_robot, "models", "nomeer_robot", "robot.sdf")
    modified_sdf_file = modify_sdf_file(sdf_file)
    with open(modified_sdf_file, "r") as infp:
        robot_desc = infp.read()

    # This includes your existing "world.launch.py",
    # which presumably starts gz simulator with the specified world.
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_petra_robot, "launch", "world.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
        }.items(),
    )
    # endregion

    # -------------------------------------------------------------------------
    # 4. robot state publisher & joint state publisher - uses the URDF (Xacro) from your YAML
    # -------------------------------------------------------------------------
    # region

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_desc},
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    # joint state publisher to publish joint state
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
        ],
    )
    # endregion

    # -------------------------------------------------------------------------
    # 5. Spawn the robot via ros_gz_sim create node
    # -------------------------------------------------------------------------
    # region
    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "nomeer_robot",
            "-file",
            sdf_file,
            "-x",
            str(x),
            "-y",
            str(y),
            "-z",
            "0.01",
            "-Y",
            str(yaw),
        ],
        parameters=[{"use_sim_time": True}],
    )
    # endregion

    # -------------------------------------------------------------------------
    # 6. Set base_footprint as a frame (not a link) and publish it via /tf
    # -------------------------------------------------------------------------
    # region
    static_tf_pub = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0.0", "0", "0", "0", "base_footprint", "base_link"],
    )
    # endregion

    # -------------------------------------------------------------------------
    # 7. Bridge, image bridge, etc. (from your new code)
    # -------------------------------------------------------------------------
    # region

    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "qos_overrides./tf_static.publisher.durability": "transient_local",  # Overrides the QoS (Quality of Service) settings for the /tf_static topic to make it durable (e.g., for transformations).
            }
        ],
    )

    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image"],
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"camera.image.compressed.jpeg_quality": 75},
        ],
    )

    relay_camera_info_node = Node(
        package="topic_tools",
        executable="relay",
        name="relay_camera_info",
        output="screen",
        arguments=["camera/camera_info", "camera/image/camera_info"],
        parameters=[{"use_sim_time": True}],
    )

    """'
    trajectory_node = Node(
        package='mogi_trajectory_server',
        executable='mogi_trajectory_server',
        name='mogi_trajectory_server',
        parameters=[{'reference_frame_id': 'odom'}]
    )
    """
    # endregion

    # -------------------------------------------------------------------------
    # 8. Assemble the final LaunchDescription
    # -------------------------------------------------------------------------
    # region

    ld = LaunchDescription()
    ld.add_action(world_arg)
    # ld.add_action(model_arg)

    ld.add_action(world_launch)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)

    ld.add_action(spawn_robot_node)
    ld.add_action(static_tf_pub)

    ld.add_action(gz_bridge_node)
    ld.add_action(gz_image_bridge_node)
    ld.add_action(relay_camera_info_node)
    # ld.add_action(trajectory_node)

    return ld
    # endregion
