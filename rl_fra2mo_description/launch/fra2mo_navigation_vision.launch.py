from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Common configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map_file")
    amcl_params_file = LaunchConfiguration("amcl_params_file")
    nav2_params_file = LaunchConfiguration("params_file")

    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation/Gazebo clock"
    )

    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value="/home/user/ros2_ws/src/rl_fra2mo_description/maps/mappa_mondo_1.yaml",
        description="Full path to the yaml map file",
    )

    amcl_params_file_arg = DeclareLaunchArgument(
        "amcl_params_file",
        default_value="/home/user/ros2_ws/src/rl_fra2mo_description/config/amcl.yaml",
        description="Full path to the ROS2 parameters file to use for the amcl node",
    )

    nav2_params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value="/home/user/ros2_ws/src/rl_fra2mo_description/config/navigation.yaml",
        description="Full path to the ROS2 parameters file for Nav2",
    )

    # Map server node (no delay)
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_file}],
    )

    # Delay for amcl_node
    amcl_node = TimerAction(
        period=3.0,  # Delay of 3 seconds
        actions=[
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                parameters=[amcl_params_file, {"use_sim_time": use_sim_time}],
            )
        ],
    )

    # Delay for nav2_bringup_launch
    nav2_bringup_launch_file = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]
    )
    nav2_bringup_launch = TimerAction(
        period=5.0,  # Delay of 5 seconds
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_bringup_launch_file),
                launch_arguments={
                    "map": map_file,
                    "params_file": nav2_params_file,
                    "use_sim_time": use_sim_time,
                }.items(),
            )
        ],
    )

    # Lifecycle manager (no delay needed)
    nav_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="nav_manager",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["map_server", "amcl"]},
        ],
    )

    # Nodo ArUco
    aruco_single = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_detect',
        parameters=[
            {
                'image_is_rectified': True,
                'marker_id': 115,
                'marker_size': 0.2,
                'reference_frame': 'camera_link',
                'camera_frame': 'camera_link',
                'marker_frame': 'aruco_marker_frame',
            }
        ],
        remappings=[
            ('/image', '/videocamera'),
            ('/camera_info', '/videocamera_info'),
        ],
        output='screen',
    )

    # Rviz con ritardo di 10 secondi
    rviz_node = TimerAction(
        period=10.0,  # Delay of 10 seconds
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', '/home/user/ros2_ws/src/rl_fra2mo_description/rviz_conf/navigation_2.rviz'],
                output='screen',
            )
        ],
    )

    # Combine everything into a single LaunchDescription
    return LaunchDescription(
        [
            use_sim_time_arg,
            map_file_arg,
            amcl_params_file_arg,
            nav2_params_file_arg,
            map_server_node,  # Map Server Node
            amcl_node,        # AMCL Node with delay
            nav2_bringup_launch,  # Nav2 Bringup with delay
            nav_manager,      # Lifecycle Manager Node
            aruco_single,     # ArUco Node
            rviz_node,        # RViz Node with delay
        ]
    )

