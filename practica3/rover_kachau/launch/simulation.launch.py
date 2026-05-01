from os.path import join
from os import environ, pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def get_model_paths(packages_names):
    model_paths = ""
    for package_name in packages_names:
        if model_paths != "":
            model_paths += pathsep
        package_path = get_package_prefix(package_name)
        model_path = join(package_path, "share")
        model_paths += model_path
    if 'GZ_SIM_RESOURCE_PATH' in environ:
        model_paths += pathsep + environ['GZ_SIM_RESOURCE_PATH']
    return model_paths

def start_gzserver(context, *args, **kwargs):
    pkg_path = get_package_share_directory('rover_kachau')
    world = join(pkg_path, 'worlds', 'empty.sdf')
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s -v 4 ', world]}.items()
    )
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [' -g ']}.items(),
    )
    return [start_gazebo_server_cmd, start_gazebo_client_cmd]

def generate_launch_description():
    pkg_path = get_package_share_directory('rover_kachau')
    model_path = get_model_paths(['rover_kachau'])

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description="use_sim_time simulation parameter"
    )

    gazebo_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-model", "rover",
            "-topic", "robot_description",
            "-use_sim_time", "True",
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rover_kachau"), "rviz", "robot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_ros_gz',
        parameters=[{
            'config_file': join(pkg_path, 'config', 'rover_bridge.yaml'),
            'use_sim_time': True,
        }],
        output='screen',
    )

    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/front_cam/image"],
        output="screen",
        parameters=[{
            'use_sim_time': True,
            'camera.image.compressed.jpeg_quality': 75
        }],
    )

    twist_stamped = Node(
        package="twist_stamper",
        executable="twist_stamper",
        name="twist_stamper",
        output="screen",
        parameters=[{"use_sim_time": True}],
        remappings=[
            ('cmd_vel_out', '/rover_base_control/cmd_vel'),
            ('cmd_vel_in', '/cmd_vel')
        ],
    )

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path))
    ld.add_action(SetEnvironmentVariable('GZ_SIM_MODEL_PATH', model_path))
    ld.add_action(declare_sim_time)
    ld.add_action(bridge)
    ld.add_action(gz_image_bridge_node)
    ld.add_action(OpaqueFunction(function=start_gzserver))
    ld.add_action(rviz_node)
    ld.add_action(gazebo_spawn_robot)
    ld.add_action(twist_stamped)
    return ld