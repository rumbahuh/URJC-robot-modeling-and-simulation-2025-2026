from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument
from controller_manager.launch_utils import generate_load_controller_launch_description

def generate_launch_description():
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
    )
    pkg_share_folder = get_package_share_directory('rover_kachau')

    joint_state_broadcaster = GroupAction([
        generate_load_controller_launch_description(
            controller_name='joint_state_broadcaster',
            controller_params_file=join(pkg_share_folder, 'config', 'rover_controllers.yaml'))
    ])

    base_controller = GroupAction([
        generate_load_controller_launch_description(
            controller_name='rover_base_control',
            controller_params_file=join(pkg_share_folder, 'config', 'rover_controllers.yaml'))
    ])

    scara_controller = GroupAction([
        generate_load_controller_launch_description(
            controller_name='scara_controller',
            controller_params_file=join(pkg_share_folder, 'config', 'rover_controllers.yaml'))
    ])

    gripper_controller = GroupAction([
        generate_load_controller_launch_description(
            controller_name='gripper_controller',
            controller_params_file=join(pkg_share_folder, 'config', 'rover_controllers.yaml'))
    ])

    ld = LaunchDescription()
    ld.add_action(declare_sim_time)
    ld.add_action(joint_state_broadcaster)
    ld.add_action(base_controller)
    ld.add_action(scara_controller)
    ld.add_action(gripper_controller)
    return ld