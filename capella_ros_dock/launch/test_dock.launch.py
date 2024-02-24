import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    
    launch_description = LaunchDescription()

    # get pkg path
    camera_pkg_path = get_package_share_directory('astra_camera')
    aruco_pkg_path = get_package_share_directory('aruco_ros')
    dock_pkg_path = get_package_share_directory('capella_ros_dock')

    # create launch configuration variables
    params_file_path = LaunchConfiguration('params_files', default=os.path.join(dock_pkg_path, 'params', 'config.yaml'))
    motion_control_log_level = LaunchConfiguration('motion_control_log_level')
    test_count = LaunchConfiguration('test_count', default = 1)
    
    # declare launch arguments   
    # test_count_launch_arg = DeclareLaunchArgument('test_count', default_value=TextSubstitution(text="1"))
    log_level_arg = DeclareLaunchArgument('log_level', default_value='info', description='define motion_control node log level')
    params_file_arg = DeclareLaunchArgument('params_files', default_value=params_file_path)
    
    # configured params file
    param_substitutions = {
        "test_count": test_count,
    }

    configured_params = RewrittenYaml(
        source_file=params_file_path,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # serial Node
    serial_node = Node(
        executable='serial_port_node',
        package='capella_ros_serial',
        name='serial_node',
        namespace=''
    )

    # wifi Node
    wifi_node = Node(
        executable='charge_server_node',
        package='capella_charge_service',
        name='wifi_server',
        respawn=True
    )
    # blutooth Node
    bluetooth_node = Node(
        executable='charge_server_bluetooth',
        package='capella_charge_service',
        name='bluetooth_server',
        respawn=True
    )

    # manual dock node
    manual_dock_node = Node(
        executable='manual_dock',
        package='capella_ros_dock',
        name='manual_dock',
        namespace='',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', ['motion_control:=', LaunchConfiguration('log_level')]]
    )

    # camera(orbbec dabai_dcw) launch file
    camera_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(camera_pkg_path, 'launch', 'dabai_dcw.launch.py'))
    )

    # aruco launch file
    aruco_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(aruco_pkg_path, 'launch', 'single.launch.py'))
    )

    # motion_control Node
    motion_control_node = Node(
        executable='motion_control',
        package='capella_ros_dock',
        name='motion_control',
        namespace='',
        output='screen',
        parameters=[configured_params],
    )

    # hazards_vector_publisher Node
    hazards_vector_publisher_node = Node(
        executable='hazards_vector_publisher',
        package='capella_ros_dock',
        name='hazards_vector_publisher',
        namespace='',
        output='screen',
        parameters=[configured_params],
    )

    # camera_point_cloud_process Node
    camera_point_cloud_process_node = Node(
        executable='camera_point_cloud_process',
        package='capella_ros_dock',
        name='camera_point_cloud_process',
        namespace='',
        output='screen',
        parameters=[configured_params],
    )
    

    # test docking Node
    test_docking_node = Node(
        executable='test_dock',
        package='capella_ros_dock',
        name='test_dock',
        parameters=[configured_params],
    )

    # charge_manager_pkg nodes
    # charge_manager node
    charge_manager_node = Node(
        executable='charge_manage',
        package='charge_manager',
        name='charge_manager_node',
        respawn=True
    )

    # charge_action node
    charge_action_node = Node(
        executable='charge_action',
        package='charge_manager',
        name='charge_action_node',
        respawn=True
    )
    # bluetooth_old node
    bluetooth_old_node = Node(
        executable='charge_bluetooth_old',
        package='charge_manager',
        name='bluetooth_old_node',
    )

    # launch_description.add_action(test_count_launch_arg)
    launch_description.add_action(log_level_arg)
    launch_description.add_action(params_file_arg)
    # launch_description.add_action(serial_node)

    # launch_description.add_action(wifi_node)
    # launch_description.add_action(bluetooth_node)
    # launch_description.add_action(bluetooth_old_node)
    launch_description.add_action(charge_manager_node)
    launch_description.add_action(charge_action_node)

    launch_description.add_action(manual_dock_node)
    # launch_description.add_action(camera_launch_file)
    launch_description.add_action(aruco_launch_file)
    launch_description.add_action(motion_control_node)
    launch_description.add_action(hazards_vector_publisher_node)
    launch_description.add_action(camera_point_cloud_process_node)
    # launch_description.add_action(test_docking_node)

    return launch_description
