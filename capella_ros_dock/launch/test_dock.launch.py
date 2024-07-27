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
    apriltag_pkg_path = get_package_share_directory('apriltag_ros')
    dock_pkg_path = get_package_share_directory('capella_ros_dock')

    # create launch configuration variables
    params_file_path = LaunchConfiguration('params_files', default=os.path.join(dock_pkg_path, 'params', 'config.yaml'))
    motion_control_log_level = LaunchConfiguration('motion_control_log_level')
    test_count = LaunchConfiguration('test_count', default = 1)
    
    charger_contact_condition_type = 0
    type_list = {
        'BLUETOOTH_ONLY': 0,
        'CAMERA_ONLY': 1,
        'BLUETOOTH_ADN_CAMERA': 2
    }
    
    try:
        if 'CHARGER_CONTACT_CONDITION_TYPE' in os.environ:
            charger_contact_condition_type_name = os.environ.get('CHARGER_CONTACT_CONDITION_TYPE')
            print(f'get charger_contact_condition_type {charger_contact_condition_type_name} from docker-compose.yaml file')
            charger_contact_condition_type = type_list[charger_contact_condition_type_name]
        else:
            charger_contact_condition_type = 0
            print("Using default charger_contact_condition_type 0.")
    except Exception as e:
        print(f'exception: {str(e)}')
        print("Please input CHARGER_CONTACT_CONDITION_TYPE in docker-compose.yaml")
        charger_contact_condition_type = 0

    last_docked_distance_offset_ = 0.60
    try:
        if 'LAST_DOCKED_DISTANCE_OFFSET' in os.environ:
            last_docked_distance_offset_ = float(os.environ.get('LAST_DOCKED_DISTANCE_OFFSET'))
            print(f'get last_docked_distance_offset_ from docker-compose.yaml file')
        else:
            last_docked_distance_offset_ = 0.60
            print("Using default last_docked_distance_offset_ 0.60")
    except Exception as e:
        print(f'exception: {str(e)}')
        print("Please input LAST_DOCKED_DISTANCE_OFFSET in docker-compose.yaml")
        last_docked_distance_offset_ = 0.60

    charging_radius = 0.8
    try:
        if 'CHARGING_RADIUS' in os.environ:
            charging_radius = float(os.environ.get('CHARGING_RADIUS'))
            print(f'get charging radius {charging_radius} from docker-compose.yml')
        else:
            charging_radius = 0.8
    except Exception as e:
        print(f'exception: {str(e)}')
        print("Please modify CHARGING_RADIUS's value in docker-compose.yml")
        charging_radius = 0.8

    
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
        
    )

    # camera(orbbec dabai_dcw) launch file
    camera_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(camera_pkg_path, 'launch', 'dabai_dcw.launch.py'))
    )

    # aruco launch file
    aruco_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(aruco_pkg_path, 'launch', 'single.launch.py'))
    )

    # apriltag launch file
    apriltag_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(apriltag_pkg_path, 'launch', 'apriltag_ros.launch.py'))
    )

    # apriltag double launch file
    apriltag_double_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(apriltag_pkg_path, 'launch', 'apriltag_ros_double.launch.py'))
    )


    # motion_control Node
    motion_control_node = Node(
        executable='motion_control',
        package='capella_ros_dock',
        name='motion_control',
        namespace='',
        output='screen',
        parameters=[configured_params, {'charger_contact_condition_type': charger_contact_condition_type, 'charging_radius': charging_radius, 'last_docked_distance_offset_': last_docked_distance_offset_}],
        arguments=['--ros-args', '--log-level', ['motion_control:=', LaunchConfiguration('log_level')]]
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
        name='charge_bluetooth_server_node',
        respawn=True
    )

    # launch_description.add_action(test_count_launch_arg)
    launch_description.add_action(log_level_arg)
    launch_description.add_action(params_file_arg)
    # launch_description.add_action(serial_node)

    # launch_description.add_action(wifi_node)
    # launch_description.add_action(bluetooth_node)
    launch_description.add_action(bluetooth_old_node)
    launch_description.add_action(charge_manager_node)
    launch_description.add_action(charge_action_node)

    launch_description.add_action(manual_dock_node)
    # launch_description.add_action(camera_launch_file)

    # choose marker type: aruco marker or apriltag marker
    if 'CHARGER_MARKER_TYPE' in os.environ:
        marker_type = os.environ.get('CHARGER_MARKER_TYPE')
        if marker_type.upper() == "ARUCO":
            launch_description.add_action(aruco_launch_file)
        elif marker_type.upper() == "APRILTAG":
            launch_description.add_action(apriltag_launch_file)
        elif marker_type.upper() == "APRILTAG_DOUBLE":
            launch_description.add_action(apriltag_double_launch_file)
        else:
            print(f'The value of CHARGER_MARKER_TYPE is {marker_type}, just use default value ARUCO.')
            launch_description.add_action(aruco_launch_file)
    else:
        launch_description.add_action(aruco_launch_file) 

    launch_description.add_action(motion_control_node)
    launch_description.add_action(hazards_vector_publisher_node)
    launch_description.add_action(camera_point_cloud_process_node)
    # launch_description.add_action(test_docking_node)

    return launch_description
