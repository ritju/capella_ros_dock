import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import TextSubstitution
from nav2_common.launch import RewrittenYaml

"""
用于获取环境变量值
参数
env: 环境变量名称
default: 环境变量为赋值时使用的默认值
返回值
返回最终采用的值
"""
def get_environment_value(env, default):
    try:
        if env in os.environ:
            value = os.environ.get(env, default)
            print(f'get {env} value: {value} from environment')
            return value
        else:
            print(f"Using default {env} value: {default}.")
            return default
    except Exception as e:
        print(f'exception: {str(e)}')
        print(f"Please input {env} in environment")
        return default


def generate_launch_description():
    
    launch_description = LaunchDescription()

    # get pkg path
    camera_pkg_path = get_package_share_directory('astra_camera')
    aruco_pkg_path = get_package_share_directory('aruco_ros')
    apriltag_pkg_path = get_package_share_directory('apriltag_ros')
    dock_pkg_path = get_package_share_directory('capella_ros_dock')
    usb_cam_pkg_path = get_package_share_directory('usb_cam')
    laserscan_3d_to_2d_path = get_package_share_directory('pointcloud_to_laserscan') 
    
    # 获取环境变量值
    dock_param_file_name = get_environment_value("DOCK_PARAM_FILE", "config.yaml")
    charger_contact_type = get_environment_value("CHARGER_CONTACT_CONDITION_TYPE", "BLUETOOTH_ONLY")
    last_docked_offset = get_environment_value("LAST_DOCKED_DISTANCE_OFFSET", "0.30")
    camera_baselink_distance = get_environment_value("CAMERA_BASELINK_DIS", "0.3")
    apriltag_double_log_level = get_environment_value("APRILTAG_DOUBLE_LOG_LEVEL", "info")
    motion_control_log_level = get_environment_value("MOTION_CONTROL_LOG_LEVEL", "info")
    
    # 类型映射
    type_mapping = {
        'BLUETOOTH_ONLY': 0,
        'CAMERA_ONLY': 1,
        'BLUETOOTH_AND_CAMERA': 2
    }
    
    # 构建参数文件路径
    params_file_path = PathJoinSubstitution([
        dock_pkg_path, 'params', dock_param_file_name
    ])
    
    # 参数替换配置 - 确保值为字符串类型
    param_substitutions = {
        "charger_contact_condition_type": str(type_mapping[charger_contact_type]),
        "offset_last_docked_distance": str(last_docked_offset),
        "camera_baselink_dis": str(camera_baselink_distance)
    }    
    
    # 配置参数文件
    configured_params = RewrittenYaml(
        source_file=params_file_path,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # 声明launch参数
    motion_control_log_level_arg = DeclareLaunchArgument(
        'motion_control_log_level', 
        default_value=motion_control_log_level, 
        description='define motion_control node log level'
    )
    apriltag_double_log_level_arg = DeclareLaunchArgument(
        'apriltag_double_log_level', 
        default_value=apriltag_double_log_level, 
        description='define motion_control node log level'
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
    
    # coord node
    coord_optimize_node = Node(
        executable='coord_optimize_node',
        package='capella_ros_dock',
        name='coord_optimize_node',
        namespace='',
        output='screen',
        parameters=[configured_params],
        # remappings=[('pose_with_id_optimize', 'pose_test')],
        
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
        PythonLaunchDescriptionSource(os.path.join(apriltag_pkg_path, 'launch', 'apriltag_ros_double.launch.py')),
        launch_arguments={
            "log_level": LaunchConfiguration("apriltag_double_log_level")
        }.items()
    )

    # 使用usb_cam包启动rgb相机
    usb_cam_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(usb_cam_pkg_path, 'launch', 'camera.launch.py'))
    )

    # 3d激光雷达转2d激光雷达
    laserscan_3d_to_2d = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(laserscan_3d_to_2d_path, 'launch', 'sample_pointcloud_to_laserscan_launch.py'))
    )

    # 对接充电桩主程序
    motion_control_node = Node(
        executable='motion_control',
        package='capella_ros_dock',
        name='motion_control',
        namespace='',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', ['motion_control:='], LaunchConfiguration("motion_control_log_level")]        
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
    launch_description.add_action(motion_control_log_level_arg)
    launch_description.add_action(apriltag_double_log_level_arg)
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
            launch_description.add_action(coord_optimize_node)
        else:
            print(f'The value of CHARGER_MARKER_TYPE is {marker_type}, just use default value ARUCO.')
            launch_description.add_action(aruco_launch_file)
    else:
        launch_description.add_action(aruco_launch_file) 

    launch_description.add_action(motion_control_node)
    launch_description.add_action(hazards_vector_publisher_node)
    launch_description.add_action(camera_point_cloud_process_node)
    # launch_description.add_action(usb_cam_launch_file)
    # launch_description.add_action(laserscan_3d_to_2d)

    # launch_description.add_action(test_docking_node)

    return launch_description
