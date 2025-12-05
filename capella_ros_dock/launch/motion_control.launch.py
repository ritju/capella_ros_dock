import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    # 获取包路径
    dock_pkg_path = get_package_share_directory("capella_ros_dock")
    
    # 获取环境变量值
    dock_param_file_name = get_environment_value("DOCK_PARAM_FILE", "config.yaml")
    charger_contact_type = get_environment_value("CHARGER_CONTACT_CONDITION_TYPE", "BLUETOOTH_ONLY")
    last_docked_offset = get_environment_value("LAST_DOCKED_DISTANCE_OFFSET", "0.30")
    camera_baselink_distance = get_environment_value("CAMERA_BASELINK_DIS", "0.3")
    motion_control_log_level = get_environment_value("DOCK_MOTION_CONTROL_LOG_LEVEL", "info")
    goal_y_correction = get_environment_value("DOCK_GOAL_Y_CORRECTION", "0.0")
    
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
    
    # 声明launch参数
    log_level_arg = DeclareLaunchArgument(
        'log_level', 
        default_value=motion_control_log_level, 
        description='define motion_control node log level'
    )
    
    # 参数替换配置 - 确保值为字符串类型
    param_substitutions = {
        "charger_contact_condition_type": str(type_mapping[charger_contact_type]),
        "offset_last_docked_distance": str(last_docked_offset),
        "camera_baselink_dis": str(camera_baselink_distance),
        "goal_y_correction": str(goal_y_correction)
    }    
    
    # 配置参数文件
    configured_params = RewrittenYaml(
        source_file=params_file_path,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    print("motion_control参数配置完成")
    
    # motion_control节点
    motion_control_node = Node(
        executable='motion_control',
        package='capella_ros_dock',
        name='motion_control',
        namespace='',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', ['motion_control:=', LaunchConfiguration('log_level')]],
    )

    launch_description = LaunchDescription()
    launch_description.add_action(log_level_arg)
    launch_description.add_action(motion_control_node)

    return launch_description
