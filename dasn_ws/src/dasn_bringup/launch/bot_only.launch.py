"""
Bot-only launch for DASN.

Launches robot base nodes: serial bridge, odometry, navigation,
obstacle avoidance, commander, stall detector, TF tree, and
robot state publisher.
"""

import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _load_yaml(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def _load_bot_node_params(bot_params_path, map_path, zones_path):
    data = _load_yaml(bot_params_path)
    params = data.get('/**', {}).get('ros__parameters', {})

    serial = params.get('serial', {})
    navigation = params.get('navigation', {})
    obstacle = params.get('obstacle_avoidance', {})
    stall = params.get('stall', {})
    battery = params.get('battery', {})
    dock = params.get('dock', {})

    return {
        'serial_bridge': {
            'serial_port': serial.get('port', '/dev/ttyUSB1'),
            'baud_rate': int(serial.get('baud_rate', 115200)),
        },
        'navigator': {
            'planner_type': navigation.get('planner_type', 'astar'),
            'map_file': map_path,
            'zone_config': zones_path,
            'lookahead_distance': float(navigation.get('lookahead_distance', 0.15)),
            'max_linear_speed': float(navigation.get('max_linear_velocity', 0.3)),
            'max_angular_speed': float(navigation.get('max_angular_velocity', 1.5)),
            'goal_tolerance': float(navigation.get('goal_tolerance', 0.1)),
        },
        'obstacle_avoidance': {
            'min_distance_mm': int(obstacle.get('min_distance', 200)),
            'slow_distance_mm': int(obstacle.get('slow_distance', 300)),
        },
        'commander': {
            'dock_position_x': float(dock.get('position_x', 0.5)),
            'dock_position_y': float(dock.get('position_y', 0.5)),
            'goal_tolerance': float(navigation.get('goal_tolerance', 0.1)),
            'low_battery_voltage': float(battery.get('low_voltage', 10.0)),
            'critical_battery_voltage': float(battery.get('critical_voltage', 9.5)),
        },
        'stall_detector': {
            'stall_encoder_timeout_s': float(stall.get('encoder_timeout_s', 0.5)),
            'stall_min_pwm': int(stall.get('min_pwm', 30)),
        },
    }


def generate_launch_description():

    bringup_share = get_package_share_directory('dasn_bringup')
    description_share = get_package_share_directory('dasn_description')

    bot_params_config = os.path.join(bringup_share, 'config', 'bot_params.yaml')
    env_map_config = os.path.join(bringup_share, 'config', 'environment_map.yaml')
    zones_config = os.path.join(bringup_share, 'config', 'zones.yaml')
    urdf_file = os.path.join(description_share, 'urdf', 'dasn_bot.urdf.xacro')
    bot_node_params = _load_bot_node_params(
        bot_params_config, env_map_config, zones_config
    )

    # Process xacro to get URDF string
    robot_description_content = xacro.process_file(urdf_file).toxml()

    # --- Bot nodes ---
    serial_bridge_node = Node(
        package='dasn_navigation',
        executable='serial_bridge',
        name='serial_bridge',
        output='screen',
        parameters=[bot_node_params['serial_bridge']],
    )

    odometry_node = Node(
        package='dasn_navigation',
        executable='odometry',
        name='odometry',
        output='screen',
    )

    navigator_node = Node(
        package='dasn_navigation',
        executable='navigator',
        name='navigator',
        output='screen',
        parameters=[bot_node_params['navigator']],
    )

    obstacle_avoidance_node = Node(
        package='dasn_navigation',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        output='screen',
        parameters=[bot_node_params['obstacle_avoidance']],
    )

    commander_node = Node(
        package='dasn_navigation',
        executable='commander',
        name='commander',
        output='screen',
        parameters=[bot_node_params['commander']],
    )

    stall_detector_node = Node(
        package='dasn_navigation',
        executable='stall_detector',
        name='stall_detector',
        output='screen',
        parameters=[bot_node_params['stall_detector']],
    )

    # --- Robot State Publisher ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
        }],
    )

    # --- Static Transform Publishers (TF tree) ---
    tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=['0', '0', '0.05', '0', '0', '0',
                   'base_link', 'imu_link'],
    )

    tf_base_to_tof_front = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_tof_front',
        arguments=['0.08', '0', '0.05', '0', '0', '0',
                   'base_link', 'tof_front'],
    )

    tf_base_to_sonar_rear = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_sonar_rear',
        arguments=['-0.08', '0', '0.05', '0', '0', '0',
                   'base_link', 'sonar_rear'],
    )

    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_camera',
        arguments=['0', '0', '0.15', '0', '0', '0',
                   'base_link', 'camera_link'],
    )

    tf_base_to_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_footprint',
        arguments=['0', '0', '-0.02', '0', '0', '0',
                   'base_link', 'base_footprint'],
    )

    return LaunchDescription([
        # Bot
        serial_bridge_node,
        odometry_node,
        navigator_node,
        obstacle_avoidance_node,
        commander_node,
        stall_detector_node,
        # TF tree
        tf_base_to_imu,
        tf_base_to_tof_front,
        tf_base_to_sonar_rear,
        tf_base_to_camera,
        tf_base_to_footprint,
        # Robot state publisher
        robot_state_publisher_node,
    ])
