"""
Full system launch for DASN (Distributed Autonomous Security Network).

Launches all subsystems: perception, sentinel, threat, bot, dashboard,
robot_state_publisher, and static TF tree.
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


def _load_zone_data(zones_path):
    data = _load_yaml(zones_path)
    zones = data.get('zones', {})
    zone_ids = list(zones.keys())
    zone_waypoints = []
    for zone_id in zone_ids:
        pos = zones.get(zone_id, {}).get('position', [0.0, 0.0])
        zone_waypoints.extend([float(pos[0]), float(pos[1]), 0.0])
    return zone_ids, zone_waypoints


def _load_fusion_params(weights_path):
    data = _load_yaml(weights_path)
    weights = data.get('threat_weights', {})
    multipliers = data.get('multipliers', {})
    return {
        'weight.radar_presence': float(weights.get('radar_presence', 0.25)),
        'weight.pir_motion': float(weights.get('pir_motion', 0.10)),
        'weight.sound_anomaly': float(weights.get('sound_anomaly', 0.20)),
        'weight.unknown_face': float(weights.get('unknown_face', 0.25)),
        'weight.gas_smoke_anomaly': float(weights.get('gas_smoke_anomaly', 0.10)),
        'weight.auth_failure': float(weights.get('auth_failure', 0.10)),
        'nighttime_start': int(multipliers.get('nighttime_start', 22)),
        'nighttime_end': int(multipliers.get('nighttime_end', 6)),
        'nighttime_factor': float(multipliers.get('nighttime_factor', 2.0)),
        'zone_history_max': float(multipliers.get('zone_history_max', 1.5)),
        'zone_history_decay_hours': int(multipliers.get('zone_history_decay_hours', 24)),
    }


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

    # Config file paths
    zones_config = os.path.join(bringup_share, 'config', 'zones.yaml')
    env_map_config = os.path.join(bringup_share, 'config', 'environment_map.yaml')
    auth_tags_config = os.path.join(bringup_share, 'config', 'authorized_tags.yaml')
    threat_weights_config = os.path.join(bringup_share, 'config', 'threat_weights.yaml')
    bot_params_config = os.path.join(bringup_share, 'config', 'bot_params.yaml')
    urdf_file = os.path.join(description_share, 'urdf', 'dasn_bot.urdf.xacro')

    zone_ids, zone_waypoints = _load_zone_data(zones_config)
    fusion_params = _load_fusion_params(threat_weights_config)
    bot_node_params = _load_bot_node_params(
        bot_params_config, env_map_config, zones_config
    )

    # --- Camera nodes ---
    phone_camera_node = Node(
        package='dasn_perception',
        executable='phone_camera_node',
        name='phone_camera',
        output='screen',
    )

    espcam_node = Node(
        package='dasn_perception',
        executable='espcam_node',
        name='espcam',
        output='screen',
    )

    # --- ML pipeline nodes ---
    face_detector_node = Node(
        package='dasn_perception',
        executable='face_detector_node',
        name='face_detector',
        output='screen',
    )

    face_recognizer_node = Node(
        package='dasn_perception',
        executable='face_recognizer_node',
        name='face_recognizer',
        output='screen',
    )

    object_detector_node = Node(
        package='dasn_perception',
        executable='object_detector_node',
        name='object_detector',
        output='screen',
    )

    pose_analyzer_node = Node(
        package='dasn_perception',
        executable='pose_analyzer_node',
        name='pose_analyzer',
        output='screen',
    )

    sound_classifier_node = Node(
        package='dasn_perception',
        executable='sound_classifier_node',
        name='sound_classifier',
        output='screen',
    )

    gas_analyzer_node = Node(
        package='dasn_perception',
        executable='gas_analyzer_node',
        name='gas_analyzer',
        output='screen',
        parameters=[{'zone_ids': zone_ids}],
    )

    # --- Sentinel nodes ---
    gateway_receiver_node = Node(
        package='dasn_sentinel',
        executable='gateway_receiver',
        name='gateway_receiver',
        output='screen',
    )

    auth_manager_node = Node(
        package='dasn_sentinel',
        executable='auth_manager',
        name='auth_manager',
        output='screen',
        parameters=[{
            'zone_ids': zone_ids,
            'auth_config_path': auth_tags_config,
        }],
    )

    # --- Threat nodes ---
    fusion_node = Node(
        package='dasn_threat',
        executable='fusion_node',
        name='fusion_node',
        output='screen',
        parameters=[{'zone_ids': zone_ids}, fusion_params],
    )

    escalation_node = Node(
        package='dasn_threat',
        executable='escalation_node',
        name='escalation_node',
        output='screen',
        parameters=[{
            'zone_ids': zone_ids,
            'zone_waypoints': zone_waypoints,
        }],
    )

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

    # --- Dashboard nodes ---
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='websocket_server',
        output='screen',
    )

    telegram_alert_node = Node(
        package='dasn_dashboard',
        executable='telegram_alert',
        name='telegram_alert',
        output='screen',
    )

    phone_speaker_node = Node(
        package='dasn_dashboard',
        executable='phone_speaker',
        name='phone_speaker',
        output='screen',
    )

    # --- Robot State Publisher ---
    robot_description_content = xacro.process_file(urdf_file).toxml()

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
        # Camera
        phone_camera_node,
        espcam_node,
        # ML pipeline
        face_detector_node,
        face_recognizer_node,
        object_detector_node,
        pose_analyzer_node,
        sound_classifier_node,
        gas_analyzer_node,
        # Sentinel
        gateway_receiver_node,
        auth_manager_node,
        # Threat
        fusion_node,
        escalation_node,
        # Bot
        serial_bridge_node,
        odometry_node,
        navigator_node,
        obstacle_avoidance_node,
        commander_node,
        stall_detector_node,
        # Dashboard
        rosbridge_node,
        telegram_alert_node,
        phone_speaker_node,
        # TF tree
        tf_base_to_imu,
        tf_base_to_tof_front,
        tf_base_to_sonar_rear,
        tf_base_to_camera,
        tf_base_to_footprint,
        # Robot state publisher
        robot_state_publisher_node,
    ])
