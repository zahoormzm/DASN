"""
Sentinel test launch for DASN.

Launches sentinel subsystem with threat processing and rosbridge
for dashboard connectivity testing.
"""

import os

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


def generate_launch_description():

    bringup_share = get_package_share_directory('dasn_bringup')

    zones_config = os.path.join(bringup_share, 'config', 'zones.yaml')
    auth_tags_config = os.path.join(bringup_share, 'config', 'authorized_tags.yaml')
    threat_weights_config = os.path.join(bringup_share, 'config', 'threat_weights.yaml')
    zone_ids, zone_waypoints = _load_zone_data(zones_config)
    fusion_params = _load_fusion_params(threat_weights_config)

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

    # --- Dashboard (rosbridge only) ---
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='websocket_server',
        output='screen',
    )

    return LaunchDescription([
        gateway_receiver_node,
        auth_manager_node,
        fusion_node,
        escalation_node,
        rosbridge_node,
    ])
