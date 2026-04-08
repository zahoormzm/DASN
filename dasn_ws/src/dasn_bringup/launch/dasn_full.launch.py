"""Full demo launch for the laptop-centric DASN presentation stack."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _load_yaml(path):
    with open(path, 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle) or {}


def _load_zone_ids(zones_path):
    data = _load_yaml(zones_path)
    zones = data.get('zones', {})
    return list(zones.keys())


def _load_bot_params(bot_params_path):
    data = _load_yaml(bot_params_path)
    params = data.get('/**', {}).get('ros__parameters', {})
    serial = params.get('serial', {})
    demo_bot = params.get('demo_bot', {})
    return {
        'serial_port': serial.get('port', '/dev/ttyUSB1'),
        'baud_rate': int(serial.get('baud_rate', 115200)),
        'pose_poll_interval_s': float(demo_bot.get('pose_poll_interval_s', 1.0)),
        'status_publish_interval_s': float(demo_bot.get('status_publish_interval_s', 0.5)),
        'door_a_x': float(demo_bot.get('door_a_x', 1.2)),
        'door_a_y': float(demo_bot.get('door_a_y', 0.0)),
        'door_b_x': float(demo_bot.get('door_b_x', 0.0)),
        'door_b_y': float(demo_bot.get('door_b_y', 0.0)),
        'dock_x': float(demo_bot.get('dock_x', 0.0)),
        'dock_y': float(demo_bot.get('dock_y', 0.0)),
        'door_b_patrol_points': demo_bot.get(
            'door_b_patrol_points',
            [0.0, 0.0, 0.25, 0.0, 0.25, 0.25, 0.0, 0.25],
        ),
    }


def _load_demo_endpoints(path):
    data = _load_yaml(path)
    params = data.get('/**', {}).get('ros__parameters', {})
    return {
        'rp5_rtsp_url': params.get('rp5_rtsp_url', 'rtsp://192.168.1.100:8554/stream'),
        'bot_phone_ip': params.get('bot_phone_ip', '192.168.1.101'),
    }


def generate_launch_description():
    bringup_share = get_package_share_directory('dasn_bringup')

    zones_config = os.path.join(bringup_share, 'config', 'zones.yaml')
    security_config = os.path.join(bringup_share, 'config', 'security_controller.yaml')
    bot_params_config = os.path.join(bringup_share, 'config', 'bot_params.yaml')
    endpoints_config = os.path.join(bringup_share, 'config', 'demo_endpoints.yaml')

    zone_ids = _load_zone_ids(zones_config)
    bot_params = _load_bot_params(bot_params_config)
    endpoints = _load_demo_endpoints(endpoints_config)

    nodes = [
        Node(
            package='dasn_perception',
            executable='rtsp_camera_node',
            name='rp5_camera',
            output='screen',
            parameters=[{'rtsp_url': endpoints['rp5_rtsp_url']}],
        ),
        Node(
            package='dasn_perception',
            executable='phone_camera_node',
            name='phone_camera',
            output='screen',
            parameters=[{'phone_ip': endpoints['bot_phone_ip']}],
        ),
        Node(
            package='dasn_perception',
            executable='object_detector_node',
            name='object_detector',
            output='screen',
        ),
        Node(
            package='dasn_perception',
            executable='face_detector_node',
            name='face_detector',
            output='screen',
        ),
        Node(
            package='dasn_sentinel',
            executable='wifi_receiver',
            name='wifi_receiver',
            output='screen',
        ),
        Node(
            package='dasn_sentinel',
            executable='security_controller',
            name='security_controller',
            output='screen',
            parameters=[security_config, {'zone_ids': zone_ids}],
        ),
        Node(
            package='dasn_navigation',
            executable='demo_bot_bridge',
            name='demo_bot_bridge',
            output='screen',
            parameters=[bot_params],
        ),
        Node(
            package='dasn_dashboard',
            executable='phone_speaker',
            name='phone_speaker',
            output='screen',
            parameters=[{'phone_ip': endpoints['bot_phone_ip']}],
        ),
    ]

    return LaunchDescription(nodes)
