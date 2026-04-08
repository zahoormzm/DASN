"""Lean demo launch for the fixed node + camera + controller flow."""

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
    return list((data.get('zones') or {}).keys())


def _load_demo_endpoints(path):
    data = _load_yaml(path)
    params = data.get('/**', {}).get('ros__parameters', {})
    return {
        'rp5_rtsp_url': params.get('rp5_rtsp_url', 'rtsp://192.168.1.100:8554/stream'),
    }


def generate_launch_description():
    bringup_share = get_package_share_directory('dasn_bringup')
    zones_config = os.path.join(bringup_share, 'config', 'zones.yaml')
    security_config = os.path.join(bringup_share, 'config', 'security_controller.yaml')
    endpoints_config = os.path.join(bringup_share, 'config', 'demo_endpoints.yaml')
    zone_ids = _load_zone_ids(zones_config)
    endpoints = _load_demo_endpoints(endpoints_config)

    return LaunchDescription([
        Node(
            package='dasn_perception',
            executable='rtsp_camera_node',
            name='rp5_camera',
            output='screen',
            parameters=[{'rtsp_url': endpoints['rp5_rtsp_url']}],
        ),
        Node(
            package='dasn_perception',
            executable='object_detector_node',
            name='object_detector',
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
    ])
