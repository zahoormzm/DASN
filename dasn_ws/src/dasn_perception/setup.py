from setuptools import setup

package_name = 'dasn_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zahoor Mashahir',
    maintainer_email='zahoor@dasn.local',
    description='DASN perception package: camera input and ML inference nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rtsp_camera_node = dasn_perception.rtsp_camera_node:main',
            'phone_camera_node = dasn_perception.phone_camera_node:main',
            'face_detector_node = dasn_perception.face_detector_node:main',
            'object_detector_node = dasn_perception.object_detector_node:main',
        ],
    },
)
