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
            'phone_camera_node = dasn_perception.phone_camera_node:main',
            'espcam_node = dasn_perception.espcam_node:main',
            'face_detector_node = dasn_perception.face_detector_node:main',
            'face_recognizer_node = dasn_perception.face_recognizer_node:main',
            'object_detector_node = dasn_perception.object_detector_node:main',
            'pose_analyzer_node = dasn_perception.pose_analyzer_node:main',
            'sound_classifier_node = dasn_perception.sound_classifier_node:main',
            'gas_analyzer_node = dasn_perception.gas_analyzer_node:main',
        ],
    },
)
