from setuptools import find_packages, setup

package_name = 'dasn_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dasn',
    maintainer_email='dasn@todo.todo',
    description='DASN bot control and navigation package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge = dasn_navigation.serial_bridge_node:main',
            'odometry = dasn_navigation.odometry_node:main',
            'navigator = dasn_navigation.navigator_node:main',
            'obstacle_avoidance = dasn_navigation.obstacle_avoidance_node:main',
            'commander = dasn_navigation.commander_node:main',
            'stall_detector = dasn_navigation.stall_detector_node:main',
        ],
    },
)
