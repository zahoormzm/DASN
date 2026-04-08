from setuptools import find_packages, setup

package_name = 'dasn_sentinel'

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
    description='DASN Sentinel node communication package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wifi_receiver = dasn_sentinel.wifi_receiver_node:main',
            'security_controller = dasn_sentinel.security_controller_node:main',
        ],
    },
)
