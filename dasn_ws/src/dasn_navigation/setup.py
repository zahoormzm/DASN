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
            'demo_bot_bridge = dasn_navigation.demo_bot_bridge_node:main',
        ],
    },
)
