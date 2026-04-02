from setuptools import find_packages, setup

package_name = 'dasn_dashboard'

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
    description='DASN dashboard alert nodes package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telegram_alert = dasn_dashboard.telegram_alert_node:main',
            'phone_speaker = dasn_dashboard.phone_speaker_node:main',
        ],
    },
)
