from setuptools import find_packages, setup

package_name = 'devkit_implement_automation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'std_msgs', 'sensor_msgs'],
    zip_safe=True,
    maintainer='Zauberzeug GmbH',
    maintainer_email='ros@zauberzeug.com',
    description='Task-level implement automations built on standard ROS/ros2_control interfaces',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'weeding_screw_automation_node = devkit_implement_automation.weeding_screw_automation:main',
        ],
    },
)
