from setuptools import setup

package_name = 'module1_ros2_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/my_launch_file.launch.py']), # Placeholder
        ('share/' + package_name + '/urdf', ['urdf/my_robot.urdf']), # Placeholder
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name', # Placeholder
    maintainer_email='your_email@example.com', # Placeholder
    description='ROS 2 examples for Module 1 - The Robotic Nervous System',
    license='Apache License 2.0', # Placeholder
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = module1_ros2_examples.nodes.simple_publisher:main',
            'simple_subscriber = module1_ros2_examples.nodes.simple_subscriber:main',
            'simple_service_server = module1_ros2_examples.nodes.simple_service_server:main',
            'simple_service_client = module1_ros2_examples.nodes.simple_service_client:main',
            'mock_robot_command_publisher = module1_ros2_examples.nodes.mock_robot_command_publisher:main',
            'mock_robot_sensor_subscriber = module1_ros2_examples.nodes.mock_robot_sensor_subscriber:main',
        ],
    },
)
