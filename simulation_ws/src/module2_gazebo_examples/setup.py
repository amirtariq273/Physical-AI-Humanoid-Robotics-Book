from setuptools import setup

package_name = 'module2_gazebo_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/my_launch_file.launch.py']), # Placeholder
        ('share/' + package_name + '/worlds', ['worlds/my_world.world']), # Placeholder
        ('share/' + package_name + '/models', ['models/my_robot/model.sdf']), # Placeholder
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name', # Placeholder
    maintainer_email='your_email@example.com', # Placeholder
    description='ROS 2 Gazebo examples for Module 2 - The Digital Twin',
    license='Apache License 2.0', # Placeholder
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Placeholders for nodes, will be filled later
        ],
    },
)
