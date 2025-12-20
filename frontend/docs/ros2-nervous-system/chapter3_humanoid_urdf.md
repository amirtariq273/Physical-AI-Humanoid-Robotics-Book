# Chapter 3: Building a Humanoid URDF

This chapter introduces the Unified Robot Description Format (URDF), an XML format for describing the physical structure and kinematics of a robot. Understanding URDF is crucial for simulating robots in environments like Gazebo or visualizing them in ROS 2 tools.

## 3.1 URDF Fundamentals

A URDF file defines a robot's links (rigid bodies) and joints (connections between links). Each link can have associated visual and collision properties, while joints define the degrees of freedom and limits of motion.

### 3.1.1 Example: Simple Humanoid URDF File

This example demonstrates a basic URDF file for a humanoid robot with a base link and two arms.

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Right Arm Link -->
  <link name="right_arm_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right Arm Joint -->
  <joint name="right_arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_arm_link"/>
    <origin xyz="0 0.1 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>

  <!-- Left Arm Link -->
  <link name="left_arm_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Arm Joint -->
  <joint name="left_arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_arm_link"/>
    <origin xyz="0 -0.1 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>

</robot>
```

_Code Location: `ros2_ws/src/module1_ros2_examples/urdf/humanoid.urdf`_

## 3.2 Links

Links are the basic building blocks of a robot model. They represent the physical segments of the robot, such as a torso, an arm, or a wheel. Each link has a name, and properties like mass, inertia, and visual/collision geometry.

## 3.3 Joints

Joints connect two links: a parent link and a child link. They define how the child link can move relative to its parent. Different joint types (e.g., `revolute`, `prismatic`, `fixed`) specify different kinds of motion.

### 3.3.1 Example: URDF Visualization Launch File

This ROS 2 launch file is used to display the `humanoid.urdf` model in `rviz2` and control its joints using `joint_state_publisher_gui`.

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the launch argument for the robot model
    robot_model_arg = DeclareLaunchArgument(
        'model',
        default_value=TextSubstitution(text=os.path.join(
            get_package_share_directory('module1_ros2_examples'),
            'urdf',
            'humanoid.urdf'
        )),
        description='Path to the robot URDF file'
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': LaunchConfiguration('model'), 'use_sim_time': True}]
    )

    # Joint State Publisher GUI node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Rviz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('module1_ros2_examples'), 'rviz_config', 'humanoid_display.rviz')], # Placeholder
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        robot_model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
```

_Code Location: `ros2_ws/src/module1_ros2_examples/launch/display_humanoid.launch.py`_

## 3.4 Building a Simple Humanoid URDF

We will walk through constructing a basic URDF file for a humanoid robot, illustrating how links and joints are defined and connected to create a complete robot model.
