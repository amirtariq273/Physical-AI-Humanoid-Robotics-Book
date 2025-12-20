# Chapter 1: Simulate Robot Physics in Gazebo

This chapter introduces the fundamental concepts of simulating robot physics, gravity, and collisions within the Gazebo environment. Gazebo is a powerful 3D robotics simulator that allows developers to accurately test and develop robot control systems without the need for physical hardware.

## 1.1 Gazebo Environment Setup

Gazebo worlds are defined using SDF (Simulation Description Format) files, which specify the environment, objects, and physics properties.

### 1.1.1 Example: Humanoid Physics World

This Gazebo world file defines a basic environment with a ground plane and configured physics.

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="humanoid_physics_world">
    <physics name="default_physics" default="0" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

    <!-- Placeholder for Humanoid Robot - will be loaded via URDF/SDF -->
    <!-- <include>
      <uri>model://humanoid_robot</uri>
    </include> -->

  </world>
</sdf>
```

_File Location: `simulation_ws/src/module2_gazebo_examples/worlds/humanoid_physics.world`_

## 1.2 Robot Models in Gazebo

Robots are typically integrated into Gazebo using URDF (Unified Robot Description Format) or SDF models. These models define the robot's links, joints, and visual/collision properties, enabling Gazebo to simulate its behavior.

### 1.2.1 Example: Basic Humanoid Robot Model

This humanoid robot model is defined using URDF and can be integrated into Gazebo.

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">

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

_File Location: `simulation_ws/src/module2_gazebo_examples/models/humanoid_robot/model.urdf`_

## 1.3 Simulating Physics and Collisions

Gazebo's physics engine handles the interactions between robot models and the environment, including gravity, friction, and collisions. Understanding how to configure these properties is crucial for realistic simulations.

---

**Code Examples:**
_(These simulation examples are referenced and explained in detail within this chapter.)_

- **Sample Gazebo World File**: `simulation_ws/src/module2_gazebo_examples/worlds/humanoid_physics.world`
- **Basic Humanoid Robot Model (URDF/SDF)**: `simulation_ws/src/module2_gazebo_examples/models/`
