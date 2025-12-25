# Chapter 1: Simulate Robot Physics in Gazebo

This chapter introduces the fundamental concepts of simulating robot physics, gravity, and collisions within the **Gazebo** environment. Gazebo is a widely used open-source 3D robotics simulator that allows developers to **test and validate robot behavior in virtual environments** without the need for physical hardware. By simulating robot physics, developers can optimize control algorithms, validate robot designs, and debug behaviors before deploying on real robots.

Gazebo provides an environment to test **motion planning, navigation, sensors, and robot interactions** with objects. This chapter focuses on setting up a humanoid robot simulation, configuring physics, and understanding collision interactions.

---

## 1.1 Gazebo Environment Setup

Gazebo simulations are structured around **worlds**, which define:

- The environment (ground, obstacles, lights)  
- Physical properties (gravity, friction, collisions)  
- Time stepping and real-time updates  

Gazebo worlds are typically defined using **SDF (Simulation Description Format)** files. These files describe **static and dynamic objects**, the **physics engine** (ODE, Bullet, or DART), and simulation parameters.  

Key considerations when setting up a Gazebo world:

1. **Physics Engine Configuration**  
   - `max_step_size`: Maximum time step per simulation iteration. Smaller values yield more accurate results but require more computation.  
   - `real_time_factor`: Simulation speed relative to real-time.  
   - `real_time_update_rate`: Number of simulation steps per second.

2. **Gravity and Inertia**  
   - Gravity must be consistent with real-world values (default: 9.81 m/s² along the negative Z-axis).  
   - Robot links should have proper **mass and inertia** defined to simulate realistic motion.

3. **Collision and Friction**  
   - Proper collision shapes prevent robot parts from intersecting with environment or each other.  
   - Friction coefficients are important for walking, grasping, and manipulation tasks.

---

### 1.1.1 Example: Humanoid Physics World

This Gazebo world defines a ground plane, sunlight, and physics configuration all in **one file**:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="humanoid_physics_world">

    <physics type="ode">
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
      <pose>0 0 10 0 0 0</pose>
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

    <!-- Humanoid Robot Model Included Inline -->
    <model name="humanoid_robot">

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

      <!-- Right Arm -->
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

      <joint name="right_arm_joint" type="revolute">
        <parent link="base_link"/>
        <child link="right_arm_link"/>
        <origin xyz="0 0.1 0.15" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
        <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
      </joint>

      <!-- Left Arm -->
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

      <joint name="left_arm_joint" type="revolute">
        <parent link="base_link"/>
        <child link="left_arm_link"/>
        <origin xyz="0 -0.1 0.15" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
        <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
      </joint>

    </model>

  </world>
</sdf>
