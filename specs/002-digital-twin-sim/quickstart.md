# Quickstart Guide: Module 2 - The Digital Twin (Gazebo & Unity)

This guide provides quick setup instructions and how to run the simulation examples for Module 2, focusing on Gazebo physics, Unity rendering, and simulated sensors for humanoid robots.

## 1. Prerequisites

*   **Operating System**: Linux (Ubuntu 20.04/22.04 recommended for Gazebo).
*   **Gazebo Installation**: A working installation of Gazebo (version compatible with ROS 2, e.g., Gazebo Garden for ROS 2 Humble/Iron).
    *   [Gazebo Installation Guide](https://gazebosim.org/docs/garden/install_ubuntu)
*   **Unity Hub & Editor**: A working installation of Unity Hub and Unity Editor (LTS version recommended).
    *   [Unity Installation Guide](https://unity.com/download)
*   **ROS 2 Installation**: If integrating with ROS 2 (for Gazebo/Unity-ROS-TCP-Connector), a working ROS 2 Humble or Iron installation.
*   **Unity Robotics Packages**: Install relevant Unity Robotics packages (e.g., ROS-TCP-Connector, Unity.Robotics.URDFImporter) via Unity Package Manager.

## 2. Setting Up Your Workspaces

### 2.1 Gazebo Workspace

1.  **Create a ROS 2 Workspace (if not already existing)**:
    ```bash
    mkdir -p ~/simulation_ws/src
    cd ~/simulation_ws/src
    ```

2.  **Clone Module 2 Gazebo Examples**:
    ```bash
    git clone [repository-url]/module2_gazebo_examples.git
    ```
    (Note: Replace `[repository-url]` with the actual repository URL once defined, and `module2_gazebo_examples.git` with the specific examples repository.)

3.  **Build the Workspace**:
    ```bash
    cd ~/simulation_ws
    colcon build
    ```

4.  **Source the Workspace**:
    ```bash
    source install/setup.bash
    ```

### 2.2 Unity Project

1.  **Open Unity Hub**: Create a new 3D project or open the cloned Unity project for Module 2.
    ```bash
    git clone [repository-url]/module2_unity_examples.git
    ```
    (Note: Replace `[repository-url]` with the actual repository URL once defined, and `module2_unity_examples.git` with the specific examples repository.)
2.  **Import URDF Model**: Use `Unity.Robotics.URDFImporter` to import humanoid robot URDF models.
3.  **Install ROS-TCP-Connector**: Via Unity Package Manager if ROS 2 integration is used.

## 3. Running Simulation Examples

### 3.1 Gazebo Physics Simulation

1.  **Launch Gazebo World with Humanoid**:
    ```bash
    ros2 launch module2_gazebo_examples humanoid_world.launch.py
    ```
    (Note: `module2_gazebo_examples` and `humanoid_world.launch.py` are placeholders)

2.  **Verify Physics**: Observe robot behavior (e.g., falling, collisions).

### 3.2 Unity High-Fidelity Rendering

1.  **Open Unity Project**: Navigate to the scene containing the humanoid robot.
2.  **Play Scene**: Run the Unity scene to observe rendering and interact with the robot.

### 3.3 Simulated Sensor Data

1.  **Launch Gazebo Simulation with Sensors**:
    ```bash
    ros2 launch module2_gazebo_examples humanoid_sensors.launch.py
    ```
2.  **View Sensor Data**: Use `rviz2` to visualize LiDAR point clouds, Depth Camera images, or plot IMU data.
    ```bash
    rviz2 -d [path/to/module2_rviz_config.rviz]
    ```

## 4. Troubleshooting

*   **Gazebo not launching**: Check ROS 2 setup, Gazebo installation, and `GAZEBO_MODEL_PATH` environment variable.
*   **Unity build errors**: Ensure Unity Robotics packages are correctly installed and dependencies are met.
*   **ROS 2 integration issues**: Verify ROS-TCP-Connector setup in Unity and ROS 2 network configuration.

---
**Next Steps**: Detailed development tasks for content creation and example implementation will be outlined in `tasks.md`.
