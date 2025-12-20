# Quickstart Guide: Module 3 - The AI-Robot Brain (NVIDIA Isaac™)

This guide provides quick setup instructions and how to run the simulation examples for Module 3, focusing on NVIDIA Isaac Sim, Isaac ROS (VSLAM), and Nav2 path planning for humanoid robots.

## 1. Prerequisites

*   **Operating System**: Linux (Ubuntu 20.04/22.04 recommended for ROS 2).
*   **NVIDIA GPU**: Compatible NVIDIA GPU is required for Isaac Sim and Isaac ROS.
*   **NVIDIA Isaac Sim**: A working installation of NVIDIA Isaac Sim (latest stable version).
    *   [Isaac Sim Installation Guide](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_overview.html)
*   **ROS 2 Installation**: A working installation of ROS 2 Humble or Iron.
*   **NVIDIA Isaac ROS**: A working installation of NVIDIA Isaac ROS (latest stable version, compatible with your Isaac Sim and ROS 2).
    *   [Isaac ROS Installation Guide](https://nvidia-isaac-ros.github.io/getting_started/getting_started_main.html)
*   **Python**: Python 3.8+ (compatible with Isaac Sim and ROS 2).

## 2. Setting Up Your ROS 2 + Isaac Sim Workspace

1.  **Create a ROS 2 Workspace and Clone Examples**:
    ```bash
    mkdir -p ~/isaac_ros_ws/src
    cd ~/isaac_ros_ws/src
    # Assuming this module's examples are in a Git repository
    # git clone <YOUR_REPO_URL>/module3_isaac_examples.git
    # For now, ensure your local directory structure matches `isaac_ros_ws/src/module3_isaac_examples/`
    ```

2.  **Install ROS Dependencies**:
    ```bash
    cd ~/isaac_ros_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  **Build the Workspace**:
    ```bash
    colcon build --packages-up-to module3_isaac_examples
    ```

4.  **Source the Workspace**:
    ```bash
    source install/setup.bash
    ```

## 3. Running Simulation Examples

### 3.1 Isaac Sim Environment Generation & Synthetic Data (Chapter 1)

1.  **Launch Isaac Sim**: Start Isaac Sim either via the Omniverse Launcher or command line.
2.  **Load Environment/Robot**:
    *   Load `isaac_sim_assets/environments/simple_room.usd`
    *   Import your humanoid robot (e.g., from `ros2_ws/src/module1_ros2_examples/urdf/humanoid.urdf`) and place it in the scene.
    *   Alternatively, load `isaac_sim_assets/robots/humanoid_with_camera.usd` directly.
3.  **Run Synthetic Data Generation Script**:
    ```bash
    # Open Isaac Sim's Script Editor and run:
    # python isaac_ros_ws/src/module3_isaac_examples/scripts/synthetic_data_generator.py
    # Or execute directly from terminal (ensure Isaac Sim environment is sourced)
    /path/to/isaac-sim/_build/python.exe isaac_ros_ws/src/module3_isaac_examples/scripts/synthetic_data_generator.py
    ```

### 3.2 Isaac ROS VSLAM Pipeline (Chapter 2)

1.  **Launch Isaac Sim with Robot and Stereo Camera**:
    *   Load your environment (e.g., `isaac_sim_assets/environments/simple_room.usd`).
    *   Ensure your humanoid robot (e.g., `isaac_sim_assets/robots/humanoid_with_camera.usd`) is in the scene with a correctly configured stereo camera (ROS 2 Camera Helper attached, publishing to `/left/image_raw`, `/right/image_raw`, etc.).
2.  **Launch Isaac ROS VSLAM Node**:
    ```bash
    ros2 launch module3_isaac_examples vslam.launch.py
    ```
3.  **Visualize Localization/Mapping**: Use `rviz2` to view robot pose and generated map.
    ```bash
    rviz2 -d isaac_ros_ws/src/module3_isaac_examples/rviz_config/vslam.rviz
    ```

### 3.3 Nav2 Path Planning (Chapter 3)

1.  **Launch Isaac Sim with Robot in Mapped Environment**:
    *   Load your environment with the robot, preferably using `isaac_sim_assets/environments/humanoid_in_room.usd`.
    *   Ensure the VSLAM pipeline (or an equivalent mapping system) has generated a map that Nav2 can use.
2.  **Launch Nav2 Stack**:
    ```bash
    ros2 launch module3_isaac_examples nav2.launch.py
    ```
3.  **Set Initial Pose and Navigation Goal**:
    *   Open `rviz2`: `rviz2 -d isaac_ros_ws/src/module3_isaac_examples/rviz_config/nav2.rviz`
    *   Use the "2D Pose Estimate" tool in RViz2 to set the robot's initial position on the map.
    *   Use the "Nav2 Goal" tool to set a navigation goal.

## 4. Troubleshooting

*   **Isaac Sim not launching**: Check NVIDIA driver installation, GPU compatibility, and Isaac Sim logs.
*   **ROS 2 nodes not found**: Verify ROS 2 workspace is sourced, and check package installation.
*   **Performance issues**: Ensure sufficient GPU resources, adjust Isaac Sim quality settings.
*   **Nav2 planning failures**: Ensure map is valid, robot footprint is correctly configured, and controller parameters are tuned for humanoid movement.
