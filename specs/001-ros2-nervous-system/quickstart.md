# Quickstart Guide: Module 1 - The Robotic Nervous System (ROS 2)

This guide provides quick setup instructions and how to run the code examples for Module 1, focusing on ROS 2 communication basics, `rclpy` Python bridge, and URDF for humanoid robots.

## 1. Prerequisites

*   **Operating System**: Ubuntu 20.04 (for ROS 2 Humble) or Ubuntu 22.04 (for ROS 2 Iron).
*   **ROS 2 Installation**: A working installation of ROS 2 Humble or ROS 2 Iron. Follow the official ROS 2 documentation for installation:
    *   [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)
    *   [ROS 2 Iron Installation Guide](https://docs.ros.org/en/iron/Installation.html)
*   **Python**: Python 3.8+ (usually comes with ROS 2 installation).

## 2. Setting Up Your ROS 2 Workspace

1.  **Create a ROS 2 Workspace**:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2.  **Clone Module 1 Code Examples**:
    ```bash
    git clone [repository-url]/module1_ros2_examples.git
    ```
    (Note: Replace `[repository-url]` with the actual repository URL once defined, and `module1_ros2_examples.git` with the specific examples repository.)

3.  **Install Python Dependencies**:
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

4.  **Build the Workspace**:
    ```bash
    cd ~/ros2_ws
    colcon build
    ```

5.  **Source the Workspace**:
    ```bash
    source install/setup.bash
    ```
    (Add this to your `~/.bashrc` for permanent sourcing: `echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc`)

## 3. Running Code Examples

### 3.1 ROS 2 Nodes and Topics Example

1.  **Open Terminal 1 (Publisher)**:
    ```bash
    ros2 run my_python_pkg simple_publisher
    ```
    (Note: `my_python_pkg` and `simple_publisher` are placeholders for actual package/node names)

2.  **Open Terminal 2 (Subscriber)**:
    ```bash
    ros2 run my_python_pkg simple_subscriber
    ```

### 3.2 ROS 2 Services Example

1.  **Open Terminal 1 (Service Server)**:
    ```bash
    ros2 run my_python_pkg simple_service_server
    ```

2.  **Open Terminal 2 (Service Client)**:
    ```bash
    ros2 run my_python_pkg simple_service_client
    ```

### 3.3 URDF Visualization

1.  **Launch URDF Viewer**:
    ```bash
    ros2 launch urdf_tutorial display.launch.py model:=src/my_urdf_pkg/urdf/humanoid.urdf
    ```
    (Note: `urdf_tutorial` and `my_urdf_pkg` are placeholders)

## 4. Troubleshooting

*   **`colcon build` errors**: Ensure all dependencies are installed (`rosdep install`), and check for syntax errors in your code.
*   **Node not found**: Verify `source install/setup.bash` is run in each new terminal.

---
**Next Steps**: Detailed development tasks for content creation and example implementation will be outlined in `tasks.md`.
