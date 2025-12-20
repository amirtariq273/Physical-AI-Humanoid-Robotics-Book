# Chapter 3: Integrate Sensor Simulation

This chapter focuses on integrating simulated sensor data into digital twin environments, specifically LiDAR, Depth Cameras, and IMUs (Inertial Measurement Units). These sensors are crucial for robots to perceive their surroundings and estimate their own state.

## 3.1 Simulated LiDAR Sensors

LiDAR (Light Detection and Ranging) sensors provide distance measurements to objects in the environment, typically represented as point clouds. Simulating LiDAR data allows for testing navigation and mapping algorithms.

### 3.1.1 Example: LiDAR Publisher Node

This ROS 2 Python node publishes mock LiDAR sensor data.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan # Example for LiDAR

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'lidar_scan', 10)
        self.get_logger().info('LidarPublisher node started (placeholder).')

    # This would typically read from Gazebo and publish
    # def publish_data(self, data):
    #     msg = LaserScan()
    #     # Fill msg with data
    #     self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    lidar_publisher = LidarPublisher()
    rclpy.spin(lidar_publisher)
    lidar_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

_Code Location: `simulation_ws/src/module2_gazebo_examples/nodes/lidar_publisher.py`_

## 3.2 Simulated Depth Cameras

Depth cameras provide both color (RGB) and depth information for each pixel, enabling robots to understand the 3D structure of their environment. This is vital for object detection, manipulation, and obstacle avoidance.

### 3.2.1 Example: Depth Camera Publisher Node

This ROS 2 Python node publishes mock Depth Camera sensor data.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo # Example for Depth Camera

class DepthCameraPublisher(Node):
    def __init__(self):
        super().__init__('depth_camera_publisher')
        self.image_publisher_ = self.create_publisher(Image, 'depth_image', 10)
        self.info_publisher_ = self.create_publisher(CameraInfo, 'depth_camera_info', 10)
        self.get_logger().info('DepthCameraPublisher node started (placeholder).')

def main(args=None):
    rclpy.init(args=args)
    depth_camera_publisher = DepthCameraPublisher()
    rclpy.spin(depth_camera_publisher)
    depth_camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

_Code Location: `simulation_ws/src/module2_gazebo_examples/nodes/depth_camera_publisher.py`_

## 3.3 Simulated IMUs

IMUs provide information about a robot's orientation, angular velocity, and linear acceleration. This data is essential for state estimation, balancing, and motion control.

### 3.3.1 Example: IMU Publisher Node

This ROS 2 Python node publishes mock IMU sensor data.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu # Example for IMU

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu_data', 10)
        self.get_logger().info('ImuPublisher node started (placeholder).')

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

_Code Location: `simulation_ws/src/module2_gazebo_examples/nodes/imu_publisher.py`_

### 3.3.2 Example: RViz2 Sensor Visualization

This `rviz2` configuration file provides a basic setup for visualizing the simulated sensor data.

```yaml
# RViz configuration for sensor display
#
# To use: rviz2 -d src/module2_gazebo_examples/rviz_config/sensor_display.rviz

Views:
  Current:
    Class: rviz_default_plugins/Views/Orbit
    Target Frame: base_link
    Theta: 1.3
    Phi: 0.9
    Distance: 5
Global Options:
  Fixed Frame: base_link
Visualizers:
  - Class: rviz_default_plugins/PointCloud2
    Name: Lidar Scan
    Topic: /lidar_scan
    Color: 255; 0; 0
    Style: Points
    Size (Pixels): 3
  - Class: rviz_default_plugins/Image
    Name: Depth Image
    Topic: /depth_image
  - Class: rviz_default_plugins/Imu
    Name: IMU
    Topic: /imu_data
    Color: 0; 255; 0
    Show Orientation: true
    Show Acceleration: true
    Show Angular Velocity: true
```

_File Location: `simulation_ws/src/module2_gazebo_examples/rviz_config/sensor_display.rviz`_

---

**Code Examples:**
\*(These simulation examples are referenced and explained in detail within this chapter.)\*\*

- **Gazebo Robot Model with Sensors**: `simulation_ws/src/module2_gazebo_examples/models/humanoid_robot_sensors/`
- **ROS 2 Sensor Data Publisher Node**: `simulation_ws/src/module2_gazebo_examples/nodes/sensor_publisher.py`
- **RViz2 Sensor Visualization Config**: `simulation_ws/src/module2_gazebo_examples/rviz_config/sensor_display.rviz`
