# Chapter 2: Python Agent → ROS Bridge with `rclpy`

This chapter explores how to bridge Python logic with ROS 2, enabling programmatic control of robotic functionalities. We will focus on `rclpy`, the official Python client library for ROS 2, and demonstrate how to interact with ROS 2 communication mechanisms from Python.

## 2.1 Introduction to `rclpy`

`rclpy` provides a Python API for writing ROS 2 applications. It allows Python developers to create Nodes, publish/subscribe to Topics, offer/call Services, and perform other ROS 2 operations using familiar Python syntax.

### 2.1.1 Example: Mock Robot Command Publisher

This example demonstrates a basic `rclpy` Node that publishes mock velocity commands to a robot.

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist # Common message type for robot velocity commands

class MockRobotCommandPublisher(Node):
    def __init__(self):
        super().__init__('mock_robot_command_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.linear_x = 0.1
        self.angular_z = 0.0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Mock Command: Linear X="%f", Angular Z="%f"' % (msg.linear.x, msg.angular.z))
        # Example: change command over time
        self.linear_x = 0.1 if self.linear_x == 0.0 else 0.0
        # self.angular_z = 0.5 if self.angular_z == 0.0 else 0.0 # uncomment to add angular motion

def main(args=None):
    rclpy.init(args=args)
    mock_robot_command_publisher = MockRobotCommandPublisher()
    rclpy.spin(mock_robot_command_publisher)
    mock_robot_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

_Code Location: `ros2_ws/src/module1_ros2_examples/nodes/mock_robot_command_publisher.py`_

## 2.2 Bridging Python to ROS 2 Topics

Using `rclpy`, a Python Node can easily publish commands to ROS 2 topics or subscribe to sensor data published by other ROS 2 Nodes. This forms the basis of controlling robots or processing their data using Python.

### 2.2.1 Example: Mock Robot Sensor Subscriber

This example demonstrates a basic `rclpy` Node that subscribes to mock sensor data.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan # Common message type for LiDAR data

class MockRobotSensorSubscriber(Node):
    def __init__(self):
        super().__init__('mock_robot_sensor_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan', # Assuming a LiDAR scan topic
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # In a real scenario, you'd process the LaserScan data.
        # Here, we'll just log some info.
        self.get_logger().info('Received mock sensor data (LaserScan). Range min: %f, max: %f' % (msg.range_min, msg.range_max))
        # self.get_logger().info('First few ranges: %s' % str(msg.ranges[:5])) # Uncomment to see actual data

def main(args=None):
    rclpy.init(args=args)
    mock_robot_sensor_subscriber = MockRobotSensorSubscriber()
    rclpy.spin(mock_robot_sensor_subscriber)
    mock_robot_sensor_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

_Code Location: `ros2_ws/src/module1_ros2_examples/nodes/mock_robot_sensor_subscriber.py`_

## 2.3 `rclpy` for Services

`rclpy` also supports ROS 2 Services, allowing Python Nodes to act as service servers (providing functionality) or service clients (requesting functionality).

---

**Code Examples:**
_(These code examples are referenced and explained in detail within this chapter.)_

- **Mock Robot Command Publisher**: `ros2_ws/src/module1_ros2_examples/nodes/mock_robot_command_publisher.py`
- **Mock Robot Sensor Subscriber**: `ros2_ws/src/module1_ros2_examples/nodes/mock_robot_sensor_subscriber.py`
