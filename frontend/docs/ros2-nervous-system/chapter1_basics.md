# Chapter 1: ROS 2 Communication Basics

This chapter introduces the fundamental concepts of ROS 2 communication, which are essential for building robotic applications. We will cover Nodes, Topics, and Services, and how they interact to form a distributed robotic system.

## 1.1 ROS 2 Nodes

In ROS 2, a Node is an executable process that performs computation. Nodes are typically designed to do one thing well, such as controlling a motor, reading sensor data, or performing a navigation algorithm.

### 1.1.1 Example: Simple Publisher Node

This example demonstrates a basic ROS 2 Node that publishes messages to a topic.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

_Code Location: `ros2_ws/src/module1_ros2_examples/nodes/simple_publisher.py`_

## 1.2 ROS 2 Topics

Topics are a powerful mechanism for Nodes to exchange messages in a publish-subscribe pattern. A Node can publish data to a Topic, and other Nodes can subscribe to that Topic to receive the data. This allows for flexible, one-to-many communication.

### 1.2.1 Example: Simple Subscriber Node

This example demonstrates a basic ROS 2 Node that subscribes to messages from a topic.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

_Code Location: `ros2_ws/src/module1_ros2_examples/nodes/simple_subscriber.py`_

## 1.3 ROS 2 Services

Services provide a request-reply communication model between Nodes. A Node can offer a Service, and other Nodes can send requests to that Service and receive a reply. This is useful for calls that expect a direct response.

### 1.3.1 Example: Simple Service Server Node

This example demonstrates a basic ROS 2 Node that acts as a service server.

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Standard ROS 2 service for demonstration

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__('simple_service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request: a: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    simple_service_server = SimpleServiceServer()
    rclpy.spin(simple_service_server)
    simple_service_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

_Code Location: `ros2_ws/src/module1_ros2_examples/nodes/simple_service_server.py`_

### 1.3.2 Example: Simple Service Client Node

This example demonstrates a basic ROS 2 Node that acts as a service client.

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys

class SimpleServiceClient(Node):
    def __init__(self):
        super().__init__('simple_service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    simple_service_client = SimpleServiceClient()

    if len(sys.argv) == 3:
        simple_service_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    else:
        simple_service_client.send_request(2, 3) # Default values
        simple_service_client.get_logger().info('Usage: ros2 run <pkg_name> simple_service_client <a> <b>')

    while rclpy.ok():
        rclpy.spin_once(simple_service_client)
        if simple_service_client.future.done():
            try:
                response = simple_service_client.future.result()
            except Exception as e:
                simple_service_client.get_logger().error('Service call failed %r' % (e,))
            else:
                simple_service_client.get_logger().info(
                    'Result of add_two_ints: for %d + %d = %d' %
                    (simple_service_client.req.a, simple_service_client.req.b, response.sum))
            break

    simple_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

_Code Location: `ros2_ws/src/module1_ros2_examples/nodes/simple_service_client.py`_

---

**Code Examples:**
_(These code examples are referenced and explained in detail within this chapter.)_

- **Simple Publisher Node**: `ros2_ws/src/module1_ros2_examples/nodes/simple_publisher.py`
- **Simple Subscriber Node**: `ros2_ws/src/module1_ros2_examples/nodes/simple_subscriber.py`
- **Simple Service Server Node**: `ros2_ws/src/module1_ros2_examples/nodes/simple_service_server.py`
- **Simple Service Client Node**: `ros2_ws/src/module1_ros2_examples/nodes/simple_service_client.py`
