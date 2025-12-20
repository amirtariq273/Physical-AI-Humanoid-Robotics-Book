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
