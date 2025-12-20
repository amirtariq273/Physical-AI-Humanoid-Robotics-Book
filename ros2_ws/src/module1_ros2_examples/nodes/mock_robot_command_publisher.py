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
