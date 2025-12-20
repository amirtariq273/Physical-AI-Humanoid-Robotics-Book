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
