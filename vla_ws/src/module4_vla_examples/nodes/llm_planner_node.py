import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
# Import your custom action messages
from module4_vla_examples.action import PickObject, NavigateTo

# Assuming you have an OpenAI API key set as an environment variable
# import openai 

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')
        self.declare_parameter('llm_api_key', 'YOUR_LLM_API_KEY') # For actual LLM integration
        self.declare_parameter('command_topic', '/voice_commands')
        self.declare_parameter('response_topic', '/robot_status')

        self.command_subscription = self.create_subscription(
            String,
            self.get_parameter('command_topic').get_parameter_value().string_value,
            self.command_callback,
            10)
        self.response_publisher = self.create_publisher(
            String,
            self.get_parameter('response_topic').get_parameter_value().string_value,
            10)

        # Action Clients
        self.pick_object_action_client = ActionClient(self, PickObject, 'pick_object')
        self.navigate_to_action_client = ActionClient(self, NavigateTo, 'navigate_to')

        self.get_logger().info("LLM Planner node started.")

    def command_callback(self, msg):
        command_text = msg.data
        self.get_logger().info(f"Received command: '{command_text}'")

        # --- LLM Integration (conceptual) ---
        # In a real scenario, you would send this command_text to an LLM API
        # and parse its response into a structured action plan.
        
        # Example: Simple keyword-based action generation (replace with LLM logic)
        if "pick up" in command_text.lower():
            object_name = command_text.lower().split("pick up")[1].strip()
            self.execute_pick_object(object_name)
        elif "go to" in command_text.lower():
            target_location = command_text.lower().split("go to")[1].strip()
            self.execute_navigate_to(target_location)
        else:
            self.get_logger().warn(f"Unknown command: '{command_text}'")
            self.publish_response(f"I don't understand '{command_text}'")

    def execute_pick_object(self, object_name):
        self.get_logger().info(f"Attempting to pick up: {object_name}")
        self.pick_object_action_client.wait_for_server()
        goal_msg = PickObject.Goal()
        goal_msg.object_name = object_name
        self.pick_object_action_client.send_goal_async(goal_msg).add_done_callback(
            lambda future: self.pick_object_goal_response_callback(future, object_name)
        )

    def pick_object_goal_response_callback(self, future, object_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal to pick up {object_name} rejected.")
            self.publish_response(f"Failed to pick up {object_name}.")
            return
        self.get_logger().info(f"Goal to pick up {object_name} accepted.")
        goal_handle.get_result_async().add_done_callback(
            lambda future: self.pick_object_result_callback(future, object_name)
        )

    def pick_object_result_callback(self, future, object_name):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"Successfully picked up {object_name}.")
            self.publish_response(f"Picked up {object_name}.")
        else:
            self.get_logger().error(f"Failed to pick up {object_name}: {result.message}")
            self.publish_response(f"Failed to pick up {object_name}: {result.message}")

    def execute_navigate_to(self, target_location):
        self.get_logger().info(f"Attempting to navigate to: {target_location}")
        self.navigate_to_action_client.wait_for_server()
        goal_msg = NavigateTo.Goal()
        # For simplicity, target_location is just a string here.
        # In a real system, you'd parse this into a geometry_msgs/PoseStamped.
        # Example: assume target_location could map to a predefined pose.
        goal_msg.target_pose.header.frame_id = "map"
        goal_msg.target_pose.pose.position.x = 1.0 # Placeholder
        goal_msg.target_pose.pose.position.y = 0.0 # Placeholder
        goal_msg.target_pose.pose.orientation.w = 1.0 # Placeholder
        
        self.navigate_to_action_client.send_goal_async(goal_msg).add_done_callback(
            lambda future: self.navigate_to_goal_response_callback(future, target_location)
        )

    def navigate_to_goal_response_callback(self, future, target_location):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal to navigate to {target_location} rejected.")
            self.publish_response(f"Failed to navigate to {target_location}.")
            return
        self.get_logger().info(f"Goal to navigate to {target_location} accepted.")
        goal_handle.get_result_async().add_done_callback(
            lambda future: self.navigate_to_result_callback(future, target_location)
        )

    def navigate_to_result_callback(self, future, target_location):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"Successfully navigated to {target_location}.")
            self.publish_response(f"Navigated to {target_location}.")
        else:
            self.get_logger().error(f"Failed to navigate to {target_location}: {result.message}")
            self.publish_response(f"Failed to navigate to {target_location}: {result.message}")

    def publish_response(self, message):
        msg = String()
        msg.data = message
        self.response_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    llm_planner_node = LLMPlannerNode()
    rclpy.spin(llm_planner_node)
    llm_planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
