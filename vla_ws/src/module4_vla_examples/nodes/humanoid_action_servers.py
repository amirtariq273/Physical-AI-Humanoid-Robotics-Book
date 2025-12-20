import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalStatus
from module4_vla_examples.action import PickObject, NavigateTo
import time

class HumanoidActionServers(Node):
    def __init__(self):
        super().__init__('humanoid_action_servers')
        self.get_logger().info("Humanoid Action Servers node started.")

        self._pick_object_action_server = ActionServer(
            self,
            PickObject,
            'pick_object',
            self.execute_pick_object_callback)
        
        self._navigate_to_action_server = ActionServer(
            self,
            NavigateTo,
            'navigate_to',
            self.execute_navigate_to_callback)
        
        self.get_logger().info("Action servers registered: pick_object, navigate_to")

    def execute_pick_object_callback(self, goal_handle):
        self.get_logger().info(f"Executing pick_object goal for '{goal_handle.request.object_name}'...")
        feedback_msg = PickObject.Feedback()
        feedback_msg.percentage_complete = 0.0
        feedback_msg.status_message = "Starting pick operation..."

        # Simulate pick operation
        for i in range(1, 6):
            feedback_msg.percentage_complete = float(i) / 5.0
            feedback_msg.status_message = f"Picking up {goal_handle.request.object_name} ({i*20}%)"
            self.get_logger().info(f"Feedback: {feedback_msg.status_message}")
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1) # Simulate work

        goal_handle.succeed()
        result = PickObject.Result()
        result.success = True
        result.message = f"Successfully picked up {goal_handle.request.object_name}."
        self.get_logger().info(f"Pick object goal for '{goal_handle.request.object_name}' succeeded.")
        return result

    def execute_navigate_to_callback(self, goal_handle):
        self.get_logger().info(f"Executing navigate_to goal for target pose...")
        feedback_msg = NavigateTo.Feedback()
        feedback_msg.distance_to_goal = 10.0 # Placeholder
        feedback_msg.current_pose.header.frame_id = "map"
        feedback_msg.current_pose.pose.position.x = 0.0 # Placeholder
        feedback_msg.current_pose.pose.position.y = 0.0 # Placeholder
        feedback_msg.current_pose.pose.orientation.w = 1.0 # Placeholder

        # Simulate navigation
        for i in range(1, 11):
            feedback_msg.distance_to_goal = 10.0 - float(i) # Simulate moving closer
            feedback_msg.current_pose.pose.position.x = float(i) # Simulate movement
            feedback_msg.status_message = f"Navigating, {feedback_msg.distance_to_goal:.1f}m to goal..."
            self.get_logger().info(f"Feedback: {feedback_msg.status_message}")
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1) # Simulate work
        
        goal_handle.succeed()
        result = NavigateTo.Result()
        result.success = True
        result.message = "Successfully navigated to target pose."
        self.get_logger().info("Navigate to goal succeeded.")
        return result

def main(args=None):
    rclpy.init(args=args)
    action_servers_node = HumanoidActionServers()
    rclpy.spin(action_servers_node)
    action_servers_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
