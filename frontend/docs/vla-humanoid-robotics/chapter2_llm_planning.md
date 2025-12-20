# Chapter 2: Cognitive Planning: LLMs → ROS 2 Action Sequences

Building on the ability to understand voice commands from the previous chapter, this chapter delves into **Cognitive Planning**. We will explore how **Large Language Models (LLMs)** can be leveraged to translate high-level natural language instructions into structured ROS 2 action sequences, enabling our humanoid robot to perform complex tasks autonomously.

## 2.1 Introduction to Cognitive Architectures for Robotics

Robotics is moving beyond simple reactive behaviors to more cognitive capabilities. A cognitive architecture allows a robot to reason, plan, learn, and adapt in complex environments. LLMs are emerging as powerful components within these architectures due to their ability to understand and generate human-like text.

## 2.2 LLMs for Robot Task Planning

LLMs can act as a "brain" for the robot, translating abstract goals into concrete steps. The process typically involves:

1.  **Prompt Engineering**: Crafting effective prompts that guide the LLM to generate desired action sequences.
2.  **Contextual Information**: Providing the LLM with information about the robot's capabilities, its environment, and available tools/actions.
3.  **Action Sequence Generation**: The LLM outputs a sequence of discrete, executable robot actions.

## 2.3 Defining Custom ROS 2 Actions

For complex tasks, we often need custom ROS 2 Actions. An Action is a request-response mechanism for long-running goals, providing feedback along the way.

Create a new directory `vla_ws/src/module4_vla_examples/action/` and define your custom actions (e.g., `PickObject.action`, `NavigateTo.action`).

**`vla_ws/src/module4_vla_examples/action/PickObject.action`**:

```
# Goal
string object_name
---
# Result
bool success
string message
---
# Feedback
float32 percentage_complete
string status_message
```

**`vla_ws/src/module4_vla_examples/action/NavigateTo.action`**:

```
# Goal
geometry_msgs/PoseStamped target_pose
---
# Result
bool success
string message
---
# Feedback
geometry_msgs/PoseStamped current_pose
float32 distance_to_goal
```

Remember to add these actions to your `package.xml` and `CMakeLists.txt` (or equivalent for Python packages) so ROS 2 can build them.

## 2.4 Implementing a ROS 2 LLM Planner Node

This node will receive text commands (e.g., from our Whisper node), query an LLM (e.g., via an API), and then dispatch the appropriate ROS 2 Actions.

Create `vla_ws/src/module4_vla_examples/nodes/llm_planner_node.py`:

```python
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

```

## 2.5 Implementing Example ROS 2 Action Servers

To make the `LLMPlannerNode` functional for testing, we need mock action servers that respond to `PickObject` and `NavigateTo` goals.

Create `vla_ws/src/module4_vla_examples/nodes/humanoid_action_servers.py`:

```python
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

```

This comprehensive chapter provides the foundation for building intelligent, language-driven robot behaviors, enabling the humanoid to understand and execute commands based on cognitive planning.
