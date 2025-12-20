# Data Model: Module 4 - Vision-Language-Action (VLA)

## 1. Humanoid Robot

Conceptual representation of the robot model used in VLA tasks.

*   **Name**: String (e.g., "VLA Humanoid")
*   **Description**: String (Brief overview of the robot's design and capabilities)
*   **Platform**: Enum (Simulated, Physical)
*   **ActionServerRefs**: List of ROS 2 Action Server references (actions it can perform)

## 2. OpenAI Whisper

Conceptual representation of the speech-to-text component.

*   **Model**: String (e.g., "base", "small", "medium", "large")
*   **APIIntegration**: String (e.g., "REST API", "SDK")
*   **Input**: String (Audio stream/file)
*   **Output**: String (Transcribed text)

## 3. Cognitive Planning Module

Conceptual representation of the natural language to action component.

*   **Name**: String (e.g., "LLM Planner")
*   **Description**: String (Details about how natural language is interpreted)
*   **Input**: String (Natural language command)
*   **Output**: List of ROS 2 Action Sequence
*   **LLMIntegration**: String (e.g., "GPT-4", "Gemini")
*   **KnowledgeBase**: String (Contextual information for planning)

## 4. ROS 2 Action Sequence

Conceptual representation of a series of robot actions.

*   **ActionID**: Unique String (e.g., "navigate_to_ball", "pick_up_object")
*   **Description**: String (Purpose of the action)
*   **ActionType**: String (e.g., "MoveBaseGoal", "GripperCommand")
*   **Parameters**: Object (Key-value pairs for action execution)
*   **ExecutionStatus**: Enum (Pending, In Progress, Completed, Failed)

## 5. VLA Pipeline

Conceptual representation of the end-to-end Vision-Language-Action system.

*   **Name**: String (e.g., "Humanoid VLA System")
*   **Description**: String (Overall flow of voice command to robot action)
*   **Components**: List of component references (Whisper, Planning Module, Robot)
*   **Input**: String (Voice command)
*   **Output**: String (Robot performing task)
*   **Status**: Enum (Idle, Processing, Executing, Error)
