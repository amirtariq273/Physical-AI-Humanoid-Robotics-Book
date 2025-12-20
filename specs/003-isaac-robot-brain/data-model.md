# Data Model: Module 3 - The AI-Robot Brain (NVIDIA Isaac™)

## 1. Humanoid Robot

Conceptual representation of the robot model used in simulations.

*   **Name**: String (e.g., "Isaac Humanoid")
*   **Description**: String (Brief overview of the robot's design)
*   **SimulationPlatform**: Enum (Isaac Sim)
*   **Capabilities**: List of Strings (e.g., "Movement", "Perception", "Navigation")
*   **URDF/USDPath**: String (Path to robot model description file)

## 2. Isaac Sim Environment

Conceptual representation of the NVIDIA Isaac Sim environment.

*   **Name**: String (e.g., "Warehouse Scene")
*   **Description**: String (Details about the simulated world)
*   **USDPath**: String (Path to Universal Scene Description file)
*   **SyntheticDataGeneration**: Boolean (Indicates if data generation is active)
*   **IntegratedRobots**: List of Humanoid Robot references

## 3. VSLAM Pipeline

Conceptual representation of the Visual SLAM module.

*   **Name**: String (e.g., "Isaac ROS VSLAM")
*   **Description**: String (Details about the VSLAM algorithm/implementation)
*   **Input**: String (e.g., "Stereo Camera Feeds", "Depth Camera Feeds")
*   **Output**: String (e.g., "Robot Pose", "Occupancy Grid Map")
*   **Framework**: String (e.g., "Isaac ROS")
*   **AccuracyMetrics**: Object (e.g., "Localization Error", "Mapping Drift")

## 4. Navigation Stack (Nav2)

Conceptual representation of the ROS 2 navigation framework.

*   **Name**: String (e.g., "ROS 2 Nav2 Stack")
*   **Description**: String (Details about Nav2's components and functionality)
*   **Input**: String (e.g., "Occupancy Grid Map", "Robot Pose", "Goal Pose")
*   **Output**: String (e.g., "Robot Velocity Commands")
*   **PathPlanningAlgorithms**: List of Strings (e.g., "Dijkstra", "A*", "TEB Local Planner")
*   **IntegratedRobots**: List of Humanoid Robot references
