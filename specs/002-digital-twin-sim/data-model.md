# Data Model: Module 2 - The Digital Twin (Gazebo & Unity)

## 1. Digital Twin

Conceptual representation of a virtual replica of a physical robot/system.

*   **Name**: String (e.g., "Humanoid Robot Digital Twin")
*   **Platform**: Enum (Gazebo, Unity)
*   **PhysicalModelRef**: Reference to Humanoid Robot (underlying model)
*   **IntegratedSensors**: List of Sensor references
*   **Description**: String (Purpose and capabilities of the digital twin)

## 2. Gazebo Simulation

Conceptual representation of the Gazebo environment.

*   **WorldFile**: String (Path to .world file)
*   **RobotModelRef**: Reference to Humanoid Robot (model used)
*   **PhysicsEngine**: String (e.g., "ODE", "Bullet", "DART")
*   **SimulatedSensors**: List of Sensor references
*   **ROS2Integration**: Boolean (Indicates if ROS 2 is used for control/data)

## 3. Unity Scene

Conceptual representation of the Unity environment.

*   **SceneFile**: String (Path to .unity scene file)
*   **RobotModelRef**: Reference to Humanoid Robot (model used)
*   **RenderingPipeline**: String (e.g., "HDRP", "URP", "Built-in")
*   **InteractiveElements**: List of String (e.g., "UI controls", "manipulable objects")
*   **ROS2Integration**: Boolean (Indicates if ROS 2 is used via ROS-TCP-Connector)

## 4. Sensor

Conceptual representation of a simulated sensor.

*   **Type**: Enum (LiDAR, Depth Camera, IMU)
*   **Name**: String (Unique identifier, e.g., "front_lidar")
*   **MountPoint**: String (Location on robot model, e.g., "head_link")
*   **DataFormat**: String (Expected output format, e.g., "PointCloud2", "Image", "Imu")
*   **SimulationPlatform**: Enum (Gazebo, Unity)
*   **RobotRef**: Reference to Humanoid Robot

## 5. Humanoid Robot

Conceptual representation of the robot model used in simulations.

*   **Name**: String (e.g., "MyHumanoid")
*   **Description**: String (Brief overview of the robot's design)
*   **URDFPath**: String (Path to URDF/SDF file describing the robot)
*   **Joints**: List of Joint (from URDF)
*   **Links**: List of Link (from URDF)
