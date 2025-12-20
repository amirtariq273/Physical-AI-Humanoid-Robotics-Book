# Data Model: Module 1 - The Robotic Nervous System (ROS 2)

## 1. ROS 2 Node

Conceptual representation of a ROS 2 computational unit.

*   **Name**: String (Unique identifier within a graph)
*   **Purpose**: String (Brief description of its function)
*   **PublishedTopics**: List of ROS 2 Topic references (Topics it writes to)
*   **SubscribedTopics**: List of ROS 2 Topic references (Topics it reads from)
*   **ProvidedServices**: List of ROS 2 Service references (Services it offers)
*   **UsedServices**: List of ROS 2 Service references (Services it calls)

## 2. ROS 2 Topic

Conceptual representation of a data stream.

*   **Name**: String (Unique identifier)
*   **MessageType**: String (Type of data published on this topic, e.g., "std_msgs/String")
*   **PublisherNodes**: List of ROS 2 Node references
*   **SubscriberNodes**: List of ROS 2 Node references

## 3. ROS 2 Service

Conceptual representation of a request/reply mechanism.

*   **Name**: String (Unique identifier)
*   **ServiceType**: String (Type of request/response, e.g., "std_srvs/Trigger")
*   **ServiceProvider**: ROS 2 Node reference
*   **ServiceClient**: List of ROS 2 Node references

## 4. URDF (Unified Robot Description Format)

Conceptual representation of a robot's physical structure.

*   **FilePath**: String (Location of the URDF file)
*   **RootLink**: Link reference (The base link of the robot)
*   **Links**: List of Link references
*   **Joints**: List of Joint references

## 5. Link

Conceptual representation of a rigid body part in URDF.

*   **Name**: String (Unique identifier within URDF)
*   **Mass**: Float (Physical property)
*   **Inertia**: Object (Defines rotational inertia)
*   **Visual**: Object (Visual properties for rendering)
*   **Collision**: Object (Collision properties for physics)

## 6. Joint

Conceptual representation of a connection between two links in URDF.

*   **Name**: String (Unique identifier within URDF)
*   **Type**: Enum (revolute, prismatic, fixed, continuous, planar, floating)
*   **ParentLink**: Link reference
*   **ChildLink**: Link reference
*   **Axis**: Object (Rotation/translation axis)
*   **Limits**: Object (Joint limits)

## 7. rclpy

Conceptual representation of the Python client library.

*   **Purpose**: String ("Python interface for ROS 2 client library")
*   **KeyFunctions**: List of Strings (e.g., "create_node", "create_publisher", "create_subscription", "create_service", "create_client")
