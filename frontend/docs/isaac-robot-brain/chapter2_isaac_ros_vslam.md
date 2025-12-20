# Chapter 2: VSLAM & Navigation using Isaac ROS

In this chapter, we bridge the gap between simulation and autonomy by implementing a **Visual SLAM (Simultaneous Localization and Mapping)** pipeline. Using the powerful, hardware-accelerated packages from **Isaac ROS**, we will enable our simulated humanoid robot to build a map of its environment and track its own position within it.

## 2.1 Introduction to VSLAM

**VSLAM** is a technique that allows a robot to build a map of an unknown environment while simultaneously keeping track of its location within that map. It's a fundamental capability for autonomous mobile robots, as it solves the "chicken-and-egg" problem: to build a map you need to know where you are, but to know where you are, you need a map.

VSLAM systems typically use data from one or more cameras to identify features in the environment. By tracking these features across multiple frames, the system can infer the robot's motion and build a 3D representation of the world.

## 2.2 Overview of Isaac ROS VSLAM Packages

NVIDIA Isaac ROS includes optimized ROS 2 packages (Gems) that are specifically designed for high-performance robotics perception tasks. For VSLAM, we will primarily use the `isaac_ros_visual_slam` package.

### Key Features:

- **Performance**: It's GPU-accelerated, allowing for real-time performance even with high-resolution camera streams.
- **Robustness**: It's designed to be robust to challenging conditions like fast motion and dynamic environments.
- **ROS 2 Integration**: It's a standard ROS 2 package, making it easy to integrate into our existing ROS 2 workspace.

## 2.3 Setting up the Simulation Environment for VSLAM

To run VSLAM, we need a simulated environment and a robot with the right sensors.

1.  **Launch Isaac Sim**: Start Isaac Sim and load the `simple_room.usd` environment we created in the previous chapter.
2.  **Import the Robot**: Import your humanoid robot USD model into the scene.
3.  **Add a Stereo Camera**: VSLAM performs best with stereo cameras.
    - In Isaac Sim, select your robot's head link.
    - Go to `Create -> Isaac -> Sensors -> Camera`.
    - In the camera's 'Property' tab, set the `Stereo Offset` to a small value (e.g., `(0.06, 0)`), effectively creating a left and a right camera.
    - **Crucially**, ensure the ROS 2 camera helper is attached to publish the camera images and info to ROS topics. Go to `Add -> Isaac -> ROS -> Camera` and configure the topics.

## 2.4 Launching and Configuring the VSLAM Node

With the simulation ready, we can now launch the Isaac ROS VSLAM node. We'll create a ROS 2 launch file for this.

Create a new file `isaac_ros_ws/src/module3_isaac_examples/launch/vslam.launch.py`:

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Launch the Isaac ROS VSLAM node."""

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Omniverse Isaac Sim) clock'
    )

    # VSLAM Node
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam',
        name='visual_slam',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'denoise_input_images': False,
            'rectified_images': True,
            # Input topics for stereo cameras
            'left_image_topic': '/left/image_raw',
            'left_camera_info_topic': '/left/camera_info',
            'right_image_topic': '/right/image_raw',
            'right_camera_info_topic': '/right/camera_info',
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        visual_slam_node
    ])

```

**To Run this Example:**

1.  Start your Isaac Sim environment with the robot and stereo camera.
2.  Source your ROS 2 workspace: `source install/setup.bash`
3.  Launch the VSLAM node: `ros2 launch module3_isaac_examples vslam.launch.py`

## 2.5 Visualizing the Map and Pose in RViz2

`RViz2` is an essential tool for visualizing robotics data. We'll use it to see the map being built and the robot's estimated position.

1.  **Create an RViz2 Configuration**: Create a new file `isaac_ros_ws/src/module3_isaac_examples/rviz_config/vslam.rviz`.
2.  **Configure RViz2**:
    - Open RViz2: `rviz2`
    - Set the 'Global Options' -> 'Fixed Frame' to `odom`.
    - Add a 'Map' display and set the topic to `/map`.
    - Add a 'Path' display to visualize the robot's trajectory, listening to `/path`.
    - Add a 'TF' display to see the robot's coordinate frames.
    - Save the configuration to the `vslam.rviz` file you created.
3.  **Launch RViz2 with the config**: `rviz2 -d src/module3_isaac_examples/rviz_config/vslam.rviz`

As you drive the robot around in Isaac Sim, you should see the VSLAM node publishing a map and the robot's pose being updated in RViz2. This real-time feedback is crucial for debugging and understanding the system's performance.
