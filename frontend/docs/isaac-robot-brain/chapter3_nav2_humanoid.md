# Chapter 3: Path Planning for Humanoid Movement with Nav2

With a map of the environment from our VSLAM system, the next step towards true autonomy is **Path Planning**. This chapter will guide you through configuring and using **Nav2**, the standard ROS 2 navigation stack, to enable our humanoid robot to navigate intelligently from a starting point to a goal while avoiding obstacles.

## 3.1 Introduction to the Nav2 Stack

Nav2 is a powerful, highly configurable navigation framework. It is not a single node, but a collection of servers, lifecycle managers, and plugins that work together to achieve robust navigation.

### Key Components:

- **BT Navigator**: Coordinates the entire navigation process using Behavior Trees.
- **Planner Server**: Provides a global path from the robot's current position to a goal (e.g., using A\* or Dijkstra's algorithm).
- **Controller Server**: Generates velocity commands to follow the global path locally, while avoiding immediate obstacles.
- **Costmaps**: 2D or 3D representations of the environment that store information about where the robot can and cannot go. There are typically a global costmap for the overall path and a local costmap for immediate obstacle avoidance.

## 3.2 Configuring Nav2 for a Humanoid Robot

Configuring Nav2 is the most critical part of this process. The configuration needs to be tailored to the specific kinematics and dynamics of your robot.

1.  **Create a Configuration File**: Create a new file `isaac_ros_ws/src/module3_isaac_examples/config/nav2.yaml`.
2.  **Define Plugins and Parameters**: This YAML file will define which plugins to use for the controller, planner, and costmaps, and set their parameters.

```yaml
# isaac_ros_ws/src/module3_isaac_examples/config/nav2.yaml
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    # Example using DWB as the local planner/controller
    FollowPath:
      plugin: 'dwb_core::DWBLocalPlanner'
      # Humanoid specific settings:
      # These parameters are highly dependent on your robot's capabilities
      max_vel_x: 0.5
      min_vel_x: -0.5
      max_vel_y: 0.0 # Assuming no strafing
      min_vel_y: 0.0
      max_vel_theta: 1.0
      acc_lim_x: 2.5
      acc_lim_theta: 3.2

planner_server:
  ros__parameters:
    use_sim_time: True
    # Example using SmacPlanner as the global planner
    GridBased:
      plugin: 'nav2_smac_planner/SmacPlanner'
      tolerance: 0.5
      downsample_costmap: false

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: True
      plugin: 'nav2_costmap_2d::StaticLayer'
      # ... other costmap parameters

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: True
      # ... costmap parameters
```

## 3.3 Launching the Nav2 Stack

A launch file is needed to bring up all the different Nav2 servers and the lifecycle manager that controls them.

Create a new file `isaac_ros_ws/src/module3_isaac_examples/launch/nav2.launch.py`:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Launch the Nav2 stack."""

    # Get the share directory
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_share = get_package_share_directory('module3_isaac_examples')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(pkg_share, 'maps', 'my_map.yaml'))
    nav2_params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_share, 'config', 'nav2.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Omniverse Isaac Sim) clock'),
        DeclareLaunchArgument(
            'map',
            default_value=map_yaml_file,
            description='Full path to map file to load'),
        DeclareLaunchArgument(
            'params_file',
            default_value=nav2_params_file,
            description='Full path to the ROS2 parameters file to use'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml_file,
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_file}.items(),
        ),
    ])
```

## 3.4 Setting Goals in RViz2

Once Nav2 is running, you can use RViz2 to set a navigation goal.

1.  **Launch RViz2 with Nav2 Config**: Nav2 provides a default RViz config. `ros2 launch nav2_bringup rviz_launch.py`
2.  **Set the 2D Pose Estimate**: Before you can navigate, you must tell the robot where it is on the map. Use the "2D Pose Estimate" button in the RViz2 toolbar to click on the map at the robot's approximate location and orientation.
3.  **Set the Navigation Goal**: Use the "Nav2 Goal" button in the toolbar to click a destination on the map. You should see a global path (a blue line) appear, and the robot should begin to move, following the path and adjusting for any local obstacles.

By completing this chapter, you will have implemented a full navigation stack, enabling your humanoid robot to move autonomously through its simulated world. This is a critical milestone in developing intelligent robotic systems.
