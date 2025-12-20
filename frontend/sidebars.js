/**
 * Creating a sidebar enables you to:
 * - create an ordered group of docs
 * - render a sidebar for each doc of that group
 * - provide next/previous navigation
 *
 * The sidebars can be generated from the filesystem, or explicitly defined here.
 *
 * Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: ROS 2 Nervous System',
      items: [
        'ros2-nervous-system/chapter1_basics',
        'ros2-nervous-system/chapter2_rclpy_bridge',
        'ros2-nervous-system/chapter3_humanoid_urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'digital-twin-sim/chapter1_gazebo_physics',
        'digital-twin-sim/chapter2_unity_rendering',
        'digital-twin-sim/chapter3_sensor_integration',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'isaac-robot-brain/chapter1_isaac_sim',
        'isaac-robot-brain/chapter2_isaac_ros_vslam',
        'isaac-robot-brain/chapter3_nav2_humanoid',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'vla-humanoid-robotics/chapter1_whisper',
        'vla-humanoid-robotics/chapter2_llm_planning',
        'vla-humanoid-robotics/chapter3_capstone',
      ],
    },
  ],
};

module.exports = sidebars;
