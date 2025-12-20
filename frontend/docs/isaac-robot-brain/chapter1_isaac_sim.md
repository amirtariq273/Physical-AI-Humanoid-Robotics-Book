# Chapter 1: Photorealistic Simulation & Synthetic Data with Isaac Sim

Welcome to the cutting edge of robotics AI. This chapter will guide you through the fundamentals of using **NVIDIA Isaac Sim**, a powerful robotics simulation platform, to create photorealistic environments and generate high-quality synthetic data for training and testing your humanoid robot's AI.

## 1.1 Introduction to Isaac Sim

**NVIDIA Isaac Sim** is a scalable robotics simulation application and synthetic data generation tool that powers the NVIDIA Isaac™ platform. Built on NVIDIA Omniverse™, it leverages the full power of NVIDIA's RTX rendering, material definition language (MDL), and physics simulation (PhysX) to create stunningly realistic, physically-accurate virtual worlds.

### Key Capabilities:

- **Photorealistic Rendering**: With RTX technology, you can simulate realistic lighting, shadows, and materials, creating environments that are visually indistinguishable from reality.
- **Accurate Physics Simulation**: Isaac Sim uses NVIDIA PhysX 5 to simulate rigid and soft body dynamics, collisions, and gravity, ensuring that your robot interacts with the world in a physically plausible way.
- **Synthetic Data Generation (SDG)**: It provides a comprehensive suite of tools to generate labeled data at scale, including camera images, depth maps, bounding boxes, and segmentation masks, which are crucial for training deep learning models.
- **ROS/ROS 2 Integration**: Isaac Sim seamlessly integrates with ROS and ROS 2, allowing you to connect your AI-driven robot control stack directly to the simulated robot.

## 1.2 Setting Up a New Isaac Sim Project

Before you begin, ensure you have the NVIDIA Omniverse Launcher installed and have downloaded the Isaac Sim application.

1.  **Open Omniverse Launcher**: Launch the application.
2.  **Go to the 'Exchange' Tab**: Find and install "Isaac Sim".
3.  **Launch Isaac Sim**: Once installed, you can launch Isaac Sim from the 'Library' tab.
4.  **Create a New Project**: On the Isaac Sim startup screen, you will be prompted to create or open a project. It's recommended to store your projects in a designated folder on your local drive.

## 1.3 Importing and Configuring a Humanoid Robot

Isaac Sim uses the **Universal Scene Description (USD)** format for describing scenes and models. You can import existing robot models in URDF or MJCF format, which will be automatically converted to USD.

1.  **Navigate to the Content Browser**: In Isaac Sim, the Content Browser is your portal to all assets.
2.  **Import Robot**: Go to `Create -> Isaac -> Import Robot`.
3.  **Select your URDF file**: Locate the `humanoid.urdf` file from `ros2_ws/src/module1_ros2_examples/urdf/`. This will trigger the URDF importer extension.
4.  **Configure Import Settings**:
    - **Physics Engine**: Ensure "PhysX" is selected.
    - **Joint Drive Type**: For humanoid robots, 'Position' or 'Velocity' are common choices for controlling joints.
    - **Fix Base**: If you want the robot to be stationary, check this box. For a humanoid, you'll likely want this unchecked.
5.  **Finalize Import**: Click "Import". Isaac Sim will create a new USD file for your robot and add it to your scene.

## 1.4 Building a Basic Simulation Environment

A realistic environment is crucial for meaningful simulation.

1.  **Create a Ground Plane**: Go to `Create -> Mesh -> Plane`. Scale it up to serve as the ground.
2.  **Apply Materials**:
    - Open the 'Materials' browser (`Window -> Browsers -> Materials`).
    - Drag and drop a material, like 'Concrete' or 'Metal', onto the ground plane and your robot to give them a realistic appearance.
3.  **Add Lighting**:
    - Go to `Create -> Light -> Dome Light`. This provides ambient, environmental lighting.
    - You can adjust its intensity and color in the 'Property' tab.
4.  **Add Physics**:
    - Select the ground plane and, in the 'Property' tab, click `Add -> Physics -> Collider`.
    - Select your robot, and ensure that all its links have the `Rigid Body` and `Collider` components. The importer should have done this automatically.

## 1.5 Principles of Synthetic Data Generation (SDG)

Synthetic data is a powerful tool for training AI models without the need for large, hand-labeled real-world datasets. Isaac Sim allows you to attach various synthetic data sensors to your robot.

### Common Data Types:

- **RGB Images**: Standard camera images.
- **Depth Maps**: Provides the distance of each pixel from the camera.
- **Bounding Boxes**: 2D or 3D boxes that enclose objects of interest.
- **Semantic Segmentation**: Pixel-wise labels where each pixel is assigned a class (e.g., 'wall', 'floor', 'robot').
- **Instance Segmentation**: Similar to semantic segmentation, but distinguishes between different instances of the same class.

## 1.6 Generating and Exporting Synthetic Data

Let's generate some camera data from our humanoid robot.

1.  **Create a Camera**: Select your robot's head link, then go to `Create -> Camera`. Position the camera to look out from the robot's perspective.
2.  **Add Synthetic Data Annotators**: With the camera selected, go to the 'Property' tab and click `Add -> Isaac -> Annotators`. You can add annotators for RGB, Depth, Bounding Box, and more.
3.  **Use the Synthetic Data Recorder**: Isaac Sim provides a built-in tool for capturing and exporting data.
    - Open the recorder via `Window -> Synthetic Data -> Recorder`.
    - Configure the output directory and the data types you want to save.
4.  **Run the Simulation**: Press the "Play" button at the top of the viewport. The simulation will start, and the recorder will begin saving data every frame.

### Example: Python Script for Data Generation

You can also control data generation programmatically using a Python script. This is powerful for automating data collection under varied conditions.

Create a new file `isaac_ros_ws/src/module3_isaac_examples/scripts/synthetic_data_generator.py`:

```python
import carb
from omni.isaac.kit import SimulationApp

# Configuration for the simulation
CONFIG = {
    "width": 1280,
    "height": 720,
    "headless": False, # Set to True for running without a GUI
}

# Start the simulation
simulation_app = SimulationApp(CONFIG)

from omni.isaac.core import World
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.replicator.core as rep

# Setup a new world
world = World()

# Add a robot to the scene - replace with your robot's USD path
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder.")
# Example using a pre-existing asset, replace with your imported robot
robot_usd_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
add_reference_to_stage(usd_path=robot_usd_path, prim_path="/World/robot")

# Add a simple cube to the scene for the robot to look at
cube = world.scene.add(
    VisualCuboid(
        prim_path="/World/cube",
        position=[0.5, 0, 0.25],
        size=0.1,
        color=[0.2, 0.2, 0.8],
    )
)

# Create a camera and attach it to the robot
camera = rep.create.camera(position=(0, 0, 1.0))

# Configure the replicator to write data
with rep.trigger.on_frame():
    with camera:
        rep.modify.pose(
            position=(0.2, 0, 0.5),
            look_at=cube.prim_path,
        )
        rep.capture.rgb()
        rep.capture.depth()
        rep.capture.semantic_segmentation()

# Create a writer to save the data
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir="_output", rgb=True, depth=True, semantic_segmentation=True)
writer.attach([camera])

# Run the simulation for a few frames
world.reset()
for i in range(100):
    world.step(render=True)
    if i == 50: # Example of moving the cube mid-simulation
        cube.set_world_pose(position=[0.5, 0.5, 0.25])

# Stop the simulation
simulation_app.close()

```

**To Run this Example:**

Save the script and execute it from the Isaac Sim script editor or from the terminal using the provided python executable in the Isaac Sim environment. The synthetic data will be saved to the `_output` directory.

This concludes the foundational chapter on Isaac Sim. You are now equipped to build realistic worlds and generate the data needed to train intelligent robot brains.
