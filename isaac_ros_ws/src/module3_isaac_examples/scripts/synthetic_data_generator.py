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
