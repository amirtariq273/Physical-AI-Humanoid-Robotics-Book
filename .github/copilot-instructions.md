# AI Coding Agent Instructions for Physical AI & Humanoid Robotics Project

This project builds a comprehensive educational book on Physical AI and Humanoid Robotics using Spec-Driven Development (SDD). The codebase consists of multiple ROS 2 workspaces, simulation environments, and a Docusaurus documentation site.

## Project Architecture

- **Modular Structure**: Four core modules (ROS 2 Nervous System, Digital Twin Simulation, Isaac Robot Brain, VLA Humanoid Robotics) with dedicated workspaces
- **Multi-Workspace Setup**: Separate ROS 2 workspaces (`ros2_ws/`, `isaac_ros_ws/`, `simulation_ws/`, `vla_ws/`) for different robotics stacks
- **Documentation Site**: Docusaurus frontend in `frontend/` serving content from `docs/`
- **Asset Management**: Isaac Sim assets in `isaac_sim_assets/`, Unity project in `unity_project/`
- **Spec-Driven Workflow**: All development follows specs → plans → tasks → implementation phases

## Key Workflows

### ROS 2 Development
```bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
# Run nodes: ros2 run <package> <node>
```

### Isaac ROS Development
```bash
cd isaac_ros_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
# Requires NVIDIA Isaac Sim environment setup
```

### Documentation Site
```bash
cd frontend
npm install
npm start  # Serves on localhost:3000
npm run build  # For production
```

### Simulation Workspaces
- `simulation_ws/`: Gazebo-based physics simulation
- `isaac_sim_assets/`: USD files for Isaac Sim environments
- `unity_project/`: Unity rendering integration

## Development Conventions

### Code Organization
- ROS 2 packages: `ws/src/module{X}_{topic}_examples/`
- Standard ROS 2 structure: `setup.py`, `package.xml`, `launch/`, `nodes/`, `urdf/`
- Python nodes use `rclpy` with standard Node class patterns

### Content Generation
- Follow Spec-Driven Development: spec.md → plan.md → tasks.md → implementation
- Record all AI interactions as Prompt History Records (PHRs) in `history/prompts/`
- Use `/sp.*` commands for planning phases

### Documentation Standards
- Markdown files in `frontend/docs/` with module/chapter structure
- Diagrams: Mermaid or ASCII only (no images)
- Teaching-first writing style for students/developers
- RAG chatbot compatibility: structure content for retrieval

### Quality Gates
- Constitution adherence: technical accuracy, clarity, runnable code
- All code examples must be executable on target platforms
- Manual testing of ROS nodes, simulations, and URDF parsing

## Integration Points

- **Cross-Module Dependencies**: ROS 2 common across modules, URDF shared between simulation and robotics
- **Asset Pipeline**: USD files from Isaac Sim, URDF for robot definitions
- **Build Dependencies**: rosdep for ROS packages, npm for frontend
- **Deployment**: GitHub Pages for documentation site

## Common Patterns

- **ROS 2 Node Template**: Standard publisher/subscriber/service patterns in `nodes/`
- **Launch Files**: XML-based launch configurations in `launch/`
- **URDF Structure**: Humanoid robot definitions with links/joints hierarchy
- **Spec Template**: User stories, requirements, success criteria format
- **PHR Format**: YAML frontmatter with id, stage, files, tests tracking

## Debugging Tips

- ROS 2: Check `ros2 topic list`, `ros2 node list` after sourcing workspace
- Isaac Sim: Verify GPU drivers and Isaac Sim installation
- Docusaurus: Clear node_modules if build fails
- Workspaces: Always source setup.bash in new terminals

## File Location Examples

- Module specs: `specs/001-ros2-nervous-system/spec.md`
- ROS 2 code: `ros2_ws/src/module1_ros2_examples/nodes/simple_publisher.py`
- Documentation: `frontend/docs/ros2-nervous-system/chapter1_basics.md`
- PHRs: `history/prompts/001-ros2-nervous-system/0001-generate-plan.plan.prompt.md`