# Quickstart Guide: Module 4 - Vision-Language-Action (VLA)

This guide provides quick setup instructions and how to run the examples for Module 4, focusing on integrating OpenAI Whisper for voice commands, LLMs for cognitive planning, and ROS 2 for action sequences in a Vision-Language-Action (VLA) pipeline.

## 1. Prerequisites

*   **Operating System**: Linux (Ubuntu recommended for ROS 2).
*   **ROS 2 Installation**: A working installation of ROS 2 Humble or Iron.
*   **Python**: Python 3.8+ (for Whisper SDK, LLM clients, ROS 2 Python nodes).
*   **OpenAI API Key**: Required for OpenAI Whisper and potentially for LLMs (if using OpenAI's models).
*   **Other LLM API Keys**: If using other LLM providers (e.g., Gemini), their respective API keys.
*   **VLA Pipeline Environment**: A suitable environment with ROS 2, Python, and necessary drivers for simulated humanoid robot (e.g., Isaac Sim or Gazebo).

## 2. Setting Up Your ROS 2 + VLA Workspace

1.  **Create a ROS 2 Workspace (if not already existing)**:
    ```bash
    mkdir -p ~/vla_ws/src
    cd ~/vla_ws/src
    ```

2.  **Clone Module 4 Examples**:
    ```bash
    # Assuming your module examples are in a Git repository
    # git clone <YOUR_REPO_URL>/module4_vla_examples.git
    # For now, ensure your local directory structure matches `vla_ws/src/module4_vla_examples/`
    ```

3.  **Install Python Dependencies**:
    ```bash
    # Ensure you have the `openai-whisper` package installed.
    # If using `ros2_audio_common`, clone it into your workspace:
    # cd ~/vla_ws/src
    # git clone https://github.com/ros-realtime/ros2_audio_common.git
    
    cd ~/vla_ws
    rosdep install --from-paths src --ignore-src -r -y
    pip install -U openai-whisper
    # Additional LLM client libraries might be needed depending on your chosen LLM (e.g., openai, google-generativeai)
    ```

4.  **Build the Workspace**:
    ```bash
    colcon build --packages-up-to module4_vla_examples
    ```

5.  **Source the Workspace**:
    ```bash
    source install/setup.bash
    ```

6.  **Configure API Keys**:
    Set your API keys as environment variables or in a secure configuration file.
    ```bash
    export OPENAI_API_KEY="sk-YOUR_OPENAI_API_KEY"
    # export GEMINI_API_KEY="YOUR_GEMINI_API_KEY"
    ```

## 3. Running VLA Examples

### 3.1 Voice Command Processing with OpenAI Whisper (Chapter 1)

1.  **Launch Audio Capture Node**:
    ```bash
    # Assuming ros2_audio_common is installed and built
    ros2 launch audio_capture audio_capture.launch.py default_microphone:="/dev/audio_device" # Replace with your microphone
    ```
2.  **Launch ROS 2 Whisper Node**:
    ```bash
    ros2 run module4_vla_examples whisper_node
    # You can also use a launch file if you create one
    ```
3.  **Speak a Command**: Speak into your configured microphone.
4.  **Observe Transcription**: The transcribed text will be published on `/voice_commands`. You can monitor with:
    ```bash
    ros2 topic echo /voice_commands
    ```

### 3.2 Cognitive Planning (LLM to ROS 2 Actions) (Chapter 2)

1.  **Launch ROS 2 LLM Planner Node**:
    ```bash
    ros2 run module4_vla_examples llm_planner_node
    ```
2.  **Launch Humanoid Action Servers (Mock)**:
    ```bash
    ros2 run module4_vla_examples humanoid_action_servers
    ```
3.  **Publish Text Command**: Send a natural language command to the LLM planner.
    ```bash
    ros2 topic pub /voice_commands std_msgs/msg/String "data: 'pick up the blue block'"
    # Or "data: 'go to the kitchen'"
    ```
4.  **Observe Action Execution**: The LLM planner will dispatch actions, and the mock action servers will report execution.

### 3.3 Autonomous Humanoid Task Execution (Capstone) (Chapter 3)

1.  **Launch Simulated Humanoid Robot Environment**:
    *   Start your chosen simulator (e.g., Isaac Sim, Gazebo) with your humanoid robot in an interactive environment.
    *   Ensure necessary ROS 2 bridges are active (e.g., `ros_bridge` in Isaac Sim).
2.  **Launch End-to-End VLA Pipeline**:
    ```bash
    ros2 launch module4_vla_examples vla_pipeline.launch.py
    ```
3.  **Issue Voice Command**: Speak a command directly to the humanoid robot (e.g., "Robot, bring me the red cube").
4.  **Observe Autonomous Behavior**: Monitor the robot's actions in the simulator and ROS 2 topics for task execution.

## 4. Troubleshooting

*   **API Key errors**: Ensure environment variables (`OPENAI_API_KEY`, etc.) are correctly set.
*   **ROS 2 node issues**: Verify ROS 2 workspace sourcing and package builds (`colcon build`).
*   **Whisper model loading**: If Whisper fails, check Python environment and GPU drivers (if using GPU version).
*   **LLM response latency**: Check network connection and LLM API rate limits.
*   **Nav2 planning failures**: (If integrated for navigation actions) Ensure map is valid, robot footprint is correctly configured, and controller parameters are tuned.
