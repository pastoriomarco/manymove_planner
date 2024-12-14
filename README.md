# ManyMove Planner

The `manymove_planner` project is designed to provide a robust and flexible framework for planning and executing robotic manipulator movements using ROS 2 and MoveIt 2. It supports various robots, including the UF850, Panda, and Lite series, with advanced configuration options to cater to diverse operational requirements.

## Features

- Action server for single and sequential manipulator movement goals.
- Integration with MoveIt 2 for motion planning and control.
- Configurable launch files for different robot platforms.
- Support for advanced planning parameters like velocity scaling, acceleration limits, and Cartesian motion constraints.

## Requirements

- ROS 2 (Foxy or later)
- MoveIt 2
- Required ROS 2 dependencies:
  - `rclcpp`
  - `moveit_core`
  - `moveit_ros_planning_interface`
  - `geometry_msgs`
  - `action_msgs`
  - `control_msgs`
  - `rosidl_default_generators`
  - `rosidl_default_runtime`

## Installation

### Dependencies
Ensure that you have the required ROS 2 and MoveIt 2 packages installed. Refer to the ROS 2 and MoveIt 2 installation guides for your platform.

### Build Instructions
1. Clone the repository into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url> manymove_planner
   ```

2. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

3. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

### Launching the Nodes
The project provides several launch files for different robot configurations:

- **UF850 Manipulator**
  ```bash
  ros2 launch manymove_planner uf850_action_server_node.launch.py
  ```

- **Lite Series Manipulator**
  ```bash
  ros2 launch manymove_planner lite_action_server_node.launch.py
  ```

- **Panda Manipulator**
  ```bash
  ros2 launch manymove_planner panda_action_server_node.launch.py
  ```

- **Generic ManyMove Planner Node**
  ```bash
  ros2 launch manymove_planner manymove_planner.launch.py
  ```

### Example Usage

#### Sending Goals to the Action Server
You can interact with the action server using a client node:

1. Run the action server:
   ```bash
   ros2 launch manymove_planner uf850_action_server_node.launch.py
   ```

2. Send a goal using the client node:
   ```bash
   ros2 run manymove_planner client_example_node
   ```

The client node demonstrates how to send action goals and receive feedback/results.

### Configuration Parameters

The launch files support various parameters to customize robot behavior:

- **Velocity and Acceleration Scaling:**
  ```bash
  --ros-args -p velocity_scaling_factor:=0.5 -p acceleration_scaling_factor:=0.5
  ```

- **Cartesian Motion Constraints:**
  ```bash
  --ros-args -p max_cartesian_speed:=0.5
  ```

- **Planning Parameters:**
  ```bash
  --ros-args -p planning_group:=<group_name> -p base_frame:=<frame_name> -p tcp_frame:=<frame_name>
  ```

Refer to the individual launch files for additional parameters and their default values.

## Message and Action Definitions

### Actions

1. **MoveManipulator.action**
   ```
   geometry_msgs/Pose target_pose
   ---
   bool success
   ---
   string feedback
   ```

2. **MoveManipulatorSequence.action**
   ```
   geometry_msgs/Pose[] target_poses
   ---
   bool success
   ---
   string feedback
   ```

### Messages

1. **MovementConfig.msg**
   ```
   float64 velocity_scaling
   float64 acceleration_scaling
   ```

2. **MoveManipulatorGoal.msg**
   ```
   geometry_msgs/Pose goal_pose
   MovementConfig config
   ```

## Core Components

### **Core Motion Planning (`manymove_planner.cpp`)**
The `manymove_planner.cpp` file implements the motion planning logic, serving as the computational backbone of the `manymove_planner` package. It interacts with MoveIt 2 to compute motion plans based on given goals, constraints, and configurations.

### **Action Server (`action_server.cpp`)**
The `action_server.cpp` file defines an action server for handling motion requests. It delegates planning tasks to `manymove_planner.cpp` and manages the lifecycle of actions, including providing feedback and results.

### **Action Server Node (`action_server_node.cpp`)**
This node initializes the action server defined in `action_server.cpp` and integrates it into the ROS 2 environment. It handles parameter configuration, node spinning, and communication setup for the server.

### **Relationship and Workflow**
1. **Interaction Initiation**:
   - Clients send motion requests via actions (`MoveManipulator.action`, `MoveManipulatorSequence.action`).
   - `action_server_node.cpp` receives these requests and passes them to the `action_server.cpp`.

2. **Planning Execution**:
   - The action server delegates motion planning tasks to `manymove_planner.cpp`, passing along the target poses and any constraints.
   - `manymove_planner.cpp` computes a feasible trajectory using MoveIt 2 APIs and returns the plan.

3. **Result Handling**:
   - The computed plan is sent back to `action_server.cpp`, which monitors its execution and provides feedback to the client.
   - Upon completion or failure, the action server sends the result back to the client.

4. **Node Deployment**:
   - The `action_server_node.cpp` ensures the action server runs within the ROS 2 ecosystem, handling ROS-specific functionality such as spinning the node and managing parameters.

## Contribution

Feel free to contribute to this project by submitting issues or pull requests. For major changes, please open an issue first to discuss what you would like to change.

## License

This project is licensed under the Apache 2.0 License. See the `LICENSE` file for details.

