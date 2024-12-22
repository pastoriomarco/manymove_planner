# ManyMove Planner

The `manymove_planner` project provides a robust framework for planning and executing robotic manipulator movements using ROS 2 and MoveIt 2. It supports diverse robots, including the UF850, Panda, and Lite series, with modular, configurable components that cater to various motion planning and execution needs.

## Features

- **Action-Based Motion Control**:
  - Single move goals with position or Cartesian targets.
  - Sequential move goals for executing a series of movements.
- **Integration with MoveIt 2**:
  - Supports motion planning, Cartesian path planning, and time parameterization.
- **Flexible Planner Options**:
  - Choose between MoveItCppPlanner and MoveGroupPlanner for motion planning.
- **Customizable Parameters**:
  - Control velocity, acceleration, Cartesian motion constraints, and more via ROS parameters or launch files.

## Architecture Overview

1. **Planner Interface**:
   - Defines abstract methods for planning, execution, and time parameterization.
   - Implemented by `MoveItCppPlanner` and `MoveGroupPlanner`.
2. **Action Server**:
   - Handles incoming motion requests and delegates tasks to the chosen planner.
   - Provides feedback and manages action lifecycle.
3. **Action Server Node**:
   - Configures and initializes the appropriate planner and action server.
   - Supports runtime parameter configuration.
4. **Client Node**:
   - Demonstrates sending single and sequence motion goals.

## Requirements

- ROS 2 (Foxy or later)
- MoveIt 2
- Required ROS 2 dependencies:
  - `rclcpp`, `moveit_core`, `geometry_msgs`, `action_msgs`, `control_msgs`, `rosidl_default_generators`

## Installation

### Dependencies
Ensure ROS 2 and MoveIt 2 are installed. Follow the [MoveIt 2 installation guide](https://moveit.ros.org/install/) for your platform.

### Build Instructions
1. Clone the repository into your workspace:
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

### Launching the Action Server

- **UF850 Manipulator**:
  ```bash
  ros2 launch manymove_planner uf850_action_server_node.launch.py
  ```

- **Lite Series Manipulator**:
  ```bash
  ros2 launch manymove_planner lite_action_server_node.launch.py
  ```

- **Panda Manipulator**:
  ```bash
  ros2 launch manymove_planner panda_action_server_node.launch.py
  ```

- **Custom Configuration**:
  ```bash
  ros2 launch manymove_planner manymove_planner.launch.py
  ```

### Client Example

- **Single Goal**:
  Sends a single motion request (e.g., moving to a pose or joint target).
  ```bash
  ros2 run manymove_planner client_example_node
  ```

- **Sequence of Goals**:
  Demonstrates planning and executing a series of moves with feedback.

### Configuration Parameters

- **Planner Type**:
  - Choose `moveitcpp` or `movegroup` planners:
    ```bash
    --ros-args -p planner_type:=moveitcpp
    ```

- **Motion Planning Parameters**:
  - Define the planning group, base frame, and tool frame:
    ```bash
    --ros-args -p planning_group:=<group_name> -p base_frame:=<frame_name> -p tcp_frame:=<frame_name>
    ```

- **Velocity and Acceleration**:
  - Configure motion scaling factors:
    ```bash
    --ros-args -p velocity_scaling_factor:=0.5 -p acceleration_scaling_factor:=0.5
    ```

- **Cartesian Constraints**:
  - Limit Cartesian speeds:
    ```bash
    --ros-args -p max_cartesian_speed:=0.5
    ```

### Workflow

1. **Client Sends Request**:
   - A single or sequence motion goal is sent using the `MoveManipulator` or `MoveManipulatorSequence` action.
2. **Action Server Processes Request**:
   - Handles the goal, plans a trajectory, applies time parameterization, and executes it using the configured planner.
3. **Feedback and Results**:
   - For sequences, periodic feedback is provided during execution.
   - Final results are sent to the client on completion or failure.

## Core Components

### **Planner Interface**
Defines a common interface for motion planners, including methods for:
- Planning single and sequence trajectories.
- Executing trajectories with or without feedback.
- Applying time parameterization.

### **MoveItCppPlanner**
Uses MoveItCpp for planning and execution. Supports:
- Flexible configurations.
- Integration with MoveIt trajectory smoothing and parameterization.

### **Action Server**
Manages action lifecycle for single and sequential moves:
- Processes goals and cancellations.
- Executes planned trajectories using the configured planner.

### **Client Example Node**
Demonstrates:
- Sending single or sequence motion goals.
- Configuring movement parameters such as velocity and acceleration.

## Contribution

Contributions are welcome. Please submit issues or pull requests for new features or bug fixes.

## License

This project is licensed under the Apache 2.0 License. See the `LICENSE` file for details.
