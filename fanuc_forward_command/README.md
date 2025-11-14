# FANUC Forward Command Controller

This package provides a configuration for using the `forward_command_controller`
from `ros2_controllers` with FANUC robots. It includes a sine wave generator for
testing joint position control.

## Overview

The `forward_command_controller` is a generic controller that forwards command
signals directly to robot joints. This package configures it to work with FANUC
robots and provides a demonstration script that generates sine wave trajectories.

## Package Contents

- **Config**: Controller configuration files
- **Launch**: Launch files for the controller and demo
- **Scripts**: Python script for generating sine wave commands

## Usage

### 1. Complete Demo with Mock Hardware

Launch the complete demo with sine wave generation using mock hardware:

```bash
ros2 launch fanuc_forward_command fanuc_sine_wave_demo.launch.py \
  robot_model:=crx10ia use_mock:=true
```

### 2. Complete Demo with Real Hardware

Launch the complete demo with real FANUC robot:

```bash
ros2 launch fanuc_forward_command fanuc_sine_wave_demo.launch.py \
  robot_model:=crx10ia use_mock:=false robot_ip:=192.168.1.100
```

### 3. Modular Usage (Two Terminals)

**Terminal 1 - Start the robot system:**

```bash
ros2 launch fanuc_forward_command fanuc_forward_command.launch.py \
  use_mock:=true robot_model:=crx10ia
```

**Terminal 2 - Run the sine wave publisher:**

```bash
ros2 run fanuc_forward_command sine_wave_publisher.py
```

## Parameters

- `robot_model`: Robot model (default: crx10ia)
- `robot_ip`: IP address of the FANUC robot (default: 192.168.1.100)
- `use_mock`: Use mock hardware for testing (default: true)
- `launch_rviz`: Whether to launch RViz for visualization (default: true)

## Sine Wave Configuration

The sine wave parameters are hardcoded in the script:

- **Amplitude**: 10 degrees (0.174533 radians)
- **Frequency**: 0.1 Hz (10 second period)
- **Publish Rate**: 500 Hz (matches controller manager update rate)
- **Pattern**: `amplitude * (1.0 - cos(phase))` (trajectory-style)
- **Joints**: Applied to all 6 joints (J1-J6) simultaneously

## Controller Configuration

The `forward_position_controller` is configured to:

- Control all 6 FANUC joints (J1-J6)
- Use position interface
- Subscribe to `/forward_position_controller/commands` topic
- Accept `std_msgs::msg::Float64MultiArray` messages

## Message Format

The controller expects `Float64MultiArray` messages with 6 elements representing
joint positions:

- Index 0: J1 position
- Index 1: J2 position
- Index 2: J3 position
- Index 3: J4 position
- Index 4: J5 position
- Index 5: J6 position
