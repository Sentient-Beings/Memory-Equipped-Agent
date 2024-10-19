# Setup and Usage Guide

This guide outlines the process of setting up and using Agentic system for robotics, divided into two main phases.

## Phase 1: Construct Memory

In this phase, we set up the simulation environment, initialize localization and navigation, and construct the robot's memory.

### 1. Launch Simulation Environment

```bash
ros2 launch simulation simulation_gazebo.launch.py
```

### 2. Initialize Localization and Autonomous Navigation

```bash
ros2 launch rabbit_bringup combined_launch.py
```
**NOTE:** Rabbit is name of my robot :P

**NOTE:** Set the initial pose of the robot in the AMCL node using the rviz2 interface.

### 3. Start Memory Construction

```bash
ros2 run construct_memory construct_embedding
```

**IMPORTANT:** After launching, move the robot around to construct its memory.

## Phase 2: Retrieval and Navigation

Once the memory is constructed, proceed to this phase for retrieval and navigation.

### 1. Launch User Interface

```bash
python3 src/retriever/retriever/user_interface.py
```

### 2. Initiate Retrieval and Navigation

```bash
ros2 run retriever navigation 
```

## Additional Notes

- Ensure all dependencies are installed before running these commands.
- The robot's memory construction in Phase 1 is crucial for effective retrieval and navigation in Phase 2.
- Monitor the robot's performance and adjust parameters as needed for optimal results.
