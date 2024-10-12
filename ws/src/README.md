# RAG for Robotics

## Phase 1: Move around and construct memory

### Launch the simulation environment

    ros2 launch simulation simulation_gazebo.launch.py

### Launch the combined navigation + localization node

    ros2 launch rabbit_bringup combined_launch.py

** NOTE: pass in the initial pose of the robot from the rviz interface**

### Launch the construct memory 

    ros2 launch construct_memory construct_memory_launch.py

** Now move the robot around to construct the memory **

## Phase 2: Retrieval + Navigation

### Launch the user interface

    python3 src/retriever/retriever/user_interface.py






