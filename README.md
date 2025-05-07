# Stardew Controls

This is a small project to demonstrate the controls subject I am learning at university using a clone of Stardew Valley's fishing minigame.

## Project Overview

The project aims to replicate the fishing minigame from Stardew Valley to explore and understand various control mechanisms and their implementations.

## Getting Started

To get started with the project, clone the repository and follow the instructions in the setup guide. You will need to install [Pygame](https://www.pygame.org/news) to run the game.

## Build and Sourcing Instructions for ROS 2 Humble

To build and source the project with ROS 2 Humble, follow these steps:

1. **Install ROS 2 Humble**
   - Follow the official installation guide for ROS 2 Humble: [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html).

2. **Build the Workspace**
   - Navigate to the workspace root and build the project:
     ```bash
     rosdep install -i --from-path . --rosdistro humble -y
     colcon build
     ```

3. **Source the Workspace**
   - Source the setup file to overlay the workspace on your ROS 2 environment:
     ```bash
     source install/setup.bash
     ```


4. **Run the Project**
   - After sourcing, you can run the project using:
   ```bash
   ros2 run ros_fishing fishing
   ```

## Published Action and Topics

### Published Action
- **Action Name**: `fish`
  - **Type**: `ros_fishing_interfaces/action/Fishing`
  - **Description**: This action starts the fishing game and provides feedback on the progress of the game.

### Published Topics
- **`/fish_pct/id_<goal_id>`**
  - **Type**: `std_msgs/Float64`
  - **Description**: Publishes position of the fish from 0.0 to 1.0

- **`/bar_pct/id_<goal_id>`**
  - **Type**: `std_msgs/Float64`
  - **Description**: Publishes position of the fishing bar from 0.0 to 1.0

- **`/fish_in_fishing_bar/id_<goal_id>`**
  - **Type**: `std_msgs/Bool`
  - **Description**: Publishes whether the fish is within the fishing bar.

### Subscribed Topics:
- **`/fishing_button_state/id_<goal_id>`**
  - **Type**: `std_msgs/Bool`
  - **Description**: Subscribes to the button state for controlling the fishing bar.
