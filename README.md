
# RoboTic-Tac-Toe Project

## Overview
This project implements a TicTacToe game where a MyCobot 280 robotic arm plays against human players. The system includes perception, decision-making, and motion planning components, all integrated and controlled via a web application. The entire setup is containerized using Docker for easy deployment.

## Table of Contents
1. [Project Setup](#project-setup)
2. [System Architecture](#system-architecture)
3. [How to Run](#how-to-run)
4. [Web Application](#configure-web-application)
5. [References](#references)

## Project Setup
### Requirements
- **Hardware**: MyCobot 280 robotic arm
- **Software**: ROS2, Moveit2, Docker, Gazebo, Python 3

### Installation
1. **Clone the Repository**
   ```bash
   git clone https://github.com/MiguelSolisSegura/robotic_tactoe
   cd robotic_tactoe
   ```
2. **Setup ROS2 and Moveit2**
   - Follow the instructions [here](https://moveit.ros.org/install-moveit2/binary/)

## System Architecture
### Overview
The project consists of several key components:
- **Motion Planning**: Controls the robotic arm to make moves.
- **Perception System**: Detects the game board and player's moves.
- **Decision-Making Algorithm**: Determines the robot's best next move.
- **Web Application**: Manages and monitors the game.

## How to Run
Run the following commands in the order shown here.

1. **Simulation**
```bash
cd ~/ros2_ws/src/mycobot_simulation/scripts
./launch_mycobotarm.sh
```

2. **ROS2 Nodes**
```bash
ros2 launch moveit_planning main_sim.launch.py
```

3. **Web Application**
```bash
ros2 launch moveit_planning main_sim.launch.py
```

## Configure Web Application
First, obtain the webpage address and navigate to the `html` folder.
1. **Get Webpage Address**
   ```bash
   webpage_address
   ```
   
Fill the ROS Bridge address with the output of the following command.

2. **Run Containers**
   ```bash
   rosbridge_address
   ```

## References
- [MyCobot 280 Documentation](https://www.elephantrobotics.com/en/mycobot-en/)
- [Moveit2 Documentation](https://moveit.ros.org/)
