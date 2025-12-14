# Quickstart Guide

This guide provides instructions for setting up the necessary environment to follow the examples and projects in the "Physical AI & Humanoid Robotics" book.

## System Requirements

- **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish) is the primary supported OS. While other Linux distributions might work, they are not officially supported.
- **Hardware**: A computer with at least 8GB of RAM and a dedicated GPU is recommended for running the simulations.

## Software Installation

### 1. ROS 2 Humble

Install ROS 2 Humble by following the official installation guide:
[https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)

### 2. Gazebo

Gazebo is installed by default with the `ros-humble-desktop` package. If you installed a different version of ROS 2, you can install Gazebo separately:
```bash
sudo apt-get install gazebo
```

### 3. Unity

Download and install Unity Hub from the official website:
[https://unity.com/download](https://unity.com/download)

### 4. NVIDIA Isaac Sim

NVIDIA Isaac Sim has specific hardware and driver requirements. Please refer to the official documentation for installation instructions:
[https://developer.nvidia.com/isaac-sim](https://developer.nvidia.com/isaac-sim)

### 5. OpenAI Whisper

Install Whisper using pip:
```bash
pip install -U openai-whisper
```

## Verification

After installing all the required software, you can verify your setup by running a simple ROS 2 example:

1.  Open a new terminal and source your ROS 2 setup file:
    ```bash
    source /opt/ros/humble/setup.bash
    ```
2.  Run the `talker` example:
    ```bash
    ros2 run demo_nodes_cpp talker
    ```
3.  Open another terminal and source your ROS 2 setup file.
4.  Run the `listener` example:
    ```bash
    ros2 run demo_nodes_py listener
    ```

If you see the listener receiving messages from the talker, your ROS 2 installation is working correctly.
