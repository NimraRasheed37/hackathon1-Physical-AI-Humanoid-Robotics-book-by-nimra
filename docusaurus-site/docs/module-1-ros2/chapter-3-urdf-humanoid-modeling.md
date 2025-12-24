---
title: Chapter 3 - Humanoid Modeling with URDF
---

# Chapter 3: Modeling the Humanoid Body with URDF

To effectively control and simulate a humanoid robot, we first need a precise digital representation of its physical structure. This is where the **Unified Robot Description Format (URDF)** comes into play. URDF is an XML-based file format used in ROS 2 (and its predecessor ROS) to describe all aspects of a robot's physical characteristics.

## Purpose of URDF

URDF serves as the standard way to model a robot's kinematics, dynamics, and visual properties. It allows various ROS 2 tools and libraries to understand your robot's layout, enabling:

-   **Visualization**: Displaying your robot in simulation environments (like Gazebo) or RVIZ (ROS Visualization).
-   **Motion Planning**: Calculating collision-free paths for your robot's limbs.
-   **Kinematics**: Determining the position and orientation of different parts of your robot based on joint angles (forward kinematics) or vice-versa (inverse kinematics).
-   **Dynamics**: Simulating the physical behavior of your robot under various forces.

## Key Elements of URDF

A URDF file primarily consists of two fundamental elements: **links** and **joints**.

### Links

A **link** represents a rigid body segment of the robot. This could be a torso, an upper arm, a forearm, a hand, a finger, or any other part that doesn't deform. Each link has associated properties that describe its:

-   **Visual characteristics**: How it looks (e.g., color, mesh model).
-   **Inertial properties**: Its mass, center of mass, and inertia tensor (crucial for dynamic simulation).
-   **Collision properties**: Its geometry used for collision detection.

### Joints

A **joint** defines the connection between two links and describes how one link can move relative to another. Joints introduce degrees of freedom (DOF) to the robot. Common joint types include:

-   **Revolute**: Allows rotation around a single axis (like an elbow or shoulder). This is the most common type for humanoid robots.
-   **Prismatic**: Allows linear movement along a single axis (like a linear actuator).
-   **Fixed**: Connects two links rigidly, with no relative motion (useful for aggregating multiple visual elements into one rigid body).

Each joint specifies:

-   **Parent and Child Links**: Which two links it connects.
-   **Type**: The kind of motion it allows (revolute, prismatic, fixed, etc.).
-   **Origin**: The pose (position and orientation) of the joint relative to its parent link.
-   **Axis**: The axis of motion for revolute and prismatic joints.
-   **Limits**: The upper and lower bounds of motion, and velocity/effort limits.

### Frames

While not explicitly a URDF tag, the concept of **frames** is fundamental. Every link and joint implicitly defines a coordinate frame. ROS 2 uses a hierarchical tree of coordinate frames (a TF tree) to keep track of the spatial relationships between all robot parts and objects in its environment.

## Using URDF with ROS 2 for Humanoids

For humanoid robots, URDF becomes even more critical due to their complex, multi-jointed structure. A typical humanoid URDF will:

-   **Define a base link**: Often the "pelvis" or "torso" from which the entire kinematic chain originates.
-   **Hierarchical structure**: Links and joints are connected in a tree-like hierarchy, from the base outwards to the limbs, hands, and head.
-   **Many revolute joints**: To mimic human-like motion, humanoids require numerous revolute joints for shoulders, elbows, wrists, hips, knees, and ankles.
-   **Visual meshes**: High-fidelity humanoid models often use external mesh files (e.g., `.stl`, `.dae`) referenced in the URDF for realistic visualization.

Once your humanoid is described in a URDF file, you can load it into ROS 2 using tools like `robot_state_publisher` to publish the robot's state to the TF tree. This allows other ROS 2 nodes to query the robot's current pose and plan movements, bringing your digital model to life in simulation or on the real hardware.
