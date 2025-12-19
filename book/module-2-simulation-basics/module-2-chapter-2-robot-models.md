## Chapter 2.2: Creating Robot Models for Simulation

### 2.2.1 Chapter Overview

For a robot to exist inside a simulator like Gazebo, it must first be *described*. This description tells the simulator what the robot looks like, how its parts are connected, how much they weigh, how they move, and what sensors they carry. In this chapter, we focus on creating robot models for simulation using **URDF** and **Xacro**, the standard description formats used in ROS 2–based robotics.

You will learn how a robot is broken down into links and joints, how physical properties such as mass and inertia are defined, and how sensors and actuators are attached. We will also see how Xacro helps manage complexity by enabling reusable, parameterized robot descriptions — a necessity for humanoid robots with many repeated components.

By the end of this chapter, you will be able to create a complete robot model, visualize it, and prepare it for simulation in Gazebo.

---

### 2.2.2 Why Robot Modeling Matters

Accurate robot modeling is the foundation of reliable simulation. If a robot’s model is incorrect, the simulation results will be misleading, and behaviors may fail when transferred to real hardware.

For humanoid robots, modeling is especially critical because:

* Balance depends on correct mass and inertia
* Joint limits affect walking and manipulation
* Sensor placement impacts perception accuracy
* Physical dimensions determine collision behavior

A well-built robot model allows you to:

* Test algorithms safely
* Debug mechanical design issues
* Train learning-based controllers
* Build digital twins of real robots

Simulation quality can never exceed model quality.

---

### 2.2.3 Unified Robot Description Format (URDF)

**URDF (Unified Robot Description Format)** is an XML-based format used to describe the structure of a robot. It defines the robot as a tree of rigid bodies connected by joints.

A URDF file typically describes:

* Links (rigid bodies)
* Joints (connections between links)
* Visual geometry (how the robot looks)
* Collision geometry (how the robot collides)
* Inertial properties (mass and inertia)

URDF does **not** simulate physics by itself — it only describes the robot. Gazebo and ROS 2 use this description to perform simulation and control.

---

### 2.2.4 Links: The Building Blocks

A **link** represents a rigid part of the robot, such as an arm segment, torso, or foot.

Each link can contain up to three elements:

#### Visual

Defines how the link appears in visualization tools such as RViz and Gazebo.

```xml
<visual>
  <geometry>
    <box size="0.2 0.1 0.05"/>
  </geometry>
</visual>
```

#### Collision

Defines the shape used for physical interactions and collision detection. This is often simpler than the visual geometry for performance reasons.

```xml
<collision>
  <geometry>
    <box size="0.2 0.1 0.05"/>
  </geometry>
</collision>
```

#### Inertial

Defines physical properties such as mass and inertia.

```xml
<inertial>
  <mass value="2.0"/>
  <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0.0" ixz="0.0" iyz="0.0"/>
</inertial>
```

Correct inertial values are essential for realistic motion, especially for humanoid robots.

---

### 2.2.5 Joints: Connecting the Robot

**Joints** define how links move relative to each other. Each joint connects a *parent link* to a *child link*.

Common joint types include:

* **fixed** – no movement
* **revolute** – rotation with limits (most common)
* **continuous** – unlimited rotation
* **prismatic** – linear movement

Example revolute joint:

```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="lower_arm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="20" velocity="1.0"/>
</joint>
```

Joint limits are critical for preventing unrealistic movements and protecting real hardware.

---

### 2.2.6 From URDF to Gazebo

URDF was originally designed for visualization and kinematics, not full physics simulation. Gazebo extends URDF using special tags and plugins.

Example Gazebo extension:

```xml
<gazebo reference="elbow_joint">
  <damping>0.1</damping>
  <friction>0.0</friction>
</gazebo>
```

Gazebo-specific elements allow you to:

* Tune joint dynamics
* Add sensors
* Attach controllers

---

### 2.2.7 Xacro: Managing Complexity

As robots grow more complex, plain URDF becomes difficult to maintain. **Xacro (XML Macros)** solves this problem.

Xacro allows:

* Reusable macros
* Parameterized dimensions
* Conditional logic
* Cleaner, modular files

Example Xacro macro:

```xml
<xacro:macro name="leg" params="prefix">
  <link name="${prefix}_thigh"/>
  <link name="${prefix}_shin"/>
</xacro:macro>
```

Using Xacro, humanoid robots can define one leg and reuse it for both sides with different parameters.

Xacro files are converted into URDF before being used by ROS 2 and Gazebo.

---

### 2.2.8 Visualizing the Robot Model

Before running a simulation, robot models should be visualized and validated.

Common tools:

* **RViz** – verifies kinematic structure
* **robot_state_publisher** – publishes joint transforms
* **joint_state_publisher_gui** – manually move joints

Typical workflow:

1. Convert Xacro → URDF
2. Load URDF into RViz
3. Check joint directions and limits
4. Verify link alignment

This step prevents many simulation issues later.

---

### 2.2.9 Humanoid Modeling Considerations

Humanoid robots introduce additional challenges:

* Large number of joints
* Symmetry (left/right limbs)
* Accurate foot geometry
* Center of mass alignment
* Joint limit realism

Best practices:

* Use Xacro extensively
* Keep collision geometry simple
* Validate inertia values
* Test balance in simulation early

---

### 2.2.10 Summary

In this chapter, we explored how to create robot models for simulation using URDF and Xacro. We learned how robots are described using links and joints, how physical properties are defined, and how Gazebo extends URDF for realistic physics. Xacro was introduced as an essential tool for managing complex humanoid models.

A well-constructed robot model is the foundation of effective simulation and a critical step toward building reliable Physical AI systems.

---

### 2.2.11 Review Questions

1. What is the purpose of URDF in ROS 2 and Gazebo?
2. Explain the difference between visual and collision geometry.
3. Why are inertial properties important in humanoid robots?
4. What problem does Xacro solve compared to plain URDF?
5. Describe a recommended workflow for validating a robot model before simulation.