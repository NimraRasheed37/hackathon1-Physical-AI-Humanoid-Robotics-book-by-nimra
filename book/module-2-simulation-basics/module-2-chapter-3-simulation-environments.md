# Module 2: Simulation & Digital Twins

## Chapter 2.3: Building Simulation Environments

### 2.3.1 Chapter Overview

A robot does not exist in isolation. Its intelligence, robustness, and usefulness are revealed only when it interacts with an environment. In simulation, this environment must be carefully designed to reflect real-world conditions while remaining controllable and repeatable. In this chapter, we focus on **building simulation environments (worlds) in Gazebo** that allow meaningful testing and development of robotic behaviors.

You will learn how Gazebo worlds are structured, how to add objects and terrains, how to configure physics properties such as gravity and friction, and how to create environments that challenge humanoid robots realistically. By the end of this chapter, you will be able to design simulation worlds that go beyond empty rooms and support perception, navigation, manipulation, and interaction experiments.

---

### 2.3.2 What Is a Gazebo World?

A **Gazebo world** defines everything that exists around the robot. It describes:

* The physical environment (ground, walls, objects)
* Gravity and physics settings
* Lighting and visual conditions
* Static and dynamic elements

Worlds are written using **SDF (Simulation Description Format)**, an XML-based format specifically designed for simulation. While URDF describes *robots*, SDF describes *environments*.

Conceptually:

* **URDF/Xacro** → describes the robot
* **SDF (World file)** → describes the world

Both come together inside Gazebo to create a complete simulation.

---

### 2.3.3 Basic World Structure

A minimal Gazebo world contains:

* A `<world>` element
* A physics engine configuration
* A ground plane
* A light source

Example (simplified):

```xml
<sdf version="1.7">
  <world name="default_world">

    <gravity>0 0 -9.81</gravity>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

  </world>
</sdf>
```

This creates a flat ground under normal Earth gravity with basic lighting — a starting point for most simulations.

---

### 2.3.4 Adding Objects to the Environment

Objects such as tables, boxes, walls, and tools are added to worlds as **models**.

Objects can be:

* **Static** (do not move, e.g., walls, floors)
* **Dynamic** (affected by physics, e.g., boxes, balls)

Example of a simple box model:

```xml
<model name="box">
  <static>false</static>
  <link name="link">
    <collision>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </collision>
    <visual>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </visual>
  </link>
</model>
```

Adding diverse objects allows you to test:

* Obstacle avoidance
* Object manipulation
* Collision handling
* Perception robustness

---

### 2.3.5 Terrains and Ground Surfaces

Real-world environments rarely have perfectly flat floors. Gazebo supports more complex terrains using:

* Heightmaps
* Sloped planes
* Stairs and ramps

Terrains are especially important for **humanoid robots**, as walking stability depends heavily on surface properties.

Key terrain factors:

* Slope angle
* Step height
* Surface roughness
* Contact friction

Testing on varied terrain helps expose weaknesses in balance and locomotion algorithms before deploying to real robots.

---

### 2.3.6 Physics Properties and Their Impact

Physics settings determine how realistic the simulation feels.

Important physics parameters include:

* **Gravity** – affects balance and falling behavior
* **Time step** – simulation accuracy vs performance
* **Real-time factor** – speed of simulation
* **Friction coefficients** – slipping and traction
* **Restitution** – bounciness of contacts

Example physics configuration:

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```

Incorrect physics settings can lead to unstable walking, unrealistic collisions, or numerical instability.

---

### 2.3.7 Lighting and Visual Conditions

Perception algorithms depend heavily on visual input. Gazebo allows control over:

* Light sources
* Shadows
* Colors and textures
* Camera viewpoints

By changing lighting conditions, you can test:

* Vision robustness
* Object detection reliability
* Depth perception accuracy

This is especially useful for preparing systems for real-world variability.

---

### 2.3.8 Dynamic Environments and Interaction

Advanced simulations include **dynamic environments** where elements move or change over time.

Examples:

* Moving obstacles
* Doors opening and closing
* Objects being dropped
* Humans walking in the scene

Dynamic environments allow testing of:

* Reactive planning
* Safety systems
* Human-robot interaction

These scenarios are critical for humanoid robots operating in real human spaces.

---

### 2.3.9 Environment Design for Humanoid Robots

When designing environments for humanoids, consider:

* Human-scale dimensions
* Furniture height and spacing
* Stair geometry
* Narrow passages
* Ground contact realism

Best practices:

* Start simple, then increase complexity
* Change one variable at a time
* Validate walking on flat ground before uneven terrain
* Record failures for analysis

Well-designed environments reveal weaknesses early and guide better robot design.

---

### 2.3.10 Summary

In this chapter, we explored how to build simulation environments in Gazebo using world files, objects, terrains, and physics settings. We saw how environment design directly affects robot behavior and why careful control of physical and visual properties is essential for realistic testing.

Simulation environments are more than backgrounds — they are active participants in the development of intelligent robotic systems. For humanoid robots, thoughtfully designed worlds are crucial for achieving stability, robustness, and real-world readiness.

---

### 2.3.11 Review Questions

1. What is the role of a Gazebo world file?
2. How does SDF differ from URDF?
3. Why are terrain and friction especially important for humanoid robots?
4. Name three physics parameters that influence simulation realism.
5. How can dynamic environments improve the robustness of robotic systems?