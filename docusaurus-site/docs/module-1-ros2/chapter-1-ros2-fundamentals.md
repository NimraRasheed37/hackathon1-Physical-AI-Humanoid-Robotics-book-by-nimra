---
title: Chapter 1 - ROS 2 Fundamentals
---

# Chapter 1: ROS 2 Fundamentals – The Robotic Nervous System

Welcome to the first chapter of our book on AI-Native robotics. This chapter introduces you to the Robotic Operating System (ROS) 2, the foundational software framework that acts as the nervous system for our humanoid robot.

## What is ROS 2?

ROS 2 is an open-source, flexible framework for writing robot software. It is a set of software libraries and tools that help you build robot applications. From drivers to state-of-the-art algorithms, and with powerful developer tools, ROS has what you need for your next robotics project.

## Core Concepts

### ROS 2 Architecture

ROS 2 is designed with a distributed architecture. A robotic system is typically composed of many small, independent processes called **nodes**. This distributed nature allows for a modular and scalable system, where different parts of the robot's software can be developed, tested, and run in isolation.

### Nodes

A **node** is the smallest unit of computation in a ROS 2 system. Each node is responsible for a single, specific task, such as controlling a motor, reading a sensor, or planning a path. Nodes communicate with each other by passing messages.

### Executors

An **executor** is a mechanism in ROS 2 that manages the execution of callbacks from one or more nodes. It controls when and how the tasks within a node are run. There are different types of executors, such as single-threaded and multi-threaded executors, which can be chosen based on the needs of the application.

### Distributed and Real-Time Design

ROS 2 is built on top of a Data Distribution Service (DDS) standard, which provides a real-time, publish-subscribe messaging system. This allows for a flexible and dynamic communication system where nodes can discover and communicate with each other without any central master. This design is crucial for building complex and robust robotic systems.
