# Module 1: ROS 2 Fundamentals

This module introduces the fundamentals of the Robot Operating System (ROS 2), the software framework that will power our humanoid robot.

## What is ROS 2?

ROS 2 is an open-source set of software libraries and tools for building robot applications. It provides a flexible framework for writing robot software, with features like hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management.

## Core Concepts

### Nodes

A ROS 2 system is a distributed network of processes, called **nodes**. Each node is responsible for a single, specific purpose, such as controlling a motor, reading a sensor, or planning a path.

### Topics

Nodes communicate with each other by publishing and subscribing to **topics**. A topic is a named bus over which nodes exchange messages.

### Messages

A **message** is a simple data structure, like an integer, a floating-point number, or a string. ROS 2 provides a rich set of predefined message types, and you can also define your own.

### Services

For request/response communication, ROS 2 provides **services**. A service is defined by a pair of messages: one for the request and one for the response.

### Actions

For long-running tasks, ROS 2 provides **actions**. An action is similar to a service, but it provides feedback on the task's progress and can be preempted.

## Hello, ROS 2!

Let's create our first ROS 2 nodes: a talker that publishes a "hello world" message and a listener that subscribes to it.

See the `examples/` directory for the code for this example.
