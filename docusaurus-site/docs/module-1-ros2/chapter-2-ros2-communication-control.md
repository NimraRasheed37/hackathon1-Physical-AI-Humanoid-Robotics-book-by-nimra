---
title: Chapter 2 - ROS 2 Communication & Control
---

# Chapter 2: Communication & Control in ROS 2

In the previous chapter, we explored the fundamental architecture of ROS 2, focusing on nodes and executors. Now, we will dive into how these nodes communicate with each other and how you can control your robot using various ROS 2 communication mechanisms.

## Communication Mechanisms

ROS 2 provides several ways for nodes to exchange data, each suited for different use cases.

### Topics

**Topics** are the most common method of communication in ROS 2. They implement a publish-subscribe model, where nodes publish data to a named topic, and other nodes subscribe to that topic to receive the data. This is ideal for continuous, asynchronous data streams like sensor readings (e.g., camera images, lidar scans) or motor commands.

-   **Publishers**: Nodes that send messages to a topic.
-   **Subscribers**: Nodes that receive messages from a topic.
-   **Messages**: The data structure exchanged over topics. Each topic has a specific message type.

### Services

**Services** implement a request-response communication model. A client node sends a request to a service server node, and the server processes the request and sends back a response. Services are typically used for discrete, synchronous operations, such as triggering an action (e.g., "take a picture," "move to a specific joint angle") or querying for specific information.

-   **Client**: A node that sends a request.
-   **Server**: A node that processes the request and sends a response.
-   **Request/Response**: The data structures for the request and response.

### Actions

**Actions** are a higher-level communication mechanism built on top of topics and services. They are designed for long-running, goal-oriented tasks that provide periodic feedback and can be preempted. Actions are suitable for tasks like "navigate to a location," "pick up an object," or "perform a complex motion sequence," where continuous monitoring of progress is important.

-   **Action Client**: Sends a goal, receives feedback, and eventually a result.
-   **Action Server**: Receives a goal, performs the task, and sends feedback/result.
-   **Goal, Feedback, Result**: The data structures defining the target, progress updates, and final outcome of an action.

## Bridging Python Agents to ROS Controllers with `rclpy`

`rclpy` is the official Python client library for ROS 2, allowing Python developers to create ROS 2 nodes, publishers, subscribers, service clients, service servers, and action clients/servers. This is crucial for integrating AI agents, often developed in Python, with the ROS 2 ecosystem that controls the robot's hardware.

### Key `rclpy` Concepts

-   **`Node`**: The base class for all ROS 2 Python nodes.
-   **`Publisher`**: Used to create publishers for topics.
-   **`Subscriber`**: Used to create subscribers for topics.
-   **`Client`**: Used to create clients for services.
-   **`Service`**: Used to create service servers.
-   **`ActionClient`**: Used to interact with action servers.
-   **`ActionServer`**: Used to implement action servers.

By using `rclpy`, your Python-based AI algorithms and decision-making processes can seamlessly communicate with the robot's lower-level controllers (which might be written in C++ or other languages) via ROS 2 topics, services, and actions.
