# Data Model: AI-Native Book Structure

This document describes the data model for the AI-Native Book, which is structured as a collection of modules and chapters.

## Entities

### Book

The root entity that represents the entire book.

-   **Attributes**:
    -   `title`: The title of the book.
    -   `description`: A short description of the book.
    -   `modules`: A list of modules in the book.

### Module

A module represents a major section of the book, focusing on a specific topic.

-   **Attributes**:
    -   `id`: A unique identifier for the module (e.g., `module-1-ros2-robotic-nervous-system`).
    -   `title`: The title of the module.
    -   `goal`: The learning objective of the module.
    -   `targetAudience`: The intended audience for the module.
    -   `chapters`: A list of chapters in the module.

### Chapter

A chapter represents a subsection of a module, breaking down the topic into smaller, digestible parts.

-   **Attributes**:
    -   `id`: A unique identifier for the chapter (e.g., `chapter-1-ros2-fundamentals`).
    -   `title`: The title of the chapter.
    -   `content`: The content of the chapter in Markdown format.

## Relationships

-   A `Book` has one or more `Modules`.
-   A `Module` has one or more `Chapters`.

## Example Instance

-   **Book**:
    -   `title`: "AI-Native Book with Integrated RAG Chatbot"
    -   `modules`:
        -   **Module**:
            -   `id`: `module-1-ros2-robotic-nervous-system`
            -   `title`: "Module 1: The Robotic Nervous System (ROS 2)"
            -   `chapters`:
                -   **Chapter**:
                    -   `id`: `chapter-1-ros2-fundamentals`
                    -   `title`: "Chapter 1: ROS 2 Fundamentals – The Robotic Nervous System"
                -   **Chapter**:
                    -   `id`: `chapter-2-ros2-communication-control`
                    -   `title`: "Chapter 2: Communication & Control in ROS 2"
                -   **Chapter**:
                    -   `id`: `chapter-3-urdf-humanoid-modeling`
                    -   `title`: "Chapter 3: Modeling the Humanoid Body with URDF"
        -   **Module**:
            -   `id`: `module-2-perception-and-vision`
            -   `title`: "Module 2: Perception and Vision"
            -   `chapters`: []
        -   **Module**:
            -   `id`: `module-3-motion-planning-and-control`
            -   `title`: "Module 3: Motion Planning and Control"
            -   `chapters`: []
        -   **Module**:
            -   `id`: `module-4-ai-and-behavior`
            -   `title`: "Module 4: AI and Behavior"
            -   `chapters`: []