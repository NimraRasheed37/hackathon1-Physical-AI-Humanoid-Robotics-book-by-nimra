# Data Model

This document outlines the key entities for the "Physical AI & Humanoid Robotics" book, based on the feature specification.

## Book

The main entity, representing the entire technical book.

- **Attributes**:
  - `title`: The title of the book ("Physical AI & Humanoid robotics: From Digital Intelligence to Embodied Systems").
  - `audience`: The target audience for the book (AI students, robotics beginners, final-year CS/AI learners).
  - `purpose`: The learning objectives of the book.
  - `structure`: The overall structure of the book (Introduction, 4 technical modules, 1 capstone project).

- **Relationships**:
  - Has many `Chapters`.
  - Has many `Modules`.
  - Has one `Capstone Project`.

## Chapter

A self-contained section of the book, focusing on a specific topic.

- **Attributes**:
  - `title`: The title of the chapter.
  - `content`: The textual content of the chapter in Markdown format.

- **Relationships**:
  - Belongs to a `Book`.

## Module

A technical section of the book with hands-on exercises.

- **Attributes**:
  - `title`: The title of the module.
  - `content`: The textual content of the module in Markdown format.
  - `examples`: Code examples and practical exercises.

- **Relationships**:
  - Belongs to a `Book`.

## Capstone Project

The final project that integrates all the concepts from the book.

- **Attributes**:
  - `title`: The title of the capstone project.
  - `description`: A detailed description of the project.
  - `project-files`: All the necessary files to build and run the project.

- **Relationships**:
  - Belongs to a `Book`.
