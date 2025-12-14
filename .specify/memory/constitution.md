<!--
---
sync-impact-report:
  version_change: "0.0.0 → 1.0.0"
  modified_principles:
    - "Principle 1: Clarity and Structure"
    - "Principle 2: Concept-First Explanations"
    - "Principle 3: Simple and Professional Language"
    - "Principle 4: Real-World Examples"
    - "Principle 5: Adherence to Technical Standards"
    - "Principle 6: Standardized Output Format"
  added_sections:
    - "Technical Standards"
    - "Output Format"
  removed_sections:
    - None
  templates_updated:
    - "(.specify/templates/plan-template.md, ⚠ pending)"
    - "(.specify/templates/spec-template.md, ⚠ pending)"
    - "(.specify/templates/tasks-template.md, ⚠ pending)"
  todos:
    - "Review and update dependent templates to align with the new constitution."
---
-->
# Physical AI & Humanoid Robotics Book Constitution

## Core Principles

### I. Clarity and Structure
Writing must be clear, structured, and beginner-to-intermediate friendly. Use headings, subheadings, diagrams descriptions, and code blocks to organize content effectively.

### II. Concept-First Explanations
Explain concepts before introducing tools. Readers should understand the "why" behind the "how."

### III. Simple and Professional Language
Keep language simple, professional, and accessible. Avoid unnecessary jargon and explain technical terms clearly when they are first used.

### IV. Real-World Examples
Use real-world robotics examples to illustrate concepts and make them relatable and practical.

### V. Adherence to Technical Standards
All technical content must adhere to the specified standards to ensure consistency and relevance.

### VI. Standardized Output Format
Follow the defined output format for all content to maintain a uniform structure across the book.

## Technical Standards

The following technical standards are adopted for all projects and examples:
- **ROS Version:** ROS 2 (Humble)
- **Programming Language:** Python (rclpy)
- **Simulation Platforms:** Gazebo, Unity, NVIDIA Isaac Sim
- **Voice Input:** OpenAI Whisper
- **AI Concepts:** LLM-based planning will be explained conceptually, without exposing any secrets or keys.

## Output Format

All content will be produced in the following format:
- **File Type:** Markdown (.md) files
- **Organization:** One chapter per folder
- **Entry Point:** Each chapter folder will contain a `README.md` file as its main entry point.

## Governance

This constitution guides the educational content, structure, and technical standards for the book. All contributions must align with these principles to ensure a high-quality, accessible learning experience. Amendments require review and approval to maintain consistency.

**Version**: 1.0.0 | **Ratified**: 2025-12-14 | **Last Amended**: 2025-12-14