---
id: 1
title: Set up project constitution
stage: constitution
date: 2025-12-14
surface: agent
model: gemini-pro
feature: none
branch: main
user: user
command: /sp.constitution
labels: [constitution, setup]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

/sp.constitution This project is an educational technical book.

Rules:
- Writing must be clear, structured, and beginner-to-intermediate friendly.
- Use headings, subheadings, diagrams descriptions, and code blocks.
- Explain concepts before tools.
- Keep language simple and professional.
- Avoid unnecessary jargon.
- Use real-world robotics examples.

Technical Standards:
- ROS 2 (Humble)
- Python (rclpy)
- Gazebo, Unity, NVIDIA Isaac Sim
- OpenAI Whisper for voice input
- LLM-based planning explained conceptually (no secrets or keys)

Output Format:
- Markdown (.md) files
- One chapter per folder
- README.md as main entry

Goal:
To teach Physical AI & Humanoid Robotics through theory, simulations, and a capstone project.

## Response snapshot

I have updated the project constitution at `.specify/memory/constitution.md` based on your input.

- **Version**: 1.0.0
- **Commit Suggestion**: `docs: amend constitution to v1.0.0 (initial setup)`

I have also created a Prompt History Record for this transaction.

## Outcome

- ✅ Impact: The project constitution was created and saved.
- 🧪 Tests: No tests were run as this was a documentation change.
- 📁 Files: 1 file was created: `history/prompts/constitution/1-set-up-project-constitution.constitution.prompt.md`
- 🔁 Next prompts: The user can now proceed with other commands.
- 🧠 Reflection: The process of creating the constitution and the PHR was successful.

## Evaluation notes (flywheel)

- Failure modes observed: The initial attempt to run the `create-phr.sh` script failed because the script was not found. The fallback mechanism of creating the file manually was successful.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
