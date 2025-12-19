---
id: 2
title: initialize project constitution
stage: constitution
date: 2025-12-17
surface: agent
model: gemini-1.5-flash
feature: none
branch: 001-create-robotics-book
user: nimra
command: /sp.constitution

labels: [constitution, initialization]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/memory/constitution.md
tests:
null
---

## Prompt

# Project Constitution: AI-Native Textbook for Physical AI & Humanoid Robotics

# 1. Mission
Build a fully AI-native, interactive, intelligent textbook that teaches the Physical AI & Humanoid Robotics course.
The product must be fast, simple, beautiful, and feel like a REAL AI-powered education platform — not just a book.

# 2. Core Deliverables
1. A Docusaurus-based interactive textbook with 6–8 short, clean, modern chapters.
2. A fully functional RAG chatbot answering questions ONLY from the book.
3. User authentication (signup/login) using Better-Auth.
4. Personalized chapter content based on user background.
5. One-click Urdu translation for every chapter.
6. Auto-generated summaries, quizzes, and learning boosters.

# 3. Success Criteria
- Clean UI, fast loading, mobile-friendly.
- Book readable in < 45 minutes total.
- RAG answers accurate, cited, and grounded.
- Personalization visibly improves text.
- Urdu translation high-quality and fast.
- Fully deployed:
  - Frontend → Vercel
  - Backend → Railway
  - Vectors → Qdrant
  - Database → Neon

# 4. Non-Goals
- No extra animations beyond minimal useful motion.
- No overly long chapters (short + clear only).
- No complex robotics code — only education content.

# 5. Architecture Principles
- Keep frontend extremely simple, readable.
- Keep backend modular (FastAPI + services + routes).
- All data must be stored cleanly in Neon + Qdrant.
- Use clean folder structure:
  - /backend
  - /website
  - /rag
  - /agents
- Use reusable agent skills for bonus scoring.

# 6. User Stories (Prioritized)
1. As a learner, I want to read the textbook smoothly.
2. As a learner, I want to ask the chatbot questions.
3. As a learner, I want personalized content based on my background.
4. As a learner, I want Urdu translation.
5. As a learner, I want summaries + quizzes.
6. As an admin, I want clean architecture and deployment.

# 7. Constraints
- Must work on free tiers (Qdrant + Neon).
- Must deploy within 90 seconds demo recording.
- Must support low-end devices (users reading on phones).
- Must avoid complexity and heavy dependencies.

# 8. Risks & Mitigation
- **RAG low accuracy** → use chunking + MiniLM embeddings.
- **Token usage high** → implement in phases.
- **User confusion** → keep UI minimal and clean.
- **Backend errors** → add health checks + logging.

# 9. Definition of Done
- All chapters visible and readable.
- Chatbot fully functional with grounded answers.
- Auth + personalization + translation working.
- Quizzes + summaries per chapter generated.
- Fully deployed URLs live and stable.
- 90-second demo recorded.

## Response snapshot

The project constitution has been successfully initialized.

**Summary:**
-   **New Constitution Version**: 0.1.0 (Initial draft)
-   **Files Flagged for Manual Follow-up**:
    *   `.specify/templates/plan-template.md`
    *   `.specify/templates/spec-template.md`
    *   `.specify/templates/tasks-template.md`
    These templates should be reviewed to ensure alignment with the new project constitution.

**Suggested Commit Message**:
```
docs: initialize project constitution v0.1.0
```

## Outcome

- ✅ Impact: Constitution document created and populated with initial project details.
- 🧪 Tests: N/A
- 📁 Files: 1 file created/modified: .specify/memory/constitution.md
- 🔁 Next prompts: User to review the constitution and decide on next steps (e.g., commit, update templates).
- 🧠 Reflection: Successfully initialized the project constitution based on user input, adapting the template structure. Manual review of dependent templates is required.

## Evaluation notes (flywheel)

- Failure modes observed: Initial attempt to use shell script for PHR creation failed. Implemented agent-native PHR generation as fallback.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Improve shell script execution robustness or streamline agent-native PHR generation.
