---
id: 2
title: Update constitution for RAG chatbot
stage: constitution
date: 2025-12-28
surface: agent
model: gemini
feature: none
branch: main
user: {{USER}}
command: /sp.constitution
labels: [constitution, rag, chatbot]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 (none)
---

## Prompt

/sp.constitution Integrated RAG Chatbot
Purpose

This constitution governs the design, implementation, and integration of a Retrieval-Augmented Generation (RAG) chatbot embedded within a published Docusaurus book. The chatbot must reliably answer user questions using only the book’s content, with optional restriction to user-selected text.

Core Principles

Retrieval-first: All responses must be grounded in retrieved book content.

No hallucination: If relevant context is not retrieved, the system must respond with uncertainty.

Modularity: Ingestion, retrieval, agent logic, and frontend integration must be decoupled.

Reproducibility: All pipelines must be deterministic and re-runnable.

Production readiness: Code must be deployable, observable, and secure by default.

System Architecture Constraints

Backend: FastAPI (Python)

Agent Framework: OpenAI Agents SDK / ChatKit

Vector Database: Qdrant (Cloud Free Tier)

Embeddings: Cohere embedding models

Relational Database: Neon Serverless Postgres

Frontend: Embedded chatbot UI within Docusaurus site

Deployment: Local development + cloud-ready configuration

Retrieval Rules

All answers must be based solely on retrieved vectors from Qdrant.

User-selected text queries must restrict retrieval scope to that selection.

Retrieved chunks must include metadata (URL, section, chunk ID).

Top-k retrieval must be configurable and logged.

No cross-document inference beyond retrieved context.

Agent Behavior Rules

The agent may summarize, explain, or quote retrieved content.

The agent must not introduce external knowledge.

If retrieval confidence is low or empty, the agent must say:
“I couldn’t find relevant information in the book for this question.”

The agent must support both:

Free-form user questions

Questions constrained to user-highlighted text

Data & State Management

Qdrant stores embeddings and document metadata only.

Neon Postgres stores:

Chat sessions

Message history

User interactions and timestamps

No raw book content duplication outside vector storage.

No user PII storage.

Performance & Quality Constraints

Retrieval latency target: < 300ms

End-to-end response time: < 2 seconds (local)

Embedding and retrieval pipelines must support incremental updates.

Errors must be logged with clear diagnostics.

Security & Configuration

All secrets must be stored in environment variables.

No API keys or credentials in source control.

CORS and request validation must be enforced in FastAPI.

Rate-limiting should be supported where applicable.

## Response snapshot

Updated the constitution at `.specify/memory/constitution.md` with the new content for the RAG chatbot. Also added a sync impact report to the top of the file.

## Outcome

- ✅ Impact: The project constitution has been updated to reflect the new requirements for the RAG chatbot.
- 🧪 Tests: No tests were run.
- 📁 Files: 
  - `.specify/memory/constitution.md`
- 🔁 Next prompts: none
- 🧠 Reflection: The process of updating the constitution and checking dependent files was smooth. The templates and command definitions are generic enough to not require updates.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
