<!--
Sync Impact Report:
- Version change: 0.0.0 -> 1.0.0
- List of modified principles: Complete overhaul.
- Added sections: Core Principles, System Architecture Constraints, Retrieval Rules, Agent Behavior Rules, Data & State Management, Performance & Quality Constraints, Security & Configuration, Governance
- Removed sections: All previous sections.
- Templates requiring updates:
  - ⚠ .specify/templates/plan-template.md
  - ⚠ .specify/templates/spec-template.md
  - ⚠ .specify/templates/tasks-template.md
  - ⚠ .gemini/commands/sp.adr.toml
  - ⚠ .gemini/commands/sp.analyze.toml
  - ⚠ .gemini/commands/sp.checklist.toml
  - ⚠ .gemini/commands/sp.clarify.toml
  - ⚠ .gemini/commands/sp.constitution.toml
  - ⚠ .gemini/commands/sp.git.commit_pr.toml
  - ⚠ .gemini/commands/sp.implement.toml
  - ⚠ .gemini/commands/sp.phr.toml
  - ⚠ .gemini/commands/sp.plan.toml
  - ⚠ .gemini/commands/sp.reverse-engineer.toml
  - ⚠ .gemini/commands/sp.specify.toml
  - ⚠ .gemini/commands/sp.tasks.toml
  - ⚠ .gemini/commands/sp.taskstoissues.toml
  - ⚠ README.md
- Follow-up TODOs: None.
-->

# Integrated RAG Chatbot Constitution

## Purpose

This constitution governs the design, implementation, and integration of a Retrieval-Augmented Generation (RAG) chatbot embedded within a published Docusaurus book. The chatbot must reliably answer user questions using only the book’s content, with optional restriction to user-selected text.

## Core Principles

- **Retrieval-first**: All responses must be grounded in retrieved book content.
- **No hallucination**: If relevant context is not retrieved, the system must respond with uncertainty.
- **Modularity**: Ingestion, retrieval, agent logic, and frontend integration must be decoupled.
- **Reproducibility**: All pipelines must be deterministic and re-runnable.
- **Production readiness**: Code must be deployable, observable, and secure by default.

## System Architecture Constraints

- **Backend**: FastAPI (Python)
- **Agent Framework**: OpenAI Agents SDK / ChatKit
- **Vector Database**: Qdrant (Cloud Free Tier)
- **Embeddings**: Cohere embedding models
- **Relational Database**: Neon Serverless Postgres
- **Frontend**: Embedded chatbot UI within Docusaurus site
- **Deployment**: Local development + cloud-ready configuration

## Retrieval Rules

- All answers must be based solely on retrieved vectors from Qdrant.
- User-selected text queries must restrict retrieval scope to that selection.
- Retrieved chunks must include metadata (URL, section, chunk ID).
- Top-k retrieval must be configurable and logged.
- No cross-document inference beyond retrieved context.

## Agent Behavior Rules

- The agent may summarize, explain, or quote retrieved content.
- The agent must not introduce external knowledge.
- If retrieval confidence is low or empty, the agent must say:
  “I couldn’t find relevant information in the book for this question.”
- The agent must support both:
  - Free-form user questions
  - Questions constrained to user-highlighted text

## Data & State Management

- Qdrant stores embeddings and document metadata only.
- Neon Postgres stores:
  - Chat sessions
  - Message history
  - User interactions and timestamps
- No raw book content duplication outside vector storage.
- No user PII storage.

## Performance & Quality Constraints

- **Retrieval latency target**: < 300ms
- **End-to-end response time**: < 2 seconds (local)
- **Embedding and retrieval pipelines must support incremental updates.**
- **Errors must be logged with clear diagnostics.**

## Security & Configuration

- All secrets must be stored in environment variables.
- No API keys or credentials in source control.
- CORS and request validation must be enforced in FastAPI.
- Rate-limiting should be supported where applicable.

## Governance

This constitution supersedes all other development practices. Amendments require documentation, formal approval, and a documented migration plan. All pull requests and reviews must verify compliance with these principles. Complexity must be justified.

**Version**: 1.0.0 | **Ratified**: 2025-12-28 | **Last Amended**: 2025-12-28