---
id: 1
title: Amend Constitution to v1.1.0
stage: constitution
date: 2025-12-23
surface: agent
model: gemini
feature: none
branch: main
user: unknown
command: /sp.constitution
labels: [constitution, documentation, project-governance]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - README.md
tests:
 - none
---

## Prompt

---
description: Create or update the project constitution from interactive or provided principle inputs, ensuring all dependent templates stay in sync.
handoffs: 
  - label: Build Specification
    agent: sp.specify
    prompt: Implement the feature specification based on the updated constitution. I want to build...
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

You are updating the project constitution at `.specify/memory/constitution.md`. This file is a TEMPLATE containing placeholder tokens in square brackets (e.g. `[PROJECT_NAME]`, `[PRINCIPLE_1_NAME]`). Your job is to (a) collect/derive concrete values, (b) fill the template precisely, and (c) propagate any amendments across dependent artifacts.

Follow this execution flow:

1. Load the existing constitution template at `.specify/memory/constitution.md`.
   - Identify every placeholder token of the form `[ALL_CAPS_IDENTIFIER]`.
   **IMPORTANT**: The user might require less or more principles than the ones used in the template. If a number is specified, respect that - follow the general template. You will update the doc accordingly.

2. Collect/derive values for placeholders:
   - If user input (conversation) supplies a value, use it.
   - Otherwise infer from existing repo context (README, docs, prior constitution versions if embedded).
   - For governance dates: `RATIFICATION_DATE` is the original adoption date (if unknown ask or mark TODO), `LAST_AMENDED_DATE` is today if changes are made, otherwise keep previous.
   - `CONSTITUTION_VERSION` must increment according to semantic versioning rules:
     - MAJOR: Backward incompatible governance/principle removals or redefinitions.
     - MINOR: New principle/section added or materially expanded guidance.
     - PATCH: Clarifications, wording, typo fixes, non-semantic refinements.
   - If version bump type ambiguous, propose reasoning before finalizing.

3. Draft the updated constitution content:
   - Replace every placeholder with concrete text (no bracketed tokens left except intentionally retained template slots that the project has chosen not to define yet—explicitly justify any left).
   - Preserve heading hierarchy and comments can be removed once replaced unless they still add clarifying guidance.
   - Ensure each Principle section: succinct name line, paragraph (or bullet list) capturing non‑negotiable rules, explicit rationale if not obvious.
   - Ensure Governance section lists amendment procedure, versioning policy, and compliance review expectations.

4. Consistency propagation checklist (convert prior checklist into active validations):
   - Read `.specify/templates/plan-template.md` and ensure any "Constitution Check" or rules align with updated principles.
   - Read `.specify/templates/spec-template.md` for scope/requirements alignment—update if constitution adds/removes mandatory sections or constraints.
   - Read `.specify/templates/tasks-template.md` and ensure task categorization reflects new or removed principle-driven task types (e.g., observability, versioning, testing discipline).
   - Read each command file in `.specify/templates/commands/*.md` (including this one) to verify no outdated references (agent-specific names like CLAUDE only) remain when generic guidance is required.
   - Read any runtime guidance docs (e.g., `README.md`, `docs/quickstart.md`, or agent-specific guidance files if present). Update references to principles changed.

5. Produce a Sync Impact Report (prepend as an HTML comment at top of the constitution file after update):
   - Version change: old → new
   - List of modified principles (old title → new title if renamed)
   - Added sections
   - Removed sections
   - Templates requiring updates (✅ updated / ⚠ pending) with file paths
   - Follow-up TODOs if any placeholders intentionally deferred.

6. Validation before final output:
   - No remaining unexplained bracket tokens.
   - Version line matches report.
   - Dates ISO format YYYY-MM-DD.
   - Principles are declarative, testable, and free of vague language ("should" → replace with MUST/SHOULD rationale where appropriate).

7. Write the completed constitution back to `.specify/memory/constitution.md` (overwrite).

8. Output a final summary to the user with:
   - New version and bump rationale.
   - Any files flagged for manual follow-up.
   - Suggested commit message (e.g., `docs: amend constitution to vX.Y.Z (principle additions + governance update)`).

Formatting & Style Requirements:

- Use Markdown headings exactly as in the template (do not demote/promote levels).
- Wrap long rationale lines to keep readability (<100 chars ideally) but do not hard enforce with awkward breaks.
- Keep a single blank line between sections.
- Avoid trailing whitespace.

If the user supplies partial updates (e.g., only one principle revision), still perform validation and version decision steps.

If critical info missing (e.g., ratification date truly unknown), insert `TODO(<FIELD_NAME>): explanation` and include in the Sync Impact Report under deferred items.

Do not create a new template; always operate on the existing `.specify/memory/constitution.md` file.

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.


/sp.constitution /sp.constitution

Project: Unified AI-Native Book with Integrated RAG Chatbot

Purpose:
To design, author, and deploy a unified, AI-driven technical book using Spec-Kit Plus and Claude Code, published via Docusaurus on GitHub Pages, and enhanced with an embedded Retrieval-Augmented Generation (RAG) chatbot capable of answering content-aware and user-selected text queries.

Core Principles:
- Accuracy through source-grounded content generation
- Clarity for a technical audience (software engineers, AI practitioners, CS graduates)
- Consistency across book content, specifications, and chatbot responses
- Transparency and traceability of all generated outputs
- Modularity and extensibility of both book and chatbot architecture

Authoring & Content Standards:
- Writing style: Technical, instructional, and example-driven
- Audience level: Intermediate to advanced (Grade 11–14 readability)
- Tone: Professional, neutral, and precise
- All explanations must be aligned with modern AI-native software development practices
- Concepts must progress logically from foundational to advanced topics

Book Structure Requirements:
- Format: Docusaurus documentation site
- Deployment: GitHub Pages
- Content organized into:
  - Introduction & conceptual foundations
  - Spec-driven development using Spec-Kit Plus
  - Claude Code workflows and AI-assisted coding
  - AI-native architecture patterns
  - RAG systems and agent-based applications
  - Practical implementation chapters
- Each chapter must include:
  - Clear objectives
  - Diagrams or structured explanations where applicable
  - Code snippets with explanations
  - Summary and key takeaways

Citation & Verification Standards:
- All factual or technical claims must be verifiable
- Source preference:
  - Official documentation
  - Peer-reviewed papers
  - Authoritative technical blogs and standards
- Citations must be consistent and traceable
- No hallucinated APIs, libraries, or capabilities
- Claims about tools (Spec-Kit Plus, Claude Code, OpenAI SDKs) must align with documented behavior

RAG Chatbot Standards:
- Architecture:
  - Backend: FastAPI
  - LLM Interface: OpenAI Agents / ChatKit SDKs
  - Vector Store: Qdrant Cloud (Free Tier)
  - Metadata & session storage: Neon Serverless Postgres
- Knowledge Scope:
  - Primary source: Book content only
  - Secondary sources allowed only if explicitly indexed
- The chatbot must:
  - Answer questions grounded strictly in retrieved content
  - Support user-selected text queries (context-limited answers)
  - Clearly indicate when information is not found in the book
  - Avoid speculation or external knowledge leakage
- Responses must cite the chapter/section used for retrieval

Technical Constraints:
- No hardcoded secrets or credentials
- Environment variables managed via `.env`
- Modular, readable, and maintainable code
- Clear separation between ingestion, retrieval, and generation layers
- Logging and basic observability enabled

Quality & Safety Requirements:
- Zero tolerance for plagiarism
- No fabricated references or citations
- No insecure coding practices
- Graceful failure when context is insufficient
- Clear user disclaimers for AI-generated responses

Reproducibility Standards:
- All steps to build, deploy, and run the system must be documented
- Commands, scripts, and configurations must be reproducible
- Repository must include:
  - README with setup instructions
  - Architecture overview
  - Data ingestion explanation for RAG pipeline

Success Criteria:
- Book builds successfully with Docusaurus
- Book is publicly accessible via GitHub Pages
- RAG chatbot is embedded and functional within the book
- Chatbot answers are accurate, grounded, and context-aware
- All content aligns with Spec-Kit Plus specifications
- No unresolved inconsistencies between book, specs, and chatbot behavior

## Response snapshot

I have amended the constitution to version 1.1.0 with the more detailed principles and requirements you provided. I also updated the README.md to reflect the new project name.

## Outcome

- ✅ Impact: The project constitution is now up-to-date with the latest detailed requirements, and the README reflects the correct project name.
- 🧪 Tests: none
- 📁 Files: 2 files modified.
- 🔁 Next prompts: none
- 🧠 Reflection: The constitution is a living document. It's important to keep it updated as the project evolves.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none
