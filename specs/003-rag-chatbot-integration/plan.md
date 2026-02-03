# Implementation Plan: RAG Chatbot Integration

**Branch**: `003-rag-chatbot-integration` | **Date**: 2026-02-03 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-rag-chatbot-integration/spec.md`

## Summary

Implement a RAG (Retrieval-Augmented Generation) chatbot integrated into the Physical AI & Humanoid Robotics Docusaurus textbook. The chatbot enables students to ask questions about textbook content, optionally using selected text for context, and receives AI-generated answers with citations to specific book sections. The system uses a vector database for semantic search over embedded book content and stores conversation history in a relational database.

## Technical Context

**Language/Version**:
- Frontend: TypeScript 5.6 (React 19, Docusaurus 3.9)
- Backend: Python 3.11 (FastAPI)

**Primary Dependencies**:
- Frontend: React 19, Docusaurus 3.9, CSS Modules
- Backend: FastAPI, OpenAI SDK, Qdrant Client, asyncpg

**Storage**:
- Vector DB: Qdrant Cloud (document embeddings for semantic search)
- Relational DB: Neon Postgres (conversations, analytics, book metadata)

**Testing**:
- Frontend: Vitest + React Testing Library
- Backend: pytest + pytest-asyncio

**Target Platform**: Web (Docusaurus site on Vercel, API on serverless)

**Project Type**: Web application (frontend + backend)

**Performance Goals**:
- Response time: <5 seconds for chat responses (SC-001)
- Concurrency: 100 concurrent users (SC-004)

**Constraints**:
- Query length: max 500 characters
- Selected text: max 2000 characters
- Mobile support: 375px minimum width

**Scale/Scope**:
- 12 chapters across 4 modules (~100-200 document chunks)
- Session-based conversation (browser session only)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Gate | Status | Notes |
|------|--------|-------|
| Constitution Defined | SKIP | Constitution template not customized for this project |
| Test-First Approach | PASS | Tests will be written before implementation per spec |
| Simplicity | PASS | Minimal viable RAG architecture chosen |
| No Over-Engineering | PASS | No auth, no persistence across sessions, no admin dashboard |

**Result**: PASS - Proceed with Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/003-rag-chatbot-integration/
├── plan.md              # This file
├── research.md          # Phase 0 output - Technology decisions
├── data-model.md        # Phase 1 output - Entity definitions
├── quickstart.md        # Phase 1 output - Dev setup guide
├── contracts/           # Phase 1 output - API specifications
│   └── api.yaml         # OpenAPI specification
├── checklists/          # Quality validation
│   └── requirements.md  # Spec quality checklist
└── tasks.md             # Phase 2 output (created by /sp.tasks)
```

### Source Code (repository root)

```text
# Backend API Service
backend/
├── app/
│   ├── __init__.py
│   ├── main.py              # FastAPI application entry point
│   ├── config.py            # Environment configuration
│   ├── models.py            # Pydantic request/response models
│   ├── database.py          # Database connection manager
│   ├── routers/
│   │   ├── __init__.py
│   │   ├── chat.py          # Chat API endpoints
│   │   └── admin.py         # Admin endpoints (embedding trigger)
│   ├── services/
│   │   ├── __init__.py
│   │   ├── rag_service.py   # Core RAG pipeline
│   │   ├── vector_store.py  # Qdrant operations
│   │   └── postgres_service.py # Neon Postgres operations
│   └── utils/
│       ├── __init__.py
│       ├── chunking.py      # Text chunking utilities
│       └── markdown_parser.py # Docusaurus markdown parser
├── scripts/
│   ├── embed_book.py        # One-time book embedding script
│   └── init_db.py           # Database initialization
├── tests/
│   ├── unit/
│   │   ├── test_chunking.py
│   │   ├── test_rag_service.py
│   │   └── test_models.py
│   └── integration/
│       ├── test_chat_api.py
│       └── test_vector_search.py
├── requirements.txt
├── requirements-dev.txt
└── .env.example

# Frontend (integrated into existing Docusaurus)
docusaurus-site/
├── src/
│   ├── components/
│   │   └── ChatBot/
│   │       ├── index.tsx        # Main chatbot component
│   │       ├── ChatWindow.tsx   # Chat window container
│   │       ├── ChatInput.tsx    # Message input field
│   │       ├── ChatMessage.tsx  # Individual message display
│   │       ├── Citation.tsx     # Citation display component
│   │       └── styles.module.css # Component styles
│   ├── hooks/
│   │   ├── useChat.ts           # Chat state and API hook
│   │   └── useTextSelection.ts  # Text selection detection
│   ├── services/
│   │   └── chatApi.ts           # API client for chat endpoints
│   └── theme/
│       └── Root.tsx             # Docusaurus root wrapper (adds ChatBot)
├── tests/
│   ├── ChatBot.test.tsx
│   ├── useChat.test.ts
│   └── useTextSelection.test.ts
└── package.json                 # Updated with new dependencies
```

**Structure Decision**: Web application with separate backend (Python/FastAPI) and frontend integration into existing Docusaurus site. The backend runs as an independent API service, while the frontend components are added to the existing Docusaurus project structure.

## Complexity Tracking

> No constitution violations requiring justification.

| Decision | Rationale | Simpler Alternative Considered |
|----------|-----------|-------------------------------|
| Separate backend service | Required for RAG pipeline and database operations | Serverless functions - rejected due to cold start latency |
| Qdrant for vectors | Purpose-built for vector search with good free tier | pgvector - rejected due to Neon free tier limitations |
| Session storage in browser | Meets spec requirement (no cross-device persistence) | Server-side sessions - over-engineering for current scope |

## Architecture Overview

```text
┌─────────────────────────────────────────────────────────────┐
│                     Docusaurus Site                         │
│  ┌─────────────┐  ┌───────────────┐  ┌──────────────────┐  │
│  │ Text Content │  │ ChatBot UI    │  │ Text Selection   │  │
│  │ (Markdown)   │  │ (React)       │  │ Hook             │  │
│  └─────────────┘  └───────┬───────┘  └────────┬─────────┘  │
└────────────────────────────┼──────────────────┼─────────────┘
                             │                  │
                             ▼                  │
┌─────────────────────────────────────────────────────────────┐
│                     FastAPI Backend                          │
│  ┌───────────────────────────────────────────────────────┐  │
│  │                   Chat Router                          │  │
│  │  POST /api/chat/query                                  │  │
│  │  POST /api/chat/query-with-context                     │  │
│  └───────────────────────────┬───────────────────────────┘  │
│                              │                               │
│  ┌───────────────────────────▼───────────────────────────┐  │
│  │                   RAG Service                          │  │
│  │  1. Generate query embedding (OpenAI)                  │  │
│  │  2. Search similar chunks (Qdrant)                     │  │
│  │  3. Build context from chunks                          │  │
│  │  4. Generate answer (OpenAI)                           │  │
│  │  5. Store interaction (Postgres)                       │  │
│  │  6. Return answer with citations                       │  │
│  └─────────────────────┬─────────────────────────────────┘  │
└─────────────────────────┼───────────────────────────────────┘
                          │
         ┌────────────────┼────────────────┐
         ▼                ▼                ▼
    ┌─────────┐     ┌─────────┐      ┌─────────┐
    │ OpenAI  │     │ Qdrant  │      │  Neon   │
    │  API    │     │  Cloud  │      │Postgres │
    └─────────┘     └─────────┘      └─────────┘
```

## Key Design Decisions

### 1. Embedding Strategy
- **Model**: OpenAI text-embedding-3-small (1536 dimensions)
- **Chunk Size**: 1000 characters with 200 character overlap
- **Metadata**: chapter, section, file_path stored with each chunk

### 2. RAG Pipeline
- **Retrieval**: Top 5 chunks above 0.7 cosine similarity threshold
- **Context Window**: Combine retrieved chunks with source attribution
- **Generation**: GPT-4o-mini with educational system prompt

### 3. Frontend Integration
- **Entry Point**: Docusaurus Root.tsx wrapper component
- **State Management**: React hooks (useChat, useTextSelection)
- **Styling**: CSS Modules for component isolation

### 4. Session Management
- **Storage**: sessionStorage (browser-based, clears on tab close)
- **ID Generation**: `session_${timestamp}_${random}` format
- **History**: Stored in Postgres, linked by session_id

## Dependencies

### External Services
| Service | Purpose | Free Tier Adequate |
|---------|---------|-------------------|
| OpenAI API | Embeddings + Chat | Yes (with rate limits) |
| Qdrant Cloud | Vector storage | Yes (1GB free) |
| Neon Postgres | Conversation storage | Yes (0.5GB free) |
| Vercel | Frontend hosting | Yes |

### Backend Dependencies
```text
fastapi>=0.109.0
uvicorn>=0.27.0
pydantic>=2.5.0
pydantic-settings>=2.1.0
openai>=1.10.0
qdrant-client>=1.7.0
asyncpg>=0.29.0
python-multipart>=0.0.6
```

### Frontend Dependencies
```text
(existing Docusaurus + React 19)
No additional dependencies required
```

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| OpenAI rate limits | Implement request queuing, cache common queries |
| Cold start latency | Use warm-up endpoint, consider always-on instance |
| Large book content | Pre-compute embeddings, store in Qdrant |
| Mobile usability | Responsive CSS, test on 375px viewport |

## Next Steps

1. Run `/sp.tasks` to generate implementation tasks
2. Set up backend project structure
3. Configure environment variables
4. Implement embedding pipeline
5. Build RAG service
6. Integrate frontend components
