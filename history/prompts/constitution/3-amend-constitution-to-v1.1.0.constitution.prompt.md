---
id: 3
title: Amend constitution to v1.1.0
stage: constitution
date: 2026-01-30
surface: agent
model: claude-opus-4-5
feature: none
branch: main
user: hackathon-participant
command: /sp.constitution
labels: [constitution, rag, chatbot, v1.1.0, hackathon]
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

/sp.constitution # RAG Chatbot Integration Constitution

## Project Vision
Integrate a sophisticated Retrieval-Augmented Generation (RAG) chatbot into the Physical AI & Humanoid Robotics textbook, enabling users to interactively query book content with context-aware responses using OpenAI Agents/ChatKit SDKs, FastAPI backend, Neon Serverless Postgres, and Qdrant Cloud vector database.

## Core Principles

### 1. User-Centric Design
- **Seamless Integration**: The chatbot must feel like a natural part of the reading experience, not a separate tool
- **Context Awareness**: Support answering questions based on user-selected text within the book
- **Responsive Interface**: Fast query responses (<2 seconds for most queries)
- **Accessibility**: Clear UI/UX that works across devices (desktop, tablet, mobile)

### 2. Technical Excellence
- **Modern Stack**: Leverage cutting-edge technologies (FastAPI, OpenAI Agents SDK, vector databases)
- **Scalability**: Design for potential thousands of concurrent users
- **Performance**: Optimize embedding generation, vector search, and LLM inference
- **Reliability**: Implement proper error handling, fallbacks, and monitoring

### 3. RAG Quality
- **Accurate Retrieval**: High-precision semantic search returning most relevant book sections
- **Contextual Responses**: Generated answers grounded in retrieved content with citations
- **Chunk Optimization**: Properly segment book content for optimal retrieval (balance between granularity and context)
- **Citation Transparency**: Always show sources/page references for chatbot responses

### 4. Data Architecture
- **Dual Database Strategy**:
  - **Neon Postgres**: Store structured data (user queries, sessions, analytics, book metadata)
  - **Qdrant Cloud**: Store vector embeddings for semantic search
- **Efficient Embedding**: Use appropriate embedding models (e.g., OpenAI text-embedding-3-small)
- **Versioning**: Support book content updates without breaking existing embeddings

### 5. Security & Privacy
- **API Key Management**: Secure storage of OpenAI and Qdrant API keys (environment variables)
- **Rate Limiting**: Prevent abuse and manage API costs
- **Data Privacy**: Comply with user data protection requirements
- **CORS Configuration**: Properly configured for GitHub Pages/Vercel deployment

### 6. Developer Experience
- **Clear Documentation**: Comprehensive setup instructions and API documentation
- **Type Safety**: Use TypeScript/Pydantic for type checking
- **Testing**: Unit and integration tests for critical paths
- **Deployment Automation**: CI/CD pipeline for seamless updates

## Architecture Components

### Frontend (Embedded in Docusaurus)
- **ChatKit UI Component**: Embedded chatbot interface using OpenAI ChatKit SDK
- **Text Selection Handler**: JavaScript to capture user-selected text
- **Response Renderer**: Display formatted responses with citations
- **Session Management**: Track conversation history

### Backend (FastAPI)
- **API Endpoints**:
  - `/api/chat` - Main query endpoint
  - `/api/chat/selected` - Query based on selected text
  - `/api/health` - Health check
  - `/api/embed` - Trigger re-embedding (admin)
- **RAG Pipeline**:
  - Query Processing → Embedding → Vector Search → Context Assembly → LLM Generation
- **Database Managers**:
  - Neon Postgres connection pool
  - Qdrant client wrapper

### Data Layer
- **Neon Postgres Schema**:
  - `conversations` table (session tracking)
  - `queries` table (user questions and responses)
  - `book_metadata` table (chapters, sections, pages)
  - `analytics` table (usage statistics)
- **Qdrant Collections**:
  - `textbook_chunks` collection (embeddings with metadata)
  - Proper indexing for fast similarity search

### Content Processing Pipeline
- **Book Parser**: Extract text from Docusaurus markdown files
- **Chunking Strategy**: Split content into semantically meaningful chunks
- **Embedding Generator**: Create vector embeddings for all chunks
- **Metadata Enricher**: Add chapter, section, page metadata to each chunk

## Success Criteria

### Functional Requirements
✅ Users can ask questions about any book content
✅ Chatbot provides accurate answers with source citations
✅ Selected text queries work seamlessly
✅ Chat history persists during session
✅ Mobile-responsive interface

### Performance Requirements
✅ Query response time: <2 seconds (p95)
✅ Vector search latency: <500ms
✅ Support 100+ concurrent users
✅ Embedding generation: Complete book in <5 minutes

### Quality Requirements
✅ Answer accuracy: >85% (based on manual evaluation)
✅ Citation accuracy: 100% (every answer shows correct source)
✅ Zero downtime deployment
✅ Error rate: <1%

## Constraints & Considerations

### Budget
- **OpenAI API**: Minimize costs through caching and efficient prompting
- **Qdrant Cloud**: Use Free Tier (1GB storage, sufficient for this project)
- **Neon Postgres**: Use Free Tier (0.5GB storage, 100 hours compute)

### Technology Choices
- **Why FastAPI**: High performance, async support, automatic API docs
- **Why Qdrant**: Best-in-class vector search, generous free tier, easy Python client
- **Why Neon**: Serverless Postgres, excellent free tier, great for educational projects
- **Why ChatKit SDK**: Official OpenAI components, production-ready

### Deployment
- **Backend**: Deploy to Vercel (serverless functions) or Railway/Render
- **Frontend**: GitHub Pages (static site) with API calls to backend
- **Databases**: Neon and Qdrant Cloud (managed services)

## Ethical Guidelines
- Transparent about AI limitations (chatbot should acknowledge when unsure)
- Respectful of intellectual property (book content)
- Privacy-first design (minimal data collection)
- Accessible to users with disabilities (WCAG compliance)

## Response snapshot

Amended constitution to v1.1.0 with comprehensive RAG chatbot integration principles:

**Version bump**: 1.0.0 → 1.1.0 (MINOR - new feature module principles)

**6 Core Principles Established**:
1. User-Centric Design (seamless integration, context awareness, <2s responses, accessibility)
2. Technical Excellence (FastAPI, OpenAI Agents SDK, scalability, reliability)
3. RAG Quality (accurate retrieval, citations, chunk optimization, 100% citation transparency)
4. Data Architecture (Neon Postgres + Qdrant Cloud dual database strategy)
5. Security & Privacy (env vars for secrets, rate limiting, CORS)
6. Developer Experience (documentation, type safety, testing, CI/CD)

**Additional Sections**:
- Architecture Components (Frontend, Backend, Data Layer, Content Processing Pipeline)
- Success Criteria (Functional, Performance, Quality metrics)
- Constraints & Considerations (Budget free tiers, tech rationale, deployment)
- Ethical Guidelines (transparency, IP, privacy, accessibility)

**Templates verified compatible** - no updates required.

## Outcome

- ✅ Impact: Constitution fully updated with RAG chatbot integration principles for hackathon
- 🧪 Tests: No tests run (constitution document)
- 📁 Files:
  - `.specify/memory/constitution.md` (updated)
- 🔁 Next prompts: `/sp.specify` to create feature spec for RAG ingestion pipeline
- 🧠 Reflection: The comprehensive user input enabled a complete constitution without ambiguity. All 6 principles are declarative and testable.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - No unresolved placeholders, version matches report
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
