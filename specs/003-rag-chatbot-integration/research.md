# Research: RAG Chatbot Integration

**Feature**: 003-rag-chatbot-integration
**Date**: 2026-02-03
**Status**: Complete

## Overview

This document captures research decisions for implementing the RAG chatbot feature. Each decision includes rationale and alternatives considered.

---

## 1. Embedding Model Selection

### Decision
Use **OpenAI text-embedding-3-small** (1536 dimensions)

### Rationale
- Best balance of cost, performance, and quality for educational content
- Native integration with OpenAI chat models (GPT-4o-mini)
- Lower cost than text-embedding-3-large ($0.02/1M tokens vs $0.13/1M)
- Sufficient quality for textbook-length content retrieval

### Alternatives Considered
| Model | Pros | Cons | Rejected Because |
|-------|------|------|------------------|
| text-embedding-3-large | Higher quality | 3x cost, 3072 dimensions | Overkill for book content |
| text-embedding-ada-002 | Cheaper | Legacy model, lower quality | Deprecated path |
| Cohere embed-v3 | Good multilingual | Extra vendor, API complexity | English-only requirement |
| Local models (all-MiniLM) | No API cost | Hosting complexity, lower quality | Adds infrastructure burden |

---

## 2. Vector Database Selection

### Decision
Use **Qdrant Cloud** (free tier)

### Rationale
- Purpose-built for vector search with excellent performance
- 1GB free tier sufficient for ~100-200 book chunks
- Simple Python SDK with async support
- Built-in filtering by metadata (chapter, section)

### Alternatives Considered
| Database | Pros | Cons | Rejected Because |
|----------|------|------|------------------|
| Pinecone | Market leader | Free tier limited, pricing unclear | Cost concerns |
| pgvector (Neon) | Single database | Vector ops consume compute quota | Free tier limitations |
| Weaviate | Full-featured | More complex setup | Over-engineering |
| ChromaDB | Simple, local | No managed hosting | Need cloud deployment |
| Milvus | Enterprise-grade | Complex deployment | Over-engineering |

---

## 3. Chat Model Selection

### Decision
Use **GPT-4o-mini** for response generation

### Rationale
- Excellent instruction following for educational context
- Good balance of quality and cost ($0.15/1M input, $0.60/1M output)
- Fast response time (<2s generation typically)
- Strong at citing sources and acknowledging limitations

### Alternatives Considered
| Model | Pros | Cons | Rejected Because |
|-------|------|------|------------------|
| GPT-4o | Highest quality | 10x cost | Overkill for Q&A |
| GPT-3.5-turbo | Cheapest | Lower quality citations | May miss nuance |
| Claude 3 Haiku | Fast, cheap | Different API, vendor lock-in | OpenAI ecosystem preferred |
| Llama 3.1 | Open source | Self-hosting required | Infrastructure burden |

---

## 4. Chunking Strategy

### Decision
- **Chunk size**: 1000 characters
- **Overlap**: 200 characters
- **Split strategy**: Sentence-boundary aware

### Rationale
- 1000 chars (~200 tokens) balances context completeness with retrieval precision
- 200 char overlap prevents information loss at chunk boundaries
- Sentence-aware splitting maintains semantic coherence
- ~100-200 chunks for entire book (manageable for free tier)

### Research Sources
- OpenAI RAG best practices recommend 200-500 token chunks
- LangChain documentation suggests 10-20% overlap
- Testing with robotics content showed sentence boundaries work well

---

## 5. Relational Database Selection

### Decision
Use **Neon Postgres** (free tier)

### Rationale
- Already integrated with project (per spec assumptions)
- 0.5GB free tier sufficient for conversation logs
- Serverless with auto-suspend (cost-effective)
- Standard PostgreSQL compatibility

### Schema Requirements
- conversations: session tracking
- queries: question/answer storage with citations
- book_metadata: chapter/section index
- analytics: usage tracking

---

## 6. Backend Framework Selection

### Decision
Use **FastAPI** with Python 3.11

### Rationale
- Async-first design matches I/O-bound RAG workflow
- Built-in OpenAPI documentation
- Excellent Pydantic integration for request/response validation
- Strong ecosystem (uvicorn, asyncpg)

### Alternatives Considered
| Framework | Pros | Cons | Rejected Because |
|-----------|------|------|------------------|
| Flask | Simpler | No native async | Async required for concurrent requests |
| Django | Full-featured | Heavy for API-only | Over-engineering |
| Express.js | JavaScript | Different language from AI libs | Python ecosystem preferred |
| Hono | Edge-first | Newer, less mature | Risk for production |

---

## 7. Frontend Integration Strategy

### Decision
Use **Docusaurus Theme Wrapper** pattern

### Rationale
- Root.tsx wrapper allows global component injection
- No swizzling of core theme components required
- Clean separation from book content
- Works with Docusaurus 3.9 theming system

### Implementation Approach
1. Create `src/theme/Root.tsx` as wrapper component
2. Inject ChatBot component at root level
3. Use CSS Modules for isolated styling
4. Hooks manage state without external libraries

---

## 8. Session Management Strategy

### Decision
Use **Browser sessionStorage** with server-side logging

### Rationale
- Meets spec requirement: session-based only (no cross-device)
- Simple implementation with no auth required
- Conversation stored server-side for analytics
- Session ID links frontend state to backend records

### Session Flow
1. Generate session ID on first chat open
2. Store in sessionStorage (persists across page navigation)
3. Include session_id in all API requests
4. Server logs interactions keyed by session_id
5. Session clears when browser tab closes

---

## 9. Deployment Strategy

### Decision
- **Frontend**: Deploy with existing Vercel setup
- **Backend**: Deploy to serverless platform (Railway/Render)

### Rationale
- Frontend already deployed on Vercel (per project setup)
- Backend needs persistent service for database connections
- Railway/Render free tiers support Python applications
- Environment variables managed per platform

### Alternatives Considered
| Platform | Pros | Cons | Rejected Because |
|----------|------|------|------------------|
| Vercel Functions | Same platform | Python support limited | Cold starts, timeout limits |
| AWS Lambda | Scalable | Complex setup, cold starts | Over-engineering |
| Fly.io | Docker-based | Requires containerization | Added complexity |
| Self-hosted | Full control | Maintenance burden | Out of scope |

---

## 10. CORS Configuration

### Decision
Allow requests from:
- `https://*.vercel.app`
- `https://*.github.io`
- `http://localhost:3000`

### Rationale
- Production Vercel deployment needs access
- GitHub Pages as potential alternative
- Local development support
- Wildcard patterns for preview deployments

---

## Research Gaps (None)

All NEEDS CLARIFICATION items from Technical Context have been resolved through this research.

## References

- OpenAI Embeddings Documentation: https://platform.openai.com/docs/guides/embeddings
- Qdrant Cloud Documentation: https://qdrant.tech/documentation/
- FastAPI Documentation: https://fastapi.tiangolo.com/
- Docusaurus Swizzling Guide: https://docusaurus.io/docs/swizzling
- RAG Best Practices: https://platform.openai.com/docs/guides/production-best-practices
