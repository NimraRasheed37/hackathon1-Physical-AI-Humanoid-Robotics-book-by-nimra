# Quickstart: RAG Chatbot Integration

**Feature**: 003-rag-chatbot-integration
**Date**: 2026-02-03

## Prerequisites

- Python 3.11+
- Node.js 20+
- Git
- Access to:
  - OpenAI API key
  - Qdrant Cloud account (free tier)
  - Neon Postgres account (free tier)

## 1. Environment Setup

### Clone and Navigate

```bash
cd hackathon1-Physical-AI-Humanoid-Robotics-book-by-nimra
git checkout 003-rag-chatbot-integration
```

### Create Backend Directory

```bash
mkdir -p backend/app/{routers,services,utils}
mkdir -p backend/{scripts,tests/unit,tests/integration}
```

### Create Python Virtual Environment

```bash
cd backend
python -m venv venv

# Windows
.\venv\Scripts\activate

# macOS/Linux
source venv/bin/activate
```

### Install Backend Dependencies

```bash
pip install fastapi uvicorn pydantic pydantic-settings openai qdrant-client asyncpg python-multipart
pip install pytest pytest-asyncio httpx  # Dev dependencies
```

## 2. Environment Variables

### Create `.env` file in `backend/`

```env
# OpenAI Configuration
OPENAI_API_KEY=sk-your-key-here

# Qdrant Configuration
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# Neon Postgres Configuration
NEON_CONNECTION_STRING=postgresql://user:pass@host.neon.tech/neondb?sslmode=require

# Model Configuration
EMBEDDING_MODEL=text-embedding-3-small
CHAT_MODEL=gpt-4o-mini
EMBEDDING_DIMENSIONS=1536

# RAG Configuration
CHUNK_SIZE=1000
CHUNK_OVERLAP=200
TOP_K_RESULTS=5
SIMILARITY_THRESHOLD=0.7

# API Configuration
ENVIRONMENT=development
API_BASE_URL=http://localhost:8000
MAX_REQUESTS_PER_MINUTE=60
```

## 3. External Services Setup

### Qdrant Cloud

1. Go to https://cloud.qdrant.io
2. Create a free cluster
3. Copy the cluster URL and API key
4. Update `.env` with QDRANT_URL and QDRANT_API_KEY

### Neon Postgres

1. Go to https://neon.tech
2. Create a new project (free tier)
3. Copy the connection string
4. Update `.env` with NEON_CONNECTION_STRING

### OpenAI API

1. Go to https://platform.openai.com/api-keys
2. Create a new API key
3. Update `.env` with OPENAI_API_KEY

## 4. Database Initialization

### Run Init Script

```bash
python scripts/init_db.py
```

This creates the required tables:
- `conversations`
- `queries`
- `book_metadata`
- `analytics`

## 5. Book Embedding

### Run Embedding Script

```bash
python scripts/embed_book.py --book-path ../docusaurus-site/docs
```

This processes all markdown files and:
1. Parses Docusaurus frontmatter
2. Chunks content into ~1000 char segments
3. Generates embeddings via OpenAI
4. Stores vectors in Qdrant

Expected output:
```
Found 12 markdown files
Processing: module-1-ros2/chapter-1-ros2-fundamentals.md
  Generated 15 chunks
Processing: module-1-ros2/chapter-2-ros2-communication-control.md
  Generated 18 chunks
...
Upserting 156 chunks to Qdrant...
Embedding complete!
```

## 6. Run Backend Server

### Development Mode

```bash
cd backend
uvicorn app.main:app --reload --port 8000
```

### Verify Health

```bash
curl http://localhost:8000/api/health
# {"status":"healthy","service":"RAG Chatbot API","version":"1.0.0"}
```

### Test Chat Endpoint

```bash
curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS2?"}'
```

## 7. Frontend Setup

### Install Dependencies (none required)

The Docusaurus site already has React 19. No additional packages needed.

### Add ChatBot Component

Create `docusaurus-site/src/theme/Root.tsx`:

```tsx
import React from 'react';
import ChatBot from '@site/src/components/ChatBot';

export default function Root({children}) {
  return (
    <>
      {children}
      <ChatBot />
    </>
  );
}
```

### Configure API URL

Create `docusaurus-site/src/config.ts`:

```typescript
export const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://your-api-domain.com'
  : 'http://localhost:8000';
```

### Run Frontend

```bash
cd docusaurus-site
npm start
```

Visit http://localhost:3000 and click the chat button.

## 8. Running Tests

### Backend Tests

```bash
cd backend
pytest tests/unit -v
pytest tests/integration -v
```

### Frontend Tests (when added)

```bash
cd docusaurus-site
npm test
```

## 9. Deployment Checklist

### Backend (Railway/Render)

- [ ] Create new web service
- [ ] Connect GitHub repository
- [ ] Set root directory to `backend`
- [ ] Set start command: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
- [ ] Add all environment variables
- [ ] Deploy and note API URL

### Frontend (Vercel)

- [ ] Update `API_BASE_URL` in config.ts
- [ ] Commit and push changes
- [ ] Vercel auto-deploys from main branch

### Post-Deployment

- [ ] Test health endpoint on production
- [ ] Test chat functionality
- [ ] Verify mobile responsiveness
- [ ] Monitor API logs for errors

## Common Issues

### "Cannot connect to Qdrant"
- Verify QDRANT_URL includes `https://`
- Check API key is correct
- Ensure cluster is not paused (free tier auto-pauses)

### "OpenAI rate limit"
- Reduce embedding batch size
- Add delays between API calls
- Consider caching embeddings locally

### "CORS error in browser"
- Verify backend CORS settings include frontend origin
- Check for typos in allowed origins
- Ensure protocol matches (http vs https)

### "Empty citations"
- Re-run embedding script
- Lower SIMILARITY_THRESHOLD in `.env`
- Check Qdrant collection has vectors

## Next Steps

1. Complete all frontend components
2. Add comprehensive error handling
3. Implement rate limiting
4. Set up monitoring/analytics
5. Write end-to-end tests
