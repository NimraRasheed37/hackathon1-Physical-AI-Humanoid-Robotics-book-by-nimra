# Deployment Guide

This guide covers deploying the RAG Chatbot for the Physical AI & Humanoid Robotics textbook.

## Architecture Overview

```
┌─────────────────────┐     ┌─────────────────────┐
│   Vercel (Frontend) │────▶│  Railway (Backend)  │
│   Docusaurus Site   │     │   FastAPI + Python  │
└─────────────────────┘     └──────────┬──────────┘
                                       │
              ┌────────────────────────┼────────────────────────┐
              │                        │                        │
              ▼                        ▼                        ▼
    ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
    │  Qdrant Cloud   │     │ Neon PostgreSQL │     │   OpenAI API    │
    │  Vector Store   │     │  Conversations  │     │  Embeddings/LLM │
    └─────────────────┘     └─────────────────┘     └─────────────────┘
```

## Prerequisites

Before deployment, ensure you have:

1. **Qdrant Cloud Account**: https://cloud.qdrant.io
2. **Neon PostgreSQL Database**: https://neon.tech
3. **OpenAI API Key**: https://platform.openai.com
4. **Vercel Account**: https://vercel.com
5. **Railway Account**: https://railway.app (or Render: https://render.com)

---

## Step 1: Set Up External Services

### 1.1 Qdrant Cloud

1. Create account at https://cloud.qdrant.io
2. Create a new cluster (free tier available)
3. Note down:
   - **Cluster URL**: `https://xxxxx.qdrant.io`
   - **API Key**: Generate from cluster settings

### 1.2 Neon PostgreSQL

1. Create account at https://neon.tech
2. Create a new project
3. Note down the **Connection String**:
   ```
   postgresql://user:password@ep-xxx.region.aws.neon.tech/neondb?sslmode=require
   ```

### 1.3 OpenAI API

1. Get API key from https://platform.openai.com/api-keys
2. Ensure you have credits for:
   - `text-embedding-3-small` (embeddings)
   - `gpt-4o-mini` (chat completion)

---

## Step 2: Deploy Backend to Railway

### 2.1 Prepare Backend

Create `backend/Procfile`:
```
web: uvicorn app.main:app --host 0.0.0.0 --port $PORT
```

Create `backend/runtime.txt`:
```
python-3.11.7
```

### 2.2 Deploy to Railway

1. Go to https://railway.app and create new project
2. Select "Deploy from GitHub repo"
3. Connect your repository
4. Set **Root Directory** to `backend`
5. Add environment variables:

| Variable | Value |
|----------|-------|
| `OPENAI_API_KEY` | Your OpenAI API key |
| `QDRANT_URL` | Your Qdrant cluster URL |
| `QDRANT_API_KEY` | Your Qdrant API key |
| `NEON_CONNECTION_STRING` | Your Neon connection string |
| `ENVIRONMENT` | `production` |
| `API_BASE_URL` | Will be assigned after deploy |

6. Deploy and note the assigned URL (e.g., `https://your-app.railway.app`)
7. Update `API_BASE_URL` with the assigned URL

### 2.3 Initialize Database

After deployment, run the initialization scripts:

```bash
# Using Railway CLI
railway run python scripts/init_db.py
railway run python scripts/embed_book.py --book-path ../docusaurus-site/docs
```

Or use the API endpoint:
```bash
curl -X POST https://your-app.railway.app/api/v1/embed \
  -H "Content-Type: application/json" \
  -d '{"book_path": "./docs", "force_reembed": true}'
```

---

## Step 3: Deploy Frontend to Vercel

### 3.1 Update Frontend Configuration

Update `docusaurus-site/src/services/config.ts`:

```typescript
export const config = {
  apiBaseUrl: process.env.REACT_APP_API_URL || 'https://your-app.railway.app',
  // ... rest of config
};
```

Or use Vercel environment variables.

### 3.2 Deploy to Vercel

1. Go to https://vercel.com and create new project
2. Import your GitHub repository
3. Configure:
   - **Framework Preset**: Other
   - **Root Directory**: `docusaurus-site`
   - **Build Command**: `npm run build`
   - **Output Directory**: `build`
   - **Install Command**: `npm install`

4. Add environment variables:

| Variable | Value |
|----------|-------|
| `REACT_APP_API_URL` | Your Railway backend URL |

5. Deploy!

### 3.3 Configure CORS

Update backend CORS settings in `backend/app/main.py` to include your Vercel domain:

```python
origins = [
    "http://localhost:3000",
    "https://your-site.vercel.app",
    "https://your-custom-domain.com",
]
```

Redeploy the backend after updating.

---

## Alternative: Deploy Backend to Render

### Render Deployment

1. Create account at https://render.com
2. Create new **Web Service**
3. Connect GitHub repository
4. Configure:
   - **Root Directory**: `backend`
   - **Runtime**: Python 3
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`

5. Add environment variables (same as Railway)
6. Deploy!

---

## Step 4: Verify Deployment

### 4.1 Test Backend Health

```bash
curl https://your-backend-url.railway.app/health
```

Expected response:
```json
{
  "status": "healthy",
  "service": "rag-chatbot-api",
  "version": "1.0.0"
}
```

### 4.2 Test Chat Endpoint

```bash
curl -X POST https://your-backend-url.railway.app/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS2?"}'
```

### 4.3 Test Frontend

1. Visit your Vercel URL
2. Click the chat button (bottom-right corner)
3. Ask a question about the textbook
4. Verify you receive a response with citations

---

## Docker Deployment (Alternative)

### Backend Dockerfile

Create `backend/Dockerfile`:

```dockerfile
FROM python:3.11-slim

WORKDIR /app

# Install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application
COPY . .

# Expose port
EXPOSE 8000

# Run application
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

### Docker Compose (Local Development)

Create `docker-compose.yml`:

```yaml
version: '3.8'

services:
  backend:
    build:
      context: ./backend
      dockerfile: Dockerfile
    ports:
      - "8000:8000"
    environment:
      - OPENAI_API_KEY=${OPENAI_API_KEY}
      - QDRANT_URL=${QDRANT_URL}
      - QDRANT_API_KEY=${QDRANT_API_KEY}
      - NEON_CONNECTION_STRING=${NEON_CONNECTION_STRING}
      - ENVIRONMENT=development
    volumes:
      - ./docusaurus-site/docs:/app/docs:ro

  frontend:
    build:
      context: ./docusaurus-site
      dockerfile: Dockerfile
    ports:
      - "3000:3000"
    environment:
      - REACT_APP_API_URL=http://localhost:8000
    depends_on:
      - backend
```

### Build and Run

```bash
# Build images
docker-compose build

# Start services
docker-compose up -d

# View logs
docker-compose logs -f

# Initialize database
docker-compose exec backend python scripts/init_db.py
docker-compose exec backend python scripts/embed_book.py --book-path /app/docs
```

---

## Environment Variables Reference

### Backend Environment Variables

| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `OPENAI_API_KEY` | Yes | - | OpenAI API key for embeddings and chat |
| `QDRANT_URL` | Yes | - | Qdrant Cloud cluster URL |
| `QDRANT_API_KEY` | Yes | - | Qdrant API key |
| `NEON_CONNECTION_STRING` | Yes | - | Neon PostgreSQL connection string |
| `EMBEDDING_MODEL` | No | `text-embedding-3-small` | OpenAI embedding model |
| `CHAT_MODEL` | No | `gpt-4o-mini` | OpenAI chat model |
| `EMBEDDING_DIMENSIONS` | No | `1536` | Embedding vector dimensions |
| `CHUNK_SIZE` | No | `1000` | Text chunk size in characters |
| `CHUNK_OVERLAP` | No | `200` | Overlap between chunks |
| `TOP_K_RESULTS` | No | `5` | Number of similar chunks to retrieve |
| `SIMILARITY_THRESHOLD` | No | `0.7` | Minimum similarity score |
| `ENVIRONMENT` | No | `development` | Environment name |
| `API_BASE_URL` | No | `http://localhost:8000` | Backend API URL |

### Frontend Environment Variables

| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `REACT_APP_API_URL` | No | `http://localhost:8000` | Backend API URL |

---

## Monitoring & Maintenance

### Logs

- **Railway**: View logs in dashboard or `railway logs`
- **Render**: View logs in dashboard
- **Vercel**: View function logs in dashboard

### Re-embedding Content

When textbook content is updated:

```bash
# Via API
curl -X POST https://your-backend-url/api/v1/embed \
  -H "Content-Type: application/json" \
  -d '{"force_reembed": true}'

# Or via CLI (Railway)
railway run python scripts/embed_book.py --force
```

### Database Migrations

The application automatically creates tables on startup. For schema changes:

1. Update `postgres_service.py` with new schema
2. Redeploy backend
3. Tables will be created if they don't exist

### Scaling

- **Railway**: Upgrade plan for more resources
- **Render**: Upgrade instance type
- **Qdrant**: Upgrade cluster tier for more vectors
- **Neon**: Upgrade for more storage/connections

---

## Troubleshooting

### Common Issues

1. **CORS Errors**
   - Ensure frontend domain is in backend CORS origins
   - Redeploy backend after updating

2. **Database Connection Failed**
   - Verify `NEON_CONNECTION_STRING` format
   - Check if SSL mode is correct (`?sslmode=require`)

3. **Qdrant Connection Failed**
   - Verify cluster is running
   - Check API key permissions

4. **Empty Search Results**
   - Run embedding script to populate vector store
   - Check `SIMILARITY_THRESHOLD` isn't too high

5. **OpenAI Rate Limits**
   - Implement retry logic (already in code)
   - Upgrade OpenAI plan if needed

### Health Checks

```bash
# Backend health
curl https://your-backend-url/health

# Embedding status
curl https://your-backend-url/api/v1/embed/status
```

---

## Security Checklist

- [ ] All API keys stored as environment variables (never in code)
- [ ] CORS restricted to specific domains in production
- [ ] PostgreSQL connection uses SSL
- [ ] Qdrant connection uses API key authentication
- [ ] Rate limiting enabled on chat endpoints
- [ ] Input validation on all user inputs
- [ ] HTTPS enforced on all endpoints

---

## Cost Estimation

### Monthly Costs (Approximate)

| Service | Free Tier | Paid Tier |
|---------|-----------|-----------|
| Vercel | Free (hobby) | $20/mo (pro) |
| Railway | $5 credit | $10-20/mo |
| Qdrant Cloud | 1GB free | $25/mo (4GB) |
| Neon | 0.5GB free | $19/mo (10GB) |
| OpenAI | Pay as you go | ~$10-50/mo |

**Total**: ~$5-100/month depending on usage
