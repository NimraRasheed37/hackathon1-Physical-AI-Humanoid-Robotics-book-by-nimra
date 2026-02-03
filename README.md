# Physical AI & Humanoid Robotics Textbook

An AI-native textbook on Physical AI and Humanoid Robotics with an integrated RAG (Retrieval-Augmented Generation) chatbot that helps students understand concepts from the textbook.

## Features

- **Interactive Textbook**: Built with Docusaurus for a modern reading experience
- **AI-Powered Chatbot**: Ask questions about any topic in the textbook
- **Text Selection Context**: Select text and ask "Explain this" or "Tell me more"
- **Citation Tracking**: Every answer includes references to source chapters/sections
- **Session Memory**: Follow-up questions maintain conversation context

## Quick Start

### Prerequisites

- Node.js 18+ and npm
- Python 3.11+
- API Keys:
  - OpenAI API key
  - Qdrant Cloud account
  - Neon PostgreSQL database

### 1. Clone and Install

```bash
# Clone the repository
git clone https://github.com/NimraRasheed37/hackathon1-Physical-AI-Humanoid-Robotics-book-by-nimra.git
cd hackathon1-Physical-AI-Humanoid-Robotics-book-by-nimra

# Install frontend dependencies
cd docusaurus-site
npm install

# Install backend dependencies
cd ../backend
python -m venv venv

# Windows
venv\Scripts\activate

# macOS/Linux
source venv/bin/activate

pip install -r requirements.txt
```

### 2. Configure Environment

```bash
# In backend directory
cp .env.example .env
```

Edit `.env` with your credentials:
```env
OPENAI_API_KEY=sk-your-openai-key
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-key
NEON_CONNECTION_STRING=postgresql://user:pass@ep-xxx.neon.tech/neondb?sslmode=require
```

### 3. Initialize Database

```bash
# In backend directory with venv activated
python scripts/init_db.py
python scripts/embed_book.py --book-path ../docusaurus-site/docs
```

### 4. Run the Application

**Terminal 1 - Backend:**
```bash
cd backend
venv\Scripts\activate  # or source venv/bin/activate
uvicorn app.main:app --reload --port 8000
```

**Terminal 2 - Frontend:**
```bash
cd docusaurus-site
npm start
```

Visit http://localhost:3000 and click the chat button in the bottom-right corner.

## Project Structure

```
├── docusaurus-site/          # Frontend (Docusaurus + React)
│   ├── docs/                 # Textbook content (Markdown)
│   ├── src/
│   │   ├── components/
│   │   │   └── ChatBot/      # Chat widget components
│   │   ├── services/         # API client
│   │   └── theme/            # Docusaurus theme overrides
│   └── docusaurus.config.ts
│
├── backend/                  # Backend (FastAPI + Python)
│   ├── app/
│   │   ├── main.py           # FastAPI application
│   │   ├── config.py         # Settings management
│   │   ├── models.py         # Pydantic models
│   │   ├── database.py       # Database connections
│   │   ├── routers/          # API endpoints
│   │   ├── services/         # Business logic
│   │   └── utils/            # Text processing utilities
│   └── scripts/              # Database & embedding scripts
│
├── specs/                    # Feature specifications
├── DEPLOYMENT.md             # Deployment guide
└── docker-compose.yml        # Docker configuration
```

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/health` | GET | Health check |
| `/api/v1/chat` | POST | Send a chat query |
| `/api/v1/chat/selected` | POST | Chat with selected text context |
| `/api/v1/embed` | POST | Trigger book embedding |
| `/api/v1/embed/status` | GET | Get embedding status |

## Deployment

See [DEPLOYMENT.md](./DEPLOYMENT.md) for detailed deployment instructions covering:

- **Frontend**: Vercel deployment
- **Backend**: Railway or Render deployment
- **Docker**: Container-based deployment
- **Environment configuration**
- **Cost estimation**

### Quick Deploy

**Frontend (Vercel):**
1. Import GitHub repo to Vercel
2. Set root directory to `docusaurus-site`
3. Add `REACT_APP_API_URL` environment variable

**Backend (Railway):**
1. Create new Railway project
2. Set root directory to `backend`
3. Add all required environment variables
4. Deploy

## Technology Stack

### Frontend
- Docusaurus 3.9
- React 19
- TypeScript 5.6
- CSS Modules

### Backend
- FastAPI 0.109+
- Python 3.11+
- OpenAI API (text-embedding-3-small, gpt-4o-mini)
- Qdrant Cloud (vector database)
- Neon PostgreSQL (conversation storage)

## Development

### Run Tests

```bash
# Backend
cd backend
pytest tests/

# Frontend
cd docusaurus-site
npm test
```

### Code Quality

```bash
# Backend formatting
cd backend
black app/
isort app/
mypy app/

# Frontend linting
cd docusaurus-site
npm run lint
```

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

MIT

## Acknowledgments

- Built for GIAIC Q4 Hackathon
- Powered by OpenAI, Qdrant, and Neon
- Docusaurus by Meta
