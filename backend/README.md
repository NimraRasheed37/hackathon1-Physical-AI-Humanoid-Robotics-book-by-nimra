# RAG Chatbot Backend

FastAPI backend for the Physical AI & Humanoid Robotics textbook chatbot.

## Features

- RAG (Retrieval-Augmented Generation) pipeline for answering questions
- Vector search using Qdrant Cloud
- Conversation storage with Neon PostgreSQL
- Session-based conversation context
- Citation tracking with chapter/section references

## Prerequisites

- Python 3.11+
- Qdrant Cloud account
- Neon PostgreSQL database
- OpenAI API key

## Quick Start

### 1. Create Virtual Environment

```bash
cd backend
python -m venv venv

# Windows
venv\Scripts\activate

# macOS/Linux
source venv/bin/activate
```

### 2. Install Dependencies

```bash
pip install -r requirements.txt
```

### 3. Configure Environment

Copy `.env.example` to `.env` and fill in your credentials:

```bash
cp .env.example .env
```

Required environment variables:
- `OPENAI_API_KEY` - Your OpenAI API key
- `QDRANT_URL` - Qdrant Cloud cluster URL
- `QDRANT_API_KEY` - Qdrant API key
- `NEON_CONNECTION_STRING` - Neon PostgreSQL connection string

### 4. Initialize Databases

```bash
python scripts/init_db.py
```

This creates the required tables in PostgreSQL and the vector collection in Qdrant.

### 5. Embed Book Content

```bash
python scripts/embed_book.py --book-path ../docusaurus-site/docs
```

This processes the Docusaurus markdown files and stores embeddings in Qdrant.

### 6. Run the Server

```bash
# Development with auto-reload
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000

# Or using the built-in runner
python -m app.main
```

The API will be available at `http://localhost:8000`.

## API Endpoints

### Health Check
```
GET /health
```

### Chat Query
```
POST /api/v1/chat
Content-Type: application/json

{
  "query": "What is inverse kinematics?",
  "session_id": "optional-session-id"
}
```

### Chat with Selected Text
```
POST /api/v1/chat/selected
Content-Type: application/json

{
  "query": "Explain this in simpler terms",
  "selected_text": "The Jacobian matrix relates...",
  "session_id": "optional-session-id"
}
```

### Embedding Status
```
GET /api/v1/embed/status
```

### Trigger Embedding
```
POST /api/v1/embed
Content-Type: application/json

{
  "book_path": "./docs",
  "force_reembed": false
}
```

## Project Structure

```
backend/
├── app/
│   ├── __init__.py
│   ├── main.py           # FastAPI application
│   ├── config.py         # Settings management
│   ├── database.py       # Database connections
│   ├── models.py         # Pydantic models
│   ├── routers/
│   │   ├── chat.py       # Chat endpoints
│   │   └── embedding.py  # Embedding endpoints
│   ├── services/
│   │   ├── chat_service.py      # RAG pipeline
│   │   ├── embedding_service.py # Book embedding
│   │   ├── postgres_service.py  # PostgreSQL operations
│   │   └── vector_store.py      # Qdrant operations
│   └── utils/
│       ├── chunking.py         # Text chunking
│       └── markdown_parser.py  # Docusaurus parser
├── scripts/
│   ├── init_db.py        # Database initialization
│   └── embed_book.py     # Book embedding script
├── requirements.txt
├── requirements-dev.txt
└── .env.example
```

## Development

### Install Dev Dependencies

```bash
pip install -r requirements-dev.txt
```

### Run Tests

```bash
pytest tests/
```

### Code Formatting

```bash
black app/
isort app/
```

### Type Checking

```bash
mypy app/
```

## Configuration

All configuration is managed through environment variables. See `.env.example` for available options.

### Model Configuration

- `EMBEDDING_MODEL`: OpenAI embedding model (default: `text-embedding-3-small`)
- `CHAT_MODEL`: OpenAI chat model (default: `gpt-4o-mini`)
- `EMBEDDING_DIMENSIONS`: Vector dimensions (default: `1536`)

### RAG Configuration

- `CHUNK_SIZE`: Text chunk size in characters (default: `1000`)
- `CHUNK_OVERLAP`: Overlap between chunks (default: `200`)
- `TOP_K_RESULTS`: Number of similar chunks to retrieve (default: `5`)
- `SIMILARITY_THRESHOLD`: Minimum similarity score (default: `0.7`)

## License

MIT
