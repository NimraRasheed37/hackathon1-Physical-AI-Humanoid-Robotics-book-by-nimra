# Data Model: RAG Chatbot Integration

**Feature**: 003-rag-chatbot-integration
**Date**: 2026-02-03
**Status**: Complete

## Overview

This document defines the data entities, relationships, and validation rules for the RAG chatbot feature. Entities are divided by storage type: vector database (Qdrant) and relational database (Postgres).

---

## 1. Vector Database Entities (Qdrant)

### TextChunk

Represents an embedded segment of textbook content for semantic search.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| id | string | MD5 hash of file_path + chunk_index | Primary key, unique |
| vector | float[1536] | OpenAI embedding vector | Required, 1536 dimensions |
| text | string | Chunk content | Required, max 1500 chars |
| chapter | string | Chapter name from frontmatter | Required |
| section | string | Section name (h2 heading) | Optional |
| file_path | string | Source markdown file path | Required |
| chunk_index | integer | Position within file | Required, >= 0 |

**Collection**: `textbook_chunks`

**Indexes**:
- Vector index: HNSW with cosine distance
- Payload index: chapter (keyword filter)

---

## 2. Relational Database Entities (Postgres)

### Conversation

Tracks a user's chat session.

| Column | Type | Description | Constraints |
|--------|------|-------------|-------------|
| session_id | VARCHAR(255) | Unique session identifier | PRIMARY KEY |
| created_at | TIMESTAMP | Session start time | NOT NULL, DEFAULT NOW() |
| last_activity | TIMESTAMP | Last interaction time | NOT NULL, DEFAULT NOW() |
| message_count | INTEGER | Total messages in session | DEFAULT 0 |

**Indexes**:
- Primary key on session_id
- Index on last_activity (for cleanup queries)

---

### Query

Stores each question-answer interaction.

| Column | Type | Description | Constraints |
|--------|------|-------------|-------------|
| id | SERIAL | Auto-increment identifier | PRIMARY KEY |
| session_id | VARCHAR(255) | Reference to conversation | FOREIGN KEY -> conversations |
| query | TEXT | User's question text | NOT NULL |
| answer | TEXT | Generated response | NOT NULL |
| citations | JSONB | Array of citation objects | DEFAULT '[]' |
| selected_text | TEXT | Optional context text | NULL allowed |
| created_at | TIMESTAMP | Query timestamp | NOT NULL, DEFAULT NOW() |
| response_time_ms | INTEGER | Processing duration | NULL allowed |

**Indexes**:
- Primary key on id
- Index on session_id (join performance)
- Index on created_at (analytics queries)

**Citations JSONB Structure**:
```json
[
  {
    "chunk_id": "abc123...",
    "chapter": "Chapter 1 - ROS2 Fundamentals",
    "section": "Installation",
    "content_preview": "First 200 characters...",
    "similarity_score": 0.85
  }
]
```

---

### BookMetadata

Tracks book content and embedding status.

| Column | Type | Description | Constraints |
|--------|------|-------------|-------------|
| id | SERIAL | Auto-increment identifier | PRIMARY KEY |
| chapter_name | VARCHAR(255) | Chapter title | NOT NULL |
| section_name | VARCHAR(255) | Section title | NULL allowed |
| file_path | VARCHAR(500) | Relative path to markdown | NOT NULL, UNIQUE |
| chunk_count | INTEGER | Number of chunks generated | DEFAULT 0 |
| last_embedded | TIMESTAMP | Last embedding update | NULL allowed |
| content_hash | VARCHAR(64) | SHA256 of file content | NULL allowed |

**Indexes**:
- Primary key on id
- Unique index on file_path
- Index on chapter_name (grouping)

---

### Analytics

Stores usage events for monitoring.

| Column | Type | Description | Constraints |
|--------|------|-------------|-------------|
| id | SERIAL | Auto-increment identifier | PRIMARY KEY |
| event_type | VARCHAR(100) | Event category | NOT NULL |
| event_data | JSONB | Event-specific data | DEFAULT '{}' |
| created_at | TIMESTAMP | Event timestamp | NOT NULL, DEFAULT NOW() |

**Event Types**:
- `chat_query`: User asked a question
- `text_selection`: User selected text for context
- `citation_view`: User expanded a citation
- `error`: System error occurred

---

## 3. API Request/Response Models (Pydantic)

### ChatRequest

```python
class ChatRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=500)
    session_id: Optional[str] = None
```

### SelectedTextChatRequest

```python
class SelectedTextChatRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=500)
    selected_text: str = Field(..., max_length=2000)
    context_metadata: Optional[dict] = None
    session_id: Optional[str] = None
```

### Citation

```python
class Citation(BaseModel):
    chunk_id: str
    content: str  # Preview (200 chars max)
    chapter: str
    section: Optional[str] = None
    similarity_score: float = Field(..., ge=0.0, le=1.0)
```

### ChatResponse

```python
class ChatResponse(BaseModel):
    answer: str
    citations: List[Citation]
    session_id: str
    timestamp: datetime
    model_used: str
```

---

## 4. Entity Relationships

```text
┌─────────────────┐       ┌─────────────────┐
│  Conversation   │       │    TextChunk    │
│  (Postgres)     │       │    (Qdrant)     │
├─────────────────┤       ├─────────────────┤
│ session_id (PK) │       │ id (PK)         │
│ created_at      │       │ vector          │
│ last_activity   │       │ text            │
│ message_count   │       │ chapter         │
└────────┬────────┘       │ section         │
         │                │ file_path       │
         │ 1:N            └─────────────────┘
         │                         │
         ▼                         │ referenced by
┌─────────────────┐               │
│     Query       │               │
│  (Postgres)     │               │
├─────────────────┤               │
│ id (PK)         │               │
│ session_id (FK) │───────────────┘
│ query           │   (citations contain chunk_ids)
│ answer          │
│ citations       │
│ selected_text   │
│ response_time   │
└─────────────────┘

┌─────────────────┐       ┌─────────────────┐
│  BookMetadata   │       │    Analytics    │
│  (Postgres)     │       │   (Postgres)    │
├─────────────────┤       ├─────────────────┤
│ id (PK)         │       │ id (PK)         │
│ chapter_name    │       │ event_type      │
│ section_name    │       │ event_data      │
│ file_path       │       │ created_at      │
│ chunk_count     │       └─────────────────┘
│ last_embedded   │
└─────────────────┘
```

---

## 5. State Transitions

### Conversation Lifecycle

```text
[New Session] → [Active] → [Idle] → [Expired]
     │             │          │
     │             │          └── No activity for 24h (cleanup eligible)
     │             │
     │             └── Each query updates last_activity
     │
     └── Created on first chat message
```

### Query Processing States

```text
[Received] → [Embedding] → [Searching] → [Generating] → [Stored] → [Returned]
     │           │             │              │            │
     │           │             │              │            └── Saved to Postgres
     │           │             │              │
     │           │             │              └── OpenAI chat completion
     │           │             │
     │           │             └── Qdrant vector search
     │           │
     │           └── OpenAI embedding generation
     │
     └── Validated (length, content)
```

---

## 6. Validation Rules Summary

| Entity | Field | Rule |
|--------|-------|------|
| ChatRequest | query | 1-500 characters, non-empty |
| SelectedTextChatRequest | selected_text | 1-2000 characters |
| Citation | similarity_score | 0.0 to 1.0 |
| TextChunk | vector | Exactly 1536 dimensions |
| TextChunk | text | Max 1500 characters |
| Conversation | session_id | Format: `session_{timestamp}_{random}` |
