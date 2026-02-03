"""PostgreSQL service for conversation storage and analytics."""

import json
import logging
from datetime import datetime
from typing import Dict, List, Optional

from app.database import get_postgres_pool

logger = logging.getLogger(__name__)


class PostgresService:
    """Service for PostgreSQL database operations."""

    async def create_tables(self):
        """Create necessary tables if they don't exist."""
        pool = await get_postgres_pool()

        async with pool.acquire() as conn:
            # Conversations table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS conversations (
                    session_id VARCHAR(255) PRIMARY KEY,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    last_activity TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    message_count INTEGER DEFAULT 0
                )
            """)

            # Queries table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS queries (
                    id SERIAL PRIMARY KEY,
                    session_id VARCHAR(255) REFERENCES conversations(session_id),
                    query TEXT NOT NULL,
                    answer TEXT NOT NULL,
                    citations JSONB DEFAULT '[]',
                    selected_text TEXT,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    response_time_ms INTEGER
                )
            """)

            # Book metadata table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS book_metadata (
                    id SERIAL PRIMARY KEY,
                    chapter_name VARCHAR(255) NOT NULL,
                    section_name VARCHAR(255),
                    file_path VARCHAR(500) NOT NULL UNIQUE,
                    chunk_count INTEGER DEFAULT 0,
                    last_embedded TIMESTAMP,
                    content_hash VARCHAR(64)
                )
            """)

            # Analytics table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS analytics (
                    id SERIAL PRIMARY KEY,
                    event_type VARCHAR(100) NOT NULL,
                    event_data JSONB DEFAULT '{}',
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            """)

            # Create indexes
            await conn.execute("""
                CREATE INDEX IF NOT EXISTS idx_queries_session_id ON queries(session_id);
                CREATE INDEX IF NOT EXISTS idx_queries_created_at ON queries(created_at);
                CREATE INDEX IF NOT EXISTS idx_conversations_last_activity ON conversations(last_activity);
                CREATE INDEX IF NOT EXISTS idx_analytics_event_type ON analytics(event_type);
                CREATE INDEX IF NOT EXISTS idx_analytics_created_at ON analytics(created_at);
            """)

        logger.info("Database tables created successfully")

    async def store_query(
        self,
        session_id: str,
        query: str,
        answer: str,
        citations: List[Dict],
        selected_text: Optional[str] = None,
        response_time_ms: Optional[int] = None
    ):
        """Store a query and its response."""
        pool = await get_postgres_pool()

        async with pool.acquire() as conn:
            # Ensure conversation exists
            await conn.execute("""
                INSERT INTO conversations (session_id)
                VALUES ($1)
                ON CONFLICT (session_id) DO UPDATE
                SET last_activity = CURRENT_TIMESTAMP,
                    message_count = conversations.message_count + 1
            """, session_id)

            # Insert query
            await conn.execute("""
                INSERT INTO queries (session_id, query, answer, citations, selected_text, response_time_ms)
                VALUES ($1, $2, $3, $4, $5, $6)
            """, session_id, query, answer, json.dumps(citations), selected_text, response_time_ms)

        logger.debug(f"Stored query for session {session_id}")

    async def get_conversation_history(
        self,
        session_id: str,
        limit: int = 10
    ) -> List[Dict]:
        """Get conversation history for a session."""
        pool = await get_postgres_pool()

        async with pool.acquire() as conn:
            rows = await conn.fetch("""
                SELECT query, answer, citations, created_at
                FROM queries
                WHERE session_id = $1
                ORDER BY created_at DESC
                LIMIT $2
            """, session_id, limit)

            return [
                {
                    "query": row["query"],
                    "answer": row["answer"],
                    "citations": json.loads(row["citations"]) if row["citations"] else [],
                    "created_at": row["created_at"].isoformat()
                }
                for row in rows
            ]

    async def log_analytics_event(
        self,
        event_type: str,
        event_data: Optional[Dict] = None
    ):
        """Log an analytics event."""
        pool = await get_postgres_pool()

        async with pool.acquire() as conn:
            await conn.execute("""
                INSERT INTO analytics (event_type, event_data)
                VALUES ($1, $2)
            """, event_type, json.dumps(event_data or {}))

        logger.debug(f"Logged analytics event: {event_type}")

    async def update_book_metadata(
        self,
        file_path: str,
        chapter_name: str,
        section_name: Optional[str],
        chunk_count: int,
        content_hash: str
    ):
        """Update book metadata after embedding."""
        pool = await get_postgres_pool()

        async with pool.acquire() as conn:
            await conn.execute("""
                INSERT INTO book_metadata (file_path, chapter_name, section_name, chunk_count, last_embedded, content_hash)
                VALUES ($1, $2, $3, $4, CURRENT_TIMESTAMP, $5)
                ON CONFLICT (file_path) DO UPDATE
                SET chapter_name = $2,
                    section_name = $3,
                    chunk_count = $4,
                    last_embedded = CURRENT_TIMESTAMP,
                    content_hash = $5
            """, file_path, chapter_name, section_name, chunk_count, content_hash)
