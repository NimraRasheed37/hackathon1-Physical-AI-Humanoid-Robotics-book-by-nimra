"""Database connection management."""

import logging
from typing import Optional

import asyncpg
from qdrant_client import QdrantClient

from app.config import get_settings

logger = logging.getLogger(__name__)

# Global connection instances
_postgres_pool: Optional[asyncpg.Pool] = None
_qdrant_client: Optional[QdrantClient] = None


async def init_postgres() -> asyncpg.Pool:
    """Initialize PostgreSQL connection pool."""
    global _postgres_pool

    if _postgres_pool is None:
        settings = get_settings()
        _postgres_pool = await asyncpg.create_pool(
            settings.neon_connection_string,
            min_size=2,
            max_size=10
        )
        logger.info("PostgreSQL connection pool initialized")

    return _postgres_pool


async def get_postgres_pool() -> asyncpg.Pool:
    """Get PostgreSQL connection pool."""
    if _postgres_pool is None:
        return await init_postgres()
    return _postgres_pool


def init_qdrant() -> QdrantClient:
    """Initialize Qdrant client."""
    global _qdrant_client

    if _qdrant_client is None:
        settings = get_settings()
        _qdrant_client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key
        )
        logger.info("Qdrant client initialized")

    return _qdrant_client


def get_qdrant_client() -> QdrantClient:
    """Get Qdrant client instance."""
    if _qdrant_client is None:
        return init_qdrant()
    return _qdrant_client


async def init_databases():
    """Initialize all database connections."""
    await init_postgres()
    init_qdrant()
    logger.info("All databases initialized successfully")


async def close_databases():
    """Close all database connections."""
    global _postgres_pool, _qdrant_client

    if _postgres_pool is not None:
        await _postgres_pool.close()
        _postgres_pool = None
        logger.info("PostgreSQL connection pool closed")

    if _qdrant_client is not None:
        _qdrant_client.close()
        _qdrant_client = None
        logger.info("Qdrant client closed")
