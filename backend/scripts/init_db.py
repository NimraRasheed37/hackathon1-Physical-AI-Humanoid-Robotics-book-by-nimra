#!/usr/bin/env python3
"""Database initialization script.

This script initializes both PostgreSQL and Qdrant databases.
Run this before starting the application for the first time.
"""

import asyncio
import logging
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.config import get_settings
from app.database import init_databases, close_databases
from app.services.postgres_service import PostgresService
from app.services.vector_store import QdrantService

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


async def init_postgres():
    """Initialize PostgreSQL tables."""
    logger.info("Initializing PostgreSQL tables...")

    try:
        postgres_service = PostgresService()
        await postgres_service.create_tables()
        logger.info("PostgreSQL tables created successfully")
        return True
    except Exception as e:
        logger.error(f"Failed to initialize PostgreSQL: {e}")
        return False


def init_qdrant():
    """Initialize Qdrant collection."""
    logger.info("Initializing Qdrant collection...")

    try:
        qdrant_service = QdrantService()
        qdrant_service.ensure_collection()

        # Get collection info
        info = qdrant_service.get_collection_info()
        logger.info(f"Qdrant collection info: {info}")
        return True
    except Exception as e:
        logger.error(f"Failed to initialize Qdrant: {e}")
        return False


async def main():
    """Main initialization routine."""
    settings = get_settings()

    logger.info("=" * 50)
    logger.info("RAG Chatbot Database Initialization")
    logger.info("=" * 50)
    logger.info(f"Environment: {settings.environment}")
    logger.info("")

    # Initialize database connections
    logger.info("Connecting to databases...")
    try:
        await init_databases()
    except Exception as e:
        logger.error(f"Failed to connect to databases: {e}")
        logger.error("Please check your environment variables and database connectivity.")
        sys.exit(1)

    # Initialize PostgreSQL
    postgres_ok = await init_postgres()

    # Initialize Qdrant
    qdrant_ok = init_qdrant()

    # Close connections
    await close_databases()

    # Summary
    logger.info("")
    logger.info("=" * 50)
    logger.info("Initialization Summary")
    logger.info("=" * 50)
    logger.info(f"PostgreSQL: {'✓ OK' if postgres_ok else '✗ FAILED'}")
    logger.info(f"Qdrant:     {'✓ OK' if qdrant_ok else '✗ FAILED'}")

    if postgres_ok and qdrant_ok:
        logger.info("")
        logger.info("All databases initialized successfully!")
        logger.info("You can now run: python scripts/embed_book.py")
        sys.exit(0)
    else:
        logger.error("Some databases failed to initialize. Please check the errors above.")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
