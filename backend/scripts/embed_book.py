#!/usr/bin/env python3
"""Book embedding script.

This script processes the Docusaurus book content and stores
embeddings in the Qdrant vector database.
"""

import argparse
import asyncio
import logging
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.config import get_settings
from app.database import init_databases, close_databases
from app.services.embedding_service import EmbeddingService

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Embed book content into vector database"
    )
    parser.add_argument(
        "--book-path",
        type=str,
        default="../docs",
        help="Path to book content directory (default: ../docs)"
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Force re-embedding even if content unchanged"
    )
    parser.add_argument(
        "--file",
        type=str,
        help="Embed a single file instead of entire book"
    )
    return parser.parse_args()


async def main():
    """Main embedding routine."""
    args = parse_args()
    settings = get_settings()

    logger.info("=" * 50)
    logger.info("RAG Chatbot Book Embedding")
    logger.info("=" * 50)
    logger.info(f"Environment: {settings.environment}")
    logger.info(f"Embedding model: {settings.embedding_model}")
    logger.info(f"Chunk size: {settings.chunk_size}")
    logger.info(f"Chunk overlap: {settings.chunk_overlap}")
    logger.info("")

    # Resolve book path
    book_path = Path(args.book_path)
    if not book_path.is_absolute():
        book_path = Path(__file__).parent.parent / args.book_path

    if args.file:
        file_path = Path(args.file)
        if not file_path.is_absolute():
            file_path = Path(__file__).parent.parent / args.file
        logger.info(f"Single file mode: {file_path}")
    else:
        logger.info(f"Book path: {book_path}")

    # Initialize database connections
    logger.info("Connecting to databases...")
    try:
        await init_databases()
    except Exception as e:
        logger.error(f"Failed to connect to databases: {e}")
        logger.error("Please check your environment variables and run init_db.py first.")
        sys.exit(1)

    # Run embedding
    try:
        embedding_service = EmbeddingService()

        if args.file:
            result = await embedding_service.embed_single_file(
                str(file_path),
                force_reembed=args.force
            )
        else:
            result = await embedding_service.embed_book(
                str(book_path),
                force_reembed=args.force
            )

        # Print results
        logger.info("")
        logger.info("=" * 50)
        logger.info("Embedding Results")
        logger.info("=" * 50)

        if args.file:
            logger.info(f"File processed: {result.get('processed', False)}")
            logger.info(f"Chunks created: {result.get('chunks', 0)}")
        else:
            logger.info(f"Status: {result.get('status', 'unknown')}")
            logger.info(f"Files processed: {result.get('files_processed', 0)}")
            logger.info(f"Files skipped: {result.get('files_skipped', 0)}")
            logger.info(f"Total chunks: {result.get('chunks_processed', 0)}")

        logger.info("")
        logger.info("Embedding complete! The chatbot is now ready to answer questions.")

    except Exception as e:
        logger.exception(f"Embedding failed: {e}")
        sys.exit(1)
    finally:
        await close_databases()


if __name__ == "__main__":
    asyncio.run(main())
