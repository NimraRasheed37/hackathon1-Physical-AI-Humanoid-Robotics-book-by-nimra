"""Embedding endpoint router for RAG Chatbot."""

import logging

from fastapi import APIRouter, BackgroundTasks, HTTPException

from app.models import EmbeddingRequest, EmbeddingResponse, ErrorResponse
from app.services.embedding_service import EmbeddingService
from app.services.postgres_service import PostgresService
from app.services.vector_store import QdrantService

logger = logging.getLogger(__name__)

router = APIRouter()


async def run_embedding_task(
    book_path: str,
    force_reembed: bool
):
    """Background task to run book embedding."""
    try:
        embedding_service = EmbeddingService()
        postgres_service = PostgresService()

        result = await embedding_service.embed_book(
            book_path=book_path,
            force_reembed=force_reembed
        )

        # Log analytics event
        await postgres_service.log_analytics_event(
            event_type="embedding_completed",
            event_data={
                "book_path": book_path,
                "force_reembed": force_reembed,
                "chunks_processed": result.get("chunks_processed", 0)
            }
        )

        logger.info(f"Embedding task completed: {result}")

    except Exception as e:
        logger.exception(f"Error in embedding background task: {e}")


@router.post(
    "/embed",
    response_model=EmbeddingResponse,
    responses={
        400: {"model": ErrorResponse, "description": "Invalid request"},
        500: {"model": ErrorResponse, "description": "Internal server error"},
    },
)
async def trigger_embedding(
    request: EmbeddingRequest,
    background_tasks: BackgroundTasks
):
    """
    Trigger book content embedding.

    This endpoint initiates the embedding process in the background.
    The book content is parsed, chunked, and stored in the vector database.
    """
    try:
        # Add embedding task to background
        background_tasks.add_task(
            run_embedding_task,
            request.book_path,
            request.force_reembed
        )

        return EmbeddingResponse(
            status="processing",
            message="Embedding task started in background",
            chunks_processed=None
        )

    except Exception as e:
        logger.exception(f"Error triggering embedding: {e}")
        raise HTTPException(
            status_code=500,
            detail={"error": "embedding_error", "message": str(e)}
        )


@router.get(
    "/embed/status",
    response_model=dict,
    responses={
        500: {"model": ErrorResponse, "description": "Internal server error"},
    },
)
async def get_embedding_status():
    """
    Get current embedding/collection status.

    Returns information about the vector store collection.
    """
    try:
        qdrant_service = QdrantService()
        collection_info = qdrant_service.get_collection_info()

        return {
            "status": "ok",
            "collection": collection_info
        }

    except Exception as e:
        logger.exception(f"Error getting embedding status: {e}")
        raise HTTPException(
            status_code=500,
            detail={"error": "status_error", "message": str(e)}
        )
