"""Chat endpoint router for RAG Chatbot."""

import logging
import time
import uuid
from datetime import datetime, timezone
from typing import List

from fastapi import APIRouter, HTTPException

from app.config import get_settings
from app.models import (
    ChatRequest,
    ChatResponse,
    Citation,
    ErrorResponse,
    SelectedTextChatRequest,
)
from app.services.chat_service import ChatService
from app.services.postgres_service import PostgresService

logger = logging.getLogger(__name__)

router = APIRouter()


def get_or_create_session_id(session_id: str | None) -> str:
    """Get existing session ID or create a new one."""
    if session_id:
        return session_id
    return str(uuid.uuid4())


@router.post(
    "/chat",
    response_model=ChatResponse,
    responses={
        400: {"model": ErrorResponse, "description": "Invalid request"},
        500: {"model": ErrorResponse, "description": "Internal server error"},
    },
)
async def chat(request: ChatRequest):
    """
    Process a chat query using RAG pipeline.

    Retrieves relevant chunks from the textbook and generates
    an AI response with citations.
    """
    start_time = time.time()
    settings = get_settings()

    try:
        session_id = get_or_create_session_id(request.session_id)

        # Initialize services
        chat_service = ChatService()
        postgres_service = PostgresService()

        # Get conversation history for context
        history = await postgres_service.get_conversation_history(session_id, limit=5)

        # Process query through RAG pipeline
        result = await chat_service.process_query(
            query=request.query,
            conversation_history=history
        )

        # Calculate response time
        response_time_ms = int((time.time() - start_time) * 1000)

        # Build citations
        citations: List[Citation] = [
            Citation(
                chunk_id=chunk["id"],
                content=chunk["text"][:200],
                chapter=chunk["metadata"]["chapter"],
                section=chunk["metadata"].get("section"),
                similarity_score=chunk["score"]
            )
            for chunk in result["sources"]
        ]

        # Store query in database
        await postgres_service.store_query(
            session_id=session_id,
            query=request.query,
            answer=result["answer"],
            citations=[c.model_dump() for c in citations],
            response_time_ms=response_time_ms
        )

        # Log analytics event
        await postgres_service.log_analytics_event(
            event_type="chat_query",
            event_data={
                "session_id": session_id,
                "query_length": len(request.query),
                "response_time_ms": response_time_ms,
                "citations_count": len(citations)
            }
        )

        return ChatResponse(
            answer=result["answer"],
            citations=citations,
            session_id=session_id,
            timestamp=datetime.now(timezone.utc),
            model_used=settings.chat_model
        )

    except Exception as e:
        logger.exception(f"Error processing chat request: {e}")
        raise HTTPException(
            status_code=500,
            detail={"error": "chat_error", "message": str(e)}
        )


@router.post(
    "/chat/selected",
    response_model=ChatResponse,
    responses={
        400: {"model": ErrorResponse, "description": "Invalid request"},
        500: {"model": ErrorResponse, "description": "Internal server error"},
    },
)
async def chat_with_selected_text(request: SelectedTextChatRequest):
    """
    Process a chat query with user-selected text as additional context.

    Uses the selected text to provide more focused, contextual answers.
    """
    start_time = time.time()
    settings = get_settings()

    try:
        session_id = get_or_create_session_id(request.session_id)

        # Initialize services
        chat_service = ChatService()
        postgres_service = PostgresService()

        # Get conversation history
        history = await postgres_service.get_conversation_history(session_id, limit=5)

        # Process query with selected text context
        result = await chat_service.process_query_with_context(
            query=request.query,
            selected_text=request.selected_text,
            context_metadata=request.context_metadata,
            conversation_history=history
        )

        # Calculate response time
        response_time_ms = int((time.time() - start_time) * 1000)

        # Build citations
        citations: List[Citation] = [
            Citation(
                chunk_id=chunk["id"],
                content=chunk["text"][:200],
                chapter=chunk["metadata"]["chapter"],
                section=chunk["metadata"].get("section"),
                similarity_score=chunk["score"]
            )
            for chunk in result["sources"]
        ]

        # Store query with selected text
        await postgres_service.store_query(
            session_id=session_id,
            query=request.query,
            answer=result["answer"],
            citations=[c.model_dump() for c in citations],
            selected_text=request.selected_text,
            response_time_ms=response_time_ms
        )

        # Log analytics event
        await postgres_service.log_analytics_event(
            event_type="chat_query_selected",
            event_data={
                "session_id": session_id,
                "query_length": len(request.query),
                "selected_text_length": len(request.selected_text),
                "response_time_ms": response_time_ms,
                "citations_count": len(citations)
            }
        )

        return ChatResponse(
            answer=result["answer"],
            citations=citations,
            session_id=session_id,
            timestamp=datetime.now(timezone.utc),
            model_used=settings.chat_model
        )

    except Exception as e:
        logger.exception(f"Error processing selected text chat request: {e}")
        raise HTTPException(
            status_code=500,
            detail={"error": "chat_error", "message": str(e)}
        )
