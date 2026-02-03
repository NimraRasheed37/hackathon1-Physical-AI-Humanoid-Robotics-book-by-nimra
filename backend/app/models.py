"""Pydantic models for request/response validation."""

from datetime import datetime
from typing import List, Optional

from pydantic import BaseModel, Field


class ChatRequest(BaseModel):
    """Request model for chat endpoint."""

    query: str = Field(..., min_length=1, max_length=500, description="User's question")
    session_id: Optional[str] = Field(None, description="Session identifier for conversation continuity")


class SelectedTextChatRequest(BaseModel):
    """Request model for chat with selected text context."""

    query: str = Field(..., min_length=1, max_length=500, description="User's question")
    selected_text: str = Field(..., max_length=2000, description="User-selected text for context")
    context_metadata: Optional[dict] = Field(None, description="Optional metadata about selection location")
    session_id: Optional[str] = Field(None, description="Session identifier")


class Citation(BaseModel):
    """Citation information for a source chunk."""

    chunk_id: str = Field(..., description="Unique identifier for the source chunk")
    content: str = Field(..., max_length=200, description="Preview of the source content")
    chapter: str = Field(..., description="Chapter name")
    section: Optional[str] = Field(None, description="Section name if available")
    similarity_score: float = Field(..., ge=0.0, le=1.0, description="Relevance score")


class ChatResponse(BaseModel):
    """Response model for chat endpoint."""

    answer: str = Field(..., description="AI-generated response")
    citations: List[Citation] = Field(default_factory=list, description="Sources used for the answer")
    session_id: str = Field(..., description="Session identifier")
    timestamp: datetime = Field(..., description="Response timestamp")
    model_used: str = Field(..., description="AI model identifier")


class EmbeddingRequest(BaseModel):
    """Request to trigger book embedding."""

    force_reembed: bool = Field(False, description="Force re-embedding even if content unchanged")
    book_path: str = Field("./docs", description="Path to book content")


class EmbeddingResponse(BaseModel):
    """Response for embedding operation."""

    status: str = Field(..., description="Operation status")
    message: str = Field(..., description="Status message")
    chunks_processed: Optional[int] = Field(None, description="Number of chunks processed")


class HealthResponse(BaseModel):
    """Health check response."""

    status: str = Field(..., description="Service health status")
    service: str = Field(..., description="Service name")
    version: str = Field(..., description="API version")


class ErrorResponse(BaseModel):
    """Error response model."""

    error: str = Field(..., description="Error code")
    message: str = Field(..., description="Human-readable error message")
    details: Optional[dict] = Field(None, description="Additional error context")
