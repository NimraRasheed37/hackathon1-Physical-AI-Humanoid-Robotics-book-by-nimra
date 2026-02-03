"""FastAPI application for RAG Chatbot backend."""

import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.config import get_settings
from app.database import close_databases, init_databases
from app.models import HealthResponse
from app.routers import chat, embedding
from app.services.postgres_service import PostgresService

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan handler for startup and shutdown."""
    # Startup
    logger.info("Starting RAG Chatbot API...")
    settings = get_settings()
    logger.info(f"Environment: {settings.environment}")

    # Initialize databases
    await init_databases()

    # Create tables if they don't exist
    postgres_service = PostgresService()
    await postgres_service.create_tables()

    logger.info("RAG Chatbot API started successfully")

    yield

    # Shutdown
    logger.info("Shutting down RAG Chatbot API...")
    await close_databases()
    logger.info("RAG Chatbot API shutdown complete")


def create_app() -> FastAPI:
    """Create and configure the FastAPI application."""
    settings = get_settings()

    app = FastAPI(
        title="RAG Chatbot API",
        description="Backend API for the Physical AI & Humanoid Robotics textbook chatbot",
        version="1.0.0",
        lifespan=lifespan,
        docs_url="/docs" if settings.environment != "production" else None,
        redoc_url="/redoc" if settings.environment != "production" else None,
    )

    # Configure CORS
    origins = [
        "http://localhost:3000",
        "http://localhost:3001",
        "http://127.0.0.1:3000",
        "http://127.0.0.1:3001",
        "https://hackathon1-physical-ai-humanoid-rob-delta.vercel.app/",
    ]

    # Add production origins if configured
    if settings.environment == "production":
        origins.extend([
            settings.api_base_url,
            # Add your production frontend URL here
        ])

    app.add_middleware(
        CORSMiddleware,
        allow_origins=origins,
        allow_credentials=True,
        allow_methods=["GET", "POST", "OPTIONS"],
        allow_headers=["*"],
        max_age=600,
    )

    # Include routers
    app.include_router(chat.router, prefix="/api/v1", tags=["chat"])
    app.include_router(embedding.router, prefix="/api/v1", tags=["embedding"])

    # Health check endpoint
    @app.get("/health", response_model=HealthResponse, tags=["health"])
    async def health_check():
        """Check API health status."""
        return HealthResponse(
            status="healthy",
            service="rag-chatbot-api",
            version="1.0.0"
        )

    @app.get("/", tags=["root"])
    async def root():
        """Root endpoint with API information."""
        return {
            "service": "RAG Chatbot API",
            "version": "1.0.0",
            "docs": "/docs",
            "health": "/health"
        }

    return app


# Create the application instance
app = create_app()


if __name__ == "__main__":
    import uvicorn
    uvicorn.run("app.main:app", host="0.0.0.0", port=8000, reload=True)
