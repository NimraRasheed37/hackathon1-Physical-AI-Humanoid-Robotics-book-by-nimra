"""Qdrant vector store service for semantic search."""

import logging
from typing import Dict, List

from qdrant_client.models import Distance, PointStruct, VectorParams

from app.config import get_settings
from app.database import get_qdrant_client

logger = logging.getLogger(__name__)


class QdrantService:
    """Service for Qdrant vector database operations."""

    COLLECTION_NAME = "textbook_chunks"

    def __init__(self):
        self.settings = get_settings()
        self.client = get_qdrant_client()

    def ensure_collection(self):
        """Create collection if it doesn't exist."""
        try:
            self.client.get_collection(self.COLLECTION_NAME)
            logger.info(f"Collection '{self.COLLECTION_NAME}' already exists")
        except Exception:
            self.client.create_collection(
                collection_name=self.COLLECTION_NAME,
                vectors_config=VectorParams(
                    size=self.settings.embedding_dimensions,
                    distance=Distance.COSINE
                )
            )
            logger.info(f"Created collection '{self.COLLECTION_NAME}'")

    def upsert_chunks(self, chunks: List[Dict]):
        """Insert or update chunks in vector store."""
        points = []
        for chunk in chunks:
            points.append(PointStruct(
                id=chunk["id"],
                vector=chunk["embedding"],
                payload={
                    "text": chunk["text"],
                    "chapter": chunk["metadata"]["chapter"],
                    "section": chunk["metadata"].get("section"),
                    "file_path": chunk["metadata"]["file_path"],
                    "chunk_index": chunk["metadata"].get("chunk_index", 0)
                }
            ))

        # Batch upsert in groups of 100
        batch_size = 100
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            self.client.upsert(
                collection_name=self.COLLECTION_NAME,
                points=batch
            )

        logger.info(f"Upserted {len(points)} chunks to Qdrant")

    def search(
        self,
        query_vector: List[float],
        top_k: int = 5,
        score_threshold: float = 0.7
    ) -> List[Dict]:
        """Search for similar chunks."""
        # Use query_points for newer qdrant-client versions
        results = self.client.query_points(
            collection_name=self.COLLECTION_NAME,
            query=query_vector,
            limit=top_k,
            score_threshold=score_threshold
        )

        return [
            {
                "id": str(point.id),
                "text": point.payload["text"],
                "score": point.score,
                "metadata": {
                    "chapter": point.payload["chapter"],
                    "section": point.payload.get("section"),
                    "file_path": point.payload["file_path"],
                    "chunk_index": point.payload.get("chunk_index", 0)
                }
            }
            for point in results.points
        ]

    def delete_collection(self):
        """Delete the entire collection (use with caution)."""
        self.client.delete_collection(self.COLLECTION_NAME)
        logger.warning(f"Deleted collection '{self.COLLECTION_NAME}'")

    def get_collection_info(self) -> Dict:
        """Get information about the collection."""
        try:
            info = self.client.get_collection(self.COLLECTION_NAME)
            return {
                "name": self.COLLECTION_NAME,
                "vectors_count": info.vectors_count,
                "points_count": info.points_count,
                "status": info.status.value
            }
        except Exception as e:
            logger.error(f"Error getting collection info: {e}")
            return {"error": str(e)}
