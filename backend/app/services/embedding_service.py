"""Embedding service for processing and storing book content."""

import hashlib
import logging
import uuid
from pathlib import Path
from typing import Dict, List, Optional

from openai import OpenAI

from app.config import get_settings
from app.services.postgres_service import PostgresService
from app.services.vector_store import QdrantService
from app.utils.chunking import chunk_text
from app.utils.markdown_parser import parse_docusaurus_markdown

logger = logging.getLogger(__name__)


class EmbeddingService:
    """Service for embedding book content into vector store."""

    def __init__(self):
        self.settings = get_settings()
        self.client = OpenAI(api_key=self.settings.openai_api_key)
        self.qdrant_service = QdrantService()
        self.postgres_service = PostgresService()

    def _get_content_hash(self, content: str) -> str:
        """Generate hash of content for change detection."""
        return hashlib.sha256(content.encode()).hexdigest()

    def _generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for a batch of texts."""
        # OpenAI allows batching up to 2048 inputs
        batch_size = 100
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            response = self.client.embeddings.create(
                model=self.settings.embedding_model,
                input=batch
            )
            embeddings = [item.embedding for item in response.data]
            all_embeddings.extend(embeddings)

        return all_embeddings

    def _generate_chunk_id(self, file_path: str, chunk_index: int) -> str:
        """Generate unique UUID for a chunk based on file path and index."""
        # Create a deterministic UUID from file path and chunk index
        unique_string = f"{file_path}:{chunk_index}"
        # Use UUID5 with a namespace for deterministic IDs
        return str(uuid.uuid5(uuid.NAMESPACE_DNS, unique_string))

    async def embed_book(
        self,
        book_path: str = "./docs",
        force_reembed: bool = False
    ) -> Dict:
        """
        Embed all book content from the specified path.

        Args:
            book_path: Path to book content directory
            force_reembed: Force re-embedding even if content unchanged

        Returns:
            Dict with processing statistics
        """
        book_dir = Path(book_path)
        if not book_dir.exists():
            raise ValueError(f"Book path does not exist: {book_path}")

        # Ensure collection exists
        self.qdrant_service.ensure_collection()

        # Find all markdown files
        md_files = list(book_dir.glob("**/*.md")) + list(book_dir.glob("**/*.mdx"))
        logger.info(f"Found {len(md_files)} markdown files")

        total_chunks = 0
        processed_files = 0
        skipped_files = 0

        for md_file in md_files:
            try:
                result = await self._process_file(md_file, force_reembed)
                if result["processed"]:
                    total_chunks += result["chunks"]
                    processed_files += 1
                else:
                    skipped_files += 1
            except Exception as e:
                logger.error(f"Error processing {md_file}: {e}")
                continue

        logger.info(
            f"Embedding complete: {processed_files} files processed, "
            f"{skipped_files} skipped, {total_chunks} chunks created"
        )

        return {
            "status": "completed",
            "files_processed": processed_files,
            "files_skipped": skipped_files,
            "chunks_processed": total_chunks
        }

    async def _process_file(
        self,
        file_path: Path,
        force_reembed: bool
    ) -> Dict:
        """Process a single markdown file."""
        content = file_path.read_text(encoding="utf-8")
        content_hash = self._get_content_hash(content)

        # Check if file needs processing (skip if unchanged and not forcing)
        if not force_reembed:
            # In a full implementation, we'd check the database here
            # For now, we process all files
            pass

        # Parse markdown content
        parsed = parse_docusaurus_markdown(content, str(file_path))
        chapter_name = parsed.get("title", file_path.stem)
        sections = parsed.get("sections", [])

        # Chunk the content
        all_chunks = []
        chunk_index = 0

        for section in sections:
            section_name = section.get("heading")
            section_content = section.get("content", "")

            if not section_content.strip():
                continue

            # Split section into chunks
            text_chunks = chunk_text(
                section_content,
                chunk_size=self.settings.chunk_size,
                overlap=self.settings.chunk_overlap
            )

            for text in text_chunks:
                chunk_id = self._generate_chunk_id(str(file_path), chunk_index)
                all_chunks.append({
                    "id": chunk_id,
                    "text": text,
                    "metadata": {
                        "chapter": chapter_name,
                        "section": section_name,
                        "file_path": str(file_path),
                        "chunk_index": chunk_index
                    }
                })
                chunk_index += 1

        if not all_chunks:
            logger.debug(f"No chunks generated for {file_path}")
            return {"processed": False, "chunks": 0}

        # Generate embeddings
        texts = [chunk["text"] for chunk in all_chunks]
        embeddings = self._generate_embeddings(texts)

        # Add embeddings to chunks
        for chunk, embedding in zip(all_chunks, embeddings):
            chunk["embedding"] = embedding

        # Upsert to vector store
        self.qdrant_service.upsert_chunks(all_chunks)

        # Update metadata in postgres
        await self.postgres_service.update_book_metadata(
            file_path=str(file_path),
            chapter_name=chapter_name,
            section_name=None,  # Multiple sections per file
            chunk_count=len(all_chunks),
            content_hash=content_hash
        )

        logger.info(f"Processed {file_path}: {len(all_chunks)} chunks")

        return {"processed": True, "chunks": len(all_chunks)}

    async def embed_single_file(
        self,
        file_path: str,
        force_reembed: bool = False
    ) -> Dict:
        """Embed a single file."""
        path = Path(file_path)
        if not path.exists():
            raise ValueError(f"File does not exist: {file_path}")

        self.qdrant_service.ensure_collection()
        return await self._process_file(path, force_reembed)
