"""Chat service for RAG pipeline processing."""

import logging
from typing import Dict, List, Optional

from openai import OpenAI

from app.config import get_settings
from app.services.vector_store import QdrantService

logger = logging.getLogger(__name__)


class ChatService:
    """Service for processing chat queries through RAG pipeline."""

    SYSTEM_PROMPT = """You are an expert AI assistant for a textbook on Physical AI and Humanoid Robotics.
Your role is to help readers understand concepts from the textbook by providing accurate,
educational responses based on the provided context.

Guidelines:
1. Base your answers primarily on the provided context from the textbook.
2. If the context doesn't contain enough information, acknowledge this and provide general guidance.
3. Use clear, educational language appropriate for textbook readers.
4. When relevant, reference specific chapters or sections from the context.
5. If asked about topics outside the textbook scope, politely redirect to the textbook content.
6. Be concise but thorough in your explanations.
7. Use examples from the context when available to illustrate concepts.

Remember: You are a helpful teaching assistant for this specific textbook."""

    def __init__(self):
        self.settings = get_settings()
        self.client = OpenAI(api_key=self.settings.openai_api_key)
        self.qdrant_service = QdrantService()

    def _get_embedding(self, text: str) -> List[float]:
        """Generate embedding for text using OpenAI."""
        response = self.client.embeddings.create(
            model=self.settings.embedding_model,
            input=text
        )
        return response.data[0].embedding

    def _build_context(self, chunks: List[Dict]) -> str:
        """Build context string from retrieved chunks."""
        if not chunks:
            return "No relevant context found in the textbook."

        context_parts = []
        for i, chunk in enumerate(chunks, 1):
            chapter = chunk["metadata"]["chapter"]
            section = chunk["metadata"].get("section", "")
            section_info = f", Section: {section}" if section else ""

            context_parts.append(
                f"[Source {i} - Chapter: {chapter}{section_info}]\n{chunk['text']}"
            )

        return "\n\n---\n\n".join(context_parts)

    def _build_messages(
        self,
        query: str,
        context: str,
        conversation_history: List[Dict],
        selected_text: Optional[str] = None
    ) -> List[Dict]:
        """Build message list for chat completion."""
        messages = [{"role": "system", "content": self.SYSTEM_PROMPT}]

        # Add conversation history (last few exchanges)
        for exchange in reversed(conversation_history[-3:]):
            messages.append({"role": "user", "content": exchange["query"]})
            messages.append({"role": "assistant", "content": exchange["answer"]})

        # Build user message with context
        user_content = f"""Context from the textbook:
{context}

"""
        if selected_text:
            user_content += f"""The user has selected the following text for context:
"{selected_text}"

"""

        user_content += f"""User question: {query}

Please provide a helpful response based on the textbook content."""

        messages.append({"role": "user", "content": user_content})

        return messages

    async def process_query(
        self,
        query: str,
        conversation_history: Optional[List[Dict]] = None
    ) -> Dict:
        """
        Process a query through the RAG pipeline.

        Args:
            query: User's question
            conversation_history: Previous conversation exchanges

        Returns:
            Dict with 'answer' and 'sources'
        """
        conversation_history = conversation_history or []

        # Generate query embedding
        query_embedding = self._get_embedding(query)

        # Search for relevant chunks
        chunks = self.qdrant_service.search(
            query_vector=query_embedding,
            top_k=self.settings.top_k_results,
            score_threshold=self.settings.similarity_threshold
        )

        # Build context from chunks
        context = self._build_context(chunks)

        # Build messages for completion
        messages = self._build_messages(query, context, conversation_history)

        # Generate response
        response = self.client.chat.completions.create(
            model=self.settings.chat_model,
            messages=messages,
            temperature=0.7,
            max_tokens=1000
        )

        answer = response.choices[0].message.content

        logger.info(f"Processed query with {len(chunks)} sources")

        return {
            "answer": answer,
            "sources": chunks
        }

    async def process_query_with_context(
        self,
        query: str,
        selected_text: str,
        context_metadata: Optional[Dict] = None,
        conversation_history: Optional[List[Dict]] = None
    ) -> Dict:
        """
        Process a query with user-selected text as additional context.

        Args:
            query: User's question
            selected_text: Text selected by user from the textbook
            context_metadata: Optional metadata about the selection location
            conversation_history: Previous conversation exchanges

        Returns:
            Dict with 'answer' and 'sources'
        """
        conversation_history = conversation_history or []

        # Generate embedding combining query and selected text
        combined_text = f"{query} {selected_text[:500]}"
        query_embedding = self._get_embedding(combined_text)

        # Search for relevant chunks
        chunks = self.qdrant_service.search(
            query_vector=query_embedding,
            top_k=self.settings.top_k_results,
            score_threshold=self.settings.similarity_threshold
        )

        # Build context from chunks
        context = self._build_context(chunks)

        # Build messages with selected text
        messages = self._build_messages(
            query, context, conversation_history, selected_text
        )

        # Generate response
        response = self.client.chat.completions.create(
            model=self.settings.chat_model,
            messages=messages,
            temperature=0.7,
            max_tokens=1000
        )

        answer = response.choices[0].message.content

        logger.info(
            f"Processed query with selected text ({len(selected_text)} chars), "
            f"{len(chunks)} sources"
        )

        return {
            "answer": answer,
            "sources": chunks
        }
