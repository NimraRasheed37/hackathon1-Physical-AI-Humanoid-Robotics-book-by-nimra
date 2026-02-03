"""Text chunking utilities for RAG pipeline."""

import re
from typing import List


def chunk_text(
    text: str,
    chunk_size: int = 1000,
    overlap: int = 200,
    separators: List[str] | None = None
) -> List[str]:
    """
    Split text into overlapping chunks.

    Uses a hierarchical approach, preferring to split at natural boundaries
    (paragraphs, sentences) rather than arbitrary character positions.

    Args:
        text: Text to split into chunks
        chunk_size: Target size for each chunk in characters
        overlap: Number of characters to overlap between chunks
        separators: List of separators to try, in order of preference

    Returns:
        List of text chunks
    """
    if not text or not text.strip():
        return []

    if separators is None:
        separators = ["\n\n", "\n", ". ", ", ", " "]

    # Clean and normalize text
    text = text.strip()
    text = re.sub(r'\n{3,}', '\n\n', text)  # Normalize multiple newlines

    # If text is smaller than chunk size, return as single chunk
    if len(text) <= chunk_size:
        return [text]

    chunks = []
    current_chunk = ""

    # Split by preferred separator
    def split_recursive(text: str, sep_index: int = 0) -> List[str]:
        """Recursively split text using separators in order of preference."""
        if sep_index >= len(separators):
            # No more separators, split by character
            return [text[i:i + chunk_size] for i in range(0, len(text), chunk_size - overlap)]

        separator = separators[sep_index]
        parts = text.split(separator)

        if len(parts) == 1:
            # Separator not found, try next
            return split_recursive(text, sep_index + 1)

        result = []
        current = ""

        for part in parts:
            # Add separator back (except for first part)
            if current:
                test_chunk = current + separator + part
            else:
                test_chunk = part

            if len(test_chunk) <= chunk_size:
                current = test_chunk
            else:
                if current:
                    result.append(current)
                # If part itself is too large, recursively split it
                if len(part) > chunk_size:
                    result.extend(split_recursive(part, sep_index + 1))
                    current = ""
                else:
                    current = part

        if current:
            result.append(current)

        return result

    raw_chunks = split_recursive(text)

    # Apply overlap
    final_chunks = []
    for i, chunk in enumerate(raw_chunks):
        if i > 0 and overlap > 0:
            # Get overlap from previous chunk
            prev_chunk = raw_chunks[i - 1]
            overlap_text = prev_chunk[-overlap:] if len(prev_chunk) >= overlap else prev_chunk
            # Find a good break point in the overlap
            break_point = overlap_text.rfind(' ')
            if break_point > 0:
                overlap_text = overlap_text[break_point + 1:]
            chunk = overlap_text + " " + chunk

        final_chunks.append(chunk.strip())

    # Filter out empty chunks
    return [c for c in final_chunks if c.strip()]


def chunk_by_tokens(
    text: str,
    max_tokens: int = 500,
    overlap_tokens: int = 50
) -> List[str]:
    """
    Split text into chunks based on approximate token count.

    Uses a simple heuristic: ~4 characters per token for English text.

    Args:
        text: Text to split
        max_tokens: Maximum tokens per chunk
        overlap_tokens: Token overlap between chunks

    Returns:
        List of text chunks
    """
    # Approximate: 1 token ≈ 4 characters for English
    chars_per_token = 4
    chunk_size = max_tokens * chars_per_token
    overlap = overlap_tokens * chars_per_token

    return chunk_text(text, chunk_size=chunk_size, overlap=overlap)


def estimate_tokens(text: str) -> int:
    """
    Estimate token count for text.

    Uses simple heuristic: ~4 characters per token for English.

    Args:
        text: Text to estimate tokens for

    Returns:
        Estimated token count
    """
    if not text:
        return 0
    return len(text) // 4
