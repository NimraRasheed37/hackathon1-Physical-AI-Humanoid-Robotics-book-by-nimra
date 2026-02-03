"""Docusaurus markdown parser for extracting structured content."""

import re
from typing import Dict, List, Optional


def parse_frontmatter(content: str) -> tuple[Dict, str]:
    """
    Extract YAML frontmatter from markdown content.

    Args:
        content: Full markdown content

    Returns:
        Tuple of (frontmatter dict, remaining content)
    """
    frontmatter = {}
    remaining = content

    # Check for frontmatter (starts with ---)
    if content.startswith("---"):
        parts = content.split("---", 2)
        if len(parts) >= 3:
            fm_content = parts[1].strip()
            remaining = parts[2].strip()

            # Parse simple YAML (key: value pairs)
            for line in fm_content.split("\n"):
                line = line.strip()
                if ":" in line:
                    key, value = line.split(":", 1)
                    key = key.strip()
                    value = value.strip().strip('"').strip("'")
                    frontmatter[key] = value

    return frontmatter, remaining


def extract_title(content: str, frontmatter: Dict) -> str:
    """
    Extract document title from frontmatter or first heading.

    Args:
        content: Markdown content (without frontmatter)
        frontmatter: Parsed frontmatter dict

    Returns:
        Document title
    """
    # Check frontmatter first
    if "title" in frontmatter:
        return frontmatter["title"]

    # Look for first h1 heading
    h1_match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
    if h1_match:
        return h1_match.group(1).strip()

    return "Untitled"


def extract_sections(content: str) -> List[Dict]:
    """
    Extract sections from markdown content based on headings.

    Args:
        content: Markdown content

    Returns:
        List of section dicts with 'heading', 'level', and 'content'
    """
    sections = []

    # Pattern to match markdown headings (## to ####)
    heading_pattern = re.compile(r'^(#{1,4})\s+(.+)$', re.MULTILINE)

    # Find all headings with their positions
    headings = []
    for match in heading_pattern.finditer(content):
        level = len(match.group(1))
        heading_text = match.group(2).strip()
        start = match.start()
        end = match.end()
        headings.append({
            "level": level,
            "heading": heading_text,
            "start": start,
            "end": end
        })

    # Extract content between headings
    if not headings:
        # No headings, treat entire content as one section
        cleaned = clean_markdown_content(content)
        if cleaned.strip():
            sections.append({
                "heading": None,
                "level": 0,
                "content": cleaned
            })
    else:
        # Content before first heading
        if headings[0]["start"] > 0:
            pre_content = content[:headings[0]["start"]]
            cleaned = clean_markdown_content(pre_content)
            if cleaned.strip():
                sections.append({
                    "heading": None,
                    "level": 0,
                    "content": cleaned
                })

        # Process each heading and its content
        for i, heading in enumerate(headings):
            content_start = heading["end"]
            content_end = headings[i + 1]["start"] if i + 1 < len(headings) else len(content)

            section_content = content[content_start:content_end]
            cleaned = clean_markdown_content(section_content)

            sections.append({
                "heading": heading["heading"],
                "level": heading["level"],
                "content": cleaned
            })

    return sections


def clean_markdown_content(content: str) -> str:
    """
    Clean markdown content for embedding.

    Removes or simplifies markdown syntax while preserving text meaning.

    Args:
        content: Raw markdown content

    Returns:
        Cleaned text content
    """
    # Remove HTML comments
    content = re.sub(r'<!--.*?-->', '', content, flags=re.DOTALL)

    # Remove Docusaurus-specific admonitions but keep content
    content = re.sub(r':::\w+\s*\n', '', content)
    content = re.sub(r':::', '', content)

    # Remove import statements (MDX)
    content = re.sub(r'^import\s+.*$', '', content, flags=re.MULTILINE)

    # Remove JSX/React components (but this is simplistic)
    content = re.sub(r'<[A-Z][a-zA-Z]*[^>]*/?>', '', content)
    content = re.sub(r'</[A-Z][a-zA-Z]*>', '', content)

    # Convert links to just text: [text](url) -> text
    content = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', content)

    # Remove images: ![alt](url) -> alt
    content = re.sub(r'!\[([^\]]*)\]\([^)]+\)', r'\1', content)

    # Remove inline code backticks but keep content
    content = re.sub(r'`([^`]+)`', r'\1', content)

    # Remove code blocks but keep content (simplified)
    content = re.sub(r'```[\w]*\n(.*?)```', r'\1', content, flags=re.DOTALL)

    # Remove bold/italic markers
    content = re.sub(r'\*\*([^*]+)\*\*', r'\1', content)
    content = re.sub(r'\*([^*]+)\*', r'\1', content)
    content = re.sub(r'__([^_]+)__', r'\1', content)
    content = re.sub(r'_([^_]+)_', r'\1', content)

    # Clean up excessive whitespace
    content = re.sub(r'\n{3,}', '\n\n', content)
    content = re.sub(r' +', ' ', content)

    return content.strip()


def parse_docusaurus_markdown(content: str, file_path: str = "") -> Dict:
    """
    Parse Docusaurus markdown/MDX file into structured format.

    Args:
        content: Full file content
        file_path: Optional file path for metadata

    Returns:
        Dict with 'title', 'frontmatter', 'sections', 'file_path'
    """
    # Extract frontmatter
    frontmatter, body = parse_frontmatter(content)

    # Extract title
    title = extract_title(body, frontmatter)

    # Extract sections
    sections = extract_sections(body)

    return {
        "title": title,
        "frontmatter": frontmatter,
        "sections": sections,
        "file_path": file_path
    }


def get_section_hierarchy(sections: List[Dict]) -> List[Dict]:
    """
    Build hierarchical structure from flat sections list.

    Args:
        sections: List of section dicts with 'heading', 'level', 'content'

    Returns:
        Nested list with parent-child relationships
    """
    hierarchy = []
    stack = []

    for section in sections:
        level = section["level"]

        # Pop from stack until we find a parent
        while stack and stack[-1]["level"] >= level:
            stack.pop()

        if stack:
            # Add as child of current parent
            if "children" not in stack[-1]:
                stack[-1]["children"] = []
            stack[-1]["children"].append(section)
        else:
            # Top-level section
            hierarchy.append(section)

        stack.append(section)

    return hierarchy
