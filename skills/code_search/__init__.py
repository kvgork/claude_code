"""
Code Search Skill

Intelligent code search with AST-based indexing and semantic understanding.
"""

from .core.search_engine import search_symbol, search_pattern
from .core.definition_finder import find_definition
from .core.usage_finder import find_usages

__all__ = [
    "search_symbol",
    "search_pattern",
    "find_definition",
    "find_usages",
]

__version__ = "0.1.0"
