"""
Doc Generator Skill

Automated documentation generation from code analysis.
"""

from .core.docstring_generator import generate_docstrings
from .core.readme_generator import generate_readme
from .core.doc_analyzer import analyze_documentation

__all__ = [
    "generate_docstrings",
    "generate_readme",
    "analyze_documentation",
]

__version__ = "0.1.0"
