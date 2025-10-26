"""
Doc Generator Operations

Standardized operations interface for agent invocation.
"""

from pathlib import Path
from typing import Dict, Any, Optional
from dataclasses import dataclass
import time

# Import core functions
from .core.docstring_generator import generate_docstrings as _generate_docstrings
from .core.readme_generator import generate_readme as _generate_readme
from .core.doc_analyzer import analyze_documentation as _analyze_documentation


@dataclass
class OperationResult:
    """Result from a skill operation."""
    success: bool
    data: Optional[Dict[str, Any]] = None
    error: Optional[str] = None
    error_code: Optional[str] = None
    duration: float = 0.0
    metadata: Optional[Dict[str, Any]] = None


def generate_docstrings(
    file_path: str,
    style: str = "google",
    include_examples: bool = False,
    **kwargs
) -> OperationResult:
    """
    Generate comprehensive docstrings for Python code.

    Args:
        file_path: Path to Python file
        style: Docstring style ('google', 'numpy', or 'sphinx')
        include_examples: Include usage examples in docstrings
        **kwargs: Additional parameters

    Returns:
        OperationResult with docstring generation results
    """
    start_time = time.time()

    try:
        result = _generate_docstrings(
            file_path=file_path,
            style=style,
            include_examples=include_examples
        )

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=result,
            duration=duration,
            metadata={
                "skill": "doc-generator",
                "operation": "generate_docstrings",
                "version": "0.1.0",
                "style": style,
                "include_examples": include_examples
            }
        )

    except FileNotFoundError as e:
        return OperationResult(
            success=False,
            error=f"File not found: {str(e)}",
            error_code="FILE_NOT_FOUND",
            duration=time.time() - start_time
        )
    except SyntaxError as e:
        return OperationResult(
            success=False,
            error=f"Syntax error in file: {str(e)}",
            error_code="SYNTAX_ERROR",
            duration=time.time() - start_time
        )
    except ValueError as e:
        return OperationResult(
            success=False,
            error=f"Invalid docstring style: {str(e)}",
            error_code="VALIDATION_ERROR",
            duration=time.time() - start_time
        )
    except Exception as e:
        return OperationResult(
            success=False,
            error=f"Docstring generation failed: {str(e)}",
            error_code="GENERATION_ERROR",
            duration=time.time() - start_time
        )


def generate_readme(
    project_path: str,
    include_api: bool = True,
    include_examples: bool = True,
    **kwargs
) -> OperationResult:
    """
    Generate comprehensive README.md from project analysis.

    Args:
        project_path: Path to project root directory
        include_api: Include API reference section
        include_examples: Include usage examples section
        **kwargs: Additional parameters

    Returns:
        OperationResult with README generation results
    """
    start_time = time.time()

    try:
        result = _generate_readme(
            project_path=project_path,
            include_api=include_api,
            include_examples=include_examples
        )

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=result,
            duration=duration,
            metadata={
                "skill": "doc-generator",
                "operation": "generate_readme",
                "version": "0.1.0",
                "include_api": include_api,
                "include_examples": include_examples
            }
        )

    except FileNotFoundError as e:
        return OperationResult(
            success=False,
            error=f"Project path not found: {str(e)}",
            error_code="FILE_NOT_FOUND",
            duration=time.time() - start_time
        )
    except Exception as e:
        return OperationResult(
            success=False,
            error=f"README generation failed: {str(e)}",
            error_code="GENERATION_ERROR",
            duration=time.time() - start_time
        )


def analyze_documentation(
    project_path: str,
    **kwargs
) -> OperationResult:
    """
    Analyze existing documentation coverage and quality.

    Args:
        project_path: Path to project root directory
        **kwargs: Additional parameters

    Returns:
        OperationResult with documentation analysis
    """
    start_time = time.time()

    try:
        result = _analyze_documentation(project_path=project_path)

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=result,
            duration=duration,
            metadata={
                "skill": "doc-generator",
                "operation": "analyze_documentation",
                "version": "0.1.0"
            }
        )

    except FileNotFoundError as e:
        return OperationResult(
            success=False,
            error=f"Project path not found: {str(e)}",
            error_code="FILE_NOT_FOUND",
            duration=time.time() - start_time
        )
    except Exception as e:
        return OperationResult(
            success=False,
            error=f"Documentation analysis failed: {str(e)}",
            error_code="ANALYSIS_ERROR",
            duration=time.time() - start_time
        )


# Export all operations
__all__ = [
    "generate_docstrings",
    "generate_readme",
    "analyze_documentation",
    "OperationResult"
]
