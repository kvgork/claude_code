"""
Code Search Operations

Standardized operations interface for agent invocation.
"""

from pathlib import Path
from typing import Dict, Any, Optional
from dataclasses import dataclass
import time

# Import core functions
from .core.search_engine import search_symbol as _search_symbol
from .core.search_engine import search_pattern as _search_pattern
from .core.definition_finder import find_definition as _find_definition
from .core.usage_finder import find_usages as _find_usages


@dataclass
class OperationResult:
    """Result from a skill operation."""
    success: bool
    data: Optional[Dict[str, Any]] = None
    error: Optional[str] = None
    error_code: Optional[str] = None
    duration: float = 0.0
    metadata: Optional[Dict[str, Any]] = None


def search_symbol(
    project_path: str,
    symbol_name: str,
    symbol_type: str = "all",
    exact_match: bool = False,
    **kwargs
) -> OperationResult:
    """
    Find symbols (functions, classes, variables) in codebase.

    Args:
        project_path: Path to project root
        symbol_name: Symbol to search for
        symbol_type: Type of symbol ('function', 'class', 'variable', 'all')
        exact_match: Exact match or fuzzy search
        **kwargs: Additional parameters

    Returns:
        OperationResult with symbol search results
    """
    start_time = time.time()

    try:
        result = _search_symbol(
            project_path=project_path,
            symbol_name=symbol_name,
            symbol_type=symbol_type,
            exact_match=exact_match
        )

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=result,
            duration=duration,
            metadata={
                "skill": "code-search",
                "operation": "search_symbol",
                "version": "0.1.0",
                "symbol_type": symbol_type,
                "exact_match": exact_match
            }
        )

    except FileNotFoundError as e:
        return OperationResult(
            success=False,
            error=f"Project path not found: {str(e)}",
            error_code="FILE_NOT_FOUND",
            duration=time.time() - start_time
        )
    except ValueError as e:
        return OperationResult(
            success=False,
            error=f"Invalid symbol type: {str(e)}",
            error_code="VALIDATION_ERROR",
            duration=time.time() - start_time
        )
    except Exception as e:
        return OperationResult(
            success=False,
            error=f"Symbol search failed: {str(e)}",
            error_code="SEARCH_ERROR",
            duration=time.time() - start_time
        )


def search_pattern(
    project_path: str,
    pattern: str,
    pattern_type: str = "ast",
    **kwargs
) -> OperationResult:
    """
    Search for code patterns in codebase.

    Args:
        project_path: Path to project root
        pattern: Pattern to search for
        pattern_type: Type of pattern ('ast', 'regex', 'text')
        **kwargs: Additional parameters

    Returns:
        OperationResult with pattern search results
    """
    start_time = time.time()

    try:
        result = _search_pattern(
            project_path=project_path,
            pattern=pattern,
            pattern_type=pattern_type
        )

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=result,
            duration=duration,
            metadata={
                "skill": "code-search",
                "operation": "search_pattern",
                "version": "0.1.0",
                "pattern_type": pattern_type
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
            error=f"Pattern search failed: {str(e)}",
            error_code="SEARCH_ERROR",
            duration=time.time() - start_time
        )


def find_definition(
    project_path: str,
    symbol_name: str,
    file_context: Optional[str] = None,
    **kwargs
) -> OperationResult:
    """
    Find the definition of a symbol.

    Args:
        project_path: Path to project root
        symbol_name: Symbol to find definition for
        file_context: Optional file where symbol is used (for context)
        **kwargs: Additional parameters

    Returns:
        OperationResult with symbol definition
    """
    start_time = time.time()

    try:
        result = _find_definition(
            project_path=project_path,
            symbol_name=symbol_name,
            file_context=file_context
        )

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=result,
            duration=duration,
            metadata={
                "skill": "code-search",
                "operation": "find_definition",
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
            error=f"Definition search failed: {str(e)}",
            error_code="SEARCH_ERROR",
            duration=time.time() - start_time
        )


def find_usages(
    project_path: str,
    symbol_name: str,
    file_path: Optional[str] = None,
    include_tests: bool = True,
    **kwargs
) -> OperationResult:
    """
    Find all usages of a symbol.

    Args:
        project_path: Path to project root
        symbol_name: Symbol to find usages for
        file_path: Optional specific file to search in
        include_tests: Include test files in search
        **kwargs: Additional parameters

    Returns:
        OperationResult with symbol usages
    """
    start_time = time.time()

    try:
        result = _find_usages(
            project_path=project_path,
            symbol_name=symbol_name,
            definition_file=file_path
        )

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=result,
            duration=duration,
            metadata={
                "skill": "code-search",
                "operation": "find_usages",
                "version": "0.1.0",
                "include_tests": include_tests
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
            error=f"Usage search failed: {str(e)}",
            error_code="SEARCH_ERROR",
            duration=time.time() - start_time
        )


# Export all operations
__all__ = [
    "search_symbol",
    "search_pattern",
    "find_definition",
    "find_usages",
    "OperationResult"
]
