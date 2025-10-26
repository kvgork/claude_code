"""
Refactor Assistant Operations

Standardized operations interface for agent invocation.
"""

from pathlib import Path
from typing import Dict, Any, Optional
from dataclasses import dataclass
import time

# Import core functions
from .core.smell_detector import detect_code_smells as _detect_code_smells
from .core.refactoring_engine import (
    suggest_refactorings as _suggest_refactorings,
    apply_refactoring as _apply_refactoring
)


@dataclass
class OperationResult:
    """Result from a skill operation."""
    success: bool
    data: Optional[Dict[str, Any]] = None
    error: Optional[str] = None
    error_code: Optional[str] = None
    duration: float = 0.0
    metadata: Optional[Dict[str, Any]] = None


def detect_code_smells(file_path: str, severity_threshold: str = "low", **kwargs) -> OperationResult:
    """
    Detect code smells and refactoring opportunities in a file.

    Args:
        file_path: Path to file to analyze
        severity_threshold: Minimum severity (low, medium, high, critical)
        **kwargs: Additional parameters

    Returns:
        OperationResult with detected code smells
    """
    start_time = time.time()

    try:
        result = _detect_code_smells(file_path, severity_threshold)

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=result,
            duration=duration,
            metadata={
                "skill": "refactor-assistant",
                "operation": "detect_code_smells",
                "version": "0.1.0",
                "severity_threshold": severity_threshold
            }
        )

    except FileNotFoundError:
        return OperationResult(
            success=False,
            error=f"File not found: {file_path}",
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
    except Exception as e:
        return OperationResult(
            success=False,
            error=f"Code smell detection failed: {str(e)}",
            error_code="DETECTION_ERROR",
            duration=time.time() - start_time
        )


def suggest_refactorings(file_path: str, max_suggestions: int = 10, **kwargs) -> OperationResult:
    """
    Suggest specific refactorings for code.

    Args:
        file_path: Path to file to analyze
        max_suggestions: Maximum number of suggestions
        **kwargs: Additional parameters

    Returns:
        OperationResult with refactoring suggestions
    """
    start_time = time.time()

    try:
        result = _suggest_refactorings(file_path, max_suggestions)

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=result,
            duration=duration,
            metadata={
                "skill": "refactor-assistant",
                "operation": "suggest_refactorings",
                "version": "0.1.0",
                "max_suggestions": max_suggestions
            }
        )

    except FileNotFoundError:
        return OperationResult(
            success=False,
            error=f"File not found: {file_path}",
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
    except Exception as e:
        return OperationResult(
            success=False,
            error=f"Refactoring suggestion failed: {str(e)}",
            error_code="SUGGESTION_ERROR",
            duration=time.time() - start_time
        )


def apply_refactoring(
    file_path: str,
    refactoring_type: str,
    location: Dict[str, Any],
    parameters: Dict[str, Any],
    run_tests: bool = False,
    **kwargs
) -> OperationResult:
    """
    Apply a refactoring transformation to code.

    Args:
        file_path: Path to file to refactor
        refactoring_type: Type of refactoring (extract_method, rename_symbol, etc.)
        location: Location information (start_line, end_line, etc.)
        parameters: Refactoring-specific parameters
        run_tests: Whether to run tests after refactoring
        **kwargs: Additional parameters

    Returns:
        OperationResult with refactoring results
    """
    start_time = time.time()

    try:
        result = _apply_refactoring(
            file_path=file_path,
            refactoring_type=refactoring_type,
            location=location,
            parameters=parameters,
            run_tests=run_tests
        )

        duration = time.time() - start_time

        return OperationResult(
            success=result.get('success', False),
            data=result,
            duration=duration,
            metadata={
                "skill": "refactor-assistant",
                "operation": "apply_refactoring",
                "version": "0.1.0",
                "refactoring_type": refactoring_type,
                "tests_run": run_tests
            }
        )

    except FileNotFoundError:
        return OperationResult(
            success=False,
            error=f"File not found: {file_path}",
            error_code="FILE_NOT_FOUND",
            duration=time.time() - start_time
        )
    except ValueError as e:
        return OperationResult(
            success=False,
            error=f"Invalid refactoring parameters: {str(e)}",
            error_code="VALIDATION_ERROR",
            duration=time.time() - start_time
        )
    except Exception as e:
        return OperationResult(
            success=False,
            error=f"Refactoring failed: {str(e)}",
            error_code="REFACTORING_ERROR",
            duration=time.time() - start_time
        )


def analyze_complexity(file_path: str, **kwargs) -> OperationResult:
    """
    Analyze code complexity metrics.

    Args:
        file_path: Path to file to analyze
        **kwargs: Additional parameters

    Returns:
        OperationResult with complexity analysis
    """
    start_time = time.time()

    try:
        # Use code smell detection to get complexity metrics
        result = _detect_code_smells(file_path, severity_threshold="low")

        # Extract complexity-related metrics
        complexity_data = {
            "file_path": result.get('file_path'),
            "metrics": result.get('metrics', {}),
            "complexity_issues": [
                smell for smell in result.get('smells', [])
                if smell['type'] in ['complex_function', 'cognitive_complexity', 'deep_nesting']
            ],
            "total_complexity_issues": len([
                smell for smell in result.get('smells', [])
                if smell['type'] in ['complex_function', 'cognitive_complexity', 'deep_nesting']
            ]),
            "recommendations": [
                smell['suggestion'] for smell in result.get('smells', [])
                if smell['type'] in ['complex_function', 'cognitive_complexity', 'deep_nesting']
            ]
        }

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=complexity_data,
            duration=duration,
            metadata={
                "skill": "refactor-assistant",
                "operation": "analyze_complexity",
                "version": "0.1.0"
            }
        )

    except FileNotFoundError:
        return OperationResult(
            success=False,
            error=f"File not found: {file_path}",
            error_code="FILE_NOT_FOUND",
            duration=time.time() - start_time
        )
    except Exception as e:
        return OperationResult(
            success=False,
            error=f"Complexity analysis failed: {str(e)}",
            error_code="ANALYSIS_ERROR",
            duration=time.time() - start_time
        )


# Export all operations
__all__ = [
    "detect_code_smells",
    "suggest_refactorings",
    "apply_refactoring",
    "analyze_complexity",
    "OperationResult"
]
