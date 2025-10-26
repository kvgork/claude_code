"""
Spec to Implementation Operations

Standardized operations interface for agent invocation.
"""

from pathlib import Path
from typing import Dict, Any, Optional
from dataclasses import dataclass
import time

# Import core functions
from .core.orchestrator import (
    implement_from_spec as _implement_from_spec,
    analyze_spec as _analyze_spec
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


def implement_from_spec(
    spec_file: str,
    output_dir: str,
    project_context: Optional[str] = None,
    quality_threshold: float = 85.0,
    include_tests: bool = True,
    include_docs: bool = True,
    **kwargs
) -> OperationResult:
    """
    Transform a specification into working, tested code.

    Args:
        spec_file: Path to specification file
        output_dir: Directory for generated code
        project_context: Optional path to existing project for context
        quality_threshold: Minimum quality score (0-100, default: 85.0)
        include_tests: Generate tests using test-orchestrator (default: True)
        include_docs: Generate documentation (default: True)
        **kwargs: Additional parameters

    Returns:
        OperationResult with implementation results
    """
    start_time = time.time()

    try:
        result = _implement_from_spec(
            spec_file=spec_file,
            output_dir=output_dir,
            project_context=project_context,
            quality_threshold=quality_threshold,
            include_tests=include_tests,
            include_docs=include_docs
        )

        duration = time.time() - start_time

        return OperationResult(
            success=result.get('success', False),
            data=result,
            duration=duration,
            metadata={
                "skill": "spec-to-implementation",
                "operation": "implement_from_spec",
                "version": "0.1.0",
                "quality_threshold": quality_threshold,
                "include_tests": include_tests,
                "include_docs": include_docs
            }
        )

    except FileNotFoundError as e:
        return OperationResult(
            success=False,
            error=f"Specification file not found: {str(e)}",
            error_code="FILE_NOT_FOUND",
            duration=time.time() - start_time
        )
    except ValueError as e:
        return OperationResult(
            success=False,
            error=f"Invalid specification or parameters: {str(e)}",
            error_code="VALIDATION_ERROR",
            duration=time.time() - start_time
        )
    except Exception as e:
        return OperationResult(
            success=False,
            error=f"Implementation failed: {str(e)}",
            error_code="IMPLEMENTATION_ERROR",
            duration=time.time() - start_time
        )


def analyze_spec(
    spec_file: str,
    project_context: Optional[str] = None,
    **kwargs
) -> OperationResult:
    """
    Analyze a specification without implementing it.

    Args:
        spec_file: Path to specification file
        project_context: Optional path to existing project for context
        **kwargs: Additional parameters

    Returns:
        OperationResult with analysis results
    """
    start_time = time.time()

    try:
        result = _analyze_spec(
            spec_file=spec_file,
            project_context=project_context
        )

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=result,
            duration=duration,
            metadata={
                "skill": "spec-to-implementation",
                "operation": "analyze_spec",
                "version": "0.1.0"
            }
        )

    except FileNotFoundError as e:
        return OperationResult(
            success=False,
            error=f"Specification file not found: {str(e)}",
            error_code="FILE_NOT_FOUND",
            duration=time.time() - start_time
        )
    except ValueError as e:
        return OperationResult(
            success=False,
            error=f"Invalid specification: {str(e)}",
            error_code="VALIDATION_ERROR",
            duration=time.time() - start_time
        )
    except Exception as e:
        return OperationResult(
            success=False,
            error=f"Analysis failed: {str(e)}",
            error_code="ANALYSIS_ERROR",
            duration=time.time() - start_time
        )


# Export all operations
__all__ = [
    "implement_from_spec",
    "analyze_spec",
    "OperationResult"
]
