"""
Dependency Guardian Operations

Standardized operations interface for agent invocation.
"""

from pathlib import Path
from typing import Dict, Any, Optional
from dataclasses import dataclass
import time

# Import core functions
from .core.dependency_analyzer import analyze_dependencies as _analyze_dependencies
from .core.vulnerability_scanner import check_vulnerabilities as _check_vulnerabilities
from .core.update_checker import check_updates as _check_updates


@dataclass
class OperationResult:
    """Result from a skill operation."""
    success: bool
    data: Optional[Dict[str, Any]] = None
    error: Optional[str] = None
    error_code: Optional[str] = None
    duration: float = 0.0
    metadata: Optional[Dict[str, Any]] = None


def analyze_dependencies(
    project_path: str,
    ecosystem: Optional[str] = None,
    **kwargs
) -> OperationResult:
    """
    Analyze all dependencies in a project.

    Args:
        project_path: Path to project directory
        ecosystem: Specific ecosystem to analyze (python, npm, auto)
        **kwargs: Additional parameters

    Returns:
        OperationResult with dependency analysis
    """
    start_time = time.time()

    try:
        result = _analyze_dependencies(project_path, ecosystem)

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=result,
            duration=duration,
            metadata={
                "skill": "dependency-guardian",
                "operation": "analyze_dependencies",
                "version": "0.1.0",
                "ecosystem": ecosystem or "auto"
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
            error=f"Invalid project or ecosystem: {str(e)}",
            error_code="VALIDATION_ERROR",
            duration=time.time() - start_time
        )
    except Exception as e:
        return OperationResult(
            success=False,
            error=f"Dependency analysis failed: {str(e)}",
            error_code="ANALYSIS_ERROR",
            duration=time.time() - start_time
        )


def check_vulnerabilities(
    project_path: str,
    ecosystem: Optional[str] = None,
    include_low: bool = True,
    **kwargs
) -> OperationResult:
    """
    Check dependencies for known security vulnerabilities.

    Args:
        project_path: Path to project directory
        ecosystem: Specific ecosystem to check (python, npm, auto)
        include_low: Include low severity vulnerabilities (default: True)
        **kwargs: Additional parameters

    Returns:
        OperationResult with vulnerability scan results
    """
    start_time = time.time()

    try:
        result = _check_vulnerabilities(project_path, ecosystem, include_low)

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=result,
            duration=duration,
            metadata={
                "skill": "dependency-guardian",
                "operation": "check_vulnerabilities",
                "version": "0.1.0",
                "ecosystem": ecosystem or "auto",
                "include_low": include_low
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
            error=f"Vulnerability check failed: {str(e)}",
            error_code="SCAN_ERROR",
            duration=time.time() - start_time
        )


def check_updates(
    project_path: str,
    ecosystem: Optional[str] = None,
    include_major: bool = False,
    **kwargs
) -> OperationResult:
    """
    Check for available updates to dependencies.

    Args:
        project_path: Path to project directory
        ecosystem: Specific ecosystem to check (python, npm, auto)
        include_major: Include major version updates (default: False)
        **kwargs: Additional parameters

    Returns:
        OperationResult with update check results
    """
    start_time = time.time()

    try:
        result = _check_updates(project_path, ecosystem, include_major)

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=result,
            duration=duration,
            metadata={
                "skill": "dependency-guardian",
                "operation": "check_updates",
                "version": "0.1.0",
                "ecosystem": ecosystem or "auto",
                "include_major": include_major
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
            error=f"Update check failed: {str(e)}",
            error_code="CHECK_ERROR",
            duration=time.time() - start_time
        )


# Export all operations
__all__ = [
    "analyze_dependencies",
    "check_vulnerabilities",
    "check_updates",
    "OperationResult"
]
