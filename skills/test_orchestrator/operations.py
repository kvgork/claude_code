"""
Test Orchestrator Operations

Standardized operations interface for agent invocation.
"""

from pathlib import Path
from typing import Dict, Any, Optional
from dataclasses import dataclass, asdict
from .core.analyzer import CodeAnalyzer
from .core.test_generator import TestGenerator
from .core.coverage_analyzer import CoverageAnalyzer


@dataclass
class OperationResult:
    """Result from a skill operation."""
    success: bool
    data: Optional[Dict[str, Any]] = None
    error: Optional[str] = None
    error_code: Optional[str] = None
    duration: float = 0.0
    metadata: Optional[Dict[str, Any]] = None


def analyze_file(source_file: str, **kwargs) -> OperationResult:
    """
    Analyze a Python source file to identify testable components.

    Args:
        source_file: Path to the Python source file
        **kwargs: Additional parameters

    Returns:
        OperationResult with analysis data
    """
    import time
    start_time = time.time()

    try:
        analyzer = CodeAnalyzer()
        analysis = analyzer.analyze_file(source_file)

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data={
                "source_file": str(analysis.file_path),
                "functions": [
                    {
                        "name": func.name,
                        "line_number": func.line_number,
                        "complexity": func.complexity,
                        "parameters": func.params,
                        "return_type": func.returns,
                        "raises": func.raises,
                        "edge_cases": func.edge_cases,
                        "dependencies": list(func.dependencies)
                    }
                    for func in analysis.functions
                ],
                "classes": [
                    {
                        "name": cls.name,
                        "line_number": cls.line_number,
                        "base_classes": cls.base_classes,
                        "methods": [method.name for method in cls.methods]
                    }
                    for cls in analysis.classes
                ],
                "imports": list(analysis.imports),
                "total_functions": analysis.total_functions,
                "total_classes": len(analysis.classes),
                "total_complexity": analysis.total_complexity,
                "avg_complexity": analysis.total_complexity / analysis.total_functions if analysis.total_functions > 0 else 0
            },
            duration=duration,
            metadata={
                "skill": "test-orchestrator",
                "operation": "analyze_file",
                "version": "0.1.0"
            }
        )

    except FileNotFoundError:
        return OperationResult(
            success=False,
            error=f"Source file not found: {source_file}",
            error_code="FILE_NOT_FOUND",
            duration=time.time() - start_time
        )
    except Exception as e:
        return OperationResult(
            success=False,
            error=f"Analysis failed: {str(e)}",
            error_code="ANALYSIS_ERROR",
            duration=time.time() - start_time
        )


def generate_tests(source_file: str, target_coverage: float = 80.0, **kwargs) -> OperationResult:
    """
    Generate comprehensive test suite for a Python source file.

    Args:
        source_file: Path to the Python source file
        target_coverage: Target coverage percentage (default: 80.0)
        **kwargs: Additional parameters

    Returns:
        OperationResult with generated tests data
    """
    import time
    start_time = time.time()

    try:
        # First analyze the file
        analyzer = CodeAnalyzer()
        analysis = analyzer.analyze_file(source_file)

        # Generate tests
        generator = TestGenerator()
        test_suite = generator.generate_tests(analysis)

        # Generate test file content
        test_content = generator.generate_test_file(test_suite)

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data={
                "source_file": str(analysis.file_path),
                "test_file": test_suite.file_path,
                "test_content": test_content,
                "tests_generated": len(test_suite.tests),
                "test_breakdown": {
                    "unit_tests": sum(1 for tc in test_suite.tests if tc.test_type == "unit"),
                    "edge_case_tests": sum(1 for tc in test_suite.tests if tc.test_type == "edge_case"),
                    "exception_tests": sum(1 for tc in test_suite.tests if tc.test_type == "exception"),
                    "parametrized_tests": sum(1 for tc in test_suite.tests if tc.test_type == "parametrized")
                },
                "fixtures": test_suite.fixtures,
                "imports": test_suite.imports,
                "completeness_score": getattr(test_suite, 'completeness_score', 0),
                "quality_score": getattr(test_suite, 'quality_score', 0),
                "coverage_estimate": getattr(test_suite, 'completeness_score', 0) * 100  # Convert to percentage
            },
            duration=duration,
            metadata={
                "skill": "test-orchestrator",
                "operation": "generate_tests",
                "version": "0.1.0",
                "target_coverage": target_coverage
            }
        )

    except FileNotFoundError:
        return OperationResult(
            success=False,
            error=f"Source file not found: {source_file}",
            error_code="FILE_NOT_FOUND",
            duration=time.time() - start_time
        )
    except Exception as e:
        return OperationResult(
            success=False,
            error=f"Test generation failed: {str(e)}",
            error_code="GENERATION_ERROR",
            duration=time.time() - start_time
        )


def analyze_coverage(test_results_file: Optional[str] = None, **kwargs) -> OperationResult:
    """
    Analyze test coverage and identify gaps.

    Args:
        test_results_file: Path to pytest coverage report (optional)
        **kwargs: Additional parameters

    Returns:
        OperationResult with coverage analysis
    """
    import time
    start_time = time.time()

    try:
        analyzer = CoverageAnalyzer()

        # Parse coverage results if provided
        if test_results_file:
            coverage_data = analyzer.parse_coverage_report(test_results_file)
        else:
            # Return placeholder data for now
            coverage_data = {
                "overall_coverage": 0.0,
                "files": [],
                "gaps": []
            }

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=coverage_data,
            duration=duration,
            metadata={
                "skill": "test-orchestrator",
                "operation": "analyze_coverage",
                "version": "0.1.0"
            }
        )

    except Exception as e:
        return OperationResult(
            success=False,
            error=f"Coverage analysis failed: {str(e)}",
            error_code="COVERAGE_ERROR",
            duration=time.time() - start_time
        )


# Export all operations
__all__ = [
    "analyze_file",
    "generate_tests",
    "analyze_coverage",
    "OperationResult"
]
