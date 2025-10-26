"""
Release Orchestrator Skill Operations

Provides operations for comprehensive release quality assessment.
"""

import time
import json
from pathlib import Path
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, asdict
import logging

from .core import (
    ReleaseOrchestrator,
    ReportGenerator,
    QualityScorer,
    ReleaseAssessment
)

logger = logging.getLogger(__name__)


@dataclass
class OperationResult:
    """Standard result format for all operations."""
    success: bool
    data: Optional[Dict[str, Any]] = None
    error: Optional[str] = None
    error_code: Optional[str] = None
    duration: float = 0.0
    metadata: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return asdict(self)


def assess_release_quality(
    release_version: str,
    project_path: str,
    baseline_version: Optional[str] = None,
    benchmark_suite: Optional[List[Dict[str, Any]]] = None,
    quality_gate_threshold: float = 70.0,
    enable_performance: bool = True,
    enable_environment: bool = True
) -> OperationResult:
    """
    Perform comprehensive release quality assessment.

    Args:
        release_version: Version identifier for this release
        project_path: Path to project root
        baseline_version: Baseline version for comparison
        benchmark_suite: Optional benchmark suite to run
        quality_gate_threshold: Minimum quality score to pass (0-100)
        enable_performance: Enable performance profiling
        enable_environment: Enable environment profiling

    Returns:
        OperationResult with assessment data
    """
    start_time = time.time()

    try:
        logger.info(f"Assessing release quality for: {release_version}")

        orchestrator = ReleaseOrchestrator(
            quality_gate_threshold=quality_gate_threshold,
            enable_performance=enable_performance,
            enable_environment=enable_environment
        )

        assessment = orchestrator.assess_release(
            release_version=release_version,
            project_path=project_path,
            baseline_version=baseline_version,
            benchmark_suite=benchmark_suite
        )

        duration = time.time() - start_time

        return OperationResult(
            success=assessment.passed_quality_gate,
            data=assessment.to_dict(),
            duration=duration,
            metadata={
                'overall_score': assessment.overall_quality_score,
                'grade': assessment.grade,
                'passed_quality_gate': assessment.passed_quality_gate,
                'warning_count': len(assessment.warnings),
                'recommendation_count': len(assessment.recommendations)
            }
        )

    except Exception as e:
        logger.error(f"Error assessing release quality: {e}")
        return OperationResult(
            success=False,
            error=str(e),
            error_code="ASSESSMENT_ERROR",
            duration=time.time() - start_time
        )


def generate_quality_report(
    assessment_data: Dict[str, Any],
    output_format: str = "markdown",
    output_file: Optional[str] = None
) -> OperationResult:
    """
    Generate release quality report.

    Args:
        assessment_data: Assessment data from assess_release_quality
        output_format: Report format ("json" or "markdown")
        output_file: Optional output file path

    Returns:
        OperationResult with report file path
    """
    start_time = time.time()

    try:
        logger.info(f"Generating {output_format} report")

        # Reconstruct ReleaseAssessment from dict
        from .core.orchestrator import SkillResult

        skill_results = {}
        for skill_name, result_dict in assessment_data['skill_results'].items():
            skill_results[skill_name] = SkillResult(**result_dict)

        assessment = ReleaseAssessment(
            release_version=assessment_data['release_version'],
            timestamp=assessment_data['timestamp'],
            overall_quality_score=assessment_data['overall_quality_score'],
            grade=assessment_data['grade'],
            skill_results=skill_results,
            quality_dimensions=assessment_data['quality_dimensions'],
            recommendations=assessment_data['recommendations'],
            warnings=assessment_data['warnings'],
            passed_quality_gate=assessment_data['passed_quality_gate']
        )

        generator = ReportGenerator()

        if output_format.lower() == "json":
            report_file = generator.generate_json_report(assessment, output_file)
        elif output_format.lower() == "markdown":
            report_file = generator.generate_markdown_report(assessment, output_file)
        else:
            raise ValueError(f"Unsupported format: {output_format}")

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data={
                'report_file': report_file,
                'format': output_format
            },
            duration=duration,
            metadata={
                'file_size_bytes': Path(report_file).stat().st_size
            }
        )

    except Exception as e:
        logger.error(f"Error generating report: {e}")
        return OperationResult(
            success=False,
            error=str(e),
            error_code="REPORT_GENERATION_ERROR",
            duration=time.time() - start_time
        )


def calculate_quality_score(
    code_analysis: Optional[Dict[str, Any]] = None,
    test_analysis: Optional[Dict[str, Any]] = None,
    dependency_analysis: Optional[Dict[str, Any]] = None,
    doc_analysis: Optional[Dict[str, Any]] = None,
    performance_analysis: Optional[Dict[str, Any]] = None,
    custom_weights: Optional[Dict[str, float]] = None
) -> OperationResult:
    """
    Calculate quality score from individual analyses.

    Args:
        code_analysis: Code quality metrics
        test_analysis: Test quality metrics
        dependency_analysis: Dependency metrics
        doc_analysis: Documentation metrics
        performance_analysis: Performance metrics
        custom_weights: Optional custom weights for dimensions

    Returns:
        OperationResult with quality score
    """
    start_time = time.time()

    try:
        logger.info("Calculating quality score")

        scorer = QualityScorer(weights=custom_weights)

        quality_score = scorer.assess_quality(
            release_version="custom",
            code_analysis=code_analysis,
            test_analysis=test_analysis,
            dependency_analysis=dependency_analysis,
            doc_analysis=doc_analysis,
            performance_analysis=performance_analysis
        )

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=quality_score.to_dict(),
            duration=duration,
            metadata={
                'overall_score': quality_score.overall_score,
                'grade': quality_score.grade,
                'dimension_count': len(quality_score.dimensions)
            }
        )

    except Exception as e:
        logger.error(f"Error calculating quality score: {e}")
        return OperationResult(
            success=False,
            error=str(e),
            error_code="SCORE_CALCULATION_ERROR",
            duration=time.time() - start_time
        )


# Export public API
__all__ = [
    'OperationResult',
    'assess_release_quality',
    'generate_quality_report',
    'calculate_quality_score'
]
