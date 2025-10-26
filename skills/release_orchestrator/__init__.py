"""
Release Orchestrator Skill

Coordinate multiple skills for comprehensive release quality assessment.
"""

# Import operations for agent invocation
from .operations import (
    assess_release_quality,
    generate_quality_report,
    calculate_quality_score,
    OperationResult
)

# Import core modules for direct use
from .core import (
    ReleaseOrchestrator,
    ReportGenerator,
    QualityScorer,
    ReleaseAssessment,
    QualityScore,
    QualityDimension,
    SkillResult
)

__all__ = [
    # Operations
    "assess_release_quality",
    "generate_quality_report",
    "calculate_quality_score",
    "OperationResult",

    # Core Classes
    "ReleaseOrchestrator",
    "ReportGenerator",
    "QualityScorer",

    # Data Classes
    "ReleaseAssessment",
    "QualityScore",
    "QualityDimension",
    "SkillResult"
]

__version__ = "0.1.0"
