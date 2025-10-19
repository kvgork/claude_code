"""
Learning Analytics Skill

Analyzes learning plan data to provide insights, detect struggles,
and generate recommendations for teaching optimization.

This skill builds on learning-plan-manager to provide:
- Learning velocity tracking and trends
- Struggle area detection
- Checkpoint performance analysis
- Learning pattern recognition
- Time estimation accuracy analysis
- Actionable recommendations for teaching optimization
"""

from .models import (
    # Enums
    StruggleIndicator,
    PatternType,
    RecommendationPriority,
    RecommendationType,

    # Models
    VelocityMetrics,
    StruggleArea,
    CheckpointAnalysis,
    CheckpointPerformance,
    LearningPattern,
    TimeEstimationAnalysis,
    LearningRecommendation,
    LearningAnalytics,
)

from .analyzer import LearningAnalyzer

__all__ = [
    # Enums
    "StruggleIndicator",
    "PatternType",
    "RecommendationPriority",
    "RecommendationType",

    # Models
    "VelocityMetrics",
    "StruggleArea",
    "CheckpointAnalysis",
    "CheckpointPerformance",
    "LearningPattern",
    "TimeEstimationAnalysis",
    "LearningRecommendation",
    "LearningAnalytics",

    # Analyzer
    "LearningAnalyzer",
]

__version__ = "1.0.0"
