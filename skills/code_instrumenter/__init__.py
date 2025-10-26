"""
Code Instrumenter Skill

Automatically add instrumentation (tracing, profiling, metrics) to existing codebases.
"""

# Import operations for agent invocation
from .operations import (
    analyze_codebase,
    add_tracing,
    add_profiling,
    add_metrics,
    add_health_checks,
    generate_config,
    OperationResult
)

__all__ = [
    # Operations
    "analyze_codebase",
    "add_tracing",
    "add_profiling",
    "add_metrics",
    "add_health_checks",
    "generate_config",
    "OperationResult"
]

__version__ = "0.1.0"
