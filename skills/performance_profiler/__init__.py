"""
Performance Profiler Skill

Comprehensive performance profiling and benchmarking for release quality assessment.
"""

# Import operations for agent invocation
from .operations import (
    profile_code_execution,
    capture_distributed_traces,
    monitor_resource_usage,
    run_benchmark_suite,
    compare_with_baseline,
    generate_performance_report,
    OperationResult
)

# Import core modules for direct use
from .core import (
    JaegerTracer,
    TracingContext,
    FunctionProfiler,
    ResourceMonitor,
    BenchmarkRunner,
    ProfileResult,
    ResourceUsageResult,
    BenchmarkResult,
    BenchmarkComparison,
    RegressionAnalysis
)

__all__ = [
    # Operations
    "profile_code_execution",
    "capture_distributed_traces",
    "monitor_resource_usage",
    "run_benchmark_suite",
    "compare_with_baseline",
    "generate_performance_report",
    "OperationResult",

    # Core Classes
    "JaegerTracer",
    "TracingContext",
    "FunctionProfiler",
    "ResourceMonitor",
    "BenchmarkRunner",

    # Data Classes
    "ProfileResult",
    "ResourceUsageResult",
    "BenchmarkResult",
    "BenchmarkComparison",
    "RegressionAnalysis"
]

__version__ = "0.1.0"
