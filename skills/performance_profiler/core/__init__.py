"""
Performance Profiler Core Modules

Provides comprehensive performance profiling capabilities including:
- Distributed tracing (Jaeger integration)
- Function profiling (cProfile)
- Resource monitoring (CPU, memory, disk, network)
- Benchmark execution and comparison
"""

from .tracer import (
    JaegerTracer,
    TracingContext,
    Trace,
    Span
)

from .profiler import (
    FunctionProfiler,
    ProfileResult,
    profile_decorator
)

from .resource_monitor import (
    ResourceMonitor,
    ResourceSnapshot,
    ResourceUsageResult,
    PSUTIL_AVAILABLE
)

from .benchmarker import (
    BenchmarkRunner,
    BenchmarkResult,
    BenchmarkComparison,
    RegressionAnalysis
)

__all__ = [
    # Tracing
    'JaegerTracer',
    'TracingContext',
    'Trace',
    'Span',

    # Profiling
    'FunctionProfiler',
    'ProfileResult',
    'profile_decorator',

    # Resource Monitoring
    'ResourceMonitor',
    'ResourceSnapshot',
    'ResourceUsageResult',
    'PSUTIL_AVAILABLE',

    # Benchmarking
    'BenchmarkRunner',
    'BenchmarkResult',
    'BenchmarkComparison',
    'RegressionAnalysis'
]

__version__ = '0.1.0'
