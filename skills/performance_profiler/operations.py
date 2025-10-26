"""
Performance Profiler Skill Operations

Integrates all core modules to provide a unified skill interface for performance profiling.
"""

import time
import json
from pathlib import Path
from typing import Dict, List, Optional, Any, Callable
from dataclasses import dataclass, asdict
import logging

from .core import (
    JaegerTracer,
    FunctionProfiler,
    ResourceMonitor,
    BenchmarkRunner,
    ProfileResult,
    ResourceUsageResult,
    BenchmarkResult,
    BenchmarkComparison
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


def profile_code_execution(
    target_function: Callable,
    function_args: tuple = (),
    function_kwargs: Optional[Dict] = None,
    save_profile: bool = True,
    top_n_functions: int = 10,
    output_dir: str = "./profiles"
) -> OperationResult:
    """
    Profile Python code execution using cProfile.

    Args:
        target_function: Function to profile
        function_args: Arguments for the function
        function_kwargs: Keyword arguments for the function
        save_profile: Save profile data to file
        top_n_functions: Number of top functions to include in result
        output_dir: Directory to store profile outputs

    Returns:
        OperationResult with profiling data
    """
    start_time = time.time()
    function_kwargs = function_kwargs or {}

    try:
        logger.info(f"Profiling function: {target_function.__name__}")

        profiler = FunctionProfiler(output_dir=output_dir)
        result = profiler.profile_function(
            target_function,
            *function_args,
            save_profile=save_profile,
            top_n=top_n_functions,
            **function_kwargs
        )

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data={
                'function_name': result.function_name,
                'total_time': result.total_time,
                'primitive_calls': result.primitive_calls,
                'total_calls': result.total_calls,
                'top_functions': result.top_functions,
                'profile_file': result.profile_file
            },
            duration=duration,
            metadata={
                'profiler': 'cProfile',
                'top_n': top_n_functions,
                'saved': save_profile
            }
        )

    except Exception as e:
        logger.error(f"Error profiling function: {e}")
        return OperationResult(
            success=False,
            error=str(e),
            error_code="PROFILE_ERROR",
            duration=time.time() - start_time
        )


def capture_distributed_traces(
    service_name: str,
    duration_seconds: int = 60,
    jaeger_endpoint: str = "http://localhost:14268/api/traces",
    sampling_rate: float = 1.0,
    operation_patterns: Optional[List[str]] = None
) -> OperationResult:
    """
    Capture distributed traces using Jaeger.

    Args:
        service_name: Name of the service being traced
        duration_seconds: How long to capture traces
        jaeger_endpoint: Jaeger collector endpoint
        sampling_rate: Sampling rate (0.0-1.0)
        operation_patterns: Operation patterns to filter (e.g., ['api.*', 'db.*'])

    Returns:
        OperationResult with trace statistics
    """
    start_time = time.time()

    try:
        logger.info(f"Capturing traces for service: {service_name}")

        tracer = JaegerTracer(
            service_name=service_name,
            jaeger_endpoint=jaeger_endpoint,
            sampling_rate=sampling_rate
        )

        trace_stats = tracer.capture_traces(
            duration_seconds=duration_seconds,
            operation_patterns=operation_patterns
        )

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=trace_stats,
            duration=duration,
            metadata={
                'service_name': service_name,
                'sampling_rate': sampling_rate,
                'duration_seconds': duration_seconds
            }
        )

    except Exception as e:
        logger.error(f"Error capturing traces: {e}")
        return OperationResult(
            success=False,
            error=str(e),
            error_code="TRACE_CAPTURE_ERROR",
            duration=time.time() - start_time
        )


def monitor_resource_usage(
    operation_function: Callable,
    operation_args: tuple = (),
    operation_kwargs: Optional[Dict] = None,
    sampling_interval_ms: int = 100,
    track_cpu: bool = True,
    track_memory: bool = True,
    track_disk: bool = True,
    track_network: bool = True
) -> OperationResult:
    """
    Monitor system resource usage during an operation.

    Args:
        operation_function: Function to monitor
        operation_args: Arguments for the function
        operation_kwargs: Keyword arguments for the function
        sampling_interval_ms: Sampling interval in milliseconds
        track_cpu: Track CPU usage
        track_memory: Track memory usage
        track_disk: Track disk I/O
        track_network: Track network I/O

    Returns:
        OperationResult with resource usage data
    """
    start_time = time.time()
    operation_kwargs = operation_kwargs or {}

    try:
        logger.info(f"Monitoring resource usage for: {operation_function.__name__}")

        monitor = ResourceMonitor()
        operation_result, usage_result = monitor.monitor_operation(
            operation_function,
            *operation_args,
            sampling_interval_ms=sampling_interval_ms,
            **operation_kwargs
        )

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data={
                'operation_result': operation_result,
                'resource_usage': usage_result.to_dict()
            },
            duration=duration,
            metadata={
                'sampling_interval_ms': sampling_interval_ms,
                'tracked_resources': {
                    'cpu': track_cpu,
                    'memory': track_memory,
                    'disk': track_disk,
                    'network': track_network
                }
            }
        )

    except Exception as e:
        logger.error(f"Error monitoring resources: {e}")
        return OperationResult(
            success=False,
            error=str(e),
            error_code="RESOURCE_MONITOR_ERROR",
            duration=time.time() - start_time
        )


def run_benchmark_suite(
    benchmarks: List[Dict[str, Any]],
    release_version: str,
    iterations: int = 10,
    warmup_iterations: int = 3,
    track_resources: bool = True,
    output_dir: str = "./benchmarks",
    regression_threshold: float = 0.05
) -> OperationResult:
    """
    Run a suite of benchmarks for a release.

    Args:
        benchmarks: List of benchmark definitions
                   [{'name': str, 'func': callable, 'args': tuple, 'kwargs': dict}]
        release_version: Version identifier for this benchmark run
        iterations: Number of iterations per benchmark
        warmup_iterations: Number of warmup runs
        track_resources: Track CPU/memory during benchmarks
        output_dir: Directory to store benchmark results
        regression_threshold: Threshold for detecting regressions (e.g., 0.05 = 5%)

    Returns:
        OperationResult with benchmark results
    """
    start_time = time.time()

    try:
        logger.info(f"Running benchmark suite for release: {release_version}")

        runner = BenchmarkRunner(
            output_dir=output_dir,
            regression_threshold=regression_threshold
        )

        results = runner.run_benchmark_suite(
            benchmarks=benchmarks,
            release_version=release_version,
            iterations=iterations,
            warmup_iterations=warmup_iterations,
            track_resources=track_resources
        )

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data={
                'release_version': release_version,
                'total_benchmarks': len(results),
                'results': [r.to_dict() for r in results],
                'summary': {
                    'avg_time_ms': sum(r.avg_time_ms for r in results) / len(results) if results else 0,
                    'total_duration_ms': sum(r.duration_ms for r in results),
                    'avg_throughput': sum(r.throughput_ops_sec for r in results) / len(results) if results else 0
                }
            },
            duration=duration,
            metadata={
                'iterations': iterations,
                'warmup_iterations': warmup_iterations,
                'track_resources': track_resources,
                'output_dir': output_dir
            }
        )

    except Exception as e:
        logger.error(f"Error running benchmark suite: {e}")
        return OperationResult(
            success=False,
            error=str(e),
            error_code="BENCHMARK_ERROR",
            duration=time.time() - start_time
        )


def compare_with_baseline(
    current_results_file: str,
    baseline_version: str,
    output_dir: str = "./benchmarks",
    regression_threshold: float = 0.05
) -> OperationResult:
    """
    Compare current benchmark results with a baseline.

    Args:
        current_results_file: Path to current benchmark results JSON file
        baseline_version: Version identifier for baseline
        output_dir: Directory containing benchmark results
        regression_threshold: Threshold for detecting regressions (e.g., 0.05 = 5%)

    Returns:
        OperationResult with comparison analysis
    """
    start_time = time.time()

    try:
        logger.info(f"Comparing with baseline version: {baseline_version}")

        # Load current results
        with open(current_results_file, 'r') as f:
            current_data = json.load(f)

        current_results = [
            BenchmarkResult(**r) for r in current_data['results']
        ]

        runner = BenchmarkRunner(
            output_dir=output_dir,
            regression_threshold=regression_threshold
        )

        comparison = runner.compare_with_baseline(
            current_results=current_results,
            baseline_version=baseline_version
        )

        duration = time.time() - start_time

        # Determine overall success (no regressions)
        has_regressions = (
            comparison.regression_analysis.has_regression
            if comparison.regression_analysis else False
        )

        return OperationResult(
            success=not has_regressions,
            data=comparison.to_dict(),
            duration=duration,
            metadata={
                'baseline_version': baseline_version,
                'regression_threshold': regression_threshold,
                'has_regressions': has_regressions,
                'regression_count': (
                    len(comparison.regression_analysis.regressed_benchmarks)
                    if comparison.regression_analysis else 0
                )
            }
        )

    except Exception as e:
        logger.error(f"Error comparing with baseline: {e}")
        return OperationResult(
            success=False,
            error=str(e),
            error_code="COMPARISON_ERROR",
            duration=time.time() - start_time
        )


def generate_performance_report(
    comparison_data: Dict[str, Any],
    output_file: str,
    include_charts: bool = False
) -> OperationResult:
    """
    Generate a comprehensive performance report.

    Args:
        comparison_data: Comparison data from compare_with_baseline
        output_file: Path to output report file
        include_charts: Include performance charts (requires matplotlib)

    Returns:
        OperationResult with report file path
    """
    start_time = time.time()

    try:
        logger.info(f"Generating performance report: {output_file}")

        report = {
            'generated_at': time.strftime('%Y-%m-%d %H:%M:%S'),
            'comparison': comparison_data,
            'summary': {
                'current_version': comparison_data['current_version'],
                'baseline_version': comparison_data['baseline_version'],
                'total_benchmarks': comparison_data['total_benchmarks']
            }
        }

        # Add regression analysis
        if comparison_data.get('regression_analysis'):
            analysis = comparison_data['regression_analysis']
            report['summary']['has_regressions'] = analysis['has_regression']
            report['summary']['regression_count'] = len(analysis['regressed_benchmarks'])
            report['summary']['improvement_count'] = len(analysis['improved_benchmarks'])
            report['summary']['overall_change_percent'] = analysis['overall_change_percent']

        # Save report
        output_path = Path(output_file)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        with open(output_path, 'w') as f:
            json.dump(report, f, indent=2)

        duration = time.time() - start_time

        logger.info(f"Performance report saved to: {output_path}")

        return OperationResult(
            success=True,
            data={
                'report_file': str(output_path),
                'summary': report['summary']
            },
            duration=duration,
            metadata={
                'include_charts': include_charts,
                'format': 'json'
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


# Export public API
__all__ = [
    'OperationResult',
    'profile_code_execution',
    'capture_distributed_traces',
    'monitor_resource_usage',
    'run_benchmark_suite',
    'compare_with_baseline',
    'generate_performance_report'
]
