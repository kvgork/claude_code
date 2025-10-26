"""
Performance Profiler Skill Demo

Demonstrates all operations of the performance_profiler skill.
"""

import time
import json
from pathlib import Path

# Import operations
from skills.performance_profiler import (
    profile_code_execution,
    monitor_resource_usage,
    run_benchmark_suite,
    compare_with_baseline,
    generate_performance_report
)


# Sample functions for testing
def fibonacci_recursive(n):
    """Recursive fibonacci - intentionally inefficient for demo."""
    if n <= 1:
        return n
    return fibonacci_recursive(n - 1) + fibonacci_recursive(n - 2)


def fibonacci_iterative(n):
    """Iterative fibonacci - more efficient."""
    if n <= 1:
        return n
    a, b = 0, 1
    for _ in range(n - 1):
        a, b = b, a + b
    return b


def list_comprehension_test():
    """Test list comprehension performance."""
    return [i**2 for i in range(10000)]


def generator_expression_test():
    """Test generator expression performance."""
    return sum(i**2 for i in range(10000))


def memory_intensive_operation():
    """Operation that uses significant memory."""
    # Create large list
    data = [i**2 for i in range(1000000)]
    # Process it
    result = sum(data)
    # Create another large structure
    matrix = [[i * j for j in range(100)] for i in range(1000)]
    return result + sum(sum(row) for row in matrix)


def print_separator(title):
    """Print a formatted separator."""
    print(f"\n{'=' * 80}")
    print(f"  {title}")
    print('=' * 80)


def demo_profile_code_execution():
    """Demonstrate code profiling."""
    print_separator("Demo 1: Profile Code Execution")

    print("\n1. Profiling recursive fibonacci (n=20)...")
    result = profile_code_execution(
        target_function=fibonacci_recursive,
        function_args=(20,),
        save_profile=True,
        top_n_functions=5
    )

    if result.success:
        print(f"✓ Profiling successful")
        print(f"  - Function: {result.data['function_name']}")
        print(f"  - Total time: {result.data['total_time']:.4f}s")
        print(f"  - Total calls: {result.data['total_calls']:,}")
        print(f"  - Profile saved to: {result.data['profile_file']}")
        print(f"\n  Top functions by cumulative time:")
        for i, func in enumerate(result.data['top_functions'][:3], 1):
            print(f"    {i}. {func['function']}: {func['cumtime']:.4f}s")
    else:
        print(f"✗ Profiling failed: {result.error}")

    print("\n2. Profiling iterative fibonacci (n=20)...")
    result = profile_code_execution(
        target_function=fibonacci_iterative,
        function_args=(20,),
        save_profile=True,
        top_n_functions=5
    )

    if result.success:
        print(f"✓ Profiling successful")
        print(f"  - Total time: {result.data['total_time']:.4f}s")
        print(f"  - Total calls: {result.data['total_calls']:,}")
        print(f"\n  Performance comparison:")
        print(f"    Iterative is much faster than recursive for fibonacci!")


def demo_monitor_resource_usage():
    """Demonstrate resource monitoring."""
    print_separator("Demo 2: Monitor Resource Usage")

    print("\n1. Monitoring memory-intensive operation...")
    result = monitor_resource_usage(
        operation_function=memory_intensive_operation,
        sampling_interval_ms=50,
        track_cpu=True,
        track_memory=True,
        track_disk=False,
        track_network=False
    )

    if result.success:
        usage = result.data['resource_usage']
        print(f"✓ Monitoring successful")
        print(f"  - Duration: {usage['duration_seconds']:.2f}s")
        print(f"  - Samples collected: {usage['samples_collected']}")

        if usage.get('cpu_usage'):
            cpu = usage['cpu_usage']
            print(f"\n  CPU Usage:")
            print(f"    - Average: {cpu['avg_percent']:.1f}%")
            print(f"    - Peak: {cpu['peak_percent']:.1f}%")

        if usage.get('memory_usage'):
            mem = usage['memory_usage']
            print(f"\n  Memory Usage:")
            print(f"    - Average: {mem['avg_mb']:.1f} MB")
            print(f"    - Peak: {mem['peak_mb']:.1f} MB")
    else:
        print(f"✗ Monitoring failed: {result.error}")


def demo_run_benchmark_suite():
    """Demonstrate benchmark suite execution."""
    print_separator("Demo 3: Run Benchmark Suite")

    # Define benchmarks
    benchmarks = [
        {
            'name': 'fibonacci_recursive_15',
            'func': fibonacci_recursive,
            'args': (15,),
            'kwargs': {},
            'metadata': {'algorithm': 'recursive', 'n': 15}
        },
        {
            'name': 'fibonacci_iterative_15',
            'func': fibonacci_iterative,
            'args': (15,),
            'kwargs': {},
            'metadata': {'algorithm': 'iterative', 'n': 15}
        },
        {
            'name': 'list_comprehension',
            'func': list_comprehension_test,
            'args': (),
            'kwargs': {},
            'metadata': {'type': 'list_comprehension'}
        },
        {
            'name': 'generator_expression',
            'func': generator_expression_test,
            'args': (),
            'kwargs': {},
            'metadata': {'type': 'generator'}
        }
    ]

    print(f"\n1. Running benchmark suite for release v1.0.0...")
    print(f"   - {len(benchmarks)} benchmarks")
    print(f"   - 20 iterations each (+ 5 warmup)")

    result = run_benchmark_suite(
        benchmarks=benchmarks,
        release_version="v1.0.0",
        iterations=20,
        warmup_iterations=5,
        track_resources=True,
        output_dir="./benchmarks"
    )

    if result.success:
        print(f"\n✓ Benchmark suite completed")
        print(f"  - Total benchmarks: {result.data['total_benchmarks']}")
        print(f"  - Average time: {result.data['summary']['avg_time_ms']:.3f}ms")
        print(f"  - Total duration: {result.data['summary']['total_duration_ms']:.1f}ms")

        print(f"\n  Individual benchmark results:")
        for bench_result in result.data['results']:
            print(f"    • {bench_result['benchmark_name']}:")
            print(f"      - Avg: {bench_result['avg_time_ms']:.3f}ms")
            print(f"      - Throughput: {bench_result['throughput_ops_sec']:.1f} ops/sec")

        # Save the results file path for comparison demo
        return result.data['results']
    else:
        print(f"✗ Benchmark suite failed: {result.error}")
        return None


def demo_compare_with_baseline(first_run_results):
    """Demonstrate baseline comparison."""
    print_separator("Demo 4: Compare with Baseline")

    if not first_run_results:
        print("⚠️ Skipping comparison demo (no baseline results)")
        return None

    # Simulate a new release with slightly different performance
    print("\n1. Simulating v1.1.0 release with performance changes...")

    # Create modified benchmarks (with intentional regression)
    benchmarks_v2 = [
        {
            'name': 'fibonacci_recursive_15',
            'func': lambda: (fibonacci_recursive(15), time.sleep(0.001)),  # Artificial slowdown
            'args': (),
            'kwargs': {},
            'metadata': {'algorithm': 'recursive', 'n': 15}
        },
        {
            'name': 'fibonacci_iterative_15',
            'func': fibonacci_iterative,
            'args': (15,),
            'kwargs': {},
            'metadata': {'algorithm': 'iterative', 'n': 15}
        },
        {
            'name': 'list_comprehension',
            'func': list_comprehension_test,
            'args': (),
            'kwargs': {},
            'metadata': {'type': 'list_comprehension'}
        },
        {
            'name': 'generator_expression',
            'func': generator_expression_test,
            'args': (),
            'kwargs': {},
            'metadata': {'type': 'generator'}
        }
    ]

    result = run_benchmark_suite(
        benchmarks=benchmarks_v2,
        release_version="v1.1.0",
        iterations=20,
        warmup_iterations=5,
        track_resources=True,
        output_dir="./benchmarks"
    )

    if not result.success:
        print(f"✗ v1.1.0 benchmark failed: {result.error}")
        return None

    print(f"✓ v1.1.0 benchmarks completed")

    # Find the results file for v1.1.0
    benchmark_dir = Path("./benchmarks")
    v110_files = sorted(benchmark_dir.glob("benchmark_v1.1.0_*.json"), reverse=True)

    if not v110_files:
        print("✗ Could not find v1.1.0 results file")
        return None

    current_file = str(v110_files[0])
    print(f"\n2. Comparing v1.1.0 with baseline v1.0.0...")

    comparison = compare_with_baseline(
        current_results_file=current_file,
        baseline_version="v1.0.0",
        output_dir="./benchmarks",
        regression_threshold=0.05  # 5% threshold
    )

    if comparison.success:
        print(f"\n✓ Comparison completed (no regressions)")
    else:
        print(f"\n⚠️ Regressions detected!")

    data = comparison.data
    analysis = data.get('regression_analysis')

    if analysis:
        print(f"\n  Regression Analysis:")
        print(f"    - Has regressions: {analysis['has_regression']}")
        print(f"    - Regressed benchmarks: {len(analysis['regressed_benchmarks'])}")
        print(f"    - Improved benchmarks: {len(analysis['improved_benchmarks'])}")
        print(f"    - Overall change: {analysis['overall_change_percent']:+.2f}%")

        if analysis['regressed_benchmarks']:
            print(f"\n  Regressed benchmarks:")
            for reg in analysis['regressed_benchmarks']:
                print(f"    ✗ {reg['benchmark']}: +{reg['change_percent']:.2f}% ({reg['change_ms']:+.2f}ms)")

        if analysis['improved_benchmarks']:
            print(f"\n  Improved benchmarks:")
            for imp in analysis['improved_benchmarks']:
                print(f"    ✓ {imp['benchmark']}: {imp['change_percent']:.2f}% ({imp['change_ms']:.2f}ms)")

    return comparison.data


def demo_generate_report(comparison_data):
    """Demonstrate report generation."""
    print_separator("Demo 5: Generate Performance Report")

    if not comparison_data:
        print("⚠️ Skipping report generation (no comparison data)")
        return

    print("\n1. Generating performance report...")

    result = generate_performance_report(
        comparison_data=comparison_data,
        output_file="./reports/performance_report_v1.1.0.json",
        include_charts=False
    )

    if result.success:
        print(f"✓ Report generated successfully")
        print(f"  - Report file: {result.data['report_file']}")
        print(f"\n  Summary:")
        summary = result.data['summary']
        print(f"    - Current version: {summary['current_version']}")
        print(f"    - Baseline version: {summary['baseline_version']}")
        print(f"    - Total benchmarks: {summary['total_benchmarks']}")
        if 'has_regressions' in summary:
            print(f"    - Has regressions: {summary['has_regressions']}")
            print(f"    - Regression count: {summary['regression_count']}")
            print(f"    - Improvement count: {summary['improvement_count']}")
            print(f"    - Overall change: {summary['overall_change_percent']:+.2f}%")
    else:
        print(f"✗ Report generation failed: {result.error}")


def main():
    """Run all demos."""
    print("\n" + "=" * 80)
    print("  PERFORMANCE PROFILER SKILL DEMO")
    print("=" * 80)

    # Demo 1: Profile code execution
    demo_profile_code_execution()

    # Demo 2: Monitor resource usage
    demo_monitor_resource_usage()

    # Demo 3: Run benchmark suite
    first_run_results = demo_run_benchmark_suite()

    # Demo 4: Compare with baseline
    comparison_data = demo_compare_with_baseline(first_run_results)

    # Demo 5: Generate report
    demo_generate_report(comparison_data)

    # Summary
    print_separator("Demo Complete")
    print("\n✓ All demos completed successfully!")
    print("\nGenerated outputs:")
    print("  - Profiles: ./profiles/")
    print("  - Benchmarks: ./benchmarks/")
    print("  - Reports: ./reports/")
    print("\nYou can explore the generated files to see detailed results.")
    print()


if __name__ == "__main__":
    main()
