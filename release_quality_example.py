#!/usr/bin/env python3
"""
Release Quality Assessment Example

This script demonstrates a practical example of using the Release Quality Assessment
system to evaluate code releases and compare them.

Usage:
    python release_quality_example.py

Requirements:
    - Python 3.8+
    - All dependencies installed (see requirements.txt)
    - Optional: Docker infrastructure running for full features
"""

import sys
import time
from pathlib import Path

# Add project root to Python path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

from skills.release_orchestrator.operations import (
    assess_release,
    compare_releases,
    generate_report
)
from skills.performance_profiler.operations import (
    run_benchmarks,
    profile_function
)
from skills.environment_profiler.operations import (
    detect_environment,
    save_snapshot
)


def example_function_slow():
    """Example function that performs slowly (for benchmarking)."""
    total = 0
    for i in range(10000):
        total += i ** 2
    return total


def example_function_fast():
    """Example function that performs faster (for comparison)."""
    return sum(i ** 2 for i in range(10000))


def demo_performance_profiling():
    """Demonstrate performance profiling capabilities."""
    print("\n" + "="*70)
    print("1. PERFORMANCE PROFILING DEMO")
    print("="*70)

    # Profile a function
    print("\n📊 Profiling example function...")
    profile_result = profile_function(
        func=example_function_slow,
        name="example_computation",
        description="Example mathematical computation"
    )

    print(f"✓ Function executed in {profile_result['execution_time_ms']:.2f}ms")
    print(f"✓ Called {profile_result['call_count']} functions")
    print(f"✓ Top functions by time:")
    for func_name, stats in list(profile_result['top_functions'].items())[:3]:
        print(f"  - {func_name}: {stats['cumtime']*1000:.2f}ms")

    # Run benchmarks
    print("\n⚡ Running performance benchmarks...")
    benchmark_suite = [
        {
            'name': 'slow_computation',
            'function': example_function_slow,
            'description': 'Slower computation method'
        },
        {
            'name': 'fast_computation',
            'function': example_function_fast,
            'description': 'Optimized computation method'
        }
    ]

    benchmark_results = run_benchmarks(
        benchmarks=benchmark_suite,
        release_version="v1.0.0",
        iterations=5
    )

    print(f"✓ Completed {len(benchmark_results)} benchmarks")
    for result in benchmark_results:
        print(f"  - {result['benchmark_name']}: {result['avg_time_ms']:.3f}ms avg")


def demo_environment_detection():
    """Demonstrate environment detection capabilities."""
    print("\n" + "="*70)
    print("2. ENVIRONMENT DETECTION DEMO")
    print("="*70)

    print("\n🔍 Detecting system environment...")
    env_profile = detect_environment()

    # Display hardware info
    hw = env_profile['hardware']
    print(f"\n💻 Hardware Profile:")
    print(f"  - CPU: {hw['cpu']['brand']} ({hw['cpu']['cores']} cores)")
    print(f"  - Memory: {hw['memory']['total_gb']:.1f} GB")
    print(f"  - Disks: {len(hw['disks'])} device(s)")
    if hw['gpus']:
        print(f"  - GPUs: {len(hw['gpus'])} detected")

    # Display software info
    sw = env_profile['software']
    print(f"\n📦 Software Profile:")
    print(f"  - OS: {sw['os']['system']} {sw['os']['release']}")
    print(f"  - Python: {sw['python']['version']}")
    print(f"  - Packages: {len(sw['packages'])} installed")

    # Save snapshot
    print(f"\n💾 Saving environment snapshot...")
    snapshot_file = save_snapshot(
        env_profile=env_profile,
        release_version="v1.0.0"
    )
    print(f"✓ Snapshot saved: {snapshot_file}")


def demo_release_assessment():
    """Demonstrate full release quality assessment."""
    print("\n" + "="*70)
    print("3. RELEASE QUALITY ASSESSMENT DEMO")
    print("="*70)

    print("\n🔬 Assessing release quality...")

    # Define benchmark suite for this release
    benchmark_suite = [
        {
            'name': 'computation_benchmark',
            'function': example_function_fast,
            'description': 'Core computation performance'
        }
    ]

    # Assess first release
    print("\n📋 Assessing v1.0.0...")
    assessment_v1 = assess_release(
        release_version="v1.0.0",
        project_path=str(project_root),
        benchmark_suite=benchmark_suite
    )

    print(f"✓ Overall Score: {assessment_v1['overall_quality_score']:.1f}/100")
    print(f"✓ Grade: {assessment_v1['grade']}")
    print(f"✓ Quality Gate: {'PASSED' if assessment_v1['passed_quality_gate'] else 'FAILED'}")

    print("\n📊 Dimension Scores:")
    for dim in assessment_v1['quality_dimensions']:
        print(f"  - {dim['name'].replace('_', ' ').title()}: {dim['score']:.1f}/100")

    # Wait a moment to simulate time passing
    print("\n⏳ Simulating development time...")
    time.sleep(2)

    # Assess second release with baseline comparison
    print("\n📋 Assessing v1.1.0 (comparing with v1.0.0)...")
    assessment_v2 = assess_release(
        release_version="v1.1.0",
        project_path=str(project_root),
        baseline_version="v1.0.0",
        benchmark_suite=benchmark_suite
    )

    print(f"✓ Overall Score: {assessment_v2['overall_quality_score']:.1f}/100")
    print(f"✓ Grade: {assessment_v2['grade']}")
    print(f"✓ Quality Gate: {'PASSED' if assessment_v2['passed_quality_gate'] else 'FAILED'}")

    # Compare releases
    print("\n🔄 Comparing releases...")
    comparison = compare_releases(
        release_v1="v1.0.0",
        release_v2="v1.1.0"
    )

    score_change = comparison['score_change']
    print(f"✓ Score Change: {score_change:+.1f} points")

    if comparison['performance_changes']:
        print("\n⚡ Performance Changes:")
        for change in comparison['performance_changes'][:3]:
            direction = "↑" if change['change_percent'] > 0 else "↓"
            print(f"  {direction} {change['benchmark_name']}: {abs(change['change_percent']):.1f}% change")

    if comparison['environment_changes']:
        print(f"\n🔧 Environment Changes: {len(comparison['environment_changes'])} detected")

    # Generate report
    print("\n📝 Generating quality report...")
    report_path = generate_report(
        assessment=assessment_v2,
        format='markdown'
    )
    print(f"✓ Report generated: {report_path}")


def main():
    """Run all demonstration examples."""
    print("="*70)
    print("RELEASE QUALITY ASSESSMENT SYSTEM - PRACTICAL EXAMPLE")
    print("="*70)
    print("\nThis example demonstrates the key capabilities of the Release Quality")
    print("Assessment System through practical demonstrations.")

    try:
        # Run demonstrations
        demo_performance_profiling()
        demo_environment_detection()
        demo_release_assessment()

        # Summary
        print("\n" + "="*70)
        print("✅ EXAMPLE COMPLETED SUCCESSFULLY")
        print("="*70)
        print("\nNext Steps:")
        print("  1. Start infrastructure: cd infrastructure && docker-compose up -d")
        print("  2. View traces in Jaeger: http://localhost:16686")
        print("  3. View metrics in Grafana: http://localhost:3000")
        print("  4. Read documentation: RELEASE_QUALITY_SYSTEM_FINAL_SUMMARY.md")
        print("\n" + "="*70)

    except Exception as e:
        print(f"\n❌ Error during demonstration: {e}")
        print(f"\nStack trace:")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
