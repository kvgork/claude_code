"""
Release Orchestrator Integration Demo

Demonstrates complete release quality assessment workflow using all skills:
- performance-profiler
- environment-profiler
- release-orchestrator
"""

import time
from pathlib import Path

# Import all skill operations
from skills.release_orchestrator import (
    assess_release_quality,
    generate_quality_report
)


def print_separator(title):
    """Print a formatted separator."""
    print(f"\n{'=' * 80}")
    print(f"  {title}")
    print('=' * 80)


# Sample benchmark functions for demo
def benchmark_list_ops():
    """Benchmark list operations."""
    return [i**2 for i in range(10000)]


def benchmark_dict_ops():
    """Benchmark dict operations."""
    return {i: i**2 for i in range(10000)}


def benchmark_set_ops():
    """Benchmark set operations."""
    return {i**2 for i in range(10000)}


def benchmark_string_concat():
    """Benchmark string concatenation."""
    result = ""
    for i in range(1000):
        result += str(i)
    return result


def benchmark_string_join():
    """Benchmark string join."""
    return "".join(str(i) for i in range(1000))


def demo_release_v1_0_0():
    """Assess v1.0.0 (baseline release)."""
    print_separator("Release Assessment: v1.0.0 (Baseline)")

    # Define benchmark suite
    benchmarks = [
        {
            'name': 'list_comprehension',
            'func': benchmark_list_ops,
            'args': (),
            'kwargs': {}
        },
        {
            'name': 'dict_comprehension',
            'func': benchmark_dict_ops,
            'args': (),
            'kwargs': {}
        },
        {
            'name': 'set_comprehension',
            'func': benchmark_set_ops,
            'args': (),
            'kwargs': {}
        },
        {
            'name': 'string_concat',
            'func': benchmark_string_concat,
            'args': (),
            'kwargs': {}
        },
        {
            'name': 'string_join',
            'func': benchmark_string_join,
            'args': (),
            'kwargs': {}
        }
    ]

    print("\n1. Running comprehensive release assessment...")
    print(f"   - Benchmarks: {len(benchmarks)}")
    print(f"   - Performance profiling: Enabled")
    print(f"   - Environment profiling: Enabled")
    print(f"   - Quality gate threshold: 70.0")

    result = assess_release_quality(
        release_version="v1.0.0",
        project_path=".",
        baseline_version=None,  # No baseline for first release
        benchmark_suite=benchmarks,
        quality_gate_threshold=70.0,
        enable_performance=True,
        enable_environment=True
    )

    if result.success:
        print(f"\n✓ Assessment completed ({result.duration:.1f}s)")
        print(f"\n  Quality Results:")
        print(f"    - Overall Score: {result.metadata['overall_score']:.1f}/100")
        print(f"    - Grade: {result.metadata['grade']}")
        print(f"    - Quality Gate: ✅ {'PASSED' if result.metadata['passed_quality_gate'] else 'FAILED'}")

        if result.data['warnings']:
            print(f"\n  Warnings ({result.metadata['warning_count']}):")
            for warning in result.data['warnings']:
                print(f"    ⚠️  {warning}")

        if result.data['recommendations']:
            print(f"\n  Recommendations ({result.metadata['recommendation_count']}):")
            for rec in result.data['recommendations'][:3]:  # Show first 3
                print(f"    💡 {rec}")

        # Show skill execution results
        print(f"\n  Skill Execution:")
        for skill_name, skill_result in result.data['skill_results'].items():
            status = "✓" if skill_result['success'] else "✗"
            print(f"    {status} {skill_name}: {skill_result['duration']:.2f}s")

        return result.data

    else:
        print(f"\n✗ Assessment failed: {result.error}")
        return None


def demo_release_v1_1_0(baseline_data):
    """Assess v1.1.0 with performance regression."""
    print_separator("Release Assessment: v1.1.0 (With Regression)")

    # Intentionally slower benchmark to simulate regression
    def slow_string_concat():
        result = ""
        for i in range(2000):  # Double the work
            result += str(i)
        return result

    # Define benchmark suite with regression
    benchmarks = [
        {
            'name': 'list_comprehension',
            'func': benchmark_list_ops,
            'args': (),
            'kwargs': {}
        },
        {
            'name': 'dict_comprehension',
            'func': benchmark_dict_ops,
            'args': (),
            'kwargs': {}
        },
        {
            'name': 'set_comprehension',
            'func': benchmark_set_ops,
            'args': (),
            'kwargs': {}
        },
        {
            'name': 'string_concat',
            'func': slow_string_concat,  # Regression here
            'args': (),
            'kwargs': {}
        },
        {
            'name': 'string_join',
            'func': benchmark_string_join,
            'args': (),
            'kwargs': {}
        }
    ]

    print("\n1. Running assessment with baseline comparison...")
    print(f"   - Baseline: v1.0.0")
    print(f"   - Benchmarks: {len(benchmarks)}")

    result = assess_release_quality(
        release_version="v1.1.0",
        project_path=".",
        baseline_version="v1.0.0",
        benchmark_suite=benchmarks,
        quality_gate_threshold=70.0,
        enable_performance=True,
        enable_environment=True
    )

    if result.success:
        print(f"\n✓ Assessment completed ({result.duration:.1f}s)")
    else:
        print(f"\n⚠️  Assessment completed with issues ({result.duration:.1f}s)")

    print(f"\n  Quality Results:")
    print(f"    - Overall Score: {result.metadata['overall_score']:.1f}/100")
    print(f"    - Grade: {result.metadata['grade']}")
    gate_status = "✅ PASSED" if result.metadata['passed_quality_gate'] else "❌ FAILED"
    print(f"    - Quality Gate: {gate_status}")

    if result.data['warnings']:
        print(f"\n  ⚠️  Warnings ({result.metadata['warning_count']}):")
        for warning in result.data['warnings']:
            print(f"    - {warning}")

    if result.data['recommendations']:
        print(f"\n  💡 Recommendations ({result.metadata['recommendation_count']}):")
        for rec in result.data['recommendations']:
            print(f"    - {rec}")

    # Show quality dimensions
    if result.data['quality_dimensions']:
        print(f"\n  Quality Dimensions:")
        for dim in result.data['quality_dimensions']:
            score = dim['score']
            status = "✓" if score >= 70 else "✗"
            print(f"    {status} {dim['name']}: {score:.1f}/100 (weight: {dim['weight']*100:.0f}%)")

    return result.data


def demo_generate_reports(v1_0_0_data, v1_1_0_data):
    """Generate quality reports."""
    print_separator("Report Generation")

    if not v1_1_0_data:
        print("⚠️  Skipping report generation (no v1.1.0 data)")
        return

    # Generate Markdown report
    print("\n1. Generating Markdown report for v1.1.0...")
    md_result = generate_quality_report(
        assessment_data=v1_1_0_data,
        output_format="markdown"
    )

    if md_result.success:
        print(f"✓ Markdown report generated")
        print(f"  - File: {md_result.data['report_file']}")
        print(f"  - Size: {md_result.metadata['file_size_bytes']:,} bytes")

        # Show preview
        report_path = Path(md_result.data['report_file'])
        if report_path.exists():
            with open(report_path, 'r') as f:
                lines = f.readlines()[:20]  # First 20 lines
            print(f"\n  Preview:")
            for line in lines:
                print(f"    {line.rstrip()}")
            if len(lines) >= 20:
                print(f"    ... (truncated)")
    else:
        print(f"✗ Markdown report failed: {md_result.error}")

    # Generate JSON report
    print("\n2. Generating JSON report for v1.1.0...")
    json_result = generate_quality_report(
        assessment_data=v1_1_0_data,
        output_format="json"
    )

    if json_result.success:
        print(f"✓ JSON report generated")
        print(f"  - File: {json_result.data['report_file']}")
        print(f"  - Size: {json_result.metadata['file_size_bytes']:,} bytes")
    else:
        print(f"✗ JSON report failed: {json_result.error}")


def main():
    """Run complete integration demo."""
    print("\n" + "=" * 80)
    print("  RELEASE QUALITY ASSESSMENT - INTEGRATION DEMO")
    print("=" * 80)
    print("\nThis demo showcases the complete release quality assessment workflow:")
    print("  1. Performance profiling (benchmarks, regression detection)")
    print("  2. Environment profiling (hardware, software, packages)")
    print("  3. Quality scoring (multi-dimensional assessment)")
    print("  4. Report generation (Markdown & JSON)")

    # Demo 1: Assess v1.0.0 (baseline)
    v1_0_0_data = demo_release_v1_0_0()

    # Wait a moment for visual separation
    time.sleep(1)

    # Demo 2: Assess v1.1.0 with regression
    v1_1_0_data = demo_release_v1_1_0(v1_0_0_data)

    # Wait a moment for visual separation
    time.sleep(1)

    # Demo 3: Generate reports
    demo_generate_reports(v1_0_0_data, v1_1_0_data)

    # Summary
    print_separator("Demo Complete")
    print("\n✓ Complete release quality assessment demonstrated!")

    print("\n📊 Generated Artifacts:")
    print("  - Performance benchmarks: ./benchmarks/")
    print("  - Environment snapshots: ./environment_snapshots/")
    print("  - Quality reports: ./reports/")

    print("\n🎯 Key Features Demonstrated:")
    print("  ✓ Multi-skill orchestration")
    print("  ✓ Baseline comparison")
    print("  ✓ Performance regression detection")
    print("  ✓ Environment change tracking")
    print("  ✓ Quality gate evaluation")
    print("  ✓ Automated report generation")

    print("\n📈 Workflow Summary:")
    if v1_1_0_data:
        print(f"  v1.0.0 → v1.1.0:")

        # Performance changes
        perf_skill = v1_1_0_data['skill_results'].get('performance')
        if perf_skill and perf_skill['success']:
            perf_data = perf_skill['data']
            comparison = perf_data.get('comparison')
            if comparison and 'regression_analysis' in comparison:
                analysis = comparison['regression_analysis']
                print(f"    Performance: {analysis['overall_change_percent']:+.1f}% " +
                      f"({len(analysis.get('regressed_benchmarks', []))} regressions)")

        # Environment changes
        env_skill = v1_1_0_data['skill_results'].get('environment')
        if env_skill and env_skill['success']:
            env_data = env_skill['data']
            comparison = env_data.get('comparison')
            if comparison:
                pkg_diff = comparison.get('package_diff', {})
                changes = pkg_diff.get('total_changes', 0)
                print(f"    Environment: {changes} package change(s)")

        # Quality score
        print(f"    Quality Score: {v1_1_0_data['overall_quality_score']:.1f} " +
              f"({v1_1_0_data['grade']})")

    print()


if __name__ == "__main__":
    main()
