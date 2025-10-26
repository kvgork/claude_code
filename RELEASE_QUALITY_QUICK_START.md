# Release Quality Assessment - Quick Start Guide

## Overview

This system provides **automated release quality assessment** through 3 coordinated skills:
- **performance-profiler**: Benchmark performance and detect regressions
- **environment-profiler**: Capture environment for reproducibility
- **release-orchestrator**: Coordinate skills and generate quality reports

## Installation

```bash
# Navigate to project
cd /path/to/claude_code

# Install optional dependencies for full functionality
pip install psutil

# Run demos to verify installation
PYTHONPATH=$(pwd) python skills/performance_profiler/demo.py
PYTHONPATH=$(pwd) python skills/environment_profiler/demo.py
PYTHONPATH=$(pwd) python skills/release_orchestrator/demo.py
```

## Quick Start: Assess Your First Release

### Step 1: Define Benchmarks

```python
# your_benchmarks.py

def benchmark_api_response():
    """Benchmark API response time."""
    # Your API call here
    return make_api_call()

def benchmark_database_query():
    """Benchmark database query."""
    # Your DB query here
    return db.query("SELECT * FROM users LIMIT 1000")

# Define benchmark suite
BENCHMARKS = [
    {
        'name': 'api_response',
        'func': benchmark_api_response,
        'args': (),
        'kwargs': {}
    },
    {
        'name': 'database_query',
        'func': benchmark_database_query,
        'args': (),
        'kwargs': {}
    }
]
```

### Step 2: Run Release Assessment

```python
# assess_release.py

from skills.release_orchestrator import assess_release_quality, generate_quality_report
from your_benchmarks import BENCHMARKS

# Assess release v1.0.0
result = assess_release_quality(
    release_version="v1.0.0",
    project_path=".",
    benchmark_suite=BENCHMARKS,
    quality_gate_threshold=70.0,  # Minimum score to pass
    enable_performance=True,
    enable_environment=True
)

# Check results
if result.success:
    print(f"✅ Quality Gate: PASSED")
    print(f"Score: {result.metadata['overall_score']:.1f}/100")
    print(f"Grade: {result.metadata['grade']}")

    # Generate report
    generate_quality_report(
        assessment_data=result.data,
        output_format="markdown",
        output_file=f"./reports/release_v1.0.0.md"
    )
else:
    print(f"❌ Quality Gate: FAILED")
    print(f"Score: {result.metadata['overall_score']:.1f}/100")
    for warning in result.data['warnings']:
        print(f"  ⚠️  {warning}")
```

### Step 3: Run with Baseline Comparison

```python
# Assess v1.1.0 with baseline comparison
result = assess_release_quality(
    release_version="v1.1.0",
    project_path=".",
    baseline_version="v1.0.0",  # Compare with v1.0.0
    benchmark_suite=BENCHMARKS,
    quality_gate_threshold=70.0
)

# Check for regressions
if result.data['warnings']:
    print("⚠️  Performance regressions detected:")
    for warning in result.data['warnings']:
        print(f"  - {warning}")
```

## Common Use Cases

### 1. CI/CD Integration

```yaml
# .github/workflows/quality-gate.yml

name: Release Quality Gate

on:
  pull_request:
  push:
    branches: [main]

jobs:
  quality-assessment:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3

      - name: Install dependencies
        run: pip install psutil

      - name: Run quality assessment
        run: |
          PYTHONPATH=$(pwd) python assess_release.py

      - name: Upload reports
        uses: actions/upload-artifact@v3
        with:
          name: quality-reports
          path: reports/

      - name: Check quality gate
        run: |
          # Fail build if quality gate failed
          python -c "import json; exit(0 if json.load(open('reports/result.json'))['passed_quality_gate'] else 1)"
```

### 2. Pre-Release Validation

```bash
#!/bin/bash
# pre-release-check.sh

RELEASE_VERSION=$1

echo "Assessing release quality for $RELEASE_VERSION..."

PYTHONPATH=$(pwd) python -c "
from skills.release_orchestrator import assess_release_quality
from your_benchmarks import BENCHMARKS

result = assess_release_quality(
    release_version='$RELEASE_VERSION',
    project_path='.',
    baseline_version='main',
    benchmark_suite=BENCHMARKS,
    quality_gate_threshold=80.0  # Stricter for releases
)

if not result.success:
    print(f'❌ Release {RELEASE_VERSION} failed quality gate')
    exit(1)

print(f'✅ Release {RELEASE_VERSION} passed quality gate')
print(f'Score: {result.metadata[\"overall_score\"]:.1f}/100')
"

if [ $? -eq 0 ]; then
    echo "✅ Ready to release!"
else
    echo "❌ Quality gate failed - do not release"
    exit 1
fi
```

### 3. Performance Regression Detection

```python
from skills.performance_profiler import run_benchmark_suite, compare_with_baseline

# Run benchmarks
run_benchmark_suite(
    benchmarks=BENCHMARKS,
    release_version="v1.1.0",
    iterations=50,  # More iterations for accuracy
    warmup_iterations=10
)

# Compare with baseline
from pathlib import Path

current_file = sorted(Path("./benchmarks").glob("benchmark_v1.1.0_*.json"))[-1]

comparison = compare_with_baseline(
    current_results_file=str(current_file),
    baseline_version="v1.0.0",
    regression_threshold=0.05  # 5% threshold
)

# Analyze regressions
if not comparison.success:
    analysis = comparison.data['regression_analysis']
    print(f"⚠️  {len(analysis['regressed_benchmarks'])} regressions detected:")

    for reg in analysis['regressed_benchmarks']:
        print(f"  {reg['benchmark']}: +{reg['change_percent']:.2f}% ({reg['change_ms']:+.2f}ms)")
```

### 4. Environment Reproducibility

```python
from skills.environment_profiler import (
    create_environment_snapshot,
    compare_environments,
    export_package_requirements
)

# Create snapshot for production deployment
snapshot = create_environment_snapshot(
    snapshot_name="production-v1.0.0",
    include_packages=True,
    filter_sensitive=True
)

print(f"Snapshot saved: {snapshot.data['snapshot_file']}")

# Export requirements for reproducibility
export_package_requirements(
    output_file="./requirements-production-v1.0.0.txt",
    include_versions=True
)

# Later: verify production environment matches snapshot
prod_snapshot = create_environment_snapshot(snapshot_name="production-current")

comparison = compare_environments(
    current_snapshot_file=prod_snapshot.data['snapshot_file'],
    baseline_snapshot_file=snapshot.data['snapshot_file']
)

if comparison.data['is_different']:
    print("⚠️  Production environment differs from expected!")
    pkg_diff = comparison.data['package_diff']
    print(f"  Package changes: {pkg_diff['total_changes']}")
```

## Configuration

### Quality Gate Thresholds

```python
# Strict (for production releases)
quality_gate_threshold=90.0

# Standard (for regular releases)
quality_gate_threshold=70.0

# Permissive (for development builds)
quality_gate_threshold=50.0
```

### Custom Quality Weights

```python
from skills.release_orchestrator import calculate_quality_score

# Custom weights (must sum to 1.0)
custom_weights = {
    'code_quality': 0.30,      # Emphasize code quality
    'test_quality': 0.30,      # Emphasize testing
    'dependencies': 0.15,
    'documentation': 0.05,
    'performance': 0.20
}

result = calculate_quality_score(
    performance_analysis={...},
    custom_weights=custom_weights
)
```

### Benchmark Configuration

```python
# For quick checks (CI pipelines)
run_benchmark_suite(
    benchmarks=BENCHMARKS,
    iterations=10,
    warmup_iterations=3
)

# For accurate measurements (pre-release)
run_benchmark_suite(
    benchmarks=BENCHMARKS,
    iterations=100,
    warmup_iterations=20,
    track_resources=True
)
```

## Output Files

All assessment artifacts are saved in organized directories:

```
./
├── benchmarks/                    # Performance benchmark results
│   ├── benchmark_v1.0.0_1234.json
│   └── benchmark_v1.1.0_5678.json
│
├── environment_snapshots/         # Environment snapshots
│   ├── snapshot_v1.0.0_1234.json
│   ├── snapshot_v1.1.0_5678.json
│   └── requirements-v1.0.0.txt
│
├── profiles/                      # cProfile outputs
│   ├── my_function_1234.prof
│   └── another_function_5678.prof
│
└── reports/                       # Quality reports
    ├── release_report_v1.0.0.md
    ├── release_report_v1.0.0.json
    └── performance_report_v1.1.0.json
```

## Interpreting Results

### Quality Score

- **90-100 (A/A+)**: Excellent - ready for production
- **80-89 (B)**: Good - minor improvements recommended
- **70-79 (C)**: Acceptable - address issues before release
- **60-69 (D)**: Below acceptable - significant issues
- **0-59 (F)**: Failing - do not release

### Quality Dimensions

Each dimension is scored independently:

```
Code Quality:     85.0/100 ✓  (Weight: 25%)
  Issues:
    - High average complexity: 12.3
    - Code smells detected: 8

Test Quality:     92.0/100 ✓  (Weight: 20%)
  Metrics:
    - Coverage: 87.5%
    - Success rate: 100%
    - Test count: 234

Performance:      78.0/100 ✓  (Weight: 20%)
  Issues:
    - Performance regressions: 2
    - Performance degradation: +8.3%
```

### Warnings vs Recommendations

- **Warnings** 🚨: Critical issues that should block release
- **Recommendations** 💡: Suggested improvements for future releases

## Troubleshooting

### psutil not available

```bash
# Install psutil for full functionality
pip install psutil

# System will work without it, but with limited hardware detection
```

### No baseline found

```
Warning: No baseline results found for version: v1.0.0
```

Solution: Run assessment for baseline version first, or remove `baseline_version` parameter.

### Quality gate always passes with 100.0

This occurs when only performance and environment dimensions are assessed (code/tests/dependencies/docs require additional skill integration).

## Next Steps

1. **Customize Benchmarks**: Add benchmarks specific to your application
2. **Set Quality Thresholds**: Adjust thresholds based on your requirements
3. **Integrate with CI/CD**: Add quality gates to your pipeline
4. **Track Trends**: Compare multiple releases to track quality trends
5. **Generate Reports**: Share quality reports with stakeholders

## Additional Resources

- **Full Documentation**: `docs/RELEASE_QUALITY_IMPLEMENTATION_COMPLETE.md`
- **Skill Documentation**:
  - `skills/performance_profiler/skill.md`
  - `skills/environment_profiler/skill.md`
  - `skills/release_orchestrator/skill.md`

- **Demos**:
  - `skills/performance_profiler/demo.py`
  - `skills/environment_profiler/demo.py`
  - `skills/release_orchestrator/demo.py`

---

*For questions or issues, refer to the comprehensive documentation in `docs/`*
