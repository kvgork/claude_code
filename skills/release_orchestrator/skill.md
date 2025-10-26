# Release Orchestrator Skill

Coordinate multiple skills for comprehensive release quality assessment with multi-dimensional scoring and automated reporting.

## Operations

### assess_release_quality

Perform comprehensive release quality assessment by coordinating multiple skills.

**Input:**
```yaml
release_version: str           # Version identifier (e.g., "v1.0.0", "production")
project_path: str             # Path to project root
baseline_version: str         # Baseline version for comparison (optional)
benchmark_suite: list         # List of benchmarks to run (optional)
                             # [{'name': str, 'func': callable, 'args': tuple, 'kwargs': dict}]
quality_gate_threshold: float # Minimum quality score to pass (default: 70.0)
enable_performance: bool      # Enable performance profiling (default: true)
enable_environment: bool      # Enable environment profiling (default: true)
```

**Output:**
```yaml
success: bool                 # True if quality gate passed
data:
  release_version: str
  timestamp: float
  overall_quality_score: float  # 0-100
  grade: str                   # A+, A, B+, B, C, D, F
  skill_results: dict
    performance:
      skill_name: str
      operation_name: str
      success: bool
      data: dict
      duration: float
    environment:
      skill_name: str
      operation_name: str
      success: bool
      data: dict
      duration: float
  quality_dimensions: list
    - name: str
      score: float             # 0-100
      weight: float            # Weight in overall score
      metrics: dict
      issues: list
  recommendations: list        # Suggested improvements
  warnings: list              # Critical issues
  passed_quality_gate: bool
duration: float
metadata:
  overall_score: float
  grade: str
  passed_quality_gate: bool
  warning_count: int
  recommendation_count: int
```

**Example:**
```python
from skills.release_orchestrator import assess_release_quality

# Define benchmarks
benchmarks = [
    {
        'name': 'api_endpoint_latency',
        'func': test_api_latency,
        'args': (),
        'kwargs': {}
    },
    {
        'name': 'database_query_performance',
        'func': test_db_query,
        'args': (),
        'kwargs': {}
    }
]

# Assess release quality
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
    print(f"✅ Quality Gate: PASSED")
    print(f"Score: {result.metadata['overall_score']:.1f}/100")
    print(f"Grade: {result.metadata['grade']}")
else:
    print(f"❌ Quality Gate: FAILED")
    print(f"Score: {result.metadata['overall_score']:.1f}/100")

    # Show warnings
    for warning in result.data['warnings']:
        print(f"⚠️  {warning}")
```

---

### generate_quality_report

Generate comprehensive quality report in Markdown or JSON format.

**Input:**
```yaml
assessment_data: dict         # Assessment data from assess_release_quality
output_format: str           # "markdown" or "json"
output_file: str             # Optional output file path
```

**Output:**
```yaml
success: bool
data:
  report_file: str           # Path to generated report
  format: str               # "markdown" or "json"
duration: float
metadata:
  file_size_bytes: int
```

**Example:**
```python
from skills.release_orchestrator import assess_release_quality, generate_quality_report

# First assess the release
assessment = assess_release_quality(
    release_version="v1.0.0",
    project_path=".",
    benchmark_suite=benchmarks
)

# Generate Markdown report
md_report = generate_quality_report(
    assessment_data=assessment.data,
    output_format="markdown",
    output_file="./reports/release_v1.0.0.md"
)

print(f"Report generated: {md_report.data['report_file']}")

# Generate JSON report for automation
json_report = generate_quality_report(
    assessment_data=assessment.data,
    output_format="json",
    output_file="./reports/release_v1.0.0.json"
)
```

---

### calculate_quality_score

Calculate quality score from individual analysis results without full orchestration.

**Input:**
```yaml
code_analysis: dict           # Code quality metrics (optional)
test_analysis: dict           # Test quality metrics (optional)
dependency_analysis: dict     # Dependency metrics (optional)
doc_analysis: dict           # Documentation metrics (optional)
performance_analysis: dict    # Performance metrics (optional)
custom_weights: dict         # Custom dimension weights (optional)
                            # {dimension_name: weight}, must sum to 1.0
```

**Output:**
```yaml
success: bool
data:
  overall_score: float        # 0-100
  grade: str                 # A+, A, B+, B, C, D, F
  dimensions: list
    - name: str
      score: float
      weight: float
      metrics: dict
      issues: list
  timestamp: float
  release_version: str
duration: float
metadata:
  overall_score: float
  grade: str
  dimension_count: int
```

**Example:**
```python
from skills.release_orchestrator import calculate_quality_score

# Calculate score with custom weights
result = calculate_quality_score(
    performance_analysis={
        'regression_count': 2,
        'avg_change_percent': 8.5,
        'memory_change_percent': 3.2,
        'throughput_change_percent': -1.5
    },
    custom_weights={
        'code_quality': 0.30,
        'test_quality': 0.30,
        'dependencies': 0.10,
        'documentation': 0.10,
        'performance': 0.20
    }
)

print(f"Overall Score: {result.data['overall_score']:.1f}")
print(f"Grade: {result.data['grade']}")

for dimension in result.data['dimensions']:
    print(f"{dimension['name']}: {dimension['score']:.1f}/100")
```

---

## Quality Scoring System

### Dimensions and Weights

| Dimension       | Default Weight | Metrics Evaluated                                    |
|-----------------|---------------|-----------------------------------------------------|
| **Code Quality**    | 25%           | Cyclomatic complexity, code smells, maintainability, duplication |
| **Test Quality**    | 20%           | Coverage %, success rate, test count, test performance |
| **Dependencies**    | 20%           | Vulnerabilities, outdated packages, license compliance |
| **Documentation**   | 15%           | Doc coverage %, README score, API docs, comment ratio |
| **Performance**     | 20%           | Regressions, resource usage, response times, throughput |

### Grading Scale

| Score Range | Grade | Quality Level        | Action                              |
|-------------|-------|---------------------|-------------------------------------|
| 97-100      | A+    | Exceptional         | Release immediately                  |
| 93-96       | A     | Excellent           | Release with confidence              |
| 90-92       | A-    | Very Good           | Release after minor review           |
| 87-89       | B+    | Good                | Consider addressing recommendations   |
| 83-86       | B     | Above Average       | Address key recommendations          |
| 80-82       | B-    | Solid               | Review warnings before release       |
| 77-79       | C+    | Acceptable          | Address warnings, consider delay     |
| 73-76       | C     | Minimally Acceptable| Fix critical issues first            |
| 70-72       | C-    | Borderline          | Significant improvements needed      |
| 60-69       | D     | Below Acceptable    | Do not release                       |
| 0-59        | F     | Failing             | Major rework required                |

### Quality Gate Thresholds

**Recommended thresholds by release type:**

- **Production Release**: 80.0+ (B- or better)
- **Staging Release**: 70.0+ (C- or better)
- **Development Build**: 60.0+ (D or better)
- **Experimental**: No threshold

---

## Integration with Other Skills

The release-orchestrator coordinates these skills:

### Currently Integrated

1. **performance-profiler** ✅
   - Benchmark execution
   - Regression detection
   - Performance scoring

2. **environment-profiler** ✅
   - Environment snapshot
   - Package tracking
   - Reproducibility verification

### Planned Integration

3. **refactor-assistant** (existing skill)
   - Code quality analysis
   - Complexity metrics
   - Code smell detection

4. **test-orchestrator** (existing skill)
   - Test execution
   - Coverage analysis
   - Test quality scoring

5. **dependency-guardian** (existing skill)
   - Vulnerability scanning
   - Outdated package detection
   - License compliance

6. **doc-generator** (existing skill)
   - Documentation coverage
   - README analysis
   - API documentation quality

---

## Skill Orchestration Flow

```
1. Environment Assessment
   ├─ Create environment snapshot (current release)
   ├─ Compare with baseline (if provided)
   └─ Extract environment metrics

2. Performance Assessment
   ├─ Run benchmark suite
   ├─ Compare with baseline (if provided)
   ├─ Detect regressions
   └─ Extract performance metrics

3. Quality Scoring
   ├─ Score each dimension independently (0-100)
   ├─ Apply dimension weights
   ├─ Calculate overall weighted score
   └─ Assign letter grade

4. Quality Gate Evaluation
   ├─ Compare score vs threshold
   ├─ Generate recommendations
   ├─ Generate warnings
   └─ Determine pass/fail status

5. Report Generation
   ├─ Compile all results
   ├─ Generate Markdown report
   ├─ Generate JSON report
   └─ Save artifacts
```

---

## Report Formats

### Markdown Report Structure

```markdown
# Release Quality Report: v1.0.0

**Generated:** 2025-10-26 13:42:05

## Quality Gate: ✅ PASSED

- **Overall Score:** 85.0/100
- **Grade:** B+

## ⚠️ Warnings
- performance assessment: 2 performance regressions detected

## Quality Dimensions

| Dimension | Score | Weight | Status |
|-----------|-------|--------|--------|
| Performance | 78.0/100 | 20% | ✓ |

### Performance

**Score:** 78.0/100

**Metrics:**
- Regression Count: 2
- Avg Change Percent: 8.50
- Memory Change Percent: 0.00
- Throughput Change Percent: 0.00

**Issues:**
- Performance regressions: 2
- Performance degradation: +8.5%

## Skill Execution Results

| Skill | Operation | Status | Duration |
|-------|-----------|--------|----------|
| environment | create_environment_snapshot | ✓ | 3.67s |
| performance | run_benchmark_suite | ✓ | 0.12s |

## Performance Analysis
[Detailed performance data...]

## Environment Profile
[Detailed environment data...]

## 📋 Recommendations
1. Investigate 2 performance regression(s) before release

---
*Report generated by Release Orchestrator*
```

### JSON Report Structure

```json
{
  "report_type": "release_quality_assessment",
  "generated_at": "2025-10-26T13:42:05",
  "release_version": "v1.0.0",
  "quality_gate": {
    "passed": true,
    "score": 85.0,
    "grade": "B+"
  },
  "assessment": {
    "release_version": "v1.0.0",
    "timestamp": 1729952525.0,
    "overall_quality_score": 85.0,
    "grade": "B+",
    "skill_results": {...},
    "quality_dimensions": [...],
    "recommendations": [...],
    "warnings": [...],
    "passed_quality_gate": true
  }
}
```

---

## Use Cases

### 1. CI/CD Quality Gate

```python
# .github/workflows/quality-gate.yml integration

from skills.release_orchestrator import assess_release_quality
import sys

result = assess_release_quality(
    release_version=os.getenv('GITHUB_REF'),
    project_path='.',
    baseline_version='main',
    benchmark_suite=BENCHMARKS,
    quality_gate_threshold=80.0  # Production threshold
)

# Exit with error code if quality gate failed
sys.exit(0 if result.success else 1)
```

### 2. Pre-Release Validation

```python
# Validate release before deployment

result = assess_release_quality(
    release_version="v2.0.0",
    project_path=".",
    baseline_version="v1.9.0",
    benchmark_suite=CRITICAL_BENCHMARKS,
    quality_gate_threshold=90.0  # Stricter for major releases
)

if not result.success:
    print("❌ Major release failed quality gate")
    print(f"Score: {result.metadata['overall_score']:.1f}/100")

    # Block deployment
    notify_team("Release v2.0.0 blocked due to quality issues")
    rollback_deployment()
else:
    print("✅ Major release passed quality gate")
    proceed_with_deployment()
```

### 3. Quality Trend Tracking

```python
# Track quality over multiple releases

releases = ["v1.0.0", "v1.1.0", "v1.2.0", "v2.0.0"]
quality_trend = []

for i, version in enumerate(releases):
    baseline = releases[i-1] if i > 0 else None

    result = assess_release_quality(
        release_version=version,
        project_path=".",
        baseline_version=baseline,
        benchmark_suite=BENCHMARKS
    )

    quality_trend.append({
        'version': version,
        'score': result.metadata['overall_score'],
        'grade': result.metadata['grade'],
        'passed': result.success
    })

# Analyze trend
print("Quality Trend:")
for entry in quality_trend:
    status = "✓" if entry['passed'] else "✗"
    print(f"{status} {entry['version']}: {entry['score']:.1f} ({entry['grade']})")
```

### 4. Custom Quality Profiles

```python
# Different quality profiles for different teams

# Backend team: emphasize performance and dependencies
backend_weights = {
    'code_quality': 0.20,
    'test_quality': 0.25,
    'dependencies': 0.25,
    'documentation': 0.10,
    'performance': 0.20
}

# Frontend team: emphasize code quality and documentation
frontend_weights = {
    'code_quality': 0.30,
    'test_quality': 0.20,
    'dependencies': 0.15,
    'documentation': 0.20,
    'performance': 0.15
}

from skills.release_orchestrator import calculate_quality_score

backend_score = calculate_quality_score(
    performance_analysis={...},
    custom_weights=backend_weights
)

frontend_score = calculate_quality_score(
    performance_analysis={...},
    custom_weights=frontend_weights
)
```

---

## Error Handling

All operations return `OperationResult` with standardized error codes:

- `ASSESSMENT_ERROR`: Error during release assessment
- `REPORT_GENERATION_ERROR`: Error generating quality report
- `SCORE_CALCULATION_ERROR`: Error calculating quality score

**Example error handling:**

```python
result = assess_release_quality(
    release_version="v1.0.0",
    project_path=".",
    benchmark_suite=BENCHMARKS
)

if not result.success:
    print(f"Error: {result.error}")
    print(f"Error Code: {result.error_code}")

    # Check metadata for partial results
    if result.metadata:
        print(f"Partial score: {result.metadata.get('overall_score', 'N/A')}")
```

---

## Best Practices

1. **Always Use Baseline Comparison**: Compare each release with the previous version to detect regressions

2. **Set Appropriate Thresholds**:
   - Production: 80.0+
   - Staging: 70.0+
   - Development: 60.0+

3. **Include Comprehensive Benchmarks**: Cover all critical code paths in your benchmark suite

4. **Track Quality Trends**: Monitor quality scores across releases to identify degradation patterns

5. **Address Warnings Immediately**: Don't release with unresolved warnings

6. **Generate Reports for Every Release**: Maintain historical record of quality assessments

7. **Customize Weights**: Adjust dimension weights based on your project priorities

8. **Automate in CI/CD**: Integrate quality gates into your deployment pipeline

9. **Review Recommendations**: Even if quality gate passes, address recommendations for future releases

10. **Use JSON Reports for Automation**: Parse JSON reports in scripts for automated decision-making

---

## Dependencies

**Required:**
- Python 3.8+
- skills.performance_profiler
- skills.environment_profiler

**Optional:**
- skills.refactor_assistant (for code quality dimension)
- skills.test_orchestrator (for test quality dimension)
- skills.dependency_guardian (for dependency dimension)
- skills.doc_generator (for documentation dimension)

---

## Limitations

1. **Code/Test/Dependency/Documentation Dimensions**: Require integration with additional skills (currently return placeholder errors)

2. **Performance Dimension**: Only available if performance-profiler enabled and benchmark suite provided

3. **Environment Dimension**: Used for reproducibility verification, doesn't directly contribute to quality score

4. **Baseline Comparison**: Requires previous release assessment results

---

## Version

0.1.0

---

## See Also

- **performance-profiler**: Performance benchmarking and profiling
- **environment-profiler**: Environment detection and comparison
- **Quick Start Guide**: `/RELEASE_QUALITY_QUICK_START.md`
- **Implementation Details**: `/docs/RELEASE_QUALITY_IMPLEMENTATION_COMPLETE.md`
