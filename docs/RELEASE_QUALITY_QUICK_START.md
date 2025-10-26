# Release Quality System - Quick Start Guide

**Version:** 1.0.0
**Date:** 2025-10-26

---

## Overview

The Release Quality System provides automated assessment of code releases through performance profiling, dependency tracking, environment snapshots, and historical comparison.

---

## Key Concepts

### 1. Release Snapshot
A complete capture of a release at a point in time, including:
- Performance metrics (latency, throughput, resource usage)
- Dependency versions and checksums
- Hardware and software environment
- Test results and coverage
- Code quality metrics

### 2. Baseline Comparison
Every release is compared against a baseline (typically the previous release) to detect:
- Performance regressions
- Dependency changes
- Quality degradations
- Security vulnerabilities

### 3. Quality Score
An overall score (0-100) calculated from:
- **Performance (40%)** - Latency, throughput, resource efficiency
- **Dependencies (20%)** - Security, freshness, compatibility
- **Testing (20%)** - Coverage, pass rate, flakiness
- **Security (20%)** - Vulnerabilities, license issues

---

## Quick Usage

### Capture Release Snapshot

```python
from skills.integration import SkillInvoker, SkillLoader, SkillRegistry

registry = SkillRegistry('skills')
loader = SkillLoader(registry)
invoker = SkillInvoker(loader)

# Capture everything for current release
snapshot = invoker.invoke('release-quality-analyzer', 'capture_snapshot', {
    'release_version': 'v1.3.0',
    'capture_performance': True,
    'capture_dependencies': True,
    'capture_environment': True,
    'run_benchmarks': True
})

print(f"Snapshot ID: {snapshot.data['snapshot_id']}")
print(f"Quality Score: {snapshot.data['quality_score']}/100")
```

### Compare Releases

```python
# Compare current release with baseline
comparison = invoker.invoke('release-comparator', 'compare_releases', {
    'release_a': 'v1.2.0',
    'release_b': 'v1.3.0',
    'include_performance': True,
    'include_dependencies': True
})

print(f"Risk Level: {comparison.data['risk_level']}")
print(f"Recommendation: {comparison.data['recommendation']}")

if comparison.data['performance_delta']['regressed_operations']:
    print("Performance Regressions:")
    for op in comparison.data['performance_delta']['regressed_operations']:
        print(f"  - {op}")
```

### Detect Regressions

```python
# Automated regression detection
regressions = invoker.invoke('release-comparator', 'detect_regressions', {
    'current_release': 'v1.3.0',
    'baseline_release': 'v1.2.0',
    'regression_threshold_percent': 5.0,  # 5% slower = regression
    'check_performance': True,
    'check_memory': True
})

if regressions.data['regressions_found']:
    print(f"Found {regressions.data['total_regressions']} regressions!")
    for reg in regressions.data['performance_regressions']:
        print(f"  {reg['operation']}: {reg['delta_percent']}% slower")
```

---

## 5 New Skills

### 1. performance-profiler
**Purpose:** Collect performance metrics and distributed traces

**Key Operations:**
- `capture_traces` - Collect Jaeger/OpenTelemetry traces
- `profile_operations` - Profile specific functions
- `benchmark_release` - Run comprehensive benchmarks
- `analyze_resource_usage` - Monitor CPU/memory/disk/network

**Example:**
```python
result = invoker.invoke('performance-profiler', 'benchmark_release', {
    'benchmark_suite': 'benchmarks/api_tests.py',
    'baseline_release': 'v1.2.0',
    'iterations': 100
})
# Returns: benchmark results with baseline comparison
```

### 2. dependency-snapshot
**Purpose:** Capture and compare dependency states

**Key Operations:**
- `capture_snapshot` - Create dependency snapshot with versions
- `compare_snapshots` - Find differences between snapshots
- `validate_snapshot` - Check for vulnerabilities and integrity

**Example:**
```python
result = invoker.invoke('dependency-snapshot', 'capture_snapshot', {
    'project_path': '.',
    'include_transitive': True,
    'ecosystem': 'python'
})
# Returns: complete dependency graph with versions and hashes
```

### 3. environment-profiler
**Purpose:** Snapshot hardware and runtime environment

**Key Operations:**
- `capture_environment` - Record hardware, OS, runtime details
- `compare_environments` - Identify environment differences
- `benchmark_hardware` - Measure hardware performance

**Example:**
```python
result = invoker.invoke('environment-profiler', 'capture_environment', {
    'include_hardware': True,
    'include_software': True,
    'include_config': True
})
# Returns: CPU, RAM, disk, OS, Python version, env vars, etc.
```

### 4. release-comparator
**Purpose:** Compare releases across all dimensions

**Key Operations:**
- `compare_releases` - Comprehensive multi-dimensional comparison
- `detect_regressions` - Automated regression detection
- `generate_release_report` - Create detailed quality report

**Example:**
```python
result = invoker.invoke('release-comparator', 'generate_release_report', {
    'release_version': 'v1.3.0',
    'baseline_version': 'v1.2.0',
    'output_format': 'html'
})
# Returns: comprehensive report with quality score and recommendations
```

### 5. metrics-storage
**Purpose:** Store and query historical metrics

**Key Operations:**
- `store_release_snapshot` - Persist release data
- `query_metrics` - Query historical data with filters
- `get_trend_analysis` - Analyze trends over time
- `export_dataset` - Export data for analysis

**Example:**
```python
result = invoker.invoke('metrics-storage', 'get_trend_analysis', {
    'metric_name': 'api_latency_p95',
    'releases': ['v1.0.0', 'v1.1.0', 'v1.2.0', 'v1.3.0'],
    'detect_anomalies': True
})
# Returns: trend analysis with statistics and anomaly detection
```

---

## CI/CD Integration

### GitHub Actions Example

```yaml
name: Release Quality Check

on:
  pull_request:
    branches: [main]
  release:
    types: [published]

jobs:
  quality-check:
    runs-on: ubuntu-latest

    steps:
      - uses: actions/checkout@v3

      - name: Install Dependencies
        run: pip install -r requirements.txt

      - name: Capture Release Snapshot
        id: snapshot
        run: |
          python -c "
          from skills.integration import SkillInvoker, SkillLoader, SkillRegistry

          registry = SkillRegistry('skills')
          loader = SkillLoader(registry)
          invoker = SkillInvoker(loader)

          # Capture complete snapshot
          result = invoker.invoke('release-quality-analyzer', 'capture_snapshot', {
              'release_version': '${{ github.ref_name }}',
              'capture_performance': True,
              'capture_dependencies': True,
              'run_benchmarks': True
          })

          print(f'quality_score={result.data[\"quality_score\"]}')
          print(f'snapshot_id={result.data[\"snapshot_id\"]}')
          "

      - name: Compare with Baseline
        run: |
          python -c "
          from skills.integration import SkillInvoker, SkillLoader, SkillRegistry

          registry = SkillRegistry('skills')
          loader = SkillLoader(registry)
          invoker = SkillInvoker(loader)

          # Compare with previous release
          result = invoker.invoke('release-comparator', 'detect_regressions', {
              'current_release': '${{ github.ref_name }}',
              'baseline_release': 'v1.2.0',
              'regression_threshold_percent': 5.0
          })

          if result.data['blocking_regressions'] > 0:
              exit(1)  # Fail CI if blocking regressions found
          "

      - name: Generate Report
        run: |
          python -c "
          from skills.integration import SkillInvoker, SkillLoader, SkillRegistry

          registry = SkillRegistry('skills')
          loader = SkillLoader(registry)
          invoker = SkillInvoker(loader)

          # Generate HTML report
          result = invoker.invoke('release-comparator', 'generate_release_report', {
              'release_version': '${{ github.ref_name }}',
              'baseline_version': 'v1.2.0',
              'output_format': 'html'
          })

          print(f'Report: {result.data[\"report_path\"]}')
          "

      - name: Upload Report
        uses: actions/upload-artifact@v3
        with:
          name: release-quality-report
          path: reports/
```

---

## Dashboard Access

### Starting the Dashboard

```bash
# Start database and Jaeger
docker-compose up -d

# Start backend API
cd skills/quality_dashboard/backend
uvicorn api.main:app --reload

# Start frontend (separate terminal)
cd skills/quality_dashboard/frontend
npm start
```

### Accessing the Dashboard

- **Dashboard:** http://localhost:3000
- **API Docs:** http://localhost:8000/docs
- **Jaeger UI:** http://localhost:16686
- **Grafana:** http://localhost:3001

---

## Common Workflows

### Workflow 1: Pre-Release Quality Gate

```python
# 1. Capture current state
snapshot = capture_snapshot(release='v1.3.0-rc1')

# 2. Run benchmarks
benchmarks = run_benchmarks(baseline='v1.2.0')

# 3. Check for regressions
regressions = detect_regressions(threshold=5.0)

# 4. Generate report
report = generate_report(format='html')

# 5. Decision
if snapshot['quality_score'] > 80 and not regressions['blocking_regressions']:
    approve_release()
else:
    request_review()
```

### Workflow 2: Post-Release Monitoring

```python
# 1. Tag release
tag_release(version='v1.3.0')

# 2. Start trace collection
start_traces(duration='24h', sampling_rate=0.1)

# 3. Monitor metrics
monitor_continuously(
    metrics=['latency', 'error_rate', 'memory'],
    alert_on_anomaly=True
)

# 4. After 24h, generate report
generate_stability_report(release='v1.3.0')
```

### Workflow 3: Trend Analysis

```python
# Analyze performance trends over last 10 releases
trend = analyze_trend(
    metric='api_latency_p95',
    releases=get_last_n_releases(10),
    detect_anomalies=True,
    forecast_periods=3
)

# Generate trend report
generate_trend_report(
    trends=[trend],
    output='trends_report.html'
)
```

---

## Data Retention

### Default Retention Policies

- **Performance Metrics:** 1 year (5-minute granularity)
- **Traces:** 30 days (sampled)
- **Release Snapshots:** Unlimited (compressed)
- **Dependency Snapshots:** Unlimited
- **Environment Snapshots:** Unlimited

### Custom Retention

```python
# Set custom retention policy
set_retention_policy({
    'performance_metrics': '2y',    # 2 years
    'traces': '90d',                # 90 days
    'snapshots': 'unlimited'
})
```

---

## Alerting

### Configure Alerts

```python
# Set up alerts for regressions
configure_alert({
    'name': 'API Latency Regression',
    'metric': 'api_latency_p95',
    'threshold': 5.0,  # 5% increase
    'window': '1h',
    'notify': ['team@example.com', 'slack://channel']
})

# Set up alerts for quality score drops
configure_alert({
    'name': 'Quality Score Drop',
    'metric': 'overall_quality_score',
    'threshold': -10,  # 10 point drop
    'notify': ['team@example.com']
})
```

---

## Best Practices

### 1. Baseline Management
- Always compare against the previous stable release
- Update baseline after each successful release
- Keep at least 3 baselines for comparison

### 2. Benchmark Consistency
- Run benchmarks on identical hardware
- Use dedicated benchmark environments
- Warm up JIT/cache before measuring
- Run multiple iterations for statistical significance

### 3. Threshold Tuning
- Start with conservative thresholds (5-10%)
- Adjust based on historical variance
- Different thresholds for different operations
- Separate thresholds for critical vs non-critical paths

### 4. Trace Sampling
- Production: 1-10% sampling rate
- Staging: 100% sampling rate
- Adjust based on traffic volume
- Always trace error cases

### 5. Data Management
- Archive old snapshots to cold storage
- Aggregate metrics for long-term trends
- Export datasets periodically for backup
- Implement data retention policies

---

## Troubleshooting

### Issue: High Trace Collection Overhead

**Solution:**
```python
# Reduce sampling rate
configure_tracing({
    'sampling_rate': 0.01,  # 1% instead of 10%
    'sample_errors': True    # Always sample errors
})
```

### Issue: Inconsistent Benchmark Results

**Solution:**
```python
# Increase iterations and warmup
benchmark_release({
    'warmup_iterations': 50,  # More warmup
    'iterations': 200,         # More iterations
    'discard_outliers': True   # Remove outliers
})
```

### Issue: Dashboard Slow to Load

**Solution:**
```python
# Enable caching and aggregation
configure_dashboard({
    'enable_cache': True,
    'cache_ttl': 300,          # 5 minutes
    'aggregate_metrics': True,  # Pre-aggregate
    'limit_results': 1000      # Limit query results
})
```

---

## Metrics Reference

### Performance Metrics
- `latency_ms` - Operation latency (avg, p50, p95, p99)
- `throughput_ops_sec` - Operations per second
- `error_rate` - Percentage of failed operations
- `cpu_percent` - CPU utilization
- `memory_mb` - Memory usage
- `disk_io_mbps` - Disk I/O throughput
- `network_mbps` - Network throughput

### Quality Metrics
- `overall_quality_score` - Overall quality (0-100)
- `performance_score` - Performance quality (0-100)
- `security_score` - Security quality (0-100)
- `test_coverage_percent` - Test coverage
- `code_quality_score` - Code quality (0-100)

### Dependency Metrics
- `total_dependencies` - Total dependency count
- `vulnerability_count` - Known vulnerabilities
- `outdated_dependencies` - Dependencies with updates
- `license_issues` - License compliance issues

---

## API Reference

### REST API Endpoints

```
GET    /api/releases                    # List all releases
GET    /api/releases/{version}          # Get release details
POST   /api/releases/{version}/snapshot # Create snapshot
GET    /api/releases/{version}/metrics  # Get release metrics

GET    /api/comparisons                 # List comparisons
POST   /api/comparisons                 # Create comparison
GET    /api/comparisons/{id}            # Get comparison details

GET    /api/metrics                     # Query metrics
GET    /api/metrics/{metric}/trend      # Get metric trend

GET    /api/traces                      # Query traces
GET    /api/traces/{trace_id}           # Get trace details

GET    /api/environment/{snapshot_id}   # Get environment snapshot
GET    /api/dependencies/{snapshot_id}  # Get dependency snapshot
```

---

## Next Steps

1. **Review the Plan:** See `docs/RELEASE_QUALITY_SYSTEM_PLAN.md`
2. **Set Up Infrastructure:** Database, Jaeger, Redis
3. **Implement Skills:** Start with performance-profiler
4. **Build Dashboard:** Backend API and frontend
5. **Integrate CI/CD:** Add quality checks to pipeline
6. **Deploy:** Production deployment

---

**Document Version:** 1.0.0
**Last Updated:** 2025-10-26
