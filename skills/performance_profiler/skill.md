# Performance Profiler Skill

Comprehensive performance profiling and benchmarking for release quality assessment.

## Operations

### profile_code_execution

Profile Python code execution using cProfile to identify performance bottlenecks.

**Input:**
```yaml
target_function: callable  # Function to profile
function_args: tuple       # Arguments for the function (optional)
function_kwargs: dict      # Keyword arguments (optional)
save_profile: bool         # Save profile data to file (default: true)
top_n_functions: int       # Number of top functions to include (default: 10)
output_dir: str           # Directory to store profiles (default: "./profiles")
```

**Output:**
```yaml
success: bool
data:
  function_name: str
  total_time: float        # Total execution time in seconds
  primitive_calls: int     # Number of primitive calls
  total_calls: int        # Total number of calls
  top_functions: list     # Top N functions by cumulative time
  profile_file: str       # Path to saved profile file
duration: float
metadata:
  profiler: str           # "cProfile"
  top_n: int
  saved: bool
```

**Example:**
```python
from skills.performance_profiler import profile_code_execution

def expensive_function(n):
    return sum(i**2 for i in range(n))

result = profile_code_execution(
    target_function=expensive_function,
    function_args=(1000000,),
    save_profile=True,
    top_n_functions=20
)
```

---

### capture_distributed_traces

Capture distributed traces using Jaeger to analyze request flows and latencies.

**Input:**
```yaml
service_name: str              # Name of the service being traced
duration_seconds: int          # How long to capture traces (default: 60)
jaeger_endpoint: str          # Jaeger collector endpoint (default: "http://localhost:14268/api/traces")
sampling_rate: float          # Sampling rate 0.0-1.0 (default: 1.0)
operation_patterns: list      # Operation patterns to filter, e.g. ['api.*', 'db.*'] (optional)
```

**Output:**
```yaml
success: bool
data:
  traces_collected: int
  trace_ids: list           # First 10 trace IDs
  avg_duration_ms: float
  p50_duration_ms: float    # 50th percentile
  p95_duration_ms: float    # 95th percentile
  p99_duration_ms: float    # 99th percentile
  error_rate: float         # Error rate (0.0-1.0)
  total_spans: int
duration: float
metadata:
  service_name: str
  sampling_rate: float
  duration_seconds: int
```

**Example:**
```python
from skills.performance_profiler import capture_distributed_traces

result = capture_distributed_traces(
    service_name="my-api-service",
    duration_seconds=120,
    sampling_rate=0.1,
    operation_patterns=["api.*", "db.query"]
)
```

---

### monitor_resource_usage

Monitor system resource usage (CPU, memory, disk, network) during an operation.

**Input:**
```yaml
operation_function: callable     # Function to monitor
operation_args: tuple           # Arguments for the function (optional)
operation_kwargs: dict          # Keyword arguments (optional)
sampling_interval_ms: int       # Sampling interval in milliseconds (default: 100)
track_cpu: bool                # Track CPU usage (default: true)
track_memory: bool             # Track memory usage (default: true)
track_disk: bool               # Track disk I/O (default: true)
track_network: bool            # Track network I/O (default: true)
```

**Output:**
```yaml
success: bool
data:
  operation_result: any         # Result from the operation
  resource_usage:
    duration_seconds: float
    samples_collected: int
    cpu_usage:
      avg_percent: float
      peak_percent: float
      min_percent: float
      samples: list
    memory_usage:
      avg_mb: float
      peak_mb: float
      min_mb: float
      samples: list
    disk_io:
      read_mb: float
      write_mb: float
      total_mb: float
    network_io:
      sent_mb: float
      recv_mb: float
      total_mb: float
duration: float
metadata:
  sampling_interval_ms: int
  tracked_resources: dict
```

**Example:**
```python
from skills.performance_profiler import monitor_resource_usage

def data_processing_task():
    # Process large dataset
    data = [i**2 for i in range(10000000)]
    return sum(data)

result = monitor_resource_usage(
    operation_function=data_processing_task,
    sampling_interval_ms=50,
    track_cpu=True,
    track_memory=True
)
```

---

### run_benchmark_suite

Run a comprehensive suite of benchmarks for a release.

**Input:**
```yaml
benchmarks: list              # List of benchmark definitions
                             # [{'name': str, 'func': callable, 'args': tuple, 'kwargs': dict}]
release_version: str         # Version identifier for this benchmark run
iterations: int              # Number of iterations per benchmark (default: 10)
warmup_iterations: int       # Number of warmup runs (default: 3)
track_resources: bool        # Track CPU/memory during benchmarks (default: true)
output_dir: str             # Directory to store results (default: "./benchmarks")
regression_threshold: float  # Threshold for detecting regressions (default: 0.05 = 5%)
```

**Output:**
```yaml
success: bool
data:
  release_version: str
  total_benchmarks: int
  results: list               # List of BenchmarkResult objects
  summary:
    avg_time_ms: float
    total_duration_ms: float
    avg_throughput: float     # Operations per second
duration: float
metadata:
  iterations: int
  warmup_iterations: int
  track_resources: bool
  output_dir: str
```

**Example:**
```python
from skills.performance_profiler import run_benchmark_suite

benchmarks = [
    {
        'name': 'list_comprehension',
        'func': lambda: [i**2 for i in range(10000)],
        'args': (),
        'kwargs': {}
    },
    {
        'name': 'generator_expression',
        'func': lambda: sum(i**2 for i in range(10000)),
        'args': (),
        'kwargs': {}
    }
]

result = run_benchmark_suite(
    benchmarks=benchmarks,
    release_version="v1.2.0",
    iterations=50,
    warmup_iterations=5
)
```

---

### compare_with_baseline

Compare current benchmark results with a baseline to detect performance regressions.

**Input:**
```yaml
current_results_file: str    # Path to current benchmark results JSON file
baseline_version: str        # Version identifier for baseline
output_dir: str             # Directory containing benchmark results (default: "./benchmarks")
regression_threshold: float  # Threshold for detecting regressions (default: 0.05 = 5%)
```

**Output:**
```yaml
success: bool                # False if regressions detected
data:
  current_version: str
  baseline_version: str
  timestamp: float
  total_benchmarks: int
  comparison_details: list
    - benchmark_name: str
      current_avg_ms: float
      baseline_avg_ms: float
      change_ms: float
      change_percent: float
      current_throughput: float
      baseline_throughput: float
      throughput_change_percent: float
      current_memory_mb: float
      baseline_memory_mb: float
      is_regression: bool
      is_improvement: bool
  regression_analysis:
    has_regression: bool
    regressed_benchmarks: list
      - benchmark: str
        change_percent: float
        change_ms: float
    improved_benchmarks: list
    unchanged_benchmarks: list
    overall_change_percent: float
duration: float
metadata:
  baseline_version: str
  regression_threshold: float
  has_regressions: bool
  regression_count: int
```

**Example:**
```python
from skills.performance_profiler import compare_with_baseline

result = compare_with_baseline(
    current_results_file="./benchmarks/benchmark_v1.2.0_1234567890.json",
    baseline_version="v1.1.0",
    regression_threshold=0.05
)

if not result.success:
    print(f"⚠️ Regressions detected: {result.metadata['regression_count']}")
    for regression in result.data['regression_analysis']['regressed_benchmarks']:
        print(f"  - {regression['benchmark']}: +{regression['change_percent']:.2f}%")
```

---

### generate_performance_report

Generate a comprehensive performance report from comparison data.

**Input:**
```yaml
comparison_data: dict        # Comparison data from compare_with_baseline
output_file: str            # Path to output report file
include_charts: bool        # Include performance charts (default: false, requires matplotlib)
```

**Output:**
```yaml
success: bool
data:
  report_file: str          # Path to generated report
  summary:
    current_version: str
    baseline_version: str
    total_benchmarks: int
    has_regressions: bool
    regression_count: int
    improvement_count: int
    overall_change_percent: float
duration: float
metadata:
  include_charts: bool
  format: str              # "json" or "html"
```

**Example:**
```python
from skills.performance_profiler import compare_with_baseline, generate_performance_report

# First compare with baseline
comparison_result = compare_with_baseline(
    current_results_file="./benchmarks/benchmark_v1.2.0_1234567890.json",
    baseline_version="v1.1.0"
)

# Then generate report
report_result = generate_performance_report(
    comparison_data=comparison_result.data,
    output_file="./reports/performance_report_v1.2.0.json",
    include_charts=False
)
```

---

## Integration with Other Skills

The performance-profiler skill integrates with:

- **release-orchestrator**: Provides performance metrics for quality scoring
- **code-instrumenter**: Uses instrumentation to capture detailed traces
- **cicd-generator**: Benchmark results are used in CI/CD quality gates
- **dependency-guardian**: Correlates performance changes with dependency updates

## Dependencies

**Required:**
- Python 3.8+
- Standard library: `cProfile`, `pstats`, `time`, `threading`, `json`, `dataclasses`

**Optional:**
- `psutil`: For resource monitoring (CPU, memory, disk, network)
- `opentelemetry`: For distributed tracing integration
- `matplotlib`: For generating performance charts in reports

**Install optional dependencies:**
```bash
pip install psutil opentelemetry-api opentelemetry-sdk opentelemetry-exporter-jaeger matplotlib
```

## Output Structure

All benchmark results are saved to the output directory with the following structure:

```
./benchmarks/
├── benchmark_v1.0.0_1234567890.json
├── benchmark_v1.1.0_1234567891.json
└── benchmark_v1.2.0_1234567892.json
```

Each benchmark file contains:
```json
{
  "release_version": "v1.2.0",
  "timestamp": 1234567892.0,
  "timestamp_iso": "2024-01-15T10:30:00",
  "total_benchmarks": 5,
  "results": [
    {
      "benchmark_name": "list_comprehension",
      "release_version": "v1.2.0",
      "timestamp": 1234567892.0,
      "duration_ms": 150.5,
      "iterations": 10,
      "avg_time_ms": 15.05,
      "min_time_ms": 14.2,
      "max_time_ms": 16.8,
      "std_dev_ms": 0.8,
      "throughput_ops_sec": 66.45,
      "memory_peak_mb": 25.3,
      "cpu_avg_percent": 45.2,
      "metadata": {}
    }
  ]
}
```

## Error Handling

All operations return `OperationResult` with standardized error codes:

- `PROFILE_ERROR`: Error during code profiling
- `TRACE_CAPTURE_ERROR`: Error capturing distributed traces
- `RESOURCE_MONITOR_ERROR`: Error monitoring system resources
- `BENCHMARK_ERROR`: Error running benchmarks
- `COMPARISON_ERROR`: Error comparing with baseline
- `REPORT_GENERATION_ERROR`: Error generating report

## Best Practices

1. **Warmup Iterations**: Always use warmup iterations to ensure JIT compilation and caching effects don't skew results
2. **Multiple Iterations**: Run enough iterations to get statistically significant results (minimum 10)
3. **Consistent Environment**: Run benchmarks in consistent environments (same hardware, load, etc.)
4. **Baseline Maintenance**: Keep baseline results for all major releases
5. **Regression Threshold**: Set appropriate regression thresholds based on your performance requirements
6. **Resource Tracking**: Enable resource tracking to identify memory leaks and CPU bottlenecks
7. **Sampling Rate**: Use lower sampling rates in production to minimize overhead

## Version

0.1.0
