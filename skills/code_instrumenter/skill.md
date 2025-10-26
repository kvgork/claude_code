---
name: code-instrumenter
version: 0.1.0
description: Automatically add instrumentation (tracing, profiling, metrics) to existing codebases
category: automation
author: Claude Code Skills
dependencies: []
tags: [instrumentation, tracing, profiling, metrics, automation, opentelemetry, jaeger]
operations:
  analyze_codebase: "Scan codebase to determine what instrumentation is needed"
  add_tracing: "Automatically add OpenTelemetry distributed tracing"
  add_profiling: "Add performance profiling decorators to functions"
  add_metrics: "Add Prometheus-style metrics endpoints"
  add_health_checks: "Add comprehensive health check endpoints"
  generate_config: "Generate complete instrumentation configuration files"
---

# Code Instrumenter

## Overview

Code Instrumenter is an automation skill that analyzes existing codebases and automatically adds production-ready instrumentation including distributed tracing, performance profiling, metrics collection, and health checks.

**Key Capability:** Transform uninstrumented code into fully observable production code in minutes.

## Capabilities

### 🔍 Intelligent Analysis
- AST-based code analysis
- Framework detection (FastAPI, Flask, Django, Express, etc.)
- Database connection detection
- API endpoint discovery
- Async operation identification

### 📊 OpenTelemetry Tracing
- Automatic OTLP/Jaeger integration
- Auto-instrument frameworks (FastAPI, SQLAlchemy, Redis, etc.)
- Manual span injection for custom code
- Context propagation
- Sampling configuration

### ⚡ Performance Profiling
- cProfile integration
- Memory profiling
- CPU profiling
- Custom profiling decorators
- Automatic slow function detection

### 📈 Metrics Collection
- Prometheus-compatible metrics
- Request/response metrics
- Error rate tracking
- Latency histograms
- Custom business metrics

### 🏥 Health Checks
- Liveness endpoints
- Readiness endpoints
- Dependency health checks (DB, Redis, external services)
- Graceful degradation

## Operations

### analyze_codebase
Scan codebase to identify instrumentation opportunities.

**Input:**
```python
{
    'project_path': str,
    'language': str,               # 'python', 'javascript', 'java'
    'framework': str,              # 'fastapi', 'flask', 'express'
    'detect_endpoints': bool,
    'detect_database': bool
}
```

**Output:**
```python
{
    'language': 'python',
    'framework': 'fastapi',
    'entry_points': [
        {'file': 'app/main.py', 'type': 'api', 'endpoints': 15}
    ],
    'database_connections': [
        {'file': 'app/db.py', 'type': 'sqlalchemy', 'models': 8}
    ],
    'async_operations': [
        {'file': 'app/services.py', 'functions': ['fetch_data']}
    ],
    'instrumentation_needed': [
        'opentelemetry-api',
        'opentelemetry-instrumentation-fastapi'
    ]
}
```

### add_tracing
Automatically add OpenTelemetry distributed tracing.

**Input:**
```python
{
    'project_path': str,
    'service_name': str,
    'jaeger_endpoint': str,        # 'http://localhost:14268'
    'sampling_rate': float,        # 0.1 = 10%
    'auto_instrument': bool,       # Auto-instrument frameworks
    'manual_spans': List[str],     # Functions to manually instrument
    'trace_database': bool,
    'trace_http': bool,
    'trace_redis': bool
}
```

**Output:**
```python
{
    'success': True,
    'files_modified': ['app/main.py', 'requirements.txt'],
    'files_created': ['app/instrumentation.py'],
    'dependencies_added': [
        'opentelemetry-api==1.21.0',
        'opentelemetry-sdk==1.21.0'
    ],
    'instrumentation_points': 15,
    'auto_instrumented': ['FastAPI', 'SQLAlchemy'],
    'manual_spans_added': ['process_payment']
}
```

### add_profiling
Add performance profiling decorators.

**Input:**
```python
{
    'project_path': str,
    'target_functions': List[str],  # Functions to profile
    'auto_detect_slow': bool,       # Auto-detect slow functions
    'profile_memory': bool,
    'profile_cpu': bool,
    'output_dir': str
}
```

**Output:**
```python
{
    'success': True,
    'profiled_functions': ['process_order', 'generate_report'],
    'profiling_module': 'app/profiling.py',
    'output_directory': './profiles'
}
```

### add_metrics
Add Prometheus-style metrics endpoints.

**Input:**
```python
{
    'project_path': str,
    'framework': str,
    'metrics_endpoint': str,       # '/metrics'
    'track_requests': bool,
    'track_latency': bool,
    'track_errors': bool,
    'custom_metrics': List[Dict]
}
```

**Output:**
```python
{
    'success': True,
    'metrics_endpoint': '/metrics',
    'metrics_added': [
        'http_requests_total',
        'http_request_duration_seconds',
        'http_errors_total'
    ]
}
```

### add_health_checks
Add comprehensive health check endpoints.

**Input:**
```python
{
    'project_path': str,
    'check_database': bool,
    'check_redis': bool,
    'check_external_services': List[str],
    'health_endpoint': str,        # '/health'
    'readiness_endpoint': str      # '/ready'
}
```

**Output:**
```python
{
    'success': True,
    'health_endpoint': '/health',
    'readiness_endpoint': '/ready',
    'checks_added': ['database', 'redis', 'external_api']
}
```

### generate_config
Generate complete instrumentation configuration.

**Input:**
```python
{
    'project_path': str,
    'environment': str,            # 'development', 'production'
    'jaeger_endpoint': str,
    'metrics_port': int,
    'log_level': str
}
```

**Output:**
```python
{
    'success': True,
    'config_files': [
        'config/instrumentation.yaml',
        'config/logging.yaml'
    ]
}
```

## Integration

Works seamlessly with other skills:
- **release-orchestrator**: Orchestrates instrumentation as part of quality setup
- **cicd-generator**: Ensures instrumented code is tested in CI/CD
- **performance-profiler**: Uses traces/metrics added by instrumenter

## Usage Example

```python
from skills.integration import SkillInvoker, SkillLoader, SkillRegistry

registry = SkillRegistry('skills')
loader = SkillLoader(registry)
invoker = SkillInvoker(loader)

# 1. Analyze existing codebase
analysis = invoker.invoke('code-instrumenter', 'analyze_codebase', {
    'project_path': './my-api',
    'language': 'python',
    'framework': 'fastapi',
    'detect_endpoints': True,
    'detect_database': True
})

print(f"Found {len(analysis.data['entry_points'])} entry points")
print(f"Instrumentation needed: {analysis.data['instrumentation_needed']}")

# 2. Add OpenTelemetry tracing
tracing = invoker.invoke('code-instrumenter', 'add_tracing', {
    'project_path': './my-api',
    'service_name': 'my-api',
    'jaeger_endpoint': 'http://localhost:14268',
    'sampling_rate': 0.1,
    'auto_instrument': True,
    'trace_database': True,
    'trace_http': True
})

print(f"Added tracing to {tracing.data['instrumentation_points']} points")
print(f"Modified files: {tracing.data['files_modified']}")

# 3. Add profiling
profiling = invoker.invoke('code-instrumenter', 'add_profiling', {
    'project_path': './my-api',
    'auto_detect_slow': True,
    'profile_memory': True,
    'profile_cpu': True
})

print(f"Profiled {len(profiling.data['profiled_functions'])} functions")

# 4. Add metrics
metrics = invoker.invoke('code-instrumenter', 'add_metrics', {
    'project_path': './my-api',
    'framework': 'fastapi',
    'track_requests': True,
    'track_latency': True,
    'track_errors': True
})

print(f"Metrics available at {metrics.data['metrics_endpoint']}")
```

## Supported Frameworks

### Python (Current)
- ✅ FastAPI
- ✅ Flask
- ✅ Django
- ✅ SQLAlchemy
- ✅ Redis
- ✅ HTTP clients (requests, httpx)

### JavaScript (Planned)
- 🔜 Express
- 🔜 Next.js
- 🔜 NestJS

### Java (Planned)
- 🔜 Spring Boot
- 🔜 Micronaut

## Best Practices

### Safety First
- ✅ Backup code before instrumentation
- ✅ Run tests after instrumentation
- ✅ Review generated code
- ✅ Use version control

### Performance
- ✅ Configure appropriate sampling rates (1-10% for production)
- ✅ Use async exporters to avoid blocking
- ✅ Monitor instrumentation overhead (<5%)
- ✅ Disable profiling in production by default

### Maintenance
- ✅ Keep instrumentation dependencies up to date
- ✅ Review and update manual spans
- ✅ Monitor trace volume and costs
- ✅ Regularly validate health checks

## Configuration

### Environment Variables
```bash
# Tracing
OTEL_SERVICE_NAME=my-service
OTEL_EXPORTER_JAEGER_ENDPOINT=http://jaeger:14268
OTEL_TRACES_SAMPLER=parentbased_traceidratio
OTEL_TRACES_SAMPLER_ARG=0.1

# Profiling
PROFILING_ENABLED=true
PROFILING_OUTPUT_DIR=./profiles
PROFILING_AUTO_DETECT=true

# Metrics
METRICS_ENABLED=true
METRICS_PORT=9090
```

## Troubleshooting

### Issue: Auto-instrumentation not working

**Check:**
1. Framework properly detected?
2. Dependencies installed?
3. Initialization code executed?

**Solution:**
```python
# Verify framework detection
analysis = invoker.invoke('code-instrumenter', 'analyze_codebase', {...})
print(analysis.data['framework'])

# Re-run with verbose logging
os.environ['OTEL_LOG_LEVEL'] = 'debug'
```

### Issue: High overhead from tracing

**Solution:**
- Reduce sampling rate: `sampling_rate=0.01` (1%)
- Use tail-based sampling
- Disable tracing for health check endpoints

## Performance

### Instrumentation Overhead
- Auto-instrumentation: <2% overhead
- Manual spans: <1% per span
- Metrics collection: <1% overhead
- Total typical overhead: 2-5%

### Code Generation Speed
- Analyze codebase: <5 seconds for medium project
- Add tracing: <30 seconds
- Add profiling: <10 seconds
- Add metrics: <10 seconds
- Total: <60 seconds for complete instrumentation

---

**Version:** 0.1.0
**Status:** Planned - To Be Implemented
**Dependencies:** None (standalone)
**Estimated Implementation:** 3-4 days
