# Release Quality System - Docker Strategy

**Version:** 1.0.0
**Date:** 2025-10-26

---

## TL;DR Recommendation

**Use a HYBRID approach:**
- ✅ **Docker for Infrastructure** - Database, Jaeger, Redis, Dashboard, API
- ⚠️ **Bare Metal or Dedicated Containers for Benchmarks** - Accurate performance measurements
- ✅ **Docker for CI/CD** - Consistent environments across pipelines
- ✅ **Docker Compose for Local Development** - Easy setup and teardown

---

## Docker Benefits for Release Quality System

### 1. Infrastructure Services ✅ HIGHLY RECOMMENDED

**Services that SHOULD run in Docker:**

```yaml
# docker-compose.yml
version: '3.8'

services:
  # Time-series database for metrics
  timescaledb:
    image: timescale/timescaledb:latest-pg15
    environment:
      POSTGRES_PASSWORD: ${DB_PASSWORD}
      POSTGRES_DB: release_quality
    volumes:
      - timescale_data:/var/lib/postgresql/data
    ports:
      - "5432:5432"

  # Distributed tracing
  jaeger:
    image: jaegertracing/all-in-one:latest
    ports:
      - "16686:16686"  # UI
      - "14268:14268"  # Collector HTTP
      - "6831:6831/udp"  # Agent
    environment:
      COLLECTOR_OTLP_ENABLED: true

  # Caching layer
  redis:
    image: redis:7-alpine
    ports:
      - "6379:6379"
    volumes:
      - redis_data:/data

  # Dashboard backend
  api:
    build: ./skills/quality_dashboard/backend
    ports:
      - "8000:8000"
    environment:
      DATABASE_URL: postgresql://postgres:${DB_PASSWORD}@timescaledb:5432/release_quality
      REDIS_URL: redis://redis:6379
      JAEGER_ENDPOINT: http://jaeger:14268/api/traces
    depends_on:
      - timescaledb
      - redis
      - jaeger

  # Dashboard frontend
  frontend:
    build: ./skills/quality_dashboard/frontend
    ports:
      - "3000:3000"
    environment:
      REACT_APP_API_URL: http://localhost:8000
      REACT_APP_JAEGER_URL: http://localhost:16686
    depends_on:
      - api

volumes:
  timescale_data:
  redis_data:
```

**Benefits:**
- ✅ **One Command Setup:** `docker-compose up -d`
- ✅ **Consistent Environment:** Same DB/cache/tracing setup everywhere
- ✅ **Easy Cleanup:** `docker-compose down -v`
- ✅ **Version Control:** Lock infrastructure versions
- ✅ **Local Development:** Matches production exactly
- ✅ **Resource Isolation:** Services don't interfere

### 2. Performance Benchmarking ⚠️ USE WITH CAUTION

**Challenge:** Docker adds overhead that can skew benchmark results

**Docker Performance Overhead:**
- CPU: ~1-3% overhead
- Memory: ~50-100MB base overhead
- Disk I/O: ~5-10% overhead (depending on driver)
- Network: ~2-5% overhead

**Solutions:**

#### Option A: Bare Metal Benchmarks (RECOMMENDED for accuracy)
```python
# Run benchmarks on host machine
result = invoker.invoke('performance-profiler', 'benchmark_release', {
    'benchmark_suite': 'benchmarks/api_tests.py',
    'run_mode': 'bare_metal',  # No containerization
    'iterations': 100
})
```

**Pros:**
- Most accurate performance measurements
- Direct hardware access
- No container overhead

**Cons:**
- Environment can vary between runs
- Harder to isolate from other processes
- Manual environment setup

#### Option B: Dedicated Benchmark Container (RECOMMENDED for consistency)
```yaml
services:
  benchmark-runner:
    build: ./skills/performance_profiler
    privileged: true  # For accurate profiling
    cpus: "4.0"       # Fixed CPU allocation
    mem_limit: 8g     # Fixed memory
    volumes:
      - ./benchmarks:/benchmarks
      - ./results:/results
      - /sys:/sys:ro    # Access to system metrics
    environment:
      BENCHMARK_MODE: isolated
      CAPTURE_OVERHEAD: true  # Measure Docker overhead
```

**Configuration for Accurate Benchmarks:**
```bash
# Run benchmark container with performance settings
docker run \
  --cpus="4.0" \              # Fixed CPU count
  --memory="8g" \             # Fixed memory
  --memory-swap="8g" \        # No swap
  --cpu-shares=1024 \         # High priority
  --pids-limit=4096 \         # Prevent fork bombs
  --ulimit nofile=65536 \     # High file descriptors
  --network=host \            # Reduce network overhead
  benchmark-runner
```

**Pros:**
- Consistent environment every run
- Reproducible results
- Can measure and subtract Docker overhead
- Easier to deploy in CI/CD

**Cons:**
- Still has ~1-3% overhead
- Requires careful resource configuration
- Need to account for overhead in results

#### Option C: Hybrid Approach (BEST OF BOTH WORLDS) ✅ RECOMMENDED
```python
# Release-orchestrator decides based on context
def benchmark_release(params):
    if params.get('environment') == 'ci':
        # CI: Use Docker for consistency
        run_in_container(
            image='benchmark-runner:latest',
            resources={'cpus': 4, 'memory': '8g'}
        )
    elif params.get('environment') == 'production':
        # Production: Use bare metal for accuracy
        run_on_host()
    else:
        # Local dev: Use Docker for convenience
        run_in_container(image='benchmark-runner:latest')
```

---

## Docker Strategy by Component

### Infrastructure Services: 🐳 DOCKER (100% Recommended)

**All these should ALWAYS run in Docker:**

| Service | Image | Why Docker? |
|---------|-------|-------------|
| TimescaleDB | `timescale/timescaledb:latest-pg15` | Consistent DB version, easy backups |
| Jaeger | `jaegertracing/all-in-one:latest` | Complex deployment, Docker simplifies |
| Redis | `redis:7-alpine` | Lightweight, perfect for containers |
| Dashboard API | Custom build | Easy updates, horizontal scaling |
| Dashboard UI | Custom build | Static files, nginx container |

### Benchmarking: ⚖️ HYBRID (Context-dependent)

**Decision Matrix:**

| Environment | Approach | Reason |
|-------------|----------|--------|
| **Local Dev** | Docker | Convenience, quick iteration |
| **CI/CD** | Docker | Consistency, reproducibility |
| **Production Monitoring** | Bare Metal | Accuracy, real performance |
| **Release Comparison** | Docker (same config) | Fair apples-to-apples comparison |

### Skills Execution: 🔄 FLEXIBLE

**Option 1: Run skills on host, connect to Docker infrastructure**
```python
# Skills run on host
from skills.integration import SkillInvoker

invoker = SkillInvoker(loader)

# But connect to Dockerized services
result = invoker.invoke('performance-profiler', 'capture_traces', {
    'jaeger_endpoint': 'http://localhost:14268',  # Docker Jaeger
    'service_name': 'my-app'
})
```

**Option 2: Containerize skills for isolation**
```dockerfile
# Dockerfile for skills
FROM python:3.11-slim

WORKDIR /app
COPY skills/ /app/skills/
COPY requirements.txt /app/

RUN pip install -r requirements.txt

# Each skill can be run independently
CMD ["python", "-m", "skills.performance_profiler.demo"]
```

---

## Complete Docker Compose Setup

```yaml
# docker-compose.yml - Full Release Quality System
version: '3.8'

services:
  # === Infrastructure ===

  timescaledb:
    image: timescale/timescaledb:2.13.0-pg15
    container_name: rq_timescaledb
    restart: unless-stopped
    environment:
      POSTGRES_USER: release_quality
      POSTGRES_PASSWORD: ${DB_PASSWORD:-changeme}
      POSTGRES_DB: release_quality
    volumes:
      - timescale_data:/var/lib/postgresql/data
      - ./database/init.sql:/docker-entrypoint-initdb.d/init.sql
    ports:
      - "5432:5432"
    healthcheck:
      test: ["CMD-SHELL", "pg_isready -U release_quality"]
      interval: 10s
      timeout: 5s
      retries: 5

  jaeger:
    image: jaegertracing/all-in-one:1.51
    container_name: rq_jaeger
    restart: unless-stopped
    environment:
      COLLECTOR_OTLP_ENABLED: true
      SPAN_STORAGE_TYPE: badger
      BADGER_EPHEMERAL: false
      BADGER_DIRECTORY_VALUE: /badger/data
      BADGER_DIRECTORY_KEY: /badger/key
    volumes:
      - jaeger_data:/badger
    ports:
      - "16686:16686"  # Jaeger UI
      - "14268:14268"  # Jaeger collector HTTP
      - "14250:14250"  # Jaeger collector gRPC
      - "6831:6831/udp"  # Jaeger agent
      - "4317:4317"    # OTLP gRPC
      - "4318:4318"    # OTLP HTTP

  redis:
    image: redis:7.2-alpine
    container_name: rq_redis
    restart: unless-stopped
    command: redis-server --appendonly yes --maxmemory 2gb --maxmemory-policy allkeys-lru
    volumes:
      - redis_data:/data
    ports:
      - "6379:6379"
    healthcheck:
      test: ["CMD", "redis-cli", "ping"]
      interval: 10s
      timeout: 3s
      retries: 5

  # === Application ===

  api:
    build:
      context: .
      dockerfile: skills/quality_dashboard/backend/Dockerfile
    container_name: rq_api
    restart: unless-stopped
    environment:
      DATABASE_URL: postgresql://release_quality:${DB_PASSWORD:-changeme}@timescaledb:5432/release_quality
      REDIS_URL: redis://redis:6379/0
      JAEGER_ENDPOINT: http://jaeger:14268/api/traces
      OTLP_ENDPOINT: http://jaeger:4318
      LOG_LEVEL: ${LOG_LEVEL:-INFO}
    volumes:
      - ./skills:/app/skills
      - ./snapshots:/app/snapshots
      - ./reports:/app/reports
    ports:
      - "8000:8000"
    depends_on:
      timescaledb:
        condition: service_healthy
      redis:
        condition: service_healthy
      jaeger:
        condition: service_started
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8000/health"]
      interval: 30s
      timeout: 10s
      retries: 3

  frontend:
    build:
      context: skills/quality_dashboard/frontend
      dockerfile: Dockerfile
    container_name: rq_frontend
    restart: unless-stopped
    environment:
      REACT_APP_API_URL: http://localhost:8000
      REACT_APP_JAEGER_URL: http://localhost:16686
    ports:
      - "3000:80"
    depends_on:
      - api

  # === Optional: Benchmark Runner ===

  benchmark-runner:
    build:
      context: .
      dockerfile: skills/performance_profiler/Dockerfile
    container_name: rq_benchmark
    profiles: ["benchmarks"]  # Only start with --profile benchmarks
    cpus: "4.0"
    mem_limit: 8g
    mem_reservation: 8g
    environment:
      JAEGER_ENDPOINT: http://jaeger:4318
      BENCHMARK_MODE: isolated
      PYTHONUNBUFFERED: 1
    volumes:
      - ./benchmarks:/benchmarks
      - ./results:/results
      - /sys:/sys:ro
    command: ["tail", "-f", "/dev/null"]  # Keep running for exec

  # === Optional: Grafana for Advanced Visualization ===

  grafana:
    image: grafana/grafana:10.2.2
    container_name: rq_grafana
    profiles: ["monitoring"]
    restart: unless-stopped
    environment:
      GF_SECURITY_ADMIN_PASSWORD: ${GRAFANA_PASSWORD:-admin}
      GF_INSTALL_PLUGINS: grafana-clock-panel
    volumes:
      - grafana_data:/var/lib/grafana
      - ./grafana/dashboards:/etc/grafana/provisioning/dashboards
      - ./grafana/datasources:/etc/grafana/provisioning/datasources
    ports:
      - "3001:3000"
    depends_on:
      - timescaledb

volumes:
  timescale_data:
  jaeger_data:
  redis_data:
  grafana_data:

networks:
  default:
    name: release_quality_network
```

---

## Environment-Specific Configurations

### Development Environment

```bash
# .env.development
DB_PASSWORD=dev_password
LOG_LEVEL=DEBUG
BENCHMARK_MODE=docker  # Use Docker for convenience
```

```bash
# Start everything
docker-compose up -d

# Run benchmarks in Docker
docker-compose --profile benchmarks up benchmark-runner

# View logs
docker-compose logs -f api
```

### CI/CD Environment

```yaml
# .github/workflows/release-quality.yml
jobs:
  quality-check:
    runs-on: ubuntu-latest

    services:
      timescaledb:
        image: timescale/timescaledb:2.13.0-pg15
        env:
          POSTGRES_PASSWORD: test
          POSTGRES_DB: release_quality
        options: >-
          --health-cmd pg_isready
          --health-interval 10s
          --health-timeout 5s
          --health-retries 5

      jaeger:
        image: jaegertracing/all-in-one:1.51

      redis:
        image: redis:7.2-alpine
        options: >-
          --health-cmd "redis-cli ping"
          --health-interval 10s
          --health-timeout 3s
          --health-retries 5

    steps:
      - uses: actions/checkout@v3

      - name: Run Benchmarks in Docker
        run: |
          docker run --rm \
            --cpus="2.0" \
            --memory="4g" \
            --network="host" \
            -v $(pwd):/app \
            -e JAEGER_ENDPOINT=http://localhost:14268 \
            benchmark-runner:latest \
            python -m benchmarks.run_all
```

### Production Environment

```bash
# .env.production
DB_PASSWORD=<strong-password>
LOG_LEVEL=INFO
BENCHMARK_MODE=bare_metal  # Bare metal for accuracy
```

```bash
# Use Docker for infrastructure only
docker-compose up -d timescaledb jaeger redis api frontend

# Run benchmarks on host
python -m skills.performance_profiler.benchmark --mode=production
```

---

## Handling Docker Overhead in Benchmarks

### 1. Measure Overhead

```python
# skills/performance_profiler/core/overhead_calibrator.py

def measure_docker_overhead():
    """Measure Docker container overhead vs bare metal."""

    # Run same benchmark on host and in Docker
    host_result = run_benchmark_on_host()
    docker_result = run_benchmark_in_docker()

    overhead = {
        'cpu_overhead_percent': (docker_result['cpu'] - host_result['cpu']) / host_result['cpu'] * 100,
        'memory_overhead_mb': docker_result['memory'] - host_result['memory'],
        'io_overhead_percent': (docker_result['io'] - host_result['io']) / host_result['io'] * 100,
        'latency_overhead_ms': docker_result['latency'] - host_result['latency']
    }

    return overhead
```

### 2. Adjust Results

```python
# Compensate for Docker overhead
def adjust_for_docker_overhead(result, overhead):
    if result.metadata.get('runtime') == 'docker':
        # Subtract measured overhead
        result.data['latency_ms'] -= overhead['latency_overhead_ms']
        result.data['cpu_percent'] -= overhead['cpu_overhead_percent']

    return result
```

### 3. Store Environment Context

```python
# Always record execution environment
snapshot['execution_environment'] = {
    'runtime': 'docker',  # or 'bare_metal'
    'container_id': '...',
    'docker_version': '24.0.7',
    'resource_limits': {
        'cpus': 4.0,
        'memory_gb': 8.0
    },
    'measured_overhead': overhead_data
}
```

---

## Best Practices

### ✅ DO:

1. **Use Docker for all infrastructure services** - DB, Jaeger, Redis
2. **Set explicit resource limits** for benchmark containers
3. **Measure Docker overhead** and document it
4. **Use same Docker configuration** for all compared releases
5. **Pin Docker image versions** for reproducibility
6. **Use health checks** to ensure services are ready
7. **Volume mount results** to persist outside containers
8. **Use Docker networks** for service isolation

### ❌ DON'T:

1. **Don't ignore Docker overhead** in benchmark results
2. **Don't use dynamic resource allocation** for benchmarks
3. **Don't compare Docker vs bare metal** without adjustment
4. **Don't share benchmark containers** with other processes
5. **Don't use latest tags** for infrastructure (pin versions)
6. **Don't run production benchmarks** in Docker without overhead compensation

---

## Resource Requirements

### Minimum System Requirements

```yaml
# For running the full stack locally
Host System:
  CPU: 4 cores (8 recommended)
  RAM: 16 GB (32 GB recommended)
  Disk: 100 GB SSD
  Network: 1 Gbps

Docker Resources:
  TimescaleDB: 2 CPU, 4 GB RAM
  Jaeger: 1 CPU, 2 GB RAM
  Redis: 0.5 CPU, 2 GB RAM
  API: 1 CPU, 2 GB RAM
  Frontend: 0.5 CPU, 512 MB RAM
  Benchmark Runner: 4 CPU, 8 GB RAM (when running)
  ─────────────────────────────────
  Total: 9 CPU, 18.5 GB RAM
```

---

## Recommendation Summary

### 🎯 Recommended Docker Strategy

**Infrastructure (ALWAYS Docker):**
```yaml
✅ TimescaleDB - docker
✅ Jaeger - docker
✅ Redis - docker
✅ API - docker
✅ Frontend - docker
✅ Grafana - docker (optional)
```

**Benchmarking (CONTEXT-DEPENDENT):**
```yaml
Development: ✅ Docker (convenience)
CI/CD: ✅ Docker (consistency)
Production: ⚠️ Bare Metal (accuracy)
Comparison: ✅ Docker (same config) OR Bare Metal (both releases same)
```

**Skills Execution (FLEXIBLE):**
```yaml
Simple operations: ✅ Host (fast)
Isolated testing: ✅ Docker (clean)
CI/CD: ✅ Docker (consistent)
```

### Quick Start Commands

```bash
# Start infrastructure
docker-compose up -d

# Run a benchmark (Docker)
docker-compose --profile benchmarks run benchmark-runner \
  python -m benchmarks.api_tests

# Run a benchmark (bare metal)
python -m skills.performance_profiler.benchmark \
  --suite=benchmarks/api_tests.py

# Access services
# Dashboard: http://localhost:3000
# API Docs: http://localhost:8000/docs
# Jaeger UI: http://localhost:16686
# Grafana: http://localhost:3001
```

---

**Document Version:** 1.0.0
**Last Updated:** 2025-10-26
**Recommendation:** Hybrid approach - Docker for infrastructure, flexible for benchmarks
