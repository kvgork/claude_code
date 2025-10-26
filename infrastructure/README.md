# Release Quality Assessment Infrastructure

This directory contains the infrastructure configuration for the Release Quality Assessment system.

## Components

### Core Services

1. **Jaeger** (Port 16686)
   - Distributed tracing visualization
   - Trace collection and storage
   - Performance analysis
   - Access: http://localhost:16686

2. **TimescaleDB** (Port 5432)
   - Time-series database for metrics
   - Release quality history
   - Benchmark results storage
   - Database: `release_quality`
   - User: `rq_user`
   - Password: `rq_password`

3. **Redis** (Port 6379)
   - Caching layer
   - Job queue for async tasks
   - Session storage
   - Password: `rq_redis_password`

### Optional Services

4. **Prometheus** (Port 9090)
   - Metrics collection
   - Time-series monitoring
   - Alerting (when configured)
   - Access: http://localhost:9090

5. **Grafana** (Port 3000)
   - Visualization dashboards
   - Quality trend analysis
   - Performance monitoring
   - Access: http://localhost:3000
   - Default credentials: admin/admin

## Quick Start

### Start All Services

```bash
# Start all services
docker-compose up -d

# Check status
docker-compose ps

# View logs
docker-compose logs -f
```

### Start Minimal Stack (Core Only)

```bash
# Start only core services (Jaeger, TimescaleDB, Redis)
docker-compose up -d jaeger timescaledb redis
```

### Access Services

- **Jaeger UI**: http://localhost:16686
- **Grafana**: http://localhost:3000 (admin/admin)
- **Prometheus**: http://localhost:9090
- **TimescaleDB**: localhost:5432 (rq_user/rq_password)
- **Redis**: localhost:6379 (password: rq_redis_password)

## Database Schema

The TimescaleDB database includes the following tables:

### Core Tables

- **releases**: Release quality assessment records
- **quality_dimensions**: Individual dimension scores per release
- **benchmarks** (hypertable): Performance benchmark time-series data
- **environment_snapshots**: Environment snapshots for reproducibility
- **skill_executions** (hypertable): Skill execution time-series data
- **recommendations**: Quality improvement recommendations
- **warnings**: Quality warnings and issues

### Views

- **latest_release_quality**: Most recent release quality data
- **performance_regressions**: Detected performance regressions
- **quality_trends** (materialized): Daily quality trends
- **benchmark_stats_hourly** (continuous aggregate): Hourly benchmark statistics

### Sample Queries

```sql
-- Connect to database
psql -h localhost -U rq_user -d release_quality

-- View latest release quality
SELECT * FROM latest_release_quality;

-- View quality trend over last 30 days
SELECT * FROM quality_trends WHERE day >= NOW() - INTERVAL '30 days';

-- View performance regressions
SELECT * FROM performance_regressions LIMIT 10;

-- View benchmark history for specific release
SELECT
    time,
    benchmark_name,
    avg_time_ms,
    throughput_ops_sec
FROM benchmarks
WHERE release_version = 'v1.0.0'
ORDER BY time DESC;

-- Compare two releases
SELECT
    b1.benchmark_name,
    b1.avg_time_ms AS v1_0_0_ms,
    b2.avg_time_ms AS v1_1_0_ms,
    ((b2.avg_time_ms - b1.avg_time_ms) / b1.avg_time_ms * 100) AS change_percent
FROM benchmarks b1
JOIN benchmarks b2 ON b1.benchmark_name = b2.benchmark_name
WHERE b1.release_version = 'v1.0.0'
    AND b2.release_version = 'v1.1.0'
    AND b1.time = (SELECT MAX(time) FROM benchmarks WHERE release_version = 'v1.0.0' AND benchmark_name = b1.benchmark_name)
    AND b2.time = (SELECT MAX(time) FROM benchmarks WHERE release_version = 'v1.1.0' AND benchmark_name = b2.benchmark_name);
```

## Data Persistence

All data is persisted in Docker volumes:

- `timescaledb-data`: Database files
- `redis-data`: Redis persistence
- `prometheus-data`: Prometheus metrics
- `grafana-data`: Grafana dashboards and settings

### Backup Data

```bash
# Backup TimescaleDB
docker-compose exec timescaledb pg_dump -U rq_user release_quality > backup.sql

# Restore TimescaleDB
cat backup.sql | docker-compose exec -T timescaledb psql -U rq_user release_quality
```

### Reset Data

```bash
# WARNING: This will delete all data
docker-compose down -v

# Restart with fresh databases
docker-compose up -d
```

## Configuration

### Environment Variables

Create a `.env` file in the root directory to customize:

```bash
# TimescaleDB
POSTGRES_DB=release_quality
POSTGRES_USER=rq_user
POSTGRES_PASSWORD=rq_password

# Redis
REDIS_PASSWORD=rq_redis_password

# Grafana
GRAFANA_ADMIN_USER=admin
GRAFANA_ADMIN_PASSWORD=admin

# API (when implemented)
API_PORT=8000
API_LOG_LEVEL=info
```

### Prometheus Scrape Configuration

Edit `infrastructure/prometheus/prometheus.yml` to add custom scrape targets:

```yaml
scrape_configs:
  - job_name: 'my-custom-metrics'
    static_configs:
      - targets: ['my-app:8080']
```

### Grafana Dashboards

Place custom dashboards in `infrastructure/grafana/provisioning/dashboards/`:

```bash
# Dashboard JSON format
{
  "dashboard": {
    "title": "My Custom Dashboard",
    "panels": [...]
  }
}
```

## Monitoring

### Health Checks

All services include health checks:

```bash
# Check all service health
docker-compose ps

# Check individual service
docker-compose exec jaeger wget --spider -q http://localhost:14269
docker-compose exec timescaledb pg_isready -U rq_user
docker-compose exec redis redis-cli ping
```

### Resource Usage

```bash
# View resource usage
docker stats

# Limit resources (in docker-compose.yml)
services:
  jaeger:
    deploy:
      resources:
        limits:
          cpus: '1.0'
          memory: 1G
```

## Integration with Skills

### Store Results in TimescaleDB

```python
import psycopg2
from datetime import datetime

# Connect to database
conn = psycopg2.connect(
    host="localhost",
    port=5432,
    database="release_quality",
    user="rq_user",
    password="rq_password"
)

# Insert release assessment
cursor = conn.cursor()
cursor.execute("""
    INSERT INTO releases (version, overall_quality_score, grade, passed_quality_gate)
    VALUES (%s, %s, %s, %s)
    RETURNING id
""", ("v1.0.0", 85.0, "B+", True))

release_id = cursor.fetchone()[0]

# Insert benchmarks
cursor.execute("""
    INSERT INTO benchmarks (
        time, release_version, benchmark_name,
        avg_time_ms, throughput_ops_sec, iterations
    ) VALUES (%s, %s, %s, %s, %s, %s)
""", (
    datetime.now(),
    "v1.0.0",
    "api_latency",
    15.5,
    64.5,
    100
))

conn.commit()
cursor.close()
conn.close()
```

### Send Traces to Jaeger

```python
from opentelemetry import trace
from opentelemetry.exporter.jaeger.thrift import JaegerExporter
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.sdk.trace.export import BatchSpanProcessor

# Configure Jaeger exporter
jaeger_exporter = JaegerExporter(
    agent_host_name="localhost",
    agent_port=6831,
)

# Set up tracing
provider = TracerProvider()
provider.add_span_processor(BatchSpanProcessor(jaeger_exporter))
trace.set_tracer_provider(provider)

# Create spans
tracer = trace.get_tracer(__name__)
with tracer.start_as_current_span("benchmark_execution"):
    # Your code here
    pass
```

## Troubleshooting

### Port Conflicts

If ports are already in use:

```bash
# Change ports in docker-compose.yml
services:
  jaeger:
    ports:
      - "16687:16686"  # Use different host port
```

### TimescaleDB Connection Issues

```bash
# Check TimescaleDB logs
docker-compose logs timescaledb

# Connect to database manually
docker-compose exec timescaledb psql -U rq_user -d release_quality

# Verify extension
SELECT * FROM pg_extension WHERE extname = 'timescaledb';
```

### Out of Memory

```bash
# Increase Docker Desktop memory limit
# Or add memory limits to services in docker-compose.yml

services:
  timescaledb:
    mem_limit: 2g
```

### Data Retention

Hypertables have automatic retention policies (90 days default):

```sql
-- Modify retention policy
SELECT remove_retention_policy('benchmarks');
SELECT add_retention_policy('benchmarks', INTERVAL '180 days');

-- Disable retention
SELECT remove_retention_policy('benchmarks');
```

## Production Deployment

### Security

1. **Change Default Passwords**:
   ```bash
   # Generate secure passwords
   openssl rand -base64 32
   ```

2. **Use External Secrets**:
   ```yaml
   services:
     timescaledb:
       environment:
         - POSTGRES_PASSWORD_FILE=/run/secrets/db_password
       secrets:
         - db_password
   ```

3. **Enable TLS/SSL**:
   - Configure PostgreSQL SSL
   - Use HTTPS for Grafana
   - Enable Jaeger TLS

4. **Network Isolation**:
   ```yaml
   networks:
     frontend:
       driver: bridge
     backend:
       driver: bridge
       internal: true  # No external access
   ```

### Scaling

```yaml
# Scale Redis for high availability
services:
  redis:
    deploy:
      replicas: 3

# Use external PostgreSQL cluster
services:
  timescaledb:
    image: timescale/timescaledb-ha:latest
```

### Monitoring

- Set up Prometheus alerts for critical metrics
- Configure Grafana notifications (Slack, email, PagerDuty)
- Enable audit logging for TimescaleDB
- Monitor Jaeger trace collection rates

## Maintenance

### Regular Tasks

```bash
# Weekly: Refresh materialized views
docker-compose exec timescaledb psql -U rq_user -d release_quality -c "REFRESH MATERIALIZED VIEW quality_trends;"

# Monthly: Vacuum database
docker-compose exec timescaledb psql -U rq_user -d release_quality -c "VACUUM ANALYZE;"

# Quarterly: Backup data
docker-compose exec timescaledb pg_dump -U rq_user release_quality | gzip > backup_$(date +%Y%m%d).sql.gz
```

### Upgrades

```bash
# Pull latest images
docker-compose pull

# Recreate containers with new images
docker-compose up -d --force-recreate

# Verify versions
docker-compose exec timescaledb psql -U rq_user -c "SELECT version();"
docker-compose exec jaeger wget -qO- http://localhost:14269/version
```

## Support

- **Jaeger**: https://www.jaegertracing.io/docs/
- **TimescaleDB**: https://docs.timescale.com/
- **Prometheus**: https://prometheus.io/docs/
- **Grafana**: https://grafana.com/docs/
- **Redis**: https://redis.io/documentation

## License

Infrastructure configuration is part of the Release Quality Assessment System.
