-- Release Quality Assessment Database Schema
-- TimescaleDB Initialization Script

-- Enable TimescaleDB extension
CREATE EXTENSION IF NOT EXISTS timescaledb;

-- Create releases table
CREATE TABLE IF NOT EXISTS releases (
    id SERIAL PRIMARY KEY,
    version VARCHAR(255) NOT NULL UNIQUE,
    timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    overall_quality_score NUMERIC(5,2),
    grade VARCHAR(3),
    passed_quality_gate BOOLEAN,
    project_path TEXT,
    baseline_version VARCHAR(255),
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_releases_version ON releases(version);
CREATE INDEX idx_releases_timestamp ON releases(timestamp DESC);

-- Create quality_dimensions table
CREATE TABLE IF NOT EXISTS quality_dimensions (
    id SERIAL PRIMARY KEY,
    release_id INTEGER NOT NULL REFERENCES releases(id) ON DELETE CASCADE,
    dimension_name VARCHAR(100) NOT NULL,
    score NUMERIC(5,2) NOT NULL,
    weight NUMERIC(3,2) NOT NULL,
    metrics JSONB,
    issues JSONB,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_quality_dimensions_release ON quality_dimensions(release_id);
CREATE INDEX idx_quality_dimensions_name ON quality_dimensions(dimension_name);

-- Create benchmarks table (hypertable for time-series data)
CREATE TABLE IF NOT EXISTS benchmarks (
    time TIMESTAMPTZ NOT NULL,
    release_version VARCHAR(255) NOT NULL,
    benchmark_name VARCHAR(255) NOT NULL,
    avg_time_ms NUMERIC(10,3),
    min_time_ms NUMERIC(10,3),
    max_time_ms NUMERIC(10,3),
    std_dev_ms NUMERIC(10,3),
    throughput_ops_sec NUMERIC(12,2),
    memory_peak_mb NUMERIC(10,2),
    cpu_avg_percent NUMERIC(5,2),
    iterations INTEGER,
    metadata JSONB
);

-- Convert benchmarks to hypertable
SELECT create_hypertable('benchmarks', 'time', if_not_exists => TRUE);

CREATE INDEX idx_benchmarks_release ON benchmarks(release_version, time DESC);
CREATE INDEX idx_benchmarks_name ON benchmarks(benchmark_name, time DESC);

-- Create environment_snapshots table
CREATE TABLE IF NOT EXISTS environment_snapshots (
    id SERIAL PRIMARY KEY,
    release_version VARCHAR(255) NOT NULL,
    snapshot_name VARCHAR(255) NOT NULL UNIQUE,
    timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    hardware JSONB NOT NULL,
    software JSONB NOT NULL,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_env_snapshots_release ON environment_snapshots(release_version);
CREATE INDEX idx_env_snapshots_timestamp ON environment_snapshots(timestamp DESC);

-- Create skill_executions table (hypertable for time-series data)
CREATE TABLE IF NOT EXISTS skill_executions (
    time TIMESTAMPTZ NOT NULL,
    release_version VARCHAR(255) NOT NULL,
    skill_name VARCHAR(100) NOT NULL,
    operation_name VARCHAR(100) NOT NULL,
    success BOOLEAN NOT NULL,
    duration_seconds NUMERIC(10,3),
    error_message TEXT,
    metadata JSONB
);

-- Convert skill_executions to hypertable
SELECT create_hypertable('skill_executions', 'time', if_not_exists => TRUE);

CREATE INDEX idx_skill_executions_release ON skill_executions(release_version, time DESC);
CREATE INDEX idx_skill_executions_skill ON skill_executions(skill_name, time DESC);

-- Create recommendations table
CREATE TABLE IF NOT EXISTS recommendations (
    id SERIAL PRIMARY KEY,
    release_id INTEGER NOT NULL REFERENCES releases(id) ON DELETE CASCADE,
    recommendation TEXT NOT NULL,
    priority VARCHAR(20),
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_recommendations_release ON recommendations(release_id);

-- Create warnings table
CREATE TABLE IF NOT EXISTS warnings (
    id SERIAL PRIMARY KEY,
    release_id INTEGER NOT NULL REFERENCES releases(id) ON DELETE CASCADE,
    warning TEXT NOT NULL,
    severity VARCHAR(20),
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_warnings_release ON warnings(release_id);

-- Create materialized view for quality trends
CREATE MATERIALIZED VIEW IF NOT EXISTS quality_trends AS
SELECT
    date_trunc('day', timestamp) AS day,
    AVG(overall_quality_score) AS avg_score,
    MAX(overall_quality_score) AS max_score,
    MIN(overall_quality_score) AS min_score,
    COUNT(*) AS release_count,
    SUM(CASE WHEN passed_quality_gate THEN 1 ELSE 0 END) AS passed_count
FROM releases
GROUP BY date_trunc('day', timestamp)
ORDER BY day DESC;

CREATE UNIQUE INDEX idx_quality_trends_day ON quality_trends(day DESC);

-- Create continuous aggregate for hourly benchmark stats
CREATE MATERIALIZED VIEW IF NOT EXISTS benchmark_stats_hourly
WITH (timescaledb.continuous) AS
SELECT
    time_bucket('1 hour', time) AS bucket,
    release_version,
    benchmark_name,
    AVG(avg_time_ms) AS avg_time_ms,
    MIN(min_time_ms) AS min_time_ms,
    MAX(max_time_ms) AS max_time_ms,
    AVG(throughput_ops_sec) AS avg_throughput,
    COUNT(*) AS sample_count
FROM benchmarks
GROUP BY bucket, release_version, benchmark_name
WITH NO DATA;

-- Refresh policy for continuous aggregate
SELECT add_continuous_aggregate_policy('benchmark_stats_hourly',
    start_offset => INTERVAL '3 hours',
    end_offset => INTERVAL '1 hour',
    schedule_interval => INTERVAL '1 hour',
    if_not_exists => TRUE
);

-- Create retention policy for old benchmarks (keep 90 days)
SELECT add_retention_policy('benchmarks', INTERVAL '90 days', if_not_exists => TRUE);

-- Create retention policy for old skill executions (keep 90 days)
SELECT add_retention_policy('skill_executions', INTERVAL '90 days', if_not_exists => TRUE);

-- Create helpful views

-- View: Latest release quality
CREATE OR REPLACE VIEW latest_release_quality AS
SELECT
    r.version,
    r.timestamp,
    r.overall_quality_score,
    r.grade,
    r.passed_quality_gate,
    COALESCE(
        json_agg(
            json_build_object(
                'dimension', qd.dimension_name,
                'score', qd.score,
                'weight', qd.weight
            )
        ) FILTER (WHERE qd.id IS NOT NULL),
        '[]'
    ) AS dimensions
FROM releases r
LEFT JOIN quality_dimensions qd ON r.id = qd.release_id
WHERE r.timestamp = (SELECT MAX(timestamp) FROM releases)
GROUP BY r.id, r.version, r.timestamp, r.overall_quality_score, r.grade, r.passed_quality_gate;

-- View: Performance regression summary
CREATE OR REPLACE VIEW performance_regressions AS
SELECT
    b1.release_version AS current_version,
    b1.benchmark_name,
    b1.avg_time_ms AS current_avg_ms,
    b2.avg_time_ms AS baseline_avg_ms,
    ((b1.avg_time_ms - b2.avg_time_ms) / b2.avg_time_ms * 100) AS change_percent,
    (b1.avg_time_ms - b2.avg_time_ms) AS change_ms
FROM benchmarks b1
JOIN benchmarks b2 ON b1.benchmark_name = b2.benchmark_name
WHERE b1.release_version != b2.release_version
    AND b1.time > b2.time
    AND ((b1.avg_time_ms - b2.avg_time_ms) / b2.avg_time_ms) > 0.05  -- 5% regression threshold
ORDER BY change_percent DESC;

-- Grant permissions
GRANT ALL PRIVILEGES ON ALL TABLES IN SCHEMA public TO rq_user;
GRANT ALL PRIVILEGES ON ALL SEQUENCES IN SCHEMA public TO rq_user;
GRANT ALL PRIVILEGES ON ALL FUNCTIONS IN SCHEMA public TO rq_user;

-- Insert sample data (optional)
-- INSERT INTO releases (version, overall_quality_score, grade, passed_quality_gate)
-- VALUES ('v1.0.0', 100.0, 'A+', true);

COMMENT ON TABLE releases IS 'Release quality assessment records';
COMMENT ON TABLE quality_dimensions IS 'Individual quality dimension scores per release';
COMMENT ON TABLE benchmarks IS 'Performance benchmark time-series data';
COMMENT ON TABLE environment_snapshots IS 'Environment snapshots for reproducibility';
COMMENT ON TABLE skill_executions IS 'Skill execution time-series data';
COMMENT ON TABLE recommendations IS 'Quality improvement recommendations per release';
COMMENT ON TABLE warnings IS 'Quality warnings per release';
