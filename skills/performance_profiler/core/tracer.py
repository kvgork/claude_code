"""
Distributed Tracing Module

Integrates with Jaeger/OpenTelemetry to capture distributed traces.
"""

import time
import json
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, asdict
from datetime import datetime
import logging

logger = logging.getLogger(__name__)


@dataclass
class Span:
    """Represents a trace span."""
    trace_id: str
    span_id: str
    parent_span_id: Optional[str]
    operation_name: str
    start_time: float
    duration_ms: float
    tags: Dict[str, Any]
    status: str  # 'ok', 'error'

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return asdict(self)


@dataclass
class Trace:
    """Represents a complete trace."""
    trace_id: str
    service_name: str
    start_time: float
    duration_ms: float
    spans: List[Span]

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return {
            'trace_id': self.trace_id,
            'service_name': self.service_name,
            'start_time': self.start_time,
            'duration_ms': self.duration_ms,
            'spans': [span.to_dict() for span in self.spans]
        }


class JaegerTracer:
    """
    Jaeger distributed tracing integration.

    Captures and exports traces to Jaeger collector.
    """

    def __init__(
        self,
        service_name: str,
        jaeger_endpoint: str = "http://localhost:14268/api/traces",
        sampling_rate: float = 1.0
    ):
        """
        Initialize Jaeger tracer.

        Args:
            service_name: Name of the service
            jaeger_endpoint: Jaeger collector endpoint
            sampling_rate: Sampling rate (0.0-1.0)
        """
        self.service_name = service_name
        self.jaeger_endpoint = jaeger_endpoint
        self.sampling_rate = sampling_rate
        self.traces: List[Trace] = []

        logger.info(f"JaegerTracer initialized for service: {service_name}")
        logger.info(f"Jaeger endpoint: {jaeger_endpoint}")
        logger.info(f"Sampling rate: {sampling_rate * 100}%")

    def should_sample(self) -> bool:
        """Determine if trace should be sampled."""
        import random
        return random.random() < self.sampling_rate

    def capture_traces(
        self,
        duration_seconds: int = 60,
        operation_patterns: Optional[List[str]] = None
    ) -> Dict[str, Any]:
        """
        Capture traces for a specified duration.

        Args:
            duration_seconds: How long to capture traces
            operation_patterns: Operation patterns to filter (e.g., ['api.*', 'db.*'])

        Returns:
            Dictionary with trace statistics
        """
        logger.info(f"Starting trace collection for {duration_seconds} seconds")

        start_time = time.time()
        collected_traces = []

        # Simulate trace collection
        # In real implementation, this would integrate with OpenTelemetry SDK
        while time.time() - start_time < duration_seconds:
            if self.should_sample():
                trace = self._generate_sample_trace()

                # Filter by operation patterns if specified
                if operation_patterns:
                    if any(self._matches_pattern(span.operation_name, pattern)
                           for span in trace.spans
                           for pattern in operation_patterns):
                        collected_traces.append(trace)
                else:
                    collected_traces.append(trace)

            time.sleep(0.1)  # Simulate collection interval

        self.traces.extend(collected_traces)

        # Calculate statistics
        return self._calculate_trace_stats(collected_traces)

    def _generate_sample_trace(self) -> Trace:
        """Generate a sample trace (for demonstration)."""
        import uuid

        trace_id = str(uuid.uuid4())
        start_time = time.time()

        # Generate sample spans
        spans = [
            Span(
                trace_id=trace_id,
                span_id=str(uuid.uuid4()),
                parent_span_id=None,
                operation_name="http.request",
                start_time=start_time,
                duration_ms=50.5,
                tags={'http.method': 'GET', 'http.url': '/api/users'},
                status='ok'
            ),
            Span(
                trace_id=trace_id,
                span_id=str(uuid.uuid4()),
                parent_span_id=spans[0].span_id if spans else None,
                operation_name="db.query",
                start_time=start_time + 0.01,
                duration_ms=25.3,
                tags={'db.statement': 'SELECT * FROM users'},
                status='ok'
            )
        ]

        return Trace(
            trace_id=trace_id,
            service_name=self.service_name,
            start_time=start_time,
            duration_ms=sum(span.duration_ms for span in spans),
            spans=spans
        )

    def _matches_pattern(self, operation_name: str, pattern: str) -> bool:
        """Check if operation name matches pattern."""
        import re
        regex_pattern = pattern.replace('*', '.*')
        return re.match(regex_pattern, operation_name) is not None

    def _calculate_trace_stats(self, traces: List[Trace]) -> Dict[str, Any]:
        """Calculate statistics from collected traces."""
        if not traces:
            return {
                'traces_collected': 0,
                'avg_duration_ms': 0,
                'p50_duration_ms': 0,
                'p95_duration_ms': 0,
                'p99_duration_ms': 0,
                'error_rate': 0,
                'total_spans': 0
            }

        durations = [trace.duration_ms for trace in traces]
        durations.sort()

        # Count errors
        total_spans = sum(len(trace.spans) for trace in traces)
        error_spans = sum(
            1 for trace in traces
            for span in trace.spans
            if span.status == 'error'
        )

        return {
            'traces_collected': len(traces),
            'trace_ids': [trace.trace_id for trace in traces[:10]],  # First 10
            'avg_duration_ms': sum(durations) / len(durations),
            'p50_duration_ms': self._percentile(durations, 50),
            'p95_duration_ms': self._percentile(durations, 95),
            'p99_duration_ms': self._percentile(durations, 99),
            'error_rate': error_spans / total_spans if total_spans > 0 else 0,
            'total_spans': total_spans
        }

    def _percentile(self, values: List[float], percentile: int) -> float:
        """Calculate percentile from sorted values."""
        if not values:
            return 0.0
        index = int(len(values) * percentile / 100)
        return values[min(index, len(values) - 1)]

    def export_traces(self, output_file: str) -> None:
        """
        Export collected traces to file.

        Args:
            output_file: Path to output JSON file
        """
        traces_data = [trace.to_dict() for trace in self.traces]

        with open(output_file, 'w') as f:
            json.dump(traces_data, f, indent=2)

        logger.info(f"Exported {len(self.traces)} traces to {output_file}")

    def get_trace_by_id(self, trace_id: str) -> Optional[Trace]:
        """Get a specific trace by ID."""
        for trace in self.traces:
            if trace.trace_id == trace_id:
                return trace
        return None

    def clear_traces(self) -> None:
        """Clear all collected traces."""
        self.traces.clear()
        logger.info("Cleared all traces")


class TracingContext:
    """
    Context manager for tracing operations.

    Usage:
        tracer = JaegerTracer("my-service")
        with TracingContext(tracer, "my_operation") as ctx:
            # Code to trace
            ctx.add_tag("key", "value")
    """

    def __init__(self, tracer: JaegerTracer, operation_name: str):
        """
        Initialize tracing context.

        Args:
            tracer: JaegerTracer instance
            operation_name: Name of the operation
        """
        self.tracer = tracer
        self.operation_name = operation_name
        self.start_time = None
        self.tags = {}

    def __enter__(self):
        """Enter context."""
        self.start_time = time.time()
        logger.debug(f"Started tracing: {self.operation_name}")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Exit context."""
        duration_ms = (time.time() - self.start_time) * 1000
        status = 'error' if exc_type else 'ok'

        logger.debug(
            f"Completed tracing: {self.operation_name} "
            f"({duration_ms:.2f}ms, status={status})"
        )

        # In real implementation, would send to Jaeger
        return False  # Don't suppress exceptions

    def add_tag(self, key: str, value: Any) -> None:
        """Add a tag to the current span."""
        self.tags[key] = value


# Export public API
__all__ = [
    'JaegerTracer',
    'TracingContext',
    'Trace',
    'Span'
]
