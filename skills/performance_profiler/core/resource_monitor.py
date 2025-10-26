"""
Resource Usage Monitoring Module

Monitor CPU, memory, disk, and network usage during operations.
"""

import time
import threading
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from datetime import datetime
import logging

logger = logging.getLogger(__name__)

# Try to import psutil, but make it optional
try:
    import psutil
    PSUTIL_AVAILABLE = True
except ImportError:
    PSUTIL_AVAILABLE = False
    logger.warning("psutil not available, resource monitoring will be limited")


@dataclass
class ResourceSnapshot:
    """Snapshot of resource usage at a point in time."""
    timestamp: float
    cpu_percent: float
    memory_mb: float
    memory_percent: float
    disk_read_mb: float = 0.0
    disk_write_mb: float = 0.0
    network_sent_mb: float = 0.0
    network_recv_mb: float = 0.0

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return {
            'timestamp': self.timestamp,
            'cpu_percent': self.cpu_percent,
            'memory_mb': self.memory_mb,
            'memory_percent': self.memory_percent,
            'disk_read_mb': self.disk_read_mb,
            'disk_write_mb': self.disk_write_mb,
            'network_sent_mb': self.network_sent_mb,
            'network_recv_mb': self.network_recv_mb
        }


@dataclass
class ResourceUsageResult:
    """Result from resource monitoring."""
    duration_seconds: float
    samples_collected: int
    cpu_usage: Dict[str, float] = field(default_factory=dict)
    memory_usage: Dict[str, float] = field(default_factory=dict)
    disk_io: Dict[str, float] = field(default_factory=dict)
    network_io: Dict[str, float] = field(default_factory=dict)
    snapshots: List[ResourceSnapshot] = field(default_factory=list)

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return {
            'duration_seconds': self.duration_seconds,
            'samples_collected': self.samples_collected,
            'cpu_usage': self.cpu_usage,
            'memory_usage': self.memory_usage,
            'disk_io': self.disk_io,
            'network_io': self.network_io,
            'snapshots': [snap.to_dict() for snap in self.snapshots]
        }


class ResourceMonitor:
    """
    Monitor system resource usage over time.
    """

    def __init__(self):
        """Initialize resource monitor."""
        self.monitoring = False
        self.monitor_thread: Optional[threading.Thread] = None
        self.snapshots: List[ResourceSnapshot] = []
        self.start_disk_io: Optional[Dict] = None
        self.start_network_io: Optional[Dict] = None

        if not PSUTIL_AVAILABLE:
            logger.warning(
                "psutil not available. Install with: pip install psutil"
            )

    def start_monitoring(
        self,
        sampling_interval_ms: int = 100,
        track_cpu: bool = True,
        track_memory: bool = True,
        track_disk: bool = True,
        track_network: bool = True
    ) -> None:
        """
        Start background resource monitoring.

        Args:
            sampling_interval_ms: Sampling interval in milliseconds
            track_cpu: Track CPU usage
            track_memory: Track memory usage
            track_disk: Track disk I/O
            track_network: Track network I/O
        """
        if self.monitoring:
            logger.warning("Monitoring already started")
            return

        if not PSUTIL_AVAILABLE:
            logger.error("Cannot start monitoring without psutil")
            return

        self.monitoring = True
        self.snapshots = []

        # Capture baseline I/O
        if track_disk:
            self.start_disk_io = psutil.disk_io_counters()._asdict()
        if track_network:
            self.start_network_io = psutil.net_io_counters()._asdict()

        # Start monitoring thread
        self.monitor_thread = threading.Thread(
            target=self._monitor_loop,
            args=(
                sampling_interval_ms / 1000,
                track_cpu,
                track_memory,
                track_disk,
                track_network
            ),
            daemon=True
        )
        self.monitor_thread.start()

        logger.info(
            f"Resource monitoring started (interval: {sampling_interval_ms}ms)"
        )

    def stop_monitoring(self) -> ResourceUsageResult:
        """
        Stop monitoring and return results.

        Returns:
            ResourceUsageResult with collected data
        """
        if not self.monitoring:
            logger.warning("Monitoring not started")
            return self._empty_result()

        self.monitoring = False

        # Wait for monitoring thread to finish
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2.0)

        logger.info(f"Resource monitoring stopped, collected {len(self.snapshots)} samples")

        return self._calculate_results()

    def monitor_operation(
        self,
        operation_func,
        *args,
        sampling_interval_ms: int = 100,
        **kwargs
    ) -> tuple[Any, ResourceUsageResult]:
        """
        Monitor resource usage during an operation.

        Args:
            operation_func: Function to monitor
            *args: Function arguments
            sampling_interval_ms: Sampling interval
            **kwargs: Function keyword arguments

        Returns:
            Tuple of (operation result, resource usage result)
        """
        self.start_monitoring(sampling_interval_ms=sampling_interval_ms)

        try:
            result = operation_func(*args, **kwargs)
        finally:
            usage_result = self.stop_monitoring()

        return result, usage_result

    def _monitor_loop(
        self,
        interval: float,
        track_cpu: bool,
        track_memory: bool,
        track_disk: bool,
        track_network: bool
    ) -> None:
        """Background monitoring loop."""
        while self.monitoring:
            snapshot = self._capture_snapshot(
                track_cpu, track_memory, track_disk, track_network
            )
            self.snapshots.append(snapshot)
            time.sleep(interval)

    def _capture_snapshot(
        self,
        track_cpu: bool,
        track_memory: bool,
        track_disk: bool,
        track_network: bool
    ) -> ResourceSnapshot:
        """Capture a single resource snapshot."""
        snapshot = ResourceSnapshot(
            timestamp=time.time(),
            cpu_percent=0.0,
            memory_mb=0.0,
            memory_percent=0.0
        )

        if not PSUTIL_AVAILABLE:
            return snapshot

        try:
            if track_cpu:
                snapshot.cpu_percent = psutil.cpu_percent(interval=0.01)

            if track_memory:
                mem = psutil.virtual_memory()
                snapshot.memory_mb = mem.used / (1024 * 1024)
                snapshot.memory_percent = mem.percent

            if track_disk and self.start_disk_io:
                disk_io = psutil.disk_io_counters()._asdict()
                snapshot.disk_read_mb = (
                    (disk_io['read_bytes'] - self.start_disk_io['read_bytes'])
                    / (1024 * 1024)
                )
                snapshot.disk_write_mb = (
                    (disk_io['write_bytes'] - self.start_disk_io['write_bytes'])
                    / (1024 * 1024)
                )

            if track_network and self.start_network_io:
                net_io = psutil.net_io_counters()._asdict()
                snapshot.network_sent_mb = (
                    (net_io['bytes_sent'] - self.start_network_io['bytes_sent'])
                    / (1024 * 1024)
                )
                snapshot.network_recv_mb = (
                    (net_io['bytes_recv'] - self.start_network_io['bytes_recv'])
                    / (1024 * 1024)
                )

        except Exception as e:
            logger.error(f"Error capturing snapshot: {e}")

        return snapshot

    def _calculate_results(self) -> ResourceUsageResult:
        """Calculate results from snapshots."""
        if not self.snapshots:
            return self._empty_result()

        duration = (
            self.snapshots[-1].timestamp - self.snapshots[0].timestamp
            if len(self.snapshots) > 1 else 0
        )

        # Extract metrics
        cpu_values = [s.cpu_percent for s in self.snapshots]
        memory_values = [s.memory_mb for s in self.snapshots]

        result = ResourceUsageResult(
            duration_seconds=duration,
            samples_collected=len(self.snapshots),
            cpu_usage={
                'avg_percent': sum(cpu_values) / len(cpu_values) if cpu_values else 0,
                'peak_percent': max(cpu_values) if cpu_values else 0,
                'min_percent': min(cpu_values) if cpu_values else 0,
                'samples': cpu_values
            },
            memory_usage={
                'avg_mb': sum(memory_values) / len(memory_values) if memory_values else 0,
                'peak_mb': max(memory_values) if memory_values else 0,
                'min_mb': min(memory_values) if memory_values else 0,
                'samples': memory_values
            },
            snapshots=self.snapshots
        )

        # Add disk I/O if available
        if self.snapshots[-1].disk_read_mb > 0:
            result.disk_io = {
                'read_mb': self.snapshots[-1].disk_read_mb,
                'write_mb': self.snapshots[-1].disk_write_mb,
                'total_mb': self.snapshots[-1].disk_read_mb + self.snapshots[-1].disk_write_mb
            }

        # Add network I/O if available
        if self.snapshots[-1].network_sent_mb > 0:
            result.network_io = {
                'sent_mb': self.snapshots[-1].network_sent_mb,
                'recv_mb': self.snapshots[-1].network_recv_mb,
                'total_mb': self.snapshots[-1].network_sent_mb + self.snapshots[-1].network_recv_mb
            }

        return result

    def _empty_result(self) -> ResourceUsageResult:
        """Return empty result."""
        return ResourceUsageResult(
            duration_seconds=0,
            samples_collected=0,
            cpu_usage={},
            memory_usage={},
            disk_io={},
            network_io={}
        )


# Export public API
__all__ = [
    'ResourceMonitor',
    'ResourceSnapshot',
    'ResourceUsageResult',
    'PSUTIL_AVAILABLE'
]
