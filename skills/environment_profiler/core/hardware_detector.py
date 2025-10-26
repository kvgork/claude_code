"""
Hardware Detection Module

Detect and profile hardware specifications (CPU, memory, disk, GPU, etc.).
"""

import platform
import os
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, asdict
import logging

logger = logging.getLogger(__name__)

# Try to import psutil for detailed hardware info
try:
    import psutil
    PSUTIL_AVAILABLE = True
except ImportError:
    PSUTIL_AVAILABLE = False
    logger.warning("psutil not available, hardware detection will be limited")


@dataclass
class CPUInfo:
    """CPU information."""
    model_name: str
    architecture: str
    physical_cores: int
    logical_cores: int
    max_frequency_mhz: float
    min_frequency_mhz: float
    current_frequency_mhz: float
    cache_size_kb: Optional[int] = None
    vendor: Optional[str] = None

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return asdict(self)


@dataclass
class MemoryInfo:
    """Memory information."""
    total_mb: float
    total_gb: float
    available_mb: float
    available_gb: float
    used_mb: float
    used_percent: float
    swap_total_mb: float
    swap_used_mb: float
    swap_percent: float

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return asdict(self)


@dataclass
class DiskInfo:
    """Disk information."""
    device: str
    mountpoint: str
    filesystem_type: str
    total_gb: float
    used_gb: float
    free_gb: float
    used_percent: float

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return asdict(self)


@dataclass
class GPUInfo:
    """GPU information."""
    name: str
    vendor: str
    memory_mb: Optional[int] = None
    driver_version: Optional[str] = None
    cuda_version: Optional[str] = None

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return asdict(self)


@dataclass
class NetworkInfo:
    """Network interface information."""
    interface_name: str
    addresses: List[str]
    is_up: bool
    speed_mbps: Optional[int] = None

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return asdict(self)


@dataclass
class HardwareProfile:
    """Complete hardware profile."""
    cpu: CPUInfo
    memory: MemoryInfo
    disks: List[DiskInfo]
    gpus: List[GPUInfo]
    network_interfaces: List[NetworkInfo]
    system_info: Dict[str, str]

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return {
            'cpu': self.cpu.to_dict(),
            'memory': self.memory.to_dict(),
            'disks': [disk.to_dict() for disk in self.disks],
            'gpus': [gpu.to_dict() for gpu in self.gpus],
            'network_interfaces': [net.to_dict() for net in self.network_interfaces],
            'system_info': self.system_info
        }


class HardwareDetector:
    """
    Detect and profile hardware specifications.
    """

    def __init__(self):
        """Initialize hardware detector."""
        self.psutil_available = PSUTIL_AVAILABLE

        if not PSUTIL_AVAILABLE:
            logger.warning(
                "psutil not available. Install with: pip install psutil\n"
                "Hardware detection will be limited to basic platform info."
            )

    def detect_cpu(self) -> CPUInfo:
        """
        Detect CPU information.

        Returns:
            CPUInfo with CPU specifications
        """
        logger.debug("Detecting CPU information")

        if PSUTIL_AVAILABLE:
            cpu_freq = psutil.cpu_freq()
            physical_cores = psutil.cpu_count(logical=False)
            logical_cores = psutil.cpu_count(logical=True)

            # Try to get CPU model name from /proc/cpuinfo on Linux
            model_name = self._get_cpu_model_name()

            return CPUInfo(
                model_name=model_name,
                architecture=platform.machine(),
                physical_cores=physical_cores or 0,
                logical_cores=logical_cores or 0,
                max_frequency_mhz=cpu_freq.max if cpu_freq else 0.0,
                min_frequency_mhz=cpu_freq.min if cpu_freq else 0.0,
                current_frequency_mhz=cpu_freq.current if cpu_freq else 0.0,
                vendor=self._get_cpu_vendor()
            )
        else:
            # Fallback to basic platform info
            return CPUInfo(
                model_name=platform.processor() or "Unknown",
                architecture=platform.machine(),
                physical_cores=os.cpu_count() or 0,
                logical_cores=os.cpu_count() or 0,
                max_frequency_mhz=0.0,
                min_frequency_mhz=0.0,
                current_frequency_mhz=0.0
            )

    def detect_memory(self) -> MemoryInfo:
        """
        Detect memory information.

        Returns:
            MemoryInfo with memory specifications
        """
        logger.debug("Detecting memory information")

        if PSUTIL_AVAILABLE:
            virtual_mem = psutil.virtual_memory()
            swap_mem = psutil.swap_memory()

            return MemoryInfo(
                total_mb=virtual_mem.total / (1024 * 1024),
                total_gb=virtual_mem.total / (1024 * 1024 * 1024),
                available_mb=virtual_mem.available / (1024 * 1024),
                available_gb=virtual_mem.available / (1024 * 1024 * 1024),
                used_mb=virtual_mem.used / (1024 * 1024),
                used_percent=virtual_mem.percent,
                swap_total_mb=swap_mem.total / (1024 * 1024),
                swap_used_mb=swap_mem.used / (1024 * 1024),
                swap_percent=swap_mem.percent
            )
        else:
            # No memory info available without psutil
            return MemoryInfo(
                total_mb=0.0,
                total_gb=0.0,
                available_mb=0.0,
                available_gb=0.0,
                used_mb=0.0,
                used_percent=0.0,
                swap_total_mb=0.0,
                swap_used_mb=0.0,
                swap_percent=0.0
            )

    def detect_disks(self) -> List[DiskInfo]:
        """
        Detect disk information for all mounted partitions.

        Returns:
            List of DiskInfo objects
        """
        logger.debug("Detecting disk information")

        disks = []

        if PSUTIL_AVAILABLE:
            partitions = psutil.disk_partitions()

            for partition in partitions:
                try:
                    usage = psutil.disk_usage(partition.mountpoint)

                    disk_info = DiskInfo(
                        device=partition.device,
                        mountpoint=partition.mountpoint,
                        filesystem_type=partition.fstype,
                        total_gb=usage.total / (1024 * 1024 * 1024),
                        used_gb=usage.used / (1024 * 1024 * 1024),
                        free_gb=usage.free / (1024 * 1024 * 1024),
                        used_percent=usage.percent
                    )
                    disks.append(disk_info)

                except (PermissionError, OSError) as e:
                    logger.warning(f"Could not access {partition.mountpoint}: {e}")
                    continue

        return disks

    def detect_gpus(self) -> List[GPUInfo]:
        """
        Detect GPU information.

        Returns:
            List of GPUInfo objects
        """
        logger.debug("Detecting GPU information")

        gpus = []

        # Try to detect NVIDIA GPUs using nvidia-smi
        try:
            import subprocess
            result = subprocess.run(
                ['nvidia-smi', '--query-gpu=name,driver_version,memory.total',
                 '--format=csv,noheader,nounits'],
                capture_output=True,
                text=True,
                timeout=5
            )

            if result.returncode == 0:
                for line in result.stdout.strip().split('\n'):
                    if line:
                        parts = [p.strip() for p in line.split(',')]
                        if len(parts) >= 3:
                            gpus.append(GPUInfo(
                                name=parts[0],
                                vendor="NVIDIA",
                                driver_version=parts[1],
                                memory_mb=int(float(parts[2]))
                            ))
        except (subprocess.SubprocessError, FileNotFoundError):
            logger.debug("nvidia-smi not available")

        # If no NVIDIA GPUs found, try to detect AMD GPUs
        if not gpus:
            try:
                import subprocess
                result = subprocess.run(
                    ['rocm-smi', '--showproductname'],
                    capture_output=True,
                    text=True,
                    timeout=5
                )

                if result.returncode == 0:
                    # Parse rocm-smi output
                    for line in result.stdout.split('\n'):
                        if 'GPU' in line or 'Card' in line:
                            gpus.append(GPUInfo(
                                name=line.strip(),
                                vendor="AMD"
                            ))
            except (subprocess.SubprocessError, FileNotFoundError):
                logger.debug("rocm-smi not available")

        if not gpus:
            logger.debug("No GPUs detected")

        return gpus

    def detect_network_interfaces(self) -> List[NetworkInfo]:
        """
        Detect network interfaces.

        Returns:
            List of NetworkInfo objects
        """
        logger.debug("Detecting network interfaces")

        interfaces = []

        if PSUTIL_AVAILABLE:
            net_if_addrs = psutil.net_if_addrs()
            net_if_stats = psutil.net_if_stats()

            for interface_name, addresses in net_if_addrs.items():
                # Get all IP addresses for this interface
                ip_addresses = [
                    addr.address for addr in addresses
                    if addr.family == 2  # AF_INET (IPv4)
                ]

                # Get interface status
                stats = net_if_stats.get(interface_name)
                is_up = stats.isup if stats else False
                speed_mbps = stats.speed if stats else None

                interfaces.append(NetworkInfo(
                    interface_name=interface_name,
                    addresses=ip_addresses,
                    is_up=is_up,
                    speed_mbps=speed_mbps
                ))

        return interfaces

    def get_system_info(self) -> Dict[str, str]:
        """
        Get general system information.

        Returns:
            Dictionary with system information
        """
        logger.debug("Collecting system information")

        info = {
            'system': platform.system(),
            'node': platform.node(),
            'release': platform.release(),
            'version': platform.version(),
            'machine': platform.machine(),
            'processor': platform.processor()
        }

        # Add boot time if psutil available
        if PSUTIL_AVAILABLE:
            import datetime
            boot_time = psutil.boot_time()
            info['boot_time'] = datetime.datetime.fromtimestamp(boot_time).isoformat()

        return info

    def detect_all(self) -> HardwareProfile:
        """
        Detect all hardware specifications.

        Returns:
            HardwareProfile with complete hardware information
        """
        logger.info("Detecting complete hardware profile")

        cpu = self.detect_cpu()
        memory = self.detect_memory()
        disks = self.detect_disks()
        gpus = self.detect_gpus()
        network_interfaces = self.detect_network_interfaces()
        system_info = self.get_system_info()

        profile = HardwareProfile(
            cpu=cpu,
            memory=memory,
            disks=disks,
            gpus=gpus,
            network_interfaces=network_interfaces,
            system_info=system_info
        )

        logger.info("Hardware profile detection complete")
        return profile

    def _get_cpu_model_name(self) -> str:
        """Get CPU model name from /proc/cpuinfo on Linux."""
        try:
            if platform.system() == 'Linux':
                with open('/proc/cpuinfo', 'r') as f:
                    for line in f:
                        if line.strip().startswith('model name'):
                            return line.split(':', 1)[1].strip()
        except (IOError, OSError):
            pass

        return platform.processor() or "Unknown CPU"

    def _get_cpu_vendor(self) -> Optional[str]:
        """Get CPU vendor from /proc/cpuinfo on Linux."""
        try:
            if platform.system() == 'Linux':
                with open('/proc/cpuinfo', 'r') as f:
                    for line in f:
                        if line.strip().startswith('vendor_id'):
                            return line.split(':', 1)[1].strip()
        except (IOError, OSError):
            pass

        return None


# Export public API
__all__ = [
    'HardwareDetector',
    'HardwareProfile',
    'CPUInfo',
    'MemoryInfo',
    'DiskInfo',
    'GPUInfo',
    'NetworkInfo',
    'PSUTIL_AVAILABLE'
]
