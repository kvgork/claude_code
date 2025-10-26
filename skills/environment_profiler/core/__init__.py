"""
Environment Profiler Core Modules

Provides comprehensive environment detection capabilities including:
- Hardware profiling (CPU, memory, disk, GPU, network)
- Software profiling (OS, Python, packages, libraries)
"""

from .hardware_detector import (
    HardwareDetector,
    HardwareProfile,
    CPUInfo,
    MemoryInfo,
    DiskInfo,
    GPUInfo,
    NetworkInfo,
    PSUTIL_AVAILABLE
)

from .software_detector import (
    SoftwareDetector,
    SoftwareProfile,
    PythonInfo,
    PackageInfo,
    OSInfo
)

__all__ = [
    # Hardware Detection
    'HardwareDetector',
    'HardwareProfile',
    'CPUInfo',
    'MemoryInfo',
    'DiskInfo',
    'GPUInfo',
    'NetworkInfo',
    'PSUTIL_AVAILABLE',

    # Software Detection
    'SoftwareDetector',
    'SoftwareProfile',
    'PythonInfo',
    'PackageInfo',
    'OSInfo'
]

__version__ = '0.1.0'
