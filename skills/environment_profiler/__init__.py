"""
Environment Profiler Skill

Detect and profile hardware and software environments for release comparison.
"""

# Import operations for agent invocation
from .operations import (
    detect_hardware_profile,
    detect_software_profile,
    create_environment_snapshot,
    compare_environments,
    export_package_requirements,
    OperationResult
)

# Import core modules for direct use
from .core import (
    HardwareDetector,
    SoftwareDetector,
    HardwareProfile,
    SoftwareProfile,
    CPUInfo,
    MemoryInfo,
    DiskInfo,
    GPUInfo,
    NetworkInfo,
    PythonInfo,
    PackageInfo,
    OSInfo
)

__all__ = [
    # Operations
    "detect_hardware_profile",
    "detect_software_profile",
    "create_environment_snapshot",
    "compare_environments",
    "export_package_requirements",
    "OperationResult",

    # Core Classes
    "HardwareDetector",
    "SoftwareDetector",

    # Profiles
    "HardwareProfile",
    "SoftwareProfile",

    # Data Classes
    "CPUInfo",
    "MemoryInfo",
    "DiskInfo",
    "GPUInfo",
    "NetworkInfo",
    "PythonInfo",
    "PackageInfo",
    "OSInfo"
]

__version__ = "0.1.0"
