"""
Environment Profiler Skill Operations

Provides operations for detecting and profiling hardware and software environments.
"""

import time
import json
from pathlib import Path
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, asdict
import logging

from .core import (
    HardwareDetector,
    SoftwareDetector,
    HardwareProfile,
    SoftwareProfile,
    PackageInfo
)

logger = logging.getLogger(__name__)


@dataclass
class OperationResult:
    """Standard result format for all operations."""
    success: bool
    data: Optional[Dict[str, Any]] = None
    error: Optional[str] = None
    error_code: Optional[str] = None
    duration: float = 0.0
    metadata: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return asdict(self)


def detect_hardware_profile() -> OperationResult:
    """
    Detect complete hardware profile.

    Returns:
        OperationResult with hardware profile data
    """
    start_time = time.time()

    try:
        logger.info("Detecting hardware profile")

        detector = HardwareDetector()
        profile = detector.detect_all()

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=profile.to_dict(),
            duration=duration,
            metadata={
                'cpu_cores': profile.cpu.logical_cores,
                'memory_gb': profile.memory.total_gb,
                'disk_count': len(profile.disks),
                'gpu_count': len(profile.gpus)
            }
        )

    except Exception as e:
        logger.error(f"Error detecting hardware profile: {e}")
        return OperationResult(
            success=False,
            error=str(e),
            error_code="HARDWARE_DETECTION_ERROR",
            duration=time.time() - start_time
        )


def detect_software_profile(
    include_env_vars: bool = True,
    include_packages: bool = True,
    filter_sensitive: bool = True
) -> OperationResult:
    """
    Detect complete software environment profile.

    Args:
        include_env_vars: Include environment variables
        include_packages: Include installed packages
        filter_sensitive: Filter sensitive information from env vars

    Returns:
        OperationResult with software profile data
    """
    start_time = time.time()

    try:
        logger.info("Detecting software profile")

        detector = SoftwareDetector()
        profile = detector.detect_all(
            include_env_vars=include_env_vars,
            include_packages=include_packages,
            filter_sensitive=filter_sensitive
        )

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=profile.to_dict(),
            duration=duration,
            metadata={
                'python_version': profile.python_info.version,
                'os_system': profile.os_info.system,
                'package_count': len(profile.installed_packages),
                'env_var_count': len(profile.environment_variables)
            }
        )

    except Exception as e:
        logger.error(f"Error detecting software profile: {e}")
        return OperationResult(
            success=False,
            error=str(e),
            error_code="SOFTWARE_DETECTION_ERROR",
            duration=time.time() - start_time
        )


def create_environment_snapshot(
    snapshot_name: str,
    output_dir: str = "./environment_snapshots",
    include_env_vars: bool = True,
    include_packages: bool = True,
    filter_sensitive: bool = True
) -> OperationResult:
    """
    Create a complete environment snapshot (hardware + software).

    Args:
        snapshot_name: Name for this snapshot (e.g., "v1.0.0", "production")
        output_dir: Directory to store snapshot
        include_env_vars: Include environment variables
        include_packages: Include installed packages
        filter_sensitive: Filter sensitive information

    Returns:
        OperationResult with snapshot file path
    """
    start_time = time.time()

    try:
        logger.info(f"Creating environment snapshot: {snapshot_name}")

        # Create output directory
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        # Detect hardware
        hardware_detector = HardwareDetector()
        hardware_profile = hardware_detector.detect_all()

        # Detect software
        software_detector = SoftwareDetector()
        software_profile = software_detector.detect_all(
            include_env_vars=include_env_vars,
            include_packages=include_packages,
            filter_sensitive=filter_sensitive
        )

        # Create snapshot
        snapshot = {
            'snapshot_name': snapshot_name,
            'timestamp': time.time(),
            'timestamp_iso': time.strftime('%Y-%m-%d %H:%M:%S'),
            'hardware': hardware_profile.to_dict(),
            'software': software_profile.to_dict()
        }

        # Save snapshot
        snapshot_file = output_path / f"snapshot_{snapshot_name}_{int(time.time())}.json"
        with open(snapshot_file, 'w') as f:
            json.dump(snapshot, f, indent=2)

        duration = time.time() - start_time

        logger.info(f"Snapshot saved to: {snapshot_file}")

        return OperationResult(
            success=True,
            data={
                'snapshot_file': str(snapshot_file),
                'snapshot_name': snapshot_name,
                'hardware_summary': {
                    'cpu': hardware_profile.cpu.model_name,
                    'memory_gb': hardware_profile.memory.total_gb,
                    'disk_count': len(hardware_profile.disks),
                    'gpu_count': len(hardware_profile.gpus)
                },
                'software_summary': {
                    'os': software_profile.os_info.system,
                    'python_version': software_profile.python_info.version,
                    'package_count': len(software_profile.installed_packages)
                }
            },
            duration=duration,
            metadata={
                'output_dir': str(output_path),
                'file_size_bytes': snapshot_file.stat().st_size
            }
        )

    except Exception as e:
        logger.error(f"Error creating environment snapshot: {e}")
        return OperationResult(
            success=False,
            error=str(e),
            error_code="SNAPSHOT_CREATION_ERROR",
            duration=time.time() - start_time
        )


def compare_environments(
    current_snapshot_file: str,
    baseline_snapshot_file: str
) -> OperationResult:
    """
    Compare two environment snapshots to identify differences.

    Args:
        current_snapshot_file: Path to current snapshot JSON file
        baseline_snapshot_file: Path to baseline snapshot JSON file

    Returns:
        OperationResult with comparison data
    """
    start_time = time.time()

    try:
        logger.info("Comparing environment snapshots")

        # Load snapshots
        with open(current_snapshot_file, 'r') as f:
            current = json.load(f)

        with open(baseline_snapshot_file, 'r') as f:
            baseline = json.load(f)

        # Compare hardware
        hardware_diff = _compare_hardware(
            current['hardware'],
            baseline['hardware']
        )

        # Compare software
        software_diff = _compare_software(
            current['software'],
            baseline['software']
        )

        # Compare packages
        current_packages = [
            PackageInfo(**pkg) for pkg in current['software']['installed_packages']
        ]
        baseline_packages = [
            PackageInfo(**pkg) for pkg in baseline['software']['installed_packages']
        ]

        software_detector = SoftwareDetector()
        package_diff = software_detector.compare_packages(
            baseline_packages,
            current_packages
        )

        duration = time.time() - start_time

        # Determine if environments are significantly different
        is_different = (
            hardware_diff['has_differences'] or
            software_diff['has_differences'] or
            package_diff['total_changes'] > 0
        )

        return OperationResult(
            success=True,
            data={
                'current_snapshot': current['snapshot_name'],
                'baseline_snapshot': baseline['snapshot_name'],
                'is_different': is_different,
                'hardware_diff': hardware_diff,
                'software_diff': software_diff,
                'package_diff': package_diff
            },
            duration=duration,
            metadata={
                'total_package_changes': package_diff['total_changes'],
                'hardware_changed': hardware_diff['has_differences'],
                'os_changed': software_diff['os_changed']
            }
        )

    except Exception as e:
        logger.error(f"Error comparing environments: {e}")
        return OperationResult(
            success=False,
            error=str(e),
            error_code="COMPARISON_ERROR",
            duration=time.time() - start_time
        )


def export_package_requirements(
    output_file: str = "./requirements.txt",
    include_versions: bool = True
) -> OperationResult:
    """
    Export installed packages to requirements.txt format.

    Args:
        output_file: Path to output requirements file
        include_versions: Include package versions

    Returns:
        OperationResult with file path
    """
    start_time = time.time()

    try:
        logger.info(f"Exporting package requirements to: {output_file}")

        detector = SoftwareDetector()
        packages = detector.detect_installed_packages()

        # Create requirements.txt content
        output_path = Path(output_file)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        with open(output_path, 'w') as f:
            for pkg in sorted(packages, key=lambda p: p.name.lower()):
                if include_versions:
                    f.write(f"{pkg.name}=={pkg.version}\n")
                else:
                    f.write(f"{pkg.name}\n")

        duration = time.time() - start_time

        logger.info(f"Exported {len(packages)} packages")

        return OperationResult(
            success=True,
            data={
                'requirements_file': str(output_path),
                'package_count': len(packages)
            },
            duration=duration,
            metadata={
                'include_versions': include_versions,
                'file_size_bytes': output_path.stat().st_size
            }
        )

    except Exception as e:
        logger.error(f"Error exporting requirements: {e}")
        return OperationResult(
            success=False,
            error=str(e),
            error_code="EXPORT_ERROR",
            duration=time.time() - start_time
        )


def _compare_hardware(
    current: Dict[str, Any],
    baseline: Dict[str, Any]
) -> Dict[str, Any]:
    """Compare hardware profiles."""
    differences = []
    has_differences = False

    # Compare CPU
    if current['cpu']['model_name'] != baseline['cpu']['model_name']:
        differences.append({
            'component': 'cpu',
            'field': 'model_name',
            'current': current['cpu']['model_name'],
            'baseline': baseline['cpu']['model_name']
        })
        has_differences = True

    # Compare memory
    if abs(current['memory']['total_gb'] - baseline['memory']['total_gb']) > 0.1:
        differences.append({
            'component': 'memory',
            'field': 'total_gb',
            'current': current['memory']['total_gb'],
            'baseline': baseline['memory']['total_gb']
        })
        has_differences = True

    # Compare GPU count
    if len(current['gpus']) != len(baseline['gpus']):
        differences.append({
            'component': 'gpu',
            'field': 'count',
            'current': len(current['gpus']),
            'baseline': len(baseline['gpus'])
        })
        has_differences = True

    return {
        'has_differences': has_differences,
        'differences': differences
    }


def _compare_software(
    current: Dict[str, Any],
    baseline: Dict[str, Any]
) -> Dict[str, Any]:
    """Compare software profiles."""
    differences = []
    has_differences = False

    # Compare OS
    os_changed = False
    if current['os_info']['system'] != baseline['os_info']['system']:
        differences.append({
            'component': 'os',
            'field': 'system',
            'current': current['os_info']['system'],
            'baseline': baseline['os_info']['system']
        })
        has_differences = True
        os_changed = True

    # Compare Python version
    if current['python_info']['version'] != baseline['python_info']['version']:
        differences.append({
            'component': 'python',
            'field': 'version',
            'current': current['python_info']['version'],
            'baseline': baseline['python_info']['version']
        })
        has_differences = True

    return {
        'has_differences': has_differences,
        'os_changed': os_changed,
        'differences': differences
    }


# Export public API
__all__ = [
    'OperationResult',
    'detect_hardware_profile',
    'detect_software_profile',
    'create_environment_snapshot',
    'compare_environments',
    'export_package_requirements'
]
