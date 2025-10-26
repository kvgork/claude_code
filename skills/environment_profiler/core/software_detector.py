"""
Software Environment Detection Module

Detect and profile software environment (OS, Python, packages, etc.).
"""

import sys
import os
import platform
import subprocess
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, asdict, field
from pathlib import Path
import logging

logger = logging.getLogger(__name__)


@dataclass
class PythonInfo:
    """Python environment information."""
    version: str
    implementation: str
    compiler: str
    build_date: str
    executable_path: str
    prefix: str
    base_prefix: str
    virtual_env: Optional[str]
    pip_version: Optional[str] = None

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return asdict(self)


@dataclass
class PackageInfo:
    """Installed package information."""
    name: str
    version: str
    location: Optional[str] = None

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return asdict(self)


@dataclass
class OSInfo:
    """Operating system information."""
    system: str
    release: str
    version: str
    architecture: str
    hostname: str
    kernel_version: Optional[str] = None
    distribution: Optional[str] = None
    distribution_version: Optional[str] = None

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return asdict(self)


@dataclass
class SoftwareProfile:
    """Complete software environment profile."""
    os_info: OSInfo
    python_info: PythonInfo
    installed_packages: List[PackageInfo]
    environment_variables: Dict[str, str]
    system_libraries: Dict[str, str] = field(default_factory=dict)

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return {
            'os_info': self.os_info.to_dict(),
            'python_info': self.python_info.to_dict(),
            'installed_packages': [pkg.to_dict() for pkg in self.installed_packages],
            'environment_variables': self.environment_variables,
            'system_libraries': self.system_libraries
        }


class SoftwareDetector:
    """
    Detect and profile software environment.
    """

    def __init__(self):
        """Initialize software detector."""
        logger.debug("SoftwareDetector initialized")

    def detect_os(self) -> OSInfo:
        """
        Detect operating system information.

        Returns:
            OSInfo with OS specifications
        """
        logger.debug("Detecting OS information")

        os_info = OSInfo(
            system=platform.system(),
            release=platform.release(),
            version=platform.version(),
            architecture=platform.machine(),
            hostname=platform.node()
        )

        # Linux-specific detection
        if platform.system() == 'Linux':
            # Try to get kernel version
            try:
                result = subprocess.run(
                    ['uname', '-r'],
                    capture_output=True,
                    text=True,
                    timeout=5
                )
                if result.returncode == 0:
                    os_info.kernel_version = result.stdout.strip()
            except (subprocess.SubprocessError, FileNotFoundError):
                pass

            # Try to detect distribution
            try:
                # Try reading /etc/os-release (standard on modern Linux)
                with open('/etc/os-release', 'r') as f:
                    os_release = {}
                    for line in f:
                        if '=' in line:
                            key, value = line.strip().split('=', 1)
                            os_release[key] = value.strip('"')

                    os_info.distribution = os_release.get('NAME', 'Unknown')
                    os_info.distribution_version = os_release.get('VERSION_ID', 'Unknown')

            except (IOError, OSError):
                # Fallback to platform.linux_distribution() if available
                try:
                    dist_info = platform.freedesktop_os_release()
                    os_info.distribution = dist_info.get('NAME', 'Unknown')
                    os_info.distribution_version = dist_info.get('VERSION_ID', 'Unknown')
                except AttributeError:
                    logger.debug("Could not detect Linux distribution")

        # macOS-specific detection
        elif platform.system() == 'Darwin':
            try:
                os_info.distribution = 'macOS'
                os_info.distribution_version = platform.mac_ver()[0]
            except:
                pass

        # Windows-specific detection
        elif platform.system() == 'Windows':
            try:
                os_info.distribution = 'Windows'
                os_info.distribution_version = platform.win32_ver()[0]
            except:
                pass

        return os_info

    def detect_python(self) -> PythonInfo:
        """
        Detect Python environment information.

        Returns:
            PythonInfo with Python specifications
        """
        logger.debug("Detecting Python information")

        # Detect virtual environment
        virtual_env = None
        if hasattr(sys, 'real_prefix') or (
            hasattr(sys, 'base_prefix') and sys.base_prefix != sys.prefix
        ):
            virtual_env = sys.prefix

        # Get pip version
        pip_version = None
        try:
            result = subprocess.run(
                [sys.executable, '-m', 'pip', '--version'],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                # Parse "pip X.Y.Z from ..." format
                pip_version = result.stdout.split()[1]
        except (subprocess.SubprocessError, FileNotFoundError):
            logger.debug("Could not detect pip version")

        python_info = PythonInfo(
            version=sys.version.split()[0],
            implementation=platform.python_implementation(),
            compiler=platform.python_compiler(),
            build_date=platform.python_build()[1],
            executable_path=sys.executable,
            prefix=sys.prefix,
            base_prefix=sys.base_prefix,
            virtual_env=virtual_env,
            pip_version=pip_version
        )

        return python_info

    def detect_installed_packages(
        self,
        include_versions: bool = True
    ) -> List[PackageInfo]:
        """
        Detect installed Python packages.

        Args:
            include_versions: Include package versions

        Returns:
            List of PackageInfo objects
        """
        logger.debug("Detecting installed packages")

        packages = []

        try:
            # Use pip list to get installed packages
            result = subprocess.run(
                [sys.executable, '-m', 'pip', 'list', '--format=json'],
                capture_output=True,
                text=True,
                timeout=30
            )

            if result.returncode == 0:
                import json
                pip_list = json.loads(result.stdout)

                for pkg in pip_list:
                    packages.append(PackageInfo(
                        name=pkg['name'],
                        version=pkg['version'],
                        location=None  # pip list doesn't provide location
                    ))

                logger.info(f"Detected {len(packages)} installed packages")

        except (subprocess.SubprocessError, FileNotFoundError, json.JSONDecodeError) as e:
            logger.error(f"Error detecting packages: {e}")

            # Fallback: try using pkg_resources
            try:
                import pkg_resources
                for pkg in pkg_resources.working_set:
                    packages.append(PackageInfo(
                        name=pkg.project_name,
                        version=pkg.version,
                        location=pkg.location
                    ))

                logger.info(f"Detected {len(packages)} packages using pkg_resources")

            except Exception as e:
                logger.error(f"pkg_resources fallback also failed: {e}")

        return packages

    def get_environment_variables(
        self,
        filter_sensitive: bool = True
    ) -> Dict[str, str]:
        """
        Get environment variables.

        Args:
            filter_sensitive: Filter out potentially sensitive variables

        Returns:
            Dictionary of environment variables
        """
        logger.debug("Collecting environment variables")

        env_vars = dict(os.environ)

        if filter_sensitive:
            # List of sensitive variable patterns
            sensitive_patterns = [
                'PASSWORD', 'SECRET', 'TOKEN', 'KEY', 'API', 'AUTH',
                'CREDENTIAL', 'PASS', 'PWD'
            ]

            # Filter out sensitive variables
            filtered_vars = {}
            for key, value in env_vars.items():
                is_sensitive = any(pattern in key.upper() for pattern in sensitive_patterns)

                if is_sensitive:
                    filtered_vars[key] = '<REDACTED>'
                else:
                    filtered_vars[key] = value

            return filtered_vars

        return env_vars

    def detect_system_libraries(self) -> Dict[str, str]:
        """
        Detect important system libraries and their versions.

        Returns:
            Dictionary of library names and versions
        """
        logger.debug("Detecting system libraries")

        libraries = {}

        # Common system libraries to check
        lib_commands = {
            'glibc': ['ldd', '--version'],
            'gcc': ['gcc', '--version'],
            'git': ['git', '--version'],
            'docker': ['docker', '--version'],
            'nodejs': ['node', '--version'],
            'npm': ['npm', '--version']
        }

        for lib_name, command in lib_commands.items():
            try:
                result = subprocess.run(
                    command,
                    capture_output=True,
                    text=True,
                    timeout=5
                )

                if result.returncode == 0:
                    # Extract version from first line
                    first_line = result.stdout.strip().split('\n')[0]
                    libraries[lib_name] = first_line

            except (subprocess.SubprocessError, FileNotFoundError):
                logger.debug(f"{lib_name} not found")
                continue

        return libraries

    def detect_all(
        self,
        include_env_vars: bool = True,
        include_packages: bool = True,
        filter_sensitive: bool = True
    ) -> SoftwareProfile:
        """
        Detect complete software environment.

        Args:
            include_env_vars: Include environment variables
            include_packages: Include installed packages
            filter_sensitive: Filter sensitive information from env vars

        Returns:
            SoftwareProfile with complete software information
        """
        logger.info("Detecting complete software profile")

        os_info = self.detect_os()
        python_info = self.detect_python()

        installed_packages = []
        if include_packages:
            installed_packages = self.detect_installed_packages()

        environment_variables = {}
        if include_env_vars:
            environment_variables = self.get_environment_variables(filter_sensitive=filter_sensitive)

        system_libraries = self.detect_system_libraries()

        profile = SoftwareProfile(
            os_info=os_info,
            python_info=python_info,
            installed_packages=installed_packages,
            environment_variables=environment_variables,
            system_libraries=system_libraries
        )

        logger.info("Software profile detection complete")
        return profile

    def compare_packages(
        self,
        baseline_packages: List[PackageInfo],
        current_packages: List[PackageInfo]
    ) -> Dict[str, Any]:
        """
        Compare two package lists to find differences.

        Args:
            baseline_packages: Baseline package list
            current_packages: Current package list

        Returns:
            Dictionary with added, removed, and updated packages
        """
        logger.debug("Comparing package lists")

        baseline_dict = {pkg.name: pkg.version for pkg in baseline_packages}
        current_dict = {pkg.name: pkg.version for pkg in current_packages}

        added = []
        removed = []
        updated = []

        # Find added and updated packages
        for name, version in current_dict.items():
            if name not in baseline_dict:
                added.append({'name': name, 'version': version})
            elif baseline_dict[name] != version:
                updated.append({
                    'name': name,
                    'old_version': baseline_dict[name],
                    'new_version': version
                })

        # Find removed packages
        for name, version in baseline_dict.items():
            if name not in current_dict:
                removed.append({'name': name, 'version': version})

        return {
            'added': added,
            'removed': removed,
            'updated': updated,
            'total_changes': len(added) + len(removed) + len(updated)
        }


# Export public API
__all__ = [
    'SoftwareDetector',
    'SoftwareProfile',
    'PythonInfo',
    'PackageInfo',
    'OSInfo'
]
