"""
Environment Profiler Skill Demo

Demonstrates all operations of the environment_profiler skill.
"""

import json
from pathlib import Path

# Import operations
from skills.environment_profiler import (
    detect_hardware_profile,
    detect_software_profile,
    create_environment_snapshot,
    compare_environments,
    export_package_requirements
)


def print_separator(title):
    """Print a formatted separator."""
    print(f"\n{'=' * 80}")
    print(f"  {title}")
    print('=' * 80)


def demo_detect_hardware():
    """Demonstrate hardware detection."""
    print_separator("Demo 1: Detect Hardware Profile")

    print("\n1. Detecting complete hardware profile...")
    result = detect_hardware_profile()

    if result.success:
        hw = result.data
        print(f"✓ Hardware detection successful ({result.duration:.2f}s)")

        print(f"\n  CPU:")
        print(f"    - Model: {hw['cpu']['model_name']}")
        print(f"    - Architecture: {hw['cpu']['architecture']}")
        print(f"    - Physical cores: {hw['cpu']['physical_cores']}")
        print(f"    - Logical cores: {hw['cpu']['logical_cores']}")
        print(f"    - Max frequency: {hw['cpu']['max_frequency_mhz']:.0f} MHz")

        print(f"\n  Memory:")
        print(f"    - Total: {hw['memory']['total_gb']:.2f} GB")
        print(f"    - Used: {hw['memory']['used_percent']:.1f}%")
        print(f"    - Available: {hw['memory']['available_gb']:.2f} GB")
        print(f"    - Swap total: {hw['memory']['swap_total_mb']:.0f} MB")

        if hw['disks']:
            print(f"\n  Disks ({len(hw['disks'])}):")
            for disk in hw['disks'][:3]:  # Show first 3
                print(f"    - {disk['mountpoint']}: {disk['total_gb']:.1f} GB " +
                      f"({disk['used_percent']:.1f}% used)")

        if hw['gpus']:
            print(f"\n  GPUs ({len(hw['gpus'])}):")
            for gpu in hw['gpus']:
                print(f"    - {gpu['name']} ({gpu['vendor']})")
                if gpu['memory_mb']:
                    print(f"      Memory: {gpu['memory_mb']} MB")
        else:
            print(f"\n  GPUs: None detected")

        if hw['network_interfaces']:
            active_interfaces = [ni for ni in hw['network_interfaces'] if ni['is_up']]
            print(f"\n  Network Interfaces ({len(active_interfaces)} active):")
            for ni in active_interfaces[:3]:  # Show first 3
                print(f"    - {ni['interface_name']}: {', '.join(ni['addresses']) if ni['addresses'] else 'No IP'}")

        print(f"\n  System:")
        print(f"    - OS: {hw['system_info']['system']}")
        print(f"    - Release: {hw['system_info']['release']}")
        print(f"    - Machine: {hw['system_info']['machine']}")

    else:
        print(f"✗ Hardware detection failed: {result.error}")


def demo_detect_software():
    """Demonstrate software detection."""
    print_separator("Demo 2: Detect Software Profile")

    print("\n1. Detecting complete software profile...")
    result = detect_software_profile(
        include_packages=True,
        include_env_vars=True,
        filter_sensitive=True
    )

    if result.success:
        sw = result.data
        print(f"✓ Software detection successful ({result.duration:.2f}s)")

        print(f"\n  Operating System:")
        print(f"    - System: {sw['os_info']['system']}")
        if sw['os_info']['distribution']:
            print(f"    - Distribution: {sw['os_info']['distribution']} " +
                  f"{sw['os_info']['distribution_version']}")
        print(f"    - Kernel: {sw['os_info']['kernel_version'] or 'N/A'}")
        print(f"    - Hostname: {sw['os_info']['hostname']}")
        print(f"    - Architecture: {sw['os_info']['architecture']}")

        print(f"\n  Python Environment:")
        print(f"    - Version: {sw['python_info']['version']}")
        print(f"    - Implementation: {sw['python_info']['implementation']}")
        print(f"    - Executable: {sw['python_info']['executable_path']}")
        print(f"    - Virtual env: {sw['python_info']['virtual_env'] or 'No'}")
        print(f"    - pip version: {sw['python_info']['pip_version'] or 'Unknown'}")

        print(f"\n  Installed Packages: {len(sw['installed_packages'])}")
        # Show first 10 packages
        print(f"    Sample packages:")
        for pkg in sorted(sw['installed_packages'], key=lambda p: p['name'])[:10]:
            print(f"      - {pkg['name']}=={pkg['version']}")

        print(f"\n  Environment Variables: {len(sw['environment_variables'])}")
        print(f"    (Sensitive values redacted)")

        if sw['system_libraries']:
            print(f"\n  System Libraries:")
            for lib, version in list(sw['system_libraries'].items())[:5]:
                print(f"    - {lib}: {version}")

    else:
        print(f"✗ Software detection failed: {result.error}")


def demo_create_snapshot():
    """Demonstrate environment snapshot creation."""
    print_separator("Demo 3: Create Environment Snapshots")

    print("\n1. Creating snapshot for 'v1.0.0'...")
    result1 = create_environment_snapshot(
        snapshot_name="v1.0.0",
        output_dir="./environment_snapshots",
        include_packages=True,
        filter_sensitive=True
    )

    if result1.success:
        print(f"✓ Snapshot created successfully ({result1.duration:.2f}s)")
        print(f"  - File: {result1.data['snapshot_file']}")
        print(f"  - Size: {result1.metadata['file_size_bytes']:,} bytes")
        print(f"\n  Hardware Summary:")
        print(f"    - CPU: {result1.data['hardware_summary']['cpu']}")
        print(f"    - Memory: {result1.data['hardware_summary']['memory_gb']:.1f} GB")
        print(f"    - Disks: {result1.data['hardware_summary']['disk_count']}")
        print(f"    - GPUs: {result1.data['hardware_summary']['gpu_count']}")
        print(f"\n  Software Summary:")
        print(f"    - OS: {result1.data['software_summary']['os']}")
        print(f"    - Python: {result1.data['software_summary']['python_version']}")
        print(f"    - Packages: {result1.data['software_summary']['package_count']}")

        return result1.data['snapshot_file']
    else:
        print(f"✗ Snapshot creation failed: {result1.error}")
        return None


def demo_compare_snapshots(baseline_file):
    """Demonstrate snapshot comparison."""
    print_separator("Demo 4: Compare Environment Snapshots")

    if not baseline_file:
        print("⚠️ Skipping comparison (no baseline snapshot)")
        return

    # Create a second snapshot
    print("\n1. Creating snapshot for 'v1.1.0' (simulated new release)...")
    result = create_environment_snapshot(
        snapshot_name="v1.1.0",
        output_dir="./environment_snapshots",
        include_packages=True,
        filter_sensitive=True
    )

    if not result.success:
        print(f"✗ Failed to create second snapshot: {result.error}")
        return

    current_file = result.data['snapshot_file']
    print(f"✓ Second snapshot created: {current_file}")

    print("\n2. Comparing v1.1.0 with baseline v1.0.0...")
    comparison = compare_environments(
        current_snapshot_file=current_file,
        baseline_snapshot_file=baseline_file
    )

    if comparison.success:
        diff = comparison.data
        print(f"✓ Comparison completed ({comparison.duration:.2f}s)")
        print(f"\n  Comparison Results:")
        print(f"    - Environments different: {diff['is_different']}")
        print(f"    - Hardware changed: {diff['hardware_diff']['has_differences']}")
        print(f"    - Software changed: {diff['software_diff']['has_differences']}")
        print(f"    - Total package changes: {diff['package_diff']['total_changes']}")

        # Show hardware differences
        if diff['hardware_diff']['has_differences']:
            print(f"\n  Hardware Differences:")
            for change in diff['hardware_diff']['differences']:
                print(f"    - {change['component']}.{change['field']}: " +
                      f"{change['baseline']} → {change['current']}")

        # Show software differences
        if diff['software_diff']['has_differences']:
            print(f"\n  Software Differences:")
            for change in diff['software_diff']['differences']:
                print(f"    - {change['component']}.{change['field']}: " +
                      f"{change['baseline']} → {change['current']}")

        # Show package changes
        pkg_diff = diff['package_diff']
        if pkg_diff['total_changes'] > 0:
            print(f"\n  Package Changes:")

            if pkg_diff['added']:
                print(f"    Added ({len(pkg_diff['added'])}):")
                for pkg in pkg_diff['added'][:5]:
                    print(f"      + {pkg['name']}=={pkg['version']}")

            if pkg_diff['removed']:
                print(f"    Removed ({len(pkg_diff['removed'])}):")
                for pkg in pkg_diff['removed'][:5]:
                    print(f"      - {pkg['name']}=={pkg['version']}")

            if pkg_diff['updated']:
                print(f"    Updated ({len(pkg_diff['updated'])}):")
                for pkg in pkg_diff['updated'][:5]:
                    print(f"      ~ {pkg['name']}: {pkg['old_version']} → {pkg['new_version']}")
        else:
            print(f"\n  No package changes detected")

    else:
        print(f"✗ Comparison failed: {comparison.error}")


def demo_export_requirements():
    """Demonstrate requirements export."""
    print_separator("Demo 5: Export Package Requirements")

    print("\n1. Exporting installed packages to requirements.txt...")
    result = export_package_requirements(
        output_file="./environment_snapshots/requirements-demo.txt",
        include_versions=True
    )

    if result.success:
        print(f"✓ Requirements exported successfully ({result.duration:.2f}s)")
        print(f"  - File: {result.data['requirements_file']}")
        print(f"  - Package count: {result.data['package_count']}")
        print(f"  - File size: {result.metadata['file_size_bytes']:,} bytes")

        # Show first few lines
        req_file = Path(result.data['requirements_file'])
        if req_file.exists():
            with open(req_file, 'r') as f:
                lines = f.readlines()[:10]
            print(f"\n  First 10 packages:")
            for line in lines:
                print(f"    {line.strip()}")

    else:
        print(f"✗ Export failed: {result.error}")


def main():
    """Run all demos."""
    print("\n" + "=" * 80)
    print("  ENVIRONMENT PROFILER SKILL DEMO")
    print("=" * 80)

    # Demo 1: Detect hardware
    demo_detect_hardware()

    # Demo 2: Detect software
    demo_detect_software()

    # Demo 3: Create snapshot
    baseline_file = demo_create_snapshot()

    # Demo 4: Compare snapshots
    demo_compare_snapshots(baseline_file)

    # Demo 5: Export requirements
    demo_export_requirements()

    # Summary
    print_separator("Demo Complete")
    print("\n✓ All demos completed successfully!")
    print("\nGenerated outputs:")
    print("  - Environment snapshots: ./environment_snapshots/")
    print("  - Requirements file: ./environment_snapshots/requirements-demo.txt")
    print("\nYou can explore the generated files to see detailed environment information.")
    print()


if __name__ == "__main__":
    main()
