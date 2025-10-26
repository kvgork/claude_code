# Environment Profiler Skill

Detect and profile hardware and software environments for release comparison and reproducibility.

## Operations

### detect_hardware_profile

Detect complete hardware profile including CPU, memory, disk, GPU, and network interfaces.

**Input:**
```yaml
# No input parameters required
```

**Output:**
```yaml
success: bool
data:
  cpu:
    model_name: str
    architecture: str
    physical_cores: int
    logical_cores: int
    max_frequency_mhz: float
    min_frequency_mhz: float
    current_frequency_mhz: float
    vendor: str
  memory:
    total_mb: float
    total_gb: float
    available_mb: float
    available_gb: float
    used_mb: float
    used_percent: float
    swap_total_mb: float
    swap_used_mb: float
    swap_percent: float
  disks: list
    - device: str
      mountpoint: str
      filesystem_type: str
      total_gb: float
      used_gb: float
      free_gb: float
      used_percent: float
  gpus: list
    - name: str
      vendor: str
      memory_mb: int
      driver_version: str
  network_interfaces: list
    - interface_name: str
      addresses: list
      is_up: bool
      speed_mbps: int
  system_info:
    system: str
    node: str
    release: str
    version: str
    machine: str
    processor: str
duration: float
metadata:
  cpu_cores: int
  memory_gb: float
  disk_count: int
  gpu_count: int
```

**Example:**
```python
from skills.environment_profiler import detect_hardware_profile

result = detect_hardware_profile()

if result.success:
    hw = result.data
    print(f"CPU: {hw['cpu']['model_name']}")
    print(f"Memory: {hw['memory']['total_gb']:.1f} GB")
    print(f"Disks: {len(hw['disks'])}")
    print(f"GPUs: {len(hw['gpus'])}")
```

---

### detect_software_profile

Detect complete software environment including OS, Python, packages, and system libraries.

**Input:**
```yaml
include_env_vars: bool     # Include environment variables (default: true)
include_packages: bool     # Include installed packages (default: true)
filter_sensitive: bool     # Filter sensitive information (default: true)
```

**Output:**
```yaml
success: bool
data:
  os_info:
    system: str
    release: str
    version: str
    architecture: str
    hostname: str
    kernel_version: str
    distribution: str
    distribution_version: str
  python_info:
    version: str
    implementation: str
    compiler: str
    build_date: str
    executable_path: str
    prefix: str
    base_prefix: str
    virtual_env: str
    pip_version: str
  installed_packages: list
    - name: str
      version: str
      location: str
  environment_variables: dict
  system_libraries: dict
duration: float
metadata:
  python_version: str
  os_system: str
  package_count: int
  env_var_count: int
```

**Example:**
```python
from skills.environment_profiler import detect_software_profile

result = detect_software_profile(
    include_packages=True,
    filter_sensitive=True
)

if result.success:
    sw = result.data
    print(f"OS: {sw['os_info']['distribution']} {sw['os_info']['distribution_version']}")
    print(f"Python: {sw['python_info']['version']}")
    print(f"Packages: {len(sw['installed_packages'])}")
```

---

### create_environment_snapshot

Create a complete environment snapshot combining hardware and software profiles.

**Input:**
```yaml
snapshot_name: str         # Name for snapshot (e.g., "v1.0.0", "production")
output_dir: str           # Directory to store snapshot (default: "./environment_snapshots")
include_env_vars: bool    # Include environment variables (default: true)
include_packages: bool    # Include installed packages (default: true)
filter_sensitive: bool    # Filter sensitive information (default: true)
```

**Output:**
```yaml
success: bool
data:
  snapshot_file: str      # Path to snapshot file
  snapshot_name: str
  hardware_summary:
    cpu: str
    memory_gb: float
    disk_count: int
    gpu_count: int
  software_summary:
    os: str
    python_version: str
    package_count: int
duration: float
metadata:
  output_dir: str
  file_size_bytes: int
```

**Example:**
```python
from skills.environment_profiler import create_environment_snapshot

# Create snapshot for release v1.0.0
result = create_environment_snapshot(
    snapshot_name="v1.0.0",
    output_dir="./snapshots",
    include_packages=True,
    filter_sensitive=True
)

if result.success:
    print(f"Snapshot saved: {result.data['snapshot_file']}")
    print(f"CPU: {result.data['hardware_summary']['cpu']}")
    print(f"Packages: {result.data['software_summary']['package_count']}")
```

---

### compare_environments

Compare two environment snapshots to identify hardware, software, and package differences.

**Input:**
```yaml
current_snapshot_file: str   # Path to current snapshot JSON file
baseline_snapshot_file: str  # Path to baseline snapshot JSON file
```

**Output:**
```yaml
success: bool
data:
  current_snapshot: str
  baseline_snapshot: str
  is_different: bool
  hardware_diff:
    has_differences: bool
    differences: list
      - component: str
        field: str
        current: any
        baseline: any
  software_diff:
    has_differences: bool
    os_changed: bool
    differences: list
  package_diff:
    added: list
      - name: str
        version: str
    removed: list
      - name: str
        version: str
    updated: list
      - name: str
        old_version: str
        new_version: str
    total_changes: int
duration: float
metadata:
  total_package_changes: int
  hardware_changed: bool
  os_changed: bool
```

**Example:**
```python
from skills.environment_profiler import compare_environments

result = compare_environments(
    current_snapshot_file="./snapshots/snapshot_v1.1.0_1234567890.json",
    baseline_snapshot_file="./snapshots/snapshot_v1.0.0_1234567880.json"
)

if result.success:
    diff = result.data
    print(f"Environments are different: {diff['is_different']}")

    if diff['package_diff']['total_changes'] > 0:
        print(f"\nPackage changes:")
        print(f"  Added: {len(diff['package_diff']['added'])}")
        print(f"  Removed: {len(diff['package_diff']['removed'])}")
        print(f"  Updated: {len(diff['package_diff']['updated'])}")

        for pkg in diff['package_diff']['updated']:
            print(f"    {pkg['name']}: {pkg['old_version']} → {pkg['new_version']}")
```

---

### export_package_requirements

Export installed packages to requirements.txt format.

**Input:**
```yaml
output_file: str          # Path to output file (default: "./requirements.txt")
include_versions: bool    # Include package versions (default: true)
```

**Output:**
```yaml
success: bool
data:
  requirements_file: str
  package_count: int
duration: float
metadata:
  include_versions: bool
  file_size_bytes: int
```

**Example:**
```python
from skills.environment_profiler import export_package_requirements

result = export_package_requirements(
    output_file="./requirements-v1.0.0.txt",
    include_versions=True
)

if result.success:
    print(f"Exported {result.data['package_count']} packages")
    print(f"File: {result.data['requirements_file']}")
```

---

## Integration with Other Skills

The environment-profiler skill integrates with:

- **release-orchestrator**: Provides environment context for release quality assessment
- **performance-profiler**: Correlates performance with hardware/software environment
- **dependency-guardian**: Cross-references package versions for security analysis
- **cicd-generator**: Generates environment validation steps in CI/CD pipelines

## Dependencies

**Required:**
- Python 3.8+
- Standard library: `platform`, `os`, `sys`, `subprocess`, `json`, `dataclasses`

**Optional:**
- `psutil`: For detailed hardware and resource information
  ```bash
  pip install psutil
  ```

**Note:** Without psutil, hardware detection will be limited to basic platform information.

## Snapshot File Structure

Environment snapshots are saved as JSON files:

```
./environment_snapshots/
├── snapshot_v1.0.0_1234567890.json
├── snapshot_v1.1.0_1234567891.json
└── snapshot_v1.2.0_1234567892.json
```

Each snapshot file contains:
```json
{
  "snapshot_name": "v1.0.0",
  "timestamp": 1234567890.0,
  "timestamp_iso": "2024-01-15 10:30:00",
  "hardware": {
    "cpu": {...},
    "memory": {...},
    "disks": [...],
    "gpus": [...],
    "network_interfaces": [...]
  },
  "software": {
    "os_info": {...},
    "python_info": {...},
    "installed_packages": [...],
    "environment_variables": {...},
    "system_libraries": {...}
  }
}
```

## Error Handling

All operations return `OperationResult` with standardized error codes:

- `HARDWARE_DETECTION_ERROR`: Error detecting hardware profile
- `SOFTWARE_DETECTION_ERROR`: Error detecting software profile
- `SNAPSHOT_CREATION_ERROR`: Error creating environment snapshot
- `COMPARISON_ERROR`: Error comparing environments
- `EXPORT_ERROR`: Error exporting package requirements

## Use Cases

### 1. Release Documentation
```python
# Document environment for each release
create_environment_snapshot(snapshot_name="v1.0.0")
export_package_requirements(output_file="requirements-v1.0.0.txt")
```

### 2. Performance Correlation
```python
# Identify if performance changes correlate with environment changes
comparison = compare_environments(
    current_snapshot_file="snapshot_v1.1.0.json",
    baseline_snapshot_file="snapshot_v1.0.0.json"
)

if comparison.data['package_diff']['updated']:
    print("Package updates may have affected performance:")
    for pkg in comparison.data['package_diff']['updated']:
        print(f"  {pkg['name']}: {pkg['old_version']} → {pkg['new_version']}")
```

### 3. Reproducibility Verification
```python
# Verify production environment matches expected snapshot
prod_snapshot = create_environment_snapshot(snapshot_name="production-current")
comparison = compare_environments(
    current_snapshot_file=prod_snapshot.data['snapshot_file'],
    baseline_snapshot_file="snapshot_v1.0.0.json"
)

if comparison.data['is_different']:
    print("⚠️ Production environment differs from release!")
```

### 4. CI/CD Environment Validation
```python
# Validate CI environment before running tests
hw = detect_hardware_profile()
sw = detect_software_profile()

# Check minimum requirements
if hw.data['memory']['total_gb'] < 4.0:
    print("⚠️ Insufficient memory for tests")

if sw.data['python_info']['version'] < "3.8":
    print("⚠️ Python version too old")
```

## Best Practices

1. **Snapshot Every Release**: Create environment snapshots for every release to maintain historical context
2. **Filter Sensitive Data**: Always use `filter_sensitive=True` when capturing environment variables
3. **Compare Before Deploy**: Compare production environment with release snapshot before deploying
4. **Track Dependencies**: Export requirements.txt for every release for reproducibility
5. **Monitor Hardware**: Track hardware changes that might affect performance benchmarks
6. **Version Snapshots**: Include version/release identifier in snapshot names for easy identification
7. **Automate in CI/CD**: Integrate snapshot creation into CI/CD pipelines for automatic tracking

## Version

0.1.0
