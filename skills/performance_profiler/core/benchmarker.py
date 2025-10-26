"""
Benchmark Suite Runner Module

Execute benchmark suites and compare results with baselines for regression detection.
"""

import time
import json
from pathlib import Path
from typing import Dict, List, Optional, Any, Callable
from dataclasses import dataclass, field, asdict
from datetime import datetime
import logging

logger = logging.getLogger(__name__)


@dataclass
class BenchmarkResult:
    """Result from a single benchmark."""
    benchmark_name: str
    release_version: str
    timestamp: float
    duration_ms: float
    iterations: int
    avg_time_ms: float
    min_time_ms: float
    max_time_ms: float
    std_dev_ms: float
    throughput_ops_sec: float = 0.0
    memory_peak_mb: float = 0.0
    cpu_avg_percent: float = 0.0
    metadata: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return asdict(self)


@dataclass
class RegressionAnalysis:
    """Analysis of performance regressions."""
    has_regression: bool
    regressed_benchmarks: List[Dict[str, Any]] = field(default_factory=list)
    improved_benchmarks: List[Dict[str, Any]] = field(default_factory=list)
    unchanged_benchmarks: List[str] = field(default_factory=list)
    overall_change_percent: float = 0.0

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return asdict(self)


@dataclass
class BenchmarkComparison:
    """Comparison between current and baseline results."""
    current_version: str
    baseline_version: str
    timestamp: float
    total_benchmarks: int
    comparison_details: List[Dict[str, Any]] = field(default_factory=list)
    regression_analysis: Optional[RegressionAnalysis] = None

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        data = asdict(self)
        if self.regression_analysis:
            data['regression_analysis'] = self.regression_analysis.to_dict()
        return data


class BenchmarkRunner:
    """
    Execute benchmark suites and compare with baselines.
    """

    def __init__(
        self,
        output_dir: str = "./benchmarks",
        regression_threshold: float = 0.05  # 5% threshold
    ):
        """
        Initialize benchmark runner.

        Args:
            output_dir: Directory to store benchmark results
            regression_threshold: Threshold for detecting regressions (e.g., 0.05 = 5%)
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.regression_threshold = regression_threshold
        self.results: List[BenchmarkResult] = []

        logger.info(f"BenchmarkRunner initialized, output dir: {output_dir}")
        logger.info(f"Regression threshold: {regression_threshold * 100}%")

    def run_benchmark_suite(
        self,
        benchmarks: List[Dict[str, Any]],
        release_version: str,
        iterations: int = 10,
        warmup_iterations: int = 3,
        track_resources: bool = True
    ) -> List[BenchmarkResult]:
        """
        Run a suite of benchmarks.

        Args:
            benchmarks: List of benchmark definitions
                       [{'name': str, 'func': callable, 'args': tuple, 'kwargs': dict}]
            release_version: Version identifier for this benchmark run
            iterations: Number of iterations per benchmark
            warmup_iterations: Number of warmup runs
            track_resources: Track CPU/memory during benchmarks

        Returns:
            List of BenchmarkResult objects
        """
        logger.info(
            f"Running benchmark suite for release {release_version}: "
            f"{len(benchmarks)} benchmarks, {iterations} iterations"
        )

        suite_results = []

        for benchmark in benchmarks:
            try:
                result = self._run_single_benchmark(
                    benchmark=benchmark,
                    release_version=release_version,
                    iterations=iterations,
                    warmup_iterations=warmup_iterations,
                    track_resources=track_resources
                )
                suite_results.append(result)
                logger.info(
                    f"Benchmark '{result.benchmark_name}': "
                    f"{result.avg_time_ms:.2f}ms avg"
                )
            except Exception as e:
                logger.error(f"Benchmark '{benchmark.get('name', 'unknown')}' failed: {e}")

        self.results.extend(suite_results)

        # Save results
        self._save_results(suite_results, release_version)

        logger.info(f"Benchmark suite completed: {len(suite_results)} successful")
        return suite_results

    def _run_single_benchmark(
        self,
        benchmark: Dict[str, Any],
        release_version: str,
        iterations: int,
        warmup_iterations: int,
        track_resources: bool
    ) -> BenchmarkResult:
        """Run a single benchmark."""
        name = benchmark['name']
        func = benchmark['func']
        args = benchmark.get('args', ())
        kwargs = benchmark.get('kwargs', {})

        logger.debug(f"Running benchmark: {name}")

        # Warmup runs
        for _ in range(warmup_iterations):
            try:
                func(*args, **kwargs)
            except Exception as e:
                logger.warning(f"Warmup error in {name}: {e}")

        # Actual benchmark runs
        times = []
        memory_samples = []
        cpu_samples = []

        for i in range(iterations):
            # Track resources if enabled
            if track_resources:
                try:
                    import psutil
                    process = psutil.Process()
                    mem_before = process.memory_info().rss / (1024 * 1024)
                    cpu_before = process.cpu_percent()
                except ImportError:
                    track_resources = False

            start = time.time()
            try:
                func(*args, **kwargs)
                elapsed = time.time() - start
                times.append(elapsed)
            except Exception as e:
                logger.error(f"Iteration {i} error in {name}: {e}")
                continue

            # Collect resource metrics
            if track_resources:
                try:
                    mem_after = process.memory_info().rss / (1024 * 1024)
                    cpu_after = process.cpu_percent()
                    memory_samples.append(mem_after - mem_before)
                    cpu_samples.append((cpu_before + cpu_after) / 2)
                except:
                    pass

        if not times:
            raise RuntimeError(f"All iterations failed for benchmark: {name}")

        # Calculate statistics
        avg_time = sum(times) / len(times)
        min_time = min(times)
        max_time = max(times)
        std_dev = self._calculate_std_dev(times)

        # Calculate throughput (ops/sec)
        throughput = 1.0 / avg_time if avg_time > 0 else 0.0

        # Resource metrics
        memory_peak = max(memory_samples) if memory_samples else 0.0
        cpu_avg = sum(cpu_samples) / len(cpu_samples) if cpu_samples else 0.0

        return BenchmarkResult(
            benchmark_name=name,
            release_version=release_version,
            timestamp=time.time(),
            duration_ms=sum(times) * 1000,
            iterations=len(times),
            avg_time_ms=avg_time * 1000,
            min_time_ms=min_time * 1000,
            max_time_ms=max_time * 1000,
            std_dev_ms=std_dev * 1000,
            throughput_ops_sec=throughput,
            memory_peak_mb=memory_peak,
            cpu_avg_percent=cpu_avg,
            metadata=benchmark.get('metadata', {})
        )

    def compare_with_baseline(
        self,
        current_results: List[BenchmarkResult],
        baseline_version: str
    ) -> BenchmarkComparison:
        """
        Compare current results with baseline.

        Args:
            current_results: Current benchmark results
            baseline_version: Version identifier for baseline

        Returns:
            BenchmarkComparison with detailed analysis
        """
        logger.info(f"Comparing with baseline version: {baseline_version}")

        # Load baseline results
        baseline_results = self._load_results(baseline_version)

        if not baseline_results:
            logger.warning(f"No baseline results found for version: {baseline_version}")
            return BenchmarkComparison(
                current_version=current_results[0].release_version if current_results else "unknown",
                baseline_version=baseline_version,
                timestamp=time.time(),
                total_benchmarks=len(current_results),
                comparison_details=[],
                regression_analysis=None
            )

        # Create lookup for baseline results
        baseline_lookup = {r.benchmark_name: r for r in baseline_results}

        comparison_details = []
        for current in current_results:
            baseline = baseline_lookup.get(current.benchmark_name)

            if baseline:
                comparison = self._compare_results(current, baseline)
                comparison_details.append(comparison)
            else:
                logger.warning(f"No baseline found for benchmark: {current.benchmark_name}")

        # Analyze regressions
        regression_analysis = self._analyze_regressions(comparison_details)

        return BenchmarkComparison(
            current_version=current_results[0].release_version,
            baseline_version=baseline_version,
            timestamp=time.time(),
            total_benchmarks=len(current_results),
            comparison_details=comparison_details,
            regression_analysis=regression_analysis
        )

    def _compare_results(
        self,
        current: BenchmarkResult,
        baseline: BenchmarkResult
    ) -> Dict[str, Any]:
        """Compare two benchmark results."""
        change_ms = current.avg_time_ms - baseline.avg_time_ms
        change_percent = (change_ms / baseline.avg_time_ms * 100) if baseline.avg_time_ms > 0 else 0

        throughput_change = current.throughput_ops_sec - baseline.throughput_ops_sec
        throughput_change_percent = (
            (throughput_change / baseline.throughput_ops_sec * 100)
            if baseline.throughput_ops_sec > 0 else 0
        )

        return {
            'benchmark_name': current.benchmark_name,
            'current_avg_ms': current.avg_time_ms,
            'baseline_avg_ms': baseline.avg_time_ms,
            'change_ms': change_ms,
            'change_percent': change_percent,
            'current_throughput': current.throughput_ops_sec,
            'baseline_throughput': baseline.throughput_ops_sec,
            'throughput_change_percent': throughput_change_percent,
            'current_memory_mb': current.memory_peak_mb,
            'baseline_memory_mb': baseline.memory_peak_mb,
            'is_regression': change_percent > (self.regression_threshold * 100),
            'is_improvement': change_percent < -(self.regression_threshold * 100)
        }

    def _analyze_regressions(
        self,
        comparison_details: List[Dict[str, Any]]
    ) -> RegressionAnalysis:
        """Analyze regressions from comparison details."""
        regressed = []
        improved = []
        unchanged = []

        for detail in comparison_details:
            if detail['is_regression']:
                regressed.append({
                    'benchmark': detail['benchmark_name'],
                    'change_percent': detail['change_percent'],
                    'change_ms': detail['change_ms']
                })
            elif detail['is_improvement']:
                improved.append({
                    'benchmark': detail['benchmark_name'],
                    'change_percent': detail['change_percent'],
                    'change_ms': detail['change_ms']
                })
            else:
                unchanged.append(detail['benchmark_name'])

        # Calculate overall change
        if comparison_details:
            overall_change = sum(d['change_percent'] for d in comparison_details) / len(comparison_details)
        else:
            overall_change = 0.0

        has_regression = len(regressed) > 0

        return RegressionAnalysis(
            has_regression=has_regression,
            regressed_benchmarks=regressed,
            improved_benchmarks=improved,
            unchanged_benchmarks=unchanged,
            overall_change_percent=overall_change
        )

    def _calculate_std_dev(self, values: List[float]) -> float:
        """Calculate standard deviation."""
        if len(values) < 2:
            return 0.0

        mean = sum(values) / len(values)
        variance = sum((x - mean) ** 2 for x in values) / (len(values) - 1)
        return variance ** 0.5

    def _save_results(
        self,
        results: List[BenchmarkResult],
        release_version: str
    ) -> None:
        """Save benchmark results to file."""
        filename = self.output_dir / f"benchmark_{release_version}_{int(time.time())}.json"

        data = {
            'release_version': release_version,
            'timestamp': time.time(),
            'timestamp_iso': datetime.fromtimestamp(time.time()).isoformat(),
            'total_benchmarks': len(results),
            'results': [r.to_dict() for r in results]
        }

        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)

        logger.info(f"Saved benchmark results to: {filename}")

    def _load_results(self, release_version: str) -> List[BenchmarkResult]:
        """Load benchmark results for a specific version."""
        # Find most recent results file for this version
        pattern = f"benchmark_{release_version}_*.json"
        files = sorted(self.output_dir.glob(pattern), reverse=True)

        if not files:
            logger.warning(f"No results found for version: {release_version}")
            return []

        latest_file = files[0]
        logger.debug(f"Loading baseline from: {latest_file}")

        try:
            with open(latest_file, 'r') as f:
                data = json.load(f)

            results = []
            for result_data in data.get('results', []):
                results.append(BenchmarkResult(**result_data))

            logger.info(f"Loaded {len(results)} baseline results from {latest_file}")
            return results

        except Exception as e:
            logger.error(f"Error loading results from {latest_file}: {e}")
            return []

    def export_comparison_report(
        self,
        comparison: BenchmarkComparison,
        output_file: str
    ) -> None:
        """
        Export comparison report to file.

        Args:
            comparison: BenchmarkComparison object
            output_file: Path to output file
        """
        report = {
            'comparison': comparison.to_dict(),
            'generated_at': datetime.now().isoformat(),
            'summary': {
                'has_regressions': comparison.regression_analysis.has_regression if comparison.regression_analysis else False,
                'regression_count': len(comparison.regression_analysis.regressed_benchmarks) if comparison.regression_analysis else 0,
                'improvement_count': len(comparison.regression_analysis.improved_benchmarks) if comparison.regression_analysis else 0,
                'overall_change_percent': comparison.regression_analysis.overall_change_percent if comparison.regression_analysis else 0
            }
        }

        output_path = Path(output_file)
        with open(output_path, 'w') as f:
            json.dump(report, f, indent=2)

        logger.info(f"Exported comparison report to: {output_path}")

    def get_summary(self) -> Dict[str, Any]:
        """Get summary of all benchmark runs."""
        return {
            'total_runs': len(self.results),
            'output_dir': str(self.output_dir),
            'regression_threshold': self.regression_threshold,
            'results': [r.to_dict() for r in self.results]
        }


# Export public API
__all__ = [
    'BenchmarkRunner',
    'BenchmarkResult',
    'BenchmarkComparison',
    'RegressionAnalysis'
]
