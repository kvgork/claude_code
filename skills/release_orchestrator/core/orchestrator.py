"""
Release Quality Orchestrator

Coordinates multiple skills to perform comprehensive release quality assessment.
"""

import time
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, asdict
import logging

logger = logging.getLogger(__name__)


@dataclass
class SkillResult:
    """Result from executing a skill operation."""
    skill_name: str
    operation_name: str
    success: bool
    data: Optional[Dict[str, Any]] = None
    error: Optional[str] = None
    duration: float = 0.0

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return asdict(self)


@dataclass
class ReleaseAssessment:
    """Complete release quality assessment."""
    release_version: str
    timestamp: float
    overall_quality_score: float
    grade: str
    skill_results: Dict[str, SkillResult]
    quality_dimensions: List[Dict[str, Any]]
    recommendations: List[str]
    warnings: List[str]
    passed_quality_gate: bool

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return {
            'release_version': self.release_version,
            'timestamp': self.timestamp,
            'overall_quality_score': self.overall_quality_score,
            'grade': self.grade,
            'skill_results': {k: v.to_dict() for k, v in self.skill_results.items()},
            'quality_dimensions': self.quality_dimensions,
            'recommendations': self.recommendations,
            'warnings': self.warnings,
            'passed_quality_gate': self.passed_quality_gate
        }


class ReleaseOrchestrator:
    """
    Orchestrate multiple skills to assess release quality.

    Coordinates:
    - performance-profiler: Performance benchmarks and regression detection
    - environment-profiler: Environment snapshot and comparison
    - refactor-assistant: Code quality analysis
    - test-orchestrator: Test execution and coverage
    - dependency-guardian: Security and dependency analysis
    - doc-generator: Documentation quality
    """

    def __init__(
        self,
        quality_gate_threshold: float = 70.0,
        enable_performance: bool = True,
        enable_environment: bool = True,
        enable_code_quality: bool = False,  # Requires existing skills
        enable_tests: bool = False,  # Requires existing skills
        enable_dependencies: bool = False,  # Requires existing skills
        enable_docs: bool = False  # Requires existing skills
    ):
        """
        Initialize release orchestrator.

        Args:
            quality_gate_threshold: Minimum quality score to pass (0-100)
            enable_performance: Enable performance profiling
            enable_environment: Enable environment profiling
            enable_code_quality: Enable code quality analysis
            enable_tests: Enable test analysis
            enable_dependencies: Enable dependency analysis
            enable_docs: Enable documentation analysis
        """
        self.quality_gate_threshold = quality_gate_threshold
        self.enabled_assessments = {
            'performance': enable_performance,
            'environment': enable_environment,
            'code_quality': enable_code_quality,
            'tests': enable_tests,
            'dependencies': enable_dependencies,
            'docs': enable_docs
        }

        logger.info(f"ReleaseOrchestrator initialized (threshold: {quality_gate_threshold})")
        logger.info(f"Enabled assessments: {[k for k, v in self.enabled_assessments.items() if v]}")

    def assess_release(
        self,
        release_version: str,
        project_path: str,
        baseline_version: Optional[str] = None,
        benchmark_suite: Optional[List[Dict[str, Any]]] = None
    ) -> ReleaseAssessment:
        """
        Perform comprehensive release quality assessment.

        Args:
            release_version: Version identifier for this release
            project_path: Path to project root
            baseline_version: Baseline version for comparison
            benchmark_suite: Optional benchmark suite to run

        Returns:
            ReleaseAssessment with complete assessment results
        """
        logger.info(f"Starting release assessment for: {release_version}")
        start_time = time.time()

        skill_results = {}

        # 1. Environment Assessment
        if self.enabled_assessments['environment']:
            env_result = self._assess_environment(
                release_version=release_version,
                baseline_version=baseline_version
            )
            skill_results['environment'] = env_result

        # 2. Performance Assessment
        if self.enabled_assessments['performance']:
            perf_result = self._assess_performance(
                release_version=release_version,
                baseline_version=baseline_version,
                benchmark_suite=benchmark_suite
            )
            skill_results['performance'] = perf_result

        # 3. Code Quality Assessment (if enabled)
        if self.enabled_assessments['code_quality']:
            code_result = self._assess_code_quality(project_path)
            skill_results['code_quality'] = code_result

        # 4. Test Assessment (if enabled)
        if self.enabled_assessments['tests']:
            test_result = self._assess_tests(project_path)
            skill_results['tests'] = test_result

        # 5. Dependency Assessment (if enabled)
        if self.enabled_assessments['dependencies']:
            dep_result = self._assess_dependencies(project_path)
            skill_results['dependencies'] = dep_result

        # 6. Documentation Assessment (if enabled)
        if self.enabled_assessments['docs']:
            doc_result = self._assess_documentation(project_path)
            skill_results['docs'] = doc_result

        # Calculate overall quality score
        quality_score, quality_dimensions = self._calculate_quality_score(skill_results)

        # Get grade
        from .quality_scorer import QualityScorer
        scorer = QualityScorer()
        grade = scorer.get_grade(quality_score)

        # Generate recommendations and warnings
        recommendations = self._generate_recommendations(skill_results, quality_dimensions)
        warnings = self._generate_warnings(skill_results, quality_dimensions)

        # Determine if quality gate passed
        passed_quality_gate = quality_score >= self.quality_gate_threshold

        assessment = ReleaseAssessment(
            release_version=release_version,
            timestamp=time.time(),
            overall_quality_score=quality_score,
            grade=grade,
            skill_results=skill_results,
            quality_dimensions=quality_dimensions,
            recommendations=recommendations,
            warnings=warnings,
            passed_quality_gate=passed_quality_gate
        )

        duration = time.time() - start_time
        logger.info(
            f"Release assessment complete ({duration:.1f}s): "
            f"{quality_score:.1f} ({grade}) - "
            f"{'PASSED' if passed_quality_gate else 'FAILED'}"
        )

        return assessment

    def _assess_environment(
        self,
        release_version: str,
        baseline_version: Optional[str]
    ) -> SkillResult:
        """Assess environment using environment-profiler skill."""
        logger.info("Assessing environment...")
        start_time = time.time()

        try:
            # Import environment-profiler operations
            from skills.environment_profiler import create_environment_snapshot

            # Create snapshot for this release
            result = create_environment_snapshot(
                snapshot_name=release_version,
                include_packages=True,
                filter_sensitive=True
            )

            if not result.success:
                return SkillResult(
                    skill_name='environment-profiler',
                    operation_name='create_environment_snapshot',
                    success=False,
                    error=result.error,
                    duration=time.time() - start_time
                )

            # Compare with baseline if provided
            comparison_data = None
            if baseline_version:
                from skills.environment_profiler import compare_environments
                from pathlib import Path

                # Find baseline snapshot file
                snapshot_dir = Path("./environment_snapshots")
                baseline_files = sorted(
                    snapshot_dir.glob(f"snapshot_{baseline_version}_*.json"),
                    reverse=True
                )

                if baseline_files:
                    comparison = compare_environments(
                        current_snapshot_file=result.data['snapshot_file'],
                        baseline_snapshot_file=str(baseline_files[0])
                    )
                    if comparison.success:
                        comparison_data = comparison.data

            return SkillResult(
                skill_name='environment-profiler',
                operation_name='create_environment_snapshot',
                success=True,
                data={
                    'snapshot': result.data,
                    'comparison': comparison_data
                },
                duration=time.time() - start_time
            )

        except Exception as e:
            logger.error(f"Environment assessment error: {e}")
            return SkillResult(
                skill_name='environment-profiler',
                operation_name='create_environment_snapshot',
                success=False,
                error=str(e),
                duration=time.time() - start_time
            )

    def _assess_performance(
        self,
        release_version: str,
        baseline_version: Optional[str],
        benchmark_suite: Optional[List[Dict[str, Any]]]
    ) -> SkillResult:
        """Assess performance using performance-profiler skill."""
        logger.info("Assessing performance...")
        start_time = time.time()

        try:
            from skills.performance_profiler import run_benchmark_suite

            if not benchmark_suite:
                logger.warning("No benchmark suite provided, skipping performance assessment")
                return SkillResult(
                    skill_name='performance-profiler',
                    operation_name='run_benchmark_suite',
                    success=False,
                    error="No benchmark suite provided",
                    duration=time.time() - start_time
                )

            # Run benchmark suite
            result = run_benchmark_suite(
                benchmarks=benchmark_suite,
                release_version=release_version,
                iterations=10,
                warmup_iterations=3,
                track_resources=True
            )

            if not result.success:
                return SkillResult(
                    skill_name='performance-profiler',
                    operation_name='run_benchmark_suite',
                    success=False,
                    error=result.error,
                    duration=time.time() - start_time
                )

            # Compare with baseline if provided
            comparison_data = None
            if baseline_version:
                from skills.performance_profiler import compare_with_baseline
                from pathlib import Path

                # Find current results file
                benchmark_dir = Path("./benchmarks")
                current_files = sorted(
                    benchmark_dir.glob(f"benchmark_{release_version}_*.json"),
                    reverse=True
                )

                if current_files:
                    comparison = compare_with_baseline(
                        current_results_file=str(current_files[0]),
                        baseline_version=baseline_version
                    )
                    if comparison.success:
                        comparison_data = comparison.data

            return SkillResult(
                skill_name='performance-profiler',
                operation_name='run_benchmark_suite',
                success=True,
                data={
                    'benchmarks': result.data,
                    'comparison': comparison_data
                },
                duration=time.time() - start_time
            )

        except Exception as e:
            logger.error(f"Performance assessment error: {e}")
            return SkillResult(
                skill_name='performance-profiler',
                operation_name='run_benchmark_suite',
                success=False,
                error=str(e),
                duration=time.time() - start_time
            )

    def _assess_code_quality(self, project_path: str) -> SkillResult:
        """Assess code quality using refactor-assistant skill."""
        logger.info("Code quality assessment placeholder")
        # Placeholder for refactor-assistant integration
        return SkillResult(
            skill_name='refactor-assistant',
            operation_name='analyze_code_quality',
            success=False,
            error="Not implemented - requires refactor-assistant integration",
            duration=0.0
        )

    def _assess_tests(self, project_path: str) -> SkillResult:
        """Assess tests using test-orchestrator skill."""
        logger.info("Test assessment placeholder")
        # Placeholder for test-orchestrator integration
        return SkillResult(
            skill_name='test-orchestrator',
            operation_name='run_tests',
            success=False,
            error="Not implemented - requires test-orchestrator integration",
            duration=0.0
        )

    def _assess_dependencies(self, project_path: str) -> SkillResult:
        """Assess dependencies using dependency-guardian skill."""
        logger.info("Dependency assessment placeholder")
        # Placeholder for dependency-guardian integration
        return SkillResult(
            skill_name='dependency-guardian',
            operation_name='check_vulnerabilities',
            success=False,
            error="Not implemented - requires dependency-guardian integration",
            duration=0.0
        )

    def _assess_documentation(self, project_path: str) -> SkillResult:
        """Assess documentation using doc-generator skill."""
        logger.info("Documentation assessment placeholder")
        # Placeholder for doc-generator integration
        return SkillResult(
            skill_name='doc-generator',
            operation_name='analyze_documentation',
            success=False,
            error="Not implemented - requires doc-generator integration",
            duration=0.0
        )

    def _calculate_quality_score(
        self,
        skill_results: Dict[str, SkillResult]
    ) -> tuple[float, List[Dict[str, Any]]]:
        """Calculate overall quality score from skill results."""
        from .quality_scorer import QualityScorer

        scorer = QualityScorer()
        dimensions = []

        # Extract metrics from each successful skill result

        # Performance metrics
        if 'performance' in skill_results and skill_results['performance'].success:
            perf_data = skill_results['performance'].data
            comparison = perf_data.get('comparison')

            if comparison and 'regression_analysis' in comparison:
                analysis = comparison['regression_analysis']
                perf_metrics = {
                    'regression_count': len(analysis.get('regressed_benchmarks', [])),
                    'avg_change_percent': analysis.get('overall_change_percent', 0),
                    'memory_change_percent': 0,  # Not available in current data
                    'throughput_change_percent': 0  # Not available in current data
                }
                dimensions.append(scorer.calculate_performance_score(perf_metrics))

        # Environment doesn't contribute to quality score directly
        # But could be used for consistency checks

        # Calculate overall score
        if dimensions:
            overall_score = scorer.calculate_overall_score(dimensions)
        else:
            # Default score if no dimensions available
            overall_score = 100.0
            logger.warning("No quality dimensions calculated, using default score: 100.0")

        return overall_score, [d.to_dict() for d in dimensions]

    def _generate_recommendations(
        self,
        skill_results: Dict[str, SkillResult],
        quality_dimensions: List[Dict[str, Any]]
    ) -> List[str]:
        """Generate recommendations based on assessment results."""
        recommendations = []

        # Performance recommendations
        if 'performance' in skill_results and skill_results['performance'].success:
            perf_data = skill_results['performance'].data
            comparison = perf_data.get('comparison')

            if comparison and 'regression_analysis' in comparison:
                analysis = comparison['regression_analysis']
                if analysis.get('has_regression'):
                    regressed = analysis.get('regressed_benchmarks', [])
                    recommendations.append(
                        f"Investigate {len(regressed)} performance regression(s) before release"
                    )

        # Environment recommendations
        if 'environment' in skill_results and skill_results['environment'].success:
            env_data = skill_results['environment'].data
            comparison = env_data.get('comparison')

            if comparison and comparison.get('is_different'):
                pkg_diff = comparison.get('package_diff', {})
                if pkg_diff.get('total_changes', 0) > 10:
                    recommendations.append(
                        f"Review {pkg_diff['total_changes']} package changes for compatibility"
                    )

        # Quality dimension recommendations
        for dimension in quality_dimensions:
            if dimension['score'] < 70:
                issues = dimension.get('issues', [])
                if issues:
                    recommendations.append(
                        f"Address {dimension['name']} issues: {', '.join(issues[:2])}"
                    )

        return recommendations

    def _generate_warnings(
        self,
        skill_results: Dict[str, SkillResult],
        quality_dimensions: List[Dict[str, Any]]
    ) -> List[str]:
        """Generate warnings based on assessment results."""
        warnings = []

        # Check for failed skills
        for skill_name, result in skill_results.items():
            if not result.success:
                warnings.append(f"{skill_name} assessment failed: {result.error}")

        # Check for critical quality issues
        for dimension in quality_dimensions:
            if dimension['score'] < 50:
                warnings.append(
                    f"CRITICAL: {dimension['name']} score very low ({dimension['score']:.1f})"
                )

        # Performance warnings
        if 'performance' in skill_results and skill_results['performance'].success:
            perf_data = skill_results['performance'].data
            comparison = perf_data.get('comparison')

            if comparison and 'regression_analysis' in comparison:
                analysis = comparison['regression_analysis']
                regressed = analysis.get('regressed_benchmarks', [])

                for reg in regressed:
                    if reg.get('change_percent', 0) > 50:
                        warnings.append(
                            f"CRITICAL: {reg['benchmark']} regressed by {reg['change_percent']:.1f}%"
                        )

        return warnings


# Export public API
__all__ = [
    'ReleaseOrchestrator',
    'ReleaseAssessment',
    'SkillResult'
]
