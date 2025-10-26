"""
Quality Scoring Module

Calculate multi-dimensional quality scores from various metrics.
"""

from typing import Dict, List, Optional, Any
from dataclasses import dataclass, asdict, field
import logging

logger = logging.getLogger(__name__)


@dataclass
class QualityDimension:
    """Individual quality dimension score."""
    name: str
    score: float  # 0-100
    weight: float  # Weight in overall score
    metrics: Dict[str, Any] = field(default_factory=dict)
    issues: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return asdict(self)


@dataclass
class QualityScore:
    """Complete quality assessment."""
    overall_score: float  # 0-100
    grade: str  # A+, A, B+, B, C, D, F
    dimensions: List[QualityDimension]
    timestamp: float
    release_version: str

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return {
            'overall_score': self.overall_score,
            'grade': self.grade,
            'dimensions': [d.to_dict() for d in self.dimensions],
            'timestamp': self.timestamp,
            'release_version': self.release_version
        }


class QualityScorer:
    """
    Calculate multi-dimensional quality scores.

    Default weights:
    - Code Quality: 25%
    - Test Quality: 20%
    - Dependencies: 20%
    - Documentation: 15%
    - Performance: 20%
    """

    def __init__(
        self,
        weights: Optional[Dict[str, float]] = None
    ):
        """
        Initialize quality scorer.

        Args:
            weights: Custom weights for each dimension
                    {dimension_name: weight}. Must sum to 1.0
        """
        self.weights = weights or {
            'code_quality': 0.25,
            'test_quality': 0.20,
            'dependencies': 0.20,
            'documentation': 0.15,
            'performance': 0.20
        }

        # Validate weights sum to 1.0
        total_weight = sum(self.weights.values())
        if abs(total_weight - 1.0) > 0.01:
            logger.warning(f"Weights sum to {total_weight}, normalizing to 1.0")
            self.weights = {k: v / total_weight for k, v in self.weights.items()}

        logger.info(f"QualityScorer initialized with weights: {self.weights}")

    def calculate_code_quality_score(
        self,
        code_analysis: Dict[str, Any]
    ) -> QualityDimension:
        """
        Calculate code quality score.

        Metrics:
        - Complexity (cyclomatic complexity, cognitive complexity)
        - Code smells
        - Maintainability index
        - Duplication ratio

        Args:
            code_analysis: Results from refactor-assistant or similar

        Returns:
            QualityDimension with code quality score
        """
        logger.debug("Calculating code quality score")

        score = 100.0
        issues = []
        metrics = {}

        # Complexity penalty
        avg_complexity = code_analysis.get('avg_cyclomatic_complexity', 5)
        metrics['avg_complexity'] = avg_complexity
        if avg_complexity > 10:
            penalty = min((avg_complexity - 10) * 2, 20)
            score -= penalty
            issues.append(f"High average complexity: {avg_complexity:.1f}")

        # Code smells penalty
        code_smells = code_analysis.get('code_smells', 0)
        metrics['code_smells'] = code_smells
        if code_smells > 10:
            penalty = min(code_smells / 2, 20)
            score -= penalty
            issues.append(f"Code smells detected: {code_smells}")

        # Maintainability index (0-100, higher is better)
        maintainability = code_analysis.get('maintainability_index', 80)
        metrics['maintainability_index'] = maintainability
        if maintainability < 60:
            penalty = (60 - maintainability) / 2
            score -= penalty
            issues.append(f"Low maintainability: {maintainability:.1f}")

        # Duplication ratio penalty
        duplication_ratio = code_analysis.get('duplication_ratio', 0)
        metrics['duplication_ratio'] = duplication_ratio
        if duplication_ratio > 0.05:  # More than 5% duplication
            penalty = min(duplication_ratio * 100, 15)
            score -= penalty
            issues.append(f"Code duplication: {duplication_ratio*100:.1f}%")

        score = max(0, min(100, score))

        return QualityDimension(
            name='code_quality',
            score=score,
            weight=self.weights['code_quality'],
            metrics=metrics,
            issues=issues
        )

    def calculate_test_quality_score(
        self,
        test_analysis: Dict[str, Any]
    ) -> QualityDimension:
        """
        Calculate test quality score.

        Metrics:
        - Test coverage
        - Test success rate
        - Test performance
        - Test count

        Args:
            test_analysis: Results from test-orchestrator

        Returns:
            QualityDimension with test quality score
        """
        logger.debug("Calculating test quality score")

        score = 100.0
        issues = []
        metrics = {}

        # Coverage score (0-100, higher is better)
        coverage = test_analysis.get('coverage_percent', 0)
        metrics['coverage'] = coverage
        if coverage < 80:
            penalty = (80 - coverage) / 2
            score -= penalty
            issues.append(f"Low test coverage: {coverage:.1f}%")

        # Test success rate
        success_rate = test_analysis.get('success_rate', 100)
        metrics['success_rate'] = success_rate
        if success_rate < 100:
            penalty = (100 - success_rate) * 2
            score -= penalty
            issues.append(f"Failing tests: {100 - success_rate:.1f}%")

        # Test count (ensure adequate tests)
        test_count = test_analysis.get('total_tests', 0)
        metrics['test_count'] = test_count
        if test_count < 10:
            penalty = (10 - test_count) * 2
            score -= penalty
            issues.append(f"Insufficient tests: {test_count}")

        # Test performance (penalize very slow tests)
        avg_test_duration = test_analysis.get('avg_test_duration_ms', 100)
        metrics['avg_test_duration_ms'] = avg_test_duration
        if avg_test_duration > 1000:  # Slower than 1 second
            penalty = min((avg_test_duration - 1000) / 1000 * 5, 10)
            score -= penalty
            issues.append(f"Slow tests: {avg_test_duration:.0f}ms average")

        score = max(0, min(100, score))

        return QualityDimension(
            name='test_quality',
            score=score,
            weight=self.weights['test_quality'],
            metrics=metrics,
            issues=issues
        )

    def calculate_dependency_score(
        self,
        dependency_analysis: Dict[str, Any]
    ) -> QualityDimension:
        """
        Calculate dependency quality score.

        Metrics:
        - Vulnerability count
        - Outdated packages
        - License compliance
        - Dependency freshness

        Args:
            dependency_analysis: Results from dependency-guardian

        Returns:
            QualityDimension with dependency score
        """
        logger.debug("Calculating dependency score")

        score = 100.0
        issues = []
        metrics = {}

        # Vulnerability penalty
        vulnerabilities = dependency_analysis.get('vulnerabilities', 0)
        critical_vulns = dependency_analysis.get('critical_vulnerabilities', 0)
        metrics['vulnerabilities'] = vulnerabilities
        metrics['critical_vulnerabilities'] = critical_vulns

        if critical_vulns > 0:
            penalty = critical_vulns * 15
            score -= penalty
            issues.append(f"Critical vulnerabilities: {critical_vulns}")
        if vulnerabilities > 0:
            penalty = vulnerabilities * 5
            score -= penalty
            issues.append(f"Vulnerabilities: {vulnerabilities}")

        # Outdated packages penalty
        outdated = dependency_analysis.get('outdated_packages', 0)
        metrics['outdated_packages'] = outdated
        if outdated > 5:
            penalty = min((outdated - 5) * 1.5, 15)
            score -= penalty
            issues.append(f"Outdated packages: {outdated}")

        # License compliance
        license_issues = dependency_analysis.get('license_issues', 0)
        metrics['license_issues'] = license_issues
        if license_issues > 0:
            penalty = license_issues * 10
            score -= penalty
            issues.append(f"License issues: {license_issues}")

        score = max(0, min(100, score))

        return QualityDimension(
            name='dependencies',
            score=score,
            weight=self.weights['dependencies'],
            metrics=metrics,
            issues=issues
        )

    def calculate_documentation_score(
        self,
        doc_analysis: Dict[str, Any]
    ) -> QualityDimension:
        """
        Calculate documentation quality score.

        Metrics:
        - Documentation coverage
        - README completeness
        - API documentation
        - Comment ratio

        Args:
            doc_analysis: Results from doc-generator or similar

        Returns:
            QualityDimension with documentation score
        """
        logger.debug("Calculating documentation score")

        score = 100.0
        issues = []
        metrics = {}

        # Documentation coverage (functions/classes documented)
        doc_coverage = doc_analysis.get('doc_coverage_percent', 50)
        metrics['doc_coverage'] = doc_coverage
        if doc_coverage < 70:
            penalty = (70 - doc_coverage) / 2
            score -= penalty
            issues.append(f"Low documentation coverage: {doc_coverage:.1f}%")

        # README completeness
        readme_score = doc_analysis.get('readme_score', 50)
        metrics['readme_score'] = readme_score
        if readme_score < 70:
            penalty = (70 - readme_score) / 3
            score -= penalty
            issues.append(f"Incomplete README: {readme_score:.1f}/100")

        # API documentation
        api_doc_coverage = doc_analysis.get('api_doc_coverage', 50)
        metrics['api_doc_coverage'] = api_doc_coverage
        if api_doc_coverage < 80:
            penalty = (80 - api_doc_coverage) / 4
            score -= penalty
            issues.append(f"Low API documentation: {api_doc_coverage:.1f}%")

        # Comment ratio
        comment_ratio = doc_analysis.get('comment_ratio', 0.1)
        metrics['comment_ratio'] = comment_ratio
        if comment_ratio < 0.15:  # Less than 15% comments
            penalty = (0.15 - comment_ratio) * 100
            score -= penalty
            issues.append(f"Low comment ratio: {comment_ratio*100:.1f}%")

        score = max(0, min(100, score))

        return QualityDimension(
            name='documentation',
            score=score,
            weight=self.weights['documentation'],
            metrics=metrics,
            issues=issues
        )

    def calculate_performance_score(
        self,
        performance_analysis: Dict[str, Any]
    ) -> QualityDimension:
        """
        Calculate performance score.

        Metrics:
        - Benchmark regressions
        - Resource usage
        - Response times
        - Throughput

        Args:
            performance_analysis: Results from performance-profiler

        Returns:
            QualityDimension with performance score
        """
        logger.debug("Calculating performance score")

        score = 100.0
        issues = []
        metrics = {}

        # Performance regressions
        regression_count = performance_analysis.get('regression_count', 0)
        metrics['regression_count'] = regression_count
        if regression_count > 0:
            penalty = regression_count * 10
            score -= penalty
            issues.append(f"Performance regressions: {regression_count}")

        # Average performance change
        avg_change_percent = performance_analysis.get('avg_change_percent', 0)
        metrics['avg_change_percent'] = avg_change_percent
        if avg_change_percent > 5:  # More than 5% slower
            penalty = min(avg_change_percent - 5, 20)
            score -= penalty
            issues.append(f"Performance degradation: +{avg_change_percent:.1f}%")

        # Memory usage increase
        memory_change_percent = performance_analysis.get('memory_change_percent', 0)
        metrics['memory_change_percent'] = memory_change_percent
        if memory_change_percent > 10:  # More than 10% memory increase
            penalty = min((memory_change_percent - 10) / 2, 15)
            score -= penalty
            issues.append(f"Memory increase: +{memory_change_percent:.1f}%")

        # Throughput decrease
        throughput_change = performance_analysis.get('throughput_change_percent', 0)
        metrics['throughput_change_percent'] = throughput_change
        if throughput_change < -5:  # More than 5% throughput decrease
            penalty = min(abs(throughput_change + 5), 15)
            score -= penalty
            issues.append(f"Throughput decrease: {throughput_change:.1f}%")

        score = max(0, min(100, score))

        return QualityDimension(
            name='performance',
            score=score,
            weight=self.weights['performance'],
            metrics=metrics,
            issues=issues
        )

    def calculate_overall_score(
        self,
        dimensions: List[QualityDimension]
    ) -> float:
        """
        Calculate weighted overall score from dimensions.

        Args:
            dimensions: List of QualityDimension objects

        Returns:
            Overall score (0-100)
        """
        overall = 0.0
        for dimension in dimensions:
            contribution = dimension.score * dimension.weight
            overall += contribution
            logger.debug(
                f"{dimension.name}: {dimension.score:.1f} * {dimension.weight} = {contribution:.1f}"
            )

        logger.info(f"Overall quality score: {overall:.1f}")
        return round(overall, 1)

    def get_grade(self, score: float) -> str:
        """
        Convert score to letter grade.

        Args:
            score: Quality score (0-100)

        Returns:
            Letter grade (A+, A, B+, B, C, D, F)
        """
        if score >= 97:
            return 'A+'
        elif score >= 93:
            return 'A'
        elif score >= 90:
            return 'A-'
        elif score >= 87:
            return 'B+'
        elif score >= 83:
            return 'B'
        elif score >= 80:
            return 'B-'
        elif score >= 77:
            return 'C+'
        elif score >= 73:
            return 'C'
        elif score >= 70:
            return 'C-'
        elif score >= 67:
            return 'D+'
        elif score >= 63:
            return 'D'
        elif score >= 60:
            return 'D-'
        else:
            return 'F'

    def assess_quality(
        self,
        release_version: str,
        code_analysis: Optional[Dict[str, Any]] = None,
        test_analysis: Optional[Dict[str, Any]] = None,
        dependency_analysis: Optional[Dict[str, Any]] = None,
        doc_analysis: Optional[Dict[str, Any]] = None,
        performance_analysis: Optional[Dict[str, Any]] = None,
        timestamp: Optional[float] = None
    ) -> QualityScore:
        """
        Perform complete quality assessment.

        Args:
            release_version: Version identifier
            code_analysis: Code quality metrics
            test_analysis: Test quality metrics
            dependency_analysis: Dependency metrics
            doc_analysis: Documentation metrics
            performance_analysis: Performance metrics
            timestamp: Assessment timestamp

        Returns:
            QualityScore with complete assessment
        """
        import time
        timestamp = timestamp or time.time()

        logger.info(f"Assessing quality for release: {release_version}")

        dimensions = []

        # Calculate each dimension if data provided
        if code_analysis:
            dimensions.append(self.calculate_code_quality_score(code_analysis))

        if test_analysis:
            dimensions.append(self.calculate_test_quality_score(test_analysis))

        if dependency_analysis:
            dimensions.append(self.calculate_dependency_score(dependency_analysis))

        if doc_analysis:
            dimensions.append(self.calculate_documentation_score(doc_analysis))

        if performance_analysis:
            dimensions.append(self.calculate_performance_score(performance_analysis))

        # Calculate overall score
        overall = self.calculate_overall_score(dimensions)
        grade = self.get_grade(overall)

        quality_score = QualityScore(
            overall_score=overall,
            grade=grade,
            dimensions=dimensions,
            timestamp=timestamp,
            release_version=release_version
        )

        logger.info(f"Quality assessment complete: {overall:.1f} ({grade})")
        return quality_score


# Export public API
__all__ = [
    'QualityScorer',
    'QualityScore',
    'QualityDimension'
]
