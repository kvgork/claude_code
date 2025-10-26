"""
Release Quality Report Generator

Generate comprehensive release quality reports in various formats.
"""

import json
from pathlib import Path
from typing import Dict, List, Optional, Any
from datetime import datetime
import logging

logger = logging.getLogger(__name__)


class ReportGenerator:
    """
    Generate release quality reports in various formats.
    """

    def __init__(self, output_dir: str = "./reports"):
        """
        Initialize report generator.

        Args:
            output_dir: Directory to store generated reports
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        logger.info(f"ReportGenerator initialized, output dir: {output_dir}")

    def generate_json_report(
        self,
        assessment: Any,  # ReleaseAssessment
        output_file: Optional[str] = None
    ) -> str:
        """
        Generate JSON report.

        Args:
            assessment: ReleaseAssessment object
            output_file: Optional output file path

        Returns:
            Path to generated report file
        """
        if output_file is None:
            timestamp = int(assessment.timestamp)
            output_file = self.output_dir / f"release_report_{assessment.release_version}_{timestamp}.json"
        else:
            output_file = Path(output_file)

        report = {
            'report_type': 'release_quality_assessment',
            'generated_at': datetime.fromtimestamp(assessment.timestamp).isoformat(),
            'release_version': assessment.release_version,
            'quality_gate': {
                'passed': assessment.passed_quality_gate,
                'score': assessment.overall_quality_score,
                'grade': assessment.grade
            },
            'assessment': assessment.to_dict()
        }

        with open(output_file, 'w') as f:
            json.dump(report, f, indent=2)

        logger.info(f"JSON report generated: {output_file}")
        return str(output_file)

    def generate_markdown_report(
        self,
        assessment: Any,  # ReleaseAssessment
        output_file: Optional[str] = None
    ) -> str:
        """
        Generate Markdown report.

        Args:
            assessment: ReleaseAssessment object
            output_file: Optional output file path

        Returns:
            Path to generated report file
        """
        if output_file is None:
            timestamp = int(assessment.timestamp)
            output_file = self.output_dir / f"release_report_{assessment.release_version}_{timestamp}.md"
        else:
            output_file = Path(output_file)

        md = self._build_markdown_report(assessment)

        with open(output_file, 'w') as f:
            f.write(md)

        logger.info(f"Markdown report generated: {output_file}")
        return str(output_file)

    def _build_markdown_report(self, assessment: Any) -> str:
        """Build markdown content for report."""
        lines = []

        # Header
        lines.append(f"# Release Quality Report: {assessment.release_version}")
        lines.append("")
        lines.append(f"**Generated:** {datetime.fromtimestamp(assessment.timestamp).strftime('%Y-%m-%d %H:%M:%S')}")
        lines.append("")

        # Quality Gate Status
        status_emoji = "✅" if assessment.passed_quality_gate else "❌"
        lines.append(f"## Quality Gate: {status_emoji} {'PASSED' if assessment.passed_quality_gate else 'FAILED'}")
        lines.append("")
        lines.append(f"- **Overall Score:** {assessment.overall_quality_score:.1f}/100")
        lines.append(f"- **Grade:** {assessment.grade}")
        lines.append("")

        # Warnings (if any)
        if assessment.warnings:
            lines.append("## ⚠️ Warnings")
            lines.append("")
            for warning in assessment.warnings:
                lines.append(f"- {warning}")
            lines.append("")

        # Quality Dimensions
        if assessment.quality_dimensions:
            lines.append("## Quality Dimensions")
            lines.append("")
            lines.append("| Dimension | Score | Weight | Status |")
            lines.append("|-----------|-------|--------|--------|")

            for dim in assessment.quality_dimensions:
                score = dim['score']
                weight_pct = dim['weight'] * 100
                status = "✓" if score >= 70 else "✗"
                lines.append(
                    f"| {dim['name'].replace('_', ' ').title()} | "
                    f"{score:.1f}/100 | {weight_pct:.0f}% | {status} |"
                )
            lines.append("")

            # Dimension Details
            for dim in assessment.quality_dimensions:
                lines.append(f"### {dim['name'].replace('_', ' ').title()}")
                lines.append("")
                lines.append(f"**Score:** {dim['score']:.1f}/100")
                lines.append("")

                if dim.get('metrics'):
                    lines.append("**Metrics:**")
                    lines.append("")
                    for key, value in dim['metrics'].items():
                        if isinstance(value, float):
                            lines.append(f"- {key.replace('_', ' ').title()}: {value:.2f}")
                        else:
                            lines.append(f"- {key.replace('_', ' ').title()}: {value}")
                    lines.append("")

                if dim.get('issues'):
                    lines.append("**Issues:**")
                    lines.append("")
                    for issue in dim['issues']:
                        lines.append(f"- {issue}")
                    lines.append("")

        # Skill Results
        lines.append("## Skill Execution Results")
        lines.append("")
        lines.append("| Skill | Operation | Status | Duration |")
        lines.append("|-------|-----------|--------|----------|")

        for skill_name, result in assessment.skill_results.items():
            status_icon = "✓" if result.success else "✗"
            duration_str = f"{result.duration:.2f}s"
            lines.append(
                f"| {skill_name} | {result.operation_name} | "
                f"{status_icon} | {duration_str} |"
            )
        lines.append("")

        # Performance Details (if available)
        if 'performance' in assessment.skill_results:
            perf_result = assessment.skill_results['performance']
            if perf_result.success and perf_result.data:
                self._add_performance_section(lines, perf_result.data)

        # Environment Details (if available)
        if 'environment' in assessment.skill_results:
            env_result = assessment.skill_results['environment']
            if env_result.success and env_result.data:
                self._add_environment_section(lines, env_result.data)

        # Recommendations
        if assessment.recommendations:
            lines.append("## 📋 Recommendations")
            lines.append("")
            for i, rec in enumerate(assessment.recommendations, 1):
                lines.append(f"{i}. {rec}")
            lines.append("")

        # Footer
        lines.append("---")
        lines.append("")
        lines.append("*Report generated by Release Orchestrator*")
        lines.append("")

        return "\n".join(lines)

    def _add_performance_section(self, lines: List[str], perf_data: Dict[str, Any]):
        """Add performance details to report."""
        lines.append("## Performance Analysis")
        lines.append("")

        comparison = perf_data.get('comparison')
        if comparison and 'regression_analysis' in comparison:
            analysis = comparison['regression_analysis']

            lines.append(f"**Overall Change:** {analysis['overall_change_percent']:+.2f}%")
            lines.append("")

            # Regressed benchmarks
            if analysis.get('regressed_benchmarks'):
                lines.append("### ⚠️ Performance Regressions")
                lines.append("")
                lines.append("| Benchmark | Change | Delta |")
                lines.append("|-----------|--------|-------|")

                for reg in analysis['regressed_benchmarks']:
                    lines.append(
                        f"| {reg['benchmark']} | "
                        f"+{reg['change_percent']:.2f}% | "
                        f"+{reg['change_ms']:.2f}ms |"
                    )
                lines.append("")

            # Improved benchmarks
            if analysis.get('improved_benchmarks'):
                lines.append("### ✓ Performance Improvements")
                lines.append("")
                lines.append("| Benchmark | Change | Delta |")
                lines.append("|-----------|--------|-------|")

                for imp in analysis['improved_benchmarks']:
                    lines.append(
                        f"| {imp['benchmark']} | "
                        f"{imp['change_percent']:.2f}% | "
                        f"{imp['change_ms']:.2f}ms |"
                    )
                lines.append("")

        # Benchmark results
        benchmarks = perf_data.get('benchmarks')
        if benchmarks and 'results' in benchmarks:
            lines.append("### Benchmark Results")
            lines.append("")
            lines.append("| Benchmark | Avg Time | Throughput |")
            lines.append("|-----------|----------|------------|")

            for result in benchmarks['results'][:10]:  # Show top 10
                lines.append(
                    f"| {result['benchmark_name']} | "
                    f"{result['avg_time_ms']:.3f}ms | "
                    f"{result['throughput_ops_sec']:.1f} ops/s |"
                )
            lines.append("")

    def _add_environment_section(self, lines: List[str], env_data: Dict[str, Any]):
        """Add environment details to report."""
        lines.append("## Environment Profile")
        lines.append("")

        snapshot = env_data.get('snapshot')
        if snapshot:
            hw_summary = snapshot.get('hardware_summary', {})
            sw_summary = snapshot.get('software_summary', {})

            lines.append("### Hardware")
            lines.append("")
            lines.append(f"- **CPU:** {hw_summary.get('cpu', 'Unknown')}")
            lines.append(f"- **Memory:** {hw_summary.get('memory_gb', 0):.1f} GB")
            lines.append(f"- **Disks:** {hw_summary.get('disk_count', 0)}")
            lines.append(f"- **GPUs:** {hw_summary.get('gpu_count', 0)}")
            lines.append("")

            lines.append("### Software")
            lines.append("")
            lines.append(f"- **OS:** {sw_summary.get('os', 'Unknown')}")
            lines.append(f"- **Python:** {sw_summary.get('python_version', 'Unknown')}")
            lines.append(f"- **Packages:** {sw_summary.get('package_count', 0)}")
            lines.append("")

        # Environment comparison
        comparison = env_data.get('comparison')
        if comparison and comparison.get('is_different'):
            lines.append("### Environment Changes")
            lines.append("")

            pkg_diff = comparison.get('package_diff', {})
            if pkg_diff.get('total_changes', 0) > 0:
                lines.append(f"**Package Changes:** {pkg_diff['total_changes']}")
                lines.append("")

                if pkg_diff.get('added'):
                    lines.append(f"- Added: {len(pkg_diff['added'])}")
                if pkg_diff.get('removed'):
                    lines.append(f"- Removed: {len(pkg_diff['removed'])}")
                if pkg_diff.get('updated'):
                    lines.append(f"- Updated: {len(pkg_diff['updated'])}")
                    lines.append("")

                    if pkg_diff['updated']:
                        lines.append("**Updated Packages:**")
                        lines.append("")
                        for pkg in pkg_diff['updated'][:10]:  # Show top 10
                            lines.append(
                                f"- {pkg['name']}: {pkg['old_version']} → {pkg['new_version']}"
                            )
                        lines.append("")

    def generate_summary_report(
        self,
        assessments: List[Any],  # List of ReleaseAssessment
        output_file: Optional[str] = None
    ) -> str:
        """
        Generate summary report comparing multiple releases.

        Args:
            assessments: List of ReleaseAssessment objects
            output_file: Optional output file path

        Returns:
            Path to generated report file
        """
        if output_file is None:
            output_file = self.output_dir / f"release_summary_{int(datetime.now().timestamp())}.md"
        else:
            output_file = Path(output_file)

        lines = []

        # Header
        lines.append("# Release Quality Summary")
        lines.append("")
        lines.append(f"**Generated:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        lines.append(f"**Releases:** {len(assessments)}")
        lines.append("")

        # Quality Trend
        lines.append("## Quality Trend")
        lines.append("")
        lines.append("| Release | Score | Grade | Quality Gate | Date |")
        lines.append("|---------|-------|-------|--------------|------|")

        for assessment in sorted(assessments, key=lambda a: a.timestamp):
            status = "✅ PASS" if assessment.passed_quality_gate else "❌ FAIL"
            date = datetime.fromtimestamp(assessment.timestamp).strftime('%Y-%m-%d')
            lines.append(
                f"| {assessment.release_version} | "
                f"{assessment.overall_quality_score:.1f} | "
                f"{assessment.grade} | {status} | {date} |"
            )
        lines.append("")

        # Latest Assessment Details
        if assessments:
            latest = assessments[-1]
            lines.append("## Latest Release Details")
            lines.append("")
            lines.append(f"**Version:** {latest.release_version}")
            lines.append(f"**Score:** {latest.overall_quality_score:.1f}/100 ({latest.grade})")
            lines.append("")

            if latest.warnings:
                lines.append("**Warnings:**")
                lines.append("")
                for warning in latest.warnings:
                    lines.append(f"- {warning}")
                lines.append("")

            if latest.recommendations:
                lines.append("**Recommendations:**")
                lines.append("")
                for rec in latest.recommendations:
                    lines.append(f"- {rec}")
                lines.append("")

        md = "\n".join(lines)

        with open(output_file, 'w') as f:
            f.write(md)

        logger.info(f"Summary report generated: {output_file}")
        return str(output_file)


# Export public API
__all__ = [
    'ReportGenerator'
]
