"""
PR Review Assistant Operations

Standardized operations interface for agent invocation.
"""

from pathlib import Path
from typing import Dict, Any, Optional, List
from dataclasses import dataclass
import time

# Import core functions
from .core.pr_analyzer import review_pull_request as _review_pull_request
from .core.comment_generator import generate_review_comment as _generate_review_comment


@dataclass
class OperationResult:
    """Result from a skill operation."""
    success: bool
    data: Optional[Dict[str, Any]] = None
    error: Optional[str] = None
    error_code: Optional[str] = None
    duration: float = 0.0
    metadata: Optional[Dict[str, Any]] = None


def review_pull_request(
    pr_changes: Dict[str, Any],
    base_branch: str = "main",
    target_branch: str = "feature",
    checklist: Optional[List[str]] = None,
    **kwargs
) -> OperationResult:
    """
    Perform comprehensive pull request review.

    Args:
        pr_changes: Dictionary with 'added', 'modified', 'deleted' file lists
        base_branch: Base branch name (default: main)
        target_branch: Target branch name (default: feature)
        checklist: Optional custom review checklist
        **kwargs: Additional parameters

    Returns:
        OperationResult with review results
    """
    start_time = time.time()

    try:
        result = _review_pull_request(
            pr_changes=pr_changes,
            base_branch=base_branch,
            target_branch=target_branch,
            checklist=checklist
        )

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=result,
            duration=duration,
            metadata={
                "skill": "pr-review-assistant",
                "operation": "review_pull_request",
                "version": "0.1.0",
                "base_branch": base_branch,
                "target_branch": target_branch,
                "files_reviewed": len(pr_changes.get('added', [])) + len(pr_changes.get('modified', []))
            }
        )

    except KeyError as e:
        return OperationResult(
            success=False,
            error=f"Invalid pr_changes format: missing key {str(e)}",
            error_code="VALIDATION_ERROR",
            duration=time.time() - start_time
        )
    except FileNotFoundError as e:
        return OperationResult(
            success=False,
            error=f"File not found: {str(e)}",
            error_code="FILE_NOT_FOUND",
            duration=time.time() - start_time
        )
    except Exception as e:
        return OperationResult(
            success=False,
            error=f"PR review failed: {str(e)}",
            error_code="REVIEW_ERROR",
            duration=time.time() - start_time
        )


def generate_review_comment(
    review_result: Dict[str, Any],
    format: str = "github",
    **kwargs
) -> OperationResult:
    """
    Generate formatted review comment from review results.

    Args:
        review_result: Result from review_pull_request operation
        format: Output format (github, gitlab, markdown)
        **kwargs: Additional parameters

    Returns:
        OperationResult with formatted comment
    """
    start_time = time.time()

    try:
        comment = _generate_review_comment(
            review_result=review_result,
            format=format
        )

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data={
                "comment": comment,
                "format": format,
                "length": len(comment),
                "overall_score": review_result.get('overall_score', 0)
            },
            duration=duration,
            metadata={
                "skill": "pr-review-assistant",
                "operation": "generate_review_comment",
                "version": "0.1.0",
                "format": format
            }
        )

    except ValueError as e:
        return OperationResult(
            success=False,
            error=f"Invalid format or review result: {str(e)}",
            error_code="VALIDATION_ERROR",
            duration=time.time() - start_time
        )
    except Exception as e:
        return OperationResult(
            success=False,
            error=f"Comment generation failed: {str(e)}",
            error_code="GENERATION_ERROR",
            duration=time.time() - start_time
        )


def analyze_change_impact(
    pr_changes: Dict[str, Any],
    **kwargs
) -> OperationResult:
    """
    Analyze the impact of PR changes.

    Args:
        pr_changes: Dictionary with 'added', 'modified', 'deleted' file lists
        **kwargs: Additional parameters

    Returns:
        OperationResult with impact analysis
    """
    start_time = time.time()

    try:
        # Extract file statistics
        added_files = pr_changes.get('added', [])
        modified_files = pr_changes.get('modified', [])
        deleted_files = pr_changes.get('deleted', [])

        # Calculate impact metrics
        total_files_changed = len(added_files) + len(modified_files) + len(deleted_files)

        # Categorize changes by file type
        file_types = {}
        for file in added_files + modified_files:
            ext = Path(file).suffix or 'no_extension'
            file_types[ext] = file_types.get(ext, 0) + 1

        # Assess risk level
        risk_level = "low"
        if total_files_changed > 20:
            risk_level = "high"
        elif total_files_changed > 10:
            risk_level = "medium"

        impact_data = {
            "total_files_changed": total_files_changed,
            "files_added": len(added_files),
            "files_modified": len(modified_files),
            "files_deleted": len(deleted_files),
            "file_types": file_types,
            "risk_level": risk_level,
            "recommendations": []
        }

        # Add recommendations based on changes
        if len(deleted_files) > 5:
            impact_data["recommendations"].append("Large number of deletions - verify no breaking changes")
        if total_files_changed > 15:
            impact_data["recommendations"].append("Large changeset - consider splitting into smaller PRs")
        if len(added_files) > len(modified_files) * 2:
            impact_data["recommendations"].append("Many new files - ensure proper documentation")

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=impact_data,
            duration=duration,
            metadata={
                "skill": "pr-review-assistant",
                "operation": "analyze_change_impact",
                "version": "0.1.0"
            }
        )

    except Exception as e:
        return OperationResult(
            success=False,
            error=f"Impact analysis failed: {str(e)}",
            error_code="ANALYSIS_ERROR",
            duration=time.time() - start_time
        )


def check_pr_quality(
    pr_changes: Dict[str, Any],
    include_tests: bool = True,
    include_security: bool = True,
    **kwargs
) -> OperationResult:
    """
    Quick quality check for PR changes.

    Args:
        pr_changes: Dictionary with 'added', 'modified', 'deleted' file lists
        include_tests: Check for test coverage (default: True)
        include_security: Check for security issues (default: True)
        **kwargs: Additional parameters

    Returns:
        OperationResult with quality check results
    """
    start_time = time.time()

    try:
        quality_checks = {
            "has_tests": False,
            "has_documentation": False,
            "follows_conventions": True,
            "security_concerns": []
        }

        all_files = pr_changes.get('added', []) + pr_changes.get('modified', [])

        # Check for tests
        if include_tests:
            test_files = [f for f in all_files if 'test' in f.lower() or f.endswith('_test.py') or f.endswith('.test.js')]
            quality_checks["has_tests"] = len(test_files) > 0
            quality_checks["test_files"] = test_files

        # Check for documentation
        doc_files = [f for f in all_files if f.endswith('.md') or 'doc' in f.lower() or f == 'README']
        quality_checks["has_documentation"] = len(doc_files) > 0

        # Basic security checks
        if include_security:
            for file in all_files:
                if any(keyword in file.lower() for keyword in ['password', 'secret', 'key', 'token', 'credential']):
                    quality_checks["security_concerns"].append(f"Potential sensitive data in: {file}")

        # Overall quality score
        score = 0
        if quality_checks["has_tests"]:
            score += 40
        if quality_checks["has_documentation"]:
            score += 30
        if quality_checks["follows_conventions"]:
            score += 20
        if len(quality_checks["security_concerns"]) == 0:
            score += 10

        quality_checks["overall_quality_score"] = score
        quality_checks["quality_level"] = "excellent" if score >= 90 else "good" if score >= 70 else "needs_improvement"

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=quality_checks,
            duration=duration,
            metadata={
                "skill": "pr-review-assistant",
                "operation": "check_pr_quality",
                "version": "0.1.0"
            }
        )

    except Exception as e:
        return OperationResult(
            success=False,
            error=f"Quality check failed: {str(e)}",
            error_code="CHECK_ERROR",
            duration=time.time() - start_time
        )


# Export all operations
__all__ = [
    "review_pull_request",
    "generate_review_comment",
    "analyze_change_impact",
    "check_pr_quality",
    "OperationResult"
]
