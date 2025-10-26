"""
Git Workflow Assistant Operations

Standardized operations interface for agent invocation.
"""

from pathlib import Path
from typing import Dict, Any, Optional
from dataclasses import dataclass
import time

# Import core functions
from .core.change_analyzer import analyze_changes as _analyze_changes
from .core.commit_generator import generate_commit_message as _generate_commit_message
from .core.branch_manager import suggest_branch_name as _suggest_branch_name
from .core.pr_generator import create_pull_request as _create_pull_request


@dataclass
class OperationResult:
    """Result from a skill operation."""
    success: bool
    data: Optional[Dict[str, Any]] = None
    error: Optional[str] = None
    error_code: Optional[str] = None
    duration: float = 0.0
    metadata: Optional[Dict[str, Any]] = None


def analyze_changes(
    repo_path: str = ".",
    include_unstaged: bool = False,
    **kwargs
) -> OperationResult:
    """
    Analyze staged and unstaged changes in a git repository.

    Args:
        repo_path: Path to git repository (default: current directory)
        include_unstaged: Include unstaged changes in analysis
        **kwargs: Additional parameters

    Returns:
        OperationResult with change analysis
    """
    start_time = time.time()

    try:
        result = _analyze_changes(
            repo_path=repo_path,
            include_unstaged=include_unstaged
        )

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=result,
            duration=duration,
            metadata={
                "skill": "git-workflow-assistant",
                "operation": "analyze_changes",
                "version": "0.1.0",
                "include_unstaged": include_unstaged
            }
        )

    except FileNotFoundError as e:
        return OperationResult(
            success=False,
            error=f"Repository not found: {str(e)}",
            error_code="REPO_NOT_FOUND",
            duration=time.time() - start_time
        )
    except Exception as e:
        return OperationResult(
            success=False,
            error=f"Change analysis failed: {str(e)}",
            error_code="ANALYSIS_ERROR",
            duration=time.time() - start_time
        )


def generate_commit_message(
    repo_path: str = ".",
    commit_type: Optional[str] = None,
    scope: Optional[str] = None,
    breaking: bool = False,
    **kwargs
) -> OperationResult:
    """
    Generate conventional commit message from staged changes.

    Args:
        repo_path: Path to git repository (default: current directory)
        commit_type: Optional commit type override (feat, fix, docs, etc.)
        scope: Optional commit scope
        breaking: Is this a breaking change
        **kwargs: Additional parameters

    Returns:
        OperationResult with generated commit message
    """
    start_time = time.time()

    try:
        result = _generate_commit_message(
            repo_path=repo_path,
            commit_type=commit_type,
            scope=scope,
            breaking=breaking
        )

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=result,
            duration=duration,
            metadata={
                "skill": "git-workflow-assistant",
                "operation": "generate_commit_message",
                "version": "0.1.0",
                "commit_type": commit_type,
                "breaking": breaking
            }
        )

    except FileNotFoundError as e:
        return OperationResult(
            success=False,
            error=f"Repository not found: {str(e)}",
            error_code="REPO_NOT_FOUND",
            duration=time.time() - start_time
        )
    except ValueError as e:
        return OperationResult(
            success=False,
            error=f"No staged changes found: {str(e)}",
            error_code="NO_CHANGES",
            duration=time.time() - start_time
        )
    except Exception as e:
        return OperationResult(
            success=False,
            error=f"Commit message generation failed: {str(e)}",
            error_code="GENERATION_ERROR",
            duration=time.time() - start_time
        )


def suggest_branch_name(
    description: str = "",
    branch_type: str = "feature",
    issue_number: Optional[str] = None,
    strategy: str = "gitflow",
    **kwargs
) -> OperationResult:
    """
    Suggest branch name following conventions.

    Args:
        description: Description of the work
        branch_type: Type of branch (feature, bugfix, hotfix, release)
        issue_number: Optional issue/ticket number
        strategy: Branching strategy (gitflow, github-flow, gitlab-flow)
        **kwargs: Additional parameters

    Returns:
        OperationResult with suggested branch name
    """
    start_time = time.time()

    try:
        result = _suggest_branch_name(
            issue_number=issue_number,
            description=description,
            branch_type=branch_type,
            strategy=strategy
        )

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=result,
            duration=duration,
            metadata={
                "skill": "git-workflow-assistant",
                "operation": "suggest_branch_name",
                "version": "0.1.0",
                "branch_type": branch_type
            }
        )

    except ValueError as e:
        return OperationResult(
            success=False,
            error=f"Invalid branch parameters: {str(e)}",
            error_code="VALIDATION_ERROR",
            duration=time.time() - start_time
        )
    except Exception as e:
        return OperationResult(
            success=False,
            error=f"Branch name suggestion failed: {str(e)}",
            error_code="SUGGESTION_ERROR",
            duration=time.time() - start_time
        )


def create_pull_request(
    repo_path: str = ".",
    base_branch: str = "main",
    head_branch: Optional[str] = None,
    **kwargs
) -> OperationResult:
    """
    Create pull request with generated description.

    Args:
        repo_path: Path to git repository (default: current directory)
        base_branch: Base branch for PR (default: main)
        head_branch: Head branch (default: current branch)
        **kwargs: Additional parameters

    Returns:
        OperationResult with PR details
    """
    start_time = time.time()

    try:
        result = _create_pull_request(
            repo_path=repo_path,
            base_branch=base_branch,
            head_branch=head_branch
        )

        duration = time.time() - start_time

        return OperationResult(
            success=True,
            data=result,
            duration=duration,
            metadata={
                "skill": "git-workflow-assistant",
                "operation": "create_pull_request",
                "version": "0.1.0",
                "base_branch": base_branch
            }
        )

    except FileNotFoundError as e:
        return OperationResult(
            success=False,
            error=f"Repository not found: {str(e)}",
            error_code="REPO_NOT_FOUND",
            duration=time.time() - start_time
        )
    except ValueError as e:
        return OperationResult(
            success=False,
            error=f"Invalid PR parameters: {str(e)}",
            error_code="VALIDATION_ERROR",
            duration=time.time() - start_time
        )
    except Exception as e:
        return OperationResult(
            success=False,
            error=f"PR creation failed: {str(e)}",
            error_code="CREATION_ERROR",
            duration=time.time() - start_time
        )


# Export all operations
__all__ = [
    "analyze_changes",
    "generate_commit_message",
    "suggest_branch_name",
    "create_pull_request",
    "OperationResult"
]
