"""
Git Workflow Assistant Skill

Intelligent git workflow automation including commit messages, branch management,
and PR creation.
"""

from .core.change_analyzer import analyze_changes
from .core.commit_generator import generate_commit_message
from .core.branch_manager import suggest_branch_name
from .core.pr_generator import create_pull_request

__all__ = [
    "analyze_changes",
    "generate_commit_message",
    "suggest_branch_name",
    "create_pull_request",
]

__version__ = "0.1.0"
