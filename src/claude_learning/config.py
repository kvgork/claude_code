"""Configuration management for Claude Learning System."""

import os
from pathlib import Path
from typing import Optional, List
from dataclasses import dataclass, field
from dotenv import load_dotenv

# Load environment variables
load_dotenv()


@dataclass
class AgentConfig:
    """Configuration for Claude Agent SDK."""

    api_key: Optional[str] = None
    model: str = "claude-sonnet-4-5-20250929"
    max_tokens: int = 8192
    temperature: float = 1.0
    allowed_tools: List[str] = field(
        default_factory=lambda: [
            "Read",
            "Write",
            "Edit",
            "Bash",
            "Glob",
            "Grep",
            "WebFetch",
            "WebSearch",
        ]
    )
    permission_mode: str = "ask"
    workspace_root: Path = field(default_factory=lambda: Path.cwd())
    agents_dir: Path = field(default_factory=lambda: Path.cwd() / "agents")
    commands_dir: Path = field(default_factory=lambda: Path.cwd() / "commands")
    plans_dir: Path = field(default_factory=lambda: Path.cwd() / "plans")

    def __post_init__(self) -> None:
        """Validate and initialize configuration."""
        if self.api_key is None:
            self.api_key = os.getenv("ANTHROPIC_API_KEY")
            if not self.api_key:
                raise ValueError(
                    "ANTHROPIC_API_KEY not found. Set it in .env or pass to AgentConfig"
                )

        # Ensure directories exist
        self.agents_dir.mkdir(parents=True, exist_ok=True)
        self.commands_dir.mkdir(parents=True, exist_ok=True)
        self.plans_dir.mkdir(parents=True, exist_ok=True)

    @classmethod
    def from_env(cls) -> "AgentConfig":
        """Create configuration from environment variables."""
        return cls(
            api_key=os.getenv("ANTHROPIC_API_KEY"),
            model=os.getenv("CLAUDE_MODEL", "claude-sonnet-4-5-20250929"),
            max_tokens=int(os.getenv("MAX_TOKENS", "8192")),
            temperature=float(os.getenv("TEMPERATURE", "1.0")),
            permission_mode=os.getenv("PERMISSION_MODE", "ask"),
        )

    def get_agent_path(self, agent_name: str) -> Path:
        """Get the file path for an agent definition."""
        return self.agents_dir / f"{agent_name}.md"

    def get_command_path(self, command_name: str) -> Path:
        """Get the file path for a command definition."""
        return self.commands_dir / f"{command_name}.md"

    def load_agent_prompt(self, agent_name: str) -> str:
        """Load an agent's system prompt from file."""
        agent_path = self.get_agent_path(agent_name)
        if not agent_path.exists():
            raise FileNotFoundError(f"Agent file not found: {agent_path}")
        return agent_path.read_text()

    def load_command_prompt(self, command_name: str) -> str:
        """Load a command's prompt from file."""
        command_path = self.get_command_path(command_name)
        if not command_path.exists():
            raise FileNotFoundError(f"Command file not found: {command_path}")
        return command_path.read_text()
