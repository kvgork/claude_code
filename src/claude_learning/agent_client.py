"""Main client for interacting with Claude Agent SDK."""

import asyncio
from typing import AsyncIterator, Optional, Dict, Any, List
from datetime import datetime
from pathlib import Path
import uuid

from claude_agent_sdk import query, ClaudeSDKClient, ClaudeAgentOptions

from .config import AgentConfig
from .models import AgentType, AgentResponse, LearningSession, LearningPhase


class AgentClient:
    """Client for interacting with Claude teaching agents using the Agent SDK."""

    def __init__(self, config: Optional[AgentConfig] = None):
        """
        Initialize the Agent Client.

        Args:
            config: Agent configuration. If None, loads from environment.
        """
        self.config = config or AgentConfig.from_env()
        self._active_sessions: Dict[str, LearningSession] = {}

    def _get_sdk_options(
        self,
        system_prompt: Optional[str] = None,
        custom_tools: Optional[List[str]] = None
    ) -> ClaudeAgentOptions:
        """Create SDK options from configuration."""
        return ClaudeAgentOptions(
            model=self.config.model,
            system_prompt=system_prompt,
            allowed_tools=custom_tools or self.config.allowed_tools,
            permission_mode=self.config.permission_mode,
        )

    async def query_agent(
        self,
        agent_type: AgentType,
        prompt: str,
        context: Optional[Dict[str, Any]] = None,
    ) -> AgentResponse:
        """
        Send a one-off query to a teaching agent.

        Args:
            agent_type: The type of agent to query
            prompt: The learning question or task
            context: Additional context for the query

        Returns:
            AgentResponse with the agent's teaching response
        """
        # Load agent's system prompt
        agent_prompt = self.config.load_agent_prompt(agent_type.value)

        # Add context to prompt if provided
        full_prompt = prompt
        if context:
            context_str = "\n\n**Context:**\n" + "\n".join(
                f"- {k}: {v}" for k, v in context.items()
            )
            full_prompt = f"{prompt}{context_str}"

        # Configure SDK options with agent's system prompt
        options = self._get_sdk_options(system_prompt=agent_prompt)

        # Collect response
        response_content = []
        tool_uses = []

        async for message in query(prompt=full_prompt, options=options):
            if isinstance(message, dict):
                if message.get("type") == "text":
                    response_content.append(message.get("content", ""))
                elif message.get("type") == "tool_use":
                    tool_uses.append(message)
            else:
                response_content.append(str(message))

        return AgentResponse(
            content="".join(response_content),
            agent_type=agent_type,
            timestamp=datetime.now(),
            metadata=context or {},
            tool_uses=tool_uses,
        )

    async def start_learning_session(
        self,
        topic: str,
        agent_type: AgentType = AgentType.LEARNING_COORDINATOR,
        initial_phase: Optional[LearningPhase] = None,
    ) -> LearningSession:
        """
        Start a new learning session with conversation context.

        Args:
            topic: The learning topic
            agent_type: The agent to work with (default: learning coordinator)
            initial_phase: Starting phase for the learning journey

        Returns:
            LearningSession object for tracking progress
        """
        session_id = str(uuid.uuid4())
        session = LearningSession(
            topic=topic,
            session_id=session_id,
            agent_type=agent_type,
            current_phase=initial_phase,
        )

        self._active_sessions[session_id] = session
        return session

    async def continue_session(
        self,
        session: LearningSession,
        prompt: str,
        update_phase: Optional[LearningPhase] = None,
    ) -> AgentResponse:
        """
        Continue an existing learning session.

        Args:
            session: The active learning session
            prompt: Next question or input
            update_phase: Update the current learning phase if provided

        Returns:
            AgentResponse from the agent
        """
        if update_phase:
            session.current_phase = update_phase

        # Build context from session history
        context = {
            "topic": session.topic,
            "session_id": session.session_id,
            "phase": session.current_phase.value if session.current_phase else "initial",
            "previous_interactions": len(session.messages),
        }

        response = await self.query_agent(
            agent_type=session.agent_type,
            prompt=prompt,
            context=context,
        )

        session.add_response(response)
        return response

    async def ask_specialist(
        self,
        agent_type: AgentType,
        question: str,
        context: Optional[Dict[str, Any]] = None,
    ) -> AgentResponse:
        """
        Ask a specific teaching specialist a question.

        Args:
            agent_type: The specialist to consult
            question: The question to ask
            context: Additional context

        Returns:
            AgentResponse with specialist's guidance
        """
        return await self.query_agent(
            agent_type=agent_type,
            prompt=question,
            context=context,
        )

    async def create_learning_plan(
        self,
        topic: str,
        student_context: Optional[Dict[str, Any]] = None,
    ) -> AgentResponse:
        """
        Create a comprehensive learning plan.

        Args:
            topic: What to learn
            student_context: Information about student background, goals, etc.

        Returns:
            AgentResponse containing the learning plan
        """
        prompt = f"Create a comprehensive learning plan for: {topic}"

        return await self.query_agent(
            agent_type=AgentType.PLAN_GENERATION_MENTOR,
            prompt=prompt,
            context=student_context,
        )

    async def check_understanding(
        self,
        topic: str,
        session: Optional[LearningSession] = None,
    ) -> AgentResponse:
        """
        Verify understanding of a topic through discussion.

        Args:
            topic: Topic to verify understanding of
            session: Optional active session for context

        Returns:
            AgentResponse with understanding check questions/discussion
        """
        context = {}
        if session:
            context = {
                "session_topic": session.topic,
                "current_phase": session.current_phase.value if session.current_phase else None,
                "learning_history": session.get_conversation_history()[-3:],  # Last 3 interactions
            }

        prompt = f"Check my understanding of: {topic}"

        return await self.query_agent(
            agent_type=AgentType.LEARNING_COORDINATOR,
            prompt=prompt,
            context=context,
        )

    def get_session(self, session_id: str) -> Optional[LearningSession]:
        """Retrieve an active session by ID."""
        return self._active_sessions.get(session_id)

    def list_active_sessions(self) -> List[LearningSession]:
        """List all active learning sessions."""
        return list(self._active_sessions.values())

    def close_session(self, session_id: str) -> None:
        """Mark a session as completed and archive it."""
        if session_id in self._active_sessions:
            session = self._active_sessions[session_id]
            session.completed = True
            # Could save to disk here
            del self._active_sessions[session_id]
