# Claude Code - Learning System

A comprehensive **teaching-first** command and agent system for Claude Code, designed for learning programming and robotics concepts through guided discovery.

## 🎯 Philosophy

This system treats you as a **learner**, not a code generator:
- **Guides** you through understanding concepts
- **Asks questions** to develop your thinking
- **Shows patterns** instead of complete solutions
- **Verifies understanding** before advancing
- **Coordinates specialists** for comprehensive learning

## 🚀 Quick Start

### Using Claude Code CLI

```bash
# Start learning something new
/start-learning autonomous navigation

# Resume your learning journey
/continue-plan

# Get help from a specialist
/ask-specialist How do ROS2 transforms work?

# Check your understanding
/check-understanding Strategy pattern

# Update your progress
/update-plan
```

### Using Python SDK

```bash
# Install dependencies
pip install -e .

# Set up environment
cp .env.example .env
# Edit .env and add your ANTHROPIC_API_KEY

# Run an example
python examples/basic_query.py
```

```python
# Programmatic usage
from claude_learning import AgentClient, AgentType

client = AgentClient.from_env()

response = await client.query_agent(
    agent_type=AgentType.ROS2_LEARNING_MENTOR,
    prompt="Explain ROS2 topics"
)
```

## 📚 What's Included

### 🗺️ Learning Plan Commands
- `/create-plan` - Generate educational implementation plans
- `/continue-plan` - Resume learning from where you left off
- `/update-plan` - Track progress and reflect on learning

### 🎓 Learning Session Commands
- `/start-learning` - Begin guided learning experience
- `/ask-specialist` - Connect with teaching specialists
- `/check-understanding` - Verify comprehension

### 🤖 ROS2 Commands
- `/learn-ros2-node` - Learn to create ROS2 nodes (teaching)
- `/new-node` - Generate ROS2 nodes (production)

### 🤖 12 Teaching Specialists

**Planning:** plan-generation-mentor
**Core Learning:** ros2-learning-mentor, python-best-practices, cpp-best-practices, code-architecture-mentor
**Domain Experts:** robotics-vision-navigator, jetank-hardware-specialist, debugging-detective, testing-specialist
**Dev Tools:** git-workflow-expert, documentation-generator

## 📖 Documentation

- **[COMMANDS_README.md](./COMMANDS_README.md)** - Complete command reference with examples
- **[docs/SDK_INTEGRATION.md](./docs/SDK_INTEGRATION.md)** - Python SDK integration guide
- **[project-context/project-context.md](./project-context/project-context.md)** - Claude Agent SDK migration plan
- **[agents/](./agents/)** - 12+ specialist teaching agents
- **[commands/](./commands/)** - Command implementations
- **[plans/](./plans/)** - Your generated learning plans
- **[examples/](./examples/)** - Python SDK usage examples

## 🌟 Key Features

### Learning Plans
- **Progressive phases**: Research → Design → Implement → Reflect
- **Understanding checkpoints**: Verify learning before advancing
- **Learning journal**: Document insights and challenges
- **Specialist mapping**: Know which experts help with what

### Teaching Approach
- ❌ **Won't** give you complete code solutions
- ✅ **Will** guide you through understanding
- ✅ **Will** teach concepts and patterns
- ✅ **Will** help you design your own solutions
- ✅ **Will** verify understanding at each step

### 12 Specialist Agents
Each specialized in teaching their domain:
- **ros2-learning-mentor** - ROS2 concepts
- **code-architecture-mentor** - 10 design patterns with situational guidance
- **robotics-vision-navigator** - Computer vision, SLAM, navigation
- **jetank-hardware-specialist** - Hardware integration with safety-first
- And 8 more specialists...

## 🎓 Example Learning Journey

```bash
# Day 1: Start ambitious project
/start-learning autonomous navigation
# → Creates 6-8 week learning plan
# → Phase 1: Understanding path planning algorithms

# Day 3: Get stuck on concept
/ask-specialist When to use A* vs RRT?
# → robotics-vision-navigator explains comparison
# → Teaches decision factors

# Day 7: Check progress
/update-plan
# → Document insights
# → Reflect on learning

# Day 10: Verify understanding
/check-understanding Phase 1
# → Discussion-based assessment
# → Cleared to proceed!

# Day 11: Continue journey
/continue-plan
# → Phase 2: System design begins
```

## 🏗️ Repository Structure

```
claude_code/
├── README.md                    # This file
├── COMMANDS_README.md           # Complete command documentation
├── pyproject.toml              # Python SDK package configuration
├── .env.example                # Environment configuration template
│
├── src/claude_learning/        # Python SDK integration
│   ├── __init__.py
│   ├── agent_client.py         # Main SDK wrapper
│   ├── config.py               # Configuration management
│   └── models.py               # Data models
│
├── examples/                    # Python SDK examples
│   ├── basic_query.py          # Simple agent query
│   ├── learning_session.py     # Multi-turn conversation
│   ├── ask_specialist.py       # Specialist consultation
│   ├── create_plan.py          # Generate learning plans
│   └── check_understanding.py  # Understanding verification
│
├── tests/                       # Test suite
│   ├── test_config.py
│   ├── test_models.py
│   └── test_agent_client.py
│
├── docs/                        # Documentation
│   ├── SDK_INTEGRATION.md      # Python SDK guide
│   └── PROJECT_PLANNING_SYSTEM.md
│
├── commands/                    # Slash commands
│   ├── create-plan.md
│   ├── continue-plan.md
│   ├── update-plan.md
│   ├── start-learning.md
│   ├── ask-specialist.md
│   └── check-understanding.md
│
├── agents/                      # 14 teaching specialists
│   ├── learning-coordinator.md
│   ├── plan-generation-mentor.md
│   ├── project-plan-orchestrator.md
│   ├── file-search-agent.md
│   ├── ros2-learning-mentor.md
│   ├── code-architecture-mentor.md
│   ├── robotics-vision-navigator.md
│   ├── jetank-hardware-specialist.md
│   ├── python-best-practices.md
│   ├── cpp-best-practices.md
│   ├── debugging-detective.md
│   ├── testing-specialist.md
│   ├── git-workflow-expert.md
│   └── documentation-generator.md
│
├── ros2/                        # ROS2-specific commands
│   ├── learn-ros2-node.md      # Teaching approach
│   ├── new-node.md             # Generation approach
│   ├── debug-node.md
│   └── test-integration.md
│
├── review/                      # Code review commands
│   ├── code-review.md
│   └── architecture-review.md
│
├── plans/                       # Your learning plans
│   └── (generated learning plans)
│
└── project-context/            # Codebase analysis & planning
    ├── README.md
    ├── project-context.md      # SDK migration plan
    └── relevant-files-*.md     # Generated codebase analysis
```

## 💡 Learning Tips

1. **Start with `/start-learning`** for new topics
2. **Use `/check-understanding`** at phase boundaries
3. **Update regularly** with `/update-plan`
4. **Ask questions** anytime with `/ask-specialist`
5. **Be patient** - deep understanding takes time
6. **Reflect** - use learning journal sections
7. **Practice** - implement what you learn

## 🎯 Best For

- Learning robotics and ROS2 development
- Understanding design patterns and architecture
- Building understanding, not just working code
- Structured multi-week learning journeys
- JETANK robot programming projects

## 🔧 Customization

All agents and commands are customizable:
- Edit agent files in `agents/` for teaching styles
- Modify commands in `commands/` for workflows
- Adjust learning-coordinator student profile
- Update plan-generation-mentor templates

## 📞 Getting Help

- `/ask-specialist [question]` - For specific help
- `/check-understanding [topic]` - To verify learning
- See `COMMANDS_README.md` for detailed command docs
- Review specialist agents in `agents/` folder

---

**Remember:** You're learning to build, not just using pre-built code! 🚀

Happy Learning! 🎓
