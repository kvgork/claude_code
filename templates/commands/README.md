# Robotics Daily Improvement Slash Commands

This directory contains slash command templates for the robotics daily improvement agent system.

## Available Commands

### `/start-daily-improvement`

**Purpose**: Start a new 30-day robotics learning plan with personalized daily micro-assignments.

**Usage**:
```
/start-daily-improvement
```

Or with a specific topic:
```
/start-daily-improvement ROS2 fundamentals
/start-daily-improvement computer vision
/start-daily-improvement robot control
```

**What it does**:
1. Conducts personalized assessment (background, goals, resources, learning style)
2. Creates customized 30-day plan with daily 15-30 minute assignments
3. Sets up progress tracking log file
4. Provides Day 1 assignment to start immediately

**Output files**:
- `learning-plans/robotics-improvement-plan-[DATE].md` - Your complete curriculum
- `learning-logs/robotics-log-[MONTH].md` - Progress tracking journal

---

### `/continue-daily-improvement`

**Purpose**: Create the next 30-day plan (Days 31-60) that builds on your completed plan.

**Usage**:
```
/continue-daily-improvement
```

**What it does**:
1. Reviews your progress from Days 1-30
2. Analyzes what you've mastered and struggled with
3. Creates Days 31-60 plan with advanced topics
4. Builds on established foundations
5. Includes integration projects combining multiple skills

**When to use**:
- After completing (or mostly completing) your first 30 days
- When ready to level up to advanced concepts
- To maintain momentum and continue growth

**Can be used multiple times**:
- First use: Creates Days 31-60
- Second use: Creates Days 61-90
- Continue as long as you want to keep improving!

**Output files**:
- `learning-plans/robotics-improvement-plan-continuation-[DATE].md` - Advanced curriculum
- `learning-logs/robotics-log-[NEW-MONTH].md` - Continuation tracking

---

## Installation

To use these commands in your Claude Code environment:

1. **Copy to your local commands directory**:
   ```bash
   cp templates/commands/start-daily-improvement.md ~/.claude/commands/
   cp templates/commands/continue-daily-improvement.md ~/.claude/commands/
   ```

   Or if using project-local commands:
   ```bash
   mkdir -p .claude/commands
   cp templates/commands/start-daily-improvement.md .claude/commands/
   cp templates/commands/continue-daily-improvement.md .claude/commands/
   ```

2. **Verify installation**:
   - Type `/` in Claude Code to see available commands
   - You should see `/start-daily-improvement` and `/continue-daily-improvement` in the list

3. **Start using**:
   ```
   /start-daily-improvement
   ```

---

## The 1% Daily Improvement Philosophy

These commands are based on the principle that **1% better every day = 37x better in a year**.

**Formula**: `1.01^365 = 37.78`

### Why It Works

- **Consistency beats intensity**: 30 min daily > 3 hours weekly
- **Compound growth**: Each day builds on previous learning
- **Habit formation**: Daily practice becomes automatic
- **Sustainable pace**: Prevents burnout, enables long-term growth

### What You'll Achieve

**After 30 days**:
- ✅ Solid foundation in chosen robotics topic
- ✅ Working projects and demos
- ✅ Daily learning habit established
- ✅ 1.01^30 = 1.35 → **35% better** than Day 1

**After 60 days** (using `/continue-daily-improvement`):
- ✅ Advanced understanding and integration skills
- ✅ Complex multi-component projects
- ✅ 1.01^60 = 1.82 → **82% better** than Day 1

**After 90 days** (second continuation):
- ✅ Mastery-level work
- ✅ Production-ready projects
- ✅ 1.01^90 = 2.45 → **145% better** than Day 1

---

## Documentation

For comprehensive usage instructions, see:
- **User Guide**: `docs/ROBOTICS_DAILY_IMPROVEMENT_GUIDE.md`
- **Agent Reference**: `docs/AGENTS_REFERENCE.md`
- **Agent Definition**: `agents/robotics-daily-improvement.md`

---

## Examples

### Example 1: Complete Beginner Learning ROS2

```
User: /start-daily-improvement ROS2

Agent: Conducts assessment...

Creates:
- 30-day plan: Nodes → Topics → Services → Navigation
- Daily assignments: 30 minutes each
- Progress log with tracking templates

User completes 28/30 days over 5 weeks

User: /continue-daily-improvement

Agent: Reviews progress, creates Days 31-60 with:
- Multi-robot systems
- Advanced navigation
- Sensor fusion
- Complex integration projects
```

### Example 2: Intermediate Learning Computer Vision

```
User: /start-daily-improvement computer vision for robotics

Agent: Assessment reveals intermediate Python, basic robotics

Creates:
- 30-day plan: OpenCV → Feature detection → Deep learning → ROS2 integration
- Hands-on assignments building toward object detection
- Emphasis on practical implementation

User completes all 30 days

User: /continue-daily-improvement

Agent: Creates advanced plan with:
- Custom model training
- Real-time optimization
- Multi-camera systems
- Production deployment
```

---

## Integration with Learning System

These commands work with the broader learning agent ecosystem:

### When You Get Stuck

The daily improvement agent coordinates with specialists:

```
Day 12: Stuck on ROS2 transforms

robotics-daily-improvement agent:
  ↓ Routes to
learning-coordinator:
  ↓ Delegates to
ros2-learning-mentor:
  → Teaches transform concepts (guides, doesn't solve)

You: Return to complete Day 12 with understanding
```

### Available Specialists

- **ros2-learning-mentor**: ROS2 concepts
- **robotics-vision-navigator**: Vision and SLAM
- **python-best-practices**: Code quality
- **debugging-detective**: Systematic debugging
- **code-architecture-mentor**: Design patterns
- **testing-specialist**: Testing strategies

---

## Teaching Philosophy

These commands create **learning journeys**, not tutorials:

### ❌ What They DON'T Do
- Provide complete code solutions
- Solve assignments for you
- Give copy-paste ready answers
- Make all decisions for you

### ✅ What They DO
- Guide discovery and exploration
- Ask thought-provoking questions
- Provide structure and direction
- Offer hints and resources
- Celebrate your progress
- Adapt to your pace

**Why?** Because you learn more by figuring things out than being given answers!

---

## Troubleshooting

### "Commands don't appear in / menu"

Check installation:
```bash
ls ~/.claude/commands/start-daily-improvement.md
ls ~/.claude/commands/continue-daily-improvement.md
```

If files don't exist, copy from templates again.

### "Agent asks me to provide previous plan"

For `/continue-daily-improvement`, you need:
- A completed (or partially completed) first 30-day plan
- The plan file location
- Your progress log

If starting fresh, use `/start-daily-improvement` instead.

### "Assignments too hard/easy"

Tell the agent during assessment or anytime:
```
"These assignments are too [hard/easy], can we adjust?"
```

The plan adapts based on your feedback.

---

## Contributing

To improve these commands:

1. Edit the template files in `templates/commands/`
2. Test your changes locally
3. Submit PR with improvements
4. Update this README if adding new commands

---

**Ready to get 1% better every day?**

Start now: `/start-daily-improvement` 🚀
