# Robotics Skill Evaluation Templates

Templates for evaluating student skill levels before creating learning plans.

## Purpose

These templates help the **robotics-skill-evaluator** agent conduct comprehensive, honest assessments that reveal:
- Actual knowledge (not self-reported)
- Specific gaps to address
- Appropriate starting level
- Custom plan adjustments needed

## Available Templates

### 1. ROS2 Evaluation (`ros2-evaluation-template.md`)
- 10 conceptual questions across 4 levels
- Code reading test
- Recommended starting points
- Gap identification guide

### 2. Computer Vision Evaluation (`computer-vision-evaluation-template.md`)
- 10 conceptual questions across 4 levels
- Code reading test
- Practical challenge
- Gap identification guide

### 3. Robot Control Evaluation (Coming soon)
### 4. SLAM Evaluation (Coming soon)

## How to Use

### For the Evaluator Agent

1. **Select appropriate template** based on student's chosen topic
2. **Ask questions** from the template (adapt wording naturally)
3. **Score responses** using the rubric provided
4. **Determine level** based on scoring matrix
5. **Identify gaps** using gap identification section
6. **Return structured results** to daily improvement agent

### For Creating New Templates

Each template should include:

#### 1. Leveled Questions
- **Beginner Check** (Q1-Q2): Can they explain basic concepts?
- **Novice Check** (Q3-Q5): Do they understand application?
- **Advanced Beginner Check** (Q6-Q8): Can they compare/design?
- **Intermediate Check** (Q9-Q10): Do they know advanced topics?

#### 2. Code Reading Test
- Show realistic code snippet
- Ask comprehension questions
- Test if they can read code at appropriate level

#### 3. Practical Challenge (Optional)
- Small task to attempt
- Reveals problem-solving approach
- Confirms assessed level

#### 4. Level Determination
- Scoring matrix
- Clear criteria for each level
- Handles edge cases

#### 5. Recommended Starting Points
- Specific Day 1 recommendations for each level
- Pace recommendations
- Content to skip/emphasize

#### 6. Gap Identification
- Maps failed questions to specific topics
- Suggests when/how to address in plan
- Enables targeted learning

## Evaluation Philosophy

### Test Understanding, Not Memory

**❌ Bad Question**:
> "What is a ROS2 node?"

Memorized answer: "An independent process."

**✅ Good Question**:
> "Explain why ROS2 uses separate nodes instead of one big program."

Tests actual understanding of the concept.

### Use Scenarios, Not Definitions

**❌ Bad Question**:
> "What is PID control?"

**✅ Good Question**:
> "Your robot is overshooting its target position. What would you adjust in your controller and why?"

### Reveal Gaps Through Application

**❌ Bad Question**:
> "Do you know about coordinate transforms?"

**✅ Good Question**:
> "You have a camera on a robot arm. The camera detects an object at position (X, Y) in camera frame. How do you convert this to the robot's base frame?"

## Level Definitions

### Complete Beginner
- **Cannot explain** basic concepts
- **No experience** with topic
- **Cannot read** simple code
- **Needs**: Foundational knowledge first

### Novice
- **Knows concepts** but cannot apply
- **Has followed** tutorials but doesn't understand why
- **Can read** simple code but cannot write
- **Needs**: Guided practice with explanation

### Advanced Beginner
- **Understands** core concepts
- **Can write** simple code with reference
- **Has built** basic projects with guidance
- **Needs**: Independent practice and depth

### Intermediate
- **Solid understanding** with application
- **Can design** solutions independently
- **Has built** projects solo
- **Needs**: Advanced topics and optimization

### Advanced
- **Expert-level** understanding
- **Extensive** project portfolio
- **Can teach** concepts to others
- **Needs**: Specialization or cutting-edge topics

## Scoring Guidelines

### Question Scoring

Each question gets one of:
- ✅ **Pass**: Demonstrates clear understanding
- ⚠️ **Partial**: Knows something but incomplete
- ❌ **Fail**: Cannot answer or incorrect

### Level Thresholds

**To be assessed at level X, must pass**:
- **Above Beginner**: Both beginner checks
- **Above Novice**: 2+ of 3 novice checks
- **Above Advanced Beginner**: 2+ of 3 advanced beginner checks
- **Intermediate**: Both intermediate checks
- **Advanced**: All intermediate + shows expertise

### Edge Cases

**Between levels**:
- Default to lower level for plan start
- Note in results: "Strong novice, approaching advanced beginner"
- Plan can accelerate if progressing well

**Inconsistent results**:
- Strong in some areas, weak in others
- Note specific gaps clearly
- Create mixed-level plan addressing gaps

**Over/Under estimation**:
- Student says "intermediate" but tests as "novice"
- Be honest but encouraging in results
- Explain why starting at assessed level is better

## Example Evaluation Flow

```
Student: "I want to learn ROS2. I'm intermediate level."

Evaluator:
1. Uses ros2-evaluation-template.md
2. Asks Q1-Q2: Student passes (above beginner)
3. Asks Q3-Q5: Student fails Q3, passes Q4-Q5 (borderline novice/advanced beginner)
4. Asks Q6-Q8: Student fails all (not advanced beginner)
5. Code test: Gets 2/4 correct
6. Determines: Novice level (not intermediate as claimed)
7. Identifies gaps: Topics vs services, QoS, TF2
8. Recommends: Start Day 1, standard pace, reinforce communication patterns

Returns to daily improvement agent:
{
  "level": "novice",
  "self_reported": "intermediate",
  "gaps": ["topic_service_distinction", "qos", "tf2_basics"],
  "strengths": ["node_architecture", "debugging_approach"],
  "start": "day-1",
  "pace": "standard",
  "adjustments": ["add_extra_communication_examples", "qos_deep_dive_week_2"]
}
```

## Tips for Evaluators

### Create Safe Environment
- "This helps me create the perfect plan for you"
- "Don't worry if you can't answer - that's valuable information!"
- "No judgment, just calibration"

### Probe Deeper
- If answer is vague: "Can you give me an example?"
- If seems memorized: "Why does that work?"
- If unsure: "What's your best guess and reasoning?"

### Be Encouraging
- Celebrate strengths: "Great understanding of X!"
- Frame gaps positively: "We'll address Y in your plan"
- Build confidence: "You're right where you need to be to make great progress"

### Be Honest
- Don't inflate assessment to please student
- Don't deflate to seem rigorous
- Match plan to reality for best outcomes

## Contributing

To add a new evaluation template:

1. Create `[topic]-evaluation-template.md`
2. Include all 6 required sections
3. Test with 3-5 students at different levels
4. Refine questions based on results
5. Submit PR with template + test results

---

**Remember**: Honest evaluation = Better learning outcomes!
