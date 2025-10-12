# ROS2 Skill Evaluation Template

Quick reference for evaluating ROS2 knowledge level.

## Evaluation Questions

### Level 1: Beginner Check

**Q1**: What is ROS2 and why would you use it?
- ✅ Pass: Explains it's for robot software, mentions communication/modularity
- ❌ Fail: Can't explain or very vague

**Q2**: What is a node?
- ✅ Pass: Independent process, does specific task, communicates with others
- ❌ Fail: Just says "a thing in ROS2" without understanding

### Level 2: Novice Check

**Q3**: Explain the difference between a topic and a service in your own words.
- ✅ Pass: Topics = continuous data stream, Services = request-response
- ⚠️ Partial: Knows they're different but can't explain when to use each
- ❌ Fail: Doesn't understand the difference

**Q4**: You have a robot with a camera and motors. How would you structure the nodes?
- ✅ Pass: Separate nodes for camera, motor control, decision-making, explains why
- ⚠️ Partial: Says "separate nodes" but can't explain structure
- ❌ Fail: Doesn't know how to approach

**Q5**: Your subscriber isn't receiving messages. What do you check?
- ✅ Pass: QoS compatibility, topic names, node running, message types
- ⚠️ Partial: Knows to check topics but not systematic
- ❌ Fail: No debugging approach

### Level 3: Advanced Beginner Check

**Q6**: Explain QoS settings and when they matter.
- ✅ Pass: Reliability, durability, explains use cases (sensors vs commands)
- ⚠️ Partial: Heard of QoS but doesn't know settings
- ❌ Fail: Doesn't know what QoS is

**Q7**: How do TF2 transforms work and why use them?
- ✅ Pass: Coordinate frame transforms, explains robot parts in different frames
- ⚠️ Partial: Knows they exist but fuzzy on details
- ❌ Fail: Doesn't understand transforms

**Q8**: When would you use an action instead of a service?
- ✅ Pass: Long-running tasks, need feedback, cancellable operations
- ⚠️ Partial: Knows actions exist but not clear on use case
- ❌ Fail: Doesn't know about actions

### Level 4: Intermediate Check

**Q9**: How would you design a multi-robot system in ROS2?
- ✅ Pass: Namespaces, domain IDs, communication patterns, explains architecture
- ⚠️ Partial: Basic ideas but missing key concepts
- ❌ Fail: No architectural thinking

**Q10**: Explain intra-process communication and when to use it.
- ✅ Pass: Zero-copy within process, performance optimization, trade-offs
- ⚠️ Partial: Heard of it but doesn't know when/why
- ❌ Fail: Doesn't know about it

## Code Reading Test

Show this code:

```python
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

**Questions**:
1. What does this code do?
2. What is the `10` parameter?
3. Why is there `self.subscription` at the end?
4. How would you change this to subscribe to a different message type?

**Scoring**:
- ✅ Advanced Beginner+: Answers all correctly
- ⚠️ Novice: Gets 1-3 correct
- ❌ Beginner: Can't follow the code

## Level Determination

**Score the answers and count**:
- Q1-Q2: Both pass → Above Beginner
- Q3-Q5: 2+ pass → Above Novice
- Q6-Q8: 2+ pass → Above Advanced Beginner
- Q9-Q10: Both pass → Intermediate+
- Code test: All correct → +1 level

## Recommended Starting Points

**Complete Beginner** (failed Q1-Q2):
- Day 1: ROS2 installation, concepts, why ROS2
- Day 2: First node (turtlesim)
- Very slow pace, lots of explanation

**Novice** (passed Q1-Q2, failed most Q3-Q5):
- Day 1: Quick ROS2 review, create simple publisher
- Day 2: Create subscriber
- Standard pace, some review

**Advanced Beginner** (passed Q3-Q5, failed most Q6-Q8):
- Day 1: Quick pub/sub review, jump to services
- Day 3-4: Start with launch files and parameters
- Faster pace, skip basics

**Intermediate** (passed Q6-Q8):
- Day 8-10: Start with advanced navigation/TF2
- Skip all basics
- Fast pace, advanced content from start

## Gap Identification

Based on failed questions, note specific gaps:
- Failed Q3 → Reinforce topics vs services early
- Failed Q4 → Add node architecture exercises
- Failed Q5 → Include debugging practice
- Failed Q6 → QoS deep dive needed
- Failed Q7 → TF2 fundamentals required
- Failed Q8 → Actions introduction needed
