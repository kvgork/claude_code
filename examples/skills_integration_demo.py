"""
Skills Integration Demo

Demonstrates how learning-plan-manager and code-analysis skills work together
to enhance agent capabilities.
"""

import sys
from pathlib import Path
import json

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from skills.learning_plan_manager import LearningPlanManager, TaskStatus
from skills.code_analysis import CodeAnalyzer
from skills.learning_analytics import LearningAnalyzer, RecommendationPriority
from skills.interactive_diagram import DiagramGenerator


def demo_context_aware_planning():
    """
    Demo: Use code-analysis to create context-aware learning plans.

    Scenario: Agent wants to help student add a new feature.
    Instead of generic advice, agent analyzes codebase first.
    """
    print("=" * 70)
    print("DEMO 1: Context-Aware Planning")
    print("=" * 70)
    print()
    print("Scenario: Student wants to add a new 'code export' feature")
    print("Agent uses code-analysis to understand existing code first...")
    print()

    # Step 1: Analyze codebase
    analyzer = CodeAnalyzer()
    print("1. Analyzing src/claude_learning/...")
    analysis = analyzer.analyze_codebase("src/claude_learning/", max_files=10)

    print(f"   ✅ Found {analysis.total_files} files, {analysis.total_lines} lines")
    print(f"   ✅ Detected {len(analysis.patterns_found)} patterns")
    print(f"   ✅ Identified {len(analysis.integration_points)} integration points")
    print()

    # Step 2: Find integration points for "export" feature
    print("2. Finding integration points for 'export' feature...")
    suggestions = analysis.suggest_integration_for_feature("export client")

    if suggestions:
        print(f"   ✅ Found {len(suggestions)} suggestion(s):")
        for sugg in suggestions[:3]:
            print(f"      - {sugg.name} at {Path(sugg.file_path).name}:{sugg.line_number}")
            print(f"        Reason: {sugg.reason}")
    else:
        print("   ⚠️  No specific integration points found")
    print()

    # Step 3: Create learning plan with context
    print("3. Creating learning plan with codebase context...")
    print("   (In real scenario, plan-generation-mentor would use this data)")
    print()

    plan_context = {
        "feature": "code export functionality",
        "integration_points": [
            {
                "name": sugg.name,
                "location": f"{sugg.file_path}:{sugg.line_number}",
                "reason": sugg.reason
            }
            for sugg in suggestions[:3]
        ],
        "existing_patterns": list(analysis.patterns_found.keys()),
        "related_files": analysis.find_related_files(
            suggestions[0].file_path if suggestions else ""
        )
    }

    print(f"   ✅ Plan context prepared:")
    print(f"      - Integration points: {len(plan_context['integration_points'])}")
    print(f"      - Patterns to follow: {plan_context['existing_patterns']}")
    print()


def demo_progress_tracking_with_analysis():
    """
    Demo: Track learning progress while analyzing code complexity.

    Scenario: Student is learning, agent tracks progress AND
    analyzes code quality as they go.
    """
    print("=" * 70)
    print("DEMO 2: Progress Tracking + Code Quality Analysis")
    print("=" * 70)
    print()
    print("Scenario: Student is working through a learning plan")
    print("Agent tracks progress AND monitors code quality...")
    print()

    # Step 1: Load learning plan
    manager = LearningPlanManager()
    print("1. Loading latest learning plan...")
    plan = manager.find_latest_plan()

    if not plan:
        print("   ⚠️  No learning plan found")
        return

    print(f"   ✅ Loaded: {plan.metadata.title}")
    progress = plan.calculate_progress()
    print(f"   Progress: {progress['overall_percentage']:.1f}%")
    print()

    # Step 2: Analyze code student is working on
    analyzer = CodeAnalyzer()
    print("2. Analyzing skills/ directory (student's work)...")
    analysis = analyzer.analyze_codebase("skills/", max_files=15)

    # Collect all code smells
    all_smells = []
    for file_analysis in analysis.files:
        all_smells.extend(file_analysis.smells)

    print(f"   ✅ Analyzed {analysis.total_files} files")
    print(f"   Code quality: {len(all_smells)} smell(s) detected")
    print()

    # Step 3: Provide integrated feedback
    print("3. Integrated progress + quality feedback:")
    print()
    print(f"   📊 Learning Progress:")
    print(f"      - Tasks completed: {progress['tasks']['completed']}/{progress['tasks']['total']}")
    print(f"      - Current phase: {plan.get_current_phase().title if plan.get_current_phase() else 'N/A'}")
    print()

    print(f"   🔍 Code Quality:")
    if all_smells:
        smell_counts = {}
        for smell in all_smells:
            smell_type = smell["smell"]
            smell_counts[smell_type] = smell_counts.get(smell_type, 0) + 1

        for smell_type, count in list(smell_counts.items())[:3]:
            print(f"      - {smell_type}: {count}")
    else:
        print(f"      ✅ No code smells - excellent work!")
    print()


def demo_architecture_understanding():
    """
    Demo: Help student understand codebase architecture.

    Scenario: Student asks "How is this codebase organized?"
    Agent uses code-analysis to provide visual architecture overview.
    """
    print("=" * 70)
    print("DEMO 3: Architecture Understanding")
    print("=" * 70)
    print()
    print("Scenario: Student asks 'How is the skills system organized?'")
    print("Agent analyzes and explains architecture...")
    print()

    analyzer = CodeAnalyzer()
    print("1. Analyzing skills/ architecture...")
    analysis = analyzer.analyze_codebase("skills/")

    print(f"   ✅ Analyzed {analysis.total_files} files")
    print()

    # Show architecture overview
    print("2. Architecture Overview:")
    print()

    # Entry points
    print("   📌 Entry Points:")
    for entry in analysis.entry_points[:5]:
        print(f"      - {entry}")
    print()

    # Patterns found
    if analysis.patterns_found:
        print("   🎨 Design Patterns:")
        for pattern, files in analysis.patterns_found.items():
            print(f"      - {pattern.value}: {len(files)} file(s)")
    print()

    # Integration points
    print("   🔌 Integration Points (where to add new features):")
    for point in analysis.integration_points[:5]:
        print(f"      - {point.name} ({point.entity_type})")
        print(f"        → {point.reason}")
    print()

    # Dependency insights
    print("   🔗 Dependency Insights:")
    cycles = analysis.dependency_graph.find_circular_dependencies()
    if cycles:
        print(f"      ⚠️  Found {len(cycles)} circular dependencies")
    else:
        print(f"      ✅ No circular dependencies")

    print(f"      Total dependencies: {len(analysis.dependency_graph.edges)}")
    print()


def demo_learning_journey_with_code_feedback():
    """
    Demo: Complete learning journey with code analysis feedback.

    Scenario: Student progresses through plan, gets code feedback at each step.
    """
    print("=" * 70)
    print("DEMO 4: Complete Learning Journey")
    print("=" * 70)
    print()
    print("Scenario: Student completes a task, gets feedback before moving on")
    print()

    # Load plan
    plan_manager = LearningPlanManager()
    plan = plan_manager.find_latest_plan()

    if not plan:
        print("   ⚠️  No learning plan found")
        return

    print(f"1. Current status: {plan.metadata.title}")
    current_phase = plan.get_current_phase()
    next_task = plan.get_next_task()

    if not next_task:
        print("   ✅ All tasks completed!")
        return

    print(f"   Phase: {current_phase.title}")
    print(f"   Next task: {next_task.title}")
    print()

    # Simulate task completion
    print("2. Student completes task...")
    print(f"   [Student implements code for: {next_task.title}]")
    print()

    # Analyze their work
    analyzer = CodeAnalyzer()
    print("3. Agent analyzes student's code...")
    analysis = analyzer.analyze_codebase("skills/", max_files=15)

    # Check complexity
    complex_functions = []
    for file_analysis in analysis.files:
        for func in file_analysis.functions:
            if func.complexity and func.complexity.cyclomatic_complexity > 10:
                complex_functions.append(func)

    print()
    print("4. Feedback before marking task complete:")
    print()

    if complex_functions:
        print("   ⚠️  Code quality concerns:")
        print(f"      - Found {len(complex_functions)} complex functions")
        print("      - Consider refactoring before proceeding")
        print()
        print("   Agent suggests: Review complexity with debugging-detective")
    else:
        print("   ✅ Code quality looks good!")
        print("   ✅ Complexity metrics within acceptable range")
        print()
        print("   Agent says: Great work! Ready to mark task complete")
        print()

        # Mark task complete (dry run)
        print("5. Marking task complete (dry run)...")
        print(f"   Would update: {next_task.id} → COMPLETED")
        print(f"   Would update progress summary")
    print()


def demo_export_analysis_report():
    """
    Demo: Export combined analysis report.

    Shows how both skills data can be combined into comprehensive report.
    """
    print("=" * 70)
    print("DEMO 5: Combined Analysis Report Export")
    print("=" * 70)
    print()
    print("Generating comprehensive report combining both skills...")
    print()

    # Get learning plan data
    plan_manager = LearningPlanManager()
    plan = plan_manager.find_latest_plan()

    # Get code analysis data
    analyzer = CodeAnalyzer()
    code_analysis = analyzer.analyze_codebase("skills/", max_files=15)

    if plan:
        # Generate combined report
        report = {
            "learning_status": {
                "plan_title": plan.metadata.title,
                "progress": plan.calculate_progress(),
                "current_phase": plan.get_current_phase().title if plan.get_current_phase() else None,
                "next_task": plan.get_next_task().title if plan.get_next_task() else None
            },
            "code_quality": {
                "total_files": code_analysis.total_files,
                "total_lines": code_analysis.total_lines,
                "patterns": [p.value for p in code_analysis.patterns_found.keys()],
                "integration_points": len(code_analysis.integration_points),
                "code_smells": sum(len(f.smells) for f in code_analysis.files)
            },
            "recommendations": []
        }

        # Add recommendations
        progress_pct = report["learning_status"]["progress"]["overall_percentage"]
        if progress_pct < 30:
            report["recommendations"].append("Focus on learning fundamentals")
        elif progress_pct < 70:
            report["recommendations"].append("Continue building implementation skills")
        else:
            report["recommendations"].append("Focus on code quality and optimization")

        if report["code_quality"]["code_smells"] > 0:
            report["recommendations"].append("Review code smells with appropriate specialist")

        print("Generated Report:")
        print(json.dumps(report, indent=2))
        print()


def demo_learning_analytics():
    """
    Demo: Use learning-analytics to detect struggles and adapt teaching.

    Scenario: Student is working through a learning plan.
    Agent uses analytics to proactively detect issues.
    """
    print("=" * 70)
    print("DEMO 6: Learning Analytics - Proactive Teaching")
    print("=" * 70)
    print()
    print("Scenario: Analyze student's learning progress and detect issues")
    print()

    # Initialize
    manager = LearningPlanManager()
    analyzer = LearningAnalyzer()

    # Load plan
    print("1. Loading learning plan...")
    plan = manager.find_latest_plan()

    if not plan:
        print("   ⚠️  No plans found")
        return

    print(f"   ✅ Loaded: {plan.metadata.title}")
    print()

    # Analyze
    print("2. Analyzing learning progress...")
    analytics = analyzer.analyze_plan(plan)

    print(f"   ✅ Analysis complete")
    print()

    # Show results
    print("=" * 70)
    print("ANALYTICS RESULTS")
    print("=" * 70)
    print()

    # Overall health
    health_emoji = {
        "excellent": "🌟",
        "good": "✅",
        "needs_attention": "⚠️",
        "struggling": "🆘"
    }
    emoji = health_emoji.get(analytics.overall_health, "❓")

    print(f"Overall Health: {emoji} {analytics.overall_health.upper()}")
    print()

    # Velocity
    print("Velocity Metrics:")
    print(f"  • Tasks per week: {analytics.velocity_metrics.tasks_per_week}")
    print(f"  • Trend: {analytics.velocity_metrics.velocity_trend}")
    print(f"  • On track: {'✅ Yes' if analytics.velocity_metrics.on_track else '⚠️  No'}")
    if analytics.velocity_metrics.estimated_completion_date:
        print(f"  • Est. completion: {analytics.velocity_metrics.estimated_completion_date}")
    print()

    # Struggles
    if analytics.struggle_areas:
        print(f"⚠️  Struggle Areas ({len(analytics.struggle_areas)}):")
        for struggle in analytics.struggle_areas[:3]:
            severity_emoji = {"minor": "⚡", "moderate": "⚠️", "severe": "🆘"}
            emoji = severity_emoji.get(struggle.severity, "❓")
            print(f"\n  {emoji} {struggle.task_title}")
            print(f"     Severity: {struggle.severity}")
            if struggle.duration_days:
                print(f"     Duration: {struggle.duration_days} days (expected: {struggle.expected_duration_days})")
            print(f"     Recommendation: {struggle.recommendation}")
            if struggle.specialist_suggestion:
                print(f"     Specialist: {struggle.specialist_suggestion}")
        print()
    else:
        print("✅ No struggles detected - student doing well!")
        print()

    # Checkpoints
    print(f"Checkpoint Performance:")
    print(f"  • Pass rate: {analytics.checkpoint_analysis.pass_rate}%")
    print(f"  • First-try pass rate: {analytics.checkpoint_analysis.first_try_pass_rate}%")
    if analytics.checkpoint_analysis.patterns:
        print(f"  • Patterns:")
        for pattern in analytics.checkpoint_analysis.patterns[:2]:
            print(f"    - {pattern}")
    print()

    # Patterns
    if analytics.learning_patterns:
        print(f"Learning Patterns ({len(analytics.learning_patterns)}):")
        for pattern in analytics.learning_patterns:
            print(f"  • {pattern.pattern_type.value}: {pattern.description}")
            print(f"    Confidence: {pattern.confidence:.0%}")
            print(f"    Impact: {pattern.impact}")
        print()

    # Recommendations
    if analytics.recommendations:
        print(f"Recommendations ({len(analytics.recommendations)}):")
        print()

        # Group by priority
        by_priority = {}
        for rec in analytics.recommendations:
            p = rec.priority.value
            if p not in by_priority:
                by_priority[p] = []
            by_priority[p].append(rec)

        for priority in ["critical", "high", "medium", "low"]:
            if priority in by_priority:
                priority_emoji = {
                    "critical": "🆘",
                    "high": "⚠️",
                    "medium": "💡",
                    "low": "ℹ️"
                }
                emoji = priority_emoji.get(priority, "•")
                print(f"  {emoji} {priority.upper()} Priority:")
                for rec in by_priority[priority]:
                    print(f"     • {rec.title}")
                    print(f"       {rec.description}")
                print()
    else:
        print("✅ No recommendations - plan progressing well!")
        print()

    # Key insights
    if analytics.key_insights:
        print("Key Insights:")
        for insight in analytics.key_insights:
            print(f"  • {insight}")
        print()

    # Agent response example
    print("=" * 70)
    print("HOW AGENT WOULD RESPOND")
    print("=" * 70)
    print()

    if analytics.overall_health == "struggling" and analytics.struggle_areas:
        print("⚠️  Agent detects severe struggles")
        print()
        print("Agent Response:")
        print("-" * 70)
        severe = [s for s in analytics.struggle_areas if s.severity == "severe"]
        if severe:
            struggle = severe[0]
            print(f'"I notice you\'ve been working on "{struggle.task_title}" for')
            if struggle.duration_days:
                print(f'{struggle.duration_days} days. This is a challenging topic!')
            print()
            print(f"Would you like me to connect you with {struggle.specialist_suggestion}")
            print("to help work through it?")
            print()
            print("Alternatively, we could break this down into smaller pieces.")
            print('What approach sounds better to you?"')
    elif analytics.struggle_areas:
        print("💡 Agent detects moderate struggles")
        print()
        print("Agent Response:")
        print("-" * 70)
        struggle = analytics.struggle_areas[0]
        print(f'"I see you\'ve been on "{struggle.task_title}" for a while.')
        print("How is it going? Is there anything specific you're stuck on?")
        print()
        print('If helpful, I can connect you with a specialist or we can')
        print('break it down together."')
    elif analytics.checkpoint_analysis.first_try_pass_rate == 100 and analytics.checkpoint_analysis.passed_checkpoints > 0:
        print("🎉 Agent detects excellent performance")
        print()
        print("Agent Response:")
        print("-" * 70)
        print('"Excellent work! You\'ve passed all checkpoints on your first try,')
        print("which shows you have a strong grasp of these concepts.")
        print()
        print(f'Your steady progress of {analytics.velocity_metrics.tasks_per_week} tasks per week')
        print("is right on track. Keep up the great work!")
        print()
        print('Ready to tackle the next phase?"')
    else:
        print("✅ Agent proceeds normally")
        print()
        print("Agent Response:")
        print("-" * 70)
        print('"Great to see you back! You\'re making solid progress.')
        print(f'You\'ve completed {analytics.velocity_metrics.tasks_completed_total} tasks')
        print("and you're on track to finish as planned.")
        print()
        print('What would you like to work on today?"')

    print("-" * 70)


def demo_interactive_diagrams():
    """
    Demo: Generate visual diagrams from skills data

    Scenario: Transform data into visual learning aids
    """
    print("=" * 70)
    print("DEMO 7: Interactive Diagrams - Visual Learning")
    print("=" * 70)
    print()
    print("Scenario: Generate visual diagrams from skills data")
    print()

    # Initialize
    manager = LearningPlanManager()
    analyzer = LearningAnalyzer()
    generator = DiagramGenerator()

    # Load plan
    print("1. Loading learning plan...")
    plan = manager.find_latest_plan()

    if not plan:
        print("   ⚠️  No plans found")
        return

    print(f"   ✅ Loaded: {plan.metadata.title}")
    print()

    # Analyze
    print("2. Analyzing progress...")
    analytics = analyzer.analyze_plan(plan)
    print(f"   ✅ Analysis complete")
    print()

    # Generate diagrams
    print("=" * 70)
    print("GENERATING VISUAL DIAGRAMS")
    print("=" * 70)
    print()

    # Diagram 1: Progress Chart
    print("📊 Progress Chart (Pie Chart)")
    print("-" * 70)
    progress_diagram = generator.generate_progress_chart(analytics)
    print(f"Title: {progress_diagram.title}")
    print(f"Type: {progress_diagram.diagram_type.value}")
    print()
    print("Mermaid Output:")
    print(progress_diagram.to_markdown())
    print()

    # Diagram 2: Learning Journey
    print("🗺️  Learning Journey (Flowchart)")
    print("-" * 70)
    journey_diagram = generator.generate_learning_journey(plan, analytics)
    print(f"Title: {journey_diagram.title}")
    print(f"Phases: {journey_diagram.metadata.get('phases_count', 0)}")
    print()
    # Show first few lines of mermaid code
    lines = journey_diagram.mermaid_code.split('\n')[:5]
    print("Mermaid Preview:")
    for line in lines:
        print(f"  {line}")
    print("  ...")
    print()

    # Diagram 3: Timeline
    print("📅 Timeline (Gantt Chart)")
    print("-" * 70)
    timeline_diagram = generator.generate_gantt_chart(plan)
    print(f"Title: {timeline_diagram.title}")
    print(f"Type: {timeline_diagram.diagram_type.value}")
    print()

    # Show how agent would use it
    print("=" * 70)
    print("HOW AGENT WOULD USE DIAGRAMS")
    print("=" * 70)
    print()

    print("Agent Response:")
    print("-" * 70)
    print('"Great to see you back! Let me show you your progress visually.')
    print()
    print(progress_diagram.to_markdown())
    print()
    print("You\\'re making solid progress! Here\\'s your learning journey:")
    print()
    print("(Learning journey diagram would be shown here)")
    print()
    print('Ready to continue with the next task?"')
    print("-" * 70)
    print()

    print("✅ Visual diagrams enhance learning by:")
    print("  • Making abstract concepts concrete")
    print("  • Showing progress at a glance")
    print("  • Providing visual motivation")
    print("  • Helping understand structure and flow")


def main():
    """Run all demos"""
    print("\n" + "=" * 70)
    print("SKILLS INTEGRATION DEMO")
    print("Showing how skills enhance agent capabilities")
    print("=" * 70 + "\n")

    demos = [
        demo_context_aware_planning,
        demo_progress_tracking_with_analysis,
        demo_architecture_understanding,
        demo_learning_journey_with_code_feedback,
        demo_export_analysis_report,
        demo_learning_analytics,
        demo_interactive_diagrams,
    ]

    for demo in demos:
        try:
            demo()
            print()
        except Exception as e:
            print(f"\n❌ Demo failed: {e}")
            import traceback
            traceback.print_exc()

    print("=" * 70)
    print("DEMO COMPLETE")
    print("=" * 70)
    print()
    print("Key Takeaways:")
    print("1. code-analysis provides architectural understanding")
    print("2. learning-plan-manager tracks educational progress")
    print("3. learning-analytics detects struggles and adapts teaching")
    print("4. interactive-diagram makes concepts visual and concrete")
    print("5. Together they enable intelligent, visual, proactive teaching")
    print("6. Agents transform from reactive helpers to adaptive visual mentors")
    print()


if __name__ == "__main__":
    main()
