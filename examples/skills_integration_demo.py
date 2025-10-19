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


def main():
    """Run all demos"""
    print("\n" + "=" * 70)
    print("SKILLS INTEGRATION DEMO")
    print("Showing how learning-plan-manager + code-analysis work together")
    print("=" * 70 + "\n")

    demos = [
        demo_context_aware_planning,
        demo_progress_tracking_with_analysis,
        demo_architecture_understanding,
        demo_learning_journey_with_code_feedback,
        demo_export_analysis_report,
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
    print("3. Together they enable intelligent, context-aware teaching")
    print("4. Agents can provide better guidance with both skills")
    print()


if __name__ == "__main__":
    main()
