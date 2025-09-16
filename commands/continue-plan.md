You are resuming work from an existing implementation plan.

## Process
1. **Load Current Plan**: 
   - Find the most recent plan in `plans/` directory
   - If multiple plans exist, ask which one to continue
   - Load the plan content using @plans/filename.md

2. **Status Assessment**:
   - Identify the current phase and task marked as "IN PROGRESS" 
   - Review any blockers or questions noted in the plan
   - Check what was completed in the last session

3. **Context Restoration**:
   - Summarize where we left off (1-2 sentences)
   - Highlight the specific task we should work on next
   - Note any decisions or technical approaches already established

4. **Execution Strategy**:
   - If there are blockers, address them first or ask for clarification
   - If ready to proceed, start implementing the next task
   - Follow the technical approach and architecture already defined in the plan
   - Update the plan as work progresses

5. **Before Starting Implementation**:
   - Ask if I want to:
     a) Continue with the next planned task
     b) Review/modify the current plan first
     c) Handle any blockers or questions first
     d) Skip to a different part of the plan

## Integration
- Reference the project's CLAUDE.md for coding standards and architecture
- Use existing patterns and conventions established in the codebase
- Maintain consistency with decisions already made in the plan

## Progress Tracking
As work progresses, update the plan file with:
- Completed task checkmarks
- New technical discoveries or decisions
- Any scope changes or new requirements
- Updated blockers or dependencies

Ready to pick up where we left off!