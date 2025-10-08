# Migration Plan: Claude Agent SDK Update

## Overview

This plan outlines the steps to migrate from the Claude Code SDK to the Claude Agent SDK. The SDK has been renamed to reflect its broader capabilities for building AI agents beyond coding tasks.

## Key Changes

### Package Renaming
- **TypeScript/JavaScript**: `@anthropic-ai/claude-code` → `@anthropic-ai/claude-agent-sdk`
- **Python**: `claude-code-sdk` → `claude-agent-sdk`

### Breaking Changes
1. System prompt no longer used by default
2. Settings sources no longer automatically loaded
3. Python: `ClaudeCodeOptions` renamed to `ClaudeAgentOptions`

## Migration Steps

### For Python Projects

#### 1. Package Management
```bash
# Uninstall old package
pip uninstall claude-code-sdk

# Install new package
pip install claude-agent-sdk
```

#### 2. Update Imports
```python
# Before
from claude_code_sdk import query, ClaudeCodeOptions

# After
from claude_agent_sdk import query, ClaudeAgentOptions
```

#### 3. Update Type References
- Replace all instances of `ClaudeCodeOptions` with `ClaudeAgentOptions`
- Review initialization code for options configuration

### For TypeScript/JavaScript Projects

#### 1. Package Management
```bash
# Uninstall old package
npm uninstall @anthropic-ai/claude-code

# Install new package
npm install @anthropic-ai/claude-agent-sdk
```

#### 2. Update Imports
```javascript
// Before
import { query, tool, createSdkMcpServer } from "@anthropic-ai/claude-code";

// After
import { query, tool, createSdkMcpServer } from "@anthropic-ai/claude-agent-sdk";
```

## Implementation Checklist

### Phase 1: Preparation
- [ ] Audit codebase for all SDK imports and usage
- [ ] Identify files using `claude-code-sdk` or `@anthropic-ai/claude-code`
- [ ] Document current system prompt configurations
- [ ] Document current settings sources
- [ ] Review existing tool implementations

### Phase 2: Package Update
- [ ] Uninstall old SDK package(s)
- [ ] Install new SDK package(s)
- [ ] Update `package.json` / `requirements.txt` / `pyproject.toml`
- [ ] Update lock files (`package-lock.json`, `poetry.lock`, etc.)

### Phase 3: Code Migration
- [ ] Update all import statements
- [ ] Rename `ClaudeCodeOptions` to `ClaudeAgentOptions` (Python)
- [ ] Explicitly set system prompts where needed
- [ ] Manually specify settings sources
- [ ] Update any tests using the SDK

### Phase 4: Configuration Review
- [ ] Review and update system prompt configurations
- [ ] Verify settings sources are properly configured
- [ ] Update permission modes if needed
- [ ] Review allowed tools configuration

### Phase 5: Testing
- [ ] Run unit tests
- [ ] Test query() functionality
- [ ] Test ClaudeSDKClient() for stateful interactions
- [ ] Test custom tools integration
- [ ] Verify streaming and non-streaming modes
- [ ] Test interrupts and context management

### Phase 6: Documentation
- [ ] Update internal documentation
- [ ] Update README files
- [ ] Update API documentation
- [ ] Document configuration changes
- [ ] Update examples and tutorials

## SDK Feature Reference

### Main Interaction Methods

#### 1. `query()` - Stateless
- Creates new session per interaction
- Best for one-off tasks
- Simple, stateless interactions

#### 2. `ClaudeSDKClient()` - Stateful
- Maintains conversation context
- Supports continuous conversations
- Enables interrupts and custom tools
- More complex, stateful interactions

### Example: Basic Usage (Python)
```python
from claude_agent_sdk import query, ClaudeAgentOptions

async def main():
    options = ClaudeAgentOptions(
        allowed_tools=["Read", "Write"],
        permission_mode='acceptEdits'
    )

    async for message in query(
        prompt="Create a Python project structure",
        options=options
    ):
        print(message)
```

## Important Considerations

1. **System Prompts**: No longer automatic - must be explicitly set if needed
2. **Settings Sources**: Must be manually specified
3. **Default Behaviors**: Review any code relying on previous defaults
4. **Tool Permissions**: Verify allowed_tools configuration
5. **Permission Modes**: Check permission_mode settings are appropriate

## Resources

- [Agent SDK Overview](https://docs.claude.com/en/api/agent-sdk/overview)
- [Python SDK Reference](https://docs.claude.com/en/api/agent-sdk/python)
- [TypeScript SDK Reference](https://docs.claude.com/en/api/agent-sdk/typescript)
- [Migration Guide](https://docs.claude.com/en/docs/claude-code/sdk/migration-guide)

## Timeline Estimate

- **Small Projects** (1-5 files): 1-2 hours
- **Medium Projects** (6-20 files): 3-6 hours
- **Large Projects** (20+ files): 1-2 days

## Rollback Plan

If issues arise during migration:

1. Uninstall new SDK package
2. Reinstall old SDK package from lock file versions
3. Revert code changes via git
4. Document issues encountered
5. Plan incremental migration approach

## Success Criteria

- [ ] All tests passing
- [ ] No import errors
- [ ] System prompts working as expected
- [ ] Settings sources properly configured
- [ ] Custom tools functioning correctly
- [ ] No breaking changes in application behavior
- [ ] Documentation updated

## Next Steps

1. Review this plan with team
2. Create backup/branch for migration work
3. Begin with Phase 1 (Preparation)
4. Execute migration in isolated environment
5. Test thoroughly before production deployment
