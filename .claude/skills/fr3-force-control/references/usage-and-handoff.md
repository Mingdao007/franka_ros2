# Usage and Handoff

## A) Use in Claude Code (local project)
1. Keep this skill folder at:
   - `.claude/skills/fr3-force-control/`
2. Make sure `SKILL.md` stays at the skill folder root.
3. Keep references under `references/` for progressive disclosure.

## B) Use in Claude.ai
1. Zip folder `fr3-force-control/`.
2. Upload via Settings → Capabilities → Skills.
3. Enable the skill and test with trigger prompts:
   - "FR3 力控仿真验证"
   - "run hybrid_circle_force_controller and check Fz RMSE"
   - "总结当前 Gazebo 力控进展"

## C) Repository mirror policy
- Canonical source: `.claude/skills/fr3-force-control/`
- Mirror target: `docs/skills/fr3-force-control/`
- Update rule:
  1. Edit canonical files first.
  2. Copy to docs mirror.
  3. Keep structure aligned.

## D) Suggested output format for future AI sessions
When validating experiments, return:
1. Experiment command + key environment variables.
2. Result path and timestamp.
3. Gate metrics and PASS/FAIL.
4. One clear next action.

## E) Handoff to Claude CLI
Yes, in most cases reading this skill is enough as the base context.

For reliable transfer, always include:
1. Skill path: `.claude/skills/fr3-force-control/SKILL.md`
2. Current state file: `references/current-state.md`
3. Active priority note (for now: ink is on hold, focus on force-control tasks)

Recommended handoff prompt:
`Read .claude/skills/fr3-force-control/SKILL.md and references/current-state.md, then continue with the highest-priority force-control validation task.`
