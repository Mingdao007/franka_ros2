---
name: workspace-maintenance
description: Organize workspace files, maintain memory and skill docs, archive old data. Trigger phrases include 整理工作区, update skills, 更新memory, 归档, clean up, organize.
metadata:
  author: mingdao-lab
  version: 0.1.0
compatibility: claude-code
---

# Workspace Maintenance Skill

## Purpose
Keep the franka_ros2_ws workspace, memory files, and skill docs clean and up-to-date.

## When to use
- User says "整理", "clean up", "organize", "archive", "update skills", "更新memory".
- After a significant feature is completed and skill docs need updating.
- When workspace has accumulated temp files, old results, or stale caches.

## Maintenance tasks

### 1. Skill doc updates
After completing a feature or fixing a bug:
1. Update `SKILL.md` version and progress snapshot.
2. Update `references/current-state.md` status table.
3. Update `references/file-map.md` if files were added/removed.
4. Update `references/troubleshooting.md` if new failure modes were found.
5. Update `references/validated-workflow.md` if workflow changed.

### 2. Memory hygiene
Principles:
- **Skill-specific** rules → belong in SKILL.md (only loaded when skill is active, saves context).
- **General behavior** rules → belong in memory (always loaded across all conversations).
- **Duplicate** entries → remove from memory if already in SKILL.md.
- Don't store info derivable from code or git history.

Checklist:
1. Read `MEMORY.md` index.
2. For each memory file, check if it's duplicated in any SKILL.md.
3. Merge overlapping memory files into single entries.
4. Remove stale/outdated memories.
5. Keep `MEMORY.md` under 200 lines.

### 3. Workspace cleanup
Periodic cleanup targets:
- `src/__pycache__/` — auto-generated, safe to delete.
- `src/build/`, `src/install/`, `src/log/` — stale inner build artifacts (real build is at workspace root).
- Old experiment results in `src/results/` — archive to `~/Documents/_archive/franka_ros2/`.
- Empty placeholder directories.

Archive workflow:
```bash
# Pack old results (keep latest N runs)
cd src/results/hybrid_circle_force/
ARCHIVE_DIRS=$(ls -d 20* | sort | head -n -5)
tar czf ~/Documents/_archive/franka_ros2/old_results_$(date +%Y%m%d).tar.gz $ARCHIVE_DIRS
rm -rf $ARCHIVE_DIRS
```

### 4. Branch hygiene
- List branches: `git branch -a`
- Identify merged/stale branches.
- Don't delete without user approval.

## Guardrails
- Never delete files without checking if they're tracked in git.
- Always archive before deleting experiment data.
- Ask before removing any memory file — user may have context you don't.
- Never modify SKILL.md files for skills you're not actively working on without reading them first.
