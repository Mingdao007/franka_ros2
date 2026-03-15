---
name: zotero-search
description: Find, read, summarize, and compare papers from local Zotero storage. Local-first — read PDFs and .zotero-ft-cache directly, use Zotero MCP only for metadata (collections, notes, annotations, authors, parent-child relations). Trigger phrases include 论文, paper, Zotero, literature, 文献.
metadata:
  author: mingdao-lab
  version: 0.1.0
compatibility: claude-code
---

# Zotero Search (Local-First)

## Policy

Local files first, MCP for metadata only.

1. Search `~/Zotero/storage/` for PDF content.
2. Prefer `.zotero-ft-cache` (extracted text) for speed; fall back to PDF if incomplete.
3. Use Zotero MCP only for: collections, notes, annotations, authors, year, parent-child relations.
4. If MCP is unavailable, continue with local files and note that metadata enrichment was skipped.

## Quick Start

| Request | Action |
|---|---|
| Read/summarize a paper | Local-first workflow below |
| Check Zotero setup | `bash scripts/check_zotero_local_first.sh` |
| Browse a collection | MCP: list collections → list items |
| Get annotations/notes | MCP: get item annotations/notes |

## Local-First Workflow

1. **Find the paper locally**: search `~/Zotero/storage/` by filename or grep `.zotero-ft-cache` for title/keywords.
2. **Read content**: prefer `.zotero-ft-cache` → PDF (via Read tool).
3. **Enrich with metadata** (optional): use MCP for collection membership, tags, annotations.
4. **Summarize/compare**: work from the extracted content.

## MCP Setup

The Zotero MCP must be configured in Claude Code project settings.

Expected config (already set for `franka_ros2_ws/src`):
```json
"zotero": {
  "type": "stdio",
  "command": "node",
  "args": ["/home/andy/.nvm/versions/node/v24.13.0/lib/node_modules/zotero-mcp/build/index.js"],
  "env": {}
}
```

If MCP tools are not available in this session, check:
- `claude mcp list` to verify
- Settings at `~/.claude/settings.json` under the project's `mcpServers`

## Rules

- Treat `~/Zotero/storage` as the primary content source.
- Do not start with MCP when the user needs document content.
- Do not block on MCP unavailability — local files are sufficient for reading.
- When both cache and PDF exist, prefer cache for speed, PDF for verification.
- Keep the distinction explicit: **local files = content**, **MCP = metadata**.

## Reference
- [`references/local-first.md`](references/local-first.md) — paths, config details, fallback guidance
