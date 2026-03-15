# Zotero Local-First Reference

## Local Paths

- `~/Zotero/zotero.sqlite` — database
- `~/Zotero/storage/` — attachment storage (each item in an 8-char hash directory)
- `~/Zotero/storage/<HASH>/.zotero-ft-cache` — extracted full text (fastest to read)
- `~/Zotero/storage/<HASH>/<filename>.pdf` — original PDF

## Searching for a Paper

```bash
# By title keyword in extracted text cache
grep -rl "keyword" ~/Zotero/storage/*/.zotero-ft-cache | head -5

# By PDF filename
find ~/Zotero/storage -name "*.pdf" | xargs grep -l "keyword" 2>/dev/null | head -5
```

## MCP Config (Claude Code)

In `~/.claude/settings.json`, under the project's `mcpServers`:

```json
"zotero": {
  "type": "stdio",
  "command": "node",
  "args": ["/home/andy/.nvm/versions/node/v24.13.0/lib/node_modules/zotero-mcp/build/index.js"],
  "env": {}
}
```

Note: currently configured for `/home/andy/franka_ros2_ws/src` project only. If running from `/home/andy/franka_ros2_ws`, MCP may not be available — use local file workflow.

## Fallback Guidance

If MCP is unavailable:
1. Continue using local storage for all content work.
2. State that metadata enrichment was unavailable.
3. Never block reading/summarization just because MCP is missing.
