# Global Codex Notes

## Available Tools

- `graphify` is installed as a Codex skill at `~/.codex/skills/graphify`. Use `$graphify` or the `graphify` CLI to build, update, query, and inspect project knowledge graphs in `graphify-out/` (`graph.json`, `GRAPH_REPORT.md`, `graph.html`).
- `serena` is configured as a Codex MCP server in `~/.codex/config.toml` under `[mcp_servers.serena]`. It is the same upstream Serena used by Claude Code: `uvx --from git+https://github.com/oraios/serena serena start-mcp-server`.
- Use Serena for semantic code navigation, symbol/reference lookup, project understanding, and LSP-backed indexing. The CLI also supports `serena project index` for saving project symbols to the LSP cache.
- Do not run `graphify install`, `graphify codex install`, Serena setup, or hook installers unless the user explicitly asks, because those commands modify assistant or project configuration.
