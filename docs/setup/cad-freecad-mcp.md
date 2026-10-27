# CAD setup: FreeCAD + freecad-mcp (Claude Code drives FreeCAD)

We model the robot (Pi 5, Active Cooler, Adeept HAT, chassis, LiDAR) in real CAD so that the fit checks and pictures are accurate. Claude Code controls FreeCAD through the **freecad-mcp** plugin ([neka-nat/freecad-mcp](https://github.com/neka-nat/freecad-mcp)). CadQuery (Python code-CAD) is the scripted backup.

## Quick setup on a new Windows PC

Prerequisites: `git`, `python` (3.11+) and `winget`. Clone this repo, then from the repo root:

```powershell
powershell -ExecutionPolicy Bypass -File scripts\setup-dev-windows.ps1
```

The script is safe to re-run. It installs:

| What | Where |
|---|---|
| FreeCAD 1.1 (winget `FreeCAD.FreeCAD`) | Program Files |
| uv / uvx (runs the MCP server; needs Python 3.12+, which uv fetches itself) | user profile |
| freecad-mcp addon (cloned to `hardware/tools-freecad-mcp/`) | copied to `%APPDATA%\FreeCAD\v1-1\Mod\FreeCADMCP\` |
| CadQuery venv (`hardware/cad/requirements.txt`) | `hardware/cad/.venv/` |

The MCP server registration is **already in the repo** (`.mcp.json`, project scope), so you don't need `claude mcp add` on the new PC.

## Every time you want Claude to use FreeCAD

1. Open **FreeCAD** and select the **MCP Addon** workbench from the workbench dropdown.
2. Click **Start RPC Server**. The status bar should show `127.0.0.1:9875`.
3. Start `claude` in the repo root. On first run, approve the project MCP server **freecad**.

Optional: the addon can start the RPC server automatically. See the plugin's `docs/configuration.md`.

## Troubleshooting

- **The "MCP Addon" workbench is missing.** Check that `%APPDATA%\FreeCAD\v1-1\Mod\FreeCADMCP\InitGui.py` exists, with no extra folder level. A FreeCAD version other than 1.1 uses a different folder. To find the right one, run `import os; print(os.path.join(FreeCAD.getUserAppDataDir(), "Mod"))` in **View → Panels → Python console**. Import errors appear in **View → Panels → Report view**.
- **Version mismatch warning.** The addon and the `freecad-mcp` package update separately. Re-run the setup script (it pulls the addon and copies it again) and restart FreeCAD. To update the server side, run `uvx --refresh freecad-mcp --help`.
- **Claude doesn't see the freecad tools.** MCP servers load only when Claude Code starts, so restart `claude`. Run `/mcp` to see their status.
- **Don't** install `freecad-mcp` into FreeCAD's bundled Python. The addon and server use separate Python environments.

## Manual install (if the script fails)

```powershell
winget install --id FreeCAD.FreeCAD -e
winget install --id astral-sh.uv -e
git clone https://github.com/neka-nat/freecad-mcp.git hardware\tools-freecad-mcp
Copy-Item -Recurse hardware\tools-freecad-mcp\addon\FreeCADMCP "$env:APPDATA\FreeCAD\v1-1\Mod\"
# .mcp.json in the repo already contains: { "freecad": { "command": "uvx", "args": ["freecad-mcp"] } }
```

*Last verified 2026-10-27: FreeCAD 1.1.4, freecad-mcp addon commit `8babe99`.*
