# RoboticProject

Learning platform: 3x Adeept 4WD mecanum kits (ADR036) + Raspberry Pi 5 8GB. The long-term goal is an autonomous AI-agent framework that later moves to a UAV fleet.

- Research and decisions: `docs/research/00-overview.md` (start here).
- Vendor docs (Adeept tutorials and datasheets, Pi 5 / Active Cooler drawings, RPLIDAR C1 datasheet): `docs/vendor/`.
- Dev PC setup (FreeCAD + freecad-mcp + CadQuery): `docs/setup/cad-freecad-mcp.md`. Run `scripts/setup-dev-windows.ps1` on a new PC.
- CAD: FreeCAD is driven through the `freecad` MCP server in `.mcp.json`. FreeCAD must be open with **MCP Addon → Start RPC Server** running. Fallback: CadQuery in `hardware/cad/.venv`.
- Dimensions: use official drawings where they exist. Adeept publishes no chassis dimensions, so mark those values as estimates until they're measured on the real kit.
