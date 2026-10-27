# Sets up the CAD toolchain on a Windows dev PC:
#   FreeCAD 1.1 + freecad-mcp addon (Claude Code drives FreeCAD) + CadQuery venv.
# Safe to re-run. Usage (from the repo root, in PowerShell):
#   powershell -ExecutionPolicy Bypass -File scripts\setup-dev-windows.ps1
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot

Write-Host '== 1/5 FreeCAD 1.1 (winget) =='
winget install --id FreeCAD.FreeCAD -e --source winget --accept-package-agreements --accept-source-agreements
# winget exits non-zero when it is already installed; that is fine.

Write-Host '== 2/5 uv / uvx (runs the MCP server, needs Python 3.12+) =='
if (-not (Get-Command uvx -ErrorAction SilentlyContinue)) {
    winget install --id astral-sh.uv -e --source winget --accept-package-agreements --accept-source-agreements
    Write-Host 'uv installed. Open a NEW terminal if uvx is not found yet.'
}

Write-Host '== 3/5 freecad-mcp addon -> FreeCAD Mod folder =='
$src = Join-Path $repo 'hardware\tools-freecad-mcp'
if (Test-Path $src) { git -C $src pull --ff-only } else { git clone https://github.com/neka-nat/freecad-mcp.git $src }
$mod = Join-Path $env:APPDATA 'FreeCAD\v1-1\Mod'
New-Item -ItemType Directory -Force $mod | Out-Null
Copy-Item -Recurse -Force (Join-Path $src 'addon\FreeCADMCP') $mod
if (-not (Test-Path (Join-Path $mod 'FreeCADMCP\InitGui.py'))) { throw "Addon copy failed: $mod\FreeCADMCP\InitGui.py missing" }

Write-Host '== 4/5 Pre-fetch the MCP server package =='
uvx freecad-mcp --help | Select-Object -First 1

Write-Host '== 5/5 CadQuery venv (hardware\cad\.venv) =='
$cad = Join-Path $repo 'hardware\cad'
if (-not (Test-Path (Join-Path $cad '.venv'))) { python -m venv (Join-Path $cad '.venv') }
& (Join-Path $cad '.venv\Scripts\python.exe') -m pip install -q --upgrade pip
& (Join-Path $cad '.venv\Scripts\python.exe') -m pip install -q -r (Join-Path $cad 'requirements.txt')

Write-Host ''
Write-Host 'Done. Next (manual): open FreeCAD -> workbench "MCP Addon" -> "Start RPC Server",'
Write-Host 'then start `claude` in the repo root and approve the "freecad" MCP server.'
