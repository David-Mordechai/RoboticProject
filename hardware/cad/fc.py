"""Tiny client for the freecad-mcp RPC server (FreeCAD must be running with the server started).

Usage:
  python fc.py run <script.py>                         # execute a FreeCAD Python script in the GUI
  python fc.py shot <out.png> [view] [width] [height]  # screenshot of the active 3D view
  python fc.py ping
"""
import base64
import sys
import xmlrpc.client

srv = xmlrpc.client.ServerProxy("http://127.0.0.1:9875", allow_none=True)

cmd = sys.argv[1]
if cmd == "ping":
    print(srv.ping())
elif cmd == "run":
    with open(sys.argv[2], encoding="utf-8") as f:
        res = srv.execute_code(f.read(), 120)
    print(res.get("message") or res.get("error") or res)
    if not res.get("success"):
        sys.exit(1)
elif cmd == "shot":
    view = sys.argv[3] if len(sys.argv) > 3 else "Isometric"
    w = int(sys.argv[4]) if len(sys.argv) > 4 else 1400
    h = int(sys.argv[5]) if len(sys.argv) > 5 else 900
    b64 = srv.get_active_screenshot(view, w, h)
    if not b64:
        sys.exit("no screenshot (is a 3D view active?)")
    with open(sys.argv[2], "wb") as f:
        f.write(base64.b64decode(b64))
    print("saved", sys.argv[2])
