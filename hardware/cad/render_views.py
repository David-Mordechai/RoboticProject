"""Clean product-style stills of the FreeCAD model (FreeCAD + MCP RPC server must be running).

    python render_views.py            -> docs/images/cad/robot-*.png
"""
import pathlib
import xmlrpc.client

OUT = pathlib.Path(__file__).parent.parent.parent / "docs" / "images" / "cad"
srv = xmlrpc.client.ServerProxy("http://127.0.0.1:9875", allow_none=True)

CODE = r'''
import FreeCAD as App, FreeCADGui as Gui, math
doc = App.getDocument("{doc}"); App.setActiveDocument("{doc}"); Gui.ActiveDocument = Gui.getDocument("{doc}")
hide = {hide!r}
for o in doc.Objects:
    o.ViewObject.Visibility = not any(o.Name.startswith(h) for h in hide)
v = Gui.ActiveDocument.ActiveView
v.setCameraType("{cam}")
# camera looks along -dir; yaw around Z (0 = from the front, +x), pitch above the horizon
yaw, pitch = math.radians({yaw}), math.radians({pitch})
r = App.Rotation(App.Vector(0, 0, 1), {yaw} + 90).multiply(App.Rotation(App.Vector(1, 0, 0), 90 - {pitch}))
v.setCameraOrientation(r.Q)
{special}
v.fitAll()
{zoom}
v.saveImage(r"{path}", {w}, {h}, "White")
'''

SHOTS = [
    # name, yaw, pitch, hide, camera
    ("robot-front-left", 35, 25, ["Laser_scan_plane", "Head_sweep_envelope"], "Perspective"),
    ("robot-rear-right", 215, 25, ["Laser_scan_plane", "Head_sweep_envelope"], "Perspective"),
    ("robot-lidar-plane", 35, 22, ["Head_sweep_envelope"], "Perspective"),
]


def shot(doc, name, yaw, pitch, hide, cam, w=1600, h=1000, zoom="", special=""):
    path = OUT / f"{name}.png"
    r = srv.execute_code(CODE.format(special=special, doc=doc, hide=hide, cam=cam, yaw=yaw, pitch=pitch, zoom=zoom, path=path, w=w, h=h), 60)
    if not r.get("success"):
        raise RuntimeError(r.get("error"))
    print("saved", path)


if __name__ == "__main__":
    for s in SHOTS:
        shot("Robot", *s)
