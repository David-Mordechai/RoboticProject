"""Professional 'how the board stack goes together' animation, rendered from the live FreeCAD model.

Needs FreeCAD running with the MCP RPC server (Claude builds the BoardStack model first).
Don't touch the FreeCAD 3D view while it renders (mouse drags block the render queue).

    .venv\\Scripts\\python stack_animation.py   -> docs/images/cad/stack-assembly.gif
"""
import math
import pathlib
import tempfile
import xmlrpc.client

from PIL import Image, ImageDraw, ImageFilter, ImageFont

HERE = pathlib.Path(__file__).parent
OUT_DIR = HERE.parent.parent / "docs" / "images" / "cad"
TMP = pathlib.Path(tempfile.gettempdir()) / "stack_frames_pro"
TMP.mkdir(exist_ok=True)
srv = xmlrpc.client.ServerProxy("http://127.0.0.1:9875", allow_none=True)

FPS = 30
RW, RH = 2560, 1440          # FreeCAD render size (supersampled 2x)
OW, OH = 1280, 720           # video size
ACCENT = (95, 61, 196)       # indigo, matches the purple HAT
INK, MUTED = (33, 37, 41), (110, 117, 125)

# group -> (object-name prefix, exploded lift in mm)
GROUPS = [("so6", "SO6_", 22), ("pi", "Pi5_", 48), ("cooler", "ActiveCooler_", 88),
          ("so21", "SO21_", 122), ("hat", "HAT_", 165)]
STEPS = [
    ("Short brass standoffs", "Four 6 mm M2.5 standoffs screw into deck plate A01"),
    ("Raspberry Pi 5", "Sits on the standoffs. Its 40 GPIO pins stand up along one edge"),
    ("Active Cooler", "Clips onto the Pi's CPU, right next to the GPIO pins"),
    ("Tall brass standoffs", "Four 21 mm M2.5 standoffs hold the HAT level above the cooler"),
    ("Adeept Robot HAT", "Its tall black socket slides onto the 40 pins; screws into the standoffs"),
]

FONT = "C:/Windows/Fonts/"


def font(name, size):
    try:
        return ImageFont.truetype(FONT + name, size)
    except OSError:
        return ImageFont.load_default()


F_TITLE, F_SUB = font("seguisb.ttf", 40), font("segoeui.ttf", 24)
F_CARD, F_CARD_SUB = font("seguisb.ttf", 28), font("segoeui.ttf", 20)
F_SMALL, F_NUM, F_CHIP = font("segoeui.ttf", 16), font("seguisb.ttf", 22), font("seguisb.ttf", 22)

SETUP = r'''
import FreeCAD as App, FreeCADGui as Gui, math
doc = App.getDocument("BoardStack"); App.setActiveDocument("BoardStack"); Gui.ActiveDocument = Gui.getDocument("BoardStack")
for o in doc.Objects:
    o.Placement = App.Placement()
_bb = doc.getObject("Pi5_PCB").Shape.BoundBox
STACK_C = App.Vector(_bb.Center.x, _bb.Center.y, 0)   # fixed turntable centre (kept in FreeCAD's session)
for o in doc.Objects:
    o.ViewObject.Visibility = o.Name != "HAT_solder_tails_zone"
    o.ViewObject.Transparency = 0
    o.ViewObject.LineWidth = 1.0
    o.ViewObject.LineColor = (0.22, 0.23, 0.26)
v = Gui.ActiveDocument.ActiveView
v.setCameraType("Perspective")
r = App.Rotation(App.Vector(0, 0, 1), 125).multiply(App.Rotation(App.Vector(1, 0, 0), 66))
v.setCameraOrientation(r.Q)
'''

FRAME = r'''
import FreeCAD as App, FreeCADGui as Gui
doc = App.getDocument("BoardStack")
offs = {offs!r}; angle = {angle}; trans = {trans!r}
c = STACK_C   # never recompute from the rotated shape: that made the model drift away
rot = App.Rotation(App.Vector(0, 0, 1), angle)
for obj in doc.Objects:
    dz = 0.0
    for prefix, z in offs.items():
        if obj.Name.startswith(prefix): dz = z
    obj.Placement = App.Placement(App.Vector(0, 0, dz) + c - rot.multVec(c), rot)
    if trans is not None and obj.Name != "HAT_solder_tails_zone":
        t = 0
        for prefix, tv in trans.items():
            if obj.Name.startswith(prefix): t = tv
        if obj.ViewObject.Transparency != t: obj.ViewObject.Transparency = t
Gui.updateGui()
{camera}
Gui.ActiveDocument.ActiveView.saveImage(r"{path}", {W}, {H}, "Transparent")
'''


def run(code):
    r = srv.execute_code(code, 90)
    if not r.get("success"):
        raise RuntimeError(r.get("error") or r)


def ease(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)


# ---------------------------------------------------------------- timeline
def timeline():
    """One dict per frame: offsets, model angle, transparency map, overlay state."""
    frames = []
    off = {p: z for _, p, z in GROUPS}
    angle = -15.0

    def add(n, **kw):
        nonlocal angle
        for i in range(n):
            st = dict(offs=dict(off), angle=angle, trans=kw.get("trans"), step=kw.get("step"),
                      phase=kw.get("phase"), t=i / max(1, n - 1), n=n, i=i)
            frames.append(st)
            angle += kw.get("spin", 0.12)

    add(66, phase="intro", trans={p: 0 for _, p, _ in GROUPS})
    for k, (key, prefix, z0) in enumerate(GROUPS):
        ghost = {p: (0 if j <= k else 82) for j, (_, p, _) in enumerate(GROUPS)}
        add(12, phase="step", step=k, trans=ghost)
        for i in range(28):
            off[prefix] = z0 * (1 - ease((i + 1) / 28))
            add(1, phase="step", step=k, trans=ghost)
        add(22, phase="step", step=k, trans=ghost)
    # reveal: HAT turns see-through so the cooler shows
    for i in range(24):
        add(1, phase="reveal", trans={"HAT_PCB": round(60 * ease(i / 23)), "HAT_cap": 0})
    # orbit 360 with the clearance chips
    n = 170
    for i in range(n):
        a0 = angle
        frames.append(dict(offs=dict(off), angle=a0 + 360 * ease(i / (n - 1)), trans=None,
                           step=None, phase="orbit", t=i / (n - 1), n=n, i=i))
    angle += 360
    add(45, phase="outro", spin=0.0)
    return frames


# ---------------------------------------------------------------- compositing
def bg():
    g = Image.new("RGB", (OW, OH))
    d = ImageDraw.Draw(g)
    for y in range(OH):
        k = y / OH
        c = tuple(int(a + (b - a) * k) for a, b in zip((250, 250, 252), (226, 229, 235)))
        d.line([(0, y), (OW, y)], fill=c)
    return g


BG = None


def rounded(d, box, r, fill, outline=None):
    d.rounded_rectangle(box, r, fill=fill, outline=outline)


def compose(img, box, st, total_steps):
    """img: RGBA render; box: smoothed crop box in render pixels."""
    x0, y0, x1, y1 = box
    crop = img.crop((int(x0), int(y0), int(x1), int(y1))).resize((OW, OH), Image.LANCZOS)
    frame = BG.copy().convert("RGBA")
    # soft contact shadow from the alpha mask, squashed onto the floor
    a = crop.split()[3]
    bbox = a.getbbox()
    if bbox:
        sh = Image.new("L", (OW, OH), 0)
        ds = ImageDraw.Draw(sh)
        cx, w = (bbox[0] + bbox[2]) / 2, (bbox[2] - bbox[0])
        ds.ellipse([cx - w * 0.42, bbox[3] - 18, cx + w * 0.42, bbox[3] + 22], fill=70)
        sh = sh.filter(ImageFilter.GaussianBlur(18))
        frame.paste((20, 24, 40, 255), (0, 0), sh)
    frame.alpha_composite(crop)
    d = ImageDraw.Draw(frame)

    # header + step progress
    d.text((40, 30), "ROBOTICPROJECT  ·  BOARD STACK", font=F_SMALL, fill=MUTED)
    step = st["step"]
    done = total_steps if st["phase"] in ("reveal", "orbit", "outro") else (step if step is not None else -1)
    for k in range(total_steps):
        x = OW - 40 - (total_steps - k) * 46
        col = ACCENT if k < done or k == step else (200, 204, 212)
        d.rounded_rectangle([x, 36, x + 38, 42], 3, fill=col)

    # fades
    def fade(i, n, fin=10, fout=10):
        return min(1.0, i / fin, (n - 1 - i) / fout if n > fout else 1.0)

    if st["phase"] == "intro":
        al = fade(st["i"], st["n"], 14, 14)
        ov = Image.new("RGBA", (OW, OH), (0, 0, 0, 0))
        do = ImageDraw.Draw(ov)
        do.text((60, OH - 190), "How the board stack goes together", font=F_TITLE, fill=INK + (int(255 * al),))
        do.text((62, OH - 136), "Raspberry Pi 5  +  Active Cooler  +  Adeept Robot HAT V3.2", font=F_SUB, fill=MUTED + (int(255 * al),))
        do.rectangle([60, OH - 92, 140, OH - 88], fill=ACCENT + (int(255 * al),))
        frame.alpha_composite(ov)
    elif st["phase"] == "step":
        title, sub = STEPS[step]
        cf = 1.0
        ov = Image.new("RGBA", (OW, OH), (0, 0, 0, 0))
        do = ImageDraw.Draw(ov)
        bx0, by0, bx1, by1 = 40, OH - 130, 40 + 740, OH - 40
        sh = Image.new("L", (OW, OH), 0)
        ImageDraw.Draw(sh).rounded_rectangle([bx0 + 4, by0 + 8, bx1 + 4, by1 + 8], 14, fill=60)
        ov.paste((0, 0, 0, 255), (0, 0), sh.filter(ImageFilter.GaussianBlur(10)))
        do.rounded_rectangle([bx0, by0, bx1, by1], 14, fill=(255, 255, 255, int(245 * cf)))
        do.ellipse([bx0 + 22, by0 + 24, bx0 + 64, by0 + 66], fill=ACCENT)
        n = str(step + 1)
        tw = do.textlength(n, font=F_NUM)
        do.text((bx0 + 43 - tw / 2, by0 + 30), n, font=F_NUM, fill="white")
        do.text((bx0 + 84, by0 + 14), title, font=F_CARD, fill=INK)
        do.text((bx0 + 84, by0 + 52), sub, font=F_CARD_SUB, fill=MUTED)
        frame.alpha_composite(ov)
    elif st["phase"] in ("reveal", "orbit", "outro"):
        al = 1.0 if st["phase"] != "reveal" else st["t"]
        ov = Image.new("RGBA", (OW, OH), (0, 0, 0, 0))
        do = ImageDraw.Draw(ov)
        do.text((60, OH - 170), "Assembled: the cooler fits under the HAT", font=F_TITLE, fill=INK + (int(255 * al),))
        chips = [("8.5 mm", "air gap over the fan"), ("3.3 mm", "tightest spot (motor connectors)"), ("21 mm", "standoff height")]
        x = 60
        for big, small in chips:
            w = max(do.textlength(big, font=F_CHIP), do.textlength(small, font=F_SMALL)) + 36
            do.rounded_rectangle([x, OH - 108, x + w, OH - 40], 12, fill=(255, 255, 255, int(235 * al)), outline=(222, 226, 230, int(255 * al)))
            do.text((x + 18, OH - 102), big, font=F_CHIP, fill=ACCENT + (int(255 * al),))
            do.text((x + 18, OH - 70), small, font=F_SMALL, fill=MUTED + (int(255 * al),))
            x += w + 14
        frame.alpha_composite(ov)
    if st["phase"] == "outro":
        k = st["i"] / max(1, st["n"] - 1)
        frame.alpha_composite(Image.new("RGBA", (OW, OH), (250, 250, 252, int(140 * ease(k)))))
    foot = "Dimensions: official Raspberry Pi drawings + Adeept assembly guide"
    d.text((OW - 40 - d.textlength(foot, font=F_SMALL), OH - 30), foot, font=F_SMALL, fill=(150, 156, 164))
    return frame.convert("RGB")


def smooth_boxes(boxes, win=21):
    """Centered moving average of crop boxes -> calm, professional camera moves."""
    out = []
    for i in range(len(boxes)):
        lo, hi = max(0, i - win // 2), min(len(boxes), i + win // 2 + 1)
        seg = boxes[lo:hi]
        out.append(tuple(sum(b[j] for b in seg) / len(seg) for j in range(4)))
    return out


def frame_box(img):
    bb = img.split()[3].getbbox() or (0, 0, RW, RH)
    cx, cy = (bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2
    w = max((bb[2] - bb[0]) * 1.35, (bb[3] - bb[1]) * 1.5 * 16 / 9, OW * 1.0)
    h = w * 9 / 16
    cy += h * 0.06       # leave room for the caption card at the bottom
    return (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)


def main():
    global BG
    BG = bg()
    frames = timeline()
    print(f"{len(frames)} frames ({len(frames) / FPS:.1f} s) -> rendering in FreeCAD...")
    run(SETUP)
    run(FRAME.format(offs={p: z for _, p, z in GROUPS}, angle=-15, trans=None,
                     camera="Gui.ActiveDocument.ActiveView.fitAll()", path=str(TMP / "fit.png"), W=RW, H=RH))
    last_trans = None
    for i, st in enumerate(frames):
        tr = st["trans"] if st["trans"] != last_trans else None
        if st["trans"] is not None:
            last_trans = st["trans"]
        run(FRAME.format(offs=st["offs"], angle=st["angle"], trans=tr, camera="", path=str(TMP / f"r{i:04d}.png"), W=RW, H=RH))
        if i % 60 == 0:
            print(f"  rendered {i}/{len(frames)}")
    # restore the model
    run(FRAME.format(offs={p: 0 for _, p, _ in GROUPS}, angle=0, trans={"HAT_PCB": 45},
                     camera="Gui.SendMsgToActiveView('ViewFit')", path=str(TMP / "end.png"), W=400, H=300))

    print("compositing...")
    boxes = smooth_boxes([frame_box(Image.open(TMP / f"r{i:04d}.png")) for i in range(len(frames))])
    gif_frames = []
    for i, st in enumerate(frames):
        if i % 2:
            continue                       # 15 fps GIF
        img = Image.open(TMP / f"r{i:04d}.png").convert("RGBA")
        out = compose(img, boxes[i], st, len(GROUPS))
        gif_frames.append(out.resize((960, 540), Image.LANCZOS).quantize(colors=192, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE))
    gif = OUT_DIR / "stack-assembly.gif"
    gif_frames[0].save(gif, save_all=True, append_images=gif_frames[1:], duration=int(2000 / FPS), loop=0, optimize=True)
    print("saved", gif)


if __name__ == "__main__":
    main()
