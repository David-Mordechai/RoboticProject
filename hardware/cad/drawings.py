"""Dimensioned 2D side-elevation drawings generated from the SAME numbers as robot_model.py.

Run:  .venv\\Scripts\\python drawings.py   -> docs/images/cad/*.png
"""
import pathlib
import sys
import types

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

HERE = pathlib.Path(__file__).parent
OUT = HERE.parent.parent / "docs" / "images" / "cad"
OUT.mkdir(parents=True, exist_ok=True)

# Load D from robot_model.py without FreeCAD (stub the imports, stop before build()).
src = (HERE / "robot_model.py").read_text(encoding="utf-8").split("COL = dict(")[0]
sys.modules["FreeCAD"] = types.SimpleNamespace(Vector=lambda *a: a)
sys.modules["Part"] = types.SimpleNamespace()
ns = {}
exec(src, ns)
D = ns["D"]

C = dict(deck="#3a3a40", pi="#1f8c48", hat="#8a3fb0", brass="#d4a537", cooler="#b9bec6", fan="#555",
         hdr="#222", warn="#e8590c", ok="#2b8a3e", laser="#e03131", head="#3b5bdb", lidar="#333", txt="#111")


def dim(ax, x, z0, z1, text, side=1, color="#111"):
    ax.annotate("", (x, z0), (x, z1), arrowprops=dict(arrowstyle="<->", color=color, lw=1.2, shrinkA=0, shrinkB=0))
    ax.text(x + side * 1.5, (z0 + z1) / 2, text, va="center", ha="left" if side > 0 else "right", fontsize=9, color=color)


def stack_drawing():
    z_pi = D["so_lower"]; z_pt = z_pi + D["pcb_t"]; z_hat = z_pt + D["so_upper"]
    z_ct = z_pt + D["cool_h"]; z_tails = z_hat - D["hat_tails"]
    fig, ax = plt.subplots(figsize=(11, 6.2), dpi=130)
    L = D["pi_len"]
    ax.add_patch(Rectangle((-10, -3), L + 20, 3, color=C["deck"]))
    ax.text(L + 11, -1.5, "Deck A01", va="center", fontsize=9)
    for u in (3.5, 61.5):
        ax.add_patch(Rectangle((u - 2.5, 0), 5, D["so_lower"], color=C["brass"]))
        ax.add_patch(Rectangle((u - 2.5, z_pt), 5, D["so_upper"], color=C["brass"]))
    ax.add_patch(Rectangle((0, z_pi), L, D["pcb_t"], color=C["pi"]))
    ax.add_patch(Rectangle((68, z_pt), 20, D["usb_h"], color="#999"))
    ax.text(78, z_pt + D["usb_h"] / 2, "USB/\nEthernet", ha="center", va="center", fontsize=8, color="white")
    ax.add_patch(Rectangle((3, z_pt), 31, D["cool_h"], color=C["cooler"]))
    ax.add_patch(Rectangle((34, z_pt), D["cool_len"] - 31, D["cool_h"], color=C["fan"]))
    ax.add_patch(Rectangle((4.5, z_pt), 3, D["cool_pin_h"], color="#eee", ec="#888"))  # push-pin head at the fin corner
    ax.text(3 + D["cool_len"] / 2, z_pt + D["cool_h"] / 2, "Active Cooler (fins + blower)", ha="center", va="center", fontsize=9, color="white")
    ax.add_patch(Rectangle((0, z_tails), D["hat_len"], D["hat_tails"], facecolor=C["warn"], alpha=0.35, hatch="////", edgecolor=C["warn"]))
    u0, u1 = D["motor_conn_u"]
    z_mc = z_hat - D["motor_conn_h"]
    ax.add_patch(Rectangle((u0, z_mc), u1 - u0, D["motor_conn_h"], facecolor="#f4f4ee", edgecolor="#777"))
    ax.annotate("4 white motor connectors\n(M1-M4) under the HAT", (u0 + 2, z_mc + 1), (-38, z_hat + 12), fontsize=8,
                arrowprops=dict(arrowstyle="->", color="#555"), color="#333")
    for name, a0, a1, b0, b1, h in D["hat_under"]:
        if h > 2:  # regulators, inductors, diodes (chips are ~1.2 mm and sit inside the hatched band)
            ax.add_patch(Rectangle((a0, z_hat - h), a1 - a0, h, facecolor="#2a2a2a", edgecolor="#000", alpha=0.85))
    ax.annotate("power-supply parts under the HAT\n(regulators + coils, ~4.5-5.5 mm)", (50, z_hat - 5.5), (62, z_hat + 16), fontsize=8,
                arrowprops=dict(arrowstyle="->", color="#555"), color="#333")
    ax.add_patch(Rectangle((0, z_hat), D["hat_len"], D["hat_t"], color=C["hat"]))
    ax.add_patch(Rectangle((50, z_hat + D["hat_t"]), 10, D["hat_cap_h"], color="#222"))
    ax.text(24, z_hat + 5, "Adeept Robot HAT V3.2", ha="center", fontsize=10, color=C["hat"], weight="bold")
    ax.text(-2, z_pi + 0.8, "Raspberry Pi 5", ha="right", va="center", fontsize=10, color=C["pi"], weight="bold")
    # dimensions
    gap_conn = z_mc - (z_pt + D["cool_pin_h"])
    dim(ax, -6, 0, z_pi, f"{D['so_lower']:.0f} mm standoff", side=-1)
    dim(ax, 92, z_pt, z_hat, f"{D['so_upper']:.0f} mm standoff\n(Adeept M2.5x21)")
    dim(ax, -6, z_pt, z_ct, f"{D['cool_h']} mm cooler\nabove the Pi", side=-1)
    dim(ax, 22, z_ct, z_tails, f"air gap {z_tails - z_ct:.1f} mm", color=C["ok"])
    hmax = max(h for *_, h in D["hat_under"])
    dim(ax, 47, z_ct, z_hat - hmax, f"{z_hat - hmax - z_ct:.1f} mm", color=C["ok"])
    dim(ax, 10.5, z_pt + D["cool_pin_h"], z_mc, f"{gap_conn:.1f} mm", color=C["warn"])
    ax.text(97, z_hat + 14, f"tightest spot: {gap_conn:.1f} mm between a\ncooler push-pin and the motor connectors\n(hatched = 2 mm solder tails under the HAT)",
            ha="left", va="top", fontsize=8, color=C["warn"])
    ax.set_title(f"Active Cooler under the Robot HAT: FITS (tightest {gap_conn:.1f} mm, over the fan {z_tails - z_ct:.1f} mm)", fontsize=13, color=C["ok"], weight="bold", loc="left")
    ax.text(-10, -9, "Side elevation, mm, to scale. Official: Pi 5 + Active Cooler drawings (cooler 13.7 mm overall incl. 3.2 mm of clip under the Pi). "
            "Standoffs: Adeept Lesson 6.\nUnderside layout: Adeept HAT product photo. Estimates: connectors 6 mm (5-7.5), coils 5.5, regulators 4.5, solder tails 2 mm, cooler base 1.2 mm.",
            fontsize=8, color="#555")
    ax.set_xlim(-42, 135); ax.set_ylim(-11, 50); ax.set_aspect("equal"); ax.axis("off")
    fig.tight_layout(); fig.savefig(OUT / "stack-dimensions.png", facecolor="white"); plt.close(fig)


def lidar_drawing():
    fig, axes = plt.subplots(1, 3, figsize=(16, 6.6), dpi=120, sharey=True)
    t = D["plate_t"]; z_a02 = -t - D["so30"] - t; z_ax = z_a02 - D["axle_below_a02"]; z_gnd = z_ax - D["wheel_d"] / 2
    for ax, h in zip(axes, (0.0, 15.0, 30.0)):
        z_a07 = D["nylon_so"]; z_lb = z_a07 + D["a07_t"] + h; z_plane = z_lb + D["lidar_plane"]
        x0, px = D["pi_cx"], D["pan_x"]
        x01 = D["a01_x0"]
        ax.add_patch(Rectangle((x01, -t), D["a01_len"], t, color="#9aa0a6"))                       # A01
        ax.add_patch(Rectangle((x01, z_a02), D["a02_bumper_x"] - x01, t, color=C["deck"]))        # A02
        for xx in (82, 60, -60, -82):
            ax.add_patch(Rectangle((xx - 2.5, z_a02 + t), 5, D["so30"], color=C["brass"]))
        for sx in (-1, 1):
            ax.add_patch(plt.Circle((sx * D["wheel_x"], z_ax), D["wheel_d"] / 2, color="#444"))
        ax.add_patch(Rectangle((-46.5, -t - 20), 77, 20, color="#222"))                            # battery holder
        ax.add_patch(Rectangle((x0 - 28, 6), 56, 1.6, color=C["pi"]))
        ax.add_patch(Rectangle((x0 - 28, 28.6), 56, 1.6, color=C["hat"]))
        for sx in (-34, 34):
            ax.add_patch(Rectangle((x0 + sx - 3, 0), 6, D["nylon_so"], color="#111"))
        ax.add_patch(Rectangle((x0 - D["a07_len"] / 2, z_a07), D["a07_len"], D["a07_t"], color=C["deck"]))
        # pan/tilt head (measured on Adeept CAD view) + its sweep envelope
        ax.add_patch(Rectangle((px - 11.4, -1), 22.8, 22.7, color="#222"))
        ax.add_patch(Rectangle((px - 14, 26.5), 28, 3, color="#9aa0a6"))
        ax.add_patch(Rectangle((px - 11.4, 33), 22.8, 22.7, color="#222"))
        ax.add_patch(Rectangle((px + 18, 40), 3, D["head_top"] - 40, color="#9aa0a6"))
        ax.add_patch(Rectangle((px + 21, 56), 14, 20, color="#1c4fb0"))
        ax.add_patch(Rectangle((px - D["head_sweep_r"], 12), 2 * D["head_sweep_r"], D["head_tilt_top"] - 12, facecolor=C["warn"], alpha=0.12, edgecolor=C["warn"], ls="--"))
        ax.text(px, D["head_tilt_top"] + 2, f"camera head top ~{D['head_top']:.0f} mm\n(~{D['head_tilt_top']:.0f} when tilted)", ha="center", fontsize=8, color=C["warn"])
        if h > 0:
            for sx in (-21.5, 21.5):
                ax.add_patch(Rectangle((x0 + sx - 2.5, z_a07 + D["a07_t"]), 5, h, color=C["brass"]))
        b = D["lidar_base"]
        ax.add_patch(Rectangle((x0 - b / 2, z_lb), b, D["lidar_base_h"], color=C["lidar"]))
        ax.add_patch(Rectangle((x0 - D["lidar_dome_r"], z_lb + D["lidar_base_h"]), 2 * D["lidar_dome_r"], D["lidar_total_h"] - D["lidar_base_h"], color="#555"))
        ax.plot([-140, 150], [z_plane, z_plane], color=C["laser"], lw=2)
        margin = z_plane - D["head_tilt_top"]
        verdict, col = ("BLOCKED: head sits in the beam", C["warn"]) if margin <= 0 else (
            ("RECOMMENDED", C["ok"]) if margin < 20 else ("works, but higher than needed", "#868e96"))
        ax.text(-138, z_plane + 2, f"laser plane {z_plane - z_gnd:.0f} mm above the floor", color=C["laser"], fontsize=9)
        ax.set_title(f"{h:.0f} mm standoffs: {verdict}\nbeam is {margin:+.0f} mm vs. the tilted head", color=col, fontsize=12, weight="bold")
        ax.set_xlim(-140, 150); ax.set_ylim(z_gnd - 5, 145); ax.set_aspect("equal"); ax.axis("off")
        ax.plot([-140, 150], [z_gnd] * 2, color="#999", lw=1)
    fig.suptitle("RPLIDAR C1 on top plate A07: how high must it sit to see over the camera head?", fontsize=14, weight="bold", x=0.01, ha="left")
    fig.text(0.01, 0.02, "Side view, mm, to scale, robot facing right. LiDAR: official Slamtec drawing (laser plane 29.8 mm above its base). "
             "Chassis and head height scaled from Adeept CAD views (+-5 mm): measure the real kit.", fontsize=9, color="#555")
    fig.tight_layout(rect=(0, 0.04, 1, 0.94)); fig.savefig(OUT / "lidar-height-options.png", facecolor="white"); plt.close(fig)


stack_drawing()
lidar_drawing()
print("written to", OUT)
