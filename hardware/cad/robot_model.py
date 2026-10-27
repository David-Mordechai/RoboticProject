# FreeCAD script: Adeept ADR036 mecanum robot + Raspberry Pi 5 + Active Cooler + Robot HAT V3.2 + RPLIDAR C1.
# Run inside FreeCAD (python fc.py run robot_model.py) or paste into FreeCAD's Python console.
#
# Frame: x = forward, y = left, z = up, mm. z = 0 is the TOP surface of the main deck plate A01.
# Every dimension is tagged:
#   OFFICIAL  - from a manufacturer drawing in docs/vendor/
#   ADEEPT    - from the Adeept assembly lesson (part names/standoff lengths), docs/vendor/adeept/tutorial
#   EST       - estimated from Adeept's renders; MEASURE ON THE REAL KIT and update here.
import FreeCAD as App
import Part

V = App.Vector

D = dict(
    # --- Raspberry Pi 5 (OFFICIAL: raspberry-pi-5-mechanical-drawing.pdf) ---
    pi_len=85.0, pi_wid=56.0, pcb_t=1.6,
    pi_holes=[(3.5, 3.5), (61.5, 3.5), (3.5, 52.5), (61.5, 52.5)],
    gpio_pin_h=8.5,            # standard 2.54 mm male header above PCB (typical)
    usb_h=16.0, eth_h=13.5,    # connector heights above PCB (typical)
    # --- Active Cooler (OFFICIAL: raspberry-pi-active-cooler-product-brief.pdf p5) ---
    # The 13.7 mm "overall" height includes 3.2 mm of push-pin BELOW the heatsink base (through the PCB).
    # Measured from the vector drawing: fins/blower top = 9.3 mm and push-pin heads = 10.5 mm above the base;
    # +1.2 mm EST for the base resting on the SoC + thermal pad.
    cool_len=63.5, cool_wid=42.5, cool_h=10.5, cool_pin_h=11.7,
    # --- Underside of the HAT (ADEEPT Lesson 6 p32 "back of the board") ---
    motor_conn_h=6.0,          # EST: 4 white side-entry motor connectors M1-M4 hang below the HAT (5-7.5 mm)
    motor_conn_u=(0.0, 9.0), motor_conn_v=(6.0, 50.0),   # along the HAT's pin-1 short edge (EST span)
    # Other underside parts, positions measured from Adeept's labelled product photo of the HAT underside
    # (docs/vendor/adeept/photos/robot-hat-v3.3-top-and-bottom-labeled.jpg; V3.3, kit is V3.2 -> same layout
    # per Adeept's V3.2 back-side drawing, exact parts may differ). (name, u0, u1, v0, v1, height) heights EST.
    hat_under=[
        ("DCDC_reg_1_XL4015", 54.7, 65.0, 29.8, 40.8, 4.5), ("DCDC_reg_2_XL4015", 54.7, 65.0, 7.2, 17.6, 4.5),
        ("DCDC_inductor_1", 43.8, 51.4, 29.8, 40.1, 5.5), ("DCDC_inductor_2", 47.4, 55.2, 18.2, 28.6, 5.5),
        ("DCDC_diode_1", 57.5, 64.5, 19.4, 26.7, 2.4), ("DCDC_diode_2", 47.0, 51.0, 7.8, 17.6, 2.4),
        ("PCA9685", 28.0, 31.5, 24.0, 34.0, 1.2), ("DRV8833_1", 14.5, 18.5, 30.0, 35.0, 1.2),
        ("DRV8833_2", 14.5, 18.5, 22.0, 27.0, 1.2), ("ADC_ADS7830", 39.5, 42.0, 11.0, 16.0, 1.2),
    ],
    # --- Stack (ADEEPT Lesson 6 steps 6.1-1 and 6.3-5) ---
    so_lower=6.0,              # M2.5*6+6 copper standoff, deck -> Pi
    so_upper=21.0,             # M2.5*21 copper standoff, Pi -> HAT
    hat_len=65.0, hat_t=1.6,   # HAT footprint 65x56 (HAT form factor, OFFICIAL hat-plus-specification)
    hat_tails=2.0,             # EST: through-hole solder tails under the HAT
    hat_cap_h=12.5,            # EST: 470uF/35V can (10x12.5)
    nylon_so=50.0,             # ADEEPT: M3*50 nylon standoffs deck -> top plate A07
    a07_t=3.0, a07_len=80.0, a07_wid=100.0,      # thickness ADEEPT-typical, size EST
    # --- Chassis. Structure from Adeept Lesson 6; sizes scaled from Adeept's CAD views using the
    #     2x18650 holder (77x41 mm) and the 50 mm nylon standoffs as references (+-5 mm). EST = measure! ---
    plate_t=3.0,
    a01_len=185.0, a01_wid=114.0, a01_waist=82.0, a01_waist_len=111.0, a01_x0=-91.0,   # A01 upper deck (EST)
    so30=30.0,                 # ADEEPT: 8x M3*30 copper standoffs A01 -> A02
    a02_bumper_x=120.0, a02_bumper_wid=148.0,   # A02 front wing tip / span (EST)
    a02_window=(-54.5, 38.5, 24.0),             # A02 central window x0, x1, half-width (EST)
    wheel_d=60.0, wheel_w=26.0, wheel_x=67.0, wheel_y=72.0, axle_below_a02=12.0,   # EST (wheelbase 134)
    pi_cx=-12.0,               # Pi centre along x (EST)
    # --- Pan/tilt head, measured on Adeept's CAD view against the 50 mm nylon standoffs (EST +-5 mm) ---
    pan_x=70.0, head_top=83.0, head_tilt_top=85.0, head_sweep_r=40.0,
    # --- RPLIDAR C1 (OFFICIAL: SLAMTEC_rplidar_datasheet_C1_v1.0_en.pdf Fig 4-1) ---
    lidar_base=55.6, lidar_base_h=23.1, lidar_total_h=41.3, lidar_plane=29.8, lidar_hole=43.0,
    lidar_dome_r=24.0,         # EST from drawing proportions
)

COL = dict(
    acrylic=(0.15, 0.15, 0.17), plate=(0.78, 0.79, 0.80), plate_dark=(0.30, 0.31, 0.33), pcb_pi=(0.10, 0.55, 0.25), pcb_hat=(0.50, 0.20, 0.65),
    brass=(0.85, 0.65, 0.20), nylon=(0.05, 0.05, 0.05), alu=(0.75, 0.77, 0.80),
    metal=(0.70, 0.70, 0.72), black=(0.08, 0.08, 0.08), motor=(0.95, 0.80, 0.10),
    wheel=(0.20, 0.20, 0.22), servo=(0.15, 0.25, 0.75), sensor=(0.10, 0.45, 0.85),
    laser=(1.0, 0.10, 0.10), danger=(1.0, 0.35, 0.0), fan=(0.25, 0.25, 0.25), lidar=(0.12, 0.12, 0.12),
)


def build(doc_name="Robot", lidar_h=15.0, with_lidar=True, stack_only=False):
    if doc_name in App.listDocuments():
        App.closeDocument(doc_name)
    doc = App.newDocument(doc_name)

    def add(name, shape, color, transp=0):
        o = doc.addObject("Part::Feature", name)
        o.Shape = shape
        if App.GuiUp:
            o.ViewObject.ShapeColor = color
            o.ViewObject.Transparency = transp
        return o

    def box(x0, y0, z0, dx, dy, dz):
        return Part.makeBox(dx, dy, dz, V(x0, y0, z0))

    def cyl_z(x, y, z0, r, h):
        return Part.makeCylinder(r, h, V(x, y, z0), V(0, 0, 1))

    def cyl(p, d, r, h):
        return Part.makeCylinder(r, h, p, d)

    def dogbone(x0, z0):
        """A01/A02 outline: full-width ends, a narrower waist between them (wheel clearance)."""
        L, W, w, wl, t = D["a01_len"], D["a01_wid"], D["a01_waist"], D["a01_waist_len"], D["plate_t"]
        end = (L - wl) / 2
        shp = box(x0, -W / 2, z0, end, W, t).fuse(box(x0 + L - end, -W / 2, z0, end, W, t))
        return shp.fuse(box(x0 + end, -w / 2, z0, wl, w, t)).removeSplitter()

    def mecanum(cx, cy, cz, right_hand):
        """Hub + 9 rollers at 45 deg; right_hand flips the roller angle (X pattern seen from above)."""
        import math
        R, wdt = D["wheel_d"] / 2, D["wheel_w"]
        hub = cyl(V(cx, cy - wdt / 2 + 3, cz), V(0, 1, 0), R - 8, wdt - 6)
        rollers = []
        sgn = 1 if right_hand else -1
        for k in range(10):
            th = 2 * math.pi * k / 10
            radial = V(math.sin(th), 0, math.cos(th))
            tang = V(math.cos(th), 0, -math.sin(th))
            d = tang * 0.7071 + V(0, sgn * 0.7071, 0)
            c = V(cx, cy, cz) + radial * (R - 3.8)
            L = wdt * 1.25
            rollers.append(cyl(c - d * (L / 2), d, 3.6, L))
        return hub, rollers

    # Pi-local -> global. Pi local u = along 85 mm (USB end at u=85), v = along 56 mm (GPIO edge at v=56).
    # Pi is mounted with its long axis across the robot (USB/Ethernet facing the robot's left side).
    def pbox(u0, v0, z0, du, dv, dz):
        return box(D["pi_cx"] + (v0 - D["pi_wid"] / 2), u0 - D["pi_len"] / 2, z0, dv, du, dz)

    def pxy(u, v):
        return D["pi_cx"] + (v - D["pi_wid"] / 2), u - D["pi_len"] / 2

    z_pi = D["so_lower"]                       # Pi PCB bottom
    z_pi_top = z_pi + D["pcb_t"]               # Pi PCB top
    z_hat = z_pi_top + D["so_upper"]           # HAT PCB bottom
    z_hat_top = z_hat + D["hat_t"]
    z_cool_top = z_pi_top + D["cool_h"]
    z_a07 = D["nylon_so"]

    # ---- Deck A01 (local section only when stack_only) ----
    if stack_only:
        add("A01_deck", box(D["pi_cx"] - 40, -55, -D["plate_t"], 80, 110, D["plate_t"]), COL["plate"])
    else:
        add("A01_deck", dogbone(D["a01_x0"], -D["plate_t"]).cut(box(D["pan_x"] - 12, -7, -5, 24, 14, 10)), COL["plate"])

    # ---- Standoffs ----
    for i, (u, v) in enumerate(D["pi_holes"]):
        x, y = pxy(u, v)
        add(f"SO6_{i}", cyl_z(x, y, 0, 2.5, D["so_lower"]), COL["brass"])
        add(f"SO21_{i}", cyl_z(x, y, z_pi_top, 2.5, D["so_upper"]), COL["brass"])

    # ---- Raspberry Pi 5 ----
    add("Pi5_PCB", pbox(0, 0, z_pi, D["pi_len"], D["pi_wid"], D["pcb_t"]), COL["pcb_pi"])
    add("Pi5_GPIO_pins", pbox(7.1, 49.5, z_pi_top, 50.8, 5.0, D["gpio_pin_h"]), COL["black"])
    add("Pi5_Ethernet", pbox(65.0, 2.0, z_pi_top, 23.0, 16.0, D["eth_h"]), COL["metal"])
    add("Pi5_USB_a", pbox(68.0, 22.6, z_pi_top, 20.0, 13.1, D["usb_h"]), COL["metal"])
    add("Pi5_USB_b", pbox(68.0, 40.4, z_pi_top, 20.0, 13.1, D["usb_h"]), COL["metal"])

    # ---- Active Cooler: fin block + blower housing with fan ----
    cu0, cv0 = 3.0, 3.0
    fins = pbox(cu0, cv0, z_pi_top, 31.0, D["cool_wid"], D["cool_h"])
    housing = pbox(cu0 + 31.0, cv0, z_pi_top, D["cool_len"] - 31.0, D["cool_wid"], D["cool_h"])
    add("ActiveCooler_fins", fins, COL["alu"])
    add("ActiveCooler_blower", housing, COL["fan"])
    fx, fy = pxy(cu0 + 31.0 + 16.0, cv0 + 25.0)
    add("ActiveCooler_fan", cyl_z(fx, fy, z_cool_top - 0.5, 12.5, 0.8), COL["black"])
    pins = [(cu0 + 3.0, cv0 + 3.0), (cu0 + D["cool_len"] - 3.0, cv0 + D["cool_wid"] - 3.0)]  # EST positions (drawing)
    for i, (u, v) in enumerate(pins):
        x, y = pxy(u, v)
        add(f"ActiveCooler_pushpin_{i}", cyl_z(x, y, z_pi_top, 3.0, D["cool_pin_h"]), (0.95, 0.95, 0.95))

    # ---- Robot HAT V3.2 (transparent so you can see the cooler underneath) ----
    add("HAT_PCB", pbox(0, 0, z_hat, D["hat_len"], D["pi_wid"], D["hat_t"]), COL["pcb_hat"], 45)
    add("HAT_solder_tails_zone", pbox(0, 0, z_hat - D["hat_tails"], D["hat_len"], D["pi_wid"], D["hat_tails"]), COL["danger"], 80)
    (u0, u1), (v0, v1) = D["motor_conn_u"], D["motor_conn_v"]
    pitch = (v1 - v0) / 4
    for i in range(4):
        add(f"HAT_motor_conn_M{i + 1}", pbox(u0, v0 + i * pitch + 1, z_hat - D["motor_conn_h"], u1 - u0, pitch - 2, D["motor_conn_h"]), (0.95, 0.95, 0.92))
    clear = {}
    for name, a0, a1, b0, b1, h in D["hat_under"]:
        add(f"HAT_under_{name}", pbox(a0, b0, z_hat - h, a1 - a0, b1 - b0, h), (0.12, 0.12, 0.12) if "reg" in name or "ind" in name else (0.2, 0.2, 0.2))
        over_cooler = a1 > 3.0 and a0 < 3.0 + D["cool_len"] and b1 > 3.0 and b0 < 3.0 + D["cool_wid"]
        if over_cooler:
            clear[name] = z_hat - h - z_cool_top
    add("HAT_tall_header", pbox(7.1, 49.5, z_pi_top + D["gpio_pin_h"] - 3, 50.8, 5.0, z_hat - (z_pi_top + D["gpio_pin_h"] - 3)), COL["black"])
    for i, (u, v) in enumerate([(52.0, 32.0), (52.0, 18.0)]):
        x, y = pxy(u, v)
        add(f"HAT_cap_{i}", cyl_z(x, y, z_hat_top, 5.0, D["hat_cap_h"]), COL["black"])
    for i, u in enumerate([8, 18, 28, 38]):
        add(f"HAT_servo_hdr_{i}", pbox(u, 3.0, z_hat_top, 8.0, 7.5, 8.5), COL["black"])

    report = {
        "pi_pcb_top": z_pi_top, "cooler_top": z_cool_top, "hat_bottom": z_hat,
        "air_gap_nominal": z_hat - z_cool_top,
        "air_gap_worst": z_hat - D["hat_tails"] - z_cool_top,
        # push-pin head at the fin corner sits under the motor connectors (EST layout)
        "motor_conn_over_pushpin": z_hat - D["motor_conn_h"] - (z_pi_top + D["cool_pin_h"]),
        "hat_top_parts": z_hat_top + D["hat_cap_h"], "a07_bottom": z_a07,
    }
    report.update({f"gap_cooler_to_{k}": v for k, v in clear.items()})

    if not stack_only:
        # ---- Top plate A07 on 4 nylon M3x50 standoffs (ADEEPT) ----
        for i, (dx, dy) in enumerate([(-34, -47), (34, -47), (-34, 47), (34, 47)]):
            add(f"Nylon50_{i}", cyl_z(D["pi_cx"] + dx, dy, 0, 3.0, D["nylon_so"]), COL["nylon"])
        add("A07_top_plate", box(D["pi_cx"] - D["a07_len"] / 2, -D["a07_wid"] / 2, z_a07, D["a07_len"], D["a07_wid"], D["a07_t"]), COL["acrylic"], 35)

        # ---- Lower plate A02 (motors underneath) on 8 copper M3x30 standoffs (ADEEPT) ----
        t = D["plate_t"]
        z_a02 = -t - D["so30"] - t            # A02 bottom face
        front = D["a01_x0"] + D["a01_len"]
        a02 = dogbone(D["a01_x0"], z_a02)
        a02 = a02.fuse(box(front, -35, z_a02, 12, 70, t))                                        # neck
        a02 = a02.fuse(box(front + 12, -D["a02_bumper_wid"] / 2, z_a02,
                           D["a02_bumper_x"] - front - 12, D["a02_bumper_wid"], t))                # front wings
        wx0, wx1, ww = D["a02_window"]
        a02 = a02.cut(box(wx0, -ww, z_a02 - 1, wx1 - wx0, 2 * ww, t + 2))
        add("A02_lower_plate", a02.removeSplitter(), COL["plate_dark"])
        for i, (x, y) in enumerate([(x, y) for x in (82, 60, -60, -82) for y in (-50, 50)]):
            add(f"Copper30_{i}", cyl_z(x, y, z_a02 + t, 2.6, D["so30"]), COL["brass"])

        # ---- Battery holder (2x 18650) hanging under A01 into A02's window ----
        add("Battery_holder", box(-46.5, -20.5, -t - 20, 77, 41, 20), COL["black"])
        for i, yy in enumerate((-10, 10)):
            add(f"Cell_18650_{i}", cyl(V(-39.0, yy, -t - 10), V(1, 0, 0), 9.0, 65.0), (0.15, 0.35, 0.85) if i else (0.85, 0.2, 0.2))

        # ---- 4x TT gear motors on brackets under A02, mecanum wheels in X pattern ----
        z_ax = z_a02 - D["axle_below_a02"]
        for sx, sy, name in ((1, 1, "FL"), (1, -1, "FR"), (-1, 1, "RL"), (-1, -1, "RR")):
            xs = sx * D["wheel_x"]
            y_out = sy * 57.0                                   # gearbox outer face at the plate edge
            y0 = y_out - 18.8 if sy > 0 else y_out
            # gearbox: 37 long, shaft 11 mm from its outer end; motor can points toward the robot centre
            gx0 = xs - 26 if sx > 0 else xs - 11
            add(f"TT_gearbox_{name}", box(gx0, y0, z_ax - 11.25, 37, 18.8, 22.5), COL["motor"])
            add(f"TT_motorcan_{name}", box(gx0 - 25 if sx > 0 else gx0 + 37, y0 + 2, z_ax - 9, 25, 15, 18), COL["metal"])
            yb = y0 - 3 if sy > 0 else y0 + 18.8
            add(f"Bracket_{name}", box(xs - 10, yb, z_ax - 11, 20, 3, z_a02 - (z_ax - 11)), COL["alu"])
            add(f"Shaft_{name}", cyl(V(xs, y_out, z_ax), V(0, sy, 0), 2.7, D["wheel_y"] - 57.0), (0.95, 0.95, 0.95))
            hub, rollers = mecanum(xs, sy * D["wheel_y"], z_ax, right_hand=(sx * sy > 0))
            add(f"Wheel_hub_{name}", hub, COL["wheel"])
            add(f"Wheel_rollers_{name}", Part.makeCompound(rollers), (0.35, 0.35, 0.37))
        report["ground_z"] = z_ax - D["wheel_d"] / 2

        # ---- Modules: line tracker under A02, WS2812 + light tracker at the front, LED matrix at the rear, OLED on A01 ----
        add("LineTracker_3ch", box(74, -25, z_a02 - 13, 40, 50, 1.6), COL["black"])
        for i, yy in enumerate((-15, 0, 15)):
            add(f"LineTracker_IR_{i}", cyl_z(100, yy, z_a02 - 17, 2.5, 4), (0.1, 0.1, 0.6))
        add("WS2812_strip", box(D["a02_bumper_x"], -25, z_a02 + t, 1.6, 50, 10), COL["black"])
        add("LightTracker", box(D["a02_bumper_x"] - 12, -45, z_a02 - 1.6, 10, 90, 1.6), COL["black"])
        add("A06_matrix_bracket", box(-104, -33, z_a02 + t, 3, 66, 46), COL["plate_dark"])
        add("LED_matrix_8x16", box(-111, -32, z_a02 + t + 4, 7, 64, 38), (0.95, 0.35, 0.45))
        add("OLED_module", box(-80, 8, 11, 27, 27, 1.6), (0.15, 0.25, 0.7))
        add("OLED_screen", box(-77, 12, 12.6, 21, 15, 1.0), (0.05, 0.05, 0.1))
        for i, (dx, dy) in enumerate([(2, 2), (25, 2), (2, 25), (25, 25)]):
            add(f"OLED_SO_{i}", cyl_z(-80 + dx, 8 + dy, 0, 1.6, 11), COL["brass"])

        # ---- Pan/tilt head: AD002 servos, A03/A04/A05 brackets, ultrasonic on top, camera below it ----
        px = D["pan_x"]
        for i, yy in enumerate((-9, 9)):
            add(f"Head_SO12_{i}", cyl_z(px - 14, yy, 0, 2.6, 12), COL["brass"])
        add("A03_pan_bracket", box(px - 18, -14, 12, 34, 28, 3), COL["plate"])
        add("Servo_pan_AD002", box(px - 11.4, -6.1, -1, 22.8, 12.2, 22.7), COL["black"])
        add("Servo_pan_horn", cyl_z(px, 0, 21.7, 3.5, 4.5), (0.9, 0.9, 0.9))
        add("A04_tilt_bracket", box(px - 14, -16, 26.5, 28, 32, 3).fuse(box(px - 14, 13, 26.5, 28, 3, 32)), COL["plate"])
        add("Servo_tilt_AD002", box(px - 11.4, -2, 33, 22.8, 12.2, 22.7), COL["black"])
        face = box(px + 18, -25, 40, 3, 50, 36)
        for yy in (-25, 17):
            face = face.fuse(box(px + 18, yy, 76, 3, 8, D["head_top"] - 76))   # the "cat ears" are the highest point
        add("A05_head_face", face.removeSplitter(), COL["plate"])
        add("Ultrasonic_HC-SR04", box(px + 21, -22.5, 56, 1.6, 45, 20), (0.1, 0.3, 0.75))
        for i, yy in enumerate((-13, 13)):
            add(f"Ultrasonic_can_{i}", cyl(V(px + 22.6, yy, 66), V(1, 0, 0), 8, 12), (0.82, 0.82, 0.85))
        add("Camera_board_OV5647", box(px + 15.4, -12.5, 41, 1.6, 25, 24), COL["pcb_pi"])
        add("Camera_lens", cyl(V(px + 17, 0, 50), V(1, 0, 0), 4, 7), COL["black"])
        add("Head_sweep_envelope", cyl_z(px, 0, 12, D["head_sweep_r"], D["head_tilt_top"] - 12), COL["danger"], 88)
        report["head_top"] = D["head_top"]
        report["head_tilt_top"] = D["head_tilt_top"]

        # ---- RPLIDAR C1 on standoffs above A07 ----
        if with_lidar:
            z_lb = z_a07 + D["a07_t"] + lidar_h
            b, cx = D["lidar_base"], D["pi_cx"]
            if lidar_h > 0:
                for i, (dx, dy) in enumerate([(-1, -1), (1, -1), (-1, 1), (1, 1)]):
                    add(f"Lidar_SO_{i}", cyl_z(cx + dx * D["lidar_hole"] / 2, dy * D["lidar_hole"] / 2, z_a07 + D["a07_t"], 2.5, lidar_h), COL["brass"])
            add("RPLIDAR_C1_base", box(cx - b / 2, -b / 2, z_lb, b, b, D["lidar_base_h"]), COL["lidar"])
            add("RPLIDAR_C1_dome", cyl_z(cx, 0, z_lb + D["lidar_base_h"], D["lidar_dome_r"], D["lidar_total_h"] - D["lidar_base_h"]), COL["lidar"])
            z_plane = z_lb + D["lidar_plane"]
            add("Laser_scan_plane", cyl_z(cx, 0, z_plane - 0.3, 170, 0.6), COL["laser"], 82)
            report.update(lidar_standoff=lidar_h, lidar_plane=z_plane,
                          plane_over_head=z_plane - D["head_top"],
                          plane_over_head_tilted=z_plane - D["head_tilt_top"])

    doc.recompute()
    if App.GuiUp:
        import FreeCADGui as Gui
        Gui.ActiveDocument.ActiveView.setCameraType("Orthographic")
        Gui.ActiveDocument.ActiveView.viewIsometric()
        Gui.SendMsgToActiveView("ViewFit")
    return doc, report


if __name__ == "__main__" or True:
    _cfg = globals().pop("CFG", {})
    _doc, _rep = build(**_cfg)
    print({k: round(v, 1) for k, v in _rep.items()})
