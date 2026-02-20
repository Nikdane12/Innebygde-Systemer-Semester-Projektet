from tkinter import *
import math

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import proj3d

# -------------------------
# Homogeneous transform utils (4x4)
# -------------------------
def T_identity():
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]

def T_mul(A, B):
    C = [[0.0]*4 for _ in range(4)]
    for r in range(4):
        for c in range(4):
            C[r][c] = A[r][0]*B[0][c] + A[r][1]*B[1][c] + A[r][2]*B[2][c] + A[r][3]*B[3][c]
    return C

def T_trans(x, y, z):
    T = T_identity()
    T[0][3] = float(x)
    T[1][3] = float(y)
    T[2][3] = float(z)
    return T

def T_roty(rad):
    c, s = math.cos(rad), math.sin(rad)
    return [
        [ c,  0.0,  s,  0.0],
        [0.0, 1.0, 0.0, 0.0],
        [-s,  0.0,  c,  0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]

def T_rotz(rad):
    c, s = math.cos(rad), math.sin(rad)
    return [
        [ c,  -s,  0.0, 0.0],
        [ s,   c,  0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]

def deg2rad(d): return d * math.pi / 180.0
def rad2deg(r): return r * 180.0 / math.pi
def clamp(v, lo, hi): return max(lo, min(hi, v))

def wrap180(deg):
    """Wrap angle to [-180, 180). Helps wrist choose positive/negative nicely."""
    while deg >= 180:
        deg -= 360
    while deg < -180:
        deg += 360
    return deg

def point_from_T(T):
    return (T[0][3], T[1][3], T[2][3])

# -------------------------
# Robot geometry
# -------------------------
BOX_H = 0.35
L1 = 1.00
L2 = 0.80
L3 = 0.60

SHOULDER_ZERO_OFFSET_DEG = 0  # slider 0 => actual shoulder -45°

# Slider limits (match your Scale widgets)
YAW_MIN, YAW_MAX = -90, 90
SH_MIN, SH_MAX = -90, 90
EL_MIN, EL_MAX = -135, 135
WR_MIN, WR_MAX = -135, 135

# -------------------------
# Forward kinematics
# -------------------------
def fk_chain(yaw_deg, shoulder_slider_deg, elbow_deg, wrist_deg):
    yaw = deg2rad(yaw_deg)
    sh  = deg2rad(shoulder_slider_deg + SHOULDER_ZERO_OFFSET_DEG)
    el  = deg2rad(elbow_deg)
    wr  = deg2rad(wrist_deg)

    T_O = T_identity()
    T_yaw = T_rotz(yaw)
    T_OA = T_trans(0, 0, BOX_H)

    T_AB = T_mul(T_roty(sh), T_trans(L1, 0, 0))
    T_BC = T_mul(T_roty(el), T_trans(L2, 0, 0))
    T_CD = T_mul(T_roty(wr), T_trans(L3, 0, 0))

    T_A = T_mul(T_mul(T_O, T_yaw), T_OA)
    T_B = T_mul(T_A, T_AB)
    T_C = T_mul(T_B, T_BC)
    T_D = T_mul(T_C, T_CD)
    return T_O, T_A, T_B, T_C, T_D

# -------------------------
# Disable 3D mouse rotation/pan/zoom reliably
# -------------------------
def disable_3d_mouse(ax, fig):
    for attr in [
        "_rotate_cid", "_zoom_cid",
        "_button_press_cid", "_button_release_cid", "_motion_notify_cid"
    ]:
        cid = getattr(ax, attr, None)
        if cid is not None:
            try:
                fig.canvas.mpl_disconnect(cid)
            except Exception:
                pass
    try:
        ax.mouse_init(rotate_btn=None, zoom_btn=None)
    except Exception:
        pass

# -------------------------
# Limit-aware 3-link IK in the arm plane
# -------------------------
def ik_3link_plane_best(r_forward, z_world):
    """
    We want D at (px, pz) relative to A.
    We search over tool angle and elbow branch, respecting joint limits,
    and pick the best solution (smallest position error).

    Returns: shoulder_slider_deg, elbow_deg, wrist_deg
    """
    r_forward = max(0.0, r_forward)
    px = r_forward
    pz = z_world - BOX_H

    best = None  # (cost, sh_slider, el_deg, wr_deg)

    # Search tool angle around pointing-to-target, but allow adjustment
    base_tool = math.atan2(pz, px if px > 1e-9 else 1e-9)
    tool_candidates = []
    # Dense-ish local search + a wider sweep
    for ddeg in range(-90, 91, 5):
        tool_candidates.append(base_tool + deg2rad(ddeg))
    for ddeg in range(-180, 181, 15):
        tool_candidates.append(deg2rad(ddeg))

    # Try both elbow branches
    for tool in tool_candidates:
        # Compute C target
        cx = px - L3 * math.cos(tool)
        cz = pz - L3 * math.sin(tool)

        a, b = L1, L2
        d2 = cx*cx + cz*cz
        d = math.sqrt(d2)

        # If C is unreachable by L1+L2, skip
        if d > a + b or d < abs(a - b):
            continue

        # 2-link IK
        cos_el = (d2 - a*a - b*b) / (2*a*b)
        cos_el = clamp(cos_el, -1.0, 1.0)
        el0 = math.acos(cos_el)  # 0..pi

        for elbow_rel in (el0, -el0):  # elbow-down and elbow-up style
            sh = math.atan2(cz, cx) - math.atan2(b*math.sin(elbow_rel), a + b*math.cos(elbow_rel))
            wr = tool - sh - elbow_rel

            sh_deg_actual = rad2deg(sh)
            el_deg = rad2deg(elbow_rel)
            wr_deg = wrap180(rad2deg(wr))

            sh_slider = sh_deg_actual - SHOULDER_ZERO_OFFSET_DEG

            # Enforce limits (same as sliders)
            if not (SH_MIN <= sh_slider <= SH_MAX):
                continue
            if not (EL_MIN <= el_deg <= EL_MAX):
                continue
            if not (WR_MIN <= wr_deg <= WR_MAX):
                continue

            # Compute resulting D error (in plane) for ranking
            # Forward kinematics in plane:
            # direction sums: sh, sh+el, sh+el+wr
            th1 = sh
            th2 = sh + elbow_rel
            th3 = sh + elbow_rel + deg2rad(wr_deg)

            x_fk = L1*math.cos(th1) + L2*math.cos(th2) + L3*math.cos(th3)
            z_fk = L1*math.sin(th1) + L2*math.sin(th2) + L3*math.sin(th3)

            err = math.hypot(x_fk - px, z_fk - pz)
            # Small preference to not slam joints:
            penalty = 0.001*(abs(sh_slider) + abs(el_deg) + abs(wr_deg))
            cost = err + penalty

            if best is None or cost < best[0]:
                best = (cost, sh_slider, el_deg, wr_deg)

    # If nothing fits limits, fall back to clamped "point at target" solution (still better than nothing)
    if best is None:
        tool = math.atan2(pz, px if px > 1e-9 else 1e-9)
        cx = px - L3 * math.cos(tool)
        cz = pz - L3 * math.sin(tool)
        a, b = L1, L2
        d2 = cx*cx + cz*cz
        d = clamp(math.sqrt(d2), 1e-6, a + b - 1e-6)
        cos_el = (d2 - a*a - b*b) / (2*a*b)
        cos_el = clamp(cos_el, -1.0, 1.0)
        elbow_rel = math.acos(cos_el)
        sh = math.atan2(cz, cx) - math.atan2(b*math.sin(elbow_rel), a + b*math.cos(elbow_rel))
        wr = tool - sh - elbow_rel

        sh_slider = clamp(rad2deg(sh) - SHOULDER_ZERO_OFFSET_DEG, SH_MIN, SH_MAX)
        el_deg = clamp(rad2deg(elbow_rel), EL_MIN, EL_MAX)
        wr_deg = clamp(wrap180(rad2deg(wr)), WR_MIN, WR_MAX)
        return sh_slider, el_deg, wr_deg

    _, sh_slider, el_deg, wr_deg = best
    return sh_slider, el_deg, wr_deg

# -------------------------
# Tkinter + Matplotlib
# -------------------------
root = Tk()
root.title("3D Arm Visualizer (Drag D + Wrist)")
root.geometry("980x650")

fig = Figure(figsize=(7.2, 5.4))
ax = fig.add_subplot(111, projection="3d")

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=LEFT, fill=BOTH, expand=True)

controls = Frame(root)
controls.pack(side=RIGHT, fill=Y, padx=12, pady=12)

info = Label(controls, text="", justify=LEFT)
info.pack(pady=8)

yaw_scale = Scale(controls, from_=YAW_MIN, to=YAW_MAX, orient=HORIZONTAL, label="Base yaw (-90..+90)", length=250)
yaw_scale.set(0)
yaw_scale.pack(fill="x")

shoulder_scale = Scale(controls, from_=-90, to=90, orient=HORIZONTAL,
                       label="Shoulder pitch (actual)", length=250)
shoulder_scale.set(0)
shoulder_scale.pack(fill="x")

elbow_scale = Scale(controls, from_=EL_MIN, to=EL_MAX, orient=HORIZONTAL, label="Elbow pitch (relative)", length=250)
elbow_scale.set(0)
elbow_scale.pack(fill="x")

wrist_scale = Scale(controls, from_=WR_MIN, to=WR_MAX, orient=HORIZONTAL, label="Wrist pitch (relative)", length=250)
wrist_scale.set(0)
wrist_scale.pack(fill="x")

disable_3d_mouse(ax, fig)

dragging = False
last_mouse = None
internal_update = False

target = {"r": 0.0, "z": BOX_H + 0.5}

def current_D():
    yaw = yaw_scale.get()
    shs = shoulder_scale.get()
    el  = elbow_scale.get()
    wr  = wrist_scale.get()
    *_, T_D = fk_chain(yaw, shs, el, wr)
    return point_from_T(T_D)

def get_D_screen_xy():
    yaw = yaw_scale.get()
    shs = shoulder_scale.get()
    el  = elbow_scale.get()
    wr  = wrist_scale.get()
    *_, T_D = fk_chain(yaw, shs, el, wr)
    Dx, Dy, Dz = point_from_T(T_D)
    x2, y2, _ = proj3d.proj_transform(Dx, Dy, Dz, ax.get_proj())
    px, py = ax.transData.transform((x2, y2))
    return px, py

def draw():
    ax.clear()

    yaw = yaw_scale.get()
    shs = shoulder_scale.get()
    el  = elbow_scale.get()
    wr  = wrist_scale.get()

    T_O, T_A, T_B, T_C, T_D = fk_chain(yaw, shs, el, wr)

    O = point_from_T(T_O)
    A = point_from_T(T_A)
    B = point_from_T(T_B)
    C = point_from_T(T_C)
    D = point_from_T(T_D)

    xs = [O[0], A[0], B[0], C[0], D[0]]
    ys = [O[1], A[1], B[1], C[1], D[1]]
    zs = [O[2], A[2], B[2], C[2], D[2]]
    ax.plot(xs, ys, zs, marker="o", linewidth=3)

    yaw_r = deg2rad(yaw)
    tx = target["r"] * math.cos(yaw_r)
    ty = target["r"] * math.sin(yaw_r)
    tz = target["z"]
    ax.scatter([tx], [ty], [tz], marker="x", s=80)

    reach = L1 + L2 + L3
    ax.set_xlim(-reach, reach)
    ax.set_ylim(-reach, reach)
    ax.set_zlim(0, BOX_H + reach)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    shoulder_actual = shs + SHOULDER_ZERO_OFFSET_DEG
    info.config(text=
        f"Yaw: {yaw:+d}°\n"
        f"Shoulder slider: {shs:+d}° -> actual: {shoulder_actual:+d}°\n"
        f"Elbow: {el:+d}°\n"
        f"Wrist: {wr:+d}°\n\n"
        f"Target r={target['r']:.3f}, z={target['z']:.3f}\n"
        f"(This IK searches tool angle + elbow branch under limits)"
    )

    canvas.draw()

def on_slider(_=None):
    global internal_update
    if internal_update:
        draw()
        return
    Dx, Dy, Dz = current_D()
    target["r"] = math.hypot(Dx, Dy)
    target["z"] = Dz
    draw()

yaw_scale.config(command=on_slider)
shoulder_scale.config(command=on_slider)
elbow_scale.config(command=on_slider)
wrist_scale.config(command=on_slider)

def on_press(event):
    global dragging, last_mouse
    if event.inaxes != ax:
        return
    if event.x is None or event.y is None:
        return
    Dx, Dy = get_D_screen_xy()
    if math.hypot(event.x - Dx, event.y - Dy) < 20:
        dragging = True
        last_mouse = (event.x, event.y)

def on_release(event):
    global dragging, last_mouse
    dragging = False
    last_mouse = None

def on_motion(event):
    global last_mouse, internal_update
    if not dragging:
        return
    if event.x is None or event.y is None:
        return
    if last_mouse is None:
        last_mouse = (event.x, event.y)
        return

    dx = event.x - last_mouse[0]
    dy = event.y - last_mouse[1]
    last_mouse = (event.x, event.y)

    gain = (L1 + L2 + L3) / 350.0
    target["r"] = max(0.0, target["r"] + dx * gain)
    target["z"] = clamp(target["z"] - dy * gain, 0.0, BOX_H + (L1 + L2 + L3))

    shs_d, el_d, wr_d = ik_3link_plane_best(target["r"], target["z"])

    internal_update = True
    shoulder_scale.set(int(round(shs_d)))
    elbow_scale.set(int(round(el_d)))
    wrist_scale.set(int(round(wr_d)))
    internal_update = False

    draw()

def reset_all():
    global internal_update
    internal_update = True
    yaw_scale.set(0)
    shoulder_scale.set(0)
    elbow_scale.set(0)
    wrist_scale.set(0)
    internal_update = False

    Dx, Dy, Dz = current_D()
    target["r"] = math.hypot(Dx, Dy)
    target["z"] = Dz
    draw()

Button(controls, text="Reset", command=reset_all).pack(pady=12)

fig.canvas.mpl_connect("button_press_event", on_press)
fig.canvas.mpl_connect("button_release_event", on_release)
fig.canvas.mpl_connect("motion_notify_event", on_motion)

Dx0, Dy0, Dz0 = current_D()
target["r"] = math.hypot(Dx0, Dy0)
target["z"] = Dz0

draw()
root.mainloop()