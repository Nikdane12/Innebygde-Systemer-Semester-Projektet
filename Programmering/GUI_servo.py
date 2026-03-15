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

SHOULDER_ZERO_OFFSET_DEG = 0

YAW_MIN, YAW_MAX = -90, 90
SH_MIN, SH_MAX = -90, 90
EL_MIN, EL_MAX = -135, 135
WR_MIN, WR_MAX = -135, 135

# -------------------------
# Forward kinematics
# FIX: negate shoulder/elbow/wrist in T_roty so that positive angles
#      lift the arm upward — consistent with the IK plane convention
#      where positive angles correspond to upward Z motion.
# -------------------------
def fk_chain(yaw_deg, shoulder_slider_deg, elbow_deg, wrist_deg):
    yaw = deg2rad(yaw_deg)
    sh  = deg2rad(shoulder_slider_deg + SHOULDER_ZERO_OFFSET_DEG)
    el  = deg2rad(elbow_deg)
    wr  = deg2rad(wrist_deg)

    T_O = T_identity()
    T_yaw = T_rotz(yaw)
    T_OA = T_trans(0, 0, BOX_H)

    # KEY FIX: negate sh/el/wr so positive angle = arm lifts upward.
    # T_roty(rad) rotates X toward -Z; negating makes X rotate toward +Z.
    T_AB = T_mul(T_roty(-sh), T_trans(L1, 0, 0))
    T_BC = T_mul(T_roty(-el), T_trans(L2, 0, 0))
    T_CD = T_mul(T_roty(-wr), T_trans(L3, 0, 0))

    T_A = T_mul(T_mul(T_O, T_yaw), T_OA)
    T_B = T_mul(T_A, T_AB)
    T_C = T_mul(T_B, T_BC)
    T_D = T_mul(T_C, T_CD)
    return T_O, T_A, T_B, T_C, T_D

# -------------------------
# Disable 3D mouse rotation/pan/zoom
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
# 3-link IK in the arm plane
# FIX: added continuity penalty seeded from previous solution so the
#      arm doesn't flip between elbow branches during drag.
# -------------------------
def ik_3link_plane_best(r_forward, z_world, prev_angles=None):
    """
    Solve shoulder/elbow/wrist for end-effector at (r_forward, z_world).
    prev_angles: optional dict with keys 'shoulder','elbow','wrist' (degrees)
                 used to prefer solutions near the current pose (smooth dragging).
    Returns: shoulder_slider_deg, elbow_deg, wrist_deg
    """
    r_forward = max(0.0, r_forward)
    px = r_forward
    pz = z_world - BOX_H

    best = None  # (cost, sh_slider, el_deg, wr_deg)

    base_tool = math.atan2(pz, px if px > 1e-9 else 1e-9)

    # Dense local search around pointing-to-target direction
    tool_candidates = []
    for ddeg in range(-90, 91, 5):
        tool_candidates.append(base_tool + deg2rad(ddeg))
    # Wider global sweep to escape local minima
    for ddeg in range(-180, 181, 15):
        tool_candidates.append(deg2rad(ddeg))
    # Fine search seeded from previous solution for smooth dragging
    if prev_angles is not None:
        prev_tool = deg2rad(prev_angles['shoulder'] + prev_angles['elbow'] + prev_angles['wrist'])
        for ddeg in range(-20, 21, 2):
            tool_candidates.append(prev_tool + deg2rad(ddeg))

    for tool in tool_candidates:
        cx = px - L3 * math.cos(tool)
        cz = pz - L3 * math.sin(tool)

        a, b = L1, L2
        d2 = cx*cx + cz*cz
        d = math.sqrt(d2)

        if d > a + b or d < abs(a - b):
            continue

        cos_el = (d2 - a*a - b*b) / (2*a*b)
        cos_el = clamp(cos_el, -1.0, 1.0)
        el0 = math.acos(cos_el)

        for elbow_rel in (el0, -el0):
            sh = math.atan2(cz, cx) - math.atan2(b*math.sin(elbow_rel), a + b*math.cos(elbow_rel))
            wr = tool - sh - elbow_rel

            sh_deg_actual = rad2deg(sh)
            el_deg = rad2deg(elbow_rel)
            wr_deg = wrap180(rad2deg(wr))

            sh_slider = sh_deg_actual - SHOULDER_ZERO_OFFSET_DEG

            if not (SH_MIN <= sh_slider <= SH_MAX):
                continue
            if not (EL_MIN <= el_deg <= EL_MAX):
                continue
            if not (WR_MIN <= wr_deg <= WR_MAX):
                continue

            # FK error in plane
            th1 = sh
            th2 = sh + elbow_rel
            th3 = sh + elbow_rel + deg2rad(wr_deg)
            x_fk = L1*math.cos(th1) + L2*math.cos(th2) + L3*math.cos(th3)
            z_fk = L1*math.sin(th1) + L2*math.sin(th2) + L3*math.sin(th3)
            err = math.hypot(x_fk - px, z_fk - pz)

            # Joint-effort penalty (prefer less extreme angles)
            penalty = 0.001*(abs(sh_slider) + abs(el_deg) + abs(wr_deg))

            # Continuity penalty: prefer solutions near previous pose
            cont = 0.0
            if prev_angles is not None:
                cont = 0.005 * (abs(sh_slider - prev_angles['shoulder']) +
                                abs(el_deg   - prev_angles['elbow']) +
                                abs(wr_deg   - prev_angles['wrist']))

            cost = err + penalty + cont

            if best is None or cost < best[0]:
                best = (cost, sh_slider, el_deg, wr_deg)

    # Fallback: clamp a direct solution when nothing satisfies limits
    if best is None:
        tool = base_tool
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

info = Label(controls, text="", justify=LEFT, font=("Courier", 9))
info.pack(pady=8)

yaw_scale = Scale(controls, from_=YAW_MIN, to=YAW_MAX, orient=HORIZONTAL,
                  label="Base yaw (-90..+90)", length=250, resolution=0.5)
yaw_scale.set(0)
yaw_scale.pack(fill="x")

shoulder_scale = Scale(controls, from_=SH_MIN, to=SH_MAX, orient=HORIZONTAL,
                       label="Shoulder pitch (+up / -down)", length=250, resolution=0.5)
shoulder_scale.set(0)
shoulder_scale.pack(fill="x")

elbow_scale = Scale(controls, from_=EL_MIN, to=EL_MAX, orient=HORIZONTAL,
                    label="Elbow pitch (relative)", length=250, resolution=0.5)
elbow_scale.set(0)
elbow_scale.pack(fill="x")

wrist_scale = Scale(controls, from_=WR_MIN, to=WR_MAX, orient=HORIZONTAL,
                    label="Wrist pitch (relative)", length=250, resolution=0.5)
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

    # Arm shadow on ground
    ax.plot(xs, ys, [0]*len(zs), color="gray", lw=1.5, alpha=0.3, linestyle="--")

    # Arm segments with per-link colours
    seg_colors = ["#2196F3", "#4CAF50", "#FF9800", "#E91E63"]
    for i in range(len(xs)-1):
        ax.plot([xs[i], xs[i+1]], [ys[i], ys[i+1]], [zs[i], zs[i+1]],
                color=seg_colors[i], linewidth=3, solid_capstyle="round")

    # Joint markers
    labels = ["O", "A", "B", "C", "D"]
    for i, (jx, jy, jz) in enumerate(zip(xs, ys, zs)):
        size = 120 if i == len(xs)-1 else 40
        color = "white" if i == len(xs)-1 else "cyan"
        ax.scatter([jx], [jy], [jz], s=size, c=color, depthshade=False, zorder=10)
        ax.text(jx, jy, jz + 0.04, labels[i], fontsize=8, color="lightgray")

    # Target marker
    yaw_r = deg2rad(yaw)
    tx = target["r"] * math.cos(yaw_r)
    ty = target["r"] * math.sin(yaw_r)
    tz = target["z"]
    ax.scatter([tx], [ty], [tz], marker="x", s=100, c="red", zorder=8)
    span = 0.1
    for dx_, dy_, dz_ in [(span,0,0),(-span,0,0),(0,span,0),(0,-span,0),(0,0,span),(0,0,-span)]:
        ax.plot([tx, tx+dx_],[ty, ty+dy_],[tz, tz+dz_], color="red", lw=1.2, alpha=0.7)

    reach = L1 + L2 + L3
    ax.set_xlim(-reach, reach)              
    ax.set_ylim(-reach, reach)
    ax.set_zlim(0, BOX_H + reach)
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")

    # End-effector world position
    Dx, Dy, Dz = D
    reach_pct = math.sqrt(Dx**2 + Dy**2 + Dz**2) / reach * 100
    warn = "  ⚠ near limit" if reach_pct > 93 else ""

    shoulder_actual = shs + SHOULDER_ZERO_OFFSET_DEG
    info.config(text=(
        f"Yaw:       {yaw:+.1f}°\n"
        f"Shoulder:  {shs:+.1f}°\n"
        f"Elbow:     {el:+.1f}°\n"
        f"Wrist:     {wr:+.1f}°\n"
        f"\n"
        f"End-effector (world):\n"
        f"  X = {Dx:+.3f} m\n"
        f"  Y = {Dy:+.3f} m\n"
        f"  Z = {Dz:+.3f} m\n"
        f"  Reach: {reach_pct:.1f}%{warn}\n"
        f"\n"
        f"IK target:\n"
        f"  r = {target['r']:.3f} m\n"
        f"  z = {target['z']:.3f} m\n"
        f"\n"
        f"Drag D to move arm\n"
        f"(+shoulder = up)"
    ))

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

    # Scale gain from actual canvas pixel width so drag feels consistent
    canvas_px_w = canvas.get_tk_widget().winfo_width()
    gain = (2 * (L1 + L2 + L3)) / max(canvas_px_w, 100) * 0.85

    target["r"] = max(0.0, target["r"] + dx * gain)
    target["z"] = clamp(target["z"] - dy * gain * 0.7,
                        0.0, BOX_H + (L1 + L2 + L3))

    # Pass current angles as hint for smooth/continuous IK
    prev = {
        "shoulder": shoulder_scale.get(),
        "elbow":    elbow_scale.get(),
        "wrist":    wrist_scale.get(),
    }
    shs_d, el_d, wr_d = ik_3link_plane_best(target["r"], target["z"], prev)

    internal_update = True
    shoulder_scale.set(shs_d)
    elbow_scale.set(el_d)
    wrist_scale.set(wr_d)
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