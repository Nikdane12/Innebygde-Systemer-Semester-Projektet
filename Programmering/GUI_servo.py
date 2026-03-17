from tkinter import *
import math
import numpy as np
from spatialmath import SE3
import roboticstoolbox as rtb

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import proj3d

# -------------------------
# Robot geometry
# -------------------------
BOX_H = 0.35
L1 = 1.00
L2 = 0.80
L3 = 0.60
REACH = L1 + L2 + L3

_lengths = {"L1": L1, "L2": L2, "L3": L3}

JOINT_OFFSET_DEG = -90  # slider 0° = physical -90°; slider ±45° = physical -135°…-45°

YAW_MIN, YAW_MAX = -90,  90    # base: 180° total arc  (-90 … +90)
SH_MIN,  SH_MAX  = -45,  45    # shoulder: 90° total arc (-45 … +45)
EL_MIN,  EL_MAX  = -45,  45    # elbow:    90° total arc (-45 … +45)
WR_MIN,  WR_MAX  = -45,  45    # wrist:    90° total arc (-45 … +45)

REACH_WARN_PCT = 93   # % of max reach that triggers warning
NUDGE_STEP     = 0.05 # metres per arrow-key press
DRAG_GAIN      = 0.85
VERT_GAIN      = 0.70
REDRAW_MS      = 30   # minimum ms between redraws (≈33 fps cap)

SEG_COLORS  = ["#2196F3", "#4CAF50", "#FF9800", "#E91E63", "#9C27B0"]
JOINT_LABELS = ["O", "A", "B", "C", "D", "E"]

# -------------------------
# Helpers
# -------------------------
def deg2rad(d): return math.radians(d)
def rad2deg(r): return math.degrees(r)
def clamp(v, lo, hi): return max(lo, min(hi, v))

def get_lengths():
    return _lengths["L1"], _lengths["L2"], _lengths["L3"]

def current_reach():
    l1, l2, l3 = get_lengths()
    return l1 + l2 + l3

def rebuild_robot():
    global robot
    l1, l2, l3 = get_lengths()
    robot = rtb.DHRobot([
        rtb.RevoluteDH(d=BOX_H, a=0,  alpha=np.pi/2, qlim=np.deg2rad([YAW_MIN, YAW_MAX])),
        rtb.RevoluteDH(d=0,     a=l1, alpha=0,        qlim=np.deg2rad([SH_MIN,  SH_MAX])),
        rtb.RevoluteDH(d=0,     a=l2, alpha=0,        qlim=np.deg2rad([EL_MIN,  EL_MAX])),
        rtb.RevoluteDH(d=0,     a=l3, alpha=np.pi/2,  qlim=np.deg2rad([WR_MIN,  WR_MAX])),
    ], name="ArmBot")

def wrap180(deg):
    while deg >=  180: deg -= 360
    while deg <  -180: deg += 360
    return deg

def point_from_T(T):
    return (float(T.t[0]), float(T.t[1]), float(T.t[2]))

# -------------------------
# Robot model (DH parameters)
# Positive shoulder/elbow/wrist = arm lifts upward.
# Angles are negated when passed to the DH model because RevoluteDH
# rotates around Z, and after alpha=pi/2 on joint 1 a positive DH
# angle lifts the arm downward in our coordinate system.
# -------------------------
robot = rtb.DHRobot([
    rtb.RevoluteDH(d=BOX_H, a=0,  alpha=np.pi/2, qlim=np.deg2rad([YAW_MIN, YAW_MAX])),
    rtb.RevoluteDH(d=0,     a=L1, alpha=0,        qlim=np.deg2rad([SH_MIN,  SH_MAX])),
    rtb.RevoluteDH(d=0,     a=L2, alpha=0,        qlim=np.deg2rad([EL_MIN,  EL_MAX])),
    rtb.RevoluteDH(d=0,     a=L3, alpha=np.pi/2,   qlim=np.deg2rad([WR_MIN,  WR_MAX])),
], name="ArmBot")

# -------------------------
# Forward kinematics
# -------------------------
def fk_chain(yaw_deg, shoulder_slider_deg, elbow_deg, wrist_deg):
    sh_off, el_off, wr_off = get_offsets()
    sh = shoulder_slider_deg + sh_off
    el = elbow_deg           + el_off
    wr = wrist_deg           + wr_off
    q  = np.deg2rad([yaw_deg, -sh, -el, -wr])
    T_all = robot.fkine_all(q)
    return SE3(), T_all[0], T_all[1], T_all[2], T_all[3], T_all[4]

# -------------------------
# Disable 3D mouse rotation/pan/zoom
# -------------------------
def disable_3d_mouse(ax, fig):
    for attr in ["_rotate_cid", "_zoom_cid",
                 "_button_press_cid", "_button_release_cid", "_motion_notify_cid"]:
        cid = getattr(ax, attr, None)
        if cid is not None:
            try: fig.canvas.mpl_disconnect(cid)
            except Exception: pass
    try: ax.mouse_init(rotate_btn=None, zoom_btn=None)
    except Exception: pass

# -------------------------
# Inverse kinematics
# -------------------------
def ik_solve(r_forward, z_world, prev_angles=None):
    r_forward = max(0.0, r_forward)
    yaw = yaw_scale.get()

    tx = r_forward * math.cos(deg2rad(yaw))
    ty = r_forward * math.sin(deg2rad(yaw))
    tz = clamp(z_world, 0.0, BOX_H + current_reach())

    T_target = SE3(tx, ty, tz)

    sh_off, el_off, wr_off = get_offsets()
    if prev_angles is not None:
        sh = prev_angles['shoulder'] + sh_off
        el = prev_angles['elbow']    + el_off
        wr = prev_angles['wrist']    + wr_off
        q0 = np.deg2rad([yaw, -sh, -el, -wr])
    else:
        q0 = np.deg2rad([0, sh_off, el_off, wr_off])

    sol = robot.ikine_LM(T_target, q0=q0, mask=[1, 1, 1, 0, 0, 0])

    if sol.success:
        _, q2, q3, q4 = sol.q
        sh_slider = clamp(-np.rad2deg(q2) - sh_off, SH_MIN, SH_MAX)
        el_deg    = clamp(-np.rad2deg(q3)  - el_off, EL_MIN, EL_MAX)
        wr_deg    = clamp(wrap180(-np.rad2deg(q4)) - wr_off, WR_MIN, WR_MAX)
        return sh_slider, el_deg, wr_deg

    if prev_angles is not None:
        return prev_angles['shoulder'], prev_angles['elbow'], prev_angles['wrist']
    return 0.0, 0.0, 0.0

# -------------------------
# Tkinter + Matplotlib
# -------------------------
root = Tk()
root.title("3D Arm Visualizer")
root.geometry("1020x680")

fig = Figure(figsize=(7.5, 5.6))
ax  = fig.add_subplot(111, projection="3d")

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=LEFT, fill=BOTH, expand=True)

controls = Frame(root)
controls.pack(side=RIGHT, fill=Y, padx=12, pady=12)

info = Label(controls, text="", justify=LEFT, font=("Courier", 9))
info.pack(pady=8)

yaw_scale = Scale(controls, from_=YAW_MIN, to=YAW_MAX, orient=HORIZONTAL,
                  label="Base yaw  (-90 … +90°)", length=260, resolution=0.5)
yaw_scale.set(0); yaw_scale.pack(fill="x")

shoulder_scale = Scale(controls, from_=SH_MIN, to=SH_MAX, orient=HORIZONTAL,
                       label="Shoulder  (+up / -down)", length=260, resolution=0.5)
shoulder_scale.set(0); shoulder_scale.pack(fill="x")

elbow_scale = Scale(controls, from_=EL_MIN, to=EL_MAX, orient=HORIZONTAL,
                    label="Elbow  (relative)", length=260, resolution=0.5)
elbow_scale.set(0); elbow_scale.pack(fill="x")

wrist_scale = Scale(controls, from_=WR_MIN, to=WR_MAX, orient=HORIZONTAL,
                    label="Wrist  (relative)", length=260, resolution=0.5)
wrist_scale.set(0); wrist_scale.pack(fill="x")

Frame(controls, height=2, bd=1, relief=SUNKEN).pack(fill="x", pady=6)

Label(controls, text="Joint zero offsets (°)", font=("TkDefaultFont", 9, "bold")).pack()

_offset_frame = Frame(controls)
_offset_frame.pack(fill="x", pady=2)

Label(_offset_frame, text="Shoulder:", width=9, anchor="w").grid(row=0, column=0, padx=4)
sh_offset = Spinbox(_offset_frame, from_=-180, to=180, width=6, increment=1)
sh_offset.delete(0, END); sh_offset.insert(0, str(JOINT_OFFSET_DEG))
sh_offset.grid(row=0, column=1, padx=4)

Label(_offset_frame, text="Elbow:", width=9, anchor="w").grid(row=1, column=0, padx=4)
el_offset = Spinbox(_offset_frame, from_=-180, to=180, width=6, increment=1)
el_offset.delete(0, END); el_offset.insert(0, str(JOINT_OFFSET_DEG))
el_offset.grid(row=1, column=1, padx=4)

Label(_offset_frame, text="Wrist:", width=9, anchor="w").grid(row=2, column=0, padx=4)
wr_offset = Spinbox(_offset_frame, from_=-180, to=180, width=6, increment=1)
wr_offset.delete(0, END); wr_offset.insert(0, str(JOINT_OFFSET_DEG))
wr_offset.grid(row=2, column=1, padx=4)

def get_offsets():
    try:    sh = float(sh_offset.get())
    except: sh = JOINT_OFFSET_DEG
    try:    el = float(el_offset.get())
    except: el = JOINT_OFFSET_DEG
    try:    wr = float(wr_offset.get())
    except: wr = JOINT_OFFSET_DEG
    return sh, el, wr

Frame(controls, height=2, bd=1, relief=SUNKEN).pack(fill="x", pady=6)

Label(controls, text="Segment lengths (m)", font=("TkDefaultFont", 9, "bold")).pack()

_len_frame = Frame(controls)
_len_frame.pack(fill="x", pady=2)

Label(_len_frame, text="Upper arm:", width=9, anchor="w").grid(row=0, column=0, padx=4)
l1_spin = Spinbox(_len_frame, from_=0.1, to=3.0, width=6, increment=0.05, format="%.2f")
l1_spin.delete(0, END); l1_spin.insert(0, f"{L1:.2f}")
l1_spin.grid(row=0, column=1, padx=4)

Label(_len_frame, text="Forearm:", width=9, anchor="w").grid(row=1, column=0, padx=4)
l2_spin = Spinbox(_len_frame, from_=0.1, to=3.0, width=6, increment=0.05, format="%.2f")
l2_spin.delete(0, END); l2_spin.insert(0, f"{L2:.2f}")
l2_spin.grid(row=1, column=1, padx=4)

Label(_len_frame, text="Wrist:", width=9, anchor="w").grid(row=2, column=0, padx=4)
l3_spin = Spinbox(_len_frame, from_=0.1, to=3.0, width=6, increment=0.05, format="%.2f")
l3_spin.delete(0, END); l3_spin.insert(0, f"{L3:.2f}")
l3_spin.grid(row=2, column=1, padx=4)

def on_length_change(_=None):
    try: _lengths["L1"] = max(0.1, float(l1_spin.get()))
    except: pass
    try: _lengths["L2"] = max(0.1, float(l2_spin.get()))
    except: pass
    try: _lengths["L3"] = max(0.1, float(l3_spin.get()))
    except: pass
    rebuild_robot()
    on_slider()

l1_spin.config(command=on_length_change)
l2_spin.config(command=on_length_change)
l3_spin.config(command=on_length_change)

disable_3d_mouse(ax, fig)

# -------------------------
# State
# -------------------------
dragging        = False
last_mouse      = None
internal_update = False
_redraw_pending = False

target = {"r": 0.0, "z": BOX_H + 0.5}

# -------------------------
# FK helpers
# -------------------------
def current_D():
    *_, T_D = fk_chain(yaw_scale.get(), shoulder_scale.get(),
                       elbow_scale.get(), wrist_scale.get())
    return point_from_T(T_D)

def get_D_screen_xy():
    *_, T_D = fk_chain(yaw_scale.get(), shoulder_scale.get(),
                       elbow_scale.get(), wrist_scale.get())
    Dx, Dy, Dz = point_from_T(T_D)
    x2, y2, _ = proj3d.proj_transform(Dx, Dy, Dz, ax.get_proj())
    px, py = ax.transData.transform((x2, y2))
    return px, py

# -------------------------
# Draw
# -------------------------
def draw():
    global _redraw_pending
    _redraw_pending = False

    ax.clear()

    yaw = yaw_scale.get()
    shs = shoulder_scale.get()
    el  = elbow_scale.get()
    wr  = wrist_scale.get()

    T_O, T_A, T_B, T_C, T_D, T_E = fk_chain(yaw, shs, el, wr)

    pts = [point_from_T(T) for T in (T_O, T_A, T_B, T_C, T_D, T_E)]
    xs  = [p[0] for p in pts]
    ys  = [p[1] for p in pts]
    zs  = [p[2] for p in pts]

    reach = current_reach()

    # Reach limit circle on ground
    theta = np.linspace(0, 2 * np.pi, 120)
    ax.plot(reach * np.cos(theta), reach * np.sin(theta),
            np.zeros(120), color="#AAAAAA", lw=0.8, linestyle=":", alpha=0.6,
            label=f"Max reach ({reach:.2f} m)")

    # Arm shadow on ground
    ax.plot(xs, ys, [0]*len(zs), color="gray", lw=1.5, alpha=0.3, linestyle="--")

    # Arm segments
    for i in range(len(xs) - 1):
        ax.plot([xs[i], xs[i+1]], [ys[i], ys[i+1]], [zs[i], zs[i+1]],
                color=SEG_COLORS[i], linewidth=3, solid_capstyle="round")

    # Joint markers + labels
    for i, (jx, jy, jz) in enumerate(zip(xs, ys, zs)):
        size  = 120 if i == len(xs) - 1 else 40
        color = "white" if i == len(xs) - 1 else "cyan"
        ax.scatter([jx], [jy], [jz], s=size, c=color, depthshade=False, zorder=10)
        ax.text(jx, jy, jz + 0.04, JOINT_LABELS[i], fontsize=8, color="dimgray")

    # IK target marker
    yaw_r = deg2rad(yaw)
    tx = target["r"] * math.cos(yaw_r)
    ty = target["r"] * math.sin(yaw_r)
    tz = target["z"]
    ax.scatter([tx], [ty], [tz], marker="x", s=120, c="red", zorder=8)

    # Line from end-effector to target (shows IK error)
    Dx, Dy, Dz = pts[-1]
    err = math.sqrt((Dx-tx)**2 + (Dy-ty)**2 + (Dz-tz)**2)
    if err > 0.005:
        ax.plot([Dx, tx], [Dy, ty], [Dz, tz],
                color="red", lw=1.0, linestyle="--", alpha=0.7)

    ax.set_xlim(-reach, reach)
    ax.set_ylim(-reach, reach)
    ax.set_zlim(0, BOX_H + reach)
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    ax.set_title("Robot Arm — drag end-effector or use sliders", fontsize=9)

    reach_pct = math.sqrt(Dx**2 + Dy**2 + Dz**2) / reach * 100
    warn = "  ⚠ near limit" if reach_pct > REACH_WARN_PCT else ""
    warn_color = "red" if reach_pct > REACH_WARN_PCT else "black"
    info.config(fg=warn_color, text=(
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
        f"  err = {err*100:.1f} cm\n"
        f"\n"
        f"Drag end-effector to move\n"
        f"R=reset  Q=quit  ←→↑↓=nudge"
    ))

    canvas.draw()

def request_draw():
    """Rate-limit redraws to REDRAW_MS to keep the UI responsive."""
    global _redraw_pending
    if not _redraw_pending:
        _redraw_pending = True
        root.after(REDRAW_MS, draw)

# -------------------------
# Slider callbacks
# -------------------------
def on_slider(_=None):
    global internal_update
    if internal_update:
        request_draw()
        return
    Dx, Dy, Dz = current_D()
    target["r"] = math.hypot(Dx, Dy)
    target["z"] = Dz
    request_draw()

yaw_scale.config(command=on_slider)
sh_offset.config(command=on_slider)
el_offset.config(command=on_slider)
wr_offset.config(command=on_slider)
shoulder_scale.config(command=on_slider)
elbow_scale.config(command=on_slider)
wrist_scale.config(command=on_slider)

# -------------------------
# Mouse drag
# -------------------------
def on_press(event):
    global dragging, last_mouse
    if event.inaxes != ax or event.x is None or event.y is None:
        return
    Dx, Dy = get_D_screen_xy()
    if math.hypot(event.x - Dx, event.y - Dy) < 22:
        dragging   = True
        last_mouse = (event.x, event.y)

def on_release(event):
    global dragging, last_mouse
    dragging   = False
    last_mouse = None

def arm_to_screen(r, z):
    """Project (r, z) in the current arm plane to figure pixel coordinates."""
    yaw_r = deg2rad(yaw_scale.get())
    wx = r * math.cos(yaw_r)
    wy = r * math.sin(yaw_r)
    x2, y2, _ = proj3d.proj_transform(wx, wy, z, ax.get_proj())
    sx, sy = ax.transData.transform((x2, y2))
    return sx, sy

def on_motion(event):
    global last_mouse, internal_update
    if not dragging or event.x is None or event.y is None:
        return
    if last_mouse is None:
        last_mouse = (event.x, event.y)
        return

    dmx = event.x - last_mouse[0]
    dmy = event.y - last_mouse[1]
    last_mouse = (event.x, event.y)

    # Local Jacobian: how much screen position changes per unit change in r / z.
    # Inverting it gives world deltas that match the mouse movement, automatically
    # accounting for the 3D view angle so the arm follows the cursor correctly.
    r0, z0 = target["r"], target["z"]
    eps = max(current_reach() * 0.05, 0.05)
    s0 = arm_to_screen(r0,       z0)
    sr = arm_to_screen(r0 + eps, z0)
    sz = arm_to_screen(r0,       z0 + eps)

    jrx = (sr[0] - s0[0]) / eps
    jry = (sr[1] - s0[1]) / eps
    jzx = (sz[0] - s0[0]) / eps
    jzy = (sz[1] - s0[1]) / eps

    det = jrx * jzy - jzx * jry
    if abs(det) > 1e-3:
        dr = ( jzy * dmx - jzx * dmy) / det
        dz = (-jry * dmx + jrx * dmy) / det
        target["r"] = max(0.0, r0 + dr)
        target["z"] = clamp(z0 + dz, 0.0, BOX_H + current_reach())

    prev = {"shoulder": shoulder_scale.get(),
            "elbow":    elbow_scale.get(),
            "wrist":    wrist_scale.get()}
    shs_d, el_d, wr_d = ik_solve(target["r"], target["z"], prev)

    internal_update = True
    shoulder_scale.set(shs_d)
    elbow_scale.set(el_d)
    wrist_scale.set(wr_d)
    internal_update = False

    # Snap target to actual EE — hard stops at joint limits.
    Dx, Dy, Dz = current_D()
    target["r"] = math.hypot(Dx, Dy)
    target["z"] = Dz

    request_draw()

# -------------------------
# Reset
# -------------------------
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

Button(controls, text="Reset  (R)", command=reset_all).pack(pady=12)

# -------------------------
# Keyboard shortcuts
# -------------------------
def on_key(event):
    key = event.keysym.lower()
    if key == "r":
        reset_all()
    elif key == "q":
        root.destroy()
    elif key in ("left", "right", "up", "down"):
            # Left/Right move radially (in/out), Up/Down move vertically
        if key == "right":
            target["r"] = max(0.0, target["r"] + NUDGE_STEP)
        elif key == "left":
            target["r"] = max(0.0, target["r"] - NUDGE_STEP)
        elif key == "up":
            target["z"] = clamp(target["z"] + NUDGE_STEP, 0.0, BOX_H + current_reach())
        elif key == "down":
            target["z"] = clamp(target["z"] - NUDGE_STEP, 0.0, BOX_H + current_reach())

        prev = {"shoulder": shoulder_scale.get(),
                "elbow":    elbow_scale.get(),
                "wrist":    wrist_scale.get()}
        shs_d, el_d, wr_d = ik_solve(target["r"], target["z"], prev)

        global internal_update
        internal_update = True
        shoulder_scale.set(shs_d)
        elbow_scale.set(el_d)
        wrist_scale.set(wr_d)
        internal_update = False
        # Snap target to actual EE (same hard-stop logic as drag)
        Dx, Dy, Dz = current_D()
        target["r"] = math.hypot(Dx, Dy)
        target["z"] = Dz
        draw()

root.bind("<KeyPress>", on_key)

# -------------------------
# Canvas events
# -------------------------
fig.canvas.mpl_connect("button_press_event",   on_press)
fig.canvas.mpl_connect("button_release_event", on_release)
fig.canvas.mpl_connect("motion_notify_event",  on_motion)

# -------------------------
# Initial draw
# -------------------------
Dx0, Dy0, Dz0 = current_D()
target["r"] = math.hypot(Dx0, Dy0)
target["z"] = Dz0

draw()
root.mainloop()
