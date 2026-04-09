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

# ─────────────────────────────────────────
# ① N-JOINT CONFIG  ← change this number
# ─────────────────────────────────────────
N_JOINTS = 4          # total revolute joints (1 = base yaw + N-1 pitch joints)

# ─────────────────────────────────────────
# Robot geometry defaults
# ─────────────────────────────────────────
BOX_H        = 0.35
DEFAULT_LINK = 0.80   # metres, applied to every link on first run

JOINT_OFFSET_DEG = -90

# Joint limits (applied identically to all pitch joints)
YAW_MIN, YAW_MAX = -90,  90
PIT_MIN, PIT_MAX = -45,  45    # shoulder / elbow / wrist / …

REACH_WARN_PCT = 93
NUDGE_STEP     = 0.05
REDRAW_MS      = 30
ANIM_STEPS     = 40
ANIM_MS        = 20

# Jacobian DLS params
DLS_DAMPING     = 0.06   # λ in (JJᵀ + λ²I)⁻¹  — raise to trade accuracy for smoothness
DLS_STEPS       = 6      # Jacobian iterations per drag event
DLS_STEP_SCALE  = 0.55   # step-size limiter per iteration (prevents overshoot)

SEG_COLORS   = ["#2196F3", "#4CAF50", "#FF9800", "#E91E63",
                "#9C27B0", "#00BCD4", "#FF5722", "#8BC34A",
                "#FFC107", "#607D8B"]
JOINT_LABELS = list("OABCDEFGHIJ")

# ─────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────
def deg2rad(d): return math.radians(d)
def rad2deg(r): return math.degrees(r)
def clamp(v, lo, hi): return max(lo, min(hi, v))

def wrap180(deg):
    while deg >=  180: deg -= 360
    while deg <  -180: deg += 360
    return deg

def point_from_T(T):
    return (float(T.t[0]), float(T.t[1]), float(T.t[2]))

# ─────────────────────────────────────────
# Dynamic link-length store
# ─────────────────────────────────────────
_link_lengths = [DEFAULT_LINK] * N_JOINTS   # one entry per joint

def get_link_lengths():
    return list(_link_lengths)

def current_reach():
    return sum(_link_lengths)

# ─────────────────────────────────────────
# Build / rebuild the DH robot
# ─────────────────────────────────────────
robot = None   # initialised by rebuild_robot()

def _make_dh_links(lengths):
    """Return a DH link list for N_JOINTS joints.

    Joint 0  : base yaw  (α = π/2 to lift the arm plane out of XY)
    Joints 1…N-2 : pitch joints in the arm plane (α = 0)
    Joint N-1 : final pitch / wrist (α = π/2 for tool orientation)
    """
    n = len(lengths)
    links = []
    for i, l in enumerate(lengths):
        if i == 0:
            # Base yaw — no link length on this joint; length contributed by link 1
            links.append(rtb.RevoluteDH(
                d=BOX_H, a=0, alpha=np.pi/2,
                qlim=np.deg2rad([YAW_MIN, YAW_MAX])))
        elif i == n - 1:
            # Last joint — α=π/2 so end-effector frame points forward
            links.append(rtb.RevoluteDH(
                d=0, a=l, alpha=np.pi/2,
                qlim=np.deg2rad([PIT_MIN, PIT_MAX])))
        else:
            links.append(rtb.RevoluteDH(
                d=0, a=l, alpha=0,
                qlim=np.deg2rad([PIT_MIN, PIT_MAX])))
    return links

def rebuild_robot():
    global robot
    robot = rtb.DHRobot(_make_dh_links(get_link_lengths()), name="ArmBot")

rebuild_robot()

# ─────────────────────────────────────────
# Offsets helpers
# ─────────────────────────────────────────
def get_offsets():
    """Return (offset_deg,) for each joint.  Joint 0 (yaw) has no offset."""
    offsets = []
    for sp in offset_spinboxes:
        try:    offsets.append(float(sp.get()))
        except: offsets.append(JOINT_OFFSET_DEG)
    return offsets   # length = N_JOINTS - 1  (pitch joints only)

def _q_from_sliders():
    """Return the full DH q-vector from current slider values + offsets."""
    yaw   = joint_scales[0].get()
    offs  = get_offsets()
    pitches = [joint_scales[i+1].get() + offs[i] for i in range(N_JOINTS - 1)]
    q = np.deg2rad([yaw] + [-p for p in pitches])
    return q

# ─────────────────────────────────────────
# Forward kinematics
# ─────────────────────────────────────────
def fk_chain():
    """Return list of SE3 transforms: [T_base, T_j0, T_j1, …, T_EE]"""
    q = _q_from_sliders()
    T_all = robot.fkine_all(q)
    return [SE3()] + list(T_all)

def current_EE():
    q = _q_from_sliders()
    T = robot.fkine(q)
    return point_from_T(T)

def get_EE_screen_xy():
    Ex, Ey, Ez = current_EE()
    x2, y2, _ = proj3d.proj_transform(Ex, Ey, Ez, ax.get_proj())
    px, py = ax.transData.transform((x2, y2))
    return px, py

# ─────────────────────────────────────────
# ② JACOBIAN DLS IK (drag / nudge)
# ─────────────────────────────────────────
def jacobian_dls_ik(tx, ty, tz, q0, steps=DLS_STEPS):
    """Fast Jacobian damped-least-squares IK for position only.

    Uses the 3-row position Jacobian (rows 0-2 of jacob0).
    Returns q (rad) clamped to joint limits.
    """
    q = q0.copy()
    t_target = np.array([tx, ty, tz])
    lims = np.array([[lnk.qlim[0], lnk.qlim[1]] for lnk in robot.links])

    for _ in range(steps):
        T_cur = robot.fkine(q)
        err   = t_target - T_cur.t          # 3-vector position error
        if np.linalg.norm(err) < 1e-4:
            break
        J3 = robot.jacob0(q)[:3, :]         # 3 × N position Jacobian
        lam2 = DLS_DAMPING ** 2
        dq   = J3.T @ np.linalg.solve(J3 @ J3.T + lam2 * np.eye(3), err)
        # scale step so max joint change ≤ DLS_STEP_SCALE rad
        max_dq = np.max(np.abs(dq))
        if max_dq > DLS_STEP_SCALE:
            dq *= DLS_STEP_SCALE / max_dq
        q = q + dq
        # clamp to joint limits
        q = np.clip(q, lims[:, 0], lims[:, 1])
    return q

def apply_q_to_sliders(q):
    """Push a raw DH q-vector back into the UI sliders."""
    global internal_update
    offs = get_offsets()
    internal_update = True
    joint_scales[0].set(round(np.rad2deg(q[0]), 2))          # yaw
    for i in range(N_JOINTS - 1):
        slider_val = clamp(-np.rad2deg(q[i+1]) - offs[i], PIT_MIN, PIT_MAX)
        joint_scales[i+1].set(round(slider_val, 2))
    internal_update = False

# ─────────────────────────────────────────
# IK solve for Go-to-POI (accurate, multi-seed)
# ─────────────────────────────────────────
def _geometric_seed(r_forward, z_world, yaw_deg):
    offs  = get_offsets()
    l0    = _link_lengths[0] if N_JOINTS > 1 else 0.5
    dz    = z_world - BOX_H
    sh_seed = clamp(math.degrees(math.atan2(r_forward, dz)), PIT_MIN, PIT_MAX)
    r_B   = l0 * math.sin(math.radians(sh_seed))
    z_B   = l0 * math.cos(math.radians(sh_seed))
    dr    = r_forward - r_B
    dz2   = dz - z_B
    el_seed = clamp(math.degrees(math.atan2(dr, dz2)), PIT_MIN, PIT_MAX)

    neutral = [-(offs[i] if i < len(offs) else 0) for i in range(N_JOINTS - 1)]
    neutral[0] = -(sh_seed + offs[0]) if offs else -sh_seed
    if len(neutral) > 1:
        neutral[1] = -(el_seed + (offs[1] if len(offs) > 1 else 0))
    return np.deg2rad([yaw_deg] + neutral)

def _best_sol(candidates, q0_ref):
    best, best_dist = None, float("inf")
    for sol in candidates:
        if sol.success:
            d = np.linalg.norm(sol.q - q0_ref)
            if d < best_dist:
                best, best_dist = sol, d
    return best

def ik_solve_accurate(r_forward, z_world):
    """Multi-seed Levenberg-Marquardt IK — used for Go-to-POI animation."""
    r_forward = max(0.0, r_forward)
    yaw = joint_scales[0].get()
    tx = r_forward * math.cos(deg2rad(yaw))
    ty = r_forward * math.sin(deg2rad(yaw))
    tz = clamp(z_world, 0.0, BOX_H + current_reach())
    T_target = SE3(tx, ty, tz)

    q0_cur = _q_from_sliders()
    q0_geo = _geometric_seed(r_forward, z_world, yaw)
    offs   = get_offsets()
    q0_neu = np.deg2rad([yaw] + [-(offs[i] if i < len(offs) else 0)
                                  for i in range(N_JOINTS - 1)])

    sols = [robot.ikine_LM(T_target, q0=q, mask=[1, 1, 1, 0, 0, 0])
            for q in (q0_cur, q0_geo, q0_neu)]
    sol = _best_sol(sols, q0_cur)
    if sol is not None:
        return sol.q
    return q0_cur   # fallback: stay put

# ─────────────────────────────────────────
# Disable 3D mouse rotation/pan/zoom
# ─────────────────────────────────────────
def disable_3d_mouse(ax, fig):
    for attr in ["_rotate_cid", "_zoom_cid",
                 "_button_press_cid", "_button_release_cid",
                 "_motion_notify_cid"]:
        cid = getattr(ax, attr, None)
        if cid is not None:
            try: fig.canvas.mpl_disconnect(cid)
            except Exception: pass
    try: ax.mouse_init(rotate_btn=None, zoom_btn=None)
    except Exception: pass

# ─────────────────────────────────────────
# ③ CLICK-ANYWHERE: ray → ground-plane
# ─────────────────────────────────────────
def screen_to_world_ground(event_x, event_y, z_plane):
    """
    Map a matplotlib figure-pixel coordinate onto a horizontal plane at z_plane
    using the current 3D projection matrix.

    Returns (wx, wy) world coordinates, or None if the ray is parallel to the plane.
    """
    # Two points along the unprojected ray at different depths
    proj = ax.get_proj()
    reach = current_reach()
    inv = np.linalg.inv(proj)

    # Convert figure pixels → axes display coords → data coords
    # We sample the ray at two w-values to get direction
    try:
        disp = ax.transData.inverted().transform((event_x, event_y))
    except Exception:
        return None

    # Use proj3d.inv_transform (available in matplotlib ≥ 3.3)
    try:
        wx0, wy0, wz0 = proj3d.inv_transform(disp[0], disp[1], 0.0, proj)
        wx1, wy1, wz1 = proj3d.inv_transform(disp[0], disp[1], 1.0, proj)
    except Exception:
        return None

    # Ray: P(t) = (wx0,wy0,wz0) + t*(wx1-wx0, wy1-wy0, wz1-wz0)
    dz = wz1 - wz0
    if abs(dz) < 1e-8:
        return None   # ray parallel to plane

    t = (z_plane - wz0) / dz
    wx = wx0 + t * (wx1 - wx0)
    wy = wy0 + t * (wy1 - wy0)

    # Sanity-clamp to workspace
    r_lim = reach * 1.05
    wx = clamp(wx, -r_lim, r_lim)
    wy = clamp(wy, -r_lim, r_lim)
    return wx, wy

# ─────────────────────────────────────────
# Tkinter + Matplotlib layout
# ─────────────────────────────────────────
root = Tk()
root.title("3D N-Joint Arm Visualizer")
root.geometry("1280x920")

left_frame = Frame(root)
left_frame.pack(side=LEFT, fill=BOTH, expand=True)

fig = Figure(figsize=(7.5, 4.2))
ax  = fig.add_subplot(111, projection="3d")

canvas = FigureCanvasTkAgg(fig, master=left_frame)
canvas.get_tk_widget().pack(side=TOP, fill=BOTH, expand=True)

bottom_frame = Frame(left_frame)
bottom_frame.pack(side=TOP, fill=BOTH, expand=True)

fig2 = Figure(figsize=(3.75, 3.0))
ax2  = fig2.add_subplot(111)
ax2.set_aspect("equal")

canvas2 = FigureCanvasTkAgg(fig2, master=bottom_frame)
canvas2.get_tk_widget().pack(side=LEFT, fill=BOTH, expand=True)

fig3 = Figure(figsize=(3.75, 3.0))
ax3  = fig3.add_subplot(111)
ax3.set_aspect("equal")

canvas3 = FigureCanvasTkAgg(fig3, master=bottom_frame)
canvas3.get_tk_widget().pack(side=LEFT, fill=BOTH, expand=True)

# Right control panel (scrollable)
ctrl_outer = Frame(root)
ctrl_outer.pack(side=RIGHT, fill=Y, padx=8, pady=8)

ctrl_canvas = Canvas(ctrl_outer, width=290)
ctrl_scroll  = Scrollbar(ctrl_outer, orient=VERTICAL, command=ctrl_canvas.yview)
ctrl_canvas.configure(yscrollcommand=ctrl_scroll.set)
ctrl_scroll.pack(side=RIGHT, fill=Y)
ctrl_canvas.pack(side=LEFT, fill=BOTH, expand=True)

controls = Frame(ctrl_canvas)
ctrl_canvas.create_window((0, 0), window=controls, anchor="nw")
controls.bind("<Configure>",
    lambda e: ctrl_canvas.configure(scrollregion=ctrl_canvas.bbox("all")))

info = Label(controls, text="", justify=LEFT, font=("Courier", 9))
info.pack(pady=6)

# ─────────────────────────────────────────
# Dynamic joint sliders  (N_JOINTS total)
# ─────────────────────────────────────────
Label(controls, text="Joint angles", font=("TkDefaultFont", 9, "bold")).pack()

joint_scales = []

def _make_joint_slider(idx):
    if idx == 0:
        lbl  = f"J0 – Base yaw  ({YAW_MIN}…{YAW_MAX}°)"
        lo, hi = YAW_MIN, YAW_MAX
    else:
        lbl  = f"J{idx} – Pitch  ({PIT_MIN}…{PIT_MAX}°)"
        lo, hi = PIT_MIN, PIT_MAX
    sc = Scale(controls, from_=lo, to=hi, orient=HORIZONTAL,
               label=lbl, length=260, resolution=0.5)
    sc.set(0)
    sc.pack(fill="x")
    return sc

for i in range(N_JOINTS):
    joint_scales.append(_make_joint_slider(i))

# ─────────────────────────────────────────
# Joint zero offsets
# ─────────────────────────────────────────
Frame(controls, height=2, bd=1, relief=SUNKEN).pack(fill="x", pady=5)
Label(controls, text="Joint zero offsets (°)", font=("TkDefaultFont", 9, "bold")).pack()

_off_frame = Frame(controls)
_off_frame.pack(fill="x", pady=2)

offset_spinboxes = []
for i in range(N_JOINTS - 1):
    Label(_off_frame, text=f"J{i+1}:", width=4, anchor="w").grid(
        row=i, column=0, padx=4)
    sp = Spinbox(_off_frame, from_=-180, to=180, width=6, increment=1)
    sp.delete(0, END); sp.insert(0, str(JOINT_OFFSET_DEG))
    sp.grid(row=i, column=1, padx=4)
    offset_spinboxes.append(sp)

# ─────────────────────────────────────────
# Segment lengths
# ─────────────────────────────────────────
Frame(controls, height=2, bd=1, relief=SUNKEN).pack(fill="x", pady=5)
Label(controls, text="Segment lengths (m)", font=("TkDefaultFont", 9, "bold")).pack()

_len_frame = Frame(controls)
_len_frame.pack(fill="x", pady=2)

len_spinboxes = []
seg_names = ["Upper arm", "Forearm", "Wrist"] + \
            [f"Link {i+4}" for i in range(N_JOINTS - 3)]

for i in range(N_JOINTS - 1):          # N-1 links (joint 0 is yaw, no length)
    name = seg_names[i] if i < len(seg_names) else f"Link {i+1}"
    Label(_len_frame, text=f"{name}:", width=9, anchor="w").grid(
        row=i, column=0, padx=4)
    sp = Spinbox(_len_frame, from_=0.05, to=3.0, width=6,
                 increment=0.05, format="%.2f")
    sp.delete(0, END); sp.insert(0, f"{DEFAULT_LINK:.2f}")
    sp.grid(row=i, column=1, padx=4)
    len_spinboxes.append(sp)

def on_length_change(_=None):
    for i, sp in enumerate(len_spinboxes):
        try:    _link_lengths[i] = max(0.05, float(sp.get()))
        except: pass
    rebuild_robot()
    on_slider()

for sp in len_spinboxes:
    sp.config(command=on_length_change)

# ─────────────────────────────────────────
# Point of Interest
# ─────────────────────────────────────────
Frame(controls, height=2, bd=1, relief=SUNKEN).pack(fill="x", pady=5)
Label(controls, text="Point of Interest (m)", font=("TkDefaultFont", 9, "bold")).pack()

_poi_frame = Frame(controls)
_poi_frame.pack(fill="x", pady=2)

poi = {"x": DEFAULT_LINK, "y": 0.0, "z": BOX_H + 0.5}

Label(_poi_frame, text="X:", width=3, anchor="w").grid(row=0, column=0, padx=4)
poi_x_entry = Entry(_poi_frame, width=8)
poi_x_entry.insert(0, f"{poi['x']:.3f}")
poi_x_entry.grid(row=0, column=1, padx=4)

Label(_poi_frame, text="Y:", width=3, anchor="w").grid(row=1, column=0, padx=4)
poi_y_entry = Entry(_poi_frame, width=8)
poi_y_entry.insert(0, f"{poi['y']:.3f}")
poi_y_entry.grid(row=1, column=1, padx=4)

Label(_poi_frame, text="Z:", width=3, anchor="w").grid(row=2, column=0, padx=4)
poi_z_entry = Entry(_poi_frame, width=8)
poi_z_entry.insert(0, f"{poi['z']:.3f}")
poi_z_entry.grid(row=2, column=1, padx=4)

# ─────────────────────────────────────────
# IK mode selector
# ─────────────────────────────────────────
Frame(controls, height=2, bd=1, relief=SUNKEN).pack(fill="x", pady=5)
Label(controls, text="Drag IK mode", font=("TkDefaultFont", 9, "bold")).pack()
ik_mode = StringVar(value="jacobian")
Radiobutton(controls, text="Jacobian DLS  (fast, smooth)",
            variable=ik_mode, value="jacobian").pack(anchor="w")
Radiobutton(controls, text="Levenberg-Marquardt  (accurate)",
            variable=ik_mode, value="lm").pack(anchor="w")

Frame(controls, height=2, bd=1, relief=SUNKEN).pack(fill="x", pady=5)
Button(controls, text="Reset  (R)", command=lambda: reset_all()).pack(pady=6)
Button(controls, text="Go to POI", command=lambda: go_to_poi()).pack(pady=4)

disable_3d_mouse(ax, fig)

# ─────────────────────────────────────────
# State
# ─────────────────────────────────────────
dragging        = False
last_mouse      = None
internal_update = False
_redraw_pending = False
_anim_steps     = []
_anim_idx       = 0
_poi_dragging   = False
_side_dragging  = False

target = {"r": DEFAULT_LINK, "z": BOX_H + 0.5}

# ─────────────────────────────────────────
# Draw
# ─────────────────────────────────────────
def draw():
    global _redraw_pending
    _redraw_pending = False
    ax.clear()

    Ts   = fk_chain()
    pts  = [point_from_T(T) for T in Ts]
    xs   = [p[0] for p in pts]
    ys   = [p[1] for p in pts]
    zs   = [p[2] for p in pts]

    reach = current_reach()

    # Reach circle
    theta = np.linspace(0, 2 * np.pi, 120)
    ax.plot(reach * np.cos(theta), reach * np.sin(theta), np.zeros(120),
            color="#AAAAAA", lw=0.8, linestyle=":", alpha=0.6,
            label=f"Max reach ({reach:.2f} m)")

    # Shadow
    ax.plot(xs, ys, [0]*len(zs), color="gray", lw=1.5, alpha=0.3, linestyle="--")

    # Segments
    n_segs = len(xs) - 1
    for i in range(n_segs):
        col = SEG_COLORS[i % len(SEG_COLORS)]
        ax.plot([xs[i], xs[i+1]], [ys[i], ys[i+1]], [zs[i], zs[i+1]],
                color=col, linewidth=3, solid_capstyle="round")

    # Joints
    for i, (jx, jy, jz) in enumerate(zip(xs, ys, zs)):
        size  = 120 if i == len(xs) - 1 else 40
        color = "white" if i == len(xs) - 1 else "cyan"
        ax.scatter([jx], [jy], [jz], s=size, c=color, depthshade=False, zorder=10)
        lbl = JOINT_LABELS[i] if i < len(JOINT_LABELS) else str(i)
        ax.text(jx, jy, jz + 0.04, lbl, fontsize=8, color="dimgray")

    # IK target
    yaw = joint_scales[0].get()
    tx  = target["r"] * math.cos(deg2rad(yaw))
    ty  = target["r"] * math.sin(deg2rad(yaw))
    tz  = target["z"]
    ax.scatter([tx], [ty], [tz], marker="x", s=140, c="red", zorder=8)

    # POI
    ax.scatter([poi["x"]], [poi["y"]], [poi["z"]],
               marker="*", s=220, c="lime", zorder=9, label="POI")
    ax.plot([poi["x"], poi["x"]], [poi["y"], poi["y"]], [0, poi["z"]],
            color="lime", lw=0.8, linestyle=":", alpha=0.5)

    # IK error line
    Ex, Ey, Ez = pts[-1]
    err = math.sqrt((Ex-tx)**2 + (Ey-ty)**2 + (Ez-tz)**2)
    if err > 0.005:
        ax.plot([Ex, tx], [Ey, ty], [Ez, tz],
                color="red", lw=1.0, linestyle="--", alpha=0.7)

    ax.set_xlim(-reach, reach)
    ax.set_ylim(-reach, reach)
    ax.set_zlim(0, BOX_H + reach)
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    ik_label = "Jacobian DLS" if ik_mode.get() == "jacobian" else "LM"
    ax.set_title(f"N={N_JOINTS}-joint Arm — drag EE, click bg, or sliders  [{ik_label}]",
                 fontsize=9)

    reach_pct   = math.sqrt(Ex**2 + Ey**2 + Ez**2) / reach * 100
    warn        = "  ⚠ near limit" if reach_pct > REACH_WARN_PCT else ""
    warn_color  = "red" if reach_pct > REACH_WARN_PCT else "black"

    angle_lines = "\n".join(
        [f"  J{i}: {joint_scales[i].get():+.1f}°" for i in range(N_JOINTS)]
    )
    info.config(fg=warn_color, text=(
        f"Joints:\n{angle_lines}\n\n"
        f"End-effector:\n"
        f"  X={Ex:+.3f}  Y={Ey:+.3f}  Z={Ez:+.3f} m\n"
        f"  Reach: {reach_pct:.1f}%{warn}\n\n"
        f"IK target:\n"
        f"  r={target['r']:.3f}  z={target['z']:.3f} m\n"
        f"  err={err*100:.1f} cm\n\n"
        f"Keys: R=reset  Q=quit  ←→↑↓=nudge\n"
        f"3D: drag EE(○) or click bg to aim"
    ))

    canvas.draw()
    draw_2d()
    draw_side()

def draw_2d():
    ax2.clear()
    reach = current_reach()

    theta = np.linspace(0, 2 * np.pi, 120)
    ax2.plot(reach * np.cos(theta), reach * np.sin(theta),
             color="#AAAAAA", lw=0.8, linestyle=":", alpha=0.6)

    Ts   = fk_chain()
    pts  = [point_from_T(T) for T in Ts]
    xs2  = [p[0] for p in pts]
    ys2  = [p[1] for p in pts]

    n_segs = len(xs2) - 1
    for i in range(n_segs):
        col = SEG_COLORS[i % len(SEG_COLORS)]
        ax2.plot([xs2[i], xs2[i+1]], [ys2[i], ys2[i+1]], color=col, lw=2.5,
                 solid_capstyle="round")
    for jx, jy in zip(xs2, ys2):
        ax2.scatter([jx], [jy], s=30, c="cyan", zorder=10)

    ax2.scatter([poi["x"]], [poi["y"]], marker="*", s=220, c="lime", zorder=9)

    ax2.set_xlim(-reach, reach); ax2.set_ylim(-reach, reach)
    ax2.set_aspect("equal")
    ax2.set_xlabel("X (m)"); ax2.set_ylabel("Y (m)")
    ax2.set_title("Top view (XY) — drag ★ to move POI", fontsize=9)
    ax2.grid(True, alpha=0.3)
    canvas2.draw()

def draw_side():
    ax3.clear()
    reach = current_reach()

    Ts  = fk_chain()
    pts = [point_from_T(T) for T in Ts]
    # r = horizontal reach at each joint (signed: negative behind base)
    rs  = [math.hypot(p[0], p[1]) for p in pts]
    zs  = [p[2] for p in pts]

    # Max-reach arc (quarter circle in r-z plane)
    arc_angles = np.linspace(0, np.pi / 2, 80)
    ax3.plot(reach * np.cos(arc_angles), BOX_H + reach * np.sin(arc_angles),
             color="#AAAAAA", lw=0.8, linestyle=":", alpha=0.6,
             label=f"Max reach ({reach:.2f} m)")

    # Ground line
    ax3.axhline(0, color="brown", lw=0.8, alpha=0.4)
    # Base box height
    ax3.axhline(BOX_H, color="gray", lw=0.6, linestyle="--", alpha=0.3)

    # Arm segments
    n_segs = len(rs) - 1
    for i in range(n_segs):
        col = SEG_COLORS[i % len(SEG_COLORS)]
        ax3.plot([rs[i], rs[i+1]], [zs[i], zs[i+1]], color=col, lw=2.5,
                 solid_capstyle="round")
    for idx, (jr, jz) in enumerate(zip(rs, zs)):
        is_ee = (idx == len(rs) - 1)
        ax3.scatter([jr], [jz], s=120 if is_ee else 30,
                    c="white" if is_ee else "cyan", zorder=10)

    # IK target
    ax3.scatter([target["r"]], [target["z"]], marker="x", s=140, c="red",
                zorder=8, label="IK target")

    # POI projected onto arm plane
    poi_r = math.hypot(poi["x"], poi["y"])
    ax3.scatter([poi_r], [poi["z"]], marker="*", s=220, c="lime",
                zorder=9, label="POI")
    ax3.plot([poi_r, poi_r], [0, poi["z"]], color="lime", lw=0.8,
             linestyle=":", alpha=0.5)

    ax3.set_xlim(0, reach * 1.1)
    ax3.set_ylim(0, BOX_H + reach * 1.1)
    ax3.set_xlabel("r — horizontal reach (m)")
    ax3.set_ylabel("Z (m)")
    ax3.set_title("Side view (r–Z arm plane)", fontsize=9)
    ax3.grid(True, alpha=0.3)
    canvas3.draw()

def request_draw():
    global _redraw_pending
    if not _redraw_pending:
        _redraw_pending = True
        root.after(REDRAW_MS, draw)

# ─────────────────────────────────────────
# Slider callbacks
# ─────────────────────────────────────────
def on_slider(_=None):
    if internal_update:
        request_draw()
        return
    Ex, Ey, Ez = current_EE()
    target["r"] = math.hypot(Ex, Ey)
    target["z"] = Ez
    request_draw()

for sc in joint_scales:
    sc.config(command=on_slider)
for sp in offset_spinboxes:
    sp.config(command=on_slider)

# ─────────────────────────────────────────
# Reset
# ─────────────────────────────────────────
def reset_all():
    global internal_update
    internal_update = True
    for sc in joint_scales:
        sc.set(0)
    internal_update = False
    Ex, Ey, Ez = current_EE()
    target["r"] = math.hypot(Ex, Ey)
    target["z"] = Ez
    draw()

# ─────────────────────────────────────────
# POI animation
# ─────────────────────────────────────────
def animate_step():
    global _anim_idx, internal_update
    if _anim_idx >= len(_anim_steps):
        return
    q_step = _anim_steps[_anim_idx]
    _anim_idx += 1

    apply_q_to_sliders(q_step)
    Ex, Ey, Ez = current_EE()
    target["r"] = math.hypot(Ex, Ey)
    target["z"] = Ez

    draw()
    if _anim_idx < len(_anim_steps):
        root.after(ANIM_MS, animate_step)

def go_to_poi():
    global _anim_steps, _anim_idx
    try:
        poi["x"] = float(poi_x_entry.get())
        poi["y"] = float(poi_y_entry.get())
        poi["z"] = float(poi_z_entry.get())
    except ValueError:
        return

    yaw_t = clamp(math.degrees(math.atan2(poi["y"], poi["x"])), YAW_MIN, YAW_MAX)
    r_t   = min(math.hypot(poi["x"], poi["y"]), current_reach() * 0.99)
    z_t   = clamp(poi["z"], 0.0, BOX_H + current_reach())

    # Temporarily set yaw slider so accurate IK can use it
    old_yaw = joint_scales[0].get()
    joint_scales[0].set(yaw_t)
    q_target = ik_solve_accurate(r_t, z_t)
    joint_scales[0].set(old_yaw)

    q0 = _q_from_sliders()
    _anim_steps = []
    for i in range(1, ANIM_STEPS + 1):
        t   = i / ANIM_STEPS
        t_s = t * t * (3 - 2 * t)      # smoothstep
        _anim_steps.append(q0 + (q_target - q0) * t_s)
    _anim_idx = 0
    animate_step()

# ─────────────────────────────────────────
# Mouse events — 3D plot
# ─────────────────────────────────────────
def on_press(event):
    global dragging, last_mouse
    if event.inaxes != ax or event.x is None:
        return
    Ex_px, Ey_px = get_EE_screen_xy()
    dist = math.hypot(event.x - Ex_px, event.y - Ey_px)

    if dist < 24:
        # Clicked near end-effector → drag mode
        dragging   = True
        last_mouse = (event.x, event.y)
    else:
        # ③ Clicked background → set IK target at current target["z"] plane
        result = screen_to_world_ground(event.x, event.y, target["z"])
        if result is not None:
            wx, wy = result
            yaw = joint_scales[0].get()
            target["r"] = math.hypot(wx, wy)
            # Also update yaw to face the click
            new_yaw = clamp(math.degrees(math.atan2(wy, wx)), YAW_MIN, YAW_MAX)
            global internal_update
            internal_update = True
            joint_scales[0].set(new_yaw)
            internal_update = False

            q0 = _q_from_sliders()
            if ik_mode.get() == "jacobian":
                q_new = jacobian_dls_ik(wx, wy, target["z"], q0)
            else:
                q_new = ik_solve_accurate(target["r"], target["z"])
            apply_q_to_sliders(q_new)
            Ex, Ey, Ez = current_EE()
            target["r"] = math.hypot(Ex, Ey)
            target["z"] = Ez
            request_draw()

def on_release(event):
    global dragging, last_mouse
    dragging   = False
    last_mouse = None

def arm_to_screen(r, z):
    yaw_r = deg2rad(joint_scales[0].get())
    wx = r * math.cos(yaw_r)
    wy = r * math.sin(yaw_r)
    x2, y2, _ = proj3d.proj_transform(wx, wy, z, ax.get_proj())
    sx, sy = ax.transData.transform((x2, y2))
    return sx, sy

def on_motion(event):
    global last_mouse, internal_update
    if not dragging or event.x is None:
        return
    if last_mouse is None:
        last_mouse = (event.x, event.y)
        return

    dmx = event.x - last_mouse[0]
    dmy = event.y - last_mouse[1]
    last_mouse = (event.x, event.y)

    # Local Jacobian of screen coords w.r.t. (r, z) — handles any view angle
    r0, z0 = target["r"], target["z"]
    eps  = max(current_reach() * 0.05, 0.05)
    s0   = arm_to_screen(r0,       z0)
    sr   = arm_to_screen(r0 + eps, z0)
    sz   = arm_to_screen(r0,       z0 + eps)
    jrx  = (sr[0] - s0[0]) / eps;  jry = (sr[1] - s0[1]) / eps
    jzx  = (sz[0] - s0[0]) / eps;  jzy = (sz[1] - s0[1]) / eps
    det  = jrx * jzy - jzx * jry

    if abs(det) > 1e-3:
        dr = ( jzy * dmx - jzx * dmy) / det
        dz = (-jry * dmx + jrx * dmy) / det
        target["r"] = max(0.0, r0 + dr)
        target["z"] = clamp(z0 + dz, 0.0, BOX_H + current_reach())

    yaw   = joint_scales[0].get()
    tx    = target["r"] * math.cos(deg2rad(yaw))
    ty    = target["r"] * math.sin(deg2rad(yaw))
    q0    = _q_from_sliders()

    if ik_mode.get() == "jacobian":
        q_new = jacobian_dls_ik(tx, ty, target["z"], q0)
    else:
        # LM path (slower, more accurate)
        from spatialmath import SE3 as _SE3
        T_tgt = _SE3(tx, ty, target["z"])
        sol = robot.ikine_LM(T_tgt, q0=q0, mask=[1, 1, 1, 0, 0, 0])
        q_new = sol.q if sol.success else q0

    apply_q_to_sliders(q_new)
    Ex, Ey, Ez = current_EE()
    target["r"] = math.hypot(Ex, Ey)
    target["z"] = Ez
    request_draw()

fig.canvas.mpl_connect("button_press_event",   on_press)
fig.canvas.mpl_connect("button_release_event", on_release)
fig.canvas.mpl_connect("motion_notify_event",  on_motion)

# ─────────────────────────────────────────
# Mouse events — 2D top view (drag POI)
# ─────────────────────────────────────────
def on_press_2d(event):
    global _poi_dragging
    if event.inaxes != ax2 or event.xdata is None:
        return
    if math.hypot(event.xdata - poi["x"], event.ydata - poi["y"]) \
            < current_reach() * 0.12:
        _poi_dragging = True

def on_release_2d(_):
    global _poi_dragging
    _poi_dragging = False

def on_motion_2d(event):
    if not _poi_dragging or event.inaxes != ax2 or event.xdata is None:
        return
    poi["x"] = event.xdata
    poi["y"] = event.ydata
    poi_x_entry.delete(0, END); poi_x_entry.insert(0, f"{poi['x']:.3f}")
    poi_y_entry.delete(0, END); poi_y_entry.insert(0, f"{poi['y']:.3f}")
    draw_2d()
    request_draw()

fig2.canvas.mpl_connect("button_press_event",   on_press_2d)
fig2.canvas.mpl_connect("button_release_event", on_release_2d)
fig2.canvas.mpl_connect("motion_notify_event",  on_motion_2d)

# ─────────────────────────────────────────
# Mouse events — side view (drag EE in r-Z)
# ─────────────────────────────────────────
def on_press_side(event):
    global _side_dragging
    if event.inaxes != ax3 or event.xdata is None:
        return
    Ex, Ey, Ez = current_EE()
    ee_r = math.hypot(Ex, Ey)
    if math.hypot(event.xdata - ee_r, event.ydata - Ez) < current_reach() * 0.12:
        _side_dragging = True

def on_release_side(_):
    global _side_dragging
    _side_dragging = False

def on_motion_side(event):
    global internal_update
    if not _side_dragging or event.inaxes != ax3 or event.xdata is None:
        return
    new_r = max(0.0, event.xdata)
    new_z = clamp(event.ydata, 0.0, BOX_H + current_reach())
    target["r"] = new_r
    target["z"] = new_z

    yaw = joint_scales[0].get()
    tx  = new_r * math.cos(deg2rad(yaw))
    ty  = new_r * math.sin(deg2rad(yaw))
    q0  = _q_from_sliders()
    if ik_mode.get() == "jacobian":
        q_new = jacobian_dls_ik(tx, ty, new_z, q0)
    else:
        T_tgt = SE3(tx, ty, new_z)
        sol   = robot.ikine_LM(T_tgt, q0=q0, mask=[1, 1, 1, 0, 0, 0])
        q_new = sol.q if sol.success else q0

    apply_q_to_sliders(q_new)
    Ex, Ey, Ez = current_EE()
    target["r"] = math.hypot(Ex, Ey)
    target["z"] = Ez
    request_draw()

fig3.canvas.mpl_connect("button_press_event",   on_press_side)
fig3.canvas.mpl_connect("button_release_event", on_release_side)
fig3.canvas.mpl_connect("motion_notify_event",  on_motion_side)

# ─────────────────────────────────────────
# Keyboard shortcuts
# ─────────────────────────────────────────
def on_key(event):
    key = event.keysym.lower()
    if key == "r":
        reset_all()
        return
    if key == "q":
        root.destroy()
        return
    if key in ("left", "right", "up", "down"):
        if key == "right": target["r"] = max(0.0, target["r"] + NUDGE_STEP)
        elif key == "left": target["r"] = max(0.0, target["r"] - NUDGE_STEP)
        elif key == "up":   target["z"] = clamp(target["z"] + NUDGE_STEP, 0,
                                                  BOX_H + current_reach())
        elif key == "down": target["z"] = clamp(target["z"] - NUDGE_STEP, 0,
                                                  BOX_H + current_reach())
        yaw = joint_scales[0].get()
        tx  = target["r"] * math.cos(deg2rad(yaw))
        ty  = target["r"] * math.sin(deg2rad(yaw))
        q0  = _q_from_sliders()
        q_new = jacobian_dls_ik(tx, ty, target["z"], q0)
        apply_q_to_sliders(q_new)
        Ex, Ey, Ez = current_EE()
        target["r"] = math.hypot(Ex, Ey)
        target["z"] = Ez
        draw()

root.bind("<KeyPress>", on_key)

# ─────────────────────────────────────────
# Boot
# ─────────────────────────────────────────
Ex0, Ey0, Ez0 = current_EE()
target["r"] = math.hypot(Ex0, Ey0)
target["z"] = Ez0

draw()
root.mainloop()