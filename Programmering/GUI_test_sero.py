from tkinter import *
import math

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

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
    """4x4 * 4x4"""
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

def T_rotx(rad):
    c, s = math.cos(rad), math.sin(rad)
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0,  c,  -s,  0.0],
        [0.0,  s,   c,  0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]

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

def deg2rad(d):
    return d * math.pi / 180.0

def point_from_T(T):
    """Extract position (x,y,z) from transform"""
    return (T[0][3], T[1][3], T[2][3])

# -------------------------
# Robot geometry (tune these)
# -------------------------
BOX_H = 0.35         # O->A height
L1 = 1.00            # A->B
L2 = 0.80            # B->C
L3 = 0.60            # C->D

SHOULDER_ZERO_OFFSET_DEG = 45  # slider 0 means actual shoulder = 45°

# -------------------------
# Forward kinematics with real 4x4 matrices
# -------------------------
def fk_chain(yaw_deg, shoulder_slider_deg, elbow_deg, wrist_deg):
    """
    Returns transforms at O, A, B, C, D as 4x4 matrices.
    Convention used here:
      - Base yaw = rotation about Z at the base (O frame)
      - Shoulder/elbow/wrist are pitch rotations about Y
      - Each link translates along +X in its *local* frame
      - O->A is a translation +Z (box height)
    """
    yaw = deg2rad(yaw_deg)
    sh  = deg2rad(shoulder_slider_deg + SHOULDER_ZERO_OFFSET_DEG)  # offset makes slider 0 = 45°
    el  = deg2rad(elbow_deg)
    wr  = deg2rad(wrist_deg)

    # O frame (world)
    T_O = T_identity()

    # Base yaw at O
    T_yaw = T_rotz(yaw)

    # O -> A (top of box)
    T_OA = T_trans(0, 0, BOX_H)

    # A -> B: shoulder rotation then translate along x by L1
    T_AB = T_mul(T_roty(sh), T_trans(L1, 0, 0))

    # B -> C: elbow rotation (relative) then translate along x by L2
    T_BC = T_mul(T_roty(el), T_trans(L2, 0, 0))

    # C -> D: wrist rotation (relative) then translate along x by L3
    T_CD = T_mul(T_roty(wr), T_trans(L3, 0, 0))

    # Chain multiply
    T_A = T_mul(T_mul(T_O, T_yaw), T_OA)
    T_B = T_mul(T_A, T_AB)
    T_C = T_mul(T_B, T_BC)
    T_D = T_mul(T_C, T_CD)

    return T_O, T_A, T_B, T_C, T_D

# -------------------------
# Tkinter + Matplotlib
# -------------------------
root = Tk()
root.title("3D Arm Visualizer (Real 4x4 Matrices)")
root.geometry("980x650")

fig = Figure(figsize=(7.2, 5.4))
ax = fig.add_subplot(111, projection="3d")

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=LEFT, fill=BOTH, expand=True)

controls = Frame(root)
controls.pack(side=RIGHT, fill=Y, padx=12, pady=12)

info = Label(controls, text="", justify=LEFT)
info.pack(pady=8)

# Base can rotate 180 total -> -90..+90
yaw_scale = Scale(controls, from_=-90, to=90, orient=HORIZONTAL, label="Base yaw (180° total)", length=250)
yaw_scale.set(0)
yaw_scale.pack(fill="x")

shoulder_scale = Scale(controls, from_=-90, to=90, orient=HORIZONTAL,
                       label="Shoulder slider (0 = 45° actual)", length=250)
shoulder_scale.set(0)
shoulder_scale.pack(fill="x")

elbow_scale = Scale(controls, from_=-135, to=135, orient=HORIZONTAL, label="Elbow pitch (relative)", length=250)
elbow_scale.set(0)
elbow_scale.pack(fill="x")

wrist_scale = Scale(controls, from_=-135, to=135, orient=HORIZONTAL, label="Wrist pitch (relative)", length=250)
wrist_scale.set(0)
wrist_scale.pack(fill="x")

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

    # Joint labels
    ax.text(O[0], O[1], O[2], "  O")
    ax.text(A[0], A[1], A[2], "  A")
    ax.text(B[0], B[1], B[2], "  B")
    ax.text(C[0], C[1], C[2], "  C")
    ax.text(D[0], D[1], D[2], "  D")

    reach = L1 + L2 + L3
    ax.set_xlim(-reach, reach)
    ax.set_ylim(-reach, reach)
    ax.set_zlim(0, BOX_H + reach)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    shoulder_actual = shs + SHOULDER_ZERO_OFFSET_DEG
    info.config(text=
        f"Yaw: {yaw:+d}° (limit -90..+90)\n"
        f"Shoulder slider: {shs:+d}°  -> actual: {shoulder_actual:+d}°\n"
        f"Elbow: {el:+d}° (relative)\n"
        f"Wrist: {wr:+d}° (relative)\n"
        f"\nEnd effector D: x={D[0]:.3f}, y={D[1]:.3f}, z={D[2]:.3f}"
    )

    canvas.draw()

def on_slider(_=None):
    draw()

yaw_scale.config(command=on_slider)
shoulder_scale.config(command=on_slider)
elbow_scale.config(command=on_slider)
wrist_scale.config(command=on_slider)

def reset_all():
    yaw_scale.set(0)
    shoulder_scale.set(0)  # actual becomes 45°
    elbow_scale.set(0)
    wrist_scale.set(0)
    draw()

Button(controls, text="Reset", command=reset_all).pack(pady=12)

draw()
root.mainloop()
