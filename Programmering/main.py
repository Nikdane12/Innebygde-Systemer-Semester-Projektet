from tkinter import *
import subprocess
import math
import i2c
import hx711

# Arm geometry (measure in cm)
L1     = 15.0   # Skulder -> Albue
L2     = 15.0   # Albue   -> Wrist
L3     = 5.0    # Wrist   -> end effector
Z_BASE =  5.0   # height of arm base above floor

# Servo mounting offsets (degrees)
MOUNT_SKULDER = 45.0 
MOUNT_ALBUE   =  0.0
MOUNT_WRIST   =  0.0

# Inverse kinematics
#
#  Midje  : θ_base = atan2(y, x)          — base rotation on floor plane
#
#  Planar IK in vertical plane (r = horizontal reach, z = height):
#
#    Wrist point:
#      xw = r - L3·cos(φ)
#      yw = z - L3·sin(φ)
#
#    Albue (θ2) via cosine rule:
#      cos(θ2) = (xw² + yw² - L1² - L2²) / (2·L1·L2)
#      θ2 = atan2(±√(1 - cos²θ2), cos θ2)    [+ = elbow up]
#
#    Skulder (θ1):
#      θ1 = atan2(yw, xw) - atan2(L2·sin θ2, L1 + L2·cos θ2)
#
#    Wrist (θ3):
#      θ3 = φ - θ1 - θ2

def solve_ik(x, y, z, phi_deg, elbow_up=True):
    phi = math.radians(phi_deg)

    # Base rotation
    theta_base = math.atan2(y, x)
    r = math.hypot(x, y)            # horizontal reach

    # Wrist point in planar (r, z) frame — z is relative to floor, subtract base mount height
    xw = r - L3 * math.cos(phi)
    yw = (z - Z_BASE) - L3 * math.sin(phi)

    # Cosine rule for albue
    cos_t2 = (xw**2 + yw**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_t2 = max(-1.0, min(1.0, cos_t2))   # clamp for numerical safety

    sign = 1 if elbow_up else -1
    sin_t2 = sign * math.sqrt(1 - cos_t2**2)
    theta2 = math.atan2(sin_t2, cos_t2)

    # Skulder
    theta1 = math.atan2(yw, xw) - math.atan2(L2 * sin_t2, L1 + L2 * cos_t2)

    # Wrist
    theta3 = phi - theta1 - theta2

    # Subtract mount offsets to convert world-space angles → servo commands
    return (
        math.degrees(theta_base),
        math.degrees(theta1) - MOUNT_SKULDER,
        math.degrees(theta2) - MOUNT_ALBUE,
        math.degrees(theta3) - MOUNT_WRIST,
    )

# GUI

root = Tk()
root.title("Arm Controller")
root.geometry("540x900")

# Joint variables (degrees / percent)
midje_var   = DoubleVar(value=0)
skulder_var = DoubleVar(value=0)
albue_var   = DoubleVar(value=0)
wrist_var   = DoubleVar(value=0)
pump_var    = DoubleVar(value=0)

JOINT_VARS = [midje_var, skulder_var, albue_var, wrist_var, pump_var]

def get_joints():
    return [v.get() for v in JOINT_VARS]

def set_joints(values):
    for var, val in zip(JOINT_VARS, values):
        var.set(val)
    i2c.drive(*values)

# Sliders

Label(root, text=" Joint Control ", font=("Segoe UI", 11, "bold")).pack(pady=(10, 2))

def make_slider(label, var, from_, to):
    f = Frame(root)
    f.pack(fill="x", padx=20, pady=1)
    Label(f, text=label, width=8, anchor="w").pack(side=LEFT)
    s = Scale(f, variable=var, from_=from_, to=to, orient=HORIZONTAL,
              length=380, resolution=1)
    s.pack(side=LEFT, fill="x", expand=True)
    s.config(command=lambda _: i2c.drive(*get_joints()))

make_slider("Midje",   midje_var,   -90, 90)
make_slider("Skulder", skulder_var, -45, 45)
make_slider("Albue",   albue_var,   -45, 45)
make_slider("Wrist",   wrist_var,   -45, 45)
make_slider("Pump",    pump_var,      0, 100)

def reset_all():
    set_joints([0, 0, 0, 0, 0])

Button(root, text="Reset", command=reset_all).pack(pady=4)

# Inverse kinematics input 

Label(root, text=" Inverse Kinematics ", font=("Segoe UI", 11, "bold")).pack(pady=(14, 2))

ik_frame = Frame(root)
ik_frame.pack(padx=20, fill="x")

def _entry_row(parent, label, default):
    f = Frame(parent)
    f.pack(fill="x", pady=1)
    Label(f, text=label, width=12, anchor="w").pack(side=LEFT)
    var = DoubleVar(value=default)
    Entry(f, textvariable=var, width=8).pack(side=LEFT)
    return var

ik_x              = _entry_row(ik_frame, "X (cm)",          15.0)
ik_y              = _entry_row(ik_frame, "Y (cm)",           0.0)
ik_z              = _entry_row(ik_frame, "Z (cm)",          10.0)
ik_phi            = _entry_row(ik_frame, "φ orient°",        0.0)
ik_zbase          = _entry_row(ik_frame, "Base ht (cm)",   Z_BASE)
ik_mount_skulder  = _entry_row(ik_frame, "Skulder mount°", MOUNT_SKULDER)

ik_status = Label(root, text="", fg="red")
ik_status.pack()

def run_ik():
    try:
        global Z_BASE, MOUNT_SKULDER
        Z_BASE        = ik_zbase.get()
        MOUNT_SKULDER = ik_mount_skulder.get()
        base, t1, t2, t3 = solve_ik(
            ik_x.get(), ik_y.get(), ik_z.get(), ik_phi.get()
        )
        target = [base, t1, t2, t3, pump_var.get()]
        global _move_start, _move_target
        _move_start  = get_joints()
        _move_target = target
        _smooth_step(0)
        ik_status.config(text=f"M:{base:+.1f}°  S:{t1:+.1f}°  A:{t2:+.1f}°  W:{t3:+.1f}°", fg="green")
    except Exception as e:
        ik_status.config(text=str(e), fg="red")

Button(root, text="Move to IK target", command=run_ik).pack(pady=4)

# Saved positions 
Label(root, text=" Saved Positions ", font=("Segoe UI", 11, "bold")).pack(pady=(14, 2))

NUM_POS = 3
saved   = [None] * NUM_POS

_move_start  = [0.0] * 5
_move_target = [0.0] * 5
MOVE_STEPS   = 40
MOVE_MS      = 15

def _smooth_step(step):
    t = step / MOVE_STEPS
    t = t * t * (3 - 2 * t)
    interp = [_move_start[i] + (_move_target[i] - _move_start[i]) * t for i in range(5)]
    set_joints(interp)
    if step < MOVE_STEPS:
        root.after(MOVE_MS, lambda: _smooth_step(step + 1))

def go_to(idx):
    if saved[idx] is None:
        return
    global _move_start, _move_target
    _move_start  = get_joints()
    _move_target = list(saved[idx])
    _smooth_step(0)

def save_pos(idx):
    saved[idx] = get_joints()
    m, s, a, w, p = saved[idx]
    pos_labels[idx].config(
        text=f"M:{m:+.0f}°  S:{s:+.0f}°  A:{a:+.0f}°  W:{w:+.0f}°  P:{p:.0f}%"
    )
    go_btns[idx].config(state=NORMAL)

pos_labels = []
go_btns    = []

for i in range(NUM_POS):
    f = Frame(root, relief="groove", bd=1)
    f.pack(fill="x", padx=20, pady=3)
    Label(f, text=f"Position {i+1}", font=("Segoe UI", 9, "bold"), width=10).pack(side=LEFT, padx=4)
    lbl = Label(f, text="(empty)", anchor="w", width=32)
    lbl.pack(side=LEFT)
    pos_labels.append(lbl)
    Button(f, text="Save", width=5, command=lambda i=i: save_pos(i)).pack(side=LEFT, padx=2)
    go = Button(f, text="Go", width=4, state=DISABLED, command=lambda i=i: go_to(i))
    go.pack(side=LEFT, padx=2)
    go_btns.append(go)

# Load cell
Label(root, text=" Load Cell ", font=("Segoe UI", 11, "bold")).pack(pady=(14, 2))

use_median = False

for lbl_text, lbl_var in [("Sensor 1", None), ("Sensor 2", None), ("Sensor 3", None)]:
    Label(root, text=lbl_text, font=("Segoe UI", 9)).pack()

weight_lbl1 = Label(root, text="-- g", font=("Segoe UI", 24, "bold"))
weight_lbl2 = Label(root, text="-- g", font=("Segoe UI", 24, "bold"))
weight_lbl3 = Label(root, text="-- g", font=("Segoe UI", 24, "bold"))
weight_lbl1.pack()
weight_lbl2.pack()
weight_lbl3.pack()

mode_btn = Button(root, text="Mode: Live", width=16)
mode_btn.pack(pady=4)

def toggle_mode():
    global use_median
    use_median = not use_median
    mode_btn.config(text="Mode: Median" if use_median else "Mode: Live")

mode_btn.config(command=toggle_mode)

def _read(sensor):
    if use_median:
        return (sensor.read_median() - sensor.hx_offset) / sensor.scale_factor
    return sensor.read_grams()

def poll_hx711():
    interval = 500 if use_median else 200
    for sensor, lbl in [(hx711.sensor1, weight_lbl1),
                        (hx711.sensor2, weight_lbl2),
                        (hx711.sensor3, weight_lbl3)]:
        try:
            lbl.config(text=f"{_read(sensor):.1f} g")
        except Exception:
            lbl.config(text="err")
    root.after(interval, poll_hx711)

def calibrate_hx711():
    def do_calibrate():
        try:
            hx711.sensor1.calibrate(float(cal_entry.get()))
            hx711.sensor2.calibrate(float(cal_entry.get()))
            hx711.sensor3.calibrate(float(cal_entry.get()))
            cal_window.destroy()
        except Exception as e:
            cal_status.config(text=str(e), fg="red")

    cal_window = Toplevel(root)
    cal_window.title("Calibrate Load Cell")
    Label(cal_window, text="Place known weight on scale, enter grams:").pack(padx=20, pady=10)
    cal_entry = Entry(cal_window)
    cal_entry.pack(padx=20, pady=5)
    cal_entry.focus()
    cal_status = Label(cal_window, text="", fg="red")
    cal_status.pack()
    Button(cal_window, text="Calibrate", command=do_calibrate).pack(pady=10)

Button(root, text="Tare",      command=hx711.sensor1.tare).pack(pady=2)
Button(root, text="Calibrate", command=calibrate_hx711).pack(pady=2)

# Benchmark launcher 
Button(root, text="Open Benchmark GUI",
       command=lambda: subprocess.Popen(["python", "GUI/GUI_benchmark.py"])
       ).pack(pady=10)

# Start

reset_all()
poll_hx711()
root.mainloop()

hx711.sensor1.close()
hx711.sensor2.close()
hx711.sensor3.close()
hx711.sck.close()
i2c.bus.close()
