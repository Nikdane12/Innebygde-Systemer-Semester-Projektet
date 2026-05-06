from tkinter import *
from tkinter import ttk
import time
import threading
import subprocess
import math
import i2c
import hx711

from gpiozero import OutputDevice
_pump_fwd = OutputDevice(21, initial_value=False)

# Main program state machine
activate     = False
_fill_state  = "idle"   # idle | moving | filling

# Arm geometry (cm)
L1     = 15.0
L2     = 15.0
L3     = 5.0
Z_BASE =  5.0

MOUNT_SKULDER = 45.0
MOUNT_ALBUE   =  0.0
MOUNT_WRIST   =  0.0


def solve_ik(x, y, z, phi_deg, elbow_up=True):
    phi = math.radians(phi_deg)
    theta_base = math.atan2(y, x)
    r = math.hypot(x, y)

    xw = r - L3 * math.cos(phi)
    yw = (z - Z_BASE) - L3 * math.sin(phi)

    cos_t2 = (xw**2 + yw**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_t2 = max(-1.0, min(1.0, cos_t2))

    sign = 1 if elbow_up else -1
    sin_t2 = sign * math.sqrt(1 - cos_t2**2)
    theta2 = math.atan2(sin_t2, cos_t2)

    theta1 = math.atan2(yw, xw) - math.atan2(L2 * sin_t2, L1 + L2 * cos_t2)
    theta3 = phi - theta1 - theta2

    return (
        math.degrees(theta_base),
        math.degrees(theta1) - MOUNT_SKULDER,
        math.degrees(theta2) - MOUNT_ALBUE,
        math.degrees(theta3) - MOUNT_WRIST,
    )

# ---------------- PID ----------------
kp = 0.5
ki = 0.1
kd = 0.05

integral = 0
prev_error = 0
prev_time = time.time()

def pid_control(setpoint, measurement, integral, prev_error, prev_time):
    error = setpoint - measurement
    current_time = time.time()
    dt = current_time - prev_time
    if dt <= 0:
        return 0, integral, error, current_time
    integral += error * dt
    derivative = (error - prev_error) / dt
    output = kp * error + ki * integral + kd * derivative
    return output, integral, error, current_time

# ---------------- Background sensor thread ----------------
# Reads all three HX711 sensors in a background thread so the GUI never blocks
# on a slow chip read. The GUI just consumes the latest cached values.
_SENSORS = [hx711.sensor1, hx711.sensor2, hx711.sensor3]
_latest_grams  = [0.0, 0.0, 0.0]
_sensor_errors = [False, False, False]
_sensor_lock   = threading.Lock()
_stop_workers  = threading.Event()

def _sensor_worker():
    while not _stop_workers.is_set():
        for i, s in enumerate(_SENSORS):
            try:
                g = s.read_grams()
                with _sensor_lock:
                    _latest_grams[i]  = g
                    _sensor_errors[i] = False
            except Exception:
                with _sensor_lock:
                    _sensor_errors[i] = True
        # Small sleep so the thread doesn't spin if reads happen to be fast
        time.sleep(0.02)

def get_grams(idx):
    with _sensor_lock:
        return _latest_grams[idx], _sensor_errors[idx]

threading.Thread(target=_sensor_worker, daemon=True).start()

# ---------------- I2C drive throttling ----------------
# Slider drags fire the command callback on every integer step. Without
# throttling this floods the I2C bus and stutters the GUI. We debounce: the
# latest value is always sent, but at most once per DRIVE_DEBOUNCE_MS.
DRIVE_DEBOUNCE_MS = 25
_drive_after_id = None

def _drive_now():
    global _drive_after_id
    _drive_after_id = None
    try:
        i2c.drive(*get_joints())
    except Exception as e:
        print("i2c.drive error:", e)

def schedule_drive():
    global _drive_after_id
    if _drive_after_id is None:
        _drive_after_id = root.after(DRIVE_DEBOUNCE_MS, _drive_now)

# ---------------- GUI ----------------
root = Tk()
root.title("Arm Controller")
root.geometry("800x800")

# Scrollable main frame
_canvas  = Canvas(root)
_vscroll = Scrollbar(root, orient=VERTICAL, command=_canvas.yview)
_canvas.configure(yscrollcommand=_vscroll.set)
_vscroll.pack(side=RIGHT, fill=Y)
_canvas.pack(side=LEFT, fill=BOTH, expand=True)
main = Frame(_canvas)
_canvas_window = _canvas.create_window((0, 0), window=main, anchor="nw")

def _on_frame_configure(_):
    _canvas.configure(scrollregion=_canvas.bbox("all"))
def _on_canvas_configure(e):
    _canvas.itemconfig(_canvas_window, width=e.width)
main.bind("<Configure>", _on_frame_configure)
_canvas.bind("<Configure>", _on_canvas_configure)

# Cross-platform mouse wheel: Windows/macOS use <MouseWheel> with delta
# multiples of 120; X11/Linux uses <Button-4>/<Button-5>.
def _on_mousewheel(e):
    if e.delta:
        _canvas.yview_scroll(-1 if e.delta > 0 else 1, "units")
root.bind_all("<MouseWheel>", _on_mousewheel)
root.bind_all("<Button-4>", lambda e: _canvas.yview_scroll(-1, "units"))
root.bind_all("<Button-5>", lambda e: _canvas.yview_scroll( 1, "units"))

# Joint variables
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
    schedule_drive()

# Sliders
Label(main, text=" Joint Control ", font=("Segoe UI", 11, "bold")).pack(pady=(10, 2))

def make_slider(label, var, from_, to):
    f = Frame(main)
    f.pack(fill="x", padx=20, pady=1)
    Label(f, text=label, width=8, anchor="w").pack(side=LEFT)
    s = Scale(f, variable=var, from_=from_, to=to, orient=HORIZONTAL,
              length=380, resolution=1)
    s.pack(side=LEFT, fill="x", expand=True)
    s.config(command=lambda _: schedule_drive())

make_slider("Midje",   midje_var,   -90, 90)
make_slider("Skulder", skulder_var, -45, 45)
make_slider("Albue",   albue_var,   -45, 45)
make_slider("Wrist",   wrist_var,   -45, 45)

# Pump slider — controls IN1 (GPIO 21) based on value
def _on_pump_slider(_):
    pct = int(pump_var.get())
    if pct == 0:
        _pump_fwd.off()
    else:
        _pump_fwd.on()
    schedule_drive()

f_pump = Frame(main)
f_pump.pack(fill="x", padx=20, pady=1)
Label(f_pump, text="Pump", width=8, anchor="w").pack(side=LEFT)
pump_scale = Scale(f_pump, variable=pump_var, from_=0, to=100,
                   orient=HORIZONTAL, length=380, resolution=1,
                   command=_on_pump_slider)
pump_scale.pack(side=LEFT, fill="x", expand=True)

def reset_all():
    set_joints([0, 0, 0, 0, 0])

Button(main, text="Reset", command=reset_all).pack(pady=4)

def toggle_activate():
    global activate
    activate = not activate
    activate_btn.config(
        text="Deactivate" if activate else "Activate Main Program",
        bg="red" if activate else _activate_btn_default_bg
    )

activate_btn = Button(main, text="Activate Main Program", command=toggle_activate,
                      font=("Segoe UI", 10, "bold"), width=24)
activate_btn.pack(pady=6)
_activate_btn_default_bg = activate_btn.cget("bg")

# Inverse kinematics input
Label(main, text=" Inverse Kinematics ", font=("Segoe UI", 11, "bold")).pack(pady=(14, 2))

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

ik_status = Label(main, text="", fg="red")
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

Button(main, text="Move to IK target", command=run_ik).pack(pady=4)

# Movement speed
_move_start  = [0.0] * 5
_move_target = [0.0] * 5
MOVE_STEPS   = 40
MOVE_MS      = 15

f_speed = Frame(main)
f_speed.pack(fill="x", padx=20, pady=(4, 0))
Label(f_speed, text="Speed", width=8, anchor="w").pack(side=LEFT)
_speed_var = IntVar(value=MOVE_MS)
Label(f_speed, text="Fast", width=4).pack(side=LEFT)

def _set_speed(v):
    global MOVE_MS
    MOVE_MS = v

Scale(f_speed, variable=_speed_var, from_=5, to=80,
      orient=HORIZONTAL, length=300, resolution=5,
      command=lambda v: _set_speed(int(v))
      ).pack(side=LEFT)
Label(f_speed, text="Slow").pack(side=LEFT)

# Saved positions
Label(main, text=" Saved Positions ", font=("Segoe UI", 11, "bold")).pack(pady=(14, 2))

NUM_POS = 3
saved   = [None] * NUM_POS

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
    f = Frame(main, relief="groove", bd=1)
    f.pack(fill="x", padx=20, pady=3)
    Label(f, text=f"Position {i+1}", font=("Segoe UI", 9, "bold"), width=10).pack(side=LEFT, padx=4)
    lbl = Label(f, text="(empty)", anchor="w", width=32)
    lbl.pack(side=LEFT)
    pos_labels.append(lbl)
    Button(f, text="Save", width=5, command=lambda i=i: save_pos(i)).pack(side=LEFT, padx=2)
    go = Button(f, text="Go", width=4, state=DISABLED, command=lambda i=i: go_to(i))
    go.pack(side=LEFT, padx=2)
    go_btns.append(go)

# ---------------- Load cell display ----------------
Label(main, text=" Load Cell ", font=("Segoe UI", 11, "bold")).pack(pady=(14, 2))

# Mode toggle (only meaningful for the calibration window's median reads,
# kept for compatibility — the live display always uses the cached values).
use_median = False

for lbl_text in ["Sensor 1", "Sensor 2", "Sensor 3"]:
    Label(main, text=lbl_text, font=("Segoe UI", 9)).pack()

weight_lbl1 = Label(main, text="-- g", font=("Segoe UI", 24, "bold"))
weight_lbl2 = Label(main, text="-- g", font=("Segoe UI", 24, "bold"))
weight_lbl3 = Label(main, text="-- g", font=("Segoe UI", 24, "bold"))
weight_lbl1.pack()
weight_lbl2.pack()
weight_lbl3.pack()
_weight_lbls = [weight_lbl1, weight_lbl2, weight_lbl3]

mode_btn = Button(main, text="Mode: Live", width=16)
mode_btn.pack(pady=4)

def toggle_mode():
    global use_median
    use_median = not use_median
    mode_btn.config(text="Mode: Median" if use_median else "Mode: Live")
mode_btn.config(command=toggle_mode)

def open_calibration():
    win = Toplevel(root)
    win.title("Load Cell Calibration")
    win.resizable(False, False)
    win_alive = {"v": True}

    def on_close():
        win_alive["v"] = False
        win.destroy()
    win.protocol("WM_DELETE_WINDOW", on_close)

    for i, sensor in enumerate(_SENSORS):
        frame = LabelFrame(win, text=f"Sensor {i+1}", padx=10, pady=6)
        frame.pack(fill="x", padx=12, pady=6)

        reading_lbl = Label(frame, text="-- g", font=("Segoe UI", 13, "bold"), width=10)
        reading_lbl.pack(side=RIGHT)

        def refresh(lbl=reading_lbl, idx=i):
            if not win_alive["v"]:
                return
            grams, err = get_grams(idx)
            lbl.config(text="err" if err else f"{grams:.1f} g")
            win.after(300, lambda: refresh(lbl, idx))
        refresh()

        Button(frame, text="Tare", width=8,
               command=lambda s=sensor: s.tare()).pack(side=LEFT, padx=4)

        cal_entry = Entry(frame, width=7)
        cal_entry.insert(0, "200")
        cal_entry.pack(side=LEFT, padx=4)
        Label(frame, text="g").pack(side=LEFT)

        status_lbl = Label(frame, text="", fg="green", width=10)
        status_lbl.pack(side=LEFT, padx=4)

        def do_cal(s=sensor, e=cal_entry, lbl=status_lbl):
            try:
                s.calibrate(float(e.get()))
                lbl.config(text="OK", fg="green")
            except Exception as ex:
                lbl.config(text=str(ex), fg="red")

        Button(frame, text="Calibrate", width=9, command=do_cal).pack(side=LEFT, padx=2)

Button(main, text="Tare & Calibrate", command=open_calibration).pack(pady=4)

# ---------------- Fill control ----------------
FILL_THRESHOLD_G = 150
FILL_TARGET_G    = 400
PID_POLL_MS      = 200

_fill_sensor_idx = 0

Label(main, text=" Fill Settings ", font=("Segoe UI", 11, "bold")).pack(pady=(14, 2))
_fill_frame = Frame(main)
_fill_frame.pack(padx=20, fill="x")

def _fill_row(parent, label, default):
    f = Frame(parent)
    f.pack(fill="x", pady=1)
    Label(f, text=label, width=16, anchor="w").pack(side=LEFT)
    var = DoubleVar(value=default)
    Entry(f, textvariable=var, width=8).pack(side=LEFT)
    Label(f, text="g").pack(side=LEFT)
    return var

_threshold_var = _fill_row(_fill_frame, "Start below (g)",  FILL_THRESHOLD_G)
_target_var    = _fill_row(_fill_frame, "Fill target (g)",  FILL_TARGET_G)

fill_status = Label(main, text="", fg="gray")
fill_status.pack()

def _apply_fill_settings():
    global FILL_THRESHOLD_G, FILL_TARGET_G
    FILL_THRESHOLD_G = _threshold_var.get()
    FILL_TARGET_G    = _target_var.get()
    fill_status.config(text=f"Threshold={FILL_THRESHOLD_G:.0f}g  Target={FILL_TARGET_G:.0f}g", fg="green")

Button(main, text="Apply Fill Settings", command=_apply_fill_settings).pack(pady=4)

def _start_fill(idx):
    global _fill_state, _fill_sensor_idx
    _fill_state      = "moving"
    _fill_sensor_idx = idx
    go_to(idx)
    root.after(MOVE_STEPS * MOVE_MS + 500, _begin_pump)

def _begin_pump():
    global _fill_state, integral, prev_error, prev_time
    _fill_state = "filling"
    _pump_fwd.on()
    integral   = 0
    prev_error = 0
    prev_time  = time.time()
    fill_status.config(text="Filling...", fg="blue")
    _pid_fill_loop()

def _pid_fill_loop():
    global integral, prev_error, prev_time
    if _fill_state != "filling":
        return

    grams, err = get_grams(_fill_sensor_idx)
    if err:
        fill_status.config(text="Sensor error", fg="red")
        root.after(PID_POLL_MS, _pid_fill_loop)
        return

    if grams < 0:
        _stop_pump()
        fill_status.config(text="Glass lifted — fill cancelled", fg="orange")
        return

    if grams >= FILL_TARGET_G:
        _stop_pump()
        return

    output, integral, prev_error, prev_time = pid_control(
        FILL_TARGET_G, grams, integral, prev_error, prev_time
    )
    power = max(5, min(100, int(output)))
    _pump_fwd.on()
    try:
        i2c.set_duty(i2c.CH_PUMP, power)
    except Exception as e:
        print("i2c.set_duty error:", e)
    pump_var.set(power)
    fill_status.config(text=f"Filling: {grams:.0f} / {FILL_TARGET_G:.0f} g  |  pump {power}%", fg="blue")

    root.after(PID_POLL_MS, _pid_fill_loop)

def _stop_pump():
    global _fill_state
    try:
        i2c.set_duty(i2c.CH_PUMP, 0)
    except Exception:
        pass
    _pump_fwd.off()
    pump_var.set(0)
    _fill_state = "idle"
    fill_status.config(text=f"Done — {FILL_TARGET_G:.0f} g reached", fg="green")
    root.after(1000, _main_loop)

def _main_loop():
    if not activate or _fill_state != "idle":
        root.after(500, _main_loop)
        return
    for idx in range(3):
        grams, err = get_grams(idx)
        if err:
            continue
        if grams < 0:
            continue
        if grams < FILL_THRESHOLD_G:
            _start_fill(idx)
            return
    root.after(500, _main_loop)

# ---------------- Water level display ----------------
GLASS_MAX_G = 500

Label(main, text=" Water Level ", font=("Segoe UI", 11, "bold")).pack(pady=(10, 2))

_bar_vars  = []
_bar_grams = []

for name in ["Glass 1", "Glass 2", "Glass 3"]:
    f = Frame(root)
    f.pack(fill="x", padx=20, pady=3)
    Label(f, text=name, width=8, anchor="w").pack(side=LEFT)
    var = IntVar(value=0)
    bar = ttk.Progressbar(f, variable=var, maximum=GLASS_MAX_G, length=300)
    bar.pack(side=LEFT, fill="x", expand=True, padx=4)
    g_lbl = Label(f, text="-- g", width=8, anchor="e")
    g_lbl.pack(side=LEFT)
    _bar_vars.append(var)
    _bar_grams.append(g_lbl)

# One unified UI poll for everything sensor-driven. Reads from the cached
# values populated by the background thread, so this never blocks.
UI_POLL_MS = 200

def _poll_sensors_ui():
    for i in range(3):
        grams, err = get_grams(i)
        if err:
            _weight_lbls[i].config(text="err")
            _bar_grams[i].config(text="err")
        else:
            _weight_lbls[i].config(text=f"{grams:.1f} g")
            _bar_vars[i].set(max(0, min(GLASS_MAX_G, int(grams))))
            _bar_grams[i].config(text=f"{grams:.0f} g")
    root.after(UI_POLL_MS, _poll_sensors_ui)

# ---------------- PID tuning ----------------
Label(main, text=" PID Controller ", font=("Segoe UI", 11, "bold")).pack(pady=(14, 2))

pid_frame = Frame(main)
pid_frame.pack(padx=20, fill="x")

def _pid_row(parent, label, default):
    f = Frame(parent)
    f.pack(fill="x", pady=1)
    Label(f, text=label, width=6, anchor="w").pack(side=LEFT)
    var = DoubleVar(value=default)
    Entry(f, textvariable=var, width=8).pack(side=LEFT)
    return var

pid_kp_var = _pid_row(pid_frame, "Kp", kp)
pid_ki_var = _pid_row(pid_frame, "Ki", ki)
pid_kd_var = _pid_row(pid_frame, "Kd", kd)

pid_status = Label(main, text=f"Active: Kp={kp}  Ki={ki}  Kd={kd}", fg="gray")
pid_status.pack()

def apply_pid():
    global kp, ki, kd, integral, prev_error, prev_time
    kp = pid_kp_var.get()
    ki = pid_ki_var.get()
    kd = pid_kd_var.get()
    integral   = 0
    prev_error = 0
    prev_time  = time.time()
    pid_status.config(text=f"Active: Kp={kp}  Ki={ki}  Kd={kd}", fg="green")

Button(main, text="Apply PID", command=apply_pid).pack(pady=4)

Button(main, text="Open Benchmark GUI",
       command=lambda: subprocess.Popen(["python", "GUI/GUI_benchmark.py"])
       ).pack(pady=10)

# ---------------- Clean shutdown ----------------
def _on_close():
    _stop_workers.set()
    try:
        root.destroy()
    except Exception:
        pass

root.protocol("WM_DELETE_WINDOW", _on_close)

# Start
reset_all()
_poll_sensors_ui()
_main_loop()
root.mainloop()

# Cleanup
_stop_workers.set()
try:
    hx711.sensor1.close()
    hx711.sensor2.close()
    hx711.sensor3.close()
    hx711.sck.close()
    _pump_fwd.close()
    i2c.bus.close()
except Exception:
    pass