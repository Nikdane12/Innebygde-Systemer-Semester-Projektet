from tkinter import *
from tkinter import ttk
from adafruit_servokit import ServoKit
import time
import subprocess
import math
import i2c
import hx711

from gpiozero import OutputDevice
_pump_fwd = OutputDevice(21, initial_value=False)

kit = ServoKit(channels=16)

hx711.sensor1.start_background(interval=0.1)
hx711.sensor2.start_background(interval=0.1)
hx711.sensor3.start_background(interval=0.1)

def drive(midje, skulder, albue, wrist, pump):
    # Constants from original i2c.py
    # center_us = 6554/2
    # us_per_deg = 1000 / 90

    # midje_min_duty_cycle = 3277
    # midje_max_duty_cycle = 6554

    # max = 6554
    # min = 3277
    min = 2350
    max = 5650


    def angle_to_us(deg):
        return int(max - ((max - min) * (deg / 90)))

    # def midje_to_us(deg):
    #     return int(((max-min)*(deg/90)) + min)

    # Set duty cycles for servos based on pulse width
    kit._pca.channels[0].duty_cycle = 65535 - angle_to_us(midje)
    kit._pca.channels[1].duty_cycle = 65535 - angle_to_us(skulder)
    kit._pca.channels[2].duty_cycle = 65535 - angle_to_us(albue)
    kit._pca.channels[3].duty_cycle = 65535 - angle_to_us(wrist)
    
    # Set duty cycle for pump (inverted as in your example)
    desired_duty = int(pump / 100 * 65535)
    kit._pca.channels[4].duty_cycle = 65535 - desired_duty
 
# Main program state machine
activate     = False
_fill_state  = "idle"   # idle | moving | filling

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
#      xw = r - L3·cos(phi)
#      yw = z - L3·sin(phi)
#
#    Albue (θ2) via cosine rule:
#      cos(θ2) = (xw² + yw² - L1² - L2²) / (2·L1·L2)
#      θ2 = atan2(±√(1 - cos²θ2), cos θ2)    [+ = elbow up]
#
#    Skulder (θ1):
#      θ1 = atan2(yw, xw) - atan2(L2·sin θ2, L1 + L2·cos θ2)
#
#    Wrist (θ3):
#      θ3 = phi - θ1 - θ2

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

#PID-controller
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

# GUI
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
root.bind_all("<MouseWheel>", lambda e: _canvas.yview_scroll(-1*(e.delta//120), "units"))

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
    drive(*values)

# Sliders
Label(main, text=" Joint Control ", font=("Segoe UI", 11, "bold")).pack(pady=(10, 2))

def make_slider(label, var, from_, to):
    f = Frame(main)
    f.pack(fill="x", padx=20, pady=1)
    Label(f, text=label, width=8, anchor="w").pack(side=LEFT)
    s = Scale(f, variable=var, from_=from_, to=to, orient=HORIZONTAL,
              length=380, resolution=1)
    s.pack(side=LEFT, fill="x", expand=True)
    s.config(command=lambda _: drive(*get_joints()))

make_slider("Midje",   midje_var,  45, -45)
make_slider("Skulder", skulder_var, 45, -45)
make_slider("Albue",   albue_var,   45, -45)
make_slider("Wrist",   wrist_var,   45, -45)

# Pump slider — auto-controls IN1 (GPIO 21) based on value
def _on_pump_slider(_):
    pct = int(pump_var.get())
    if pct == 0:
        _pump_fwd.off()
    else:
        _pump_fwd.on()
    drive(*get_joints())

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
MOVE_MS      = 15   # ms per step — lower = faster

f_speed = Frame(main)
f_speed.pack(fill="x", padx=20, pady=(4, 0))
Label(f_speed, text="Speed", width=8, anchor="w").pack(side=LEFT)
_speed_var = IntVar(value=MOVE_MS)
Label(f_speed, text="Fast", width=4).pack(side=LEFT)
Scale(f_speed, variable=_speed_var, from_=5, to=80,
      orient=HORIZONTAL, length=300, resolution=5,
      command=lambda v: _set_speed(int(v))
      ).pack(side=LEFT)

def _set_speed(v):
    global MOVE_MS
    MOVE_MS = v
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

# Load cell
Label(main, text=" Load Cell ", font=("Segoe UI", 11, "bold")).pack(pady=(14, 2))

use_median = False

for lbl_text, lbl_var in [("Sensor 1", None), ("Sensor 2", None), ("Sensor 3", None)]:
    Label(main, text=lbl_text, font=("Segoe UI", 9)).pack()

weight_lbl1 = Label(main, text="-- g", font=("Segoe UI", 24, "bold"))
weight_lbl2 = Label(main, text="-- g", font=("Segoe UI", 24, "bold"))
weight_lbl3 = Label(main, text="-- g", font=("Segoe UI", 24, "bold"))
weight_lbl1.pack()
weight_lbl2.pack()
weight_lbl3.pack()

mode_btn = Button(main, text="Mode: Live", width=16)
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

_sensors = [hx711.sensor1, hx711.sensor2, hx711.sensor3]

def open_calibration():
    win = Toplevel(root)
    win.title("Load Cell Calibration")
    win.resizable(False, False)

    for i, sensor in enumerate(_sensors):
        frame = LabelFrame(win, text=f"Sensor {i+1}", padx=10, pady=6)
        frame.pack(fill="x", padx=12, pady=6)

        reading_lbl = Label(frame, text="-- g", font=("Segoe UI", 13, "bold"), width=10)
        reading_lbl.pack(side=RIGHT)

        def refresh(lbl=reading_lbl, s=sensor):
            try:
                lbl.config(text=f"{s.read_grams():.1f} g")
            except Exception:
                lbl.config(text="err")
            win.after(300, lambda: refresh(lbl, s))

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
                lbl.config(text="OK")
            except Exception as ex:
                lbl.config(text=str(ex), fg="red")

        Button(frame, text="Calibrate", width=9, command=do_cal).pack(side=LEFT, padx=2)

Button(main, text="Tare & Calibrate", command=open_calibration).pack(pady=4)

FILL_THRESHOLD_G = 150   # start filling if glass is below this
FILL_TARGET_G    = 400   # stop filling when glass reaches this
PID_POLL_MS      = 200   # sensor read interval during fill (ms)

_fill_sensor_idx = 0

# Fill settings — editable from GUI
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
    try:
        sensors = [hx711.sensor1, hx711.sensor2, hx711.sensor3]
        grams   = sensors[_fill_sensor_idx].read_grams()

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
        desired_duty = int(power / 100 * 65535)
        kit._pca.channels[4].duty_cycle = 65535 - desired_duty
        pump_var.set(power)
        fill_status.config(text=f"Filling: {grams:.0f} / {FILL_TARGET_G:.0f} g  |  pump {power}%", fg="blue")

    except Exception as ex:
        fill_status.config(text=f"Sensor error: {ex}", fg="red")

    root.after(PID_POLL_MS, _pid_fill_loop)

def _stop_pump():
    global _fill_state
    kit._pca.channels[4].duty_cycle = 0
    _pump_fwd.off()
    pump_var.set(0)
    _fill_state = "idle"
    fill_status.config(text=f"Done — {FILL_TARGET_G:.0f} g reached", fg="green")
    root.after(1000, _main_loop)

def _main_loop():
    if not activate or _fill_state != "idle":
        root.after(500, _main_loop)
        return
    sensors = [hx711.sensor1, hx711.sensor2, hx711.sensor3]
    for idx, sensor in enumerate(sensors):
        try:
            grams = sensor.read_grams()
            if grams < 0:
                continue   # glass lifted from coaster — skip
            if grams < FILL_THRESHOLD_G:
                _start_fill(idx)
                return
        except Exception:
            pass
    root.after(500, _main_loop)


# Water level display
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

def _update_bars():
    for var, lbl, sensor in zip(_bar_vars, _bar_grams,
                                [hx711.sensor1, hx711.sensor2, hx711.sensor3]):
        try:
            g = sensor.read_grams()
            var.set(max(0, min(GLASS_MAX_G, int(g))))
            lbl.config(text=f"{g:.0f} g")
        except Exception:
            lbl.config(text="err")
    root.after(300, _update_bars)

# PID tuning
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

# Benchmark launcher
Button(main, text="Open Benchmark GUI",
       command=lambda: subprocess.Popen(["python", "GUI/GUI_benchmark.py"])
       ).pack(pady=10)

# Start
reset_all()
poll_hx711()
_update_bars()
_main_loop()
root.mainloop()

hx711.sensor1.close()
hx711.sensor2.close()
hx711.sensor3.close()
hx711.sck.close()
_pump_fwd.close()
