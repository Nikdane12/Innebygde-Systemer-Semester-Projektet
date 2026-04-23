import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from tkinter import *
from tkinter import simpledialog
import hx711 as hx
import i2c as i2c

# HX711 — using shared module
sensors = [hx.sensor1, hx.sensor2, hx.sensor3]

# Tkinter setup
root = Tk()
root.title("Benchmark")
root.geometry("500x750")

def gap():
    Label(root, text="").pack()

# Midje
Label(root, text="Midje").pack()
midje_label = Label(root, text="Angle: 0° | Pulse: 1500 µs")
midje_label.pack()

def set_midje(value):
    angle = int(float(value))
    pulse = i2c.midje_to_us(angle)
    i2c.set_pwm(i2c.CH_MIDJE, pulse)
    midje_label.config(text=f"Angle: {angle:+d}° | Pulse: {pulse} µs")
    print(f"[MIDJE] Angle={angle}, Pulse={pulse} µs")

midje_scale = Scale(root, from_=-90, to=90, orient=HORIZONTAL, command=set_midje)
midje_scale.set(0)
midje_scale.pack(fill="x", padx=20)
gap()

# Skulder
Label(root, text="Skulder").pack()
skulder_label = Label(root, text="Angle: 0° | Pulse: 1500 µs")
skulder_label.pack()

def set_skulder(value):
    angle = int(float(value))
    pulse = i2c.angle_to_us(angle)
    i2c.set_pwm(i2c.CH_SKULDER, pulse)
    skulder_label.config(text=f"Angle: {angle:+d}° | Pulse: {pulse} µs")
    print(f"[SKULDER] Angle={angle}, Pulse={pulse} µs")

skulder_scale = Scale(root, from_=-45, to=45, orient=HORIZONTAL, command=set_skulder)
skulder_scale.set(0)
skulder_scale.pack(fill="x", padx=20)
gap()

# Albue
Label(root, text="Albue").pack()
albue_label = Label(root, text="Angle: 0° | Pulse: 1500 µs")
albue_label.pack()

def set_albue(value):
    angle = int(float(value))
    pulse = i2c.angle_to_us(angle)
    i2c.set_pwm(i2c.CH_ALBUE, pulse)
    albue_label.config(text=f"Angle: {angle:+d}° | Pulse: {pulse} µs")
    print(f"[ALBUE] Angle={angle}, Pulse={pulse} µs")

albue_scale = Scale(root, from_=-45, to=45, orient=HORIZONTAL, command=set_albue)
albue_scale.set(0)
albue_scale.pack(fill="x", padx=20)
gap()

# Wrist
Label(root, text="Wrist").pack()
wrist_label = Label(root, text="Angle: 0° | Pulse: 1500 µs")
wrist_label.pack()

def set_wrist(value):
    angle = int(float(value))
    pulse = i2c.angle_to_us(angle)
    i2c.set_pwm(i2c.CH_WRIST, pulse)
    wrist_label.config(text=f"Angle: {angle:+d}° | Pulse: {pulse} µs")
    print(f"[WRIST] Angle={angle}, Pulse={pulse} µs")

wrist_scale = Scale(root, from_=-45, to=45, orient=HORIZONTAL, command=set_wrist)
wrist_scale.set(0)
wrist_scale.pack(fill="x", padx=20)
gap()

# Pump
Label(root, text="Pump").pack()
pump_label = Label(root, text="Power: 0% | Pulse: 500 µs")
pump_label.pack()

def set_pump(value):
    power = int(float(value))
    pulse = i2c.pump_to_us(power)
    i2c.set_pwm(i2c.CH_PUMP, pulse)
    pump_label.config(text=f"Power: {power}% | Pulse: {pulse} µs")
    print(f"[PUMP] Power={power}%, Pulse={pulse} µs")

pump_scale = Scale(root, from_=0, to=100, orient=HORIZONTAL, command=set_pump)
pump_scale.set(0)
pump_scale.pack(fill="x", padx=20)

# Reset
def reset_all():
    midje_scale.set(0);   set_midje(0)
    skulder_scale.set(0); set_skulder(0)
    albue_scale.set(0);   set_albue(0)
    wrist_scale.set(0);   set_wrist(0)
    pump_scale.set(0);    set_pump(0)

Button(root, text="Reset", command=reset_all).pack(pady=10)

# Weight display
Label(root, text="Weight").pack()
display = Text(root, height=1, width=10, font=("Segoe UI", 36, "bold"),
               bd=3, relief="sunken")
display.pack(padx=20, pady=10)
display.insert(END, "-- g")
display.config(state=DISABLED)

def update_display(grams: float):
    display.config(state=NORMAL)
    display.delete("1.0", END)
    display.insert(END, f"{grams:.1f} g")
    display.config(state=DISABLED)

def do_tare():
    for s in sensors:
        s.tare()
    print("[TARE] all sensors")

def do_calibrate():
    known = simpledialog.askfloat("Calibrate", "Weight of object on scale (grams):", minvalue=1, parent=root)
    if known is None:
        return
    for s in sensors:
        s.calibrate(known)
    print(f"[CALIBRATE] known={known}g")

Button(root, text="Tare",      command=do_tare).pack()
Button(root, text="Calibrate", command=do_calibrate).pack()

use_median = False

mode_btn = Button(root, text="Mode: Live", width=14)
mode_btn.pack(pady=4)

def toggle_mode():
    global use_median
    use_median = not use_median
    mode_btn.config(text="Mode: Median" if use_median else "Mode: Live")

mode_btn.config(command=toggle_mode)

def poll_hx711():
    try:
        if use_median:
            grams = (sensors[0].read_median() - sensors[0].hx_offset) / sensors[0].scale_factor
        else:
            grams = sensors[0].read_grams()
        update_display(grams)
    except Exception as e:
        print(f"[HX711] {e}")
    interval = 500 if use_median else 200
    root.after(interval, poll_hx711)

reset_all()
poll_hx711()
root.mainloop()

for s in sensors:
    s.close()
hx.sck.close()
i2c.bus.close()
