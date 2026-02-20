from tkinter import *
from gpiozero import Servo

# ---------------- Servo setup (gpiozero only, NO pigpio) ----------------
# HD-6001HB: wider usable range using ~800–2200 µs
servo1 = Servo(18, min_pulse_width=0.8/1000, max_pulse_width=2.2/1000)  # Midje (GPIO18)
servo2 = Servo(19, min_pulse_width=0.8/1000, max_pulse_width=2.2/1000)  # Skulder (GPIO19)

# If you later add more servos:
# servo3 = Servo(<pin>, min_pulse_width=0.8/1000, max_pulse_width=2.2/1000)
# servo4 = Servo(<pin>, min_pulse_width=0.8/1000, max_pulse_width=2.2/1000)

# Slider angle mapping:
# HD-6001HB lists ~165° travel over 800–2200 µs, so half-range ≈ 82.5°
SERVO_MAX_DEG = 82.5

def angle_to_servo_value(angle_deg: int) -> float:
    v = angle_deg / SERVO_MAX_DEG
    if v > 1.0:
        v = 1.0
    if v < -1.0:
        v = -1.0
    return v

# ---------------- Tkinter setup ----------------
root = Tk()
root.title("Benchmark")
root.geometry("500x700")

CENTER_US = 1500
US_PER_DEG = 1000 / 90   # display only (kept from your code)

def gap():
    Label(root, text="").pack()

# ---------------- Midje ----------------
Label(root, text="Midje").pack()
midje_label = Label(root, text="Angle: 0° | Pulse: 1500 µs")
midje_label.pack()

def set_midje(value):
    angle = int(float(value))
    pulse = int(CENTER_US + angle * US_PER_DEG)

    servo1.value = angle_to_servo_value(angle)

    midje_label.config(text=f"Angle: {angle:+d}° | Pulse: {pulse} µs")
    print(f"[MIDJE] Angle={angle}, Pulse={pulse} µs, servo1.value={servo1.value:.2f}")

midje_scale = Scale(root, from_=-90, to=90, orient=HORIZONTAL, command=set_midje)
midje_scale.set(0)
midje_scale.pack(fill="x", padx=20)

gap()

# ---------------- Skulder ----------------
Label(root, text="Skulder").pack()
skulder_label = Label(root, text="Angle: 0° | Pulse: 1500 µs")
skulder_label.pack()

def set_skulder(value):
    angle = int(float(value))
    pulse = int(CENTER_US + angle * US_PER_DEG)

    servo2.value = angle_to_servo_value(angle)

    skulder_label.config(text=f"Angle: {angle:+d}° | Pulse: {pulse} µs")
    print(f"[SKULDER] Angle={angle}, Pulse={pulse} µs, servo2.value={servo2.value:.2f}")

skulder_scale = Scale(root, from_=-90, to=90, orient=HORIZONTAL, command=set_skulder)
skulder_scale.set(0)
skulder_scale.pack(fill="x", padx=20)

gap()

# ---------------- Albue (GUI only for now) ----------------
Label(root, text="Albue (GUI only)").pack()
albue_label = Label(root, text="Angle: 0° | Pulse: 1500 µs")
albue_label.pack()

def set_albue(value):
    angle = int(float(value))
    pulse = int(CENTER_US + angle * US_PER_DEG)
    albue_label.config(text=f"Angle: {angle:+d}° | Pulse: {pulse} µs")
    print(f"[ALBUE] Angle={angle}, Pulse={pulse} µs")

albue_scale = Scale(root, from_=-90, to=90, orient=HORIZONTAL, command=set_albue)
albue_scale.set(0)
albue_scale.pack(fill="x", padx=20)

gap()

# ---------------- Wrist (GUI only for now) ----------------
Label(root, text="Wrist (GUI only)").pack()
wrist_label = Label(root, text="Angle: 0° | Pulse: 1500 µs")
wrist_label.pack()

def set_wrist(value):
    angle = int(float(value))
    pulse = int(CENTER_US + angle * US_PER_DEG)
    wrist_label.config(text=f"Angle: {angle:+d}° | Pulse: {pulse} µs")
    print(f"[WRIST] Angle={angle}, Pulse={pulse} µs")

wrist_scale = Scale(root, from_=-90, to=90, orient=HORIZONTAL, command=set_wrist)
wrist_scale.set(0)
wrist_scale.pack(fill="x", padx=20)

gap()

# ---------------- Pump (GUI only, shows pulse) ----------------
PUMP_MIN_US = 500
PUMP_MAX_US = 2500

Label(root, text="Pump (GUI only)").pack()
pump_label = Label(root, text="Power: 0% | Pulse: 500 µs")
pump_label.pack()

def set_pump(value):
    power = int(float(value))  # 0..100
    pulse = int(PUMP_MIN_US + (power * (PUMP_MAX_US - PUMP_MIN_US) / 100))
    pump_label.config(text=f"Power: {power}% | Pulse: {pulse} µs")
    print(f"[PUMP] Power={power}%, Pulse={pulse} µs")

pump_scale = Scale(root, from_=0, to=100, orient=HORIZONTAL, command=set_pump)
pump_scale.set(0)
pump_scale.pack(fill="x", padx=20)

# ---------------- Reset ----------------
def reset_all():
    midje_scale.set(0); set_midje(0)
    skulder_scale.set(0); set_skulder(0)
    albue_scale.set(0); set_albue(0)
    wrist_scale.set(0); set_wrist(0)
    pump_scale.set(0); set_pump(0)

Button(root, text="Reset", command=reset_all).pack(pady=10)

#Weight
display = Text(
    root,
    height=1,
    width=10,
    font=("Segoe UI", 36, "bold"),
    bd=3,
    relief="sunken"
)
display.pack(padx=20, pady=20)

display.insert(END, "0.0")
display.config(state=DISABLED)

def set_value(value):
    display.config(state=NORMAL)
    display.delete("1.0", END)
    display.insert(END, value)
    display.config(state=DISABLED)

# Example buttons to prove it works
Button(root, text="Set 12.5", command=lambda: set_value("15")).pack()
Button(root, text="Tare", command=lambda: set_value("0.0")).pack()

# ==========================================================
# WEIGHT DISPLAY — AT THE END (after Reset)
# ==========================================================
Label(root, text="Weight").pack()

display = Text(
    root,
    height=1,
    width=10,
    font=("Segoe UI", 36, "bold"),
    bd=3,
    relief="sunken"
)
display.pack(padx=20, pady=10)

display.insert(END, "0.0 g")
display.config(state=DISABLED)

tare_offset = 0.0
current_weight = 0.0

def set_weight(value):
    global current_weight
    current_weight = value
    shown = current_weight - tare_offset

    display.config(state=NORMAL)
    display.delete("1.0", END)
    display.insert(END, f"{shown:.1f} g")
    display.config(state=DISABLED)

def tare():
    global tare_offset
    tare_offset = current_weight
    set_weight(current_weight)

Button(root, text="Set 25 g (test)", command=lambda: set_weight(25)).pack()
Button(root, text="Tare", command=tare).pack()

# Initialize outputs at center
reset_all()

root.mainloop()
