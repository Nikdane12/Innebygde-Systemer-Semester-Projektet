from tkinter import *
import smbus2
import time

# PCA9685 setup
# Pins: SDA = GPIO 4 (pin 7), SCL = GPIO 3 (pin 5)
# Requires in /boot/firmware/config.txt:
#   dtoverlay=i2c-gpio,bus=4,i2c_gpio_sda=4,i2c_gpio_scl=3

I2C_BUS  = 4
PCA_ADDR = 0x40

bus = smbus2.SMBus(I2C_BUS)

# Sleep -> set prescaler (50 Hz) -> wake up
bus.write_byte_data(PCA_ADDR, 0x00, 0x10)
bus.write_byte_data(PCA_ADDR, 0xFE, 0x79)   # 0x79 = 121 ≈ 50 Hz
bus.write_byte_data(PCA_ADDR, 0x00, 0x00)
time.sleep(0.01)
print("PCA9685 ready, MODE1 =", hex(bus.read_byte_data(PCA_ADDR, 0x00)))

# PCA9685 channel assignments
CH_MIDJE   = 0
CH_SKULDER = 1
CH_ALBUE   = 2
CH_WRIST   = 3
CH_PUMP    = 4

def set_pwm(channel, pulse_us):
    ticks = round(pulse_us / 20000 * 4096)
    off_l = ticks & 0xFF
    off_h = (ticks >> 8) & 0x0F
    reg   = 0x06 + channel * 4
    bus.write_byte_data(PCA_ADDR, reg + 0, 0x00)
    bus.write_byte_data(PCA_ADDR, reg + 1, 0x00)
    bus.write_byte_data(PCA_ADDR, reg + 2, off_l)
    bus.write_byte_data(PCA_ADDR, reg + 3, off_h)

# Servo config
CENTER_US    = 1500
US_PER_DEG   = 1000 / 90
SERVO_MIN_US = 800
SERVO_MAX_US = 2200

PUMP_MIN_US  = 500
PUMP_MAX_US  = 2500

def angle_to_us(angle_deg):
    pulse = CENTER_US + angle_deg * US_PER_DEG
    return max(SERVO_MIN_US, min(SERVO_MAX_US, int(pulse)))

# Tkinter setup
root = Tk()
root.title("Benchmark")
root.geometry("500x700")

def gap():
    Label(root, text="").pack()

# Midje
Label(root, text="Midje").pack()
midje_label = Label(root, text="Angle: 0° | Pulse: 1500 µs")
midje_label.pack()

def set_midje(value):
    angle = int(float(value))
    pulse = angle_to_us(angle)
    set_pwm(CH_MIDJE, pulse)
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
    pulse = angle_to_us(angle)
    set_pwm(CH_SKULDER, pulse)
    skulder_label.config(text=f"Angle: {angle:+d}° | Pulse: {pulse} µs")
    print(f"[SKULDER] Angle={angle}, Pulse={pulse} µs")

skulder_scale = Scale(root, from_=-90, to=90, orient=HORIZONTAL, command=set_skulder)
skulder_scale.set(0)
skulder_scale.pack(fill="x", padx=20)
gap()

# Albue 
Label(root, text="Albue").pack()
albue_label = Label(root, text="Angle: 0° | Pulse: 1500 µs")
albue_label.pack()

def set_albue(value):
    angle = int(float(value))
    pulse = angle_to_us(angle)
    set_pwm(CH_ALBUE, pulse)
    albue_label.config(text=f"Angle: {angle:+d}° | Pulse: {pulse} µs")
    print(f"[ALBUE] Angle={angle}, Pulse={pulse} µs")

albue_scale = Scale(root, from_=-90, to=90, orient=HORIZONTAL, command=set_albue)
albue_scale.set(0)
albue_scale.pack(fill="x", padx=20)
gap()

# Wrist 
Label(root, text="Wrist").pack()
wrist_label = Label(root, text="Angle: 0° | Pulse: 1500 µs")
wrist_label.pack()

def set_wrist(value):
    angle = int(float(value))
    pulse = angle_to_us(angle)
    set_pwm(CH_WRIST, pulse)
    wrist_label.config(text=f"Angle: {angle:+d}° | Pulse: {pulse} µs")
    print(f"[WRIST] Angle={angle}, Pulse={pulse} µs")

wrist_scale = Scale(root, from_=-90, to=90, orient=HORIZONTAL, command=set_wrist)
wrist_scale.set(0)
wrist_scale.pack(fill="x", padx=20)
gap()

# Pump
Label(root, text="Pump").pack()
pump_label = Label(root, text="Power: 0% | Pulse: 500 µs")
pump_label.pack()

def set_pump(value):
    power = int(float(value))
    pulse = int(PUMP_MIN_US + (power * (PUMP_MAX_US - PUMP_MIN_US) / 100))
    set_pwm(CH_PUMP, pulse)
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
display.insert(END, "0.0 g")
display.config(state=DISABLED)

tare_offset    = 0.0
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

reset_all()
root.mainloop()

bus.close()
