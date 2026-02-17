from tkinter import *

root = Tk()
root.title("Servo Control")
root.geometry("500x500")

CENTER_US = 1500
US_PER_DEG = 1000 / 90   # ±90° → ±1000 µs

gap_label = Label(root)
gap_label.pack()

#Midje
Label(root, text="Midje").pack()

midje_label = Label(root, text="Angle: 0° | Pulse: 1500 µs")
midje_label.pack()

def set_midje(value):
    angle = int(value)
    pulse = int(CENTER_US + angle * US_PER_DEG)
    midje_label.config(text=f"Angle: {angle}° | Pulse: {pulse} µs")
    print(f"[MIDJE] Angle={angle}, Pulse={pulse}")

midje_scale = Scale(root, from_=-90, to=90, orient=HORIZONTAL, command=set_midje)
midje_scale.set(0)
midje_scale.pack(fill="x", padx=20)

gap_label = Label(root)
gap_label.pack()

#Skulder
Label(root, text="Skulder").pack()

skulder_label = Label(root, text="Angle: 0° | Pulse: 1500 µs")
skulder_label.pack()

def set_skulder(value):
    angle = int(value)
    pulse = int(CENTER_US + angle * US_PER_DEG)
    skulder_label.config(text=f"Angle: {angle}° | Pulse: {pulse} µs")
    print(f"[SKULDER] Angle={angle}, Pulse={pulse}")

skulder_scale = Scale(root, from_=-90, to=90, orient=HORIZONTAL, command=set_skulder)
skulder_scale.set(0)
skulder_scale.pack(fill="x", padx=20)

gap_label = Label(root)
gap_label.pack()

#Albue
Label(root, text="Albue").pack()

albue_label = Label(root, text="Angle: 0° | Pulse: 1500 µs")
albue_label.pack()

def set_albue(value):
    angle = int(value)
    pulse = int(CENTER_US + angle * US_PER_DEG)
    albue_label.config(text=f"Angle: {angle}° | Pulse: {pulse} µs")
    print(f"[Albue] Angle={angle}, Pulse={pulse}")

albue_scale = Scale(root, from_=-90, to=90, orient=HORIZONTAL, command=set_albue)
albue_scale.set(0)
albue_scale.pack(fill="x", padx=20)

#Reset
def reset_all():
    midje_scale.set(0)
    skulder_scale.set(0)
    albue_scale.set(0)
    # update label + print

reset_btn = Button(root, text="Reset", command=reset_all)
reset_btn.pack(pady=10)

def on_close():
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()
