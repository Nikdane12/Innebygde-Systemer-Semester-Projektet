#L298N motordriver – GUI
#  IN1  → GPIO 17  (fremover)
#  IN2  → GPIO 27  (bakover)
#  ENA  → GPIO 19  (PWM hastighet)


import tkinter as tk
from gpiozero import Motor

motor = Motor(forward=17, backward=27, enable=19, pwm=True)


def set_speed(val):
    speed = float(val) / 100
    if speed == 0:
        motor.stop()
        lbl_status.config(text="Status: Stoppet", fg="red")
    else:
        motor.forward(speed)
        lbl_status.config(text=f"Status: Kjører ({int(val)}%)", fg="green")


def stop():
    slider.set(0)
    motor.stop()
    lbl_status.config(text="Status: Stoppet", fg="red")


root = tk.Tk()
root.title("Pumpekontroll")
root.geometry("340x200")
root.resizable(False, False)

tk.Label(root, text="Pumpehastighet", font=("Helvetica", 14, "bold")).pack(pady=10)

slider = tk.Scale(
    root,
    from_=0, to=100,
    orient=tk.HORIZONTAL,
    length=280,
    label="Effekt (%)",
    command=set_speed,
)
slider.set(0)
slider.pack()

tk.Button(root, text="STOPP", bg="red", fg="white", width=12,
          command=stop).pack(pady=8)

lbl_status = tk.Label(root, text="Status: Stoppet", fg="red")
lbl_status.pack()

root.mainloop()
motor.stop()
