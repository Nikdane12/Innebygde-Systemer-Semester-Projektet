#!/usr/bin/env python3
"""
Op9 – IO-kort GUI
Styrer LEDs, servo, summer og leser ADC/temperatur via serieport.
"""

import tkinter as tk
from tkinter import ttk
import serial
import threading
import time

#Serieport
PORT  = '/dev/ttyAMA0'
BAUD  = 38400

ser = None

def connect_serial():
    global ser
    try:
        ser = serial.Serial(PORT, BAUD, timeout=2)
        time.sleep(0.5)
        log(f"Tilkoblet {PORT} @ {BAUD}")
    except Exception as e:
        log(f"[FEIL] Kan ikke åpne {PORT}: {e}")

def send_command(cmd: str) -> str:
    """Send en kommando og returner svaret."""
    if ser is None or not ser.is_open:
        log(f"[IKKE TILKOBLET] {cmd}")
        return ""
    try:
        ser.write((cmd + '\n').encode())
        response = ser.readline().decode().strip()
        log(f">>> {cmd}   <<< {response}")
        return response
    except Exception as e:
        log(f"[FEIL] {e}")
        return ""


#LED
class LED:
    """Representerer en LED på IO-kortet med på/av-tilstand."""

    def __init__(self, index: int):
        self.index = index      # 0–3
        self._state = False     # False = av, True = på

    @property
    def state(self) -> bool:
        return self._state

    def turn_on(self):
        if not self._state:
            resp = send_command(f"LED:{self.index}")
            if resp == "OK":
                self._state = True

    def turn_off(self):
        if self._state:
            resp = send_command(f"LED:{self.index}")
            if resp == "OK":
                self._state = False

    def toggle(self):
        if self._state:
            self.turn_off()
        else:
            self.turn_on()


#Hjelper: tråd for polling
def poll_sensor(cmd: str, label: tk.Label, prefix: str):
    """Spør om sensor i bakgrunnstråd og oppdater label."""
    def task():
        resp = send_command(cmd)
        label.config(text=f"{prefix}{resp}")
    threading.Thread(target=task, daemon=True).start()


#Logg
_log_widget = None

def log(msg: str):
    if _log_widget is not None:
        _log_widget.config(state=tk.NORMAL)
        _log_widget.insert(tk.END, msg + '\n')
        _log_widget.see(tk.END)
        _log_widget.config(state=tk.DISABLED)
    else:
        print(msg)


#UI
def build_gui():
    global _log_widget

    root = tk.Tk()
    root.title("IO-kort styrepanel")
    root.resizable(False, False)

    style = ttk.Style(root)
    style.theme_use("clam")

    #Tittel
    tk.Label(root, text="IO-kort styrepanel", font=("Segoe UI", 14, "bold"),
             pady=8).grid(row=0, column=0, columnspan=2, sticky="ew")

    #LED-seksjon
    led_frame = ttk.LabelFrame(root, text="LEDs", padding=10)
    led_frame.grid(row=1, column=0, padx=10, pady=6, sticky="nsew")

    leds = [LED(i) for i in range(4)]
    led_buttons  = []
    led_circles  = []   # Canvas for visuell indikator

    LED_ON_COLOR  = "#00cc44"
    LED_OFF_COLOR = "#555555"
    CIRCLE_SIZE   = 20

    def refresh_led(i: int):
        color  = LED_ON_COLOR if leds[i].state else LED_OFF_COLOR
        status = "PÅ" if leds[i].state else "AV"
        led_buttons[i].config(text=f"LED {i}  [{status}]")
        led_circles[i].itemconfig("dot", fill=color)

    def make_toggle(i: int):
        def cb():
            leds[i].toggle()
            refresh_led(i)
        return cb

    for i in range(4):
        row_frame = tk.Frame(led_frame)
        row_frame.pack(anchor="w", pady=3)

        # Fargesirkel
        c = tk.Canvas(row_frame, width=CIRCLE_SIZE, height=CIRCLE_SIZE,
                      highlightthickness=0, bg=root.cget("bg"))
        c.create_oval(2, 2, CIRCLE_SIZE-2, CIRCLE_SIZE-2,
                      fill=LED_OFF_COLOR, tags="dot")
        c.pack(side=tk.LEFT, padx=(0, 6))
        led_circles.append(c)

        btn = tk.Button(row_frame, text=f"LED {i}  [AV]", width=18,
                        font=("Segoe UI", 10),
                        command=make_toggle(i))
        btn.pack(side=tk.LEFT)
        led_buttons.append(btn)

    # Alle på / alle av
    ctrl_row = tk.Frame(led_frame)
    ctrl_row.pack(pady=(8, 0))

    def all_on():
        def task():
            for i in range(4):
                leds[i].turn_on()
                root.after(0, refresh_led, i)
                time.sleep(0.1)
        threading.Thread(target=task, daemon=True).start()

    def all_off():
        def task():
            for i in range(4):
                leds[i].turn_off()
                root.after(0, refresh_led, i)
                time.sleep(0.1)
        threading.Thread(target=task, daemon=True).start()

    tk.Button(ctrl_row, text="Alle PÅ",  width=10, command=all_on).pack(side=tk.LEFT, padx=4)
    tk.Button(ctrl_row, text="Alle AV", width=10, command=all_off).pack(side=tk.LEFT, padx=4)
    tk.Button(ctrl_row, text="Synk GUI", width=10, command=sync_off,
              fg="orange").pack(side=tk.LEFT, padx=4)

    #Sensor-seksjon
    sens_frame = ttk.LabelFrame(root, text="Sensorer", padding=10)
    sens_frame.grid(row=2, column=0, padx=10, pady=6, sticky="nsew")

    adc_label = tk.Label(sens_frame, text="ADC: –", font=("Courier", 11),
                         anchor="w", width=28)
    adc_label.pack(anchor="w")
    tk.Button(sens_frame, text="Les ADC",
              command=lambda: poll_sensor("ADC", adc_label, "ADC: ")
              ).pack(anchor="w", pady=(2, 6))

    tmp_label = tk.Label(sens_frame, text="Temperatur: –", font=("Courier", 11),
                         anchor="w", width=28)
    tmp_label.pack(anchor="w")
    tk.Button(sens_frame, text="Les temperatur",
              command=lambda: poll_sensor("TMP", tmp_label, "Temperatur: ")
              ).pack(anchor="w", pady=2)

    #Servo-seksjon
    servo_frame = ttk.LabelFrame(root, text="Servo", padding=10)
    servo_frame.grid(row=1, column=1, padx=10, pady=6, sticky="nsew")

    servo_label = tk.Label(servo_frame, text="Vinkel: 90°", font=("Segoe UI", 10))
    servo_label.pack()

    def on_servo_change(val):
        angle = int(float(val))
        servo_label.config(text=f"Vinkel: {angle}°")

    servo_scale = tk.Scale(servo_frame, from_=0, to=180,
                           orient=tk.HORIZONTAL, length=200,
                           command=on_servo_change)
    servo_scale.set(90)
    servo_scale.pack()

    def send_servo():
        angle = servo_scale.get()
        send_command(f"SERVO:{angle}")

    tk.Button(servo_frame, text="Send servo", command=send_servo,
              width=14).pack(pady=6)

    #Buzzer-seksjon
    buzz_frame = ttk.LabelFrame(root, text="Summer", padding=10)
    buzz_frame.grid(row=2, column=1, padx=10, pady=6, sticky="nsew")

    buzz_label = tk.Label(buzz_frame, text="Frekvens: 500 Hz", font=("Segoe UI", 10))
    buzz_label.pack()

    def on_buzz_change(val):
        freq = int(float(val))
        buzz_label.config(text=f"Frekvens: {freq} Hz")

    buzz_scale = tk.Scale(buzz_frame, from_=0, to=4000,
                          orient=tk.HORIZONTAL, length=200,
                          command=on_buzz_change)
    buzz_scale.set(500)
    buzz_scale.pack()

    btn_row = tk.Frame(buzz_frame)
    btn_row.pack(pady=4)

    def send_buzz():
        freq = buzz_scale.get()
        send_command(f"BUZZ:{freq}")

    def stop_buzz():
        buzz_scale.set(0)
        send_command("BUZZ:0")

    tk.Button(btn_row, text="Spill av", command=send_buzz,  width=10).pack(side=tk.LEFT, padx=3)
    tk.Button(btn_row, text="Stopp",    command=stop_buzz,  width=10).pack(side=tk.LEFT, padx=3)

    #Logg
    log_frame = ttk.LabelFrame(root, text="Kommunikasjonslogg", padding=6)
    log_frame.grid(row=3, column=0, columnspan=2, padx=10, pady=6, sticky="nsew")

    _log_widget = tk.Text(log_frame, height=8, state=tk.DISABLED,
                          font=("Courier", 9), bg="#111", fg="#0f0")
    sb = ttk.Scrollbar(log_frame, command=_log_widget.yview)
    _log_widget.config(yscrollcommand=sb.set)
    _log_widget.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    sb.pack(side=tk.RIGHT, fill=tk.Y)

    root.mainloop()


#Hovedprogram
if __name__ == "__main__":
    connect_serial()
    build_gui()
    if ser and ser.is_open:
        ser.close()
