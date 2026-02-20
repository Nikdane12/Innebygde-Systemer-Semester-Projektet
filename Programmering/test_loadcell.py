import tkinter as tk
from tkinter import ttk
import random
import time

class ScaleGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Kitchen Scale")

        # --- Scale parameters ---
        self.offset = 0.0       # tare offset (in raw units)
        self.scale = 1.0        # raw -> grams factor (set later after calibration)

        # smoothing (optional but makes it look like a real scale)
        self.filtered = 0.0
        self.alpha = 0.2        # 0..1 higher = snappier, lower = smoother

        # --- UI ---
        self.value_var = tk.StringVar(value="0.0 g")
        self.status_var = tk.StringVar(value="Ready")

        big_font = ("Segoe UI", 48, "bold")
        small_font = ("Segoe UI", 12)

        frame = ttk.Frame(root, padding=16)
        frame.grid(sticky="nsew")

        self.value_label = ttk.Label(frame, textvariable=self.value_var, font=big_font)
        self.value_label.grid(row=0, column=0, columnspan=3, pady=(0, 8), sticky="ew")

        ttk.Button(frame, text="Tare", command=self.tare).grid(row=1, column=0, sticky="ew", padx=(0, 8))
        ttk.Button(frame, text="Zero", command=self.zero).grid(row=1, column=1, sticky="ew", padx=(0, 8))
        ttk.Button(frame, text="Quit", command=root.destroy).grid(row=1, column=2, sticky="ew")

        ttk.Label(frame, textvariable=self.status_var, font=small_font).grid(row=2, column=0, columnspan=3, pady=(10, 0), sticky="w")

        for c in range(3):
            frame.columnconfigure(c, weight=1)

        # start updating
        self.update_loop()

    # ---- Replace this with HX711 reading later ----
    def read_sensor(self) -> float:
        """
        Return a 'raw' sensor value.
        Replace this with your HX711 raw reading (average of N samples).
        """
        # Simulate drift + noise + occasional load
        base = 80000 + 200 * (time.time() % 10)  # slow drift
        load = 0
        if int(time.time()) % 7 < 3:             # pretend something is on scale sometimes
            load = 12000
        noise = random.uniform(-200, 200)
        return base + load + noise

    def tare(self):
        raw = self.read_sensor()
        self.offset = raw
        self.status_var.set("Tared (offset set).")

    def zero(self):
        self.offset = 0.0
        self.status_var.set("Offset cleared (not tared).")

    def update_loop(self):
        raw = self.read_sensor()
        net_raw = raw - self.offset
        grams = net_raw * self.scale

        # simple smoothing
        self.filtered = (1 - self.alpha) * self.filtered + self.alpha * grams

        self.value_var.set(f"{self.filtered:0.1f} g")
        self.root.after(50, self.update_loop)  # update every 50ms (20 Hz)

if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("420x220")
    ScaleGUI(root)
    root.mainloop()
