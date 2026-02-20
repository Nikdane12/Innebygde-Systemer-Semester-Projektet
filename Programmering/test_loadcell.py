from tkinter import *

root = Tk()
root.title("Value Display")

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

root.mainloop()