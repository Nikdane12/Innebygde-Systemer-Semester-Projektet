from tkinter import * 
from tkinter import messagebox
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, 
NavigationToolbar2Tk)


# create root window
root = Tk()

# root window title and dimension
root.title("Welcome to GeekForGeeks")
# Set geometry(widthxheight)
root.geometry('350x200')

# adding a label to the root window
lbl = Label(root, text = "Are you a Geek?")
lbl.grid()

# function to display text when
# button is clicked
def clicked():
    messagebox.showinfo("showinfo", "Information") 
    print(f"Hello world")

# button widget with red color text
# inside
btn = Button(root, text = "Click me" ,
             fg = "red", command=clicked)
# set Button grid
btn.grid(column=1, row=0)

# Execute Tkinter
root.mainloop()