from gpiozero import Button
from signal import pause

# Initialize the button (connects to GPIO 2 and pulls up internally)
button = Button(2, pull_up=True)

# Function to run when pressed
def button_pressed():
    print("Button was pressed!")

# Function to run when released
def button_released():
    print("Button was released!")

# Assign functions to events
button.when_pressed = button_pressed
button.when_released = button_released

# Keep the script running
pause()
