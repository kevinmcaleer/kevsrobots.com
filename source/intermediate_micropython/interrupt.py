# Create an Interrupt Service Routine (ISR) that flashes the LED

from machine import Pin, Timer
from time import sleep

# Define the LED pin
led = Pin(Pin("LED"), Pin.OUT)

# Define the timer callback function
def toggle_led(timer):
    led.value(not led.value())
    print(f"led value is: {led.value()}")

# Initialize the timer
timer = Timer(-1)  # Use virtual timer
timer.init(period=1000, mode=Timer.PERIODIC, callback=toggle_led)