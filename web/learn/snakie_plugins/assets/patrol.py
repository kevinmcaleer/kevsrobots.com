from machine import Pin
import time
import clubbot

led = Pin(25, Pin.OUT)
buzzer = Pin(15, Pin.OUT)
sense = Pin(29, Pin.IN)

for lap in range(4):
    clubbot.forward(30)
    clubbot.turn(90)
    led.toggle()
    time.sleep(500)

clubbot.stop()
