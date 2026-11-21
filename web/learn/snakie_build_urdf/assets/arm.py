import instruments as inst
import time

waist = inst.servo_on(0)
shoulder = inst.servo_on(1)
elbow = inst.servo_on(2)


def move(w, s, e):
    waist.angle(w)
    shoulder.angle(s)
    elbow.angle(e)


move(90, 90, 90)   # rest: every servo at its middle
time.sleep(1)

for wave in range(3):
    move(50, 140, 130)
    time.sleep(0.6)
    move(50, 60, 10)
    time.sleep(0.6)

move(50, 140, 130)
print("done")
