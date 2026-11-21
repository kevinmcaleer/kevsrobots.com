---
title: x
---
```python
import machine, time
adc = machine.ADC(26)
print(adc.read())
```

```python
from machine import Pin, PWM
p = PWM(Pin(15))
p.duty(512)
```

```python
import time
print(time.strftime("%H"))
```

```python
import datetime
```

```python
print("hi", flush=True)
```

```python
from machine import Pin
led = Pin("LED", Pin.OUT)
while True:
    led.toggle()
    time.sleep(0.5)
```
{: .no-snakie }

<!-- code-check: skip -->
```python
this is not python
```

```python
from machine import Pin
import time
led = Pin(25, Pin.OUT)
while True:
    led.toggle()
    time.sleep_ms(200)
```
