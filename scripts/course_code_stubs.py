# Strict stand-ins for the Raspberry Pi Pico's MicroPython modules.
#
# This file is MicroPython, not CPython. check_course_code.py prepends it to a
# runner and executes a lesson's code block on the MicroPython unix port, so
# the core language (f-strings, print(), time.localtime() returning a tuple,
# missing CPython modules) behaves exactly as it does on the Pico.
#
# The classes below only define what the rp2 port really has (MicroPython
# 1.2x docs: docs.micropython.org/en/latest/library/machine.html and the rp2
# quick reference). Calling anything else raises AttributeError, just like the
# board would. Sensors return plausible readings, and sleep() calls are
# counted so `while True:` loops stop after a few passes.
import sys
import io
import json

_real_time = __import__("time")
_MAX_SLEEPS = 25
_state = {"sleeps": 0, "clock_ms": 0}


class _StopLesson(BaseException):
    pass


def _tick(ms):
    _state["clock_ms"] += int(ms)
    _state["sleeps"] += 1
    if _state["sleeps"] > _MAX_SLEEPS:
        raise _StopLesson()


class _Module:
    def __init__(self, name):
        self.__name__ = name


def _localtime(secs=None):
    t = _real_time.localtime(secs) if secs is not None else _real_time.localtime()
    return tuple(t[:8])  # rp2 returns 8 items: no DST flag


time = _Module("time")
time.sleep = lambda s: _tick(s * 1000)
time.sleep_ms = lambda ms: _tick(ms)
time.sleep_us = lambda us: _tick(us / 1000)
time.ticks_ms = lambda: _state["clock_ms"] & 0x3FFFFFFF
time.ticks_us = lambda: (_state["clock_ms"] * 1000) & 0x3FFFFFFF
time.ticks_cpu = lambda: (_state["clock_ms"] * 125000) & 0x3FFFFFFF
time.ticks_add = lambda t, d: (t + d) & 0x3FFFFFFF
time.ticks_diff = lambda a, b: ((a - b + 0x20000000) & 0x3FFFFFFF) - 0x20000000
time.time = lambda: _real_time.time() + _state["clock_ms"] // 1000
time.time_ns = lambda: (_real_time.time() * 1000 + _state["clock_ms"]) * 1000000
time.localtime = _localtime
time.gmtime = _localtime
time.mktime = _real_time.mktime


def _call_once(cb, arg):
    if cb is not None:
        cb(arg)


class Pin:
    IN = 0
    OUT = 1
    OPEN_DRAIN = 2
    ALT = 3
    PULL_UP = 1
    PULL_DOWN = 2
    IRQ_FALLING = 4
    IRQ_RISING = 8
    IRQ_LOW_LEVEL = 1
    IRQ_HIGH_LEVEL = 2

    def __init__(self, id, mode=-1, pull=-1, *, value=None, alt=-1):
        self._id = id
        self._v = value or 0

    def init(self, mode=-1, pull=-1, *, value=None, alt=-1):
        if value is not None:
            self._v = value

    def value(self, x=None):
        if x is None:
            return self._v
        self._v = 1 if x else 0

    def __call__(self, x=None):
        return self.value(x)

    def on(self):
        self._v = 1

    def off(self):
        self._v = 0

    def high(self):
        self._v = 1

    def low(self):
        self._v = 0

    def toggle(self):
        self._v = 1 - self._v

    def irq(self, handler=None, trigger=12, *, hard=False):
        _call_once(handler, self)


class Signal:
    def __init__(self, pin, *args, invert=False, **kw):
        self._pin = pin if isinstance(pin, Pin) else Pin(pin)
        self._inv = invert

    def value(self, x=None):
        if x is None:
            return self._pin.value() ^ self._inv
        self._pin.value(bool(x) ^ self._inv)

    def on(self):
        self.value(1)

    def off(self):
        self.value(0)


class ADC:
    CORE_TEMP = 4

    def __init__(self, pin, *, atten=None):
        self._id = pin

    def read_u16(self):
        return 14300  # ~0.72 V: about 20 degrees C on the core sensor


class PWM:
    def __init__(self, dest, *, freq=None, duty_u16=None, duty_ns=None, invert=False):
        self._f = freq or 1000
        self._d = duty_u16 or 0

    def init(self, *, freq=None, duty_u16=None, duty_ns=None, invert=False):
        pass

    def deinit(self):
        pass

    def freq(self, value=None):
        if value is None:
            return self._f
        self._f = value

    def duty_u16(self, value=None):
        if value is None:
            return self._d
        if not 0 <= value <= 65535:
            raise ValueError("duty_u16 must be 0 to 65535")
        self._d = value

    def duty_ns(self, value=None):
        return 0 if value is None else None


class _I2CBase:
    def __init__(self, id=0, *, scl=None, sda=None, freq=400000, timeout=50000):
        pass

    def init(self, *a, **kw):
        pass

    def deinit(self):
        pass

    def scan(self):
        return []

    def start(self):
        pass

    def stop(self):
        pass

    def readinto(self, buf, nack=True):
        pass

    def write(self, buf):
        return len(buf)

    def readfrom(self, addr, nbytes, stop=True):
        return bytes(nbytes)

    def readfrom_into(self, addr, buf, stop=True):
        pass

    def writeto(self, addr, buf, stop=True):
        return len(buf)

    def writevto(self, addr, vector, stop=True):
        return len(vector)

    def readfrom_mem(self, addr, memaddr, nbytes, *, addrsize=8):
        return bytes(nbytes)

    def readfrom_mem_into(self, addr, memaddr, buf, *, addrsize=8):
        pass

    def writeto_mem(self, addr, memaddr, buf, *, addrsize=8):
        pass


class I2C(_I2CBase):
    pass


class SoftI2C(_I2CBase):
    pass


class _SPIBase:
    MSB = 0
    LSB = 1

    def __init__(self, id=0, *a, **kw):
        pass

    def init(self, *a, **kw):
        pass

    def deinit(self):
        pass

    def read(self, nbytes, write=0):
        return bytes(nbytes)

    def readinto(self, buf, write=0):
        pass

    def write(self, buf):
        pass

    def write_readinto(self, write_buf, read_buf):
        pass


class SPI(_SPIBase):
    pass


class SoftSPI(_SPIBase):
    pass


class UART:
    INV_TX = 1
    INV_RX = 2
    CTS = 1
    RTS = 2

    def __init__(self, id, baudrate=115200, *a, **kw):
        pass

    def init(self, *a, **kw):
        pass

    def deinit(self):
        pass

    def any(self):
        return 0

    def read(self, nbytes=None):
        return None

    def readinto(self, buf, nbytes=None):
        return None

    def readline(self):
        return None

    def write(self, buf):
        return len(buf)

    def sendbreak(self):
        pass

    def flush(self):
        pass

    def txdone(self):
        return True

    def irq(self, *a, **kw):
        pass


class Timer:
    ONE_SHOT = 0
    PERIODIC = 1

    def __init__(self, id=-1, *, mode=1, freq=-1, period=-1, callback=None):
        _call_once(callback, self)

    def init(self, *, mode=1, freq=-1, period=-1, callback=None):
        _call_once(callback, self)

    def deinit(self):
        pass


class RTC:
    def __init__(self, id=0):
        pass

    def datetime(self, dt=None):
        if dt is None:
            t = _localtime()
            # (year, month, day, weekday, hours, minutes, seconds, subseconds)
            return (t[0], t[1], t[2], t[6], t[3], t[4], t[5], 0)


class WDT:
    def __init__(self, id=0, timeout=5000):
        pass

    def feed(self):
        pass


machine = _Module("machine")
for _n, _v in (("Pin", Pin), ("Signal", Signal), ("ADC", ADC), ("PWM", PWM),
               ("I2C", I2C), ("SoftI2C", SoftI2C), ("SPI", SPI), ("SoftSPI", SoftSPI),
               ("UART", UART), ("Timer", Timer), ("RTC", RTC), ("WDT", WDT)):
    setattr(machine, _n, _v)
machine.PWRON_RESET = 1
machine.WDT_RESET = 3
machine.freq = lambda hz=None: 125000000 if hz is None else None
machine.reset = lambda: (_ for _ in ()).throw(_StopLesson())
machine.soft_reset = machine.reset
machine.reset_cause = lambda: 1
machine.bootloader = machine.reset
machine.idle = lambda: None
machine.lightsleep = lambda ms=0: _tick(ms)
machine.deepsleep = lambda ms=0: (_ for _ in ()).throw(_StopLesson())
machine.unique_id = lambda: b"\xe6\x61\x38\x52\x13\x4a\x2c\x2f"
machine.time_pulse_us = lambda pin, level, timeout_us=1000000: -1
machine.bitstream = lambda pin, encoding, timing, data: None
machine.disable_irq = lambda: 0
machine.enable_irq = lambda state=0: None


class _StateMachine:
    def __init__(self, id, program=None, *a, **kw):
        pass

    def init(self, *a, **kw):
        pass

    def active(self, value=None):
        return 0 if value is None else None

    def restart(self):
        pass

    def exec(self, instr):
        pass

    def get(self, buf=None, shift=0):
        return 0

    def put(self, value, shift=0):
        pass

    def rx_fifo(self):
        return 0

    def tx_fifo(self):
        return 0

    def irq(self, *a, **kw):
        pass


class _PIO:
    IN_LOW = 0
    IN_HIGH = 1
    OUT_LOW = 2
    OUT_HIGH = 3
    SHIFT_LEFT = 0
    SHIFT_RIGHT = 1
    JOIN_NONE = 0
    JOIN_TX = 1
    JOIN_RX = 2
    IRQ_SM0 = 256

    def __init__(self, id):
        pass

    def add_program(self, prog):
        pass

    def remove_program(self, prog=None):
        pass

    def state_machine(self, id, *a, **kw):
        return _StateMachine(id)


rp2 = _Module("rp2")
rp2.PIO = _PIO
rp2.StateMachine = _StateMachine
rp2.asm_pio = lambda *a, **kw: (lambda f: f)
rp2.bootsel_button = lambda: 0
rp2.country = lambda c=None: "GB"


class NeoPixel:
    ORDER = (1, 0, 2, 3)

    def __init__(self, pin, n, *, bpp=3, timing=1):
        self.n = n
        self._px = [(0,) * bpp for _ in range(n)]

    def __len__(self):
        return self.n

    def __setitem__(self, i, v):
        self._px[i] = v

    def __getitem__(self, i):
        return self._px[i]

    def fill(self, v):
        for i in range(self.n):
            self._px[i] = v

    def write(self):
        pass


neopixel = _Module("neopixel")
neopixel.NeoPixel = NeoPixel


class _DHTBase:
    def __init__(self, pin):
        pass

    def measure(self):
        pass

    def temperature(self):
        return 21

    def humidity(self):
        return 45


dht = _Module("dht")
dht.DHT11 = type("DHT11", (_DHTBase,), {})
dht.DHT22 = type("DHT22", (_DHTBase,), {})


def _install():
    for name, mod in (("time", time), ("utime", time), ("machine", machine),
                      ("rp2", rp2), ("neopixel", neopixel), ("dht", dht)):
        sys.modules[name] = mod
    import builtins
    builtins.input = lambda prompt="": (print(prompt, end="") or "1")


def _report(status, payload):
    print("@@CHECK " + status + " " + json.dumps(payload))


def _run_lesson_block(path):
    with open(path) as f:
        src = f.read()
    _install()
    g = {"__name__": "__main__"}
    try:
        exec(compile(src, "lesson_block.py", "exec"), g)
    except _StopLesson:
        _report("stopped", {})
        return
    except SystemExit:
        _report("ok", {})
        return
    except KeyboardInterrupt:
        _report("stopped", {})
        return
    except BaseException as e:
        buf = io.StringIO()
        sys.print_exception(e, buf)
        tb = buf.getvalue()
        line = 1
        marker = 'File "lesson_block.py", line '
        for row in tb.split("\n"):
            if marker in row:
                line = int(row.split(marker)[1].split(",")[0])
        _report("error", {"type": type(e).__name__, "msg": str(e), "line": line})
        return
    _report("ok", {})
