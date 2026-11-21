"""Strict stand-in for arduino.app_utils: Bridge, App and the decorators.

Bridge.call() checks the name against what the lesson's sketch provides
(passed in $UNO_Q_PROVIDED as {name: example return value}) and returns a
value of the type that sketch function returns. App.run() calls the user loop
a few times and stops.
"""
import json
import os
import time as _time

__all__ = ["App", "Bridge", "Leds", "Logger", "call", "notify", "provide"]

_PROVIDED = json.loads(os.environ.get("UNO_Q_PROVIDED", "{}"))
# {name: number of arguments} for every call the lesson's sketch makes to Python
_SKETCH_CALLS = json.loads(os.environ.get("UNO_Q_SKETCH_CALLS", "{}"))
_MAX_LOOPS = 5
_MAX_SLEEPS = 25
_state = {"sleeps": 0, "in_handler": False}
_bricks = []


class StopLesson(BaseException):
    pass


def _sleep(_secs=0):
    _state["sleeps"] += 1
    if _state["sleeps"] > _MAX_SLEEPS:
        raise StopLesson()


_time.sleep = _sleep


class _Bridge:
    """The names on the real Bridge class, and no others."""
    __slots__ = ("_handlers",)

    def __init__(self):
        self._handlers = {}

    def _check(self, name):
        if not isinstance(name, str):
            raise TypeError("Bridge method name must be a str")
        if _PROVIDED and name not in _PROVIDED:
            raise ValueError(f'no sketch in this lesson provides "{name}"')

    def call(self, method_name, *args, timeout=10):
        if _state["in_handler"]:
            raise RuntimeError("Bridge.call() inside a provided handler: nested calls are not supported")
        self._check(method_name)
        return _PROVIDED.get(method_name)

    def notify(self, method_name, *args):
        self._check(method_name)

    def provide(self, method_name, handler):
        if _state["in_handler"]:
            raise RuntimeError("Bridge.provide() inside a provided handler")
        if not callable(handler):
            raise ValueError("handler is not callable")
        if method_name in _PROVIDED:
            raise ValueError(f'the sketch already provides "{method_name}"')
        self._handlers[method_name] = handler

    def unprovide(self, method_name):
        self._handlers.pop(method_name, None)


Bridge = _Bridge()


class _App:
    __slots__ = ()

    def run(self, user_loop=None):
        for brick in _bricks:
            brick._start()
        # Play the sketch's part: call each provided function once, the way the
        # sketch in this lesson calls it. Inside a handler Bridge.call() raises,
        # as it does on the board.
        for name, nargs in _SKETCH_CALLS.items():
            handler = Bridge._handlers.get(name)
            if handler is None:
                continue
            _state["in_handler"] = True
            try:
                handler(*([0] * nargs))
            except RuntimeError:
                raise
            except Exception:  # noqa: BLE001  a made-up 0 isn't real sensor data
                pass
            finally:
                _state["in_handler"] = False
        if user_loop is not None:
            if not callable(user_loop):
                raise TypeError("user_loop must be a function")
            for _ in range(_MAX_LOOPS):
                user_loop()
        raise StopLesson()


App = _App()


def call(method_name=None, timeout=10):
    def wrap(fn):
        name = method_name or fn.__name__
        def proxy(*args, timeout=timeout):
            return Bridge.call(name, *args, timeout=timeout)
        return proxy
    return wrap


def notify(method_name=None):
    def wrap(fn):
        name = method_name or fn.__name__
        def proxy(*args):
            return Bridge.notify(name, *args)
        return proxy
    return wrap


def provide(method_name=None):
    def wrap(fn):
        Bridge.provide(method_name or fn.__name__, fn)
        return fn
    return wrap


class _Leds:
    """The two RGB LEDs on the Linux side (UNO Q user manual, "MPU Controlled LEDs")."""
    __slots__ = ()

    @staticmethod
    def _check(r, g, b):
        for v in (r, g, b):
            if v not in (0, 1, True, False):
                raise ValueError("LED colour values are 0 or 1")

    def set_led1_color(self, r, g, b):
        self._check(r, g, b)

    def set_led2_color(self, r, g, b):
        self._check(r, g, b)


Leds = _Leds()


class Logger:
    """arduino.app_utils.Logger as used in App Lab's logging example."""
    __slots__ = ("name", "level")

    def __init__(self, name, level=None):
        self.name = name
        self.level = level

    def _log(self, msg, *args):
        str(msg) % args if args else str(msg)

    debug = info = warning = error = critical = exception = _log
