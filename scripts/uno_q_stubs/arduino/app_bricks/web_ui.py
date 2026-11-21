"""Strict stand-in for the WebUI - HTML Brick.

Method names are the ones Arduino's own example Apps use
(github.com/arduino/app-bricks-examples, 01-led-blink/04-blinking-an-led-with-ui
and 08-web-ui-basics): on_message, on_connect, on_disconnect, send_message,
expose_api. The real Brick serves the App's assets/ folder on port 7000.

Registered message handlers are each called once with a fake client and an
empty payload when App.run() starts, so a handler that calls a Bridge name no
sketch provides is caught.
"""
from arduino import app_utils as _utils

__all__ = ["WebUI"]


class WebUI:
    __slots__ = ("_handlers",)

    def __init__(self, **options):
        self._handlers = []
        _utils._bricks.append(self)

    def on_message(self, event, handler):
        if not isinstance(event, str) or not callable(handler):
            raise TypeError("on_message(event_name, handler)")
        self._handlers.append((handler, ("client-1", {})))

    def on_connect(self, handler):
        if not callable(handler):
            raise TypeError("on_connect(handler)")
        self._handlers.append((handler, ("client-1",)))

    def on_disconnect(self, handler):
        if not callable(handler):
            raise TypeError("on_disconnect(handler)")

    def send_message(self, event, data=None, client=None):
        if not isinstance(event, str):
            raise TypeError("send_message(event_name, data)")

    def expose_api(self, method, path, function):
        if method not in ("GET", "POST", "PUT", "DELETE", "PATCH"):
            raise ValueError("unknown HTTP method: %r" % (method,))
        if not callable(function):
            raise TypeError("expose_api(method, path, function)")

    def _start(self):
        for handler, args in self._handlers:
            try:
                handler(*args)
            except (KeyError, TypeError, ValueError) as exc:
                # An empty payload isn't what the page really sends: only a
                # Bridge problem is the lesson's fault.
                if "provides" in str(exc):
                    raise
