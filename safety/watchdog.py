"""
Watchdog running in its OWN thread, independent of the control loop.

Why: plutocontrol keeps re-sending the last RC values at ~45 Hz from its own thread. If the control loop
hangs (camera driver stall, exception inside a callback, debugger pause ...) the drone would keep executing
the last command. The control loop must call kick() every cycle; if it does not for timeout_s while the
watchdog is armed, on_timeout() is called ONCE (the program passes: neutral sticks + firmware LAND).
"""

import threading
import time


class Watchdog:

    def __init__(self, timeout_s, on_timeout, poll_s=0.02):
        self.timeout_s = float(timeout_s)
        self.on_timeout = on_timeout
        self.poll_s = poll_s
        self._last_kick = time.perf_counter()
        self._armed = False
        self._fired = False
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, name="Watchdog", daemon=True)
        self._thread.start()

    def kick(self):
        with self._lock:
            self._last_kick = time.perf_counter()

    def arm(self):
        with self._lock:
            self._armed = True
            self._fired = False
            self._last_kick = time.perf_counter()

    def disarm(self):
        with self._lock:
            self._armed = False

    @property
    def fired(self):
        return self._fired

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)

    def _run(self):
        while not self._stop.is_set():
            with self._lock:
                expired = self._armed and not self._fired and time.perf_counter() - self._last_kick > self.timeout_s
                if expired:
                    self._fired = True
            if expired:
                try:
                    self.on_timeout()
                except Exception as exc:          # the watchdog itself must never die
                    print(f"WATCHDOG: on_timeout raised {exc}")
            time.sleep(self.poll_s)
