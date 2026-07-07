from __future__ import annotations

import threading
import time
from collections.abc import Callable
from typing import Any


class RecurrentThread:
    """Portable recurrent worker compatible with Unitree SDK2 examples."""

    def __init__(
        self,
        interval: float = 1.0,
        target: Callable[..., Any] | None = None,
        name: str | None = None,
        args: tuple[Any, ...] = (),
        kwargs: dict[str, Any] | None = None,
    ):
        self.interval = max(0.0, float(interval or 0.0))
        self.target = target
        self.args = args
        self.kwargs = {} if kwargs is None else kwargs
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=name, daemon=True)

    def Start(self):  # noqa: N802 - match Unitree SDK2 API.
        self._thread.start()

    def Wait(self, timeout: float | None = None):  # noqa: N802 - match Unitree SDK2 API.
        self._stop.set()
        self._thread.join(timeout=timeout)

    def _run(self):
        while not self._stop.is_set():
            started = time.monotonic()
            if self.target is not None:
                self.target(*self.args, **self.kwargs)
            elapsed = time.monotonic() - started
            if self.interval <= 0:
                continue
            self._stop.wait(max(0.0, self.interval - elapsed))

