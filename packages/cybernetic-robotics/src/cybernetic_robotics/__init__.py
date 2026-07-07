"""Friendly Python controls for Cybernetic IDE robotics.

The beginner entrypoint is :class:`G1Robot`:

```python
from cybernetic_robotics import G1Robot

with G1Robot.connect() as robot:
    robot.raise_right_hand()
    robot.snapshot("right-hand-up.jpg")
```

Power users can drop down to :class:`SimulatorClient` and :class:`TinyWebSocket`
to work directly with the Booster-style simulator protocol.
"""

from .config import RobotEndpoints
from .errors import CyberneticRoboticsError, ProtocolError, SimulatorUnavailable
from .g1 import G1Robot, G1Status, connect
from .simulator import CameraState, SimulatorClient, SimulatorStatus
from .unitree import LocoClient
from .websocket import TinyWebSocket

__all__ = [
    "CameraState",
    "CyberneticRoboticsError",
    "G1Robot",
    "G1Status",
    "LocoClient",
    "ProtocolError",
    "RobotEndpoints",
    "SimulatorClient",
    "SimulatorStatus",
    "SimulatorUnavailable",
    "TinyWebSocket",
    "connect",
]
