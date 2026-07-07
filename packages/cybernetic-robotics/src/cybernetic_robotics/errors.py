class CyberneticRoboticsError(RuntimeError):
    """Base exception for Cybernetic robotics helpers."""


class SimulatorUnavailable(CyberneticRoboticsError):
    """Raised when the local simulator endpoint cannot be reached."""


class ProtocolError(CyberneticRoboticsError):
    """Raised when the simulator returns an invalid or unsuccessful response."""
