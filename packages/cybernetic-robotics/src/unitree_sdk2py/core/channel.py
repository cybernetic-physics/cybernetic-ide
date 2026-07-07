from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ChannelFactoryConfig:
    domain_id: int = 0
    network_interface: str | None = None


_CONFIG = ChannelFactoryConfig()


def ChannelFactoryInitialize(id: int = 0, networkInterface: str | None = None):
    """Record SDK2 channel settings.

    Unitree's real SDK initializes CycloneDDS here. Cybernetic's simulator shim
    keeps the same call visible to user code and routes high-level actions to
    the local MuJoCo harness.
    """

    global _CONFIG
    _CONFIG = ChannelFactoryConfig(id, networkInterface)


def current_channel_factory_config() -> ChannelFactoryConfig:
    return _CONFIG


class ChannelPublisher:
    def __init__(self, name: str, message_type):
        self.name = name
        self.message_type = message_type
        self.inited = False

    def Init(self):
        self.inited = True

    def Write(self, _message):
        raise NotImplementedError(
            "Cybernetic's bootstrap Unitree shim only supports high-level "
            "G1ArmActionClient actions. DDS topic publishing is the next backend."
        )

    def Close(self):
        self.inited = False


class ChannelSubscriber:
    def __init__(self, name: str, message_type):
        self.name = name
        self.message_type = message_type
        self.inited = False

    def Init(self, handler=None, queueLen: int = 0):
        self.handler = handler
        self.queue_len = queueLen
        self.inited = True

    def Close(self):
        self.inited = False

    def Read(self, timeout: int | None = None):
        raise NotImplementedError(
            "Cybernetic's bootstrap Unitree shim does not yet expose DDS telemetry."
        )
