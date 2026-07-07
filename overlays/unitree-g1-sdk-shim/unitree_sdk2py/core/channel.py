"""Small SDK2 channel facade for Cybernetic's local G1 simulator.

The official Unitree SDK2 initializes a CycloneDDS channel factory here. This
shim records the same session knobs, then lets higher-level clients route to
the local simulator bridge. Keeping this import path stable is what lets us
replace the bridge with real SDK2/DDS internals later.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ChannelFactoryConfig:
    domain_id: int = 0
    network_interface: str | None = None


_CONFIG = ChannelFactoryConfig()


def ChannelFactoryInitialize(id: int = 0, networkInterface: str | None = None):
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
