# Unitree G1 SDK Shim

This overlay is the first Cybernetic IDE simulator transport for Unitree-shaped
Python code.

User code should look like the official Unitree SDK2 Python examples:

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map

ChannelFactoryInitialize(0, "cyber-sim")

arm = G1ArmActionClient()
arm.SetTimeout(10.0)
arm.Init()
arm.ExecuteAction(action_map["right hand up"])
```

In this bootstrap layer, `G1ArmActionClient.ExecuteAction()` sends a local JSON
command to the G1 MuJoCo protocol container. That keeps the user API aligned
with Unitree SDK2 while the backend moves toward the official SDK2/CycloneDDS
transport used by `unitree_mujoco`.

Run the checked-in demo from the repo root:

```sh
python3 examples/g1_raise_hand_sdk.py
```

The supported simulated action set is intentionally tiny:

| Unitree action | Action ID | Simulator command |
| --- | ---: | --- |
| `right hand up` | `23` | `{"command":"pose","pose":"raise_right_hand"}` |
| `release arm` | `99` | `{"command":"pose","pose":"neutral"}` |

Unsupported action IDs return a non-zero code. That makes missing simulator
coverage obvious while preserving the high-level Unitree client shape.
