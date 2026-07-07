---
name: unitree-g1-sdk
description: Use when writing, reviewing, or running Python code that controls a Unitree G1 through Cybernetic IDE's Unitree SDK facade, including hand/arm actions, ChannelFactoryInitialize, G1ArmActionClient, and local MuJoCo simulator demos.
---

# Unitree G1 SDK Facade

Use this skill when the user asks to write Python for the Unitree G1, run a G1 control demo, raise or release the robot arm, or make code look like official `unitree_sdk2py` usage.

## Current Runtime Shape

Cybernetic IDE currently ships a simulator-facing Unitree SDK facade at:

- `overlays/unitree-g1-sdk-shim/unitree_sdk2py/`
- `examples/g1_raise_hand_sdk.py`
- `examples/control_g1_sim.py`

The facade intentionally mirrors official SDK naming:

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
```

The current simulator facade maps high-level SDK arm actions onto the local MuJoCo protocol. It is not yet a full DDS/SDK2 bridge.

## Preferred Workflow

1. Check simulator state with the `cybernetic-robotics` MCP tool `sim_status`.
2. If needed, prepare/start the runtime with `sim_prepare_runtime` and `sim_start`.
3. Generate or write the Python with `unitree_sdk_scaffold_python`, or edit a normal file with the agent's file tools.
4. Run the script with `python_control_run` for short demos or `python_control_start` for longer jobs.
5. Verify the result with `viewer_snapshot`, `sim_status`, or `g1_execute_action`.

## Minimal Script

```python
#!/usr/bin/env python3
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient


def main():
    ChannelFactoryInitialize(0)
    client = G1ArmActionClient()
    client.Init()
    client.SetTimeout(10.0)
    result = client.ExecuteAction(client.action_map["right hand up"])
    if result != 0:
        raise SystemExit(f"G1 action failed with status {result}")
    print("G1 right hand raised")


if __name__ == "__main__":
    main()
```

## Notes

- Use `right hand up` for the raise-hand demo.
- Use `release arm` to return to neutral.
- Use the MCP tool `g1_list_actions` before inventing new action names.
- For real robot support, do not bypass safety tooling; future real-robot commands must go through explicit mode/profile gates.
