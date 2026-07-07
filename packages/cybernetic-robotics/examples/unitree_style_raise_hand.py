#!/usr/bin/env python3
"""The same demo using Unitree SDK2-shaped imports."""

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map


ChannelFactoryInitialize(0, "cyber-sim")

arm = G1ArmActionClient()
arm.SetTimeout(10.0)
arm.Init()
result = arm.ExecuteAction(action_map["right hand up"])
print("ExecuteAction(right hand up) ->", result)
print(arm.last_response)
