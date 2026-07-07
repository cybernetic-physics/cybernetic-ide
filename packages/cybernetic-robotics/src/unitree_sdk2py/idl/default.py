from __future__ import annotations

from .unitree_go.msg.dds_ import MotorCmd_, MotorCmds_, SportModeState_
from .unitree_hg.msg.dds_ import IMUState_, LowCmd_, LowState_, MotorCmd_, MotorState_


def unitree_hg_msg_dds__IMUState_() -> IMUState_:
    return IMUState_()


def unitree_hg_msg_dds__MotorCmd_() -> MotorCmd_:
    return MotorCmd_()


def unitree_hg_msg_dds__MotorState_() -> MotorState_:
    return MotorState_()


def unitree_hg_msg_dds__LowCmd_() -> LowCmd_:
    return LowCmd_()


def unitree_hg_msg_dds__LowState_() -> LowState_:
    return LowState_()


def unitree_go_msg_dds__SportModeState_() -> SportModeState_:
    return SportModeState_()


def unitree_go_msg_dds__MotorCmd_() -> MotorCmd_:
    return MotorCmd_()


def unitree_go_msg_dds__MotorCmds_() -> MotorCmds_:
    return MotorCmds_()
