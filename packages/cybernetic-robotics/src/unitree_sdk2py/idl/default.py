from __future__ import annotations

from .unitree_go.msg.dds_ import MotorCmd_ as GoMotorCmd_, MotorCmds_, SportModeState_
from .unitree_hg.msg.dds_ import (
    HandCmd_,
    HandState_,
    IMUState_,
    LowCmd_,
    LowState_,
    MotorCmd_ as HgMotorCmd_,
    MotorState_,
    PressSensorState_,
)


def unitree_hg_msg_dds__IMUState_() -> IMUState_:
    return IMUState_()


def unitree_hg_msg_dds__MotorCmd_() -> HgMotorCmd_:
    return HgMotorCmd_()


def unitree_hg_msg_dds__MotorState_() -> MotorState_:
    return MotorState_()


def unitree_hg_msg_dds__LowCmd_() -> LowCmd_:
    return LowCmd_()


def unitree_hg_msg_dds__LowState_() -> LowState_:
    return LowState_()


def unitree_hg_msg_dds__PressSensorState_() -> PressSensorState_:
    return PressSensorState_()


def unitree_hg_msg_dds__HandCmd_() -> HandCmd_:
    return HandCmd_()


def unitree_hg_msg_dds__HandState_() -> HandState_:
    return HandState_()


def unitree_go_msg_dds__SportModeState_() -> SportModeState_:
    return SportModeState_()


def unitree_go_msg_dds__MotorCmd_() -> GoMotorCmd_:
    return GoMotorCmd_()


def unitree_go_msg_dds__MotorCmds_() -> MotorCmds_:
    return MotorCmds_()
