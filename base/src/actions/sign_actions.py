#!/usr/bin/env python3
"""Shared landmark action sequences used by YOLO decisions and manual keys 1-6."""
# -*- coding: utf-8 -*-
import time

from src.actions.base_action import (
    Advance,
    ShiftLeft,
    ShiftRight,
    Sleep,
    SpinAntiClockwise,
    SpinClockwise,
    Stop,
    TurnLeft,
)


def reset_controller_timing(ctrl):
    ctrl.last_modify_time = time.time()
    save = getattr(ctrl, "_save", None)
    if callable(save):
        save()


def _clear(ctrl, clear_func=None):
    if clear_func is not None:
        clear_func()
    else:
        reset_controller_timing(ctrl)


def _advance_interrupt(ctrl, speed):
    interrupt = getattr(ctrl, "interrupt_and_execute", None)
    if callable(interrupt):
        interrupt(Advance(speed=speed))
    else:
        ctrl.execute(Advance(speed=speed))


def execute_left_turn(ctrl, clear_func=None):
    _advance_interrupt(ctrl, 28)
    ctrl.execute(Sleep(1.3))
    _clear(ctrl, clear_func)
    ctrl.execute(SpinAntiClockwise(speed=32))
    ctrl.execute(Sleep(0.65))
    ctrl.execute(Stop())
    ctrl.execute(Sleep(0.2))


def execute_right_turn(ctrl, clear_func=None):
    _advance_interrupt(ctrl, 27)
    ctrl.execute(Sleep(1.3))
    _clear(ctrl, clear_func)
    ctrl.execute(SpinClockwise(speed=32))
    ctrl.execute(Sleep(0.6))
    ctrl.execute(Stop())
    ctrl.execute(Sleep(0.2))


def execute_turnaround_entry(ctrl, clear_func=None):
    ctrl.execute(Advance(speed=32))
    ctrl.execute(Sleep(1))
    _clear(ctrl, clear_func)
    ctrl.execute(Stop())
    ctrl.execute(Sleep(1.5))
    _clear(ctrl, clear_func)
    ctrl.execute(Advance(speed=32))
    ctrl.execute(Sleep(0.5))
    _clear(ctrl, clear_func)
    ctrl.execute(TurnLeft(speed=15, degree=3))
    ctrl.execute(Sleep(1.6))
    ctrl.execute(Stop())


def execute_turnaround_finish(ctrl, clear_func=None):
    ctrl.execute(Advance(speed=30))
    ctrl.execute(Sleep(1.0))
    _clear(ctrl, clear_func)
    ctrl.execute(Stop())
    ctrl.execute(Sleep(15))
    _clear(ctrl, clear_func)


def execute_park(ctrl, clear_func=None):
    ctrl.execute(Advance(speed=32))
    ctrl.execute(Sleep(1.1))
    _clear(ctrl, clear_func)
    ctrl.execute(Stop())
    ctrl.execute(Sleep(1))
    _clear(ctrl, clear_func)
    ctrl.execute(ShiftRight(speed=40))
    ctrl.execute(Sleep(1.5))
    _clear(ctrl, clear_func)
    ctrl.execute(Stop())
    ctrl.execute(Sleep(2))
    _clear(ctrl, clear_func)
    ctrl.execute(ShiftLeft(speed=40))
    ctrl.execute(Sleep(1.5))
    _clear(ctrl, clear_func)
    ctrl.execute(Advance(speed=30))
    ctrl.execute(Sleep(0.5))


def execute_stop_sign(ctrl):
    ctrl.execute(Stop())
    ctrl.execute(Sleep(2))


MANUAL_SIGN_ACTIONS = {
    "1": ("left_turn", execute_left_turn),
    "2": ("right_turn", execute_right_turn),
    "3": ("turnaround_entry", execute_turnaround_entry),
    "4": ("turnaround_finish", execute_turnaround_finish),
    "5": ("park", execute_park),
    "6": ("stop_sign", execute_stop_sign),
}


def get_manual_sign_action(key):
    return MANUAL_SIGN_ACTIONS.get(key)
