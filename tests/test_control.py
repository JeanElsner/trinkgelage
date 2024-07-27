from __future__ import annotations

import socket
from unittest import mock

import pytest

from trinkgelage.demo import control


@mock.patch("panda_py.Panda")
@mock.patch("panda_py.libfranka.Gripper")
@mock.patch("trinkgelage.demo.control.DemoModel.cup_full", return_value=True)
def test_control(mock_cup_full, mock_gripper, mock_panda):
    del mock_cup_full, mock_panda
    mock_gripper.return_value.read_once.return_value.width = 0.03
    model = control.DemoModel("left", "right")
    ctrl = control.DemoControl(model)

    assert ctrl.current_state == control.DemoControl.idle
    ctrl.start_demo()
    assert ctrl.current_state == control.DemoControl.idle
    model.cups = 0
    ctrl.start_demo()
    assert ctrl.current_state == control.DemoControl.cups_empty
    ctrl.refill_cups()
    assert ctrl.current_state == control.DemoControl.idle


@mock.patch("panda_py.Panda")
@mock.patch("panda_py.libfranka.Gripper")
@mock.patch("trinkgelage.demo.control.DemoModel.cup_full", return_value=True)
def test_gui_connection(mock_cup_full, mock_gripper, mock_panda):
    del mock_cup_full, mock_gripper, mock_panda
    model = control.DemoModel("left", "right", use_gui=True)
    with pytest.raises(socket.gaierror):
        control.DemoControl(model).start_demo()
