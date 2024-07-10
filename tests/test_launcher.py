from __future__ import annotations

import argparse
from unittest import mock

from trinkgelage.launchers import demo


@mock.patch("panda_py.Panda")
@mock.patch("panda_py.libfranka.Gripper")
@mock.patch("simple_term_menu.TerminalMenu")
@mock.patch("xmlrpc.client.ServerProxy")
@mock.patch("trinkgelage.demo.control.DemoModel.cup_full", return_value=True)
@mock.patch(
    "argparse.ArgumentParser.parse_args",
    return_value=argparse.Namespace(gui_hostname="", gui_port=0, start_position=1),
)
def test_launcher(
    mock_argparse, mock_cup_full, mock_proxy, mock_menu, mock_gripper, mock_panda
):
    del mock_argparse, mock_cup_full, mock_proxy, mock_panda
    mock_gripper.return_value.read_once.return_value.width = 0.03
    mock_menu.return_value.show.side_effect = [0, 1, 2, 3]
    demo.main()
