from __future__ import annotations

import logging

import simple_term_menu

from ..demo import control, start_button
from ..robot import utils

logging.basicConfig(level=logging.INFO)


class StartDemo(start_button.StartButton):
    def __init__(self, control: control.DemoControl) -> None:
        self.control = control
        super().__init__()

    def handle_event(self) -> None:
        if self.control.current_state == control.DemoControl.idle:
            self.control.pick_cup()


def main() -> None:
    left, right = utils.get_robot_hostnames()
    model = control.DemoModel(left, right)
    sm = control.DemoControl(model)
    btn = StartDemo(sm)

    sm._graph().write("statemachine.png", format="png")  # pylint: disable=no-member, protected-access

    options = ["Trigger Demo", "Draw beer Manually", "Exit"]
    terminal_menu = simple_term_menu.TerminalMenu(options)
    while True:
        choice = terminal_menu.show()
        if choice == 0 and sm.current_state == control.DemoControl.idle:
            sm.pick_cup()
        elif choice == 1 and sm.current_state == control.DemoControl.idle:
            sm.pick_cup(user=True)
        elif choice == 2:
            break

    btn.close()
