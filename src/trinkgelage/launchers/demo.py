from __future__ import annotations

import argparse
import logging
import threading

import simple_term_menu

from ..demo import control, start_button
from ..robot import utils

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("trinkgelage")


class StartDemo(start_button.StartButton):
    """Physical start button communicating over serial port."""

    def __init__(
        self, demo_control: control.DemoControl, menu: simple_term_menu.TerminalMenu
    ) -> None:
        self.control = demo_control
        self.menu = menu
        super().__init__()

    def handle_event(self) -> None:
        logger.info("Start button pressed")
        if self.control.current_state == control.DemoControl.idle:
            threading.Thread(target=self.run_demo).start()
        else:
            logger.warning("Demo already in progress")
            self.menu._paint_menu()  # pylint: disable=protected-access

    def run_demo(self) -> None:
        self.control.start_demo()
        self.menu._paint_menu()  # pylint: disable=protected-access


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--start-position",
        "-n",
        type=int,
        default=1,
        help="Cup position to start the demo",
    )
    parser.add_argument("--gui-hostname", type=str, default="gap-nuc-003.local")
    parser.add_argument("--gui-port", type=int, default=8000)
    args = parser.parse_args()

    options = ["Trigger Demo", "Draw beer Manually", "Confirm cups refilled", "Exit"]
    terminal_menu = simple_term_menu.TerminalMenu(options)

    left, right = utils.get_robot_hostnames()
    model = control.DemoModel(
        left,
        right,
        gui_url=f"http://{args.gui_hostname}:{args.gui_port}",
        start_position=args.start_position,
    )
    sm = control.DemoControl(model)
    btn = StartDemo(sm, terminal_menu)

    while True:
        choice = terminal_menu.show()
        if choice == 0 and sm.current_state == control.DemoControl.idle:
            sm.start_demo()
        elif choice == 1 and sm.current_state == control.DemoControl.idle:
            sm.start_demo(user=True)
        elif choice == 2 and sm.current_state == control.DemoControl.cups_empty:
            sm.refill_cups()
        elif choice == 3:
            break

    btn.close()
