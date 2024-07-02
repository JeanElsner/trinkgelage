from __future__ import annotations

import logging

from ..demo import control

logging.basicConfig(level=logging.INFO)


def main() -> None:
    sm = control.DemoControl(control.DemoModel())

    while input() != "q":
        sm.pick_cup(user=True)

    sm._graph().write_png("tmp.png")  # pylint: disable=no-member, protected-access
