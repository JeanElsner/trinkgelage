from __future__ import annotations

import logging

from ..demo import control
from ..robot import utils

logging.basicConfig(level=logging.INFO)


def main() -> None:
    left, right = utils.get_robot_hostnames()
    model = control.DemoModel(left, right)
    sm = control.DemoControl(model)

    while input() != "q":
        sm.pick_cup(user=True)

    sm._graph().write_png("tmp.png")  # pylint: disable=no-member, protected-access
