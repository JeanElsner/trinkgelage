from __future__ import annotations

import os


def get_robot_hostnames(required: bool = True) -> tuple[str, str]:
    """
    Retrieve the left and right robot ips (hostnames) from
    the environment variables `PANDA_LEFT` and `PANDA_RIGHT`.
    """
    left, right = os.environ.get("PANDA_LEFT"), os.environ.get("PANDA_RIGHT")
    if required and (left is None or right is None):
        raise RuntimeError(
            "Please make sure the environment variables "
            + "PANDA_LEFT and PANDA_RIGHT are set to the respective robot hostnames."
        )
    return left, right  # type: ignore[return-value]
