from __future__ import annotations

import logging
import pathlib
import threading
import typing

import numpy as np
import panda_py
from panda_py import controllers, libfranka

log = logging.getLogger("actions")

DATA_PATH = pathlib.Path(pathlib.Path(__file__).parent.parent) / "data"


def two_arm_motion_from_files(
    left: panda_py.Panda,
    right: panda_py.Panda,
    left_filenames: str | typing.Iterable[str],
    right_filenames: str | typing.Iterable[str],
    left_speed_factor: float = 0.2,
    right_speed_factor: float = 0.2,
    max_retries: int = 3,
) -> None:
    threads: list[threading.Thread] = []
    t_left = threading.Thread(
        target=motion_from_file,
        args=(left, left_filenames),
        kwargs={"speed_factor": left_speed_factor, "max_retries": max_retries},
    )
    t_right = threading.Thread(
        target=motion_from_file,
        args=(right, right_filenames),
        kwargs={"speed_factor": right_speed_factor, "max_retries": max_retries},
    )
    t_left.start()
    t_right.start()
    for t in [t_left, t_right]:
        t.join()


def motion_from_file(
    robot: panda_py.Panda,
    filenames: str | typing.Iterable[str],
    speed_factor: float = 0.2,
    max_retries: int = 3,
) -> bool:
    if isinstance(filenames, str):
        items = [filenames]
    else:
        items = list(filenames)

    queue: list[np.ndarray] = []
    last_shape = None

    def process_queue(queue: list[np.ndarray]) -> bool:
        if not queue:
            return True
        shape = queue[0].shape
        if len(shape) == 1:
            for q in queue:
                assert q.shape[0] == 7
            return move_to_joint_position(
                robot, queue, speed_factor=speed_factor, max_retries=max_retries
            )
        elif len(shape) == 2:
            success = True
            for q in queue:
                assert q.shape[1] == 14
                success = success and play_trajectory(
                    robot,
                    q[:, :7],
                    q[:, 7:],
                    speed_factor=speed_factor,
                    max_retries=max_retries,
                )
            return success
        else:
            raise RuntimeError("Invalid file format")

    success = True
    for fn in items:
        data = load_csv(fn)
        if last_shape is None:
            last_shape = data.shape
        if data.shape != last_shape:
            if queue:
                success = success and process_queue(queue)
                queue = []
            last_shape = data.shape
        queue.append(data)
    if queue:
        success = success and process_queue(queue)
    return success


def load_csv(filename: str) -> np.ndarray:
    return np.loadtxt(DATA_PATH / filename, delimiter=",")


def move_to_pose(
    robot: panda_py.Panda,
    poses: np.ndarray | list[np.ndarray],
    speed_factor: float = 0.2,
    max_retries: int = 3,
) -> bool:
    try:
        return robot.move_to_pose(
            poses,
            speed_factor=speed_factor,
            impedance=np.diag([600, 600, 600, 30, 30, 30]),
        )
    except RuntimeError as e:
        log.error(e)
        if max_retries > 0:
            return move_to_pose(
                robot,
                poses,
                speed_factor=speed_factor,
                max_retries=max_retries - 1,
            )
        else:
            log.error("Maximum retries reached for move_to_joint_position")
            return False


def move_to_joint_position(
    robot: panda_py.Panda,
    joint_positions: np.ndarray | list[np.ndarray],
    speed_factor: float = 0.2,
    max_retries: int = 3,
) -> bool:
    try:
        return robot.move_to_joint_position(joint_positions, speed_factor=speed_factor)
    except RuntimeError as e:
        log.error(e)
        if max_retries > 0:
            return move_to_joint_position(
                robot,
                joint_positions,
                speed_factor=speed_factor,
                max_retries=max_retries - 1,
            )
        else:
            log.error("Maximum retries reached for move_to_joint_position")
            return False


def play_trajectory(
    robot: panda_py.Panda,
    q: list[np.ndarray],
    dq: list[np.ndarray],
    at_index: int = 0,
    speed_factor: float = 0.05,
    max_retries: int = 3,
) -> bool:
    i = at_index
    robot.move_to_joint_position(q[i], speed_factor=speed_factor)
    ctrl = controllers.JointPosition()
    robot.start_controller(ctrl)
    try:
        with robot.create_context(frequency=1000, max_runtime=len(q) / 1000.0) as ctx:
            while ctx.ok():
                if i >= len(q):
                    break
                ctrl.set_control(q[i], dq[i])
                i += 1
        return True
    except RuntimeError as e:
        log.error(e)
        if max_retries > 0:
            return play_trajectory(
                robot,
                q,
                dq,
                at_index=i,
                speed_factor=speed_factor,
                max_retries=max_retries - 1,
            )
        else:
            log.error("Maximum retries reached for play_trajectory")
            return False


def release(gripper: libfranka.Gripper) -> None:
    if not gripper.move(0.08, 0.05):
        raise RuntimeError("Opening gripper failed")


def grasp(gripper: libfranka.Gripper) -> None:
    if not gripper.grasp(0, 0.05, 20, 0.08, 0.08):
        raise RuntimeError("Grasping failed")
