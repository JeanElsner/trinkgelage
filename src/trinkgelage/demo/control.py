from __future__ import annotations

import logging

import numpy as np
import panda_py
import statemachine  # type: ignore[import-not-found]
from panda_py import libfranka

from ..robot import actions

log = logging.getLogger("control")


class DemoControl(statemachine.StateMachine):  # type: ignore[misc]
    """Control flow program of the demo."""

    idle = statemachine.State(initial=True)
    holding_empty_cup = statemachine.State()
    pouring = statemachine.State()
    holding_filled_cup = statemachine.State()
    cups_empty = statemachine.State()
    waiting_for_user_pickup = statemachine.State()

    pick_cup = idle.to(holding_empty_cup, cond="cup_available") | idle.to(
        cups_empty, unless="cup_available"
    )
    refill_cups = cups_empty.to(idle)
    open_faucet = holding_empty_cup.to(
        pouring, on="measure_cup", cond="cup_grasped"
    ) | holding_empty_cup.to(idle, unless="cup_grasped")
    close_faucet = pouring.to(holding_filled_cup, cond="cup_full") | pouring.to(
        pouring, unless="cup_full", on="measure_cup"
    )
    place_cup = holding_filled_cup.to(waiting_for_user_pickup)
    return_to_idle = waiting_for_user_pickup.to(
        idle, cond="user_pickup"
    ) | waiting_for_user_pickup.to(waiting_for_user_pickup, unless="user_pickup")

    def on_enter_holding_empty_cup(self, user: bool = False) -> None:
        self.open_faucet(user=user)

    def on_enter_pouring(self, user: bool = True) -> None:
        self.close_faucet(user=user)

    def on_enter_holding_filled_cup(self) -> None:
        self.place_cup()

    def on_enter_waiting_for_user_pickup(self) -> None:
        self.return_to_idle()

    def on_enter_cups_empty(self) -> None:
        self.refill_cups()


class DemoModel:
    """Implements the demo actions."""

    max_cups = 12

    def __init__(self, left: str, right: str, enforce_rt: bool = True) -> None:
        self.cups = self.max_cups
        self.bias = np.zeros(6)
        self.load = np.zeros(6)
        if enforce_rt:
            rt = libfranka.RealtimeConfig.kEnforce
        else:
            rt = libfranka.RealtimeConfig.kIgnore
        self.left = panda_py.Panda(left, realtime_config=rt)
        self.left_gripper = libfranka.Gripper(left)
        self.right = panda_py.Panda(right, realtime_config=rt)
        self.right_gripper = libfranka.Gripper(right)
        self.init_robot()

    def init_robot(self) -> None:
        # t1 = threading.Thread(target=self.left_gripper.homing)
        # t2 = threading.Thread(target=self.right_gripper.homing)
        # t1.start()
        # t2.start()
        # for t in [t1, t2]:
        #     t.join()
        actions.two_arm_motion_from_files(
            self.left, self.right, "left_idle.csv", "right_idle.csv"
        )

    def on_pick_cup(self, target: statemachine.State, user: bool = False) -> None:
        if target != DemoControl.cups_empty:
            self.cups -= 1
            self.cups = max(0, self.cups)

            idx = self.max_cups - self.cups
            log.info("Picking up glass at position %d", idx)

            disp_x = (idx - 1) % 3 * 0.15
            disp_z = -np.floor((idx - 1) / 3) * 0.1

            q = actions.load_csv("grasp_cup_1.csv")
            grasp_cup = panda_py.fk(q)
            grasp_cup[0, 3] += disp_x
            grasp_cup[2, 3] += disp_z

            q = actions.load_csv("grasp_cup_1.csv")
            pre_grasp_cup = panda_py.fk(q)
            pre_grasp_cup[2, 3] += 0.15

            pre_grasp_cup_x = pre_grasp_cup.copy()
            pre_grasp_cup_x[0, 3] += disp_x

            pre_grasp_cup_z = pre_grasp_cup.copy()
            pre_grasp_cup_z[0, 3] += disp_x
            pre_grasp_cup_z[2, 3] += disp_z

            post_grasp_cup = grasp_cup.copy()
            post_grasp_cup[1, 3] += 0.15

            # actions.motion_from_file(self.right, "pre_grasp_cup_2.csv")
            actions.move_to_pose(self.right, [pre_grasp_cup_z, grasp_cup])
            actions.grasp(self.right_gripper)
            actions.move_to_pose(self.right, post_grasp_cup)

            # actions.motion_from_file(
            #     self.right, [f"pre_grasp_cup_{idx}.csv", f"grasp_cup_{idx}.csv"]
            # )
            # actions.grasp(self.right_gripper)
            # actions.motion_from_file(self.right, f"post_grasp_cup_{idx}.csv")

            if user:
                actions.motion_from_file(self.right, "move_cup_to_faucet.csv")
            else:
                actions.two_arm_motion_from_files(
                    self.left,
                    self.right,
                    ["pre_grasp_faucet.csv", "grasp_faucet.csv"],
                    "move_cup_to_faucet.csv",
                )

            self.bias = np.array(self.right.get_state().O_F_ext_hat_K)
            # f"post_grasp_cup_{idx}.csv"
            # actions.motion_from_file("move_cup_to_faucet.csv")
            # TODO pick up cup, move to faucet, bias force sensor, grasp faucet if user False

    def on_open_faucet(self, target: statemachine.State, user: bool = False) -> None:
        if target == DemoControl.pouring:
            if not user:
                actions.grasp(self.left_gripper)
                actions.motion_from_file(self.left, "open_faucet.csv")
        else:
            actions.two_arm_motion_from_files(
                self.left,
                self.right,
                "left_idle.csv",
                ["post_place_cup.csv", "right_idle.csv"],
            )
        # TODO open faucet if user is false

    def on_close_faucet(self, target: statemachine.State, user: bool = False) -> None:
        if target == DemoControl.holding_filled_cup:
            if not user:
                actions.two_arm_motion_from_files(
                    self.left, self.right, "grasp_faucet.csv", "level_cup.csv"
                )
                actions.release(self.left_gripper)
                actions.motion_from_file(self.left, "pre_grasp_faucet.csv")
            else:
                actions.motion_from_file(self.right, "level_cup.csv")
                # TODO close faucet if user is false

    def on_place_cup(self) -> None:
        actions.motion_from_file(self.right, "place_cup.csv")
        actions.release(self.right_gripper)
        actions.motion_from_file(self.right, "post_place_cup.csv")
        # TODO place cup on tray

    def on_return_to_idle(self, target: statemachine.State) -> None:
        if target != DemoControl.waiting_for_user_pickup:
            actions.two_arm_motion_from_files(
                self.left, self.right, "left_idle.csv", "right_idle.csv"
            )
            # TODO return to idle pose

    def on_refill_cups(self) -> None:
        # TODO prompt user to refill cups
        input("Press enter to confirm cups refilled...")
        self.cups = self.max_cups

    def before_transition(self, event: str) -> None:
        log.info('Action "%s" triggered', event)

    def on_enter_state(self, state: statemachine.state.State) -> None:
        log.info('Entered state "%s"', state.id)

    def cup_available(self) -> bool:
        log.info("%d cups remaining", self.cups)
        return self.cups >= 1

    def measure_cup(self) -> None:
        print("testing")
        import time

        time.sleep(1.0 / 50)
        self.load = np.array(self.right.get_state().O_F_ext_hat_K)

    def cup_full(self) -> bool:
        print(np.linalg.norm(self.load[:3] - self.bias[:3]))

        return np.linalg.norm(self.load[:3] - self.bias[:3]) > 3.5

    def user_pickup(self) -> bool:
        log.info("Checking if user picked up cup...")
        return True

    def cup_grasped(self) -> bool:
        return self.right_gripper.read_once().width > 0.05 and self.right_gripper.grasp
