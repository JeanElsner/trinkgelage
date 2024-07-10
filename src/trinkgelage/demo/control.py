from __future__ import annotations

import logging
import time
from xmlrpc import client

import numpy as np
import panda_py
import statemachine
from panda_py import libfranka

from ..robot import actions

log = logging.getLogger("trinkgelage")


class DemoControl(statemachine.StateMachine):  # type: ignore[misc]
    """Control flow program of the demo."""

    idle = statemachine.State(initial=True)
    start = statemachine.State()
    holding_empty_cup = statemachine.State()
    pouring = statemachine.State()
    holding_filled_cup = statemachine.State()
    cups_empty = statemachine.State()
    waiting_for_user_pickup = statemachine.State()

    start_demo = idle.to(start)

    pick_cup = start.to(holding_empty_cup, cond="cup_available") | start.to(
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

    def on_enter_start(self, user: bool = False) -> None:
        self.pick_cup(user=user)

    def on_enter_holding_empty_cup(self, user: bool = False) -> None:
        self.open_faucet(user=user)

    def on_enter_pouring(self, user: bool = True) -> None:
        self.close_faucet(user=user)

    def on_enter_holding_filled_cup(self) -> None:
        self.place_cup()

    def on_enter_waiting_for_user_pickup(self) -> None:
        self.return_to_idle()


class DemoModel:
    """Implements the demo actions."""

    max_cups = 12

    def __init__(
        self,
        left: str,
        right: str,
        enforce_rt: bool = True,
        gui_url: str = "http://garmi-gui.local:8000",
        start_position: int = 1,
    ) -> None:
        self.cups: int = np.clip(self.max_cups - (start_position - 1), 0, 12)
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
        self.gui = client.ServerProxy(gui_url)
        self.render_text_settings = (10, (0, 255, 255), 200)
        self.show_text_settings = ((0, 255, 255), 200)
        self.init_robot()

    def on_enter_idle(self) -> None:
        self.gui.show_image("sleep.png")

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
            log.info("Picking up cup at position %d", idx)
            self.gui.play_sound("confirm.wav")
            self.gui.render_text(
                f"Picking up cup at position #{idx}...", *self.render_text_settings
            )

            disp_x = (idx - 1) % 3 * 0.15
            disp_z = -np.floor((idx - 1) / 3) * 0.1

            q = actions.load_csv("grasp_cup_1.csv")
            grasp_cup = panda_py.fk(q)
            grasp_cup[0, 3] += disp_x
            grasp_cup[2, 3] += disp_z

            pre_grasp_cup = grasp_cup.copy()
            pre_grasp_cup[2, 3] += 0.15

            post_grasp_cup = grasp_cup.copy()
            post_grasp_cup[1, 3] += 0.15

            actions.move_to_pose(self.right, [pre_grasp_cup, grasp_cup])
            actions.grasp(self.right_gripper)
            actions.move_to_pose(self.right, post_grasp_cup)

            self.gui.show_image("eyes.png")

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
        else:
            self.gui.play_sound("attention.wav")
            self.gui.render_text("Please refill cups!", *self.render_text_settings)

    def on_open_faucet(self, target: statemachine.State, user: bool = False) -> None:
        if target == DemoControl.pouring:
            if not user:
                actions.grasp(self.left_gripper)
                actions.motion_from_file(self.left, "open_faucet.csv")
        else:
            self.gui.play_sound("failure.wav")
            self.gui.render_text("I'm not holding a cup...", *self.render_text_settings)
            actions.release(self.right_gripper)
            actions.two_arm_motion_from_files(
                self.left,
                self.right,
                ["pre_grasp_faucet.csv", "left_idle.csv"],
                ["post_place_cup.csv", "right_idle.csv"],
            )

    def on_close_faucet(self, target: statemachine.State, user: bool = False) -> None:
        if target == DemoControl.holding_filled_cup:
            self.gui.show_image("happy.png")
            self.gui.play_sound("success.wav")
            if not user:
                actions.two_arm_motion_from_files(
                    self.left, self.right, "grasp_faucet.csv", "level_cup.csv"
                )
                actions.release(self.left_gripper)
                actions.motion_from_file(self.left, "pre_grasp_faucet.csv")
            else:
                actions.motion_from_file(self.right, "level_cup.csv")

    def on_place_cup(self) -> None:
        actions.motion_from_file(self.right, "place_cup.csv")
        actions.release(self.right_gripper)
        actions.motion_from_file(self.right, "post_place_cup.csv")
        self.gui.play_sound("attention.wav")
        self.gui.render_text("Please retrieve your cup!", *self.render_text_settings)

    def on_return_to_idle(self, target: statemachine.State) -> None:
        if target != DemoControl.waiting_for_user_pickup:
            actions.two_arm_motion_from_files(
                self.left, self.right, "left_idle.csv", "right_idle.csv"
            )

    def on_refill_cups(self) -> None:
        self.cups = self.max_cups

    def before_transition(self, event: str) -> None:
        log.info('Action "%s" triggered', event)

    def on_enter_state(self, state: statemachine.state.State) -> None:
        log.info('Entered state "%s"', state.id)

    def cup_available(self) -> bool:
        log.info("%d cups remaining", self.cups)
        return self.cups >= 1

    def measure_cup(self) -> None:
        time.sleep(1.0 / 10)
        self.load = np.array(self.right.get_state().O_F_ext_hat_K)

    def cup_full(self) -> bool:
        load = np.linalg.norm(self.load[:3] - self.bias[:3])
        self.gui.show_text(f"measuring...\n{load*100:3.0f}ml", *self.show_text_settings)
        return bool(load > 3.5)

    def user_pickup(self) -> bool:
        time.sleep(5)
        return True

    def cup_grasped(self) -> bool:
        gripper_state = self.right_gripper.read_once()
        return gripper_state.width > 0.02 and gripper_state.is_grasped
