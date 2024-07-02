from __future__ import annotations

import logging

import statemachine  # type: ignore[import-not-found]

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
    open_faucet = holding_empty_cup.to(pouring)
    close_faucet = pouring.to(holding_filled_cup, cond="cup_full") | pouring.to(
        pouring, unless="cup_full"
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

    def __init__(self) -> None:
        self.cups = 12

    def on_pick_cup(self, target: statemachine.State, user: bool = False) -> None:
        del user
        if target != DemoControl.cups_empty:
            self.cups -= 1
            self.cups = max(0, self.cups)
            # TODO pick up cup, move to faucet, bias force sensor, grasp faucet if user False

    def on_open_faucet(self, user: bool = False) -> None:
        # TODO open faucet if user is false
        del user

    def on_close_faucet(self, target: statemachine.State, user: bool = False) -> None:
        del user
        if target == DemoControl.holding_filled_cup:
            pass
            # TODO close faucet if user is false

    def on_place_cup(self) -> None:
        # TODO place cup on tray
        pass

    def on_return_to_idle(self, target: statemachine.State) -> None:
        if target != DemoControl.waiting_for_user_pickup:
            # TODO return to idle pose
            pass

    def on_refill_cups(self) -> None:
        # TODO prompt user to refill cups
        input("Press enter to confirm cups refilled...")
        self.cups = 12

    def before_transition(self, event: str) -> None:
        log.info('Action "%s" triggered', event)

    def on_enter_state(self, state: statemachine.state.State) -> None:
        log.info('Entered state "%s"', state.id)

    def cup_available(self) -> bool:
        log.info("%d cups remaining", self.cups)
        return self.cups >= 1

    def cup_full(self) -> bool:
        log.info("Measuring cup...")
        return True

    def user_pickup(self) -> bool:
        log.info("Checking if user picked up cup...")
        return True
