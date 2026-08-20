# Gamepad Helper Functions
# Everything that knows about the pad lives here: the button and axis mapping, the
# per-pad quirks, and the edge detection that turns a held control into a single intent.
# Callers get discrete intents out of Gamepad.poll() and never touch pygame themselves.
#
# The mapping constants are for an Xbox 360 pad under the xpad driver. Another pad needs
# different numbers -- run 90_gamepad_teleop.py --probe and edit the block below.

import logging
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import pygame

# ── Pad mapping ───────────────────────────────────────────────────────────────────────
# Axis indices. xpad reports six axes in the evdev order ABS_X ABS_Y ABS_Z ABS_RX ABS_RY
# ABS_RZ, i.e. LX LY LT RX RY RT. Note the left trigger sits between the two sticks:
# right stick X is axis 3, not 2.
# (Pro Controller under hid-nintendo has only four axes and wants 0, 1, 2 here.)
AX_LX, AX_LY, AX_RX = 0, 1, 3
# Analog triggers, when the pad exposes them (6-axis pads such as Xbox/DS4).
AX_L2, AX_R2 = 2, 5
# hid-nintendo reports ZL/ZR as digital buttons instead, with only 4 axes.
# Triggers are only a fallback for z -- the D-pad is preferred.
BTN_ZL, BTN_ZR = 7, 8

# D-pad. SDL usually exposes it as hat 0; some drivers report it as four buttons.
HAT_DPAD = 0
BTN_DPAD_UP, BTN_DPAD_DOWN = 11, 12

# Arm selection. L / R shoulder buttons -- NOT the ZL / ZR triggers above.
# The operator sits facing the robot, so their left is the robot's right: L selects the
# robot's RIGHT arm and R its LEFT. Holding one and pressing the other selects both.
# Xbox 360 pad: LB / RB. (Pro Controller wants 5, 6.)
BTN_L, BTN_R = 4, 5

# Capture button: right stick click on an Xbox 360 pad. (Pro Controller wants 13.) That
# pad exposes 11 buttons, 0..10, so an out-of-range index here silently does nothing
# rather than failing -- see button(). A stick click is safe for this: it is a plain
# digital button, entirely separate from the stick's axes, so it does not disturb yaw.
BTN_CAPTURE = 10

# Episode control, mirroring omy_collect_pick.py's Start / Back / A. These three are the
# buttons an Xbox 360 pad has spare once arm selection, capture and the triggers are
# spoken for: A=0, Back=6, Start=7.
BTN_START, BTN_BACK, BTN_A = 7, 6, 0

DEADZONE = 0.15

# Triggers rest at -1.0 and travel to +1.0 -- but a pad that has not been touched since it
# was opened can report 0.0 for an untouched trigger, so anything at or below zero has to
# count as released. A mid-scale threshold reads correctly either way, no calibration.
TRIGGER_ON = 0.5

SIDES = ("right", "left")


def deadzone(v):
    """Apply deadzone and rescale so output stays continuous from 0."""
    return 0.0 if abs(v) < DEADZONE else (v - np.sign(v) * DEADZONE) / (1 - DEADZONE)


def open_pad():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        logging.error("No gamepad found. Check /dev/input/js* and VMware USB passthrough.")
        exit(1)
    pad = pygame.joystick.Joystick(0)
    pad.init()
    logging.info("Gamepad: %s (%d axes, %d buttons)", pad.get_name(), pad.get_numaxes(), pad.get_numbuttons())
    return pad


def button(pad, index):
    """Button state, 0 when the pad does not expose that index."""
    return pad.get_button(index) if 0 <= index < pad.get_numbuttons() else 0


def axis(pad, index):
    """Axis value, 0.0 when the pad does not expose that index."""
    return float(pad.get_axis(index)) if 0 <= index < pad.get_numaxes() else 0.0


def make_z_reader(pad):
    """Return read() -> z in -1..1. D-pad up/down, falling back to the triggers."""
    if pad.get_numhats() > HAT_DPAD:
        logging.info("Z axis: D-pad hat %d (up/down)", HAT_DPAD)
        return lambda: float(pad.get_hat(HAT_DPAD)[1])

    if pad.get_numbuttons() > max(BTN_DPAD_UP, BTN_DPAD_DOWN):
        logging.info("Z axis: D-pad buttons %d/%d", BTN_DPAD_UP, BTN_DPAD_DOWN)
        return lambda: float(button(pad, BTN_DPAD_UP)) - float(button(pad, BTN_DPAD_DOWN))

    if pad.get_numaxes() > max(AX_L2, AX_R2):
        # Triggers rest at -1.0 and travel to +1.0 -> remap to 0..1.
        logging.info("Z axis: no D-pad found, using analog triggers %d/%d", AX_L2, AX_R2)
        return lambda: (pad.get_axis(AX_R2) - pad.get_axis(AX_L2)) / 2.0

    if pad.get_numbuttons() > max(BTN_ZL, BTN_ZR):
        logging.info("Z axis: no D-pad found, using trigger buttons %d/%d", BTN_ZL, BTN_ZR)
        return lambda: float(button(pad, BTN_ZR)) - float(button(pad, BTN_ZL))

    logging.error(
        "Gamepad exposes %d axes / %d buttons / %d hats — no usable z control. Run with --probe.",
        pad.get_numaxes(),
        pad.get_numbuttons(),
        pad.get_numhats(),
    )
    exit(1)


def probe(pad):
    """Print live axis/button values so the mapping constants can be verified."""
    logging.info("Move sticks and press buttons. Ctrl+C to quit.")
    while True:
        pygame.event.pump()
        ax = [round(pad.get_axis(i), 2) for i in range(pad.get_numaxes())]
        btn = [i for i in range(pad.get_numbuttons()) if pad.get_button(i)]
        hats = [pad.get_hat(i) for i in range(pad.get_numhats())]
        print(f"axes={ax}  buttons={btn}  hats={hats}      ", end="\r", flush=True)
        time.sleep(0.05)


def selection_name(sides):
    """Human-readable name for the selected arms."""
    return "BOTH arms" if len(sides) == 2 else f"the {sides[0].upper()} arm"


@dataclass
class PadInput:
    """One cycle of pad state, already reduced to intents.

    direction and yaw are None unless this cycle is the one where a fresh push began:
    holding the stick over does not repeat. The caller can therefore act on them
    unconditionally without tracking edges of its own.
    """

    selected: Tuple[str, ...]
    selection_changed: bool
    capture: bool
    start: bool
    back: bool
    clear: bool
    grip_toggle: Tuple[str, ...]
    direction: Optional[np.ndarray]
    yaw: Optional[float]

    @property
    def idle(self):
        """True when nothing is being jogged this cycle."""
        return self.direction is None and self.yaw is None


class Gamepad:
    """A pad read as discrete intents rather than raw axes.

    All the edge detection lives here. A control that is held down produces its intent
    once, on the cycle it was pressed, and stays quiet until released and pressed again.
    """

    def __init__(self, selected=("right",)):
        self.pad = open_pad()
        self._read_z = make_z_reader(self.pad)
        self.selected = tuple(selected)
        self._pressed = {"l": 0, "r": 0, "capture": 0, "lt": 0, "rt": 0,
                         "start": 0, "back": 0, "a": 0}
        # A push only fires once; the stick has to return to centre to fire again.
        self._engaged = {"move": False, "yaw": False}

    def poll(self):
        pygame.event.pump()
        # Ordered, not inlined into the constructor call: _poll_selection sets the
        # changed flag as a side effect, so it has to run before that flag is read.
        selected, changed = self._poll_selection()
        return PadInput(
            selected=selected,
            selection_changed=changed,
            capture=self._poll_edge(BTN_CAPTURE, "capture"),
            start=self._poll_edge(BTN_START, "start"),
            back=self._poll_edge(BTN_BACK, "back"),
            clear=self._poll_edge(BTN_A, "a"),
            grip_toggle=self._poll_triggers(),
            direction=self._poll_direction(),
            yaw=self._poll_yaw(),
        )

    def close(self):
        pygame.quit()

    # ── internals ────────────────────────────────────────────────────────────────────
    def _poll_selection(self):
        """L / R each pick one arm; pressing one while the other is still held picks both.

        The test is "edge, and what is held at that moment", so the two presses need not
        land in the same cycle -- either order, any gap, as long as the first is not
        released first. Releasing changes nothing, so both arms stay selected until some
        button is pressed again.
        """
        l_now, r_now = button(self.pad, BTN_L), button(self.pad, BTN_R)
        l_edge, r_edge = l_now and not self._pressed["l"], r_now and not self._pressed["r"]
        self._pressed["l"], self._pressed["r"] = l_now, r_now

        changed = False
        if l_edge or r_edge:
            if l_now and r_now:
                choice = ("right", "left")
            elif l_edge:
                choice = ("right",)
            else:
                choice = ("left",)
            if choice != self.selected:
                self.selected = choice
                changed = True
        return self.selected, changed

    def _poll_edge(self, index, key):
        now = button(self.pad, index)
        fired = bool(now and not self._pressed[key])
        self._pressed[key] = now
        return fired

    def _poll_triggers(self):
        """LT / RT, as the sides whose gripper the operator just asked to flip.

        The operator's left trigger drives the robot's right hand, the same crossing as
        L / R above.
        """
        toggled = []
        for side, ax, key in (("right", AX_L2, "lt"), ("left", AX_R2, "rt")):
            now = axis(self.pad, ax) > TRIGGER_ON
            if now and not self._pressed[key]:
                toggled.append(side)
            self._pressed[key] = now
        return tuple(toggled)

    def _poll_direction(self):
        """Unit vector for one fresh push, or None."""
        d = np.array(
            [
                deadzone(axis(self.pad, AX_LY)),   # stick up = -x, i.e. away from the operator
                deadzone(axis(self.pad, AX_LX)),   # stick left = -y, i.e. the operator's left
                self._read_z(),                    # D-pad up = +z
            ]
        )
        moving = bool(np.any(d))
        fresh = moving and not self._engaged["move"]
        self._engaged["move"] = moving
        return d / np.linalg.norm(d) if fresh else None

    def _poll_yaw(self):
        """+1 / -1 for one fresh push of the right stick, or None."""
        v = -deadzone(axis(self.pad, AX_RX))
        turning = v != 0.0
        fresh = turning and not self._engaged["yaw"]
        self._engaged["yaw"] = turning
        return float(np.sign(v)) if fresh else None
