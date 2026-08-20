################### CAUTION ###################
# CAUTION:
# Ensure that the robot has enough surrounding clearance before running this example.
###############################################

# Gamepad Teleoperation (Cartesian jog, either arm or both)
# Jogs the selected arm one fixed step per stick push. Each step is issued as a single
# Cartesian command seeded from the measured pose, so the arm moves only while a step
# is running and stops where it lands. With both arms selected the same step goes to
# both in one command, so they set off together.
# Place this file in examples/python/ so that `00_helper` resolves.
#
# Usage example:
#     python 90_gamepad_teleop.py                  # defaults: localhost:50051, model a
#     python 90_gamepad_teleop.py --address 192.168.30.1:50051 --model a
#     python 90_gamepad_teleop.py --probe          # print axis/button indices only
#     python 90_gamepad_teleop.py --gripper        # also drive the grippers (real robot only)
#     python 90_gamepad_teleop.py --fake-gripper   # log the toggles; nothing moves (sim)
#
# Controls (defaults; verify with --probe):
#   Left stick     : one 2 cm step in x / y per push (re-centre to step again)
#   D-pad up/down  : one 2 cm step in z per press
#   Right stick X  : one wrist-yaw step per push
#   L button       : control the RIGHT arm
#   R button       : control the LEFT arm
#   L + R together : control BOTH arms (hold one down, press the other)
#   R stick click  : print both arms' joint angles and end effector poses
#   LT / RT        : toggle the RIGHT / LEFT gripper open<->closed
#                    (--gripper on real hardware, --fake-gripper to log only)
#   Ctrl+C         : stop

import argparse
import importlib
import logging
import os
import signal
import sys
import threading
import time

import numpy as np
import pygame
import rby1_sdk as rby

# Resolve 00_helper no matter which directory the script is launched from.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

helper = importlib.import_module("00_helper")
initialize_robot = helper.initialize_robot
movej = helper.movej

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

_DEFAULT_ADDRESS = "localhost:50051"

_DT = 0.05                  # 20 Hz stream
_HOLD_TIME = 1e6            # keep control alive between streamed commands
_DEADZONE = 0.15

# Cartesian command limits: linear vel, angular vel, acceleration
_LIN_VEL_LIMIT = 0.3
_ANG_VEL_LIMIT = 100.0
_ACC_LIMIT = 0.8

# Elbow guard (see _ELBOW). A null-space joint target pins the elbow outright rather
# than merely biasing it -- the velocity argument is a rate limit, not a weight -- so
# applying it every cycle costs the arm the DOF it needs to reach a Cartesian target:
# measured on the simulator, a 2 cm step then falls 25% short and never converges. So
# engage it only once the elbow has actually straightened past _ELBOW_GUARD, where the
# singularity is the real risk.
_ELBOW_GUARD = -45.0    # deg; pull the elbow back once flexion is shallower than this
_ELBOW_LOCK_VEL = 1.0
_ELBOW_LOCK_ACC = 100.0

# Safety: clamp the target inside a box around the starting pose (metres)
_BOX = np.array([0.35, 0.35, 0.30])

# Jog steps. One push of the stick = one step, issued as a single Cartesian command and
# seeded from the measured pose so residual error cannot accumulate. Streaming a target
# every cycle instead restarts the trajectory generator each time, which caps the arm at
# about 2 cm/s and leaves it crawling long after the stick is centred -- measured on the
# simulator, 2 s of deflection left the target 23 cm ahead of the hand.
_STEP_SIZE = 0.02       # metres per push
_YAW_STEP = 0.10        # rad per push
_STEP_TIME = 0.3        # minimum_time for one step; 2 cm completes in ~0.38 s

# Axis indices — Xbox 360 pad under xpad, which reports six axes in the evdev order
# ABS_X ABS_Y ABS_Z ABS_RX ABS_RY ABS_RZ, i.e. LX LY LT RX RY RT. Note the left trigger
# sits between the two sticks: right stick X is axis 3, not 2. Check with --probe.
# (Pro Controller under hid-nintendo has only four axes and wants 0, 1, 2 here.)
_AX_LX, _AX_LY, _AX_RX = 0, 1, 3
# Analog triggers, when the pad exposes them (6-axis pads such as Xbox/DS4).
_AX_L2, _AX_R2 = 2, 5
# hid-nintendo reports ZL/ZR as digital buttons instead, with only 4 axes.
# Triggers are only a fallback for z now -- the D-pad is preferred.
_BTN_ZL, _BTN_ZR = 7, 8

# D-pad. SDL usually exposes it as hat 0; some drivers report it as four buttons.
_HAT_DPAD = 0
_BTN_DPAD_UP, _BTN_DPAD_DOWN = 11, 12

# Arm selection. L / R shoulder buttons -- NOT the ZL / ZR triggers above.
# The operator sits facing the robot, so their left is the robot's right: L selects the
# robot's RIGHT arm and R its LEFT, and both stick axes are mirrored to match -- pushing
# the stick away moves the hand away from the operator, which is the robot's -x.
# Holding one and pressing the other selects both arms; the mirroring above is a single
# operator-to-robot frame flip rather than anything per-arm, so one stick push means the
# same direction for both hands.
# Xbox 360 pad: LB / RB. (Pro Controller wants 5, 6.)
_BTN_L, _BTN_R = 4, 5

# Triggers rest at -1.0 and travel to +1.0 -- but a pad that has not been touched since
# it was opened can report 0.0 for an untouched trigger, so anything at or below zero has
# to count as released. A mid-scale threshold reads correctly either way, with no
# calibration pass.
_TRIGGER_ON = 0.5

# Capture button: dump both arms' joint angles and EE poses.
# Xbox 360 pad: right stick click. (Pro Controller wants 13.) The pad exposes 11 buttons,
# 0..10, so an out-of-range index here silently does nothing rather than failing -- see
# button(). A stick click is safe to use for this: it is a plain digital button, entirely
# separate from the stick's axes, so pressing it does not disturb the yaw reading.
_BTN_CAPTURE = 10


def _dz(v):
    """Apply deadzone and rescale so output stays continuous from 0."""
    return 0.0 if abs(v) < _DEADZONE else (v - np.sign(v) * _DEADZONE) / (1 - _DEADZONE)


def mat_to_quat(mat):
    """Rotation matrix -> quaternion (x, y, z, w), Shepperd's method.

    Branching on the largest diagonal term keeps this accurate for 180 deg rotations,
    where the naive trace formula divides by ~0.
    """
    t = mat[0, 0] + mat[1, 1] + mat[2, 2]
    if t > 0.0:
        r = np.sqrt(1.0 + t)
        s = 0.5 / r
        return np.array(
            [(mat[2, 1] - mat[1, 2]) * s, (mat[0, 2] - mat[2, 0]) * s, (mat[1, 0] - mat[0, 1]) * s, 0.5 * r]
        )
    i = int(np.argmax([mat[0, 0], mat[1, 1], mat[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    r = np.sqrt(1.0 + mat[i, i] - mat[j, j] - mat[k, k])
    s = 0.5 / r
    q = np.empty(4)
    q[i] = 0.5 * r
    q[j] = (mat[j, i] + mat[i, j]) * s
    q[k] = (mat[k, i] + mat[i, k]) * s
    q[3] = (mat[k, j] - mat[j, k]) * s
    return q


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def open_pad():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        logging.error("No gamepad found. Check /dev/input/js0 and VMware USB passthrough.")
        exit(1)
    pad = pygame.joystick.Joystick(0)
    pad.init()
    logging.info("Gamepad: %s (%d axes, %d buttons)", pad.get_name(), pad.get_numaxes(), pad.get_numbuttons())
    return pad


def button(pad, index):
    """Button state, 0 when the pad does not expose that index."""
    return pad.get_button(index) if 0 <= index < pad.get_numbuttons() else 0


def make_z_reader(pad):
    """Return read() -> z in -1..1. D-pad up/down, falling back to the triggers."""
    if pad.get_numhats() > _HAT_DPAD:
        logging.info("Z axis: D-pad hat %d (up/down)", _HAT_DPAD)
        return lambda: float(pad.get_hat(_HAT_DPAD)[1])

    if pad.get_numbuttons() > max(_BTN_DPAD_UP, _BTN_DPAD_DOWN):
        logging.info("Z axis: D-pad buttons %d/%d", _BTN_DPAD_UP, _BTN_DPAD_DOWN)
        return lambda: float(button(pad, _BTN_DPAD_UP)) - float(button(pad, _BTN_DPAD_DOWN))

    if pad.get_numaxes() > max(_AX_L2, _AX_R2):
        # Triggers rest at -1.0 and travel to +1.0 -> remap to 0..1.
        logging.info("Z axis: no D-pad found, using analog triggers %d/%d", _AX_L2, _AX_R2)
        return lambda: (pad.get_axis(_AX_R2) - pad.get_axis(_AX_L2)) / 2.0

    if pad.get_numbuttons() > max(_BTN_ZL, _BTN_ZR):
        logging.info("Z axis: no D-pad found, using trigger buttons %d/%d", _BTN_ZL, _BTN_ZR)
        return lambda: float(button(pad, _BTN_ZR)) - float(button(pad, _BTN_ZL))

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


# Grippers hang off the UPC's Dynamixel bus rather than the robot's command stream, so
# they are driven entirely separately from the arm motion below -- and only on real
# hardware, since the simulator has no such bus. Adapted from
# 35_leader_arm_teleop_with_monitor.py, which is the reference implementation.
#
# Dynamixel ID 0 is the right gripper and 1 the left, the order set_target expects.
_GRIP_IDS = [0, 1]
_GRIP_OPEN, _GRIP_CLOSED = 0.0, 1.0   # normalised; homing maps these onto the real travel
_GRIP_HOLD_TORQUE = 5.0
_GRIP_HOMING_TORQUE = 0.3
_GRIP_PERIOD = 0.1
_GRIP_SETTLED = 30      # identical encoder reads that mean a stop has been reached


class Gripper:
    """Both grippers on one Dynamixel bus, held at a normalised target by a worker thread.

    Homing is not optional. A normalised 0..1 target means nothing until the travel limits
    have actually been measured, and set_target refuses to act before then. Homing finds
    them by driving both grippers into each stop under current control, so keep hands and
    workpieces clear of the fingers while it runs.
    """

    def __init__(self):
        self.bus = rby.DynamixelBus(rby.upc.GripperDeviceName)
        self.bus.open_port()
        self.bus.set_baud_rate(2_000_000)
        self.bus.set_torque_constant([1, 1])
        self.min_q = np.array([np.inf, np.inf])
        self.max_q = np.array([-np.inf, -np.inf])
        self.target_q = None
        self._running = False
        self._thread = None

    def initialize(self):
        alive = True
        for dev_id in _GRIP_IDS:
            if not self.bus.ping(dev_id):
                logging.error("Gripper Dynamixel ID %d is not responding", dev_id)
                alive = False
        if alive:
            self.bus.group_sync_write_torque_enable([(i, 1) for i in _GRIP_IDS])
        return alive

    def set_operating_mode(self, mode):
        self.bus.group_sync_write_torque_enable([(i, 0) for i in _GRIP_IDS])
        self.bus.group_sync_write_operating_mode([(i, mode) for i in _GRIP_IDS])
        self.bus.group_sync_write_torque_enable([(i, 1) for i in _GRIP_IDS])

    def homing(self):
        """Drive to both stops and record the encoder extremes."""
        self.set_operating_mode(rby.DynamixelBus.CurrentControlMode)
        direction = 0
        q = np.array([0.0, 0.0])
        prev_q = np.array([0.0, 0.0])
        counter = 0
        while direction < 2:
            sign = 1 if direction == 0 else -1
            self.bus.group_sync_write_send_torque([(i, _GRIP_HOMING_TORQUE * sign) for i in _GRIP_IDS])
            rv = self.bus.group_fast_sync_read_encoder(_GRIP_IDS)
            if rv is not None:
                for dev_id, enc in rv:
                    q[dev_id] = enc
            self.min_q = np.minimum(self.min_q, q)
            self.max_q = np.maximum(self.max_q, q)
            # Deliberately not reset when the encoder does move: this counts stalled
            # reads in total, not consecutively, exactly as the reference does. A run of
            # 30 in a row would be the stricter test, but one stray encoder tick would
            # then restart it and leave homing leaning on the stops indefinitely.
            if np.array_equal(prev_q, q):
                counter += 1
            prev_q = q.copy()
            if counter >= _GRIP_SETTLED:
                direction += 1
                counter = 0
            time.sleep(_GRIP_PERIOD)

    def start(self):
        if self._thread is None or not self._thread.is_alive():
            self._running = True
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join()
            self._thread = None

    def _loop(self):
        self.set_operating_mode(rby.DynamixelBus.CurrentBasedPositionControlMode)
        self.bus.group_sync_write_send_torque([(i, _GRIP_HOLD_TORQUE) for i in _GRIP_IDS])
        while self._running:
            if self.target_q is not None:
                self.bus.group_sync_write_send_position(list(enumerate(self.target_q.tolist())))
            time.sleep(_GRIP_PERIOD)

    def set_target(self, normalized_q):
        if not np.isfinite(self.min_q).all() or not np.isfinite(self.max_q).all():
            logging.error("Gripper travel limits unknown -- homing has not run.")
            return
        n = np.clip(np.asarray(normalized_q, dtype=np.float64), 0.0, 1.0)
        # 1 is closed, so it maps to min_q: the same convention as the leader-arm teleop,
        # where squeezing the trigger harder closes the hand.
        self.target_q = (1 - n) * (self.max_q - self.min_q) + self.min_q


class FakeGripper:
    """Stand-in for a bus that is not there: logs the target instead of writing it.

    The simulator serves a 24-joint robot with no fingers in it and no gripper service of
    any kind, so nothing can move the hands there. This exists to exercise the trigger
    latch rather than the hardware: that one press toggles exactly once, that the two
    hands stay independent, and that the normalised target is what you expect.
    """

    def initialize(self):
        return True

    def homing(self):
        pass

    def start(self):
        pass

    def stop(self):
        pass

    def set_target(self, normalized_q):
        logging.info("[fake gripper] normalised target right/left = %s", list(normalized_q))


def open_grippers(fake=False):
    """Bring both grippers up, or return None so teleop can still run without them."""
    if fake:
        logging.info("Fake grippers: LT / RT toggle and log only, nothing moves.")
        return FakeGripper()
    try:
        gripper = Gripper()
        if not gripper.initialize():
            logging.error("Gripper servos did not answer -- trigger toggles disabled.")
            return None
        logging.info("Homing grippers -- keep clear, both hands travel to their stops.")
        gripper.homing()
        gripper.set_target([_GRIP_OPEN, _GRIP_OPEN])
        gripper.start()
        logging.info("Grippers ready, both open. LT toggles the RIGHT hand, RT the LEFT.")
        return gripper
    except Exception as e:
        # The bus lives on the UPC; in simulation it is simply not there. That is not a
        # reason to refuse to jog the arms.
        logging.error("Gripper bus unavailable (%s) -- trigger toggles disabled.", e)
        return None


# Starting arm pose: arms hanging down beside the body, with the end effector rotation
# left exactly at identity (aligned with the base frame).
#
# How far down the arms can hang is limited by the elbow, not by taste. A straight arm
# is a Cartesian singularity: measured on the simulator, arm manipulability falls from
# 0.028 at 60 deg of elbow flexion to 0.002 at 20 deg. Below roughly 30 deg the
# whole-body solver faults on the very first streamed target, the server closes the
# stream, and send_command raises "This command stream is expired".
#
# Pinning the EE rotation to identity makes this worse, because it spends the arm's
# redundancy: with the wrist orientation fixed, the solver straightens the elbow to
# reach a moved target. The null-space target in the loop below counteracts that.
# Verified on the simulator under a 10 cm Cartesian excursion: 60 deg holds with
# margin, 45 deg is the most vertical that survives, 30 deg faults.
_SHOULDER = 10.0  # arm_1 abduction (deg)
_ELBOW = 60.0     # arm_3 flexion (deg); lower = more vertical, 45 is the practical floor


def ready_arm(shoulder_deg, elbow_deg):
    """Arm pose for the given shoulder/elbow angles, with EE rotation at identity.

    arm_1 turns about x and arm_3 about y, so both tilt the hand. The arm_4/5/6 wrist
    is a ZYZ triple meeting at a single point, so it can absorb that tilt exactly:
    solving Rz(q4) Ry(q5) Rz(q6) = Ry(-q3) Rx(-q1) as a ZYZ Euler decomposition gives
    the wrist angles that bring the end effector back to zero rotation.
    """
    q1, q3 = np.deg2rad([shoulder_deg, elbow_deg])
    m = rot_y(-q3) @ rot_x(-q1)
    q4 = np.arctan2(m[1, 2], m[0, 2])
    q5 = np.arccos(np.clip(m[2, 2], -1.0, 1.0))
    q6 = np.arctan2(m[2, 1], -m[2, 0])
    return np.array([0.0, q1, 0.0, q3, q4, q5, q6])


_READY_RIGHT_ARM = ready_arm(-_SHOULDER, -_ELBOW)
_READY_LEFT_ARM = ready_arm(_SHOULDER, -_ELBOW)


def report_stream_loss(robot, error):
    """The server closes the stream when the control manager faults."""
    logging.error("Command stream stopped: %s", error)
    logging.error("Control manager state: %s", robot.get_control_manager_state().state)
    logging.error("Joint positions (deg): %s", np.round(np.rad2deg(robot.get_state().position), 1))


def selection_name(sides):
    """Human-readable name for the selected arms."""
    return "BOTH arms" if len(sides) == 2 else f"the {sides[0].upper()} arm"


def log_arm_poses(robot, robot_model, dyn_robot, dyn_state):
    """Print both arms' measured joint angles and end effector poses."""
    q = np.array(robot.get_state().position)
    dyn_state.set_q(q)
    dyn_robot.compute_forward_kinematics(dyn_state)
    names = list(robot_model.robot_joint_names)

    logging.info("---- capture ----")
    for side, link_idx in (("right", 1), ("left", 2)):
        joints = np.rad2deg(q[[names.index(f"{side}_arm_{i}") for i in range(7)]])
        t = dyn_robot.compute_transformation(dyn_state, 0, link_idx)
        quat = mat_to_quat(t[:3, :3])
        logging.info("  %-5s joints (deg) : %s", side, np.array2string(joints, precision=2, suppress_small=True))
        logging.info("  %-5s ee xyz (m)   : %s", side, np.array2string(t[:3, 3], precision=4, suppress_small=True))
        logging.info("  %-5s ee quat xyzw : %s", side, np.array2string(quat, precision=4, suppress_small=True))


def move_to_ready(robot, robot_model):
    if not movej(
        robot,
        torso=None if robot_model.model_name == "UB" else np.deg2rad([0.0, 45.0, -90.0, 45.0, 0.0, 0.0]),
        right_arm=_READY_RIGHT_ARM,
        left_arm=_READY_LEFT_ARM,
        minimum_time=4.0,
    ):
        exit(1)


def main(address, model, power, servo, gripper_mode):
    pad = open_pad()
    read_z = make_z_reader(pad)
    gripper = open_grippers(fake=gripper_mode == "fake") if gripper_mode else None

    robot = initialize_robot(address, model, power, servo)
    robot.set_parameter("cartesian_command.cutoff_frequency", "5")

    robot_model = robot.model()
    move_to_ready(robot, robot_model)

    dyn_robot = robot.get_dynamics()
    dyn_state = dyn_robot.make_state(["base", "ee_right", "ee_left"], robot_model.robot_joint_names)
    joint_names = list(robot_model.robot_joint_names)

    ready = {"right": _READY_RIGHT_ARM, "left": _READY_LEFT_ARM}
    link_of = {"right": 1, "left": 2}
    elbow_of = {side: joint_names.index(f"{side}_arm_3") for side in link_of}

    def measure():
        """Measured EE transform and elbow angle for both arms, from one state read."""
        q = robot.get_state().position
        dyn_state.set_q(q)
        dyn_robot.compute_forward_kinematics(dyn_state)
        return {
            side: (dyn_robot.compute_transformation(dyn_state, 0, idx).copy(), q[elbow_of[side]])
            for side, idx in link_of.items()
        }

    # Box centre and orientation reference are fixed at the ready pose; each step is
    # measured from where the hand actually is, so nothing accumulates between steps.
    origin = {side: {"p0": t[:3, 3].copy(), "R0": t[:3, :3].copy()} for side, (t, _) in measure().items()}

    def arm_command(side, target, elbow):
        cmd = rby.CartesianCommandBuilder().set_command_header(
            rby.CommandHeaderBuilder().set_control_hold_time(_HOLD_TIME)
        )
        # Elbow guard: only once the elbow has actually straightened, so a normal step
        # keeps the full DOF it needs to converge.
        if elbow > np.deg2rad(_ELBOW_GUARD):
            cmd = cmd.add_joint_position_target(
                f"{side}_arm_3", ready[side][3], _ELBOW_LOCK_VEL, _ELBOW_LOCK_ACC
            )
        return (
            cmd.add_target("base", f"ee_{side}", target, _LIN_VEL_LIMIT, _ANG_VEL_LIMIT, _ACC_LIMIT)
            .set_minimum_time(_STEP_TIME)
        )

    stream = robot.create_command_stream()

    running = [True]

    def _stop(signum, frame):
        logging.info("Ctrl+C — stopping.")
        running[0] = False

    signal.signal(signal.SIGINT, _stop)

    # Which arms take the jog. A tuple rather than a single side so that both can be
    # driven at once; the command below already addresses each arm separately, so the
    # only thing that changes with two selected is which targets actually move.
    selected = ("right",)
    # Keyed by control, not by arm: the dual-arm test needs the raw L / R states.
    pressed = {"l": 0, "r": 0, "capture": 0, "lt": 0, "rt": 0}
    # Gripper is a latch, not a live value: each trigger press flips one hand.
    grip_closed = {"right": False, "left": False}
    # A push only fires once; the stick has to return to centre to fire again.
    engaged = {"move": False, "yaw": False}
    logging.info(
        "Teleop active, controlling %s. %.0f cm per push. L / R to switch, L+R for both. Ctrl+C to stop.",
        selection_name(selected),
        _STEP_SIZE * 100,
    )

    while running[0]:
        pygame.event.pump()

        # L / R each pick one arm; pressing one while the other is still held picks both.
        # The test is "edge, and what is held at that moment", so the two presses need
        # not land in the same cycle -- either order, any gap, as long as the first is
        # not released first. Releasing changes nothing, so both arms stay selected until
        # some button is pressed again.
        l_now, r_now = button(pad, _BTN_L), button(pad, _BTN_R)
        l_edge, r_edge = l_now and not pressed["l"], r_now and not pressed["r"]
        if l_edge or r_edge:
            if l_now and r_now:
                choice = ("right", "left")
            elif l_edge:
                choice = ("right",)
            else:
                choice = ("left",)
            if choice != selected:
                selected = choice
                logging.info("Controlling %s.", selection_name(selected))
        pressed["l"], pressed["r"] = l_now, r_now

        now = button(pad, _BTN_CAPTURE)
        if now and not pressed["capture"]:
            log_arm_poses(robot, robot_model, dyn_robot, dyn_state)
        pressed["capture"] = now

        # LT / RT latch a gripper shut and the next press opens it again. The operator's
        # left trigger drives the robot's right hand, the same crossing as L / R above.
        # This sits ahead of the stick reading because the loop skips the rest of the
        # cycle when nothing is being jogged, and a toggle must not be skipped with it.
        for side, axis, key in (("right", _AX_L2, "lt"), ("left", _AX_R2, "rt")):
            now = float(pad.get_axis(axis)) > _TRIGGER_ON if pad.get_numaxes() > axis else False
            if now and not pressed[key] and gripper is not None:
                grip_closed[side] = not grip_closed[side]
                gripper.set_target(
                    [_GRIP_CLOSED if grip_closed[s] else _GRIP_OPEN for s in ("right", "left")]
                )
                logging.info("%s gripper %s", side.upper(), "closed" if grip_closed[side] else "open")
            pressed[key] = now

        # One step per push, on the edge where the stick leaves the deadzone.
        direction = np.array(
            [
                _dz(pad.get_axis(_AX_LY)),   # stick up = -x, i.e. away from the operator
                _dz(pad.get_axis(_AX_LX)),   # stick left = -y, i.e. the operator's left
                read_z(),                    # D-pad up = +z
            ]
        )
        yaw_input = -_dz(pad.get_axis(_AX_RX))

        moving = bool(np.any(direction))
        step = direction / np.linalg.norm(direction) * _STEP_SIZE if moving and not engaged["move"] else None
        engaged["move"] = moving

        turning = yaw_input != 0.0
        yaw_step = np.sign(yaw_input) * _YAW_STEP if turning and not engaged["yaw"] else None
        engaged["yaw"] = turning

        if step is None and yaw_step is None:
            # Nothing pushed: send nothing. Re-sending the target every cycle restarts
            # the trajectory generator and cripples the step, and polling the stream with
            # request_feedback() instead was observed to wedge it (the call blocks inside
            # gRPC holding the GIL, so even Ctrl+C stops working). control_hold_time keeps
            # both arms held meanwhile, and a fault surfaces on the next step.
            time.sleep(_DT)
            continue

        # Seed the step from the measured pose so residual error cannot accumulate.
        state = measure()
        targets = {}
        for side, (t_meas, _) in state.items():
            target = t_meas.copy()
            if side in selected:
                # An unselected arm is commanded to hold where it already is, so
                # switching arms never releases the one being left behind. With both
                # selected neither is held: the same step is applied to each, and the two
                # targets ride out in the one command below, so the arms set off together
                # rather than a cycle apart.
                if step is not None:
                    target[:3, 3] = t_meas[:3, 3] + step
                o = origin[side]
                target[:3, 3] = np.clip(target[:3, 3], o["p0"] - _BOX, o["p0"] + _BOX)
                rel = o["R0"].T @ t_meas[:3, :3]
                yaw = np.arctan2(rel[1, 0], rel[0, 0]) + (yaw_step if yaw_step is not None else 0.0)
                target[:3, :3] = o["R0"] @ rot_z(yaw)
            targets[side] = target

        body = rby.BodyComponentBasedCommandBuilder()
        body.set_right_arm_command(arm_command("right", targets["right"], state["right"][1]))
        body.set_left_arm_command(arm_command("left", targets["left"], state["left"][1]))
        rc = rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(body)
        )
        try:
            stream.send_command(rc)
        except RuntimeError as e:
            report_stream_loss(robot, e)
            break

        time.sleep(_DT)

    stream.cancel()
    if gripper is not None:
        gripper.stop()
    pygame.quit()
    logging.info("Done.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="90_gamepad_teleop")
    parser.add_argument(
        "--address", type=str, default=_DEFAULT_ADDRESS, help=f"Robot address (default: '{_DEFAULT_ADDRESS}')"
    )
    parser.add_argument("--model", type=str, default="a", help="Robot Model Name (default: 'a')")
    parser.add_argument("--power", type=str, default=".*", help="Power device name regex")
    parser.add_argument("--servo", type=str, default=".*", help="Servo name regex")
    parser.add_argument("--probe", action="store_true", help="Print axis/button indices and exit")
    parser.add_argument(
        "--gripper",
        action="store_true",
        help="Drive the grippers from LT/RT. Real robot only, and homes both hands at startup.",
    )
    parser.add_argument(
        "--fake-gripper",
        action="store_true",
        help="Log LT/RT gripper toggles without touching any bus. For the simulator, which has none.",
    )
    args = parser.parse_args()

    if args.probe:
        probe(open_pad())
    else:
        mode = "fake" if args.fake_gripper else ("real" if args.gripper else None)
        main(args.address, args.model, args.power, args.servo, mode)
