################### CAUTION ###################
# CAUTION:
# Ensure that the robot has enough surrounding clearance before running this example.
###############################################

# Gamepad Teleoperation (Cartesian jog, either arm)
# Jogs the selected arm one fixed step per stick push. Each step is issued as a single
# Cartesian command seeded from the measured pose, so the arm moves only while a step
# is running and stops where it lands.
# Place this file in examples/python/ so that `00_helper` resolves.
#
# Usage example:
#     python 90_gamepad_teleop.py                  # defaults: localhost:50051, model a
#     python 90_gamepad_teleop.py --address 192.168.30.1:50051 --model a
#     python 90_gamepad_teleop.py --probe          # print axis/button indices only
#
# Controls (defaults; verify with --probe):
#   Left stick     : one 2 cm step in x / y per push (re-centre to step again)
#   D-pad up/down  : one 2 cm step in z per press
#   Right stick X  : one wrist-yaw step per push
#   L button       : control the RIGHT arm
#   R button       : control the LEFT arm
#   Capture button : print both arms' joint angles and end effector poses
#   Ctrl+C         : stop

import argparse
import importlib
import logging
import os
import signal
import sys
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

# Axis indices — Pro Controller under hid-nintendo. Check with --probe.
_AX_LX, _AX_LY, _AX_RX = 0, 1, 2
# Analog triggers, when the pad exposes them (6-axis pads such as Xbox/DS4).
_AX_L2, _AX_R2 = 4, 5
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
_BTN_L, _BTN_R = 5, 6

# Capture ("screenshot") button: dump both arms' joint angles and EE poses.
_BTN_CAPTURE = 13


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


def main(address, model, power, servo):
    pad = open_pad()
    read_z = make_z_reader(pad)

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

    selected = "right"
    pressed = {"left": 0, "right": 0, "capture": 0}
    # A push only fires once; the stick has to return to centre to fire again.
    engaged = {"move": False, "yaw": False}
    logging.info(
        "Teleop active, controlling the %s arm. %.0f cm per push. L / R to switch. Ctrl+C to stop.",
        selected.upper(),
        _STEP_SIZE * 100,
    )

    while running[0]:
        pygame.event.pump()

        for side, btn in (("right", _BTN_L), ("left", _BTN_R)):
            now = button(pad, btn)
            if now and not pressed[side] and side != selected:
                selected = side
                logging.info("Controlling the %s arm.", selected.upper())
            pressed[side] = now

        now = button(pad, _BTN_CAPTURE)
        if now and not pressed["capture"]:
            log_arm_poses(robot, robot_model, dyn_robot, dyn_state)
        pressed["capture"] = now

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
            if side == selected:
                # The unselected arm is commanded to hold where it already is, so
                # switching arms never releases the one being left behind.
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
    args = parser.parse_args()

    if args.probe:
        probe(open_pad())
    else:
        main(args.address, args.model, args.power, args.servo)
