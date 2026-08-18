################### CAUTION ###################
# CAUTION:
# Ensure that the robot has enough surrounding clearance before running this example.
###############################################

# Gamepad Teleoperation (right arm, Cartesian delta)
# Streams Cartesian targets for the right arm, accumulating deltas from a gamepad.
# Place this file in examples/python/ so that `00_helper` resolves.
#
# Usage example:
#     python 90_gamepad_teleop.py --address localhost:50051 --model a
#     python 90_gamepad_teleop.py --probe          # print axis/button indices only
#
# Controls (defaults; verify with --probe):
#   Left stick   : EE x / y
#   Right stick X: wrist yaw (rotation about the tool z-axis)
#   L2 / R2      : EE z down / up
#   Ctrl+C       : stop

import argparse
import importlib
import logging
import signal
import time

import numpy as np
import pygame
import rby1_sdk as rby

helper = importlib.import_module("00_helper")
initialize_robot = helper.initialize_robot
movej = helper.movej

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

_DT = 0.05                  # 20 Hz stream
_MIN_TIME = 0.1             # slightly longer than _DT
_HOLD_TIME = 1e6            # keep control alive between streamed commands
_DEADZONE = 0.15

_LIN_SPEED = 0.15           # m/s at full stick deflection
_YAW_SPEED = 1.0            # rad/s at full stick deflection

# Cartesian command limits: linear vel, angular vel, acceleration
_LIN_VEL_LIMIT = 0.3
_ANG_VEL_LIMIT = 100.0
_ACC_LIMIT = 0.8

# Safety: clamp the target inside a box around the starting pose (metres)
_BOX = np.array([0.35, 0.35, 0.30])

# Axis indices — Pro Controller under hid-nintendo. Check with --probe.
_AX_LX, _AX_LY, _AX_RX = 0, 1, 2
_AX_L2, _AX_R2 = 4, 5


def _dz(v):
    """Apply deadzone and rescale so output stays continuous from 0."""
    return 0.0 if abs(v) < _DEADZONE else (v - np.sign(v) * _DEADZONE) / (1 - _DEADZONE)


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


def probe(pad):
    """Print live axis/button values so the mapping constants can be verified."""
    logging.info("Move sticks and press buttons. Ctrl+C to quit.")
    while True:
        pygame.event.pump()
        ax = [round(pad.get_axis(i), 2) for i in range(pad.get_numaxes())]
        btn = [i for i in range(pad.get_numbuttons()) if pad.get_button(i)]
        print(f"axes={ax}  buttons={btn}", end="\r", flush=True)
        time.sleep(0.05)


def move_to_ready(robot, robot_model):
    if not movej(
        robot,
        torso=None if robot_model.model_name == "UB" else np.deg2rad([0.0, 45.0, -90.0, 45.0, 0.0, 0.0]),
        right_arm=np.deg2rad([0.0, -5.0, 0.0, -120.0, 0.0, 40.0, 0.0]),
        left_arm=np.deg2rad([0.0, 5.0, 0.0, -120.0, 0.0, 40.0, 0.0]),
        minimum_time=4.0,
    ):
        exit(1)


def main(address, model, power, servo):
    pad = open_pad()

    robot = initialize_robot(address, model, power, servo)
    robot.set_parameter("cartesian_command.cutoff_frequency", "5")

    robot_model = robot.model()
    move_to_ready(robot, robot_model)

    # Forward kinematics ONCE — this becomes the accumulator, not a per-loop readback.
    dyn_robot = robot.get_dynamics()
    dyn_state = dyn_robot.make_state(["base", "ee_right"], robot_model.robot_joint_names)
    dyn_state.set_q(robot.get_state().position)
    dyn_robot.compute_forward_kinematics(dyn_state)
    T_target = dyn_robot.compute_transformation(dyn_state, 0, 1).copy()

    p0 = T_target[:3, 3].copy()   # box centre
    R0 = T_target[:3, :3].copy()  # orientation reference
    yaw = 0.0

    stream = robot.create_command_stream()

    running = [True]

    def _stop(signum, frame):
        logging.info("Ctrl+C — stopping.")
        running[0] = False

    signal.signal(signal.SIGINT, _stop)

    logging.info("Teleop active. Ctrl+C to stop.")

    while running[0]:
        pygame.event.pump()

        dx = -_dz(pad.get_axis(_AX_LY)) * _LIN_SPEED * _DT   # stick up = +x
        dy = -_dz(pad.get_axis(_AX_LX)) * _LIN_SPEED * _DT   # stick left = +y
        # triggers rest at -1.0 and travel to +1.0 → remap to 0..1
        up = (pad.get_axis(_AX_R2) + 1.0) / 2.0
        down = (pad.get_axis(_AX_L2) + 1.0) / 2.0
        dz = (up - down) * _LIN_SPEED * _DT
        dyaw = -_dz(pad.get_axis(_AX_RX)) * _YAW_SPEED * _DT

        # Accumulate into the target, then clamp to the safety box.
        T_target[:3, 3] += np.array([dx, dy, dz])
        T_target[:3, 3] = np.clip(T_target[:3, 3], p0 - _BOX, p0 + _BOX)

        yaw += dyaw
        T_target[:3, :3] = R0 @ rot_z(yaw)

        rc = rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.BodyComponentBasedCommandBuilder().set_right_arm_command(
                    rby.CartesianCommandBuilder()
                    .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(_HOLD_TIME))
                    .add_target("base", "ee_right", T_target, _LIN_VEL_LIMIT, _ANG_VEL_LIMIT, _ACC_LIMIT)
                    .set_minimum_time(_MIN_TIME)
                )
            )
        )
        stream.send_command(rc)

        time.sleep(_DT)

    stream.cancel()
    pygame.quit()
    logging.info("Done.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="90_gamepad_teleop")
    parser.add_argument("--address", type=str, help="Robot address")
    parser.add_argument("--model", type=str, default="a", help="Robot Model Name (default: 'a')")
    parser.add_argument("--power", type=str, default=".*", help="Power device name regex")
    parser.add_argument("--servo", type=str, default=".*", help="Servo name regex")
    parser.add_argument("--probe", action="store_true", help="Print axis/button indices and exit")
    args = parser.parse_args()

    if args.probe:
        probe(open_pad())
    else:
        if not args.address:
            parser.error("--address is required")
        main(args.address, args.model, args.power, args.servo)
