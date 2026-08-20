################### CAUTION ###################
# CAUTION:
# Ensure that the robot has enough surrounding clearance before running this example.
###############################################

# Gamepad Teleoperation (Cartesian jog, either arm or both)
# Jogs the selected arm one fixed step per stick push. Each step is issued as a single
# Cartesian command seeded from the measured pose, so the arm moves only while a step is
# running and stops where it lands. With both arms selected the same step goes to both in
# one command, so they set off together.
#
# The pad mapping and its edge detection live in 00_helper_joystick; the robot motion and
# the grippers live in 00_helper. What is left here is the loop that joins them.
#
# Those two helpers are resolved from this file's own directory, not the working one, so
# the three files run from anywhere as long as they sit together. Copying them into a
# folder of their own is the tidiest way to carry the teleop to another machine -- it
# leaves that machine's SDK examples, 00_helper.py included, untouched.
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
import time

# Resolve the helpers no matter which directory the script is launched from.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

helper = importlib.import_module("00_helper")
joystick = importlib.import_module("00_helper_joystick")

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

_DEFAULT_ADDRESS = "localhost:50051"

_DT = 0.05          # 20 Hz stream
_STEP_SIZE = 0.02   # metres per push
_YAW_STEP = 0.10    # rad per push


def main(address, model, power, servo, gripper_mode):
    pad = joystick.Gamepad()
    gripper = helper.open_grippers(fake=gripper_mode == "fake") if gripper_mode else None

    robot = helper.initialize_robot(address, model, power, servo)
    robot.set_parameter("cartesian_command.cutoff_frequency", "5")
    robot_model = robot.model()
    helper.move_to_ready(robot, robot_model)

    jogger = helper.ArmJogger(robot, robot_model)

    running = [True]

    def _stop(signum, frame):
        logging.info("Ctrl+C — stopping.")
        running[0] = False

    signal.signal(signal.SIGINT, _stop)

    logging.info(
        "Teleop active, controlling %s. %.0f cm per push. L / R to switch, L+R for both. Ctrl+C to stop.",
        joystick.selection_name(pad.selected),
        _STEP_SIZE * 100,
    )

    while running[0]:
        pad_input = pad.poll()

        if pad_input.selection_changed:
            logging.info("Controlling %s.", joystick.selection_name(pad_input.selected))

        if pad_input.capture:
            jogger.log_poses()

        for side in pad_input.grip_toggle:
            if gripper is not None:
                gripper.toggle(side)

        # Nothing pushed: send nothing. Re-sending the target every cycle restarts the
        # trajectory generator and cripples the step, and polling the stream with
        # request_feedback() instead was observed to wedge it (the call blocks inside gRPC
        # holding the GIL, so even Ctrl+C stops working). control_hold_time keeps both arms
        # held meanwhile, and a fault surfaces on the next step.
        if not pad_input.idle:
            delta = None if pad_input.direction is None else pad_input.direction * _STEP_SIZE
            yaw = None if pad_input.yaw is None else pad_input.yaw * _YAW_STEP
            if not jogger.step(pad_input.selected, delta, yaw):
                break

        time.sleep(_DT)

    jogger.cancel()
    if gripper is not None:
        gripper.stop()
    pad.close()
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
        joystick.probe(joystick.open_pad())
    else:
        mode = "fake" if args.fake_gripper else ("real" if args.gripper else None)
        main(args.address, args.model, args.power, args.servo, mode)
