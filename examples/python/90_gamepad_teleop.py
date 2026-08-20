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
#     python 90_gamepad_teleop.py --collect --camera-right 409122271622
#     python 90_gamepad_teleop.py --collect --fake-camera   # sim: synthetic frames
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
#   Start          : begin recording; press again to end the episode and save
#   Back           : stop recording, discard, return to the ready pose
#   A              : discard what has been recorded so far
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
collect = importlib.import_module("00_helper_collect")

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

_DEFAULT_ADDRESS = "localhost:50051"

_DT = 0.05          # 20 Hz stream
_STEP_SIZE = 0.02   # metres per push
_YAW_STEP = 0.10    # rad per push


def main(address, model, power, servo, gripper_mode, collect_args):
    pad = joystick.Gamepad()
    gripper = helper.open_grippers(fake=gripper_mode == "fake") if gripper_mode else None

    robot = helper.initialize_robot(address, model, power, servo)
    robot.set_parameter("cartesian_command.cutoff_frequency", "5")
    robot_model = robot.model()
    helper.move_to_ready(robot, robot_model)

    jogger = helper.ArmJogger(robot, robot_model)

    cameras, collector, recording = {}, None, False
    if collect_args is not None:
        cameras = collect.open_cameras(
            collect_args["serials"], fake=collect_args["fake"],
            width=collect_args["width"], height=collect_args["height"], fps=collect_args["fps"],
        )
        collector = collect.ArmCollector(robot, robot_model, cameras, gripper, collect_args["dataset"])
        logging.info("Recording armed. Start = begin/end episode, Back = discard, A = clear.")

    running = [True]

    def _stop(signum, frame):
        logging.info("Ctrl+C — stopping.")
        running[0] = False

    signal.signal(signal.SIGINT, _stop)

    def return_to_ready():
        """Reset between episodes. The jog stream has to let go first.

        movej is a one-shot command and the stream holds the arms with a long
        control_hold_time, so a reset issued while the stream is open is simply refused.
        Hand control back, move, then take it again.
        """
        jogger.cancel()
        if not helper.move_to_ready(robot, robot_model, fatal=False):
            logging.error("Ready pose refused -- the arms are where you left them.")
        jogger.reopen()

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

        # Episode control, same meaning as omy_collect_pick.py: the first Start opens an
        # episode and the second closes and writes it.
        if collector is not None:
            if pad_input.start:
                if not recording:
                    collector.start()
                    recording = True
                else:
                    collector.stop()
                    collector.save()
                    recording = False
                    return_to_ready()
            if pad_input.back:
                if recording:
                    collector.stop()
                    recording = False
                collector.clear()
                return_to_ready()
            if pad_input.clear:
                if recording:
                    collector.stop()
                    recording = False
                collector.clear()

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

    if collector is not None and recording:
        collector.stop()
    # dict(...) by identity: one camera can be shared by both sides, and stopping a
    # pipeline twice raises.
    for cam in {id(c): c for c in cameras.values()}.values():
        cam.stop()
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
    parser.add_argument("--collect", action="store_true", help="Record episodes to --dataset")
    parser.add_argument("--dataset", type=str, default="dataset", help="Dataset root (default: ./dataset)")
    parser.add_argument("--camera-right", type=str, default=None, help="Serial of the right gripper's D405")
    parser.add_argument("--camera-left", type=str, default=None, help="Serial of the left gripper's D405")
    parser.add_argument("--fake-camera", action="store_true", help="Synthetic frames instead of a D405")
    parser.add_argument("--cam-width", type=int, default=collect.CAM_WIDTH)
    parser.add_argument("--cam-height", type=int, default=collect.CAM_HEIGHT)
    parser.add_argument("--cam-fps", type=int, default=collect.CAM_FPS)
    args = parser.parse_args()

    if args.probe:
        joystick.probe(joystick.open_pad())
    else:
        mode = "fake" if args.fake_gripper else ("real" if args.gripper else None)
        collect_args = None
        if args.collect:
            # Only sides that were given a camera get one. With a single camera on the
            # bench, name the arm it is mounted on; the other arm still records poses.
            serials = {}
            if args.fake_camera:
                serials = {"right": None, "left": None}
            else:
                if args.camera_right is not None:
                    serials["right"] = args.camera_right or None
                if args.camera_left is not None:
                    serials["left"] = args.camera_left or None
            collect_args = {
                "serials": serials, "fake": args.fake_camera, "dataset": args.dataset,
                "width": args.cam_width, "height": args.cam_height, "fps": args.cam_fps,
            }
        main(args.address, args.model, args.power, args.servo, mode, collect_args)
