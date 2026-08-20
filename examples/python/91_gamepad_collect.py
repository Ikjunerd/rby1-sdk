################### CAUTION ###################
# CAUTION:
# Ensure that the robot has enough surrounding clearance before running this example.
###############################################

# Gamepad Teleoperation with Data Collection
# The same teleop as 90_gamepad_teleop.py, with recording and the grippers already
# switched on and the real robot's address, dataset path and camera serials filled in.
# Nothing here that 90_ cannot do from the command line -- this is that command line,
# kept in a file so an episode starts with one word.
#
# Everything stays overridable, so the same script serves the simulator:
#     python 91_gamepad_collect.py                              # real robot, as configured
#     python 91_gamepad_collect.py --address localhost:50051 --fake-camera --no-gripper
#
# Controls: as 90_gamepad_teleop.py. Recording is on Start -- press once to open an
# episode, again to close it and write both arms to --dataset.

import argparse
import importlib
import logging
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

collect = importlib.import_module("00_helper_collect")
teleop = importlib.import_module("90_gamepad_teleop")

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

# ── Site configuration ────────────────────────────────────────────────────────────────
# The robot this script is meant for. Override with --address to drive the simulator.
ADDRESS = "192.168.30.1:50051"
MODEL = "a"

DATASET = os.path.expanduser("~/dataset")

# One serial per gripper camera, from `rs.context().devices` -- NOT the serial sysfs
# shows for the same device, which differs.
#
# Both sides currently name the one camera on the bench, which shares it: each arm's
# dataset then holds the same pictures. Give the second D405 its own serial here before
# recording anything that is meant to be trained on.
CAMERA_RIGHT = "409122271622"
CAMERA_LEFT = "409122271622"


def main():
    parser = argparse.ArgumentParser(description="91_gamepad_collect")
    parser.add_argument("--address", type=str, default=ADDRESS, help=f"Robot address (default: '{ADDRESS}')")
    parser.add_argument("--model", type=str, default=MODEL, help=f"Robot model (default: '{MODEL}')")
    parser.add_argument("--power", type=str, default=".*", help="Power device name regex")
    parser.add_argument("--servo", type=str, default=".*", help="Servo name regex")
    parser.add_argument("--dataset", type=str, default=DATASET, help=f"Dataset root (default: '{DATASET}')")
    parser.add_argument("--camera-right", type=str, default=CAMERA_RIGHT, help="Right gripper camera serial")
    parser.add_argument("--camera-left", type=str, default=CAMERA_LEFT, help="Left gripper camera serial")
    parser.add_argument("--fake-camera", action="store_true", help="Synthetic frames instead of the D405s")
    parser.add_argument("--cam-width", type=int, default=collect.CAM_WIDTH)
    parser.add_argument("--cam-height", type=int, default=collect.CAM_HEIGHT)
    parser.add_argument("--cam-fps", type=int, default=collect.CAM_FPS)
    parser.add_argument(
        "--no-gripper",
        action="store_true",
        help="Skip the gripper bus. Use in simulation, where there is none, to drop the startup homing.",
    )
    args = parser.parse_args()

    if args.camera_right == args.camera_left and not args.fake_camera:
        logging.warning("Both arms are set to camera %s -- they will record the same view.", args.camera_right)

    collect_args = {
        "serials": {"right": args.camera_right, "left": args.camera_left},
        "fake": args.fake_camera,
        "dataset": args.dataset,
        "width": args.cam_width,
        "height": args.cam_height,
        "fps": args.cam_fps,
    }
    teleop.main(
        args.address,
        args.model,
        args.power,
        args.servo,
        None if args.no_gripper else "real",
        collect_args,
    )


if __name__ == "__main__":
    main()
