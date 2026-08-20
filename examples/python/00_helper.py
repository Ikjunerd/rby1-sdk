# Helper Functions
# This file contains helper functions for the examples.
# 1. initialize_robot: poweron, servo on, enable control manager.
# 2. movej: move robot to joint position.
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.


import rby1_sdk as rby
import logging
import threading
import time

import numpy as np


def initialize_robot(address, model, power=".*", servo=".*"):
    robot = rby.create_robot(address, model)
    if not robot.connect():
        logging.error(f"Failed to connect robot {address}")
        exit(1)
    if not robot.is_power_on(power):
        if not robot.power_on(power):
            logging.error(f"Failed to turn power ({power}) on")
            exit(1)
    if not robot.is_servo_on(servo):
        if not robot.servo_on(servo):
            logging.error(f"Failed to servo ({servo}) on")
            exit(1)
    if robot.get_control_manager_state().state in [
        rby.ControlManagerState.State.MajorFault,
        rby.ControlManagerState.State.MinorFault,
    ]:
        if not robot.reset_fault_control_manager():
            logging.error(f"Failed to reset control manager")
            exit(1)
    if not robot.enable_control_manager():
        logging.error(f"Failed to enable control manager")
        exit(1)
    return robot


def movej(robot, torso=None, right_arm=None, left_arm=None, minimum_time=0):
    rc = rby.BodyComponentBasedCommandBuilder()
    if torso is not None:
        rc.set_torso_command(
            rby.JointPositionCommandBuilder()
            .set_minimum_time(minimum_time)
            .set_position(torso)
        )
    if right_arm is not None:
        rc.set_right_arm_command(
            rby.JointPositionCommandBuilder()
            .set_minimum_time(minimum_time)
            .set_position(right_arm)
        )
    if left_arm is not None:
        rc.set_left_arm_command(
            rby.JointPositionCommandBuilder()
            .set_minimum_time(minimum_time)
            .set_position(left_arm)
        )

    rv = robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(rc)
        ),
        1,
    ).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        logging.error("Failed to conduct movej.")
        return False

    return True


# ══════════════════════════════════════════════════════════════════════════════════════
# Local additions (not upstream). Teleop support: rotations, ready pose, Cartesian
# jogging, and the grippers. Kept in one block at the end so that pulling from upstream
# conflicts here and nowhere else.
# ══════════════════════════════════════════════════════════════════════════════════════

# ── Rotations ─────────────────────────────────────────────────────────────────────────


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


# ── Ready pose ────────────────────────────────────────────────────────────────────────
# Arms hanging down beside the body, with the end effector rotation left exactly at
# identity (aligned with the base frame).
#
# How far down the arms can hang is limited by the elbow, not by taste. A straight arm is
# a Cartesian singularity: measured on the simulator, arm manipulability falls from 0.028
# at 60 deg of elbow flexion to 0.002 at 20 deg. Below roughly 30 deg the whole-body
# solver faults on the very first streamed target, the server closes the stream, and
# send_command raises "This command stream is expired".
#
# Pinning the EE rotation to identity makes this worse, because it spends the arm's
# redundancy: with the wrist orientation fixed, the solver straightens the elbow to reach
# a moved target. ArmJogger's null-space target counteracts that. Verified on the
# simulator under a 10 cm Cartesian excursion: 60 deg holds with margin, 45 deg is the
# most vertical that survives, 30 deg faults.
SHOULDER = 10.0  # arm_1 abduction (deg)
ELBOW = 60.0     # arm_3 flexion (deg); lower = more vertical, 45 is the practical floor


def ready_arm(shoulder_deg, elbow_deg):
    """Arm pose for the given shoulder/elbow angles, with EE rotation at identity.

    arm_1 turns about x and arm_3 about y, so both tilt the hand. The arm_4/5/6 wrist is a
    ZYZ triple meeting at a single point, so it can absorb that tilt exactly: solving
    Rz(q4) Ry(q5) Rz(q6) = Ry(-q3) Rx(-q1) as a ZYZ Euler decomposition gives the wrist
    angles that bring the end effector back to zero rotation.
    """
    q1, q3 = np.deg2rad([shoulder_deg, elbow_deg])
    m = rot_y(-q3) @ rot_x(-q1)
    q4 = np.arctan2(m[1, 2], m[0, 2])
    q5 = np.arccos(np.clip(m[2, 2], -1.0, 1.0))
    q6 = np.arctan2(m[2, 1], -m[2, 0])
    return np.array([0.0, q1, 0.0, q3, q4, q5, q6])


READY_RIGHT_ARM = ready_arm(-SHOULDER, -ELBOW)
READY_LEFT_ARM = ready_arm(SHOULDER, -ELBOW)


def move_to_ready(robot, robot_model, minimum_time=4.0):
    if not movej(
        robot,
        torso=None if robot_model.model_name == "UB" else np.deg2rad([0.0, 45.0, -90.0, 45.0, 0.0, 0.0]),
        right_arm=READY_RIGHT_ARM,
        left_arm=READY_LEFT_ARM,
        minimum_time=minimum_time,
    ):
        exit(1)


# ── Cartesian jogging ─────────────────────────────────────────────────────────────────
# Cartesian command limits: linear vel, angular vel, acceleration
LIN_VEL_LIMIT = 0.3
ANG_VEL_LIMIT = 100.0
ACC_LIMIT = 0.8

HOLD_TIME = 1e6         # keep control alive between streamed commands
STEP_TIME = 0.3         # minimum_time for one step; 2 cm completes in ~0.38 s

# Safety: clamp the target inside a box around the starting pose (metres)
BOX = np.array([0.35, 0.35, 0.30])

# Elbow guard. A null-space joint target pins the elbow outright rather than merely
# biasing it -- the velocity argument is a rate limit, not a weight -- so applying it
# every cycle costs the arm the DOF it needs to reach a Cartesian target: measured on the
# simulator, a 2 cm step then falls 25% short and never converges. So engage it only once
# the elbow has actually straightened past ELBOW_GUARD, where the singularity is the real
# risk.
ELBOW_GUARD = -45.0     # deg; pull the elbow back once flexion is shallower than this
ELBOW_LOCK_VEL = 1.0
ELBOW_LOCK_ACC = 100.0


class ArmJogger:
    """Cartesian jogging of both arms over one command stream.

    Every step is a single Cartesian command per arm, seeded from the measured pose, so
    residual error cannot accumulate. Streaming a target every cycle instead restarts the
    trajectory generator each time, which caps the arm at about 2 cm/s and leaves it
    crawling long after the stick is centred -- measured on the simulator, 2 s of
    deflection left the target 23 cm ahead of the hand. So call step() only when there is
    actually something to move; control_hold_time holds both arms in between.

    Arms not named in a step are commanded to hold where they already are, so handing
    control from one arm to the other never releases the one being left behind.
    """

    LINK_OF = {"right": 1, "left": 2}

    def __init__(self, robot, robot_model):
        self.robot = robot
        self.names = list(robot_model.robot_joint_names)
        self.dyn = robot.get_dynamics()
        self.dyn_state = self.dyn.make_state(["base", "ee_right", "ee_left"], robot_model.robot_joint_names)
        self.ready = {"right": READY_RIGHT_ARM, "left": READY_LEFT_ARM}
        self.elbow_of = {side: self.names.index(f"{side}_arm_3") for side in self.LINK_OF}
        # Box centre and orientation reference are fixed wherever the arms start; each
        # step is measured from where the hand actually is, so nothing accumulates.
        self.origin = {side: {"p0": t[:3, 3].copy(), "R0": t[:3, :3].copy()} for side, (t, _) in self.measure().items()}
        self.stream = robot.create_command_stream()

    def measure(self):
        """Measured EE transform and elbow angle for both arms, from one state read."""
        q = self.robot.get_state().position
        self.dyn_state.set_q(q)
        self.dyn.compute_forward_kinematics(self.dyn_state)
        return {
            side: (self.dyn.compute_transformation(self.dyn_state, 0, idx).copy(), q[self.elbow_of[side]])
            for side, idx in self.LINK_OF.items()
        }

    def step(self, sides, delta=None, yaw_delta=None):
        """Move the named arms by delta (m) and yaw_delta (rad). False if the stream died."""
        state = self.measure()
        targets = {}
        for side, (t_meas, _) in state.items():
            target = t_meas.copy()
            if side in sides:
                if delta is not None:
                    target[:3, 3] = t_meas[:3, 3] + delta
                o = self.origin[side]
                target[:3, 3] = np.clip(target[:3, 3], o["p0"] - BOX, o["p0"] + BOX)
                rel = o["R0"].T @ t_meas[:3, :3]
                yaw = np.arctan2(rel[1, 0], rel[0, 0]) + (yaw_delta or 0.0)
                target[:3, :3] = o["R0"] @ rot_z(yaw)
            targets[side] = target

        body = rby.BodyComponentBasedCommandBuilder()
        body.set_right_arm_command(self._arm_command("right", targets["right"], state["right"][1]))
        body.set_left_arm_command(self._arm_command("left", targets["left"], state["left"][1]))
        try:
            self.stream.send_command(
                rby.RobotCommandBuilder().set_command(
                    rby.ComponentBasedCommandBuilder().set_body_command(body)
                )
            )
        except RuntimeError as e:
            # The server closes the stream when the control manager faults.
            logging.error("Command stream stopped: %s", e)
            logging.error("Control manager state: %s", self.robot.get_control_manager_state().state)
            logging.error("Joint positions (deg): %s", np.round(np.rad2deg(self.robot.get_state().position), 1))
            return False
        return True

    def log_poses(self):
        """Print both arms' measured joint angles and end effector poses."""
        q = np.array(self.robot.get_state().position)
        self.dyn_state.set_q(q)
        self.dyn.compute_forward_kinematics(self.dyn_state)

        logging.info("---- capture ----")
        for side, link_idx in self.LINK_OF.items():
            joints = np.rad2deg(q[[self.names.index(f"{side}_arm_{i}") for i in range(7)]])
            t = self.dyn.compute_transformation(self.dyn_state, 0, link_idx)
            quat = mat_to_quat(t[:3, :3])
            logging.info("  %-5s joints (deg) : %s", side, np.array2string(joints, precision=2, suppress_small=True))
            logging.info("  %-5s ee xyz (m)   : %s", side, np.array2string(t[:3, 3], precision=4, suppress_small=True))
            logging.info("  %-5s ee quat xyzw : %s", side, np.array2string(quat, precision=4, suppress_small=True))

    def cancel(self):
        self.stream.cancel()

    def _arm_command(self, side, target, elbow):
        cmd = rby.CartesianCommandBuilder().set_command_header(
            rby.CommandHeaderBuilder().set_control_hold_time(HOLD_TIME)
        )
        # Elbow guard: only once the elbow has actually straightened, so a normal step
        # keeps the full DOF it needs to converge.
        if elbow > np.deg2rad(ELBOW_GUARD):
            cmd = cmd.add_joint_position_target(
                f"{side}_arm_3", self.ready[side][3], ELBOW_LOCK_VEL, ELBOW_LOCK_ACC
            )
        return cmd.add_target("base", f"ee_{side}", target, LIN_VEL_LIMIT, ANG_VEL_LIMIT, ACC_LIMIT).set_minimum_time(
            STEP_TIME
        )


# ── Grippers ──────────────────────────────────────────────────────────────────────────
# Grippers hang off the UPC's Dynamixel bus rather than the robot's command stream, so
# they are driven entirely separately from the arm motion above -- and only on real
# hardware, since the simulator has no such bus. Adapted from
# 35_leader_arm_teleop_with_monitor.py, which is the reference implementation.
#
# Dynamixel ID 0 is the right gripper and 1 the left, the order set_target expects.
GRIP_IDS = [0, 1]
GRIP_OPEN, GRIP_CLOSED = 0.0, 1.0   # normalised; homing maps these onto the real travel
GRIP_HOLD_TORQUE = 5.0
GRIP_HOMING_TORQUE = 0.3
GRIP_PERIOD = 0.1
GRIP_SETTLED = 30       # stalled encoder reads that mean a stop has been reached


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
        self.closed = {"right": False, "left": False}
        self._running = False
        self._thread = None

    def initialize(self):
        alive = True
        for dev_id in GRIP_IDS:
            if not self.bus.ping(dev_id):
                logging.error("Gripper Dynamixel ID %d is not responding", dev_id)
                alive = False
        if alive:
            self.bus.group_sync_write_torque_enable([(i, 1) for i in GRIP_IDS])
        return alive

    def set_operating_mode(self, mode):
        self.bus.group_sync_write_torque_enable([(i, 0) for i in GRIP_IDS])
        self.bus.group_sync_write_operating_mode([(i, mode) for i in GRIP_IDS])
        self.bus.group_sync_write_torque_enable([(i, 1) for i in GRIP_IDS])

    def homing(self):
        """Drive to both stops and record the encoder extremes."""
        self.set_operating_mode(rby.DynamixelBus.CurrentControlMode)
        direction = 0
        q = np.array([0.0, 0.0])
        prev_q = np.array([0.0, 0.0])
        counter = 0
        while direction < 2:
            sign = 1 if direction == 0 else -1
            self.bus.group_sync_write_send_torque([(i, GRIP_HOMING_TORQUE * sign) for i in GRIP_IDS])
            rv = self.bus.group_fast_sync_read_encoder(GRIP_IDS)
            if rv is not None:
                for dev_id, enc in rv:
                    q[dev_id] = enc
            self.min_q = np.minimum(self.min_q, q)
            self.max_q = np.maximum(self.max_q, q)
            # Deliberately not reset when the encoder does move: this counts stalled reads
            # in total, not consecutively, exactly as the reference does. A run of 30 in a
            # row would be the stricter test, but one stray encoder tick would then
            # restart it and leave homing leaning on the stops indefinitely.
            if np.array_equal(prev_q, q):
                counter += 1
            prev_q = q.copy()
            if counter >= GRIP_SETTLED:
                direction += 1
                counter = 0
            time.sleep(GRIP_PERIOD)

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

    def toggle(self, side):
        """Flip one hand between open and closed, and send the pair."""
        self.closed[side] = not self.closed[side]
        self.apply()
        logging.info("%s gripper %s", side.upper(), "closed" if self.closed[side] else "open")

    def apply(self):
        self.set_target([GRIP_CLOSED if self.closed[s] else GRIP_OPEN for s in ("right", "left")])

    def set_target(self, normalized_q):
        if not np.isfinite(self.min_q).all() or not np.isfinite(self.max_q).all():
            logging.error("Gripper travel limits unknown -- homing has not run.")
            return
        n = np.clip(np.asarray(normalized_q, dtype=np.float64), 0.0, 1.0)
        # 1 is closed, so it maps to min_q: the same convention as the leader-arm teleop,
        # where squeezing the trigger harder closes the hand.
        self.target_q = (1 - n) * (self.max_q - self.min_q) + self.min_q

    def _loop(self):
        self.set_operating_mode(rby.DynamixelBus.CurrentBasedPositionControlMode)
        self.bus.group_sync_write_send_torque([(i, GRIP_HOLD_TORQUE) for i in GRIP_IDS])
        while self._running:
            if self.target_q is not None:
                self.bus.group_sync_write_send_position(list(enumerate(self.target_q.tolist())))
            time.sleep(GRIP_PERIOD)


class FakeGripper:
    """Stand-in for a bus that is not there: logs the target instead of writing it.

    The simulator serves a 24-joint robot with no fingers in it and no gripper service of
    any kind, so nothing can move the hands there. This exists to exercise the caller's
    latch rather than the hardware.
    """

    def __init__(self):
        self.closed = {"right": False, "left": False}

    def initialize(self):
        return True

    def homing(self):
        pass

    def start(self):
        pass

    def stop(self):
        pass

    def toggle(self, side):
        self.closed[side] = not self.closed[side]
        self.apply()
        logging.info("%s gripper %s", side.upper(), "closed" if self.closed[side] else "open")

    def apply(self):
        logging.info("[fake gripper] normalised target right/left = %s",
                     [GRIP_CLOSED if self.closed[s] else GRIP_OPEN for s in ("right", "left")])


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
        gripper.apply()
        gripper.start()
        logging.info("Grippers ready, both open. LT toggles the RIGHT hand, RT the LEFT.")
        return gripper
    except Exception as e:
        # The bus lives on the UPC; in simulation it is simply not there. That is not a
        # reason to refuse to jog the arms.
        logging.error("Gripper bus unavailable (%s) -- trigger toggles disabled.", e)
        return None
