# Data Collection Helper Functions
# Records one episode per arm: the gripper camera's frames, the end effector pose, the
# gripper opening, and the times each of those was taken. Each arm is written to its own
# directory in the layout omy_collect_pick.py produces, so an arm's dataset is complete on
# its own and the two are never interleaved.
#
# Cameras are read straight through pyrealsense2. No ROS: the D405s hang off the control
# PC by USB, and a topic would only add a hop.

import importlib
import os
import sys
import threading
import time
from datetime import datetime

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
helper = importlib.import_module("00_helper")

# pyrealsense2 is only needed for real cameras. Importing it lazily keeps this module
# usable on a machine that has none -- the simulator path never touches it.
try:
    import pyrealsense2 as rs
except ImportError:
    rs = None

# Only used to label the synthetic frames; the collector itself needs no OpenCV.
try:
    import cv2
except ImportError:
    cv2 = None

SIDES = ("right", "left")

# Measured on one D405 over USB 3.0: 424x240@15, 848x480@30 and even 1280x720@30 all
# run without dropping a frame. Two cameras share the bus on the real robot, though, so
# confirm with 92_camera_check.py there before raising this further.
#
# Not every size is a real profile -- the device advertises a fixed list, and asking for
# one that is not on it fails at pipeline start. 92_camera_check.py prints the list.
CAM_WIDTH, CAM_HEIGHT, CAM_FPS = 640, 480, 30

# Warm-up before the camera counts as started. A D405 takes colour from its stereo
# imager and its auto-exposure drifts for a while after the pipeline opens -- measured on
# one D405 at 640x480@30, the frame mean was still climbing at 2 s and only levelled off
# near 2.8 s. Discarding a fixed frame count is the wrong shape for that, since how long
# it takes depends on the scene and the frame rate. Watch the brightness instead and stop
# when it stops moving, with a ceiling so a genuinely dark scene cannot stall startup.
CAM_WARMUP_MAX_S = 3.5
CAM_WARMUP_STABLE = 4       # consecutive frames within CAM_WARMUP_EPS
CAM_WARMUP_EPS = 0.5        # mean brightness change that counts as "still settling"
CAM_DARK_MEAN = 2.0         # below this the picture is black, not merely dim

# Sampling. Same knobs as the reference: the loop ticks every COLLECT_SLEEP and takes
# every COLLECT_INTERVAL-th tick, so the default is ~20 Hz.
COLLECT_INTERVAL = 5
COLLECT_SLEEP = 0.01

DATASET_DIRNAME = {"right": "rby-right-arm", "left": "rby-left-arm"}


class RealSenseCamera:
    """One D405, read by a background thread into a single latest frame.

    Bind by serial, never by enumeration order: which camera comes up first changes
    between boots, and a silently swapped pair would mislabel every episode. Note the
    serial librealsense reports is not the one sysfs shows for the same device -- this
    wants the librealsense one.
    """

    def __init__(self, serial=None, width=CAM_WIDTH, height=CAM_HEIGHT, fps=CAM_FPS, label="camera"):
        if rs is None:
            raise RuntimeError("pyrealsense2 is not installed")
        self.serial = serial
        self.label = label
        self.width, self.height, self.fps = width, height, fps
        self._pipe = None
        self._frame = None          # (image, device_ts_ms, wall_time)
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    def start(self):
        cfg = rs.config()
        if self.serial:
            cfg.enable_device(self.serial)
        cfg.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        self._pipe = rs.pipeline()
        profile = self._pipe.start(cfg)
        dev = profile.get_device()
        self.serial = dev.get_info(rs.camera_info.serial_number)
        helper.logging.info(
            "Camera %s: %s serial %s, %dx%d@%d",
            self.label, dev.get_info(rs.camera_info.name), self.serial, self.width, self.height, self.fps
        )
        self._warmup()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name=f"cam-{self.label}")
        self._thread.start()

    def _warmup(self):
        """Pull frames until the exposure stops moving, so nothing records the ramp."""
        deadline = time.time() + CAM_WARMUP_MAX_S
        prev, stable, mean = None, 0, None
        while time.time() < deadline and stable < CAM_WARMUP_STABLE:
            try:
                c = self._pipe.wait_for_frames(2000).get_color_frame()
            except RuntimeError:
                break
            if not c:
                continue
            mean = float(np.asanyarray(c.get_data()).mean())
            stable = stable + 1 if prev is not None and abs(mean - prev) < CAM_WARMUP_EPS else 0
            prev = mean
        if mean is None:
            helper.logging.warning("Camera %s: no frames during warm-up.", self.label)
        elif mean < CAM_DARK_MEAN:
            helper.logging.warning(
                "Camera %s: frames are almost black after warm-up (mean %.1f). Check the lighting "
                "and that nothing is covering the lens -- run 92_camera_check.py.", self.label, mean)
        else:
            helper.logging.info("Camera %s: exposure settled, frame mean %.1f", self.label, mean)

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._pipe is not None:
            self._pipe.stop()
            self._pipe = None

    @property
    def latest(self):
        """(image, device_ts_ms, wall_time), or (None, None, None) before the first frame."""
        with self._lock:
            return self._frame if self._frame is not None else (None, None, None)

    def _loop(self):
        while self._running:
            try:
                frames = self._pipe.wait_for_frames(2000)
            except RuntimeError as e:
                helper.logging.warning("Camera %s: %s", self.label, e)
                continue
            color = frames.get_color_frame()
            if not color:
                continue
            # Copy: the frame's buffer is recycled once this reference is dropped, so a
            # view handed to the collector would change under it.
            img = np.asanyarray(color.get_data()).copy()
            with self._lock:
                self._frame = (img, color.get_timestamp(), time.time())


class DummyCamera:
    """Synthetic frames, for exercising the collector without hardware.

    Deliberately loud. An earlier version wrote a frame counter into a corner of an
    otherwise black image, and an episode recorded with it was indistinguishable at a
    glance from one shot with the lens covered -- it cost a real debugging session to
    tell the two apart. So these frames say what they are, in the picture, where anyone
    scrubbing the data will see it. The counter is still there for checking drops.
    """

    def __init__(self, width=CAM_WIDTH, height=CAM_HEIGHT, fps=CAM_FPS, label="dummy"):
        self.width, self.height, self.fps = width, height, fps
        self.label = label
        self.serial = "dummy"
        self._n = 0
        self._running = False
        self._thread = None
        self._frame = None
        self._lock = threading.Lock()

    def start(self):
        helper.logging.info("Camera %s: dummy frames, %dx%d@%d", self.label, self.width, self.height, self.fps)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name=f"cam-{self.label}")
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    @property
    def latest(self):
        with self._lock:
            return self._frame if self._frame is not None else (None, None, None)

    def _loop(self):
        while self._running:
            with self._lock:
                self._frame = (self._render(), time.time() * 1000.0, time.time())
            self._n += 1
            time.sleep(1.0 / self.fps)

    def _render(self):
        h, w = self.height, self.width
        # A drifting diagonal band, so motion is visible and successive frames differ.
        yy, xx = np.mgrid[0:h, 0:w]
        band = ((xx + yy + self._n * 6) % 256).astype(np.uint8)
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[..., 0] = band // 3 + 40
        img[..., 1] = 40
        img[..., 2] = 120 - band // 4
        # Frame counter in the corner, as before, for spotting drops and repeats.
        img[:8, :8] = np.uint8(self._n % 256)
        if cv2 is not None:
            cv2.putText(img, "SYNTHETIC - NOT CAMERA DATA", (10, h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, max(0.4, w / 900.0), (255, 255, 255), 2)
            cv2.putText(img, f"frame {self._n}", (10, h // 2 + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, max(0.35, w / 1400.0), (255, 255, 255), 1)
        return img


def open_cameras(serials, fake=False, width=CAM_WIDTH, height=CAM_HEIGHT, fps=CAM_FPS):
    """Open one camera per side. serials maps side -> serial (or None for any device).

    A side missing from serials simply has no camera: its episode is then poses only,
    which is what the reference does when it runs without one. Returns side -> camera.

    Naming the same serial for both sides shares one camera between them rather than
    failing: a device can only be opened once, so the second pipeline would come back
    busy. That is a bench arrangement for exercising the two-arm path with one camera on
    the desk -- both datasets then hold the same pictures, which is not training data.
    """
    cameras = {}
    opened = {}
    for side, serial in serials.items():
        key = serial if serial is not None else "__any__"
        if key in opened:
            helper.logging.warning("Camera %s: sharing the %s camera -- both arms record the same view.",
                                   side, [s for s, c in cameras.items() if c is opened[key]][0])
            cameras[side] = opened[key]
            continue
        cam = DummyCamera(width, height, fps, label=side) if fake else RealSenseCamera(serial, width, height, fps, side)
        try:
            cam.start()
        except Exception as e:
            helper.logging.error("Camera %s failed to start (%s) -- that arm records poses only.", side, e)
            continue
        cameras[side] = cam
        opened[key] = cam
    return cameras


class ArmCollector:
    """Records both arms from one thread, into one buffer per arm.

    One thread, not two, so that both arms' samples come from the same robot state read
    and cannot drift apart in time.

    It builds its own dynamics state rather than sharing ArmJogger's. Forward kinematics
    there is stateful -- set_q, then compute, then read -- so two threads driving one
    state would interleave and quietly record poses that were never held. Reading
    robot.get_state() from both is fine; only the FK scratch space has to be private.
    """

    def __init__(self, robot, robot_model, cameras, gripper=None, dataset_dir="dataset",
                 collect_interval=COLLECT_INTERVAL, collect_sleep=COLLECT_SLEEP,
                 dedupe_frames=True):
        self.robot = robot
        self.cameras = cameras
        self.gripper = gripper
        self.dataset_dir = dataset_dir
        self.collect_interval = collect_interval
        self.collect_sleep = collect_sleep
        # Sampling faster than the camera would otherwise store the same picture twice
        # under two different poses -- measured at 20 Hz against a 15 fps D405, a quarter
        # of the frames were byte-identical repeats. An arm is sampled only when its
        # camera has produced a frame not already recorded, so its rate follows the
        # camera. Set False to keep every tick, as the reference does.
        self.dedupe_frames = dedupe_frames
        self._last_frame_ts = {s: None for s in SIDES}

        self.dyn = robot.get_dynamics()
        self.dyn_state = self.dyn.make_state(["base", "ee_right", "ee_left"], robot_model.robot_joint_names)

        self.imgs = {s: [] for s in SIDES}
        self.motions = {s: [] for s in SIDES}
        self.timestamps = {s: [] for s in SIDES}
        self.times = {s: [] for s in SIDES}     # (sample, pose, frame_device_ms, frame_wall)

        self.steps = 0
        self.episode_count = 0
        self.current_hz = 0.0
        self._last_collect_time = time.time()
        self._running = False
        self._thread = None

    # ── lifecycle ────────────────────────────────────────────────────────────────────
    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        os.makedirs(self.dataset_dir, exist_ok=True)
        self._last_collect_time = time.time()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="collect")
        self._thread.start()
        helper.logging.info("Collection started -> %s", os.path.abspath(self.dataset_dir))

    def stop(self):
        if self._thread is not None and self._thread.is_alive():
            self._running = False
            self._thread.join(timeout=2.0)
        self._thread = None
        helper.logging.info("Collection stopped (right %d, left %d samples)",
                            len(self.motions["right"]), len(self.motions["left"]))

    def clear(self):
        for s in SIDES:
            self.imgs[s].clear()
            self.motions[s].clear()
            self.timestamps[s].clear()
            self.times[s].clear()
        self.steps = 0
        self._last_frame_ts = {s: None for s in SIDES}
        helper.logging.info("Collected data cleared")

    # ── sampling ─────────────────────────────────────────────────────────────────────
    def _loop(self):
        while self._running:
            self.collect()
            time.sleep(self.collect_sleep)

    def collect(self):
        self.steps += 1
        if self.steps <= self.collect_interval:
            return
        if self.steps % self.collect_interval != 0:
            return

        now = time.time()
        dt = now - self._last_collect_time
        self.current_hz = 1.0 / dt if dt > 0 else 0.0
        self._last_collect_time = now

        # One state read and one FK pass feed both arms.
        pose_wall = time.time()
        q = np.array(self.robot.get_state().position)
        self.dyn_state.set_q(q)
        self.dyn.compute_forward_kinematics(self.dyn_state)

        for side in SIDES:
            t = self.dyn.compute_transformation(self.dyn_state, 0, helper.ArmJogger.LINK_OF[side])
            motion = np.hstack((t[:3, 3], helper.mat_to_quat(t[:3, :3]), self._gripper_value(side)))

            cam = self.cameras.get(side)
            if cam is not None:
                img, frame_ts, frame_wall = cam.latest
                if img is None:
                    # Drop this arm's sample rather than storing a pose with no picture:
                    # every dataset has to keep len(imgs) == len(motions).
                    continue
                if self.dedupe_frames and frame_ts == self._last_frame_ts[side]:
                    continue
                self._last_frame_ts[side] = frame_ts
                self.imgs[side].append(img)
            else:
                frame_ts, frame_wall = np.nan, np.nan

            self.motions[side].append(motion)
            self.timestamps[side].append(datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
            self.times[side].append((now, pose_wall, frame_ts, frame_wall))

    def _gripper_value(self, side):
        """Measured opening, 0 open .. 1 closed, or NaN when it cannot be known.

        NaN rather than 0: without homing the travel limits are unknown, and 0 would be
        indistinguishable from a hand that is genuinely wide open.
        """
        if self.gripper is None:
            return np.nan
        return self.gripper.measured.get(side, np.nan)

    # ── saving ───────────────────────────────────────────────────────────────────────
    def save(self):
        """Write one episode per arm. Both arms share a timestamp so they pair up."""
        if not any(self.motions[s] for s in SIDES):
            helper.logging.info("No data to save")
            return

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        for side in SIDES:
            if not self.motions[side]:
                continue
            out = os.path.join(self.dataset_dir, DATASET_DIRNAME[side])
            os.makedirs(out, exist_ok=True)

            if self.imgs[side]:
                np.save(os.path.join(out, f"imgs_{ts}.npy"), np.array(self.imgs[side]))
            np.save(os.path.join(out, f"motions_{ts}.npy"), np.array(self.motions[side]))

            with open(os.path.join(out, f"timestamps_{ts}.txt"), "w") as f:
                f.write("\n".join(self.timestamps[side]) + "\n")

            cam = self.cameras.get(side)
            if isinstance(cam, DummyCamera):
                camera_desc = "DUMMY -- synthetic frames, not camera data"
            elif cam is not None:
                camera_desc = f"realsense d405 {cam.serial}"
            else:
                camera_desc = "none"
            with open(os.path.join(out, f"meta_{ts}.txt"), "w") as f:
                f.write(f"robots: rby1_{side}_arm\n")
                f.write(f"camera: {camera_desc}\n")
                f.write("per-robot format: [x, y, z, qx, qy, qz, qw, gripper]\n")
                f.write(f"gripper: measured encoder, normalised 0 open .. 1 closed"
                        f"{'' if self.gripper else ' (unavailable, NaN)'}\n")
                if cam is not None:
                    f.write(f"image: {cam.width}x{cam.height}@{cam.fps} bgr8\n")
                    f.write(f"frame dedupe: {self.dedupe_frames}\n")

            # Sample, pose and frame times are separate columns because they are separate
            # events: the frame in hand at sample time was captured earlier, and how much
            # earlier is the thing you need when the alignment looks wrong later.
            with open(os.path.join(out, f"times_{ts}.csv"), "w") as f:
                f.write("sample_wall,pose_wall,frame_device_ms,frame_wall\n")
                for row in self.times[side]:
                    f.write("%.6f,%.6f,%.3f,%.6f\n" % row)

            helper.logging.info("Saved %s | images: %d, motions: %d -> %s",
                                side, len(self.imgs[side]), len(self.motions[side]), out)
            if isinstance(cam, DummyCamera):
                helper.logging.warning("  %s images are SYNTHETIC (--fake-camera) -- not training data.", side)

        self.episode_count += 1
        helper.logging.info("Episode %d saved (ts %s)", self.episode_count, ts)
        self.clear()
