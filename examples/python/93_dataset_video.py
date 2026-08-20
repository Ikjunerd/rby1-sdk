# Dataset Video
# Renders a recorded episode as an mp4 with the motion that was logged alongside each
# frame burned into it, so a dataset can be checked by watching it.
#
# What this is for: the numbers and the pictures are stored in separate files and nothing
# in the format proves they line up. Playing them back together is how a mismatch shows
# itself -- a gripper value that flips a second after the fingers visibly close, or a
# pose that moves while the picture does not.
#
# One video per arm. The two arms are not rendered side by side because they do not have
# the same number of frames: each arm samples at its own camera's rate, so their indices
# do not correspond.
#
# Usage:
#     python 93_dataset_video.py                              # latest episode, both arms
#     python 93_dataset_video.py --dataset ~/dataset --arm right
#     python 93_dataset_video.py --episode 20260820_112712
#     python 93_dataset_video.py --out /tmp                   # where to write

import argparse
import glob
import os

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

ARM_DIR = {"right": "rby-right-arm", "left": "rby-left-arm"}

FONT = cv2.FONT_HERSHEY_SIMPLEX if cv2 else None
FG = (255, 255, 255)
DIM = (170, 170, 170)
BAR_BG = (60, 60, 60)
BAR_FG = (80, 220, 80)


def episodes(arm_dir):
    """Episode timestamps present in a directory, oldest first."""
    return sorted(os.path.basename(p)[len("motions_"):-len(".npy")]
                  for p in glob.glob(os.path.join(arm_dir, "motions_*.npy")))


def load_episode(arm_dir, ts):
    imgs_path = os.path.join(arm_dir, f"imgs_{ts}.npy")
    if not os.path.exists(imgs_path):
        return None, None, None
    # mmap: an episode can be gigabytes, and only one frame is needed at a time.
    imgs = np.load(imgs_path, mmap_mode="r")
    motions = np.load(os.path.join(arm_dir, f"motions_{ts}.npy"))

    times_path = os.path.join(arm_dir, f"times_{ts}.csv")
    sample_wall = None
    if os.path.exists(times_path):
        rows = np.genfromtxt(times_path, delimiter=",", names=True)
        sample_wall = np.atleast_1d(rows["sample_wall"])
    return imgs, motions, sample_wall


def playback_fps(sample_wall, fallback):
    """Real-time playback rate, taken from the recorded sample times when available.

    The median gap rather than the mean: a pause while the operator repositions would
    drag a mean down and play the whole episode back in slow motion.
    """
    if sample_wall is None or len(sample_wall) < 2:
        return fallback
    dt = np.median(np.diff(sample_wall))
    return float(np.clip(1.0 / dt, 1.0, 120.0)) if dt > 0 else fallback


def draw_overlay(frame, i, total, motion, elapsed):
    h, w = frame.shape[:2]
    pad, line = 8, 18
    box_h = line * 4 + pad
    # Darken behind the text rather than drawing on the picture: the scene is whatever the
    # gripper is looking at, and white text on a pale object is unreadable.
    strip = frame[h - box_h:h, 0:w]
    frame[h - box_h:h, 0:w] = (strip * 0.35).astype(np.uint8)

    x, y, z, qx, qy, qz, qw, grip = motion
    y0 = h - box_h + line
    cv2.putText(frame, f"frame {i+1}/{total}   t {elapsed:6.2f}s", (pad, y0), FONT, 0.45, DIM, 1)
    cv2.putText(frame, f"xyz  {x:+.4f} {y:+.4f} {z:+.4f}", (pad, y0 + line), FONT, 0.45, FG, 1)
    cv2.putText(frame, f"quat {qx:+.3f} {qy:+.3f} {qz:+.3f} {qw:+.3f}", (pad, y0 + 2 * line), FONT, 0.45, FG, 1)

    gy = y0 + 3 * line
    if np.isnan(grip):
        cv2.putText(frame, "grip  n/a (no gripper bus)", (pad, gy), FONT, 0.45, DIM, 1)
    else:
        cv2.putText(frame, f"grip  {grip:.2f}", (pad, gy), FONT, 0.45, FG, 1)
        bx, bw, bh = pad + 110, 120, 10
        cv2.rectangle(frame, (bx, gy - bh), (bx + bw, gy), BAR_BG, -1)
        cv2.rectangle(frame, (bx, gy - bh), (bx + int(bw * float(np.clip(grip, 0, 1))), gy), BAR_FG, -1)
        cv2.putText(frame, "open" if grip < 0.5 else "closed", (bx + bw + 8, gy), FONT, 0.4, DIM, 1)
    return frame


def render(arm, arm_dir, ts, out_dir, fallback_fps):
    imgs, motions, sample_wall = load_episode(arm_dir, ts)
    if imgs is None:
        print(f"  {arm}: episode {ts} has no imgs_*.npy (poses only) -- skipped")
        return None
    n = min(len(imgs), len(motions))
    if n == 0:
        print(f"  {arm}: episode {ts} is empty -- skipped")
        return None
    if len(imgs) != len(motions):
        # Should not happen: the collector drops a sample rather than let these diverge.
        print(f"  {arm}: WARNING imgs {len(imgs)} != motions {len(motions)}, rendering {n}")

    fps = playback_fps(sample_wall, fallback_fps)
    h, w = imgs.shape[1:3]
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f"{ARM_DIR[arm]}_{ts}.mp4")
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    if not writer.isOpened():
        print(f"  {arm}: could not open a writer for {out_path}")
        return None

    t0 = sample_wall[0] if sample_wall is not None and len(sample_wall) else 0.0
    for i in range(n):
        frame = np.array(imgs[i])       # mmap slice -> writable copy
        elapsed = (sample_wall[i] - t0) if sample_wall is not None and i < len(sample_wall) else i / fps
        writer.write(draw_overlay(frame, i, n, motions[i], elapsed))
    writer.release()
    print(f"  {arm}: {n} frames, {w}x{h} @ {fps:.1f} fps -> {out_path}")
    return out_path


def main():
    ap = argparse.ArgumentParser(description="93_dataset_video")
    ap.add_argument("--dataset", type=str, default=os.path.expanduser("~/dataset"))
    ap.add_argument("--arm", choices=("right", "left", "both"), default="both")
    ap.add_argument("--episode", type=str, default=None, help="Episode timestamp (default: the latest)")
    ap.add_argument("--all", action="store_true", help="Render every episode, not just one")
    ap.add_argument("--out", type=str, default=None, help="Output directory (default: alongside the data)")
    ap.add_argument("--fps", type=float, default=15.0, help="Playback fps when the episode has no times_*.csv")
    args = ap.parse_args()

    if cv2 is None:
        print("opencv is required: pip3 install opencv-python")
        return

    arms = ("right", "left") if args.arm == "both" else (args.arm,)
    for arm in arms:
        arm_dir = os.path.join(args.dataset, ARM_DIR[arm])
        if not os.path.isdir(arm_dir):
            print(f"{arm}: {arm_dir} does not exist -- skipped")
            continue
        eps = episodes(arm_dir)
        if not eps:
            print(f"{arm}: no episodes in {arm_dir}")
            continue
        if args.episode:
            if args.episode not in eps:
                print(f"{arm}: episode {args.episode} not found. Available: {', '.join(eps)}")
                continue
            chosen = [args.episode]
        else:
            chosen = eps if args.all else [eps[-1]]

        print(f"{arm}: {len(eps)} episode(s) in {arm_dir}")
        for ts in chosen:
            render(arm, arm_dir, ts, args.out or arm_dir, args.fps)


if __name__ == "__main__":
    main()
