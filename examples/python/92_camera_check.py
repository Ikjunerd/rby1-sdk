# Camera Check
# Lists what the attached D405s can stream, opens one in a preview window, and reports
# how bright the frames are.
#
# The brightness readout is the point of the tool. A D405 takes its colour from the
# stereo imager and the auto-exposure drifts for a couple of seconds after the pipeline
# opens, so the first frames of a fresh stream are darker than the scene. If the mean
# climbs and then levels off, the camera is fine and a recording simply began too early;
# if it sits near zero, the scene is dark or the lens is covered.
#
# The window shows the frames exactly as they are stored: BGR uint8, no conversion. What
# you see here is what lands in imgs_*.npy.
#
# Usage:
#     python 92_camera_check.py                       # preview until q
#     python 92_camera_check.py --width 848 --height 480 --fps 30
#     python 92_camera_check.py --serial 409122271622
#     python 92_camera_check.py --no-show --seconds 6  # headless, numbers only

import argparse
import time

import numpy as np
import pyrealsense2 as rs

try:
    import cv2
except ImportError:
    cv2 = None


def list_profiles(dev):
    by_res = {}
    for s in dev.sensors:
        for p in s.profiles:
            if p.stream_type() == rs.stream.color:
                v = p.as_video_stream_profile()
                by_res.setdefault((v.width(), v.height()), set()).add(p.fps())
    print("  color profiles:")
    for (w, h), fps in sorted(by_res.items()):
        print(f"    {w}x{h} @ {sorted(fps)}")
    return set(by_res)


def main():
    ap = argparse.ArgumentParser(description="92_camera_check")
    ap.add_argument("--serial", type=str, default=None)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="Stop after this long. 0 means run until q (or 6 s with --no-show).")
    ap.add_argument("--no-show", action="store_true", help="Skip the preview window")
    args = ap.parse_args()

    show = not args.no_show and cv2 is not None
    if not args.no_show and cv2 is None:
        print("opencv is not installed -- running without a window.")
    seconds = args.seconds or (0.0 if show else 6.0)

    devs = list(rs.context().devices)
    if not devs:
        print("No RealSense device found.")
        return
    for d in devs:
        print(f"{d.get_info(rs.camera_info.name)}  serial {d.get_info(rs.camera_info.serial_number)}  "
              f"usb {d.get_info(rs.camera_info.usb_type_descriptor)}")
        res = list_profiles(d)
        want = (args.width, args.height)
        print(f"  requested {want[0]}x{want[1]}: {'supported' if want in res else 'NOT SUPPORTED'}")
    print()

    cfg = rs.config()
    if args.serial:
        cfg.enable_device(args.serial)
    cfg.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    pipe = rs.pipeline()
    try:
        pipe.start(cfg)
    except Exception as e:
        print(f"start failed: {e}")
        return

    print(f"streaming {args.width}x{args.height}@{args.fps}"
          + (" -- press q in the window to stop" if show else f" for {seconds}s"))
    print("  t(s)   mean  min  max   dtype/shape")

    t0 = time.time()
    n = 0
    try:
        while True:
            if seconds and time.time() - t0 >= seconds:
                break
            frames = pipe.wait_for_frames(2000)
            c = frames.get_color_frame()
            if not c:
                continue
            img = np.asanyarray(c.get_data())
            n += 1
            mean = img.mean()
            if n % 15 == 1:
                print(f"  {time.time()-t0:5.2f}  {mean:5.1f}  {img.min():3d}  {img.max():3d}   "
                      f"{img.dtype} {img.shape}")
            if show:
                view = img.copy()
                cv2.putText(view, f"{args.width}x{args.height}@{args.fps}  mean {mean:5.1f}",
                            (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(view, "q to quit", (10, view.shape[0] - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.imshow("D405 color (BGR, as stored)", view)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        pipe.stop()
        if show:
            cv2.destroyAllWindows()
    print(f"\n{n} frames. mean near 0 = dark scene or covered lens; rising then flat = auto-exposure settling.")


if __name__ == "__main__":
    main()
