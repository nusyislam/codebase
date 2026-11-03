"""
OAK-D S2 RGB-D demo for ESE 440 (Progress Report I).

Shows a 2x2 grid:
  [ RGB + obstacle overlay | colorized depth      ]
  [ hole-filled depth      | Canny edges on RGB   ]

Basic processing demonstrated:
  - RGB/depth capture with depth aligned to the RGB camera
  - Depth colorization (blue = near, red = far, black = no depth)
  - Distance threshold masking (anything closer than THRESH is highlighted red)
  - Nearest obstacle per zone (left / center / right), a preview of haptic logic
  - Median-filter hole filling
  - Canny edge detection
  - Click on the depth panel to read the distance at that pixel

Keys:
  q       quit
  s       save a snapshot (RGB, colorized depth, raw depth, full grid)
  r       start/stop recording RGB + raw depth frames to a folder
  + / -   raise / lower the obstacle threshold by 0.1 m
  [ / ]   lower / raise the max distance used for coloring by 0.5 m

Setup:
  pip install "depthai<3" opencv-python numpy
  python depth_demo.py              # standard mode, min depth ~35 cm at 400P
  python depth_demo.py --extended   # extended disparity, min depth ~20 cm
"""

import argparse
import os
import time

import cv2
import depthai as dai
import numpy as np

W, H = 640, 360  # panel size (RGB ISP output after 1/3 scaling of 1080p)


def build_pipeline(extended: bool) -> dai.Pipeline:
    pipeline = dai.Pipeline()

    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setIspScale(1, 3)  # 1920x1080 -> 640x360
    cam_rgb.setFps(30)

    mono_l = pipeline.create(dai.node.MonoCamera)
    mono_r = pipeline.create(dai.node.MonoCamera)
    for mono, sock in ((mono_l, dai.CameraBoardSocket.CAM_B),
                       (mono_r, dai.CameraBoardSocket.CAM_C)):
        mono.setBoardSocket(sock)
        mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono.setFps(30)

    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)       # removes occlusion errors at edges
    stereo.setExtendedDisparity(extended)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)  # align depth to RGB
    stereo.setOutputSize(W, H)

    mono_l.out.link(stereo.left)
    mono_r.out.link(stereo.right)

    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    cam_rgb.isp.link(xout_rgb.input)

    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_depth.setStreamName("depth")
    stereo.depth.link(xout_depth.input)

    return pipeline, cam_rgb


def colorize(depth_mm: np.ndarray, max_m: float) -> np.ndarray:
    """uint16 depth in mm -> BGR image. Near = blue, far = red, invalid = black."""
    max_mm = max_m * 1000.0
    d = np.clip(depth_mm.astype(np.float32), 0, max_mm)
    norm = (d / max_mm * 255).astype(np.uint8)
    color = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
    color[depth_mm == 0] = 0
    return color


def fill_holes(depth_mm: np.ndarray) -> np.ndarray:
    """Fill small gaps with a median filter, keeping originally valid pixels."""
    filled = cv2.medianBlur(depth_mm, 5)
    return np.where(depth_mm > 0, depth_mm, filled)


def zone_distances(depth_mm: np.ndarray) -> list:
    """Nearest valid distance (m) in left/center/right thirds of the middle band.
    Uses the 5th percentile instead of the minimum to ignore stray noisy pixels."""
    h, w = depth_mm.shape
    band = depth_mm[h // 4: 3 * h // 4]
    out = []
    for i in range(3):
        zone = band[:, i * w // 3:(i + 1) * w // 3]
        valid = zone[zone > 0]
        out.append(np.percentile(valid, 5) / 1000.0 if valid.size > 200 else None)
    return out


def label(img, text, y=24, color=(255, 255, 255)):
    cv2.putText(img, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--extended", action="store_true",
                    help="enable extended disparity for closer minimum depth")
    ap.add_argument("--out", default="captures", help="folder for snapshots/recordings")
    args = ap.parse_args()

    pipeline, cam_rgb = build_pipeline(args.extended)
    os.makedirs(args.out, exist_ok=True)

    state = {"click": None, "thresh_m": 1.0, "max_m": 3.0}

    def on_mouse(event, x, y, flags, param):
        # Depth panel is the top-right quadrant of the grid
        if event == cv2.EVENT_LBUTTONDOWN and x >= W and y < H:
            state["click"] = (x - W, y)

    with dai.Device() as device:
        # The S2-AF has an autofocus RGB lens; lock it at the factory calibration
        # position so depth stays correctly aligned with the RGB image.
        try:
            lens_pos = device.readCalibration2().getLensPosition(dai.CameraBoardSocket.CAM_A)
            if lens_pos:
                cam_rgb.initialControl.setManualFocus(lens_pos)
        except Exception as e:
            print(f"Could not read lens position, leaving autofocus on: {e}")

        device.startPipeline(pipeline)
        print(f"Connected. USB speed: {device.getUsbSpeed().name} "
              f"(SUPER = USB 3, HIGH = USB 2)")
        print(f"Extended disparity: {'ON' if args.extended else 'OFF'}")

        q_rgb = device.getOutputQueue("rgb", maxSize=4, blocking=False)
        q_depth = device.getOutputQueue("depth", maxSize=4, blocking=False)

        cv2.namedWindow("RGB-D demo")
        cv2.setMouseCallback("RGB-D demo", on_mouse)

        recording, rec_dir, rec_idx = False, None, 0
        t_prev, fps = time.time(), 0.0

        while True:
            rgb = q_rgb.get().getCvFrame()
            depth = q_depth.get().getFrame()  # uint16, millimeters

            if rgb.shape[:2] != (H, W):
                rgb = cv2.resize(rgb, (W, H))
            if depth.shape != (H, W):
                depth = cv2.resize(depth, (W, H), interpolation=cv2.INTER_NEAREST)

            now = time.time()
            fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_prev, 1e-6))
            t_prev = now

            # 1. Obstacle overlay: highlight anything closer than threshold
            thresh_mm = state["thresh_m"] * 1000
            near = (depth > 0) & (depth < thresh_mm)
            overlay = rgb.copy()
            overlay[near] = (0.5 * overlay[near] + 0.5 * np.array([0, 0, 255])).astype(np.uint8)
            for i in (1, 2):
                cv2.line(overlay, (i * W // 3, 0), (i * W // 3, H), (255, 255, 255), 1)
            zones = zone_distances(depth)
            names = ("L", "C", "R")
            zone_txt = "  ".join(
                f"{n}:{z:.2f}m" if z is not None else f"{n}:--" for n, z in zip(names, zones))
            label(overlay, f"RGB + obstacles < {state['thresh_m']:.1f} m")
            label(overlay, zone_txt, y=48, color=(0, 255, 255))
            label(overlay, f"{fps:.1f} fps", y=H - 10)

            # 2. Colorized depth + click readout
            depth_vis = colorize(depth, state["max_m"])
            label(depth_vis, f"Depth (0 - {state['max_m']:.1f} m)")
            if state["click"]:
                cx, cy = state["click"]
                patch = depth[max(cy - 2, 0):cy + 3, max(cx - 2, 0):cx + 3]
                valid = patch[patch > 0]
                txt = f"{np.median(valid) / 1000:.2f} m" if valid.size else "no depth"
                cv2.drawMarker(depth_vis, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 16, 2)
                label(depth_vis, f"clicked: {txt}", y=48)

            # 3. Hole-filled depth
            filled = fill_holes(depth)
            filled_vis = colorize(filled, state["max_m"])
            pct_before = 100 * np.mean(depth == 0)
            pct_after = 100 * np.mean(filled == 0)
            label(filled_vis, "Hole-filled depth (median 5x5)")
            label(filled_vis, f"invalid: {pct_before:.0f}% -> {pct_after:.0f}%", y=48)

            # 4. Edge detection on RGB
            gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
            edges = cv2.cvtColor(cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 50, 150),
                                 cv2.COLOR_GRAY2BGR)
            label(edges, "Canny edges (RGB)")

            if recording:
                label(overlay, "REC", y=72, color=(0, 0, 255))

            grid = np.vstack([np.hstack([overlay, depth_vis]),
                              np.hstack([filled_vis, edges])])
            cv2.imshow("RGB-D demo", grid)

            if recording:
                cv2.imwrite(os.path.join(rec_dir, f"rgb_{rec_idx:05d}.jpg"), rgb)
                cv2.imwrite(os.path.join(rec_dir, f"depth_{rec_idx:05d}.png"), depth)  # 16-bit mm
                rec_idx += 1

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("s"):
                stamp = time.strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(os.path.join(args.out, f"{stamp}_grid.png"), grid)
                cv2.imwrite(os.path.join(args.out, f"{stamp}_rgb.png"), rgb)
                cv2.imwrite(os.path.join(args.out, f"{stamp}_depth_color.png"), depth_vis)
                np.save(os.path.join(args.out, f"{stamp}_depth_mm.npy"), depth)
                print(f"Saved snapshot {stamp} | zones {zone_txt} | "
                      f"invalid {pct_before:.0f}% -> {pct_after:.0f}% | {fps:.1f} fps")
            elif key == ord("r"):
                recording = not recording
                if recording:
                    rec_dir = os.path.join(args.out, time.strftime("rec_%Y%m%d_%H%M%S"))
                    os.makedirs(rec_dir, exist_ok=True)
                    rec_idx = 0
                    print(f"Recording to {rec_dir}")
                else:
                    print(f"Stopped recording ({rec_idx} frames)")
            elif key in (ord("+"), ord("=")):
                state["thresh_m"] = min(state["thresh_m"] + 0.1, 5.0)
            elif key == ord("-"):
                state["thresh_m"] = max(state["thresh_m"] - 0.1, 0.2)
            elif key == ord("]"):
                state["max_m"] = min(state["max_m"] + 0.5, 10.0)
            elif key == ord("["):
                state["max_m"] = max(state["max_m"] - 0.5, 1.0)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
