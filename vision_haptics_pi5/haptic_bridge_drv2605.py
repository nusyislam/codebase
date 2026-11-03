"""OAK-D Pro -> Raspberry Pi 5 -> Adafruit DRV2605L -> ONE mini vibration motor.

The Pico and haptic_bridge.py/main.py are not used for this configuration.

Run:
    python3 haptic_bridge_drv2605.py --dry-run  # Camera only, displays target intensity
    python3 haptic_bridge_drv2605.py             # Camera + DRV2605L + motor
    python3 haptic_bridge_drv2605.py --headless  # SSH without a graphical window

Depends on depth_demo.py from the original project, DepthAI 2.x,
OpenCV, numpy, adafruit-circuitpython-drv2605 and Adafruit Blinka.
"""
import argparse
import time

import cv2
import depthai as dai

from depth_demo import W, H, build_pipeline, colorize, label, zone_distances

NEAR_M = 0.4      # Full programmed strength for obstacles <= 0.4m away
FAR_M = 1.5       # No vibration for obstacles >= 1.5m away
MAX_RTP = 95      # Conservative initial level. DRV2605L RTP signed range: 0..127
UPDATE_HZ = 20   # How frequently to update motor strength
SMOOTH = 0.4      # Smoothing across recent depth frames
STALE_S = 0.5    # If camera data stops, turn motor off


def distance_to_rtp(dist_m):
    """One obstacle distance in meters -> safe positive real-time drive value."""
    if dist_m is None or dist_m >= FAR_M:
        return 0
    if dist_m <= NEAR_M:
        return MAX_RTP
    fraction = (FAR_M - dist_m) / (FAR_M - NEAR_M)
    return round(fraction * MAX_RTP)


def init_motor():
    import board
    import adafruit_drv2605

    motor = adafruit_drv2605.DRV2605(board.I2C())
    motor.use_ERM()
    motor.realtime_value = 0
    motor.mode = adafruit_drv2605.MODE_REALTIME
    return motor, adafruit_drv2605


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dry-run", action="store_true", help="camera processing only, no I2C motor")
    ap.add_argument("--headless", action="store_true", help="no OpenCV window (for SSH)")
    ap.add_argument("--standard", action="store_true", help="disable extended disparity")
    args = ap.parse_args()

    drv = None
    library = None
    if not args.dry_run:
        drv, library = init_motor()
        print("DRV2605L connected over I2C at address 0x5A")

    last_rtp = -1
    last_update = 0.0
    last_frame = time.monotonic()
    smoothed = None
    # In the original repo this function sets up the OAK-D stereo depth pipeline.
    pipeline, cam_rgb = build_pipeline(extended=not args.standard)

    try:
        with dai.Device() as device:
            try:
                lens_pos = device.readCalibration2().getLensPosition(dai.CameraBoardSocket.CAM_A)
                if lens_pos:
                    cam_rgb.initialControl.setManualFocus(lens_pos)
            except Exception:
                pass
            device.startPipeline(pipeline)
            depth_queue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            print("Camera running. Move something closer and farther away. Ctrl+C or q exits.")
            while True:
                packet = depth_queue.tryGet()  # Avoid blocking with the motor left on.
                now = time.monotonic()
                if packet is None:
                    if drv is not None and now - last_frame > STALE_S and last_rtp != 0:
                        drv.realtime_value = 0
                        last_rtp = 0
                    if not args.headless and cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    time.sleep(0.01)
                    continue

                last_frame = now
                depth = packet.getFrame()  # Millimeters
                if depth.shape != (H, W):
                    depth = cv2.resize(depth, (W, H), interpolation=cv2.INTER_NEAREST)
                zone_dists = zone_distances(depth)
                visible = [d for d in zone_dists if d is not None]
                nearest = min(visible) if visible else None
                if nearest is None:
                    smoothed = None
                elif smoothed is None:
                    smoothed = nearest
                else:
                    smoothed = SMOOTH * smoothed + (1 - SMOOTH) * nearest

                rtp = distance_to_rtp(smoothed)
                if now - last_update >= 1.0 / UPDATE_HZ:
                    if drv is not None and rtp != last_rtp:
                        drv.realtime_value = rtp
                    last_rtp = rtp
                    last_update = now
                    dist_text = f"{smoothed:.2f}m" if smoothed is not None else "no depth"
                    print(f"nearest: {dist_text:>9} | motor level: {rtp:>3}/{MAX_RTP}    ", end="\r", flush=True)

                if not args.headless:
                    vis = colorize(depth, 3.0)
                    for i in (1, 2):
                        cv2.line(vis, (i * W // 3, 0), (i * W // 3, H), (255, 255, 255), 1)
                    distance_label = f"{smoothed:.2f}m" if smoothed is not None else "NO DEPTH"
                    label(vis, f"Nearest: {distance_label}   Motor: {rtp}/{MAX_RTP}")
                    cv2.imshow("OAK-D Pro Haptic Demo", vis)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
    except KeyboardInterrupt:
        pass
    finally:
        if drv is not None:
            try:
                drv.realtime_value = 0
                drv.mode = library.MODE_INTTRIG
            except OSError:
                print("Warning: could not stop motor over I2C. Disconnect power if needed.")
        cv2.destroyAllWindows()
        print("\nStopped.")


if __name__ == "__main__":
    main()
