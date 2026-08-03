#!/usr/bin/env python3
"""
Make Test Data — generates fake flight data to test laptop_processor.py
========================================================================
This creates a fake flight folder with:
  - 1 transmitter log
  - 3 receiver folders, each with fake camera images (green laser spots)
    and fake NovAtel flight logs

You run this ONCE to create test data, then run laptop_processor.py on it.
No drones, no Pi, no hardware needed — pure software test.

RUN:
  python3 make_test_data.py

Then:
  python3 laptop_processor.py --flight_dir ./test_flight --output ./test_results
"""

import numpy as np
import cv2
import csv
from pathlib import Path
import shutil

# -----------------------------------------------------------------------
# SETUP — where the fake drones are
# -----------------------------------------------------------------------
TX_POS = np.array([0.0, 0.0, 100.0])    # transmitter 100m up
RX_POS = {
    0: np.array([15.0,  0.0, 50.0]),    # receiver 0 — east
    1: np.array([0.0,  15.0, 50.0]),    # receiver 1 — north
    2: np.array([-15.0, 0.0, 50.0]),    # receiver 2 — west
}
GROUND = np.array([0.0, 0.0, 0.0])      # laser hits the origin on the ground

# Camera settings (matches Arducam IMX219)
CAM_FX, CAM_FY, CAM_CX, CAM_CY = 820.0, 820.0, 640.0, 360.0
W, H = 1280, 720

N_FRAMES = 6
t0 = 1718000000.0   # fake GPS start time


def world_to_pixel(ground_pt, cam_pos):
    """Figure out where the laser spot appears in this camera's image."""
    d = ground_pt - cam_pos
    d = d / np.linalg.norm(d)
    u_cam = np.array([d[0], -d[1], -d[2]])
    u_cam /= u_cam[2]
    px = u_cam[0] * CAM_FX + CAM_CX
    py = u_cam[1] * CAM_FY + CAM_CY
    return px, py


def main():
    base = Path('./test_flight')
    if base.exists():
        shutil.rmtree(base)

    # --- Transmitter log ---
    tx_dir = base / 'transmitter'
    tx_dir.mkdir(parents=True)
    with open(tx_dir / 'transmitter_log.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['timestamp_gps', 'lat', 'lon', 'alt',
                    'x_enu', 'y_enu', 'z_enu',
                    'roll_deg', 'pitch_deg', 'yaw_deg',
                    'roll_rad', 'pitch_rad', 'yaw_rad', 'ins_status'])
        for i in range(N_FRAMES * 3):
            t = t0 + i * 0.05
            w.writerow([f"{t:.6f}", 0, 0, 100,
                        f"{TX_POS[0]:.4f}", f"{TX_POS[1]:.4f}", f"{TX_POS[2]:.4f}",
                        0, 0, 0, 0, 0, 0, 'INS_SOLUTION_GOOD'])

    # --- Receiver folders ---
    for rx_id, pos in RX_POS.items():
        rdir = base / f'receiver_{rx_id}'
        (rdir / 'images').mkdir(parents=True)

        with open(rdir / 'flight_log.csv', 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['timestamp_gps', 'lat', 'lon', 'alt',
                        'x_enu', 'y_enu', 'z_enu',
                        'roll_deg', 'pitch_deg', 'yaw_deg',
                        'roll_rad', 'pitch_rad', 'yaw_rad',
                        'ins_status', 'image_file'])

            for i in range(N_FRAMES):
                t_img = t0 + i * 0.1 + rx_id * 0.01

                # Make a dark image with a green laser spot
                img = np.zeros((H, W, 3), dtype=np.uint8)
                img[:] = (30, 40, 30)
                px, py = world_to_pixel(GROUND, pos)
                if 0 <= px < W and 0 <= py < H:
                    cv2.circle(img, (int(round(px)), int(round(py))),
                               6, (80, 255, 120), -1)   # green spot
                    cv2.circle(img, (int(round(px)), int(round(py))),
                               3, (220, 255, 230), -1)  # bright core

                fname = f"{t_img:.6f}.jpg"
                cv2.imwrite(str(rdir / 'images' / fname), img)

                w.writerow([f"{t_img:.6f}", 0, 0, 50,
                            f"{pos[0]:.4f}", f"{pos[1]:.4f}", f"{pos[2]:.4f}",
                            0, 0, 0, 0, 0, 0, 'INS_SOLUTION_GOOD', fname])

        print(f"Receiver {rx_id}: spot appears at pixel "
              f"{tuple(round(v,1) for v in world_to_pixel(GROUND, pos))}")

    print(f"\nTest data created in: {base.resolve()}")
    print(f"True ground point is at: (0, 0, 0)")
    print(f"\nNow run:")
    print(f"  python3 laptop_processor.py --flight_dir ./test_flight --output ./test_results")


if __name__ == '__main__':
    main()
