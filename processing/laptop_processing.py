#!/usr/bin/env python3
"""
Loads data from:
  - Transmitter drone: position + attitude (transmitter_log.csv)
  - 3 receiver drones: images + position + attitude (flight_log.csv)

What it does: 
Matches everything by GPS timestamp.
Detects green laser spots in images.
Corrects for drone movement using roll/pitch/yaw.
Solves 3D ground points using the paper's math.
Saves results as CSV + side-by-side images + point cloud.

FOLDER STRUCTURE:
  flight_001/
  ├── transmitter/
  │   └── transmitter_log.csv
  ├── receiver_0/
  │   ├── images/
  │   └── flight_log.csv
  ├── receiver_1/
  │   ├── images/
  │   └── flight_log.csv
  └── receiver_2/
      ├── images/
      └── flight_log.csv

RUN:
  python3 laptop_processor.py --flight_dir ~/flight_001 --output ~/results

INSTALL ON MACBOOK:
  pip3 install opencv-python numpy pandas matplotlib
"""

import argparse
import csv
import math
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from async_lidar_math import (solve_ground_point, compute_dop,
                               rotation_matrix_from_euler)

# -----------------------------------------------------------------------
# PI CAMERA V2 INTRINSICS
# Calibrate with a checkerboard for best accuracy.
# These defaults work for most Pi Camera v2 setups at 1280x720.
# -----------------------------------------------------------------------
CAM_FX = 820.0
CAM_FY = 820.0
CAM_CX = 640.0
CAM_CY = 360.0


# -----------------------------------------------------------------------
# GREEN LASER SPOT DETECTOR
# -----------------------------------------------------------------------

def detect_laser_spots(image_bgr):
    """
    Find green 520nm laser spots in a camera image.
    Returns list of (x_pixel, y_pixel, area, brightness) sorted brightest first.
    """
    hsv         = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask_green  = cv2.inRange(hsv, np.array([35, 100, 180]),
                                    np.array([85, 255, 255]))
    mask_bright = cv2.inRange(hsv, np.array([30,  50, 240]),
                                    np.array([90, 255, 255]))
    mask        = cv2.bitwise_or(mask_green, mask_bright)
    kernel      = np.ones((3, 3), np.uint8)
    mask        = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask        = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    spots = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 5 or area > 500:
            continue
        M = cv2.moments(cnt)
        if M['m00'] == 0:
            continue
        cx   = M['m10'] / M['m00']
        cy   = M['m01'] / M['m00']
        x, y, w, h = cv2.boundingRect(cnt)
        conf = float(np.mean(image_bgr[y:y+h, x:x+w, 1]))
        spots.append((cx, cy, area, conf))
    spots.sort(key=lambda s: s[3], reverse=True)
    return spots


# -----------------------------------------------------------------------
# PIXEL → WORLD UNIT VECTOR (corrected for drone attitude)
# -----------------------------------------------------------------------

def pixel_to_world_vector(px, py, roll_rad, pitch_rad, yaw_rad):
    """
    Convert a pixel location in the camera image to a unit vector
    in the world (ENU) frame, corrected for the drone's attitude.

    This is A_r from the paper (Eq. 3): A_r = R_br * u_r
    """
    # Pixel → camera frame unit vector
    x_norm = (px - CAM_CX) / CAM_FX
    y_norm = (py - CAM_CY) / CAM_FY
    u_cam  = np.array([x_norm, y_norm, 1.0])
    u_cam  /= np.linalg.norm(u_cam)

    # Camera frame → body frame (camera points down on drone)
    # 180° rotation about X: cam X→body X, cam Y(image-down)→body -Y,
    # cam Z(optical axis/forward)→body -Z (down). Center pixel → (0,0,-1). ✓
    u_body = np.array([u_cam[0], -u_cam[1], -u_cam[2]])
    u_body /= np.linalg.norm(u_body)

    # Body frame → world (ENU) using rotation matrix from attitude
    R   = rotation_matrix_from_euler(roll_rad, pitch_rad, yaw_rad)
    A_r = R @ u_body
    A_r[2] = -abs(A_r[2])  # must point downward in ENU
    return A_r / np.linalg.norm(A_r)


def transmitter_beam_vector(roll_rad, pitch_rad, yaw_rad):
    """
    Compute the laser beam unit vector A_t from the transmitter's attitude.
    The laser points straight down in the drone body frame = [0, 0, -1] in ENU.
    After rotating by the drone's attitude, we get the actual beam direction.

    This is A_t from the paper (Eq. 2): A_t = R_bt * u_t
    """
    u_laser = np.array([0.0, 0.0, -1.0])  # nadir in body frame
    R       = rotation_matrix_from_euler(roll_rad, pitch_rad, yaw_rad)
    A_t     = R @ u_laser
    A_t[2]  = -abs(A_t[2])  # force downward
    return A_t / np.linalg.norm(A_t)


# -----------------------------------------------------------------------
# LOG LOADERS
# -----------------------------------------------------------------------

def load_log(csv_path):
    """Load any flight_log.csv or transmitter_log.csv into a DataFrame."""
    df = pd.read_csv(csv_path)
    df['timestamp_gps'] = df['timestamp_gps'].astype(float)
    return df.set_index('timestamp_gps').sort_index()


def get_state(log_df, timestamp):
    """
    Get interpolated position + attitude at a specific GPS timestamp.
    Returns (pos_xyz, roll_rad, pitch_rad, yaw_rad)
    """
    idx = log_df.index.searchsorted(timestamp)
    idx = min(max(idx, 1), len(log_df) - 1)

    t0 = log_df.index[idx - 1]
    t1 = log_df.index[idx]
    r0 = log_df.iloc[idx - 1]
    r1 = log_df.iloc[idx]

    alpha = 0.0 if t1 == t0 else (timestamp - t0) / (t1 - t0)

    pos = np.array([
        r0['x_enu'] + alpha * (r1['x_enu'] - r0['x_enu']),
        r0['y_enu'] + alpha * (r1['y_enu'] - r0['y_enu']),
        r0['z_enu'] + alpha * (r1['z_enu'] - r0['z_enu']),
    ])
    roll  = r0['roll_rad']  + alpha * (r1['roll_rad']  - r0['roll_rad'])
    pitch = r0['pitch_rad'] + alpha * (r1['pitch_rad'] - r0['pitch_rad'])
    yaw   = r0['yaw_rad']   + alpha * (r1['yaw_rad']   - r0['yaw_rad'])

    return pos, roll, pitch, yaw


# -----------------------------------------------------------------------
# IMAGE MATCHING BY GPS TIMESTAMP
# -----------------------------------------------------------------------

def match_images(receiver_dirs, time_tolerance=0.05):
    """Match images across all receivers by GPS timestamp."""
    all_times = {}
    for rx_id, rx_dir in enumerate(receiver_dirs):
        img_dir = Path(rx_dir) / 'images'
        times   = {}
        for f in sorted(img_dir.glob('*.jpg')):
            try:
                times[float(f.stem)] = f
            except ValueError:
                continue
        all_times[rx_id] = times
        print(f"  Receiver {rx_id}: {len(times)} images")

    matched = []
    for t_ref in sorted(all_times[0].keys()):
        group = {'timestamp': t_ref, 0: all_times[0][t_ref]}
        ok    = True
        for rx_id in range(1, len(receiver_dirs)):
            times_arr = np.array(sorted(all_times[rx_id].keys()))
            if len(times_arr) == 0:
                ok = False; break
            idx     = np.argmin(np.abs(times_arr - t_ref))
            t_match = times_arr[idx]
            if abs(t_match - t_ref) > time_tolerance:
                ok = False; break
            group[rx_id] = all_times[rx_id][t_match]
        if ok:
            matched.append(group)

    print(f"  Matched {len(matched)} synchronized frame groups")
    return matched


# -----------------------------------------------------------------------
# VISUALIZATION
# -----------------------------------------------------------------------

def save_visualization(images, spots, ground_points,
                        frame_idx, timestamp, out_path):
    """
    Save side-by-side view of all 3 camera images
    with laser spots labeled with spot ID and solved ground point XY.
    """
    vis_dir = out_path / 'visualizations'
    vis_dir.mkdir(exist_ok=True)

    n       = len(images)
    fig, axes = plt.subplots(1, n, figsize=(6 * n, 5))
    if n == 1:
        axes = [axes]

    colors = ['lime', 'cyan', 'magenta', 'orange']

    for rx_id, ax in enumerate(axes):
        if rx_id not in images:
            continue

        img_rgb = cv2.cvtColor(images[rx_id], cv2.COLOR_BGR2RGB)
        ax.imshow(img_rgb)
        ax.set_title(f'Camera {rx_id}  t={timestamp:.2f}',
                     color='white', fontsize=9,
                     bbox=dict(boxstyle='round', facecolor='black', alpha=0.8))
        ax.axis('off')

        col = colors[rx_id % len(colors)]
        for spot_id, (px, py, area, conf) in enumerate(
                spots.get(rx_id, [])[:50]):

            circle = plt.Circle((px, py),
                                  radius=max(math.sqrt(area), 8),
                                  color=col, fill=False, linewidth=2)
            ax.add_patch(circle)

            gp    = ground_points.get(spot_id)
            label = (f'#{spot_id}\n({gp[0]:.1f},{gp[1]:.1f})'
                     if gp is not None else f'#{spot_id}')

            ax.annotate(label, xy=(px, py),
                        xytext=(px + 15, py - 20),
                        color=col, fontsize=6, fontweight='bold',
                        arrowprops=dict(arrowstyle='->', color=col, lw=1))

    fig.patch.set_facecolor('black')
    plt.suptitle(
        f'Frame {frame_idx} — {len(ground_points)} ground points solved',
        color='white', fontsize=10)
    plt.tight_layout()
    plt.savefig(str(vis_dir / f'frame_{frame_idx:04d}.png'),
                dpi=100, bbox_inches='tight', facecolor='black')
    plt.close()


# -----------------------------------------------------------------------
# MAIN PROCESSOR
# -----------------------------------------------------------------------

def process(flight_dir, output_dir):
    flight_path = Path(flight_dir)
    out_path    = Path(output_dir)
    out_path.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("  Async Lidar Post-processor v3")
    print("  Real life — transmitter + receivers")
    print("=" * 60)

    # Load transmitter log
    tx_log_path = flight_path / 'transmitter' / 'transmitter_log.csv'
    if tx_log_path.exists():
        tx_log = load_log(str(tx_log_path))
        print(f"\nTransmitter log: {len(tx_log)} records")
    else:
        tx_log = None
        print("\nWARNING: No transmitter log found — using receiver 0 as pseudo-transmitter")

    # Find receiver folders
    rx_dirs = sorted([d for d in flight_path.iterdir()
                      if d.is_dir() and d.name.startswith('receiver_')])
    print(f"Found {len(rx_dirs)} receivers\n")

    # Load receiver flight logs
    rx_logs = {}
    for rx_dir in rx_dirs:
        rx_id    = int(rx_dir.name.split('_')[1])
        log_path = rx_dir / 'flight_log.csv'
        if log_path.exists():
            rx_logs[rx_id] = load_log(str(log_path))
            print(f"  Receiver {rx_id}: {len(rx_logs[rx_id])} records")

    # Match images by GPS timestamp
    print("\nMatching images by GPS timestamp...")
    matched = match_images([str(d) for d in rx_dirs])

    if not matched:
        print("ERROR: No matched frames found")
        return

    # Process each matched frame group
    all_ground_points = []
    results_csv  = open(out_path / 'ground_points.csv', 'w', newline='')
    writer       = csv.writer(results_csv)
    writer.writerow(['frame', 'spot_id',
                     'x_g', 'y_g', 'z_g',
                     'hdop', 'vdop', 'pdop',
                     'sigma_x', 'sigma_y', 'sigma_z'])

    print(f"\nProcessing {len(matched)} frames...\n")
    print(f"  {'Frame':>6}  {'Spots':>6}  {'Solved':>7}  {'PDOP':>7}")
    print("  " + "-" * 34)

    for frame_idx, frame_group in enumerate(matched):
        timestamp  = frame_group['timestamp']
        all_images = {}
        all_spots  = {}
        rx_states  = {}

        # Load each receiver's image + state at this timestamp
        for rx_id, rx_dir in enumerate(rx_dirs):
            if rx_id not in frame_group:
                continue
            img = cv2.imread(str(frame_group[rx_id]))
            if img is None:
                continue
            all_images[rx_id] = img
            all_spots[rx_id]  = detect_laser_spots(img)

            if rx_id in rx_logs:
                pos, roll, pitch, yaw = get_state(rx_logs[rx_id], timestamp)
            else:
                pos = np.zeros(3); roll = pitch = yaw = 0.0
            rx_states[rx_id] = (pos, roll, pitch, yaw)

        if len(all_spots) < 2:
            continue

        # Get transmitter state at this timestamp
        if tx_log is not None:
            tx_pos, tx_roll, tx_pitch, tx_yaw = get_state(tx_log, timestamp)
        else:
            # Fallback: use receiver 0 position + 50m up
            rx0_pos = rx_states[0][0]
            tx_pos  = rx0_pos.copy(); tx_pos[2] += 50.0
            tx_roll = tx_pitch = tx_yaw = 0.0

        # Transmitter beam direction corrected for attitude
        A_t = transmitter_beam_vector(tx_roll, tx_pitch, tx_yaw)

        # Solve ground point for each detected spot
        ref_spots     = all_spots.get(0, [])
        frame_gpoints = {}
        n_solved      = 0
        mean_pdop     = 0.0

        for spot_id, (px0, py0, _, _) in enumerate(ref_spots[:50]):

            # Receiver 0 unit vector corrected for its attitude
            pos0, roll0, pitch0, yaw0 = rx_states[0]
            A_r0 = pixel_to_world_vector(px0, py0, roll0, pitch0, yaw0)
            receivers = [(pos0, A_r0)]

            # Add other receivers
            for rx_id in range(1, len(rx_dirs)):
                if rx_id not in all_spots or not all_spots[rx_id]:
                    continue
                # Match spot by closest position in image
                best = all_spots[rx_id][0]
                px_r, py_r = best[0], best[1]
                pos_r, roll_r, pitch_r, yaw_r = rx_states[rx_id]
                A_r = pixel_to_world_vector(px_r, py_r,
                                             roll_r, pitch_r, yaw_r)
                receivers.append((pos_r, A_r))

            if len(receivers) < 2:
                continue

            # Solve! Paper Equations 4-12
            r_g, cov, ok = solve_ground_point(
                tx_pos, A_t, receivers, ground_z_approx=0.0)

            if ok and r_g is not None and cov is not None:
                hdop, vdop, pdop = compute_dop(cov)
                if pdop < 5.0 and np.isfinite(pdop):
                    all_ground_points.append(r_g)
                    frame_gpoints[spot_id] = r_g
                    n_solved  += 1
                    mean_pdop += pdop
                    writer.writerow([
                        frame_idx, spot_id,
                        f"{r_g[0]:.4f}", f"{r_g[1]:.4f}", f"{r_g[2]:.4f}",
                        f"{hdop:.4f}", f"{vdop:.4f}", f"{pdop:.4f}",
                        f"{math.sqrt(max(cov[0],0)):.4f}",
                        f"{math.sqrt(max(cov[1],0)):.4f}",
                        f"{math.sqrt(max(cov[2],0)):.4f}",
                    ])

        if n_solved > 0:
            mean_pdop /= n_solved

        total_spots = sum(len(v) for v in all_spots.values())
        print(f"  {frame_idx:>6}  {total_spots:>6}  {n_solved:>7}  "
              f"{mean_pdop:>7.3f}")

        # Save side-by-side visualization every 5 frames
        if frame_idx % 5 == 0 and all_images:
            save_visualization(all_images, all_spots, frame_gpoints,
                               frame_idx, timestamp, out_path)

    results_csv.close()

    print(f"\nDone!")
    print(f"  Total ground points: {len(all_ground_points)}")
    print(f"  Results CSV:  {out_path / 'ground_points.csv'}")
    print(f"  Images:       {out_path / 'visualizations/'}")

    # Save top-down point cloud plot
    if all_ground_points:
        pts = np.array(all_ground_points)
        fig, ax = plt.subplots(figsize=(8, 8))
        sc = ax.scatter(pts[:, 0], pts[:, 1],
                         c=pts[:, 2], cmap='viridis', s=15, alpha=0.7)
        plt.colorbar(sc, label='Z height (m)')
        ax.set_xlabel('X — East (m)')
        ax.set_ylabel('Y — North (m)')
        ax.set_title(f'{len(pts)} solved ground points')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plot_path = out_path / 'point_cloud.png'
        plt.savefig(str(plot_path), dpi=150)
        plt.close()
        print(f"  Point cloud: {plot_path}")


# -----------------------------------------------------------------------
# ENTRY POINT
# -----------------------------------------------------------------------

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Async lidar post-flight processor')
    parser.add_argument('--flight_dir', required=True,
        help='Folder with transmitter/ and receiver_0,1,2/ subfolders')
    parser.add_argument('--output', default='./results',
        help='Output folder for results')
    args = parser.parse_args()
    process(args.flight_dir, args.output)
