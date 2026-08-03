"""
Monte Carlo Validation — compares output to Table 4 of the paper.
Run with: python3 test_monte_carlo.py
"""
import numpy as np
import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from async_lidar_math import solve_ground_point, compute_dop, perturb_unit_vector

N_SIMULATIONS = 1000
SIGMA_XY   = 0.02
SIGMA_Z    = 0.04
SIGMA_RP   = math.radians(0.005)   # IMU roll/pitch noise
SIGMA_H    = math.radians(0.010)   # IMU heading noise
SIGMA_SCAN = math.radians(0.0086)  # scan angle noise

# Combined attitude noise: approximate as mean of roll/pitch/heading
SIGMA_ATT  = (2*SIGMA_RP + SIGMA_H) / 3

RX_TABLE = [
    ( 275.0,    0.0),
    (   0.0,  275.0),
    (-275.0,    0.0),
    (   0.0, -275.0),
    ( 275.0,  275.0),
    (-275.0,  275.0),
    (-275.0, -275.0),
    ( 275.0, -275.0),
]

def simulate_nadir(tx_height, rx_height, n_rx):
    true_r_t  = np.array([0.0, 0.0, float(tx_height)])
    true_r_rs = [np.array([x, y, float(rx_height)]) for x,y in RX_TABLE[:n_rx]]
    true_r_g  = np.zeros(3)
    # Nadir unit vector: pointing DOWN in ENU local frame = [0, 0, -1]
    true_u_t  = np.array([0.0, 0.0, -1.0])

    errs_xy, errs_z = [], []

    for _ in range(N_SIMULATIONS):
        # --- Transmitter noise ---
        noisy_r_t = true_r_t + np.array([
            np.random.normal(0, SIGMA_XY),
            np.random.normal(0, SIGMA_XY),
            np.random.normal(0, SIGMA_Z),
        ])
        # Perturb transmitter pointing direction with attitude + scan noise
        A_t = perturb_unit_vector(true_u_t, SIGMA_ATT)
        A_t = perturb_unit_vector(A_t, SIGMA_SCAN)

        # --- Receiver noise ---
        rxs = []
        for true_r_r in true_r_rs:
            noisy_r_r = true_r_r + np.array([
                np.random.normal(0, SIGMA_XY),
                np.random.normal(0, SIGMA_XY),
                np.random.normal(0, SIGMA_Z),
            ])
            # True ENU unit vector from receiver to ground point
            diff   = true_r_g - true_r_r
            u_r    = diff / np.linalg.norm(diff)
            # Perturb with attitude + scan noise
            A_r = perturb_unit_vector(u_r, SIGMA_ATT)
            A_r = perturb_unit_vector(A_r, SIGMA_SCAN)
            rxs.append((noisy_r_r, A_r))

        # --- Solve ---
        r_g, cov, ok = solve_ground_point(
            noisy_r_t, A_t, rxs, ground_z_approx=0.0
        )
        if ok and r_g is not None:
            e = r_g - true_r_g
            errs_xy.append(math.sqrt(e[0]**2 + e[1]**2))
            errs_z.append(abs(e[2]))

    if not errs_xy:
        return None, None
    return float(np.mean(errs_xy)), float(np.mean(errs_z))


if __name__ == '__main__':
    print("=" * 62)
    print("  Asynchronous Lidar — Monte Carlo Validation")
    print("  Reproducing Table 4 (nadir) — Glennie et al. 2025")
    print(f"  {N_SIMULATIONS} simulations per config")
    print("=" * 62)
    print(f"\n  Transmitter height: 500 m  |  Receiver height: 200 m\n")
    print(f"  {'Rcvrs':>5}  {'Got H (m)':>10}  {'Got V (m)':>10}  "
          f"{'Paper H':>9}  {'Paper V':>9}  {'OK?':>5}")
    print("  " + "-" * 54)

    paper = {2:(0.0674,0.0456), 3:(0.0674,0.0360), 4:(0.0674,0.0306),
             5:(0.0674,0.0269), 6:(0.0674,0.0241)}

    all_ok = True
    for n in [2, 3, 4, 5, 6]:
        h, v = simulate_nadir(500, 200, n)
        if h is None:
            print(f"  {n:>5}  FAILED"); continue
        ph, pv = paper[n]
        ok = abs(h - ph) < 0.015 and abs(v - pv) < 0.015
        if not ok: all_ok = False
        mark = "  ✓" if ok else "  ✗"
        print(f"  {n:>5}  {h:>10.4f}  {v:>10.4f}  {ph:>9.4f}  {pv:>9.4f}  {mark}")

    print()
    if all_ok:
        print("  ✓ All values match the paper within ±0.015 m")
    else:
        print("  ✗ Some values differ (expected range: ±0.015 m)")
    print()
    print("  Key insight from the paper:")
    print("  → Nadir horizontal error is CONSTANT (adding receivers doesn't help XY)")
    print("  → Vertical error DECREASES with more receivers")
    print("  → Paper recommends 3–4 receivers as the sweet spot")
