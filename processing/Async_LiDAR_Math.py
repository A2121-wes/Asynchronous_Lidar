"""
async_lidar_math.py ***This is required to run all other things****
====================
Pure Python/NumPy implementation of the asynchronous lidar solver.
NO ROS2 dependency — can be imported and tested anywhere.

The ROS nodes (transmitter_node.py, receiver_node.py, fusion_node.py)
all import from THIS file for the actual math.

PAPER REFERENCE: Glennie et al. 2025
  Equations implemented here:
    Eq. (2)  — transmitter forward model
    Eq. (3)  — receiver forward model
    Eq. (4)  — functional model
    Eq. (5)  — expanded coordinate form
    Eq. (6)  — range equations
    Eq. (7)  — linearized Taylor expansion
    Eq. (8)  — initial expansion point
    Eq. (9)  — initial range estimates
    Eq. (10) — ground point update
    Eq. (12) — matrix form
    Eq. (13) — cofactor matrix
    Eq. (14) — DOP computation
"""

import numpy as np
import math


# -----------------------------------------------------------------------
# ROTATION MATRIX (used by transmitter and receiver to apply GNSS/IMU)
# -----------------------------------------------------------------------

def rotation_matrix_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Build 3x3 rotation matrix from roll/pitch/yaw (radians).
    This is R_l_b (local-level ← body frame) from the paper.

    Used in Eq. (2): A_t = R_bt * u_t
    Used in Eq. (3): A_r = R_br * u_r
    """
    cx, sx = math.cos(roll),  math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw),   math.sin(yaw)
    return np.array([
        [cy*cz,  cy*sz, -sy],
        [sx*sy*cz - cx*sz, sx*sy*sz + cx*cz,  sx*cy],
        [cx*sy*cz + sx*sz, cx*sy*sz - sx*cz,  cx*cy]
    ])


# -----------------------------------------------------------------------
# BEAM UNIT VECTORS (used by transmitter)
# -----------------------------------------------------------------------

def make_beam_unit_vectors(scan_angles_x_deg: list,
                            scan_angles_y_deg: list) -> np.ndarray:
    """
    Generate unit vectors for a multi-beam laser in scanner body frame.
    Paper Section 3: -30° to +30° in 5° steps → 13×13 = 169 unique angles
    but the 361 count in the paper includes center so 13×13 -(13-1) adjustments...
    Actually: (-30,-25,...,30) = 13 steps per axis → 13*13 = 169 not 361.
    The paper says 361 = 19*19 grid (from -30 to +30 at ~3.16° steps? 
    Let's use the paper's description exactly: "-30 to 30 at 5-degree intervals"
    which gives 13 values per axis → 169 points total.
    We keep this faithful to the paper.

    Returns:
        np.ndarray of shape (N, 3) — unit vectors pointing from scanner toward ground
    """
    vectors = []
    for tx_deg in scan_angles_x_deg:
        for ty_deg in scan_angles_y_deg:
            tx = math.radians(tx_deg)
            ty = math.radians(ty_deg)
            # In ENU local-level frame, DOWN is -Z.
            # Nadir beam (0°,0°) → u = [0, 0, -1]
            # Scan angles deflect from nadir.
            ux =  math.sin(tx)
            uy =  math.sin(ty) * math.cos(tx)
            uz = -math.cos(tx) * math.cos(ty)   # NEGATIVE = downward in ENU
            norm = math.sqrt(ux**2 + uy**2 + uz**2)
            vectors.append([ux/norm, uy/norm, uz/norm])
    return np.array(vectors)


# -----------------------------------------------------------------------
# UNIT VECTOR PERTURBATION
# -----------------------------------------------------------------------

def perturb_unit_vector(u: np.ndarray, sigma_angle_rad: float) -> np.ndarray:
    """
    Perturb a unit vector by a small random angle (Rodrigues rotation).
    This correctly simulates IMU noise on a pointing direction.
    """
    rand = np.random.randn(3)
    axis = np.cross(u, rand)
    norm = np.linalg.norm(axis)
    if norm < 1e-9:
        return u.copy()
    axis /= norm
    angle = np.random.normal(0, sigma_angle_rad)
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    u_perturbed = (cos_a * u + sin_a * np.cross(axis, u) +
                   (1 - cos_a) * np.dot(axis, u) * axis)
    n = np.linalg.norm(u_perturbed)
    return u_perturbed / n if n > 1e-9 else u.copy()


# -----------------------------------------------------------------------
# RAY-PLANE INTERSECTION (initial ground estimate)
# -----------------------------------------------------------------------

def intersect_beam_with_ground(r_t: np.ndarray, A_t: np.ndarray,
                                ground_z: float = 0.0):
    """
    Find where laser beam hits the ground plane Z = ground_z.

    This implements the 'approximate ground elevation' step described
    in Section 2 (the initial estimate for the Gauss-Newton iteration).

    Returns:
        (r_g, d_t) — ground point and transmitter-to-ground distance
        (None, None) if beam is parallel to ground or pointing upward
    """
    # In ENU frame, beam must have negative Z component to hit the ground below
    if abs(A_t[2]) < 1e-9:
        return None, None
    d_t = (ground_z - r_t[2]) / A_t[2]
    if d_t <= 0:
        return None, None
    return r_t + A_t * d_t, d_t


# -----------------------------------------------------------------------
# CORE SOLVER — Equations (4)–(12)
# -----------------------------------------------------------------------

def solve_ground_point(r_t: np.ndarray,
                        A_t: np.ndarray,
                        receivers: list,
                        ground_z_approx: float = 0.0,
                        max_iter: int = 50,
                        tol: float = 1e-6) -> tuple:
    """
    Solve for the 3D ground point given transmitter and N receivers.

    THIS IS THE HEART OF THE PAPER.

    Implements the non-linear least squares from Section 2:
      Functional model (Eq. 4):   F = r_t - r_r = -A_t*d_t + A_r*d_r
      Linearized form  (Eq. 7):   F ≈ F^0 + (dF/dr_g) * Δr_g
      Solution         (Eq. 10):  r_g = r_g^0 + Δr_g

    Args:
        r_t:              [X_t, Y_t, Z_t] — transmitter position (GNSS)
        A_t:              [A_tx, A_ty, A_tz] — transmitter unit vector (rotated)
        receivers:        list of (r_r, A_r) tuples — one per receiver
                          r_r = [X_r, Y_r, Z_r], A_r = receiver unit vector
        ground_z_approx:  initial Z guess (from DEM or known terrain)
        max_iter:         max Gauss-Newton iterations
        tol:              convergence threshold in metres

    Returns:
        (r_g, cov_diag, converged)
        r_g:       [X_g, Y_g, Z_g] — solved ground point
        cov_diag:  [σ²_x, σ²_y, σ²_z] — diagonal of cofactor matrix (Eq. 13)
        converged: True if iteration converged within tol
    """
    N = len(receivers)
    if N < 2:
        return None, None, False

    # Step 1: Initial estimate (Eq. 8–9)
    # Use the transmitter-to-ground ray to seed the iteration
    if abs(A_t[2]) < 1e-9:
        return None, None, False

    d_t0 = (ground_z_approx - r_t[2]) / A_t[2]
    if d_t0 <= 0:
        return None, None, False

    r_g = r_t + A_t * d_t0   # Initial estimate of ground point

    # Step 2: Iterate (Gauss-Newton)
    for _ in range(max_iter):

        # Build design matrix (J) and misclosure vector (w)
        # Dimensions: rows = 3*N (3 coordinate equations per receiver)
        #             cols = 3   (unknowns: Δx_g, Δy_g, Δz_g)
        J = np.zeros((3 * N, 3))
        w = np.zeros(3 * N)

        for i, (r_r, A_r) in enumerate(receivers):

            # Current range estimates (Eq. 9)
            d_t = np.linalg.norm(r_t - r_g)
            d_r = np.linalg.norm(r_r - r_g)

            if d_t < 1e-9 or d_r < 1e-9:
                return None, None, False

            # Modeled observation F^0 (Eq. 8)
            F0 = -A_t * d_t + A_r * d_r

            # Measured observation (Eq. 5): F_meas = r_t - r_r
            F_meas = r_t - r_r

            # Misclosure: w = F_meas - F^0
            row = i * 3
            w[row:row+3] = F_meas - F0

            # Partial derivatives: ∂F/∂r_g
            # ∂d_t/∂r_g = (r_g - r_t) / d_t
            # ∂d_r/∂r_g = (r_g - r_r) / d_r
            # ∂F/∂r_g   = -A_t ⊗ ∂d_t/∂r_g + A_r ⊗ ∂d_r/∂r_g
            dd_t = (r_g - r_t) / d_t   # shape (3,)
            dd_r = (r_g - r_r) / d_r   # shape (3,)
            J[row:row+3, :] = -np.outer(A_t, dd_t) + np.outer(A_r, dd_r)

        # Solve: Δr_g = (J^T J)^{-1} J^T w
        JtJ = J.T @ J
        if abs(np.linalg.det(JtJ)) < 1e-12:
            return None, None, False  # Singular — co-planar geometry

        delta = np.linalg.solve(JtJ, J.T @ w)
        r_g   = r_g + delta

        if np.linalg.norm(delta) < tol:
            # Converged — compute cofactor matrix Q = (J^T J)^{-1} (Eq. 13)
            try:
                Q = np.linalg.inv(JtJ)
                cov_diag = np.array([Q[0, 0], Q[1, 1], Q[2, 2]])
            except np.linalg.LinAlgError:
                cov_diag = np.full(3, np.nan)
            return r_g, cov_diag, True

    return r_g, None, False   # Did not converge


# -----------------------------------------------------------------------
# DOP COMPUTATION — Equation (14)
# -----------------------------------------------------------------------

def compute_dop(cov_diag: np.ndarray) -> tuple:
    """
    Dilution of Precision from cofactor matrix diagonal (Eq. 14).

    HDOP = sqrt(σ²_x + σ²_y)               — horizontal precision
    VDOP = sqrt(σ²_z)                        — vertical precision
    PDOP = sqrt(σ²_x + σ²_y + σ²_z)        — 3D positional precision

    Lower DOP = stronger geometry = more reliable solution.
    Paper Section 4: networks with PDOP > ~5 are considered weak.

    Returns: (hdop, vdop, pdop)
    """
    sx2, sy2, sz2 = (max(float(v), 0.0) for v in cov_diag)
    return (
        math.sqrt(sx2 + sy2),           # HDOP
        math.sqrt(sz2),                  # VDOP
        math.sqrt(sx2 + sy2 + sz2),      # PDOP
    )
