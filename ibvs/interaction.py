"""
Interaction matrix of [P1] for the 4 controllable degrees of freedom (v_x, v_y, v_z, w_z):

    [P1] Eq.(10), p.5:      | L_xn |   | -1   0   0   y_n |
                       L_s =| L_yn | = |  0  -1   0  -x_n |          s_dot = L_s v_c ,  v_c = [vx vy vz wz]^T
                            | L_an |   |  0   0  -1    0  |
                            | L_al |   |  0   0   0   -1  |

It is the 3x6 matrix of Eq.(7) (normalised x_n, y_n, a_n) reduced to the columns vx, vy, vz, wz (Eq.(8)),
plus the orientation row L_alpha = [0 0 0 -1] (Eq.(9) with columns 4, 5 removed). It holds for an
eye-in-hand camera observing a static planar object parallel to the image plane (A = B = 0, [P1] p.4).

Dimensions: s (4,), L_s (4, 4), L_s^+ (4, 4), v_c (4,).   det(L_s) = 1, so L_s is never singular; its
condition number grows only with |x_n|, |y_n| (marker far from the optical axis).

Estimate used in the control law ([P1] p.5, citing [18], [23]):
        L_hat^+ = 1/2 ( L_s(s)^+ + L_s(s*)^+ )
i.e. the MEAN OF THE PSEUDO-INVERSES at the current and the desired features (not the pseudo-inverse of
the mean matrix). "current" and "desired" options are provided for comparison experiments only.
"""

import numpy as np


ESTIMATES = ("mean_pinv", "current", "desired")


def interaction_matrix(s):
    """L_s(s) of [P1] Eq.(10). s = [x_n, y_n, a_n, alpha]."""
    x_n, y_n = float(s[0]), float(s[1])
    return np.array([
        [-1.0, 0.0, 0.0, y_n],
        [0.0, -1.0, 0.0, -x_n],
        [0.0, 0.0, -1.0, 0.0],
        [0.0, 0.0, 0.0, -1.0],
    ])


def pseudo_inverse(matrix):
    return np.linalg.pinv(matrix)


def interaction_estimate_pinv(s, s_star, estimate="mean_pinv"):
    """L_hat^+ used in Eq.(1)."""
    if estimate == "mean_pinv":
        return 0.5 * (pseudo_inverse(interaction_matrix(s)) + pseudo_inverse(interaction_matrix(s_star)))
    if estimate == "current":
        return pseudo_inverse(interaction_matrix(s))
    if estimate == "desired":
        return pseudo_inverse(interaction_matrix(s_star))
    raise ValueError(f"estimate must be one of {ESTIMATES}")


def matrix_diagnostics(matrix):
    """Rank, singular values and condition number (logged every cycle)."""
    singular_values = np.linalg.svd(matrix, compute_uv=False)
    smallest = singular_values[-1]
    return {
        "rank": int(np.sum(singular_values > 1e-9 * max(1.0, singular_values[0]))),
        "singular_values": singular_values,
        "condition": float(singular_values[0] / smallest) if smallest > 0 else float("inf"),
        "det": float(np.linalg.det(matrix)) if matrix.shape[0] == matrix.shape[1] else float("nan"),
    }
