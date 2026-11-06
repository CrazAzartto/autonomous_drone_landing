"""Paper 1 features (Eq. 2, p.4-5) and interaction matrix (Eq. 10), verified numerically."""
import math

import numpy as np
import pytest

from helpers import marker_corners_cam, project
from ibvs.features import (desired_from_model, feature_error, features_from_points, model_desired_area,
                           teach_desired)
from ibvs.interaction import interaction_estimate_pinv, interaction_matrix, matrix_diagnostics
from ibvs.moments import discrete_moments

L = 0.04


def test_moments_of_a_square():
    pts = np.array([[-1, 1], [1, 1], [1, -1], [-1, -1]], dtype=float) * 0.5 + [2.0, 3.0]
    m = discrete_moments(pts)
    assert m["m00"] == 4 and m["x_g"] == pytest.approx(2.0) and m["y_g"] == pytest.approx(3.0)
    assert m["mu20"] == pytest.approx(1.0) and m["mu02"] == pytest.approx(1.0) and m["mu11"] == pytest.approx(0.0)


def test_model_area_matches_projection():
    z = 0.8
    pts = project(marker_corners_cam(np.array([0, 0, z]), 0.3, L))
    m = discrete_moments(pts)
    assert m["mu20"] + m["mu02"] == pytest.approx(model_desired_area(L, z), rel=1e-12)


@pytest.mark.parametrize("center,alpha", [((0.1, -0.05, 0.9), 0.3), ((-0.2, 0.15, 0.5), -2.5), ((0, 0, 1.3), 3.1)])
def test_features_equal_metric_position(center, alpha):
    desired = desired_from_model(L, 0.8)
    s = features_from_points(project(marker_corners_cam(np.array(center), alpha, L)), desired)
    assert s.vector()[:3] == pytest.approx(np.array(center), abs=1e-12)
    assert s.alpha == pytest.approx(alpha, abs=1e-12)


def test_alpha_definitions_differ_by_constant():
    desired = desired_from_model(L, 0.8)
    for alpha in (0.0, 1.0, -2.0):
        pts = project(marker_corners_cam(np.array([0.05, 0.02, 0.7]), alpha, L))
        a = features_from_points(pts, desired, "edge_0_1").alpha
        b = features_from_points(pts, desired, "diagonal_0_2").alpha
        assert ((b - a + math.pi) % (2 * math.pi) - math.pi) == pytest.approx(math.pi / 4)   # x_M - y_M


def test_error_wraps_orientation():
    e = feature_error([0, 0, 1, math.radians(179)], [0, 0, 1, math.radians(-179)])
    assert e[3] == pytest.approx(math.radians(-2))


def test_interaction_matrix_shape_and_det():
    L_s = interaction_matrix([0.1, -0.2, 0.8, 0.4])
    assert L_s.shape == (4, 4)
    diag = matrix_diagnostics(L_s)
    assert diag["rank"] == 4 and diag["det"] == pytest.approx(1.0)


@pytest.mark.parametrize("vc", [np.array([0.03, -0.02, 0.05, 0.2]), np.array([-0.1, 0.07, -0.04, -0.5])])
def test_eq10_against_finite_differences_eye_in_hand(vc):
    """Camera moving with v_c observing a static marker: s_dot must equal L_s v_c ([P1] Eq. 10)."""
    desired = desired_from_model(L, 0.8)
    P = marker_corners_cam(np.array([0.12, -0.07, 0.85]), 0.4, L)
    s0 = features_from_points(project(P), desired).vector()
    dt = 1e-6
    P2 = P + dt * (-vc[:3] - np.cross(np.array([0, 0, vc[3]]), P))
    s1 = features_from_points(project(P2), desired).vector()
    assert (s1 - s0) / dt == pytest.approx(interaction_matrix(s0) @ vc, abs=1e-5)


def test_mean_pinv_estimate():
    s, s_star = np.array([0.2, -0.1, 0.9, 0.3]), np.array([0, 0, 0.8, 0])
    expected = 0.5 * (np.linalg.pinv(interaction_matrix(s)) + np.linalg.pinv(interaction_matrix(s_star)))
    assert interaction_estimate_pinv(s, s_star) == pytest.approx(expected)


def test_teach_recovers_desired_image():
    model = desired_from_model(L, 0.8)
    obs = [features_from_points(project(marker_corners_cam(np.array([0.03, 0.01, 0.6]), 0.2, L)), model)
           for _ in range(5)]
    taught = teach_desired(obs, z_star=0.6)
    assert taught.a_star == pytest.approx(model_desired_area(L, 0.6))
    assert taught.x_g_star == pytest.approx(0.05) and taught.alpha_star == pytest.approx(0.2)
    s = features_from_points(project(marker_corners_cam(np.array([0.03, 0.01, 0.6]), 0.2, L)), taught)
    assert np.linalg.norm(feature_error(s.vector(), taught.vector())) < 1e-12
