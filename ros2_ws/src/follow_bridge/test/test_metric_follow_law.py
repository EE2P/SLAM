"""Offline checks for the metric follow law (no ROS, no robot, no deps).

Exercises the sign/gating/clamp behaviour of compute_metric_cmd. The law is vendored
into the follow_bridge package, so put the package root on sys.path.
"""
import math
import sys
from pathlib import Path

# .../ros2_ws/src/follow_bridge/test/<this> -> package root (holds follow_bridge/) is parents[1].
PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))

from follow_bridge.metric_follow_law import (  # noqa: E402
    MetricFollowParams, compute_metric_cmd,
)

P = MetricFollowParams()


def test_too_far_drives_forward_capped():
    linear, yaw = compute_metric_cmd(4.0, 0.0, 1.0, pixel_x=320.0, valid_depth_pixels=100, p=P)
    assert linear == P.MAX_LINEAR          # far error saturates the forward cap
    assert yaw == 0.0                       # centred -> no yaw


def test_too_close_reverses_capped():
    linear, yaw = compute_metric_cmd(0.4, 0.0, 1.0, pixel_x=320.0, valid_depth_pixels=100, p=P)
    assert linear == -P.MAX_REVERSE         # too close -> reverse, capped
    assert yaw == 0.0


def test_within_stop_band_holds():
    linear, _ = compute_metric_cmd(P.DESIRED_DISTANCE_M, 0.0, 1.0,
                                   pixel_x=320.0, valid_depth_pixels=100, p=P)
    assert linear == 0.0


def test_bearing_sign_right_is_positive_yaw():
    _, yaw_right = compute_metric_cmd(P.DESIRED_DISTANCE_M, 1.0, 1.0,
                                      pixel_x=320.0, valid_depth_pixels=100, p=P)
    _, yaw_left = compute_metric_cmd(P.DESIRED_DISTANCE_M, -1.0, 1.0,
                                     pixel_x=320.0, valid_depth_pixels=100, p=P)
    assert yaw_right > 0.0                   # target to the right -> yaw right (law convention)
    assert yaw_left < 0.0
    assert yaw_right == -yaw_left


def test_bearing_dead_zone():
    _, yaw = compute_metric_cmd(P.DESIRED_DISTANCE_M, 0.005, 1.0,
                                pixel_x=320.0, valid_depth_pixels=100, p=P)
    assert yaw == 0.0                        # bearing inside DEAD_ZONE_RAD


def test_forward_gated_when_off_centre():
    # Far (would drive forward) but well off to the side -> forward suppressed, yaw only.
    linear, yaw = compute_metric_cmd(4.0, 3.0, 1.0, pixel_x=600.0, valid_depth_pixels=100, p=P)
    assert linear == 0.0
    assert yaw > 0.0


def test_yaw_clamped_to_max():
    _, yaw = compute_metric_cmd(P.DESIRED_DISTANCE_M, 100.0, 1.0,
                                pixel_x=320.0, valid_depth_pixels=100, p=P)
    assert yaw == P.MAX_ANGULAR


def test_nan_distance_falls_back_to_pixel_yaw():
    nan = float('nan')
    linear, yaw = compute_metric_cmd(nan, nan, nan, pixel_x=500.0, valid_depth_pixels=0, p=P)
    assert linear == 0.0                     # no trusted range -> never drive forward
    assert yaw > 0.0                          # pixel right of centre -> yaw right
    # and the opposite side:
    _, yaw_left = compute_metric_cmd(nan, nan, nan, pixel_x=140.0, valid_depth_pixels=0, p=P)
    assert yaw_left < 0.0


def test_low_depth_support_is_untrusted():
    # Far distance but too few depth pixels -> treated as untrusted -> no forward drive.
    linear, yaw = compute_metric_cmd(4.0, 0.0, 1.0, pixel_x=320.0,
                                     valid_depth_pixels=P.MIN_VALID_DEPTH_PIXELS - 1, p=P)
    assert linear == 0.0
    assert yaw == 0.0                         # pixel centred -> no fallback yaw either


def test_out_of_range_distance_is_untrusted():
    linear, _ = compute_metric_cmd(P.MAX_RANGE_M + 1.0, 0.0, 1.0,
                                   pixel_x=320.0, valid_depth_pixels=100, p=P)
    assert linear == 0.0


def test_outputs_are_finite():
    linear, yaw = compute_metric_cmd(2.0, 0.5, 2.0, pixel_x=400.0, valid_depth_pixels=100, p=P)
    assert math.isfinite(linear) and math.isfinite(yaw)
