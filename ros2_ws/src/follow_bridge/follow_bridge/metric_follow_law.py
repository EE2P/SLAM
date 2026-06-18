"""Metric person-follow control law (depth-based distance + bearing).

Sibling of follow_law.py. Where `compute_cmd` steers on a PIXEL offset and proxies
distance by box AREA fraction (the on-Pi Hailo follower has no depth), this law
consumes REAL metric data from a depth camera: the target's distance in meters and
its 3D position in the camera optical frame. That lets the follow distance be set
and held in actual meters instead of an area fraction.

Used by the ROS 2 follow bridge (ros2_ws/src/follow_bridge) that grafts the Jetson
`person_distance` detector onto the Momento wheel-legged robot: the bridge picks one
tracked person, calls `compute_metric_cmd`, and publishes the result as a Twist on
/cmd_vel for the robot driver to realise.

Pure Python, zero third-party deps (no numpy/cv2/ROS) so it imports anywhere and is
unit-testable on a bare interpreter -- same philosophy as follow_law.py. It reuses
that module's ideas verbatim (a `clamp`, a tunable dataclass, a stop-band dead-zone,
and a forward gate); only the units change from pixels/area to meters/radians.

Sign convention (camera OPTICAL frame: x right, y down, z forward):
    pos_x > 0  (target to the right)  -> bearing > 0 -> angular > 0 -> yaw right
    distance > desired                -> linear > 0  -> drive forward
    distance < desired                -> linear < 0  -> reverse (too close)
This is the law's OWN convention (matching compute_cmd). Mapping it onto a chassis /
to REP-103 /cmd_vel (where +angular.z is a LEFT turn) is the caller's job via its
DRIVE_SIGN / YAW_SIGN, exactly as stalker/pi/hailo_follower.py applies them at the
send site -- so those knobs stay OUT of this pure law.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, fields


@dataclass
class MetricFollowParams:
    """Runtime-tunable metric person-follow gains (distance in m, bearing in rad)."""

    DESIRED_DISTANCE_M: float = 1.5     # hold this range to the target (m)
    STOP_BAND_M: float = 0.2            # +/- hold band around the setpoint (m): hold
                                        # (zero drive) within this of DESIRED_DISTANCE_M,
                                        # then ramp from zero at the band edge -> kills the
                                        # forward<->reverse chatter at the follow distance
                                        # (the metric twin of follow_law's STOP_BAND_AREA).
    KP_LINEAR_M: float = 0.6            # m/s per meter of range error
    KP_YAW_BEARING: float = 1.5         # rad/s per radian of target bearing
    DEAD_ZONE_RAD: float = 0.05         # no yaw while bearing within this of centre (rad)
    FORWARD_GATE_RAD: float = 0.35      # don't drive forward until the target is roughly
                                        # centred -> stops charging a target off to the side
                                        # (the metric twin of follow_law's FORWARD_GATE_PX).
    MAX_LINEAR: float = 0.4             # m/s forward cap
    MAX_REVERSE: float = 0.4            # m/s reverse cap (back away when too close)
    MAX_ANGULAR: float = 1.5            # rad/s yaw cap
    RECOVERY_YAW: float = 0.3           # rad/s gentle search rotation on target loss (the
                                        # bridge applies this; exposed here for one tuning home)
    MAX_RANGE_M: float = 8.0            # ignore a target / its depth beyond this (m)
    MIN_VALID_DEPTH_PIXELS: int = 50    # require at least this much depth support to trust distance
    CONF_MIN: float = 0.3              # min detection confidence to act on (used by the selector)
    # Steering fallback when depth is missing/untrusted: yaw on the pixel offset instead of
    # the metric bearing, so the robot can still centre on the target and re-acquire depth.
    IMAGE_WIDTH_PX: float = 640.0       # frame width for the pixel-offset fallback centre
    KP_YAW_PIXEL: float = 0.004         # rad/s per pixel of horizontal offset (fallback only)
    DEAD_ZONE_PX: float = 40.0          # pixel dead-zone for the fallback yaw

    @classmethod
    def field_names(cls) -> tuple[str, ...]:
        return tuple(f.name for f in fields(cls))


# Live, mutable default parameter set (mirrors follow_law.PARAMS). Mutate fields in
# place to change the behaviour of callers that rely on the default `p`.
PARAMS = MetricFollowParams()


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _finite(x) -> bool:
    return isinstance(x, (int, float)) and math.isfinite(x)


def _stop_band_ramp(err: float, band: float) -> float:
    """Subtractive dead-band: 0 within +/-band, then ramp from zero at the edge.

    Identical shape to follow_law.compute_cmd's STOP_BAND_AREA handling.
    """
    if err > band:
        return err - band          # too far -> forward
    if err < -band:
        return err + band          # too close -> reverse
    return 0.0                      # within the band -> hold position


def depth_trustworthy(distance_m, pos_x, pos_z, valid_depth_pixels,
                      p: MetricFollowParams = PARAMS) -> bool:
    """True when distance/position are usable for metric (range-holding) control.

    Requires a finite in-range distance, a finite forward position, and enough depth
    support. When False the caller should steer on pixels only and not drive forward.
    """
    if not (_finite(distance_m) and 0.0 < distance_m <= p.MAX_RANGE_M):
        return False
    if not (_finite(pos_x) and _finite(pos_z) and pos_z > 1e-3):
        return False
    if valid_depth_pixels is not None and valid_depth_pixels < p.MIN_VALID_DEPTH_PIXELS:
        return False
    return True


def _pixel_fallback_yaw(pixel_x, p: MetricFollowParams) -> float:
    if not _finite(pixel_x):
        return 0.0
    offset = pixel_x - p.IMAGE_WIDTH_PX / 2.0
    if abs(offset) <= p.DEAD_ZONE_PX:
        return 0.0
    return p.KP_YAW_PIXEL * offset


def compute_metric_cmd(distance_m, pos_x, pos_z, *, pixel_x=None,
                       valid_depth_pixels=None, p: MetricFollowParams = PARAMS):
    """Target distance/position -> (linear_mps, angular_radps) in the law's convention.

    distance_m: metric range to the target (m); may be NaN/None when depth is missing.
    pos_x, pos_z: target position in the camera optical frame (m), x right / z forward;
        used for the bearing-based yaw. May be NaN/None.
    pixel_x: target horizontal pixel centre (used only for the no-depth yaw fallback).
    valid_depth_pixels: depth-support count; below p.MIN_VALID_DEPTH_PIXELS the distance
        is treated as untrustworthy and we fall back to pixel steering (yaw only).

    Forward motion is gated until the target is roughly centred; reverse is always
    allowed (safety). Returns clamped (linear, angular); the caller applies any chassis
    sign flips. See the module docstring for the sign convention.
    """
    if depth_trustworthy(distance_m, pos_x, pos_z, valid_depth_pixels, p):
        bearing = math.atan2(pos_x, pos_z)             # rad, >0 target to the right
        yaw = p.KP_YAW_BEARING * bearing if abs(bearing) > p.DEAD_ZONE_RAD else 0.0
        err_eff = _stop_band_ramp(distance_m - p.DESIRED_DISTANCE_M, p.STOP_BAND_M)
        linear = p.KP_LINEAR_M * err_eff
        if linear > 0.0 and abs(bearing) > p.FORWARD_GATE_RAD:
            linear = 0.0                               # gate forward until centred
    else:
        # No trusted range: steer toward the target on pixels, but never drive forward.
        yaw = _pixel_fallback_yaw(pixel_x, p)
        linear = 0.0

    return (clamp(linear, -p.MAX_REVERSE, p.MAX_LINEAR),
            clamp(yaw, -p.MAX_ANGULAR, p.MAX_ANGULAR))
