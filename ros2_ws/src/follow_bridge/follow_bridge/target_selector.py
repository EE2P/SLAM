"""Pick and lock ONE tracked person across frames for the follow bridge.

Consumes the per-frame list of detected persons (each a lightweight `Candidate` with
a STABLE track_id, distance, position, pixel centre, confidence, depth support) and
decides which single person the robot should follow, with hysteresis so a momentary
miss or a passing stranger cannot steal the lock.

Pure Python, no ROS / numpy -- the node adapts PersonDistanceArray messages into the
`Candidate` records this module expects, so the policy is unit-testable without a ROS
graph or a camera.

The per-frame `PersonDistance.id` is just a detection index and is NOT used; locking
keys on `track_id` (which the Jetson detector publishes once it runs YOLO in tracking
mode -- see the plan's Part B).

Lost-target policy (chosen with the user -- "search then stop"):
    LOCKED -> (miss) -> HOLD for lost_hold_s (pause in place)
           -> SEARCH (bounded recovery yaw toward the last-seen side) until reacquire_s
           -> if the same track_id reappears: re-LOCK
           -> else: STOPPED (zero command, do NOT auto-lock a stranger; needs reset()).
Initial acquisition DOES auto-lock the nearest valid person so following can start; the
no-auto-lock rule only applies AFTER a target has been lost.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# Decision actions (what the node should command this tick).
FOLLOW = "follow"          # run the metric law on `target`
HOLD = "hold"              # locked but missed briefly -> zero command, wait
SEARCH = "search"          # bounded recovery yaw toward last_seen_sign, zero linear
IDLE = "idle"              # zero command (acquiring with nobody, or stopped)

# Selector states.
ACQUIRING = "acquiring"    # initial / after reset(): will auto-lock the nearest valid person
LOCKED = "locked"
LOST = "lost"
STOPPED = "stopped"        # permanent loss: idle until reset()


@dataclass(frozen=True)
class Candidate:
    track_id: int
    distance: float            # meters; may be nan when depth is missing
    pos_x: float               # meters, camera optical x (right); may be nan
    pos_z: float               # meters, camera optical z (forward); may be nan
    pixel_x: float             # mask centroid x, pixels
    confidence: float          # detection confidence [0..1]
    valid_depth_pixels: int    # depth-support count behind `distance`


@dataclass(frozen=True)
class FollowDecision:
    action: str                # FOLLOW | HOLD | SEARCH | IDLE
    state: str                 # selector state (for telemetry/heartbeat)
    target: Candidate | None   # the person to follow this tick (FOLLOW only)
    last_seen_sign: float      # +1 target last seen to the right, -1 to the left
    track_id: int              # locked id, or -1 when not locked


def _finite(x) -> bool:
    return isinstance(x, (int, float)) and math.isfinite(x)


class TargetSelector:
    """Lock one track_id and keep it through brief misses; see module docstring."""

    def __init__(self, *, conf_min: float = 0.3, max_range_m: float = 8.0,
                 min_valid_depth_pixels: int = 50, lost_hold_s: float = 0.5,
                 reacquire_s: float = 3.0):
        self.conf_min = float(conf_min)
        self.max_range_m = float(max_range_m)
        self.min_valid_depth_pixels = int(min_valid_depth_pixels)
        self.lost_hold_s = float(lost_hold_s)
        self.reacquire_s = float(reacquire_s)

        self.state = ACQUIRING
        self.locked_id = -1
        self.last_seen_sign = 1.0
        self._t_lost: float | None = None

    def reset(self) -> None:
        """Operator re-arm: forget any lock and resume acquiring the nearest person."""
        self.state = ACQUIRING
        self.locked_id = -1
        self._t_lost = None

    # -- candidate gates -----------------------------------------------------
    def _acquirable(self, c: Candidate) -> bool:
        """Strict gate for INITIAL acquisition (needs trusted depth + confidence)."""
        return (
            c.track_id >= 0
            and c.confidence >= self.conf_min
            and _finite(c.distance) and 0.0 < c.distance <= self.max_range_m
            and c.valid_depth_pixels >= self.min_valid_depth_pixels
        )

    def _nearest_acquirable(self, candidates) -> Candidate | None:
        best = None
        for c in candidates:
            if self._acquirable(c) and (best is None or c.distance < best.distance):
                best = c
        return best

    def _find_locked(self, candidates) -> Candidate | None:
        # Maintenance match is by id presence only -- a momentary depth dropout must
        # NOT drop the lock (the law has a pixel-only fallback for NaN distance).
        for c in candidates:
            if c.track_id == self.locked_id:
                return c
        return None

    def _sign_of(self, c: Candidate) -> float:
        if _finite(c.pos_x) and abs(c.pos_x) > 1e-3:
            return 1.0 if c.pos_x > 0 else -1.0
        if _finite(c.pixel_x):
            # No lateral position -> infer side from pixel centre is left to the node's
            # image width; keep the previous sign rather than guess here.
            return self.last_seen_sign
        return self.last_seen_sign

    # -- main update ---------------------------------------------------------
    def update(self, candidates, now: float) -> FollowDecision:
        candidates = list(candidates)

        if self.state == STOPPED:
            return FollowDecision(IDLE, self.state, None, self.last_seen_sign, -1)

        if self.state == ACQUIRING:
            target = self._nearest_acquirable(candidates)
            if target is not None:
                self.locked_id = target.track_id
                self.state = LOCKED
                self._t_lost = None
                self.last_seen_sign = self._sign_of(target)
                return FollowDecision(FOLLOW, self.state, target,
                                      self.last_seen_sign, self.locked_id)
            return FollowDecision(IDLE, self.state, None, self.last_seen_sign, -1)

        # LOCKED or LOST: try to re-find the locked id this frame.
        target = self._find_locked(candidates)
        if target is not None:
            self.state = LOCKED
            self._t_lost = None
            self.last_seen_sign = self._sign_of(target)
            return FollowDecision(FOLLOW, self.state, target,
                                  self.last_seen_sign, self.locked_id)

        # Locked id missing this frame.
        if self.state == LOCKED:
            self.state = LOST
            self._t_lost = now
        elapsed = now - (self._t_lost if self._t_lost is not None else now)
        if elapsed < self.lost_hold_s:
            return FollowDecision(HOLD, self.state, None, self.last_seen_sign, self.locked_id)
        if elapsed < self.reacquire_s:
            return FollowDecision(SEARCH, self.state, None, self.last_seen_sign, self.locked_id)

        # Past the re-acquire window -> permanent loss. Stop; do NOT auto-lock a stranger.
        self.state = STOPPED
        self.locked_id = -1
        self._t_lost = None
        return FollowDecision(IDLE, self.state, None, self.last_seen_sign, -1)
