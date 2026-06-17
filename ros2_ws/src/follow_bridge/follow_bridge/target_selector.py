"""Follow the NEAREST valid person, every frame (no identity locking).

Each frame we simply pick the closest acceptable person and follow them. If the
nearest person changes, we switch to them immediately. There is NO persistent lock
and NO permanent "stop" state, so following resumes the instant a valid person
reappears -- you never have to reset or restart the node.

When nobody valid is visible we rotate briefly toward where the target was last seen
(to help re-find them) for `search_timeout_s`, then hold still.

`track_id` is NOT used for selection (it is carried in `Candidate` for telemetry
only); a person is followable even when the detector reports it untracked (-1).

Pure Python, no ROS / numpy -- the node adapts PersonDistanceArray messages into the
`Candidate` records this module expects, so the policy is unit-testable without a ROS
graph or a camera.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# Decision actions (what the node should command this tick).
FOLLOW = "follow"          # run the metric law on `target`
SEARCH = "search"          # bounded recovery yaw toward last_seen_sign, zero linear
IDLE = "idle"              # zero command (nobody visible and search window elapsed)

# Selector states (for telemetry/heartbeat).
FOLLOWING = "following"
SEARCHING = "searching"
IDLING = "idle"


@dataclass(frozen=True)
class Candidate:
    track_id: int              # carried for telemetry only; NOT used for selection
    distance: float            # meters; may be nan when depth is missing
    pos_x: float               # meters, camera optical x (right); may be nan
    pos_z: float               # meters, camera optical z (forward); may be nan
    pixel_x: float             # mask centroid x, pixels
    confidence: float          # detection confidence [0..1]
    valid_depth_pixels: int    # depth-support count behind `distance`


@dataclass(frozen=True)
class FollowDecision:
    action: str                # FOLLOW | SEARCH | IDLE
    state: str                 # selector state (for telemetry/heartbeat)
    target: Candidate | None   # the person to follow this tick (FOLLOW only)
    last_seen_sign: float      # +1 target last seen to the right, -1 to the left
    track_id: int              # followed person's id (telemetry), or -1 when none


def _finite(x) -> bool:
    return isinstance(x, (int, float)) and math.isfinite(x)


class TargetSelector:
    """Follow the nearest valid person each frame; see module docstring."""

    def __init__(self, *, conf_min: float = 0.3, max_range_m: float = 8.0,
                 min_valid_depth_pixels: int = 50, search_timeout_s: float = 2.0):
        self.conf_min = float(conf_min)
        self.max_range_m = float(max_range_m)
        self.min_valid_depth_pixels = int(min_valid_depth_pixels)
        self.search_timeout_s = float(search_timeout_s)

        self.last_seen_sign = 1.0
        self._t_last_seen: float | None = None

    def reset(self) -> None:
        """Kept for API compatibility; the nearest policy holds no lock to clear."""
        self._t_last_seen = None

    # -- candidate gate ------------------------------------------------------
    def _valid(self, c: Candidate) -> bool:
        """Acceptable to follow: trusted depth + confidence (track_id is irrelevant)."""
        return (
            c.confidence >= self.conf_min
            and _finite(c.distance) and 0.0 < c.distance <= self.max_range_m
            and c.valid_depth_pixels >= self.min_valid_depth_pixels
        )

    def _nearest(self, candidates) -> Candidate | None:
        best = None
        for c in candidates:
            if self._valid(c) and (best is None or c.distance < best.distance):
                best = c
        return best

    def _sign_of(self, c: Candidate) -> float:
        if _finite(c.pos_x) and abs(c.pos_x) > 1e-3:
            return 1.0 if c.pos_x > 0 else -1.0
        return self.last_seen_sign

    # -- main update ---------------------------------------------------------
    def update(self, candidates, now: float) -> FollowDecision:
        target = self._nearest(list(candidates))
        if target is not None:
            self.last_seen_sign = self._sign_of(target)
            self._t_last_seen = now
            return FollowDecision(FOLLOW, FOLLOWING, target,
                                  self.last_seen_sign, target.track_id)

        # Nobody valid this frame: search briefly toward the last-seen side, then idle.
        if self._t_last_seen is not None and (now - self._t_last_seen) < self.search_timeout_s:
            return FollowDecision(SEARCH, SEARCHING, None, self.last_seen_sign, -1)
        return FollowDecision(IDLE, IDLING, None, self.last_seen_sign, -1)
