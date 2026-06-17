"""Offline checks for the nearest-person follow policy (no ROS, no deps).

The selector lives in the follow_bridge package; put the package root on sys.path so
`follow_bridge.target_selector` imports without colcon.
"""
import sys
from pathlib import Path

# .../ros2_ws/src/follow_bridge/test/<this> -> package root (holds follow_bridge/) is parents[1].
PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))

from follow_bridge.target_selector import (  # noqa: E402
    Candidate, TargetSelector, FOLLOW, SEARCH, IDLE,
)


def cand(track_id, *, distance=2.0, pos_x=0.0, pos_z=2.0, pixel_x=320.0,
         confidence=0.9, vdp=100):
    return Candidate(track_id, distance, pos_x, pos_z, pixel_x, confidence, vdp)


def _sel(**kw):
    kw.setdefault('search_timeout_s', 2.0)
    return TargetSelector(**kw)


def test_follows_nearest():
    s = _sel()
    d = s.update([cand(1, distance=2.0), cand(2, distance=1.0)], now=0.0)
    assert d.action == FOLLOW
    assert d.target.track_id == 2          # nearest wins


def test_switches_to_a_closer_person():
    s = _sel()
    s.update([cand(1, distance=2.0)], now=0.0)          # following id 1
    d = s.update([cand(1, distance=2.0), cand(2, distance=0.5)], now=0.1)
    assert d.action == FOLLOW and d.target.track_id == 2  # switch to the closer one


def test_untracked_person_is_followable():
    # track_id is irrelevant now: a -1 (untracked) person is still followed.
    s = _sel()
    d = s.update([cand(-1, distance=1.0)], now=0.0)
    assert d.action == FOLLOW and d.target.track_id == -1


def test_nobody_search_then_idle():
    s = _sel(search_timeout_s=2.0)
    s.update([cand(1)], now=0.0)                         # seen
    assert s.update([], now=1.0).action == SEARCH        # within search window
    assert s.update([], now=2.5).action == IDLE          # past search window


def test_reacquires_without_reset():
    # No permanent stop: after going idle, a fresh person is followed immediately.
    s = _sel(search_timeout_s=1.0)
    s.update([cand(1)], now=0.0)
    assert s.update([], now=5.0).action == IDLE          # long gone -> idle
    d = s.update([cand(2, distance=1.5)], now=5.1)        # someone appears
    assert d.action == FOLLOW and d.target.track_id == 2  # follow them, no reset needed


def test_search_yaw_sign_follows_last_seen_side():
    s = _sel()
    s.update([cand(1, pos_x=1.0)], now=0.0)              # last seen to the right -> +1
    d = s.update([], now=0.5)
    assert d.action == SEARCH and d.last_seen_sign == 1.0


def test_gate_rejects_weak_candidates():
    for bad in (cand(1, confidence=0.1),                 # low confidence
                cand(1, distance=float('nan')),          # no depth
                cand(1, vdp=10),                          # too little depth support
                cand(1, distance=99.0)):                 # out of range
        s = _sel()
        d = s.update([bad], now=0.0)
        assert d.action != FOLLOW, f'should not follow {bad}'


def test_nearest_among_valid_only():
    # The closest candidate is invalid (no depth); follow the next-closest valid one.
    s = _sel()
    d = s.update([cand(1, distance=0.5, vdp=10), cand(2, distance=1.2)], now=0.0)
    assert d.action == FOLLOW and d.target.track_id == 2
