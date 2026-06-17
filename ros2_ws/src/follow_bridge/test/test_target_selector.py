"""Offline checks for the track_id lock/hold/search/stop policy (no ROS, no deps).

The selector lives in the follow_bridge package; put the package root on sys.path so
`follow_bridge.target_selector` imports without colcon.
"""
import sys
from pathlib import Path

# .../ros2_ws/src/follow_bridge/test/<this> -> package root (holds follow_bridge/) is parents[1].
PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))

from follow_bridge.target_selector import (  # noqa: E402
    Candidate, TargetSelector, FOLLOW, HOLD, SEARCH, IDLE, LOCKED, STOPPED,
)


def cand(track_id, *, distance=2.0, pos_x=0.0, pos_z=2.0, pixel_x=320.0,
         confidence=0.9, vdp=100):
    return Candidate(track_id, distance, pos_x, pos_z, pixel_x, confidence, vdp)


def _sel(**kw):
    kw.setdefault('lost_hold_s', 0.5)
    kw.setdefault('reacquire_s', 2.0)
    return TargetSelector(**kw)


def test_initial_acquire_locks_nearest():
    s = _sel()
    d = s.update([cand(1, distance=2.0), cand(2, distance=1.0)], now=0.0)
    assert d.action == FOLLOW and d.state == LOCKED
    assert d.track_id == 2 and d.target.track_id == 2   # nearest acquirable wins


def test_locked_target_survives_a_closer_stranger():
    s = _sel()
    s.update([cand(1, distance=2.0)], now=0.0)            # lock id 1
    d = s.update([cand(1, distance=2.0), cand(2, distance=0.5)], now=0.1)
    assert d.action == FOLLOW and d.target.track_id == 1  # ignore the closer stranger


def test_lost_timeline_hold_then_search_then_stop():
    s = _sel()
    s.update([cand(1)], now=0.0)                          # LOCKED
    assert s.update([], now=1.0).action == HOLD          # t_lost=1.0, elapsed 0
    assert s.update([], now=1.3).action == HOLD          # elapsed 0.3 < 0.5
    assert s.update([], now=1.6).action == SEARCH        # 0.5 <= 0.6 < 2.0
    stopped = s.update([], now=3.1)                       # elapsed 2.1 >= 2.0
    assert stopped.action == IDLE and stopped.state == STOPPED


def test_search_yaw_sign_follows_last_seen_side():
    s = _sel()
    s.update([cand(1, pos_x=1.0)], now=0.0)               # last seen to the right -> +1
    s.update([], now=1.0)                                 # first miss -> t_lost=1.0, HOLD
    d = s.update([], now=1.6)                             # elapsed 0.6 -> SEARCH window
    assert d.action == SEARCH and d.last_seen_sign == 1.0


def test_reacquire_same_id_relocks():
    s = _sel()
    s.update([cand(1)], now=0.0)
    s.update([], now=1.0)                                 # HOLD
    d = s.update([cand(1)], now=1.6)                      # same id returns
    assert d.action == FOLLOW and d.state == LOCKED and d.track_id == 1


def test_no_auto_lock_stranger_after_stop():
    s = _sel()
    s.update([cand(1)], now=0.0)
    s.update([], now=1.0)                                 # first miss -> t_lost=1.0
    s.update([], now=3.5)                                 # elapsed 2.5 >= reacquire -> STOPPED
    d = s.update([cand(2)], now=3.6)                      # a valid stranger appears
    assert d.action == IDLE and d.state == STOPPED        # stays stopped, no auto-lock


def test_reset_allows_acquiring_again():
    s = _sel()
    s.update([cand(1)], now=0.0)
    s.update([], now=1.0)                                 # first miss -> t_lost=1.0
    assert s.update([], now=3.5).state == STOPPED         # past reacquire -> STOPPED
    s.reset()
    d = s.update([cand(2)], now=4.0)
    assert d.action == FOLLOW and d.track_id == 2


def test_acquire_gates_reject_weak_candidates():
    for bad in (cand(1, confidence=0.1),                  # low confidence
                cand(1, distance=float('nan')),           # no depth
                cand(1, vdp=10),                           # too little depth support
                cand(1, distance=99.0),                   # out of range
                cand(-1)):                                 # untracked (no track_id)
        s = _sel()
        d = s.update([bad], now=0.0)
        assert d.action == IDLE, f'should not lock {bad}'


def test_lock_maintained_through_depth_dropout():
    s = _sel()
    s.update([cand(1, distance=2.0, vdp=100)], now=0.0)   # LOCKED
    d = s.update([cand(1, distance=float('nan'), vdp=0)], now=0.1)
    assert d.action == FOLLOW and d.target.track_id == 1  # id present -> keep lock
