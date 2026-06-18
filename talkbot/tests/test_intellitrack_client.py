import asyncio

import pytest

from robot_mcp import config
from robot_mcp.intellitrack_client import ZERO_DRIVE, IntellitrackClient, format_drive


@pytest.fixture
async def client(fake_intellitrack):
    base_url, state = fake_intellitrack
    c = IntellitrackClient(base_url)
    yield c, state
    await c.close()


async def test_arm_sends_E(client):
    c, state = client
    result = await c.arm()
    assert result == {"armed": True}
    await asyncio.sleep(0.05)
    assert "E" in state.serial_lines


async def test_drive_clamps_and_zeroes(client):
    c, state = client
    await c.arm()
    result = await c.drive(0.5, 9.0, 99.0)
    assert result["speed_m_s"] == pytest.approx(config.MAX_SPEED_M_S)
    assert result["yaw_rad_s"] == pytest.approx(config.MAX_YAW_RAD_S)
    assert result["duration_s"] == pytest.approx(config.MAX_DRIVE_S)
    # don't wait the full 5 s: cancel — the loop must still send the zero frame
    await asyncio.sleep(0.35)
    await c._cancel_drive()
    await asyncio.sleep(0.05)
    lines = state.serial_lines
    drive_line = format_drive(config.MAX_SPEED_M_S, config.MAX_YAW_RAD_S)
    assert drive_line == "C 0.1500 1.5000"
    sent = [l for l in lines if l == drive_line]
    # 0.35 s at 10 Hz -> at least 3 resends, each gap < 0.25 s stale threshold
    assert len(sent) >= 3
    assert lines[-1] == ZERO_DRIVE


async def test_drive_refused_when_disarmed(client):
    c, _ = client
    result = await c.drive(0.1)
    assert "error" in result


async def test_drive_refused_when_autonomy_on(client):
    c, state = client
    await c.arm()
    state.stalker["autonomy"] = True
    result = await c.drive(0.1)
    assert "error" in result


async def test_new_drive_replaces_old(client):
    c, state = client
    await c.arm()
    await c.drive(0.1, 0.0, 5.0)
    await asyncio.sleep(0.15)
    await c.drive(-0.1, 0.0, 0.2)
    await asyncio.sleep(0.4)
    lines = state.serial_lines
    # first drive was cancelled -> zero frame, then reverse drive, then final zero
    assert format_drive(0.1, 0.0) in lines
    assert format_drive(-0.1, 0.0) in lines
    assert lines[-1] == ZERO_DRIVE


async def test_emergency_stop_sequence(client):
    c, state = client
    await c.arm()
    state.stalker["autonomy"] = True
    result = await c.emergency_stop()
    assert result["stopped"] is True
    await asyncio.sleep(0.05)
    assert ZERO_DRIVE in state.serial_lines
    assert "D" in state.serial_lines
    assert state.stalker["autonomy"] is False
    assert c.armed_intent is False


async def test_telemetry_skips_hello(client):
    c, _ = client
    sample = await c.telemetry()
    assert sample is not None
    assert sample.get("type") != "hello"
    assert sample["state"] == "Running"
    assert sample["armed"] == 1.0


async def test_stalker_start_stop(client):
    c, _ = client
    status = await c.stalker_start()
    assert status["running"] is True
    status = await c.stalker_stop()
    assert status["running"] is False


async def test_autonomy_body_shape(client):
    c, state = client
    result = await c.set_autonomy(True)
    assert result == {"autonomy": True}
    assert state.stalker["autonomy"] is True
