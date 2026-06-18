"""Exercise the FastMCP tool functions directly against the fake Intellitrack."""

import asyncio

import pytest

import robot_mcp.server as server_mod
from robot_mcp.intellitrack_client import IntellitrackClient


@pytest.fixture
async def tools(fake_intellitrack, monkeypatch):
    base_url, state = fake_intellitrack
    client = IntellitrackClient(base_url)
    monkeypatch.setattr(server_mod, "_client", client)
    yield server_mod, state
    await client.close()


async def test_robot_status(tools):
    server, _ = tools
    status = await server.robot_status()
    assert status["connected"] is True
    assert status["armed"] is True
    assert status["telemetry"]["state"] == "Running"
    assert status["telemetry"]["battery_v"] == pytest.approx(11.8)
    assert status["tracking"] == {"running": False, "autonomy": False}


async def test_arm_drive_estop_flow(tools):
    server, state = tools
    assert await server.arm_robot() == {"armed": True}
    result = await server.drive(0.1, 0.0, 0.3)
    assert result["driving"] is True
    await asyncio.sleep(0.45)
    assert "C 0.1000 0.0000" in state.serial_lines
    assert state.serial_lines[-1] == "C 0.0000 0.0000"
    stopped = await server.emergency_stop()
    assert stopped["stopped"] is True


async def test_drive_error_is_returned_not_raised(tools):
    server, _ = tools
    result = await server.drive(0.1)  # not armed
    assert "error" in result


async def test_tracking_tools(tools):
    server, _ = tools
    assert (await server.start_tracking())["running"] is True
    assert (await server.set_autonomy(True)) == {"autonomy": True}
    assert (await server.set_autonomy(False)) == {"autonomy": False}
    assert (await server.stop_tracking())["running"] is False
