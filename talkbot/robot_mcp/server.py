"""FastMCP stdio server exposing the balance robot to MCP clients.

Run with `robot-mcp` (console script) or `python -m robot_mcp.server`.
Talks to Intellitrack at INTELLITRACK_URL (default http://127.0.0.1:8000).

Tool docstrings double as MCP descriptions and are phrased for a voice
model: short, imperative, with the safety contract spelled out. All tools
return plain dicts; failures come back as {"error": "..."} so the assistant
can speak them instead of crashing the call.
"""

import contextlib
from collections.abc import AsyncGenerator
from contextlib import asynccontextmanager

from mcp.server.fastmcp import FastMCP

from .intellitrack_client import IntellitrackClient

_client: IntellitrackClient | None = None


def get_client() -> IntellitrackClient:
    global _client
    if _client is None:
        _client = IntellitrackClient()
    return _client


@asynccontextmanager
async def _lifespan(_server) -> AsyncGenerator[None, None]:
    try:
        yield
    finally:
        # Best-effort stop + disarm on shutdown; the bridge's
        # disconnect-disarm is the backstop if this never runs.
        if _client is not None:
            with contextlib.suppress(Exception):
                await _client.close()


mcp = FastMCP("balance-robot", lifespan=_lifespan)


def _err(exc: Exception) -> dict:
    return {"error": f"{type(exc).__name__}: {exc}"}


@mcp.tool()
async def robot_status() -> dict:
    """Get the robot's current state: armed, balance state, battery voltage,
    pitch, speed, and whether person tracking is running."""
    client = get_client()
    sample = await client.telemetry()
    tracking = None
    with contextlib.suppress(Exception):
        st = await client.stalker_status()
        tracking = {"running": st.get("running"), "autonomy": st.get("autonomy")}
    if sample is None:
        return {"connected": False, "telemetry": None, "tracking": tracking,
                "note": "no telemetry received — is Intellitrack running and the robot powered?"}
    return {
        "connected": True,
        "armed": bool(sample.get("armed")),
        "telemetry": {
            "state": sample.get("state"),
            "battery_v": sample.get("batt"),
            "pitch_deg": sample.get("pitch_deg"),
            "speed_m_s": sample.get("speed_m_s"),
            "link": sample.get("link"),
        },
        "tracking": tracking,
    }


@mcp.tool()
async def arm_robot() -> dict:
    """Arm the robot so it starts balancing. The robot must be upright.
    Required before drive()."""
    try:
        return await get_client().arm()
    except Exception as exc:
        return _err(exc)


@mcp.tool()
async def disarm_robot() -> dict:
    """Disarm the robot: motors stop and it will no longer balance.
    Cancels any drive in progress."""
    try:
        return await get_client().disarm()
    except Exception as exc:
        return _err(exc)


@mcp.tool()
async def drive(speed_m_s: float, yaw_rad_s: float = 0.0,
                duration_s: float = 2.0) -> dict:
    """Drive the robot for a short, bounded move, then it stops automatically.
    speed_m_s: forward (+) / backward (-), clamped to ±0.15.
    yaw_rad_s: turn left (+) / right (-), clamped to ±1.5.
    duration_s: how long to drive, max 5 seconds.
    Refused while disarmed or while autonomy (tracking) is driving.
    A new call replaces the previous move."""
    try:
        return await get_client().drive(speed_m_s, yaw_rad_s, duration_s)
    except Exception as exc:
        return _err(exc)


@mcp.tool()
async def emergency_stop() -> dict:
    """STOP NOW: zero the motors, disarm, and switch autonomy off.
    Always available — use whenever the user says stop."""
    try:
        return await get_client().emergency_stop()
    except Exception as exc:
        return _err(exc)


@mcp.tool()
async def start_tracking() -> dict:
    """Start the vision person-tracking process (Stalker). It only drives the
    robot after set_autonomy(true)."""
    try:
        return await get_client().stalker_start()
    except Exception as exc:
        return _err(exc)


@mcp.tool()
async def stop_tracking() -> dict:
    """Stop the vision person-tracking process and stop the chassis."""
    try:
        return await get_client().stalker_stop()
    except Exception as exc:
        return _err(exc)


@mcp.tool()
async def set_autonomy(on: bool) -> dict:
    """Allow (true) or block (false) the tracking process from driving the
    robot. Manual drive() is refused while autonomy is on."""
    try:
        return await get_client().set_autonomy(on)
    except Exception as exc:
        return _err(exc)


def main() -> None:
    mcp.run()  # stdio transport


if __name__ == "__main__":
    main()
