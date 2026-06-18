"""Async client for Intellitrack's control surfaces.

Intellitrack (FastAPI on the Pi, default :8000) owns the USB serial link to
the ESP32 balance robot. Two facts about its bridge shape everything here:

- The `/ws` websocket forwards raw serial text to the ESP32. When the last
  client disconnects (and no monitor client is attached) the bridge sends `D`
  (disarm). So a controlling client must HOLD its `/ws` connection while the
  robot is armed; that same connection makes the bridge feed the firmware
  comms watchdog with `C 0.0000 0.0000` whenever no command has been sent for
  0.25 s.
- Consequently any non-zero drive command must be re-sent faster than 0.25 s
  or the bridge zeroes it; we resend at DRIVE_RESEND_HZ (10 Hz).

Telemetry comes from `/monitor/ws` (hello frame, then parsed JSON samples).
Connecting to it turns on the firmware's compact logging, and a lingering
monitor client would defeat the disconnect-disarm safety net, so we only
connect transiently per query.
"""

import asyncio
import contextlib
import json

import httpx
import websockets

from . import config

ZERO_DRIVE = "C 0.0000 0.0000"


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, float(value)))


def format_drive(speed_m_s: float, yaw_rad_s: float) -> str:
    # Same formatting Intellitrack's cloud bridge uses (cloud_iot.py).
    return f"C {speed_m_s:.4f} {yaw_rad_s:.4f}"


class IntellitrackClient:
    def __init__(self, base_url: str = config.INTELLITRACK_URL):
        self.base_url = base_url.rstrip("/")
        # http -> ws, https -> wss
        self.ws_base = self.base_url.replace("http", "ws", 1)
        self._http = httpx.AsyncClient(base_url=self.base_url, timeout=5.0)
        self._ws = None
        self._ws_lock = asyncio.Lock()
        self._drain_task: asyncio.Task | None = None
        self._drive_task: asyncio.Task | None = None
        self.armed_intent = False

    # ---------------- control websocket ----------------

    async def ensure_control_ws(self):
        async with self._ws_lock:
            if self._ws is not None:
                return self._ws
            ws = await websockets.connect(self.ws_base + "/ws")
            self._ws = ws
            self._drain_task = asyncio.create_task(self._drain(ws))
            return ws

    async def _drain(self, ws) -> None:
        # The bridge broadcasts raw serial text to every /ws client; we don't
        # parse it (telemetry comes from /monitor/ws), but the socket must be
        # read or backpressure stalls the connection.
        try:
            async for _ in ws:
                pass
        except Exception:
            pass
        finally:
            if self._ws is ws:
                self._ws = None
                self.armed_intent = False

    async def send(self, text: str) -> None:
        ws = await self.ensure_control_ws()
        try:
            await ws.send(text)
        except Exception:
            # Connection died; one reconnect attempt so a single drop doesn't
            # fail the command. The bridge already disarmed on our disconnect.
            self._ws = None
            ws = await self.ensure_control_ws()
            await ws.send(text)

    # ---------------- arm / disarm / drive ----------------

    async def arm(self) -> dict:
        await self.ensure_control_ws()
        await self.send("E")
        self.armed_intent = True
        return {"armed": True}

    async def disarm(self) -> dict:
        await self._cancel_drive()
        await self.send("D")
        self.armed_intent = False
        return {"armed": False}

    async def drive(self, speed_m_s: float, yaw_rad_s: float = 0.0,
                    duration_s: float = 2.0) -> dict:
        speed = _clamp(speed_m_s, config.MAX_SPEED_M_S)
        yaw = _clamp(yaw_rad_s, config.MAX_YAW_RAD_S)
        duration = max(0.1, min(float(duration_s), config.MAX_DRIVE_S))

        if not self.armed_intent:
            return {"error": "robot is not armed — call arm_robot first"}
        with contextlib.suppress(Exception):
            status = await self.stalker_status()
            if status.get("autonomy"):
                return {"error": "autonomy (person tracking) is driving the robot — "
                                 "call set_autonomy(false) before manual driving"}

        await self._cancel_drive()
        line = format_drive(speed, yaw)
        self._drive_task = asyncio.create_task(self._drive_loop(line, duration))
        # Return immediately; the background task keeps the command fresh and
        # zeroes it when the duration elapses.
        return {"driving": True, "speed_m_s": speed, "yaw_rad_s": yaw,
                "duration_s": duration}

    async def _drive_loop(self, line: str, duration: float) -> None:
        loop = asyncio.get_running_loop()
        deadline = loop.time() + duration
        try:
            while loop.time() < deadline:
                await self.send(line)
                await asyncio.sleep(1.0 / config.DRIVE_RESEND_HZ)
        finally:
            with contextlib.suppress(Exception):
                await self.send(ZERO_DRIVE)

    async def _cancel_drive(self) -> None:
        task, self._drive_task = self._drive_task, None
        if task is not None and not task.done():
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await task

    async def emergency_stop(self) -> dict:
        await self._cancel_drive()
        with contextlib.suppress(Exception):
            await self.send(ZERO_DRIVE)
            await self.send("D")
        with contextlib.suppress(Exception):
            await self.set_autonomy(False)
        self.armed_intent = False
        return {"stopped": True, "armed": False, "autonomy": False}

    # ---------------- telemetry ----------------

    async def telemetry(self) -> dict | None:
        """Fetch one parsed telemetry sample, or None if none arrives in time."""
        try:
            async with asyncio.timeout(config.TELEMETRY_TIMEOUT_S):
                async with websockets.connect(self.ws_base + "/monitor/ws") as ws:
                    async for raw in ws:
                        msg = json.loads(raw)
                        if isinstance(msg, dict) and msg.get("type") == "hello":
                            continue
                        return msg
        except Exception:
            return None
        return None

    # ---------------- stalker (vision tracking) ----------------

    async def stalker_status(self) -> dict:
        resp = await self._http.get("/stalker/status")
        resp.raise_for_status()
        return resp.json()

    async def stalker_start(self) -> dict:
        resp = await self._http.post("/stalker/start")
        resp.raise_for_status()
        return resp.json()

    async def stalker_stop(self) -> dict:
        resp = await self._http.post("/stalker/stop")
        resp.raise_for_status()
        return resp.json()

    async def set_autonomy(self, on: bool) -> dict:
        resp = await self._http.post("/stalker/autonomy", json={"on": bool(on)})
        resp.raise_for_status()
        return resp.json()

    # ---------------- lifecycle ----------------

    async def close(self) -> None:
        await self._cancel_drive()
        ws, self._ws = self._ws, None
        if ws is not None:
            with contextlib.suppress(Exception):
                await ws.send(ZERO_DRIVE)
                await ws.send("D")
            with contextlib.suppress(Exception):
                await ws.close()
        if self._drain_task is not None:
            self._drain_task.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await self._drain_task
            self._drain_task = None
        self.armed_intent = False
        await self._http.aclose()
