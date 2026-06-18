"""In-process fake of Intellitrack's control surfaces, for tests.

Reproduces the contracts the MCP server depends on (shapes copied from
Intellitrack's serial_bridge.py / parser.py / supervisor.py):
- WS /ws: records every inbound serial line; appends "D" when the last
  control client disconnects and no monitor client is attached
  (disconnect-disarm semantics).
- WS /monitor/ws: sends the hello frame, then parsed telemetry samples.
- HTTP /stalker/status|start|stop|autonomy with a mutable state dict.
"""

import asyncio
from dataclasses import dataclass, field

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse

HELLO = {
    "type": "hello",
    "port": "/dev/ttyFAKE",
    "baud": 921600,
    "csv": {"enabled": False, "paused": False, "path": None, "source": "wired compact"},
}

# Field names follow Intellitrack's parser.py (L_FIELDS + derived fields).
SAMPLE = {
    "t": 1.0,
    "ms": 1000.0,
    "tilt": 0.01,
    "pitch_deg": 0.57,
    "rate": 0.002,
    "rate_deg_s": 0.11,
    "speed_m_s": 0.02,
    "vcmd_m_s": 0.0,
    "batt": 11.8,
    "state_id": 1.0,
    "state": "Running",
    "armed": 1.0,
    "link": 1.0,
    "controller_mode": 0.0,
    "telemetry_source": "serial",
    "bridge_mode": "serial",
}


@dataclass
class FakeState:
    serial_lines: list = field(default_factory=list)
    control_clients: int = 0
    monitor_clients: int = 0
    stalker: dict = field(default_factory=lambda: {
        "running": False, "pid": None, "autonomy": False, "uptime_s": None,
        "started_epoch": None, "monitor_port": 8081, "cmd_count": 0,
        "last_cmd": {"v": 0.0, "w": 0.0}, "last_cmd_age_s": None,
        "exit_code": None, "launch": "fake",
    })


def create_app(state: FakeState) -> FastAPI:
    app = FastAPI()

    @app.websocket("/ws")
    async def control_ws(ws: WebSocket):
        await ws.accept()
        state.control_clients += 1
        state.serial_lines.append("g")  # bridge sends gain sync on connect
        try:
            while True:
                text = await ws.receive_text()
                if len(text) <= 192:
                    state.serial_lines.append(text)
        except WebSocketDisconnect:
            pass
        finally:
            state.control_clients -= 1
            if state.control_clients == 0 and state.monitor_clients == 0:
                state.serial_lines.append("D")

    @app.websocket("/monitor/ws")
    async def monitor_ws(ws: WebSocket):
        await ws.accept()
        state.monitor_clients += 1
        try:
            await ws.send_json(HELLO)
            while True:
                await ws.send_json(SAMPLE)
                await asyncio.sleep(0.05)
        except WebSocketDisconnect:
            pass
        finally:
            state.monitor_clients -= 1

    @app.get("/stalker/status")
    async def stalker_status():
        return JSONResponse(state.stalker)

    @app.post("/stalker/start")
    async def stalker_start():
        state.stalker["running"] = True
        state.stalker["pid"] = 4242
        return JSONResponse(state.stalker)

    @app.post("/stalker/stop")
    async def stalker_stop():
        state.stalker["running"] = False
        state.stalker["pid"] = None
        return JSONResponse(state.stalker)

    @app.post("/stalker/autonomy")
    async def stalker_autonomy(request: Request):
        body = {}
        try:
            body = await request.json()
        except Exception:
            pass
        if not isinstance(body, dict):
            body = {}
        on = bool(body.get("on", body.get("enabled", False)))
        state.stalker["autonomy"] = on
        return JSONResponse({"autonomy": on})

    return app
