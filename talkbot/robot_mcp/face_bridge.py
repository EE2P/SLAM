"""Tiny local WebSocket bridge between voice_pi and the pixel-face screen.

The face is a separate Chromium page (UI/pixel-face-pi-app), so it cannot read
voice_pi's state directly. This server gives them one channel:

    voice_pi  --(speaking on/off, expression)-->  ws://127.0.0.1:8765  -->  face
    voice_pi  <--------------- (touch) ----------------------------------  face

It is best-effort and self-contained: if `websockets` is missing or the port is
taken, voice_pi catches the error and simply runs without a face. Nothing here
blocks the voice loop — every send is fire-and-forget across connected clients.
"""

import asyncio
import contextlib
import json
import os

import websockets


class FaceServer:
    """Broadcasts state to face clients and collects taps from them."""

    def __init__(self, host: str | None = None, port: int | None = None):
        self.host = host or os.environ.get("TALKBOT_FACE_HOST", "127.0.0.1")
        self.port = int(port or os.environ.get("TALKBOT_FACE_PORT", "8765"))
        self._clients: set = set()
        self._server = None
        # Last-known state, replayed to any page that connects late so a
        # freshly-loaded face syncs mid-conversation.
        self._state = {"speaking": False, "expression": "neutral"}
        self.touch_events: "asyncio.Queue[dict]" = asyncio.Queue()

    async def start(self) -> None:
        self._server = await websockets.serve(self._handler, self.host, self.port)

    async def close(self) -> None:
        if self._server is not None:
            self._server.close()
            with contextlib.suppress(Exception):
                await self._server.wait_closed()
            self._server = None
        for ws in list(self._clients):
            with contextlib.suppress(Exception):
                await ws.close()
        self._clients.clear()

    # accept (ws) on new websockets and (ws, path) on the legacy server
    async def _handler(self, ws, *_args) -> None:
        self._clients.add(ws)
        print(f"[face] UI connected — {len(self._clients)} client(s)")
        with contextlib.suppress(Exception):
            await ws.send(json.dumps({"type": "state", **self._state}))
        try:
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                except (ValueError, TypeError):
                    continue
                if isinstance(msg, dict) and msg.get("type") == "touch":
                    await self.touch_events.put(msg)
        except Exception:
            pass
        finally:
            self._clients.discard(ws)
            print(f"[face] UI disconnected — {len(self._clients)} client(s)")

    async def _broadcast(self, payload: dict) -> None:
        data = json.dumps(payload)
        for ws in list(self._clients):
            try:
                await ws.send(data)
            except Exception:
                self._clients.discard(ws)

    async def set_speaking(self, speaking: bool) -> None:
        speaking = bool(speaking)
        if self._state["speaking"] == speaking:
            return
        self._state["speaking"] = speaking
        await self._broadcast({"type": "speaking", "value": speaking})

    async def set_expression(self, name: str) -> None:
        self._state["expression"] = name
        await self._broadcast({"type": "expression", "value": name})
