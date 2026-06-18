#!/usr/bin/env python3
"""Manual stdio smoke test: spawn the real robot-mcp server as a subprocess,
pointed at an in-process fake Intellitrack, and call a few tools.

Run from the repo root (needs the dev extras installed):

    uv run python tests/smoke_stdio.py
"""

import asyncio
import os
import sys
from pathlib import Path

import uvicorn
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tests.fake_intellitrack import FakeState, create_app  # noqa: E402


async def main() -> None:
    state = FakeState()
    config = uvicorn.Config(create_app(state), host="127.0.0.1", port=0,
                            log_level="error", lifespan="off")
    server = uvicorn.Server(config)
    server_task = asyncio.create_task(server.serve())
    while not server.started:
        await asyncio.sleep(0.01)
    port = server.servers[0].sockets[0].getsockname()[1]
    print(f"fake intellitrack on :{port}")

    params = StdioServerParameters(
        command=sys.executable,
        args=["-m", "robot_mcp.server"],
        env={**os.environ, "INTELLITRACK_URL": f"http://127.0.0.1:{port}"},
    )
    async with stdio_client(params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            tools = (await session.list_tools()).tools
            print("tools:", [t.name for t in tools])

            for name, args in [
                ("robot_status", {}),
                ("arm_robot", {}),
                ("drive", {"speed_m_s": 0.1, "duration_s": 0.5}),
                ("emergency_stop", {}),
            ]:
                result = await session.call_tool(name, args)
                text = "; ".join(c.text for c in result.content
                                 if getattr(c, "text", None))
                print(f"{name}: {text}")
                await asyncio.sleep(0.6 if name == "drive" else 0.1)

    print("serial lines seen by fake bridge:")
    for line in state.serial_lines:
        print(" ", line)

    server.should_exit = True
    await server_task


if __name__ == "__main__":
    asyncio.run(main())
