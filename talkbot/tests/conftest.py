import asyncio
import sys
from pathlib import Path

import pytest
import uvicorn

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tests.fake_intellitrack import FakeState, create_app  # noqa: E402


@pytest.fixture
async def fake_intellitrack():
    """Run the fake Intellitrack on an ephemeral port; yield (base_url, state)."""
    state = FakeState()
    config = uvicorn.Config(create_app(state), host="127.0.0.1", port=0,
                            log_level="error", lifespan="off")
    server = uvicorn.Server(config)
    task = asyncio.create_task(server.serve())
    while not server.started:
        await asyncio.sleep(0.01)
    port = server.servers[0].sockets[0].getsockname()[1]
    yield f"http://127.0.0.1:{port}", state
    server.should_exit = True
    await task
