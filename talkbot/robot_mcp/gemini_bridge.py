"""Bridge between the robot MCP server and a Gemini Live session.

voice_pi.py creates one RobotToolBridge for the lifetime of the process. It
spawns `python -m robot_mcp.server` as an MCP stdio subprocess, converts the
MCP tool list into Gemini function declarations, and dispatches Live API
tool calls back to the MCP server.

We bridge manually (FunctionDeclaration + send_tool_response) instead of
passing the mcp ClientSession straight into config.tools: the SDK's native
MCP path is experimental for the Live API and we cannot verify it offline.
Native alternative, if you want to try it:

    config = types.LiveConnectConfig(..., tools=[bridge.mcp_session, ...])

Known-uncertain SDK details (cannot test without an API key), flagged here:
- tool calls arrive as a TOP-LEVEL `msg.tool_call` on LiveServerMessage
  (LiveServerToolCall with .function_calls), not under server_content;
- whether the native-audio preview model honors function declarations as
  reliably as the half-cascade models.
"""

import contextlib
import os
import sys
from contextlib import AsyncExitStack

from google.genai import types
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

_TYPE_MAP = {
    "string": types.Type.STRING,
    "number": types.Type.NUMBER,
    "integer": types.Type.INTEGER,
    "boolean": types.Type.BOOLEAN,
    "array": types.Type.ARRAY,
    "object": types.Type.OBJECT,
}


def _schema_from_json(js: dict | None) -> types.Schema | None:
    """Convert the flat JSON-Schema subset FastMCP emits into types.Schema.

    Hand-rolled instead of FunctionDeclaration.parameters_json_schema, whose
    Live API support is less proven. Returns None for empty object schemas
    (Gemini wants no `parameters` at all for zero-arg functions)."""
    if not js:
        return None
    t = js.get("type")
    if isinstance(t, list):  # e.g. ["number", "null"]
        t = next((x for x in t if x != "null"), "string")
    if t is None and "anyOf" in js:  # FastMCP optionals: anyOf [X, null]
        for option in js["anyOf"]:
            if option.get("type") != "null":
                return _schema_from_json(option)
        return None
    schema = types.Schema(type=_TYPE_MAP.get(t or "object", types.Type.STRING))
    if js.get("description"):
        schema.description = js["description"]
    if js.get("enum"):
        schema.enum = [str(v) for v in js["enum"]]
    if js.get("items"):
        schema.items = _schema_from_json(js["items"])
    props = js.get("properties")
    if props:
        schema.properties = {k: _schema_from_json(v) for k, v in props.items()}
        if js.get("required"):
            schema.required = list(js["required"])
    elif schema.type == types.Type.OBJECT:
        return None
    return schema


def _result_payload(result) -> dict:
    """Flatten an mcp CallToolResult into a JSON-able FunctionResponse body."""
    if getattr(result, "structuredContent", None):
        body = result.structuredContent
    else:
        texts = [c.text for c in (result.content or [])
                 if getattr(c, "text", None) is not None]
        body = "\n".join(texts) if texts else ""
    if getattr(result, "isError", False):
        return {"error": body}
    return body if isinstance(body, dict) else {"result": body}


class RobotToolBridge:
    def __init__(self):
        self._stack = AsyncExitStack()
        self.mcp_session: ClientSession | None = None
        self._tools: list = []

    async def start(self) -> None:
        params = StdioServerParameters(
            command=sys.executable,
            args=["-m", "robot_mcp.server"],
            env=dict(os.environ),
        )
        read, write = await self._stack.enter_async_context(stdio_client(params))
        self.mcp_session = await self._stack.enter_async_context(
            ClientSession(read, write))
        await self.mcp_session.initialize()
        self._tools = (await self.mcp_session.list_tools()).tools

    async def close(self) -> None:
        await self._stack.aclose()
        self.mcp_session = None

    def tool_names(self) -> list[str]:
        return [t.name for t in self._tools]

    def gemini_tools_decls(self) -> list[types.FunctionDeclaration]:
        return [
            types.FunctionDeclaration(
                name=t.name,
                description=t.description or "",
                parameters=_schema_from_json(t.inputSchema),
            )
            for t in self._tools
        ]

    async def handle(self, session, tool_call) -> None:
        """Run every function call in a Live tool_call message against the MCP
        server and send the responses back to Gemini."""
        responses = []
        for fc in getattr(tool_call, "function_calls", None) or []:
            try:
                result = await self.mcp_session.call_tool(fc.name, dict(fc.args or {}))
                payload = _result_payload(result)
            except Exception as exc:
                payload = {"error": f"{type(exc).__name__}: {exc}"}
            print(f"[robot] {fc.name}({dict(fc.args or {})}) -> {payload}")
            responses.append(types.FunctionResponse(
                id=fc.id, name=fc.name, response=payload))
        if responses:
            await session.send_tool_response(function_responses=responses)

    async def emergency_safe(self) -> None:
        """Best-effort emergency stop, for when the Gemini session drops while
        the robot might be armed or mid-drive."""
        if self.mcp_session is None:
            return
        with contextlib.suppress(Exception):
            await self.mcp_session.call_tool("emergency_stop", {})
