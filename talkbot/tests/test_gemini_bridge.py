"""Schema conversion tests — no network, no API key, no MCP subprocess."""

from google.genai import types

from robot_mcp.gemini_bridge import _result_payload, _schema_from_json

# Shape FastMCP emits for drive(speed_m_s: float, yaw_rad_s: float = 0.0, ...)
DRIVE_SCHEMA = {
    "type": "object",
    "properties": {
        "speed_m_s": {"title": "Speed M S", "type": "number"},
        "yaw_rad_s": {"title": "Yaw Rad S", "type": "number", "default": 0.0},
        "duration_s": {"title": "Duration S", "type": "number", "default": 2.0},
    },
    "required": ["speed_m_s"],
    "title": "driveArguments",
}


def test_drive_schema_converts():
    schema = _schema_from_json(DRIVE_SCHEMA)
    assert schema is not None
    assert schema.type == types.Type.OBJECT
    assert set(schema.properties) == {"speed_m_s", "yaw_rad_s", "duration_s"}
    assert schema.properties["speed_m_s"].type == types.Type.NUMBER
    assert schema.required == ["speed_m_s"]


def test_empty_object_schema_becomes_none():
    assert _schema_from_json({"type": "object", "properties": {}}) is None
    assert _schema_from_json({"type": "object", "title": "robot_statusArguments"}) is None
    assert _schema_from_json(None) is None


def test_anyof_optional_unwraps():
    js = {"anyOf": [{"type": "number"}, {"type": "null"}], "default": None}
    schema = _schema_from_json(js)
    assert schema is not None
    assert schema.type == types.Type.NUMBER


def test_bool_and_enum():
    js = {"type": "object",
          "properties": {"on": {"type": "boolean"},
                         "mode": {"type": "string", "enum": ["person", "line"]}},
          "required": ["on"]}
    schema = _schema_from_json(js)
    assert schema.properties["on"].type == types.Type.BOOLEAN
    assert schema.properties["mode"].enum == ["person", "line"]


class _FakeText:
    def __init__(self, text):
        self.text = text


class _FakeResult:
    def __init__(self, *, text=None, structured=None, is_error=False):
        self.content = [_FakeText(text)] if text is not None else []
        self.structuredContent = structured
        self.isError = is_error


def test_result_payload_prefers_structured():
    assert _result_payload(_FakeResult(structured={"armed": True})) == {"armed": True}


def test_result_payload_text_and_error():
    assert _result_payload(_FakeResult(text='{"ok": 1}')) == {"result": '{"ok": 1}'}
    assert _result_payload(_FakeResult(text="boom", is_error=True)) == {"error": "boom"}
