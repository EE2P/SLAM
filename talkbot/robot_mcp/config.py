"""Safety limits and environment configuration for the robot MCP server."""

import os

# Intellitrack server (owns the USB serial link to the ESP32).
INTELLITRACK_URL = os.environ.get("INTELLITRACK_URL", "http://127.0.0.1:8000")

# Drive limits — intentionally tighter than the firmware clamps
# (firmware: ±0.2 m/s, ±3.0 rad/s; see Balance_Embedded serial protocol).
MAX_SPEED_M_S = 0.15
MAX_YAW_RAD_S = 1.5
MAX_DRIVE_S = 5.0

# The bridge zeroes any command older than SERIAL_COMMAND_STALE_S (0.25 s),
# so the drive loop must resend faster than that.
DRIVE_RESEND_HZ = 10.0

TELEMETRY_TIMEOUT_S = 2.5
