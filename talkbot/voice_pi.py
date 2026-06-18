#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Real-time voice chat for Raspberry Pi (Gemini Live + local wake word + VAD gating).

mic PCM -> Gemini Live -> speaker PCM; STT/LLM/TTS run in the cloud.
Hard wake word (openWakeWord, offline): sleeps until it hears the wake word,
then opens a VAD-gated conversation, then sleeps again after AWAKE_TIMEOUT_MS
of no speech. Local VAD only uploads while you speak. Barge-in + auto-reconnect.

  pip install google-genai sounddevice webrtcvad numpy openwakeword onnxruntime
  sudo apt install -y portaudio19-dev libportaudio2   # Raspberry Pi only
  export GEMINI_API_KEY="..."
  python3 voice_pi.py

Wake word is currently the built-in "hey_jarvis". To use "hi Helen": train a
model with openWakeWord's tooling, then set WAKE_MODEL to its .onnx path.
"""

import os
import sys
import queue
import asyncio
import threading
import warnings
import contextlib
import re
import shutil
import subprocess
import time
from types import SimpleNamespace

os.environ.setdefault("ORT_LOG_SEVERITY_LEVEL", "3")
warnings.filterwarnings(
    "ignore",
    message="pkg_resources is deprecated as an API.*",
    category=UserWarning,
)
warnings.filterwarnings(
    "ignore",
    message="Specified provider 'CUDAExecutionProvider'.*",
    category=UserWarning,
)

import numpy as np
import sounddevice as sd
import webrtcvad
import openwakeword
from openwakeword.model import Model

from google import genai
from google.genai import types

# Robot control tools (MCP). Disable with TALKBOT_ROBOT=0; if the bridge or
# its deps are missing we run voice-only, exactly as before.
ROBOT_ENABLED = os.environ.get("TALKBOT_ROBOT", "1") != "0"
try:
    from robot_mcp.gemini_bridge import RobotToolBridge
except Exception:
    RobotToolBridge = None

# Pixel-face screen (local WebSocket). Disable with TALKBOT_FACE=0; if the
# bridge or websockets is missing we run faceless, exactly as before.
FACE_ENABLED = os.environ.get("TALKBOT_FACE", "1") != "0"
try:
    from robot_mcp.face_bridge import FaceServer
except Exception:
    FaceServer = None

# Emotions the face screen can show; mirrored by the UI's expression masks.
FACE_EXPRESSIONS = [
    "neutral", "happy", "sad", "angry", "surprise", "look_left", "look_right",
]

MODEL = "gemini-2.5-flash-native-audio-preview-12-2025"   # native audio: natural voice, stable multi-turn
VOICE = "Aoede"            # female; other female voices: Kore, Leda, Zephyr
LANGUAGE_CODE = "en-US"

SEND_RATE = 16000          # upload rate (fixed)
RECV_RATE = 24000          # download rate (fixed)

FRAME_MS = 30                                   # webrtcvad accepts 10/20/30 ms only
FRAME_SAMPLES = SEND_RATE * FRAME_MS // 1000    # 480 samples = one mic frame

VAD_AGGRESSIVENESS = 2     # 0..3, higher = stricter (only used for the sleep timer now)

WAKE_MODEL = "hey_jarvis"          # built-in word; swap to "path/hi_helen.onnx" later
WAKE_THRESHOLD = float(os.environ.get("TALKBOT_WAKE_THRESHOLD", "0.5"))
WAKE_CHUNK = 1280                  # openWakeWord wants 80ms/16kHz frames
WAKE_CHUNK_BYTES = WAKE_CHUNK * 2  # int16 -> 2 bytes/sample
AWAKE_TIMEOUT_MS = int(os.environ.get("TALKBOT_AWAKE_TIMEOUT_MS", "60000"))
PLAYBACK_COOLDOWN_MS = int(os.environ.get("TALKBOT_PLAYBACK_COOLDOWN_MS", "250"))

# Noise gate on the uploaded mic audio: pass speech (plus a short hangover for
# word tails) at full level, but duck sustained background noise toward silence
# so the server VAD sees a clean end-of-speech and replies promptly. Attenuate
# rather than zero — pure silence collapses the server VAD noise floor and hangs
# the turn. Raise GATE_ATTENUATION (toward 0.2) if turns start hanging; lower it
# (toward 0.04) if room noise still bleeds through.
NOISE_GATE = os.environ.get("TALKBOT_NOISE_GATE", "0") == "1"
GATE_ATTENUATION = float(os.environ.get("TALKBOT_GATE_ATTENUATION", "0.18"))
GATE_HANGOVER_MS = int(os.environ.get("TALKBOT_GATE_HANGOVER_MS", "500"))
GATE_HANGOVER_FRAMES = GATE_HANGOVER_MS // FRAME_MS
MIC_QUEUE_FRAMES = int(os.environ.get("TALKBOT_MIC_QUEUE_FRAMES", "80"))

SYSTEM_INSTRUCTION = (
    "You are a voice assistant built into a robot, named Helen. "
    "The user talks to you by voice; understand their intent and reply. "
    "Keep replies short, conversational, and natural. "
    "Use Google Search for current facts, recent events, prices, weather, "
    "sports, or anything that may have changed recently. "
    "You can control your robot body with the robot tools: check status, "
    "arm or disarm, drive short bounded moves, start or stop person "
    "tracking, and emergency-stop. Confirm with the user before driving, "
    "keep moves short, and call emergency_stop immediately if they say stop. "
    "You have a face on a screen: call set_face_expression to show how you "
    "feel (happy, sad, angry, surprise) — smile when greeting or when the "
    "user asks you to make a face like 'smile' or '笑一个'. Don't call it on "
    "every reply, only when the emotion or the user's request really calls "
    "for it."
)

MIME_PCM = f"audio/pcm;rate={SEND_RATE}"


def _make_beep(freq=880, ms=120, vol=0.25) -> bytes:
    """Short tone (24kHz/int16) played as wake feedback."""
    n = int(RECV_RATE * ms / 1000)
    t = np.arange(n) / RECV_RATE
    return (np.sin(2 * np.pi * freq * t) * vol * 32767).astype(np.int16).tobytes()


WAKE_BEEP = _make_beep()
READY_BEEP = _make_beep(freq=1320, ms=90, vol=0.2)   # "your turn" cue after a reply


def resolve_wake_model_path() -> str:
    """Resolve a built-in wake word name or custom path to an ONNX model path."""
    if os.path.exists(WAKE_MODEL):
        return WAKE_MODEL

    wake_name = WAKE_MODEL.replace(" ", "_")
    get_paths = getattr(openwakeword, "get_pretrained_model_paths", None)
    if get_paths is not None:
        for args in (("onnx",), ()):
            try:
                pretrained_paths = get_paths(*args)
            except TypeError:
                continue
            for path in pretrained_paths:
                stem = os.path.splitext(os.path.basename(path))[0]
                if stem == wake_name or stem.startswith(f"{wake_name}_"):
                    return path

    models = getattr(openwakeword, "MODELS", {})
    model_info = models.get(WAKE_MODEL) or models.get(wake_name)
    if model_info:
        path = model_info.get("model_path", "")
        onnx_path = os.path.splitext(path)[0] + ".onnx"
        if os.path.exists(onnx_path):
            return onnx_path
        if os.path.exists(path):
            return path

    raise FileNotFoundError(
        f"Could not find wake word model '{WAKE_MODEL}'. "
        "Run openwakeword model download once, or set WAKE_MODEL to a real .onnx file path."
    )


def load_wakeword_model() -> Model:
    """Create openWakeWord Model across old/new keyword names."""
    model_path = resolve_wake_model_path()
    print(f"wake word: {WAKE_MODEL} -> {model_path}")
    attempts = (
        ("wakeword_models + onnx", {
            "wakeword_models": [model_path],
            "inference_framework": "onnx",
        }),
        ("wakeword_model_paths + onnx", {
            "wakeword_model_paths": [model_path],
            "inference_framework": "onnx",
        }),
        ("wakeword_models", {"wakeword_models": [model_path]}),
        ("wakeword_model_paths", {"wakeword_model_paths": [model_path]}),
        ("positional path", ([model_path],)),
    )

    errors = []
    for label, params in attempts:
        try:
            if isinstance(params, tuple):
                return Model(*params)
            return Model(**params)
        except TypeError as exc:
            if "unexpected keyword argument" not in str(exc):
                raise
            errors.append(f"{label}: {exc}")

    raise TypeError("could not create openWakeWord Model:\n" + "\n".join(errors))


class Playback:
    """Speaker output buffer."""

    def __init__(self):
        self._buf = np.empty(0, dtype=np.float32)
        self._lock = threading.Lock()
        self._samplerate = RECV_RATE
        self._channels = 1
        self._mode = "sounddevice"
        self._aplay_cmd = None
        self._aplay_proc = None
        self._play_until = 0.0

    def configure(self, samplerate: int, channels: int):
        self._mode = "sounddevice"
        self._samplerate = samplerate
        self._channels = channels

    def configure_aplay(self, cmd: list[str]):
        self._mode = "aplay"
        self._aplay_cmd = cmd
        self._start_aplay()

    def _start_aplay(self):
        if self._aplay_cmd is None:
            return
        if self._aplay_proc and self._aplay_proc.poll() is None:
            return
        self._aplay_proc = subprocess.Popen(
            self._aplay_cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def close(self):
        proc = self._aplay_proc
        self._aplay_proc = None
        if proc and proc.poll() is None:
            if proc.stdin:
                with contextlib.suppress(Exception):
                    proc.stdin.close()
            proc.terminate()
            with contextlib.suppress(Exception):
                proc.wait(timeout=1)
            if proc.poll() is None:
                with contextlib.suppress(Exception):
                    proc.kill()

    def __enter__(self):
        if self._mode == "aplay":
            self._start_aplay()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def callback(self, outdata, frames, time_info, status):
        if status:
            print(f"[speaker] {status}", file=sys.stderr)

        with self._lock:
            n = min(frames, len(self._buf))
            samples = self._buf[:n]
            self._buf = self._buf[n:]

        outdata[:] = 0
        if n:
            block = samples[:, None]
            if self._channels > 1:
                block = np.repeat(block, self._channels, axis=1)

            if np.issubdtype(outdata.dtype, np.integer):
                info = np.iinfo(outdata.dtype)
                block = np.clip(block, -1.0, 1.0) * info.max
                outdata[:n, :] = block.astype(outdata.dtype)
            else:
                outdata[:n, :] = block

    def add(self, data: bytes):
        if self._mode == "aplay":
            self._start_aplay()
            proc = self._aplay_proc
            if proc and proc.stdin:
                try:
                    proc.stdin.write(data)
                    proc.stdin.flush()
                except (BrokenPipeError, OSError):
                    self.close()
                    self._start_aplay()
                    proc = self._aplay_proc
                    if proc and proc.stdin:
                        proc.stdin.write(data)
                        proc.stdin.flush()

            seconds = len(data) / 2 / RECV_RATE
            with self._lock:
                now = time.monotonic()
                self._play_until = max(now, self._play_until) + seconds
            return

        samples = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
        if self._samplerate != RECV_RATE and len(samples) > 1:
            out_len = max(1, int(round(len(samples) * self._samplerate / RECV_RATE)))
            x_old = np.arange(len(samples), dtype=np.float32)
            x_new = np.linspace(0, len(samples) - 1, out_len, dtype=np.float32)
            samples = np.interp(x_new, x_old, samples).astype(np.float32)

        with self._lock:
            self._buf = np.concatenate((self._buf, samples))

    def clear(self):
        if self._mode == "aplay":
            self.close()
            self._start_aplay()
            with self._lock:
                self._play_until = 0.0
            return

        with self._lock:
            self._buf = np.empty(0, dtype=np.float32)

    def is_playing(self) -> bool:
        if self._mode == "aplay":
            with self._lock:
                return time.monotonic() < self._play_until

        with self._lock:
            return len(self._buf) > 0


playback = Playback()
mic_q: "queue.Queue[bytes]" = queue.Queue(maxsize=MIC_QUEUE_FRAMES)


def mic_callback(indata, frames, time_info, status):
    if status:
        print(f"[mic] {status}", file=sys.stderr)
    frame = bytes(indata)
    try:
        mic_q.put_nowait(frame)
    except queue.Full:
        # Keep the stream live: stale audio is worse than dropping one frame.
        with contextlib.suppress(queue.Empty):
            mic_q.get_nowait()
        with contextlib.suppress(queue.Full):
            mic_q.put_nowait(frame)


def _device_channels(device, direction: str) -> int:
    key = "max_input_channels" if direction == "input" else "max_output_channels"
    return int(device.get(key, 0))


def _find_audio_device(direction: str) -> int | None:
    """Pick a usable PortAudio device for input or output.

    Env override:
      TALKBOT_INPUT_DEVICE=4 or TALKBOT_INPUT_DEVICE=default
      TALKBOT_OUTPUT_DEVICE=5 or TALKBOT_OUTPUT_DEVICE=default
    """
    devices = sd.query_devices()
    env_name = f"TALKBOT_{direction.upper()}_DEVICE"
    requested = os.environ.get(env_name)

    def usable(index: int) -> bool:
        return _device_channels(devices[index], direction) > 0

    if requested:
        try:
            index = int(requested)
        except ValueError:
            matches = [
                i for i, d in enumerate(devices)
                if requested.lower() in d["name"].lower() and usable(i)
            ]
            if matches:
                return matches[0]
        else:
            if 0 <= index < len(devices) and usable(index):
                return index
        raise RuntimeError(f"{env_name}={requested!r} is not a usable {direction} device")

    # Prefer ALSA's plug/asym default, but only if it can actually do this direction.
    for i, d in enumerate(devices):
        if d["name"] == "default" and usable(i):
            return i

    for i, _ in enumerate(devices):
        if usable(i):
            return i

    return None


def _check_output_settings(device: int) -> dict | None:
    device_info = sd.query_devices()[device]
    max_channels = min(int(device_info["max_output_channels"]), 2)
    rates = []
    default_rate = int(device_info.get("default_samplerate") or 0)
    if default_rate:
        rates.append(default_rate)
    rates.extend([48000, 44100, 32000, 24000])

    seen_rates = []
    for rate in rates:
        if rate not in seen_rates:
            seen_rates.append(rate)

    for rate in seen_rates:
        for channels in range(max_channels, 0, -1):
            for dtype in ("int16", "int32", "float32"):
                try:
                    sd.check_output_settings(
                        device=device,
                        samplerate=rate,
                        channels=channels,
                        dtype=dtype,
                    )
                except Exception:
                    continue
                return {
                    "device": device,
                    "samplerate": rate,
                    "channels": channels,
                    "dtype": dtype,
                }

    return None


def _find_output_settings() -> dict | None:
    devices = sd.query_devices()
    requested = os.environ.get("TALKBOT_OUTPUT_DEVICE")

    if requested:
        try:
            candidates = [int(requested)]
        except ValueError:
            candidates = [
                i for i, d in enumerate(devices)
                if requested.lower() in d["name"].lower()
            ]
    else:
        candidates = [
            i for i, d in enumerate(devices)
            if d["name"] == "default" and _device_channels(d, "output") > 0
        ]
        candidates.extend(
            i for i, d in enumerate(devices)
            if i not in candidates and _device_channels(d, "output") > 0
        )

    for device in candidates:
        if not (0 <= device < len(devices)):
            continue
        if _device_channels(devices[device], "output") <= 0:
            continue
        settings = _check_output_settings(device)
        if settings is not None:
            return settings

    return None


def _aplay_command_for_device(device: int) -> list[str]:
    requested = os.environ.get("TALKBOT_ALSA_OUTPUT_DEVICE")
    if requested:
        alsa_device = requested
    else:
        name = sd.query_devices()[device]["name"]
        match = re.search(r"\(hw:(\d+),(\d+)\)", name)
        if match:
            alsa_device = f"plughw:{match.group(1)},{match.group(2)}"
        else:
            alsa_device = "default"

    return [
        shutil.which("aplay") or "aplay",
        "-q",
        "-D", alsa_device,
        "-t", "raw",
        "-f", "S16_LE",
        "-r", str(RECV_RATE),
        "-c", "1",
    ]


async def sender(session, oww: Model):
    """Sleep on wake word; stream audio while awake (server VAD handles turns);
    sleep again after AWAKE_TIMEOUT_MS of silence. Half-duplex: deaf while bot talks."""
    vad = webrtcvad.Vad(VAD_AGGRESSIVENESS)
    loop = asyncio.get_event_loop()

    while not mic_q.empty():        # drop stale audio from before a reconnect
        try:
            mic_q.get_nowait()
        except queue.Empty:
            break

    awake = False
    idle_frames = 0
    cooldown_frames = 0
    gate_open = 0
    wake_buf = bytearray()

    print(f"💤 sleeping — say the wake word ('{WAKE_MODEL}')")

    while True:
        frame = await loop.run_in_executor(None, mic_q.get)

        # ---------- asleep: run wake word locally, upload nothing ----------
        if not awake:
            wake_buf.extend(frame)
            while len(wake_buf) >= WAKE_CHUNK_BYTES:
                chunk = bytes(wake_buf[:WAKE_CHUNK_BYTES])
                del wake_buf[:WAKE_CHUNK_BYTES]
                scores = oww.predict(np.frombuffer(chunk, dtype=np.int16))
                if max(scores.values()) >= WAKE_THRESHOLD:   # one model loaded
                    awake = True
                    idle_frames = 0
                    cooldown_frames = 0
                    wake_buf.clear()
                    playback.add(WAKE_BEEP)
                    print("👂 awake — go ahead")
                    break
            continue

        # ---------- awake, half-duplex: while the bot talks (+cooldown), send NOTHING ----------
        # Open mic + speaker have no echo cancellation, so don't forward the bot's own voice.
        # Crucially, don't send zero-PCM either: a stream of pure digital silence drags the
        # server VAD's noise floor to 0, then real mic hiss reads as "user never stops talking"
        # and the turn hangs. Just stop sending; the server VAD keeps its noise floor.
        if playback.is_playing():
            idle_frames = 0
            cooldown_frames = PLAYBACK_COOLDOWN_MS // FRAME_MS
            continue
        if cooldown_frames > 0:
            cooldown_frames -= 1
            continue

        # ---------- awake + listening: stream noise-gated mic audio; server VAD detects turns ----------
        try:
            is_speech = vad.is_speech(frame, SEND_RATE)
        except Exception:
            is_speech = False

        # Noise gate: speech (+ hangover tail) goes out at full level; sustained
        # background noise is ducked so the server VAD ends the turn promptly.
        if is_speech:
            gate_open = GATE_HANGOVER_FRAMES
        out_frame = frame
        if NOISE_GATE and gate_open == 0:
            samples = np.frombuffer(frame, dtype=np.int16)
            out_frame = (samples * GATE_ATTENUATION).astype(np.int16).tobytes()
        elif gate_open > 0 and not is_speech:
            gate_open -= 1
        await session.send_realtime_input(audio=types.Blob(data=out_frame, mime_type=MIME_PCM))

        if is_speech:
            idle_frames = 0
        else:
            idle_frames += 1
            if idle_frames * FRAME_MS >= AWAKE_TIMEOUT_MS:
                awake = False
                wake_buf.clear()
                print(f"💤 timed out — sleeping (say '{WAKE_MODEL}' to wake)")


def face_tool_decl():
    """Gemini function declaration for showing an emotion on the face screen."""
    return types.FunctionDeclaration(
        name="set_face_expression",
        description=(
            "Show an emotion on your face screen so the user can see how you "
            "feel. Use this only when the user explicitly asks you to make a "
            "face (e.g. 'smile', 'laugh', '笑一个', '生气') or when a strong "
            "emotion clearly matters. Do not call it for ordinary replies."
        ),
        parameters=types.Schema(
            type=types.Type.OBJECT,
            properties={
                "expression": types.Schema(
                    type=types.Type.STRING,
                    enum=FACE_EXPRESSIONS,
                    description="The emotion to display.",
                ),
            },
            required=["expression"],
        ),
    )


async def dispatch_tool_call(session, tool_call, bridge, face):
    """Route a Live tool_call: face emotes handled locally, the rest to MCP."""
    face_responses = []
    robot_calls = []
    for fc in getattr(tool_call, "function_calls", None) or []:
        if face is not None and fc.name == "set_face_expression":
            expr = str(dict(fc.args or {}).get("expression", "neutral"))
            try:
                await face.set_expression(expr)
                payload = {"ok": True, "expression": expr}
            except Exception as exc:
                payload = {"error": f"{type(exc).__name__}: {exc}"}
            print(f"[face] set_face_expression({expr}) -> {payload}")
            face_responses.append(types.FunctionResponse(
                id=fc.id, name=fc.name, response=payload))
        else:
            robot_calls.append(fc)
    if face_responses:
        await session.send_tool_response(function_responses=face_responses)
    if robot_calls and bridge is not None:
        await bridge.handle(session, SimpleNamespace(function_calls=robot_calls))


async def mouth_pump(face):
    """Animate the face's mouth only while Helen is actually speaking.

    Polls the speaker buffer and pushes speaking on/off edges to the face, so
    the mouth rests when idle and moves through a reply."""
    speaking = False
    while True:
        await asyncio.sleep(0.05)
        now = playback.is_playing()
        if now != speaking:
            speaking = now
            with contextlib.suppress(Exception):
                await face.set_speaking(now)


async def touch_pump(session, face):
    """Voice reaction to screen taps: a normal tap gets a brief spoken reaction;
    rapid 'annoyed' poking gets an irritated 'stop poking me'. The face itself is
    set in the browser by the tap, so Helen is told not to change it here."""
    while True:
        evt = await face.touch_events.get()
        mood = evt.get("mood") if isinstance(evt, dict) else None
        if mood == "annoyed":
            text = ("[The user keeps poking your face screen over and over and "
                    "it's getting annoying. Snap at them to stop poking you — "
                    "one short, irritated sentence, in English. Your face is "
                    "already showing the anger, so do NOT call set_face_expression.]")
        else:
            text = ("[The user tapped your face screen to get your attention. "
                    "Say one short, clear, friendly sentence to them out loud "
                    "in English — actual words, not just a sound. They are "
                    "choosing your face by tapping, so do NOT call "
                    "set_face_expression.]")
        with contextlib.suppress(Exception):
            await session.send_client_content(
                turns=types.Content(role="user",
                                    parts=[types.Part(text=text)]),
                turn_complete=True,
            )


async def receiver(session, bridge=None, face=None):
    """Play model audio; keep listening after each completed model turn."""
    while True:
        saw_message = False
        async for msg in session.receive():
            saw_message = True
            # tool calls arrive top-level on the message, not in server_content
            tool_call = getattr(msg, "tool_call", None)
            if tool_call is not None and (bridge is not None or face is not None):
                asyncio.create_task(dispatch_tool_call(session, tool_call, bridge, face))
            sc = getattr(msg, "server_content", None)
            if sc is not None:
                model_turn = getattr(sc, "model_turn", None)
                for part in getattr(model_turn, "parts", None) or []:
                    inline_data = getattr(part, "inline_data", None)
                    data = getattr(inline_data, "data", None)
                    if data:
                        mime_type = getattr(inline_data, "mime_type", "") or ""
                        if mime_type.startswith("audio/"):
                            playback.add(data)

                if getattr(sc, "interrupted", False):
                    playback.clear()
                if getattr(sc, "turn_complete", False):
                    playback.add(READY_BEEP)      # plays right after the reply -> "your turn"
                    print("reply done (your turn)")

        if not saw_message:
            raise ConnectionError("live receive ended")


async def main():
    api_key = os.environ.get("GEMINI_API_KEY")
    if not api_key:
        sys.exit("set GEMINI_API_KEY first")

    client = genai.Client(api_key=api_key)

    tools = [types.Tool(google_search=types.GoogleSearch())]
    bridge = None
    if ROBOT_ENABLED and RobotToolBridge is not None:
        bridge = RobotToolBridge()
        try:
            await bridge.start()
            decls = bridge.gemini_tools_decls()
            if decls:
                tools.append(types.Tool(function_declarations=decls))
            print(f"robot tools ready: {', '.join(bridge.tool_names())}")
        except Exception as e:
            print(f"[robot] MCP bridge unavailable ({e}); voice-only mode",
                  file=sys.stderr)
            with contextlib.suppress(Exception):
                await bridge.close()
            bridge = None

    face = None
    if FACE_ENABLED and FaceServer is not None:
        face = FaceServer()
        try:
            await face.start()
            tools.append(types.Tool(function_declarations=[face_tool_decl()]))
            print(f"face screen ready: ws://{face.host}:{face.port} "
                  f"(expressions: {', '.join(FACE_EXPRESSIONS)})")
        except Exception as e:
            print(f"[face] screen bridge unavailable ({e}); running without face",
                  file=sys.stderr)
            with contextlib.suppress(Exception):
                await face.close()
            face = None

    config = types.LiveConnectConfig(
        response_modalities=["AUDIO"],
        system_instruction=SYSTEM_INSTRUCTION,
        tools=tools,
        speech_config=types.SpeechConfig(
            voice_config=types.VoiceConfig(
                prebuilt_voice_config=types.PrebuiltVoiceConfig(voice_name=VOICE)
            ),
            language_code=LANGUAGE_CODE,
        ),
        # Automatic (server-side) VAD handles turn-taking. Manual VAD (disabled=True)
        # triggers the known "no response after ActivityEnd / 1011 keepalive" bug.
    )

    input_dev = _find_audio_device("input")
    output_backend = os.environ.get("TALKBOT_OUTPUT_BACKEND", "auto").lower()
    output_settings = None
    aplay_cmd = None

    if output_backend != "sounddevice" and os.name == "posix" and shutil.which("aplay"):
        output_dev = _find_audio_device("output")
        if output_dev is not None:
            aplay_cmd = _aplay_command_for_device(output_dev)
    else:
        output_dev = None

    if aplay_cmd is None:
        output_settings = _find_output_settings()
        output_dev = output_settings["device"] if output_settings else None

    if input_dev is None or output_dev is None:
        print(sd.query_devices(), file=sys.stderr)
        sys.exit("audio: could not find both a usable input device and output device")

    devices = sd.query_devices()
    sd.default.device = (input_dev, output_dev)
    output_desc = (
        f"aplay {' '.join(aplay_cmd[1:])}"
        if aplay_cmd is not None else
        f"{output_settings['samplerate']} Hz, "
        f"{output_settings['channels']} ch, {output_settings['dtype']}"
    )
    print(
        "audio: mic -> "
        f"{devices[input_dev]['name']} (index {input_dev}, "
        f"{devices[input_dev]['max_input_channels']} ch); "
        "speaker -> "
        f"{devices[output_dev]['name']} (index {output_dev}, "
        f"{devices[output_dev]['max_output_channels']} ch, {output_desc})"
    )
    print(
        "listen: "
        f"wake_threshold={WAKE_THRESHOLD}, "
        f"noise_gate={'on' if NOISE_GATE else 'off'}, "
        f"cooldown_ms={PLAYBACK_COOLDOWN_MS}, "
        f"mic_queue_frames={MIC_QUEUE_FRAMES}"
    )

    # wake word engine: created once, reused across reconnects
    print("loading wake word model...")
    # first run downloads pretrained models; helper moved across openWakeWord versions
    _dl = getattr(openwakeword.utils, "download_models", None) or \
          getattr(openwakeword, "download_models", None)
    if _dl:
        _dl()
    oww = load_wakeword_model()

    if aplay_cmd is not None:
        playback.configure_aplay(aplay_cmd)
        print(f"audio: speaker stream -> {' '.join(aplay_cmd)}")
        out_stream = playback
    else:
        output_rate = output_settings["samplerate"]
        output_channels = output_settings["channels"]
        output_dtype = output_settings["dtype"]
        playback.configure(output_rate, output_channels)
        print(f"audio: speaker stream -> {output_rate} Hz, {output_channels} ch, {output_dtype}")
        out_stream = sd.OutputStream(
            samplerate=output_rate, channels=output_channels, dtype=output_dtype,
            device=output_dev, callback=playback.callback,
        )
    in_stream = sd.RawInputStream(
        samplerate=SEND_RATE, channels=1, dtype="int16",
        device=input_dev, blocksize=FRAME_SAMPLES, callback=mic_callback,
    )

    mouth_task = None
    try:
        with out_stream, in_stream:
            if face is not None:        # animate the mouth across all reconnects
                mouth_task = asyncio.create_task(mouth_pump(face))
            while True:             # reconnect on session timeout / network drop
                try:
                    print("connecting...")
                    async with client.aio.live.connect(model=MODEL, config=config) as session:
                        print("ready.")
                        tasks = [
                            asyncio.create_task(sender(session, oww)),
                            asyncio.create_task(receiver(session, bridge, face)),
                        ]
                        if face is not None:
                            tasks.append(asyncio.create_task(touch_pump(session, face)))
                        try:
                            await asyncio.gather(*tasks)
                        finally:        # tear down this session's tasks before reconnecting
                            for t in tasks:
                                t.cancel()
                            with contextlib.suppress(Exception):
                                await asyncio.gather(*tasks, return_exceptions=True)
                except asyncio.CancelledError:
                    raise
                except Exception as e:
                    import traceback
                    if bridge is not None:
                        # don't leave the robot armed/driving while Gemini is down
                        await bridge.emergency_safe()
                    if "invalid authentication credentials" in str(e).lower():
                        sys.exit(
                            "Gemini authentication failed: check GEMINI_API_KEY on this device. "
                            "If you pasted this key anywhere, revoke it and create a new one."
                        )
                    traceback.print_exc()
                    print(f"[disconnected] {type(e).__name__}: {e}; reconnecting in 1s", file=sys.stderr)
                    playback.clear()
                    await asyncio.sleep(1)
    finally:
        if mouth_task is not None:
            mouth_task.cancel()
            with contextlib.suppress(Exception):
                await mouth_task
        if face is not None:
            with contextlib.suppress(Exception):
                await face.close()
        if bridge is not None:
            with contextlib.suppress(Exception):
                await bridge.emergency_safe()
                await bridge.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nbye.")
