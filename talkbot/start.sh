#!/usr/bin/env bash
# start.sh — 一键启动 Helen：激活 talkbot 虚拟环境，后台起脸 UI，前台跑 voice_pi。
# Ctrl-C（或 voice_pi 退出）时，脸 UI 会被一并关掉。
#
# 用法：   bash start.sh
# 可覆盖（运行前 export，或写进 ~/.talkbot.env）：
#   TALKBOT_VENV   talkbot 虚拟环境路径（默认自动探测常见位置）
#   TALKBOT_DIR    talkbot 目录（默认：本脚本所在目录，即 SLAM/talkbot）
#   FACE_DIR       脸 UI 的 pixel-face-pi-app 路径（默认 SLAM/ui/UI-test/pixel-face-pi-app）
#   TALKBOT_ROBOT  机器人 MCP 开关（默认 0=关；搬到 Jetson 后 MCP 暂未接，见下方说明）
#   TALKBOT_ENV_FILE  存放 GEMINI_API_KEY / 设备变量的文件（默认 ~/.talkbot.env）
#   FACE_LOG       脸 UI 日志文件（默认 /tmp/pixel-face.log）
set -euo pipefail

# talkbot 与脸 UI 现在同住 SLAM 库：talkbot/ 和 ui/ 是兄弟目录，所以默认路径都从本
# 脚本位置推导，不再依赖老的独立仓库位置（~/talkbot、~/UI）。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TALKBOT_DIR="${TALKBOT_DIR:-$SCRIPT_DIR}"
FACE_DIR="${FACE_DIR:-$SCRIPT_DIR/../ui/UI-test/pixel-face-pi-app}"
ENV_FILE="${TALKBOT_ENV_FILE:-$HOME/.talkbot.env}"
FACE_LOG="${FACE_LOG:-/tmp/pixel-face.log}"

# 机器人 MCP 随仓库搬到了 Jetson，但控制接口还没接（Jetson 这套是 ROS2 /cmd_vel +
# follow_bridge，不是原来 Pi 上的 Intellitrack 串口）。默认关掉机器人工具，只跑
# 「语音 + 像素脸」；以后接好了或想试，export TALKBOT_ROBOT=1 即可。
export TALKBOT_ROBOT="${TALKBOT_ROBOT:-0}"

# 1) 载入密钥 / 音频设备变量（可选）。文件里每行形如：GEMINI_API_KEY=xxxx
if [[ -f "$ENV_FILE" ]]; then
  set -a; . "$ENV_FILE"; set +a
fi

export TALKBOT_OUTPUT_BACKEND="${TALKBOT_OUTPUT_BACKEND:-aplay}"
export TALKBOT_ALSA_OUTPUT_DEVICE="${TALKBOT_ALSA_OUTPUT_DEVICE:-pipewire}"
export TALKBOT_INPUT_BACKEND="${TALKBOT_INPUT_BACKEND:-arecord}"
export TALKBOT_ALSA_INPUT_DEVICE="${TALKBOT_ALSA_INPUT_DEVICE:-plughw:0,0}"
TALKBOT_PIPEWIRE_SINK_NAME="${TALKBOT_PIPEWIRE_SINK_NAME:-USB2.0 Device Analog Stereo}"
TALKBOT_PIPEWIRE_SOURCE_NAME="${TALKBOT_PIPEWIRE_SOURCE_NAME:-SF-558 Mono}"
TALKBOT_PIPEWIRE_SINK_VOLUME="${TALKBOT_PIPEWIRE_SINK_VOLUME:-1.0}"
TALKBOT_PIPEWIRE_SOURCE_VOLUME="${TALKBOT_PIPEWIRE_SOURCE_VOLUME:-1.0}"

find_wpctl_node_id() {
  local section="$1"
  local name="$2"

  wpctl status | awk -v section="$section" -v name="$name" '
    index($0, section) { in_section = 1; next }
    in_section && index($0, "endpoints:") { exit }
    in_section && index($0, name) {
      if (match($0, /[0-9]+\./)) {
        print substr($0, RSTART, RLENGTH - 1)
        exit
      }
    }
  '
}

configure_pipewire_audio() {
  command -v wpctl >/dev/null 2>&1 || return 0

  local sink_id source_id
  sink_id="$(find_wpctl_node_id "Sinks:" "$TALKBOT_PIPEWIRE_SINK_NAME" || true)"
  source_id="$(find_wpctl_node_id "Sources:" "$TALKBOT_PIPEWIRE_SOURCE_NAME" || true)"

  if [[ -n "$sink_id" ]]; then
    wpctl set-default "$sink_id" || true
    wpctl set-mute "$sink_id" 0 || true
    wpctl set-volume "$sink_id" "$TALKBOT_PIPEWIRE_SINK_VOLUME" || true
    TALKBOT_PIPEWIRE_SINK_ID="$sink_id"
    echo "audio route: sink '$TALKBOT_PIPEWIRE_SINK_NAME' -> wpctl node $sink_id"
  else
    echo "audio route: sink '$TALKBOT_PIPEWIRE_SINK_NAME' not found; leaving default output" >&2
  fi

  if [[ -n "$source_id" ]]; then
    wpctl set-default "$source_id" || true
    wpctl set-mute "$source_id" 0 || true
    wpctl set-volume "$source_id" "$TALKBOT_PIPEWIRE_SOURCE_VOLUME" || true
    TALKBOT_PIPEWIRE_SOURCE_ID="$source_id"
    echo "audio route: source '$TALKBOT_PIPEWIRE_SOURCE_NAME' -> wpctl node $source_id"
  else
    echo "audio route: source '$TALKBOT_PIPEWIRE_SOURCE_NAME' not found; leaving default input" >&2
  fi
}

configure_pipewire_audio

# 2) 定位名为 talkbot 的虚拟环境
find_venv() {
  [[ -n "${TALKBOT_VENV:-}" ]] && { printf '%s\n' "$TALKBOT_VENV"; return 0; }
  local c
  for c in "$TALKBOT_DIR/.venv" "$TALKBOT_DIR/venv" \
           "$HOME/.virtualenvs/talkbot" "$HOME/venvs/talkbot" "$HOME/talkbot-venv" \
           "$HOME/miniconda3/envs/talkbot" "$HOME/miniforge3/envs/talkbot"; do
    if [[ -x "$c/bin/python" || -x "$c/bin/python3" ]]; then printf '%s\n' "$c"; return 0; fi
  done
  return 1
}
VENV="$(find_venv || true)"
if [[ -z "$VENV" ]]; then
  echo "✗ 找不到名为 talkbot 的虚拟环境。" >&2
  echo "  解决：export TALKBOT_VENV=/路径/到/venv 后重试，或把 venv 建在 $TALKBOT_DIR/.venv" >&2
  exit 1
fi

# 激活（普通 venv 有 activate 就 source；activate 脚本对 set -u 不友好，临时关掉）
if [[ -f "$VENV/bin/activate" ]]; then
  set +u; . "$VENV/bin/activate"; set -u
fi
PYTHON="$VENV/bin/python"; [[ -x "$PYTHON" ]] || PYTHON="$VENV/bin/python3"

# 3) 基本检查，提前把问题讲清楚而不是跑一半才崩
[[ -f "$TALKBOT_DIR/voice_pi.py" ]] || { echo "✗ 没找到 $TALKBOT_DIR/voice_pi.py（TALKBOT_DIR 设对了吗？）" >&2; exit 1; }
[[ -f "$FACE_DIR/run.sh" ]]        || { echo "✗ 没找到脸 UI：$FACE_DIR/run.sh（FACE_DIR 设对了吗？）" >&2; exit 1; }
if [[ -z "${GEMINI_API_KEY:-}" ]]; then
  echo "✗ GEMINI_API_KEY 未设置。写进 $ENV_FILE（一行：GEMINI_API_KEY=你的key）或先 export 再跑。" >&2
  exit 1
fi
if ! "$PYTHON" -c "import websockets" 2>/dev/null; then
  echo "✗ 这个 venv 里没装 websockets（联动桥需要）。在该 venv 里跑：pip install -e .  或  pip install websockets" >&2
  exit 1
fi

# 4) 退出时收拾脸 UI（脑在前台，退出后必经此处）
FACE_PID=""
cleanup() {
  trap - INT TERM EXIT
  if [[ -n "$FACE_PID" ]] && kill -0 "$FACE_PID" 2>/dev/null; then
    echo; echo "↩ 关闭脸 UI…" >&2
    kill "$FACE_PID" 2>/dev/null || true
    for _ in 1 2 3 4 5; do kill -0 "$FACE_PID" 2>/dev/null || break; sleep 0.3; done
    kill -9 "$FACE_PID" 2>/dev/null || true
  fi
}
trap cleanup INT TERM EXIT

echo "venv : $VENV"
echo "脸UI : $FACE_DIR/run.sh   (日志 → $FACE_LOG)"
echo "脑   : $TALKBOT_DIR/voice_pi.py"
echo "----------------------------------------"

# 5) 先起脸（后台；Chromium 噪音重定向到日志文件，保持终端干净）
( cd "$FACE_DIR" && exec ./run.sh ) >"$FACE_LOG" 2>&1 &
FACE_PID=$!

sleep 1
if ! kill -0 "$FACE_PID" 2>/dev/null; then
  echo "⚠ 脸 UI 启动后很快退出了，详情看 $FACE_LOG。脑会继续单独运行。" >&2
  FACE_PID=""
fi

# 6) 脑在前台跑（唤醒词/回复日志直接显示；Ctrl-C 会触发上面的 cleanup 收尾脸）
cd "$TALKBOT_DIR"
"$PYTHON" voice_pi.py
