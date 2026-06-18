# 平衡车四仓库部署 / 联调指南

> Deployment guide for the balance-robot stack: ESP32 firmware
> (Balance_Embedded) + Pi orchestrator (Intellitrack) + vision follower
> (Stalker) + voice assistant with robot MCP tools (talkbot).

## 1. 系统总览

```
                       ┌──────────────  Raspberry Pi 5 ──────────────┐
  「hey jarvis」语音    │                                              │
  ┌──────────┐  MCP    │  ┌───────────┐  stdout   ┌─────────────┐     │  USB 串口
  │ talkbot  │ stdio   │  │  Stalker  │ @CMD v w  │ Intellitrack │    │  921600
  │ (Helen)  ├────────►│  │ 视觉跟随  ├──────────►│  FastAPI     ├────┼─────────► ESP32
  └────┬─────┘         │  └───────────┘  (子进程) │  :8000       │    │       Balance_Embedded
       │ robot-mcp     │        ▲                 └──────┬───────┘    │        固件
       └─ HTTP/WS ─────┼────────┴── /stalker/* ──────────┘            │
                       └──────────────────────────────────────────────┘
```

- **Balance_Embedded**：ESP32 固件，串口协议 + 平衡控制。
- **Intellitrack**：Pi 上的总控，独占 USB 串口，托管网页（调参/监控/跟踪）并拉起 Stalker。
- **Stalker**：Hailo 视觉人/线跟随，作为 Intellitrack 的子进程运行。
- **talkbot**：语音助手 Helen（Gemini Live）。本仓库的 `robot_mcp/` 是一个 MCP server，
  voice_pi.py 启动时自动以 stdio 拉起它，把机器人控制工具注入 Gemini。

### 端口 / 接口速查

| 接口 | 说明 |
|---|---|
| 串口 921600 baud | Pi ↔ ESP32（CP2102，`/dev/ttyUSB0`） |
| `:8000` | Intellitrack 网页 + `/ws`（串口命令）+ `/monitor/ws`（遥测）+ `/stalker/*` |
| `:8081` | Stalker follower 自带 monitor（由 Intellitrack 反向代理） |

### 串口协议速查

| 命令 | 含义 |
|---|---|
| `E` / `D` | 使能（armed）/ 失能 |
| `C <v> <w>` | 驾驶：速度 m/s（固件钳位 ±0.2）、转向 rad/s（±3.0） |
| `K` | IMU 校准 |
| `g` | 请求增益同步 |
| `LLOG 1/0` | 开/关紧凑遥测帧 `L` |

**看门狗**：固件要求驾驶命令持续刷新；Intellitrack 桥在有 `/ws` 客户端时每 0.25 s
自动补 `C 0 0`，且**最后一个客户端断开会自动发 `D` 失能**——这是兜底安全机制。

## 2. Balance_Embedded（ESP32 固件）

```bash
# Mac 或 Pi 上装 PlatformIO 后：
cd Balance_Embedded
pio run -e esp32dev_prod -t upload     # 生产固件：纯串口，给 Intellitrack 用
# 开发/带 WiFi-UDP 调试用 esp32dev 环境
```

- 烧录口与运行口都是同一个 CP2102 USB 串口。
- `esp32dev_prod` 不含 WiFi/UDP，开机即等串口命令。

## 3. Intellitrack（Pi 总控）

```bash
cd Intellitrack
./run.sh            # 自动创建 .venv 并 pip install -e .，然后启动 server
```

- 常用参数：`--serial-port /dev/ttyUSB0`（默认自动探测 CP2102）。
- 网页：`http://<pi>:8000/`（Tuning / Monitor / Tracking 三个 tab）。
- AWS IoT / Kinesis 云端部分可选，见 `docs/cloud-setup.md`，本地联调不需要。
- 注意：Intellitrack 必须先于 talkbot 启动（MCP 工具通过它发命令）。

## 4. Stalker（视觉跟随）

```bash
# 一次性：装进 hailo-apps 的 venv（Hailo 依赖都在那里）
"$HOME/hailo-apps/venv_hailo_apps/bin/pip" install -e ~/Stalker
```

- 不需要手动运行：网页 Tracking tab 或语音「start tracking」会经
  `POST /stalker/start` 由 Intellitrack 拉起 `stalker-follow`。
- HEF 模型路径等启动参数由 Intellitrack 的 `STALKER_LAUNCH` 环境变量覆盖。

## 5. talkbot（语音 + 机器人 MCP）

```bash
cd talkbot
uv sync                                  # 安装 pyproject.toml 里钉好的依赖
export GEMINI_API_KEY="..."              # 必需
export INTELLITRACK_URL="http://127.0.0.1:8000"   # 默认即此，跨机器时改
uv run python voice_pi.py
```

- voice_pi.py 启动时自动以 stdio 拉起 `python -m robot_mcp.server`；
  失败（依赖缺失、Intellitrack 没起）会自动退回纯语音模式。
- `TALKBOT_ROBOT=0` 可显式关闭机器人工具。
- 音频设备：`TALKBOT_INPUT_DEVICE` / `TALKBOT_OUTPUT_DEVICE` /
  `TALKBOT_OUTPUT_BACKEND`（见 voice_pi.py 顶部注释）。
- 同一个 MCP server 也可注册给 Claude Code 调试用（repo 根目录 `.mcp.json`）：

```json
{
  "mcpServers": {
    "balance-robot": {
      "command": "uv",
      "args": ["run", "robot-mcp"],
      "env": { "INTELLITRACK_URL": "http://<pi>:8000" }
    }
  }
}
```

### MCP 工具一览

| 工具 | 功能 | 安全限制 |
|---|---|---|
| `robot_status` | 遥测 + 跟踪状态汇总 | 只读 |
| `arm_robot` / `disarm_robot` | 使能 / 失能 | arm 前先建立持久 `/ws` 连接 |
| `drive(speed, yaw, duration)` | 限幅短程驾驶 | ±0.15 m/s、±1.5 rad/s、≤5 s，到时自动归零；未 arm 或 autonomy 开启时拒绝 |
| `emergency_stop` | 急停 | 归零 + 失能 + 关 autonomy，永远可用 |
| `start_tracking` / `stop_tracking` | 启停 Stalker | stop 同时停车 |
| `set_autonomy(on)` | 允许/禁止跟踪驱动底盘 | 与手动驾驶互斥 |

## 6. 启动顺序（联调）

1. ESP32 上电（已刷 `esp32dev_prod`），扶正待命。
2. Pi 上启动 Intellitrack：`./run.sh`（确认日志里串口已打开）。
3. 启动 talkbot：`uv run python voice_pi.py`（看到 `robot tools ready: ...`）。
4. 说「hey jarvis」唤醒，先问 "what's your status?" 验证遥测，再 arm / 短程驾驶。
5. 跟踪：语音「start tracking」→「enable autonomy」，或用网页 Tracking tab。

## 7. 安全机制（多层兜底）

| 层 | 机制 |
|---|---|
| MCP 工具 | ±0.15 m/s / ±1.5 rad/s / 单次 ≤5 s，自动归零；未 arm、autonomy 开启时拒绝驾驶 |
| talkbot | Gemini 断线或退出时自动调 `emergency_stop` |
| Intellitrack 桥 | `/ws` 全部断开自动发 `D`；0.25 s 无命令自动补 `C 0 0` |
| 固件 | 钳位 ±0.2 m/s / ±3.0 rad/s + 通信看门狗超时停车 |

## 8. 故障排查

- **`robot tools ready` 没出现**：依赖没装（`uv sync`）或被 `TALKBOT_ROBOT=0` 关了；
  看 stderr 里 `[robot] MCP bridge unavailable (...)` 的原因。
- **`robot_status` 返回 `connected: false`**：Intellitrack 没起、`INTELLITRACK_URL`
  不对，或固件没在发 `L` 遥测帧（`/monitor/ws` 连上会自动开 `LLOG 1`，仍无数据
  则检查串口与固件）。
- **drive 被拒绝**：先 `arm_robot`；或 autonomy 开着（先 `set_autonomy false`）。
- **串口找不到**：`ls /dev/ttyUSB*`，确认 CP2102 驱动与线缆；Intellitrack 默认
  自动探测，必要时 `--serial-port` 指定。
- **音频设备错**：`python3 -c "import sounddevice; print(sounddevice.query_devices())"`
  后用 `TALKBOT_*_DEVICE` 指定。

## 9. 测试（不需要硬件 / API key）

```bash
cd talkbot
uv sync --extra dev
uv run pytest                       # 对内置的假 Intellitrack 跑单测
uv run python tests/smoke_stdio.py  # stdio 冒烟：真 MCP 子进程 + 假 Intellitrack
```
