# Pixel Face Raspberry Pi App

这是把 `像素脸·会说话.html` 包好的树莓派无边框版本。它不需要 Node、Electron 或本地服务器，直接用 Raspberry Pi OS 上的 Chromium 以 kiosk 模式打开本地 HTML。

## 文件

- `app/index.html`：应用页面，已从原始 HTML 复制成 ASCII 路径，方便脚本启动。
- `run.sh`：直接运行入口。
- `bin/pixel-face`：Chromium kiosk 启动器。
- `install.sh`：可选安装脚本，会生成桌面启动器，也能开启桌面登录后自启动。

## 在树莓派上运行

先确保系统有 Chromium：

```bash
sudo apt update
sudo apt install -y chromium-browser unclutter
```

把整个 `pixel-face-pi-app` 文件夹复制到树莓派，然后在树莓派桌面环境里运行：

```bash
cd pixel-face-pi-app
chmod +x run.sh install.sh bin/pixel-face
./run.sh
```

默认是全屏无边框 kiosk 模式。退出时可用 `Alt+F4`。

`run.sh` 会优先使用当前环境里的 `DISPLAY` 或 `WAYLAND_DISPLAY`。如果你从 SSH、TTY 或普通终端启动时没有这些变量，它会用 `loginctl` 自动寻找当前用户唯一的 active 图形会话，并补上 X11/Wayland、`XDG_RUNTIME_DIR`、DBus 和 Xauthority 环境后再启动全屏。

如果同一个用户同时有多个图形会话，脚本会停止并提示你手动指定，例如：

```bash
DISPLAY=:0 ./run.sh
```

或：

```bash
XDG_RUNTIME_DIR=/run/user/1000 WAYLAND_DISPLAY=wayland-0 ./run.sh
```

## 安装桌面入口

```bash
./install.sh
```

安装后可运行：

```bash
~/pixel-face-pi-app/run.sh
```

## 开机自启动

```bash
./install.sh --autostart
```

这会把启动器写到 `~/.config/autostart/pixel-face.desktop`。它会在 Raspberry Pi Desktop 登录后自动启动。

## 运行模式

默认模式是 kiosk：

```bash
./run.sh
```

如果只想用普通 app 窗口测试：

```bash
PIXEL_FACE_MODE=app PIXEL_FACE_WIDTH=800 PIXEL_FACE_HEIGHT=480 ./run.sh
```

如果 Chromium 不在默认命令名上，可以指定：

```bash
CHROMIUM=/path/to/chromium ./run.sh
```
