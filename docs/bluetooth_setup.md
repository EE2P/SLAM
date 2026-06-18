# Jetson 蓝牙启用与串口权限说明

设备：Jetson (L4T, Ubuntu 24.04 aarch64, kernel `6.8.x-tegra`)，蓝牙网卡为 **Intel AX200**（USB `8087:0029`）。

本文记录两件事：
1. 开机后蓝牙没有 `hci0` 的根因与修复（Jetson + Intel 网卡特有的坑）。
2. `usb_bridge_node` 报 `/dev/ttyACM0 Permission denied` 的串口权限修复。

---

## 一、蓝牙：开机没有 hci0

### 现象
- `hciconfig -a` 为空、`bluetoothctl list` 为空、`bluetoothctl` 提示 `No default controller available`。
- 硬件其实存在：`lsusb` 能看到 `Intel Corp. AX200 Bluetooth`。

### 根因（两个独立问题叠加）

**问题 1：NVIDIA 不加载内核自带的 `btusb` 驱动。**
`/etc/modprobe.d/nvidia-preferred-oot-modules.conf` 里有一条规则：只要系统里**存在** Realtek 的 `rtk_btusb` 驱动，就拒绝加载内核自带的 `btusb`，假设由 Realtek 驱动自己认领设备。但本机是 **Intel** 网卡，`rtk_btusb` 根本不接管它 → 没有任何驱动绑定 Intel 蓝牙 → 没有 `hci0`。

```
dmesg: modprobe: btusb: not loading in-tree driver, out-of-tree rtk_btusb is present
```

**问题 2：Intel 固件只有 zstd 压缩版，内核解不了。**
即使手动加载了 `btusb`，控制器停在 bootloader，索要 `intel/ibt-20-1-3.sfi`，但 `/lib/firmware/intel/` 里只有压缩的 `ibt-20-1-3.sfi.zst`，而这块 Tegra 内核**没开 zstd 固件透明解压**，于是报文件不存在：

```
dmesg: Bluetooth: hci0: Failed to load Intel firmware file intel/ibt-20-1-3.sfi (-2)
hciconfig: BD Address 00:00:00:00:00:00, Features 全 0x00, DOWN   ← 固件没灌进去
```

### 修复

**第 1 步：解压出未压缩固件（写进硬盘，永久）**
```bash
cd /lib/firmware/intel
sudo zstd -d -k -f ibt-20-1-3.sfi.zst -o ibt-20-1-3.sfi
sudo zstd -d -k -f ibt-20-1-3.ddc.zst -o ibt-20-1-3.ddc   # ddc 可选，跟随软链接解压
ls -l ibt-20-1-3.sfi                                      # 应约 800 KB
```

**第 2 步：强制加载内核自带 btusb，重灌固件**
```bash
sudo systemctl stop bluetooth
sudo modprobe -r btusb && sudo modprobe --ignore-install btusb
sleep 2
sudo systemctl start bluetooth
hciconfig -a       # BD Address 变真实 MAC、UP RUNNING 即成功
```
成功时 dmesg 应出现 `Found device firmware: intel/ibt-20-1-3.sfi` → `Firmware loaded` → `Device booted`。

> `--ignore-install` 是为了绕过 NVIDIA 那条拒载规则。普通 `modprobe btusb` 会被拦下。

### 让它开机自动生效（关键，否则每次重启都要手动）

固件是写在硬盘的、重启还在；但 `modprobe` 只在内存，重启就没了，而 NVIDIA 的拒载规则每次开机照旧。所以需要一个开机服务，每次开机替你加载 btusb（顺便防固件被升级删掉时自动重解压）。

```bash
sudo tee /etc/systemd/system/intel-btusb.service >/dev/null <<'EOF'
[Unit]
Description=Decompress Intel AX200 BT firmware + force-load in-tree btusb (NVIDIA OOT workaround)
Before=bluetooth.service
Wants=bluetooth.service

[Service]
Type=oneshot
RemainAfterExit=yes
# 内核不支持 zstd 固件：只剩 .zst 时先解压出未压缩的 .sfi
ExecStartPre=/bin/sh -c 'test -f /lib/firmware/intel/ibt-20-1-3.sfi || /usr/bin/zstd -d -k -f /lib/firmware/intel/ibt-20-1-3.sfi.zst -o /lib/firmware/intel/ibt-20-1-3.sfi'
ExecStart=/sbin/modprobe --ignore-install btusb

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable intel-btusb.service        # 必须看到 "Created symlink ..." 才算生效
systemctl is-enabled intel-btusb.service         # 应显示 enabled
```

> ⚠️ 易错点：只 `daemon-reload` 不 `enable` 等于没启用。`is-enabled` 必须显示 `enabled`、
> `/etc/systemd/system/multi-user.target.wants/intel-btusb.service` 软链接必须存在，开机才会自动跑。

**最终验证**：`sudo reboot`，重连后**不手动敲任何命令**，`hciconfig -a` 直接是 `UP RUNNING` 即彻底完成。

### 升级注意
`apt upgrade` 升级 `linux-firmware` 可能把解压出的 `.sfi` 覆盖掉、只留 `.zst`。上面的 `ExecStartPre` 已经会在开机时自动重新解压，无需手动干预。

---

## 二、串口权限：/dev/ttyACM0 Permission denied

### 现象
`usb_bridge_node` 反复刷：
```
failed to open /dev/ttyACM0: [Errno 13] Permission denied: '/dev/ttyACM0'
```

### 根因
`/dev/ttyACM0` 属主是 `root:dialout`（`crw-rw---- root dialout`），而 `test` 用户不在 `dialout` 组里。

### 修复
```bash
sudo usermod -aG dialout test     # -a 千万不能漏，否则会清空其它组
```
**组变更对已登录会话不生效，必须重新登录**（退出 SSH 重连，或 `sudo reboot`）：
```bash
exit          # 重新 ssh
id | grep dialout    # 看到 dialout(20) 即生效
```
之后重新跑 launch，`/dev/ttyACM0` 不再报权限错误。永久生效，以后开机不用再管。

---

## 快速排查小抄

| 症状 | 先查 | 修复 |
|------|------|------|
| `No default controller available` | `hciconfig -a` 有没有 hci0 | `sudo modprobe --ignore-install btusb` |
| hci0 存在但 DOWN、BD Address 全 0 | `dmesg \| grep firmware` | 解压 `ibt-20-1-3.sfi`，见上 |
| 重启后蓝牙又没了 | `systemctl is-enabled intel-btusb` | 没 enabled 就 `sudo systemctl enable intel-btusb.service` |
| `/dev/ttyACM0` Permission denied | `id \| grep dialout` | `sudo usermod -aG dialout test` 后重新登录 |

## 常用蓝牙指令

```bash
hciconfig -a                       # 适配器状态
sudo systemctl restart bluetooth   # 重启蓝牙服务

bluetoothctl                       # 进交互模式，然后：
#   power on / scan on / scan off
#   pair  <MAC>  / trust <MAC> / connect <MAC>
#   devices / paired-devices / info <MAC> / remove <MAC>
```
配对手柄/音箱标准流程：`power on` → `scan on`（记下 MAC）→ `pair` → `trust` → `connect`。
`trust` 必做，否则每次开机要重连。连不上时先 `remove <MAC>` 删旧记录再重配。
