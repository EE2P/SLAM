# Host tools

## Install

```bash
pip install -r requirements.txt
```

The dashboard needs PySide6 and pyserial. The Xbox controller panel works
only on Windows (uses the XInput DLL); on other platforms the panel still
appears but reports "XInput is only available on Windows".

---

## Momento control dashboard

```bash
python momento_gui.py
```

Real-time GUI for the Momento wheel-legged robot: 4 hip-joint position
dials, 2 wheel velocity dials (RPM, +/- 300 max), the Enable / Arm /
Disarm / Set-Zero state machine, and the embedded Xbox gamepad panel.

### Connection workflow

1. Plug the STM32 into the host over USB. A COM port shows up.
2. Pick the port in the top toolbar -> **Connect**. Status goes green.
3. Press **Enable**. State goes `BOOT` -> `IDLE` -> `ENABLED`. Motor
   indicator lights on each motor should turn solid green.
4. (Optional) Set per-hip zero -- see "Set-Zero" below.
5. Press **Arm**. State goes `ENABLED` -> `ARMING` -> `ARMED`. The robot
   now follows the host setpoints (sticks, sliders, or keyboard).
6. Drive the robot with the gamepad.
7. Press **Disarm** to stop control output. Press **Disable** to turn
   the motors off.

### Set-Zero (DM hips only, ENABLED state)

The "zero" button under each hip dial sets the current hip angle as the
new zero offset **and writes it to the motor's flash** so it survives
power cycles. Sequence run internally (~400 ms): disable -> set zero in
RAM -> save to flash -> re-enable. The button is greyed out unless the
robot is in `ENABLED` state (not armed). The wheel "stop" button is
different -- it only resets the target RPM to 0, no flash write.

DM flash is rated ~10,000 erase cycles, so do not spam this.

---

## Xbox controller mapping

The gamepad panel polls XInput on the dashboard's 50 ms UI tick. Sticks
only have any effect while the robot is **ARMED** and the controller is
connected.

> When the gamepad writes a setpoint, it **overrides** any slider /
> keyboard input on that tick. To use sliders / keyboard for fine tuning,
> either disarm, disconnect the gamepad, or hold sticks centred.

### Right stick -- driving the wheels

| Stick                     | Effect                                              |
|---------------------------|-----------------------------------------------------|
| RY up (full)              | Both wheels forward at +300 RPM                     |
| RY down                   | Both wheels backward                                |
| RX right (full)           | Yaw right (R wheel slows / reverses, L wheel forward) |
| RX left                   | Yaw left                                            |
| RY + RX                   | Mixed: arc curves while moving                      |
| **Release (centred)**     | **Both wheels stop (RPM = 0)**                      |

Internally the GUI computes a world-frame velocity per wheel
(`fwd ¡À yaw`), then applies a per-wheel sign (`WHEEL_MOTOR_SIGN` in
`momento_gui.py`) because the two wheel motors are mirror-mounted on
opposite sides of the chassis. If a wheel rolls the wrong direction
in practice, flip that wheel's `WHEEL_MOTOR_SIGN` entry (default is
`{2: +1.0, 5: -1.0}`).

Saturation: per-wheel command is clipped to +/- 300 RPM. The dial ring
turns red when the actual measured wheel velocity exceeds 300 RPM.

### Left stick -- posing the legs

`LY` is a **velocity / integrated** input. `LX` is a **direct** input.

| Stick                     | Effect                                              |
|---------------------------|-----------------------------------------------------|
| LY up (held)              | Both legs extend at 0.6 rad/s (per hip), up to 1.9 rad |
| LY down (held)            | Both legs retract at 0.6 rad/s, down to 0 rad        |
| LY centred                | **Leg length HOLDS** -- no spring-return             |
| LX right                  | Right leg shortens, left leg extends (body tilts right) |
| LX left                   | Mirror of right                                     |
| LX centred                | Tilt returns to 0 (this channel IS spring-return)   |
| Press **Disarm**          | Resets the leg-extension integrator to 0            |

The legs hold their commanded position via a standard MIT-mode PD loop
(`kp`, `kd` set in firmware, live-tunable in Ozone). The robot will
actively push back if you try to move the leg by hand.

Critical angles (matches firmware `motors[i].pos_min/pos_max`):

| Joint   | Allowed range     |
|---------|-------------------|
| R_hip1  | -1.9 .. 0    rad  |
| R_hip2  |  0   .. +1.9 rad  |
| L_hip1  |  0   .. +1.9 rad  |
| L_hip2  | -1.9 .. 0    rad  |

The GUI and the firmware both clamp setpoints to these windows. If a
hip moves the wrong way when you push LY up, flip its sign in
`HIP_EXT_SIGN` in `momento_gui.py`.

### Buttons -- visual only

All face buttons (A / B / X / Y), bumpers (LB / RB), triggers (LT / RT),
DPad, L3 / R3, and Back / Start light up on the gamepad panel for
diagnostics, but **none of them trigger any robot action**. The Enable
/ Arm / Disarm / Disable / Set-Zero buttons are software-only and must
be clicked in the dashboard toolbar.

### Player slot

If multiple Xbox controllers are paired with the PC, switch between
them with the `Player:` dropdown in the gamepad panel.

---

## Keyboard fallback

When the gamepad is disconnected (or the robot is unarmed), the
selected joint can be nudged with the arrow keys:

| Key      | Selected hip        | Selected wheel        |
|----------|---------------------|-----------------------|
| Up       | +0.10 rad (coarse)  | +10 rpm (coarse)      |
| Down     | -0.10 rad           | -10 rpm               |
| Right    | +0.01 rad (fine)    | +1 rpm                |
| Left     | -0.01 rad           | -1 rpm                |

Click on a joint dial to select it. The gamepad LY integrator does
NOT pick up changes you made via keyboard; pushing the stick afterwards
overrides those values.

---

## Other host tools

- **4-channel USB remote**: `python usb_remote_ui.py [COMx]` -- GUI;
  use `--console` for terminal only.
- **USB stress test**: `python usb_stress_test.py [COMx] [num_packets]`
  -- echo mode for USB CDC throughput / loss; `--one-way` for one-way
  RX (firmware reports total bytes); `--crc` adds CRC32 verification.

See `../resource_N_docs/USB_remote_and_stress_test.md` for protocol and
STM32 usage.
