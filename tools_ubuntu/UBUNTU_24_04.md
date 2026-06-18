# Momento Host Tools on Ubuntu 24.04

## Install

```bash
cd /path/to/tools_ubuntu
sudo apt update
sudo apt install -y python3-venv python3-pip python3-tk
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
sudo usermod -aG dialout,input "$USER"
```

Log out and log back in after adding yourself to `dialout` / `input`.

## Run

```bash
source .venv/bin/activate
python momento_gui.py
```

The STM32 USB CDC port normally appears as `/dev/ttyACM0` or `/dev/ttyACM1`.

## Display

`momento_gui.py` is tuned for a 1024 x 768 screen:

- Starts at `1024 x 768`.
- Auto-maximizes on small displays.
- Uses compact toolbar labels and a compact gamepad panel.

## Gamepad

Ubuntu uses pygame/SDL for the Xbox-compatible gamepad panel.

Default axis map:

- `LX=0`
- `LY=1`
- `LT=2`
- `RX=3`
- `RY=4`
- `RT=5`

If your controller maps differently, override before launching:

```bash
MOMENTO_GAMEPAD_AXIS_LX=0 MOMENTO_GAMEPAD_AXIS_LY=1 \
MOMENTO_GAMEPAD_AXIS_LT=2 MOMENTO_GAMEPAD_AXIS_RX=3 \
MOMENTO_GAMEPAD_AXIS_RY=4 MOMENTO_GAMEPAD_AXIS_RT=5 \
python momento_gui.py
```

