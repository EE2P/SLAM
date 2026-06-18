#!/usr/bin/env bash
set -euo pipefail

SOURCE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="${PIXEL_FACE_INSTALL_DIR:-$HOME/pixel-face-pi-app}"
AUTOSTART=0
CREATE_DESKTOP=1

usage() {
  cat <<'EOF'
Usage: ./install.sh [options]

Options:
  --autostart       Start Pixel Face automatically after Raspberry Pi Desktop logs in.
  --no-desktop      Do not create a desktop/application launcher.
  --install-dir DIR Install to DIR instead of $HOME/pixel-face-pi-app.
  -h, --help        Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --autostart)
      AUTOSTART=1
      shift
      ;;
    --no-desktop)
      CREATE_DESKTOP=0
      shift
      ;;
    --install-dir)
      if [[ $# -lt 2 ]]; then
        echo "--install-dir requires a directory path." >&2
        exit 1
      fi
      INSTALL_DIR="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ "${EUID:-$(id -u)}" -eq 0 ]]; then
  echo "Run this as the desktop user, not with sudo, so autostart lands in the right home directory." >&2
  exit 1
fi

if [[ "$INSTALL_DIR" != /* ]]; then
  INSTALL_DIR="$(pwd)/$INSTALL_DIR"
fi

mkdir -p "$INSTALL_DIR"
if [[ "$SOURCE_ROOT" != "$INSTALL_DIR" ]]; then
  cp -R "$SOURCE_ROOT/app" "$SOURCE_ROOT/bin" "$SOURCE_ROOT/README.md" "$SOURCE_ROOT/run.sh" "$INSTALL_DIR/"
fi

chmod +x "$INSTALL_DIR/run.sh" "$INSTALL_DIR/bin/pixel-face"

if [[ "$CREATE_DESKTOP" -eq 1 ]]; then
  DESKTOP_DIR="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
  DESKTOP_FILE="$DESKTOP_DIR/pixel-face.desktop"
  mkdir -p "$DESKTOP_DIR"
  {
    printf '%s\n' '[Desktop Entry]'
    printf '%s\n' 'Type=Application'
    printf '%s\n' 'Name=Pixel Face'
    printf '%s\n' 'Comment=Borderless talking pixel face'
    printf 'Exec=%s\n' "$INSTALL_DIR/run.sh"
    printf 'Path=%s\n' "$INSTALL_DIR"
    printf '%s\n' 'Terminal=false'
    printf '%s\n' 'Categories=Utility;'
  } > "$DESKTOP_FILE"
  chmod +x "$DESKTOP_FILE"
  echo "Desktop launcher: $DESKTOP_FILE"

  if [[ "$AUTOSTART" -eq 1 ]]; then
    AUTOSTART_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/autostart"
    mkdir -p "$AUTOSTART_DIR"
    cp "$DESKTOP_FILE" "$AUTOSTART_DIR/pixel-face.desktop"
    echo "Autostart enabled: $AUTOSTART_DIR/pixel-face.desktop"
  fi
fi

if ! command -v chromium-browser >/dev/null 2>&1 && ! command -v chromium >/dev/null 2>&1; then
  echo "Chromium is not installed yet. On Raspberry Pi OS, run:" >&2
  echo "  sudo apt update && sudo apt install -y chromium-browser unclutter" >&2
fi

echo "Installed Pixel Face app to: $INSTALL_DIR"
echo "Run it with: $INSTALL_DIR/run.sh"
