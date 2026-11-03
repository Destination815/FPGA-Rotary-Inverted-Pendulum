#!/bin/zsh
set -e
j280_gui_root="$(cd "$(dirname "$0")" && pwd)"
cd "$j280_gui_root"
export PYTHONUTF8=1
if [[ -x .venv_gui/bin/python3 ]]; then
  j280_gui_python="$j280_gui_root/.venv_gui/bin/python3"
else
  j280_gui_python="$(command -v python3)"
fi
if ! "$j280_gui_python" prepare_gui.py; then
  echo 'Setup failed. Install Python 3.12 and retry. Press Enter to close.'
  read -r
  exit 1
fi
if ! .venv_gui/bin/python3 ../target_ui_fix/launch_target.py "$@"; then
  echo 'Launch failed. Keep this error message. Press Enter to close.'
  read -r
  exit 1
fi
