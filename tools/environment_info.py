"""Show practical installation information without connecting or sending commands."""
from pathlib import Path
import importlib.util
import json
import os
import platform
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]

def main():
    print('Project:', ROOT)
    print('Python:', sys.version.split()[0], platform.machine(), sys.executable)
    env_python = ROOT/'host/.venv_gui'/('Scripts/python.exe' if os.name=='nt' else 'bin/python3')
    selected = str(env_python) if env_python.exists() else sys.executable
    code = "import importlib.util,json; print(json.dumps({m:importlib.util.find_spec(m) is not None for m in ['PySide6','pyqtgraph','numpy','serial']}))"
    print('GUI interpreter:', selected)
    result = subprocess.run([selected,'-c',code],capture_output=True,text=True)
    print('GUI dependencies:', result.stdout.strip() or result.stderr.strip())
    print('Gowin CLI (environment/PATH):',os.environ.get('GOWIN_GW_SH') or shutil.which('gw_sh') or 'Use installed Gowin; compile entry also searches standard install locations')
    fs = ROOT/'bitstream/j280_draft.fs'
    print('Ready-to-download bitstream:',fs if fs.exists() else 'Not present')
    print('Verilator (optional):',shutil.which('verilator') or 'Not installed')
    print('Dependency caches:', ', '.join(p.name for p in (ROOT/'offline/wheels').glob('*')) or 'None; first setup uses internet')
    print('Logs:',ROOT/'logs')
    print('No hardware connection or motion command was made.')

if __name__ == '__main__':
    main()
