"""Download the locked GUI wheels before travelling; no hardware access."""
import argparse
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--python-version', default='3.12', choices=['3.10','3.11','3.12','3.13'])
    p.add_argument('--out', type=Path, help='Optional wheel cache directory')
    p.add_argument('--print-command', action='store_true')
    a = p.parse_args()
    version = a.python_version.replace('.','')
    dest = a.out.resolve() if a.out else ROOT/'offline'/'wheels'/f'windows-py{version}'
    cmd = [sys.executable, '-m', 'pip', 'download', '--disable-pip-version-check',
           '--only-binary=:all:', '--platform','win_amd64','--python-version',a.python_version,
           '--implementation','cp','--abi',f'cp{version}',
           '-r',str(ROOT/'host/requirements_gui.txt'),'-d',str(dest)]
    if a.print_command:
        import json
        print(json.dumps(cmd, ensure_ascii=False, indent=2)); return
    dest.mkdir(parents=True, exist_ok=True)
    subprocess.run(cmd, check=True)
    print('Ready:', dest)
    print('Copy the whole project, including offline/wheels, to the Windows computer.')
    print('Install matching 64-bit Python, then run start_R43_windows.cmd.')

if __name__ == '__main__':
    main()
