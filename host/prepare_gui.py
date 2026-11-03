#!/usr/bin/env python3
"""Prepare a private environment with locked GUI dependencies; preserves console venv."""
import hashlib
import os
from pathlib import Path
import subprocess
import sys
import venv

def main():
    if not (3,10)<=sys.version_info[:2]<(3,14):
        print('Use 64-bit Python 3.10-3.13 (recommended: Python 3.12).');return 1
    root=Path(__file__).resolve().parent;env=root/'.venv_gui'
    python=env/('Scripts/python.exe' if os.name=='nt' else 'bin/python3')
    if not python.exists():
        print('Creating private GUI environment:',env,flush=True);venv.create(env,with_pip=True)
    requirement=root/'requirements_gui.txt';fingerprint=hashlib.sha256(requirement.read_bytes()).hexdigest();stamp=env/'j280_requirements.sha256'
    check=subprocess.run([str(python),'-c','import PySide6, pyqtgraph, numpy, serial; from PySide6 import QtWidgets'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    if check.returncode or not stamp.exists() or stamp.read_text(encoding='utf-8')!=fingerprint:
        subprocess.run([str(python),'-m','ensurepip','--upgrade'],check=True)
        platform='windows' if os.name=='nt' else sys.platform
        cache=root.parent/'offline'/'wheels'/f'{platform}-py{sys.version_info.major}{sys.version_info.minor}'
        install=[str(python),'-m','pip','install','--disable-pip-version-check']
        if cache.exists() and any(cache.glob('*.whl')):
            print('Installing from local dependency cache:',cache,flush=True)
            install+=['--no-index','--find-links',str(cache)]
        else:
            print('Installing locked dependencies. First setup needs internet (about 250-450 MB).',flush=True)
        subprocess.run(install+['-r',str(requirement)],check=True)
        subprocess.run([str(python),'-c','from PySide6 import QtWidgets; import pyqtgraph, serial'],check=True)
        stamp.write_text(fingerprint)
    print('GUI environment ready:',python,flush=True);return 0

if __name__=='__main__':
    try:raise SystemExit(main())
    except Exception as e:print('Setup failed:',e,flush=True);raise SystemExit(1)
