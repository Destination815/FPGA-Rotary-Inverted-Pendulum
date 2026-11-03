"""Compatibility import: the R10 host and diagnostic tools share one decoder."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"host"))
from protocol import *
