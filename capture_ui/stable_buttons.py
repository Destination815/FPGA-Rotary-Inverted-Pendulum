"""Commit one final enable state per render. Intermediate legacy gates must not
cancel Qt mouse presses before the authoritative R10 gate is evaluated.
Real final disable still cancels a press, as Qt normally does.
"""
from PySide6 import QtWidgets
class StableButton(QtWidgets.QPushButton):
    def begin_enabled_update(self):
        self._enabled_batch=True
        self._next_enabled=None
    def setEnabled(self,enabled):
        if getattr(self,'_enabled_batch',False):self._next_enabled=bool(enabled)
        else:super().setEnabled(enabled)
    def finish_enabled_update(self,failed=False):
        value=False if failed else self._next_enabled
        self._enabled_batch=False
        if value is not None:super().setEnabled(value)

def button(text,callback,primary=False):
    q=StableButton(text);q.clicked.connect(callback)
    if primary:q.setObjectName('primary')
    return q
