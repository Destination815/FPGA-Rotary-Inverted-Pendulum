"""Pure received-frame STOP evidence. No commands, timers, IO or retry authority."""
class StopTailGate:
    duration_ms = 5000
    max_gap_ms = 100

    def __init__(self):
        self.seq = self.ack_ms = self.since = self.last_ms = None
        self.complete = False

    def event(self, event):
        kind, name = event.get('event'), event.get('name')
        if kind == 'send' and name == 'stop':
            self.__init__(); self.seq = event.get('seq')
        elif kind == 'ack' and name == 'stop':
            if self.seq is not None and event.get('seq') == self.seq and event.get('status') == 0:
                self.ack_ms = int(event['ms']); self.since = self.last_ms = None
            else:
                self.__init__()
        elif kind in ('reset', 'stale', 'disconnected', 'error', 'unsafe_stop', 'firmware_changed') or (kind == 'send' and name not in ('stop', 'd02info')):
            self.__init__()
        elif kind == 'timeout' and name == 'stop':
            self.__init__()

    def feed(self, row, fresh=True):
        ms = int(row.get('ms', 0))
        valid = (fresh and self.ack_ms is not None and
                 ((ms-self.ack_ms)&0xffffffff) < 0x80000000 and
                 row.get('state') == 0 and row.get('fault') == 0 and
                 not int(row.get('flags', 1)) & 1 and row.get('u') == 0)
        if not valid:
            self.since = self.last_ms = None; self.complete = False; return False
        if self.last_ms == ms:
            return self.complete  # Repeated UI snapshots cannot advance board time.
        gap = None if self.last_ms is None else (ms-self.last_ms)&0xffffffff
        if gap is not None and not 0 < gap <= self.max_gap_ms:
            self.since = None; self.complete = False
        if self.since is None:self.since = ms
        self.last_ms = ms
        self.complete = ((ms-self.since)&0xffffffff) >= self.duration_ms
        return self.complete

    def evidence(self):
        return dict(schema='j280.a39.stop-tail.v1', stop_seq=self.seq, ack_ms=self.ack_ms,
                    first_idle_ms=self.since, last_idle_ms=self.last_ms,
                    duration_ms=0 if self.since is None or self.last_ms is None else (self.last_ms-self.since)&0xffffffff,
                    required_ms=self.duration_ms, maximum_gap_ms=self.max_gap_ms, complete=self.complete)
