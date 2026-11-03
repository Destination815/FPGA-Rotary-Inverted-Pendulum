"""C831 downloader 1.0: offline decode or explicitly requested read-only serial.

This research tool has no START/ARM/STOP/CAL/target command interface.
The new format gets raw_samples_1khz.csv and schema.json, never the legacy name.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import struct
import time
import compact_protocol as cp
import legacy_protocol as legacy
import legacy_r32_protocol as legacy_r32

VERSION='R32-R33-legacy-read-only-1.0'
ALLOWED_OPCODES=frozenset((0x30,0x32))

class ReadOnlyLink:
    def __init__(self,transport):self.transport=transport;self.seq=0
    def send(self,opcode):
        if opcode not in ALLOWED_OPCODES:raise ValueError('read-only: only INFO and READ are permitted')
        self.seq=(self.seq+1)&255
        self.transport.write(legacy.command(self.seq,opcode,0))

class Stream:
    def __init__(self):self.buffer=bytearray();self.coordinates=None;self.bad=0;self.frames=[]
    def feed(self,data):
        self.buffer.extend(data);out=[]
        while len(self.buffer)>=64:
            p=self.buffer.find(b'\xa5\x5a')
            if p<0:self.bad+=len(self.buffer)-1;self.buffer[:]=self.buffer[-1:];break
            if p:self.bad+=p;del self.buffer[:p]
            if len(self.buffer)<64:break
            frame=bytes(self.buffer[:64])
            try:
                if frame[2] in (7,8):r=cp.decode(frame)
                elif frame[2] in (1,5):r={'kind':'live','row':legacy.decode(frame)}
                else:
                    build=int.from_bytes(frame[52:56] if frame[2]==3 else frame[4:8],'little')
                    decoder=legacy_r32 if build==0x4a070231 else legacy
                    r=decoder.decode_d02(frame,coordinates=self.coordinates)
                    if r['kind']=='coordinates':
                        if not decoder.descriptor_valid(r):raise ValueError('invalid coordinate descriptor')
                        self.coordinates=r
            except ValueError:
                self.bad+=1;del self.buffer[0];continue
            self.frames.append(frame);out.append(r);del self.buffer[:64]
        return out

def frozen_info(records,expected_build):
    infos=[r for r in records if r['kind']=='compact_info']
    if infos:
        info=infos[-1]
        if info['build']!=expected_build:raise ValueError('unexpected build')
        if not info['valid'] or info['busy']:raise ValueError('compact capture is not frozen')
        if not info['capture_id']&0x80000000:raise ValueError('not a passive capture ID')
        return info
    infos=[r for r in records if r['kind']=='info']
    if not infos:raise ValueError('no supported INFO')
    info=infos[-1]
    if info['build'] not in (0x4a070231,0x4a070232) or info['build']!=expected_build or info['sample_bytes']!=44 or not 0<info['count']<=4096 or info['phase']!=4:
        raise ValueError('unsupported legacy capture')
    return info

def safe_idle(records):
    live={r['row']['ms']:r['row'] for r in records if r['kind']=='live'}
    if len(live)<3:raise ValueError('need three distinct live samples before read')
    for row in list(live.values())[-3:]:
        if row['state']!=0 or row['flags']&0x11 or row['u']!=0:raise ValueError('read requires stopped, idle, disabled device')

def export(wire,out,expected_build):
    parser=Stream();records=parser.feed(wire);info=frozen_info(records,expected_build)
    info=dict(info,coordinate_descriptor=parser.coordinates,coordinate_verified=bool(parser.coordinates))
    out=Path(out);out.mkdir(parents=True,exist_ok=False)
    if info['kind']=='compact_info':
        rows,summary=cp.validate_capture(info,[r for r in records if r['kind']=='compact_data'])
        name='raw_samples_1khz.csv';schema=dict(schema=cp.SCHEMA,fields=cp.FIELDS,integer_units=True,
            actual_theta_field='theta_q16',actual_omega_field='dtheta_q16',
            missing=['filtered_theta_q18','raw_adc_instantaneous','reference','reference_velocity','dphi'],
            interpretation='No missing signal is replaced by zero or interpolation. Reference reconstruction is a separate derived artifact.')
    else:
        packets=[r for r in records if r['kind']=='sample'];by_index={}
        for r in packets:
            if (r['build'],r['capture_id'],r['count'])!=(info['build'],info['capture_id'],info['count']):raise ValueError('legacy capture identity changed')
            if r['index'] in by_index and by_index[r['index']]!=r['row']:raise ValueError('conflicting legacy row')
            by_index[r['index']]=r['row']
        if sorted(by_index)!=list(range(info['count'])):raise ValueError('missing legacy rows')
        rows=[by_index[i] for i in range(info['count'])];name='samples_1khz.csv'
        schema=dict(schema='legacy_352_read_only',coordinate_conversion='original verified C101 decoder')
        summary=dict(schema='legacy_352_read_only',retained_count=len(rows))
    with (out/name).open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    (out/'wire.bin').write_bytes(wire)
    (out/'info.json').write_text(json.dumps(info,indent=2)+'\n')
    (out/'schema.json').write_text(json.dumps(schema,indent=2)+'\n')
    (out/'metadata_frames.json').write_text(json.dumps([r for r in records if r['kind'] not in ('compact_data','sample','live')],indent=2)+'\n')
    summary.update(downloader=VERSION,parser_rejected_bytes=parser.bad,partial_rx_bytes=len(parser.buffer),
                   wire_sha256=hashlib.sha256(wire).hexdigest(),motion_commands_sent=False)
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest()for p in out.iterdir()if p.is_file()}
    (out/'SHA256.json').write_text(json.dumps(hashes,indent=2)+'\n')
    return summary

def collect(transport,expected_build,timeout=100):
    link=ReadOnlyLink(transport);stream=Stream();all_records=[];wire=bytearray()
    link.send(0x30);deadline=time.monotonic()+5
    while time.monotonic()<deadline:
        chunk=transport.read(4096);wire.extend(chunk);all_records.extend(stream.feed(chunk))
        try:info=frozen_info(all_records,expected_build);safe_idle(all_records)
        except ValueError:continue
        break
    else:raise ValueError('no frozen capture with verified build and idle state')
    link.send(0x32);deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        chunk=transport.read(4096);wire.extend(chunk);all_records.extend(stream.feed(chunk))
        current=frozen_info(all_records,expected_build)
        if (current['capture_id'],current['count'])!=(info['capture_id'],info['count']):raise ValueError('capture restarted during download')
        if info['kind']=='compact_info':
            try:cp.validate_capture(info,[r for r in all_records if r['kind']=='compact_data'])
            except ValueError:continue
            return bytes(wire)
        indices={r['index'] for r in all_records if r['kind']=='sample' and r['capture_id']==info['capture_id']}
        if indices==set(range(info['count'])):return bytes(wire)
    raise TimeoutError('incomplete read; do not repeat motion to retry read')

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--version',action='version',version=VERSION)
    source=p.add_mutually_exclusive_group(required=True);source.add_argument('--wire',type=Path);source.add_argument('--port')
    p.add_argument('--expected-build',required=True,type=lambda x:int(x,16));p.add_argument('--out',required=True,type=Path)
    a=p.parse_args()
    if a.expected_build not in (0x4a070231,0x4a070232):raise ValueError('Legacy reader supports frozen R32/R33 only')
    if a.wire:wire=a.wire.read_bytes()
    else:
        import serial
        with serial.Serial(a.port,115200,timeout=.2) as transport:wire=collect(transport,a.expected_build)
    print(json.dumps(export(wire,a.out,a.expected_build),indent=2))

if __name__=='__main__':main()
