"""R33 frozen-window reader. Only INFO (48) and READ (50), never changes control."""
from pathlib import Path
from datetime import datetime
import argparse
import csv
import hashlib
import json
import sys
import time
import zipfile
from protocol import CONTROL_BUILD, Decoder, STATES, command


def window_metadata(snapshot, ordered):
    """Describe retained evidence, without treating a tail sentinel as a trigger."""
    trigger = snapshot['trigger_index']
    has_capture = (0 < trigger < len(ordered) and
                   ordered[trigger]['state'] == 2 and
                   ordered[trigger - 1]['state'] in (1, 6, 7))
    from_start = snapshot.get('recording_session') == 'automatic_start'
    if snapshot['reason'] == 6:
        if not from_start or len(ordered) != 4096:
            raise RuntimeError('自动首窗标识/4096样本不一致；保留诊断，不重做运动。')
    if snapshot['reason'] == 5 and (not has_capture or len(ordered)-trigger-1 != 3072):
        raise RuntimeError('接管窗口标识与真实状态样本不一致；保留诊断，不重做运动。')
    return dict(
        window_kind='automatic_start_first_samples' if from_start else 'automatic_capture_transition' if has_capture else 'tail_before_freeze',
        recording_session=snapshot.get('recording_session','legacy_unspecified'),
        recording_completed_by_limit=snapshot['reason'] == 6,
        buffer_freeze_is_motion_stop=False,
        freeze_reason=snapshot['reason'],
        freeze_reason_text={1:'STOP',3:'first_fault',4:'32_second_recording_limit',
                            5:'3072_samples_after_first_automatic_capture',
                            6:'4096_samples_from_automatic_START'}.get(snapshot['reason'],'unknown'),
        start_ms=ordered[0]['ms'], end_ms=ordered[-1]['ms'],
        capture_id=snapshot['capture_id'], trigger_index=trigger,
        capture_transition_ms=ordered[trigger]['ms'] if has_capture else None,
        samples_before_capture=trigger if has_capture else None,
        samples_after_capture=len(ordered)-trigger-1 if has_capture else None,
        scope='Only this retained 1kHz window. Full-run 50Hz/raw_RX must also be returned; later events may be outside this window.')


def validate(info, live):
    if info is None:
        raise RuntimeError('没有收到板端 INFO；检查端口及供电，不要复位。')
    if info['build'] != CONTROL_BUILD:
        raise RuntimeError(f'版本不符：{info["build"]:08X}，本工具仅用于 R42 / 4A070256。')
    if not info.get('coordinate_verified'):raise RuntimeError('缺少一致的坐标描述帧；仅重读，不需运动。')
    if live is None:
        raise RuntimeError('没有收到实时状态；不能判断是否停止。保持供电，返回诊断 ZIP。')
    detail = (f'state={STATES.get(live["state"], live["state"])} '
              f'fault={live["fault"]} flags=0x{live["flags"]:04X} u={live["u"]}')
    if live['flags'] & 0x11 or live['u'] != 0:
        raise RuntimeError(detail + '：仍使能或输出非零。关闭本工具，用 Studio 停止输出，'
                           '确认未使能且输出为零，关闭 Studio 后重试。')
    if live['state'] == 5:
        raise RuntimeError(detail + '：故障已锁存。STOP 不会清除故障，R10 板端也不允许在 FAULT 读缓存。'
                           '先保存 run，再在 Studio 点击“清除故障（未使能时）”；'
                           '确认待机/未使能/输出零后关闭 Studio 并重试。清除故障不清空缓存。')
    if live['state'] != 0:
        raise RuntimeError(detail + '：尚非待机。用 Studio 停止输出并等待待机；不要再次捕获。')
    if not info['capture_id'] & 0x80000000 or info['phase'] != 4 or not 0 < info['count'] <= 4096:
        raise RuntimeError(f'缓存未冻结或不存在：phase={info["phase"]}, count={info["count"]}, '
                           f'capture_id={info["capture_id"]:08X}。保留供电并返回诊断 ZIP，不自动重做运动。')


def collect_legacy(link, out, timeout=4, read_timeout=60):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    decoder = Decoder()
    evidence = dict(tool='R33 frozen window reader 2', commands_sent=[], status='FAILED',
                    scope='Read only. No STOP/CLEAR/ARM/START/CAPTURE/RESET is sent.')
    info, live, rows, history = None, None, {}, []
    wire = bytearray()
    error = None

    def send(seq, opcode):
        assert opcode in (48, 50)
        link.write(command(seq, opcode))
        evidence['commands_sent'].append(opcode)

    def receive():
        nonlocal info, live
        data = link.read(4096)  # Drain diagnostic queue every chunk, below its 512-frame capacity.
        wire.extend(data)
        frames = decoder.feed(data)
        if frames:
            live = frames[-1]
            history.extend(frames)
        diagnostics = decoder.take_diagnostics()
        for item in diagnostics:
            if item['kind'] == 'info':
                info = dict(item,coordinate_verified=bool(decoder.coordinates),coordinate_descriptor=decoder.coordinates)
        return frames, diagnostics

    try:
        send(231, 48)
        deadline = time.monotonic() + timeout
        distinct_times = set()
        while time.monotonic() < deadline:
            frames, _ = receive()
            distinct_times.update(f['ms'] for f in frames)
            if info is not None and decoder.coordinates is not None and decoder.coordinates['capture_id']==info['capture_id'] and len(distinct_times) >= 3:
                break
        if info is not None:info=dict(info,coordinate_verified=bool(decoder.coordinates),coordinate_descriptor=decoder.coordinates)
        validate(info, live)
        if len(distinct_times) < 3:
            raise RuntimeError('实时状态不足三帧，未开始读取；保留供电并返回诊断 ZIP。')
        capture_id, count = info['capture_id'], info['count']
        print(f'正在读取已冻结窗口 {count} 个毫秒样本；完整 4096 点约需半分钟，请保持供电。', flush=True)
        snapshot = dict(info)
        send(232, 50)
        deadline = time.monotonic() + read_timeout
        while len(rows) < count and time.monotonic() < deadline:
            _, diagnostics = receive()
            if info['capture_id'] != capture_id or info['count'] != count:
                raise RuntimeError('读取期间缓存标识/数量改变；已保存原始数据。')
            if live['state'] != 0 or live['flags'] & 0x11 or live['u'] != 0:
                raise RuntimeError('读取期间板端不再处于未使能待机零输出；结束读取并保存诊断。')
            for item in diagnostics:
                if item['kind'] != 'sample':
                    continue
                if (item['capture_id'] != capture_id or item['build'] != CONTROL_BUILD
                        or item['count'] != count or item['phase'] != 4):
                    raise RuntimeError('混入不同缓存/版本/数量/阶段的样本。')
                index = item['index']
                if not 0 <= index < count:
                    raise RuntimeError('样本索引越界。')
                if index in rows and rows[index] != item['row']:
                    raise RuntimeError('同一索引收到不同内容。')
                rows[index] = item['row']
        if set(rows) != set(range(count)):
            raise RuntimeError(f'读取不完整 {len(rows)}/{count}；可重读同一缓存，不必重新运动。')
        ordered = [rows[i] for i in range(count)]
        if any(row['sample_index'] != i for i, row in enumerate(ordered)):
            raise RuntimeError('样本内索引不一致。')
        if any(((b['ms'] - a['ms']) & 0xffffffff) != 1 for a, b in zip(ordered, ordered[1:])):
            raise RuntimeError('记录存在非 1 ms 间隔，原始数据已保留。')
        with (out / 'samples_1khz.csv').open('w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=list(ordered[0]))
            writer.writeheader()
            writer.writerows(ordered)
        evidence.update(status='PASS', count=count, interval_ms=1,
                        trigger_index=snapshot['trigger_index'],
                        recording_window=window_metadata(snapshot, ordered))
    except Exception as exc:
        error = str(exc)
        evidence['error'] = error
    finally:
        evidence.update(received_samples=len(rows), bad_frames=decoder.bad_frames)
        (out / 'wire.bin').write_bytes(wire)
        for name, obj in [('info.json', info), ('live_status.json', live),
                          ('status_history.json', history), ('summary.json', evidence),
                          ('partial_samples.json', rows)]:
            (out / name).write_text(json.dumps(obj, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir() if p.is_file()}
        (out / 'SHA256.json').write_text(json.dumps(hashes, indent=2) + '\n', encoding='utf-8')
        archive = out.with_suffix('.zip')
        with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as z:
            for p in out.iterdir():
                z.write(p, out.name + '/' + p.name)
    print(f'{evidence["status"]}: {archive}')
    if error:
        raise RuntimeError(error)
    return archive


def collect(link, out, timeout=5, read_timeout=100):
    """Default C831/v1 collector; legacy public helpers above remain read-only."""
    import compact_protocol as cp
    from coordinates import descriptor_valid
    out=Path(out);out.mkdir(parents=True,exist_ok=False)
    decoder=Decoder();wire=bytearray();history=[];records=[];info=None;legacy_info=None;live=None;error=None
    evidence=dict(tool='R33 C831 reader 3',status='FAILED',commands_sent=[],motion_commands_sent=False)
    def send(op):
        if op not in (48,50):raise ValueError('Read only')
        link.write(command(230+len(evidence['commands_sent']),op));evidence['commands_sent'].append(op)
    def receive():
        nonlocal info,legacy_info,live
        data=link.read(4096);wire.extend(data);frames=decoder.feed(data);history.extend(frames)
        if frames:live=frames[-1]
        for item in decoder.take_diagnostics():
            records.append(item)
            if item['kind']=='compact_info':info=item
            elif item['kind']=='info':legacy_info=item
        return frames
    try:
        send(48);deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            receive()
            if info and decoder.coordinates and len({r['ms'] for r in history})>=3:break
        if info is None:raise RuntimeError('No C831 INFO; use a matching legacy reader for frozen 352-bit firmware. Do not repeat motion.')
        descriptor=decoder.coordinates
        if info['build']!=CONTROL_BUILD or not info['capture_id']&0x80000000 or not info['valid'] or info['busy']:raise RuntimeError('Wrong version or cache not frozen')
        if not descriptor or not descriptor_valid(descriptor) or descriptor['capture_id']!=info['capture_id']:raise RuntimeError('Missing capture-bound coordinate descriptor')
        if len({r['ms'] for r in history})<3 or any(r['state']!=0 or r['flags']&0x11 or r['u'] for r in history[-3:]):raise RuntimeError('Read requires three distinct IDLE/disabled/zero-output live samples')
        info=dict(info,phase=4,coordinate_verified=True,coordinate_descriptor=descriptor,motion_config=(legacy_info or {}).get('motion_config'))
        snapshot=dict(info);send(50);deadline=time.monotonic()+read_timeout
        while time.monotonic()<deadline:
            receive()
            if (info['build'],info['capture_id'],info['count'])!=(snapshot['build'],snapshot['capture_id'],snapshot['count']):raise RuntimeError('Cache identity changed during read')
            if live['state']!=0 or live['flags']&0x11 or live['u']:raise RuntimeError('Device left stopped state during read')
            try:ordered,summary=cp.validate_capture(snapshot,[r for r in records if r['kind']=='compact_data'])
            except ValueError as e:
                if str(e)=='missing retained rows':continue
                raise
            break
        else:raise TimeoutError('Incomplete cache read; re-read without motion')
        info=snapshot
        with (out/'raw_samples_1khz.csv').open('w',newline='',encoding='utf-8') as f:
            writer=csv.DictWriter(f,fieldnames=list(ordered[0]));writer.writeheader();writer.writerows(ordered)
        schema=dict(schema=cp.SCHEMA,fields=cp.FIELDS,integer_units=True,actual_theta_field='theta_q16',actual_omega_field='dtheta_q16',missing=['filtered_theta_q18','raw_adc_instantaneous','reference','reference_velocity','dphi'])
        (out/'schema.json').write_text(json.dumps(schema,indent=2)+'\n')
        evidence.update(summary,status='PASS',count=len(ordered),buffer_freeze_is_motion_stop=False)
    except Exception as exc:error=str(exc);evidence['error']=error
    finally:
        evidence['bad_frames']=decoder.bad_frames
        (out/'wire.bin').write_bytes(wire)
        for name,obj in [('info.json',info),('live_status.json',live),('status_history.json',history),('summary.json',evidence),('metadata_frames.json',[r for r in records if r['kind']!='compact_data'])]:
            (out/name).write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir() if p.is_file()}
        (out/'SHA256.json').write_text(json.dumps(hashes,indent=2)+'\n')
        archive=out.with_suffix('.zip')
        with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
            for p in out.iterdir():z.write(p,out.name+'/'+p.name)
    print(evidence['status']+': '+str(archive))
    if error:raise RuntimeError(error)
    return archive

def main():
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port');parser.add_argument('--legacy-build',choices=['4A070231','4A070232'],help='explicit read-only legacy352 mode')
    parser.add_argument('--out', type=Path, default=Path(__file__).resolve().parent.parent / 'logs' / 'passive')
    args = parser.parse_args()
    import serial
    from serial.tools import list_ports
    if not args.port:
        ports = list(list_ports.comports())
        for i, port in enumerate(ports, 1):
            print(f'{i}: {port.device} {port.description}')
        if not ports:
            raise RuntimeError('未找到串口。')
        n = int(input('Select port number: '))
        if not 1 <= n <= len(ports):
            raise RuntimeError('序号不在列表内。')
        args.port = ports[n - 1].device
    destination = args.out / datetime.now().strftime('passive_%Y%m%d_%H%M%S_%f')
    with serial.Serial(args.port, 115200, timeout=.1) as link:
        if args.legacy_build:
            import legacy_capture_reader as legacy
            build=int(args.legacy_build,16);wire=legacy.collect(link,build)
            legacy.export(wire,destination,build)
            archive=destination.with_suffix('.zip')
            with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED)as z:
                for p in destination.iterdir():z.write(p,destination.name+'/'+p.name)
            print('PASS legacy read-only: '+str(archive))
        else:collect(link, destination)


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('FAILED: ' + str(exc))
        sys.exit(1)
