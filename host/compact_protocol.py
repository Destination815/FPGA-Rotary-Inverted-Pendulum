"""C831/v1 raw 176-bit format. Actual theta/omega integers; no synthetic reference or missing derivatives."""
import binascii
import struct

TAG=0xC831
VERSION=1
CAPABILITIES=0x60
SCHEMA='j280.raw_head_tail_176.v1'
FIELDS=[('ms',32,False),('adc_q4',14,False),('encoder',32,True),
        ('command_pwm',10,True),('applied_pwm',10,True),('flags',16,False),
        ('state',4,False),('limit',9,False),('adc_fine_valid',1,False),('theta_q16',21,True),('dtheta_q16',23,True),('fault',4,False)]
assert sum(x[1] for x in FIELDS)==176

def pack_row(row):
    value=shift=0
    for name,bits,signed in FIELDS:
        x=int(row[name]);lo=-(1<<(bits-1)) if signed else 0
        hi=(1<<(bits-1))-1 if signed else (1<<bits)-1
        if not lo<=x<=hi:raise ValueError(f'{name} outside [{lo}, {hi}]: {x}')
        value|=(x&((1<<bits)-1))<<shift;shift+=bits
    return value.to_bytes(22,'little')

def unpack_row(raw):
    if len(raw)!=22:raise ValueError('raw row must be exactly 22 bytes')
    value=int.from_bytes(raw,'little');row={}
    for name,bits,signed in FIELDS:
        x=value&((1<<bits)-1);value>>=bits
        if signed and x&(1<<(bits-1)):x-=1<<bits
        row[name]=x
    return row

def finalize(payload):
    if len(payload)!=62:raise ValueError('62-byte pre-CRC payload required')
    return payload+struct.pack('<H',binascii.crc_hqx(payload[2:],0xffff))

def decode(frame):
    if len(frame)!=64 or frame[:2]!=b'\xa5\x5a' or frame[3]!=64 or binascii.crc_hqx(frame[2:62],0xffff)!=int.from_bytes(frame[62:],'little'):
        raise ValueError('invalid frame or CRC')
    u16=lambda i:struct.unpack_from('<H',frame,i)[0]
    u32=lambda i:struct.unpack_from('<I',frame,i)[0]
    if frame[2]==7:
        if (u16(12),frame[14],frame[15],u16(16),u16(18))!=(TAG,VERSION,22,CAPABILITIES,8192):
            raise ValueError('unsupported compact INFO contract')
        r=dict(kind='compact_info',build=u32(4),capture_id=u32(8),format_tag=TAG,version=VERSION,
               sample_bytes=22,capabilities=CAPABILITIES,capacity=8192,
               head_count=u16(20),tail_count=u16(22),tail_base=u16(24),count=u16(26),
               total_samples=u32(28),begin_ms=u32(32),event_ms=u32(36),last_ms=u32(40),
               first_gap_ms=u32(44),gaps=u16(48),trigger_index=u16(50),trigger_ms=u32(52),
               reason=frame[56],busy=bool(frame[57]&1),valid=bool(frame[57]&2),
               automatic_session=bool(frame[57]&4),discontinuous=bool(frame[57]&8),freeze_fault=frame[58])
        if not (r['head_count']<=4096 and r['tail_count']<=4096 and r['tail_base']<4096):raise ValueError('invalid head/tail bounds')
        if r['valid']:
            if r['busy'] or r['count']!=r['head_count']+r['tail_count'] or not 0<r['count']<=8192:raise ValueError('invalid frozen count')
            if r['head_count']!=min(r['total_samples'],4096) or r['tail_count']!=min(max(r['total_samples']-4096,0),4096):raise ValueError('inconsistent sample coverage')
            if r['trigger_index']!=65535 and r['trigger_index']>=r['count']:raise ValueError('trigger index out of bounds')
        return r
    if frame[2]==8:
        if frame[11]!=VERSION or u16(60)!=TAG or frame[10] not in (1,2):raise ValueError('unsupported compact DATA contract')
        n=frame[10]
        if u16(8)+n>8192:raise ValueError('invalid DATA bounds')
        return dict(kind='compact_data',capture_id=u32(4),index=u16(8),rows=[unpack_row(frame[12+22*j:34+22*j]) for j in range(n)],build=u32(56))
    # Read-only legacy352 support preserves the original C101 conversion/validation.
    raise ValueError('not a compact packet')

def validate_capture(info,packets):
    if not info['valid'] or info['busy']:raise ValueError('capture is not frozen')
    rows={}
    for packet in packets:
        if (packet['build'],packet['capture_id'])!=(info['build'],info['capture_id']):
            raise ValueError('capture identity changed during read')
        for j,row in enumerate(packet['rows']):
            index=packet['index']+j
            if index>=info['count']:raise ValueError('DATA index exceeds frozen count')
            if index in rows and rows[index]!=row:raise ValueError('conflicting duplicate row')
            rows[index]=row
    if sorted(rows)!=list(range(info['count'])):raise ValueError('missing retained rows')
    out=[];h=info['head_count'];n=info['total_samples'];t=info['tail_count'];observed=[]
    for i in range(info['count']):
        row=dict(rows[i]);ordinal=i if i<h else n-t+i-h
        row.update(retained_index=i,source_ordinal=ordinal,segment='head' if i<h else 'tail')
        if i and ordinal==out[-1]['source_ordinal']+1 and (row['ms']-out[-1]['ms'])&0xffffffff!=1:observed.append(i)
        out.append(row)
    if out[-1]['ms']!=info['last_ms']:raise ValueError('last sample timestamp does not match INFO')
    if observed and not info['gaps']:raise ValueError('unreported timestamp discontinuity')
    return out,dict(schema=SCHEMA,retained_count=len(out),total_samples=n,
                    omitted_samples=max(n-8192,0),observed_discontinuity_indices=observed,
                    session_discontinuities=info['gaps'],reference='not recorded; reconstruction must be marked derived',
                    theta='actual estimator theta Q16 integer, not filtered_theta_q18',dtheta='actual estimator dtheta Q16 integer',arm_velocity='not recorded',filtered_theta='not recorded',raw_adc_instantaneous='not recorded',
                    adc_q4='true 512-conversion block mean Q4; validity is per row',
                    timestamp='stored uint32 millis per row; source ordinal orders wrap/duplicates',
                    full_window_stops_motion=False)
