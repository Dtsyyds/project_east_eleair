"""Strict decoded-frame to source timestamp evidence shared by all consumers.

Legacy count equality alone is insufficient. New maps are generated on close
from packet VCL identity or the rawvideo encoder's unique input PTS contract.
"""
from pathlib import Path
import hashlib
import json
import re
import subprocess

class VideoAlignmentError(ValueError):
    pass

def sha256_file(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(4*1024*1024), b''): h.update(block)
    return h.hexdigest()

def probe_video(video, packets=False):
    args = ['ffprobe','-v','error','-select_streams','v:0']
    args += ['-show_packets','-show_data'] if packets else ['-show_frames']
    r = subprocess.run(args+['-of','json',str(video)],capture_output=True,text=True,timeout=120)
    if r.returncode or r.stderr.strip():
        raise VideoAlignmentError(f'decode/probe failed: {r.stderr[:1000]}')
    data = json.loads(r.stdout)
    values = data.get('packets' if packets else 'frames', [])
    if not values: raise VideoAlignmentError('video has no decoded frames/packets')
    return values

def load_rows(sidecar):
    rows = [json.loads(line) for line in Path(sidecar).read_text().splitlines() if line.strip()]
    previous = None
    for i,row in enumerate(rows):
        stamp = int(row['ros_header_stamp_ns'])
        if stamp <= 0 or (previous is not None and stamp <= previous):
            raise VideoAlignmentError('non-positive, repeated or regressing source timestamp')
        if int(row['frame_index']) != i: raise VideoAlignmentError('source journal index is not contiguous')
        previous = stamp
    return rows

def _vcl_digest(data, annexb):
    if annexb:
        nals = re.split(b'\x00\x00\x00?\x01', data)[1:]
    else:
        nals=[]; offset=0
        while offset < len(data):
            n = int.from_bytes(data[offset:offset+4], 'big'); offset += 4
            if n <= 0 or offset+n > len(data): raise VideoAlignmentError('invalid AVCC NAL size')
            nals.append(data[offset:offset+n]); offset += n
    vcl = [n.rstrip(b'\x00') for n in nals if n and n[0]&31 in {1,5}]
    if not vcl: return None
    return hashlib.sha256(b''.join(len(n).to_bytes(4,'big')+n for n in vcl)).hexdigest()

def _packet_bytes(text):
    chunks=[]
    for line in text.splitlines():
        if ':' not in line: continue
        chunk=line.split(':',1)[1].lstrip().split('  ',1)[0].replace(' ','')
        chunks.append(bytes.fromhex(chunk))
    return b''.join(chunks)

def create_frame_map(video, sidecar, destination, raw=None):
    video,sidecar,destination = Path(video),Path(sidecar),Path(destination)
    if destination.exists(): raise FileExistsError(destination)
    rows=load_rows(sidecar); frames=probe_video(video)
    by_source={}
    if raw is not None:
        with Path(raw).open('rb') as stream:
            for i,row in enumerate(rows):
                stream.seek(int(row['raw_offset'])); data=stream.read(int(row['raw_size']))
                digest=_vcl_digest(data, True)
                if digest:
                    if digest in by_source: raise VideoAlignmentError('ambiguous repeated VCL identity')
                    by_source[digest]=i
        packet_sources={}
        for packet in probe_video(video, True):
            digest=_vcl_digest(_packet_bytes(packet['data']),False)
            if digest not in by_source: raise VideoAlignmentError('packet has no unique source VCL identity')
            packet_sources[str(packet['pos'])]=by_source[digest]
        indices=[packet_sources.get(str(frame.get('pkt_pos'))) for frame in frames]
        evidence='unique_vcl_identity_and_decoder_packet_position'
    else:
        if not all(row.get('encoder_contract') == 'rawvideo_fixed_fps_unique_pts_v2' for row in rows):
            raise VideoAlignmentError('legacy sidecar has no strict mapping evidence')
        # Matroska timestamps are quantized to milliseconds. Reject ambiguous matches.
        source_pts=[int(row['encoded_pts_ns']) for row in rows]
        indices=[]
        for frame in frames:
            pts=round(float(frame['best_effort_timestamp_time'])*1e9)
            matches=[i for i,value in enumerate(source_pts) if abs(value-pts)<=600000]
            if len(matches)!=1: raise VideoAlignmentError('decoder PTS does not identify a unique input')
            indices.append(matches[0])
        evidence='rawvideo_encoder_input_pts_matroska_0.6ms_tolerance'
    if any(i is None for i in indices) or len(set(indices))!=len(indices):
        raise VideoAlignmentError('decoded frames cannot be mapped uniquely')
    if indices != sorted(indices): raise VideoAlignmentError('decoder source order regressed')
    document={'schema_version':'decoded_frame_map_v2','evidence':evidence,
              'video_sha256':sha256_file(video),'sidecar_sha256':sha256_file(sidecar),
              'source_count':len(rows),'decoded_count':len(frames),
              'unmapped_source_indices': sorted(set(range(len(rows)))-set(indices)),
              'frames':[{'decoded_frame_index':d,'source_frame_index':i,
                         'ros_header_stamp_ns':int(rows[i]['ros_header_stamp_ns'])} for d,i in enumerate(indices)]}
    destination.write_text(json.dumps(document,indent=2)+'\n')
    return document

def aligned_rows(video, sidecar=None, map_path=None):
    video=Path(video); sidecar=Path(sidecar) if sidecar else video.with_suffix('.frames.jsonl')
    map_path=Path(map_path) if map_path else video.with_suffix('.frame_map.json')
    rows=load_rows(sidecar)
    if not map_path.is_file():
        raise VideoAlignmentError('no verified decoded-frame map; legacy run is not strictly alignable')
    m=json.loads(map_path.read_text())
    if m.get('schema_version')!='decoded_frame_map_v2' or not m.get('evidence'):
        raise VideoAlignmentError('unsupported frame map evidence')
    if m.get('video_sha256')!=sha256_file(video) or m.get('sidecar_sha256')!=sha256_file(sidecar):
        raise VideoAlignmentError('frame map source SHA mismatch')
    frames=probe_video(video)
    mapped=m.get('frames',[])
    if len(frames)!=len(mapped) or len(mapped)!=m.get('decoded_count'):
        raise VideoAlignmentError('decoded count differs from verified map')
    result=[]; seen=set(); last=-1
    for d,entry in enumerate(mapped):
        i=int(entry['source_frame_index'])
        if entry['decoded_frame_index']!=d or i in seen or i<=last or not 0<=i<len(rows):
            raise VideoAlignmentError('invalid/duplicate decoded-frame mapping')
        if int(entry['ros_header_stamp_ns'])!=int(rows[i]['ros_header_stamp_ns']):
            raise VideoAlignmentError('mapped source timestamp mismatch')
        seen.add(i); last=i
        result.append({**rows[i], 'source_frame_index':i, 'frame_index':d,
                       'decoded_frame_index':d, 'alignment_evidence':m['evidence']})
    if len(rows)!=m.get('source_count') or sorted(set(range(len(rows)))-seen)!=m.get('unmapped_source_indices'):
        raise VideoAlignmentError('source conservation differs from map')
    return result
