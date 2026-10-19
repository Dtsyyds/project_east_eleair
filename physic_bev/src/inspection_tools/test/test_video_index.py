"""Actual H.264 decoding fixtures for the shared frame identity contract."""
import json
from pathlib import Path
import subprocess
import pytest
from inspection_tools.video_index import create_frame_map, aligned_rows, VideoAlignmentError


def fixture(root, indices=None, raw=False):
    video = root / "front_000001.mkv"
    root.mkdir(parents=True, exist_ok=True)
    indices = [0,1,2,3,4] if indices is None else indices
    # Different pixel values and unique generated PTS make missing positions known.
    data = b"".join(bytes([i*30])*32*24*3 for i in range(5))
    elementary = root / "source.h264"
    target = elementary if raw else video
    cmd=['ffmpeg','-v','error','-f','rawvideo','-pixel_format','bgr24','-video_size','32x24',
         '-framerate','10','-i','pipe:0','-c:v','libx264','-preset','ultrafast','-tune','zerolatency']
    if indices != list(range(5)):
        cmd += ['-vf', 'select=' + '+'.join('eq(n\\,'+str(i)+')' for i in indices), '-vsync', '0']
    subprocess.run(cmd+['-f','h264' if raw else 'matroska',str(target)],input=data,check=True,capture_output=True)
    rows=[{'frame_index':i,'ros_header_stamp_ns':1000000000+i*100000000,
           'encoded_pts_ns':i*100000000,'encoder_contract':'rawvideo_fixed_fps_unique_pts_v2'} for i in range(5)]
    sidecar=video.with_suffix('.frames.jsonl')
    sidecar.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    return video,sidecar


def test_normal_and_cross_segment_exact_mapping(tmp_path):
    for n in range(2):
        video,sidecar=fixture(tmp_path/str(n))
        create_frame_map(video,sidecar,video.with_suffix('.frame_map.json'))
        rows=aligned_rows(video)
        assert [r['source_frame_index'] for r in rows] == list(range(5))
        assert [r['ros_header_stamp_ns'] for r in rows] == [1000000000+i*100000000 for i in range(5)]

@pytest.mark.parametrize('indices', [[1,2,3,4],[0,1,3,4]])
def test_missing_first_or_middle_has_known_pts_mapping(tmp_path,indices):
    video,sidecar=fixture(tmp_path,indices)
    m=create_frame_map(video,sidecar,video.with_suffix('.frame_map.json'))
    assert [r['source_frame_index'] for r in aligned_rows(video)] == indices
    assert m['unmapped_source_indices']==sorted(set(range(5))-set(indices))


def test_legacy_equal_counts_does_not_prove_alignment(tmp_path):
    video,sidecar=fixture(tmp_path)
    with pytest.raises(VideoAlignmentError,match='no verified'):
        aligned_rows(video)
    rows=[json.loads(x) for x in sidecar.read_text().splitlines()]
    for row in rows: row.pop('encoder_contract')
    sidecar.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    with pytest.raises(VideoAlignmentError,match='legacy'):
        create_frame_map(video,sidecar,video.with_suffix('.frame_map.json'))


def test_bad_video_and_mutated_sidecar_rejected(tmp_path):
    video,sidecar=fixture(tmp_path)
    create_frame_map(video,sidecar,video.with_suffix('.frame_map.json'))
    sidecar.write_text(sidecar.read_text().replace('1000000000','999999999'))
    with pytest.raises(VideoAlignmentError,match='SHA'):
        aligned_rows(video)
    video.write_bytes(b'corrupt video')
    with pytest.raises(VideoAlignmentError):
        create_frame_map(video,sidecar,tmp_path/'corrupt_map.json')


def test_new_h264_segment_maps_vcl_identity_after_mux(tmp_path):
    import ast
    import os
    import typing
    from inspection_tools.video_index import probe_video
    raw=tmp_path/'fixture.h264'
    subprocess.run(['ffmpeg','-v','error','-f','lavfi','-i','testsrc=size=64x48:rate=10',
                    '-frames:v','5','-c:v','libx264','-preset','ultrafast','-tune','zerolatency',
                    '-pix_fmt','yuv420p','-f','h264',str(raw)],check=True,capture_output=True)
    packets=probe_video(raw,True)
    # Compile only the actual mux Segment class: importing the ROS recorder
    # would unnecessarily require img_msgs even though this fixture opens no ROS node.
    source=Path(__file__).parents[3]/'scripts/front_h264_recorder.py'
    tree=ast.parse(source.read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Segment')
    namespace={'Path':Path,'Any':typing.Any,'json':json,'os':os,'subprocess':subprocess,
               'create_frame_map':create_frame_map}
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(source),'exec'),namespace)
    output=tmp_path/'video';output.mkdir()
    segment=namespace['Segment'](output,1,10.,b'')
    with raw.open('rb') as stream:
        for i,packet in enumerate(packets):
            stream.seek(int(packet['pos']));data=stream.read(int(packet['size']))
            segment.write({'data':data,'ros_header_stamp_ns':1000000000+i*100000000,
                'pts_ns':1000000000+i*100000000,'dts_ns':1000000000+i*100000000,
                'receive_monotonic_ns':i,'width':64,'height':48,'source_sequence':i,
                'is_idr':i==0,'dropped_before_encode':0,'encoder_queue_depth':0})
    status=segment.close()
    assert status['complete'] is True
    video=Path(status['video'])
    mapped=aligned_rows(video)
    assert len(mapped)==5 and [r['source_sequence'] for r in mapped]==list(range(5))
    assert all(r['alignment_evidence']=='unique_vcl_identity_and_decoder_packet_position' for r in mapped)
    assert not list(output.glob('*.partial'))


def test_delayed_encoder_flush_with_b_frames_has_exact_pts_mapping(tmp_path):
    video=tmp_path/'buffered.mkv'
    # No zerolatency: libx264 buffers input and uses B frames. communicate closes
    # stdin and waits for encoder flush; actual decoded presentation PTS is used.
    data=b''.join(bytes([i*25])*32*24*3 for i in range(8))
    subprocess.run(['ffmpeg','-v','error','-f','rawvideo','-pixel_format','bgr24',
                    '-video_size','32x24','-framerate','10','-i','pipe:0',
                    '-c:v','libx264','-preset','medium','-bf','2','-pix_fmt','yuv420p',
                    str(video)],input=data,check=True,capture_output=True)
    sidecar=video.with_suffix('.frames.jsonl')
    rows=[{'frame_index':i,'ros_header_stamp_ns':1000000000+i*100000000,
           'encoded_pts_ns':i*100000000,'encoder_contract':'rawvideo_fixed_fps_unique_pts_v2'} for i in range(8)]
    sidecar.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    mapping=create_frame_map(video,sidecar,video.with_suffix('.frame_map.json'))
    assert mapping['decoded_count']==8 and mapping['unmapped_source_indices']==[]
    assert [row['source_frame_index'] for row in aligned_rows(video)]==list(range(8))
