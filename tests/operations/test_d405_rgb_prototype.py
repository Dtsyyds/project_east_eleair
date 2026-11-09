"""RGB preprocessing, visible-only supervision, and held-out-label isolation."""
from dataclasses import replace
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest

from d405_seam_tracking.cli import synthetic_frame
from d405_seam_tracking.contracts import sha256
from d405_seam_tracking.dataset import save_sample
from d405_seam_tracking.learning import preprocess, training_labels, training_split_status


def annotation():
    return dict(status='approved', origin='human', author='synthetic_test_fixture', no_seam=False,
                left_xy=[[30,10],[30,50]], right_xy=[[50,10],[50,50]])


def test_rgb_channel_order_normalization_and_default_grayscale():
    image=np.zeros((64,96,3),np.uint8);image[:,:,0]=255
    before=image.copy()
    rgb, transform=preprocess(image,(64,96),rgb=True)
    np.testing.assert_allclose(rgb[0,:,0,0],[(1-.485)/.229,-.456/.224,-.406/.225],rtol=1e-6)
    assert rgb.shape==(1,3,64,96)
    gray,_=preprocess(image,(64,96))
    assert gray.shape==(1,1,64,96)
    assert gray[0,0,0,0]==pytest.approx(76/255)
    np.testing.assert_array_equal(image,before)


def test_visible_range_and_uncertain_boundaries_are_ignored():
    target=training_labels(annotation(),(64,96,3),2)
    assert (target[:10]==-1).all() and (target[51:]==-1).all()
    assert (target[10:51,28:33]==-1).all() and (target[10:51,48:53]==-1).all()
    assert (target[10:51,33:48]==1).all()
    assert (target[10:51,:28]==0).all()


def test_segment_split_never_renames_real_session_or_allows_leakage():
    records=[dict(session='real_session',capture_segment='a',split='train'),
             dict(session='real_session',capture_segment='b',split='test')]
    result=training_split_status(records,'capture_segment')
    assert result['session_count']==1 and not result['independent_generalization_sufficient']
    records[1]['capture_segment']='a'
    with pytest.raises(ValueError,match='segment_leaks'):
        training_split_status(records,'capture_segment')
    with pytest.raises(ValueError,match='session_leaks'):
        training_split_status(records)


def test_rgb_training_never_opens_test_frame_and_loads_encoder_exactly(tmp_path):
    torch=pytest.importorskip('torch');pytest.importorskip('segmentation_models_pytorch')
    from safetensors.torch import save_file
    from d405_seam_tracking.learning import build_model, train
    torch.set_num_threads(2)
    encoder_state={k:v.clone() for k,v in build_model(in_channels=3).encoder.state_dict().items()}
    weights=tmp_path/'fixture_encoder.safetensors';save_file(encoder_state,str(weights))
    frame=synthetic_frame(shape=(64,96))
    frame=replace(frame,image=np.repeat(frame.image[:,:,None],3,axis=2))
    record=save_sample(tmp_path,frame,'train','real_session',split='train',annotation=annotation())
    record['capture_segment']='a'
    test=dict(sample='nonexistent_test.npz',sha256='deliberately_unreadable',session='real_session',
              split='test',capture_segment='b',annotation=None)
    index=tmp_path/'index.json';index.write_text(json.dumps(dict(schema='d405_dataset_v1',samples=[record,test])))
    checkpoint=train(index,tmp_path/'trained',epochs=1,size=(64,96),rgb=True,encoder_weights_path=weights,
                     split_group='capture_segment',ignore_boundary_pixels=2,freeze_encoder_epochs=1)
    value=torch.load(checkpoint,map_location='cpu',weights_only=True)
    assert value['training_samples']==['train.npz']
    assert value['architecture']=='smp_unet_mobilenet_v2_rgb_imagenet'
    assert value['encoder_weights']==sha256(weights)
    for key,tensor in encoder_state.items():
        assert torch.equal(value['model']['encoder.'+key],tensor)


def test_protocol_frame_categories_and_evaluation_clip():
    directory=Path(__file__).resolve().parents[2]/'experiments/vision/scripts'
    sys.path.insert(0,str(directory))
    try:
        spec=importlib.util.spec_from_file_location('rgb_round1',directory/'d405_unet_round1.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    finally:
        sys.path.pop(0)
    row=dict(prediction=True,annotated_rows=100,covered_rows=60,errors={'center':dict(mean=7,p95=15)})
    assert module.classify(row,module.JUDGEMENT)=='correct_seam'
    row['errors']['center']['mean']=30
    assert module.classify(row,module.JUDGEMENT)=='wrong_seam'
    row['prediction']=False
    assert module.classify(row,module.JUDGEMENT)=='no_output'
    original=annotation();clipped=module.clip_annotation_y(original,20,40)
    assert original['left_xy']==[[30,10],[30,50]]
    assert clipped['left_xy']==[[30.,20.],[30.,40.]]


def test_new_capture_split_is_exhaustive_and_never_reuses_held_out_segments():
    directory=Path(__file__).resolve().parents[2]/'experiments/vision/scripts'
    sys.path.insert(0,str(directory))
    try:
        spec=importlib.util.spec_from_file_location('rgb_round1_splits',directory/'d405_unet_round1.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    finally:
        sys.path.pop(0)
    split=dict(segments=[[1,3],[4,8],[9,16],[17,26],[27,30]],
               folds=[['segment_01','segment_04'],['segment_02'],['segment_03','segment_05']],
               basis='Visual pose groups before training')
    segments,folds=module.validate_split_spec(split)
    assert module.capture_segment(8,segments)=='segment_02'
    assert module.capture_segment(30,segments)=='segment_05'
    import copy
    overlap=copy.deepcopy(split);overlap['segments'][1][0]=3
    with pytest.raises(ValueError,match='cover_each_frame_once'):
        module.validate_split_spec(overlap)
    missing=copy.deepcopy(split);missing['segments'][-1][1]=29
    with pytest.raises(ValueError,match='cover_each_frame_once'):
        module.validate_split_spec(missing)
    repeated=copy.deepcopy(split);repeated['folds'][2][0]='segment_02'
    with pytest.raises(ValueError,match='held_out_once'):
        module.validate_split_spec(repeated)
    with pytest.raises(ValueError,match='nonempty_train_and_test'):
        module.validate_split_spec(dict(split,folds=[sum(folds,[])]))


def test_capture_only_requires_explicit_frozen_decoder_config(tmp_path):
    directory=Path(__file__).resolve().parents[2]/'experiments/vision/scripts'
    sys.path.insert(0,str(directory))
    try:
        spec=importlib.util.spec_from_file_location('rgb_capture_config',directory/'d405_unet_round1.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    finally:
        sys.path.pop(0)
    raw=tmp_path/'config.yaml'
    raw.write_text('algorithm: capture_only\nsync_tolerance_ms: 12.0\n')
    before=raw.read_bytes()
    with pytest.raises(ValueError,match='capture_only_requires_explicit_visual_config'):
        module.resolve_visual_config(tmp_path)
    decoder=directory.parents[2]/'config/d405/round1_a.yaml'
    path,config=module.resolve_visual_config(tmp_path,decoder)
    assert path==decoder.resolve() and config['candidate_selection']=='continuous'
    assert raw.read_bytes()==before
    (tmp_path/'decoder_config.yaml').write_bytes(decoder.read_bytes())
    (tmp_path/'visual_config.json').write_text(json.dumps(config))
    (tmp_path/'protocol.json').write_text(json.dumps({
        'decoder_config_sha256':sha256(tmp_path/'decoder_config.yaml'),
        'visual_config_sha256':sha256(tmp_path/'visual_config.json')}))
    (tmp_path/'visual_config.json').write_text('{}')
    with pytest.raises(ValueError,match='frozen_visual_config_changed'):
        module.verify_protocol(tmp_path)
