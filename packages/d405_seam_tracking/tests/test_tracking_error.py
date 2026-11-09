"""Public error signs, local support and invalid-frame behavior (no model/hardware)."""
import copy
import math
import numpy as np
import pytest
from d405_seam_tracking.tracking_error import ErrorReference, tracking_error, invalid_error


def measured(center=288., heading=math.radians(3), curvature=0.):
    y = np.arange(168, 456, 3, dtype=float)
    x = center + math.tan(heading)*(y-360) + curvature*(y-360)**2
    return dict(valid_2d=True, predicted_only=False, state='VISION_2D',
                coordinate_2d='original_image_pixel_centers', visual={'valid': True},
                confidence=.95, points_2d_y_left_right=np.column_stack((y,x-10,x+10)).tolist())


@pytest.mark.parametrize('center,angle', [(288,3),(352,-3),(320,0),(320,85),(320,-85)])
def test_sign_and_reference_examples_without_input_mutation(center,angle):
    result=measured(center,math.radians(angle))
    if abs(angle)>80:
        result['points_2d_y_left_right']=[p for p in result['points_2d_y_left_right'] if 336<=p[0]<=384]
    before=copy.deepcopy(result)
    fields,reason=tracking_error(result,640,480,ErrorReference())
    assert reason is None and fields['valid']
    assert fields['lateral_error_norm']==pytest.approx((center-320)/320,abs=1e-12)
    assert fields['heading_error_rad']==pytest.approx(math.radians(angle),abs=1e-12)
    assert fields['confidence']==.95 and result==before


def test_quadratic_tangent_and_sloped_reference():
    reference=ErrorReference(reference_x=300,reference_dx_dy=math.tan(math.radians(1)))
    fields,reason=tracking_error(measured(curvature=.002),640,480,reference)
    assert reason is None
    assert fields['lateral_error_norm']==pytest.approx(-12/320)
    assert fields['heading_error_rad']==pytest.approx(math.radians(2))


@pytest.mark.parametrize('change', [
    {'state':'LOST'}, {'state':'PARTIAL_2D'}, {'state':'PREDICTED','predicted_only':True},
    {'state':'CONFLICT_2D'}, {'valid_2d':False}, {'visual':{'valid':False}},
    {'confidence':float('nan')}, {'confidence':float('inf')}, {'confidence':1.1},
    {'confidence':0.}, {'points_2d_y_left_right':[]}, {'coordinate_2d':'model_pixels'},
])
def test_invalid_states_never_carry_previous_error(change):
    result=measured();result.update(change)
    fields,reason=tracking_error(result,640,480,ErrorReference())
    assert fields==invalid_error() and reason


@pytest.mark.parametrize('kind', ['nan','cross','out_of_bounds','unsorted','short','noise'])
def test_bad_or_unsupported_centerline(kind):
    result=measured();points=np.asarray(result['points_2d_y_left_right'])
    if kind=='nan':points[1,1]=np.nan
    elif kind=='cross':points[1,1]=points[1,2]+1
    elif kind=='out_of_bounds':points[1,2]=640
    elif kind=='unsorted':points=points[::-1]
    elif kind=='short':points=points[:5]
    elif kind=='noise':points[points[:,0]==360,1:]+=30
    result['points_2d_y_left_right']=points.tolist()
    fields,reason=tracking_error(result,640,480,ErrorReference())
    assert fields==invalid_error() and reason


def test_dimension_change_and_empty_reference_are_not_silently_scaled():
    assert tracking_error(measured(),1280,720,ErrorReference())[0]==invalid_error()
    with pytest.raises(ValueError):ErrorReference(reference_y=500)
    with pytest.raises(ValueError):ErrorReference(heading_y_min=360)
    with pytest.raises(ValueError):ErrorReference(min_local_rows=5)


@pytest.mark.parametrize('center_only',[False,True])
@pytest.mark.parametrize('ys',[
    list(range(168,379,3)),  # Live failure: supported to y=378, not y=384.
    [336,339,342,345,348,351],  # Exactly six, all above reference row.
    [366,369,372,375,378,381],  # Exactly six, all below reference row.
    [336,339,342,369,372,375],  # A gap no longer rejects a successful fit.
    [345,348,351,354,357,360],  # Neither endpoint of the configured region.
])
def test_six_local_points_accept_partial_support_and_keep_reference(center_only,ys):
    result=measured(curvature=.002)
    points=[p for p in result['points_2d_y_left_right'] if p[0] in ys]
    if center_only:
        result.update(state='CENTER_2D',center_2d_valid=True,points_2d_y_left_right=[],
                      points_2d_y_center=[[y,(left+right)/2] for y,left,right in points])
    else: result['points_2d_y_left_right']=points
    before=copy.deepcopy(result)
    fields,reason=tracking_error(result,640,480,ErrorReference())
    assert reason is None and fields['valid']
    assert fields['lateral_error_norm']==pytest.approx(-.1,abs=1e-10)
    assert fields['heading_error_rad']==pytest.approx(math.radians(3),abs=1e-10)
    assert result==before


@pytest.mark.parametrize('local_count',[0,1,5])
def test_points_outside_direction_region_do_not_count(local_count):
    result=measured()
    result['points_2d_y_left_right']=[p for p in result['points_2d_y_left_right']
        if p[0]<336 or 336<=p[0]<336+3*local_count]
    fields,reason=tracking_error(result,640,480,ErrorReference())
    assert fields==invalid_error() and reason=='insufficient_local_rows'
