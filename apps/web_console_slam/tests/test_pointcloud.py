import struct
from types import SimpleNamespace as NS

import pytest

from slam_console.pointcloud import xyz_preview


@pytest.mark.parametrize('big', [False, True])
@pytest.mark.parametrize('datatype,code,size', [(7,'f',4),(8,'d',8)])
def test_endian_float_and_row_padding(big, datatype, code, size):
    endian = '>' if big else '<'
    row = struct.pack(endian+code*6, 1,2,3,float('nan'),2,3)+b'padding!'
    row2 = struct.pack(endian+code*6, 4,5,6,7,8,9)+b'padding!'
    msg = NS(width=2,height=2,point_step=size*3,row_step=size*6+8,is_bigendian=big,data=row+row2,
             fields=[NS(name=n,offset=i*size,datatype=datatype,count=1) for i,n in enumerate('xyz')])
    assert xyz_preview(msg) == [[1,2,3],[4,5,6],[7,8,9]]
    msg.data = msg.data[:-1]
    with pytest.raises(ValueError): xyz_preview(msg)


def test_empty_and_limit():
    assert xyz_preview(NS(width=0,height=1)) == []
    msg = NS(width=10000,height=1,point_step=12,row_step=120000,is_bigendian=False,
             data=struct.pack('<fff',1,2,3)*10000,
             fields=[NS(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate('xyz')])
    assert len(xyz_preview(msg)) == 2000
    msg.fields[2].datatype=2
    with pytest.raises(ValueError): xyz_preview(msg)
