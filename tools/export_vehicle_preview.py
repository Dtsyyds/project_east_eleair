#!/usr/bin/env python3
"""Export bounded CAD mesh geometry and an explicitly estimated display transform.

No device access, TF publication, URDF modification or calibration-file writes.
Run after sourcing deploy/env.sh, with a saved stationary IMU observation file.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def transform(origin):
    xyz=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
    r,p,y=np.fromstring(origin.get('rpy','0 0 0'),sep=' ')
    cr,sr,cp,sp,cy,sy=np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(y),np.sin(y)
    matrix=np.eye(4)
    matrix[:3,:3]=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                   [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
    matrix[:3,3]=xyz
    return matrix


def stl_vertices(path):
    data=path.read_bytes();count=int.from_bytes(data[80:84],'little')
    assert len(data)==84+count*50, f'Expected binary STL: {path}'
    dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')])
    return np.frombuffer(data,dtype,offset=84,count=count)['vertices'].reshape(-1,3).astype(float)


def simplify(vertices,limit):
    # Vertex clustering retains a coherent surface, unlike dropping triangles.
    cell=.002
    while True:
        _,inverse=np.unique(np.rint(vertices/cell).astype(np.int64),axis=0,return_inverse=True)
        counts=np.bincount(inverse);v=np.zeros((len(counts),3));np.add.at(v,inverse,vertices);v/=counts[:,None]
        faces=inverse.reshape(-1,3)
        faces=faces[(faces[:,0]!=faces[:,1])&(faces[:,0]!=faces[:,2])&(faces[:,1]!=faces[:,2])]
        _,unique=np.unique(np.sort(faces,axis=1),axis=0,return_index=True);faces=faces[unique]
        if len(faces)<=limit:
            used,remap=np.unique(faces,return_inverse=True)
            return np.round(v[used],5),remap.reshape(-1,3),cell
        cell*=1.3


def export(description,observations):
    import xacro
    document=xacro.process_file(str(description/'urdf/duct_robot.urdf.xacro'),
                                mappings={'drive_mode':'tracked_2wd','use_sim':'false'}).toxml()
    tree=ET.fromstring(document);transforms={'base_link':np.eye(4)}
    for _ in range(len(tree.findall('joint'))):
        for joint in tree.findall('joint'):
            parent=joint.find('parent').get('link');child=joint.find('child').get('link')
            if parent in transforms and child not in transforms:
                transforms[child]=transforms[parent]@transform(joint.find('origin').attrib)
    parts=[];sources={}
    def add(name,path,matrix,scale,color,limit):
        raw=stl_vertices(path)*scale
        vertices=raw@matrix[:3,:3].T+matrix[:3,3]
        v,f,cell=simplify(vertices,limit)
        parts.append({'name':name,'vertices':v.tolist(),'faces':f.tolist(),'color':color,
                      'cluster_cell_m':cell,'original_triangles':len(raw)//3})
        sources[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
        return vertices
    for link in tree.findall('link'):
        name=link.get('name')
        if name not in transforms:continue
        for visual in link.findall('visual'):
            mesh=visual.find('geometry/mesh')
            if mesh is None:continue
            matrix=transforms[name]@transform(visual.find('origin').attrib)
            add(name,description/'meshes'/Path(mesh.get('filename')).name,matrix,
                np.fromstring(mesh.get('scale','1 1 1'),sep=' '),[60,145,220] if name=='base_link' else [80,90,106],
                3200 if name=='base_link' else 450)
            if name=='base_link':cad_matrix=matrix
    lidar=add('lidar',description/'meshes/lidar.stl',cad_matrix,.001,[225,230,237],450)
    add('gripper',description/'meshes/gripper_simplified.stl',cad_matrix,.001,[215,171,74],700)
    translation=(lidar.min(0)+lidar.max(0))/2
    obs=json.loads(observations.read_text());a=np.array(obs['imu']['accel_median']);up=a/np.linalg.norm(a)
    assert 9<np.linalg.norm(a)<11 and max(obs['imu']['accel_std'])<.2 and np.linalg.norm(obs['imu']['gyro_median'])<.03,'IMU must be approximately stationary'
    target=np.array([0.,0.,1.]);v=np.cross(up,target);c=np.dot(up,target)
    assert c>-.99
    K=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    R=np.eye(3)+K+K@K/(1+c)
    assert np.allclose(R@up,target) and np.allclose(R@R.T,np.eye(3))
    return {'schema_version':1,'estimated':True,'source_frame':'livox_frame','display_frame':'vehicle_preview',
            'translation_m':np.round(translation,6).tolist(),'rotation':np.round(R,9).tolist(),
            'tilt_degrees':float(np.degrees(np.arccos(c))),'stationary_acceleration_m_s2':a.tolist(),
            'assumptions':['LiDAR STL and chassis STL share original CAD coordinates.',
                           'LiDAR measurement origin approximated by its mesh bounding-box center.',
                           'Display aligns stationary accelerometer up to +Z; vehicle is drawn horizontal.',
                           'Yaw and real vehicle-to-LiDAR orientation are not measured. No TF/ROS data changed.'],
            'parts':parts,'source_sha256':sources,'urdf_sha256':hashlib.sha256(document.encode()).hexdigest()}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--description-root',required=True,type=Path)
    p.add_argument('--observations',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path)
    a=p.parse_args();model=export(a.description_root,a.observations)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text('/* CAD-derived display estimate; not calibrated extrinsics. */\n"use strict";\nwindow.VehiclePreviewModel=Object.freeze('+json.dumps(model,separators=(',',':'))+');\n')
    print(json.dumps({'parts':len(model['parts']),'triangles':sum(len(x['faces']) for x in model['parts']),
                      'translation_m':model['translation_m'],'tilt_degrees':model['tilt_degrees'],'bytes':a.output.stat().st_size},indent=2))


if __name__=='__main__':main()
