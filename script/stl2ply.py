#!/usr/bin/env /usr/bin/python3

import open3d as o3d
import numpy as np
import copy
from scipy.spatial.transform import Rotation as rot

name="The_Tomato"
mesh = o3d.io.read_triangle_mesh(name+".stl")
pcd=mesh.sample_points_uniformly(number_of_points=1000000)
pcdd=pcd.voxel_down_sample(voxel_size=1.0)
pn=np.array(pcdd.points)
cen=np.mean(pn,axis=0)
print(cen)
rt=np.eye(4)
rt[:3,3]=cen
R=rot.from_euler('Y',(-60),degrees=True)
rt[:3,:3]=R.as_matrix()
pcdd.transform(np.linalg.inv(rt))
o3d.io.write_point_cloud(name+".ply",pcdd)
