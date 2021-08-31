#!/usr/bin/env python3

import numpy as np
import os
import sys
import subprocess
import copy
import open3d as o3d
import roslib
import rospy
from scipy.spatial.transform import Rotation as rot
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from geometry_msgs.msg import Transform
from rovi_utils import tflib

Config={
  "model":"test/shaft.ply",
  "layers":3,
  "dmin":100,
  "range_x":50,
  "range_y":1000,
}

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def lay1st(base,offset):
  while True:
    cen0=(np.random.rand(2)-0.5)*Config["dmin"]/2+np.array([-Config["range_y"]/2+Config["dmin"]/2,offset])
    if len(base)==0:
      cen0[1]=0
      return cen0
    else:
      dist=np.linalg.norm(base[-1].T[-1][1:3].T-cen0,axis=1)
      print("dist",dist)
      dmin=np.min(dist)
      if dmin>Config["dmin"] and dmin<Config["dmin"]*1.5:
        print("lay1",cen0)
        return cen0

def alayer(base,offset):
  cens=np.array([]).reshape((-1,2))
  cen0=lay1st(base,offset)
  cens=np.vstack((cens,cen0))
  for n in range(1000):
    if n>=100:
      print("lay failed",offset)
      return None
    while True:
      Dy=np.random.rand()*Config["dmin"]*2
      if len(base)>0:
        Dz=(np.random.rand()-0.5)*Config["dmin"]*1
        cen=np.array([cens[-1,0],np.mean(cens[:,1])])+np.array([Dy,Dz])
        dist=np.linalg.norm(base[-1].T[-1][1:3].T-cen,axis=1)
        dmin=np.min(dist)
        if dmin>Config["dmin"] and dmin<Config["dmin"]*1.5:
          break
      else: #### initial layer ####
        cen=cen0+np.array([Dy,0])
        break
    dmin=np.linalg.norm(cen0-cen)
    if dmin>Config["dmin"] and dmin<Config["dmin"]*1.5:
      if np.abs(cen[0])<Config["range_y"]/2-Config["dmin"]/2:
        cen0=cen
        cens=np.vstack((cens,cen0))
      else:
        return cens

def RT(cod):
  tr=np.eye(4)
  tr[:,3]=np.append(cod,[1]).T
  R=rot.from_rotvec(np.array([np.pi*np.random.rand()*2,0,0]))
  tr[:3,:3]=R.as_matrix()
  return tr

def place():
  layout=[]
  for n in range(Config["layers"]):
    l=alayer(layout,Config["dmin"]*n)
    if l is None: return None
    xs=(np.random.rand(len(l),1)-0.5)*Config["range_x"]
    l=np.array(list(map(RT,np.hstack((xs,l)))))
    layout.append(l)
  return layout

def mkscene(pcd,tfs):
  scn=np.reshape(np.asarray(pcd.points),(-1,3))
  for layer in tfs:
    for tr in layer:
      p=copy.deepcopy(pcd)
      p.transform(tr)
      pn=np.array(p.points)
      scn=np.vstack((scn,pn))
  return scn

def cb_redraw(msg):
  pub_wp.publish(np2F(scenePn))

def cb_place(msg):
  global sceneTf,scenePn
  while True:
    sceneTf=place()
    if sceneTf is not None: break
  print("sceneTf",sceneTf)
  print("PLY file",thispath+'/'+Config['model'])
  pcd=o3d.io.read_point_cloud(thispath+'/'+Config['model'])
  print("PLY",len(pcd.points))
  scenePn=mkscene(pcd,sceneTf)
  cb_redraw(0)

def cb_place1(msg):
  global sceneTf,scenePn
  sceneTf=[[np.eye(4)]]
  print("sceneTf",sceneTf)
  pcd=o3d.io.read_point_cloud(thispath+'/'+Config['model'])
  print("PLY",len(pcd.points))
  scenePn=mkscene(pcd,sceneTf)
  cb_redraw(0)

def parse_argv(argv):
  args={}
  for arg in argv:
    tokens = arg.split(":=")
    if len(tokens) == 2:
      key = tokens[0]
      args[key]=tokens[1]
  return args

########################################################
rospy.init_node("vstacker",anonymous=True)
thispath=subprocess.getoutput("rospack find rovi_sim")
###Load params
try:
  Config.update(rospy.get_param("/config/vstacker"))
except Exception as e:
  print("get_param exception:",e.args)
#try:
#  Param.update(rospy.get_param("~param"))
#except Exception as e:
#  print("get_param exception:",e.args)
###Topics
rospy.Subscriber("/rsim/place",Bool,cb_place)
rospy.Subscriber("/rsim/place1",Bool,cb_place1)
rospy.Subscriber("/request/redraw",Bool,cb_redraw)
pub_wp=rospy.Publisher("/rovi/wp_floats",numpy_msg(Floats),queue_size=1)

#if __name__=="__main__":
#

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
