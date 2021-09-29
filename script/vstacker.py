#!/usr/bin/env python3

import numpy as np
import os
import sys
import subprocess
import copy
import open3d as o3d
import roslib
import rospy
import tf
import tf2_ros
from scipy.spatial.transform import Rotation as rot
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from geometry_msgs.msg import Transform
from rovi_utils import tflib

Config={
  "model":"model.ply",
  "layers":3,
  "dmin":100,
  "range_x":50,
  "range_y":1000,
  "solve_frame_id":"camera/capture0/solve0",
  "master_frame_id":"camera/master0",
  "bucket_frame_id":"user1",
  "precision":5,
  "env_tr":[0,0,-190, 0,0,.707,.707]
}

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    rospy.loginfo("getRT::TF lookup success "+base+"->"+ref)
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    RT=None
  return RT

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

def toRT(cod):
  tr=np.eye(4)
  tr[:,3]=np.append(cod,[1]).T
  R=rot.from_rotvec(np.array([np.pi*np.random.rand()*2,0,0]))
  tr[:3,:3]=R.as_matrix()
  return np.asmatrix(tr)

def place():
  layout=[]
  for n in range(Config["layers"]):
    l=alayer(layout,Config["dmin"]*n)
    if l is None: return None
    xs=(np.random.rand(len(l),1)-0.5)*Config["range_x"]
    l=np.array(list(map(toRT,np.hstack((xs,l)))))
    layout.append(l)
  return layout

def mkscene(pcd,stack):
  scn=np.asarray([]).reshape((-1,3))
  for layer in stack:
    for tr,xo,wp in zip(layer["tf"],layer["xo"],layer["wp"]):
      if not xo: continue
      p=copy.deepcopy(pcd[wp])
      p.transform(tr)
      pn=np.array(p.points)
      scn=np.vstack((scn,pn))
  pn=np.array(Pcd[-1].points)  #environ points
  print("Env",len(pn),np.max(pn.T[1]),np.min(pn.T[1]))
  scn=np.vstack((scn,pn))
  return scn

def cb_redraw(msg):
  pub_wp.publish(np2F(Scene))

def mkstack(tfs):
  res=[]
  for tf in tfs:
    xo=np.ones(len(tf),dtype=bool)
    wp=np.zeros(len(tf),dtype=int)
    res.append({"tf":tf,"xo":xo,"wp":wp})
  return res

def cb_place(msg):
  global Scene,Stack
  if mError.data>0:
    pub_err.publish(mError)
    return
  if len(Stack)>0:
    mError.data=9001
    pub_err.publish(mError)
    return
  while True:
    tfs=place()
    if tfs is not None: break
  print("stack Tfs",tfs)
  Stack=mkstack(tfs)
  txo=np.random.rand(len(Stack[-1]["xo"]))<np.random.rand()
  txo[0]=True
  txo[-1]=True
  Stack[-1]["xo"]=txo
  Scene=mkscene(Pcd,Stack)
  cb_redraw(0)

def cb_place1(msg):
  global Scene,Stack
  Scene=mkscene(Pcd,[{"tf":[np.eye(4)],"xo":[True],"wp":[0]}])
  Stack=[]
  cb_redraw(0)
  cb_clear(0)

def chkdist(Ts):
  global mError
  mTs=getRT(Config["master_frame_id"],Config["solve_frame_id"])
  bTm=getRT("world",Config["master_frame_id"])
  bTx=bTm.dot(mTs).dot(bTm.I)
  d=np.linalg.norm(Ts[:,:3,3].T-bTx[:3,3],axis=0)
  print("dist",d)
  nd=np.argmin(d)
  if d[nd]<Config["precision"] and Stack[-1]["xo"][nd]:
    return nd
  else:
    mError.data=9000
    pub_err.publish(mError)
    print("dist err",d[nd])
    return -1

def cb_pick1(msg):
  global Scene,Stack,mError
  if mError.data>0:
    pub_err.publish(mError)
    return
  tos=Stack[-1]
  n=chkdist(tos["tf"])
  print("pick1",n,len(tos["tf"]))
  if n<0:
    mError.data=9010
    pub_err.publish(mError)
    return
  if n==0 or n>=len(tos["tf"])-1:
    if abs(tos["tf"][n][1,3])>Config["range_y"]/2-Config["dmin"]:
      mError.data=9011
      pub_err.publish(mError)
      return
  tos["xo"][n]=False
  if not any(tos["xo"]): Stack.pop(-1)
  Scene=mkscene(Pcd,Stack)
  cb_redraw(0)

def cb_pick2(msg):
  global Scene,Stack,mError
  if mError.data>0:
    pub_err.publish(mError)
    return
  tos=Stack[-1]
  n=chkdist(tos["tf"])
  print("pick2",n,len(tos["tf"]))
  if n<0:
    mError.data=9020
    pub_err.publish(mError)
    return
  if n<int(len(tos["xo"]/3)):
    if any(tos["xo"][n+1:n+3]):
      mError.data=9021
      pub_err.publish(mError)
      return
  elif n>int(len(tos["xo"]*2/3)):
    if any(tos["xo"][n-3:n-1]):
      mError.data=9022
      pub_err.publish(mError)
      return
  else:
    mError.data=9023
    pub_err.publish(mError)
    return
  tos["xo"][n]=False
  if not any(tos["xo"]): Stack.pop(-1)
  Scene=mkscene(Pcd,Stack)
  cb_redraw(0)

def cb_pick3(msg):
  pass
def cb_clear(msg):
  global mError
  mError.data=0
  pub_err.publish(mError)

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
rospy.Subscriber("/rsim/pick1",Bool,cb_pick1)
rospy.Subscriber("/rsim/pick2",Bool,cb_pick2)
rospy.Subscriber("/rsim/pick3",Bool,cb_pick3)
rospy.Subscriber("/rsim/clear",Bool,cb_clear)
rospy.Subscriber("/request/redraw",Bool,cb_redraw)
pub_wp=rospy.Publisher("/rovi/wp_floats",numpy_msg(Floats),queue_size=1)
pub_err=rospy.Publisher("/rsim/error",Int32,queue_size=1)
###TF
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
###Global
mError=Int32()
Scene=[]  #will be point cloud ndarray as [[x,y,z]...]
Stack=[]  #will be as [{"tf":[...],"xo":[...],"wp":[...]}...]
Pcd=[]
Pcd.append(o3d.io.read_point_cloud(thispath+'/'+Config['model']))
Pcd.append(o3d.io.read_point_cloud(thispath+'/'+Config['environ']))
#Pcd.append(o3d.io.read_point_cloud(thispath+'/'+Config['diff']))
if len(Pcd)>1:
  Pcd[-1].transform(tflib.toRTfromVec(Config["env_tr"]))
#if __name__=="__main__":
#

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
