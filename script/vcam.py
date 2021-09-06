#!/usr/bin/env python3

import numpy as np
import roslib
import rospy
import tf
import tf2_ros
import open3d as o3d
import os
import sys
import subprocess
import copy
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from geometry_msgs.msg import Transform
from rovi_utils import tflib

Config={
  "source_frame_id":"world",
  "target_frame_id":"camera",
  "trim_x":800,
  "trim_y":300,
  "trim_far":600,
  "trim_near":300,
  "view":[[-200,0,0],[200,0,0]],
  "view_r":50000,
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

def cb_ps(msg):
  global Scene
  Scene=np.reshape(msg.data,(-1,3))
  print("vcam sub scene",Scene.shape)
  return

def cb_capture(msg):
  try:
    Config.update(rospy.get_param("/config/vcam"))
  except Exception as e:
    print("get_param exception:",e.args)
  RT=getRT(Config["target_frame_id"],Config["source_frame_id"])
  scn_1=np.vstack((Scene.T,np.ones(len(Scene))))
  scn_1=RT.dot(scn_1)
  scn=scn_1[:3].T
  scn=scn[np.abs(np.ravel(scn_1[1]))<Config["trim_y"]/2]
  zp=np.ravel(scn.T[2])
  scn=scn[zp<Config["trim_far"]]
  zp=np.ravel(scn.T[2])
  scn=scn[zp>Config["trim_near"]]
  print("vcam trimmed",scn.shape)
  if len(scn)<5000:
    print("vcam points too few, abort hidden...",len(scn))
    pub_ps.publish(np2F(scn))
    return  
  pcd=o3d.geometry.PointCloud()
  pcd.points=o3d.utility.Vector3dVector(scn)
  pset=set([])
  for v in Config["view"]:
    _, pm=pcd.hidden_point_removal(v,Config["view_r"])
    pset=pset.union(set(pm))
  plst=np.array(list(pset))
  pcd=pcd.select_by_index(plst)
  pub_ps.publish(np2F(np.array(pcd.points)))

########################################################
rospy.init_node("vcam",anonymous=True)
thispath=subprocess.getoutput("rospack find rovi_sim")
###Load params
try:
  Config.update(rospy.get_param("/config/vcam"))
except Exception as e:
  print("get_param exception:",e.args)
###Topics
rospy.Subscriber("/rovi/wp_floats",numpy_msg(Floats),cb_ps)
rospy.Subscriber("/rovi/X1",Bool,cb_capture)
pub_ps=rospy.Publisher("/rovi/ps_floats",numpy_msg(Floats),queue_size=1)
###TF
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)

#if __name__=="__main__":
#

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
