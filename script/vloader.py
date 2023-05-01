#!/usr/bin/env python3

import numpy as np
import roslib
import rospy
import open3d as o3d
import os
import sys
import subprocess
import copy
import yaml
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import Transform
from rovi_utils import tflib
from rtk_tools import dictlib

Config={
  "folder":"/home/ros/.ros/YOODS/LOG",
  "tf":["flange"]
}
Param={
  "folder":"",
  "index":0,
  "startswith":"2023"
}

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def getFolder():
  try:
    Param.update(rospy.get_param("/vloader"))
  except Exception as e:
    print("get_param exception:",e.args)
  datapath=Config["folder"]
  ls=os.listdir(datapath)
  sel=list(filter(lambda f:f.startswith(Param["startswith"]),ls))
  sel.sort()
  print("vloader::Folders",sel)
  n=Param["index"]
  if n>=len(sel):
    print("vloader::index out of folders")
    return None
  return datapath+'/'+sel[n]

def cb_move():
  pth=getFolder()
  if pth is None: return
  tf=rospy.get_param('config_tf')
  try:
    with open(pth+'/rcalib.yaml') as file:
      rcalib= yaml.safe_load(file)
      print("vloader::rcalib",rcalib)
      dictlib.merge(tf,rcalib['config_tf'])
  except:
    print("vloader::No rcalib.yaml")
    return
  for f in Config["tf"]:
    try:
      with open(pth+'/'+f+'.yaml') as file:
        tr=yaml.safe_load(file)
        print("vloader::tf",f,tr)
        d={}
        d[f]={'transform':tr}
        dictlib.merge(tf,d)
    except:
      print("vloader::No flange.yaml")
      return
  rospy.set_param('config_tf',tf)
  rospy.Timer(rospy.Duration(0.1),lambda ev: pb_reload(),oneshot=True)

def cb_capture():
  pth=getFolder()
  if pth is None: return

  pcd=o3d.io.read_point_cloud(pth+'/test.ply')
  rospy.Timer(rospy.Duration(0.5),lambda ev: pub_ps.publish(np2F(np.array(pcd.points))),oneshot=True)
  rospy.Timer(rospy.Duration(1),lambda ev: pub_done.publish(mTrue),oneshot=True)
  Param["index"]=Param["index"]+1
  rospy.Timer(rospy.Duration(5),lambda ev: cb_move(),oneshot=True)
  rospy.set_param('/vloader/index',Param["index"])
  rospy.set_param('/vloader/folder',pth.split('/')[-1])

def pb_reload():
  pub_reload.publish(mTrue)

def pb_pcount(n):
  msg=Int32()
  msg.data=n
  pub_pcount.publish(msg)

########################################################
rospy.init_node("vloader",anonymous=True)
###Load params
try:
  Config.update(rospy.get_param("/config/vloader"))
except Exception as e:
  print("get_param exception:",e.args)
###Topics
rospy.Subscriber("/rovi/X1",Bool,lambda f:cb_capture())
rospy.Subscriber("/request/clear",Bool,lambda f:cb_move())
pub_reload=rospy.Publisher("/reload/config_tf",Bool,queue_size=1)
pub_ps=rospy.Publisher("/rovi/ps_floats",numpy_msg(Floats),queue_size=1)
pub_done=rospy.Publisher("/rovi/Y1",Bool,queue_size=1)
pub_pcount=rospy.Publisher("/rovi/pcount",Int32,queue_size=1)
###Globals
mTrue=Bool();mTrue.data=True
mFalse=Bool()

rospy.Timer(rospy.Duration(1),lambda e:pb_pcount(0),oneshot=True)
rospy.Timer(rospy.Duration(1),lambda e:rospy.set_param('/vloader/index',0),oneshot=True)
rospy.Timer(rospy.Duration(1),lambda e:rospy.set_param('/vloader/folder',''),oneshot=True)

#if __name__=="__main__":
#

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
