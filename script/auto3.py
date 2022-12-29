#!/usr/bin/env python3

import numpy as np
import roslib
import rospy
import tf
import tf2_ros
import os
import sys
import subprocess
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Transform
from rovi_utils import tflib

Param={
  "pos0":[800,0,900,0,180,180],
  "pos1":[
      [800,120,900,0,180,180],
      [800,240,900,0,180,180],
      [800,360,900,0,180,180],
      [800,400,900,-10,180,180]],
  "pos2":[
      [800,-120,900,0,180,180],
      [800,-240,900,0,180,180],
      [800,-360,900,0,180,180],
      [800,-400,900,10,180,180]],
  "zshift":[0,80,160,240],
  "yoffset":[0,60,-60],
  "zoffset":[0,-30,-30],
}
Config={
}

mode=0   #0:center, 1:scan+, 2:scan-
yindex=0
zindex=0
x1cema=0
retry=0
merge=0
error=0
lot=0
stats={}

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    rospy.loginfo("getRT::TF lookup success "+base+"->"+ref)
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    RT=None
  return RT

def X0(t):
  global merge
  merge=0
  rospy.Timer(rospy.Duration(t),lambda ev: pub_clear.publish(mTrue),oneshot=True)  #X0

def X1(t):
  print("pub_capt",mode)
  rospy.Timer(rospy.Duration(t),lambda ev: pub_capt.publish(mTrue),oneshot=True)  #X1

def X2(t):
  rospy.Timer(rospy.Duration(t),lambda ev: pub_solve.publish(mTrue),oneshot=True)  #X2

def mov(Tc):
  print("move",Tc[1,3],Tc[2,3])
  rospy.set_param("/rsim/org_y",float(Tc[1,3]))
  rospy.set_param("/rsim/org_z",float(Tc[2,3]))
  rot=R.from_matrix(Tc[:3,:3]).as_euler('ZXY',degrees=True)
  rospy.set_param("/rsim/org_rz",float(rot[0]))
  rospy.set_param("/rsim/org_rx",float(rot[1]))
  pub_movo.publish(mTrue)

def captMov():
  global bTc,merge
  bTc=np.eye(4)
  if mode==0:
    pos=Param["pos0"]
  elif mode==1:
    pos=Param["pos1"][yindex]
  elif mode==2:
    pos=Param["pos2"][yindex]
  bTc[0,3]=pos[0]
  bTc[1,3]=pos[1]+Param["yoffset"][merge]
  bTc[2,3]=pos[2]-Param["zshift"][zindex]+Param["zoffset"][merge]
  bTc[:3,:3]=R.from_euler('X',pos[3],degrees=True).as_matrix()
  mov(bTc)

def cb_start(msg):
  global bTc,lot,mode,yindex,zindex
  mode=0   #0 center
  yindex=0
  zindex=0
  merge=0
  if lot>0: pub_place.publish(mTrue)    #place a new bucket
  lot=lot+1
  print("auto lot",lot)
  captMov()
  X0(1)
  X1(2)

def cb_cont(msg):
  global mode,yindex,zindex,merge
  print("cb_cont",mode,yindex)
  merge=0
  if mode==0: #=>mode1
    mode=1
    yindex=0
  else:
    ylen=len(Param["pos1"]) if mode==1 else len(Param["pos2"])
    yindex=yindex+1
    if yindex>=ylen:
      if mode==1: #=>mode2
        mode=2
        yindex=0
      else: #=>mode0
        mode=0
        zindex=zindex+1
        if zindex>=len(Param["zshift"]):
          print("Finished, reload next stack")
          rospy.Timer(rospy.Duration(5),cb_start,oneshot=True)
          return
  captMov()
  X0(1)
  X1(2)

def cb_capture(msg):
  global mode,retry,merge,yindex
  print("cb_capture",msg.data,stats["probables"])
  if(msg.data):
    if stats["probables"]>0:
      merge=merge+1
      retry=0
      X2(2)
    else:
      rospy.Timer(rospy.Duration(3),lambda ev: cb_cont(mTrue),oneshot=True)
  else:
    print("capture failed")

def cb_solve(msg):
  global mode,retry,merge,yindex,zindex
  print("cb_solve",mode,stats)
  if msg.data:
    if stats["prob_m"]<150:
      print("auto pick2")
      pub_pick2.publish(mTrue)  #VT move
    else:
      print("auto pick1")
      pub_pick1.publish(mTrue)  #VT move
    X0(1)
    X1(2)
  elif retry<3:
    retry=retry+1
    X2(0.1)
  elif merge<len(Param["yoffset"]):
    captMov()
    X1(1)
  else: #exit: terminate
    err=Int32()
    err.data=10000+mode
    pub_error.publish(err)


def cb_error(msg):
  global error
  error=msg.data

def cb_report(msg):
  global stats
  d=eval(msg.data)
  stats.update(d)
  print("cb_report",stats)

########################################################
rospy.init_node("autotest",anonymous=True)
thispath=subprocess.getoutput("rospack find rovi_sim")
###Load params
try:
  Config.update(rospy.get_param("/config/auto"))
except Exception as e:
  print("get_param exception:",e.args)
###Topics
rospy.Subscriber("/response/capture",Bool,cb_capture)
rospy.Subscriber("/response/solve",Bool,cb_solve)
rospy.Subscriber("/rsim/error",Int32,cb_error)
rospy.Subscriber("/report",String,cb_report)
pub_clear=rospy.Publisher("/request/clear",Bool,queue_size=1)
pub_capt=rospy.Publisher("/request/capture",Bool,queue_size=1)
pub_solve=rospy.Publisher("/request/solve",Bool,queue_size=1)
pub_movo=rospy.Publisher("/rsim/mov_o",Bool,queue_size=1)
pub_jogy=rospy.Publisher("/rsim/jog_y",Bool,queue_size=1)
pub_pick1=rospy.Publisher("/rsim/pick1",Bool,queue_size=1)
pub_pick2=rospy.Publisher("/rsim/pick2",Bool,queue_size=1)
pub_place=rospy.Publisher("/rsim/place",Bool,queue_size=1)
pub_error=rospy.Publisher("/rsim/error",Int32,queue_size=1)
###Bool message
mTrue=Bool();mTrue.data=True
mFalse=Bool();mFalse.data=False
###TF
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
broadcaster=tf2_ros.StaticTransformBroadcaster()

#if __name__=="__main__":

bTc=np.eye(4)
rospy.Timer(rospy.Duration(1),cb_start,oneshot=True)

while not rospy.is_shutdown():
  if error!=0: sys.exit(0)
  rospy.sleep(0.1)

