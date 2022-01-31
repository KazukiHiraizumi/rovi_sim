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
  "pos0":[0,0,800,0,0,0],
  "pos1":[
      [0,120,800,0,0,0],
      [0,160,800,10,0,0],
      [0,280,800,10,0,0],
      [0,300,800,20,0,0],],
  "pos2":[
      [0,-120,800,0,0,0],
      [0,-160,800,-10,0,0],
      [0,-280,800,-10,0,0],
      [0,-300,800,-20,0,0]],
  "zshift":[0,80,160,240],
}
Config={
}

mode=0   #0:center, 1:scan+, 2:scan-
nmode=0
yindex=0
zindex=0
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

def mov(Tc):
  print("move",Tc[1,3],Tc[2,3])
  rospy.set_param("/rsim/org_y",float(Tc[1,3]))
  rospy.set_param("/rsim/org_z",float(Tc[2,3]))
  rot=R.from_matrix(Tc[:3,:3]).as_euler('ZXY',degrees=True)
  rospy.set_param("/rsim/org_rz",float(rot[0]))
  rospy.set_param("/rsim/org_rx",float(rot[1]))
  pub_movo.publish(mTrue)

def captMov():
  global bTc
  bTc=np.eye(4)
  if mode==0:
    pos=Param["pos0"]
  elif mode==1:
    pos=Param["pos1"][yindex]
  elif mode==2:
    pos=Param["pos2"][yindex]
  bTc[0,3]=pos[0]
  bTc[1,3]=pos[1]
  bTc[2,3]=pos[2]-Param["zshift"][zindex] #-merge*40
  bTc[:3,:3]=R.from_euler('X',pos[3],degrees=True).as_matrix()
  mov(bTc)

def cb_start(msg):
  global bTc,lot,mode,yindex,zindex,retry
  mode=0   #0 center, 1:scan mid,2:scan edge
  yindex=0
  zindex=0
  retry=0
  merge=0
  nmode=0
  entry0()
  if lot>0: pub_place.publish(mTrue)    #place a new bucket
  lot=lot+1
  print("auto lot",lot)

def cb_clear(msg):
  global merge,retry
  print("cb_clear")
  merge=0
  retry=0

def cb_capture(msg):
  global merge
  if(msg.data):
    merge=merge+1
    if merge<3:
      rospy.Timer(rospy.Duration(0.1),lambda ev: pub_solve.publish(mTrue),oneshot=True)  #X2
    else: #exit: terminate
      err=Int32()
      err.data=10000
      pub_error.publish(err)
  else:
    print("capture failed")

def entry0():
  global retry,merge,mode,nmode,yindex,zindex
  zindex=0 if mode==0 else zindex+1
  if zindex<len(Param["zshift"]):
    mode=0
    nmode=1
    rospy.Timer(rospy.Duration(1),lambda ev: pub_clear.publish(mTrue),oneshot=True)  #X0
    rospy.Timer(rospy.Duration(5),lambda ev: pub_capt.publish(mTrue),oneshot=True)  #X1
    captMov()
  else:
    print("Finished, reload next stack")
    rospy.Timer(rospy.Duration(5),cb_start,oneshot=True)

def entry12():
  global retry,merge,mode,nmode,yindex,zindex
  rospy.Timer(rospy.Duration(5),lambda ev: pub_capt.publish(mTrue),oneshot=True)  #X1
  yindex=0
  nmode=nmode+1

def entry1():
  global mode
  entry12()
  mode=1
  captMov()

def entry2():
  global mode
  entry12()
  mode=2
  captMov()

def prog0(f):
  global retry,merge,mode,nmode,yindex,zindex
  if stats["volume"][1]!=0:  #exit:points few
    print("Mode0 points too few")
    rospy.Timer(rospy.Duration(1),lambda ev: pub_clear.publish(mTrue),oneshot=True)  #X0
    entry1()
  elif(f):
    if stats["margin"][1]!=0:
      print("auto pick2")
      pub_pick2.publish(mTrue)  #VT move
    else:
      print("auto pick1")
      pub_pick1.publish(mTrue)  #VT move
    rospy.Timer(rospy.Duration(1),lambda ev: pub_clear.publish(mTrue),oneshot=True)  #X0
    rospy.Timer(rospy.Duration(5),lambda ev: pub_capt.publish(mTrue),oneshot=True)  #X1
  elif retry<3:
    retry=retry+1
    rospy.Timer(rospy.Duration(0.1),lambda ev: pub_solve.publish(mTrue),oneshot=True) #X2
  else:  #exit:solve error
    if stats["margin"][0]>0:
      entry1()
    else:
      entry2()

def prog12(f):
  global retry,merge,mode,nmode,yindex,zindex
  ylen=len(Param["pos1"]) if mode==1 else len(Param["pos2"])
  if stats["volume"][1]!=0:  #points few
    print("Mode12 points too few")
    rospy.Timer(rospy.Duration(1),lambda ev: pub_clear.publish(mTrue),oneshot=True)  #X0
    yindex=yindex+1
    if yindex<ylen:
      captMov()
      rospy.Timer(rospy.Duration(5),lambda ev: pub_capt.publish(mTrue),oneshot=True)  #X1
    else: #exit: bucket end
      if nmode==2:
        if mode==1: entry2()
        else: entry1()
      else: entry0()
  elif(f):
    if stats["margin"][1]!=0:
      print("auto pick2")
      pub_pick2.publish(mTrue)  #VT move
    else:
      print("auto pick1")
      pub_pick1.publish(mTrue)  #VT move
    rospy.Timer(rospy.Duration(1),lambda ev: pub_clear.publish(mTrue),oneshot=True)  #X0
    rospy.Timer(rospy.Duration(5),lambda ev: pub_capt.publish(mTrue),oneshot=True)  #X1
  elif retry<3:
    retry=retry+1
    rospy.Timer(rospy.Duration(0.1),lambda ev: pub_solve.publish(mTrue),oneshot=True) #X2
  else:
    yindex=yindex+1
    if yindex<ylen:
      retry=0
      captMov()
      rospy.Timer(rospy.Duration(5),lambda ev: pub_capt.publish(mTrue),oneshot=True)  #X1
    else: #exit: terminate
      err=Int32()
      err.data=10000+mode
      pub_error.publish(err)
#      if nmode==2:
#        if mode==1: entry2()
#        else: entry1()
#      else: entry0()

def cb_solve(msg):
  global bTc,retry,mode,yindex,zindex
  if mode==0: prog0(msg.data)
  elif mode==1: prog12(msg.data)
  elif mode==2: prog12(msg.data)

def cb_error(msg):
  global error
  error=msg.data

def cb_report(msg):
  global stats
  d=eval(msg.data)
  stats.update(d)


########################################################
rospy.init_node("autotest",anonymous=True)
thispath=subprocess.getoutput("rospack find rovi_sim")
###Load params
try:
  Config.update(rospy.get_param("/config/auto"))
except Exception as e:
  print("get_param exception:",e.args)
###Topics
rospy.Subscriber("/request/clear",Bool,cb_clear) #In place of "/response/clear" because no one responses to it
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

