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
  "pos_y":[-240,-120,0,120,240,300,-300],
  "pos_rx":[0,0,0,0,0,10,-10],
  "pos_z":[800,720,640,560],
  "pick2":[False,False,False,False,False,True,True],
  "end_y":320,
  "wd":500
}
Config={
}

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

def cb_start(msg):
  global bTc,lot,locate,retry
  locate={"y":0,"z":0}
  retry={"solve":0,"capture":0}
  bTc=np.eye(4)
  bTc[:3,3]=np.array([0,Param["pos_y"][0],Param["pos_z"][0]]).T
  bTc[:3,:3]=R.from_euler('X',Param["pos_rx"][0],degrees=True).as_matrix()
  rospy.Timer(rospy.Duration(1),lambda ev: mov(bTc),oneshot=True)
  rospy.Timer(rospy.Duration(5),lambda ev: pub_capt.publish(mTrue),oneshot=True)
  if lot>0: pub_place.publish(mTrue)
  lot=lot+1
  print("auto lot",lot)

def cb_capture(msg):
  if(msg.data):
    rospy.Timer(rospy.Duration(0.1),lambda ev: pub_solve.publish(mTrue),oneshot=True)
  else:
    print("capture failed")

def cb_solve(msg):
  global bTc,retry
  if(msg.data):
    if not Param["pick2"][locate["y"]]: pub_pick1.publish(mTrue)
    else: pub_pick2.publish(mTrue)
    rospy.Timer(rospy.Duration(1),lambda ev: pub_clear.publish(mTrue),oneshot=True)
    rospy.Timer(rospy.Duration(5),lambda ev: pub_capt.publish(mTrue),oneshot=True)
  elif stats["volume"][1]!=0:
    print("Volume too few")
    retry["solve"]=0
    locate["y"]=locate["y"]+1
    if locate["y"]<len(Param["pos_y"]):
      bTc[1,3]=Param["pos_y"][locate["y"]]
      bTc[:3,:3]=R.from_euler('X',Param["pos_rx"][locate["y"]],degrees=True).as_matrix()
      mov(bTc)
      rospy.set_param('/prepro/crop_edge',not Param["pick2"][locate["y"]])
      rospy.Timer(rospy.Duration(2),lambda ev: pub_capt.publish(mTrue),oneshot=True)
    else:
      locate["y"]=0
      locate["z"]=locate["z"]+1
      if locate["z"]<len(Param["pos_z"]):
        bTc[1,3]=Param["pos_y"][locate["y"]]
        bTc[2,3]=Param["pos_z"][locate["z"]]
        bTc[:3,:3]=R.from_euler('X',Param["pos_rx"][locate["y"]],degrees=True).as_matrix()
        mov(bTc)
        rospy.set_param('/prepro/crop_edge',Param["pick2"][locate["y"]])
        rospy.Timer(rospy.Duration(2),lambda ev: pub_capt.publish(mTrue),oneshot=True)
      else:
         print("Finished, reload next stack")
         rospy.Timer(rospy.Duration(5),cb_start,oneshot=True)
  elif retry["solve"]<3:
    retry["solve"]=retry["solve"]+1
    rospy.Timer(rospy.Duration(0.1),lambda ev: pub_solve.publish(mTrue),oneshot=True)
  else:
    print("Solve failed")
    err=Int32()
    err.data=9100
    pub_error.publish(err)

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

error=0
lot=0
retry={"solve":0,"capture":0}
locate={"y":0,"z":0}
stats={}
while not rospy.is_shutdown():
  if error!=0: sys.exit(0)
  rospy.sleep(0.1)

