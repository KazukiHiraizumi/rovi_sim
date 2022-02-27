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
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from rovi_utils import tflib

Param={
  "org":[0,0,500],
  "step_y":150,
  "step_z":50
}
Config={
  "source_frame_id":"base",
  "target_frame_id":"tool0_controller",
}

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    rospy.loginfo("cropper::getRT::TF lookup success "+base+"->"+ref)
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    RT=None
  return RT

def mov(pos,relative=False):
  pos.extend([0,0,0])
  rot=R.from_euler('xyz',pos[3:6],degrees=True)
  Rt=np.eye(4)
  Rt[:3,:3]=rot.as_matrix()
  Rt[:3,3]=np.array(pos[:3]).T
  if relative:
    Ro=getRT(Config["source_frame_id"],Config["target_frame_id"])
    Rt=Rt.dot(Ro)
  tf=TransformStamped()
  tf.header.stamp=rospy.Time.now()
  tf.header.frame_id=Config["source_frame_id"]
  tf.child_frame_id=Config["target_frame_id"]
  tf.transform=tflib.fromRT(Rt)
  pub_tf.publish(tf);
  
def cb_jogy(msg):
  step=Param["step_y"]
  if not msg.data: step=-step
  mov([0,step,0],relative=True)

def cb_jogz(msg):
  pass

def cb_org(msg):
  try:
    Param.update(rospy.get_param("~param"))
  except Exception as e:
    print("get_param exception:",e.args)
  p=Param
  mov([p["org_x"],p["org_y"],p["org_z"],p["org_rx"],p["org_ry"],p["org_rz"]])
########################################################
rospy.init_node("vrobo",anonymous=True)
thispath=subprocess.getoutput("rospack find rovi_sim")
###Load params
try:
  Config.update(rospy.get_param("/config/vrobo"))
except Exception as e:
  print("get_param exception:",e.args)
###Topics
rospy.Subscriber("/rsim/jog_y",Bool,cb_jogy)
rospy.Subscriber("/rsim/jog_z",Bool,cb_jogz)
rospy.Subscriber("/rsim/mov_o",Bool,cb_org)
pub_tf=rospy.Publisher("/update/config_tf",TransformStamped,queue_size=1);
###TF
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)

#if __name__=="__main__":
#

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
