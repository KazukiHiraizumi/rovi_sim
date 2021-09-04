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
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import Transform
from rovi_utils import tflib

Param={
}
Config={
  "source_frame_id":"world",
  "target_frame_id":"camera",
  "solve_frame_id":"camera/capture0"
}

def cb_redraw(msg):
  pub_ps.publish(np2F(Scene))

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    rospy.loginfo("cropper::getRT::TF lookup success "+base+"->"+ref)
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    RT=None
  return RT

def tf_set(pos):
  tf=TransformStamped()
  tf.header.stamp=rospy.Time.now()
  tf.header.frame_id=Config["solve_frame_id"]
  tf.child_frame_id=Config["solve_frame_id"]+"/solve0"
  tf.transform.translation.x=pos[0]
  tf.transform.translation.y=pos[1]
  tf.transform.translation.z=pos[2]
  tf.transform.rotation.x=0
  tf.transform.rotation.y=0
  tf.transform.rotation.z=0
  tf.transform.rotation.w=1
  broadcaster.sendTransform([])

def tf_clear():
  broadcaster.sendTransform([])

def cb_ps(msg):
  global Scene
  Scene=np.reshape(msg.data,(-1,3))
  pub_ps.publish(np2F(Scene))
  return

def cb_solve(msg):
  global Scene
  try:
    Param.update(rospy.get_param("-param"))
  except Exception as e:
    print("get_param exception:",e.args)
  cb_redraw(0)
  rospy.Timer(rospy.Duration(0.1),lambda ev: pub_sol.publish(mTrue),oneshot=True)

########################################################
rospy.init_node("prepro",anonymous=True)
thispath=subprocess.getoutput("rospack find rovi_sim")
###Load params
try:
  Config.update(rospy.get_param("/config/prepro"))
except Exception as e:
  print("get_param exception:",e.args)
###Topics
rospy.Subscriber("~in/floats",numpy_msg(Floats),cb_ps)
rospy.Subscriber("/request/solve",Bool,cb_solve)
rospy.Subscriber("/request/redraw",Bool,cb_redraw)
pub_ps=rospy.Publisher("~out/floats",numpy_msg(Floats),queue_size=1)
pub_str=rospy.Publisher("/report",String,queue_size=1)
pub_sol=rospy.Publisher("~solve",Bool,queue_size=1)
###Bool message
mTrue=Bool();mTrue.data=True
mFalse=Bool();mFalse.data=False
###TF
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
broadcaster=tf2_ros.StaticTransformBroadcaster()

#if __name__=="__main__":
#

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
