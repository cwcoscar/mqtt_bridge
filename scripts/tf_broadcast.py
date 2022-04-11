#!/usr/bin/env python  
import roslib
roslib.load_manifest('mqtt_bridge')
import rospy
import tf
import pymap3d
import math
import time
from mqtt_bridge.msg import DetectedObjectWst
from jsk_recognition_msgs.msg import BoundingBox,BoundingBoxArray

def handle_pose(msg):
    br = tf.TransformBroadcaster()

    for k in range(len(msg.boxes)):
        tag='obj'+str(k)
        br.sendTransform((msg.boxes[k].pose.position.x, msg.boxes[k].pose.position.y, msg.boxes[k].pose.position.z),
                     (msg.boxes[k].pose.orientation.x,msg.boxes[k].pose.orientation.y,msg.boxes[k].pose.orientation.z,msg.boxes[k].pose.orientation.w)
                     ,msg.boxes[k].header.stamp,tag,"rsu")
    

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    rospy.Subscriber('new_array',BoundingBoxArray,handle_pose)
    rospy.spin()

