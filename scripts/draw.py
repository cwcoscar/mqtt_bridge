#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from mqtt_bridge.msg import DetectedObjectWst
from jsk_recognition_msgs.msg import BoundingBox,BoundingBoxArray



def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    return

def listener(): 

    rospy.init_node('draw', anonymous=True) 
    rospy.Subscriber('new_array', BoundingBoxArray, callback)
    rospy.spin()
    
 
if __name__ == '__main__':
     listener()

