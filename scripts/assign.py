#!/usr/bin/env python

import rospy
import pymap3d
import math
from std_msgs.msg import String
from mqtt_bridge.msg import DetectedObjectWst
from jsk_recognition_msgs.msg import BoundingBox,BoundingBoxArray

##reference position
lat0=22.92310681
lon0=120.28771186
h0=44.59
h=1

pub = rospy.Publisher('new_array', BoundingBoxArray , queue_size=10)

def callback(data):
    bba=BoundingBoxArray()
    bba.header.stamp.secs=data.DO.timestamp//1000
    bba.header.stamp.nsecs=(data.DO.timestamp%1000)*(10**6)
    bba.header.frame_id='rsu'
    for k in range(len(data.DO.obj)):
	bb=BoundingBox()
        #print(len(data.DO.obj))        
        if data.DO.obj[k]: 
            #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.DO.obj[k])
            #print(data.DO.obj[k].do_coordinate)
            #print('----',rospy.get_rostime())
            bb.header.stamp.secs=data.DO.timestamp//1000
            bb.header.stamp.nsecs=(data.DO.timestamp%1000)*(10**6)
            bb.header.frame_id='rsu'
            c=pymap3d.geodetic2ned(data.DO.obj[k].do_coordinate[0], data.DO.obj[k].do_coordinate[1], h,lat0,lon0,h0)
            #print('---',c)
            bb.pose.position.x=c[0]
            bb.pose.position.y=c[1]
            bb.pose.position.z=c[2]
            bb.pose.orientation.x=data.DO.obj[k].do_heading[0]
            bb.pose.orientation.y=data.DO.obj[k].do_heading[1]
            bb.pose.orientation.z=data.DO.obj[k].do_heading[2]
            bb.pose.orientation.w=data.DO.obj[k].do_heading[3]
            bb.dimensions.x=data.DO.obj[k].do_dimension[0]
            bb.dimensions.y=data.DO.obj[k].do_dimension[1]
            bb.dimensions.z=data.DO.obj[k].do_dimension[2]
            bb.value=1
            bb.label=int(data.DO.obj[k].tid)
            #print('imbb',bb,'end')
            bba.boxes.append(bb)                         
        else:
            break  
    pub.publish(bba) 


def listener(): 

    rospy.init_node('processer', anonymous=True) 
    rospy.Subscriber('wsttest', DetectedObjectWst, callback)
    rospy.spin()

 
if __name__ == '__main__':
     listener()



