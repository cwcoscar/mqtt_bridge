#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from gps_common.msg import GPSFix
# from mqtt_bridge.msg import Lilee_DO
# from mqtt_bridge.msg import Lilee_DO_array
from autoware_msgs.msg import DetectedObject
from autoware_msgs.msg import DetectedObjectArray
import time


def callback_gps(data):
    output = DetectedObjectArray()

    value = DetectedObject()

    for i in data.objects:
        value = i
        value.header.frame_id = "lilee_RSU"

        nowTime = time.time() # get nowtime
        transmission_time = value.header.stamp.secs*1000 + value.header.stamp.nsecs/1000000 #millisec
        value.latency = nowTime*1000 - transmission_time
        value.source = "lilee_RSU"
        output.objects.append(value)

    output.header = data.header
    output.header.frame_id = "lilee_RSU"

    pub.publish(output)


rospy.init_node('transfer_Lilee2autoware_DO')
sub_gps = rospy.Subscriber('/lilee_s0_DO', DetectedObjectArray, callback_gps)

pub = rospy.Publisher('/lileeRSU/s0/objects', DetectedObjectArray, queue_size=100)


if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
