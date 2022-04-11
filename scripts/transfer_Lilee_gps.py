#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from gps_common.msg import GPSFix
from mqtt_bridge.msg import Lilee_gps
from mqtt_bridge.msg import Lilee_gnss
import time


def callback_gps(data):
    output=Lilee_gps()

    value = Lilee_gnss()

    List = []
    List.append(data.latitude)
    List.append(data.longitude)
    List.append(data.altitude)

    value.coord = List

    value.speed = data.speed
    value.heading = data.track

    nowTime = time.time() # get nowtime
    value.timestamp = nowTime*1000
    value.source_time = nowTime*1000

    # value.timestamp = data.header.stamp.secs*1000 + data.header.stamp.nsecs / 1000000
    # value.source_time = data.header.stamp.secs*1000 + data.header.stamp.nsecs / 1000000

    output.gnss = value
    output.vid = '072e92d501f6'
    pub.publish(output)


rospy.init_node('transfer_Lilee_gps')
#rate = rospy.Rate(0.5)
sub_gps = rospy.Subscriber('/novatel/gps', GPSFix, callback_gps)

pub = rospy.Publisher('/lilee_gnss', Lilee_gps, queue_size=100)

rospy.loginfo('Subscribing from topic "/novatel/gps"')
rospy.loginfo('Publishing msg in topic "/lilee_gnss"')

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
