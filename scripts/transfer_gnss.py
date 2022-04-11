#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from std_msgs.msg import Float64
from gps_common.msg import GPSFix
from mqtt_bridge.msg import GPSFixWst




def callback(data):
    output=GPSFixWst()
    output.vid = '558e429c54ca'
    # output.gnss.coord.latitude = data.latitude
    # output.gnss.coord.longitude = data.longitude
    # output.gnss.coord.altitude = data.altitude
    list=[]

    output.gnss.coord = Float64()
    output.gnss.coord = data.latitude
    list.append(output.gnss.coord)

    output.gnss.coord = Float64()
    output.gnss.coord = data.longitude
    list.append(output.gnss.coord)

    output.gnss.coord = Float64()
    output.gnss.coord = data.altitude 
    list.append(output.gnss.coord)

    output.gnss.coord = list
    output.gnss.speed = data.speed
    output.gnss.heading = data.track
    output.gnss.timestamp = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
    output.gnss.source_time = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
    pub.publish(output)

rospy.init_node('transfer_gnss')
#rate = rospy.Rate(0.5)
sub = rospy.Subscriber('/novatel/gps', GPSFix, callback)
pub = rospy.Publisher('/novatel/gps_wst', GPSFixWst, queue_size=20)
rospy.loginfo('Subscribing from topic "/novatel/gps"')
rospy.loginfo('Publishing msg in topic "/novatel/gps_wst"')

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
