#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from dbw_mkz_msgs.msg import BrakeReport
from std_msgs.msg import ColorRGBA, Empty, Bool
from mqtt_bridge.msg import DrivingModeWst

output = DrivingModeWst()

def callback1(data):
    global output
    output.vid = '558e429c54ca'
    output.ecu.timestamp = data.header.stamp.secs * pow(10,3) + data.header.stamp.nsecs / 1000000
    output.ecu.source_time = data.header.stamp.secs * pow(10,3) + data.header.stamp.nsecs / 1000000
    pub.publish(output)

def cb_en(data):
    global output
    output.ecu.driving_mode = 1
    pub.publish(output)

def cb_dis(data):
    global output
    output.ecu.driving_mode = 0
    pub.publish(output)


rospy.init_node('transfer_driving_mode')
#rate = rospy.Rate(0.5)
sub_brakereport = rospy.Subscriber('/vehicle/brake_report', BrakeReport, callback1)
sub_en  = rospy.Subscriber('/vehicle/enable', Empty, cb_en)
sub_dis = rospy.Subscriber('/vehicle/disable', Empty, cb_dis)
pub = rospy.Publisher('/DrivingModeWst', DrivingModeWst, queue_size=20)
rospy.loginfo('Subscribing from topic "/vehicle/enable" & "/vehicle/disable"')
rospy.loginfo('Publishing msg in topic "/DrivingModeWst"')

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
