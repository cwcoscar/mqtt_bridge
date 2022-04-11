#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from dbw_mkz_msgs.msg import ThrottleInfoReport
from mqtt_bridge.msg import ThrottleInfoReportWst


output=ThrottleInfoReportWst()

def callback(data):
    global output
    output.vid = '558e429c54ca'
    output.ecu.accelerator_pos = data.throttle_pc * 100
    output.ecu.timestamp = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
    output.ecu.source_time = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
    pub.publish(output)

rospy.init_node('transfer_throttle')
#rate = rospy.Rate(0.5)
sub = rospy.Subscriber('/vehicle/throttle_info_report', ThrottleInfoReport, callback)
pub = rospy.Publisher('/vehicle/throttle_info_report_wst', ThrottleInfoReportWst, queue_size=20)
rospy.loginfo('Subscribing from topic "/vehicle/throttle_info_report"')
rospy.loginfo('Publishing msg in topic "/vehicle/throttle_info_report_wst"')

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
