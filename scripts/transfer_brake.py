#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from dbw_mkz_msgs.msg import BrakeReport
from dbw_mkz_msgs.msg import BrakeInfoReport
from mqtt_bridge.msg import BrakeReportWst


output=BrakeReportWst()

def callback1(data):
    global output
    output.vid = '558e429c54ca'
    output.ecu.brake_pos = (data.pedal_output-0.15) * (100/35) * 100
    output.ecu.timestamp = data.header.stamp.secs * pow(10,3) + data.header.stamp.nsecs / 1000000
    output.ecu.source_time = data.header.stamp.secs * pow(10,3) + data.header.stamp.nsecs / 1000000
    pub.publish(output)

def callback2(data):
    global output

    if data.parking_brake.status == 0:
        output.ecu.parking_brake_status = 0
    elif data.parking_brake.status == 1:
        output.ecu.parking_brake_status = 3
    elif data.parking_brake.status == 2:
        output.ecu.parking_brake_status = 1
    else:
        output.ecu.parking_brake_status = 2

rospy.init_node('transfer_brake')
#rate = rospy.Rate(0.5)
sub_brakereport = rospy.Subscriber('/vehicle/brake_report', BrakeReport, callback1)
sub_brakeinforeport = rospy.Subscriber('/vehicle/brake_info_report', BrakeInfoReport, callback2)
pub = rospy.Publisher('/BrakeReportWst', BrakeReportWst, queue_size=20)
rospy.loginfo('Subscribing from topic "/vehicle/brake_report" & "/vehicle/brake_info_report"')
rospy.loginfo('Publishing msg in topic "/BrakeReportWst"')

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
