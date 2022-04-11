#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from dbw_mkz_msgs.msg import BrakeReport
from dbw_mkz_msgs.msg import BrakeInfoReport
from mqtt_bridge.msg import BrakeLightReportWst


output=BrakeLightReportWst()

def callback1(data):
    global output
    output.vid = '558e429c54ca'
    if data.boo_output == True :
       output.ecu.brake_light_pos = 1
    else:
       output.ecu.brake_light_pos = 0
    pub.publish(output)

def callback2(data):
    global output
    output.ecu.timestamp = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
    output.ecu.source_time = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
    pub.publish(output)

rospy.init_node('transfer_brake_light')
sub_brakereport = rospy.Subscriber('/vehicle/brake_report', BrakeReport, callback1)
sub_brakeinforeport = rospy.Subscriber('/vehicle/brake_info_report', BrakeInfoReport, callback2)
pub = rospy.Publisher('/BrakeLightReportWst', BrakeLightReportWst, queue_size=20)
rospy.loginfo('Subscribing from topic "/vehicle/brake_report" & "/vehicle/brake_info_report"')
rospy.loginfo('Publishing msg in topic "/BrakeLightReportWst"')
if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
