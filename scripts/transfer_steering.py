#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from dbw_mkz_msgs.msg import SteeringReport
from mqtt_bridge.msg import SteeringReportWst


output=SteeringReportWst()

def callback(data):
    global output
    output.vid = '558e429c54ca'
    output.ecu.steering_wheel_angle = -math.degrees(data.steering_wheel_angle)
    output.ecu.timestamp = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
    output.ecu.source_time = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
    pub.publish(output)

rospy.init_node('transfer_steering')
#rate = rospy.Rate(0.5)
sub = rospy.Subscriber('/vehicle/steering_report', SteeringReport, callback)
pub = rospy.Publisher('/vehicle/steering_report_wst', SteeringReportWst, queue_size=20)
rospy.loginfo('Subscribing from topic "/vehicle/steering_report"')
rospy.loginfo('Publishing msg in topic "/vehicle/steering_report_wst"')

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
