#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from dbw_mkz_msgs.msg import FuelLevelReport
from mqtt_bridge.msg import FuelLevelReportWst


output=FuelLevelReportWst()

def callback(data):
    global output
    output.vid = '558e429c54ca'
    output.ecu.fuel_level = data.fuel_level
    output.ecu.timestamp = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
    output.ecu.source_time = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
    pub.publish(output)

rospy.init_node('transfer_fuel')
#rate = rospy.Rate(0.5)
sub = rospy.Subscriber('/vehicle/fuel_level_report', FuelLevelReport, callback)
pub = rospy.Publisher('/vehicle/fuel_level_report_wst', FuelLevelReportWst, queue_size=20)
rospy.loginfo('Subscribing from topic "/vehicle/fuel_level_report"')
rospy.loginfo('Publishing msg in topic "/vehicle/fuel_level_report_wst"')

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
