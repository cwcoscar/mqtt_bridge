#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from dbw_mkz_msgs.msg import GearReport
from mqtt_bridge.msg import GearReportWst


output=GearReportWst()

def callback(data):
    global output
    Gear_msg0 = 'None'
    Gear_msg1 = 'P'
    Gear_msg2 = 'R'
    Gear_msg3 = 'N'
    Gear_msg4 = 'D'
    Gear_msg5 = 'Low'
    output.vid = '558e429c54ca'
    if data.state.gear == 0:
        output.ecu.gear_state = Gear_msg0
    elif data.state.gear == 1:
        output.ecu.gear_state = Gear_msg1
    elif data.state.gear == 2:
        output.ecu.gear_state = Gear_msg2
    elif data.state.gear == 3:
        output.ecu.gear_state = Gear_msg3
    elif data.state.gear == 4:
        output.ecu.gear_state = Gear_msg4
    else:
        output.ecu.gear_state = Gear_msg5
    
    output.ecu.timestamp = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
    output.ecu.source_time = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
    pub.publish(output)

rospy.init_node('transfer_gear')
#rate = rospy.Rate(0.5)
sub = rospy.Subscriber('/vehicle/gear_report', GearReport, callback)
pub = rospy.Publisher('/vehicle/gear_report_wst', GearReportWst, queue_size=20)
rospy.loginfo('Subscribing from topic "/vehicle/gear_report"')
rospy.loginfo('Publishing msg in topic "/vehicle/gear_report_wst"')

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
