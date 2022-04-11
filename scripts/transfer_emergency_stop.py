#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from dbw_mkz_msgs.msg import BrakeReport
from std_msgs.msg import ColorRGBA, Empty, Bool
from mqtt_bridge.msg import DrivingModeWst
from mqtt_bridge.msg import EmergencyStopWst


output = EmergencyStopWst()

def callback1(data):
    global output
    output.vid = '558e429c54ca'
    output.ecu.timestamp = data.ecu.timestamp
    output.ecu.source_time = data.ecu.source_time
    pub.publish(output)

en = 2
dis = 2

def cb_en(data):
    if data is not None:
	en = 1
    else:
	en = 0

def cb_dis(data):
    if data is not None:
	dis = 1
    else:
	dis = 0

if en == 0 and dis == 0 :
   output.ecu.emergency_stop_button = 1
else:
   output.ecu.emergency_stop_button = 0

rospy.init_node('transfer_emergency_stop')
#rate = rospy.Rate(0.5)
sub_brakereport = rospy.Subscriber('/DrivingModeWst', DrivingModeWst, callback1)
sub_en  = rospy.Subscriber('/vehicle/enable', Empty, cb_en)
sub_dis = rospy.Subscriber('/vehicle/disable', Empty, cb_dis)
pub = rospy.Publisher('/EmergencyStopWst', EmergencyStopWst, queue_size=20)
rospy.loginfo('Publishing msg in topic "/EmergencyStopWst"')

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
