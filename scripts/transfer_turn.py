#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from dbw_mkz_msgs.msg import TurnSignalCmd
from dbw_mkz_msgs.msg import ThrottleInfoReport
from mqtt_bridge.msg import TurnSignalCmdWst


output=TurnSignalCmdWst()

def callback1(data):
    global output
    output.vid = '558e429c54ca'
    output.ecu.turn_signal_pos = data.cmd.value
    pub.publish(output)

def callback2(data):
    global output
    output.vid = '558e429c54ca'
    output.ecu.timestamp = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
    output.ecu.source_time = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
    pub.publish(output)

rospy.init_node('transfer_turn')
#rate = rospy.Rate(0.5)
sub_turn = rospy.Subscriber('/vehicle/turn_signal_cmd', TurnSignalCmd, callback1)
sub_throttle = rospy.Subscriber('/vehicle/throttle_info_report', ThrottleInfoReport, callback2)
pub = rospy.Publisher('/vehicle/turn_signal_cmd_wst', TurnSignalCmdWst, queue_size=40)
rospy.loginfo('Subscribing from topic "/vehicle/turn_signal_cmd" & "/vehicle/throttle_info_report"')
rospy.loginfo('Publishing msg in topic "/vehicle/turn_signal_cmd_wst"')

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
