#!/usr/bin/env python
# license removed for brevity


import rospy
import math
from threading import Timer
#import time
#from datetime import timedelta
from std_msgs.msg import String
from mqtt_bridge.msg import BrakeReportWst
from mqtt_bridge.msg import DetectedObjectWst
from mqtt_bridge.msg import FuelLevelReportWst
from mqtt_bridge.msg import GearReportWst
from mqtt_bridge.msg import GPSFixWst
from mqtt_bridge.msg import ImuWst
from mqtt_bridge.msg import RadarTrackWst
from mqtt_bridge.msg import SteeringReportWst
from mqtt_bridge.msg import ThrottleInfoReportWst
from mqtt_bridge.msg import TurnSignalCmdWst
from mqtt_bridge.msg import VehicleSpeedWst
from mqtt_bridge.msg import DrivingModeWst
from mqtt_bridge.msg import SensorStatusWst
from mqtt_bridge.msg import hexcmd

import sys 
sys.path.append('../src/mqtt_bridge')
from mqtt_bridge import json_message_converter as js_converter


output=hexcmd()


def callback(data):
    data_out = js_converter.convert_ros_message_to_json(data)     
    data_hex = ''.join([hex(ord(x))[2:] for x in data_out]) 
    output.hexcmd = data_hex.replace('\n', '').replace('\r', '')
    pub.publish(output)


rospy.init_node('transfer_hex')
#rate = rospy.Rate(0.5)
sub_brake = rospy.Subscriber('/BrakeReportWst', BrakeReportWst, callback)
#sub_DO = rospy.Subscriber('/DetectedObjectWst', DetectedObjectWst, callback2)
#sub_fuel = rospy.Subscriber('/vehicle/fuel_level_report_wst', FuelLevelReportWst, callback)
#sub_gear = rospy.Subscriber('/vehicle/gear_report_wst', GearReportWst, callback)
#sub_gnss = rospy.Subscriber('/novatel/gps_wst', GPSFixWst, callback)
#sub_imu = rospy.Subscriber('/novatel/imu_wst', ImuWst, callback)
#sub_radar = rospy.Subscriber('/RadarTrackWst', RadarTrackWst, callback)
#sub_steering = rospy.Subscriber('/vehicle/steering_report_wst', SteeringReportWst, callback)
#sub_throttle = rospy.Subscriber('/vehicle/throttle_info_report_wst', ThrottleInfoReportWst, callback)
#sub_turn = rospy.Subscriber('/vehicle/turn_signal_cmd_wst', TurnSignalCmdWst, callback)
#sub_vehiclespeed = rospy.Subscriber('/vehicle/steering_report_speed_wst', VehicleSpeedWst, callback)
#sub_drivingmode = rospy.Subscriber('/DrivingModeWst', DrivingModeWst, callback)
#sub_sensorstatus = rospy.Subscriber('/SensorStatusWst', SensorStatusWst, callback)
pub = rospy.Publisher('/HexCmd', hexcmd, queue_size=20)

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



