#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from radar_msgs.msg import RadarTrackArray
from dbw_mkz_msgs.msg import BrakeReport
from mqtt_bridge.msg import SensorStatusWst


output = SensorStatusWst()

output.vid = '558e429c54ca'
output.radar.uid = 'radar'
output.lidar.uid = 'lidar'
output.camera.uid = 'usb_cam'

def callback(data):
    global output
    output.radar.timestamp = data.header.stamp.secs * pow(10,3) + data.header.stamp.nsecs / 1000000
    output.radar.source_time = data.header.stamp.secs * pow(10,3) + data.header.stamp.nsecs / 1000000
    output.lidar.timestamp = data.header.stamp.secs * pow(10,3) + data.header.stamp.nsecs / 1000000
    output.lidar.source_time = data.header.stamp.secs * pow(10,3) + data.header.stamp.nsecs / 1000000
    output.camera.timestamp = data.header.stamp.secs * pow(10,3) + data.header.stamp.nsecs / 1000000
    output.camera.source_time = data.header.stamp.secs * pow(10,3) + data.header.stamp.nsecs / 1000000
    pub.publish(output)

def callback1(data):
    global output
    if data is not None:
	output.radar.status = 3
    else:
	output.radar.status = 0
    output.radar.timestamp = data.header.stamp.secs * pow(10,3) + data.header.stamp.nsecs / 1000000
    output.radar.source_time = data.header.stamp.secs * pow(10,3) + data.header.stamp.nsecs / 1000000
    pub.publish(output)

def callback2(data):
    global output
    if data is not None:
	output.lidar.status = 3
    else:
	output.lidar.status = 0
    output.lidar.timestamp = data.header.stamp.secs * pow(10,3) + data.header.stamp.nsecs / 1000000
    output.lidar.source_time = data.header.stamp.secs * pow(10,3) + data.header.stamp.nsecs / 1000000
    pub.publish(output)

def callback3(data):
    global output
    if data is not None:
	output.camera.status = 3
    else:
	output.camera.status = 0
    output.camera.timestamp = data.header.stamp.secs * pow(10,3) + data.header.stamp.nsecs / 1000000
    output.camera.source_time = data.header.stamp.secs * pow(10,3) + data.header.stamp.nsecs / 1000000
    pub.publish(output)


rospy.init_node('transfer_sensor')
sub_brakereport = rospy.Subscriber('/vehicle/brake_report', BrakeReport, callback)
sub_radar = rospy.Subscriber('/as_tx/radar_tracks', RadarTrackArray, callback1)
sub_lidar = rospy.Subscriber('/points_raw', PointCloud2, callback2)
sub_camera1 = rospy.Subscriber('/gmsl_camera/port_0/cam_0/image_raw', Image, callback3)
sub_camera3 = rospy.Subscriber('/usb_cam/image_raw', Image, callback2)
pub = rospy.Publisher('/SensorStatusWst', SensorStatusWst, queue_size=20)

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
