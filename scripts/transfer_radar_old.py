#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from mqtt_bridge.msg import RadarTrackWst
from mqtt_bridge.msg import SRR
from mqtt_bridge.msg import ESR



def callback_esr(data):
    if (data.pose.position.x == 0):
        pass
    else:

        output=RadarTrackWst()
        output.vid = '558e429c54ca'
        output.radar.uid = 'esr'
        output.radar.status = 3


        list=[]
        
        output.radar.ESR=ESR()
        output.radar.ESR.tid = str(data.id)
        
        output.radar.ESR.range = math.sqrt(math.pow(data.pose.position.x,2)+math.pow(data.pose.position.y,2))

        if data.pose.position.x != 0 :
            output.radar.ESR.bearing_angle = math.degrees(math.atan(data.pose.position.y/data.pose.position.x))
        else :
            output.radar.ESR.bearing_angle = 0

        list.append(output.radar.ESR)
        
        output.radar.ESR=list

        output.radar.timestamp = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
        output.radar.source_time = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
        pub.publish(output)

def callback_srr_left(data):
    if (data.pose.position.x == 0):
        pass
    else:
        output=RadarTrackWst()
        output.vid = '78d004246e27'
        output.radar.uid = 'srr_left'
        output.radar.status = 3
        

        list=[]
        
        output.radar.SRR=SRR()
        output.radar.SRR.tid = str(data.id)
        
        output.radar.SRR.range = math.sqrt(math.pow(data.pose.position.x,2)+math.pow(data.pose.position.y,2))

        if data.pose.position.x != 0 :
            output.radar.SRR.bearing_angle = math.degrees(math.atan(data.pose.position.y/data.pose.position.x))
        else :
            output.radar.SRR.bearing_angle = 0

        list.append(output.radar.SRR)
        
        output.radar.SRR=list

        output.radar.timestamp = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
        output.radar.source_time = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
        pub.publish(output)

def callback_srr_right(data):
    if (data.pose.position.x == 0):
        pass
    else:
        output=RadarTrackWst()
        output.vid = '78d004246e27'
        output.radar.uid = 'srr_right'
        output.radar.status = 3


        list=[]
        
        output.radar.SRR=SRR()
        output.radar.SRR.tid = str(data.id)
        
        output.radar.SRR.range = math.sqrt(math.pow(data.pose.position.x,2)+math.pow(data.pose.position.y,2))

        if data.pose.position.x != 0 :
            output.radar.SRR.bearing_angle = math.degrees(math.atan(data.pose.position.y/data.pose.position.x))
        else :
            output.radar.SRR.bearing_angle = 0

        list.append(output.radar.SRR)
        
        output.radar.SRR=list

        output.radar.timestamp = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
        output.radar.source_time = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
        pub.publish(output)

rospy.init_node('transfer_radar')
#rate = rospy.Rate(0.5)
sub_1 = rospy.Subscriber('/as_tx/radar_markers', Marker, callback_esr)
sub_2 = rospy.Subscriber('/srr_left/as_tx/radar_markers', Marker, callback_srr_left)
sub_3 = rospy.Subscriber('/srr_right/as_tx/radar_markers', Marker, callback_srr_right)
pub = rospy.Publisher('/RadarTrackWst', RadarTrackWst, queue_size=60)
rospy.loginfo('Subscribing from topic "/as_tx/radar_markers" & "/srr_left/as_tx/radar_markers" & "/srr_right/as_tx/radar_markers"')
rospy.loginfo('Publishing msg in topic "/RadarTrackWst"')

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
