#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from mqtt_bridge.msg import ImuWst


output=ImuWst()

def callback(data):
    global output
    output.vid = '558e429c54ca'
    output.imu.uid = 'novatel_imu'


    gyro_list=[]
    ### x
    output.imu.gyro   =  Float64()
    output.imu.gyro   =  math.degrees(data.angular_velocity.x)
    gyro_list.append(output.imu.gyro)
    ### y
    output.imu.gyro   =  Float64()
    output.imu.gyro   =  math.degrees(data.angular_velocity.y)
    gyro_list.append(output.imu.gyro)
    ### z
    output.imu.gyro   =  Float64()
    output.imu.gyro   =  math.degrees(data.angular_velocity.z)
    gyro_list.append(output.imu.gyro)
    output.imu.gyro   = gyro_list

    output.imu.roll_rate   = data.angular_velocity.x
    output.imu.pitch_rate  = data.angular_velocity.y
    output.imu.yaw_rate    = data.angular_velocity.z

    acc_list=[]
    ### x
    output.imu.acc   =  Float64()
    output.imu.acc   =  data.linear_acceleration.x/9.80665
    acc_list.append(output.imu.acc)
    ### y
    output.imu.acc   =  Float64()
    output.imu.acc   =  data.linear_acceleration.y/9.80665
    acc_list.append(output.imu.acc)
    ### z
    output.imu.acc   =  Float64()
    output.imu.acc   =  data.linear_acceleration.z/9.80665
    acc_list.append(output.imu.acc)
    output.imu.acc   = acc_list

    d2dt_list=[]
    ### x
    output.imu.d2dt   =  Float64()
    output.imu.d2dt   =  data.linear_acceleration.x
    d2dt_list.append(output.imu.d2dt)
    ### y
    output.imu.d2dt   =  Float64()
    output.imu.d2dt   =  data.linear_acceleration.y
    d2dt_list.append(output.imu.d2dt)
    ### z
    output.imu.d2dt   =  Float64()
    output.imu.d2dt   =  data.linear_acceleration.z
    d2dt_list.append(output.imu.d2dt)
    output.imu.d2dt   = d2dt_list

    output.imu.timestamp   = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
    output.imu.source_time = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
    pub.publish(output)

rospy.init_node('transfer_imu')
#rate = rospy.Rate(0.5)
sub = rospy.Subscriber('/novatel/imu', Imu, callback)
pub = rospy.Publisher('/novatel/imu_wst', ImuWst, queue_size=20)
rospy.loginfo('Subscribing from topic "/novatel/imu"')
rospy.loginfo('Publishing msg in topic "/novatel/imu_wst"')

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
