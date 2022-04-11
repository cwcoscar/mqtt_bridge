#!/usr/bin/env python
# license removed for brevity
import rospy
import tf
import math
import pymap3d as pm

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_msgs.msg import Float64
from mqtt_bridge.msg import DetectedObjectWst
from mqtt_bridge.msg import Obj
#from visualization_msgs.msg import Marker
from radar_msgs.msg import RadarTrackArray
from autoware_msgs.msg import DetectedObjectArray



def callback_lidar(data):# data.objects is type <list>
    pose_in_velodyne = PoseStamped()
    pose_in_map      = PoseStamped()

    output=DetectedObjectWst()
    #output.uid = '558e429c54ca'
    output.DO.timestamp   = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
    output.DO.source_time = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000

    list=[]
     
    
    for i in range(len(data.objects)):
        output.DO.obj = Obj()
        output.DO.obj.tid        = str(data.objects[i].id)


        pose_in_velodyne.header    = data.objects[i].header
        pose_in_velodyne.pose      = data.objects[i].pose
        try:
            #listener.waitForTransform('velodyne', 'map', data.header.stamp, rospy.Duration(4.0))
            pose_in_map                = listener.transformPose('map',pose_in_velodyne)
        except (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException):
            pass

        pt_in_geodetic=pm.enu2geodetic(pose_in_map.pose.position.x,\
                                        pose_in_map.pose.position.y,\
                                        pose_in_map.pose.position.z,\
                                        22.99665875,\
                                        120.222584889,\
                                        98.211                 #lat0, lon0, alt0
        )
        ##print(pt_in_geodetic)
        # output.DO.obj.coord.latitude  = pt_in_geodetic[0]
        # output.DO.obj.coord.longitude = pt_in_geodetic[1]
        # output.DO.obj.coord.altitude  = pt_in_geodetic[2]
        coord_list=[]
        for j in range(len(pt_in_geodetic)):
            output.DO.obj.do_coordinate = Float64()
            output.DO.obj.do_coordinate = pt_in_geodetic[j]
            coord_list.append(output.DO.obj.do_coordinate)

        output.DO.obj.do_coordinate   = coord_list
    

        ##There is no orientation in lla coord. so doesn't assign value
        ##output.DO.obj.orien      = orientation_in_lla
        do_heading_list=[]
        ### x
        output.DO.obj.do_heading   =  Float64()
        output.DO.obj.do_heading   =  0.0
        do_heading_list.append(output.DO.obj.do_heading)
        ### y
        output.DO.obj.do_heading   =  Float64()
        output.DO.obj.do_heading   =  0.0
        do_heading_list.append(output.DO.obj.do_heading)
        ### z
        output.DO.obj.do_heading   =  Float64()
        output.DO.obj.do_heading   =  0.0
        do_heading_list.append(output.DO.obj.do_heading)
        ### w
        output.DO.obj.do_heading   =  Float64()
        output.DO.obj.do_heading   =  1.0
        do_heading_list.append(output.DO.obj.do_heading)

        output.DO.obj.do_heading   =  do_heading_list
        #output.DO.obj.orien.w = 1

        dim_list=[]
        ### x
        output.DO.obj.do_dimension   =  Float64()
        output.DO.obj.do_dimension   =  data.objects[i].dimensions.x
        dim_list.append(output.DO.obj.do_dimension)
        ### y
        output.DO.obj.do_dimension   =  Float64()
        output.DO.obj.do_dimension   =  data.objects[i].dimensions.y
        dim_list.append(output.DO.obj.do_dimension)
        ### z
        output.DO.obj.do_dimension   =  Float64()
        output.DO.obj.do_dimension   =  data.objects[i].dimensions.z
        dim_list.append(output.DO.obj.do_dimension)

        output.DO.obj.do_dimension   = dim_list
        

        #output.DO.obj.dim        = data.objects[i].dimensions

        # output.DO.obj.roi.hgt    = data.objects[i].roi_image.height
        # output.DO.obj.roi.wid    = data.objects[i].roi_image.width
        # output.DO.obj.roi.step   = data.objects[i].roi_image.step
        # output.DO.obj.roi.bigend = bool(data.objects[i].roi_image.is_bigendian)
        # output.DO.obj.roi.fourcc = data.objects[i].roi_image.encoding
        # output.DO.obj.roi.data   = data.objects[i].roi_image.data
        list.append(output.DO.obj)
    #print(pt_in_map)  
    output.DO.obj=list
    ##  show the number of objects 
    #print(len(data.objects)) 
    pub.publish(output)





# def callback_image(data):
#     if len(data.objects)!=0 :
#         output=DetectedObjectMqtt()
#         output.vid = '78d004246e27'
#         output.DO.source_type = '4'
#         output.DO.timestamp   = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
#         output.DO.source_time = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000

#         list=[]
#         for i in range(len(data.objects)):
#             output.DO.obj = Obj()
#             output.DO.obj.label        = data.objects[i].label
#             output.DO.obj.score        = data.objects[i].score
#             output.DO.obj.image_frame  = data.objects[i].image_frame
#             output.DO.obj.x            = data.objects[i].x
#             output.DO.obj.y            = data.objects[i].y
#             output.DO.obj.width        = data.objects[i].width
#             output.DO.obj.height       = data.objects[i].height
#             output.DO.obj.angle        = data.objects[i].angle
#             list.append(output.DO.obj)
#         output.DO.obj=list


#         pub.publish(output)
#     else:
#         pass






def callback_esr(data):
    


    pose_in_esr = PoseStamped()
    pose_in_map = PoseStamped()

    output=DetectedObjectWst()
    #output.uid = '558e429c54ca'
    output.DO.timestamp   = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
    output.DO.source_time = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000


    list=[]

    for i in range(len(data.tracks)):
        
        output.DO.obj = Obj()
        output.DO.obj.tid    = str(data.tracks[i].track_id)



        sum_x=0
        sum_y=0
        sum_z=0
        for j in range(len(data.tracks[i].track_shape.points)): 
            sum_x = sum_x + data.tracks[i].track_shape.points[j].x
            sum_y = sum_y + data.tracks[i].track_shape.points[j].y
            sum_z = sum_z + data.tracks[i].track_shape.points[j].z
        avg_x=sum_x/len(data.tracks[i].track_shape.points)
        avg_y=sum_y/len(data.tracks[i].track_shape.points)
        avg_z=sum_z/len(data.tracks[i].track_shape.points)
        
        pose_in_esr.header               = data.header
        pose_in_esr.pose.position.x      = avg_x
        pose_in_esr.pose.position.y      = avg_y
        pose_in_esr.pose.position.z      = avg_z
        
        try:
            #listener.waitForTransform('delphi_esr', 'map', data.header.stamp, rospy.Duration(4.0))
            #listener.waitForTransform('delphi_esr', 'map', data.header.stamp, rospy.Duration(4.0))
            pose_in_map                = listener.transformPose('map',pose_in_esr)
        except (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException):
            pass

        
        pt_in_geodetic=pm.enu2geodetic(pose_in_map.pose.position.x,\
                                        pose_in_map.pose.position.y,\
                                        pose_in_map.pose.position.z,\
                                        22.99665875,\
                                        120.222584889,\
                                        98.211                 #lat0, lon0, alt0
        )

        coord_list=[]
        for k in range(len(pt_in_geodetic)):
            output.DO.obj.do_coordinate = Float64()
            output.DO.obj.do_coordinate = pt_in_geodetic[k]
            coord_list.append(output.DO.obj.do_coordinate)

        output.DO.obj.do_coordinate   = coord_list


        ##There is no orientation in lla coord. so doesn't assign value
        ##output.DO.obj.orien      = orientation_in_lla
        do_heading_list=[]
        ### x
        output.DO.obj.do_heading   =  Float64()
        output.DO.obj.do_heading   =  0.0
        do_heading_list.append(output.DO.obj.do_heading)
        ### y
        output.DO.obj.do_heading   =  Float64()
        output.DO.obj.do_heading   =  0.0
        do_heading_list.append(output.DO.obj.do_heading)
        ### z
        output.DO.obj.do_heading   =  Float64()
        output.DO.obj.do_heading   =  0.0
        do_heading_list.append(output.DO.obj.do_heading)
        ### w
        output.DO.obj.do_heading   =  Float64()
        output.DO.obj.do_heading   =  1.0
        do_heading_list.append(output.DO.obj.do_heading)

        output.DO.obj.do_heading   =  do_heading_list
        #output.DO.obj.orien.w = 1

        dim_list=[]
        ### x(depth)
        output.DO.obj.do_dimension     =  Float64()
        output.DO.obj.do_dimension     =  0
        dim_list.append(output.DO.obj.do_dimension)
        ### y(width)
        output.DO.obj.do_dimension    =  Float64()
        output.DO.obj.do_dimension    =  abs(data.tracks[i].track_shape.points[1].y-data.tracks[i].track_shape.points[0].y)
        dim_list.append(output.DO.obj.do_dimension)
        ### z(height)
        output.DO.obj.do_dimension    =  Float64()
        output.DO.obj.do_dimension    =  abs(data.tracks[i].track_shape.points[3].z-data.tracks[i].track_shape.points[0].z)
        dim_list.append(output.DO.obj.do_dimension)


        output.DO.obj.do_dimension   =  dim_list


        list.append(output.DO.obj)

    output.DO.obj=list

    pub.publish(output)


# def callback_srr_left(data):
#     if (data.pose.position.x == 0)& (data.pose.position.x == 0):
#         pass
#     else:
#         pose_in_srr_left = PoseStamped()
#         pose_in_map = PoseStamped()

#         output=DetectedObjectMqtt()
#         output.vid = '78d004246e27'
#         output.DO.source_type = '2'
#         output.DO.timestamp   = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
#         output.DO.source_time = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000


#         list=[]
        
#         output.DO.obj = Obj()
#         output.DO.obj.tid          = str(data.id)
#         pose_in_srr_left.header    = data.header
#         pose_in_srr_left.pose      = data.pose
        
#         try:
#             pose_in_map                = listener.transformPose('map',pose_in_srr_left)
#         except (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException):
#             pass

        
#         pt_in_geodetic=pm.enu2geodetic(pose_in_map.pose.position.x,\
#                                         pose_in_map.pose.position.y,\
#                                         pose_in_map.pose.position.z,\
#                                         22.99665875,\
#                                         120.222584889,\
#                                         98.211                 #lat0, lon0, alt0
#         )

#         coord_list=[]
#         for j in range(len(pt_in_geodetic)):
#             output.DO.obj.coord = Float64()
#             output.DO.obj.coord = pt_in_geodetic[j]
#             coord_list.append(output.DO.obj.coord)

#         output.DO.obj.coord   = coord_list


#         ##There is no orientation in lla coord. so doesn't assign value
#         ##output.DO.obj.orien      = orientation_in_lla
#         orien_list=[]
#         ### x
#         output.DO.obj.orien   =  Float64()
#         output.DO.obj.orien   =  0
#         orien_list.append(output.DO.obj.orien)
#         ### y
#         output.DO.obj.orien   =  Float64()
#         output.DO.obj.orien   =  0
#         orien_list.append(output.DO.obj.orien)
#         ### z
#         output.DO.obj.orien   =  Float64()
#         output.DO.obj.orien   =  0
#         orien_list.append(output.DO.obj.orien)
#         ### w
#         output.DO.obj.orien   =  Float64()
#         output.DO.obj.orien   =  1
#         orien_list.append(output.DO.obj.orien)

#         output.DO.obj.orien   =  orien_list
#         #output.DO.obj.orien.w = 1


#         list.append(output.DO.obj)

#         output.DO.obj=list

#         pub.publish(output)



# def callback_srr_right(data):
#     if (data.pose.position.x == 0)& (data.pose.position.x == 0):
#         pass
#     else:
#         pose_in_srr_right = PoseStamped()
#         pose_in_map = PoseStamped()

#         output=DetectedObjectMqtt()
#         output.vid = '78d004246e27'
#         output.DO.source_type = '2'
#         output.DO.timestamp   = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000
#         output.DO.source_time = data.header.stamp.secs*pow(10,3)+data.header.stamp.nsecs/1000000


#         list=[]
        
#         output.DO.obj = Obj()
#         output.DO.obj.tid          = str(data.id)
#         pose_in_srr_right.header    = data.header
#         pose_in_srr_right.pose      = data.pose
        
#         try:
#             pose_in_map                = listener.transformPose('map',pose_in_srr_right)
#         except (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException):
#             pass

        
#         pt_in_geodetic=pm.enu2geodetic(pose_in_map.pose.position.x,\
#                                         pose_in_map.pose.position.y,\
#                                         pose_in_map.pose.position.z,\
#                                         22.99665875,\
#                                         120.222584889,\
#                                         98.211                 #lat0, lon0, alt0
#         )

#         coord_list=[]
#         for j in range(len(pt_in_geodetic)):
#             output.DO.obj.coord = Float64()
#             output.DO.obj.coord = pt_in_geodetic[j]
#             coord_list.append(output.DO.obj.coord)

#         output.DO.obj.coord   = coord_list


#         ##There is no orientation in lla coord. so doesn't assign value
#         ##output.DO.obj.orien      = orientation_in_lla
#         orien_list=[]
#         ### x
#         output.DO.obj.orien   =  Float64()
#         output.DO.obj.orien   =  0
#         orien_list.append(output.DO.obj.orien)
#         ### y
#         output.DO.obj.orien   =  Float64()
#         output.DO.obj.orien   =  0
#         orien_list.append(output.DO.obj.orien)
#         ### z
#         output.DO.obj.orien   =  Float64()
#         output.DO.obj.orien   =  0
#         orien_list.append(output.DO.obj.orien)
#         ### w
#         output.DO.obj.orien   =  Float64()
#         output.DO.obj.orien   =  1
#         orien_list.append(output.DO.obj.orien)

#         output.DO.obj.orien   =  orien_list
    


#         list.append(output.DO.obj)

#         output.DO.obj=list

#         pub.publish(output)

rospy.init_node('transfer_DO')
listener = tf.TransformListener()
sub_1 = rospy.Subscriber('/detection/lidar_detector/objects', DetectedObjectArray, callback_lidar)
# sub_2 = rospy.Subscriber('/detection/image_detector/objects', DetectedObjectArray, callback_image)
sub_3 = rospy.Subscriber('/as_tx/radar_tracks', RadarTrackArray, callback_esr)
# sub_4 = rospy.Subscriber('/srr_left/as_tx/radar_markers', Marker, callback_srr_left)
# sub_5 = rospy.Subscriber('/srr_right/as_tx/radar_markers', Marker, callback_srr_right)
pub = rospy.Publisher('/DetectedObjectWst', DetectedObjectWst, queue_size=60)
rospy.loginfo('Subscribing from topic "/detection/lidar_detector/objects" & "/as_tx/radar_tracks"')
rospy.loginfo('Publishing msg in topic "/DetectedObjectWst"')

if __name__ == '__main__':
    try: 
        rospy.spin()
    except (rospy.ROSInterruptException,rospy.ROSTimeMovedBackwardsException):
        pass
