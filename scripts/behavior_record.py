#!/usr/bin/env python
import rospy
import tf
import math
import os
import pendulum
import subprocess
import signal
import shlex
import time
from datetime import datetime

# from autoware_msgs.msg import ndt_stat
# from sensor_msgs.msg import Imu
# from nmea_msgs.msg import Sentence
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import TwistStamped

# from dbw_mkz_msgs.msg import BrakeCmd
# from dbw_mkz_msgs.msg import BrakeInfoReport
# from dbw_mkz_msgs.msg import FuelLevelReport
# from dbw_mkz_msgs.msg import GearReport
# from dbw_mkz_msgs.msg import Misc1Report
# from dbw_mkz_msgs.msg import SteeringCmd
# from dbw_mkz_msgs.msg import SteeringReport
# from dbw_mkz_msgs.msg import SurroundReport
# from dbw_mkz_msgs.msg import SteeringReport
# from dbw_mkz_msgs.msg import ThrottleCmd
# from dbw_mkz_msgs.msg import ThrottleInfoReport
# from dbw_mkz_msgs.msg import ThrottleReport
# from dbw_mkz_msgs.msg import TirePressureReport
# from dbw_mkz_msgs.msg import TurnSignalCmd
# from dbw_mkz_msgs.msg import WheelPositionReport
# from dbw_mkz_msgs.msg import WheelSpeedReport
from delphi_esr_msgs.msg import EsrStatus1
from delphi_esr_msgs.msg import EsrStatus2
from delphi_esr_msgs.msg import EsrStatus3
from delphi_esr_msgs.msg import EsrStatus4
from delphi_esr_msgs.msg import EsrStatus5
from delphi_esr_msgs.msg import EsrStatus6
from delphi_esr_msgs.msg import EsrStatus7
from delphi_esr_msgs.msg import EsrStatus8
from delphi_esr_msgs.msg import EsrStatus9
from delphi_esr_msgs.msg import EsrTrack
from delphi_esr_msgs.msg import EsrTrackMotionPower
from delphi_esr_msgs.msg import EsrValid1
from delphi_esr_msgs.msg import EsrValid2
from delphi_srr_msgs.msg import SrrDebug3
from delphi_srr_msgs.msg import SrrDebug4
from delphi_srr_msgs.msg import SrrDebug5
from delphi_srr_msgs.msg import SrrFeatureAlert
from delphi_srr_msgs.msg import SrrFeatureSwVersion
from delphi_srr_msgs.msg import SrrStatus1
from delphi_srr_msgs.msg import SrrStatus2
from delphi_srr_msgs.msg import SrrStatus3
from delphi_srr_msgs.msg import SrrStatus4
from delphi_srr_msgs.msg import SrrStatus5
from delphi_srr_msgs.msg import SrrTrack


class record_bag():
  button_state = False
  recording = False
  process = 0
  record_cmd = ""
  session_id = 1
  node_name = "recording_bag"
  tz = pendulum.timezone('Asia/Taipei')
  # topic = "/vehicle/dbw_enabled /camera/18259971/image_raw/compressed /camera/18259972/image_raw/compressed /camera/18259877/image_raw/compressed /camera/18259970/image_raw/compressed /nmea_sentence /lidar1/points_raw /lidar2/points_raw /lidar3/points_raw /lidar4/points_raw /gBus_obd_rpm /gBus_obd_speed /vehicle/steering_report /vehicle/throttle_report /vehicle/brake_report /vehicle/steering_cmd /vehicle/throttle_cmd /vehicle/brake_cmd /vehicle/cmd_vel_stamped /vehicle/req_accel /spat /occai/vehicle_status"
  topic = "/gmsl_camera/port_0/cam_0/image_raw /points_raw /as_tx/radar_tracks /as_tx/radar_markers /srr_left/as_tx/radar_markers /srr_right/as_tx/radar_markers"
  #camera_idx = 0,lidar_idx = 1
  # separate_topic = ["-e \"/camera/(.*)/compressed\"","/lidar1/points_raw /lidar2/points_raw /lidar3/points_raw /lidar4/points_raw"]
  separate_topic = ["-e \"(.*)/as_tx/(.*)\"","/points_raw","/gmsl_camera/port_0/cam_0/image_raw"]
  separate_node_name = ["radar_recording_bag","lidar_recording_bag","camera_recording_bag"]
  separate_file_name = ["558e429c54ca_radar_","558e429c54ca_lidar_","558e429c54ca_camera_"]
  separate_folder_name = ["ncku_558e429c54ca_radar_","ncku_558e429c54ca_lidar_","ncku_558e429c54ca_camera_"]
  def __init__(self,home_folder):
    self.home_folder = home_folder
    print(home_folder)
  def enable_callback(self):
    if self.button_state == False:    
      self.button_state = True
      print("Deadman switch off.")

    if self.recording == False:
      print("start to record bagfile")
      self.recording = True
      current_time = datetime.now(self.tz).strftime("%Y-%m-%d-%H-%M-%S") 
      self.process = subprocess.Popen("rosbag record -O /home/mec/record/%s.bag %s __name:=%s"%(current_time,self.topic,self.node_name),shell=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,env=os.environ)
      #record sensor data to corresponding file
      current_time_folder = datetime.now(self.tz).strftime("%Y%m%d")
      current_time_file = datetime.now(self.tz).strftime("%Y%m%d_%H%M")
      for i in range(len(self.separate_file_name)):
        folder = self.create_folder(i,current_time_folder)
        print("folder = %s",folder)
        subprocess.Popen("rosbag record --split --size=1024 -O %s/%s%s.bag %s __name:=%s"%(folder,self.separate_file_name[i],current_time_file,self.separate_topic[i],self.separate_node_name[i]),shell=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,env=os.environ)
  
      # # start logginig files to csv
      # start_log(self.home_folder)
  # def disable_callback(self,data):
  def disable_callback(self):
    if self.button_state == True:
      self.button_state = False
      print("Deadman switch on, wait for another 15 seconds to ensure a propoer end of recording.")
      rospy.Timer(rospy.Duration(15), self.handle_timer_event, oneshot=True)
  def handle_timer_event(self, event):
    if self.recording == True and self.button_state == False:
      print("end the recording bagfile")
      self.recording = False
      subprocess.Popen("rosnode kill %s"%self.node_name,shell=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,env=os.environ)
      # stop sensor data
      for i in range(len(self.separate_file_name)):
        subprocess.Popen("rosnode kill %s"%self.separate_node_name[i],shell=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,env=os.environ)
      # stop logging files to csv
      stop_log()
      self.session_id = 1
    else:
      print("timer event: deadman switch is off, continue on bag recording.")
      
  def create_folder(self,sensor_idx,current_time):
    while os.path.exists(self.home_folder+"/"+self.separate_folder_name[sensor_idx]+current_time+"_"+str(self.session_id)):
      self.session_id+=1
    os.makedirs(self.home_folder+"/"+self.separate_folder_name[sensor_idx]+current_time+"_"+str(self.session_id))
    return self.home_folder+"/"+self.separate_folder_name[sensor_idx]+current_time+"_"+str(self.session_id)
# ndt_stat_csv = ""
# ndt_csv = ""
# imu_csv = ""
# gnss_csv = ""
# steering_cmd_csv = ""
# steering_report_csv = ""
# throttle_cmd_csv = ""
# throttle_report_csv = ""
# brake_cmd_csv = ""
# brake_report_csv = ""
# j1939_speed_csv = ""
# j1939_rpm_csv = ""
# Start_record = 0

# lat = "na"
# lng = "na"
# height = "na"
# heading_hdt = "na"
# heading_pashr = "na"
# roll = "na"
# pitch = "na"
# status = "na"

# def imu_callback(data):
#   global Start_record
#   if Start_record == 1:
#     global imu_csv
#     ax = data.linear_acceleration.x
#     ay = data.linear_acceleration.y
#     az = data.linear_acceleration.z
#     wx = data.angular_velocity.x
#     wy = data.angular_velocity.y
#     wz = data.angular_velocity.z

#     quaternion = (
#       data.orientation.x,
#       data.orientation.y,
#       data.orientation.z,
#       data.orientation.w)
#     euler = tf.transformations.euler_from_quaternion(quaternion)
#     roll = euler[0]
#     pitch = euler[1]
#     yaw = euler[2]

#     word = "{},{},{},{},{},{},{},{},{},{}\n".format(
#       data.header.stamp, ax, ay, az, wx, wy, wz, yaw, pitch, roll)
#     imu_csv.write(word)

# def nmea_callback(data):
#   global Start_record
#   if Start_record == 1:
#     global lat, lng, height, heading_hdt, heading_pashr, status, roll, pitch, gnss_csv
#     now = rospy.get_rostime()
#     #gnss_csv.write("{}\n".format(data.sentence))
#     if "$GPHDT" in data.sentence:
#       fields = data.sentence.split(",")
#       heading_hdt = fields[1]
#     if "PASHR" in data.sentence:
#       fields = data.sentence.split(",")
#       heading_pashr = fields[2]
#       roll = fields[4]
#       pitch = fields[5]
#     if "BESTPOSA" in data.sentence:
#       fields = data.sentence.split(",")
#       lat = float(fields[11])
#       lng = float(fields[12])
#       height = float(fields[13])
#       status = fields[10]
#       #print(data.sentence)
#       word_gps = "{},{},{},{},{},{},{},{},{}\n".format(now, status, lat, lng, height, heading_pashr, pitch, roll, heading_hdt)
#       gnss_csv.write(word_gps)

# def ndt_pose_callback(data):
#   global Start_record
#   if Start_record == 1: 
#     global ndt_csv
#     #print("{} {} {} {}".format(data.header.stamp, data.pose.position.x, data.pose.position.y, data.pose.position.z))
#     #quaternion = (
#     #  data.pose.orientation.x,
#     #  data.pose.orientation.y,
#     #  data.pose.orientation.z,
#     #  data.pose.orientation.w)
#     #euler = tf.transformations.euler_from_quaternion(quaternion)
#     #roll = euler[0]
#     #pitch = euler[1]
#     #yaw = euler[2]
#     word = "{},{},{},{},{},{},{},{}\n".format(data.header.stamp, data.pose.position.x, data.pose.position.y, data.pose.position.z, data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z)
#     ndt_csv.write(word)
    
# def ndt_stat_callback(data):
#   global Start_record
#   if Start_record == 1: 
#     global ndt_stat_csv
#     word = "{},{},{},{},{},{},{}\n".format(data.header.stamp, data.exe_time, data.iteration, data.score, data.velocity, data.acceleration, data.use_predict_pose)
#     ndt_stat_csv.write(word)
    
# def steering_cmd_callback(data):
#   global Start_record
#   if Start_record == 1:
#     global steering_cmd_csv
#     now = rospy.get_rostime()
#     word = "{},{},{}\n".format(now, data.steering_wheel_angle_cmd, data.enable)
#     steering_cmd_csv.write(word)

# def steering_report_callback(data):
#   global Start_record
#   if Start_record == 1:
#     global steering_report_csv
#     word = "{},{},{},{},{}\n".format(data.header.stamp, data.steering_wheel_rpm, data.steering_wheel_angle, data.steering_wheel_torque, data.voltage_level)
#     steering_report_csv.write(word)

# def throttle_cmd_callback(data):
#   global Start_record
#   if Start_record == 1:
#     global throttle_cmd_csv
#     now = rospy.get_rostime()
#     word = "{},{},{},{},{},{},{}\n".format(now, data.pedal_cmd, data.pedal_cmd_type, data.enable, data.clear, data.ignore, data.count)
#     throttle_cmd_csv.write(word)

# def throttle_report_callback(data):
#   global Start_record
#   if Start_record == 1:
#     global throttle_report_csv
#     word = "{},{},{},{},{},{},{},{},{},{}\n".format(data.header.stamp, data.pedal_input, data.pedal_cmd, data.pedal_output, data.enabled, data.override, data.driver, data.timeout, data.watchdog_counter.source, data.fault_wdc)
#     throttle_report_csv.write(word)

# def brake_cmd_callback(data):
#   global Start_record
#   if Start_record == 1:
#     global brake_cmd_csv
#     now = rospy.get_rostime()
#     word = "{},{},{},{},{},{},{}\n".format(now, data.pedal_cmd, data.pedal_cmd_type, data.enable, data.clear, data.ignore, data.count)
#     brake_cmd_csv.write(word)

# def brake_report_callback(data):
#   global Start_record
#   if Start_record == 1:
#     global brake_report_csv
#     word = "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(data.header.stamp, data.pedal_input, data.pedal_cmd, data.pedal_output, data.torque_input, data.torque_cmd, data.torque_output, data.boo_input, data.boo_cmd, data.boo_output, data.enabled, data.override, data.driver, data.timeout, data.watchdog_counter.source, data.fault_wdc)
#     brake_report_csv.write(word)
    
# def j1939_speed_callback(data):
#   global Start_record
#   if Start_record == 1:
#     global j1939_speed_csv
#     word = "{}, {}\n".format(data.header.stamp, data.speed)
#     j1939_speed_csv.write(word)

# def j1939_rpm_callback(data):
#   global Start_record
#   if Start_record == 1:
#     global j1939_rpm_csv
#     word = "{}, {}\n".format(data.header.stamp, data.rpm)
#     j1939_rpm_csv.write(word)

# def stop_log():  
#   global Start_record
#   print("Stop logging")
#   Start_record=0
#   close_file()

# def start_log(home_folder):
#   global Start_record
#   print("Start logging")
#   if Start_record == 0:
#     open_file(home_folder)
#   Start_record = 1
    
# def open_file(home_folder):
  # global ndt_stat_csv, ndt_csv,imu_csv, gnss_csv, steering_cmd_csv, steering_report_csv, throttle_cmd_csv, throttle_report_csv, brake_cmd_csv, brake_report_csv, j1939_speed_csv, j1939_rpm_csv
  # tz = pendulum.timezone('Asia/Taipei')
  # current_time = datetime.now(tz).strftime("%Y-%m-%d-%H-%M-%S")
  
  # ndt_stat_csv = open(home_folder+"/ndt_stat-%s.csv"%current_time, "w+")
  # ndt_stat_csv.write("time, exe_time, iteration, score, velocity, acceleration, use_predict_pose\n")
  
  # ndt_csv = open(home_folder+"/ndt-%s.csv"%current_time, "w+")
  # ndt_csv.write("time, x, y, z, o_w, o_x, o_y, o_z\n")

  # imu_csv = open(home_folder+"/imu-%s.csv"%current_time, "w+")
  # imu_csv.write("time, ax, ay, az, wx, wy, wz, y, p, r\n")

  # gnss_csv = open(home_folder+"/nmea-%s.csv"%current_time, "w+")
  # gnss_csv.write("time, sol_type, x, y, z, head_pashr, pitch, roll, head_hdt\n")
  
  # steering_cmd_csv = open(home_folder+"/steering_cmd-%s.csv"%current_time, "w+")
  # steering_cmd_csv.write("time, angle, enable\n")
  
  # steering_report_csv = open(home_folder+"/steering_report-%s.csv"%current_time, "w+")
  # steering_report_csv.write("time, rpm, angle, torque, volt_level\n")
  
  # throttle_cmd_csv = open(home_folder+"/throttle_cmd-%s.csv"%current_time, "w+")
  # throttle_cmd_csv.write("time, pedal_cmd, pedal_cmd_type, enable, clear, ignore, count\n")
  
  # throttle_report_csv = open(home_folder+"/throttle_report-%s.csv"%current_time, "w+")
  # throttle_report_csv.write("time, pedal_input, pedal_cmd, pedal_output, enabled, override, driver, timeout, watchdog_src, watchdog\n")
  
  # brake_cmd_csv = open(home_folder+"/brake_cmd-%s.csv"%current_time, "w+")
  # brake_cmd_csv.write("time, pedal_cmd, pedal_cmd_type, enable, clear, ignore, count\n")
  
  # brake_report_csv = open(home_folder+"/brake_report-%s.csv"%current_time, "w+")
  # brake_report_csv.write("time, pedal_input, pedal_cmd, pedal_output, torque_input, torque_cmd, torque_output, boo_input, boo_cmd, boo_output, enabled, override, driver, timeout, watchdog_src, watchdog\n ")
  
  # j1939_speed_csv = open(home_folder+"/j1939_speed-%s.csv"%current_time, "w+")
  # j1939_speed_csv.write("time, speed\n")
  
  # j1939_rpm_csv = open(home_folder+"/j1939_rpm-%s.csv"%current_time, "w+")
  # j1939_rpm_csv.write("time, engine_rpm\n")
  

# def close_file():
  # global ndt_stat_csv, ndt_csv, imu_csv, gnss_csv, steering_cmd_csv, steering_report_csv, throttle_cmd_csv, throttle_report_csv, brake_cmd_csv, brake_report_csv, j1939_speed_csv, j1939_rpm_csv
  
  # ndt_stat_csv.flush()
  # ndt_stat_csv.close()
  # imu_csv.flush()
  # imu_csv.close()
  # ndt_csv.flush()
  # ndt_csv.close()
  # gnss_csv.flush()
  # gnss_csv.close()
  # steering_cmd_csv.flush()
  # steering_cmd_csv.close()
  # steering_report_csv.flush()
  # steering_report_csv.close()
  # throttle_cmd_csv.flush()
  # throttle_cmd_csv.close()
  # throttle_report_csv.flush()
  # throttle_report_csv.close()
  # brake_cmd_csv.flush()
  # brake_cmd_csv.close()
  # brake_report_csv.flush()
  # brake_report_csv.close()
  # j1939_speed_csv.flush()
  # j1939_speed_csv.close()
  # j1939_rpm_csv.flush()
  # j1939_rpm_csv.close()
  
def main():

  # rospy.init_node("csv_generation")
  rospy.init_node("behavior_record")
  home_folder = rospy.get_param("~home_folder")
  #rospy.Subscriber( "/start_sortie", Bool, start_callback)
  #rospy.Subscriber("/end_sortie", Bool, shut_down_callback,queue_size=10)
  # rospy.Subscriber("/ndt_stat", ndt_stat, ndt_stat_callback,queue_size=10)
  # rospy.Subscriber("/gps/imu/raw", Imu, imu_callback,queue_size=125)
  # rospy.Subscriber("/ndt_pose", PoseStamped, ndt_pose_callback, queue_size=10)
  # rospy.Subscriber("/nmea_sentence", Sentence, nmea_callback, queue_size=150)
  # rospy.Subscriber("/vehicle/steering_cmd", SteeringCmd, steering_cmd_callback, queue_size=50)
  # rospy.Subscriber("/vehicle/steering_report", SteeringReport, steering_report_callback, queue_size=50)
  # rospy.Subscriber("/vehicle/throttle_cmd", ThrottleCmd, throttle_cmd_callback, queue_size=50)
  # rospy.Subscriber("/vehicle/throttle_report", ThrottleReport, throttle_report_callback, queue_size=50)
  # rospy.Subscriber("/vehicle/brake_cmd", BrakeCmd, brake_cmd_callback, queue_size=50)
  # rospy.Subscriber("/vehicle/brake_report", BrakeReport, brake_report_callback, queue_size=50)
  # rospy.Subscriber("/gBus_obd_speed", OBDSpeed, j1939_speed_callback, queue_size=10)
  # rospy.Subscriber("/gBus_obd_rpm", OBDEngineRPM, j1939_rpm_callback, queue_size=50)
  # rospy.Subscriber("/usb_cam/image_raw", OBDEngineRPM, j1939_rpm_callback, queue_size=50)
  # rospy.Subscriber("/gBus_obd_rpm", OBDEngineRPM, j1939_rpm_callback, queue_size=50)
  # rospy.Subscriber("/gBus_obd_rpm", OBDEngineRPM, j1939_rpm_callback, queue_size=50)
  #wait for stableness test
  #rospy.Subscriber("/vehicle/wheel_based_speed_report", WheelBasedSpeedReport, j1939_speed_callback, queue_size=10)
  #rospy.Subscriber("/vehicle/engine_rpm_report", EngineRPMReport, j1939_rpm_callback, queue_size=50)
  
  
  recordbag = record_bag(home_folder+"/record") 
  recordbag.enable_callback()
  #time.sleep(10)
  #recordbag.disable_callback()
  # rospy.Subscriber("/vehicle/enable",Empty,recordbag.enable_callback,queue_size=1)
  # rospy.Subscriber("/vehicle/disable",Empty,recordbag.disable_callback,queue_size=1)
  rospy.spin()
  



if __name__ == '__main__':
  main()

