cmake_minimum_required(VERSION 2.8.3)
project(mqtt_bridge)

find_package(catkin REQUIRED COMPONENTS 
   rospy 
   roscpp 
   message_generation
   geometry_msgs
   sensor_msgs
   autoware_msgs
)

catkin_python_setup()

add_message_files(
   FILES
#Brian
   Imu.msg
   ImuWst.msg
   DO.msg
   Obj.msg
   Roi.msg
   DetectedObjectWst.msg
   SRR.msg
   ESR.msg
#Ken
   Gnss.msg
   GPSFixWst.msg
   Radar.msg
   RadarTrackWst.msg
#Yeehou
   BrakeReportWst.msg
   Ecu_BrakeReport.msg
   Ecu_FuelLevelReport.msg
   Ecu_GearReport.msg
   Ecu_SteeringReport.msg
   Ecu_ThrottleReport.msg
   Ecu_TurnSignalCmd.msg
   Ecu_VehicleSpeed.msg
   FuelLevelReportWst.msg
   GearReportWst.msg
   SteeringReportWst.msg
   ThrottleInfoReportWst.msg
   TurnSignalCmdWst.msg
   VehicleSpeedWst.msg
#SH
   BrakeLightReportWst.msg
   Ecu_BrakeLightReport.msg
   DrivingModeWst.msg
   Ecu_DrivingMode.msg
   EmergencyStopWst.msg
   Ecu_EmergencyStop.msg
   SensorStatusWst.msg
   Status.msg
   Summarize.msg
   hexcmd.msg
   convertAPI.msg
#cwc_lilee
  Lilee_gps.msg
  Lilee_gnss.msg
  Lilee_SPaT.msg
  Lilee_SPaT_version.msg
  Lilee_SPaT_id.msg
  Lilee_SPaT_states.msg
  Lilee_SPaT_stateTimeSpeed.msg
  Lilee_SPaT_timing.msg
  Lilee_motc.msg
  Lilee_motc_listsignalstatus.msg
  Lilee_DO.msg
  Lilee_DO_array.msg
 )


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  autoware_msgs
)

catkin_package(CATKIN_DEPENDS 
   message_runtime
   roscpp
   rospy
   std_msgs
)



install(PROGRAMS
  scripts/mqtt_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###Brian
install(PROGRAMS
  scripts/transfer_imu.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/transfer_DO.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS
  scripts/behavior_record.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
###Ken
install(PROGRAMS
  scripts/transfer_gnss.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/transfer_radar_new.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/transfer_radar_old.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
###Yeehou

install(PROGRAMS
  scripts/transfer_throttle.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/transfer_brake.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/transfer_fuel.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/transfer_gear.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/transfer_steering.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/transfer_turn.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/transfer_vehicle_speed.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#SH
install(PROGRAMS
  scripts/summarize.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/transfer_emergency_stop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/transfer_driving_mode.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/transfer_sensor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/transfer_brake_light.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/transfer_hex.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/convert_API.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/convert_API_json.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###cwc
install(PROGRAMS
  scripts/transfer_Lilee_gps.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
###cwc

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

