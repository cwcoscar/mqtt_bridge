#!/bin/bash
#iot.omega.smartiply.com:3001

while [ "true" ]
do

  lidar=$(rostopic list | grep -c /points_raw)
  if [ $lidar != 0 ]; then
    now=$(date +%s%3N)
	  mosquitto_pub -h iot.stois.nchc.tw -k 60 -p 3054 -t vehicle/report/558e429c54ca -m\
    '{"vid": "558e429c54ca","lidar":[{"uid":"lidar_front","status":3,"timestamp":'$now',"source_time":'$now'}]}'\
     --cafile ./../key/cert_mkz/ca.crt --cert ./../key/cert_mkz/client.crt --key ./../key/cert_mkz/client.key
    #echo lidar_on
  else
    now=$(date +%s%3N)
	  mosquitto_pub -h iot.stois.nchc.tw -k 60 -p 3054 -t vehicle/report/558e429c54ca -m\
    '{"vid": "558e429c54ca","lidar":[{"uid":"lidar_front","status":0,"timestamp":'$now',"source_time":'$now'}]}'\
     --cafile ./../key/cert_mkz/ca.crt --cert ./../key/cert_mkz/client.crt --key ./../key/cert_mkz/client.key
    #echo lidar_off
  fi

  cam=$(rostopic list | grep -c /usb_cam.*)
  if [ $cam > 2 ]; then
    now=$(date +%s%3N)
	  mosquitto_pub -h iot.stois.nchc.tw -k 60 -p 3054 -t vehicle/report/558e429c54ca -m\
    '{"vid": "558e429c54ca","camera":[{"uid":"usb_cam","status":3,"timestamp":'$now',"source_time":'$now'}]}'\
     --cafile ./../key/cert_mkz/ca.crt --cert ./../key/cert_mkz/client.crt --key ./../key/cert_mkz/client.key
    #echo camera_on
  else
    now=$(date +%s%3N)
	  mosquitto_pub -h iot.stois.nchc.tw -k 60 -p 3054 -t vehicle/report/558e429c54ca -m\
    '{"vid": "558e429c54ca","camera":[{"uid":"usb_cam","status":0,"timestamp":'$now',"source_time":'$now'}]}'\
     --cafile ./../key/cert_mkz/ca.crt --cert ./../key/cert_mkz/client.crt --key ./../key/cert_mkz/client.key
    #echo camera_off
  fi
   
done
