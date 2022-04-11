#!/usr/bin/env python
# license removed for brevity


import rospy
import math
import time
import requests
from requests.exceptions import HTTPError

from std_msgs.msg import String
from mqtt_bridge.msg import hexcmd
from mqtt_bridge.msg import convertAPI

import sys 
sys.path.append('../src/mqtt_bridge')
from mqtt_bridge import json_message_converter as js_converter


start = time.time()

output=convertAPI()

def callback2(): 
    end = time.time()
    a= int(end-start)

    if a % 60 == 0 :
        cmdoutput =  js_converter.convert_ros_message_to_json(output)        
	try:
    	    url = 'http://192.168.43.106:8080/eventHub/json/input'
    	    headers = {'content-type':'application/json'}
            r = requests.post(url, data=cmdoutput, headers=headers)
            print(r.status_code)
            output.cmd=''
            jsonResponse = r.json()
    	    print("Entire JSON response")
    	    print(jsonResponse)

        except HTTPError as http_err:
            print('HTTP error occurred : {}'.format(http_err))
        except Exception as err:
            print('Other error occurred: {}'.format(err))



def callback(data):
    output.cmd = output.cmd + data.hexcmd
    output.consumerReceivedTime = 1607910431935
    output.destination = '1607910431935.json'
    output.producerSentTime = 1607910431935
    output.source = 'OCC_dev75ba942a9c7e_test1214'
    callback2()
    pub.publish(output)

rospy.init_node('convert_API')
sub = rospy.Subscriber('/HexCmd', hexcmd, callback)
pub = rospy.Publisher('/convertAPI', convertAPI, queue_size=20)


if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



