# -*- coding: utf-8 -*-
from __future__ import absolute_import

from abc import ABCMeta, abstractmethod

import inject
import paho.mqtt.client as mqtt
import rospy
import json

from mqtt_bridge import json_message_converter as js_converter #Brian
from mqtt_bridge import message_converter as msg_converter     #Brian
from .util import lookup_object, extract_values, populate_instance


def create_bridge(factory, msg_type, topic_from, topic_to, **kwargs):
    u""" bridge generator function

    :param (str|class) factory: Bridge class
    :param (str|class) msg_type: ROS message type
    :param str topic_from: incoming topic path
    :param str topic_to: outgoing topic path
    :param (float|None) frequency: publish frequency
    :return Bridge: bridge object
    """
    if isinstance(factory, basestring):
        factory = lookup_object(factory)
    if not issubclass(factory, Bridge):
        raise ValueError("factory should be Bridge subclass")
    if isinstance(msg_type, basestring):
        msg_type = lookup_object(msg_type)
    if not issubclass(msg_type, rospy.Message):
        raise TypeError(
            "msg_type should be rospy.Message instance or its string"
            "reprensentation")
    return factory(
        topic_from=topic_from, topic_to=topic_to, msg_type=msg_type, **kwargs)


class Bridge(object):
    u""" Bridge base class

    :param mqtt.Client _mqtt_client: MQTT client
    :param _serialize: message serialize callable
    :param _deserialize: message deserialize callable
    """
    __metaclass__ = ABCMeta

    _mqtt_client = inject.attr(mqtt.Client)
    _serialize = inject.attr('serializer')
    _deserialize = inject.attr('deserializer')
    _extract_private_path = inject.attr('mqtt_private_path_extractor')


class RosToMqttBridge(Bridge):
    u""" Bridge from ROS topic to MQTT

    :param str topic_from: incoming ROS topic path
    :param str topic_to: outgoing MQTT topic path
    :param class msg_type: subclass of ROS Message
    :param (float|None) frequency: publish frequency
    """

    def __init__(self, topic_from, topic_to, msg_type, frequency=None):
        self._topic_from = topic_from
        self._topic_to = self._extract_private_path(topic_to)
        self._last_published = rospy.get_time()
        self._interval = 0 if frequency is None else 1.0 / frequency
        rospy.Subscriber(topic_from, msg_type, self._callback_ros)

    def _callback_ros(self, msg):
        rospy.logdebug("ROS received from {}".format(self._topic_from))
        now = rospy.get_time()
        if now - self._last_published > self._interval:
            self._publish(msg)
            self._last_published = now

    def _publish(self, msg):
        #payload = bytearray(self._serialize(extract_values(msg)))
        #payload = extract_values(msg)['data']
        #dict = {'a': 1, 'b': 2, 'b': '3'}
        data_out=js_converter.convert_ros_message_to_json(msg)  #Brain
        payload=data_out
        self._mqtt_client.publish(topic=self._topic_to , payload=payload, qos=0)


class MqttToRosBridge(Bridge):
    u""" Bridge from MQTT to ROS topic

    :param str topic_from: incoming MQTT topic path
    :param str topic_to: outgoing ROS topic path
    :param class msg_type: subclass of ROS Message
    :param (float|None) frequency: publish frequency
    :param int queue_size: ROS publisher's queue size
    """

    def __init__(self, topic_from, topic_to, msg_type, frequency=None,
                 queue_size=10):
        self._topic_from = self._extract_private_path(topic_from)
        self._topic_to = topic_to
        self._msg_type = msg_type
        self._queue_size = queue_size
        self._last_published = rospy.get_time()
        self._interval = None if frequency is None else 1.0 / frequency
	#self._mqtt_client.subscribe(self._topic_from)
        self._mqtt_client.subscribe(topic_from)
	#self._mqtt_client.message_callback_add(self._topic_from, self._callback_mqtt)
        self._mqtt_client.message_callback_add(topic_from, self._callback_mqtt)
        self._publisher = rospy.Publisher(
            self._topic_to, self._msg_type, queue_size=self._queue_size)

	
    def _callback_mqtt(self, client, userdata, mqtt_msg):
        u""" callback from MQTT

        :param mqtt.Client client: MQTT client used in connection
        :param userdata: user defined data
        :param mqtt.MQTTMessage mqtt_msg: MQTT message
        """
        rospy.logdebug("MQTT received from {}".format(mqtt_msg.topic))
        now = rospy.get_time()

        if (self._interval is None) or (now - self._last_published >= self._interval):
            try:
     
                ros_msg = self._create_ros_message(mqtt_msg)
 
                self._publisher.publish(ros_msg)
                self._last_published = now
            except Exception as e:
                rospy.logerr(e)

        


    def _create_ros_message(self, mqtt_msg):
        u""" create ROS message from MQTT payload

        :param mqtt.Message mqtt_msg: MQTT Message
        :return rospy.Message: ROS Message
        """
        #msg_dict = self._deserialize(mqtt_msg.payload)
        #msg_dict = self._deserialize(self._serialize({'data':mqtt_msg.payload}))

        msg_dict=json.loads(mqtt_msg.payload)
        dict={}
        for key, value in msg_dict.items():
            if key == 'imu':# add the specific case if the dict need to be included in list type //Brian
                dict[key] = value[0]
            elif key == 'gnss':
                dict[key] = value[0]
            elif key == 'ecu':
                dict[key] = value[0]
            #elif key == 'DO':
                #dict[key] = value[0]
            elif key == 'radar':
                dict[key] = value[0]
            # elif key == 'states':
            #     dict_2={}
            #     # dict[key] = value[0]
            #     # print(value)
            #     for i in value:
            #         for key_2, value_2 in i.items():
            #             # print(key_2)
            #             # print(value_2)
            #             if key_2 == 'stateTimeSpeed':
            #                 dict_2[key_2] = value_2[0]
            #             else:
            #                 dict_2[key_2] = value_2
            #     dict[key] = dict_2
            else:
                dict[key] = value
                
        return populate_instance(dict, self._msg_type())
   

        #return js_converter.convert_json_to_ros_message(self._msg_type,mqtt_msg.payload)



__all__ = ['register_bridge_factory', 'create_bridge', 'Bridge',
           'RosToMqttBridge', 'MqttToRosBridge']
