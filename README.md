# mqtt_bridge

mqtt_bridge provides a functionality to bridge between ROS and MQTT in bidirectional.


## Principle

`mqtt_bridge` uses ROS message as its protocol. Messages from ROS are serialized by json (or messagepack) for MQTT, and messages from MQTT are deserialized for ROS topic. So MQTT messages should be ROS message compatible. (We use `rosbridge_library.internal.message_conversion` for message conversion.)

This limitation can be overcome by defining custom bridge class, though.

## Requirement for connecting and subscribing topics from Lilee and Wistron broker

### Install python modules

#### check
- [pymap3d >= 1.5.1](https://pypi.org/project/pymap3d/1.5.1/)

`$ pip install pymap3d==1.5.1`
- [inject >= 3.5.4](https://pypi.org/project/Inject/)

`$ pip install Inject==3.5.4`
- [paho-mqtt >= 1.3.1](https://pypi.org/project/paho-mqtt/)
        
`$ pip install paho-mqtt==1.3.1`
- [msgpack >= 1.0.2](https://pypi.org/project/msgpack/)

`$ pip install msgpack==1.0.2`
- [bson>=0.5.2](https://pypi.org/project/bson/)

`$ pip install bson==0.5.2`
- [pymongo>=3.8.0](https://pypi.org/project/pymongo/)

`$ pip install pymongo==3.8.0`

### Make scripts to be executable
    - `$ cd /home/meclab/catkin_ws/src/mqtt_bridge/scripts`
    - `$ chmod +x *`
    
### Add package `radar_msgs`
[Reference](http://docs.ros.org/en/kinetic/api/radar_msgs/html/index-msg.html)

[Download link](https://drive.google.com/drive/folders/15obW24PpzWEXgLuNPfCYz6D5MvhLGR3z?usp=sharing)
    
### **Usage Description :**
#### **From ROS publishing to MQTT :**
    - Revise the file `/home/meclab/catkin_ws/src/mqtt_bridge/config/wst_pub.yaml`
    ```yaml
    factory: mqtt_bridge.bridge:MqttToRosBridge
    msg_type: mqtt_bridge.msg:DetectedObjectWst
    *topic_from: (MQTT_topic)
    *topic_to: (ROS_topic)
    ```
    
#### **From MQTT publishing to ROS :**
    - Revise the file `/home/meclab/catkin_ws/src/mqtt_bridge/config/wst_pub.yaml`
    ```yaml
        factory: mqtt_bridge.bridge:MqttToRosBridge
        msg_type: mqtt_bridge.msg:DetectedObjectWst
        *topic_from: (ROS_topic)
        *topic_to: (MQTT_topic)
    ```
<!--     - Add subscriber in the code `/home/meclab/catkin_ws/src/mqtt_bridge/src/mqtt_bridge/app.py`
    ```python
    def _on_connect(client, userdata, flags, response_code):
        rospy.loginfo('MQTT connected')
        print('response_code', response_code)
        client.subscribe("vehicle/report/475e30c916c8")
        client.subscribe("vehicle/report/dev89dcbcc5df1c2")
        client.subscribe("vehicle/report/558e429c54ca")
        client.subscribe("roadside/smartrsu/4t68QO37WBd1pf")
        **client.subscribe(MQTT_topic)**
    ``` -->

## Demo of setting up your broker

### prepare MQTT broker and client

```
$ sudo apt-get install mosquitto mosquitto-clients
```

### Install python modules

```bash
$ pip install -r requirements.txt
```

### launch node

``` bash
$ roslaunch mqtt_bridge demo.launch
```

Publish to `/ping`,

```
$ rostopic pub /ping std_msgs/Bool "data: true"
```

and see response to `/pong`.

```
$ rostopic echo /pong
data: True
---
```

Publish "hello" to `/echo`,

```
$ rostopic pub /echo std_msgs/String "data: 'hello'"
```

and see response to `/back`.

```
$ rostopic echo /back
data: hello
---
```

You can also see MQTT messages using `mosquitto_sub`

```
$ mosquitto_sub -t '#'
```

## Usage

parameter file (config.yaml):

``` yaml
mqtt:
  client:
    protocol: 4      # MQTTv311
  connection:
    host: localhost
    port: 1883
    keepalive: 60
bridge:
  # ping pong
  - factory: mqtt_bridge.bridge:RosToMqttBridge
    msg_type: std_msgs.msg:Bool
    topic_from: /ping
    topic_to: ping
  - factory: mqtt_bridge.bridge:MqttToRosBridge
    msg_type: std_msgs.msg:Bool
    topic_from: ping
    topic_to: /pong
```

launch file:

``` xml
<launch>
  <node name="mqtt_bridge" pkg="mqtt_bridge" type="mqtt_bridge_node.py" output="screen">
    <rosparam file="/path/to/config.yaml" command="load" />
  </node>
</launch>
```


## Configuration

### mqtt

Parameters under `mqtt` section are used for creating paho's `mqtt.Client` and its configuration.

#### subsections

* `client`: used for `mqtt.Client` constructor
* `tls`: used for tls configuration
* `account`: used for username and password configuration
* `message`: used for MQTT message configuration
* `userdata`: used for MQTT userdata configuration
* `will`: used for MQTT's will configuration

See `mqtt_bridge.mqtt_client` for detail.

### mqtt private path

If `mqtt/private_path` parameter is set, leading `~/` in MQTT topic path will be replaced by this value. For example, if `mqtt/pivate_path` is set as "device/001", MQTT path "~/value" will be converted to "device/001/value".

### serializer and deserializer

`mqtt_bridge` uses `json` as a serializer in default. But you can also configure other serializers. For example, if you want to use messagepack for serialization, add following configuration.

``` yaml
serializer: msgpack:dumps
deserializer: msgpack:loads
```

### bridges

You can list ROS <--> MQTT tranfer specifications in following format.

``` yaml
bridge:
  # ping pong
  - factory: mqtt_bridge.bridge:RosToMqttBridge
    msg_type: std_msgs.msg:Bool
    topic_from: /ping
    topic_to: ping
  - factory: mqtt_bridge.bridge:MqttToRosBridge
    msg_type: std_msgs.msg:Bool
    topic_from: ping
    topic_to: /pong
```

* `factory`: bridge class for transfering message from ROS to MQTT, and vise versa.
* `msg_type`: ROS Message type transfering through the bridge.
* `topic_from`: topic incoming from (ROS or MQTT)
* `topic_to`: topic outgoing to (ROS or MQTT)

Also, you can create custom bridge class by inheriting `mqtt_brige.bridge.Bridge`.


## License

This software is released under the MIT License, see LICENSE.txt.
