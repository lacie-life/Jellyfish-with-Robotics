
import rospy
from std_msgs.msg import String

import time
import sys
import paho.mqtt.client as mqtt
import paho.mqtt.subscribe as subscribe

pub = rospy.Publisher('jetbot_cmd', String, queue_size=10)
rospy.init_node('mqtt')
rate = rospy.Rate(10) # 10hz

def cmd(msg):
    pub.publish(String(msg))

def print_msg(client, userdata, message):
    print("%s : %s" % (message.topic, message.payload))
    msg = message.payload.decode('utf-8')
    print(type(msg))
    cmd(msg)

subscribe.callback(print_msg, "demo/test", hostname="www.citlab.xyz")