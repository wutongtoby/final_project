import serial
import time
import paho.mqtt.client as paho
import numpy as np

# MQTT setting---------------------------------------------------
mqttc = paho.Client()
# Settings for connection

host = "localhost"
topic= "mbed"
port = 1883

# Callbacks
def on_connect(self, mosq, obj, rc):
    print("Connected rc: " + str(rc))

def on_message(mosq, obj, msg):
    print("[Received] Topic: " + msg.topic + ", Message: " + str(msg.payload) + "\n")

def on_subscribe(mosq, obj, mid, granted_qos):
    print("Subscribed OK")

def on_unsubscribe(mosq, obj, mid, granted_qos):
    print("Unsubscribed OK")

# Set callbacks
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
mqttc.on_unsubscribe = on_unsubscribe

# Connect and subscribe
print("Connecting to " + host + "/" + topic)
mqttc.connect(host, port=1883, keepalive=60)
mqttc.subscribe(topic, 0)
#------------------------------------------------------------
# XBee setting
serdev = '/dev/ttyUSB0'
s = serial.Serial(serdev, 9600)

# read and send the log data 
while True:
    line = s.readline()
    print(line)
    mqttc.publish(topic, str(line))
