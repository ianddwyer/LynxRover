# -*- coding: utf-8 -*-
"""
SET THIS FILE AT ROOT OF C DRIVE FOR EASY ACCESS. RUN AS ADMIN!!!!
"""

import paho.mqtt.client as mqtt
import json
import time

global ack 
ack = False
global ack_msg 
ack_msg = ""
global ack_msg_old
ack_msg_old = ""

#print ack messages
def on_message(client, userdata, msg):
    #print(f"\n{str(msg.payload.decode())}") # then you can check the value
    time.sleep(1)
    #mqttc.loop_stop() #break once receiving ack
    ack = True
    ack_msg= f"\n{str(msg.payload.decode())}"


def on_connect(client, userdata, flags, rc):
    client.subscribe("ack")

    
motor = [0,0,0,0,0,0] 
# Define Variables
MQTT_HOST = "192.168.42.33"
MQTT_PORT = 1883
MQTT_KEEPALIVE_INTERVAL = 5
MQTT_TOPIC = "write/motor"

# Initiate MQTT Client
mqttc = mqtt.Client()

mqttc.on_message = on_message
mqttc.on_connect = on_connect

# Connect with MQTT Broker
mqttc.connect(MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE_INTERVAL)


escape = 0
while(escape<1):
    
    if ack == True and ack_msg != ack_msg_old:
        mode_sel = input()
        ack_msg_old = ack_msg

    if escape ==1:
        break

    print("\n\n\n\nSelect Input Type: \n\n\tMode Type:    'vp' \n\tMode Control: 'io' \n\tValue Input:  'val'")
    if ack == False:
        mode_sel = input()
        
    if mode_sel == "val":
        print("Start Sequence of 6:")
        print("input 'x' to escape at any time")
        for ii in range(len(motor)): 
            print(f"Vel/Pos for Motor {ii}:")
            motor[ii] = str(input())
            if motor[ii]=="x": 
                escape = 1
                break
        if escape ==1:
            break
        else:
            print(f"Array passed to controller: {motor}")
            print("End Squence \n")
            write_motorValues = json.dumps({"M0": [f"{motor[0]}",f"{motor[1]}",f"{motor[2]}",f"{motor[3]}",f"{motor[4]}",f"{motor[5]}"]})
            #print(write_motorValues)
            mqttc.publish("write/motor", write_motorValues)
            write_motorValues = json.dumps({"M0": [f"{motor[0]}",f"{motor[1]}",f"{motor[2]}",f"{motor[3]}",f"{motor[4]}",f"{motor[5]}"]})
            mqttc.publish("write/motor", write_motorValues)
            mqttc.loop_start()#grab ack from motor
            time.sleep(2)
            
    elif mode_sel == "vp":
        print("\n\nInput 'velocity' or 'position'")
        pos_vel = input()
        if pos_vel == "velocity":
            mqttc.publish("mode/vel", "")
            mqttc.loop_start()#grab ack from motor
            time.sleep(2)
        elif pos_vel == "position":
            mqttc.publish("mode/pos", "")
            mqttc.loop_start()#grab ack from motor
            time.sleep(2)
        else:
            escape =1
            
    elif mode_sel == "io":
        print("\n\nInput 'enter', 'exit', or 'reset' for mode")
        enter_exit = input()
        if enter_exit == "enter":
            mqttc.publish("mode/enter", "")
            mqttc.loop_start()#grab ack from motor
            time.sleep(3)
        elif enter_exit == "exit":
            mqttc.publish("mode/exit", "")
            mqttc.loop_start()#grab ack from motor
            time.sleep(4)
        elif enter_exit == "reset": #USE ONLY WHEN EXITED FROM MOTOR MODE
            mqttc.publish("restart", "")
            time.sleep(4)
        else:
            escape =1
    else:
        escape =1
        
    


