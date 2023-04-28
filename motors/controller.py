from evdev import InputDevice, categorize, ecodes
import paho.mqtt.client as mqtt
import json
import time



# Declare Connection Data
MQTT_HOST = "192.168.42.33"
MQTT_PORT = 1883
MQTT_KEEPALIVE_INTERVAL = 45
MQTT_TOPIC = "write/motor"



# Initialize MQTT Client and Connect
mqttc = mqtt.Client("aewrwefcweefc")
mqttc.connect(MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE_INTERVAL)



# Pack 6 motor values into JSON
motor = [0,0,0,0,0,0] #works for position and velocity. Only expecting velocity from drive motors

#creates object 'gamepad' to store the data
#you can call it whatever you like
gamepad = InputDevice('/dev/input/event7')

#prints out device info at start
print(gamepad)
up = 17
down = 17

maxSpd = 15
ltrig = 294 #forward left
rtrig = 295 #forward right
ltrig2 = 292 #back left
rtrig2 = 293 #back right
start = 4
select =4
square = 288
circle = 290
touchpad = 301
home = 300
triangle = 291
x = 289
left = 16
right = 16


def publishDriveMQTT():
    write_motorValues = json.dumps({"M0": [f"{motor[0]}",f"{motor[1]}",f"{motor[2]}",f"{motor[3]}",f"{motor[4]}",f"{motor[5]}"]})
    mqttc.publish("write/motor", write_motorValues,retain=True)  
def publishArmMQTT():
    write_motorValues = json.dumps({"M0": [f"{motor[0]}",f"{motor[1]}",f"{motor[2]}",f"{motor[3]}",f"{motor[4]}",f"{motor[5]}"]})
    mqttc.publish("write/motor/arm", write_motorValues,retain=True)  
def on_disconnect(client,userdata,rc):
    print("MQTT Disconnected")
    mqttc.reconnect()
def on_connect(client, userdata, flags, rc):
    print(f"rc= {rc} ; client = {client}")

mqttc.on_connect = on_connect
mqttc.on_disconnect = on_disconnect
 
speed_default = 7
speed = speed_default #default
speed_limit = 14 #limit

armSpeed = 7

mode = "" 

#evdev takes care of polling the controller in a loop
for event in gamepad.read_loop():

    #print(event) #use this to learn how the buttons are mapped
    if event.code == home and event.value ==1:
        print("\nArm Mode")
        mqttc.publish("mode/exit","")
        print("->exiting all motors")
        time.sleep(6)
        mqttc.publish("mode/vel/arm")
        time.sleep(2)
        mqttc.publish("mode/enter/arm","")
        print("->entering arm motor mode")
        time.sleep(4)
        mode = "arm"
        print("->Entered Arm Mode\n")
        mqttc.publish("restart","") # restarts drive MCU for safety
        
    if event.code == touchpad and event.value ==1:
        print("\nDrive Mode")
        mqttc.publish("mode/exit/arm","")
        mqttc.publish("restart/arm","")
        mqttc.publish("restart","")
        print("->exiting all motors")
        time.sleep(6)
        mqttc.publish("mode/vel")
        time.sleep(2)
        mqttc.publish("mode/enter","")
        print("->entering drive motor mode")
        time.sleep(4)
        mode = "drive"
        print("->Entered Drive Mode\n")
        mqttc.publish("restart/arm","") # restarts arm MCU for safety
        
        
        
    
    if mode == "drive":
        
        if event.code == up and event.value ==-1: 
            print("up-on")
            motor = [speed,-speed,speed,-speed,speed,-speed]
            publishDriveMQTT()
        if event.code == up and event.value ==0:
			#print("up-off")
            motor = [0,0,0,0,0,0]
            publishDriveMQTT()
        if event.code == down and event.value ==1:
            print("down-on")
            motor = [-speed,speed,-speed,speed,-speed,speed]
            publishDriveMQTT()
        if event.code == down and event.value ==0:
 			#print("down-off")	
            motor = [0,0,0,0,0,0]
            publishDriveMQTT()
            
        if event.code == rtrig and event.value ==1:
            print("right-on")
            motor = [0,-speed,0,-speed,0,-speed]
            publishDriveMQTT()
        if event.code == rtrig and event.value ==0:
            print("right-off")
            motor = [0,0,0,0,0,0]
            publishDriveMQTT()
        if event.code == ltrig and event.value ==1:
            print("left-on")
            motor = [speed,0,speed,0,speed,0]
            publishDriveMQTT()
        if event.code == ltrig and event.value ==0:
            print("left-off")
            motor = [0,0,0,0,0,0]
            publishDriveMQTT()
            
        if event.code == rtrig2 and event.value ==1:
            print("right-on")
            motor = [0,speed,0,speed,0,speed]
            publishDriveMQTT()
        if event.code == rtrig2 and event.value ==0:
            print("right-off")
            motor = [0,0,0,0,0,0]
            publishDriveMQTT()
        if event.code == ltrig2 and event.value ==1:
            print("left-on")
            motor = [-speed,0,-speed,0,-speed,0]
            publishDriveMQTT()
        if event.code == ltrig2 and event.value ==0:
            print("left-off")
            motor = [0,0,0,0,0,0]
            publishDriveMQTT()
            
        if event.code == select and event.value ==589833:
            mqttc.publish("mode/exit","")
            print("\nexiting motor mode\n")
            time.sleep(6)
            mqttc.publish("restart","")
            mqttc.publish("restart/arm","")
            mode = ""
            
        if event.code == square and event.value ==1:
            speed-=1
            print(f"speed down: {speed}")
        if event.code == circle and event.value ==1:
            speed+=1 
            print(f"speed up: {speed}")
	       
            
           
            
           
            
    if mode == "arm":
        
        if event.code == up and event.value ==-1: 
            print("shoulder up")
            motor = [armSpeed,0,0,0,0,0]
            publishArmMQTT()
        if event.code == up and event.value ==0: 
            motor = [0,0,0,0,0,0]
            publishArmMQTT()
        if event.code == down and event.value ==1:
            print("shoulder down")
            motor = [-armSpeed,0,0,0,0,0]
            publishArmMQTT()
        if event.code == down and event.value ==0: 
            motor = [0,0,0,0,0,0]
            publishArmMQTT()
            
        if event.code == right and event.value ==-1:     
            print("shoulder clockwise")
            motor = [0,-armSpeed,0,0,0,0]
            publishArmMQTT()
        if event.code == right and event.value ==0:     
            motor = [0,0,0,0,0,0]
            publishArmMQTT()
        if event.code == left and event.value ==1:
            print("shoulder counter-clockwise")
            motor = [0,armSpeed,0,0,0,0]
            publishArmMQTT()
        if event.code == left and event.value ==0:     
            motor = [0,0,0,0,0,0]
            publishArmMQTT()
            
        if event.code == triangle and event.value ==1: 
            print("wrist up")
            motor = [0,0,0,0,armSpeed,0]
            publishArmMQTT()
        if event.code == triangle and event.value ==0: 
            motor = [0,0,0,0,0,0]
            publishArmMQTT()
        if event.code == x and event.value ==1:      
            print("wrist down")
            motor = [0,0,0,0,-armSpeed,0]
            publishArmMQTT()
        if event.code == x and event.value ==0: 
            motor = [0,0,0,0,0,0]
            publishArmMQTT()
            
        if event.code == triangle and event.value ==1:    
            print("elbow up")
            motor = [0,0,armSpeed,0,0,0]
            publishArmMQTT()
        if event.code == triangle and event.value ==0:    
            motor = [0,0,0,0,0,0]
            publishArmMQTT()
        if event.code == x and event.value ==1:      
            print("elbow down")
            motor = [0,0,-armSpeed,0,0,0]
            publishArmMQTT()
        if event.code == x and event.value ==0:    
            motor = [0,0,0,0,0,0]
            publishArmMQTT()
            
        if event.code == rtrig2 and event.value ==1:
            print("rotate wrist clock")
            motor = [0,0,0,0,1,0,0]
            publishArmMQTT()
        if event.code == rtrig2 and event.value ==0:
            motor = [0,0,0,0,0,0]
            publishArmMQTT()
        if event.code == ltrig2 and event.value ==1:
            HvelWrist = -4
            print("rotate wrist counterclock")
            motor = [0,0,0,0,-1,0,0]
            publishArmMQTT()
        if event.code == ltrig2 and event.value ==0:
            motor = [0,0,0,0,0,0]
            publishArmMQTT()
            
        if event.code == rtrig and event.value ==1:
            print("claw tighten")
            motor = [0,0,0,0,0,armSpeed]
            publishArmMQTT()
        if event.code == rtrig and event.value ==0:
            motor = [0,0,0,0,0,0]
            publishArmMQTT()
        if event.code == ltrig and event.value ==1:
            print("claw loosen")
            motor = [0,0,0,0,0,-armSpeed]
            publishArmMQTT()
        if event.code == ltrig and event.value ==0:
            motor = [0,0,0,0,0,0]
            publishArmMQTT()

        if event.code == select and event.value ==589833:
            mqttc.publish("mode/exit","")
            print("\nexiting all motors\n")
            time.sleep(6)
            mqttc.publish("restart/arm","")
            mqttc.publish("restart","")
            mode = ""
        

        	
    if speed >=speed_limit:
        speed = speed_limit #limit
    if speed<=speed_default:
            speed=speed_default#prevents reverse velocity
          
        
        
    
