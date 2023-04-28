
import json
from paho.mqtt import client as mqtt
import time

f0 = False
f1 = False
f2 = False



class read_MQTT:
    def __init__(self, f0,f1,f2): 
        
        self.flag0 = f0
        self.flag1 = f1
        self.flag2 = f2
     
        self.clientID = "ROS_server"
        self.broker = "76.25.64.215"
        self.port = 1883
        self.timeout = 60
        self.sub_topics = [("read/driveMotors",2),("read/GPS",2),("read/sensors",2)]
        self.client = mqtt.Client(self.clientID)
        self.client.connect(self.broker, self.port, self.timeout)

        
        self.message0= ""
        self.message0= ""
        self.message1= ""
        self.message2= ""
        

        def on_connect(client, userdata, flags, rc):
            self.client.subscribe(self.sub_topics)
        
        def on_disconnect():
            self.client.reconnect()
        
        def on_message(client,usrdata,msg):
            
            if str(msg.topic) == "read/driveMotors":
                # print(msg.topic)
                # print(msg.payload.decode()) #
                payload = json.loads(msg.payload)  # you can use json.loads to convert string to json
                read_motor0 = payload["VPTMotor0"]
                vel0 = read_motor0[0]
                pos0 = read_motor0[1]
                torq0 = read_motor0[2]
                strM0 = f"\n\nMOTOR 0:: \n\t vel: {vel0} \n\t pos: {pos0} \n\t torq: {torq0}"
                #print(strM0)
                read_motor1 = payload["VPTMotor1"]
                vel1 = read_motor1[0]
                pos1 = read_motor1[1]
                torq1 = read_motor1[2]
                strM1 = f"\nMOTOR 1:: \n\t vel: {vel1} \n\t pos: {pos1} \n\t torq: {torq1} \n"
                #print(f"Motor 1 \n\t vel: {vel1} \n\t pos: {pos1} \n\t torq: {torq1} \n")
                self.message0 = strM0 + strM1
                self.flag0 = True
                
        
        
            elif str(msg.topic) == "read/GPS":
                payload = json.loads(msg.payload)
                longitude = payload["Long"]
                longitude = longitude[0] * (10 ** -7)
                latitude = payload['Lat']
                latitude = latitude[0] * (10 ** -7)
                strM0 = f"\n\nGPS DATA:: \n\t Longitude: {longitude} [deg]\n\t Latitude:   {latitude} [deg]"
                self.message1 = strM0
                self.flag1 = True
        
                
        
            elif str(msg.topic) == "read/sensors":
                payload = json.loads(msg.payload)
                pitch = int(payload["pitch"][0])
                roll = int(payload["roll"][0])
                heading = int(payload["heading"][0])
                vibration = int(payload["vibration"][0])
                strM0 = f"\nIMU:: \n\t pitch:     {pitch} [deg]\n\t roll:       {roll} [deg]\n\t heading:    {heading} [deg]\n\t vibration:  {vibration}"
                ultrasonic = payload["ultrasonic"][0]
                strM1 = f"\nUltrasonic:: \n\t US0:        {ultrasonic} [cm]\n"
                self.message2 = strM0 + strM1
                self.flag2 = True
        
                
        

            print( str(self.message0 + self.message1 + self.message2))


        self.client.on_connect = on_connect
        self.client.on_message = on_message
        self.client.loop_forever()           

read_MQTT(f0,f1,f2)

        # To Publish:
        # Important - send mode select (vel or pos) and use "enter mode" to start the motors. Must use exit to stop motors!
        # The motors are in position or velocity using topic "mode/pos" or "mode/vel" with empty payload
        # Enter mode activated at topic "mode/enter" with no payload
        # exit mode is activated at topic "mode/exit" with no payload
        # the motor array is at topic "write/motor"
        # motor_value = [str(mot0),str(mo1),str(mot2),str(mot3),str(mot4),str(mot5)] #these are either position or velocity values loaded depending on mode (uses same array)
        # write_motorValues = json.dumps({"DriveMotors": *motor_value}) #pack values into json for synchronous update
        # client.publish("write/motor", write_motorValues)


