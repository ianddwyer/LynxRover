# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
from paho.mqtt import client as mqtt


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')

        self.publisher_ = self.create_publisher(String, 'topic', 10)
        #self.publisher_ = self.create_publisher(String, 'MotorVPT', 10)
        #self.publisher_ = self.create_publisher(String, 'GPS', 10)
        #self.publisher_ = self.create_publisher(String, 'IMU', 10)
        #self.publisher_ = self.create_publisher(String, 'USonic', 10)


        self.message0= ""
        self.message00= ""
        self.message1= ""
        self.message2= ""

        def on_connect(client, userdata, flags, rc):
            self.client.subscribe(self.sub_topics)

        def on_disconnect():
            self.client.reconnect()

        def on_message(client,usrdata,msg):
            if str(msg.topic) == "read/driveMotors1":
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
                read_motor2 = payload["VPTMotor2"]
                vel2 = read_motor2[0]
                pos2 = read_motor2[1]
                torq2 = read_motor2[2]
                strM2 = f"\nMOTOR 2:: \n\t vel: {vel2} \n\t pos: {pos2} \n\t torq: {torq2} \n"
                #print(f"Motor 1 \n\t vel: {vel1} \n\t pos: {pos1} \n\t torq: {torq1} \n")

                self.message0 = strM0 + strM1 + strM2 
                flagMSG0 = True

            elif str(msg.topic) == "read/driveMotors2":
                payload = json.loads(msg.payload) 
                read_motor3 = payload["VPTMotor3"]
                vel3 = read_motor3[0]
                pos3 = read_motor3[1]
                torq3 = read_motor3[2]
                strM3 = f"\nMOTOR 3:: \n\t vel: {vel3} \n\t pos: {pos3} \n\t torq: {torq3} \n"
                #print(f"Motor 1 \n\t vel: {vel1} \n\t pos: {pos1} \n\t torq: {torq1} \n")
                read_motor4 = payload["VPTMotor4"]
                vel4 = read_motor4[0]
                pos4 = read_motor4[1]
                torq4 = read_motor4[2]
                strM4 = f"\nMOTOR 4:: \n\t vel: {vel4} \n\t pos: {pos4} \n\t torq: {torq4} \n"
                #print(f"Motor 1 \n\t vel: {vel1} \n\t pos: {pos1} \n\t torq: {torq1} \n")
                read_motor5 = payload["VPTMotor5"]
                vel5 = read_motor5[0]
                pos5 = read_motor5[1]
                torq5 = read_motor5[2]
                strM5 = f"\nMOTOR 5:: \n\t vel: {vel5} \n\t pos: {pos5} \n\t torq: {torq5} \n"
                #print(f"Motor 1 \n\t vel: {vel1} \n\t pos: {pos1} \n\t torq: {torq1} \n")
                
                self.message00 = strM3 + strM4 + strM5
                flagMSG0 = True


            elif str(msg.topic) == "read/GPS":
                payload = json.loads(msg.payload)
                longitude = payload["Long"]
                longitude = longitude[0] * (10 ** -7)
                latitude = payload['Lat']
                latitude = latitude[0] * (10 ** -7)
                strM0 = f"\n\nGPS DATA:: \n\t Longitude: {longitude} [deg]\n\t Latitude:   {latitude} [deg]"
                self.message1 = strM0

                flagMSG1 = True

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

                flagMSG2 = True

            #if (flagMSG0 and flagMSG1 and flagMSG2): #uses slowest update rate, but reaqures all to be publishing
            msgs = String()
            msgs.data = str(self.message0 + self.message00 + self.message1 + self.message2)
            self.publisher_.publish(msgs)
            self.get_logger().info('"%s"' % msgs.data)







        clientID = "ROS_server"
        broker = "192.168.42.33"
        port = 1883
        timeout = 60
        self.sub_topics = [("read/driveMotors1",2),("read/driveMotors2",2),("read/GPS",2),("read/sensors",2)]
        self.client = mqtt.Client(clientID)
        self.client.connect(broker, port, timeout)
        self.client.on_connect = on_connect
        self.client.on_message = on_message
        self.client.loop_start()
        # To Publish:
        # Important - send mode select (vel or pos) and use "enter mode" to start the motors. Must use exit to stop motors!
        # The motors are in position or velocity using topic "mode/pos" or "mode/vel" with empty payload
        # Enter mode activated at topic "mode/enter" with no payload
        # exit mode is activated at topic "mode/exit" with no payload
        # the motor array is at topic "write/motor"
        # motor_value = [str(mot0),str(mo1),str(mot2),str(mot3),str(mot4),str(mot5)] #these are either position or velocity values loaded depending on mode (uses same array)
        # write_motorValues = json.dumps({"DriveMotors": *motor_value}) #pack values into json for synchronous update
        # client.publish("write/motor", write_motorValues)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
