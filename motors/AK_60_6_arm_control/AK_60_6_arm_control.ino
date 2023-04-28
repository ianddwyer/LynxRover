// This skecth is for testing the cubemars Ak60-6 motor using arduino CAN shield
/// use serial monitor for input the commands to motor
/// o -- turns on the motor mode
/// f -- turns of the motor mode
/// u -- steps up the position increment according to fix offset, can be adjusted in loop
/// u -- steps down the position increment according to fix offset, can be adjusted in loop

#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>
#include "WiFi.h"
#include <PubSubClient.h>
#include <string.h>
#include <time.h>
#include "WiFi_Logins.h"
#include <ArduinoJson.h>
#include <ESP32_Servo.h> //accesses the Arduino Servo Library

Servo esc;  // creates servo object to control a servo


const int neopin = 16;
const int linpin = 17;


long unsigned int rxId;

// the cs pin of the version v1.4 is default to D10
                                 // Set CS pin
//network setup with static IP assigned, otherwise does not work with ubiquiti
#define SSID  WIFI_NETWORK_D
#define WIFI_PW  WIFI_PASSWORD_D
IPAddress local_IP(192, 168, 42, 83);
IPAddress gateway(192, 168, 42, 10);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);   //optional
IPAddress secondaryDNS(0, 0, 0, 0); //optional

WiFiClient MOTORS;
PubSubClient client(MOTORS);
const char* device_name = "arm_controller";

int calledID = 0; // test motor has ID 5 but calls motor index 0 for now. Change this later to prevent confusion!!!!!!

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -40.0f
#define V_MAX 40.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -15.0f
#define T_MAX 15.0f

//mode is set for each motor that is controlled before initilizing the motor mode
//mode at position corresponds to that motor number
String mode = "";
byte motID[6] =  {0x05,0x01,0x03,0x04,0x07,0x06}; //max expected motor control from a single node is drive control (6)
float vel_old[6] = {0,0,0,0,0,0};

float motor_values_in[6] = {0,0,0,0,0,0}; // filled by parsed array of a single packet to avoid 6 asynchronous commands EITHER POS OR VEL VALUES BASED ON MODE!
const int ENA_PIN = 17; // the Arduino pin connected to the EN1 pin L298N
const int IN1_PIN = 26; // the Arduino pin connected to the IN1 pin L298N
const int IN2_PIN = 25; // the Arduino pin connected to the IN2 pin L298N

// Set Values

float p_in= 0.0f;
float v_in= 0.0f;
float kp_in= 0.00f;
float kd_in= 0.00f;
float t_in= 0.0f;

float p_inOLD = 0.0f;

//measured values - responses from the motor
float p_out[6] = {0.0,0.0,0.0,0.0,0.0,0.0};  // actual position
float v_out[6] = {0.0,0.0,0.0,0.0,0.0,0.0};   // actual velocity
float t_out[6] = {0.0,0.0,0.0,0.0,0.0,0.0};   // actual torque

const int SPI_CS_PIN =5;      //pin 5 for esp32 nodemcu

const int start = 33; //pin used for on button
const int stop = 27; //pin used for off button
bool onButton = 0;
bool offButton = 0;
String input_mode = "velocity";

float pos_serial = 0.0;
bool new_data = true;
bool onFlag = false;

MCP_CAN CAN(SPI_CS_PIN);               //Set CS pin

//initialize adc potentiometer
adc_attenuation_t att = ADC_11db;
#define adc_pin   35
#define rezESP32  8 //bitwidth of adc reading. Reduced bitwidth (12->8) as 256bits in hopes to stabilize reading



void sub2topics(){
    //subscriibe(topic,qos)
    client.subscribe("mode/exit");
    client.subscribe("mode/enter/arm");
    client.subscribe("mode/vel/arm");
    client.subscribe("mode/pos/arm");
    client.subscribe("write/motor/arm");  
    client.subscribe("set/zero/arm");
    client.subscribe("restart/arm");
}

void checkConnection() {
  WiFi.mode(WIFI_STA);
  if (WiFi.status() != WL_CONNECTED)
  {
    while (WiFi.status() != WL_CONNECTED)
    {
      WiFi.begin(SSID, WIFI_PW);//check for known network HOME
      Serial.print(".");
      delay(2000);
    }
  }
}

void checkBroker(){
  if(client.state()!=0){
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(callback);
  
  if(client.connect (device_name,NULL,NULL)) {
    Serial.println ("Connected to MQTT Broker");
  } 
  else {
      Serial.print("MQTT Broker connection failed");
      Serial.println (client.state());
      delay(200);
  }
  sub2topics();
  }
}

void publishDouble(double sensor_data,const char* topic_name, int precision) {

    double reading = sensor_data;
    constexpr size_t BUFFER_SIZE = sizeof(double)+3; //1 char for the sign, 1 char for the decimal dot, 4 chars for the value & 1 char for null termination
    char buffer[BUFFER_SIZE]; 
    dtostrf(reading, BUFFER_SIZE - 1 /*width, including the decimal dot and minus sign*/, precision /*precision*/, buffer);
    client.publish(topic_name, buffer, BUFFER_SIZE); //notice we're using the overload where you specify the length of the buffer, as we know it and it saves a call to strlen
}

void callback(char* topic, byte* payload, unsigned int length) {
  //DO NOT RETAIN VALUES FOR THESE MOTORS AS IT MAY START IN UNWANTED STATE. HANDLE STATES IN CODE

  //Serial.print(topic);
  //Serial.print(" ");
  String msg;
  int i=0;
    for (i;i<length;i++) {
      //Serial.print((char)payload[i]);
      msg += String((char)payload[i]);
    }
  //prints msg for all topics but datatype is float
  Serial.print("Command from ");
  Serial.print(String(topic));
  Serial.print(": ");
  Serial.println(msg);
  //for topics that return float values
  //Serial.println(msg.toFloat());
  //delay(750);

 if (String(topic) == ("mode/exit")) {
   offButton = 1;
 }
 else if (String(topic) == ("mode/enter/arm")) {
   onButton = 1;
 }
 else if ((String(topic) == ("mode/vel/arm")) && (onFlag==false)) {
   input_mode = "velocity";
   //reads mode for test motor at index 0 from MQTT
   mode = input_mode; 
   client.publish("ack/arm","Velocity Mode Entered");
   Serial.println("Velocity Mode");
 }
 else if ((String(topic) == ("mode/pos/arm")) && (onFlag==false)) {
   input_mode = "position";
   //reads mode for test motor at index 0 from MQTT
   mode = input_mode;
   client.publish("ack/arm","Position Mode Entered");
   Serial.println("Position Mode");
 }

 else if (String(topic)==("set/zero/arm")){
   for (int ii=0;ii<4;ii++){
    Zero(ii);
    EnterMotorMode(ii);
    unpack_reply();
    ExitMotorMode(ii);
   }
   client.publish("ack/arm","Position Zeroed");
 }

 //update all motor values synchronously from ROS/Python
 else if (String(topic) == ("write/motor/arm")) {
    StaticJsonDocument <512> doc;
    deserializeJson(doc,payload);
    // deserializeJson(doc,str); can use string instead of payload
    //LOAD ALL 6 VALUES IN FROM PYTHON CONTROLLER TO MOTOR ARRAY
    for (int ii = 0; ii<6;ii++){
      motor_values_in[ii] = doc["M0"][ii];
      //Serial.print("motor value in");
      //Serial.println(motor_values_in[ii]);
      //delay(500);
    }
 }
 else if (String(topic)=="restart/arm"){ 
   ESP.restart();
 }

}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  while(CAN_OK !=CAN.begin( CAN_1000KBPS)){
     Serial.println("CAN BUS Shield init fail");
     Serial.println("Init CAN BUS Shield again");    
     delay(1000);
  }

  Serial.println("CAN BUS Shield init ok!");

  delay(100);
  

  //pinMode(start,INPUT);
  //pinMode(stop,INPUT);


  analogReadResolution(rezESP32);
  analogSetAttenuation(att);//ADC_0db,ADC_2_5db,ADC_6db,ADC_11db
  adcAttachPin(adc_pin);//ADC GPIO STARTS AT 3 FOR ESP32-S3 

  for (int ii=0; ii<4;ii++){
    Zero(ii); //ZERO BEFORE ENTERING MOTOR MODE!!! THIS PUTS POS TO ZERO TO PREVENT MOVENT
    EnterMotorMode(ii);
    ////////////////////////////////read all values. Set values to resting based on reading for safety!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//////////////////////////////
    if(CAN_MSGAVAIL == CAN.checkReceive()){ // check if data coming{
        unpack_reply(); //read before sending position or load current position as hold spot instead of zeroing to ensure that you are at 0!!!!
    }
    ExitMotorMode(ii);
  }
  Serial.println(mode);

  delay(10);

  //connect to wifi network
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
      Serial.println("STA Failed to configure");
  }

  WiFi.begin(SSID,WIFI_PW);//check for known network HOME
  delay(1000);
  Serial.println (" ");
  Serial.println ("Connecting to AP");
  checkConnection();
  Serial.println ("Connected to WiFi AP!!");
  Serial.print ("Got an IP address :");
  Serial.println (WiFi.localIP());

  //setup linear actuatior
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  digitalWrite(ENA_PIN, HIGH);
  int velocity = 128;
  analogWrite(ENA_PIN,velocity);

  //setup neo550
  esc.attach(neopin);
  esc.writeMicroseconds(1000);

  checkBroker();
   
}

long previousMillis =0;

boolean newData = false;


void loop() {

  checkConnection();
  checkBroker();
  client.loop();

  //note that a pulldown is used to hold zero state
  //WILL TOGGLE ON/OFF IF LOGIC PIN NOT CONNECTED!!
      //onButton = digitalRead(start); //enable for hardware control of mode
      if (onButton){
        //enters mode for all motors
        Serial.println("Entering MIT Mode");
        for (int ii = 0; ii<4; ii++){
          EnterMotorMode(ii);

          delay(100);//prevent double press
          onButton = 0;
          onFlag = true;
        }
        Serial.println("Entered MIT Mode");
        client.publish("ack/arm","Entered Motor Mode");
      }

      //offButton = digitalRead(stop); // enable for hardware control of mode
      if (offButton){
        //enters mode for all motors
        Serial.println("Exiting Motor Mode");
        for (int ii = 0; ii<4;ii++){
          v_in = 0; //send velocity to 0 before exiting
          float kp = 0;
          float kd = 0;
          byte ID = motID[ii];
          float pos = p_out[ii]; //uses current position loaded in globals to prevent jerks when shutting off
          pack_cmd(ID,pos,v_in,kp,kd); //sends velocity to 0 before running
          
          delay(100); //wait for motor to stop
          ExitMotorMode(ii);
          delay(100);
          Zero(ii);//sets to origin after velocity mode to prevent running on mode switch
          delay(100);//prevent double press
          offButton = 0;
          onFlag = false;
          mode = ""; //ensures that desired mode is triggered upon restart
        }
        Serial.println("Exited Motor Mode");
        client.publish("ack/arm","Exited Motor Mode");
      }

  //READ TELEMETRY (MOTORID,MOTORMODE,MOTORDATA)
  /*
  if ((Serial.available()>0) && (!onFlag)){
    char mychar = Serial.read();
    if (mychar=='v'){
      mode[0] = "velocity";
      Serial.println("Velocity Mode: Mot 0");
    }
    if (mychar=='p'){
      mode[0] = "position";
      Serial.println("Position Mode: Mot 0");
    }  
  }
  */
  
  /*
    //reads from serial input for postition
  if ((Serial.available()>0) && (!newData) ){
    //add zero on string since it doesnt show left side of decimal otherwise
    String mystring = Serial.readString();
    p_in = mystring.toFloat();
    
    //Serial.print("String In: ");
    //Serial.println(p_in);
    
    newData = true;
    p_inOLD = p_in;
  }
  */


  if (onFlag){
    //send a 0 state for motors not being used. Assumed full individual control
    //pull these values from comm method with controls after fully understood
    run_Motors(mode);
    /*
    if (mode[calledID]=="position"){
      float vel = 0;
      //p_in = potentCMD()/10; // read from potentiometer using motor 0. division by 10 with max of 45 gives 4.5 radians of control from potentiometer
      //Serial.print("Position Potentiometer: ");
      //Serial.println(p_in);
      run_Motor(calledID,mode[calledID],vel,p_in); //the ID from the hardware indicates which motor is in control mode
    }
    else{ //velocity mode
      //float vel = potentCMD(); // read from potentiometer using motor 0;
      float vel = v_in; //from MQTT 
      run_Motor(calledID,mode[calledID],vel,0); //0 with zeroing origin in exit call prevents motor from running after switching modes
      //Serial.println("Running Velocity Mode");
    }
    */
    
  }
  
  // receive CAN... one for each motor outout waiting due to pushing all 6 through at a time
  for (int ii =0;ii<4;ii++){
    if(CAN_MSGAVAIL == CAN.checkReceive()){ // check if data coming{
      unpack_reply(); //read before sending position or load current position as hold spot instead of zeroing to ensure that you are at 0!!!!
    }
  }

  //Serial.print(millis()-previousMillis);
  previousMillis=millis();
  delay(100);

  //publishJSON();//REMOVE THIS EXTRA PUBLISH. FOR TESTING FUNCTION ONLY
}

void EnterMotorMode(int calledID){
  byte buf[8];
  buf[0]=0xFF;
  buf[1]=0xFF;
  buf[2]=0xFF;
  buf[3]=0xFF;
  buf[4]=0xFF;
  buf[5]=0xFF;
  buf[6]=0xFF;
  buf[7]=0xFC;
  byte ID = motID[calledID];
  CAN.sendMsgBuf(ID,0,8, buf);
  
  }

void ExitMotorMode(int calledID){
  
  byte buf[8];
  buf[0]=0xFF;
  buf[1]=0xFF;
  buf[2]=0xFF;
  buf[3]=0xFF;
  buf[4]=0xFF;
  buf[5]=0xFF;
  buf[6]=0xFF;
  buf[7]=0xFD;
  byte ID = motID[calledID]; 
  CAN.sendMsgBuf(ID,0,8, buf);
 
}

void Zero(int calledID){
  
  byte buf[8];
  buf[0]=0xFF;
  buf[1]=0xFF;
  buf[2]=0xFF;
  buf[3]=0xFF;
  buf[4]=0xFF;
  buf[5]=0xFF;
  buf[6]=0xFF;
  buf[7]=0xFE;
  byte ID = motID[calledID]; 
  CAN.sendMsgBuf(ID,0,8, buf);
 
}


void pack_cmd(int calledID, float p, float v, float kp_in,float kd_in){
  p_in = p;
  v_in = v;
  byte ID = motID[calledID]; 
  Serial.print("Input for Motor # ");
  Serial.print(calledID);
  Serial.print(" ID # ");
  Serial.print(ID);
  Serial.print(": ");
  Serial.println(v_in);

  
  byte buf[8];

  //Serial.print("Velocity Sent: ");
  //Serial.println(v_in);


  // limit data to  be within bounds ///

  float p_des= constrain(p_in,P_MIN,P_MAX);
  float v_des= constrain(v_in,V_MIN,V_MAX);
  float kp= constrain(kp_in,KP_MIN,KP_MAX);
  float kd= constrain(kd_in,KD_MIN,KD_MAX);
  float t_ff= constrain(t_in,T_MIN,T_MAX);

  unsigned int p_int = float_to_uint(p_des,P_MIN,P_MAX,16);
  unsigned int v_int = float_to_uint(v_des,V_MIN,V_MAX,12);
  unsigned int kp_int = float_to_uint(kp,KP_MIN,KP_MAX,12);
  unsigned int kd_int = float_to_uint(kd,KD_MIN,KD_MAX,12);
  unsigned int t_int = float_to_uint(t_ff,T_MIN,T_MAX,12);

 //pack ints into can buffer ///

  buf[0]=p_int >>8;
  buf[1]=p_int & 0xFF;
  buf[2]=v_int >> 4;
  buf[3]=((v_int&0xF)<< 4)| (kp_int >>8);
  buf[4]= kp_int &0xFF;
  buf[5]= kd_int >>4;
  buf[6]= ((kd_int&0xF)<< 4)| (t_int >>8);
  buf[7]= t_int &0xFF;
  
  CAN.sendMsgBuf(ID,0,8,buf);

}

void unpack_reply(){

  byte len=0;
  byte buf[8];
  CAN.readMsgBuf(&len,buf); //&rxId was added from mcp_can example
  unsigned long canId=rxId; //changed from CAN.getCanId()

  ///unpack ints from can buffer///

  unsigned int id = buf[0];
  unsigned int p_int = (buf[1]<<8)| buf[2];
  unsigned int v_int =(buf[3]<<4)|(buf[4]>>4);
  unsigned int i_int = ((buf[4]&0xF )<<8)| buf[5];

  //dynamic indexing of motor id to value in output array of each reading
  int index = 0;
  for (int ii=0; ii<4;ii++){
    if (id==int(motID[ii])){
      index = ii;
    }
  }

  /// convert uints to floats ///
  p_out[index] = uint_to_float(p_int,P_MIN,P_MAX, 16);
  v_out[index] = uint_to_float(v_int,V_MIN,V_MAX, 12);
  t_out[index] = uint_to_float(i_int,T_MIN,T_MAX, 12);

  //package and send these values for each motor back to controls!!!!!!!!!!!!
  Serial.print("ID: ");
  publishDouble(id, "read/id/arm",0);
  Serial.println(id);
  Serial.print("position: ");
  publishDouble(p_out[index], "read/pos/arm",2);
  Serial.println(p_out[index]);
  Serial.print("velocity: ");
  publishDouble(v_out[index], "read/vel/arm",2);
  Serial.println(v_out[index]);
  Serial.print("torque: ");
  publishDouble(t_out[index], "read/torq/arm",2);
  Serial.println(t_out[index]);
  Serial.println("");
  
  publishJSON();
  }

 unsigned int float_to_uint(float x, float x_min, float x_max, float bits){
  
  //convert a float to an unsigned int, given range and number of bits  ///

  float span = x_max-x_min;
 float offset = x_min;
 unsigned int pgg=0;

 if(bits ==12){
  pgg= (unsigned int)((x-offset)*4095.0/span);
  }
 if(bits ==16){
  pgg= (unsigned int)((x-offset)*65535.0/span);
  }
  return pgg;
  
 }

float potentCMD(){
  float adc_readout =0.0;
  int averages = 1024;
  //average for reading stabilizaiton
  for (int ii =0; ii<averages;ii++){
    //accumulate readings
    adc_readout += analogRead(adc_pin);
  }
  //average the accumulation
  adc_readout /= averages;
  //shift to make middle the zero point
  adc_readout -= (pow(2,rezESP32)/2);
  //normalize
  adc_readout /= (pow(2,rezESP32)/2);
  //expand to max and min of motor controller at 30 or 50 depending
  adc_readout *= V_MAX;

  //Serial.print(F("Vel Input: "));
  //Serial.println(adc_readout);
  //delay(500);
  
  return adc_readout;
}


float uint_to_float(unsigned int x_int, float x_min,float x_max, int bits){
  /// converts unsined int to float, given range and numbers of bits///
 float span = x_max - x_min;
 float offset = x_min;
 float pgg=0;
  if (bits ==12){
    pgg=((float)x_int)*span/4095.0 + offset;
    
    }  
if (bits ==16){
    pgg=((float)x_int)*span/65535.0 + offset;
    
    }
    return pgg;
  }

void extend(){
  // extend the actuator
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
}

void retract(){
  // retracts the actuator
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
}

void stopLA(){
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
}

void setOtherMotors(){
  float neoZero = 92.5;
  float neoSet = motor_values_in[4];
  neoSet = map(neoSet, 0, 1023, 1000, 2000);     
  esc.writeMicroseconds(neoSet);   

  float linActSet = motor_values_in[5];
  neoSet /= V_MAX;
  neoSet *=neoZero;
  neoSet +=neoZero;
  if (linActSet > 0){
    extend();
  }
  else if (linActSet<0){
    retract();
  }
  else{
    stopLA();
  }
}

//note that called ID is an integer for indexing while motID is the hex value for the motor
void run_Motors(String mode){
  if (mode=="velocity"){
    //vel_old[calledID] = int(floor(vel)); // update vel_old here before changing vel in conditionals below
    //v_in = vel_old[calledID]; //sends to global for pack_cmd()
    for (int ii=0;ii<4;ii++){
      v_in = motor_values_in[ii];
      //use these prints for serial monitor
      //Serial.print("Vel Input: ");
      //Serial.print(int(vel));
      //Serial.println(v_in);
      float pos = 0.0;
      float kp = 0.0;
      float kd = 0.05; //value found to work from testing despite being "loose"
      byte ID = motID[ii]; //this is not the motor by
      pack_cmd(ii,pos,v_in,kp,kd); //pack_cmd(calledID,position,velocity,kp,kd)
    }
    //this sets the other 2 motors in the function
    setOtherMotors();
    Serial.println("");//makes empty line between command printouts
  }

  
  if ((mode=="position") ){
      v_in = 0.00;
      //values for kp and kd found in youtube video were 22 and 0.5. doing 100th of this value seems to work well
      float kp = 0.47; // effective values gathered from youtube video "you need these motors in your rover"
      float kd = 0.048;
      for (int ii=0;ii<4;ii++){
        float pos = motor_values_in[ii];
        //int ID = motID[ii];
        //Serial.print("Position Input for Motor ");
        //Serial.print(ii);
        //Serial.print(": ");
        //Serial.println(pos);
        pack_cmd(ii,pos,v_in,kp,kd); //pack_cmd(calledID,position,velocity,kp,kd)
      }
      setOtherMotors();
      Serial.println("");//makes empty line between command printouts
      newData = false;
  }

      //delay(10); //wait for encoder to update before print
    
}

void publishJSON(){ //contains index of reading for motor and ID of motor at that index
    StaticJsonDocument<512> doc;
    JsonArray data = doc.createNestedArray("VPTMotor0");
    data.add(v_out[0]);
    data.add(p_out[0]);
    data.add(t_out[0]);
    data = doc.createNestedArray("VPTMotor1");
    data.add(v_out[1]);
    data.add(p_out[1]);
    data.add(t_out[1]);
    data = doc.createNestedArray("VPTMotor2");
    data.add(v_out[2]);
    data.add(p_out[2]);
    data.add(t_out[2]);
    data = doc.createNestedArray("VPTMotor3");
    data.add(v_out[3]);
    data.add(p_out[3]);
    data.add(t_out[3]);
    /*
    data = doc.createNestedArray("VPTMotor4");
    data.add(v_out[4]);
    data.add(p_out[4]);
    data.add(t_out[4]);
    data = doc.createNestedArray("VPTMotor5");
    data.add(v_out[5]);
    data.add(p_out[5]);
    data.add(t_out[5]);
    //doc["data"]=data;
    */
    // Generate the minified JSON and send it to the Serial port.
    //
    char out[256];
    int b =serializeJson(doc, out);
    //Serial.print("bytes = ");//use to ensure no overflow
    //Serial.println(b,DEC);
    client.publish("read/armMotors", out);
  }

