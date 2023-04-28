

#include <Arduino.h>
#include "WiFi.h"
#include <PubSubClient.h>
#include <string.h>
#include <time.h>
#include "WiFi_Logins.h"
#include <ArduinoJson.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

float thetaM;
float phiM;
float thetaFold=0;
float thetaFnew;
float phiFold=0;
float phiFnew;

float thetaG=0;
float phiG=0;

float theta;
float phi;

float thetaRad;
float phiRad;

float Xm;
float Ym;
float psi;


float dt;
unsigned long millisOld;




Adafruit_BNO055 bno = Adafruit_BNO055(55);

//network setup with static IP assigned, otherwise does not work with ubiquiti
#define SSID  WIFI_NETWORK_D
#define WIFI_PW  WIFI_PASSWORD_D
IPAddress local_IP(192, 168, 42, 63);
IPAddress gateway(192, 168, 42, 10);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);   //optional
IPAddress secondaryDNS(0, 0, 0, 0); //optional


const char* device_name = "nodeIMU";
WiFiClient BNO055;
PubSubClient client(BNO055);
bool rebooted = true; // initialize functions/conditions on reboot

int num_sensors = 3;
int ultrasonic[5] = {0,0,0,0,0}; // provides integer centimeter readout

int pitch = 0;
int roll = 0;
float yaw = 0;
int heading = 0;
float yaw_old =0;
float jerkMag = 0; 
int vibration = 0;

float z_vibeOld,y_vibeOld,x_vibeOld;

String configString;

int ta = 0;
int tb = 0;

//for US sensor. If ther
const int trigPin[5] = {33,34,25,14,16};
const int echoPin[5] = {32,35,26,27,17};

long duration;
int objDistance[10]; //setup for 10 point variance check to avoid noise of 10m (says 10m when unsure)
int objDist;//initialize final container for object distance
int objDistOld[5]={100,100,100,100,100};//initialize to expected distance (mounted)   of 1m
int stdDev;
int distMean;


void checkConnection() {
  WiFi.mode(WIFI_STA);
  if (WiFi.status() != WL_CONNECTED){
    while (WiFi.status() != WL_CONNECTED){
      WiFi.begin(SSID, WIFI_PW);//check for known network HOME
      Serial.print(".");
      delay(100);
    }
  }
}

void checkBroker(){
  if(client.state()!=0){
    client.setServer(mqtt_broker, mqtt_port);
  
  if(client.connect (device_name,NULL,NULL)) {
    Serial.println ("Connected to MQTT Broker");
  } 
  else {
      Serial.print("MQTT Broker connection failed");
      Serial.println (client.state());
      delay(100);
    }
  }
}


void publishDouble(double sensor_data,const char* topic_name, int precision) {

    double reading = sensor_data;
    constexpr size_t BUFFER_SIZE = sizeof(double)+3; //1 char for the sign, 1 char for the decimal dot, 4 chars for the value & 1 char for null termination
    char buffer[BUFFER_SIZE]; 
    dtostrf(reading, BUFFER_SIZE - 1 /*width, including the decimal dot and minus sign*/, precision /*precision*/, buffer);
    client.publish(topic_name, buffer, BUFFER_SIZE); //notice we're using the overload where you specify the length of the buffer, as we know it and it saves a call to strlen
}



void setup()
{
  delay(1000);

  //use serial with previous I2C read
  Serial.begin(115200);

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

  checkBroker();

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  Serial.println("BNO Connected!!!");
  
  delay(1000);
  bno.setExtCrystalUse(true);
  

  pinMode(trigPin[0], OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin[1], INPUT); // Sets the echoPin as an Input 
  pinMode(trigPin[2], OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin[3], INPUT); // Sets the echoPin as an Input 
  pinMode(trigPin[4], OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin[5], INPUT); // Sets the echoPin as an Input 

  millisOld = millis();

}



void ultraSonic(int sensor){ //specify which ultrasonic sensor for proper updates
    //ultrasonic sensor
  //signal acquisition from tutorial code at https://howtomechatronics.com/tutorials/arduino/ultrasonic-sensor-hc-sr04/
  //filtering was needed after using this method
  // Clears the trigPin
  digitalWrite(trigPin[sensor], LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin[sensor], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin[sensor], LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin[sensor], HIGH);
  // Calculating the distance
  // Prints the distance on the Serial Monitor
  //Serial.print("Object Distance: ");

  //10-point data collection
  objDistance[9] = objDistance[8];
  objDistance[8] = objDistance[7];
  objDistance[7] = objDistance[6];
  objDistance[6] = objDistance[5];
  objDistance[5] = objDistance[4];
  objDistance[4] = objDistance[3];
  objDistance[3] = objDistance[2];
  objDistance[2] = objDistance[1];
  objDistance[1] = objDistance[0];
  objDistance[0] = duration * 0.034 / 2;

  //filtering unwated values that run high when sensor is unsure filtered
  //trust based filter. Low confidence due to high variation or numbers greater than usable range. Prevents slow change in response and distortion from averaging filters
  //using standard deviation vs mean
  int distTemp = objDistance[sensor]; // prevents it from posting while taking average. Initialize with useful value
  //10 is used since false signals dont appear to propigate more than 6 times in a row
  int points = 7; // THIS SLOWS THE RESONIVENESS WITH MORE POINTS TO CONdSIDER... MAKES IT MORE "PICKY"
  for (int ii = 1; ii<points;ii++){//starts at 1 to initialize withuseful value
    distTemp += objDistance[ii];
  }
  distMean = distTemp/points;
  int dev = pow(objDistance[sensor]-distMean,2); // initialize with a useful value
  for (int ii = 1; ii<points; ii++){
    dev += pow(objDistance[ii]-distMean,2);
  }
  stdDev = pow(dev/points, 0.5);//calculate standard deviation
  //THIS SLOWS RESPONSIVENESS THE MOST, RAISE THRESHOLD IF TOO SLOW
  int thresh = distMean*2;//fairly trusting
  if ((stdDev<thresh) && (objDistance[sensor]<900)){//note that values above 900 imply no signal returned so prevent storing above this value and just repeats previous known value if above 900
    objDist = objDistance[sensor];//use raw reading when std is not too high (low variation vs mean) cant use mean since it would drive value high or low with false readings
  }
  else {ultrasonic[sensor] = objDistOld[sensor];}//if std deviation is high versus the mean then only use old/trusted values


  objDistOld[sensor] = ultrasonic[sensor];//retain used value in for use when redout is unreliable 

}

void IMU(){
  uint8_t system, gyro, accel, mg = 0;
  bno.getCalibration(&system, &gyro, &accel, &mg);
  imu::Vector<3> acc =bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyr =bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag =bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  thetaM=-atan2(acc.x()/9.8,acc.z()/9.8)/2/3.141592654*360;
  phiM=-atan2(acc.y()/9.8,acc.z()/9.8)/2/3.141592654*360;
  phiFnew=.95*phiFold+.05*phiM;
  thetaFnew=.95*thetaFold+.05*thetaM;

  dt=(millis()-millisOld)/1000.;
  millisOld=millis();
  theta=(theta+gyr.y()*dt)*.95+thetaM*.05;
  phi=(phi-gyr.x()*dt)*.95+ phiM*.05;
  thetaG=thetaG+gyr.y()*dt;
  phiG=phiG-gyr.x()*dt;

  phiRad=phi/360*(2*3.14);
  thetaRad=theta/360*(2*3.14);

  Xm=mag.x()*cos(thetaRad)-mag.y()*sin(phiRad)*sin(thetaRad)+mag.z()*cos(phiRad)*sin(thetaRad);
  Ym=mag.y()*cos(phiRad)+mag.z()*sin(phiRad);
  
  
  //sensor appears to be about 10 degrees off from reference compass
  float offset = 10*(M_PI/180); // must be done in radians to be cyclical in trig relation

  yaw=(atan2(Ym,Xm)+offset)/(2*M_PI)*360;

  if (yaw<0){
    heading = int(360+yaw);
  }
  else {
    heading = int(yaw);
  }
  sensors_event_t event; 
  bno.getEvent(&event);
  
  /* Get a new sensor event */ 
  int averages = 128;
  for (int ii = 0; ii<averages;ii++){
    pitch += int(event.orientation.y);
    roll += int(event.orientation.z);
  }
  pitch /=averages;
  roll /=averages;


  tb = millis();
  float z_vibe = ((acc.z()-9.8)*1000); //keeps data sitting around 0 
  float y_vibe = ((acc.y())*1000); //keeps data sitting around 0 
  float x_vibe = ((acc.x())*1000); //keeps data sitting around 0 
  // jerk=accel*d/dt
  double dt = tb-ta/1000.00; // discrete time interval in seconds 
  float dAz = z_vibe - z_vibeOld;// using absolute value to just find change in accel
  float dAy = y_vibe - y_vibeOld;// using absolute value to just find change in accel
  float dAx = x_vibe - x_vibeOld;// using absolute value to just find change in accel
  float jerkz = dAz/dt;// find change. z should always be + if right side up
  //jerkz *= 1000; //use gain to magnify data for visualization on combined graph
  float jerky = dAy/dt;// find change. y should always be + if right side up
  //jerky *= 1000; //use gain to magnify data for visualization on combined graph
  float jerkx = dAx/dt;// find change. x should always be + if right side up
  //jerkx *= 1000; //use gain to magnify data for visualization on combined graph
  jerkMag = sqrt(pow(jerkz,2)+pow(jerky,2)+pow(jerkx,2));
  jerkMag*=1000;
  if (int(jerkMag)>=2000){
    vibration = 5;
  }
  else if (int(jerkMag)<2000 && int(jerkMag)>=800){
    vibration = 4;
  }
  else if (int(jerkMag)<800 && int(jerkMag)>=400){
    vibration = 3;
  }
  else if (int(jerkMag)<400 && int(jerkMag)>=200){
    vibration = 2;
  }
  else if (int(jerkMag)<=200 && int(jerkMag)>=50){
    vibration = 1;
  }
  else {
    vibration = 0;
  }
  ta = tb;
  z_vibeOld = z_vibe;
  y_vibeOld = y_vibe;
  x_vibeOld = x_vibe;
  /*
  //prevent small changes an constants from showing
  if (jerk<10){
    z_vibe = 0; //use this threshold only if change in acceleration does not prove to be useful
  }
  */

  
}

void loop()
{
  checkConnection();
  checkBroker();
  client.loop();

  IMU();

  
  ultraSonic(0);
  ultraSonic(1);
  ultraSonic(2);


  displayInfo();
  publishJSON();
  delay(10);
}

void publishJSON(){ //contains index of reading for motor and ID of motor at that index
    StaticJsonDocument<256> doc;
    JsonArray data = doc.createNestedArray("pitch");
    data.add(pitch);
    data = doc.createNestedArray("roll");
    data.add(roll);
    data = doc.createNestedArray("heading");
    data.add(heading);
    data = doc.createNestedArray("vibration");
    data.add(vibration);
    data = doc.createNestedArray("ultrasonic");
    data.add(ultrasonic[0]);
    data.add(ultrasonic[1]);
    data.add(ultrasonic[2]);

    //doc["data"]=data;
    // Generate the minified JSON and send it to the Serial port.
    //
    char out[128];
    int b =serializeJson(doc, out);
    //Serial.print("bytes = ");//use to ensure no overflow
    //Serial.println(b,DEC);
    client.publish("read/sensors", out);
    Serial.println("Published JSON!");


}

void displayInfo(){


  //NEED TO ADD CALIBRATION FOR THE COMPASS TO WORK!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  /* Display the floating point data */
  Serial.print(pitch, 4);
  Serial.print(",");
  Serial.print(roll, 4);
  Serial.print(",");
  //Serial.print(Xm, 4);
  //Serial.print(",");
  //Serial.print(Ym, 4);
  //Serial.print(",");
  Serial.print(yaw, 4); // converts to degrees for display only
  

  //using raw acceleration data to obtain vibrational noise data
  
  //find the rate of change in vertical acceleration data to quantify vibrations
  //maybe do a for loop that counts multiple changes of a certain magnitude threshold before throwing errors to user or slowing the vehicle 
  Serial.print(",");
  Serial.print(vibration);//Jerk (derivative of accel) is plotted to find density of change in acceleration. Multiple highs suggest rapid change in axial acceleration providing vibrational denisty


  //From ultrasonic reading
  Serial.print(",");
  Serial.println(objDist);
  /* DEBUG ULTRASONIC FILTERING
  Serial.println("STD: ");
  Serial.println(stdDev);
  Serial.println("Mean: ");
  Serial.println(distMean);
  Serial.println("Accepted Distance: ");
  Serial.println(objDist);
  delay(2000);
  */
  objDistOld[0] = objDist;//retain used value in for use when redout is unreliable  

  delay(10);

}
