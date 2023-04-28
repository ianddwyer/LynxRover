#include <Wire.h> //Needed for I2C to GNSS
#include <Arduino.h>
#include "WiFi.h"
#include <PubSubClient.h>
#include <string.h>
#include <time.h>
#include "WiFi_Logins.h"
#include <ArduinoJson.h>

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;

//network setup with static IP assigned, otherwise does not work with ubiquiti
#define SSID  WIFI_NETWORK_D
#define WIFI_PW  WIFI_PASSWORD_D
IPAddress local_IP(192, 168, 42, 45);
IPAddress gateway(192, 168, 42, 10);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);   //optional
IPAddress secondaryDNS(0, 0, 0, 0); //optional

bool flagLED = false;

WiFiClient GPS;
PubSubClient client(GPS);
const char* device_name = "GPS_SENSOR";

const char* LED_top = "led_status";
const int blue_led_pin = 33;
const int red_led_pin = 32;
const int green_led_pin = 27;
float last_message_received = 0;
bool green_led_blink = false;

void set_leds(int led_on) {
  digitalWrite(red_led_pin, LOW);
  digitalWrite(blue_led_pin, LOW);
  digitalWrite(green_led_pin, LOW);
  if (led_on == 1) {
    digitalWrite(red_led_pin, HIGH);
  } else if (led_on == 2) {
    digitalWrite(blue_led_pin, HIGH);
  } else if (led_on == 3) {
    green_led_blink = true;
  }
}

void sub2topics(){
  client.subscribe(LED_top);
}

void turn_off_leds() {
  digitalWrite(red_led_pin, LOW);
  digitalWrite(blue_led_pin, LOW);
  digitalWrite(green_led_pin, LOW);
}

void checkConnection() {
  WiFi.mode(WIFI_STA);
  if (WiFi.status() != WL_CONNECTED)
  {
    while (WiFi.status() != WL_CONNECTED)
    {
      WiFi.begin(SSID, WIFI_PW);//check for known network HOME
      Serial.print(".");
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

  last_message_received = msg.toFloat();
  

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

void publishJSON(double lon,double lat){ //contains index of reading for motor and ID of motor at that index
    StaticJsonDocument<256> doc;
    JsonArray data = doc.createNestedArray("Long");
    data.add(lon);
    data = doc.createNestedArray("Lat");
    data.add(lat);
    //doc["data"]=data;
    // Generate the minified JSON and send it to the Serial port.
    //
    char out[128];
    int b =serializeJson(doc, out);
    //Serial.print("bytes = ");//use to ensure no overflow
    //Serial.println(b,DEC);
    client.publish("read/GPS", out);
}





void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing.");
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  //myGNSS.saveConfiguration(); //Optional: Save the current settings to flash and BBR

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
  

  //LED callback
  

  
  pinMode(blue_led_pin, OUTPUT);
  pinMode(red_led_pin, OUTPUT);
  pinMode(green_led_pin, OUTPUT);

  checkBroker();
   
}

void loop()
{
  checkConnection();
  checkBroker();
  client.loop();
  //Query module. The module only responds when a new position is available
  long latitude = 0;
  long longitude = 0;
  long altitude = 0;
  long accuracy = 0;
  if (myGNSS.getPVT())
  {
    latitude = (myGNSS.getLatitude());//*pow(10,-7);
    Serial.print("\nLat: ");
    Serial.print(latitude*pow(10,-7),5);
    Serial.println(" [degrees]");

    longitude = (myGNSS.getLongitude());//*pow(10,-7);
    Serial.print("Long: ");
    Serial.print(longitude*pow(10,-7),5);
    Serial.println(" [degrees]");

    altitude = (myGNSS.getAltitude())*0.00328084;
    Serial.print("Alt: ");
    Serial.print(altitude,9);
    Serial.println(" [ft]");
  }
  
  if (myGNSS.getNAVHPPOSECEF())
  {
    accuracy = (myGNSS.getPositionAccuracy())*0.00328084;
    Serial.print("3D Positional Accuracy: ");
    Serial.print(accuracy,8);
    Serial.println(" [ft]\n");
  }

  if (last_message_received == 1) {
    set_leds(1);
    green_led_blink = false;
  } else if (last_message_received == 2) {
    set_leds(2);
    green_led_blink = false;
  } else if (last_message_received == 3) {
    if (green_led_blink) {
      if (!flagLED){
      digitalWrite(green_led_pin, HIGH);
      flagLED = true;
      }
      else if (flagLED){
      digitalWrite(green_led_pin, LOW);
      flagLED = false;
      }
    } else {
      set_leds(3);
    }
  } else {
    turn_off_leds();
  }

  publishJSON(longitude,latitude);
}