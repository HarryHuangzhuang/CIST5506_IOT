#include "config.h"

#include <WiFiClientSecure.h>
#include <MQTTClient.h> //MQTT Library Source: https://github.com/256dpi/arduino-mqtt

#include <ArduinoJson.h> //ArduinoJson Library Source: https://github.com/bblanchon/ArduinoJson
#include "WiFi.h"

// MQTT topics for the device
#define AWS_IOT_PUBLISH_TOPIC   "occupancymonitor/room1/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "occupancymonitor/room1/sub"

// Infrared sensors
#define IR_ENTRY 33
#define IR_EXIT 32
// LEDs
#define RED_LED 13
#define GREEN_LED 14
//#define IR_MODEL SharpIR::GP2Y0A02YK0F

// timeouts
uint32_t MOVEMENT_TIMEOUT = 1000;

WiFiClientSecure wifi_client = WiFiClientSecure();
MQTTClient mqtt_client = MQTTClient(256); //256 indicates the maximum size for packets being published and received.

uint32_t t1;
//SharpIR ir_ent(IR_MODEL, IR_ENTRY);
//SharpIR ir_ext(IR_MODEL, IR_EXIT);
uint8_t d1;
uint8_t d2;

// counting
int count;
int threshold_entry; // threshold value of entrance
int threshold_exit; // threshold value of exit
boolean entNotBlocked; // true if entrance is emtpy
boolean exitNotBlocked; // true if exit is empty
boolean movement; // false if no movement, true if somebody is in the process of entering/exiting
boolean entryVal; // true when entry IR sensor value is > threshold
boolean exitVal; // true when exit IR sensor value is > threshold
uint32_t movement_ts; // timestamp for last recorded movement

void calibrate_sensors() {
  uint32_t end_t = millis() + 10000;
  delay(50);
  int max_ent = 0;
  int max_exit = 0;
  int val1, val2;
  Serial.println("Calibrating sensors. Please stand clear of the device...");
  while(millis() < end_t) {
    val1 = analogRead(IR_ENTRY);
    val2 = analogRead(IR_EXIT);
    
    // calibrate sensors for 10 seconds
    if(max_ent < val1) {
      max_ent = val1;
    }
    if(max_exit < val2) {
      max_exit = val2;
    }
    delay(50);
  }
  threshold_entry = max_ent + 30;
  threshold_exit = max_exit + 30;
  Serial.println("Completed calibration. Values: ");
  Serial.println("- Entry threshold: ");
  Serial.println(threshold_entry);
  Serial.println("- Exit threshold: ");
  Serial.println(threshold_exit);
  delay(5000);
}

void connectAWS()
{
  //Begin WiFi in station mode
  WiFi.mode(WIFI_STA); 
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting to Wi-Fi");

  //Wait for WiFi connection
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  // Configure wifi_client with the correct certificates and keys
  wifi_client.setCACert(AWS_CERT_CA);
  wifi_client.setCertificate(AWS_CERT_CRT);
  wifi_client.setPrivateKey(AWS_CERT_PRIVATE);

  //Connect to AWS IOT Broker. 8883 is the port used for MQTT
  mqtt_client.begin(AWS_IOT_ENDPOINT, 8883, wifi_client);

  //Set action to be taken on incoming messages
  mqtt_client.onMessage(incomingMessageHandler);

  Serial.print("Connecting to AWS IOT");

  //Wait for connection to AWS IoT
  while (!mqtt_client.connect(THINGNAME)) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();

  if(!mqtt_client.connected()){
    Serial.println("AWS IoT Timeout!");
    return;
  }

  //Subscribe to a topic
  mqtt_client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);

  Serial.println("AWS IoT Connected!");
}

void publishMessage()
{
  //Create a JSON document of size 200 bytes, and populate it
  //See https://arduinojson.org/
  StaticJsonDocument<200> doc;
  doc["room_name"] = "room1";
  //doc["timestamp"] = 
  //int c2 = random(30);
  doc["count"] = String(count);
  //sprintf(doc["count"], "%d", count); // TODO: Update this to actual count
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to mqtt_client

  //Publish to the topic
  mqtt_client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
  Serial.println("-- Sent a message --");
}

void incomingMessageHandler(String &topic, String &payload) {
  Serial.println("Message received!");
  Serial.println("Topic: " + topic);
  Serial.println("Payload: " + payload);
}

void setup() {
  Serial.begin(115200);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  t1 = millis();
  count = 0;
  threshold_entry = 80;
  threshold_exit = 80;
  entNotBlocked = true;
  exitNotBlocked = true;
  movement = false;
  calibrate_sensors();
  connectAWS();
  digitalWrite(RED_LED, LOW);
}

void loop() {
  mqtt_client.loop();
  
  if(count > 0) {
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
  } else {
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, LOW);
  }
  delay(20);
  // read both sensor values
  entryVal = analogRead(IR_ENTRY) > threshold_entry;
  exitVal = analogRead(IR_EXIT) > threshold_exit;

  //Serial.println( analogRead(IR_ENTRY));
  //Serial.println( analogRead(IR_EXIT));
  
  // publish msg over MQTT
  if(millis() > t1){
    t1 = millis() + 60000;
    publishMessage();
  }

  if(movement && (millis() > movement_ts + MOVEMENT_TIMEOUT)) {
    Serial.println("False movement detected - resetting movement");
    movement = false;
    delay(100);
  }

  /** if the entry sensor is blocked by a person and there was no movement
   *  recorded, record movement happening
   */
  if(entryVal && !movement && entNotBlocked) {
    Serial.println("Detected person at exit...");
    entNotBlocked = false;
    movement = true;
    movement_ts = millis();
    delay(80);
  }
   /** If the exit IR sensor was blocked after there was movement detected,
    *  a person has exited the room and the count must be updated
    */
  else if(exitVal && movement && exitNotBlocked) {
    count--;
    if(count < 0) {
      count = 0;
    }
    Serial.println("Count of people in room: ");
    Serial.println(count);
    movement = false;
    exitNotBlocked = false;
    delay(1200);
  }
   /** If the exit IR sensor is blocked by a person and there was no movement
    *  recorded, record movement happening
    */
  else if(exitVal && !movement && exitNotBlocked) {
    Serial.println("Detected person at entrance...");
    exitNotBlocked = false;
    movement = true;
    movement_ts = millis();
    delay(80);
  }
    /** If the entry IR sensor was blocked after there was movement detected,
     *  a person has entered the room and the count must be updated
     */
  else if(entryVal && movement && entNotBlocked) {
    count++;
    Serial.println("Count of people in room: ");
    Serial.println(count);
    movement = false;
    entNotBlocked = false;
    delay(1200);
  }


  // if an IR sensor isn't blocked, indicate this for the next loop iteration
  if(!entryVal) {
    entNotBlocked = true;
  }
  if(!exitVal) {
    exitNotBlocked = true;
  }

  

//  d1 = ir_ent.getDistance();
//  d2 = ir_ext.getDistance();
  //Serial.println(analogRead(33));
  //delay(200);
}
