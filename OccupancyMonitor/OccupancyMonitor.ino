#include "config.h"

#include <WiFiClientSecure.h>
#include <MQTTClient.h> //MQTT Library Source: https://github.com/256dpi/arduino-mqtt

#include <ArduinoJson.h> //ArduinoJson Library Source: https://github.com/bblanchon/ArduinoJson
#include "WiFi.h"
#include "NewPing.h"

// MQTT topics for the device
#define AWS_IOT_PUBLISH_TOPIC   "occupancymonitor/room1/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "occupancymonitor/room1/sub"

// Infrared sensors
#define IR_CHAIR 33
#define DIST_THRESH 75
// LEDs
#define RED_LED 13
#define GREEN_LED 14

// button
#define BUTTON 23

// timeouts
uint32_t MOVEMENT_TIMEOUT = 1000;

WiFiClientSecure wifi_client = WiFiClientSecure();
MQTTClient mqtt_client = MQTTClient(256); //256 indicates the maximum size for packets being published and received.

uint32_t t1;

// counting
int count;


// button
uint32_t button_ts;
int button_press;
int val;
int buttonState;

// Ultrasonic sensor
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement


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
  doc["count"] = String(count);
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
  Serial.begin(115200); //115200
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  pinMode(BUTTON, INPUT);
  digitalWrite(BUTTON, HIGH);
  t1 = millis();
  count = 0;
  connectAWS();
//  digitalWrite(RED_LED, LOW);
  button_press = 0;
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

  buttonState = digitalRead(BUTTON);
  //Serial.println(buttonState);
  if(buttonState == 0) {
    Serial.println("Button is pushed");
    button_press = 1;
    button_ts = millis();
  }

  if(button_press == 1 && millis() > (button_ts + 30000)) {
    // button press lasts 30 seconds before being reset
    button_press = 0;
  }

  // check IR sensor reading
  val = analogRead(IR_CHAIR); //> threshold_entry;
  //Serial.println(val);

  if(button_press == 1 || val >= DIST_THRESH) {
    // condition met for occupancy of seat
    count = 1;
  } else {
    count = 0;
  }

  delay(200);
  

  // publish msg over MQTT
  if(millis() > t1){
    t1 = millis() + 60000;
    publishMessage();
  }


}
