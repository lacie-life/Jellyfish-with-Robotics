// PubNub MQTT example using ESP8266.
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "Timer.h"
#include <SoftwareSerial.h>
// Connection info.
const char* ssid = "........................";
const char* password = "86868686";
const char* mqttServer = "192.168.1.219";
const int mqttPort = 1883;
const char* clientID = "Superman";
const char* Topic = "hello_world";
WiFiClient espClient;
PubSubClient client(espClient);
SoftwareSerial NodeMCU(D2,D3);

void callback(char* topic, byte* payload, unsigned int length) {
  String payload_buff;
  for (int i=0;i<length;i++) {
    payload_buff = payload_buff+String((char)payload[i]);
  }
  Serial.println(payload_buff); // Print out messages.
  Serial.println(payload_buff.toInt());
  Serial.println();
  if(payload_buff.toInt() > 500){
    NodeMCU.print(0);
  }
  else {
    NodeMCU.print(1);
  }
  
}
long lastReconnectAttempt = 0;
boolean reconnect() {
  if (client.connect(clientID)) {
    client.subscribe(Topic); // Subscribe.
  }
  return client.connected();
}
void setup() {
  Serial.begin(9600);
  NodeMCU.begin(9600);
  pinMode(D2,INPUT);
  pinMode(D3,OUTPUT);
  Serial.println("Attempting to connect...");
  WiFi.begin(ssid, password); // Connect to WiFi.
  if(WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.println("Couldn't connect to WiFi.");
      while(1) delay(100);
  }
  client.setServer(mqttServer, mqttPort); // Connect to PubNub.
  client.setCallback(callback);
  lastReconnectAttempt = 0;
}
void loop() {
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) { // Try to reconnect.
      lastReconnectAttempt = now;
      if (reconnect()) { // Attempt to reconnect.
        lastReconnectAttempt = 0;
      }
    }
  } else { // Connected.
    client.loop();
//    client.publish(Topic,"234"); // Publish message.
    delay(500);
  }
}
