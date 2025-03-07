#include <Arduino.h>
#include "ntc.h"
#include "fan.h"
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif
#if defined(ESP8266)
#include <ESP8266WebServer.h>
#else
#include <WebServer.h>
#endif
#include <PubSubClient.h>

#define INTERVAL 1000
#define FANTHRESHOLD 34
#define HEATERTHRESHOLD 100
unsigned long startTime;
bool isHeating = false;
bool IsCooling = false;
int threashold = 0;

//===========================
// Settings
//===========================
const char *mqtt_server = "test.mosquitto.org"; // MQTT server
const char *ssid = "12connect";                 // WiFi ssid
const char *password = "";                      // wifi password

WiFiClient espClient;
PubSubClient client(espClient);

void Reconnect()
{
  while (!client.connected())
  {
    while (WiFi.isConnected() == false)
    {
      WiFi.begin(ssid, password);
      Serial.println("Connecting to WiFi..");
      delay(5000);
    }
    Serial.println("Attempting MQTT connection...");
    if (client.connect("extruder"))
    {
      Serial.println("Connected");
      // Once connected, publishes an announcement...
      // ... and resubscribes
      client.subscribe("studentinc/acme/extruder/temp");
    }
    else
    {
      Serial.print("fail, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Waits 5 seconds before retrying
      delay(5000);
    }
  }
}

void ConnectMqtt()
{
  client.connect("extruder"); // ESP will connect to MQTT broker with clientID
  Serial.println("Connected to MQTT");
  // Once connected, publishes an announcement...
  // ... and resubscribes
  client.subscribe("studentinc/acme/extruder/temp");
  client.publish("studentinc/acme/extruder/temp", "connected to temperature");
  client.publish("studentinc/acme/extruder/fan", "connected to fan");
  Reconnect();
}

void ReadCommand(char *topic, byte *payload, unsigned int length)
{
  // Callback includes topic where the payload is comming.

  // Prints the topic.
  // Serial.print("Topic: \"");
  // Serial.print(topic);
  // Serial.println("\"");

  // // Prints the received message.
  // Serial.print("Received message: \"");
  for (unsigned int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }

  // Copies the received message in a string.
  // String payload_string = String((char *)payload);
  // Serial.println(payload_string);
  // Reads the message and executes the corresponding task.
  // Format: e.g., "x50y1780z343;"
  // if (payload_string[0] == 's' && payload_string[1] == 'e' && payload_string[2] == 't')
  // {
  //   for (int i = 0; i < length; i++)
  //   {
  //     threashold = payload_string.substring(3, length - 3);
  //   }
  //   isHeating = true;
  // }
  // else if (payload_string[0] == 'O' && payload_string[1] == 'f' && payload_string[2] == 'f')
  // {
  //   isHeating = false;
  // }
  // // Every other message (which is invalid)
  // else
  // {
  //   // client.publish("studentinc/acme/extruder/response", "ERR_INVALID_MESSAGE");
  // }
}

void setup()
{
  Serial.begin(9600);
  pinMode(GPIO_27, OUTPUT);
  pinMode(GPIO_33, OUTPUT);
  startTime = millis();
  WiFi.begin(ssid, password);
  Serial.println("Connected");
  client.setServer(mqtt_server, 1883); // connecting to mqtt server
  client.setCallback(ReadCommand);
  delay(5000);
  ConnectMqtt();
}

void loop()
{
  Reconnect();
  client.loop();
  float temp;
  if (millis() - startTime > INTERVAL)
  {
    startTime = millis();
    
    temp = GetTemperature();
    
    client.publish("studentinc/acme/extruder/temp", String(temp).c_str());
    client.publish("studentinc/acme/extruder", String(IsCooling).c_str());

    if (temp > FANTHRESHOLD)
    {
      ControlFan(100);
      IsCooling = true;
    }
    else
    {
      ControlFan(0);
      IsCooling = true;
    }
    if (temp < HEATERTHRESHOLD && isHeating)
    {
      digitalWrite(GPIO_33, HIGH);
    }
    else
    {
      digitalWrite(GPIO_33, LOW);
    }
  }
}
