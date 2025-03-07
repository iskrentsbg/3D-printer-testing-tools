#include "mqttManager.h"
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

MqttManager::MqttManager(const char *mqttServer, const char *ssid, const char *password, const char *environmentTopic)
    : mqtt_server(mqttServer), ssid(ssid), password(password), environmentTopic(environmentTopic),
      client(espClient)
{
}

void MqttManager::callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");

  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }

  Serial.println();
}

void MqttManager::reconnect()
{
  if (!client.connected())
  {
    while (!client.connected())
    {
      Serial.print("Attempting MQTT connection...");
      if (client.connect("environment"))
      {
        Serial.println("connected");
        client.publish("studentinc/acme/environment", "Nodemcu connected to MQTT");
        client.subscribe("studentinc/acme/environment");
      }
      else
      {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
        delay(5000);
      }
    }
  }
}

void MqttManager::publish(const char *topic, const char *payload)
{
  if (client.connected())
  {
    client.publish(topic, payload);
  }
}

void MqttManager::connectMqtt()
{
  reconnect();
  client.loop();
}

void MqttManager::setup()
{
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  connectMqtt();
}

void MqttManager::loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
}