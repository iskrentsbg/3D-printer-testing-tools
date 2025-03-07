#ifndef MQTTMANAGER_H
#define MQTTMANAGER_H

#include <PubSubClient.h>
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

class MqttManager {
public:
  MqttManager(const char *mqttServer, const char *ssid, const char *password, const char *environmentTopic);
  void setup();
  void loop();
  void publish(const char* topic, const char* payload);

private:
  const char *mqtt_server;
  const char *ssid;
  const char *password;
  const char *environmentTopic;
  WiFiClient espClient;
  PubSubClient client;

  void reconnect();
  void connectMqtt();
  static void callback(char *topic, byte *payload, unsigned int length);
};

#endif
