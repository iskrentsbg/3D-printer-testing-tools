#include <Wire.h>
#include <SPI.h>
#include <WEMOS_SHT3X.h>
#include "MPU9250.h"
#include "Adafruit_CCS811.h"
#include <ESP32Servo360.h>
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif
#include "MqttManager.h"

const char *mqtt_server = "test.mosquitto.org";
const char *ssid = "12connect";
const char *password = "";
const char *environmentTopic = "studentinc/acme/environment";

MqttManager mqttManager(mqtt_server, ssid, password, environmentTopic);

const int irSensorPin = 35;
const int magneticSwitchPin = 33;
const int servoPin = 18;

struct SensorData
{
  float temperature;
  float humidity;
  float roll;
  float pitch;
  float gyroX;
  float gyroY;
  float gyroZ;
  float eCO2;
  float TVOC;
  float filamentPercentage;
  String currentState;
  byte magneticSwitchState;
};

const int INTERVAL = 200;

// initialize thresholds
const float TEMPERATURE_THRESHOLD = 0.5;
const float HUMIDITY_THRESHOLD = 5.0;
const int LEVELING_THRESHOLD = 5;
const float SIGNIFICANCE_THRESHOLD = 2.0;
const float FILAMENT_CHANGE_THRESHOLD = 5.0;

// moduldo variables for the bed leveling and filament change
const int NUM_SAMPLES = 10;
float rollSamples[NUM_SAMPLES];
float pitchSamples[NUM_SAMPLES];
int IRsensorSamples[NUM_SAMPLES];
int sampleIndex = 0;
String previousState;

// initialize sensor data
SensorData bedLevelData;
SensorData previousAirQualityData;
SensorData previousFilamentData;
float previousFilamentPercentage = 0.0;

unsigned long previousTime = 0;
float elapsedTime, elapsedTimeTotal = 0;

const unsigned long SENSOR_UPDATE_INTERVAL = 100;

// Sensor objects
SHT3X sht(0x45);
MPU9250 IMU(Wire, 0x68);
Adafruit_CCS811 ccs;
ESP32Servo360 servo;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  WiFi.begin(ssid, password);
  int status;
  unsigned long startTime = millis();
  unsigned long timeout = 5000; // 5 seconds

  // SHT3X
  while ((status = IMU.begin()) < 0)
  {
    if (millis() - startTime > timeout)
    {
      Serial.println("IMU initialization unsuccessful");
      Serial.println("Please check the connections!");
      // Block indefinitely in case of failure
      while (1)
      {
      }
    }
  }

  // CCS811
  // if (!ccs.begin())
  // {
  //   Serial.println("Failed to CCS start sensor! Please check your wiring.");
  //   while (1)
  //     ;
  // }

  mqttManager.setup();

  // IR sensor
  pinMode(irSensorPin, INPUT);
  adcAttachPin(irSensorPin);

  // Door system
  pinMode(magneticSwitchPin, INPUT_PULLUP);
  // servo.attach(5, 32);
}

// IMU sensor functions
void readIMUSensorData()
{
  IMU.readSensor();

  float ax = IMU.getAccelX_mss();
  float ay = IMU.getAccelY_mss();
  float az = IMU.getAccelZ_mss();

  // roll and pitch angles are calculated from the accelerometer readings
  float newRoll = atan2(ay, az) * 180.0 / PI;
  float newPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  // Add new roll and pitch angles to samples array
  rollSamples[sampleIndex] = newRoll;
  pitchSamples[sampleIndex] = newPitch;

  // Update sample index (Modulo technique)
  sampleIndex = (sampleIndex + 1) % NUM_SAMPLES;

  // Calculate moving average of roll and pitch angles
  bedLevelData.roll = 0;
  bedLevelData.pitch = 0;
  bedLevelData.currentState = "";

  for (int i = 0; i < NUM_SAMPLES; i++)
  {
    bedLevelData.roll += rollSamples[i];
    bedLevelData.pitch += pitchSamples[i];
  }
  bedLevelData.roll /= NUM_SAMPLES;
  bedLevelData.pitch /= NUM_SAMPLES;

  bedLevelData.gyroX = IMU.getGyroX_rads();
  bedLevelData.gyroY = IMU.getGyroY_rads();
  bedLevelData.gyroZ = IMU.getGyroZ_rads();
}

bool isSignificantChange(SensorData &currentData, SensorData &previousData, float threshold)
{
  return (abs(currentData.roll - previousData.roll) > threshold ||
          abs(currentData.pitch - previousData.pitch) > threshold);
}

void updateAngles(unsigned long currentTime)
{
  elapsedTime = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  bedLevelData.roll += bedLevelData.gyroX * elapsedTime;
  bedLevelData.pitch += bedLevelData.gyroY * elapsedTime;
}

void printIMUSensorData()
{
  Serial.print("Roll: ");
  Serial.print(bedLevelData.roll);

  Serial.print("\tPitch: ");
  Serial.print(bedLevelData.pitch);
  Serial.print("\tGyro X: ");
  Serial.print(bedLevelData.gyroX);
  Serial.print("\tGyro Y: ");
  Serial.print(bedLevelData.gyroY);
  Serial.print("\tGyro Z: ");
  Serial.println(bedLevelData.gyroZ);
}

void checkBedLevel()
{
  int state;
  if (abs(bedLevelData.roll) < LEVELING_THRESHOLD && abs(bedLevelData.pitch) < LEVELING_THRESHOLD)
  {
    Serial.println("The bed is level.");
    state = 0;
    bedLevelData.currentState = "L";
  }
  else if (bedLevelData.roll > LEVELING_THRESHOLD)
  {
    Serial.println("The bed is tilted along the X-axis.");
    state = 1;
    bedLevelData.currentState = "X+";
  }
  else if (bedLevelData.roll < -LEVELING_THRESHOLD)
  {
    Serial.println("The bed is tilted along the X-axis.");
    state = 2;
    bedLevelData.currentState = "X-";
  }
  else if (bedLevelData.pitch > LEVELING_THRESHOLD)
  {
    Serial.println("The bed is tilted along the Y-axis.");
    state = 3;
    bedLevelData.currentState = "Y+";
  }
  else if (bedLevelData.pitch < -LEVELING_THRESHOLD)
  {
    Serial.println("The bed is tilted along the Y-axis.");
    state = 4;
    bedLevelData.currentState = "Y-";
  }
  if (bedLevelData.currentState != previousState)
  {
    // publishing bedlevel data
    mqttManager.publish("studentinc/acme/environment/leveling/roll", String(bedLevelData.roll).c_str());
    mqttManager.publish("studentinc/acme/environment/leveling/pitch", String(bedLevelData.pitch).c_str());

    if (bedLevelData.currentState == "L")
    {
      mqttManager.publish("studentinc/acme/environment/leveling", String(bedLevelData.currentState).c_str());
    }
    else if (bedLevelData.currentState == "X+")
    {
      mqttManager.publish("studentinc/acme/environment/leveling", String(bedLevelData.currentState).c_str());
    }
    else if (bedLevelData.currentState == "X-")
    {
      mqttManager.publish("studentinc/acme/environment/leveling", String(bedLevelData.currentState).c_str());
    }
    else if (bedLevelData.currentState == "Y+")
    {
      mqttManager.publish("studentinc/acme/environment/leveling", String(bedLevelData.currentState).c_str());
    }
    else if (bedLevelData.currentState == "Y-")
    {
      mqttManager.publish("studentinc/acme/environment/leveling", String(bedLevelData.currentState).c_str());
    }
    previousState = bedLevelData.currentState;
  }
}

// Air quality sensor functions
void readAirQualitySensorData()
{
  if (ccs.available())
  {
    if (!ccs.readData())
    {
      previousAirQualityData.eCO2 = ccs.geteCO2();
      previousAirQualityData.TVOC = ccs.getTVOC();
      mqttManager.publish(environmentTopic, String(previousAirQualityData.eCO2).c_str());
      mqttManager.publish(environmentTopic, String(previousAirQualityData.TVOC).c_str());
    }
    else
    {
      Serial.println("CCS811 Sensor Error!");
    }
  }
}

// Environment sensor functions
SensorData readEnvironmentSensorData()
{
  byte error = sht.get();
  SensorData tempHumidityData;

  unsigned long startTime = millis();
  while (error != 0)
  {
    Serial.println("Error reading from SHT3X");
    if (millis() - startTime > INTERVAL)
    {
      Serial.println("Sensor reading error, please check the connections!");
      tempHumidityData.temperature = -1.0f;
      tempHumidityData.humidity = -1.0f;
      return tempHumidityData;
    }
    error = sht.get();
  }

  tempHumidityData.temperature = sht.cTemp;
  tempHumidityData.humidity = sht.humidity;

  return tempHumidityData;
}

void displayEnvironmentSensorData(SensorData &data, float &prevTemp, float &prevHum)
{
  // Display the temperature and humidity values only if there is a difference of more than 1.0
  if (abs(data.temperature - prevTemp) > TEMPERATURE_THRESHOLD || abs(data.humidity - prevHum) > HUMIDITY_THRESHOLD)
  {
    Serial.print("Temperature: ");
    Serial.print(data.temperature);
    Serial.print(" °C, Humidity: ");
    Serial.print(data.humidity);
    Serial.println(" %");
    mqttManager.publish("studentinc/acme/environment/temp", String(data.temperature).c_str());
    mqttManager.publish("studentinc/acme/environment/hum", String(data.humidity).c_str());

    prevTemp = data.temperature;
    prevHum = data.humidity;
  }
}

// IR sensor functions
float voltageToDistance(int sensorValue)
{
  return 12.08 * pow(sensorValue / 1024.0, -1.204);
}

bool isSignificantChangeFilaments(float currentPercentage, float previousPercentage)
{
  return abs(currentPercentage - previousPercentage) > FILAMENT_CHANGE_THRESHOLD;
}

SensorData getFilamentData()
{
  SensorData filamentData;

  int sensorValue = analogRead(irSensorPin);

  IRsensorSamples[sampleIndex] = sensorValue;
  sampleIndex = (sampleIndex + 1) % NUM_SAMPLES;

  int averageSensorValue = 0;
  for (int i = 0; i < NUM_SAMPLES; i++)
  {
    averageSensorValue += IRsensorSamples[i];
  }
  averageSensorValue /= NUM_SAMPLES;

  float currentDistance = voltageToDistance(averageSensorValue);
  filamentData.filamentPercentage = (currentDistance - 4) / (30 - 4) * 100;
  filamentData.filamentPercentage = max(filamentData.filamentPercentage, 0.0f);
  filamentData.filamentPercentage = min(filamentData.filamentPercentage, 100.0f);

  if (isSignificantChangeFilaments(filamentData.filamentPercentage, previousFilamentPercentage))
  {
    Serial.println("Significant change in filament percentage detected!");
    Serial.println("Filament percentage: " + String(filamentData.filamentPercentage));
    mqttManager.publish("studentinc/acme/environment/filament", String(filamentData.filamentPercentage).c_str());
  }

  previousFilamentPercentage = filamentData.filamentPercentage;

  return filamentData;
}

// Door system function
void checkMagneticSwitch(SensorData &data)
{
  data.magneticSwitchState = digitalRead(magneticSwitchPin);

  static byte previousState = data.magneticSwitchState;

  if (data.magneticSwitchState != previousState)
  {
    if (data.magneticSwitchState == LOW)
    {
      Serial.println("Door is closed.");
      servo.rotateTo(0);
      mqttManager.publish("studentinc/acme/environment/door", "closed");
    }
    else
    {
      Serial.println("Door is opened.");
      servo.rotateTo(180);
      mqttManager.publish("studentinc/acme/environment/door", "opened");
    }
  }

  previousState = data.magneticSwitchState;
}

void loop()
{
  mqttManager.loop();
  unsigned long currentTime = millis();
  static unsigned long previousTime = 0;
  static float prevTemp = 0.0f;
  static float prevHum = 0.0f;
  static SensorData previousBedLevelData;

  if (currentTime - previousTime >= INTERVAL)
  {
    previousTime = currentTime;
    // IMU sensor functions
    readIMUSensorData();
    updateAngles(currentTime);

    if (isSignificantChange(bedLevelData, previousBedLevelData, SIGNIFICANCE_THRESHOLD))
    {

      printIMUSensorData();
      checkBedLevel();

      previousBedLevelData = bedLevelData;
    }

    // Environment sensor functions
    SensorData data = readEnvironmentSensorData();
    displayEnvironmentSensorData(data, prevTemp, prevHum);

    // Air quality sensor functions
    // readAirQualitySensorData();

    // IR sensor functions
    SensorData filamentData = getFilamentData();

    // Door system function
    checkMagneticSwitch(data);
  }
}