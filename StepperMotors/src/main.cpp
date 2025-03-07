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

#include "xyz_motors_manipulations.h"

typedef enum
{
	MAIN_STATE_INVALID_MSG,
	MAIN_STATE_RESET,
	MAIN_STATE_TEST,
	MAIN_STATE_STOP,
	MAIN_STATE_MOVE_MOTORS,
	MAIN_STATE_FINISH,
	MAIN_STATE_WAIT_FOR_NEW_MSG
} Main_State;

Main_State main_state = MAIN_STATE_WAIT_FOR_NEW_MSG;

//===========================
// Settings
//===========================
const char *mqtt_server = "test.mosquitto.org"; // MQTT server
const char *ssid = "12connect";					// WiFi ssid
const char *password = "";						// wifi password
// const char *ssid = "Doctor Cuyperslaan 120"; // WiFi ssid
// const char *password = "solutio365";		 // WiFi password

WiFiClient espClient;
PubSubClient client(espClient);

int x_value;
int y_value;
int z_value;

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
		if (client.connect("motor"))
		{
			Serial.println("Connected");
			// Once connected, publishes an announcement...
			// ... and resubscribes
			client.subscribe("studentinc/acme/xyzManipulator");
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
	client.connect("motor"); // ESP will connect to MQTT broker with ID motor
	Serial.println("Connected to MQTT");
	// Once connected, publishes an announcement...
	// ... and resubscribes
	client.subscribe("studentinc/acme/xyzManipulator");

	Reconnect();
}

void ReadCommand(char *topic, byte *payload, unsigned int length)
{
	// Prints the topic.
	Serial.print("Topic: \"");
	Serial.print(topic);
	Serial.println("\"");

	// Prints the received message.
	Serial.print("Received message: \"");
	for (unsigned int i = 0; i < length; i++)
	{
		Serial.print((char)payload[i]);
	}
	Serial.println("\"");

	// Copies the received message in a string.
	String payload_string = String((char *)payload);

	// Reads the message and executes the corresponding task.
	// Format: e.g., "x50y1780z343;" or "x4096y4096z4096;"
	if (payload_string[0] == 'x')
	{
		// Finds the indexes of 'x', 'y', 'z' and ';' (the end) in the payload.
		int x_index = payload_string.indexOf('x');
		int y_index = payload_string.indexOf('y');
		int z_zndex = payload_string.indexOf('z');
		int end_index = payload_string.indexOf(';');

		// Extracts the substrings.
		x_value = payload_string.substring(x_index + 1, y_index).toInt();
		y_value = payload_string.substring(y_index + 1, z_zndex).toInt();
		z_value = payload_string.substring(z_zndex + 1, end_index).toInt();

		main_state = MAIN_STATE_MOVE_MOTORS;
	}
	// Format: "Reset"
	else if (payload_string[0] == 'R' && payload_string[1] == 'e')
	{
		main_state = MAIN_STATE_RESET;
	}
	// Format: "Test"
	else if (payload_string[0] == 'T' && payload_string[1] == 'e')
	{
		main_state = MAIN_STATE_TEST;
	}
	// Format: "Stop"
	else if (payload_string[0] == 'S' && payload_string[1] == 't')
	{
		make_motors_idle();

		main_state = MAIN_STATE_STOP;
	}
	// Every other message (which is invalid)
	else
	{
		main_state = MAIN_STATE_INVALID_MSG;
	}

	client.publish("studentinc/acme/xyzManipulator/response", "ACK");
	Serial.println("Sent message: ACK");


	enable_outputs();
}

void setup()
{
	motors_setup();
	WiFi.begin(ssid, password);
	Serial.println("Connected");
	client.setServer(mqtt_server, 1883); // connecting to mqtt server
	client.setCallback(ReadCommand);
	delay(5000);
	ConnectMqtt();
}

const char *msg_to_send = "";
int msg_to_send_length = 0;

void loop()
{
	Reconnect();
	client.loop();

	switch (main_state)
	{
	case MAIN_STATE_INVALID_MSG:
	{
		msg_to_send = "INVALID_MSG";
		main_state = MAIN_STATE_FINISH;
		break;
	}
	case MAIN_STATE_RESET:
	{
		if (go_homing())
		{
			msg_to_send = "RESET_SUCCESSFUL";
			msg_to_send_length = 16;
			main_state = MAIN_STATE_FINISH;
		}
		break;
	}
	case MAIN_STATE_TEST:
	{
		if (test_motors())
		{
			msg_to_send = "TEST_SUCCESSFUL";
			msg_to_send_length = 15;
			main_state = MAIN_STATE_FINISH;
		}
		break;
	}
	case MAIN_STATE_MOVE_MOTORS:
	{
		if (move_motors(x_value, y_value, z_value))
		{
			msg_to_send = "MOVE_MOTORS_SUCCESSFUL";
			msg_to_send_length = 22;
			main_state = MAIN_STATE_FINISH;
		}
		break;
	}
	case MAIN_STATE_STOP:
	{
		msg_to_send = "STOP_SUCCESSFUL";
		msg_to_send_length = 15;

		main_state = MAIN_STATE_FINISH;
		break;
	}
	case MAIN_STATE_FINISH:
	{
		client.publish("studentinc/acme/xyzManipulator/response", msg_to_send);

		// Prints the sent message.
		Serial.print("Sent message: \"");
		for (unsigned int i = 0; i < msg_to_send_length; i++)
		{
			Serial.print(msg_to_send[i]);
		}
		Serial.println("\"");

		main_state = MAIN_STATE_WAIT_FOR_NEW_MSG;
		disalble_outputs();
		break;
	}
	case MAIN_STATE_WAIT_FOR_NEW_MSG:
	{
		break;
	}
	}
}
