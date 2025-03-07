/*
 * MQTT Client example
 * 
 * Course     : Embedded Systems 2
 * Assignment : Final Group Assignment
 * Author     : Gerald H. Hilderink <g.hilderink@fontys.nl>
 */

using Component_Testing_Tool;
using System;
using System.Diagnostics;
using System.Text;
using System.Windows.Forms;
using uPLibrary.Networking.M2Mqtt;
using uPLibrary.Networking.M2Mqtt.Messages;

// 
// Create a Windows Desktop application (e.g. name it MQTTclient) in Visual Studio.
// Go to Tools->NuGet Package Manager->Manage NuGet Packaged for Solution...
// and search for mqtt and install 'M2Mqtt by Paolo Patiemo' to the solution.
// Now you can use this example.
//
// Install MQTT.fx to access the MQTT broker to see what are in the topics.
//
// Links:
// https://mosquitto.org/
// https://www.hivemq.com/blog/mqtt-client-library-encyclopedia-m2mqtt/

/// <summary>
/// The MQTT Client connects to a MQTT broker and subscribes to a topic.
/// The client also publishes a message to the same topic. As a result, the 
/// client will immediately be notified and it receives the messages due to 
/// the subscription. The received messages are printed in the console (listbox).
/// </summary>
namespace MQTTclient
{
    class Client
    {

        //===========================================
        // CONSTANTS
        //===========================================

        // The url address of MQTT broker.
        const string BROKER_URL = "test.mosquitto.org";

        //===========================================
        // GLOBAL VARIABLES
        //===========================================

        MqttClient client;
        ListBox console;
        Extruder extruder;
        Cabinet cabinet;
        string message;

        //===========================================
        // PROTOTYPES AND DELEGATES
        //===========================================

        // Helpers for ListBox control.
        public delegate void AddListItem();
        public AddListItem myDelegate;

        //===========================================
        // CONSTRUCTORS AND DESTRUCTOR
        //===========================================

        /// <summary>
        /// Instantiate a client with specified listbox to print messages.
        /// </summary>
        /// <param name="console">The listbox to print messages.</param>
        public Client(ListBox console, Label tbHum, Label tbTmp, Label labelBed, Panel levelPanel, Label labelFilament, Label labelDoor, Label lbExtTemp, Label lbFanStat)
        {
            this.console = console;
            extruder = new Extruder(lbExtTemp, lbFanStat);
            cabinet = new Cabinet(tbHum, tbTmp, labelFilament, labelDoor, labelBed, levelPanel);

            myDelegate = new AddListItem(AddListItemMethod);
            //
            // Instantiate MQTT client object.
            //
            //MqttClient client = new MqttClient("broker.hivemq.com");
            client = new MqttClient(BROKER_URL);
            //
            // Set the events. Add them if necessary.
            //
            //client.MqttMsgPublished += Client_MqttMsgPublished;
            client.MqttMsgPublishReceived += Client_MqttMsgPublishReceived;
            client.MqttMsgSubscribed += Client_MqttMsgSubscribed;
            client.MqttMsgUnsubscribed += Client_MqttMsgUnsubscribed;
            //
            // Connect to MQTT broker with a unique id.
            //
            byte code = client.Connect(Guid.NewGuid().ToString());
            //console.Items.Add(code);

            ushort msgId = client.Subscribe(new string[] { "studentinc/acme/xyzManipulator" },
                   new byte[] { MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE });
            //console.Items.Add(msgId);

            ushort msgId2 = client.Subscribe(new string[] { "studentinc/acme/environment/hum" },
                   new byte[] { MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE });
            //console.Items.Add(msgId2);

            ushort msgId3 = client.Subscribe(new string[] { "studentinc/acme/environment/temp" },
            new byte[] { MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE });
            //console.Items.Add(msgId3);
            ushort msgId4 = client.Subscribe(new string[] { "studentinc/acme/environment/leveling" },
            new byte[] { MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE });
            //console.Items.Add(msgId4);
            ushort msgId5 = client.Subscribe(new string[] { "studentinc/acme/environment/filament" },
            new byte[] { MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE });
            //console.Items.Add(msgId5);
            ushort msgId6 = client.Subscribe(new string[] { "studentinc/acme/environment/door" },
            new byte[] { MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE });
            //console.Items.Add(msgId5);
            ushort msgId7 = client.Subscribe(new string[] { "studentinc/acme/xyzManipulator/response" },
            new byte[] { MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE });
            ushort msgId8 = client.Subscribe(new string[] { "studentinc/acme/extruder/temp" },
            new byte[] { MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE });
            ushort msgId9 = client.Subscribe(new string[] { "studentinc/acme/extruder/fan" },
            new byte[] { MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE });
            //console.Items.Add(msgId5);

            //
            // Lets publish something to a topic on the MQTT broker.
            //

        }

        /// <summary>
        /// Destruct the client.
        /// </summary>
        ~Client()
        {
            if (client.IsConnected)
            {
                client.Disconnect();
            }
        }

        //===========================================
        // METHODS
        //===========================================
        public void PublishOnClick(string message, string topic)
        {
            ushort msgId = client.Publish(topic, // topic
            Encoding.UTF8.GetBytes(message), // message body, empty string clears the topic.
            MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE, // QoS level
            true); // retained

            console.Items.Add(msgId);

        }
        /// <summary>
        /// Unsubscribe the client from all subscribed topics.
        /// </summary>
        public void Unsubscribe()
        {
            client.Unsubscribe(new string[] { "studentinc/acme/xyzManipulator" });
            client.Unsubscribe(new string[] { "studentinc/acme/xyzManipulator/response" });
            client.Unsubscribe(new string[] { "studentinc/acme/environment/leveling" });
            client.Unsubscribe(new string[] { "studentinc/acme/environment/filament" });
            client.Unsubscribe(new string[] { "studentinc/acme/environment/door" });
            client.Unsubscribe(new string[] { "studentinc/acme/environment/hum" });
            client.Unsubscribe(new string[] { "studentinc/acme/environment/temp" });
            client.Unsubscribe(new string[] { "studentinc/acme/extruder/fan" });
            client.Unsubscribe(new string[] { "studentinc/acme/extruder/temp" });
        }

        /// <summary>
        /// Disconnect from the mqtt broker.
        /// </summary>
        public void Disconnect()
        {
            if (client.IsConnected)
            {
                client.Disconnect();
            }
        }

        private void AddListItemMethod()
        {
            console.Items.Insert(0, message);
        }

        //===========================================
        // EVENTS HANDLING METHODS
        //===========================================

        void Client_MqttMsgSubscribed(object sender, MqttMsgSubscribedEventArgs e)
        {
            message = "Subscribed " + e.MessageId;
            console.Invoke(myDelegate);
        }

        private void Client_MqttMsgPublishReceived(object sender, MqttMsgPublishEventArgs e)
        {
            if (e.Topic == "studentinc/acme/environment/hum")
            {

                message = Encoding.Default.GetString(e.Message);
                cabinet.UpdateValueCabinet(message, "hum");
                console.Invoke(myDelegate);
            }
            else if (e.Topic == "studentinc/acme/environment/temp")
            {

                message = Encoding.Default.GetString(e.Message);
                cabinet.UpdateValueCabinet(message, "temp");
                console.Invoke(myDelegate);

            }
            else if (e.Topic == "studentinc/acme/xyzManipulator")
            {
                message = Encoding.Default.GetString(e.Message);
                console.Invoke(myDelegate);
            }

            else if (e.Topic == "studentinc/acme/environment/leveling")
            {
                message = Encoding.Default.GetString(e.Message);
                cabinet.GetBedLevelling(message);
                console.Invoke(myDelegate);
            }
            else if (e.Topic == "studentinc/acme/environment/filament")
            {
                message = Encoding.Default.GetString(e.Message);
                cabinet.UpdateValueCabinet(message, "filament");
                console.Invoke(myDelegate);
            }
            else if (e.Topic == "studentinc/acme/environment/door")
            {
                message = Encoding.Default.GetString(e.Message);
                cabinet.UpdateValueCabinet(message, "door");
                console.Invoke(myDelegate);
            }
            else if (e.Topic == "studentinc/acme/xyzManipulator/response")
            {
                message = Encoding.Default.GetString(e.Message);
                console.Invoke(myDelegate);
            }
            else if (e.Topic == "studentinc/acme/extruder/fan")
            {
                message = Encoding.Default.GetString(e.Message);
                extruder.HandleHeatingMessage(message, "fan");
                console.Invoke(myDelegate);

            }
            else if (e.Topic == "studentinc/acme/extruder/temp")
            {
                message = Encoding.Default.GetString(e.Message);
                if (float.TryParse(message, out float val))
                {
                    extruder.HandleHeatingMessage(message, "temp");
                    console.Invoke(myDelegate);
                }

            }
        }

        private void Client_MqttMsgPublished(object sender, MqttMsgPublishedEventArgs e)
        {
            message = "Published " + e.MessageId.ToString();
            console.Invoke(myDelegate);
        }

        private void Client_MqttMsgUnsubscribed(object sender, MqttMsgUnsubscribedEventArgs e)
        {
            // Cannot print to console, because console is closed at FormClosed().
        }
    }

}
