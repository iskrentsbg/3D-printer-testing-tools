using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;
using System.Text;
using System.Threading.Tasks;

namespace Component_Testing_Tool
{
    public class Cabinet
    {
        Label labelHum;
        Label labelTemp;
        Label lbBedLevelling;
        Label labelFilament;
        Label labelDoor;
        Panel panel;

        public Cabinet(Label labelHum, Label labelTemp, Label labelFilament, Label labelDoor, Label lbBedLevelling, Panel panel) {
            this.labelHum = labelHum;
            this.labelTemp = labelTemp;
            this.labelFilament = labelFilament;
            this.labelDoor = labelDoor;
            this.panel = panel;
            this.lbBedLevelling = lbBedLevelling;
        }

        public void UpdateValueCabinet(string message, string dataType)
        {
            switch (dataType)
            {
                case "hum":
                    if (labelHum.InvokeRequired)
                    {
                        labelHum.Invoke(new Action<string, string>(UpdateValueCabinet), $"Humidity: {message}", "hum");
                    }
                    else
                    {
                        labelHum.Text = message;
                    }

                    break;
                case "temp":
                    if (labelTemp.InvokeRequired)
                    {
                        labelHum.Invoke(new Action<string, string>(UpdateValueCabinet), $"Temperature: {message}", "temp");
                    }
                    else
                    {
                        labelTemp.Text = message;
                    }
                    break;
                case "filament":
                    if (labelTemp.InvokeRequired)
                    {
                        labelFilament.Invoke(new Action<string, string>(UpdateValueCabinet), $"Filament level: {message}%", "filament");
                    }
                    else
                    {
                        labelFilament.Text = message;
                    }
                    break;
                case "door":
                    if (labelDoor.InvokeRequired)
                    {
                        labelDoor.Invoke(new Action<string, string>(UpdateValueCabinet), $"Door status: {message}", "door");
                    }
                    else
                    {
                        labelDoor.Text = message;
                    }
                    break;
            }
        }

        public void GetBedLevelling(string bedMessage)
        {
            switch (bedMessage)
            {
                case "L":
                    if (lbBedLevelling.InvokeRequired)
                    {
                        lbBedLevelling.Invoke(new Action<string>(GetBedLevelling), bedMessage);
                    }
                    else
                    {
                        lbBedLevelling.Text = "Bed is levelled";
                        panel.BackColor = System.Drawing.Color.Olive;

                    }
                    break;
                case "X+":
                    if (lbBedLevelling.InvokeRequired)
                    {
                        lbBedLevelling.Invoke(new Action<string>(GetBedLevelling), bedMessage);
                    }
                    else
                    {
                        lbBedLevelling.Text = "Bed is tilted to the left";
                        panel.BackColor = System.Drawing.Color.IndianRed;

                    }
                    break;
                case "X-":
                    if (lbBedLevelling.InvokeRequired)
                    {
                        lbBedLevelling.Invoke(new Action<string>(GetBedLevelling), bedMessage);
                    }
                    else
                    {
                        lbBedLevelling.Text = "Bed is tilted to the right";
                        panel.BackColor = System.Drawing.Color.IndianRed;

                    }
                    break;
                case "Y+":
                    if (lbBedLevelling.InvokeRequired)
                    {
                        lbBedLevelling.Invoke(new Action<string>(GetBedLevelling), bedMessage);
                    }
                    else
                    {
                        lbBedLevelling.Text = "Bed is tilted forward";
                        panel.BackColor = System.Drawing.Color.IndianRed;

                    }
                    break;
                case "Y-":
                    if (lbBedLevelling.InvokeRequired)
                    {
                        lbBedLevelling.Invoke(new Action<string>(GetBedLevelling), bedMessage);
                    }
                    else
                    {
                        lbBedLevelling.Text = "Bed is tilted backward";
                        panel.BackColor = System.Drawing.Color.IndianRed;

                    }
                    break;
                default:
                    if (lbBedLevelling.InvokeRequired)
                    {
                        lbBedLevelling.Invoke(new Action<string>(GetBedLevelling), bedMessage);
                    }
                    else
                    {
                        lbBedLevelling.Text = "No data";
                        panel.BackColor = System.Drawing.Color.IndianRed;

                    }
                    break;
            }

        }



    }
}
