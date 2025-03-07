using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;

using System.Text;
using System.Threading.Tasks;

namespace Component_Testing_Tool
{
    public class Extruder
    {
        Label lbExtTemp;
        Label lbFanStat;
        public Extruder(Label lbExtTemp, Label lbFanStat) { 
            this.lbExtTemp = lbExtTemp;
            this.lbFanStat = lbFanStat;
        }

        public void HandleHeatingMessage(string heatingMessage, string dataType)
        {
            switch (dataType)
            {
                case "temp":
                    if (lbExtTemp.InvokeRequired)
                    {
                        lbExtTemp.Invoke(new Action<string, string>(HandleHeatingMessage), $"Current temperature: {heatingMessage} °C", "temp");
                    }
                    else
                    {
                        lbExtTemp.Text = heatingMessage;
                    }
                    break;
                case "fan":
                    if (lbFanStat.InvokeRequired)
                    {
                        lbFanStat.Invoke(new Action<string, string>(HandleHeatingMessage), $"Fan status: {heatingMessage}", "fan");
                    }
                    else
                    {
                        lbFanStat.Text = heatingMessage;
                    }
                    break;
                default:
                    if (lbExtTemp.InvokeRequired)
                    {
                        lbExtTemp.Invoke(new Action<string, string>(HandleHeatingMessage), $"Current temperature: N/A");
                    }
                    else
                    {
                        lbExtTemp.Text = heatingMessage;
                    }
                    break;


            }
        }


    }
}
