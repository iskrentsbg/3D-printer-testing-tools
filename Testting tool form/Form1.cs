using MQTTclient;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace Component_Testing_Tool
{
    public partial class Form1 : Form
    {
        Client client;
        int maximumMapBorder = 0;
        const string stepperTopic = "studentinc/acme/xyzManipulator";
        const string humTopic = "studentinc/acme/environment/hum";
        const string tempTopic = "studentinc/acme/environment/temp";
        const string extruderTopic = "studentinc/acme/extruder/temp";
        public Form1()
        {
            InitializeComponent();
            client = new Client(log, labelHumidity, labelTemperature,lbBedLevelling, pnLevelling, labelFilament, labelDoor, lbHeaterTemp, lbFanStat);
            SwitchPage("xyz");
            maximumMapBorder = Convert.ToInt32(nudMaxBorder.Value);
        }
        private int MapRange(int value, int originalMin, int originalMax, int newMin, int newMax)
        {
            double percentage = (double)(value - originalMin) / (originalMax - originalMin);
            int mappedValue = (int)(percentage * (newMax - newMin) + newMin);
            return mappedValue;
        }

        private void SwitchPage(string page)
        {
            switch (page)
            {
                case "xyz":
                    IndexPanel.Height = BtnXYZTab.Height;
                    IndexPanel.Top = BtnXYZTab.Top;
                    PanelEnvoirement.Visible = false;
                    PanelHeater.Visible = false;
                    Panelxyz.Visible = true;
                    break;
                case "heater":
                    IndexPanel.Height = BtnHeaterTab.Height;
                    IndexPanel.Top = BtnHeaterTab.Top;
                    PanelEnvoirement.Visible = false;
                    Panelxyz.Visible = false;
                    PanelHeater.Visible = true;
                    break;
                case "envoirement":
                    IndexPanel.Height = BtnEnvironmentTab.Height;
                    IndexPanel.Top = BtnEnvironmentTab.Top;
                    Panelxyz.Visible = false;
                    PanelHeater.Visible = false;
                    PanelEnvoirement.Visible = true;
                    break;
                default:
                    SwitchPage("xyz");
                    break;
            }
        }
        private void UpdatePage(string page)
        {
            switch (page)
            {
                case "xyz":
                    break;
                case "heater":
                    break;
                case "envoirement":
                    break;

            }
        }


        private void BtnXYZTab_Click(object sender, EventArgs e)
        {
            SwitchPage("xyz");
        }

        private void BtnHeaterTab_Click(object sender, EventArgs e)
        {
            SwitchPage("heater");
        }

        private void BtnEnvoirementTab_Click(object sender, EventArgs e)
        {
            SwitchPage("envoirement");
        }

        private void TrackbarX_Scroll(object sender, EventArgs e)
        {
            UpdatePage("xyz");
        }

        private void TrackbarY_Scroll(object sender, EventArgs e)
        {
            UpdatePage("xyz");

        }

        private void TrackbarZ_Scroll(object sender, EventArgs e)
        {
            UpdatePage("xyz");

        }

        private void BtnSetNuD_Click(object sender, EventArgs e)
        {
            client.PublishOnClick($"x{MapRange(Convert.ToInt32(numericUpDownX.Value), 0, 100, 0, maximumMapBorder)}y{MapRange(Convert.ToInt32(numericUpDownY.Value), 0, 100, 0, maximumMapBorder)}z{MapRange(Convert.ToInt32(numericUpDownZ.Value), 0, 100, 0, maximumMapBorder)};", stepperTopic);
            TrackbarX.Value = Convert.ToInt32(numericUpDownX.Value);
            TrackbarY.Value = Convert.ToInt32(numericUpDownY.Value);
            TrackbarZ.Value = Convert.ToInt32(numericUpDownZ.Value);
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {

        }

        private void btnStartTesting_Click(object sender, EventArgs e)
        {
            client.PublishOnClick("Test", stepperTopic);
        }

        private void TrackbarX_MouseUp(object sender, MouseEventArgs e)
        {
            numericUpDownX.Value = TrackbarX.Value;
        }

        private void TrackbarY_MouseUp(object sender, MouseEventArgs e)
        {
            numericUpDownY.Value = TrackbarY.Value;
        }

        private void TrackbarZ_MouseUp(object sender, MouseEventArgs e)
        {
            numericUpDownZ.Value = TrackbarZ.Value;
        }
        private void Form1_FormClosed(object sender, FormClosedEventArgs e)
        {
            client.Unsubscribe();
        }

        private void btnReset_Click(object sender, EventArgs e)
        {
            client.PublishOnClick("Reset", stepperTopic);
        }

        private void btnSetTemp_Click(object sender, EventArgs e)
        {
            client.PublishOnClick($"set{nudSetTemp.Value}", extruderTopic);
        }

        private void btnStopHeating_Click(object sender, EventArgs e)
        {
            client.PublishOnClick($"off", extruderTopic);

        }

        private void cbxDisableX_CheckedChanged(object sender, EventArgs e)
        {
            TrackbarX.Enabled = cbxDisableX.Checked;
            numericUpDownX.Enabled = cbxDisableX.Checked;
            if (cbxDisableX.Checked)
            {
                cbxDisableX.Text = "Enabled";
            }
            else
            {
                cbxDisableX.Text = "Disabled";

            }

        }

        private void cbxDisableY_CheckedChanged(object sender, EventArgs e)
        {
            TrackbarY.Enabled = cbxDisableY.Checked;
            numericUpDownY.Enabled = cbxDisableY.Checked;

            if (cbxDisableY.Checked)
            {
                cbxDisableY.Text = "Enabled";
            }
            else
            {
                cbxDisableY.Text = "Disabled";

            }

        }

        private void cbxDisableZ_CheckedChanged(object sender, EventArgs e)
        {
            TrackbarZ.Enabled = cbxDisableZ.Checked;
            numericUpDownZ.Enabled = cbxDisableZ.Checked;

            if (cbxDisableZ.Checked)
            {
                cbxDisableZ.Text = "Enabled";
            }
            else
            {
                cbxDisableZ.Text = "Disabled";

            }
        }

        private void nudMaxBorder_ValueChanged(object sender, EventArgs e)
        {
            maximumMapBorder = Convert.ToInt32(nudMaxBorder.Value);
        }

        private void btnStop_Click(object sender, EventArgs e)
        {
            client.PublishOnClick("Stop", stepperTopic);
        }
    }
}
