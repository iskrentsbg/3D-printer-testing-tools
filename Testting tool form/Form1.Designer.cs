namespace Component_Testing_Tool
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Form1));
            this.PanelIndex = new System.Windows.Forms.Panel();
            this.pictureBox1 = new System.Windows.Forms.PictureBox();
            this.PageDivider = new System.Windows.Forms.Panel();
            this.IndexPanel = new System.Windows.Forms.Panel();
            this.log = new System.Windows.Forms.ListBox();
            this.BtnXYZTab = new System.Windows.Forms.Button();
            this.BtnHeaterTab = new System.Windows.Forms.Button();
            this.BtnEnvironmentTab = new System.Windows.Forms.Button();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.serialPort1 = new System.IO.Ports.SerialPort(this.components);
            this.Panelxyz = new System.Windows.Forms.Panel();
            this.label2 = new System.Windows.Forms.Label();
            this.nudMaxBorder = new System.Windows.Forms.NumericUpDown();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.cbxDisableZ = new System.Windows.Forms.CheckBox();
            this.cbxDisableY = new System.Windows.Forms.CheckBox();
            this.cbxDisableX = new System.Windows.Forms.CheckBox();
            this.btnReset = new System.Windows.Forms.Button();
            this.LabelXAxis = new System.Windows.Forms.Label();
            this.btnStartTesting = new System.Windows.Forms.Button();
            this.labelY = new System.Windows.Forms.Label();
            this.BtnSetNuD = new System.Windows.Forms.Button();
            this.LabelZ = new System.Windows.Forms.Label();
            this.numericUpDownZ = new System.Windows.Forms.NumericUpDown();
            this.TrackbarX = new System.Windows.Forms.TrackBar();
            this.numericUpDownY = new System.Windows.Forms.NumericUpDown();
            this.TrackbarY = new System.Windows.Forms.TrackBar();
            this.numericUpDownX = new System.Windows.Forms.NumericUpDown();
            this.TrackbarZ = new System.Windows.Forms.TrackBar();
            this.labelxyz = new System.Windows.Forms.Label();
            this.PanelEnvoirement = new System.Windows.Forms.Panel();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.pnLevelling = new System.Windows.Forms.Panel();
            this.lbBedLevelling = new System.Windows.Forms.Label();
            this.gbCab = new System.Windows.Forms.GroupBox();
            this.labelDoor = new System.Windows.Forms.Label();
            this.labelFilament = new System.Windows.Forms.Label();
            this.labelTemperature = new System.Windows.Forms.Label();
            this.labelHumidity = new System.Windows.Forms.Label();
            this.Labelenvoirement = new System.Windows.Forms.Label();
            this.PanelHeater = new System.Windows.Forms.Panel();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.lbFanStat = new System.Windows.Forms.Label();
            this.lbHeaterTemp = new System.Windows.Forms.Label();
            this.button6 = new System.Windows.Forms.Button();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.btnStopHeating = new System.Windows.Forms.Button();
            this.label1 = new System.Windows.Forms.Label();
            this.btnSetTemp = new System.Windows.Forms.Button();
            this.button3 = new System.Windows.Forms.Button();
            this.nudSetTemp = new System.Windows.Forms.NumericUpDown();
            this.labelHeater = new System.Windows.Forms.Label();
            this.btnStop = new System.Windows.Forms.Button();
            this.PanelIndex.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).BeginInit();
            this.PageDivider.SuspendLayout();
            this.Panelxyz.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.nudMaxBorder)).BeginInit();
            this.groupBox2.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownZ)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.TrackbarX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownY)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.TrackbarY)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.TrackbarZ)).BeginInit();
            this.PanelEnvoirement.SuspendLayout();
            this.groupBox1.SuspendLayout();
            this.gbCab.SuspendLayout();
            this.PanelHeater.SuspendLayout();
            this.groupBox4.SuspendLayout();
            this.groupBox3.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.nudSetTemp)).BeginInit();
            this.SuspendLayout();
            // 
            // PanelIndex
            // 
            this.PanelIndex.Controls.Add(this.pictureBox1);
            this.PanelIndex.Controls.Add(this.PageDivider);
            this.PanelIndex.Controls.Add(this.log);
            this.PanelIndex.Controls.Add(this.BtnXYZTab);
            this.PanelIndex.Controls.Add(this.BtnHeaterTab);
            this.PanelIndex.Controls.Add(this.BtnEnvironmentTab);
            this.PanelIndex.Dock = System.Windows.Forms.DockStyle.Left;
            this.PanelIndex.Location = new System.Drawing.Point(0, 0);
            this.PanelIndex.Margin = new System.Windows.Forms.Padding(5, 4, 5, 4);
            this.PanelIndex.Name = "PanelIndex";
            this.PanelIndex.Size = new System.Drawing.Size(218, 573);
            this.PanelIndex.TabIndex = 21;
            // 
            // pictureBox1
            // 
            this.pictureBox1.Image = ((System.Drawing.Image)(resources.GetObject("pictureBox1.Image")));
            this.pictureBox1.Location = new System.Drawing.Point(25, 9);
            this.pictureBox1.Name = "pictureBox1";
            this.pictureBox1.Size = new System.Drawing.Size(153, 75);
            this.pictureBox1.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
            this.pictureBox1.TabIndex = 25;
            this.pictureBox1.TabStop = false;
            // 
            // PageDivider
            // 
            this.PageDivider.BackColor = System.Drawing.Color.Black;
            this.PageDivider.Controls.Add(this.IndexPanel);
            this.PageDivider.Location = new System.Drawing.Point(207, 4);
            this.PageDivider.Margin = new System.Windows.Forms.Padding(5, 4, 5, 4);
            this.PageDivider.Name = "PageDivider";
            this.PageDivider.Size = new System.Drawing.Size(7, 565);
            this.PageDivider.TabIndex = 20;
            // 
            // IndexPanel
            // 
            this.IndexPanel.BackColor = System.Drawing.Color.DarkOrange;
            this.IndexPanel.Location = new System.Drawing.Point(0, 146);
            this.IndexPanel.Margin = new System.Windows.Forms.Padding(5, 4, 5, 4);
            this.IndexPanel.Name = "IndexPanel";
            this.IndexPanel.Size = new System.Drawing.Size(14, 90);
            this.IndexPanel.TabIndex = 20;
            // 
            // log
            // 
            this.log.FormattingEnabled = true;
            this.log.ItemHeight = 21;
            this.log.Location = new System.Drawing.Point(0, 387);
            this.log.Name = "log";
            this.log.Size = new System.Drawing.Size(204, 151);
            this.log.TabIndex = 11;
            // 
            // BtnXYZTab
            // 
            this.BtnXYZTab.FlatAppearance.BorderColor = System.Drawing.SystemColors.Control;
            this.BtnXYZTab.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.BtnXYZTab.Font = new System.Drawing.Font("Century Gothic", 11.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.BtnXYZTab.Image = ((System.Drawing.Image)(resources.GetObject("BtnXYZTab.Image")));
            this.BtnXYZTab.Location = new System.Drawing.Point(0, 92);
            this.BtnXYZTab.Margin = new System.Windows.Forms.Padding(5, 4, 5, 4);
            this.BtnXYZTab.Name = "BtnXYZTab";
            this.BtnXYZTab.Size = new System.Drawing.Size(197, 90);
            this.BtnXYZTab.TabIndex = 22;
            this.BtnXYZTab.Text = "xyz Axis";
            this.BtnXYZTab.TextImageRelation = System.Windows.Forms.TextImageRelation.ImageAboveText;
            this.BtnXYZTab.UseVisualStyleBackColor = true;
            this.BtnXYZTab.Click += new System.EventHandler(this.BtnXYZTab_Click);
            // 
            // BtnHeaterTab
            // 
            this.BtnHeaterTab.FlatAppearance.BorderColor = System.Drawing.SystemColors.Control;
            this.BtnHeaterTab.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.BtnHeaterTab.Font = new System.Drawing.Font("Century Gothic", 11.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.BtnHeaterTab.Image = ((System.Drawing.Image)(resources.GetObject("BtnHeaterTab.Image")));
            this.BtnHeaterTab.Location = new System.Drawing.Point(0, 190);
            this.BtnHeaterTab.Margin = new System.Windows.Forms.Padding(5, 4, 5, 4);
            this.BtnHeaterTab.Name = "BtnHeaterTab";
            this.BtnHeaterTab.Size = new System.Drawing.Size(197, 90);
            this.BtnHeaterTab.TabIndex = 23;
            this.BtnHeaterTab.Text = "Heater";
            this.BtnHeaterTab.TextImageRelation = System.Windows.Forms.TextImageRelation.ImageAboveText;
            this.BtnHeaterTab.UseVisualStyleBackColor = true;
            this.BtnHeaterTab.Click += new System.EventHandler(this.BtnHeaterTab_Click);
            // 
            // BtnEnvironmentTab
            // 
            this.BtnEnvironmentTab.FlatAppearance.BorderColor = System.Drawing.SystemColors.Control;
            this.BtnEnvironmentTab.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.BtnEnvironmentTab.Font = new System.Drawing.Font("Century Gothic", 11.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.BtnEnvironmentTab.Image = ((System.Drawing.Image)(resources.GetObject("BtnEnvironmentTab.Image")));
            this.BtnEnvironmentTab.Location = new System.Drawing.Point(0, 288);
            this.BtnEnvironmentTab.Margin = new System.Windows.Forms.Padding(5, 4, 5, 4);
            this.BtnEnvironmentTab.Name = "BtnEnvironmentTab";
            this.BtnEnvironmentTab.Size = new System.Drawing.Size(197, 90);
            this.BtnEnvironmentTab.TabIndex = 24;
            this.BtnEnvironmentTab.Text = "Environment";
            this.BtnEnvironmentTab.TextImageRelation = System.Windows.Forms.TextImageRelation.ImageAboveText;
            this.BtnEnvironmentTab.UseVisualStyleBackColor = true;
            this.BtnEnvironmentTab.Click += new System.EventHandler(this.BtnEnvoirementTab_Click);
            // 
            // timer1
            // 
            this.timer1.Enabled = true;
            this.timer1.Interval = 1000;
            // 
            // serialPort1
            // 
            this.serialPort1.PortName = "COM6";
            // 
            // Panelxyz
            // 
            this.Panelxyz.Controls.Add(this.label2);
            this.Panelxyz.Controls.Add(this.nudMaxBorder);
            this.Panelxyz.Controls.Add(this.groupBox2);
            this.Panelxyz.Controls.Add(this.labelxyz);
            this.Panelxyz.Location = new System.Drawing.Point(224, 4);
            this.Panelxyz.Margin = new System.Windows.Forms.Padding(5, 4, 5, 4);
            this.Panelxyz.Name = "Panelxyz";
            this.Panelxyz.Size = new System.Drawing.Size(939, 565);
            this.Panelxyz.TabIndex = 22;
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(509, 162);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(232, 21);
            this.label2.TabIndex = 15;
            this.label2.Text = "Maximum mapping value: ";
            // 
            // nudMaxBorder
            // 
            this.nudMaxBorder.Location = new System.Drawing.Point(706, 160);
            this.nudMaxBorder.Maximum = new decimal(new int[] {
            10000,
            0,
            0,
            0});
            this.nudMaxBorder.Minimum = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.nudMaxBorder.Name = "nudMaxBorder";
            this.nudMaxBorder.Size = new System.Drawing.Size(120, 27);
            this.nudMaxBorder.TabIndex = 14;
            this.nudMaxBorder.Value = new decimal(new int[] {
            4096,
            0,
            0,
            0});
            this.nudMaxBorder.ValueChanged += new System.EventHandler(this.nudMaxBorder_ValueChanged);
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.btnStop);
            this.groupBox2.Controls.Add(this.cbxDisableZ);
            this.groupBox2.Controls.Add(this.cbxDisableY);
            this.groupBox2.Controls.Add(this.cbxDisableX);
            this.groupBox2.Controls.Add(this.btnReset);
            this.groupBox2.Controls.Add(this.LabelXAxis);
            this.groupBox2.Controls.Add(this.btnStartTesting);
            this.groupBox2.Controls.Add(this.labelY);
            this.groupBox2.Controls.Add(this.BtnSetNuD);
            this.groupBox2.Controls.Add(this.LabelZ);
            this.groupBox2.Controls.Add(this.numericUpDownZ);
            this.groupBox2.Controls.Add(this.TrackbarX);
            this.groupBox2.Controls.Add(this.numericUpDownY);
            this.groupBox2.Controls.Add(this.TrackbarY);
            this.groupBox2.Controls.Add(this.numericUpDownX);
            this.groupBox2.Controls.Add(this.TrackbarZ);
            this.groupBox2.Location = new System.Drawing.Point(91, 189);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(735, 299);
            this.groupBox2.TabIndex = 13;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "Motor control";
            // 
            // cbxDisableZ
            // 
            this.cbxDisableZ.AutoSize = true;
            this.cbxDisableZ.Checked = true;
            this.cbxDisableZ.CheckState = System.Windows.Forms.CheckState.Checked;
            this.cbxDisableZ.Location = new System.Drawing.Point(634, 149);
            this.cbxDisableZ.Name = "cbxDisableZ";
            this.cbxDisableZ.Size = new System.Drawing.Size(101, 25);
            this.cbxDisableZ.TabIndex = 16;
            this.cbxDisableZ.Text = "Enabled";
            this.cbxDisableZ.UseVisualStyleBackColor = true;
            this.cbxDisableZ.CheckedChanged += new System.EventHandler(this.cbxDisableZ_CheckedChanged);
            // 
            // cbxDisableY
            // 
            this.cbxDisableY.AutoSize = true;
            this.cbxDisableY.Checked = true;
            this.cbxDisableY.CheckState = System.Windows.Forms.CheckState.Checked;
            this.cbxDisableY.Location = new System.Drawing.Point(634, 100);
            this.cbxDisableY.Name = "cbxDisableY";
            this.cbxDisableY.Size = new System.Drawing.Size(101, 25);
            this.cbxDisableY.TabIndex = 15;
            this.cbxDisableY.Text = "Enabled";
            this.cbxDisableY.UseVisualStyleBackColor = true;
            this.cbxDisableY.CheckedChanged += new System.EventHandler(this.cbxDisableY_CheckedChanged);
            // 
            // cbxDisableX
            // 
            this.cbxDisableX.AutoSize = true;
            this.cbxDisableX.Checked = true;
            this.cbxDisableX.CheckState = System.Windows.Forms.CheckState.Checked;
            this.cbxDisableX.Location = new System.Drawing.Point(634, 49);
            this.cbxDisableX.Name = "cbxDisableX";
            this.cbxDisableX.Size = new System.Drawing.Size(101, 25);
            this.cbxDisableX.TabIndex = 14;
            this.cbxDisableX.Text = "Enabled";
            this.cbxDisableX.UseVisualStyleBackColor = true;
            this.cbxDisableX.CheckedChanged += new System.EventHandler(this.cbxDisableX_CheckedChanged);
            // 
            // btnReset
            // 
            this.btnReset.Location = new System.Drawing.Point(139, 203);
            this.btnReset.Name = "btnReset";
            this.btnReset.Size = new System.Drawing.Size(105, 55);
            this.btnReset.TabIndex = 13;
            this.btnReset.Text = "Reset";
            this.btnReset.UseVisualStyleBackColor = true;
            this.btnReset.Click += new System.EventHandler(this.btnReset_Click);
            // 
            // LabelXAxis
            // 
            this.LabelXAxis.AutoSize = true;
            this.LabelXAxis.Location = new System.Drawing.Point(23, 50);
            this.LabelXAxis.Name = "LabelXAxis";
            this.LabelXAxis.Size = new System.Drawing.Size(86, 21);
            this.LabelXAxis.TabIndex = 1;
            this.LabelXAxis.Text = "Control X";
            // 
            // btnStartTesting
            // 
            this.btnStartTesting.Location = new System.Drawing.Point(28, 203);
            this.btnStartTesting.Name = "btnStartTesting";
            this.btnStartTesting.Size = new System.Drawing.Size(105, 55);
            this.btnStartTesting.TabIndex = 12;
            this.btnStartTesting.Text = "Start testing process";
            this.btnStartTesting.UseVisualStyleBackColor = true;
            this.btnStartTesting.Click += new System.EventHandler(this.btnStartTesting_Click);
            // 
            // labelY
            // 
            this.labelY.AutoSize = true;
            this.labelY.Location = new System.Drawing.Point(24, 101);
            this.labelY.Name = "labelY";
            this.labelY.Size = new System.Drawing.Size(87, 21);
            this.labelY.TabIndex = 2;
            this.labelY.Text = "Control Y";
            // 
            // BtnSetNuD
            // 
            this.BtnSetNuD.Location = new System.Drawing.Point(553, 203);
            this.BtnSetNuD.Name = "BtnSetNuD";
            this.BtnSetNuD.Size = new System.Drawing.Size(75, 27);
            this.BtnSetNuD.TabIndex = 10;
            this.BtnSetNuD.Text = "Set";
            this.BtnSetNuD.UseVisualStyleBackColor = true;
            this.BtnSetNuD.Click += new System.EventHandler(this.BtnSetNuD_Click);
            // 
            // LabelZ
            // 
            this.LabelZ.AutoSize = true;
            this.LabelZ.Location = new System.Drawing.Point(25, 150);
            this.LabelZ.Name = "LabelZ";
            this.LabelZ.Size = new System.Drawing.Size(84, 21);
            this.LabelZ.TabIndex = 3;
            this.LabelZ.Text = "Control Z";
            // 
            // numericUpDownZ
            // 
            this.numericUpDownZ.Location = new System.Drawing.Point(510, 148);
            this.numericUpDownZ.Name = "numericUpDownZ";
            this.numericUpDownZ.Size = new System.Drawing.Size(120, 27);
            this.numericUpDownZ.TabIndex = 9;
            // 
            // TrackbarX
            // 
            this.TrackbarX.BackColor = System.Drawing.Color.Snow;
            this.TrackbarX.Location = new System.Drawing.Point(106, 50);
            this.TrackbarX.Maximum = 100;
            this.TrackbarX.Name = "TrackbarX";
            this.TrackbarX.Size = new System.Drawing.Size(400, 56);
            this.TrackbarX.TabIndex = 4;
            this.TrackbarX.TickFrequency = 5;
            this.TrackbarX.Scroll += new System.EventHandler(this.TrackbarX_Scroll);
            this.TrackbarX.MouseUp += new System.Windows.Forms.MouseEventHandler(this.TrackbarX_MouseUp);
            // 
            // numericUpDownY
            // 
            this.numericUpDownY.Location = new System.Drawing.Point(510, 99);
            this.numericUpDownY.Name = "numericUpDownY";
            this.numericUpDownY.Size = new System.Drawing.Size(120, 27);
            this.numericUpDownY.TabIndex = 8;
            // 
            // TrackbarY
            // 
            this.TrackbarY.Location = new System.Drawing.Point(106, 101);
            this.TrackbarY.Maximum = 100;
            this.TrackbarY.Name = "TrackbarY";
            this.TrackbarY.Size = new System.Drawing.Size(400, 56);
            this.TrackbarY.TabIndex = 5;
            this.TrackbarY.TickFrequency = 5;
            this.TrackbarY.Scroll += new System.EventHandler(this.TrackbarY_Scroll);
            this.TrackbarY.MouseUp += new System.Windows.Forms.MouseEventHandler(this.TrackbarY_MouseUp);
            // 
            // numericUpDownX
            // 
            this.numericUpDownX.Location = new System.Drawing.Point(510, 48);
            this.numericUpDownX.Name = "numericUpDownX";
            this.numericUpDownX.Size = new System.Drawing.Size(120, 27);
            this.numericUpDownX.TabIndex = 7;
            // 
            // TrackbarZ
            // 
            this.TrackbarZ.Location = new System.Drawing.Point(107, 152);
            this.TrackbarZ.Maximum = 100;
            this.TrackbarZ.Name = "TrackbarZ";
            this.TrackbarZ.Size = new System.Drawing.Size(399, 56);
            this.TrackbarZ.TabIndex = 6;
            this.TrackbarZ.TickFrequency = 5;
            this.TrackbarZ.Scroll += new System.EventHandler(this.TrackbarZ_Scroll);
            this.TrackbarZ.MouseUp += new System.Windows.Forms.MouseEventHandler(this.TrackbarZ_MouseUp);
            // 
            // labelxyz
            // 
            this.labelxyz.AutoSize = true;
            this.labelxyz.Font = new System.Drawing.Font("Century Gothic", 22F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelxyz.Location = new System.Drawing.Point(111, 142);
            this.labelxyz.Name = "labelxyz";
            this.labelxyz.Size = new System.Drawing.Size(312, 44);
            this.labelxyz.TabIndex = 0;
            this.labelxyz.Text = "XYZ manipulator";
            // 
            // PanelEnvoirement
            // 
            this.PanelEnvoirement.Controls.Add(this.groupBox1);
            this.PanelEnvoirement.Controls.Add(this.gbCab);
            this.PanelEnvoirement.Controls.Add(this.Labelenvoirement);
            this.PanelEnvoirement.Location = new System.Drawing.Point(224, 4);
            this.PanelEnvoirement.Margin = new System.Windows.Forms.Padding(5, 4, 5, 4);
            this.PanelEnvoirement.Name = "PanelEnvoirement";
            this.PanelEnvoirement.Size = new System.Drawing.Size(939, 565);
            this.PanelEnvoirement.TabIndex = 24;
            this.PanelEnvoirement.Visible = false;
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.pnLevelling);
            this.groupBox1.Controls.Add(this.lbBedLevelling);
            this.groupBox1.Location = new System.Drawing.Point(381, 217);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(535, 160);
            this.groupBox1.TabIndex = 7;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Bed levelling";
            // 
            // pnLevelling
            // 
            this.pnLevelling.BackColor = System.Drawing.Color.IndianRed;
            this.pnLevelling.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
            this.pnLevelling.Location = new System.Drawing.Point(447, 59);
            this.pnLevelling.Name = "pnLevelling";
            this.pnLevelling.Size = new System.Drawing.Size(50, 50);
            this.pnLevelling.TabIndex = 7;
            // 
            // lbBedLevelling
            // 
            this.lbBedLevelling.AutoSize = true;
            this.lbBedLevelling.Font = new System.Drawing.Font("Century Gothic", 20F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbBedLevelling.Location = new System.Drawing.Point(18, 68);
            this.lbBedLevelling.Name = "lbBedLevelling";
            this.lbBedLevelling.Size = new System.Drawing.Size(154, 40);
            this.lbBedLevelling.TabIndex = 6;
            this.lbBedLevelling.Text = "No data";
            // 
            // gbCab
            // 
            this.gbCab.Controls.Add(this.labelDoor);
            this.gbCab.Controls.Add(this.labelFilament);
            this.gbCab.Controls.Add(this.labelTemperature);
            this.gbCab.Controls.Add(this.labelHumidity);
            this.gbCab.Location = new System.Drawing.Point(33, 217);
            this.gbCab.Name = "gbCab";
            this.gbCab.Size = new System.Drawing.Size(341, 252);
            this.gbCab.TabIndex = 5;
            this.gbCab.TabStop = false;
            this.gbCab.Text = "Current state";
            // 
            // labelDoor
            // 
            this.labelDoor.AutoSize = true;
            this.labelDoor.Font = new System.Drawing.Font("Century Gothic", 20F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelDoor.Location = new System.Drawing.Point(6, 190);
            this.labelDoor.Name = "labelDoor";
            this.labelDoor.Size = new System.Drawing.Size(280, 40);
            this.labelDoor.TabIndex = 6;
            this.labelDoor.Text = "Door status: N/A";
            // 
            // labelFilament
            // 
            this.labelFilament.AutoSize = true;
            this.labelFilament.Font = new System.Drawing.Font("Century Gothic", 20F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelFilament.Location = new System.Drawing.Point(6, 143);
            this.labelFilament.Name = "labelFilament";
            this.labelFilament.Size = new System.Drawing.Size(319, 40);
            this.labelFilament.TabIndex = 5;
            this.labelFilament.Text = "Filament level: N/A";
            // 
            // labelTemperature
            // 
            this.labelTemperature.AutoSize = true;
            this.labelTemperature.Font = new System.Drawing.Font("Century Gothic", 20F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelTemperature.Location = new System.Drawing.Point(6, 90);
            this.labelTemperature.Name = "labelTemperature";
            this.labelTemperature.Size = new System.Drawing.Size(306, 40);
            this.labelTemperature.TabIndex = 4;
            this.labelTemperature.Text = "Temperature: N/A";
            // 
            // labelHumidity
            // 
            this.labelHumidity.AutoSize = true;
            this.labelHumidity.Font = new System.Drawing.Font("Century Gothic", 20F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelHumidity.Location = new System.Drawing.Point(6, 41);
            this.labelHumidity.Name = "labelHumidity";
            this.labelHumidity.Size = new System.Drawing.Size(240, 40);
            this.labelHumidity.TabIndex = 3;
            this.labelHumidity.Text = "Humidity: N/A";
            // 
            // Labelenvoirement
            // 
            this.Labelenvoirement.AutoSize = true;
            this.Labelenvoirement.Font = new System.Drawing.Font("Century Gothic", 22F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.Labelenvoirement.Location = new System.Drawing.Point(50, 160);
            this.Labelenvoirement.Name = "Labelenvoirement";
            this.Labelenvoirement.Size = new System.Drawing.Size(385, 44);
            this.Labelenvoirement.TabIndex = 0;
            this.Labelenvoirement.Text = "Printing environment";
            // 
            // PanelHeater
            // 
            this.PanelHeater.Controls.Add(this.groupBox4);
            this.PanelHeater.Controls.Add(this.groupBox3);
            this.PanelHeater.Controls.Add(this.labelHeater);
            this.PanelHeater.Location = new System.Drawing.Point(224, 4);
            this.PanelHeater.Margin = new System.Windows.Forms.Padding(5, 4, 5, 4);
            this.PanelHeater.Name = "PanelHeater";
            this.PanelHeater.Size = new System.Drawing.Size(939, 565);
            this.PanelHeater.TabIndex = 25;
            this.PanelHeater.Visible = false;
            // 
            // groupBox4
            // 
            this.groupBox4.Controls.Add(this.lbFanStat);
            this.groupBox4.Controls.Add(this.lbHeaterTemp);
            this.groupBox4.Controls.Add(this.button6);
            this.groupBox4.Location = new System.Drawing.Point(80, 211);
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.Size = new System.Drawing.Size(444, 166);
            this.groupBox4.TabIndex = 15;
            this.groupBox4.TabStop = false;
            this.groupBox4.Text = "Heater control";
            // 
            // lbFanStat
            // 
            this.lbFanStat.AutoSize = true;
            this.lbFanStat.Font = new System.Drawing.Font("Century Gothic", 20F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbFanStat.Location = new System.Drawing.Point(6, 88);
            this.lbFanStat.Name = "lbFanStat";
            this.lbFanStat.Size = new System.Drawing.Size(261, 40);
            this.lbFanStat.TabIndex = 11;
            this.lbFanStat.Text = "Fan status: N/A";
            // 
            // lbHeaterTemp
            // 
            this.lbHeaterTemp.AutoSize = true;
            this.lbHeaterTemp.Font = new System.Drawing.Font("Century Gothic", 20F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbHeaterTemp.Location = new System.Drawing.Point(6, 42);
            this.lbHeaterTemp.Name = "lbHeaterTemp";
            this.lbHeaterTemp.Size = new System.Drawing.Size(447, 40);
            this.lbHeaterTemp.TabIndex = 1;
            this.lbHeaterTemp.Text = "Current temperature:  N/A";
            // 
            // button6
            // 
            this.button6.Location = new System.Drawing.Point(638, 98);
            this.button6.Name = "button6";
            this.button6.Size = new System.Drawing.Size(75, 27);
            this.button6.TabIndex = 10;
            this.button6.Text = "Set";
            this.button6.UseVisualStyleBackColor = true;
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.btnStopHeating);
            this.groupBox3.Controls.Add(this.label1);
            this.groupBox3.Controls.Add(this.btnSetTemp);
            this.groupBox3.Controls.Add(this.button3);
            this.groupBox3.Controls.Add(this.nudSetTemp);
            this.groupBox3.Location = new System.Drawing.Point(550, 211);
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.Size = new System.Drawing.Size(325, 166);
            this.groupBox3.TabIndex = 14;
            this.groupBox3.TabStop = false;
            this.groupBox3.Text = "Heater control";
            // 
            // btnStopHeating
            // 
            this.btnStopHeating.Location = new System.Drawing.Point(132, 87);
            this.btnStopHeating.Name = "btnStopHeating";
            this.btnStopHeating.Size = new System.Drawing.Size(105, 55);
            this.btnStopHeating.TabIndex = 13;
            this.btnStopHeating.Text = "Stop heating";
            this.btnStopHeating.UseVisualStyleBackColor = true;
            this.btnStopHeating.Click += new System.EventHandler(this.btnStopHeating_Click);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Font = new System.Drawing.Font("Century Gothic", 20F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.Location = new System.Drawing.Point(6, 40);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(284, 40);
            this.label1.TabIndex = 1;
            this.label1.Text = "Set temperature";
            // 
            // btnSetTemp
            // 
            this.btnSetTemp.Location = new System.Drawing.Point(12, 87);
            this.btnSetTemp.Name = "btnSetTemp";
            this.btnSetTemp.Size = new System.Drawing.Size(105, 55);
            this.btnSetTemp.TabIndex = 12;
            this.btnSetTemp.Text = "Set";
            this.btnSetTemp.UseVisualStyleBackColor = true;
            this.btnSetTemp.Click += new System.EventHandler(this.btnSetTemp_Click);
            // 
            // button3
            // 
            this.button3.Location = new System.Drawing.Point(638, 98);
            this.button3.Name = "button3";
            this.button3.Size = new System.Drawing.Size(75, 27);
            this.button3.TabIndex = 10;
            this.button3.Text = "Set";
            this.button3.UseVisualStyleBackColor = true;
            // 
            // nudSetTemp
            // 
            this.nudSetTemp.Location = new System.Drawing.Point(243, 50);
            this.nudSetTemp.Maximum = new decimal(new int[] {
            150,
            0,
            0,
            0});
            this.nudSetTemp.Name = "nudSetTemp";
            this.nudSetTemp.Size = new System.Drawing.Size(55, 27);
            this.nudSetTemp.TabIndex = 7;
            this.nudSetTemp.Value = new decimal(new int[] {
            50,
            0,
            0,
            0});
            // 
            // labelHeater
            // 
            this.labelHeater.AutoSize = true;
            this.labelHeater.Font = new System.Drawing.Font("Century Gothic", 22F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelHeater.Location = new System.Drawing.Point(73, 123);
            this.labelHeater.Name = "labelHeater";
            this.labelHeater.Size = new System.Drawing.Size(139, 44);
            this.labelHeater.TabIndex = 1;
            this.labelHeater.Text = "Heater";
            // 
            // btnStop
            // 
            this.btnStop.ForeColor = System.Drawing.Color.Red;
            this.btnStop.Location = new System.Drawing.Point(250, 203);
            this.btnStop.Name = "btnStop";
            this.btnStop.Size = new System.Drawing.Size(105, 55);
            this.btnStop.TabIndex = 17;
            this.btnStop.Text = "STOP";
            this.btnStop.UseVisualStyleBackColor = true;
            this.btnStop.Click += new System.EventHandler(this.btnStop_Click);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(10F, 21F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.Color.Snow;
            this.ClientSize = new System.Drawing.Size(1169, 573);
            this.Controls.Add(this.PanelIndex);
            this.Controls.Add(this.PanelHeater);
            this.Controls.Add(this.Panelxyz);
            this.Controls.Add(this.PanelEnvoirement);
            this.Font = new System.Drawing.Font("Century Gothic", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedDialog;
            this.Margin = new System.Windows.Forms.Padding(5, 4, 5, 4);
            this.MaximizeBox = false;
            this.Name = "Form1";
            this.Text = "Acme-Testing-Tool";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.Form1_FormClosing);
            this.FormClosed += new System.Windows.Forms.FormClosedEventHandler(this.Form1_FormClosed);
            this.PanelIndex.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).EndInit();
            this.PageDivider.ResumeLayout(false);
            this.Panelxyz.ResumeLayout(false);
            this.Panelxyz.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.nudMaxBorder)).EndInit();
            this.groupBox2.ResumeLayout(false);
            this.groupBox2.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownZ)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.TrackbarX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownY)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.TrackbarY)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.TrackbarZ)).EndInit();
            this.PanelEnvoirement.ResumeLayout(false);
            this.PanelEnvoirement.PerformLayout();
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.gbCab.ResumeLayout(false);
            this.gbCab.PerformLayout();
            this.PanelHeater.ResumeLayout(false);
            this.PanelHeater.PerformLayout();
            this.groupBox4.ResumeLayout(false);
            this.groupBox4.PerformLayout();
            this.groupBox3.ResumeLayout(false);
            this.groupBox3.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.nudSetTemp)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Panel PanelIndex;
        private System.Windows.Forms.Button BtnXYZTab;
        private System.Windows.Forms.Panel IndexPanel;
        private System.Windows.Forms.Panel PageDivider;
        private System.Windows.Forms.Timer timer1;
        private System.IO.Ports.SerialPort serialPort1;
        private System.Windows.Forms.Panel Panelxyz;
        private System.Windows.Forms.Button BtnHeaterTab;
        private System.Windows.Forms.Button BtnEnvironmentTab;
        private System.Windows.Forms.Panel PanelEnvoirement;
        private System.Windows.Forms.Label labelxyz;
        private System.Windows.Forms.Label Labelenvoirement;
        private System.Windows.Forms.Panel PanelHeater;
        private System.Windows.Forms.TrackBar TrackbarZ;
        private System.Windows.Forms.TrackBar TrackbarY;
        private System.Windows.Forms.TrackBar TrackbarX;
        private System.Windows.Forms.Label LabelZ;
        private System.Windows.Forms.Label labelY;
        private System.Windows.Forms.Label LabelXAxis;
        private System.Windows.Forms.Button BtnSetNuD;
        private System.Windows.Forms.NumericUpDown numericUpDownZ;
        private System.Windows.Forms.NumericUpDown numericUpDownY;
        private System.Windows.Forms.NumericUpDown numericUpDownX;
        private System.Windows.Forms.Button btnStartTesting;
        private System.Windows.Forms.ListBox log;
        private System.Windows.Forms.Label labelTemperature;
        private System.Windows.Forms.Label labelHumidity;
        private System.Windows.Forms.GroupBox gbCab;
        private System.Windows.Forms.Label lbBedLevelling;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.Panel pnLevelling;
        private System.Windows.Forms.PictureBox pictureBox1;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.Label labelHeater;
        private System.Windows.Forms.Button btnReset;
        private System.Windows.Forms.Label labelFilament;
        private System.Windows.Forms.Label labelDoor;
        private System.Windows.Forms.GroupBox groupBox4;
        private System.Windows.Forms.Label lbFanStat;
        private System.Windows.Forms.Label lbHeaterTemp;
        private System.Windows.Forms.Button button6;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.Button btnStopHeating;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Button btnSetTemp;
        private System.Windows.Forms.Button button3;
        private System.Windows.Forms.NumericUpDown nudSetTemp;
        private System.Windows.Forms.CheckBox cbxDisableZ;
        private System.Windows.Forms.CheckBox cbxDisableY;
        private System.Windows.Forms.CheckBox cbxDisableX;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.NumericUpDown nudMaxBorder;
        private System.Windows.Forms.Button btnStop;
    }
}

