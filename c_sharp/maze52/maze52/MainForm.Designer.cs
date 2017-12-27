﻿namespace Maze52
{
    partial class MainForm
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
            this.rowsLbl = new System.Windows.Forms.Label();
            this.rowsSpinner = new System.Windows.Forms.NumericUpDown();
            this.columnsLbl = new System.Windows.Forms.Label();
            this.columnsSpinner = new System.Windows.Forms.NumericUpDown();
            this.ResetButton = new System.Windows.Forms.Button();
            this.MazeButton = new System.Windows.Forms.Button();
            this.ToolTip = new System.Windows.Forms.ToolTip(this.components);
            this.ClearButton = new System.Windows.Forms.Button();
            this.RealTimeButton = new System.Windows.Forms.Button();
            this.StepButton = new System.Windows.Forms.Button();
            this.AnimationButton = new System.Windows.Forms.Button();
            this.slider = new System.Windows.Forms.TrackBar();
            this.dfs = new System.Windows.Forms.RadioButton();
            this.bfs = new System.Windows.Forms.RadioButton();
            this.aStar = new System.Windows.Forms.RadioButton();
            this.greedy = new System.Windows.Forms.RadioButton();
            this.dijkstra = new System.Windows.Forms.RadioButton();
            this.diagonal = new System.Windows.Forms.CheckBox();
            this.drawArrows = new System.Windows.Forms.CheckBox();
            this.delayLbl = new System.Windows.Forms.Label();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.robotLbl = new System.Windows.Forms.Label();
            this.targetLbl = new System.Windows.Forms.Label();
            this.frontierLbl = new System.Windows.Forms.Label();
            this.closedLbl = new System.Windows.Forms.Label();
            this.AboutButton = new System.Windows.Forms.Button();
            this.message = new System.Windows.Forms.Label();
            this.timer = new System.Windows.Forms.Timer(this.components);
            ((System.ComponentModel.ISupportInitialize)(this.rowsSpinner)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.columnsSpinner)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.slider)).BeginInit();
            this.groupBox1.SuspendLayout();
            this.SuspendLayout();
            // 
            // rowsLbl
            // 
            this.rowsLbl.AutoSize = true;
            this.rowsLbl.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(161)));
            this.rowsLbl.Location = new System.Drawing.Point(528, 8);
            this.rowsLbl.MaximumSize = new System.Drawing.Size(116, 15);
            this.rowsLbl.MinimumSize = new System.Drawing.Size(116, 15);
            this.rowsLbl.Name = "rowsLbl";
            this.rowsLbl.Size = new System.Drawing.Size(116, 15);
            this.rowsLbl.TabIndex = 0;
            this.rowsLbl.Text = "# of rows (5-83):";
            this.rowsLbl.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // rowsSpinner
            // 
            this.rowsSpinner.Location = new System.Drawing.Point(648, 8);
            this.rowsSpinner.Maximum = new decimal(new int[] {
            83,
            0,
            0,
            0});
            this.rowsSpinner.Minimum = new decimal(new int[] {
            5,
            0,
            0,
            0});
            this.rowsSpinner.Name = "rowsSpinner";
            this.rowsSpinner.Size = new System.Drawing.Size(32, 20);
            this.rowsSpinner.TabIndex = 1;
            this.rowsSpinner.Value = new decimal(new int[] {
            41,
            0,
            0,
            0});
            // 
            // columnsLbl
            // 
            this.columnsLbl.AutoSize = true;
            this.columnsLbl.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(161)));
            this.columnsLbl.Location = new System.Drawing.Point(528, 32);
            this.columnsLbl.Name = "columnsLbl";
            this.columnsLbl.Size = new System.Drawing.Size(116, 15);
            this.columnsLbl.TabIndex = 2;
            this.columnsLbl.Text = "# of columns (5-83):";
            this.columnsLbl.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // columnsSpinner
            // 
            this.columnsSpinner.Location = new System.Drawing.Point(648, 32);
            this.columnsSpinner.Maximum = new decimal(new int[] {
            83,
            0,
            0,
            0});
            this.columnsSpinner.Minimum = new decimal(new int[] {
            5,
            0,
            0,
            0});
            this.columnsSpinner.Name = "columnsSpinner";
            this.columnsSpinner.Size = new System.Drawing.Size(32, 20);
            this.columnsSpinner.TabIndex = 3;
            this.columnsSpinner.Value = new decimal(new int[] {
            41,
            0,
            0,
            0});
            // 
            // ResetButton
            // 
            this.ResetButton.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(161)));
            this.ResetButton.Location = new System.Drawing.Point(520, 64);
            this.ResetButton.Name = "ResetButton";
            this.ResetButton.Size = new System.Drawing.Size(168, 28);
            this.ResetButton.TabIndex = 4;
            this.ResetButton.Text = "New grid";
            this.ToolTip.SetToolTip(this.ResetButton, "Clears and redraws the grid according to the given rows and columns");
            this.ResetButton.UseVisualStyleBackColor = true;
            this.ResetButton.Click += new System.EventHandler(this.ResetButton_Click);
            // 
            // MazeButton
            // 
            this.MazeButton.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(161)));
            this.MazeButton.Location = new System.Drawing.Point(520, 96);
            this.MazeButton.Name = "MazeButton";
            this.MazeButton.Size = new System.Drawing.Size(168, 28);
            this.MazeButton.TabIndex = 5;
            this.MazeButton.Text = "Maze";
            this.ToolTip.SetToolTip(this.MazeButton, "Creates a random maze");
            this.MazeButton.UseVisualStyleBackColor = true;
            this.MazeButton.Click += new System.EventHandler(this.MazeButton_Click);
            // 
            // ClearButton
            // 
            this.ClearButton.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(161)));
            this.ClearButton.Location = new System.Drawing.Point(520, 128);
            this.ClearButton.Name = "ClearButton";
            this.ClearButton.Size = new System.Drawing.Size(168, 28);
            this.ClearButton.TabIndex = 6;
            this.ClearButton.Text = "Clear";
            this.ToolTip.SetToolTip(this.ClearButton, "First click: clears search, Second click: clears obstacles");
            this.ClearButton.UseVisualStyleBackColor = true;
            this.ClearButton.Click += new System.EventHandler(this.ClearButton_Click);
            // 
            // RealTimeButton
            // 
            this.RealTimeButton.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(161)));
            this.RealTimeButton.Location = new System.Drawing.Point(520, 160);
            this.RealTimeButton.Name = "RealTimeButton";
            this.RealTimeButton.Size = new System.Drawing.Size(168, 28);
            this.RealTimeButton.TabIndex = 7;
            this.RealTimeButton.Text = "Real-Time";
            this.ToolTip.SetToolTip(this.RealTimeButton, "Position of obstacles, robot and target can be changed when search is underway");
            this.RealTimeButton.UseVisualStyleBackColor = true;
            this.RealTimeButton.Click += new System.EventHandler(this.RealTimeButton_Click);
            // 
            // StepButton
            // 
            this.StepButton.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(161)));
            this.StepButton.Location = new System.Drawing.Point(520, 192);
            this.StepButton.Name = "StepButton";
            this.StepButton.Size = new System.Drawing.Size(168, 28);
            this.StepButton.TabIndex = 8;
            this.StepButton.Text = "Step-by-Step";
            this.ToolTip.SetToolTip(this.StepButton, "The search is performed step-by-step for every click");
            this.StepButton.UseVisualStyleBackColor = true;
            this.StepButton.Click += new System.EventHandler(this.StepButton_Click);
            // 
            // AnimationButton
            // 
            this.AnimationButton.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(161)));
            this.AnimationButton.Location = new System.Drawing.Point(520, 224);
            this.AnimationButton.Name = "AnimationButton";
            this.AnimationButton.Size = new System.Drawing.Size(168, 28);
            this.AnimationButton.TabIndex = 9;
            this.AnimationButton.Text = "Animation";
            this.ToolTip.SetToolTip(this.AnimationButton, "The search is performed automatically");
            this.AnimationButton.UseVisualStyleBackColor = true;
            this.AnimationButton.Click += new System.EventHandler(this.AnimationButton_Click);
            // 
            // slider
            // 
            this.slider.Location = new System.Drawing.Point(520, 272);
            this.slider.Maximum = 1000;
            this.slider.MaximumSize = new System.Drawing.Size(168, 30);
            this.slider.Minimum = 1;
            this.slider.Name = "slider";
            this.slider.Size = new System.Drawing.Size(168, 45);
            this.slider.SmallChange = 10;
            this.slider.TabIndex = 11;
            this.slider.TickFrequency = 50;
            this.slider.TickStyle = System.Windows.Forms.TickStyle.TopLeft;
            this.ToolTip.SetToolTip(this.slider, "Regulates the delay for each step (0 to 1000 msec)");
            this.slider.Value = 500;
            // 
            // dfs
            // 
            this.dfs.AutoSize = true;
            this.dfs.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(161)));
            this.dfs.Location = new System.Drawing.Point(16, 24);
            this.dfs.Name = "dfs";
            this.dfs.Size = new System.Drawing.Size(52, 20);
            this.dfs.TabIndex = 0;
            this.dfs.TabStop = true;
            this.dfs.Text = "DFS";
            this.ToolTip.SetToolTip(this.dfs, "Depth First Search algorithm");
            this.dfs.UseVisualStyleBackColor = true;
            // 
            // bfs
            // 
            this.bfs.AutoSize = true;
            this.bfs.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(161)));
            this.bfs.Location = new System.Drawing.Point(88, 24);
            this.bfs.Name = "bfs";
            this.bfs.Size = new System.Drawing.Size(52, 20);
            this.bfs.TabIndex = 1;
            this.bfs.TabStop = true;
            this.bfs.Text = "BFS";
            this.ToolTip.SetToolTip(this.bfs, "Breadth First Search algorithm");
            this.bfs.UseVisualStyleBackColor = true;
            // 
            // aStar
            // 
            this.aStar.AutoSize = true;
            this.aStar.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(161)));
            this.aStar.Location = new System.Drawing.Point(16, 48);
            this.aStar.Name = "aStar";
            this.aStar.Size = new System.Drawing.Size(40, 20);
            this.aStar.TabIndex = 2;
            this.aStar.TabStop = true;
            this.aStar.Text = "A*";
            this.ToolTip.SetToolTip(this.aStar, "A* algorithm");
            this.aStar.UseVisualStyleBackColor = true;
            // 
            // greedy
            // 
            this.greedy.AutoSize = true;
            this.greedy.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(161)));
            this.greedy.Location = new System.Drawing.Point(88, 48);
            this.greedy.Name = "greedy";
            this.greedy.Size = new System.Drawing.Size(72, 20);
            this.greedy.TabIndex = 3;
            this.greedy.TabStop = true;
            this.greedy.Text = "Greedy";
            this.ToolTip.SetToolTip(this.greedy, "Greedy search algorithm");
            this.greedy.UseVisualStyleBackColor = true;
            // 
            // dijkstra
            // 
            this.dijkstra.AutoSize = true;
            this.dijkstra.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(161)));
            this.dijkstra.Location = new System.Drawing.Point(16, 72);
            this.dijkstra.Name = "dijkstra";
            this.dijkstra.Size = new System.Drawing.Size(73, 20);
            this.dijkstra.TabIndex = 4;
            this.dijkstra.TabStop = true;
            this.dijkstra.Text = "Dijkstra";
            this.ToolTip.SetToolTip(this.dijkstra, "Dijkstra\'s algorithm");
            this.dijkstra.UseVisualStyleBackColor = true;
            // 
            // diagonal
            // 
            this.diagonal.AutoSize = true;
            this.diagonal.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(161)));
            this.diagonal.Location = new System.Drawing.Point(520, 408);
            this.diagonal.Name = "diagonal";
            this.diagonal.Size = new System.Drawing.Size(161, 20);
            this.diagonal.TabIndex = 13;
            this.diagonal.Text = "Diagonal movements";
            this.ToolTip.SetToolTip(this.diagonal, "Diagonal movements are also allowed");
            this.diagonal.UseVisualStyleBackColor = true;
            // 
            // drawArrows
            // 
            this.drawArrows.AutoSize = true;
            this.drawArrows.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(161)));
            this.drawArrows.Location = new System.Drawing.Point(520, 432);
            this.drawArrows.Name = "drawArrows";
            this.drawArrows.Size = new System.Drawing.Size(174, 20);
            this.drawArrows.TabIndex = 14;
            this.drawArrows.Text = "Arrows to predecessors";
            this.ToolTip.SetToolTip(this.drawArrows, "Draw arrows to predecessors");
            this.drawArrows.UseVisualStyleBackColor = true;
            // 
            // delayLbl
            // 
            this.delayLbl.AutoSize = true;
            this.delayLbl.Location = new System.Drawing.Point(520, 256);
            this.delayLbl.MaximumSize = new System.Drawing.Size(168, 13);
            this.delayLbl.MinimumSize = new System.Drawing.Size(168, 13);
            this.delayLbl.Name = "delayLbl";
            this.delayLbl.Size = new System.Drawing.Size(168, 13);
            this.delayLbl.TabIndex = 10;
            this.delayLbl.Text = "Delay (1-1000 msec)";
            this.delayLbl.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.dijkstra);
            this.groupBox1.Controls.Add(this.greedy);
            this.groupBox1.Controls.Add(this.aStar);
            this.groupBox1.Controls.Add(this.bfs);
            this.groupBox1.Controls.Add(this.dfs);
            this.groupBox1.FlatStyle = System.Windows.Forms.FlatStyle.System;
            this.groupBox1.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(161)));
            this.groupBox1.Location = new System.Drawing.Point(520, 296);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(168, 100);
            this.groupBox1.TabIndex = 12;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Algorithms";
            // 
            // robotLbl
            // 
            this.robotLbl.AutoSize = true;
            this.robotLbl.Font = new System.Drawing.Font("Microsoft Sans Serif", 11.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(161)));
            this.robotLbl.ForeColor = System.Drawing.Color.Red;
            this.robotLbl.Location = new System.Drawing.Point(528, 464);
            this.robotLbl.MinimumSize = new System.Drawing.Size(80, 16);
            this.robotLbl.Name = "robotLbl";
            this.robotLbl.Size = new System.Drawing.Size(80, 18);
            this.robotLbl.TabIndex = 15;
            this.robotLbl.Text = "Robot";
            this.robotLbl.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // targetLbl
            // 
            this.targetLbl.AutoSize = true;
            this.targetLbl.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(161)));
            this.targetLbl.ForeColor = System.Drawing.Color.Green;
            this.targetLbl.Location = new System.Drawing.Point(608, 464);
            this.targetLbl.MinimumSize = new System.Drawing.Size(80, 16);
            this.targetLbl.Name = "targetLbl";
            this.targetLbl.Size = new System.Drawing.Size(80, 20);
            this.targetLbl.TabIndex = 16;
            this.targetLbl.Text = "Target";
            this.targetLbl.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // frontierLbl
            // 
            this.frontierLbl.AutoSize = true;
            this.frontierLbl.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(161)));
            this.frontierLbl.ForeColor = System.Drawing.Color.Blue;
            this.frontierLbl.Location = new System.Drawing.Point(528, 488);
            this.frontierLbl.MinimumSize = new System.Drawing.Size(80, 16);
            this.frontierLbl.Name = "frontierLbl";
            this.frontierLbl.Size = new System.Drawing.Size(80, 20);
            this.frontierLbl.TabIndex = 17;
            this.frontierLbl.Text = "Frontier";
            this.frontierLbl.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // closedLbl
            // 
            this.closedLbl.AutoSize = true;
            this.closedLbl.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(161)));
            this.closedLbl.ForeColor = System.Drawing.Color.Cyan;
            this.closedLbl.Location = new System.Drawing.Point(608, 488);
            this.closedLbl.MinimumSize = new System.Drawing.Size(80, 16);
            this.closedLbl.Name = "closedLbl";
            this.closedLbl.Size = new System.Drawing.Size(84, 20);
            this.closedLbl.TabIndex = 18;
            this.closedLbl.Text = "Closed set";
            this.closedLbl.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // AboutButton
            // 
            this.AboutButton.Font = new System.Drawing.Font("Arial", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(161)));
            this.AboutButton.Location = new System.Drawing.Point(520, 512);
            this.AboutButton.Name = "AboutButton";
            this.AboutButton.Size = new System.Drawing.Size(168, 28);
            this.AboutButton.TabIndex = 19;
            this.AboutButton.Text = "About Maze";
            this.AboutButton.UseVisualStyleBackColor = true;
            this.AboutButton.Click += new System.EventHandler(this.AboutButton_Click);
            // 
            // message
            // 
            this.message.AutoSize = true;
            this.message.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(161)));
            this.message.ForeColor = System.Drawing.Color.Blue;
            this.message.Location = new System.Drawing.Point(8, 520);
            this.message.MinimumSize = new System.Drawing.Size(500, 16);
            this.message.Name = "message";
            this.message.Size = new System.Drawing.Size(500, 20);
            this.message.TabIndex = 20;
            this.message.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // timer
            // 
            this.timer.Interval = 500;
            this.timer.Tick += new System.EventHandler(this.Timer_Tick);
            // 
            // MainForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(695, 547);
            this.Controls.Add(this.message);
            this.Controls.Add(this.AboutButton);
            this.Controls.Add(this.closedLbl);
            this.Controls.Add(this.frontierLbl);
            this.Controls.Add(this.targetLbl);
            this.Controls.Add(this.robotLbl);
            this.Controls.Add(this.drawArrows);
            this.Controls.Add(this.diagonal);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.slider);
            this.Controls.Add(this.delayLbl);
            this.Controls.Add(this.AnimationButton);
            this.Controls.Add(this.StepButton);
            this.Controls.Add(this.RealTimeButton);
            this.Controls.Add(this.ClearButton);
            this.Controls.Add(this.MazeButton);
            this.Controls.Add(this.ResetButton);
            this.Controls.Add(this.columnsSpinner);
            this.Controls.Add(this.columnsLbl);
            this.Controls.Add(this.rowsSpinner);
            this.Controls.Add(this.rowsLbl);
            this.DoubleBuffered = true;
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.Fixed3D;
            this.MaximizeBox = false;
            this.Name = "MainForm";
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
            this.Text = "Maze 5.2";
            this.Paint += new System.Windows.Forms.PaintEventHandler(this.MainForm_Paint);
            this.MouseDown += new System.Windows.Forms.MouseEventHandler(this.MainForm_MouseDown);
            this.MouseMove += new System.Windows.Forms.MouseEventHandler(this.MainForm_MouseMove);
            this.MouseUp += new System.Windows.Forms.MouseEventHandler(this.MainForm_MouseUp);
            ((System.ComponentModel.ISupportInitialize)(this.rowsSpinner)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.columnsSpinner)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.slider)).EndInit();
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Label rowsLbl;
        private System.Windows.Forms.NumericUpDown rowsSpinner;
        private System.Windows.Forms.Label columnsLbl;
        private System.Windows.Forms.NumericUpDown columnsSpinner;
        private System.Windows.Forms.Button ResetButton;
        private System.Windows.Forms.Button MazeButton;
        private System.Windows.Forms.ToolTip ToolTip;
        private System.Windows.Forms.Button ClearButton;
        private System.Windows.Forms.Button RealTimeButton;
        private System.Windows.Forms.Button StepButton;
        private System.Windows.Forms.Button AnimationButton;
        private System.Windows.Forms.Label delayLbl;
        private System.Windows.Forms.TrackBar slider;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.RadioButton dfs;
        private System.Windows.Forms.RadioButton dijkstra;
        private System.Windows.Forms.RadioButton greedy;
        private System.Windows.Forms.RadioButton aStar;
        private System.Windows.Forms.RadioButton bfs;
        private System.Windows.Forms.CheckBox diagonal;
        private System.Windows.Forms.CheckBox drawArrows;
        private System.Windows.Forms.Label robotLbl;
        private System.Windows.Forms.Label targetLbl;
        private System.Windows.Forms.Label frontierLbl;
        private System.Windows.Forms.Label closedLbl;
        private System.Windows.Forms.Button AboutButton;
        private System.Windows.Forms.Label message;
        private System.Windows.Forms.Timer timer;
    }
}

