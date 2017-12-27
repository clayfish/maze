using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace Maze52
{
    /// <summary>
    /// Displays the About Box form
    /// </summary>
    public partial class AboutBox : Form
    {
        /// <summary>
        /// Constructor
        /// </summary>
        public AboutBox()
        {
            InitializeComponent();
        }

        private void linkLabel1_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            System.Diagnostics.Process.Start("https://goo.gl/tRaLfe");
        }

        private void linkLabel2_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            System.Diagnostics.Process.Start("https://youtu.be/40iDvNZPulY");
        }
    }
}
