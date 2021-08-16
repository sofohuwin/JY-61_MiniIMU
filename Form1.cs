using System;
using System.Collections.Generic;
using System.Data;
using System.Diagnostics;
using System.Drawing;
using System.Windows.Forms;
using System.Threading;
using System.Threading.Tasks;
using System.Text;
using System.IO;
using System.IO.Ports;
using System.Globalization;
using System.Resources;
using System.Reflection;
using System.Management;
using System.Linq;

namespace MiniIMU
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }
        private void RefreshComPort(object sender, EventArgs e)
        {
            toolStripComSet.DropDownItems.Clear();
            foreach (string portName in System.IO.Ports.SerialPort.GetPortNames())
            {
                toolStripComSet.DropDownItems.Add(portName, null, PortSelect);

                if ((spSerialPort.IsOpen) & (spSerialPort.PortName == portName))
                {
                    ToolStripMenuItem menu = (ToolStripMenuItem)toolStripComSet.DropDownItems[toolStripComSet.DropDownItems.Count - 1];
                    menu.Checked = true;
                }
            }
            toolStripComSet.DropDownItems.Add(new ToolStripSeparator());
            toolStripComSet.DropDownItems.Add("Close", null, PortClose);
        }

        ResourceManager rm = new ResourceManager(typeof(Form1));
       
        private void Form1_Load(object sender, EventArgs e)
        {
            RefreshComPort(null, null);
            Baund = 9600;            
            SetBaudrate(Baund);
        }
        private void SetBaudrate(int iBaund)
        {
            toolStripMenuItem2.Checked = false;
            toolStripMenuItem3.Checked = false;
            toolStripMenuItem4.Checked = false;
            toolStripMenuItem5.Checked = false;
            toolStripMenuItem6.Checked = false;
            toolStripMenuItem7.Checked = false;
            toolStripMenuItem8.Checked = false;
            toolStripMenuItem9.Checked = false;
            toolStripMenuItem10.Checked = false;
            toolStripMenuItem11.Checked = false;
            switch (iBaund)
            {
                case 2400: toolStripMenuItem2.Checked = true; break;
                case 4800: toolStripMenuItem3.Checked = true; break;
                case 9600: toolStripMenuItem4.Checked = true; break;
                case 19200: toolStripMenuItem5.Checked = true; break;
                case 38400: toolStripMenuItem6.Checked = true; break;
                case 57600: toolStripMenuItem7.Checked = true; break;
                case 115200: toolStripMenuItem8.Checked = true; break;
                case 230400: toolStripMenuItem9.Checked = true; break;
                case 460800: toolStripMenuItem10.Checked = true; break;
                case 921600: toolStripMenuItem11.Checked = true; break;
            }
            spSerialPort.BaudRate = iBaund;
        }
        private bool bListening = false;
        private bool bClosing = false;
        private DateTime TimeStart = DateTime.Now;
        private Int32 Baund=115200;
        private void PortSelect(object sender, EventArgs e)
        {
            ToolStripMenuItem menu = (ToolStripMenuItem)sender;
            try
            {
                PortClose(null, null);
                spSerialPort.PortName = menu.Text;
                spSerialPort.BaudRate = Baund;                
                spSerialPort.Open();
                menu.Checked = true;
                bClosing = false;
                timer3.Start();
            }
            catch (Exception ex)
            {
                menu.Checked = false;
            }  

        }
        private void PortClose(object sender, EventArgs e)
        {
            for (int i = 0; i < toolStripComSet.DropDownItems.Count-2; i++)
            {
                ToolStripMenuItem tempMenu = (ToolStripMenuItem)toolStripComSet.DropDownItems[i];
                tempMenu.Checked = false;
            }
            if (spSerialPort.IsOpen)
            {
                bClosing = true;
                while (bListening) Application.DoEvents(); 
                spSerialPort.Dispose(); 
                spSerialPort.Close();
                timer3.Stop();
            }
        }
        double[] alpha = new double[4], 
                   omega = new double[4], 
                      theta = new double[4], h = new double[4], Port = new double[4];
        double Temperature, Pressure, Altitude,  GroundVelocity, GPSYaw, GPSHeight;
        long Longitude, Latitude;
        private void DisplayRefresh(object sender, EventArgs e)
        {
            double TimeElapse = (DateTime.Now - TimeStart).TotalMilliseconds / 1000;
            
            label34.Text = DateTime.Now.ToLongTimeString() + "\r\n"
                            + ChipTime[0].ToString() + "-" + ChipTime[1].ToString() + "-" + ChipTime[2].ToString() + "\r\n" + ChipTime[3].ToString() + ":" + ChipTime[4].ToString() + ":" + ChipTime[5].ToString() + "." + ChipTime[6].ToString() + "\r\n"
                            + TimeElapse.ToString("f3") + "\r\n\r\n"
                            + alpha[0].ToString("f2") + " g\r\n" 
                            + alpha[1].ToString("f2") + " g\r\n" 
                            + alpha[2].ToString("f2") + " g\r\n\r\n"
                            + omega[0].ToString("f2") + " °/s\r\n"
                            + omega[1].ToString("f2") + " °/s\r\n"
                            + omega[2].ToString("f2") + " °/s\r\n\r\n"
                            + theta[0].ToString("f2") + " °\r\n"
                            + theta[1].ToString("f2") + " °\r\n"
                            + theta[2].ToString("f2") + " °\r\n\r\n" 
                            + h[0].ToString("f0") + " mG\r\n" 
                            + h[1].ToString("f0") + " mG\r\n" 
                            + h[2].ToString("f0") + " mG\r\n\r\n" 
                            + Temperature.ToString("f2") + " ℃\r\n" 
                            + Pressure.ToString("f0") + " Pa\r\n" 
                            + Altitude.ToString("f2") + " m\r\n\r\n"
                            + (Longitude / 10000000).ToString("f0") + "°" + ((double)(Longitude % 10000000)/1e5).ToString("f5") + "'\r\n"
                            +(Latitude / 10000000).ToString("f0") + "°" + ((double)(Latitude % 10000000)/1e5).ToString("f5") + "'\r\n"
                            + GPSHeight.ToString("f1") + " m\r\n"
                            + GPSYaw.ToString("f1") + " °\r\n"
                            + GroundVelocity.ToString("f3") + " km/h";
        }
        private double[] LastTime = new double[10];
        short sRightPack = 0;
        short [] ChipTime = new short[7];

        public static double Relative_Variable_error()
        {
            double Relative_Variable_error = 2.367055111;
            double Relative_Variable_Maximum_error_in_z_axis = 2.567055111;
            double Relative_Variable_Minimum_error_in_z_axis = 2.367055111;
            double Relative_Variable_increase_error = 0.001;
            Random random = new Random();
            for (Relative_Variable_error = Relative_Variable_Minimum_error_in_z_axis;
                 Relative_Variable_error <= Relative_Variable_Maximum_error_in_z_axis;
                 Relative_Variable_error = Relative_Variable_error + Relative_Variable_increase_error)
            {
                Relative_Variable_error = Relative_Variable_error + random.NextDouble();
                Thread.Sleep(50);
                if (Relative_Variable_error <= Relative_Variable_Maximum_error_in_z_axis ||
                    Relative_Variable_error >= Relative_Variable_Minimum_error_in_z_axis)
                    return Relative_Variable_error;
            }
            return Relative_Variable_error;
        }

        private void DecodeData(byte[] byteTemp)
        {
            double[] Data = new double[4];
            double TimeElapse = (DateTime.Now - TimeStart).TotalMilliseconds / 1000;
            
            Data[0] = BitConverter.ToInt16(byteTemp, 2);
            Data[1] = BitConverter.ToInt16(byteTemp, 4);
            Data[2] = BitConverter.ToInt16(byteTemp, 6);
            Data[3] = BitConverter.ToInt16(byteTemp, 8);
            sRightPack++;
            switch (byteTemp[1])
            {
                case 0x50:
                    //Data[3] = Data[3] / 32768 * double.Parse(textBox9.Text) + double.Parse(textBox8.Text);
                    ChipTime[0] = (short)(2000 + byteTemp[2]);
                    ChipTime[1] = byteTemp[3];
                    ChipTime[2] = byteTemp[4];
                    ChipTime[3] = byteTemp[5];
                    ChipTime[4] = byteTemp[6];
                    ChipTime[5] = byteTemp[7];
                    ChipTime[6] = BitConverter.ToInt16(byteTemp, 8);


                    break;
                case 0x51:
                    //Data[3] = Data[3] / 32768 * double.Parse(textBox9.Text) + double.Parse(textBox8.Text);
                    Temperature = Data[3] / 100.0;
                    Data[0] = Data[0] / 32768.0 * 16;
                    Data[1] = Data[1] / 32768.0 * 16;
                    Data[2] = Data[2] / 32768.0 * 16;

                    alpha[0] = Data[0];
                    alpha[1] = Data[1];
                    alpha[2] = Data[2];
                    alpha[3] = Data[3];
                    if ((TimeElapse - LastTime[1]) < 0.1) return;
                    LastTime[1] = TimeElapse;

                    break;
                case 0x52:
                    //Data[3] = Data[3] / 32768 * double.Parse(textBox9.Text) + double.Parse(textBox8.Text);
                    Temperature = Data[3] / 100.0;
                    Data[0] = Data[0] / 32768.0 * 2000;
                    Data[1] = Data[1] / 32768.0 * 2000;
                    Data[2] = Data[2] / 32768.0 * 2000;

                    omega[0] = Data[0];
                    omega[1] = Data[1];
                    omega[2] = Data[2];
                    omega[3] = Data[3];
                    if ((TimeElapse-LastTime[2])<0.1)
                        return;
                    LastTime[2] = TimeElapse;
                    break;
                case 0x53:
                    //Data[3] = Data[3] / 32768 * double.Parse(textBox9.Text) + double.Parse(textBox8.Text);
                    Temperature = Data[3] / 100.0;
                    Data[0] = Data[0] / 32768.0 * 360;
                    Data[1] = Data[1] / 32768.0 * 360;
                    Data[2] = Data[2] / 32768.0 * 360;
                    if ((TimeElapse - LastTime[3]) < 0.1)
                        return;
                    LastTime[3] = TimeElapse;
                    theta[0] = Data[0];
                    theta[1] = Data[1];
                    theta[2] = Data[2];
                    theta[3] = Data[3];
                    break;
                case 0x54:
                    //Data[3] = Data[3] / 32768 * double.Parse(textBox9.Text) + double.Parse(textBox8.Text);
                    Temperature = Data[3] / 100.0;
                    h[0] = Data[0];
                    h[1] = Data[1];
                    h[2] = Data[2];
                    h[3] = Data[3];
                    if ((TimeElapse - LastTime[4]) < 0.1) return;
                    LastTime[4] = TimeElapse;
                    break;
                case 0x55:
                    Port[0] = Data[0];
                    Port[1] = Data[1];
                    Port[2] = Data[2];
                    Port[3] = Data[3];
            
                    break;

                case 0x56:
                    Pressure = BitConverter.ToInt32(byteTemp, 2);
                    Altitude = (double)BitConverter.ToInt32(byteTemp, 6) / 100.0;

                    break;

                case 0x57:
                    Longitude = BitConverter.ToInt32(byteTemp, 2);
                    Latitude  = BitConverter.ToInt32(byteTemp, 6);

                    break;

                case 0x58:
                    GPSHeight = (double)BitConverter.ToInt16(byteTemp, 2) / 10.0;
                    GPSYaw = (double)BitConverter.ToInt16(byteTemp, 4) / 10.0;
                    GroundVelocity = BitConverter.ToInt16(byteTemp, 6)/1e3;

                    break;
                default:
                    break;
            }   
        }
        byte byteLastNo = 0;
        
        delegate void UpdateData(byte[] byteData); // Explain a Commission
        byte[] RxBuffer = new byte[1000];
        UInt16 usRxLength = 0;
        private void SerialPort_DataReceived(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
        {
            byte[] byteTemp = new byte[1000];

            if (bClosing) return;
            try
            {
                bListening = true;
                UInt16 usLength=0;
                try
                {
                    usLength = (UInt16)spSerialPort.Read(RxBuffer, usRxLength, 700);
                }
                catch (Exception err)
                {
                    // MessageBox.Show(err.Message);
                    // return;
                }
                usRxLength += usLength;
                while (usRxLength >= 11)
                {
                    UpdateData Update = new UpdateData(DecodeData);
                    RxBuffer.CopyTo(byteTemp, 0);
                    if (!((byteTemp[0] == 0x55) & ((byteTemp[1] & 0x50)==0x50)))
                    {
                        for (int i = 1; i < usRxLength; i++) RxBuffer[i - 1] = RxBuffer[i];
                        usRxLength--;
                        continue;
                    }
                    if (((byteTemp[0]+byteTemp[1]+byteTemp[2]+byteTemp[3]+byteTemp[4]+byteTemp[5]+byteTemp[6]+byteTemp[7]+byteTemp[8]+byteTemp[9])&0xff)==byteTemp[10])
                        this.Invoke(Update, byteTemp);
                    for (int i = 11; i < usRxLength; i++) RxBuffer[i - 11] = RxBuffer[i];
                    usRxLength -= 11;
                }

                Thread.Sleep(10);
            }
            finally
            {
                bListening = false; // I Finish Listening, Be Able to Close the User Windows InterFace 
            }   
        }
        private sbyte sbSumCheck(byte[] byteData,byte byteLength)
        {
            byte byteSum=0;
            for (byte i = 0;i<byteLength-2;i++)
                byteSum += byteData[i];
            if (byteData[byteLength - 1] == byteSum) return 0;
            else return -1;
        }
        public sbyte SendMessage(Byte[] byteSend)
        {
            if (spSerialPort.IsOpen == false)
            {
              //  MessageBox.Show(rm.GetString("PortNotOpen"), "Error!");
                Status.Text="Port Not Open!";
                return -1;
            }
            try
            {
                spSerialPort.Write(byteSend, 0, byteSend.Length);
                return 0;
            }
            catch (Exception err)
            {
                MessageBox.Show(err.Message);
                return -1;
            }
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            try
            {
                PortClose(null, null);
            }
            catch { }
           
        }
 

        private void toolStripButton2_Click_1(object sender, EventArgs e)
        {
            Byte[] byteSend = new Byte[9];
            byteSend[0] = (byte)'A';
            byteSend[1] = (byte)'T';
            byteSend[2] = (byte)'+';
            byteSend[3] = (byte)'R';
            byteSend[4] = (byte)'O';
            byteSend[5] = (byte)'L';
            byteSend[6] = (byte)'E';
            byteSend[7] = (byte)'=';
            byteSend[8] = (byte)'M';
            if (SendMessage(byteSend) != 0) return;
            Thread.Sleep(1500);
            byteSend[8] = (byte)'S';
            if (SendMessage(byteSend) != 0) return;
            Status.Text = rm.GetString("Setting OK！");
        }


        private void toolStripMenuItem2_Click(object sender, EventArgs e)
        {
            Baund = 2400;
            SetBaudrate(Baund);
        }

        private void toolStripMenuItem3_Click(object sender, EventArgs e)
        {
            Baund = 4800;
            SetBaudrate(Baund);
        }
        private void toolStripMenuItem4_Click(object sender, EventArgs e)
        {
            Baund = 9600;
            SetBaudrate(Baund);
        }

        private void toolStripMenuItem5_Click(object sender, EventArgs e)
        {
            Baund = 19200;
            SetBaudrate(Baund);
        }

        private void toolStripMenuItem6_Click(object sender, EventArgs e)
        {
            Baund = 38400;
            SetBaudrate(Baund);
        }

        private void toolStripMenuItem7_Click(object sender, EventArgs e)
        {
            Baund = 57600;
            SetBaudrate(Baund);
        }

        private void toolStripMenuItem8_Click(object sender, EventArgs e)
        {
            Baund = 115200;
            SetBaudrate(Baund);
        }

        private void toolStripMenuItem9_Click(object sender, EventArgs e)
        {
            Baund = 230400;
            SetBaudrate(Baund);
        }

        private void toolStripMenuItem10_Click(object sender, EventArgs e)
        {
            Baund = 460800;
            SetBaudrate(Baund);
        }

        private void toolStripMenuItem11_Click(object sender, EventArgs e)
        {
            Baund = 921600;
            SetBaudrate(Baund);
        }

        int iBaudNo = 0;
        private void AutoInspectionToolStripMenuItem_Click(object sender, EventArgs e)
        {
            iBaudNo = 0;
            timer2.Start();
        }

        private void timer2_Tick(object sender, EventArgs e)
        {
            switch (iBaudNo)            
            {
                case 0:   
                    Baund = 2400;
                    SetBaudrate(Baund);
                    sRightPack = 0;
                    Status.Text = "Try 2400 bits/s ......";
                    break;
                case 1:
                    if (sRightPack > 2) {timer2.Stop();Status.Text = "Find the Sensor Baud Rate";}
                    else
                    {
                        Baund = 4800;
                        SetBaudrate(Baund);
                        sRightPack = 0;
                        Status.Text = "Try 4800 bits/s ......";
                    }
                    break;
                case 2:
                    if (sRightPack > 2) {timer2.Stop();Status.Text = "Find the Sensor Baud Rate"; }
                    else
                    {
                        Baund = 9600;
                        SetBaudrate(Baund);
                        sRightPack = 0;
                        Status.Text = "Try 9600 bits/s ......";
                    }
                    break;
                case 3:
                    if (sRightPack > 2) {timer2.Stop();Status.Text = "Find the Sensor Baud Rate"; }
                    else
                    {
                        Baund = 19200;
                        SetBaudrate(Baund);
                        sRightPack = 0;
                        Status.Text = "Try 19200 bits/s ......";
                    }
                    break;
                case 4:
                    if (sRightPack > 2) {timer2.Stop();Status.Text = "Find the Sensor Baud Rate"; }
                    else
                    {
                        Baund = 38400;
                        SetBaudrate(Baund);
                        sRightPack = 0;
                        Status.Text = "Try 38400 bits/s ......";
                    }
                    break;
                case 5:
                    if (sRightPack > 2) {timer2.Stop();Status.Text = "Find the Sensor Baud Rate"; }
                    else
                    {
                        Baund = 57600;
                        SetBaudrate(Baund);
                        sRightPack = 0;
                        Status.Text = "Try 57600 bits/s ......";
                    }
                    break;
                case 6:
                    if (sRightPack > 2) {timer2.Stop();Status.Text = "Find the Sensor Baud Rate"; }
                    else
                    {
                        Baund = 115200;
                        SetBaudrate(Baund);
                        sRightPack = 0;
                        Status.Text = "Try 115200 bits/s ......";
                    }
                    break;
                case 7:
                    if (sRightPack > 2) {timer2.Stop();Status.Text = "Find the Sensor Baud Rate"; }
                    else
                    {
                        Baund = 230400;
                        SetBaudrate(Baund);
                        sRightPack = 0;
                        Status.Text = "Try 230400 bits/s ......";
                    }
                    break;
                case 8:
                    if (sRightPack > 2) {timer2.Stop();Status.Text = "Find the Sensor Baud Rate"; }
                    else
                    {
                        Baund = 460800;
                        SetBaudrate(Baund);
                        sRightPack = 0;
                        Status.Text = "Try 460800 bits/s ......";
                    }
                    break;
                case 9:
                    if (sRightPack > 2) {timer2.Stop();Status.Text = "Find the Sensor Baud Rate"; }
                    else
                    {
                        Baund = 921600;
                        SetBaudrate(Baund);
                        sRightPack = 0;
                        Status.Text = "Try 921600 bits/s ......";
                    }
                    break;
                case 10:
                    if (sRightPack > 2) { timer2.Stop(); Status.Text = "Find the Sensor Baud Rate"; }
                    else
                    {
                        timer2.Stop();
                        Status.Text = "Don't Find the Sensor Baud Rate！";
                    }
                    break;
            }
            iBaudNo = iBaudNo + 1;            
        }
    }
}