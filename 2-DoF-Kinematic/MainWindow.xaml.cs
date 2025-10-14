using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.IO.Ports;
using System.Windows.Threading;

namespace P1_PRNO
{
    public partial class MainWindow : Window
    {
        private SerialPort serialPort;
        private DispatcherTimer refreshTimer;

        public MainWindow()
        {
            InitializeComponent();
            InitializeSerialPort();
            RefreshPortList();

            // Timer untuk refresh daftar COM port tiap 2 detik
            refreshTimer = new DispatcherTimer();
            refreshTimer.Interval = TimeSpan.FromSeconds(2);
            refreshTimer.Tick += (s, e) => RefreshPortList();
            refreshTimer.Start();
        }

        private void InitializeSerialPort()
        {
            serialPort = new SerialPort();
        }

        private void RefreshPortList()
        {
            string currentSelection = cmbPorts.SelectedItem?.ToString();
            cmbPorts.Items.Clear();
            string[] ports = SerialPort.GetPortNames();

            foreach (string port in ports)
            {
                cmbPorts.Items.Add(port);
            }
            if (!string.IsNullOrEmpty(currentSelection) && cmbPorts.Items.Contains(currentSelection))
            {
                cmbPorts.SelectedItem = currentSelection;
            }
            else if (cmbPorts.Items.Count > 0)
            {
                cmbPorts.SelectedIndex = 0;
            }
        }

        private void BtnConnect_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (serialPort.IsOpen)
                {
                    serialPort.Close();
                    btnConnect.Content = "Connect";
                }
                else
                {
                    if (cmbPorts.SelectedItem == null)
                    {
                        MessageBox.Show("Please select a COM port");
                        return;
                    }

                    serialPort.PortName = cmbPorts.SelectedItem.ToString();
                    serialPort.BaudRate = 115200;
                    serialPort.Parity = Parity.None;
                    serialPort.DataBits = 8;
                    serialPort.StopBits = StopBits.One;
                    serialPort.Handshake = Handshake.None;
                    serialPort.Open();
                    btnConnect.Content = "Disconnect";

                    // --- Kirim posisi awal servo #1 ---
                    string initMessage = "#1 P2220 S100\r\n";  //Inisialisasi Servo 1 dalam posisi Riak Air
                    string initMessage2 = "#0 P1525 S100\r\n"; //Inisialisasi Servo 0
                    string initMessage3 = "#2 P1502 S100\r\n"; //Inisialisasi Servo 2
                    serialPort.Write(initMessage);
                    serialPort.Write(initMessage2);
                    serialPort.Write(initMessage3);
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error: {ex.Message}");
            }
        }

        private void BtnClose_Click(object sender, RoutedEventArgs e)
        {
            if (serialPort.IsOpen)
            {
                serialPort.Close();
                btnConnect.Content = "Connect";
            }
        }

        private void BtnOrigin_Click(object sender, RoutedEventArgs e)
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                // --- Kirim posisi default (sesuaikan dengan posisi origin robotmu) ---
                string originServo0 = "#0 P1525 S500\r\n";  // Servo 0 ke tengah
                string originServo1 = "#1 P2220 S500\r\n";  // Servo 1 origin
                string originServo2 = "#2 P1502 S500\r\n";  // Servo 2 origin
                string originServo3 = "#3 P1550 S500\r\n";  // Servo 3 origin
                string originClaw = "#4 P1500 S500\r\n";   // Claw origin (opsional)

                serialPort.Write(originServo0);
                serialPort.Write(originServo1);
                serialPort.Write(originServo2);
                serialPort.Write(originServo3);
                serialPort.Write(originClaw);

                // --- Update UI slider supaya sinkron ---
                sliderServo0.Value = 90;   // otomatis update txtSudut0 & interp0 lewat event ValueChanged
                sliderServo2.Value = 0;    // otomatis update txtSudut2 & interp2
                sliderServo3.Value = 0;    // otomatis update txtSudut3 & interp3
            }
            else
            {
                MessageBox.Show("Serial port belum terbuka. Silakan Connect dulu.");
            }
        }


        private void SliderControl_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (sender is Slider slider && slider.Tag != null && int.TryParse(slider.Tag.ToString(), out int servoId))
            {
                double angle = e.NewValue;
                double pulse = 0;

                if (servoId == 0)
                {
                    // Servo 0: 0–180 derajat → 2430–620 us
                    pulse = Interpolate(angle, 0, 2430, 180, 620);
                    txtSudut0.Text = angle.ToString("F0");
                    txtInterp0.Text = pulse.ToString("F0");
                }
                else if (servoId == 2)
                {
                    // Servo 2: -90–+90 derajat → 2450–660 us
                    pulse = Interpolate(angle, -90, 2450, 90, 660);
                    txtSudut2.Text = angle.ToString("F0");
                    txtInterp2.Text = pulse.ToString("F0");
                }
                else if (servoId == 3)
                {
                    // Servo 3: -90–+90 derajat → 2430–650 us
                    pulse = Interpolate(angle, -90, 2430, 90, 650);
                    txtSudut3.Text = angle.ToString("F0");
                    txtInterp3.Text = pulse.ToString("F0");
                }

                // kirim ke servo kalau port terbuka
                if (serialPort != null && serialPort.IsOpen)
                {
                    string message = $"#{servoId} P{(int)pulse} S500\r\n";
                    serialPort.Write(message);
                }
                CalculateForwardKinematics();
            }
        }




        // fungsi linear interpolation
        private double Interpolate(double x, double x1, double y1, double x2, double y2)
        {
            return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
        }

        private void CalculateForwardKinematics()
        {
            try
            {
                double a1 = double.Parse(m_a1.Text);
                double a2 = double.Parse(m_a2.Text);

                // Ambil sudut servo dari slider (dalam radian)
                double theta1 = sliderServo0.Value * Math.PI / 180.0;
                double theta2 = sliderServo2.Value * Math.PI / 180.0;

                // Rumus FK
                double Px = a1 * Math.Cos(theta1) + a2 * Math.Cos(theta1 + theta2);
                double Py = a1 * Math.Sin(theta1) + a2 * Math.Sin(theta1 + theta2);

                m_px.Text = Px.ToString("F3");
                m_py.Text = Py.ToString("F3");
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error FK: " + ex.Message);
            }
        }

        // === Tombol Calculate Solusi 1 ===
        private void cal_inv1_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                double a1 = double.Parse(m_a1.Text);
                double a2 = double.Parse(m_a2.Text);

                // Ambil nilai Px dan Py dari input user
                double Px = double.Parse(m_px.Text);
                double Py = double.Parse(m_py.Text);

                // Pastikan target masih dalam jangkauan lengan
                double distance = Math.Sqrt(Px * Px + Py * Py);
                if (distance > a1 + a2 || distance < Math.Abs(a1 - a2))
                {
                    MessageBox.Show("Target di luar jangkauan robot.");
                    return;
                }

                // Rumus inverse kinematics (solusi elbow-up)
                double cos_teta2 = (Px * Px + Py * Py - a1 * a1 - a2 * a2) / (2 * a1 * a2);
                cos_teta2 = Math.Max(-1.0, Math.Min(1.0, cos_teta2)); // clamp agar tidak NaN
                double teta2 = Math.Acos(cos_teta2);
                double teta1 = Math.Atan2(Py, Px) - Math.Atan2(a2 * Math.Sin(teta2), a1 + a2 * Math.Cos(teta2));

                // Konversi ke derajat
                double deg_teta1 = teta1 * 180.0 / Math.PI;
                double deg_teta2 = teta2 * 180.0 / Math.PI;

                // Tampilkan ke textbox
                m_satu.Text = deg_teta1.ToString("F2");
                m_dua.Text = deg_teta2.ToString("F2");
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error IK Solusi 1: " + ex.Message);
            }
        }

        // === Tombol Run Solusi 1 ===
        private void run_inv1_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (serialPort == null || !serialPort.IsOpen)
                {
                    MessageBox.Show("Serial port belum terbuka.");
                    return;
                }

                // ambil nilai dari textbox (hasil calculate sebelumnya)
                double deg_teta1 = double.Parse(m_satu.Text);
                double deg_teta2 = double.Parse(m_dua.Text);

                // konversi ke pulse servo sesuai mapping
                double pulse0 = Interpolate(deg_teta1, 0, 2430, 180, 620);   // servo 0
                double pulse2 = Interpolate(deg_teta2, -90, 2450, 90, 660); // servo 2

                string cmd0 = $"#0 P{(int)pulse0} S500\r\n"; 
                string cmd2 = $"#2 P{(int)pulse2} S500\r\n"; 

                serialPort.Write(cmd0); 
                serialPort.Write(cmd2);

                // === sinkronkan slider & textbox dengan hasil IK ===
                //sliderServo0.Value = deg_teta1;
                //sliderServo2.Value = deg_teta2;

                txtSudut0.Text = deg_teta1.ToString("F0");
                txtSudut2.Text = deg_teta2.ToString("F0");

                
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error Run IK: " + ex.Message);
            }
        }


        // === Tombol Calculate Solusi 2 ===
        private void cal_inv2_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                double a1 = double.Parse(m_a1.Text);
                double a2 = double.Parse(m_a2.Text);

                // Ambil nilai Px dan Py dari input user
                double Px = double.Parse(m_px.Text);
                double Py = double.Parse(m_py.Text);

                // Pastikan target masih dalam jangkauan
                double distance = Math.Sqrt(Px * Px + Py * Py);
                if (distance > a1 + a2 || distance < Math.Abs(a1 - a2))
                {
                    MessageBox.Show("Target di luar jangkauan robot.");
                    return;
                }

                // Rumus inverse kinematics (solusi elbow-down)
                double cos_teta2 = (Px * Px + Py * Py - a1 * a1 - a2 * a2) / (2 * a1 * a2);
                cos_teta2 = Math.Max(-1.0, Math.Min(1.0, cos_teta2)); // clamp
                double teta2 = -Math.Acos(cos_teta2);
                double teta1 = Math.Atan2(Py, Px) - Math.Atan2(a2 * Math.Sin(teta2), a1 + a2 * Math.Cos(teta2));

                // Konversi ke derajat
                double deg_teta1 = teta1 * 180.0 / Math.PI;
                double deg_teta2 = teta2 * 180.0 / Math.PI;

                // Tampilkan hasil ke textbox
                m_satu1.Text = deg_teta1.ToString("F2");
                m_dua2.Text = deg_teta2.ToString("F2");
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error IK Solusi 2: " + ex.Message);
            }
        }

        // === Tombol Run Solusi 2 ===
        private void run_inv2_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (serialPort == null || !serialPort.IsOpen)
                {
                    MessageBox.Show("Serial port belum terbuka.");
                    return;
                }

                // ambil nilai dari textbox (hasil calculate sebelumnya)
                double deg_teta1 = double.Parse(m_satu1.Text);
                double deg_teta2 = double.Parse(m_dua2.Text);

                // konversi ke pulse servo sesuai mapping
                double pulse0 = Interpolate(deg_teta1, 0, 2430, 180, 620);   // servo 0
                double pulse2 = Interpolate(deg_teta2, -90, 2450, 90, 660); // servo 2

                string cmd0 = $"#0 P{(int)pulse0} S500\r\n"; 
                string cmd2 = $"#2 P{(int)pulse2} S500\r\n"; 
                
                serialPort.Write(cmd0); 
                serialPort.Write(cmd2);

                // === sinkronkan slider & textbox dengan hasil IK ===
                //sliderServo0.Value = deg_teta1;
                //sliderServo2.Value = deg_teta2;

                txtSudut0.Text = deg_teta1.ToString("F0");
                txtSudut2.Text = deg_teta2.ToString("F0");

                
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error Run IK Solusi 2: " + ex.Message);
            }
        }



        private void BtnClaw_Open_Click(object sender, RoutedEventArgs e)
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                // Servo 4 open (contoh: 1010us)
                string message = "#4 P770 S200\r\n";
                serialPort.Write(message);
            }
        }

        private void BtnClaw_Close_Click(object sender, RoutedEventArgs e)
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                // Servo 4 close (contoh: 2000us)
                string message = "#4 P2200 S200\r\n";
                serialPort.Write(message);
            }
        }


    }
}