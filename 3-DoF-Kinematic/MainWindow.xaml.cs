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
                else if (servoId == 4)
                {
                    // Ambil nilai Min & Maks dari textbox
                    double minPulse = double.Parse(txtClawMin.Text);
                    double maxPulse = double.Parse(txtClawMax.Text);

                    // Nilai slider = lebar jepit (mm), misal 0..200
                    double clawMM = angle;

                    // Interpolasi: 0 mm → minPulse, 200 mm → maxPulse
                    pulse = Interpolate(clawMM, 0, minPulse, 200, maxPulse);

                    // Update UI
                    txtClawMM.Text = clawMM.ToString("F0");
                    txtClawInterp.Text = pulse.ToString("F0");
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
                // Ambil parameter panjang link
                double a1 = double.Parse(m_a1.Text);
                double a2 = double.Parse(m_a2.Text);
                double a3 = double.Parse(m_a3.Text);

                // Sudut servo dari slider (dalam derajat → radian)
                double teta1 = sliderServo0.Value * Math.PI / 180.0;
                double teta2 = (sliderServo2.Value) * Math.PI / 180.0; // sudah normalisasi di slider
                double teta3 = (sliderServo3.Value) * Math.PI / 180.0;

                // Hitung orientasi


                // Hitung qx, qy sesuai rumus FK
                double qx = a1 * Math.Cos(teta1)
                          + a2 * Math.Cos(teta1 + teta2)
                          + a3 * Math.Cos(teta1 + teta2 + teta3);

                double qy = a1 * Math.Sin(teta1)
                          + a2 * Math.Sin(teta1 + teta2)
                          + a3 * Math.Sin(teta1 + teta2 + teta3);

                // Orientasi φ = θ1 + θ2 + θ3 (ubah ke derajat)
                double orientasi = (teta1 + teta2 + teta3) * 180.0 / Math.PI;

                // Tampilkan
                m_qx.Text = qx.ToString("F3");
                m_qy.Text = qy.ToString("F3");
                m_orientasi.Text = orientasi.ToString("F2");

            }
            catch (Exception ex)
            {
                MessageBox.Show("Error FK: " + ex.Message);
            }
        }

        private void CalculateInverseKinematics(bool isSolution2)
        {
            try
            {
                // Ambil panjang link
                double a1 = double.Parse(m_a1.Text);
                double a2 = double.Parse(m_a2.Text);
                double a3 = double.Parse(m_a3.Text);

                // Ambil input manual dari textbox
                double qx = double.Parse(m_qx.Text);
                double qy = double.Parse(m_qy.Text);
                double orientasiDeg = double.Parse(m_orientasi.Text);
                double orientasi = orientasiDeg * Math.PI / 180.0; // ubah ke radian

                // === Hitung posisi wrist (titik pergelangan tangan) ===
                double px = qx - a3 * Math.Cos(orientasi);
                double py = qy - a3 * Math.Sin(orientasi);

                // === Hitung sudut-sudut ===
                double D = (px * px + py * py - a1 * a1 - a2 * a2) / (2 * a1 * a2);
                D = Math.Max(-1.0, Math.Min(1.0, D)); // jaga agar tetap valid

                double teta2 = isSolution2 ? -Math.Acos(D) : Math.Acos(D);
                double teta1 = Math.Atan2(py, px) - Math.Atan2(a2 * Math.Sin(teta2), a1 + a2 * Math.Cos(teta2));
                double teta3 = orientasi - (teta1 + teta2);

                // === Konversi ke derajat ===
                double deg_teta1 = teta1 * 180.0 / Math.PI;
                double deg_teta2 = teta2 * 180.0 / Math.PI;
                double deg_teta3 = teta3 * 180.0 / Math.PI;

                // === Tampilkan hasil ===
                if (!isSolution2)
                {
                    m_satu.Text = deg_teta1.ToString("F2");
                    m_dua.Text = deg_teta2.ToString("F2");
                    m_tiga.Text = deg_teta3.ToString("F2");
                }
                else
                {
                    m_satu2.Text = deg_teta1.ToString("F2");
                    m_dua2.Text = deg_teta2.ToString("F2");
                    m_tiga2.Text = deg_teta3.ToString("F2");
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error perhitungan Inverse Kinematics: " + ex.Message);
            }
        }



        // === Tombol Calculate Solusi 1 ===
        private void cal_inv1_Click(object sender, RoutedEventArgs e)
        {
            try { CalculateInverseKinematics(false); }
            catch (Exception ex) { MessageBox.Show("Error IK Solusi 1: " + ex.Message); }
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

                // Ambil nilai hasil IK
                double deg_teta1 = double.Parse(m_satu.Text);
                double deg_teta2 = double.Parse(m_dua.Text);
                double deg_teta3 = double.Parse(m_tiga.Text);

                // Konversi ke pulse servo sesuai mapping
                double pulse0 = Interpolate(deg_teta1, 0, 2430, 180, 620);    // servo 0
                double pulse2 = Interpolate(deg_teta2, -90, 2450, 90, 660);   // servo 2
                double pulse3 = Interpolate(deg_teta3, -90, 2430, 90, 650);   // servo 3

                string cmd0 = $"#0 P{(int)pulse0} S500\r\n";
                string cmd2 = $"#2 P{(int)pulse2} S500\r\n";
                string cmd3 = $"#3 P{(int)pulse3} S500\r\n";

                serialPort.Write(cmd0);
                serialPort.Write(cmd2);
                serialPort.Write(cmd3);

                // Sinkronkan slider dengan hasil IK
                sliderServo0.Value = deg_teta1;
                sliderServo2.Value = deg_teta2;
                sliderServo3.Value = deg_teta3;

        
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error Run IK Solusi 1: " + ex.Message);
            }
        }



        // === Tombol Calculate Solusi 2 ===
        private void cal_inv2_Click(object sender, RoutedEventArgs e)
        {
            try { CalculateInverseKinematics(true); }
            catch (Exception ex) { MessageBox.Show("Error IK Solusi 2: " + ex.Message); }
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

                // Ambil hasil solusi 2
                double deg_teta1 = double.Parse(m_satu2.Text);
                double deg_teta2 = double.Parse(m_dua2.Text);
                double deg_teta3 = double.Parse(m_tiga2.Text);

                // Konversi ke pulse servo
                double pulse0 = Interpolate(deg_teta1, 0, 2430, 180, 620);    // servo 0
                double pulse2 = Interpolate(deg_teta2, -90, 2450, 90, 660);   // servo 2
                double pulse3 = Interpolate(deg_teta3, -90, 2430, 90, 650);   // servo 3

                string cmd0 = $"#0 P{(int)pulse0} S500\r\n";
                string cmd2 = $"#2 P{(int)pulse2} S500\r\n";
                string cmd3 = $"#3 P{(int)pulse3} S500\r\n";

                serialPort.Write(cmd0);
                serialPort.Write(cmd2);
                serialPort.Write(cmd3);

                // Sinkronkan slider
                sliderServo0.Value = deg_teta1;
                sliderServo2.Value = deg_teta2;
                sliderServo3.Value = deg_teta3;

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