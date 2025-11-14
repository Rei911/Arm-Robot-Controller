using LiveCharts;
using LiveCharts.Defaults;
using LiveCharts.Wpf;
using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Windows;
using System.Windows.Media;
using System.Windows.Threading;

namespace Piano
{
    public partial class MainWindow : Window
    {
        // ==============================
        // 🔹 PARAMETER DAN DATA ROBOT
        // ==============================
        private double a1, a2, a3; // Panjang link

        

        // ==============================
        // 🔹 DATA UNTUK CHART (LiveCharts)
        // ==============================
        public ChartValues<ObservablePoint> Link1Points { get; set; }
        public ChartValues<ObservablePoint> Link2Points { get; set; }
        public ChartValues<ObservablePoint> Link3Points { get; set; }
        public ChartValues<ObservablePoint> BasePoint { get; set; }
        public ChartValues<ObservablePoint> ElbowPoint { get; set; }
        public ChartValues<ObservablePoint> WristPoint { get; set; }
        public ChartValues<ObservablePoint> EndEffectorPoint { get; set; }

        // ==============================
        // 🔹 KONFIGURASI NADA PIANO
        // ==============================
        private Dictionary<string, (double t1, double t2, double t3)> noteAngles;
        private (double t1, double t2, double t3) currentAngles;

        // ======================
        // Variabel untuk Robot AL5A
        // ======================
        private SerialPort serialPort;
        private bool isRobotConnected = false;
        private DispatcherTimer refreshTimer;


        public MainWindow()
        {
            InitializeComponent();

            // ✅ Buat objek SerialPort terlebih dahulu
            serialPort = new SerialPort();

            // === Panjang lengan (mm) ===
            a1 = 120;
            a2 = 130;
            a3 = 85;

            // === Panjang lengan (mm) ===
            a1 = 120;
            a2 = 130;
            a3 = 85;

            // === Inisialisasi data chart ===
            Link1Points = new ChartValues<ObservablePoint>();
            Link2Points = new ChartValues<ObservablePoint>();
            Link3Points = new ChartValues<ObservablePoint>();
            BasePoint = new ChartValues<ObservablePoint>();
            ElbowPoint = new ChartValues<ObservablePoint>();
            WristPoint = new ChartValues<ObservablePoint>();
            EndEffectorPoint = new ChartValues<ObservablePoint>();

            DataContext = this;

            // === Pemetaan sudut tiap not ===
            noteAngles = new Dictionary<string, (double, double, double)>
                {
                { "Do",   (101, 0, 0) },
                { "Re",   (97, 0, 0) },
                { "Mi",   (96, 0, 0) },
                { "Fa",   (93, 0, 0) },
                { "Sol",  (89, 0, 0) },
                { "La",   (85, 0, 0) },
                { "Si",   (81, 0, 0) },
                { "Do2",  (77, 0, 0) }
            };

            // === Posisi awal di "Do" ===
            currentAngles = noteAngles["Do"];

            // Gambar posisi awal (tanpa animasi)
            var (x1, y1, x2, y2, x3, y3) = ForwardKinematics(currentAngles.t1, currentAngles.t2, currentAngles.t3);
            UpdateChart(x1, y1, x2, y2, x3, y3);

            txtRobotInfo.Text = "🤖 Robot siap di posisi awal (Do)";
            txtLastCommand.Text = $"θ1={currentAngles.t1}, θ2={currentAngles.t2}, θ3={currentAngles.t3}";

            // === Inisialisasi COM Port List ===
            cmbPorts.Items.Clear();
            foreach (string port in SerialPort.GetPortNames())
            {
                cmbPorts.Items.Add(port);
            }

            if (cmbPorts.Items.Count > 0)
            {
                cmbPorts.SelectedIndex = 0;
                txtRobotInfo.Text += $"\n🟢 {cmbPorts.Items.Count} port terdeteksi.";
            }
            else
            {
                txtRobotInfo.Text += "\n🔴 Tidak ada COM port terdeteksi.";
            }
            // ⚠️ Jangan panggil animasi otomatis di awal
            // PlayNote("Do");
            // SimulatePress();
        }


        // ==============================
        // 🔹 EVENT HANDLER TAMBAHAN (kosong sementara)
        // ==============================
        
        private void BtnSendToRobot_Click(object sender, RoutedEventArgs e) { }
        private void BtnRobotOrigin_Click(object sender, RoutedEventArgs e)
        {
            if (!isRobotConnected || serialPort == null || !serialPort.IsOpen)
            {
                MessageBox.Show("Robot not connected!");
                return;
            }

            try
            {
                // Sudut origin
                double theta0 = 90.0; // Servo 0
                double theta2 = 0.0;  // Servo 2
                double theta3 = 0.0;  // Servo 3

                // Konversi ke pulsa servo
                int p0 = (int)AngleToPulse(0, theta0);
                int p2 = (int)AngleToPulse(1, theta2);
                int p3 = (int)AngleToPulse(2, theta3);

                // Format perintah SSC-32 ke robot
                string originCmd = $"#0 P{p0} #1 P2120 #2 P{p2} #3 P{p3} T800\r\n";

                // Kirim perintah via serial
                serialPort.Write(originCmd);

                txtRobotInfo.Text = $"🏁 Robot ke posisi origin: θ0={theta0}, θ2={theta2}, θ3={theta3}";
                txtLastCommand.Text = originCmd;

                // Update posisi visual robot
                var (x1, y1, x2, y2, x3, y3) = ForwardKinematics(theta0, theta2, theta3);
                UpdateChart(x1, y1, x2, y2, x3, y3);
            }
            catch (Exception ex)
            {
                txtRobotInfo.Text = $"❌ Gagal kirim perintah origin: {ex.Message}";
            }
        }


        private void BtnExit_Click(object sender, RoutedEventArgs e) => Close();
        private void BtnConnectRobot_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (serialPort.IsOpen)
                {
                    // Disconnect robot
                    serialPort.Close();
                    btnConnectRobot.Content = "Connect Robot";
                    txtRobotStatus.Text = "Disconnected";
                    txtRobotStatus.Foreground = Brushes.Red;
                    isRobotConnected = false;
                    txtRobotInfo.Text = "Robot disconnected";
                }
                else
                {
                    // Connect robot
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

                    btnConnectRobot.Content = "Disconnect Robot";
                    txtRobotStatus.Text = "Connected";
                    txtRobotStatus.Foreground = Brushes.Green;
                    isRobotConnected = true;
                    txtRobotInfo.Text = "Robot AL5A connected to " + serialPort.PortName;

                    string initMessage = "#1 P2220 S100\r\n";  //Inisialisasi Servo 1 dalam posisi Riak Air
                    string initMessage2 = "#0 P1525 S100\r\n"; //Inisialisasi Servo 0
                    string initMessage3 = "#2 P1502 S100\r\n"; //Inisialisasi Servo 2
                    string originServo3 = "#3 P1560 S500\r\n";  // Servo 3 origin


                    serialPort.Write(initMessage);
                    serialPort.Write(initMessage2);
                    serialPort.Write(initMessage3);
                    serialPort.Write(originServo3);

                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error: {ex.Message}");
                txtRobotInfo.Text = "Connection failed: " + ex.Message;
            }
        }
        private void BtnClearPath_Click(object sender, RoutedEventArgs e) { }
        private void RbTimeTrack_Click(object sender, RoutedEventArgs e) { }
        private void RbTimePath_Click(object sender, RoutedEventArgs e) { }

        // ==============================
        // 🔹 FORWARD KINEMATICS
        // ==============================
        private (double x1, double y1, double x2, double y2, double x3, double y3)
            ForwardKinematics(double theta1Deg, double theta2Deg, double theta3Deg)
        {
            double t1 = theta1Deg * Math.PI / 180.0;
            double t2 = theta2Deg * Math.PI / 180.0;
            double t3 = theta3Deg * Math.PI / 180.0;

            double x1 = a1 * Math.Cos(t1);
            double y1 = a1 * Math.Sin(t1);
            double x2 = x1 + a2 * Math.Cos(t1 + t2);
            double y2 = y1 + a2 * Math.Sin(t1 + t2);
            double x3 = x2 + a3 * Math.Cos(t1 + t2 + t3);
            double y3 = y2 + a3 * Math.Sin(t1 + t2 + t3);

            return (x1, y1, x2, y2, x3, y3);
        }

        // ==============================
        // 🔹 KONVERSI SUDUT → PULSE SERVO
        // ==============================
        private double AngleToPulse(int servoId, double angle)
        {
            switch (servoId)
            {
                case 0: // Base servo: 0-180° → 2430-620µs
                    return Interpolate(angle, 0, 2390, 180, 640);
                case 2: // Link 2: -90-90° → 2450-660µs
                    return Interpolate(angle, -90, 2450, 90, 670);
                case 3: // Link 3: -90-90° → 2430-650µs
                    return Interpolate(angle, -90, 2400, 90, 610);
                default:
                    return 1500; // Default posisi tengah
            }
        }

        // ==============================
        // 🔹 KIRIM DATA KE ROBOT VIA SERIAL (PAKAI KONVERSI SUDUT → PULSE)
        // ==============================
        private void SendAnglesToRobot(double t0, double t2, double t3)
        {
            if (!isRobotConnected || serialPort == null || !serialPort.IsOpen)
            {
                txtRobotInfo.Text = "⚠️ Robot belum terkoneksi. Perintah tidak dikirim.";
                return;
            }

            try
            {
                int p0 = (int)AngleToPulse(0, t0);
                int p2 = (int)AngleToPulse(1, t2);
                int p3 = (int)AngleToPulse(2, t3);

                // Servo 1 tetap di posisi "Up" (2185)
                string cmd = $"#0 P{p0} #1 P2185 #2 P{p2} #3 P{p3} T100\r\n";

                serialPort.Write(cmd);
                txtRobotInfo.Text = $"📤 Dikirim ke robot: {cmd.Trim()}";
            }
            catch (Exception ex)
            {
                txtRobotInfo.Text = $"❌ Gagal kirim data: {ex.Message}";
            }
        }

        private async System.Threading.Tasks.Task PressRataAirAsync(string noteName)
        {
            if (!isRobotConnected || serialPort == null || !serialPort.IsOpen)
            {
                Dispatcher.Invoke(() =>
                {
                    txtPianoStatus.Text = "⚠️ Robot belum terkoneksi.";
                });
                return;
            }

            try
            {
                // 🔹 Update status (harus dari UI thread)
                Dispatcher.Invoke(() =>
                {
                    txtPianoStatus.Text = $"🎵 Menekan not {noteName}...";
                });

                // 🔹 Turunkan (Down)
                serialPort.Write("#1 P2235 T100\r\n");
                await System.Threading.Tasks.Task.Delay(500); // tahan sedikit

                // 🔹 Naikkan kembali (Up)
                serialPort.Write("#1 P2175 T200\r\n");
                

                // 🔹 Kembali ke Idle
                Dispatcher.Invoke(() =>
                {
                    txtPianoStatus.Text = "Idle";
                });

                Dispatcher.Invoke(() =>
                {
                    txtRobotInfo.Text = "👆 Servo 1: Tekan rata air (Down → Up)";
                });
            }
            catch (Exception ex)
            {
                Dispatcher.Invoke(() =>
                {
                    txtPianoStatus.Text = "Error: " + ex.Message;
                });
            }
        }



        private double Interpolate(double x, double x1, double y1, double x2, double y2)
        {
            return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
        }

        private void BtnClaw_Open_Click(object sender, RoutedEventArgs e)
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                // Servo 4 open (contoh: 1010us)
                string message = "#4 P770 T700\r\n";
                serialPort.Write(message);
            }
        }

        private void BtnClaw_Close_Click(object sender, RoutedEventArgs e)
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                // Servo 4 close (contoh: 2000us)
                string message = "#4 P2200 T700\r\n";
                serialPort.Write(message);
            }
        }
        // ==============================
        // 🔹 UPDATE VISUAL ROBOT DI CHART
        // ==============================
        private void UpdateChart(double x1, double y1, double x2, double y2, double x3, double y3)
        {
            Link1Points.Clear();
            Link2Points.Clear();
            Link3Points.Clear();
            BasePoint.Clear();
            ElbowPoint.Clear();
            WristPoint.Clear();
            EndEffectorPoint.Clear();

            Link1Points.Add(new ObservablePoint(0, 0));
            Link1Points.Add(new ObservablePoint(x1, y1));

            Link2Points.Add(new ObservablePoint(x1, y1));
            Link2Points.Add(new ObservablePoint(x2, y2));

            Link3Points.Add(new ObservablePoint(x2, y2));
            Link3Points.Add(new ObservablePoint(x3, y3));

            BasePoint.Add(new ObservablePoint(0, 0));
            ElbowPoint.Add(new ObservablePoint(x1, y1));
            WristPoint.Add(new ObservablePoint(x2, y2));
            EndEffectorPoint.Add(new ObservablePoint(x3, y3));
        }

        // ==============================
        // 🔹 MEMAINKAN SATU NADA
        // ==============================
        private async System.Threading.Tasks.Task PlayNote(string note)
        {
            if (!noteAngles.ContainsKey(note))
            {
                txtNoteInfo.Text = $"Note '{note}' tidak ditemukan.";
                return;
            }

            var (targetT1, targetT2, targetT3) = noteAngles[note];
            txtNoteInfo.Text = $"🎶 Memainkan note: {note}";
            txtLastCommand.Text = $"Target: θ1={targetT1}, θ2={targetT2}, θ3={targetT3}";

            // ==== 💨 PERPINDAHAN CEPAT ====
            int totalSteps = 6;   // dari 20 → 8
            int stepDelay = 8;   // dari 30ms → 10ms

            for (int step = 0; step <= totalSteps; step++)
            {
                double t = (double)step / totalSteps;
                double iT1 = currentAngles.t1 + (targetT1 - currentAngles.t1) * t;
                double iT2 = currentAngles.t2 + (targetT2 - currentAngles.t2) * t;
                double iT3 = currentAngles.t3 + (targetT3 - currentAngles.t3) * t;

                var (x1, y1, x2, y2, x3, y3) = ForwardKinematics(iT1, iT2, iT3);
                UpdateChart(x1, y1, x2, y2, x3, y3);

                if (step % 2 == 0)
                    SendAnglesToRobot(iT1, iT2, iT3);

                await Task.Delay(stepDelay);
            }

            currentAngles = (targetT1, targetT2, targetT3);

            // Delay kecil sesudah sampai posisi not
            await Task.Delay(80);

            // ==== 👇 TEKAN DENGAN JEDA ====
            await PressRataAirAsync(note);
        }



        // ==============================
        // 🔹 MEMAINKAN LAGU (DAFTAR NOT)
        // ==============================
        // ==============================
        // 🔹 VARIABEL KONTROL PIANO
        // ==============================
        private bool isPlaying = false;
        private bool isPaused = false;
        private CancellationTokenSource pianoToken;

        // ==============================
        // 🔹 MEMAINKAN LAGU (DENGAN PAUSE & REWIND)
        // ==============================
        private async Task PlaySongAsync(List<(string note, int durasi)> lagu, CancellationToken token)
        {
            isPlaying = true;
            isPaused = false;

            foreach (var (note, durasi) in lagu)
            {
                // Jika dibatalkan (Rewind)
                if (token.IsCancellationRequested)
                {
                    txtRobotInfo.Text = "⏹ Dihentikan / Rewind.";
                    break;
                }

                // Jika sedang pause, tunggu
                while (isPaused)
                {
                    await Task.Delay(100);
                    if (token.IsCancellationRequested)
                        break;
                }

                if (note.Equals("jeda", StringComparison.OrdinalIgnoreCase))
                {
                    txtRobotInfo.Text = $"🤖 Diam selama {durasi} ms...";
                    await Task.Delay(durasi, token);
                }
                else
                {
                    await PlayNote(note);
                    await Task.Delay(durasi, token);
                }
            }

            isPlaying = false;
            txtRobotInfo.Text = "🎵 Lagu selesai dimainkan.";
        }

        // ==============================
        // 🔹 BUTTON: START / PAUSE / REWIND
        // ==============================
        private async void BtnStartPiano_Click(object sender, RoutedEventArgs e)
        {
            if (isPlaying)
            {
                txtRobotInfo.Text = "⚠️ Lagu sedang dimainkan!";
                return;
            }

            var lagu = new List<(string note, int durasi)>
            {
                ("Do", 400), ("Mi", 400), ("Do", 400), ("Mi", 400), ("Fa", 400), ("Sol", 400), ("Sol", 400),
                ("jeda", 400),
                ("Si", 400), ("Do2", 400), ("Si", 400), ("Do2", 400), ("Si", 400), ("Sol", 400),
                ("jeda", 400),
                ("Do", 400), ("Mi", 400), ("Do", 400), ("Mi", 400), ("Fa", 400), ("Sol", 400), ("Sol", 400),
                ("jeda", 400),
                ("Si", 400), ("Do2", 400), ("Si", 400), ("Do2", 400), ("Si", 400), ("Sol", 400),
                ("jeda", 400),
                ("Do", 400), ("Mi", 400), ("Sol", 400), ("Fa", 400), ("Fa", 400),
                ("jeda", 400),
                ("Sol", 400), ("Fa", 400), ("Mi", 400), ("Do", 400), ("Fa", 400), ("Mi", 400), ("Do", 400),
                ("jeda", 400),
                ("Do", 400), ("Mi", 400), ("Sol", 400), ("Fa", 400), ("Fa", 400),
                ("jeda", 400),
                ("Sol", 400), ("Fa", 400), ("Mi", 400), ("Do", 400), ("Fa", 400), ("Mi", 400), ("Do", 400)

            };

            pianoToken = new CancellationTokenSource();

            try
            {
                await PlaySongAsync(lagu, pianoToken.Token);
            }
            catch (TaskCanceledException)
            {
                txtRobotInfo.Text = "🚫 Lagu dihentikan.";
            }
        }

        // Tombol Pause / Resume
        private void BtnPausePiano_Click(object sender, RoutedEventArgs e)
        {
            if (!isPlaying)
            {
                txtRobotInfo.Text = "⚠️ Tidak ada lagu yang sedang dimainkan.";
                return;
            }

            isPaused = !isPaused;
            Pause_Piano.Content = isPaused ? "Resume" : "Pause";
            txtRobotInfo.Text = isPaused ? "⏸ Lagu dijeda." : "▶️ Lanjut bermain.";
        }

        // Tombol Rewind
        private void BtnRewindPiano_Click(object sender, RoutedEventArgs e)
        {
            if (pianoToken != null)
            {
                pianoToken.Cancel();
                isPlaying = false;
                isPaused = false;
                Pause_Piano.Content = "Pause";
                txtRobotInfo.Text = "⏪ Lagu diulang ke awal.";
            }
        }

    }
}
