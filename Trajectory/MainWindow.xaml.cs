using LiveCharts;
using LiveCharts.Defaults;
using LiveCharts.Wpf;
using System;
using System.ComponentModel;
using System.IO.Ports;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace Trajectory
{
    public partial class MainWindow : Window
    {
        public ChartValues<ObservablePoint> TrajectoryPoints { get => trajectoryPoints; }
        public ChartValues<ObservablePoint> Link1Points { get => link1Points; }
        public ChartValues<ObservablePoint> Link2Points { get => link2Points; }
        public ChartValues<ObservablePoint> Link3Points { get => link3Points; }
        public ChartValues<ObservablePoint> BasePoint { get => basePoint; }
        public ChartValues<ObservablePoint> ElbowPoint { get => elbowPoint; }
        public ChartValues<ObservablePoint> WristPoint { get => wristPoint; }
        public ChartValues<ObservablePoint> EndEffectorPoint { get => endEffectorPoint; }

        public SeriesCollection RobotSeries
        {
            get { return robotSeries; }
            set
            {
                robotSeries = value;
                OnPropertyChanged();
            }
        }

        public Func<double, string> XFormatter { get; set; }
        public Func<double, string> YFormatter { get; set; }

        // Variabel LiveCharts
        private SeriesCollection robotSeries;
        private ChartValues<ObservablePoint> trajectoryPoints;
        private ChartValues<ObservablePoint> link1Points, link2Points, link3Points;
        private ChartValues<ObservablePoint> basePoint, elbowPoint, wristPoint, endEffectorPoint;



        private void InitializeChart()
        {
            trajectoryPoints = new ChartValues<ObservablePoint>();
            link1Points = new ChartValues<ObservablePoint>();
            link2Points = new ChartValues<ObservablePoint>();
            link3Points = new ChartValues<ObservablePoint>();
            basePoint = new ChartValues<ObservablePoint>();
            elbowPoint = new ChartValues<ObservablePoint>();
            wristPoint = new ChartValues<ObservablePoint>();
            endEffectorPoint = new ChartValues<ObservablePoint>();

            RobotSeries = new SeriesCollection
    {
        new LineSeries
        {
            Title = "Link 1",
            Values = link1Points,
            Stroke = Brushes.Blue,
            Fill = Brushes.Transparent,
            PointGeometry = null,
            StrokeThickness = 2
        },
        new LineSeries
        {
            Title = "Link 2",
            Values = link2Points,
            Stroke = Brushes.Green,
            Fill = Brushes.Transparent,
            PointGeometry = null,
            StrokeThickness = 2
        },
        new LineSeries
        {
            Title = "Link 3",
            Values = link3Points,
            Stroke = Brushes.Red,
            Fill = Brushes.Transparent,
            PointGeometry = null,
            StrokeThickness = 2
        },
        new LineSeries
        {
            Title = "Trajectory",
            Values = trajectoryPoints,
            Stroke = Brushes.OrangeRed,
            Fill = Brushes.Transparent,
            PointGeometry = null,
            StrokeThickness = 1.5
        },
        new ScatterSeries
        {
            Title = "End-Effector",
            Values = endEffectorPoint,
            Fill = Brushes.Red,
            MinPointShapeDiameter = 6,
            MaxPointShapeDiameter = 6
        }
    };

            XFormatter = val => val.ToString("0");
            YFormatter = val => val.ToString("0");

            DataContext = this;
        }


        private void UpdateRobotChart(double sdt1, double sdt2, double sdt3)
        {
            // Hapus data link sebelumnya (tapi jangan hapus lintasan)
            link1Points.Clear();
            link2Points.Clear();
            link3Points.Clear();
            basePoint.Clear();
            elbowPoint.Clear();
            wristPoint.Clear();
            endEffectorPoint.Clear();

            // Hitung posisi joint
            double r1 = sdt1 * Math.PI / 180.0;
            double r2 = sdt2 * Math.PI / 180.0;
            double r3 = sdt3 * Math.PI / 180.0;

            double kx = a1 * Math.Cos(r1);
            double ky = a1 * Math.Sin(r1);

            double px = kx + a2 * Math.Cos(r1 + r2);
            double py = ky + a2 * Math.Sin(r1 + r2);

            double qx = px + a3 * Math.Cos(r1 + r2 + r3);
            double qy = py + a3 * Math.Sin(r1 + r2 + r3);

            // Tambahkan titik tiap joint
            link1Points.Add(new ObservablePoint(0, 0));
            link1Points.Add(new ObservablePoint(kx, ky));

            link2Points.Add(new ObservablePoint(kx, ky));
            link2Points.Add(new ObservablePoint(px, py));

            link3Points.Add(new ObservablePoint(px, py));
            link3Points.Add(new ObservablePoint(qx, qy));

            basePoint.Add(new ObservablePoint(0, 0));
            elbowPoint.Add(new ObservablePoint(kx, ky));
            wristPoint.Add(new ObservablePoint(px, py));
            endEffectorPoint.Add(new ObservablePoint(qx, qy));

            // Tambahkan ke trajectory (jejak lintasan end-effector)
            trajectoryPoints.Add(new ObservablePoint(qx, qy));

            robotChart.Update(true, true);  // force redraw
        }








        public event PropertyChangedEventHandler PropertyChanged;
        protected virtual void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        // ======================
        // Variabel utama trajectory
        // ======================
        double a1 = 120, a2 = 130, a3 = 85;
        int nPoints = 25;
        int timeMs = 1000;

        int mode = 0;       // 0 = Time/Path, 1 = Time/Track
        int space = 0;      // 0 = Joint Space, 1 = Work Space
        int currentStep = 0;

        double[] theta1, theta2, theta3;
        double[] qx, qy, orient;
        double originX, originY;
        double scale = 1.0;
        Polyline trackPath = new Polyline
        {
            Stroke = Brushes.OrangeRed,
            StrokeThickness = 1.5,
            Opacity = 0.6
        };
        DispatcherTimer timer;

        // ======================
        // Variabel untuk Robot AL5A
        // ======================
        private SerialPort serialPort;
        private bool isRobotConnected = false;
        private DispatcherTimer refreshTimer;

        public MainWindow()
        {
            InitializeComponent();
            InitializeChart();

            // Timer untuk animasi trajectory
            timer = new DispatcherTimer();
            timer.Tick += OnTimer;

            // Inisialisasi serial port untuk robot
            InitializeSerialPort();

        }

        // ======================
        // 1. Fungsi Inisialisasi Serial Port
        // ======================

        private void InitializeSerialPort()
        {
            serialPort = new SerialPort();
            RefreshPortList();

            // Timer untuk refresh daftar COM port setiap 2 detik
            refreshTimer = new DispatcherTimer();
            refreshTimer.Interval = TimeSpan.FromSeconds(2);
            refreshTimer.Tick += (s, e) => RefreshPortList();
            refreshTimer.Start();
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

        // ======================
        // 2. Fungsi Koneksi Robot
        // ======================

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

        private void BtnRobotOrigin_Click(object sender, RoutedEventArgs e)
        {
            if (!isRobotConnected)
            {
                MessageBox.Show("Robot not connected!");
                return;
            }

            // Posisi origin: theta1=90°, theta2=0°, theta3=0°
            SendToRobot(90, 0, 0);
            txtRobotInfo.Text = "Robot moved to origin position";
        }

        private void BtnSendToRobot_Click(object sender, RoutedEventArgs e)
        {
            if (!isRobotConnected)
            {
                MessageBox.Show("Robot not connected!");
                return;
            }

            try
            {
                // Ambil sudut terakhir dari trajectory
                if (space == 0 && theta1 != null && theta1.Length > 0)
                {
                    int lastIndex = Math.Min(currentStep, theta1.Length - 1);
                    SendToRobot(theta1[lastIndex], theta2[lastIndex], theta3[lastIndex]);
                    txtRobotInfo.Text = "Sent current trajectory position to robot";
                }
                else if (space == 1 && qx != null && qx.Length > 0)
                {
                    int lastIndex = Math.Min(currentStep, qx.Length - 1);
                    // Untuk workspace, perlu hitung IK dulu
                    CalculateAndSendToRobot(qx[lastIndex], qy[lastIndex], orient[lastIndex]);
                    txtRobotInfo.Text = "Sent current workspace position to robot";
                }
                else
                {
                    MessageBox.Show("No trajectory calculated yet!");
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error: {ex.Message}");
            }
        }

        // ======================
        // 3. Fungsi Konversi dan Komunikasi Robot
        // ======================

        private double AngleToPulse(int servoId, double angle)
        {
            switch (servoId)
            {
                case 0: // Base servo: 0-180° → 2430-620us
                    return Interpolate(angle, 0, 2390, 180, 640);
                case 2: // Link 2: -90-90° → 2450-660us
                    return Interpolate(angle, -90, 2450, 90, 670);
                case 3: // Link 3: -90-90° → 2430-650us
                    return Interpolate(angle, -90, 2400, 90, 610);
                default:
                    return 1500; // Default center position
            }
        }

        private double Interpolate(double x, double x1, double y1, double x2, double y2)
        {
            return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
        }

        private async void SendToRobot(double theta1, double theta2, double theta3)
        {
            if (!isRobotConnected || serialPort == null || !serialPort.IsOpen)
                return;

            try
            {
                // Konversi sudut ke pulse
                double pulse0 = AngleToPulse(0, theta1);
                double pulse2 = AngleToPulse(2, theta2);
                double pulse3 = AngleToPulse(3, theta3);
                string cmd;
                // Kirim perintah ke servo dengan delay

                if (mode == 0)
                {
                     cmd = $"#0 P{(int)pulse0} #2 P{(int)pulse2} #3 P{(int)pulse3} T{(int)timeMs}\r\n";
                } else {
                     cmd = $"#0 P{(int)pulse0} #2 P{(int)pulse2} #3 P{(int)pulse3} T{(int)timeMs/nPoints}\r\n";
                };

                serialPort.Write(cmd);
                

                // Update status
                txtLastCommand.Text = $"Last: θ1={theta1:0.0}°→{pulse0:0}, θ2={theta2:0.0}°→{pulse2:0}, θ3={theta3:0.0}°→{pulse3:0}";

                Console.WriteLine($"Sent to Robot: θ1={theta1:0.0}°→{pulse0:0}, θ2={theta2:0.0}°→{pulse2:0}, θ3={theta3:0.0}°→{pulse3:0}");
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error sending to robot: {ex.Message}");
                txtRobotInfo.Text = "Error sending command: " + ex.Message;
            }
        }

        private void CalculateAndSendToRobot(double qx, double qy, double orientation)
        {
            // Hitung inverse kinematics dan kirim ke robot
            InverseKinematic(qx, qy, orientation);
        }

        // ======================
        // 4. Fungsi Menggambar Lengan (Existing)
        // ======================

        double ToCanvasX(double x)
        {
            return originX + x * scale;
        }

        double ToCanvasY(double y)
        {
            return originY - y * scale;
        }

        void UpdateUserInput()
        {
            // Panjang link
            a1 = double.Parse(txtA1.Text);
            a2 = double.Parse(txtA2.Text);
            a3 = double.Parse(txtA3.Text);

            // Trajectory config
            nPoints = int.Parse(txtNPoints.Text);
            timeMs = int.Parse(txtTimeMs.Text);
        }





        

           

        // ======================
        // 5. Fungsi Inverse Kinematic (Dimodifikasi)
        // ======================

        void InverseKinematic(double iqx, double iqy, double iorient)
        {
            // posisi wrist
            double ipx = iqx - a3 * Math.Cos(iorient * Math.PI / 180);
            double ipy = iqy - a3 * Math.Sin(iorient * Math.PI / 180);

            // hitung teta2
            double teta2 = (Math.Pow(ipx, 2) + Math.Pow(ipy, 2) - Math.Pow(a1, 2) - Math.Pow(a2, 2)) / (2 * a1 * a2);
            if (teta2 >= 1.0) teta2 = 1.0;
            else if (teta2 <= -1.0) teta2 = -1.0;
            teta2 = Math.Acos(teta2);

            // hitung teta1
            double temp = (a2 * Math.Sin(teta2)) / (a1 + a2 * Math.Cos(teta2));
            double teta1 = Math.Atan2(ipy, ipx) - Math.Atan(temp);

            // konversi ke derajat
            teta2 = teta2 * (180.0 / Math.PI);
            teta1 = teta1 * (180.0 / Math.PI);

            // hitung teta3
            double teta3 = iorient - teta2 - teta1;

            // batasan 0–180 derajat
            if (teta1 < 0) teta1 = 0; 
            else if (teta1 > 180) teta1 = 180; 
            
            if (teta2 < 0) teta2 = 0; 
            else if (teta2 > 180) teta2 = 180; 
            
            if (teta3 < 0) teta3 = 0; 
            else if (teta3 > 180) teta3 = 180;

            // gambar lengan
            UpdateRobotChart(teta1, teta2, teta3);


            // Kirim ke robot jika terkoneksi
            if (isRobotConnected)
            {
                SendToRobot(teta1, teta2, teta3);
            }
        }

        // ======================
        // 6. Fungsi Trajectory Planning (Existing)
        // ======================

        private void BtnJointCalc_Click(object sender, RoutedEventArgs e)
        {
            UpdateUserInput();

            txtJointOutput.Clear();

            double t1Init = double.Parse(txtTheta1Init.Text);
            double t2Init = double.Parse(txtTheta2Init.Text);
            double t3Init = double.Parse(txtTheta3Init.Text);

            double t1Final = double.Parse(txtTheta1Final.Text);
            double t2Final = double.Parse(txtTheta2Final.Text);
            double t3Final = double.Parse(txtTheta3Final.Text);

            theta1 = new double[nPoints + 1];
            theta2 = new double[nPoints + 1];
            theta3 = new double[nPoints + 1];

            for (int i = 0; i <= nPoints; i++)
            {
                theta1[i] = t1Init + ((t1Final - t1Init) / (float)nPoints) * i;
                theta2[i] = t2Init + ((t2Final - t2Init) / (float)nPoints) * i;
                theta3[i] = t3Init + ((t3Final - t3Init) / (float)nPoints) * i;

                string line = $"{i,3} {theta1[i],6:0.0} {theta2[i],6:0.0} {theta3[i],6:0.0}";
                txtJointOutput.AppendText(line + "\n");
            }

            txtJointOutput.AppendText(
                $"\nJoint trajectory generated from " +
                $"({t1Init:0.0}, {t2Init:0.0}, {t3Init:0.0}) → " +
                $"({t1Final:0.0}, {t2Final:0.0}, {t3Final:0.0})"
            );
        }

        private void BtnJointRun_Click(object sender, RoutedEventArgs e)
        {
            UpdateUserInput();

            space = 0;
            currentStep = 0;
            timer.Interval = TimeSpan.FromMilliseconds(mode == 0 ? timeMs : timeMs / nPoints);
            timer.Start();

            if (isRobotConnected)
            {
                txtRobotInfo.Text = "Running joint space trajectory with robot...";
            }
        }

        private void BtnWorkCalc_Click(object sender, RoutedEventArgs e)
        {
            UpdateUserInput();

            txtWorkOutput.Clear();

            double qxInit = double.Parse(txtQxInit.Text);
            double qyInit = double.Parse(txtQyInit.Text);
            double orientInit = double.Parse(txtOrientInit.Text);

            double qxFinal = double.Parse(txtQxFinal.Text);
            double qyFinal = double.Parse(txtQyFinal.Text);
            double orientFinal = double.Parse(txtOrientFinal.Text);

            qx = new double[nPoints + 1];
            qy = new double[nPoints + 1];
            orient = new double[nPoints + 1];

            for (int i = 0; i <= nPoints; i++)
            {
                qx[i] = qxInit + ((qxFinal - qxInit) / (float)nPoints) * i;
                qy[i] = qyInit + ((qyFinal - qyInit) / (float)nPoints) * i;
                orient[i] = orientInit + ((orientFinal - orientInit) / (float)nPoints) * i;

                txtWorkOutput.AppendText($"{i,3} {qx[i],6:0.0} {qy[i],6:0.0} {orient[i],6:0.0}\n");
            }
        }

        private void BtnWorkRun_Click(object sender, RoutedEventArgs e)
        {
            UpdateUserInput();

            int time;
            currentStep = 0;
            space = 1;

            time = (mode == 0) ? timeMs : timeMs / nPoints;

            timer.Interval = TimeSpan.FromMilliseconds(time);
            timer.Start();

            if (isRobotConnected)
            {
                txtRobotInfo.Text = "Running workspace trajectory with robot...";
            }
        }

        // ======================
        // 7. Event Handler Timer (Dimodifikasi)
        // ======================

        private void OnTimer(object? sender, EventArgs e)
        {
            if (space == 0 && theta1 != null && currentStep < theta1.Length)
            {
                UpdateRobotChart(theta1[currentStep], theta2[currentStep], theta3[currentStep]);

                // Kirim ke robot
                if (isRobotConnected)
                {
                    SendToRobot(theta1[currentStep], theta2[currentStep], theta3[currentStep]);
                }
            }
            else if (space == 1 && qx != null && currentStep < qx.Length)
            {
                InverseKinematic(qx[currentStep], qy[currentStep], orient[currentStep]);
                // Robot sudah dikirim melalui fungsi InverseKinematic
            }

            currentStep++;
            if (currentStep > nPoints)
            {
                timer.Stop();
                currentStep = 0;
                if (isRobotConnected)
                {
                    txtRobotInfo.Text = "Trajectory completed - Robot stopped";
                }
            }
        }

        // ======================
        // 8. Fungsi Lainnya (Existing)
        // ======================

        private void BtnClearPath_Click(object sender, RoutedEventArgs e)
        {
            trajectoryPoints.Clear();
            robotChart.Update(true);
            robotChart.Update(true, true);
        }

        private void RbTimePath_Click(object sender, RoutedEventArgs e)
        {
            mode = 0;
        }

        private void RbTimeTrack_Click(object sender, RoutedEventArgs e)
        {
            mode = 1;
        }

        private void BtnExit_Click(object sender, RoutedEventArgs e)
        {
            // Tutup serial port sebelum exit
            if (serialPort != null && serialPort.IsOpen)
            {
                serialPort.Close();
            }
            Close();
        }
    }
}
