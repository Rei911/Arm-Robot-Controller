using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using System.IO.Ports;
using System.Threading.Tasks;

namespace Trajectory
{
    public partial class MainWindow : Window
    {
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
                    string originClaw = "#4 P2200 S500\r\n";   // Claw origin (opsional)

                    serialPort.Write(initMessage);
                    serialPort.Write(initMessage2);
                    serialPort.Write(initMessage3);
                    serialPort.Write(originServo3);
                    serialPort.Write(originClaw);
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

        void ArmDraw(double sdt1, double sdt2, double sdt3)
        {
            // Jangan hapus seluruh canvas agar bayangan tidak hilang
            plotCanvas.Children.Clear();
            DrawGridAndAxes();

            // Tambahkan kembali track path ke canvas (agar tetap ada)
            if (!plotCanvas.Children.Contains(trackPath))
                plotCanvas.Children.Add(trackPath);

            // ==== Origin di bawah tengah ====
            originX = plotCanvas.ActualWidth / 2;
            originY = plotCanvas.ActualHeight - 10;

            // ==== Skala otomatis supaya lengan pas ====
            double totalLength = a1 + a2 + a3;
            scale = (plotCanvas.ActualHeight - 20) / totalLength;

            // ==== Ubah derajat ke radian ====
            double r1 = sdt1 * Math.PI / 180.0;
            double r2 = sdt2 * Math.PI / 180.0;
            double r3 = sdt3 * Math.PI / 180.0;

            // ==== Hitung posisi tiap joint ====
            double kx = a1 * Math.Cos(r1);
            double ky = a1 * Math.Sin(r1);

            double px = kx + a2 * Math.Cos(r1 + r2);
            double py = ky + a2 * Math.Sin(r1 + r2);

            double qx = px + a3 * Math.Cos(r1 + r2 + r3);
            double qy = py + a3 * Math.Sin(r1 + r2 + r3);

            // ==== Tambahkan titik lintasan end-effector ====
            Point effectorPoint = new Point(ToCanvasX(qx), ToCanvasY(qy));
            trackPath.Points.Add(effectorPoint);

            // ==== Gambar setiap link ====
            DrawLink(ToCanvasX(0), ToCanvasY(0), ToCanvasX(kx), ToCanvasY(ky), Brushes.Black);
            DrawLink(ToCanvasX(kx), ToCanvasY(ky), ToCanvasX(px), ToCanvasY(py), Brushes.DarkBlue);
            DrawLink(ToCanvasX(px), ToCanvasY(py), ToCanvasX(qx), ToCanvasY(qy), Brushes.Red);

            // ==== Gambar titik end-effector (bulatan kecil) ====
            Ellipse ee = new Ellipse
            {
                Width = 6,
                Height = 6,
                Fill = Brushes.Red
            };
            Canvas.SetLeft(ee, effectorPoint.X - 3);
            Canvas.SetTop(ee, effectorPoint.Y - 3);
            plotCanvas.Children.Add(ee);
        }

        void DrawLink(double x1, double y1, double x2, double y2, Brush color)
        {
            Line line = new Line
            {
                X1 = x1,
                Y1 = y1,
                X2 = x2,
                Y2 = y2,
                Stroke = color,
                StrokeThickness = 2
            };
            plotCanvas.Children.Add(line);
        }

        private void PlotCanvas_Loaded(object sender, RoutedEventArgs e)
        {
            DrawGridAndAxes();
        }

        void DrawGridAndAxes()
        {
            plotCanvas.Children.Clear();

            double width = plotCanvas.ActualWidth;
            double height = plotCanvas.ActualHeight;

            // ======================
            // 1. SETUP SKALA DAN ORIGIN YANG KONSISTEN
            // ======================

            // Origin di bawah tengah (seperti lengan robot)
            originX = width / 2;
            originY = height - 10;

            // Skala yang sama dengan perhitungan lengan
            double totalLength = a1 + a2 + a3;
            scale = (plotCanvas.ActualHeight - 20) / totalLength;

            // Grid spacing dalam mm dunia nyata
            double worldGridSpacing = 50; // 50mm
            double canvasGridSpacing = worldGridSpacing * scale;

            // Minor grid spacing
            double minorWorldSpacing = 10; // 10mm
            double minorCanvasSpacing = minorWorldSpacing * scale;

            // ======================
            // 2. GRID VERTIKAL (Garis X)
            // ======================

            // Grid ke kanan dari origin
            for (double worldX = 0; worldX <= width; worldX += worldGridSpacing)
            {
                double canvasX = originX + worldX * scale;
                if (canvasX > width) break;

                // Garis grid utama
                plotCanvas.Children.Add(new Line
                {
                    X1 = canvasX,
                    Y1 = 0,
                    X2 = canvasX,
                    Y2 = originY,
                    Stroke = Brushes.LightGray,
                    StrokeThickness = 0.4
                });

                // Label sumbu X positif
                TextBlock label = new TextBlock
                {
                    Text = worldX.ToString("0"),
                    FontSize = 10,
                    Foreground = Brushes.Gray,
                    Background = Brushes.White
                };
                Canvas.SetLeft(label, canvasX - 8);
                Canvas.SetTop(label, originY + 2);
                plotCanvas.Children.Add(label);

                // Grid minor
                for (double minorX = worldX + minorWorldSpacing;
                     minorX < worldX + worldGridSpacing && (originX + minorX * scale) < width;
                     minorX += minorWorldSpacing)
                {
                    double minorCanvasX = originX + minorX * scale;
                    plotCanvas.Children.Add(new Line
                    {
                        X1 = minorCanvasX,
                        Y1 = 0,
                        X2 = minorCanvasX,
                        Y2 = originY,
                        Stroke = Brushes.LightGray,
                        StrokeThickness = 0.2,
                        StrokeDashArray = new DoubleCollection() { 2, 3 }
                    });
                }
            }

            // Grid ke kiri dari origin
            for (double worldX = 0; worldX >= -width; worldX -= worldGridSpacing)
            {
                double canvasX = originX + worldX * scale;
                if (canvasX < 0) break;

                if (worldX != 0) // Hindari garis di origin (sumbu Y)
                {
                    plotCanvas.Children.Add(new Line
                    {
                        X1 = canvasX,
                        Y1 = 0,
                        X2 = canvasX,
                        Y2 = originY,
                        Stroke = Brushes.LightGray,
                        StrokeThickness = 0.4
                    });

                    // Label sumbu X negatif
                    TextBlock label = new TextBlock
                    {
                        Text = worldX.ToString("0"),
                        FontSize = 10,
                        Foreground = Brushes.Gray,
                        Background = Brushes.White
                    };
                    Canvas.SetLeft(label, canvasX - 12);
                    Canvas.SetTop(label, originY + 2);
                    plotCanvas.Children.Add(label);
                }

                // Grid minor
                for (double minorX = worldX - minorWorldSpacing;
                     minorX > worldX - worldGridSpacing && (originX + minorX * scale) > 0;
                     minorX -= minorWorldSpacing)
                {
                    double minorCanvasX = originX + minorX * scale;
                    plotCanvas.Children.Add(new Line
                    {
                        X1 = minorCanvasX,
                        Y1 = 0,
                        X2 = minorCanvasX,
                        Y2 = originY,
                        Stroke = Brushes.LightGray,
                        StrokeThickness = 0.2,
                        StrokeDashArray = new DoubleCollection() { 2, 3 }
                    });
                }
            }

            // ======================
            // 3. GRID HORIZONTAL (Garis Y)
            // ======================

            // Grid ke atas dari origin
            for (double worldY = 0; worldY <= totalLength; worldY += worldGridSpacing)
            {
                double canvasY = originY - worldY * scale;
                if (canvasY < 0) break;

                if (worldY != 0) // Hindari garis di origin (sumbu X)
                {
                    plotCanvas.Children.Add(new Line
                    {
                        X1 = 0,
                        Y1 = canvasY,
                        X2 = width,
                        Y2 = canvasY,
                        Stroke = Brushes.LightGray,
                        StrokeThickness = 0.4
                    });

                    // Label sumbu Y positif
                    TextBlock label = new TextBlock
                    {
                        Text = worldY.ToString("0"),
                        FontSize = 10,
                        Foreground = Brushes.Gray,
                        Background = Brushes.White
                    };
                    Canvas.SetLeft(label, originX + 4);
                    Canvas.SetTop(label, canvasY - 8);
                    plotCanvas.Children.Add(label);
                }

                // Grid minor
                for (double minorY = worldY + minorWorldSpacing;
                     minorY < worldY + worldGridSpacing && (originY - minorY * scale) > 0;
                     minorY += minorWorldSpacing)
                {
                    double minorCanvasY = originY - minorY * scale;
                    plotCanvas.Children.Add(new Line
                    {
                        X1 = 0,
                        Y1 = minorCanvasY,
                        X2 = width,
                        Y2 = minorCanvasY,
                        Stroke = Brushes.LightGray,
                        StrokeThickness = 0.2,
                        StrokeDashArray = new DoubleCollection() { 2, 3 }
                    });
                }
            }

            // ======================
            // 4. SUMBU KOORDINAT UTAMA
            // ======================

            // Sumbu X (garis horizontal di y=0)
            plotCanvas.Children.Add(new Line
            {
                X1 = 0,
                Y1 = originY,
                X2 = width,
                Y2 = originY,
                Stroke = Brushes.Gray,
                StrokeThickness = 1.5
            });

            // Sumbu Y (garis vertikal di x=0)
            plotCanvas.Children.Add(new Line
            {
                X1 = originX,
                Y1 = 0,
                X2 = originX,
                Y2 = originY,
                Stroke = Brushes.Gray,
                StrokeThickness = 1.5
            });

            // ======================
            // 5. LABEL SUMBU UTAMA
            // ======================

            TextBlock labelX = new TextBlock
            {
                Text = "X (mm)",
                FontSize = 12,
                FontWeight = FontWeights.Bold,
                Foreground = Brushes.Black,
                Background = Brushes.White
            };
            Canvas.SetLeft(labelX, width - 50);
            Canvas.SetTop(labelX, originY - 25);
            plotCanvas.Children.Add(labelX);

            TextBlock labelY = new TextBlock
            {
                Text = "Y (mm)",
                FontSize = 12,
                FontWeight = FontWeights.Bold,
                Foreground = Brushes.Black,
                Background = Brushes.White
            };
            Canvas.SetLeft(labelY, originX + 10);
            Canvas.SetTop(labelY, 5);
            plotCanvas.Children.Add(labelY);

            // Label Origin
            TextBlock labelOrigin = new TextBlock
            {
                Text = "(0,0)",
                FontSize = 10,
                Foreground = Brushes.DarkBlue,
                FontWeight = FontWeights.Bold,
                Background = Brushes.White
            };
            Canvas.SetLeft(labelOrigin, originX + 5);
            Canvas.SetTop(labelOrigin, originY - 18);
            plotCanvas.Children.Add(labelOrigin);
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
            ArmDraw(teta1, teta2, teta3);

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
                ArmDraw(theta1[currentStep], theta2[currentStep], theta3[currentStep]);

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
            plotCanvas.Children.Clear();
            trackPath.Points.Clear();
            DrawGridAndAxes();
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