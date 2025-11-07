using jakaApi;
using jkType;
using System;
using System.Globalization;
using System.Net;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Input;
using System.Windows.Media;

// sequence + file
using System.Collections.ObjectModel;
using System.IO;
using System.Text.Json;
using Microsoft.Win32;

namespace WPF_robot_sim
{
    public partial class MainWindow : Window
    {
        // =================== Config ===================
        private const bool SEND_IN_RADIANS = true;
        private const int SERVO_BURST_MS = 240;
        private const int SERVO_LOOP_PERIOD_MS = 8;

        // MMF filter
        private const int MMF_MAX_BUF = 20;
        private const double MMF_KP = 0.2;
        private const double MMF_KV = 0.4;
        private const double MMF_KA = 0.2;

        // =================== State ===================
        private int handle = 0;
        private bool isConnected = false;

        private CancellationTokenSource _servoCts;   // for short servo bursts
        private CancellationTokenSource _seqCts;     // for sequence loop
        private bool _isRunningSequence = false;

        // ==== Cartesian config (TCP) ====
        private const bool CART_POS_IN_METERS = true;
        private const bool CART_ANG_IN_RADIANS = true;
        private const int CART_SERVO_BURST_MS = 240;

        private double MmToMeters(double mm) => mm / 1000.0;
        private double DegToRad(double deg) => deg * Math.PI / 180.0;

        // ====== IP input helpers ======
        private static readonly Regex _ipAllowed = new Regex(@"^[0-9\.]+$");
        private static readonly CultureInfo CI = CultureInfo.InvariantCulture;

        // ====== Joint limits (deg) — per spec image ======
        private static readonly double[] JOINT_MIN = { -360, -50, -155, -85, -360, -360 };
        private static readonly double[] JOINT_MAX = { 360, 230, 155, 265, 360, 360 };

        // ====== Sequence model/binding ======
        public class JointPoint
        {
            public string Name { get; set; } = "P";
            public double J1 { get; set; }
            public double J2 { get; set; }
            public double J3 { get; set; }
            public double J4 { get; set; }
            public double J5 { get; set; }
            public double J6 { get; set; }
            public int DelayMs { get; set; } = 1000;
        }
        public ObservableCollection<JointPoint> Points { get; } = new ObservableCollection<JointPoint>();

        public MainWindow()
        {
            InitializeComponent();
            DataContext = this;
            WireJointBoxClampEvents();
            UpdateUiState(false);
            Log("Application started.");
        }

        // hook LostFocus to clamp J1..J6
        private void WireJointBoxClampEvents()
        {
            tbJ1.LostFocus += JointBox_LostFocus;
            tbJ2.LostFocus += JointBox_LostFocus;
            tbJ3.LostFocus += JointBox_LostFocus;
            tbJ4.LostFocus += JointBox_LostFocus;
            tbJ5.LostFocus += JointBox_LostFocus;
            tbJ6.LostFocus += JointBox_LostFocus;
        }

        private void JointBox_LostFocus(object sender, RoutedEventArgs e)
        {
            if (sender is not TextBox tb) return;
            int idx = tb.Name switch
            {
                "tbJ1" => 0,
                "tbJ2" => 1,
                "tbJ3" => 2,
                "tbJ4" => 3,
                "tbJ5" => 4,
                "tbJ6" => 5,
                _ => -1
            };
            if (idx < 0) return;

            if (!ParseBox(tb, out double v)) v = 0.0;
            v = Math.Max(JOINT_MIN[idx], Math.Min(JOINT_MAX[idx], v));
            tb.Text = v.ToString("F2", CI);
        }

        // ========== CONNECTION ==========
        private async void BtnConnect_Click(object sender, RoutedEventArgs e)
        {
            string ipRaw = TxtIp.Text?.Trim();
            string normalized = NormalizeIp(ipRaw);
            if (normalized == null)
            {
                Log("Invalid IP address.");
                return;
            }
            TxtIp.Text = normalized;

            Log($"Connecting to {normalized} ...");
            UpdateUiState(isBusy: true);

            int ret = await Task.Run(() => jakaAPI.create_handler(normalized, ref handle));
            if (ret != 0) { FailConnect($"create_handler failed (ret={ret})"); return; }

            ret = await Task.Run(() => jakaAPI.power_on(ref handle));
            if (ret != 0) { FailConnect($"power_on failed (ret={ret})"); return; }

            await Task.Delay(1200);

            ret = await Task.Run(() => jakaAPI.enable_robot(ref handle));
            if (ret != 0) { FailConnect($"enable_robot failed (ret={ret})"); return; }

            // configure filter BEFORE enabling servo mode
            ret = await Task.Run(() => jakaAPI.servo_move_use_joint_MMF(ref handle, MMF_MAX_BUF, MMF_KP, MMF_KV, MMF_KA));
            if (ret != 0) { FailConnect($"servo_move_use_joint_MMF failed (ret={ret})"); return; }

            await Task.Run(() => jakaAPI.servo_move_enable(ref handle, false));

            // optional re-apply
            ret = await Task.Run(() => jakaAPI.servo_move_use_joint_MMF(ref handle, MMF_MAX_BUF, MMF_KP, MMF_KV, MMF_KA));
            if (ret != 0) Log($"MMF not available (ret={ret}).");

            ret = await Task.Run(() => jakaAPI.servo_move_enable(ref handle, true));
            if (ret != 0) { FailConnect($"servo_move_enable(true) failed (ret={ret})"); return; }

            isConnected = true;
            TxtStatus.Text = $"Status: Connected ({normalized})";

            // <<< NEW: reset Joint & Cartesian boxes to 0.0 >>>
            ResetAllInputBoxesToZero();

            Log("Connected. Inputs reset to 0.0.");
            UpdateUiState(connected: true);
        }

        private void ResetAllInputBoxesToZero()
        {
            string z2 = 0.0.ToString("F1", CI);
            tbJ1.Text = tbJ2.Text = tbJ3.Text = tbJ4.Text = tbJ5.Text = tbJ6.Text = z2;
            tbX.Text = tbY.Text = tbZ.Text = z2;
            tbRX.Text = tbRY.Text = tbRZ.Text = z2;
        }

        private void FailConnect(string detail)
        {
            Log($"Connection failed: {detail}");
            TxtStatus.Text = "Status: Disconnected";
            isConnected = false;
            UpdateUiState(connected: false);
        }

        private async void BtnDisconnect_Click(object sender, RoutedEventArgs e)
        {
            Log("Disconnecting...");
            UpdateUiState(isBusy: true);

            _servoCts?.Cancel();
            _seqCts?.Cancel();

            if (isConnected)
            {
                await Task.Run(() => jakaAPI.servo_move_enable(ref handle, false));
                await Task.Run(() => jakaAPI.disable_robot(ref handle));
                await Task.Run(() => jakaAPI.power_off(ref handle));
            }

            await Task.Run(() => jakaAPI.destory_handler(ref handle));
            isConnected = false;

            Log("Disconnected.");
            TxtStatus.Text = "Status: Disconnected";
            UpdateUiState(connected: false);
        }

        private void BtnRefresh_Click(object sender, RoutedEventArgs e)
        {
            Log("Refreshing network (demo).");
        }

        // ========== AUTO HOME ==========
        private async void btnAutoHome_Click(object sender, RoutedEventArgs e)
        {
            if (!isConnected) { Log("Robot is not connected."); return; }

            await Task.Run(() => jakaAPI.servo_move_enable(ref handle, false));

            var home = new JKTYPE.JointValue { jVal = new double[6] { 0, 0, 0, 0, 0, 0 } };
            var homeSend = new JKTYPE.JointValue { jVal = new double[6] };
            for (int i = 0; i < 6; i++)
                homeSend.jVal[i] = SEND_IN_RADIANS ? DegToRad(home.jVal[i]) : home.jVal[i];

            int speed = 20;
            Log("Executing Auto Home (ABS) ...");
            UpdateUiState(isBusy: true);

            int ret = await Task.Run(() => jakaAPI.joint_move(ref handle, ref homeSend, JKTYPE.MoveMode.ABS, true, speed));
            Log(ret == 0 ? "Auto Home completed." : $"Auto Home failed (ret={ret}).");

            await Task.Run(() => jakaAPI.servo_move_enable(ref handle, true));
            UpdateUiState(connected: isConnected);

            // <<< NEW: after homing, reflect zeros to Joint Position >>>
            ResetJointBoxesToZero();
        }

        private void ResetJointBoxesToZero()
        {
            string z2 = 0.0.ToString("F2", CI);
            tbJ1.Text = tbJ2.Text = tbJ3.Text = tbJ4.Text = tbJ5.Text = tbJ6.Text = z2;
        }

        // ========== JOINT SERVO BURST (optional) ==========
        private async Task CommitServoBurstAsync()
        {
            if (!isConnected)
            {
                Log("Cannot send command: Robot not connected.");
                return;
            }
            if (!BuildJointTarget(out var jTarget)) return;

            _servoCts?.Cancel();
            _servoCts = new CancellationTokenSource();
            var ct = _servoCts.Token;

            Log("Sending servo_j burst (ABS) ...");
            UpdateUiState(isBusy: true);

            try
            {
                int loops = Math.Max(1, SERVO_BURST_MS / SERVO_LOOP_PERIOD_MS);
                for (int i = 0; i < loops; i++)
                {
                    ct.ThrowIfCancellationRequested();
                    int ret = jakaAPI.servo_j(ref handle, ref jTarget, JKTYPE.MoveMode.ABS);
                    if (ret != 0)
                    {
                        Log($"servo_j failed (ret={ret}).");
                        break;
                    }
                    await Task.Delay(SERVO_LOOP_PERIOD_MS, ct);
                }
                Log("Joint movement completed (servo burst).");
            }
            catch (TaskCanceledException) { }
            finally
            {
                UpdateUiState(connected: isConnected);
            }
        }

        // ========== MOVE (ABS) ==========
        private async void BtnMove_Click(object sender, RoutedEventArgs e)
        {
            if (!isConnected) { Log("Robot is not connected."); return; }

            if (!BuildJointTarget(out var jTarget))
            {
                Log("Failed to read joint target.");
                return;
            }

            UpdateUiState(isBusy: true);
            await Task.Run(() => jakaAPI.servo_move_enable(ref handle, false));

            int speed = 20;
            Log($"Moving to ABS target at speed ≈ {speed} ...");

            int ret = await Task.Run(() =>
                jakaAPI.joint_move(ref handle, ref jTarget, JKTYPE.MoveMode.ABS, true, speed));

            if (ret == 0) Log("Move completed.");
            else Log($"Move failed (ret={ret}).");

            await Task.Run(() => jakaAPI.servo_move_enable(ref handle, true));
            UpdateUiState(connected: isConnected);
        }

        // ========== CARTESIAN SERVO BURST ==========
        private async Task CommitCartesianServoBurstAsync(JKTYPE.MoveMode mode = JKTYPE.MoveMode.ABS)
        {
            if (!isConnected) { Log("Cannot send Cartesian command: Robot not connected."); return; }
            if (!BuildCartesianTarget(out var pose)) return;

            _servoCts?.Cancel();
            _servoCts = new CancellationTokenSource();
            var ct = _servoCts.Token;

            Log($"Sending servo_p burst ({mode}) ...");
            UpdateUiState(isBusy: true);

            try
            {
                int loops = Math.Max(1, CART_SERVO_BURST_MS / SERVO_LOOP_PERIOD_MS);
                for (int i = 0; i < loops; i++)
                {
                    ct.ThrowIfCancellationRequested();
                    int ret = jakaAPI.servo_p(ref handle, ref pose, mode);
                    if (ret != 0)
                    {
                        Log($"servo_p failed (ret={ret}).");
                        break;
                    }
                    await Task.Delay(SERVO_LOOP_PERIOD_MS, ct);
                }
                Log("Cartesian movement completed (servo burst).");
            }
            catch (TaskCanceledException) { }
            finally
            {
                UpdateUiState(connected: isConnected);
            }
        }

        // ========== BUILD TARGETS from TextBoxes ==========
        private bool BuildJointTarget(out JKTYPE.JointValue j)
        {
            j = new JKTYPE.JointValue { jVal = new double[6] };

            if (!ParseBox(tbJ1, out double j1) ||
                !ParseBox(tbJ2, out double j2) ||
                !ParseBox(tbJ3, out double j3) ||
                !ParseBox(tbJ4, out double j4) ||
                !ParseBox(tbJ5, out double j5) ||
                !ParseBox(tbJ6, out double j6))
            {
                Log("Invalid joint values. Please check J1..J6.");
                return false;
            }

            double[] deg = new[] { j1, j2, j3, j4, j5, j6 };

            // clamp + write back to UI
            for (int i = 0; i < 6; i++)
            {
                deg[i] = Math.Max(JOINT_MIN[i], Math.Min(JOINT_MAX[i], deg[i]));
            }
            UpdateJointBoxesFromArray(deg);

            for (int i = 0; i < 6; i++)
                j.jVal[i] = SEND_IN_RADIANS ? DegToRad(deg[i]) : deg[i];

            return true;
        }

        private void UpdateJointBoxesFromArray(double[] deg)
        {
            tbJ1.Text = deg[0].ToString("F2", CI);
            tbJ2.Text = deg[1].ToString("F2", CI);
            tbJ3.Text = deg[2].ToString("F2", CI);
            tbJ4.Text = deg[3].ToString("F2", CI);
            tbJ5.Text = deg[4].ToString("F2", CI);
            tbJ6.Text = deg[5].ToString("F2", CI);
        }

        private bool BuildCartesianTarget(out JKTYPE.CartesianPose pose)
        {
            pose = new JKTYPE.CartesianPose
            {
                tran = new JKTYPE.CartesianTran(),
                rpy = new JKTYPE.Rpy()
            };

            if (!ParseBox(tbX, out double x_mm) ||
                !ParseBox(tbY, out double y_mm) ||
                !ParseBox(tbZ, out double z_mm) ||
                !ParseBox(tbRX, out double rx_d) ||
                !ParseBox(tbRY, out double ry_d) ||
                !ParseBox(tbRZ, out double rz_d))
            {
                Log("Invalid Cartesian values. Please check X/Y/Z and RX/RY/RZ.");
                return false;
            }

            pose.tran.x = CART_POS_IN_METERS ? MmToMeters(x_mm) : x_mm;
            pose.tran.y = CART_POS_IN_METERS ? MmToMeters(y_mm) : y_mm;
            pose.tran.z = CART_POS_IN_METERS ? MmToMeters(z_mm) : z_mm;

            pose.rpy.rx = CART_ANG_IN_RADIANS ? DegToRad(rx_d) : rx_d;
            pose.rpy.ry = CART_ANG_IN_RADIANS ? DegToRad(ry_d) : ry_d;
            pose.rpy.rz = CART_ANG_IN_RADIANS ? DegToRad(rz_d) : rz_d;

            return true;
        }

        // parse double with '.' or ','
        private bool ParseBox(TextBox box, out double val)
        {
            string s = (box.Text ?? "").Trim();
            s = s.Replace(',', '.');
            return double.TryParse(s, NumberStyles.Float, CI, out val);
        }

        // ========== IP input ==========
        private void TxtIp_PreviewTextInput(object sender, TextCompositionEventArgs e)
        {
            e.Handled = !_ipAllowed.IsMatch(e.Text);
        }

        private void TxtIp_Pasting(object sender, DataObjectPastingEventArgs e)
        {
            if (e.DataObject.GetDataPresent(DataFormats.Text))
            {
                var text = (string)e.DataObject.GetData(DataFormats.Text);
                if (!_ipAllowed.IsMatch(text)) e.CancelCommand();
            }
            else e.CancelCommand();
        }

        private void TxtIp_LostFocus(object sender, RoutedEventArgs e)
        {
            string normalized = NormalizeIp(TxtIp.Text);
            if (normalized == null) { Log("Invalid IP address."); return; }
            TxtIp.Text = normalized;
        }

        private string NormalizeIp(string input)
        {
            if (string.IsNullOrWhiteSpace(input)) return null;
            var raw = (input ?? "").Split('.', StringSplitOptions.None);
            var parts = new int[4];

            for (int i = 0; i < 4; i++)
            {
                string cell = i < raw.Length ? raw[i] : "0";
                if (!int.TryParse(string.IsNullOrWhiteSpace(cell) ? "0" : cell, out int v)) return null;
                if (v < 0) v = 0; if (v > 255) v = 255;
                parts[i] = v;
            }

            string candidate = $"{parts[0]}.{parts[1]}.{parts[2]}.{parts[3]}";
            return IPAddress.TryParse(candidate, out _) ? candidate : null;
        }

        // ========== STOP ==========
        private async void btnStop_Click(object sender, RoutedEventArgs e)
        {
            if (!isConnected) { Log("Robot is not connected."); return; }

            Log("Stopping and powering off ...");
            UpdateUiState(isBusy: true);

            _servoCts?.Cancel();
            _seqCts?.Cancel();

            await Task.Run(() => jakaAPI.servo_move_enable(ref handle, false));
            await Task.Run(() => jakaAPI.disable_robot(ref handle));
            await Task.Run(() => jakaAPI.power_off(ref handle));
            await Task.Run(() => jakaAPI.destory_handler(ref handle));

            isConnected = false;
            Log("Robot stopped & disconnected.");
            TxtStatus.Text = "Status: Disconnected";
            UpdateUiState(connected: false);
        }

        // ========== Cartesian slider handlers (kept if referenced in XAML) ==========
        private async void CartesianSlider_DragCompleted(object sender, DragCompletedEventArgs e)
        {
            await CommitCartesianServoBurstAsync(JKTYPE.MoveMode.ABS);
        }
        private async void CartesianSlider_KeyUp(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
                await CommitCartesianServoBurstAsync(JKTYPE.MoveMode.ABS);
        }
        private void tbZ_TextChanged(object sender, TextChangedEventArgs e) { }

        // ========== Sequence helpers ==========
        private bool TryReadCurrentJointBoxes(out double[] deg)
        {
            deg = new double[6];
            bool ok =
                ParseBox(tbJ1, out deg[0]) &&
                ParseBox(tbJ2, out deg[1]) &&
                ParseBox(tbJ3, out deg[2]) &&
                ParseBox(tbJ4, out deg[3]) &&
                ParseBox(tbJ5, out deg[4]) &&
                ParseBox(tbJ6, out deg[5]);

            if (!ok) Log("Invalid joint values in Joint Position.");

            // clamp & write back
            for (int i = 0; i < 6; i++)
                deg[i] = Math.Max(JOINT_MIN[i], Math.Min(JOINT_MAX[i], deg[i]));
            UpdateJointBoxesFromArray(deg);

            return ok;
        }

        private bool TryReadDelay(out int delayMs)
        {
            delayMs = 1000;
            var s = (tbDelay.Text ?? "").Trim().Replace(',', '.');
            if (int.TryParse(s, out int v) && v >= 0) { delayMs = v; return true; }
            Log("Invalid delay. Using 1000 ms.");
            return false;
        }

        private bool TryReadLoops(out int loops)
        {
            loops = 0; // 0 = infinite
            var s = (tbLoops.Text ?? "").Trim().Replace(',', '.');
            if (int.TryParse(s, out int v) && v >= 0) { loops = v; return true; }
            Log("Invalid loops. Using 0 (infinite).");
            return false;
        }

        // ========== Sequence UI handlers ==========
        private void btnAddFromCurrent_Click(object sender, RoutedEventArgs e)
        {
            if (!TryReadCurrentJointBoxes(out var deg)) return;
            TryReadDelay(out int dly);

            var p = new JointPoint
            {
                Name = $"P{Points.Count + 1}",
                J1 = deg[0],
                J2 = deg[1],
                J3 = deg[2],
                J4 = deg[3],
                J5 = deg[4],
                J6 = deg[5],
                DelayMs = dly
            };
            Points.Add(p);
        }

        private void btnUpdateFromCurrent_Click(object sender, RoutedEventArgs e)
        {
            if (dgPoints.SelectedItem is not JointPoint sel) { Log("Select a point to update."); return; }
            if (!TryReadCurrentJointBoxes(out var deg)) return;
            TryReadDelay(out int dly);

            sel.J1 = deg[0]; sel.J2 = deg[1]; sel.J3 = deg[2];
            sel.J4 = deg[3]; sel.J5 = deg[4]; sel.J6 = deg[5];
            sel.DelayMs = dly;

            dgPoints.Items.Refresh();
        }

        private void btnUp_Click(object sender, RoutedEventArgs e)
        {
            if (dgPoints.SelectedItem is not JointPoint sel) return;
            int i = Points.IndexOf(sel);
            if (i > 0) { Points.Move(i, i - 1); dgPoints.SelectedIndex = i - 1; }
        }

        private void btnDown_Click(object sender, RoutedEventArgs e)
        {
            if (dgPoints.SelectedItem is not JointPoint sel) return;
            int i = Points.IndexOf(sel);
            if (i >= 0 && i < Points.Count - 1) { Points.Move(i, i + 1); dgPoints.SelectedIndex = i + 1; }
        }

        private void btnRemove_Click(object sender, RoutedEventArgs e)
        {
            var sel = dgPoints.SelectedItems;
            if (sel.Count == 0) return;

            var toRemove = new JointPoint[sel.Count];
            sel.CopyTo(toRemove, 0);
            foreach (var p in toRemove) Points.Remove(p);

            dgPoints.Items.Refresh();
        }

        private async void btnRun_Click(object sender, RoutedEventArgs e)
        {
            if (!isConnected) { Log("Robot is not connected."); return; }
            if (Points.Count == 0) { Log("Point list is empty."); return; }
            if (_isRunningSequence) { Log("Sequence is already running."); return; }

            TryReadLoops(out int loops);

            _seqCts = new CancellationTokenSource();
            _isRunningSequence = true;
            btnRun.IsEnabled = false;
            btnStopRun.IsEnabled = true;

            try
            {
                await RunSequenceAsync(loops, _seqCts.Token);
            }
            catch (TaskCanceledException)
            {
                Log("Sequence stopped.");
            }
            finally
            {
                _isRunningSequence = false;
                btnRun.IsEnabled = true;
                btnStopRun.IsEnabled = false;
            }
        }

        private void btnStopRun_Click(object sender, RoutedEventArgs e)
        {
            _seqCts?.Cancel();
        }

        private async Task RunSequenceAsync(int loops, CancellationToken ct)
        {
            Log(loops <= 0 ? "Starting sequence (infinite)..." : $"Starting sequence ({loops} loop(s))...");
            int loopCount = 0;

            while (loops <= 0 || loopCount < loops)
            {
                loopCount++;

                foreach (var p in Points)
                {
                    ct.ThrowIfCancellationRequested();

                    // build target
                    var jTarget = new JKTYPE.JointValue { jVal = new double[6] };
                    double[] deg = { p.J1, p.J2, p.J3, p.J4, p.J5, p.J6 };
                    for (int i = 0; i < 6; i++)
                    {
                        deg[i] = Math.Max(JOINT_MIN[i], Math.Min(JOINT_MAX[i], deg[i]));
                        jTarget.jVal[i] = SEND_IN_RADIANS ? DegToRad(deg[i]) : deg[i];
                    }

                    // execute
                    UpdateUiState(isBusy: true);
                    await Task.Run(() => jakaAPI.servo_move_enable(ref handle, false));
                    int speed = 20;

                    Log($"Move → {p.Name}  [{deg[0]:0.##}, {deg[1]:0.##}, {deg[2]:0.##}, {deg[3]:0.##}, {deg[4]:0.##}, {deg[5]:0.##}]  delay={p.DelayMs}ms");
                    int ret = await Task.Run(() =>
                        jakaAPI.joint_move(ref handle, ref jTarget, JKTYPE.MoveMode.ABS, true, speed));

                    await Task.Run(() => jakaAPI.servo_move_enable(ref handle, true));
                    UpdateUiState(connected: isConnected);

                    if (ret != 0)
                    {
                        Log($"Move failed at {p.Name} (ret={ret}). Stop.");
                        return;
                    }

                    if (p.DelayMs > 0)
                        await Task.Delay(p.DelayMs, ct);
                }
            }

            Log("Sequence finished.");
        }

        // ========== Save/Load ==========
        private void btnSaveConfig_Click(object sender, RoutedEventArgs e)
        {
            var sfd = new SaveFileDialog
            {
                Filter = "Seq Config (*.json)|*.json",
                FileName = "joints_seq.json"
            };
            if (sfd.ShowDialog() == true)
            {
                var opts = new JsonSerializerOptions { WriteIndented = true };
                File.WriteAllText(sfd.FileName, JsonSerializer.Serialize(Points, opts));
                Log($"Saved config: {sfd.FileName}");
            }
        }

        private void btnLoadConfig_Click(object sender, RoutedEventArgs e)
        {
            var ofd = new OpenFileDialog
            {
                Filter = "Seq Config (*.json)|*.json"
            };
            if (ofd.ShowDialog() == true)
            {
                try
                {
                    var json = File.ReadAllText(ofd.FileName);
                    var list = JsonSerializer.Deserialize<ObservableCollection<JointPoint>>(json) ?? new ObservableCollection<JointPoint>();
                    Points.Clear();
                    foreach (var p in list) Points.Add(p);
                    dgPoints.Items.Refresh();
                    Log($"Loaded config: {ofd.FileName}");
                }
                catch (Exception ex)
                {
                    Log($"Load config failed: {ex.Message}");
                }
            }
        }

        // ========== UI & LOG ==========
        private void UpdateUiState(bool? connected = null, bool isBusy = false)
        {
            if (connected.HasValue)
            {
                bool c = connected.Value;
                BtnConnect.IsEnabled = !c && !isBusy;
                BtnDisconnect.IsEnabled = c && !isBusy;
                btnAutoHome.IsEnabled = c && !isBusy;
                BtnMove.IsEnabled = c && !isBusy;

                tbJ1.IsEnabled = tbJ2.IsEnabled = tbJ3.IsEnabled =
                tbJ4.IsEnabled = tbJ5.IsEnabled = tbJ6.IsEnabled = c && !isBusy;

                tbX.IsEnabled = tbY.IsEnabled = tbZ.IsEnabled =
                tbRX.IsEnabled = tbRY.IsEnabled = tbRZ.IsEnabled = c && !isBusy;

                tbDelay.IsEnabled = tbLoops.IsEnabled = c && !isBusy;
                dgPoints.IsEnabled = c && !isBusy;

                TxtIp.IsEnabled = !c && !isBusy;
            }
            BtnRefresh.IsEnabled = !isBusy;
        }

        private void Log(string message)
        {
            var ts = DateTime.Now.ToString("HH:mm:ss");
            LogBox.AppendText($"[{ts}] {message}{Environment.NewLine}");
            LogBox.ScrollToEnd();
        }
    }
}
