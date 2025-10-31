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

namespace WPF_robot_sim
{
    public partial class MainWindow : Window
    {
        // =================== Config ===================
        private const bool SEND_IN_RADIANS = true;      // đổi false nếu SDK nhận độ
        private const int SERVO_BURST_MS = 240;         // thời gian bơm lệnh servo sau mỗi thao tác
        private const int SERVO_LOOP_PERIOD_MS = 8;     // chu kỳ controller ~8ms theo tài liệu

        // Tham số MMF
        private const int MMF_MAX_BUF = 20;
        private const double MMF_KP = 0.2;
        private const double MMF_KV = 0.4;
        private const double MMF_KA = 0.2;

        // =================== State ===================
        private int handle = 0;
        private bool isConnected = false;
        private CancellationTokenSource _servoCts;

        // ==== Cartesian config (TCP) ====
        private const bool CART_POS_IN_METERS = true;    // SDK thường x,y,z là mét
        private const bool CART_ANG_IN_RADIANS = true;   // SDK thường rx,ry,rz là rad
        private const int CART_SERVO_BURST_MS = 240;

        private double MmToMeters(double mm) => mm / 1000.0;
        private double DegToRad(double deg) => deg * Math.PI / 180.0;

        // ====== IP input helpers ======
        private static readonly Regex _ipAllowed = new Regex(@"^[0-9\.]+$");

        // Dùng invariant để chấp nhận dấu chấm thập phân
        private static readonly CultureInfo CI = CultureInfo.InvariantCulture;

        public MainWindow()
        {
            InitializeComponent();
            UpdateUiState(false);
            Log("Application started.");
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

            // cấu hình filter TRƯỚC khi bật servo mode
            ret = await Task.Run(() => jakaAPI.servo_move_use_joint_MMF(ref handle, MMF_MAX_BUF, MMF_KP, MMF_KV, MMF_KA));
            if (ret != 0) { FailConnect($"servo_move_use_joint_MMF failed (ret={ret})"); return; }

            await Task.Run(() => jakaAPI.servo_move_enable(ref handle, false));

            // thử set lại MMF (một số FW yêu cầu gọi khi không ở servo)
            ret = await Task.Run(() => jakaAPI.servo_move_use_joint_MMF(ref handle, MMF_MAX_BUF, MMF_KP, MMF_KV, MMF_KA));
            if (ret != 0)
            {
                Log($"MMF not available (ret={ret}). You may try LPF if supported.");
                // ví dụ fallback nếu SDK có:
                // ret = await Task.Run(() => jakaAPI.servo_move_use_joint_LPF(ref handle, 10));
            }

            ret = await Task.Run(() => jakaAPI.servo_move_enable(ref handle, true));
            if (ret != 0) { FailConnect($"servo_move_enable(true) failed (ret={ret})"); return; }

            isConnected = true;
            Log($"Connected successfully to {normalized}. SERVO mode enabled (MMF set).");
            TxtStatus.Text = $"Status: Connected ({normalized})";
            UpdateUiState(connected: true);
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

        // ========== AUTO HOME (ABS move) ==========
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
            Log(ret == 0 ? "Auto Home completed successfully." : $"Auto Home failed (ret={ret}).");

            await Task.Run(() => jakaAPI.servo_move_enable(ref handle, true));
            UpdateUiState(connected: isConnected);
        }

        // ========== JOINT SERVO BURST (nếu muốn dùng với nút Move nhanh) ==========
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
            catch (TaskCanceledException)
            {
                // bị thay thế – bỏ qua
            }
            finally
            {
                UpdateUiState(connected: isConnected);
            }
        }

        // ========== MOVE (ABS) bằng nút BtnMove ==========
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
            catch (TaskCanceledException)
            {
                // bị thao tác mới thay thế – không báo lỗi
            }
            finally
            {
                UpdateUiState(connected: isConnected);
            }
        }

        // ========== BUILD TARGETS TỪ TEXTBOX ==========
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

            // clamp theo dải an toàn tham khảo (điều chỉnh theo robot thực)
            double[] min = { -360, -50, -155, -85, -360, -360 };
            double[] max = { 360, 230, 155, 265, 360, 360 };

            for (int i = 0; i < 6; i++)
            {
                if (deg[i] < min[i]) deg[i] = min[i];
                if (deg[i] > max[i]) deg[i] = max[i];
                j.jVal[i] = SEND_IN_RADIANS ? DegToRad(deg[i]) : deg[i];
            }

            return true;
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

            // (tuỳ chọn) clamp workspace ở đây nếu muốn

            pose.tran.x = CART_POS_IN_METERS ? MmToMeters(x_mm) : x_mm;
            pose.tran.y = CART_POS_IN_METERS ? MmToMeters(y_mm) : y_mm;
            pose.tran.z = CART_POS_IN_METERS ? MmToMeters(z_mm) : z_mm;

            pose.rpy.rx = CART_ANG_IN_RADIANS ? DegToRad(rx_d) : rx_d;
            pose.rpy.ry = CART_ANG_IN_RADIANS ? DegToRad(ry_d) : ry_d;
            pose.rpy.rz = CART_ANG_IN_RADIANS ? DegToRad(rz_d) : rz_d;

            return true;
        }

        // Parse helper: chấp nhận cả "1.23" lẫn "1,23" (tự chuyển , -> .)
        private bool ParseBox(TextBox box, out double val)
        {
            string s = (box.Text ?? "").Trim();
            s = s.Replace(',', '.');
            return double.TryParse(s, NumberStyles.Float, CI, out val);
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

                // Enable/disable input boxes
                tbJ1.IsEnabled = tbJ2.IsEnabled = tbJ3.IsEnabled =
                tbJ4.IsEnabled = tbJ5.IsEnabled = tbJ6.IsEnabled = c && !isBusy;

                tbX.IsEnabled = tbY.IsEnabled = tbZ.IsEnabled =
                tbRX.IsEnabled = tbRY.IsEnabled = tbRZ.IsEnabled = c && !isBusy;

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

        // ========== STOP (nếu bạn có nút Stop) ==========
        private async void btnStop_Click(object sender, RoutedEventArgs e)
        {
            if (!isConnected) { Log("Robot is not connected."); return; }

            Log("Stopping and powering off ...");
            UpdateUiState(isBusy: true);

            _servoCts?.Cancel();
            await Task.Run(() => jakaAPI.servo_move_enable(ref handle, false));
            await Task.Run(() => jakaAPI.disable_robot(ref handle));
            await Task.Run(() => jakaAPI.power_off(ref handle));
            await Task.Run(() => jakaAPI.destory_handler(ref handle));

            isConnected = false;
            Log("Robot stopped & disconnected.");
            TxtStatus.Text = "Status: Disconnected";
            UpdateUiState(connected: false);
        }

        // Các handler “slider” dưới đây không còn dùng vì UI hiện tại dùng TextBox.
        // Để tránh lỗi build khi XAML có EventSetter, bạn có thể để trống/không gọi.
        private async void CartesianSlider_DragCompleted(object sender, DragCompletedEventArgs e)
        {
            await CommitCartesianServoBurstAsync(JKTYPE.MoveMode.ABS);
        }
        private async void CartesianSlider_KeyUp(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
                await CommitCartesianServoBurstAsync(JKTYPE.MoveMode.ABS);
        }

        // Nếu bạn cần bám TextChanged của Z (đã có trong code cũ)
        private void tbZ_TextChanged(object sender, TextChangedEventArgs e) { }
    }
}
