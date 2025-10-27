using jakaApi;
using jkType;
using System;
using System.Net;
using System.Runtime.InteropServices;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;   // DragCompletedEventArgs
using System.Windows.Input;
using System.Windows.Media;                // VisualTreeHelper
using System.Windows.Media.Media3D;

namespace WPF_robot_sim
{
    public partial class MainWindow : Window
    {
        // =================== Config ===================
        private const bool SEND_IN_RADIANS = true;   // đổi false nếu SDK nhận độ
        private const int SERVO_BURST_MS = 240;      // thời gian bơm lệnh servo sau mỗi thao tác (≈8ms*30)
        private const int SERVO_LOOP_PERIOD_MS = 8;  // chu kỳ controller ~8ms theo tài liệu
        // Tham số MMF (bạn có thể chỉnh theo nhu cầu smoothing/độ bám):
        private const int MMF_MAX_BUF = 20;         // cửa sổ trung bình
        private const double MMF_KP = 0.2;           // acc factor (tài liệu mô tả kp/kv/ka)
        private const double MMF_KV = 0.4;           // vel factor
        private const double MMF_KA = 0.2;           // pos factor

        // =================== State ===================
        private int handle = 0;
        private bool isConnected = false;

        private CancellationTokenSource _servoCts;   // hủy vòng lặp bơm servo khi có thao tác mới


        // ==== Cartesian config (TCP) ====
        private const bool CART_POS_IN_METERS = true;  // SDK thường x,y,z là mét
        private const bool CART_ANG_IN_RADIANS = true;  // SDK thường rx,ry,rz là rad
        private const int CART_SERVO_BURST_MS = 240;   // thời gian bơm servo_p, giống khớp

        private double MmToMeters(double mm) => mm / 1000.0;
        private double DegToRad(double deg) => deg * Math.PI / 180.0;

        // ====== IP input helpers ======
        private static readonly Regex _ipAllowed = new Regex(@"^[0-9\.]+$"); // only digits and dot


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

            // Power on & enable robot
            ret = await Task.Run(() => jakaAPI.power_on(ref handle));
            if (ret != 0) { FailConnect($"power_on failed (ret={ret})"); return; }

            await Task.Delay(1200); // chờ nguồn ổn định

            ret = await Task.Run(() => jakaAPI.enable_robot(ref handle));
            if (ret != 0) { FailConnect($"enable_robot failed (ret={ret})"); return; }

            // === SERVO FILTER (MMF) --> MUST set BEFORE enabling servo mode ===
            // Theo docs: filter cấu hình khi đang KHÔNG ở servo mode
            ret = await Task.Run(() => jakaAPI.servo_move_use_joint_MMF(ref handle, MMF_MAX_BUF, MMF_KP, MMF_KV, MMF_KA));
            if (ret != 0) { FailConnect($"servo_move_use_joint_MMF failed (ret={ret})"); return; }

            // Enable SERVO mode
            await Task.Run(() => jakaAPI.servo_move_enable(ref handle, false));

            // thử MMF trước
            ret = await Task.Run(() => jakaAPI.servo_move_use_joint_MMF(ref handle, MMF_MAX_BUF, MMF_KP, MMF_KV, MMF_KA));
            if (ret != 0)
            {
                Log($"MMF not available (ret={ret}). Try LPF...");
                // Fallback: LPF (chọn cutoff nhẹ, ví dụ 10)
                // Nếu wrapper của bạn có API này, dùng; nếu không, bỏ qua filter.
                // ví dụ:
                // ret = await Task.Run(() => jakaAPI.servo_move_use_joint_LPF(ref handle, 10));
                // if (ret != 0) Log($"LPF also failed (ret={ret}). Continue without joint filter.");
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

            // Tạm thoát SERVO mode, dùng joint_move về home cho êm
            await Task.Run(() => jakaAPI.servo_move_enable(ref handle, false));

            var home = new JKTYPE.JointValue { jVal = new double[6] { 0, 0, 0, 0, 0, 0 } };
            var homeSend = new JKTYPE.JointValue { jVal = new double[6] };

            for (int i = 0; i < 6; i++)
                homeSend.jVal[i] = SEND_IN_RADIANS ? Deg2Rad(home.jVal[i]) : home.jVal[i];

            int speed = 20; // bạn có thể map từ slider speed nếu có
            Log("Executing Auto Home (ABS) ...");
            UpdateUiState(isBusy: true);

            int ret = await Task.Run(() => jakaAPI.joint_move(ref handle, ref homeSend, JKTYPE.MoveMode.ABS, true, speed));
            Log(ret == 0 ? "Auto Home completed successfully." : $"Auto Home failed (ret={ret}).");

            // Enable lại SERVO mode sau khi về home
            await Task.Run(() => jakaAPI.servo_move_enable(ref handle, true));

            // Đồng bộ UI
            slJ1.Value = slJ2.Value = slJ3.Value = slJ4.Value = slJ5.Value = slJ6.Value = 0;
            UpdateUiState(connected: isConnected);
        }

        // ========== JOINTS: Auto-execute on release ==========
        private async void JointSlider_DragCompleted(object sender, DragCompletedEventArgs e)
        {
            Slider slider = sender as Slider ?? FindAncestorSlider(e.OriginalSource as DependencyObject);
            if (slider != null) await CommitServoBurstAsync();
        }

        // Enter để commit
        private async void JointSlider_KeyUp(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
                await CommitServoBurstAsync();
        }

        /// <summary>
        /// Gom toàn bộ 6 joint hiện tại, áp clamp, đổi đơn vị (deg->rad nếu cần),
        /// rồi bơm lệnh servo_j liên tục theo chu kỳ 8ms trong thời gian ngắn (SERVO_BURST_MS).
        /// </summary>
        private async Task CommitServoBurstAsync()
        {
            if (!isConnected)
            {
                Log("Cannot send command: Robot not connected.");
                return;
            }
            if (!BuildJointTarget(out var jTarget)) return;

            // Hủy burst cũ nếu còn
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

                    // Một số bản SDK C# có overload step_num. Nếu có, dùng:
                    // int ret = jakaAPI.servo_j(ref handle, ref jTarget, JKTYPE.MoveMode.ABS, 1);
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
                // Bị thay thế bởi một thao tác mới — không log lỗi
            }
            finally
            {
                UpdateUiState(connected: isConnected);
            }
        }

        // ========== Helpers ==========
        private bool BuildJointTarget(out JKTYPE.JointValue j)
        {
            j = new JKTYPE.JointValue { jVal = new double[6] };

            double[] deg = new double[]
            {
                slJ1.Value, slJ2.Value, slJ3.Value, slJ4.Value, slJ5.Value, slJ6.Value
            };

            double[] min = { -360, -50, -155, -85, -360, -360 };
            double[] max = { 360, 230, 155, 265, 360, 360 };

            for (int i = 0; i < 6; i++)
            {
                if (deg[i] < min[i]) deg[i] = min[i];
                if (deg[i] > max[i]) deg[i] = max[i];
                j.jVal[i] = SEND_IN_RADIANS ? Deg2Rad(deg[i]) : deg[i];
            }
            return true;
        }

        private double Deg2Rad(double deg) => deg * Math.PI / 180.0;

        private Slider FindAncestorSlider(DependencyObject obj)
        {
            while (obj != null && obj is not Slider)
                obj = VisualTreeHelper.GetParent(obj);
            return obj as Slider;
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
                

                slJ1.IsEnabled = c && !isBusy;
                slJ2.IsEnabled = c && !isBusy;
                slJ3.IsEnabled = c && !isBusy;
                slJ4.IsEnabled = c && !isBusy;
                slJ5.IsEnabled = c && !isBusy;
                slJ6.IsEnabled = c && !isBusy;
            }
            BtnRefresh.IsEnabled = !isBusy;
        }


        private void Log(string message)
        {
            var ts = DateTime.Now.ToString("HH:mm:ss");
            LogBox.AppendText($"[{ts}] {message}{Environment.NewLine}");
            LogBox.ScrollToEnd();
        }

        // ========== IP input: 3 tính năng ==========
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
            await Task.Run(() => jakaAPI.servo_move_enable(ref handle, false));
            await Task.Run(() => jakaAPI.disable_robot(ref handle));
            await Task.Run(() => jakaAPI.power_off(ref handle));
            await Task.Run(() => jakaAPI.destory_handler(ref handle));

            isConnected = false;
            Log("Robot stopped & disconnected.");
            TxtStatus.Text = "Status: Disconnected";
            UpdateUiState(connected: false);
        }
        private async void BtnMove_Click(object sender, RoutedEventArgs e)
        {
            if (!isConnected) { Log("Robot is not connected."); return; }

            // Lấy target từ sliders
            if (!BuildJointTarget(out var jTarget))
            {
                Log("Failed to read joint target.");
                return;
            }

            // Nếu đang ở SERVO mode, tắt tạm để dùng ABS move êm hơn
            UpdateUiState(isBusy: true);
            await Task.Run(() => jakaAPI.servo_move_enable(ref handle, false));

            int speed = 20; // hoặc lấy từ slider speed của bạn nếu có
            Log($"Moving to ABS target at speed ≈ {speed} ...");

            int ret = await Task.Run(() =>
                jakaAPI.joint_move(ref handle, ref jTarget, JKTYPE.MoveMode.ABS, true, speed));

            if (ret == 0) Log("Move completed.");
            else Log($"Move failed (ret={ret}).");

            // Bật lại SERVO nếu bạn vẫn muốn giữ (hoặc bỏ nếu không dùng servo nữa)
            await Task.Run(() => jakaAPI.servo_move_enable(ref handle, true));

            UpdateUiState(connected: isConnected);
        }

        private void tbZ_TextChanged(object sender, TextChangedEventArgs e)
        {

        }
        private async Task CommitCartesianServoBurstAsync(JKTYPE.MoveMode mode = JKTYPE.MoveMode.ABS)
        {
            if (!isConnected) { Log("Cannot send Cartesian command: Robot not connected."); return; }
            if (!BuildCartesianTarget(out var pose)) return;

            // Huỷ burst cũ nếu có (tận dụng _servoCts đang dùng)
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

                    // Một số bản SDK cần step_num, khi đó dùng overload có tham số cuối là 1.
                    // int ret = jakaAPI.servo_p(ref handle, ref pose, mode, 1);
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

        private bool BuildCartesianTarget(out JKTYPE.CartesianPose pose)
        {
            pose = new JKTYPE.CartesianPose
            {
                tran = new JKTYPE.CartesianTran(),  // x, y, z
                rpy = new JKTYPE.Rpy()     // rx, ry, rz
            };

            // Lấy từ sliders (UI đang ở mm & deg cho dễ dùng)
            double x_mm = slX.Value, y_mm = slY.Value, z_mm = slZ.Value;
            double rx_d = slRX.Value, ry_d = slRY.Value, rz_d = slRZ.Value;

            // (Tuỳ chọn) clamp theo workspace thực tế của robot để an toàn
            // ví dụ:
            // x_mm = Math.Max(-800, Math.Min(800, x_mm));
            // ...

            // Đổi đơn vị đúng theo flag cấu hình
            pose.tran.x = CART_POS_IN_METERS ? MmToMeters(x_mm) : x_mm;
            pose.tran.y = CART_POS_IN_METERS ? MmToMeters(y_mm) : y_mm;
            pose.tran.z = CART_POS_IN_METERS ? MmToMeters(z_mm) : z_mm;

            pose.rpy.rx = CART_ANG_IN_RADIANS ? DegToRad(rx_d) : rx_d;
            pose.rpy.ry = CART_ANG_IN_RADIANS ? DegToRad(ry_d) : ry_d;
            pose.rpy.rz = CART_ANG_IN_RADIANS ? DegToRad(rz_d) : rz_d;

            return true;
        }
        private async void CartesianSlider_DragCompleted(object sender, System.Windows.Controls.Primitives.DragCompletedEventArgs e)
        {
            await CommitCartesianServoBurstAsync(JKTYPE.MoveMode.ABS);
        }

        private async void CartesianSlider_KeyUp(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
                await CommitCartesianServoBurstAsync(JKTYPE.MoveMode.ABS);
        }
        // ====== gọi trong constructor sau InitializeComponent() ======
        

    }
}
