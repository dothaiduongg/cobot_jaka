using System;
using System.IO;
using System.Windows;

namespace JakaWpfDemo
{
    public partial class App : Application
    {
        protected override void OnStartup(StartupEventArgs e)
        {
            base.OnStartup(e);

            // Tùy theo layout SDK của bạn:
            // Ví dụ: đặt các DLL ở .\out\shared\Release\ (relative to exe)
            var exeDir = AppContext.BaseDirectory;
            var sdkDir = Path.Combine(exeDir, "out", "shared", "Release");

            var path = Environment.GetEnvironmentVariable("PATH") ?? "";
            if (!path.Contains(sdkDir, StringComparison.OrdinalIgnoreCase) && Directory.Exists(sdkDir))
            {
                Environment.SetEnvironmentVariable("PATH", sdkDir + ";" + path);
            }
        }
    }
}
