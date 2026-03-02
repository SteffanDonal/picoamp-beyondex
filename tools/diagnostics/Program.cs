using System.ComponentModel;
using System.Diagnostics;
using System.Reflection.Metadata;
using System.Runtime.InteropServices;
using HidSharp;

// Must match `beyondex_diag.h`
[StructLayout(LayoutKind.Sequential, Pack = 1, Size = 24)]
struct AudioDiag
{
  public uint Magic;
  public uint Underruns;
  public uint Overflows;
  public int BufferLength;
  public int BufferTime;
  public int Feedback;
  public uint NumSofCalls;
  public uint NumSofUpdates;
  public ushort StreamingEnabled;
}

class Program
{
  static readonly int ReportSize = Marshal.SizeOf<AudioDiag>();
  static AudioDiag Parse(ReadOnlySpan<byte> payload) => MemoryMarshal.Read<AudioDiag>(payload[..ReportSize]);

  static HidDevice? GetDevice()
  {
    const int vid = 0xCAFE;
    const int pid = 0x4014;

    var devices = DeviceList.Local.GetHidDevices(vid, pid).ToList();

    return devices.FirstOrDefault();
  }

  static void Main()
  {
    HidDevice? device = null;
    HidStream? stream = null;
    byte[] reportBuffer = [];
    var stopwatch = new Stopwatch();

    while (true)
    {
      if (device == null)
      {
        device = GetDevice();
        if (device is null)
        {
          Thread.Sleep(1000);
          continue;
        }

        stream = device.Open();
        stream.ReadTimeout = Timeout.Infinite;

        reportBuffer = new byte[device.GetMaxInputReportLength()];

        stopwatch.Reset();

        Console.WriteLine("Time (ms),Underruns,Overflows,Buffer Length,Buffer Remaining (µs),Feedback,SOF Calls,SOF Updates,Streaming Enabled");
      }

      try
      {
        if (stream is null)
        {
          device = null;
          continue;
        }

        int bytesRead = stream.Read(reportBuffer, 0, reportBuffer.Length);
        if (bytesRead <= 1) continue;

        if (!stopwatch.IsRunning)
          stopwatch.Start();

        // Strip Report ID byte (WinHID includes it).
        var reportSpan = new ReadOnlySpan<byte>(reportBuffer, 1, bytesRead - 1);
        if (reportSpan.Length < ReportSize) continue;

        var report = Parse(reportSpan);
        if (report.Magic != 0x44584542u) continue;

        Console.WriteLine($"{stopwatch.ElapsedMilliseconds},{report.Underruns},{report.Overflows},{report.BufferLength},{report.BufferTime},{report.Feedback},{report.NumSofCalls},{report.NumSofUpdates},{report.StreamingEnabled > 0}");
      }
      catch (IOException)
      {
        device = null;
        stream?.Dispose();
        stream = null;
        Thread.Sleep(500);
      }
    }
  }
}
