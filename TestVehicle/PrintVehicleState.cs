using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Asv.Common;
using Asv.IO;
using Asv.Mavlink.Minimal;
using Asv.Mavlink;
using Spectre.Console;

namespace TestVehicle
{
    /// <summary>
    /// Управление и мониторинг состояния летального устройства в реальном времени
    /// </summary>
    public class PrintVehicleState
    {
        private readonly CancellationTokenSource _cancel = new();
        private const int DeltaXy = 10;
        private string _lastCommand = string.Empty;
        private Thread? _actionsThread;
        private ArduCopterClientDevice? _device;
        private IDeviceExplorer? _deviceExplorer;
        private Table? _table;
        private Table? _headerTable;
        private Table? _statusTable;
        private Table? _logTable;
        private readonly Queue<string> _commandsJournal = new();

        private readonly List<KeyValuePair<string, string>> _telemetry =
        [
            new("Link", ""),
            new("PacketRateHz", ""),
            new("Type", ""),
            new("SystemStatus", ""),
            new("Autopilot", ""),
            new("BaseMode", ""),
            new("CustomMode", ""),
            new("MavlinkVersion", ""),
            new("Home", ""),
            new("GlobalPosition", ""),
            new("LastCommand", "")
        ];

        /// <summary>
        /// Vehicle state real time monitoring
        /// </summary>
        /// <param name="connection">-c, Connection string. Default "tcp://127.0.0.1:5760"</param>
        //[Command("print-vehicle-state")]
        public async Task<int> Run(string connection = "tcp://127.0.0.1:5760")
        {
            DeviceHelper.CreateDeviceExplorer(connection, out _deviceExplorer);
            while (_device is null)
            {
                var device = await DeviceHelper.DeviceAwaiter(_deviceExplorer, 3000);
                if (device is null)
                {
                    AnsiConsole.MarkupLine("\n[bold yellow]Program interrupted. Exiting gracefully...[/]");
                    return 0;
                }

                _device = device as ArduCopterClientDevice;
                if (_device is not null) continue;
                AnsiConsole.Clear();
                AnsiConsole.WriteLine("This command available only to MavType = 2 devices");
                AnsiConsole.WriteLine("Press R to repeat or any key to exit");
                var key = Console.ReadKey();
                if (key.Key == ConsoleKey.R) continue;
                return 0;
            }

            var status2 = @"Waiting for init device";
            AnsiConsole.Status().StartAsync(status2, statusContext =>
            {
                statusContext.Spinner(Spinner.Known.Bounce);
                while (_device is { State.CurrentValue: ClientDeviceState.Uninitialized })
                {
                    Task.Delay(TimeSpan.FromMilliseconds(100));
                }

                return Task.CompletedTask;
            });
            CreateTables();
            Task.Factory.StartNew(() => RunAsync(_device), TaskCreationOptions.LongRunning);
            Task.Factory.StartNew(KeyListen, TaskCreationOptions.LongRunning);
            _actionsThread = new Thread(KeyListen);
            _actionsThread.Start();

            if (_table != null)
                AnsiConsole.Live(_table).AutoClear(true).StartAsync(async ctx =>
                {
                    while (_cancel.IsCancellationRequested is false)
                    {
                        await Task.Delay(TimeSpan.FromMilliseconds(35));
                        if (_table is null) continue;
                        ctx.Refresh();
                    }
                });
            return 0;
        }

        private void CreateTables()
        {
            _logTable = new Table().AddColumn("Log");
            _headerTable = new Table().Expand().AddColumns("[red]UpArrow[/]", "[red]DownArrow[/]", "[red]LeftArrow[/]",
                    "[red]RightArrow[/]", "[red]T[/]", "[red]Space[/]", "[red]Q[/]")
                .Title("[aqua]Controls[/]");
            _headerTable.AddRow("Move Up", $"Move Down", "Move Left", "Move Right", "Take Off", "Do Land", "Quit");
            _table = new Table().AddColumns("Status", "Log").Expand().Title($"{Markup.Escape(_device.Name.CurrentValue)}");
            _statusTable = new Table().AddColumns("Param", "Value").BorderColor(Spectre.Console.Color.Green);
            foreach (var item in _telemetry)
            {
                _statusTable.AddRow($"{item.Key}", $"[aqua]{item.Value}[/]");
            }
            _table.AddRow(_statusTable);
            _table.AddRow(_headerTable);
        }

        protected async Task<int> RunAsync(ArduVehicleClientDevice? vehicle)
        {
            while (!_cancel.IsCancellationRequested)
            {
                Print(vehicle);
                await Task.Delay(1000, _cancel.Token).ConfigureAwait(false);
            }
            return 0;
        }

        private void Print(ArduVehicleClientDevice? vehicle)
        {
            if (vehicle is null) return;
            if (vehicle.GetMicroservice<IHeartbeatClient>() is null) return;

            var heartbeat = vehicle.GetMicroservice<IHeartbeatClient>()?.RawHeartbeat.CurrentValue;
            var position = vehicle.GetMicroservice<IPositionClient>();
            var homePos = position?.Home.CurrentValue;
            var homePosString = "Not Accessible";
            var currentPosString = homePosString;
            if (homePos is not null)
            {
                homePosString = $"{homePos?.Longitude} {homePos?.Longitude} {homePos?.Altitude}";
            }
            var currentPos = position?.GlobalPosition.CurrentValue;
            if (currentPos is not null)
            {
                currentPosString =
                    $"Lat: {currentPos?.Lat} Lon: {currentPos?.Lon} Alt: {currentPos?.Alt} RelativeAlt: {currentPos?.RelativeAlt}" ?? "";
            }

            var dict = new Dictionary<string, string>
            {
                { nameof(ArduCopterClientDevice.Link), vehicle.Link.State.ToString() ?? string.Empty },
                {
                    nameof(HeartbeatClient.PacketRateHz), string.Empty
                    //vehicle.Microservices.FirstOrDefault(r => r is IHeartbeatClient).GetPropertyValue("PacketRateHz")
                    //    .ToString() ?? string.Empty
                },
                {
                    nameof(HeartbeatPayload.SystemStatus),
                    heartbeat?.SystemStatus.ToString() ?? string.Empty
                },
                {
                    nameof(HeartbeatPayload.Type),
                    heartbeat?.Type.ToString() ?? string.Empty
                },
                {
                    nameof(HeartbeatPayload.Autopilot),
                    heartbeat?.Autopilot.ToString() ?? string.Empty
                },
                {
                    nameof(HeartbeatPayload.BaseMode),
                    heartbeat?.BaseMode.ToString("F") ?? string.Empty
                },
                {
                    nameof(HeartbeatPayload.CustomMode),
                    heartbeat?.Autopilot.ToString() ?? string.Empty
                },
                {
                    nameof(HeartbeatPayload.MavlinkVersion),
                    heartbeat?.MavlinkVersion.ToString() ?? string.Empty
                },
                {
                    nameof(PositionClient.Home),
                    homePosString
                },
                {
                    nameof(PositionClient.GlobalPosition),
                    currentPosString
                },
                { "LastCommand", _lastCommand }
            };
            var count = 0;
            foreach (var item in dict)
            {
                _statusTable.UpdateCell(count, 1, $"{item.Value}");
                count++;
            }

            _table.UpdateCell(0, 0, _statusTable);

            _table.UpdateCell(0, 1, _logTable);
        }

        private static GeoPoint GetCurrentPosition(ArduCopterClientDevice client)
        {
            var pos = client.Microservices.FirstOrDefault(_ => _ is IGnssClient) as GnssClient;
            if (pos?.Main.CurrentValue is null) return GeoPoint.Zero;
            return new GeoPoint(pos.Main.CurrentValue.Lat, pos.Main.CurrentValue.Lon,
                pos.Main.CurrentValue.Alt);
        }

        private void WriteJournal(string log)
        {
            _commandsJournal.Enqueue(log);
            if (_commandsJournal.Count >= 10)
            {
                _commandsJournal.Dequeue();
            }

            _logTable = new Table().AddColumn("Log");
            foreach (var item in _commandsJournal)
            {
                _logTable.AddRow(item);
            }
        }

        private void KeyListen()
        {
            var controlClient =
                _device.Microservices.FirstOrDefault(_ => _ is IControlClient) as ArduCopterControlClient;
            if (controlClient is null)
            {
                AnsiConsole.Clear();
                AnsiConsole.WriteLine($"Unable to get control of {_device?.Name.CurrentValue}");
                throw new NullReferenceException();
            }

            while (true)
            {
                var key = Console.ReadKey(true);
                switch (key.Key)
                {
                    case ConsoleKey.RightArrow:
                        _lastCommand = $"GOTO: delta={DeltaXy} Azimuth=90";
                        var newPoint =
                            GetCurrentPosition(_device).RadialPoint(DeltaXy, 90);
                        controlClient.GoTo(newPoint, _cancel.Token).Wait();
                        break;
                    case ConsoleKey.LeftArrow:
                        _lastCommand = $"GOTO: delta={DeltaXy} Azimuth=270";
                        var newPoint1 = GetCurrentPosition(_device).RadialPoint(DeltaXy, 270);
                        controlClient.GoTo(newPoint1, _cancel.Token).Wait();
                        break;
                    case ConsoleKey.UpArrow:
                        _lastCommand = $"GOTO: delta={DeltaXy} Azimuth=0";
                        var newPoint2 = GetCurrentPosition(_device).RadialPoint(DeltaXy, 0);
                        controlClient.GoTo(newPoint2, _cancel.Token).Wait();
                        break;
                    case ConsoleKey.DownArrow:
                        _lastCommand = $"GOTO: delta={DeltaXy} Azimuth=180";
                        var newPoint3 = GetCurrentPosition(_device).RadialPoint(DeltaXy, 180);
                        controlClient.GoTo(newPoint3, _cancel.Token).Wait();
                        break;
                    case ConsoleKey.Q:
                        _cancel.Cancel(false);
                        break;
                    case ConsoleKey.T:
                        _lastCommand = "Takeoff";
                        controlClient.SetGuidedMode();
                        controlClient.TakeOff(GetCurrentPosition(_device).Altitude + 50f,
                            CancellationToken.None).Wait();
                        break;
                    case ConsoleKey.Spacebar:
                        _lastCommand = "DoLand";
                        controlClient.DoLand(_cancel.Token).Wait();
                        break;
                }

                WriteJournal(_lastCommand);
            }
        }
    }
}
