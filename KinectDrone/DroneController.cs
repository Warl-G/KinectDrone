using AR.Drone.Client;
using AR.Drone.Client.Video;
using AR.Drone.Client.Command;
using AR.Drone.Client.Configuration;
using AR.Drone.Data.Navigation;
using System;
using System.Windows.Media;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Timers;
using AR.Drone.Video;

using AR.Drone.Media;
using AR.Drone.Data;

using AR.Drone.Infrastructure;

using System.Drawing;

using FaceDetection;


namespace KinectDrone
{
    class DroneController
    {
        public class DroneCommandChangedEventArgs
        {
            public string CommandText { get; set; }
        }

        public class VideoFrameArrivedEventArgs
        {
            public Bitmap image { get; set; }
        }

        public class BatteryChangedEventArgs
        {
            public Battery battery { set; get; }
        }

        public class AltitudeChangedEventArgs
        {
            public float altitude { set; get; }
        }

        public delegate void DroneCommandChangedDelegate(object sender, DroneCommandChangedEventArgs args);

        public event DroneCommandChangedDelegate DroneCommandChanged;

        public delegate void VideoFrameArrivedDelegate(object sender, VideoFrameArrivedEventArgs args);

        public event VideoFrameArrivedDelegate VideoFrameArrived;

        public delegate void BatteryChangedDelegate(object sender, BatteryChangedEventArgs args);

        public event BatteryChangedDelegate BatteryChanged;

        public delegate void AltitudeChangedDelegate(object sender, AltitudeChangedEventArgs args);

        public event AltitudeChangedDelegate AltitudeChanged;

        private DroneClient _client;
        private Settings settings;
        private float _altitude = -100;
        private Battery _battery;

        private System.Windows.Forms.Timer tmrVideoUpdate;
        private readonly VideoPacketDecoderWorker _videoPacketDecoderWorker;
        private VideoFrame _frame;
        private Bitmap _frameBitmap;
        private uint _frameNumber;
        
        
        public DroneController()
        {
            //this.image = new ImageBrush();
            
            _client = new DroneClient("192.168.1.1");
            settings = new Settings();

            outDoor(false);
            flightWithoutShell(false);

            _battery = new Battery();
            _battery.Percentage = 100;
            _videoPacketDecoderWorker = new VideoPacketDecoderWorker(AR.Drone.Video.PixelFormat.BGR24, true, OnVideoPacketDecoded);
            _videoPacketDecoderWorker.Start();
            _client.VideoPacketAcquired += OnVideoPacketAcquired;
            
            tmrVideoUpdate = new System.Windows.Forms.Timer();
            tmrVideoUpdate.Interval = 20;
            tmrVideoUpdate.Tick += new System.EventHandler(this.tmrVideoUpdate_Tick);
            tmrVideoUpdate.Enabled = true;
            _videoPacketDecoderWorker.UnhandledException += UnhandledException;
        }

        private void UnhandledException(object sender, Exception exception)
        {
            //MessageBox.Show(exception.ToString(), "Unhandled Exception (Ctrl+C)", MessageBoxButtons.OK, MessageBoxIcon.Error);
            Console.WriteLine("videoPacketDecoderWorker.UnhandledException");
        }
        private void OnVideoPacketAcquired(VideoPacket packet)
        {

            if (_videoPacketDecoderWorker.IsAlive)
                _videoPacketDecoderWorker.EnqueuePacket(packet);
        }

        private void OnVideoPacketDecoded(VideoFrame frame)
        {
            _frame = frame;
        }

        private void tmrVideoUpdate_Tick(object sender, EventArgs e)
        {
            if(this._altitude != _client.NavigationData.Altitude)
            {
                this._altitude = _client.NavigationData.Altitude;
                AltitudeChanged(this, new AltitudeChangedEventArgs { altitude = _altitude});
            }
            if (this._battery.Percentage != _client.NavigationData.Battery.Percentage)
            {
                this._battery.Percentage = _client.NavigationData.Battery.Percentage;
                BatteryChanged(this, new BatteryChangedEventArgs { battery = _battery });
            }

            if (_frame == null || _frameNumber == _frame.Number)
                return;
            _frameNumber = _frame.Number;

            if (_frameBitmap == null)
                _frameBitmap = VideoHelper.CreateBitmap(ref _frame);
            else
                VideoHelper.UpdateBitmap(ref _frameBitmap, ref _frame);


            VideoFrameArrived(_client, new VideoFrameArrivedEventArgs { image = _frameBitmap });
            
        }

        

        public DroneController(DroneClient client)
        {
            _client = client;
        }

        public void Start()
        {
            _client.Start();
           // _client.FlatTrim();
            if (_client.IsActive == true)
            {
                Console.WriteLine("actived");
            }
            if (_client.IsAlive == true)
            {
                Console.WriteLine("alived");
            }
            if (_client.IsConnected == true)
            {
                Console.WriteLine("connected");
            }
            
        }

        public void flightWithoutShell(bool flag)
        {
            settings.Control.FlightWithoutShell = flag;
            _client.Send(settings);
        }

        public void outDoor(bool flag)
        {
            settings.Control.Outdoor = flag;
            _client.Send(settings);
        }

        private bool isFlying
        {
            get
            {
                return _client.NavigationData.State.HasFlag(NavigationState.Flying);
            }
  
        }

        private bool isLanded
        {
            get
            {
                return _client.NavigationData.State.HasFlag(NavigationState.Landed);
            }
        }
        public LedAnimationType ledAnimationType
        {
            set;
            get;
        }

        public FlightAnimationType flightAnimationType
        {
            set;
            get;
        }

        public bool isLedEnable
        {
            set;
            get;
        }

        public bool isFlightEnable
        {
            set;
            get;
        }
        private void LedAnimation(LedAnimationType ledAnimationType)
        {
            settings.Leds.LedAnimation = new LedAnimation(ledAnimationType, 2.0f, 5);
            _client.Send(settings);
        }

        private void FlightAnimation(FlightAnimationType flightAnimationType)
        {
            settings.Control.FlightAnimation = new FlightAnimation(flightAnimationType);
            _client.Send(settings);
            // throw new NotImplementedException();

        }

        public void Takeoff()
        {
            _client.Takeoff();
        }

        public void Land()
        {
            _client.Land();
        }


        public void Stop()
        {
            _client.Stop();
        }

        public void Hover()
        {
            _client.Hover();
        }

        public void Emergency()
        {
            _client.Emergency();
        }

        public void ResetEmergency()
        {
            _client.ResetEmergency();
        }


        public void SwitchCamera()
        {
            var configuration = new Settings();
            configuration.Video.Channel = VideoChannelType.Next;
            _client.Send(configuration);
        }




        //control part
        public void SubscribeToGestures()
        {
            // Right Hand
            GestureDetection.RightHandUpDownChanged += GestureDetection_RightHandUpDownChanged;
            GestureDetection.RightHandLeftRightChanged += GestureDetection_RightHandLeftRightChanged;
            GestureDetection.RightHandBackForwardsChanged += GestureDetection_RightHandBackForwardsChanged;

            // Left Hand
            GestureDetection.LeftHandUpDownChanged += GestureDetection_LeftHandUpDownChanged;
            GestureDetection.LeftHandLeftRightChanged += GestureDetection_LeftHandLeftRightChanged;
            GestureDetection.LeftHandBackForwardsChanged += GestureDetection_LeftHandBackForwardsChanged;
        }

        void GestureDetection_LeftHandBackForwardsChanged(object sender, HandPositionChangedArgs args)
        {
            switch (args.Position)
            {
                case HandPosition.Center:
                    break;
                case HandPosition.Backwards:
                    if (isLanded)
                        _client.FlatTrim();
                        DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "Flat Trim " });
                    break;
                case HandPosition.Forwards:
                    _client.Hover();
                    DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "Hover" });
                    break;
            }
        }

        void GestureDetection_RightHandBackForwardsChanged(object sender, HandPositionChangedArgs args)
        {
            switch (args.Position)
            {
                case HandPosition.Center:
                    break;
                case HandPosition.Backwards:

                    
                    {
                        _client.Progress(FlightMode.Progressive, pitch: 0.1f);
                        DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "Backwards" });
                        
                    }
                    break;
                case HandPosition.Forwards:
                   
                    
                    {
                        _client.Progress(FlightMode.Progressive, pitch: -0.1f);
                        DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "Forwards" });
                        
                    }
                    break;
            }
        }

        void GestureDetection_RightHandLeftRightChanged(object sender, HandPositionChangedArgs args)
        {
            switch (args.Position)
            {
                case HandPosition.Center:
                    break;
                case HandPosition.Left:
                    
                    
                    {
                        _client.Progress(FlightMode.Progressive, roll: -0.1f);
                        DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "Leftwards" });
                    }
                    break;
                case HandPosition.Right:
                    
                    {
                    _client.Progress(FlightMode.Progressive, roll: 0.1f);
                    DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "Rightwards" });
                    }
                    break;
            }
        }

        void GestureDetection_LeftHandLeftRightChanged(object sender, HandPositionChangedArgs args)
        {
            switch (args.Position)
            {
                case HandPosition.Center:
                    break;
                case HandPosition.Left:
                    _client.Progress(FlightMode.Progressive, yaw: 0.25f);
                    DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "Turn Left" });
                    break;
                case HandPosition.Right:
                    _client.Progress(FlightMode.Progressive, yaw: -0.25f);
                    DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "Turn Right" });
                    break;
            }
        }

        void GestureDetection_RightHandUpDownChanged(object sender, HandPositionChangedArgs args)
        {
            switch (args.Position)
            {
                case HandPosition.Up:
                    if (args.MotionState == MotionState.Animation)
                    {
                        if (isFlightEnable)
                        {
                            this.FlightAnimation(this.flightAnimationType);
                            DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = this.flightAnimationType.ToString() });
                        }
                        else
                        {
                            DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "Animation Unable" });
                        }
                    }
                    
                    {
                        _client.Progress(FlightMode.Progressive, gaz: 0.25f);
                        DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "Up" });
                    }
                    break;
                case HandPosition.Center:
                    break;
                case HandPosition.Down:             
                    {
                        _client.Progress(FlightMode.Progressive, gaz: -0.25f);
                        DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "Down" });
                    }
                         break;
            }
        }

        

        void GestureDetection_LeftHandUpDownChanged(object sender, HandPositionChangedArgs args)
        {
            switch (args.Position)
            {
                case HandPosition.Up:
                   if(isLanded)
                    {
                        _client.Takeoff();
                        DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "Take Off" });
                    }
                   else
                   {
                       _client.Land();
                       DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = "Land" });
                   }
                    break;
                case HandPosition.Center:
                    if (args.MotionState == MotionState.Animation)
                    {
                        if (isLedEnable)
                        {
                            LedAnimation(this.ledAnimationType);
                            DroneCommandChanged(_client, new DroneCommandChangedEventArgs { CommandText = this.ledAnimationType.ToString() });
                        }
                    }
                    break;
                case HandPosition.Down:
                    
                    break;
            }
        }

        
    }
}
