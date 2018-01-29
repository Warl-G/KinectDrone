

namespace KinectDrone
{
    using System;
    using System.Collections.Generic;
    using System.Drawing;
    using System.Linq;
    using System.Text;
    using System.Threading.Tasks;
    using System.Windows;
    using System.Windows.Controls;
    using System.Windows.Data;
    using System.Windows.Documents;
    using System.Windows.Input;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using System.Windows.Navigation;
    using System.Windows.Shapes;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
   
    using System.Runtime.InteropServices;
    using Microsoft.Kinect;
    using AR.Drone.Media;
    using AR.Drone.Video;
    using AR.Drone.Data;
    using AR.Drone.Infrastructure;

    using FaceDetection;
    using Emgu.CV;
    using Emgu.CV.Structure;
#if !IOS
    using Emgu.CV.GPU;
    using AR.Drone.Client.Configuration;
    using AR.Drone.Client;
    using System.Security.Policy;
using AR.Drone.Data.Navigation;
#endif

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        

        private Guide guide;

        private DroneController droneController;

        /// <summary>
        /// Boolean value, true if on, false if off
        /// </summary>
        private bool isDroneOn;

        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly System.Windows.Media.Brush handClosedBrush = new System.Windows.Media.SolidColorBrush(System.Windows.Media.Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly System.Windows.Media.Brush handOpenBrush = new System.Windows.Media.SolidColorBrush(System.Windows.Media.Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly System.Windows.Media.Brush handLassoBrush = new System.Windows.Media.SolidColorBrush(System.Windows.Media.Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly System.Windows.Media.Brush trackedJointBrush = new System.Windows.Media.SolidColorBrush(System.Windows.Media.Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly System.Windows.Media.Brush inferredJointBrush = System.Windows.Media.Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly System.Windows.Media.Pen inferredBonePen = new System.Windows.Media.Pen(System.Windows.Media.Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage kinectImageSource;

        private System.Windows.Controls.Image droneImage;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<System.Windows.Media.Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        /// 
 

        public MainWindow()
        {

            string ffmpegPath = string.Format(@"../../../FFmpeg.AutoGen/FFmpeg/bin/windows/{0}", Environment.Is64BitProcess ? "x64" : "x86");
            
            InteropHelper.RegisterLibrariesSearchPath(ffmpegPath);
            //InteropHelper.RegisterLibrariesSearchPath(opencvPath);

            this.guide = new Guide();
            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<System.Windows.Media.Pen>();

            this.bodyColors.Add(new System.Windows.Media.Pen(System.Windows.Media.Brushes.Red, 6));
            this.bodyColors.Add(new System.Windows.Media.Pen(System.Windows.Media.Brushes.Orange, 6));
            this.bodyColors.Add(new System.Windows.Media.Pen(System.Windows.Media.Brushes.Green, 6));
            this.bodyColors.Add(new System.Windows.Media.Pen(System.Windows.Media.Brushes.Blue, 6));
            this.bodyColors.Add(new System.Windows.Media.Pen(System.Windows.Media.Brushes.Indigo, 6));
            this.bodyColors.Add(new System.Windows.Media.Pen(System.Windows.Media.Brushes.Violet, 6));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.kinectImageSource = new DrawingImage(this.drawingGroup);

            this.droneImage = new System.Windows.Controls.Image();

            // use the window object as the view model in this simple example
            this.DataContext = this;

            this.droneController = new DroneController();


            droneController.VideoFrameArrived += droneController_VideoFrameArrived;
            droneController.BatteryChanged += droneController_BatteryChanged;
            droneController.AltitudeChanged += droneController_AltitudeChanged;
            // initialize the components (controls) of the window

            
            
            this.InitializeComponent();
            
            this.initAnimationComponent();
        }

        private void initAnimationComponent()
        {
            LedAnimationCB.ItemsSource = System.Enum.GetNames(typeof(LedAnimationType));
            LedAnimationType defaultLedType = LedAnimationType.BlinkGreenRed;
            LedAnimationCB.SelectedItem = defaultLedType.ToString();
            LedAnimationCB.IsEnabled = false;
            droneController.ledAnimationType = defaultLedType;
            droneController.isLedEnable = false;
            
            FlightAnimationCB.ItemsSource = System.Enum.GetNames(typeof(FlightAnimationType));
            FlightAnimationType defaultFlightType = FlightAnimationType.FlipRight;
            FlightAnimationCB.SelectedItem = defaultFlightType.ToString();
            FlightAnimationCB.IsEnabled = false;
            droneController.flightAnimationType = defaultFlightType;
            droneController.isFlightEnable = false;
        }

        private void droneController_VideoFrameArrived(object sender, DroneController.VideoFrameArrivedEventArgs args)
        {
            //updateBattery();
            Bitmap bitImage = args.image;
            if(FaceDectectCheck.IsChecked == true)
                bitImage = FaceDetectController.detectFace(bitImage);

            //DroneImage.Source = VideoHelper.ChangeBitmapToImageSource(bitImage);
            if(this.isDroneOn)
                DroneEllipse.Fill = System.Windows.Media.Brushes.Green;
            else if (!this.isDroneOn)
                DroneEllipse.Fill = System.Windows.Media.Brushes.Red;
            
            this.DroneImageSource = VideoHelper.ChangeBitmapToImageSource(bitImage);
        }


        private void droneController_BatteryChanged(object sender, KinectDrone.DroneController.BatteryChangedEventArgs args)
        {
            updateBattery(args.battery);
        }
        private void updateBattery(Battery battery)
        {
            SolidColorBrush color = System.Windows.Media.Brushes.Green;
            if (battery.Percentage < 35)
                color = System.Windows.Media.Brushes.Orange;
            if (battery.Percentage < 20)
                color = System.Windows.Media.Brushes.Red;
         
            BatteryLabel.Content = battery.Percentage.ToString();
            BatteryBar.Background = color;
            BatteryBar.Value = 100.0 - battery.Percentage;      
        }

        private void droneController_AltitudeChanged(object sender, KinectDrone.DroneController.AltitudeChangedEventArgs args)
        {
           AltitudeLabel.Content = "Altitude:"+ args.altitude.ToString()+"M";
        }


        public event PropertyChangedEventHandler PropertyChanged;

        public ImageSource KinectImageSource
        {
            get
            {
                return this.kinectImageSource;
            }
        }

        public ImageSource DroneImageSource
        {
            get
            {
                return this.droneImage.Source;
            }
            set
            {
                if (this.droneImage.Source != value)
                {
                    this.droneImage.Source = value;
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("DroneImageSource"));
                    }
                }
            }
        }

       
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {


            droneController.SubscribeToGestures();
            droneController.DroneCommandChanged += droneController_DroneCommandChanged;
            
            if (null != this.kinectSensor)
            {
                // Connect to the drone
                //droneController = new DroneController();
                
                

            }
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }

            GestureDetection.RightHandUpDownChanged += OnRightHandUpDownChanged;
            GestureDetection.RightHandLeftRightChanged += OnRightHandLeftRightChanged;
            GestureDetection.RightHandBackForwardsChanged += OnRightHandBackFordwardChanged;

            GestureDetection.LeftHandUpDownChanged += OnLeftHandUpDownChanged;
            GestureDetection.LeftHandLeftRightChanged += OnLeftHandLeftRightChanged;
            GestureDetection.LeftHandBackForwardsChanged += OnLeftHandBackFordwardChanged;
        }

        void droneController_DroneCommandChanged(object sender, DroneController.DroneCommandChangedEventArgs args)
        {
            if (isDroneOn)
                CommandTextBlock.Text = args.CommandText;
        }

        void OnRightHandUpDownChanged(object sender, HandPositionChangedArgs args)
        {
            RightUpButton.IsEnabled = args.Position == HandPosition.Up;
            RightCenter1Button.IsEnabled = args.Position == HandPosition.Center;
            RightDownButton.IsEnabled = args.Position == HandPosition.Down;
        }

        void OnRightHandLeftRightChanged(object sender, HandPositionChangedArgs args)
        {
            RightLeftButton.IsEnabled = args.Position == HandPosition.Left;
            RightCenter2Button.IsEnabled = args.Position == HandPosition.Center;
            RightRightButton.IsEnabled = args.Position == HandPosition.Right;
        }

        void OnRightHandBackFordwardChanged(object sender, HandPositionChangedArgs args)
        {
            RightFordwardsButton.IsEnabled = args.Position == HandPosition.Forwards;
            RightCenter3Button.IsEnabled = args.Position == HandPosition.Center;
            RightBackwardsButton.IsEnabled = args.Position == HandPosition.Backwards;
        }

        void OnLeftHandUpDownChanged(object sender, HandPositionChangedArgs args)
        {
            LeftUpButton.IsEnabled = args.Position == HandPosition.Up;
            LeftCenter1Button.IsEnabled = args.Position == HandPosition.Center;
            LeftDownButton.IsEnabled = args.Position == HandPosition.Down;
        }

        void OnLeftHandLeftRightChanged(object sender, HandPositionChangedArgs args)
        {
            LeftLeftButton.IsEnabled = args.Position == HandPosition.Left;
            LeftCenter2Button.IsEnabled = args.Position == HandPosition.Center;
            LeftRightButton.IsEnabled = args.Position == HandPosition.Right;
        }

        void OnLeftHandBackFordwardChanged(object sender, HandPositionChangedArgs args)
        {
            LeftFordwardsButton.IsEnabled = args.Position == HandPosition.Forwards;
            LeftCenter3Button.IsEnabled = args.Position == HandPosition.Center;
            LeftBackwardsButton.IsEnabled = args.Position == HandPosition.Backwards;
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
            if(isDroneOn)
            {
                
                this.droneController.Land();
                this.droneController.Stop();
                this.isDroneOn = false;
            }

           
        }

    

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                  
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            foreach (Body body in bodies)
            {
                if (body != null)
                {
                    // Detect gestures from first skeleton
                    GestureDetection.FrameReady(body);

                    // Skip subsequent skeletons
                    break;
                }
            }

            if (dataReceived)
            {
                KinectEllipse.Fill = System.Windows.Media.Brushes.Green;
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(System.Windows.Media.Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {
                        System.Windows.Media.Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, System.Windows.Point> jointPoints = new Dictionary<JointType, System.Windows.Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new System.Windows.Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                        }
                    }

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                
                }
                
                
            }
            else
            {
                KinectEllipse.Fill = System.Windows.Media.Brushes.Red;
            }
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, System.Windows.Point> jointPoints, DrawingContext drawingContext, System.Windows.Media.Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                System.Windows.Media.Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, System.Windows.Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, System.Windows.Media.Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            System.Windows.Media.Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, System.Windows.Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    System.Windows.Media.Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    System.Windows.Media.Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    System.Windows.Media.Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    System.Windows.Media.Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }
        private void Guide_Click(object sender, RoutedEventArgs e)
        {
            guide.Show();
        }

        private void Start_Click(object sender, RoutedEventArgs e)
        {
            
            if(Start.Content.ToString() != "Stop")//&&this.kinectSensor!=null&& KinectSensor.GetDefault() != null)
            {
                Start.Content = "Stop";
                //Start.Background = System.Windows.Media.Brushes.Red;
               
                    //this.kinectSensor.Open();
                    this.droneController.Start();
                    isDroneOn = true;
                    //this.droneController.Takeoff();
                CommandTextBlock.Text = "Start";
               
                
                //Rect.Fill = System.Windows.Media.Brushes.Blue;
            }
            else
            {
                this.droneController.Land();
                Start.Content = "Start";
                //Rect.Fill = System.Windows.Media.Brushes.Red;
                //Start.Background = System.Windows.Media.Brushes.Green;
                CommandTextBlock.Text = "Stop";
                droneController.Stop();
                isDroneOn = false;
                DroneEllipse.Fill = System.Windows.Media.Brushes.Red;
                
                
                
            }
            
        }

        private void Emergency_Click(object sender, RoutedEventArgs e)
        {
            
            if (this.Emergency.Content.ToString() == "Emergency")
            {
                droneController.Emergency();
                this.Emergency.Content = "Reset\nEmergency";
                CommandTextBlock.Text = "Emergency";
            }
            else
            {
                droneController.ResetEmergency();
                this.Emergency.Content = "Emergency";
                CommandTextBlock.Text = "Reset Emergency";
            }
        }

        private void SwitchCamera_Click(object sender, RoutedEventArgs e)
        {
            droneController.SwitchCamera();

        }

        private void LedAnimationCB_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            LedAnimationType selectedAnimation = (LedAnimationType)Enum.Parse(typeof(LedAnimationType), LedAnimationCB.SelectedItem.ToString(), false);
            
            droneController.ledAnimationType = selectedAnimation;
        }

        private void FlightAnimationCB_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            FlightAnimationType selectedAnimation = (FlightAnimationType)Enum.Parse(typeof(FlightAnimationType), FlightAnimationCB.SelectedItem.ToString(), false);
            
            droneController.flightAnimationType = selectedAnimation;
        }

       
        private void LedCheck_Checked(object sender, RoutedEventArgs e)
        {
           
            LedAnimationCB.IsEnabled = true;
            droneController.isLedEnable = true;
        }

        private void FlightCheck_Checked(object sender, RoutedEventArgs e)
        {
            FlightAnimationCB.IsEnabled = true;
            droneController.isFlightEnable = true;
        }

        private void LedCheck_Unchecked(object sender, RoutedEventArgs e)
        {
            LedAnimationCB.IsEnabled = false;
            droneController.isLedEnable = false;
        }

        private void FlightCheck_Unchecked(object sender, RoutedEventArgs e)
        {
            FlightAnimationCB.IsEnabled = false;
            droneController.isFlightEnable = false;
        }

        

        private void FlightWithoutShell_Checked(object sender, RoutedEventArgs e)
        {
            droneController.flightWithoutShell(true);
        }

        private void OutDoor_Checked(object sender, RoutedEventArgs e)
        {
            droneController.outDoor(true);
        }

        private void FlightWithoutShell_Unchecked(object sender, RoutedEventArgs e)
        {
            droneController.flightWithoutShell(false);
        }

        private void OutDoor_Unchecked(object sender, RoutedEventArgs e)
        {
            droneController.outDoor(false);
        }


        private void Button_Click(object sender, RoutedEventArgs e)
        {
            //droneController.LedAnimation(droneController.ledAnimationType);
            //droneController.FlightAnimation(droneController.flightAnimationType);
        }
        private void Button_Click_1(object sender, RoutedEventArgs e)
        {
           // if (droneController.isLanded)
           //     droneController.Takeoff();
           // else
           // {droneController.Land();}

        }
    }
}
