using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;


namespace KinectDrone
{
    public enum HandPosition { Unknown, Up, Center, Down, Left, Right, Backwards, Forwards }

    
    public enum MotionState{ FreeControl , Animation}

    public class HandPositionChangedArgs
    {
        public HandPosition Position { get; set; }
        public MotionState MotionState { get; set; }
   
    }


    public delegate void HandPositionChangedDelegate(object sender, HandPositionChangedArgs args);
    
    public static class GestureDetection
    {
        public static void FrameReady(Body body)
        {
            ProcessRightHand(body);
            ProcessLeftHand(body);
        }

        

        // Right Hand
        private static Dictionary<ulong, HandPosition> rightHandUpDownDictionary = new Dictionary<ulong, HandPosition>();
        private static Dictionary<ulong, HandPosition> rightHandLeftRightDictionary = new Dictionary<ulong, HandPosition>();
        private static Dictionary<ulong, HandPosition> rightHandBackForwardsDictionary = new Dictionary<ulong, HandPosition>();

        public static event HandPositionChangedDelegate RightHandUpDownChanged;
        public static event HandPositionChangedDelegate RightHandLeftRightChanged;
        public static event HandPositionChangedDelegate RightHandBackForwardsChanged;
        public static void ProcessRightHand(Body body)
        {



            //ulong bodyKey = body.TrackingId;
            ulong bodyKey = 0;

            Joint rightHand = body.Joints[JointType.HandRight];
            Joint rightShoulder = body.Joints[JointType.ShoulderRight];

            CameraSpacePoint rightHandPoint = rightHand.Position;
            CameraSpacePoint rightShoulderPoint = rightShoulder.Position;

            HandPosition previousRightHandUpDownPosition = (rightHandUpDownDictionary.ContainsKey(bodyKey)) ? rightHandUpDownDictionary[bodyKey] : HandPosition.Unknown;
            HandPosition newRightHandUpDownPosition = HandPosition.Unknown;

            HandPosition previousRightHandLeftRightPosition = (rightHandLeftRightDictionary.ContainsKey(bodyKey)) ? rightHandLeftRightDictionary[bodyKey] : HandPosition.Unknown;
            HandPosition newRightHandLeftRightPosition = HandPosition.Unknown;

            HandPosition previousRightHandBackForwardsPosition = (rightHandBackForwardsDictionary.ContainsKey(bodyKey)) ? rightHandBackForwardsDictionary[bodyKey] : HandPosition.Unknown;
            HandPosition newRightHandBackForwardsPosition = HandPosition.Unknown;

            
            MotionState newRightMotionState = MotionState.FreeControl;
           


            if ((rightHand.TrackingState == TrackingState.NotTracked) || (rightShoulder.TrackingState == TrackingState.NotTracked))
            {
                newRightHandUpDownPosition = HandPosition.Unknown;
                newRightHandLeftRightPosition = HandPosition.Unknown;
                newRightHandBackForwardsPosition = HandPosition.Unknown;
            }
            else
            {
                if (body.HandRightState ==HandState.Lasso)
                {
                    newRightMotionState = MotionState.Animation;
                }
                else {
                    newRightMotionState = MotionState.FreeControl;
                }

               


                // Up/Down
                if (rightHandPoint.Y - rightShoulderPoint.Y > 0.2&&
                    rightShoulderPoint.Z - rightHandPoint.Z < 0.3 &&
                    Math.Abs(rightHandPoint.X - rightShoulderPoint.X) < 0.2)
                {
                    newRightHandUpDownPosition = HandPosition.Up;
                }
                else if (Math.Abs(rightHandPoint.Y - rightShoulderPoint.Y) > 0.2&&
                         rightShoulderPoint.Z - rightHandPoint.Z < 0.3&&
                         Math.Abs(rightHandPoint.X - rightShoulderPoint.X) < 0.2)
                {
                    newRightHandUpDownPosition = HandPosition.Down;
                }
                else if (rightShoulderPoint.Z - rightHandPoint.Z < 0.3 &&
                    Math.Abs(rightHandPoint.X - rightShoulderPoint.X) < 0.2)
                {
                    newRightHandUpDownPosition = HandPosition.Center;
                }

                // Left/Right
                if (rightHandPoint.X - rightShoulderPoint.X > 0.2&&
                    Math.Abs(rightHandPoint.Y - rightShoulderPoint.Y) < 0.2&&
                    rightShoulderPoint.Z - rightHandPoint.Z < 0.3)
                {
                    newRightHandLeftRightPosition = HandPosition.Right;
                }
                else if (Math.Abs(rightHandPoint.X - rightShoulderPoint.X) > 0.2&&
                         Math.Abs(rightHandPoint.Y - rightShoulderPoint.Y) < 0.2 &&
                         rightShoulderPoint.Z - rightHandPoint.Z < 0.3)
                {
                    newRightHandLeftRightPosition = HandPosition.Left;
                }
                else if (Math.Abs(rightHandPoint.Y - rightShoulderPoint.Y) < 0.2 &&
                         rightShoulderPoint.Z - rightHandPoint.Z < 0.3)
                {
                    newRightHandLeftRightPosition = HandPosition.Center;
                }

                // Backwards/Forwards
                if (rightShoulderPoint.Z - rightHandPoint.Z > 0.5&&
                    Math.Abs(rightHandPoint.Y - rightShoulderPoint.Y) < 0.2&&
                    Math.Abs(rightHandPoint.X - rightShoulderPoint.X) < 0.2)
                {
                    newRightHandBackForwardsPosition = HandPosition.Forwards;
                }
                else if (rightShoulderPoint.Z - rightHandPoint.Z < 0.25 &&
                    Math.Abs(rightHandPoint.Y - rightShoulderPoint.Y) < 0.2 &&
                    Math.Abs(rightHandPoint.X - rightShoulderPoint.X) < 0.2)
                {
                    newRightHandBackForwardsPosition = HandPosition.Backwards;
                }
                else if (Math.Abs(rightHandPoint.Y - rightShoulderPoint.Y) < 0.2 &&
                    Math.Abs(rightHandPoint.X - rightShoulderPoint.X) < 0.2)
                {
                    newRightHandBackForwardsPosition = HandPosition.Center;
                }
            }

            if (previousRightHandUpDownPosition != newRightHandUpDownPosition)
            {
                rightHandUpDownDictionary[bodyKey] = newRightHandUpDownPosition;
                if (RightHandUpDownChanged != null)
                {
                    //Console.WriteLine("RightHandUpDownChanged");
                    RightHandUpDownChanged(body, new HandPositionChangedArgs() { Position = newRightHandUpDownPosition, MotionState = newRightMotionState });
                }
            }

            if (previousRightHandLeftRightPosition != newRightHandLeftRightPosition)
            {
                rightHandLeftRightDictionary[bodyKey] = newRightHandLeftRightPosition;
                if (RightHandLeftRightChanged != null)
                {
                    //Console.WriteLine("RightHandLeftRightChanged");
                    RightHandLeftRightChanged(body, new HandPositionChangedArgs() { Position = newRightHandLeftRightPosition, MotionState = newRightMotionState });
                }
            }

            if (previousRightHandBackForwardsPosition != newRightHandBackForwardsPosition)
            {
                rightHandBackForwardsDictionary[bodyKey] = newRightHandBackForwardsPosition;
                if (RightHandBackForwardsChanged != null)
                {
                    //Console.WriteLine("RightHandBackForwardsChanged");
                    RightHandBackForwardsChanged(body, new HandPositionChangedArgs() { Position = newRightHandBackForwardsPosition, MotionState = newRightMotionState });
                }
            }
        }

        // Left Hand
        private static Dictionary<ulong, HandPosition> leftHandUpDownDictionary = new Dictionary<ulong, HandPosition>();
        private static Dictionary<ulong, HandPosition> leftHandLeftRightDictionary = new Dictionary<ulong, HandPosition>();
        private static Dictionary<ulong, HandPosition> leftHandBackForwardsDictionary = new Dictionary<ulong, HandPosition>();

        public static event HandPositionChangedDelegate LeftHandUpDownChanged;
        public static event HandPositionChangedDelegate LeftHandLeftRightChanged;
        public static event HandPositionChangedDelegate LeftHandBackForwardsChanged;

        public static void ProcessLeftHand(Body body)
        {

            //ulong bodyKey = body.TrackingId;
            ulong bodyKey = 0;

            Joint leftHand = body.Joints[JointType.HandLeft];
            Joint leftShoulder = body.Joints[JointType.ShoulderLeft];

            CameraSpacePoint leftHandPoint = leftHand.Position;
            CameraSpacePoint leftShoulderPoint = leftShoulder.Position;

            HandPosition previousLeftHandUpDownPosition = (leftHandUpDownDictionary.ContainsKey(bodyKey)) ? leftHandUpDownDictionary[bodyKey] : HandPosition.Unknown;
            HandPosition newLeftHandUpDownPosition = HandPosition.Unknown;

            HandPosition previousLeftHandLeftRightPosition = (leftHandLeftRightDictionary.ContainsKey(bodyKey)) ? leftHandLeftRightDictionary[bodyKey] : HandPosition.Unknown;
            HandPosition newLeftHandLeftRightPosition = HandPosition.Unknown;

            HandPosition previousLeftHandBackForwardsPosition = (leftHandBackForwardsDictionary.ContainsKey(bodyKey)) ? leftHandBackForwardsDictionary[bodyKey] : HandPosition.Unknown;
            HandPosition newLeftHandBackForwardsPosition = HandPosition.Unknown;

            MotionState newLeftMotionState = MotionState.FreeControl;
             
            if ((leftHand.TrackingState == TrackingState.NotTracked) || (leftShoulder.TrackingState == TrackingState.NotTracked))
            {
                newLeftHandUpDownPosition = HandPosition.Unknown;
                newLeftHandLeftRightPosition = HandPosition.Unknown;
                newLeftHandBackForwardsPosition = HandPosition.Unknown;
            }
            else
            {
                if(body.HandLeftState == HandState.Lasso)
                {
                    newLeftMotionState = MotionState.Animation;
                }
                else{
                    newLeftMotionState = MotionState.FreeControl;
                }


                // Up/Down
                if (leftHandPoint.Y - leftShoulderPoint.Y > 0.2&&
                    Math.Abs(leftHandPoint.X - leftShoulderPoint.X) < 0.2&&
                    leftShoulderPoint.Z - leftHandPoint.Z < 0.3)
                {
                    newLeftHandUpDownPosition = HandPosition.Up;
                }
                else if (Math.Abs(leftHandPoint.Y - leftShoulderPoint.Y) > 0.2&&
                    Math.Abs(leftHandPoint.X - leftShoulderPoint.X) < 0.2 &&
                    leftShoulderPoint.Z - leftHandPoint.Z < 0.3)
                {
                    newLeftHandUpDownPosition = HandPosition.Down;
                }
                else if (Math.Abs(leftHandPoint.X - leftShoulderPoint.X) < 0.2 &&
                    leftShoulderPoint.Z - leftHandPoint.Z < 0.3)
                {
                    newLeftHandUpDownPosition = HandPosition.Center;
                }

                // Left/Right
                if (leftHandPoint.X - leftShoulderPoint.X > 0.2&&
                    Math.Abs(leftHandPoint.Y - leftShoulderPoint.Y) < 0.2&&
                    leftShoulderPoint.Z - leftHandPoint.Z < 0.3)
                {
                    newLeftHandLeftRightPosition = HandPosition.Right;
                }
                else if (Math.Abs(leftHandPoint.X - leftShoulderPoint.X) > 0.2&&
                    Math.Abs(leftHandPoint.Y - leftShoulderPoint.Y) < 0.2 &&
                    leftShoulderPoint.Z - leftHandPoint.Z < 0.3)
                {
                    newLeftHandLeftRightPosition = HandPosition.Left;
                }
                else if (Math.Abs(leftHandPoint.Y - leftShoulderPoint.Y) < 0.2 &&
                    leftShoulderPoint.Z - leftHandPoint.Z < 0.3)
                {
                    newLeftHandLeftRightPosition = HandPosition.Center;
                }

                // Backwards/Forwards
                if (leftShoulderPoint.Z - leftHandPoint.Z > 0.5&&
                    Math.Abs(leftHandPoint.Y - leftShoulderPoint.Y) < 0.2&&
                    Math.Abs(leftHandPoint.X - leftShoulderPoint.X) < 0.2
                    )
                {
                    newLeftHandBackForwardsPosition = HandPosition.Forwards;
                }
                else if (leftShoulderPoint.Z - leftHandPoint.Z < 0.25&&
                    Math.Abs(leftHandPoint.Y - leftShoulderPoint.Y) < 0.2 &&
                    Math.Abs(leftHandPoint.X - leftShoulderPoint.X) < 0.2)
                {
                    newLeftHandBackForwardsPosition = HandPosition.Backwards;
                }
                else if (Math.Abs(leftHandPoint.Y - leftShoulderPoint.Y) < 0.2 &&
                    Math.Abs(leftHandPoint.X - leftShoulderPoint.X) < 0.2)
                {
                    newLeftHandBackForwardsPosition = HandPosition.Center;
                }
            }

            if (previousLeftHandUpDownPosition != newLeftHandUpDownPosition)
            {
                leftHandUpDownDictionary[bodyKey] = newLeftHandUpDownPosition;
                if (LeftHandUpDownChanged != null)
                {
                    //Console.WriteLine("LeftHandUpDownChanged");
                    LeftHandUpDownChanged(body, new HandPositionChangedArgs() { Position = newLeftHandUpDownPosition ,MotionState = newLeftMotionState});
                }
            }

            if (previousLeftHandLeftRightPosition != newLeftHandLeftRightPosition)
            {
                leftHandLeftRightDictionary[bodyKey] = newLeftHandLeftRightPosition;
                if (LeftHandLeftRightChanged != null)
                {
                    LeftHandLeftRightChanged(body, new HandPositionChangedArgs() { Position = newLeftHandLeftRightPosition, MotionState = newLeftMotionState });
                }
            }
            /*
            if (previousLeftHandBackForwardsPosition != newLeftHandBackForwardsPosition)
            {
                leftHandBackForwardsDictionary[bodyKey] = newLeftHandBackForwardsPosition;
                if (LeftHandBackForwardsChanged != null)
                {
                    LeftHandBackForwardsChanged(body, new HandPositionChangedArgs() { Position = newLeftHandBackForwardsPosition , MotionState = newLeftMotionState});
                }
            }*/



        }
    }
}