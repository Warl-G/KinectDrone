//----------------------------------------------------------------------------
//  Copyright (C) 2004-2013 by EMGU. All rights reserved.       
//----------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Runtime.InteropServices;
using System.Windows.Forms;
using Emgu.CV;
using Emgu.CV.Structure;
using Emgu.CV.UI;
using Emgu.CV.GPU;

namespace FaceDetection
{
    
   public static class FaceDetectController
   {
      
       
      public static Bitmap detectFace(Bitmap bitImage)
      {
          Image<Bgr, Byte> image = new Image<Bgr, byte>(bitImage); //Read the files as an 8-bit Bgr image  
          Console.WriteLine("face");
          long detectionTime;
         List<Rectangle> faces = new List<Rectangle>();
         List<Rectangle> eyes = new List<Rectangle>();
         DetectFace.Detect(image, "haarcascade_frontalface_default.xml", "haarcascade_eye.xml", faces, eyes, out detectionTime);
         foreach (Rectangle face in faces)
            image.Draw(face, new Bgr(Color.Red), 2);
         foreach (Rectangle eye in eyes)
            image.Draw(eye, new Bgr(Color.Blue), 2);

         return image.Bitmap;
      }
       
   }
}