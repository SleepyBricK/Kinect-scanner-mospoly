using System;
using System.Globalization;
using System.IO;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using Microsoft.Kinect;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using SharpGL;
using SharpGL.SceneGraph;
using OpenCvSharp;
using OpenCvSharp.Aruco;
using OpenCvSharp.Extensions;

using System.Text.RegularExpressions;
using SharpGL.SceneGraph.Assets;
using SharpGL.SceneGraph.Lighting;
using GSF;
using GSF.Drawing;
using System.Collections;
using System.Drawing.Drawing2D;
using System.Web.UI.WebControls;
using static System.Net.Mime.MediaTypeNames;
using System.Web;
using SharpGL.SceneGraph.Raytracing;
using System.Security.Cryptography;
using System.Windows.Forms.VisualStyles;
using GSF.Collections;

namespace Kursach
{
    public partial class Form1 : Form
    {
        KinectSensor kinectSensor;
        Bitmap bitmapFrame;
        bool fl = false;
        bool dep = false;
        ColorImageFormat imageFormat = ColorImageFormat.RgbResolution640x480Fps30;
        private short[] depthPixelData;
        System.Drawing.Point[] points = new System.Drawing.Point[4];
        private byte[] colorPixelData;
        private byte[] depthFrame32;
        Point2f[][] corners_markers;
        Bitmap bmp = new Bitmap(90, 90, System.Drawing.Imaging.PixelFormat.Format32bppRgb);
        Bitmap bmp2 = new Bitmap(640, 480, System.Drawing.Imaging.PixelFormat.Format32bppRgb);
        const int BlueIndex = 0;
        const int GreenIndex = 1;
        const int RedIndex = 2;
        int height;
        int width;
        float rtri = 0;
        int centerX = 280;
        int centerY = 200;
        double angle = 0;
        double angleStep = Math.PI / 180;
        CascadeClassifier faceCascade = new CascadeClassifier();
        public Form1()
        {
            InitializeComponent();
            cmbDisplayMode.Items.Add(ColorImageFormat.RgbResolution640x480Fps30);
            cmbDisplayMode.SelectedIndex = 0;
            PopulateAvailableSensors();
            faceCascade.Load("C:\\Users\\BricK\\source\\repos\\Kursach\\haarcascade_frontalcatface_extended.xml");
        }

        private void cmbSensors_SelectedIndexChanged(object sender, EventArgs e)
        {

        }

        private void PopulateAvailableSensors()
        {
            cmbSensors.Items.Clear();
            foreach (KinectSensor sensor in KinectSensor.KinectSensors)
            {
                cmbSensors.Items.Add(sensor.UniqueKinectId);
                cmbSensors.SelectedItem = sensor.UniqueKinectId;
            }
        }

        private void btnRefresh_Click(object sender, EventArgs e)
        {
            PopulateAvailableSensors();
        }

        private void btnActivate_Click(object sender, EventArgs e)
        {
            if (cmbSensors.SelectedIndex != -1)
            {
                DeActivateSensor();
                string sensorID = cmbSensors.SelectedItem.ToString();
                foreach (KinectSensor sensor in KinectSensor.KinectSensors)
                {
                    if (sensor.UniqueKinectId == sensorID)
                    {
                        kinectSensor = sensor;
                        SetupSensorVideoInput();
                    }
                }                   
                foreach (KinectSensor sensor in KinectSensor.KinectSensors)
                {
                    if (sensor.UniqueKinectId == sensorID)
                    {
                        kinectSensor = sensor;
                        kinectSensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
                        depthPixelData = new short[kinectSensor.DepthStream.FramePixelDataLength];
                        depthFrame32 = new byte[kinectSensor.DepthStream.FramePixelDataLength * sizeof(int)];
                        kinectSensor.DepthFrameReady += SensorDepthFrameReady;
                        kinectSensor.Start();
                    }                        
                }
            }            
        }
        private void SetupSensorVideoInput()
        {
            if (kinectSensor != null)
            {
                imageFormat = (ColorImageFormat)cmbDisplayMode.SelectedItem;
                kinectSensor.ColorStream.Enable(imageFormat);


                kinectSensor.ColorFrameReady +=
                new EventHandler<ColorImageFrameReadyEventArgs>(kinectSensor_ColorFrameReady);


                kinectSensor.Start();
            }
        }

        private void SensorDepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame != null)
                {
                    this.depthFrame32 = new byte[depthFrame.Width * depthFrame.Height * 4];
                    //Update the image to the new format
                    this.depthPixelData = new short[depthFrame.PixelDataLength];
                    depthFrame.CopyPixelDataTo(this.depthPixelData);                   
                    byte[] convertedDepthBits = this.ConvertDepthFrame(this.depthPixelData, ((KinectSensor)sender).DepthStream);                    
                    Bitmap bmap = new Bitmap(depthFrame.Width, depthFrame.Height, System.Drawing.Imaging.PixelFormat.Format32bppRgb);
                    BitmapData bmapdata = bmap.LockBits(new Rectangle(0, 0, depthFrame.Width, depthFrame.Height), ImageLockMode.WriteOnly, bmap.PixelFormat);
                    IntPtr ptr = bmapdata.Scan0;
                    Marshal.Copy(convertedDepthBits, 0, ptr, 4 * depthFrame.PixelDataLength);
                    bmap.UnlockBits(bmapdata);
                    if (chkThird.Checked == true)
                    {                     
                        picVideoDisplay.Image = bmap;
                    }
                    if (dep == true)
                    {
                        picVideoDisplay.Image = conus(bmap);
                    }
                }
            }
        }


        Bitmap conus(Bitmap bb)
        {
            Mat depthImage = BitmapConverter.ToMat(bb);
            Mat cannyImage = new Mat(depthImage.Size(), depthImage.Type());
            Cv2.CvtColor(depthImage, cannyImage, ColorConversionCodes.RGB2GRAY);
            Cv2.MedianBlur(depthImage, depthImage, 5);
            Cv2.Blur(cannyImage, cannyImage, new OpenCvSharp.Size(3, 3), new OpenCvSharp.Point(-1, -1), BorderTypes.Default); // Fuzzy treatment (lower noise)
            Cv2.Canny(cannyImage, cannyImage, 100, 200, 3, true);
            OpenCvSharp.Point[][] contours;           
            HierarchyIndex[] hierarchy;
            Cv2.FindContours(cannyImage, out contours, out hierarchy, RetrievalModes.List, ContourApproximationModes.ApproxSimple);            
            // Поиск конуса по контурам
            foreach (var contour in contours)
            {
                OpenCvSharp.Point[] approx = Cv2.ApproxPolyDP(contour, 1, true);
                var area = Cv2.ContourArea(contour);
                if ((area > 1000)&& (approx.Length <60)) // условие для определения конуса
                {
                    Rect rect = Cv2.BoundingRect(approx);
                    points[0].X = rect.X;
                    points[0].Y = rect.Y;
                    width = rect.Width;
                    points[1].X = rect.X + width;
                    points[1].Y = rect.Y + width;
                    height = rect.Height;
                    // Отображение найденного конуса на изображении глубины
                    var hull = Cv2.ConvexHull(contour);
                    var color = Scalar.Red;
                    foreach (var point in hull)
                    {
                        depthImage.Circle(point, 5, color, -1);
                    }                                    
                }
            }
            bb = OpenCvSharp.Extensions.BitmapConverter.ToBitmap(depthImage);
            return bb;
        }
        private byte[] ConvertDepthFrame(short[] depthFrame, DepthImageStream depthStream)
        {
            //Run through the depth frame making the correlation between the two arrays
            for (int i16 = 0, i32 = 0; i16 < depthFrame.Length && i32 < this.depthFrame32.Length; i16++, i32 += 4)
            {
                // Console.WriteLine(i16 + "," + i32);
                //We don’t care about player’s information here, so we are just going to rule it out by shifting the value.
                int realDepth = depthFrame[i16] >> DepthImageFrame.PlayerIndexBitmaskWidth;
                //We are left with 13 bits of depth information that we need to convert into an 8 bit number for each pixel.
                //There are hundreds of ways to do this. This is just the simplest one.
                //Lets create a byte variable called Distance.
                //We will assign this variable a number that will come from the conversion of those 13 bits.
                byte Distance = 0;
                //XBox Kinects (default) are limited between 800mm and 4096mm.
                int MinimumDistance = 800;
                int MaximumDistance = hScrollBar1.Value;
                //XBox Kinects (default) are not reliable closer to 800mm, so let’s take those useless measurements out.
                //If the distance on this pixel is bigger than 800mm, we will paint it in its equivalent gray
                if (dep == true)
                {
                    if ((realDepth > MinimumDistance) && (realDepth < MaximumDistance))
                    {
                        Distance = (byte)(255 - ((realDepth - MinimumDistance) * 255 / (MaximumDistance - MinimumDistance)));
                        //Use the distance to paint each layer (R G &  of the current pixel.
                        //Painting R, G and B with the same color will make it go from black to gray
                        this.depthFrame32[i32 + RedIndex] = (byte)(Distance);
                        this.depthFrame32[i32 + GreenIndex] = (byte)(Distance);
                        this.depthFrame32[i32 + BlueIndex] = (byte)(Distance);
                    }
                    //If we are closer than 800mm, the just paint it red so we know this pixel is not giving a good value
                    else
                    {
                        this.depthFrame32[i32 + RedIndex] = 0;
                        this.depthFrame32[i32 + GreenIndex] = 0;
                        this.depthFrame32[i32 + BlueIndex] = 0;
                    }
                }
                else
                {
                    if ((realDepth > MinimumDistance) && (realDepth < MaximumDistance))
                    {
                        this.depthFrame32[i32 + RedIndex] = colorPixelData[i32 + RedIndex];
                        this.depthFrame32[i32 + GreenIndex] = colorPixelData[i32 + GreenIndex];
                        this.depthFrame32[i32 + BlueIndex] = colorPixelData[i32 + BlueIndex];
                    }
                    //If we are closer than 800mm, the just paint it red so we know this pixel is not giving a good value
                    else
                    {
                        this.depthFrame32[i32 + RedIndex] = 0;
                        this.depthFrame32[i32 + GreenIndex] = 0;
                        this.depthFrame32[i32 + BlueIndex] = 0;
                    }
                }
            }
            return depthFrame32;
        }
        private void kinectSensor_ColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            {
                if (colorFrame != null)
                {
                    if (dep != true)
                    {
                        this.colorPixelData = new byte[colorFrame.PixelDataLength];
                        colorFrame.CopyPixelDataTo(this.colorPixelData);
                        bitmapFrame = ColorImageFrameToBitmap(colorFrame);
                        if (chkSecond.Checked == true)
                        {
                            detecting(bitmapFrame);
                        }

                        var graphics = Graphics.FromImage(bitmapFrame);
                        graphics.CompositingMode = CompositingMode.SourceOver;
                        if (chkFirst.Checked == true)
                        {
                            int radius = 100;
                            int x = centerX + (int)(radius * Math.Cos(angle));
                            int y = centerY + (int)(radius * Math.Sin(angle));
                            bmp.MakeTransparent(Color.White);
                            graphics.DrawImage(bmp, x, y);
                            angle += angleStep;
                            if (angle >= 2 * Math.PI)
                            {
                                angle = 0;
                            }
                        }
                        if (chkSecond.Checked == true)
                        {
                            findcon(bitmapFrame);
                            if (fl == true)
                            {
                                bmp2.MakeTransparent(Color.White);
                                graphics.DrawImage(bmp2, 0, 0);
                            }
                            picVideoDisplay.Image = bitmapFrame;
                        }
                        else
                        {
                            picVideoDisplay.Image = bitmapFrame;
                        }
                    }
                }
            }           
        }
        private Bitmap ColorImageFrameToBitmap(ColorImageFrame colorFrame)
        {
            byte[] pixelBuffer = new byte[colorFrame.PixelDataLength];
            colorFrame.CopyPixelDataTo(pixelBuffer);
            Bitmap bitmapFrame = new Bitmap(colorFrame.Width, colorFrame.Height,
            System.Drawing.Imaging.PixelFormat.Format32bppRgb);     
            BitmapData bitmapData = bitmapFrame.LockBits(new Rectangle(0, 0, colorFrame.Width, colorFrame.Height),
            ImageLockMode.WriteOnly, bitmapFrame.PixelFormat);
            IntPtr intPointer = bitmapData.Scan0;
            Marshal.Copy(pixelBuffer, 0, intPointer, colorFrame.PixelDataLength);
            bitmapFrame.UnlockBits(bitmapData);
            return bitmapFrame;
        }

        private void DeActivateSensor()
        {
            if (kinectSensor != null)
            {
                kinectSensor.Stop();


                kinectSensor.ColorFrameReady -=
                new EventHandler<ColorImageFrameReadyEventArgs>
                (kinectSensor_ColorFrameReady);


                kinectSensor.Dispose();
            }
        }

        private void openGLControl1_OpenGLInitialized(object sender, EventArgs e)
        {
        }

        private void openGLControl1_OpenGLInitialized_1(object sender, EventArgs e)
        { 
        }
        void detecting(Bitmap bit)
        {
            Mat img = new Mat(bit.Width, bit.Height, MatType.CV_8UC3, new Scalar(255, 255, 255));
            img = BitmapConverter.ToMat(bit);
            var newSize = new OpenCvSharp.Size(640, 480);
            var smallFrame = new Mat();
            Cv2.Resize(img, smallFrame, newSize);
            var gray = new Mat();
            Cv2.CvtColor(img, gray, ColorConversionCodes.BGR2GRAY);
            Cv2.GaussianBlur(img, img, new OpenCvSharp.Size(5, 5), 0);
            var faces = faceCascade.DetectMultiScale(gray, 1.3, 5);
            foreach (var face in faces)
            {
                centerY = face.Top;
                centerX = face.Left;
            }
        }
        void findcon(Bitmap bit)
        {
            Mat img = new Mat(bit.Width, bit.Height, MatType.CV_8UC3, new Scalar(255, 255, 255));
            img = BitmapConverter.ToMat(bit);
            Mat out_flow = img.Clone();
            Cv2.MedianBlur(out_flow, out_flow, 5);
            // BGR to GRAY
            Cv2.CvtColor(img, img, ColorConversionCodes.BGR2GRAY);
            Cv2.GaussianBlur(img, img, new OpenCvSharp.Size(11, 11), 0);
            Dictionary ff = CvAruco.GetPredefinedDictionary(PredefinedDictionaryName.Dict4X4_1000);
            var detectorParameters = DetectorParameters.Create();
            detectorParameters.CornerRefinementMethod = CornerRefineMethod.Subpix;
            //detectorParameters.CornerRefinementMethod = CornerRefineMethod.None;
            detectorParameters.CornerRefinementWinSize = 9;
            CvAruco.DetectMarkers(img, ff, out corners_markers, out int[] id_markers, detectorParameters, out Point2f[][] ref_markers);
            CvAruco.DrawDetectedMarkers(out_flow, corners_markers, id_markers, Scalar.Crimson);
            if (corners_markers.Length != 0)
            {
                fl = true;
            }
            else
            {
                fl = false;
            }
        }

        private void openGLControl1_OpenGLDraw(object sender, RenderEventArgs args)
        {
            if (chkSecond.Checked == false)
            {

                // Создаем экземпляр
                OpenGL gl = this.openGLControl1.OpenGL;
                Color temp = Color.FromArgb(255, 255, 255, 255);// Недавно добавлено, вот настройка цвета. Мы создали новый класс Color и назначили класс цвета с помощью функции Color.FromArgb (). Пожалуйста, узнайте об использовании этого метода.
                openGLControl1.OpenGL.ClearColor(1f, 1f, 1f, 1f);
                // Очистка экрана и буфера глубин
                gl.Clear(OpenGL.GL_COLOR_BUFFER_BIT | OpenGL.GL_DEPTH_BUFFER_BIT);
                // Сбрасываем модельно-видовую матрицу
                gl.LoadIdentity();
                // Сдвигаем перо влево от центра и вглубь экрана
                gl.Translate(0.0f, 0.0f, -5.0f);
                // Вращаем куб вокруг ее оси Y
                gl.Rotate(rtri, 0.0f, 1.0f, 0.0f);
                // Рисуем куб (грани)
                gl.Begin(OpenGL.GL_QUADS);
                // Top
                gl.Color(0.7f, 0.7f, 0.7f);
                gl.Vertex(1.0f, 1.0f, -1.0f);
                gl.Vertex(-1.0f, 1.0f, -1.0f);
                gl.Vertex(-1.0f, 1.0f, 1.0f);
                gl.Vertex(1.0f, 1.0f, 1.0f);
                // Bottom
                gl.Color(0.7f, 0.7f, 0.7f);
                gl.Vertex(1.0f, -1.0f, 1.0f);
                gl.Vertex(-1.0f, -1.0f, 1.0f);
                gl.Vertex(-1.0f, -1.0f, -1.0f);
                gl.Vertex(1.0f, -1.0f, -1.0f);
                // Front
                gl.Color(0.7f, 0.7f, 0.7f);
                gl.Vertex(1.0f, 1.0f, 1.0f);
                gl.Vertex(-1.0f, 1.0f, 1.0f);
                gl.Vertex(-1.0f, -1.0f, 1.0f);
                gl.Vertex(1.0f, -1.0f, 1.0f);
                // Back
                gl.Color(0.7f, 0.7f, 0.7f);
                gl.Vertex(1.0f, -1.0f, -1.0f);
                gl.Vertex(-1.0f, -1.0f, -1.0f);
                gl.Vertex(-1.0f, 1.0f, -1.0f);
                gl.Vertex(1.0f, 1.0f, -1.0f);
                // Left
                gl.Color(0.7f, 0.7f, 0.7f);
                gl.Vertex(-1.0f, 1.0f, 1.0f);
                gl.Vertex(-1.0f, 1.0f, -1.0f);
                gl.Vertex(-1.0f, -1.0f, -1.0f);
                gl.Vertex(-1.0f, -1.0f, 1.0f);
                // Right
                gl.Color(0.7f, 0.7f, 0.7f);
                gl.Vertex(1.0f, 1.0f, -1.0f);
                gl.Vertex(1.0f, 1.0f, 1.0f);
                gl.Vertex(1.0f, -1.0f, 1.0f);
                gl.Vertex(1.0f, -1.0f, -1.0f);
                gl.End();
                // Контроль полной отрисовки следующего изображения
                gl.Flush();
                // Меняем угол поворота 
                rtri -= 3.0f;
                Rectangle rec = new Rectangle(0, 0, 90, 90);
                openGLControl1.DrawToBitmap(bmp, rec);
            }
        }

        private void openGLControl2_OpenGLDraw(object sender, RenderEventArgs args)
        {
            if ((chkSecond.Checked == true) && (fl == true) || (dep == true))
            {
                if (dep != true)
                {
                    points[0] = new System.Drawing.Point((int)corners_markers[0][0].X, (int)corners_markers[0][0].Y);
                    points[1] = new System.Drawing.Point((int)corners_markers[0][1].X, (int)corners_markers[0][1].Y);
                    points[2] = new System.Drawing.Point((int)corners_markers[0][2].X, (int)corners_markers[0][2].Y);
                    points[3] = new System.Drawing.Point((int)corners_markers[0][3].X, (int)corners_markers[0][3].Y);
                    SharpGL.OpenGL gl = new SharpGL.OpenGL();
                    gl.ClearColor(1f, 1f, 1f, 1f);
                    gl.LineWidth(2f);
                    gl.Clear(OpenGL.GL_COLOR_BUFFER_BIT | OpenGL.GL_DEPTH_BUFFER_BIT);
                    gl.MatrixMode(OpenGL.GL_PROJECTION);
                    gl.LoadIdentity();
                    gl.Ortho(0, bitmapFrame.Width, bitmapFrame.Height, 0, -1, 1);
                    gl.MatrixMode(OpenGL.GL_MODELVIEW);
                    gl.LoadIdentity();
                    gl.Begin(OpenGL.GL_POLYGON);
                    gl.Color(0.0f, 0.0f, 1.0f);
                    gl.Vertex(points[0].X, points[0].Y, 0.0);
                    gl.Vertex(points[1].X, points[1].Y, 0.0);
                    gl.Vertex(points[2].X, points[2].Y, 0.0);
                    gl.Vertex(points[3].X, points[3].Y, 0.0);
                    gl.End();
                    gl.Flush();
                    Rectangle rec = new Rectangle(0, 0, 640, 480);
                    openGLControl2.DrawToBitmap(bmp2, rec);
                }
                else
                {
                    float x = points[0].X;
                    float y = points[0].Y + height;
                    float z = 0;
                    
                    float[,] vertices = new float[8, 3]
                    {
                        {x, y, z},
                        {x + width, y, z},
                        {x + width, y + width, z},
                        {x, y + width, z},
                        {x, y, z + height},
                        {x + width, y, z + height},
                        {x + width, y + width, z + height},
                        {x, y + width, z + height}
                    };
                    int[,] edges = new int[12, 2]
                    {
                        {0, 1},
                        {1, 2},
                        {2, 3},
                        {3, 0},
                        {4, 5},
                        {5, 6},
                        {6, 7},
                        {7, 4},
                        {0, 4},
                        {1, 5},
                        {2, 6},
                        {3, 7}
                    };
                    SharpGL.OpenGL gl = new SharpGL.OpenGL();
                    gl.ClearColor(1f, 1f, 1f, 1f);
                    gl.LineWidth(2f);
                    gl.Clear(OpenGL.GL_COLOR_BUFFER_BIT | OpenGL.GL_DEPTH_BUFFER_BIT);
                    gl.MatrixMode(OpenGL.GL_PROJECTION);
                    gl.LoadIdentity();
                    gl.Ortho(0, bitmapFrame.Width, bitmapFrame.Height, 0, -1, 1);
                    gl.MatrixMode(OpenGL.GL_MODELVIEW);
                    gl.LoadIdentity();
                    gl.Begin(OpenGL.GL_LINES);
                    for (int i = 0; i < edges.GetLength(0); i++)
                    {
                        gl.Color(0.0f, 0.0f, 1.0f);
                        int vertex1 = edges[i, 0];
                        int vertex2 = edges[i, 1];
                        gl.Vertex(vertices[vertex1, 0], vertices[vertex1, 1], vertices[vertex1, 2]);
                        gl.Vertex(vertices[vertex2, 0], vertices[vertex2, 1], vertices[vertex2, 2]);
                    }
                    gl.End();
                }
            }
        }

        private void button1_Click(object sender, EventArgs e)
        {
            if (dep == true)
            {
                dep = false;
            }
            else
            {
                dep = true;
            }
        }
    }
}
