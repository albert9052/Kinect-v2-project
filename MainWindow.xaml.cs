using System;
using System.Collections.Generic;
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
using Microsoft.Kinect;
using System.Runtime.Remoting.Messaging;
using System.IO;
using System.Threading;
using System.Diagnostics;

namespace testing
{
    /// <summary>
    /// MainWindow.xaml 的互動邏輯
    /// </summary>
    public partial class MainWindow : Window
    {
        private KinectSensor sensor = null;
        private FrameDescription frameDescription = null;
        private WriteableBitmap bitmap = null;

        // For color frame reader
        private ColorFrameReader colorFrameReader = null;

        // For body frame reader
        private BodyFrameReader bodyFrameReader = null;
        public int boneThickness = 3;
        public int jointThickness = 10;
        private const float InferredZPositionClamp = 0.1f;
        //public double Xratio = 1280.0 / 512;
        //public double Yratio = 720.0 / 424;
        public double Xratio = 640.0 / 512;
        public double Yratio = 360.0 / 424;
        //public double XratioForRecognition = 1416.0 / 512;
        //public double YratioForRecognition = 712.0 / 424;

        private CoordinateMapper coordinateMapper = null;
        private Body[] bodies = null;
        private List<Tuple<JointType, JointType>> bones;

        private double thresholdForHandDirection = 1.0 / 2.0;
        private double thresholdForHandBending = Math.PI * 3.0 / 4.0;

        private List<Point> drawingPoints;
        private DateTime lastTimeDrawingHandClosed;
        private DateTime lastTimeDrawingHandOpened;
        private Point pointLastTime;
        private Point pointLastTimeForRecognition;
        private TimeSpan tolerantIntervalForDiscontinue;
        private TimeSpan intervalBetweenEachDrawingPoint;
        public int drawingStrokeWidth = 5;
        private const bool STORE_EVERY_SHAPE = true;
        private List<Point> previousPoints;
        private Point startingVertex;
        private Point lastPoint;
        public int numOfPointsToVertex = 3;
        public int rangeOfVertex = 10;
        public int distForAutoConnectingStartingVertex = 60;
        public int distForVertices = 20;
        private Point anchorForAutoConnectingStartingVertex;
        private int countDownForStoreFrame = 0;
        private bool drawingFinished;
        private int counterForLinesDrawnWhenHandNotClosed;

        private Program classifyProgram;

        public MainWindow()
        {
            this.sensor = KinectSensor.GetDefault();
            this.frameDescription = this.sensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Rgba);

            // For color frame
            this.colorFrameReader = this.sensor.ColorFrameSource.OpenReader();
            // Whenever a frame arrived, the program will call the function ColorFrameRReader_FrameArrived
            this.colorFrameReader.FrameArrived += ColorFrameReader_FrameArrived;

            // For body frame
            this.bodyFrameReader = this.sensor.BodyFrameSource.OpenReader();
            // Wnenever a frame arrived, the program will call the function BodyFrameReader_FrameArrived
            this.bodyFrameReader.FrameArrived += BodyFrameReader_FrameArrived;
            this.coordinateMapper = this.sensor.CoordinateMapper;

            // Create connection of joints
            this.bones = new List<Tuple<JointType, JointType>>();

            #region Initialize skeleton
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
            #endregion

            this.drawingPoints = new List<Point>();
            this.lastTimeDrawingHandClosed = DateTime.Now;
            this.lastTimeDrawingHandOpened = DateTime.Now;
            this.pointLastTime = new Point(-1, -1);
            this.pointLastTimeForRecognition = new Point(-1, -1);
            this.tolerantIntervalForDiscontinue = new TimeSpan(1000 * 10000);
            this.intervalBetweenEachDrawingPoint = new TimeSpan(1 * 10000);
            this.previousPoints = new List<Point>();
            this.startingVertex = new Point(-1, -1);
            this.lastPoint = new Point(-1, -1);
            this.countDownForStoreFrame = 0;
            this.anchorForAutoConnectingStartingVertex = new Point(-1, -1);
            this.drawingFinished = false;
            this.counterForLinesDrawnWhenHandNotClosed = 0;

            classifyProgram = new Program();

            //
            this.bitmap = new WriteableBitmap(this.frameDescription.Width, this.frameDescription.Height, 96, 96, PixelFormats.Bgr32, null);
            // Start Kinect
            this.sensor.Open();
        }

        private void ColorFrameReader_FrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {

            using (ColorFrame colorFrame = e.FrameReference.AcquireFrame())
            {
                if (colorFrame == null)
                    return;
                using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                {
                    this.bitmap.Lock();

                    // verify data and write the new color frame data to the display bitmap
                    if ((this.frameDescription.Width == this.bitmap.PixelWidth) &&
                        (this.frameDescription.Height == this.bitmap.PixelHeight))
                    {
                        colorFrame.CopyConvertedFrameDataToIntPtr(
                            this.bitmap.BackBuffer,
                            (uint)(frameDescription.Width * frameDescription.Height * 4),
                            ColorImageFormat.Bgra);

                        this.bitmap.AddDirtyRect(new Int32Rect(0, 0, this.bitmap.PixelWidth, this.bitmap.PixelHeight));
                    }

                    this.bitmap.Unlock();
                }
            }

        }

        private void BodyFrameReader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            // "ImageCanvas" is like an id, and it's defined in MainWindow.xaml
            this.ImageCanvasForMouse.Children.Clear();
            TextBlock.Text = "";
            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame == null)
                    return;

                if (this.bodies == null)
                {
                    this.bodies = new Body[bodyFrame.BodyCount];
                }
                bodyFrame.GetAndRefreshBodyData(this.bodies);

                foreach (Body body in this.bodies)
                {
                    if (body.IsTracked)
                    {
                        IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                        Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                        foreach (JointType jointType in joints.Keys)
                        {
                            // sometimes the depth(Z) of an inferred joint may show as negative
                            // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                            CameraSpacePoint position = joints[jointType].Position;
                            if (position.Z < 0)
                            {
                                position.Z = InferredZPositionClamp; // 0.1
                            }

                            DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position); // I guess this is for mapping the position to what we see the world, instead of the camera's own coordinates. 
                            jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);

                            //if (depthSpacePoint.X != position.X)
                            //{
                            //    TextBlock.Text += "Not the same for X" + "\n";
                            //}
                            //if (depthSpacePoint.Y != position.Y)
                            //{
                            //    TextBlock.Text += "Not the same for Y" + "\n";
                            //}
                        }

                        HandDirection rightHandDirection = getHandDirection(jointPoints[JointType.ShoulderRight], jointPoints[JointType.ElbowRight], Side.RIGHT);
                        HandDirection leftHandDirection = getHandDirection(jointPoints[JointType.ShoulderLeft], jointPoints[JointType.ElbowLeft], Side.LEFT);
                        bool rightHandIsBending = isHandBending(jointPoints[JointType.ShoulderRight], jointPoints[JointType.ElbowRight], jointPoints[JointType.WristRight]);
                        bool leftHandIsBending = isHandBending(jointPoints[JointType.ShoulderLeft], jointPoints[JointType.ElbowLeft], jointPoints[JointType.WristLeft]);

                        TextBlock.Text += "動作: \n";

                        if (rightHandDirection == HandDirection.UP && leftHandDirection == HandDirection.UP)
                        {
                            if (rightHandIsBending == true && leftHandIsBending == true)
                            {
                                TextBlock.Text += "圈圈";
                            }
                            else if (rightHandIsBending == false && leftHandIsBending == false)
                            {
                                TextBlock.Text += "萬歲";
                            }
                        }
                        else if (rightHandDirection == HandDirection.MIDDLE && leftHandDirection == HandDirection.MIDDLE)
                        {
                            if (rightHandIsBending == false && leftHandIsBending == false)
                            {
                                TextBlock.Text += "平舉";
                            }
                        }
                        else if (rightHandDirection == HandDirection.DOWN && leftHandDirection == HandDirection.DOWN)
                        {
                            if (rightHandIsBending == true && leftHandIsBending == true)
                            {
                                TextBlock.Text += "插腰";
                            }
                            else if (rightHandIsBending == false && leftHandIsBending == false)
                            {
                                TextBlock.Text += "無動作";
                            }
                        }
                        if ((rightHandDirection == HandDirection.UP || rightHandDirection == HandDirection.MIDDLE) &&
                                    (leftHandDirection == HandDirection.UP || leftHandDirection == HandDirection.MIDDLE))
                        {
                            if (rightHandIsBending == true && leftHandIsBending == false)
                            {
                                TextBlock.Text += "Left Dab";
                            }
                            else if (rightHandIsBending == false && leftHandIsBending == true)
                            {
                                TextBlock.Text += "Right Dab";
                            }
                        }

                        //if (isIntersect(jointPoints[JointType.ElbowLeft], jointPoints[JointType.WristLeft], jointPoints[JointType.ElbowLeft], jointPoints[JointType.WristLeft]))
                        //{
                        //    TextBlock.Text = "INTERSECT";
                        //}
                        //else if (isIntersect(jointPoints[JointType.ElbowRight], jointPoints[JointType.HandRight], jointPoints[JointType.ElbowLeft], jointPoints[JointType.HandLeft]))
                        //{
                        //    TextBlock.Text = "INTERSECT";
                        //}
                        //else if (isIntersect(jointPoints[JointType.ElbowRight], jointPoints[JointType.HandTipRight], jointPoints[JointType.ElbowLeft], jointPoints[JointType.HandTipLeft]))
                        //{
                        //    TextBlock.Text = "INTERSECT";
                        //}
                        //else if (isIntersect(jointPoints[JointType.ShoulderRight], jointPoints[JointType.WristRight], jointPoints[JointType.ShoulderLeft], jointPoints[JointType.WristLeft]))
                        //{
                        //    TextBlock.Text = "INTERSECT";
                        //}
                        //else if (isIntersect(jointPoints[JointType.ShoulderRight], jointPoints[JointType.HandRight], jointPoints[JointType.ShoulderLeft], jointPoints[JointType.HandLeft]))
                        //{
                        //    TextBlock.Text = "INTERSECT";
                        //}
                        //else if (isIntersect(jointPoints[JointType.ShoulderRight], jointPoints[JointType.HandTipRight], jointPoints[JointType.ShoulderLeft], jointPoints[JointType.HandTipLeft]))
                        //{
                        //    TextBlock.Text = "INTERSECT";
                        //}
                        //else
                        //{
                        //    TextBlock.Text = "NOT INTERSECT";
                        //}

                        //drawJoints(jointPoints);
                        //drawBones(jointPoints);

                        if (body.HandRightState == HandState.Closed)
                        {
                            lastPoint.X = jointPoints[JointType.HandRight].X;
                            lastPoint.Y = jointPoints[JointType.HandRight].Y;
                            if (startingVertex.X == -1 && startingVertex.Y == -1)
                            {
                                this.startingVertex.X = jointPoints[JointType.HandRight].X;
                                this.startingVertex.Y = jointPoints[JointType.HandRight].Y;
                                this.pointLastTimeForRecognition.X = this.startingVertex.X;
                                this.pointLastTimeForRecognition.Y = this.startingVertex.Y;
                            }
                            if ((DateTime.Now - lastTimeDrawingHandClosed).CompareTo(intervalBetweenEachDrawingPoint) > 0)
                            {
                                drawStroke(jointPoints[JointType.HandRight]);
                                // If we start drawing, we have to regard the first point we draw as a vertex no matter what. 
                                if (isVertex(jointPoints[JointType.HandRight]) && dist(pointLastTimeForRecognition, jointPoints[JointType.HandRight]) > distForVertices)
                                {
                                    drawStrokeForRecognition(jointPoints[JointType.HandRight]);
                                }
                                lastTimeDrawingHandClosed = DateTime.Now;
                            }
                            counterForLinesDrawnWhenHandNotClosed = 0;
                        }
                        else
                        {
                            //if (this.ImageCanvasForRecognition.Children.Count != 0 && ((DateTime.Now - lastTimeDrawingHandClosed).CompareTo(intervalBetweenEachDrawingPoint) > 0 || (DateTime.Now - lastTimeDrawingHandOpened).CompareTo(intervalBetweenEachDrawingPoint) > 0))
                            //{
                            //    if (isVertex(jointPoints[JointType.HandRight]) && dist(pointLastTimeForRecognition, jointPoints[JointType.HandRight]) > distForVertices)
                            //    {
                            //        counterForLinesDrawnWhenHandNotClosed += 1;
                            //        drawStrokeForRecognition(jointPoints[JointType.HandRight]);
                            //    }
                            //    lastTimeDrawingHandOpened = DateTime.Now;
                            //}
                            drawMousePoint(jointPoints[JointType.HandRight]);
                        }
                        if (body.HandLeftState == HandState.Closed)
                        {
                            //ImageCanvasForRecognition.Children.Clear();
                        }
                        if (countDownForStoreFrame == 1)
                        {
                            string filename = "./frames/frame-" + DateTime.Now.ToString("yyyyMMddTHHmmss") + ".png";
                            RenderVisualService.RenderToPNGFile(this.ImageCanvasForRecognition, filename, this.ImageCanvasForRecognition);
                            //RenderVisualService.RenderToPNGFile(this.ImageCanvasForRecognition, "./pictureForClassifying.png", this.ImageCanvasForRecognition);

                            this.ImageCanvasForRecognition.Children.Clear();
                            this.ImageCanvas.Children.Clear();
                            this.pointLastTime.X = -1;
                            this.pointLastTime.Y = -1;
                            lastTimeDrawingHandClosed = DateTime.Now;
                            lastTimeDrawingHandOpened = DateTime.Now;
                            startingVertex.X = -1;
                            startingVertex.Y = -1;

                            isVertex(jointPoints[JointType.HandRight], true);
                            this.pointLastTimeForRecognition.X = -1;
                            this.pointLastTimeForRecognition.Y = -1;
                            string output = ExecuteCommandSync("SignContrast.exe \"" + filename + "\"");
                            TextBlockForRecognition.Text += output + "\n";
                            int answer = Int16.Parse(output);


                            //int answer = classifyProgram.classify(filename);
                            TextBlockForRecognition.Text = "Recognition file: \n" + filename + "\n";
                            TextBlockForRecognition.Text += "Recognition result: ";
                            switch (answer)
                            {
                                case 1:
                                    TextBlockForRecognition.Text += "Square\n";
                                    break;
                                case 2:
                                    TextBlockForRecognition.Text += "Line\n";
                                    break;
                                case 3:
                                    TextBlockForRecognition.Text += "Triangle\n";
                                    break;
                                case 4:
                                    TextBlockForRecognition.Text += "Broken Square\n";
                                    break;
                                case 5:
                                    TextBlockForRecognition.Text += "Big V\n";
                                    break;
                                default:
                                    TextBlockForRecognition.Text += "No result. Number: " + answer.ToString() + "\n";
                                    break;
                            }
                            this.drawingFinished = false;
                        }
                        if (countDownForStoreFrame > 0)
                        {
                            countDownForStoreFrame -= 1;
                        }
                        drawMousePoint(startingVertex);
                        if (this.drawingFinished == false && (DateTime.Now - lastTimeDrawingHandClosed).CompareTo(tolerantIntervalForDiscontinue) > 0 && this.ImageCanvas.Children.Count != 0)
                        {
                            this.drawingFinished = true;

                            // Delete lines drawn when hand is not closed. 

                            //for (int i = 0; i < counterForLinesDrawnWhenHandNotClosed; i++)
                            //{
                            //    for (int j = 0; j < 3; j++)
                            //    {
                            //        this.ImageCanvasForRecognition.Children.RemoveAt(this.ImageCanvasForRecognition.Children.Count - 1);
                            //    }
                            //}

                            // If the last point the user drew is far away from the last vertex we recorded, then we connect the shape to it. 
                            if (dist(lastPoint, pointLastTimeForRecognition) > distForVertices)
                            {
                                drawStrokeForRecognition(lastPoint);
                            }

                            if (dist(pointLastTimeForRecognition, startingVertex) < distForAutoConnectingStartingVertex)
                            {
                                //TextBlockForRecognition.Text = "\nMatched\n";
                                if (ImageCanvasForRecognition.Children.Count != 0)
                                {
                                    // There are two dots and one line for a line. 
                                    for (int i = 0; i < 3; i++)
                                    {
                                        ImageCanvasForRecognition.Children.RemoveAt(ImageCanvasForRecognition.Children.Count - 1);
                                    }
                                    pointLastTimeForRecognition = anchorForAutoConnectingStartingVertex;
                                    drawStrokeForRecognition(startingVertex);
                                }
                            }
                            else
                            {
                                //TextBlockForRecognition.Text = "\nUnMatched\n";
                            }
                            if (STORE_EVERY_SHAPE)
                            {
                                countDownForStoreFrame = 2;
                            }
                        }
                    }
                }
            }
        }
        /// <span class="code-SummaryComment"><summary></span>
        /// Executes a shell command synchronously.
        /// <span class="code-SummaryComment"></summary></span>
        /// <span class="code-SummaryComment"><param name="command">string command</param></span>
        /// <span class="code-SummaryComment"><returns>string, as output of the command.</returns></span>
        private string ExecuteCommandSync(object command)
        {
            try
            {
                // create the ProcessStartInfo using "cmd" as the program to be run,
                // and "/c " as the parameters.
                // Incidentally, /c tells cmd that we want it to execute the command that follows,
                // and then exit.
                System.Diagnostics.ProcessStartInfo procStartInfo =
                    new System.Diagnostics.ProcessStartInfo("cmd", "/c " + command);

                // The following commands are needed to redirect the standard output.
                // This means that it will be redirected to the Process.StandardOutput StreamReader.
                procStartInfo.RedirectStandardOutput = true;
                procStartInfo.UseShellExecute = false;
                // Do not create the black window.
                procStartInfo.CreateNoWindow = true;
                // Now we create a process, assign its ProcessStartInfo and start it
                System.Diagnostics.Process proc = new System.Diagnostics.Process();
                proc.StartInfo = procStartInfo;
                proc.Start();
                // Get the output into a string
                string result = proc.StandardOutput.ReadToEnd();
                // Display the command output.
                return result;
            }
            catch (Exception objException)
            {
                // Log the exception
                return "";
            }
        }
        public static class RenderVisualService
        {
            private const double defaultDpi = 96.0;

            public static ImageSource RenderToPNGImageSource(Visual targetControl)
            {
                var renderTargetBitmap = GetRenderTargetBitmapFromControl(targetControl);

                var encoder = new PngBitmapEncoder();
                encoder.Frames.Add(BitmapFrame.Create(renderTargetBitmap));

                var result = new BitmapImage();

                using (var memoryStream = new MemoryStream())
                {
                    encoder.Save(memoryStream);
                    memoryStream.Seek(0, SeekOrigin.Begin);

                    result.BeginInit();
                    result.CacheOption = BitmapCacheOption.OnLoad;
                    result.StreamSource = memoryStream;
                    result.EndInit();
                }

                return result;
            }

            public static void RenderToPNGFile(Visual targetControl, string filename, Canvas canvas)
            {
                var renderTargetBitmap = GetRenderTargetBitmapFromControl(targetControl);

                //System.Drawing.Bitmap bitmap;
                //using (var outStream = new MemoryStream())
                //{
                //    BitmapEncoder enc = new BmpBitmapEncoder();
                //    enc.Frames.Add(BitmapFrame.Create(renderTargetBitmap));
                //    enc.Save(outStream);
                //    bitmap = new System.Drawing.Bitmap(outStream);
                //}

                ////bitmap.SetResolution(1416, 712);

                //try
                //{
                //    bitmap.Save(filename, System.Drawing.Imaging.ImageFormat.Png);
                //}
                //catch(Exception ex)
                //{
                //    bitmap.Save(filename.Substring(0, filename.Length - 4) + "(1).png", System.Drawing.Imaging.ImageFormat.Png);
                //}

                var encoder = new PngBitmapEncoder();
                encoder.Frames.Add(BitmapFrame.Create(renderTargetBitmap));

                var result = new BitmapImage();

                try
                {
                    using (var fileStream = new FileStream(filename, FileMode.Create))
                    {
                        encoder.Save(fileStream);
                    }
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine($"There was an error saving the file: {ex.Message}");
                }
            }

            private static BitmapSource GetRenderTargetBitmapFromControl(Visual targetControl, double dpi = defaultDpi)
            {
                if (targetControl == null) return null;

                var bounds = VisualTreeHelper.GetDescendantBounds(targetControl);
                var renderTargetBitmap = new RenderTargetBitmap((int)(bounds.Width * dpi / 96.0),
                                                                (int)(bounds.Height * dpi / 96.0),
                                                                dpi,
                                                                dpi,
                                                                PixelFormats.Pbgra32);

                var drawingVisual = new DrawingVisual();

                using (var drawingContext = drawingVisual.RenderOpen())
                {
                    var visualBrush = new VisualBrush(targetControl);
                    drawingContext.DrawRectangle(visualBrush, null, new Rect(new Point(), bounds.Size));
                }

                renderTargetBitmap.Render(drawingVisual);
                return renderTargetBitmap;
            }
        }

        enum HandDirection
        {
            UP,
            MIDDLE,
            DOWN,
            UNKNOWN
        }

        enum Side
        {
            RIGHT,
            LEFT
        }

        private HandDirection getHandDirection(Point shoulder, Point elbow, Side side)
        {
            double slope = (elbow.Y - shoulder.Y) / (elbow.X - shoulder.X);
            if (side == Side.RIGHT)
            {
                if (slope > thresholdForHandDirection)
                {
                    return HandDirection.DOWN;
                }
                else if (slope < -thresholdForHandDirection)
                {
                    return HandDirection.UP;
                }
                else
                {
                    return HandDirection.MIDDLE;
                }
            }
            else if (side == Side.LEFT)
            {
                if (slope < -thresholdForHandDirection)
                {
                    return HandDirection.DOWN;
                }
                else if (slope > thresholdForHandDirection)
                {
                    return HandDirection.UP;
                }
                else
                {
                    return HandDirection.MIDDLE;
                }
            }
            else
            {
                return HandDirection.UNKNOWN;
            }
        }

        private bool isHandBending(Point shoulder, Point elbow, Point wrist)
        {
            Tuple<double, double> vectorES = new Tuple<double, double>(shoulder.X - elbow.X, shoulder.Y - elbow.Y);
            Tuple<double, double> vectorEW = new Tuple<double, double>(wrist.X - elbow.X, wrist.Y - elbow.Y);
            double lengthOfVectorES = Math.Sqrt(Math.Pow(vectorES.Item1, 2) + Math.Pow(vectorES.Item2, 2));
            double lengthOfVectorEW = Math.Sqrt(Math.Pow(vectorEW.Item1, 2) + Math.Pow(vectorEW.Item2, 2));
            double elbowCosValue = (vectorES.Item1 * vectorEW.Item1 + vectorES.Item2 * vectorEW.Item2) / (lengthOfVectorES * lengthOfVectorEW);
            if (elbowCosValue < Math.Cos(thresholdForHandBending))
            {
                return false;
            }
            return true;
        }

        private bool isIntersect(Point elbowRight, Point wristRight, Point elbowLeft, Point wristLeft)
        {
            // Compute one of the line
            double slope = (elbowRight.Y - wristRight.Y) / (elbowRight.X - wristRight.X);
            double b = elbowRight.Y - elbowRight.X * slope;
            double valueForElbowLeft = elbowLeft.X * slope - elbowLeft.Y + b;
            double valueForWristLeft = wristLeft.X * slope - wristLeft.Y + b;
            if (valueForElbowLeft > 0 && valueForWristLeft > 0)
            {
                return false;
            }
            else if (valueForElbowLeft < 0 && valueForWristLeft < 0)
            {
                return false;
            }
            slope = (elbowLeft.Y - wristLeft.Y) / (elbowLeft.X - wristLeft.X);
            b = elbowLeft.Y - elbowLeft.X * slope;
            double valueForElbowRight = elbowRight.X * slope - elbowRight.Y + b;
            double valueForWristRight = wristRight.X * slope - wristRight.Y + b;
            if (valueForElbowRight > 0 && valueForWristLeft > 0)
            {
                return false;
            }
            else if (valueForElbowRight < 0 && valueForWristRight < 0)
            {
                return false;
            }
            return true;
        }

        private void drawBones(Dictionary<JointType, Point> jointsPoints)
        {
            double X1 = 0, X2 = 0, Y1 = 0, Y2 = 0;
            foreach (Tuple<JointType, JointType> bone in this.bones)
            {
                Line myLine = new Line();
                myLine.Stroke = System.Windows.Media.Brushes.Black;
                X1 = jointsPoints[bone.Item1].X * this.Xratio;
                Y1 = jointsPoints[bone.Item1].Y * this.Yratio;
                X2 = jointsPoints[bone.Item2].X * this.Xratio;
                Y2 = jointsPoints[bone.Item2].Y * this.Yratio;
                if (0 < X1 && X1 < this.ImageSource.Width && 0 < X2 && X2 < this.ImageSource.Width)
                {
                    if (0 < Y1 && Y1 < this.ImageSource.Height && 0 < Y2 && Y2 < this.ImageSource.Height)
                    {
                        myLine.X1 = jointsPoints[bone.Item1].X * this.Xratio;
                        myLine.Y1 = jointsPoints[bone.Item1].Y * this.Yratio;
                        myLine.X2 = jointsPoints[bone.Item2].X * this.Xratio;
                        myLine.Y2 = jointsPoints[bone.Item2].Y * this.Yratio;
                    }
                }

                myLine.StrokeThickness = this.boneThickness;
                this.ImageCanvas.Children.Add(myLine);
            }

        }

        private void drawJoints(Dictionary<JointType, Point> jointsPoints)
        {

            double X = 0.0;
            double Y = 0.0;

            foreach (KeyValuePair<JointType, Point> item in jointsPoints)
            {
                Ellipse myEllipse = new Ellipse();
                SolidColorBrush mySolidColorBrush = new SolidColorBrush();
                mySolidColorBrush.Color = Color.FromArgb(255, 255, 0, 0);
                myEllipse.Fill = mySolidColorBrush;
                myEllipse.Width = this.jointThickness;
                myEllipse.Height = this.jointThickness;
                this.ImageCanvas.Children.Add(myEllipse);
                X = item.Value.X * this.Xratio;
                Y = item.Value.Y * this.Yratio;
                if (0 < X && X < this.ImageSource.Width)
                {
                    if (0 < Y && Y < this.ImageSource.Height)
                    {
                        Canvas.SetLeft(myEllipse, X);
                        Canvas.SetTop(myEllipse, Y);
                    }
                }
            }
        }

        private void drawStroke(Point point)
        {
            double startingAndEndingPointDiameter = this.drawingStrokeWidth * 0.8;
            if (this.pointLastTime.X == -1 && this.pointLastTime.Y == -1)
            {
                this.pointLastTime.X = point.X;
                this.pointLastTime.Y = point.Y;
                return;
            }
            double X1 = 0, X2 = 0, Y1 = 0, Y2 = 0;
            Line myLine = new Line();
            myLine.Stroke = System.Windows.Media.Brushes.White;

            Ellipse ellipse1 = new Ellipse();
            Ellipse ellipse2 = new Ellipse();
            SolidColorBrush mySolidColorBrush = new SolidColorBrush();
            mySolidColorBrush.Color = Color.FromArgb(255, 255, 255, 255);
            ellipse1.Fill = mySolidColorBrush;
            ellipse1.Width = startingAndEndingPointDiameter;
            ellipse1.Height = startingAndEndingPointDiameter;
            ellipse2.Fill = mySolidColorBrush;
            ellipse2.Width = startingAndEndingPointDiameter;
            ellipse2.Height = startingAndEndingPointDiameter;
            this.ImageCanvas.Children.Add(ellipse1);
            this.ImageCanvas.Children.Add(ellipse2);
            X1 = this.pointLastTime.X * this.Xratio;
            Y1 = this.pointLastTime.Y * this.Yratio;
            X2 = point.X * this.Xratio;
            Y2 = point.Y * this.Yratio;
            if (0 < X1 && X1 < this.ImageSource.Width && 0 < X2 && X2 < this.ImageSource.Width)
            {
                if (0 < Y1 && Y1 < this.ImageSource.Height && 0 < Y2 && Y2 < this.ImageSource.Height)
                {
                    myLine.X1 = X1;
                    myLine.Y1 = Y1;
                    myLine.X2 = X2;
                    myLine.Y2 = Y2;
                    Canvas.SetLeft(ellipse1, X1 - startingAndEndingPointDiameter / 2);
                    Canvas.SetTop(ellipse1, Y1 - startingAndEndingPointDiameter / 2);
                    Canvas.SetLeft(ellipse2, X2 - startingAndEndingPointDiameter / 2);
                    Canvas.SetTop(ellipse2, Y2 - startingAndEndingPointDiameter / 2);
                }
            }

            myLine.StrokeThickness = this.drawingStrokeWidth;
            this.ImageCanvas.Children.Add(myLine);
            this.pointLastTime.X = point.X;
            this.pointLastTime.Y = point.Y;
        }
        private void drawStrokeForRecognition(Point point)
        {
            double startingAndEndingPointDiameter = this.drawingStrokeWidth * 0.8;
            if (this.pointLastTimeForRecognition.X == -1 && this.pointLastTimeForRecognition.Y == -1)
            {
                this.pointLastTimeForRecognition.X = point.X;
                this.pointLastTimeForRecognition.Y = point.Y;
                return;
            }
            double X1 = 0, X2 = 0, Y1 = 0, Y2 = 0;
            Line myLine = new Line();
            myLine.Stroke = System.Windows.Media.Brushes.White;

            Ellipse ellipse1 = new Ellipse();
            Ellipse ellipse2 = new Ellipse();
            SolidColorBrush mySolidColorBrush = new SolidColorBrush();
            mySolidColorBrush.Color = Color.FromArgb(255, 255, 255, 255);
            ellipse1.Fill = mySolidColorBrush;
            ellipse1.Width = startingAndEndingPointDiameter;
            ellipse1.Height = startingAndEndingPointDiameter;
            ellipse2.Fill = mySolidColorBrush;
            ellipse2.Width = startingAndEndingPointDiameter;
            ellipse2.Height = startingAndEndingPointDiameter;
            this.ImageCanvasForRecognition.Children.Add(ellipse1);
            this.ImageCanvasForRecognition.Children.Add(ellipse2);
            X1 = this.pointLastTimeForRecognition.X * this.Xratio;
            Y1 = this.pointLastTimeForRecognition.Y * this.Yratio;
            X2 = point.X * this.Xratio;
            Y2 = point.Y * this.Yratio;
            if (0 < X1 && X1 < this.ImageSource.Width && 0 < X2 && X2 < this.ImageSource.Width)
            {
                if (0 < Y1 && Y1 < this.ImageSource.Height && 0 < Y2 && Y2 < this.ImageSource.Height)
                {
                    myLine.X1 = X1;
                    myLine.Y1 = Y1;
                    myLine.X2 = X2;
                    myLine.Y2 = Y2;
                    Canvas.SetLeft(ellipse1, X1 - startingAndEndingPointDiameter / 2);
                    Canvas.SetTop(ellipse1, Y1 - startingAndEndingPointDiameter / 2);
                    Canvas.SetLeft(ellipse2, X2 - startingAndEndingPointDiameter / 2);
                    Canvas.SetTop(ellipse2, Y2 - startingAndEndingPointDiameter / 2);
                }
            }

            myLine.StrokeThickness = this.drawingStrokeWidth;
            this.ImageCanvasForRecognition.Children.Add(myLine);
            this.anchorForAutoConnectingStartingVertex.X = this.pointLastTimeForRecognition.X;
            this.anchorForAutoConnectingStartingVertex.Y = this.pointLastTimeForRecognition.Y;
            this.pointLastTimeForRecognition.X = point.X;
            this.pointLastTimeForRecognition.Y = point.Y;
        }
        private void drawMousePoint(Point point)
        {
            double X = 0.0;
            double Y = 0.0;

            Ellipse myEllipse = new Ellipse();
            SolidColorBrush mySolidColorBrush = new SolidColorBrush();
            mySolidColorBrush.Color = Color.FromArgb(255, 255, 0, 0);
            myEllipse.Fill = mySolidColorBrush;
            myEllipse.Width = this.drawingStrokeWidth;
            myEllipse.Height = this.drawingStrokeWidth;
            this.ImageCanvasForMouse.Children.Add(myEllipse);
            X = point.X * this.Xratio;
            Y = point.Y * this.Yratio;
            if (0 < X && X < this.ImageSource.Width)
            {
                if (0 < Y && Y < this.ImageSource.Height)
                {
                    Canvas.SetLeft(myEllipse, X);
                    Canvas.SetTop(myEllipse, Y);
                }
            }
        }
        private void drawPoint(Point point)
        {
            double X = 0.0;
            double Y = 0.0;

            Ellipse myEllipse = new Ellipse();
            SolidColorBrush mySolidColorBrush = new SolidColorBrush();
            mySolidColorBrush.Color = Color.FromArgb(255, 0, 255, 0);
            myEllipse.Fill = mySolidColorBrush;
            myEllipse.Width = this.drawingStrokeWidth;
            myEllipse.Height = this.drawingStrokeWidth;
            this.ImageCanvas.Children.Add(myEllipse);
            X = point.X * this.Xratio;
            Y = point.Y * this.Yratio;
            if (0 < X && X < this.ImageSource.Width)
            {
                if (0 < Y && Y < this.ImageSource.Height)
                {
                    Canvas.SetLeft(myEllipse, X);
                    Canvas.SetTop(myEllipse, Y);
                }
            }
        }
        private bool isVertex(Point point, bool reset = false)
        {
            if (reset)
            {
                previousPoints.Clear();
                previousPoints.Add(point);
                return false;
            }
            if (previousPoints.Count() == numOfPointsToVertex)
            {
                previousPoints.RemoveAt(0);
            }
            previousPoints.Add(point);
            if (previousPoints.Count() < numOfPointsToVertex)
            {
                return false;
            }
            else
            {
                for (int i = 0; i < previousPoints.Count(); i++)
                {
                    for (int j = 0; j < previousPoints.Count(); j++)
                    {
                        if (i == j)
                        {
                            continue;
                        }
                        if (dist(previousPoints[i], previousPoints[j]) > rangeOfVertex)
                        {
                            return false;
                        }
                    }
                }
            }
            return true;
        }

        private double dist(Point point1, Point point2)
        {
            return Math.Sqrt(Math.Pow((point1.X - point2.X), 2) + Math.Pow((point1.Y - point2.Y), 2));
        }
        private void Kinect_Class2_Loaded(object sender, RoutedEventArgs e)
        {
            // "ImageSource" is like an id, and it's defined in the MainWindow.xaml1
            ImageSource.Source = this.bitmap;
        }

        private void Kinect_Class2_Unloaded(object sender, RoutedEventArgs e)
        {
            if (this.colorFrameReader != null)
            {
                // ColorFrameReder is IDisposable
                this.colorFrameReader.Dispose();
                this.colorFrameReader = null;
            }

            if (this.sensor != null)
            {
                this.sensor.Close();
                this.sensor = null;
            }
        }
    }
}