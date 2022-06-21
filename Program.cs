using System;
using OpenCvSharp;

class Program
{
    Point[][][] runecontours = new Point[5][][];
    HierarchyIndex[][] runehierarchy = new HierarchyIndex[5][];
    Point[][] contours;
    HierarchyIndex[] hierarchy;
    Mat[] image = new Mat[5];
    Mat[] runeContours = new Mat[5];
    public Program()
    {
        image[0] = Cv2.ImRead("rune/1.png", ImreadModes.Grayscale).GaussianBlur(new Size(3, 3), 0).Canny(100, 250);
        Cv2.FindContours(image[0], out runecontours[0], out runehierarchy[0], RetrievalModes.External, ContourApproximationModes.ApproxSimple);

        image[1] = Cv2.ImRead("rune/2.png", ImreadModes.Grayscale).GaussianBlur(new Size(3, 3), 0).Canny(100, 250);
        Cv2.FindContours(image[1], out runecontours[1], out runehierarchy[1], RetrievalModes.External, ContourApproximationModes.ApproxSimple);

        image[2] = Cv2.ImRead("rune/3.png", ImreadModes.Grayscale).GaussianBlur(new Size(3, 3), 0).Canny(100, 250);
        Cv2.FindContours(image[2], out runecontours[2], out runehierarchy[2], RetrievalModes.External, ContourApproximationModes.ApproxSimple);

        image[3] = Cv2.ImRead("rune/4.png", ImreadModes.Grayscale).GaussianBlur(new Size(3, 3), 0).Canny(100, 250);
        Cv2.FindContours(image[3], out runecontours[3], out runehierarchy[3], RetrievalModes.External, ContourApproximationModes.ApproxSimple);

        image[4] = Cv2.ImRead("rune/5.png", ImreadModes.Grayscale).GaussianBlur(new Size(3, 3), 0).Canny(100, 250);
        Cv2.FindContours(image[4], out runecontours[4], out runehierarchy[4], RetrievalModes.External, ContourApproximationModes.ApproxSimple);

        runeContours[0] = Mat.Zeros(image[0].Size(), MatType.CV_8UC1);
        runeContours[1] = Mat.Zeros(image[1].Size(), MatType.CV_8UC1);
        runeContours[2] = Mat.Zeros(image[2].Size(), MatType.CV_8UC1);
        runeContours[3] = Mat.Zeros(image[3].Size(), MatType.CV_8UC1);
        runeContours[4] = Mat.Zeros(image[4].Size(), MatType.CV_8UC1);
    }
    public int classify(String fileName)
    {


        /*image[0] = Cv2.ImRead("rune/1.png", ImreadModes.Grayscale).GaussianBlur(new Size(3, 3), 0).Canny(100, 250);
        Cv2.FindContours(image[0] , out runecontours[0], out runehierarchy[0], RetrievalModes.External, ContourApproximationModes.ApproxSimple);

        image[1] = Cv2.ImRead("rune/2.png", ImreadModes.Grayscale).GaussianBlur(new Size(3, 3), 0).Canny(100, 250);
        Cv2.FindContours(image[1], out runecontours[1], out runehierarchy[1], RetrievalModes.External, ContourApproximationModes.ApproxSimple);

        image[2] = Cv2.ImRead("rune/3.png", ImreadModes.Grayscale).GaussianBlur(new Size(3, 3), 0).Canny(100, 250);
        Cv2.FindContours(image[2], out runecontours[2], out runehierarchy[2], RetrievalModes.External, ContourApproximationModes.ApproxSimple);

        image[3] = Cv2.ImRead("rune/4.png", ImreadModes.Grayscale).GaussianBlur(new Size(3, 3), 0).Canny(100, 250);
        Cv2.FindContours(image[3], out runecontours[3], out runehierarchy[3], RetrievalModes.External, ContourApproximationModes.ApproxSimple);

        image[4] = Cv2.ImRead("rune/5.png", ImreadModes.Grayscale).GaussianBlur(new Size(3, 3), 0).Canny(100, 250);
        Cv2.FindContours(image[4], out runecontours[4], out runehierarchy[4], RetrievalModes.External, ContourApproximationModes.ApproxSimple);

        Mat[] runeContours = { Mat.Zeros(image[0].Size(), MatType.CV_8UC1), Mat.Zeros(image[1].Size(), MatType.CV_8UC1), Mat.Zeros(image[2].Size(), MatType.CV_8UC1),
            Mat.Zeros(image[3].Size(), MatType.CV_8UC1), Mat.Zeros(image[4].Size(), MatType.CV_8UC1)};*/

        Mat test = Cv2.ImRead(fileName, ImreadModes.Grayscale).GaussianBlur(new Size(3, 3), 0).Canny(100, 250);
        Cv2.FindContours(test, out contours, out hierarchy, RetrievalModes.External, ContourApproximationModes.ApproxSimple);

        Mat imageContours = Mat.Zeros(test.Size(), MatType.CV_8UC1);
        for (int i = 0; i < 5; i++)
            for (int j = 0; j < runecontours[i].Length; j++)
                Cv2.DrawContours(runeContours[i], runecontours[i], j, new Scalar(255), -1, LineTypes.Link8, runehierarchy[i]);
        for (int j = 0; j < contours.Length; j++)
            Cv2.DrawContours(imageContours, contours, j, new Scalar(255), -1, LineTypes.Link8, hierarchy);

        double[] Con = { 0, 0, 0, 0, 0 };
        int[] Rune = { -1, -1, -1, -1, -1 };
        int RuneSize = 0;

        for (int i = 0; i < 5; i++)
        {
            Con[i] += Cv2.MatchShapes(runeContours[i], imageContours, ShapeMatchModes.I1, 0.0);
            Con[i] += Cv2.MatchShapes(runeContours[i], imageContours, ShapeMatchModes.I2, 0.0);
            Con[i] += Cv2.MatchShapes(runeContours[i], imageContours, ShapeMatchModes.I3, 0.0);
            Console.WriteLine(Con[i]);
            if (Con[i] <= 1)
                Rune[RuneSize++] = i;
        }
        int min = Rune[0];
        for (int i = 1; i < RuneSize; i++)
            if (Con[Rune[i]] < Con[min])
                min = Rune[i];

        switch (min)
        {
            case 0:
                Console.WriteLine("1");
                return 1;
            case 1:
                Console.WriteLine("2");
                return 2;
            case 2:
                Console.WriteLine("3");
                return 3;
            case 3:
                Console.WriteLine("4");
                return 4;
            case 4:
                Console.WriteLine("5");
                return 5;
            default:
                Console.WriteLine("-1");
                return -1;
        }
        return 0;
    }
}