package com.pmproject.pmproject;

import android.util.Log;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

public class ImageProcessor {

    public static final int kNumPoints = 60;
    public static final double kMaxDist = 4;
    public static final double kMinMeanLength = 3;
    public static final double kMaxStdLength = 0.18f;

    public static final double kZoomFactor = 1.25;

    public static final int kOrigWidth = 920;
    public static final int kOrigHeight = 748;
    public static final int kZoomedWidth = 690;
    public static final int kZoomedHeight = 561;
    public static final int kSmallWidth = 690;
    public static final int kSmallHeight = 561;

    public static final int kCalibrateFrames = 120;
    public static final int kCalibrateUpdateInterval = 20;
    public static final double kCalibrateThreshold = 0.25;

    private AverageLine leftLine = new AverageLine();
    private AverageLine rightLine = new AverageLine();
    private AverageLine bottomLine = new AverageLine();
    private AverageLine topLine = new AverageLine();
    private int lastDirection = -1;
    private int calibrationTime = 0;
    private Mat proj = null;
    private Mat imgOrigSmall = null;
    private Vec2 pointer = null, oldPointer = null;

    private double[] bestLeft = null;
    private double[] bestRight = null;
    private double[] bestTop = null;
    private double[] bestBottom = null;

    private final Tracker tracker = new Tracker(kNumPoints);

    private final Function<Integer, Void> onDirectionUpdate;
    private final Function<Void, Void> onCalibrationDone;

    public ImageProcessor(Function<Integer, Void> onDirectionUpdate,
                          Function<Void, Void> onCalibrationDone)
    {
        this.onDirectionUpdate = onDirectionUpdate;
        this.onCalibrationDone = onCalibrationDone;
    }

    private void resetProjection()
    {
        Mat src_points = new Mat(4, 2, CvType.CV_32FC1);
        src_points.put(0, 0,
                new float[] { 0, 0, kZoomedWidth, 0, kZoomedWidth, kZoomedHeight, 0, kZoomedHeight} );
        Mat dst_points = new Mat(4, 2, CvType.CV_32FC1);
        dst_points.put(0, 0,
                new float[] { 0, 0, kSmallWidth, 0, kSmallWidth, kSmallHeight, 0, kSmallHeight } );
        proj = Imgproc.getPerspectiveTransform(src_points, dst_points);
    }

    public void startCalibrating()
    {
        calibrationTime = kCalibrateFrames;
        resetProjection();
        bestLeft = null;
        bestRight = null;
        bestTop = null;
        bestBottom = null;
        topLine = new AverageLine();
        bottomLine = new AverageLine();
        rightLine = new AverageLine();
        leftLine = new AverageLine();
    }

    private void calibrate(boolean updateProjection, boolean updateLines, Mat drawOn)
    {
        Mat canny = new Mat(), lines = new Mat();
        Imgproc.Canny(imgOrigSmall, canny, 12, 25); // 20 100
        Imgproc.HoughLinesP(canny, lines, 1, Math.PI / 180.f, 4, 200, 40);

        Imgproc.cvtColor(canny, canny, Imgproc.COLOR_GRAY2RGBA);
        Imgproc.resize(canny, drawOn, drawOn.size());

        for (int i = 0; i < lines.cols(); i++) {
            double[] l = lines.get(0, i);

            if (drawOn != null)
                Imgproc.line(drawOn, new Point(l[0], l[1]), new Point(l[2], l[3]), new Scalar(0, 255, 0), 8);

            if (l[0] < kZoomedWidth * kCalibrateThreshold && l[2] < kZoomedWidth * kCalibrateThreshold) {
                if (bestLeft == null || Math.max(bestLeft[0], bestLeft[2]) < Math.max(l[0], l[2]))
                    bestLeft = l.clone();
            }

            else if (l[0] > kZoomedWidth * (1 - kCalibrateThreshold) && l[2] > kZoomedWidth * (1 - kCalibrateThreshold)) {
                if (bestRight == null || Math.min(bestRight[0], bestRight[2]) > Math.min(l[0], l[2]))
                    bestRight = l.clone();
            }

            else if (l[1] < kZoomedHeight * kCalibrateThreshold && l[3] < kZoomedHeight * kCalibrateThreshold) {
                if (bestTop == null || Math.max(bestTop[1], bestTop[3]) < Math.max(l[1], l[3]))
                    bestTop = l.clone();
            }

            else if (l[1] > kZoomedHeight * (1 - kCalibrateThreshold) && l[3] > kZoomedHeight * (1 - kCalibrateThreshold)) {
                if (bestBottom == null || Math.min(bestBottom[1], bestBottom[3]) > Math.min(l[1], l[3]))
                    bestBottom = l.clone();
            }
        }

        if (updateLines)
        {
            if (bestLeft != null) {
                leftLine.addLine(new Vec2(bestLeft[0], bestLeft[1]), new Vec2(bestLeft[2], bestLeft[3]));
                Log.i(MainActivity.TAG, "Left   ( " + bestLeft[0] + " , " + bestLeft[1] + " ) ; ( " + bestLeft[2] + " , " + bestLeft[3] + " )");
            }
            if (bestRight != null) {
                rightLine.addLine(new Vec2(bestRight[0], bestRight[1]), new Vec2(bestRight[2], bestRight[3]));
                Log.i(MainActivity.TAG, "Right  ( " + bestRight[0] + " , " + bestRight[1] + " ) ; ( " + bestRight[2] + " , " + bestRight[3] + " )");
            }
            if (bestBottom != null) {
                bottomLine.addLine(new Vec2(bestBottom[0], bestBottom[1]), new Vec2(bestBottom[2], bestBottom[3]));
                Log.i(MainActivity.TAG, "Bottom ( " + bestBottom[0] + " , " + bestBottom[1] + " ) ; ( " + bestBottom[2] + " , " + bestBottom[3] + " )");
            }
            if (bestTop != null) {
                topLine.addLine(new Vec2(bestTop[0], bestTop[1]), new Vec2(bestTop[2], bestTop[3]));
                Log.i(MainActivity.TAG, "Top    ( " + bestTop[0] + " , " + bestTop[1] + " ) ; ( " + bestTop[2] + " , " + bestTop[3] + " )");
            }

            bestLeft = null;
            bestRight = null;
            bestBottom = null;
            bestTop = null;
        }

        if (updateProjection)
        {
            topLine.submit();
            bottomLine.submit();
            rightLine.submit();
            leftLine.submit();

            if (topLine.n == 0) {
                topLine.a = 0;
                topLine.b = 1;
                topLine.c = 0;
            }
            if (bottomLine.n == 0) {
                bottomLine.a = 0;
                bottomLine.b = 1;
                bottomLine.c = -kSmallHeight;
            }
            if (leftLine.n == 0) {
                leftLine.a = 1;
                leftLine.b = 0;
                leftLine.c = 0;
            }
            if (rightLine.n == 0) {
                rightLine.a = 1;
                rightLine.b = 0;
                rightLine.c = -kSmallWidth;
            }

            Vec2 topLeft = topLine.intersect(leftLine);
            Vec2 topRight = topLine.intersect(rightLine);
            Vec2 bottomLeft = bottomLine.intersect(leftLine);
            Vec2 bottomRight = bottomLine.intersect(rightLine);

            Mat src_points = new Mat(4, 2, CvType.CV_32FC1);
            src_points.put(0, 0, new float[] {
                    (float) topLeft.x, (float) topLeft.y, (float) topRight.x, (float) topRight.y,
                    (float) bottomRight.x, (float) bottomRight.y, (float) bottomLeft.x, (float) bottomLeft.y } );
            Mat dst_points = new Mat(4, 2, CvType.CV_32FC1);
            dst_points.put(0, 0,
                    new float[] { 0, 0, kSmallWidth, 0, kSmallWidth, kSmallHeight, 0, kSmallHeight } );
            proj = Imgproc.getPerspectiveTransform(src_points, dst_points);
            onCalibrationDone.apply(null);
        }
    }

    private void updatePointer(Mat imgSmall)
    {
        Mat imgHSV = new Mat(), mask = new Mat(), hierarchy = new Mat(), imgRGB = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();

        Imgproc.cvtColor(imgSmall, imgRGB, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(imgRGB, imgHSV, Imgproc.COLOR_RGB2HSV);
        Core.inRange(imgHSV, new Scalar(0, 35, 174), new Scalar(180, 255, 255), mask);
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        oldPointer = pointer;
        Vec2 closest = null;
        double minDist = 100000;
        for (MatOfPoint p : contours)
        {
            Point center = new Point();
            float[] radius = new float[1];
            Imgproc.minEnclosingCircle(new MatOfPoint2f(p.toArray()), center, radius);
            if (closest == null || pointer == null)
                closest = new Vec2(center.x, center.y);
            else
            {
                double dx = center.x - pointer.x;
                double dy = center.y - pointer.y;
                double dist = dx * dx + dy * dy;
                if (dist < minDist)
                {
                    closest = new Vec2(center.x, center.y);
                    minDist = dist;
                }
            }
        }
        pointer = closest;
    }

    private void updateTracker()
    {
        if (pointer != null && oldPointer != null)
        {
            double dx = pointer.x - oldPointer.x;
            double dy = pointer.y - oldPointer.y;
            double plength = Math.sqrt(dx * dx + dy * dy);
            if (plength > 0.0001) {
                int n = (int) Math.ceil(plength / kMaxDist);
                Vec2 p = new Vec2(oldPointer.x, oldPointer.y);
                dx /= n;
                dy /= n;
                for (int i = 0; i < n; i++)
                {
                    p.x += dx;
                    p.y += dy;
                    tracker.addPoint(p);
                }
            }
        }
        else if (pointer != null)
        {
            tracker.addPoint(pointer);
        }
        else
        {
            resetTracker();
        }
    }

    private boolean updateDirection()
    {
        double tml = tracker.getMean().length();
        // double tdl = tracker.getSigma().length();
        // double tnml = tracker.getNormMean().length();
        double tndl = tracker.getNormSigma().length();
        if (tracker.isReady() && tml >= kMinMeanLength && tndl <= kMaxStdLength)
        {
            int direction = (int)((Math.atan2(tracker.getMean().y, tracker.getMean().x) + Math.PI * 1.125) * 4 / Math.PI) % 8;

            if (direction != lastDirection)
            {
                lastDirection = direction;
                return true;
            }
        }
        return false;
    }

    public Mat processFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame)
    {
        if (proj == null)
            resetProjection();
        if (imgOrigSmall == null)
            imgOrigSmall = new Mat();

        Mat input = inputFrame.rgba();
        int h = input.rows();
        int w = input.cols();
        int nh = (int)(h / kZoomFactor);
        int nw = (int)(w / kZoomFactor);
        int dw = (w - nw) / 2;
        int dh = (h - nh) / 2;
        Mat zoomed = input.submat(dh, h - dh, dw, w - dw);
        Imgproc.resize(zoomed, imgOrigSmall, new Size(kZoomedWidth, kZoomedHeight));

        if (calibrationTime > 0)
        {
            calibrationTime--;
            calibrate(calibrationTime == 0,
                    calibrationTime % kCalibrateUpdateInterval == 0,
                    imgOrigSmall);
        }

        Mat imgSmall = new Mat();
        Imgproc.warpPerspective(imgOrigSmall, imgSmall, proj, new Size(kSmallWidth, kSmallHeight));

        updatePointer(imgSmall);
        updateTracker();
        boolean updated = updateDirection();
        if (updated)
        {
            onDirectionUpdate.apply(lastDirection);
        }

        for (Vec2 p : tracker.getPoints())
        {
            Imgproc.circle(imgSmall, new Point(p.x, p.y), 10, new Scalar(255, 0, 0), 2);
        }

        Mat output = new Mat();
        Imgproc.resize(imgSmall, output, input.size());
        return output;
    }

    public void resetTracker()
    {
        tracker.clear();
        lastDirection = -1;
    }

    public Vec2 getPointer()
    {
        return pointer;
    }
}
