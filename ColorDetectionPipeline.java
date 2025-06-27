package org.firstinspires.ftc.teamcode;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class ColorDetectionPipeline extends OpenCvPipeline {

    Telemetry telemetry;

    public int redCount = 0, yellowCount = 0, blueCount = 0;
    public Point redCenter = new Point(-1, -1);
    public Point yellowCenter = new Point(-1, -1);
    public Point blueCenter = new Point(-1, -1);

    public List<Point> redCenters = new ArrayList<>();

    public ColorDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        redCenters.clear();

        // 三种颜色检测
        detectColor(hsv, input, new Scalar(0, 100, 100), new Scalar(10, 255, 255), new Scalar(0, 0, 255), "Red");
        detectColor(hsv, input, new Scalar(22, 100, 100), new Scalar(32, 255, 255), new Scalar(0, 255, 255), "Yellow");
        detectColor(hsv, input, new Scalar(100, 150, 0), new Scalar(130, 255, 255), new Scalar(255, 0, 0), "Blue");

        hsv.release();
        return input;
    }

    private void detectColor(Mat hsv, Mat output, Scalar lower, Scalar upper, Scalar drawColor, String label) {
        Mat mask = new Mat();
        Core.inRange(hsv, lower, upper, mask);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        int count = 0;
        Point avgCenter = new Point(0, 0);

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > 100) {
                Imgproc.drawContours(output, List.of(contour), -1, drawColor, 2);

                Moments M = Imgproc.moments(contour);
                if (M.m00 != 0) {
                    Point center = new Point(M.m10 / M.m00, M.m01 / M.m00);
                    avgCenter.x += center.x;
                    avgCenter.y += center.y;
                    count++;

                    if (label.equals("Red")) {
                        redCenters.add(center);
                    }

                    Imgproc.circle(output, center, 5, drawColor, -1);
                    Imgproc.putText(output, "(" + (int) center.x + "," + (int) center.y + ")", center,
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, drawColor, 1);
                }
            }
        }

        if (count > 0) {
            avgCenter.x /= count;
            avgCenter.y /= count;

            switch (label) {
                case "Red":
                    redCount = count;
                    redCenter = avgCenter;
                    break;
                case "Yellow":
                    yellowCount = count;
                    yellowCenter = avgCenter;
                    break;
                case "Blue":
                    blueCount = count;
                    blueCenter = avgCenter;
                    break;
            }
        } else {
            switch (label) {
                case "Red":
                    redCount = 0;
                    redCenter = new Point(-1, -1);
                    break;
                case "Yellow":
                    yellowCount = 0;
                    yellowCenter = new Point(-1, -1);
                    break;
                case "Blue":
                    blueCount = 0;
                    blueCenter = new Point(-1, -1);
                    break;
            }
        }

        mask.release();
        hierarchy.release();
    }
}
