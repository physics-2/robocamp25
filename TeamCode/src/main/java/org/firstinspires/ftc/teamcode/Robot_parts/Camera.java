package org.firstinspires.ftc.teamcode.Robot_parts;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpMode;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class Camera extends OpMode {

    private OpenCvCamera controlHubCam;
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 360;


    private static final double REAL_CUBE_WIDTH_INCHES = 3.75;
    private static final double FOCAL_LENGTH_PIXELS = 728;


    private double centerX = 0;
    private double centerY = 0;
    private double widthInPixels = 0;
    private boolean isCubeDetected = false;

    public Camera(HardwareMap hardwareMap) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.startCameraStream(controlHubCam, 30);

        initOpenCV(hardwareMap);
    }

    private void initOpenCV(HardwareMap hardwareMap) {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        controlHubCam.setPipeline(new BlueCubePipeline());

        controlHubCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Ошибка камеры", errorCode);
                telemetry.update();
            }
        });
    }

    private class BlueCubePipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            isCubeDetected = false;

            //  RGBA → RGB
            Mat rgb = new Mat();
            Imgproc.cvtColor(input, rgb, Imgproc.COLOR_RGBA2RGB);

            //  RGB → HSV
            Mat hsv = new Mat();
            Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV);

            // 3. Find blue in  HSV
            Scalar lowerBlue = new Scalar(100, 150, 50);   // H: 100–130 — синий
            Scalar upperBlue = new Scalar(130, 255, 255);  // S и V — отсекаем тени/блики

            Mat mask = new Mat();
            Core.inRange(hsv, lowerBlue, upperBlue, mask);

            // 4. Remove noise
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

            // Find contours
            ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();

            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find largest contours
            MatOfPoint largestContour = null;
            double maxArea = 500; // minimal area

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            // If contours find
            if (largestContour != null) {
                // Calculating center point
                Moments moments = Imgproc.moments(largestContour);
                centerX = moments.m10 / moments.m00;
                centerY = moments.m01 / moments.m00;

                // Boreder of rect
                Rect boundingRect = Imgproc.boundingRect(largestContour);
                widthInPixels = boundingRect.width;

                isCubeDetected = true;

                //draw a counter
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour),
                        new Scalar(255, 0, 0, 255), 3);

                // Center dot
                Imgproc.circle(input, new Point(centerX, centerY), 7, new Scalar(0, 255, 0, 255), -1);

                // Writings
                Imgproc.putText(input, "Синий куб", new Point(centerX + 10, centerY - 10),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(255, 0, 0, 255), 2);

                Imgproc.putText(input, String.format("X:%d Y:%d", (int) centerX, (int) centerY),
                        new Point(centerX + 10, centerY + 25),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0, 255), 1);

                Imgproc.putText(input, String.format("Ширина: %d px", (int) widthInPixels),
                        new Point(centerX + 10, centerY + 45),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0, 255), 1);
            }

            rgb.release();
            hsv.release();
            mask.release();
            kernel.release();

            return input;
        }
    }

    private double calculateDistance(double widthInPixels) {
        if (widthInPixels <= 0) return 0;
        return (REAL_CUBE_WIDTH_INCHES * FOCAL_LENGTH_PIXELS) / widthInPixels;
    }

    public double getCubeX() {
        return centerX;
    }


    public double getCubeY() {
        return centerY;
    }


    public double getCubeWidthInPixels() {
        return widthInPixels;
    }


    public boolean isCubeDetected() {
        return isCubeDetected;
    }


    public double getDistance() {
        if (!isCubeDetected || widthInPixels <= 0) return 0;
        return (REAL_CUBE_WIDTH_INCHES * FOCAL_LENGTH_PIXELS) / widthInPixels;
    }


    public double getCubeXNormalized() {
        if (!isCubeDetected) return 0;
        return (centerX - CAMERA_WIDTH / 2.0) / (CAMERA_WIDTH / 2.0);
    }


    public double getRelativeSize() {
        if (!isCubeDetected) return 0;
        return widthInPixels / CAMERA_WIDTH;
    }

}